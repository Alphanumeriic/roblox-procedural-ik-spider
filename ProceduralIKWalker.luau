-- Connected Discord-GitHub | Discord: yousif.dev | Roblox: Yousxif
--[[
	procedural ik spider. you ARE the spider

	one file. hides everyones avatar and swaps it for a 4 legged spider mech
	built out of plain parts at runtime, all driven camera-first: mouse yaws
	the whole creature around whatever surface its clinging to, wasd moves on
	that surfaces tangent plane, space pounces, and any face you run into or
	land on becomes the new floor. walls, cliff sides, under an arch, w/e

	whats in here:
	 - analytic 2 bone IK per leg (law of cosines inside a pole plane), all
	   solved in the body frame so upside down needs no special cases
	 - raycast trot gait. diagonal pairs, steps fire off drift not a timer,
	   bezier swing arcs, feet land ahead of where youre going
	 - spring damped body. hover, stride thumps, support tilt, banking,
	   breathing
	 - surface attach solver, wall grab / edge wrap / ballistic leaps

	no server code, no remotes at all. every client runs this same file,
	hides all avatars locally, pilots its OWN spider (so zero input lag) and
	writes the pose into its character root, which roblox replicates anyway
	since the client owns it. everyone else re-derives gait + IK from those
	roots. the root cframe is the entire netcode

	usage: LocalScript in StarterPlayer > StarterPlayerScripts, hit play.
	mouse steers, wasd walks, space pounces, wheel zooms
]]

local RunService = game:GetService("RunService")
local Players = game:GetService("Players")

local WORLD_UP = Vector3.yAxis

------------------------------------ tuning ------------------------------------
-- every magic number lives here, dont scatter them around
local CONFIG = {
	-- skeleton
	UpperLength = 4.0,               -- femur
	LowerLength = 5.2,               -- tibia. longer than the femur reads way more arachnid
	BodySize = Vector3.new(6.5, 2.4, 8.5),
	HipX = 3.1, HipY = -0.55, HipZ = 3.4, -- hip pivots, just inside the torso corners
	HomeX = 6.8, HomeZ = 5.4,        -- neutral stance splay from body center
	Hover = 4.5,                     -- body height above the surface (along its normal, not world Y)

	-- gait
	StepTrigger = 2.4,               -- step fires once a home point drifts this far from the planted foot
	SwingTime = 0.18,                -- air time per step. cadence falls out of drift, theres no clock
	StepHeight = 1.9,                -- how far the arc bulges along the body normal
	LeadTime = 0.25,                 -- feet land this many secs ahead of the motion so they dont trail
	ProbeUp = 4, ProbeDown = 10,     -- foot ray span along the body normal
	MaxFootSlope = 0.3,              -- foothold normal has to roughly agree with the body normal

	-- piloting
	MoveSpeed = 14,
	MouseSens = 0.0074,              -- radians per pixel
	CamDist = 22, CamFocusUp = 2,    -- third person orbit defaults, wheel zooms
	CamUpBlend = 3.2,                -- how fast the cam rolls onto a new surface. never teleport the view
	CamFocusRate = 14, CamRelaxRate = 6, -- focus chase speed + occlusion zoom-out smoothing
	GrabLock = 0.18,                 -- min secs between wall grabs, corner seams strobe without this
	GrabDist = 6,                    -- how far ahead a wall can be and still get climbed
	WrapDist = 8,                    -- how far back under a rim the edge wrap ray looks
	UpBlend = 10,                    -- normal blending speed on noisy terrain
	LeapUp = 30, LeapFwd = 16,       -- pounce impulse, along the normal + move dir
	LeapGravity = 85,                -- our own gravity, lighter than the worlds so arcs read nicer
	VoidY = -25,                     -- fell in the sea or off the world, respawn

	-- body feel
	PosFreq = 3.4,                   -- body position spring, Hz
	UpFreq = 2.2,                    -- render normal spring, how fast the body visually rolls onto walls
	ImpactKick = 1.4,                -- velocity shoved along the normal per landing foot. stride thump
	BreathRate = 1.7, BreathAmp = 0.07,
	BankGain = 0.0022,               -- roll per (yaw rate * speed), banks into mouse turns
	LeanGain = 0.005,                -- nose down per stud/s of stride speed
	AirPitchGain = 0.01,             -- nose follows vertical vel while airborne
	AirTuck = 2.4, AirTuckRate = 8,  -- feet pull into a crouch mid pounce
}

-- rig colors
local CHARCOAL = Color3.fromRGB(36, 38, 44)
local STEEL = Color3.fromRGB(86, 92, 104)
local EMBER = Color3.fromRGB(255, 70, 52)

---------------------------------- groundwork ----------------------------------

local UserInputService = game:GetService("UserInputService")
local player = Players.LocalPlayer

-- one folder for all rigs so a single exclusion entry hides every spider from
-- every ray at once (feet, climbing, camera)
local rigFolder = Instance.new("Folder")
rigFolder.Name = "SpiderRigs_" .. player.Name
rigFolder.Parent = workspace

local GAIT_PARAMS = RaycastParams.new()
GAIT_PARAMS.FilterType = Enum.RaycastFilterType.Exclude
GAIT_PARAMS.RespectCanCollide = true -- cant stand on non-collide decor

-- characters are invisible but rays still hit them, so exclude them manually
-- or feet start standing on ghost capsules. found that out the fun way
local function refreshExclusion()
	local list = { rigFolder }
	for _, plr in Players:GetPlayers() do
		if plr.Character then
			table.insert(list, plr.Character)
		end
	end
	GAIT_PARAMS.FilterDescendantsInstances = list
end

--------------------------------- math helpers ---------------------------------

-- kill the component of v along up. basically everything below leans on this
local function flatOn(v, up)
	return v - up * v:Dot(up)
end

-- rotate v around an axis. this is how mouse yaw spins the creature around
-- whatever its current "up" happens to be, wall or ceiling included
local function rotAround(v, axis, angle)
	return CFrame.fromAxisAngle(axis, angle):VectorToWorldSpace(v)
end

-- first candidate that survives being projected onto the tangent plane of up.
-- surface transitions need this: run straight at a wall and the new up IS
-- your old forward, so the naive projection collapses to zero and something
-- else has to decide what forward means now
local function tangentOn(up, ...)
	for _, candidate in { ... } do
		local t = flatOn(candidate, up)
		if t.Magnitude > 0.05 then
			return t.Unit
		end
	end
	local t = flatOn(Vector3.xAxis, up)
	if t.Magnitude < 0.05 then
		t = flatOn(Vector3.zAxis, up) -- nothing is parallel to both world x and z
	end
	return t.Unit
end

-- lerp for direction vectors, with a guard. near opposite normals lerp
-- through ~zero and normalizing that is NaN city
local function blendDir(a, b, t)
	local m = a:Lerp(b, math.min(t, 1))
	return m.Magnitude > 0.05 and m.Unit or b
end

-- quadratic bezier as a lerp of lerps. with the mid point hoisted along the
-- body normal it draws the foot swing arc on floors, walls, ceilings alike
local function bezier(p0, p1, p2, t)
	return p0:Lerp(p1, t):Lerp(p1:Lerp(p2, t), t)
end

--------------------------------- part factory ---------------------------------

-- rig parts are pure visuals. anchored so physics leaves them alone, CanQuery
-- off so rays and the camera see through them, CanCollide off so swing arcs
-- can pass through terrain lips
local function mkPart(parent, size, color, material)
	local p = Instance.new("Part")
	p.Size = size
	p.Color = color
	p.Material = material or Enum.Material.Metal
	p.Anchored = true
	p.CanCollide = false
	p.CanQuery = false
	p.CanTouch = false
	p.Parent = parent
	return p
end

local function mkBall(parent, diameter, color, material)
	local p = mkPart(parent, Vector3.one * diameter, color, material)
	p.Shape = Enum.PartType.Ball
	return p
end

------------------------------------ spring ------------------------------------
-- one damped oscillator for scalars AND vector3s, luau operator overloads
-- make that free. critically damped so it settles fast and never rings
local Spring = {}
Spring.__index = Spring

function Spring.new(value, freq)
	local k = (math.pi * 2 * freq) ^ 2 -- k = w^2 for unit mass
	return setmetatable({
		x = value,
		v = value * 0,                 -- *0 gives 0 or Vector3.zero depending, cheeky type generic init
		k = k,
		damp = 2 * math.sqrt(k),       -- 2*sqrt(k*m) with m=1, critical
	}, Spring)
end

function Spring:step(target, dt)
	-- hooke pulls toward the target, damping bleeds velocity. semi implicit
	-- euler (v first, then x) or it blows up at game framerates
	local accel = (target - self.x) * self.k - self.v * self.damp
	self.v += accel * dt
	self.x += self.v * dt
	return self.x
end

------------------------------------- limb -------------------------------------
-- one leg. bone lengths + hip socket + stance home, gait state (planted or
-- mid swing), and the 2 bone solver
local Limb = {}
Limb.__index = Limb

-- sx/sz are -1 or 1, which torso corner the leg hangs off. all the asymmetry
-- in the whole creature comes from these two signs
function Limb.new(model, sx, sz)
	local self = setmetatable({}, Limb)
	self.L1, self.L2 = CONFIG.UpperLength, CONFIG.LowerLength
	-- bones cant span less than |L1-L2| or more than L1+L2. the margins keep
	-- acos off its domain edges where poses snap
	self.minReach = math.abs(self.L1 - self.L2) + 0.1
	self.maxReach = self.L1 + self.L2 - 0.05
	self.hipOffset = Vector3.new(CONFIG.HipX * sx, CONFIG.HipY, CONFIG.HipZ * sz)
	self.homeOffset = Vector3.new(CONFIG.HomeX * sx, 0, CONFIG.HomeZ * sz)
	self.outDir = Vector3.new(sx, 0, 0) -- pole hint, knees fold away from the midline
	self.upper = mkPart(model, Vector3.new(0.95, 0.95, self.L1), STEEL)
	self.lower = mkPart(model, Vector3.new(0.58, 0.58, self.L2), STEEL)
	self.hipBall = mkBall(model, 1.5, CHARCOAL)
	self.kneeBall = mkBall(model, 1.05, CHARCOAL)
	self.footBall = mkBall(model, 0.8, Color3.fromRGB(20, 20, 22), Enum.Material.SmoothPlastic)
	self.plantPos = Vector3.zero
	self.footPos = Vector3.zero
	self.swing = nil
	return self
end

-- homes ride the body frame, so yawing in place drags them sideways and
-- triggers turn-in-place steps, and strafing gives a crab walk. both free
-- from the same drift rule that drives normal walking
function Limb:homeWorld(bodyCF)
	return bodyCF:PointToWorldSpace(self.homeOffset)
end

-- probe along the BODY normal, not world down. on a wall these rays fire
-- horizontally into the wall, thats literally what makes wall gait work.
-- starting ProbeUp above the sample catches ledges higher than the foot
function Limb:probe(at, up)
	local hit = workspace:Raycast(at + up * CONFIG.ProbeUp, -up * (CONFIG.ProbeUp + CONFIG.ProbeDown), GAIT_PARAMS)
	-- foothold has to roughly agree with the surface we cling to, and water
	-- isnt floor. nil = dont step there
	if hit and hit.Normal:Dot(up) > CONFIG.MaxFootSlope and hit.Material ~= Enum.Material.Water then
		return hit.Position
	end
	return nil
end

-- narrow stuff (pillars, fence tops, rims) is where spider legs look coolest
-- and also exactly where the raw home point overhangs the edge, the ray
-- whiffs and the leg freezes. so retry the probe closer and closer to the
-- bodys own support line and hug whatever sliver of surface actually exists
function Limb:probeNear(aim, up, center)
	local hit = self:probe(aim, up)
	if hit then
		return hit
	end
	hit = self:probe(center + (aim - center) * 0.5, up)
	if hit then
		return hit
	end
	return self:probe(center + (aim - center) * 0.15, up)
end

function Limb:beginStep(target)
	self.swing = { from = self.plantPos, target = target, age = 0 }
end

-- advance a mid air foot. returns true on the touchdown frame so the body
-- can react to the landing
function Limb:updateSwing(dt, up)
	local s = self.swing
	s.age += dt
	local t = math.min(s.age / CONFIG.SwingTime, 1)
	-- smoothstep, zero vel at liftoff and landing so plants dont look teleported
	local e = t * t * (3 - 2 * t)
	local mid = s.from:Lerp(s.target, 0.5) + up * CONFIG.StepHeight -- arc bulges along the body normal
	self.footPos = bezier(s.from, mid, s.target, e)
	if t >= 1 then
		self.plantPos = s.target
		self.footPos = s.target
		self.swing = nil
		return true
	end
	return false
end

-- 2 bone analytic IK. note theres no world axis anywhere in here, the whole
-- solve lives in the body frame, which is why the same code poses legs on
-- the floor, a wall, or hanging under an arch
function Limb:solve(bodyCF)
	local hip = bodyCF:PointToWorldSpace(self.hipOffset)
	local toFoot = self.footPos - hip
	local d = toFoot.Magnitude
	local dir = d > 1e-4 and toFoot / d or -bodyCF.UpVector -- guard, one NaN here poisons every cframe downstream
	-- clamp to what rigid bones can cover. the shin ends at the clamped point
	-- so bones never stretch, even when the target is out of reach
	local reach = math.clamp(d, self.minReach, self.maxReach)
	local target = hip + dir * reach
	-- the pole vector tips the bend plane up and outward so knees fold spider
	-- style. cosine rule gives you the bend angle but not which SIDE, this is
	-- what picks it
	local pole = bodyCF.UpVector + bodyCF:VectorToWorldSpace(self.outDir) * 0.4
	local plane = CFrame.lookAt(hip, target, pole) -- -z runs hip to foot, +y leans at the pole
	-- law of cosines for the hip corner of the hip/knee/foot triangle:
	-- cos A = (L1^2 + d^2 - L2^2) / (2 L1 d)
	local cosA = (self.L1 * self.L1 + reach * reach - self.L2 * self.L2) / (2 * self.L1 * reach)
	local hipAngle = math.acos(math.clamp(cosA, -1, 1))
	-- pitch the aim frame by that angle and its look vector swings from the
	-- foot line onto the femur. one rotation places the knee and the shin is
	-- forced after that. thats the whole trick of 2 bone ik
	local knee = hip + (plane * CFrame.Angles(hipAngle, 0, 0)).LookVector * self.L1
	self.hipBall.CFrame = CFrame.new(hip)
	self.kneeBall.CFrame = CFrame.new(knee)
	self.footBall.CFrame = CFrame.new(target)
	self.upper.CFrame = CFrame.lookAt(hip:Lerp(knee, 0.5), knee, pole)
	self.lower.CFrame = CFrame.lookAt(knee:Lerp(target, 0.5), target, pole)
end

------------------------------------ walker ------------------------------------
-- one spider. the local players walker also owns piloting (input, climbing,
-- camera), everyone elses just mirrors the pose their pilot wrote into the
-- replicated character root and re-derives gait + ik locally
local Walker = {}
Walker.__index = Walker

function Walker.new(plr, character)
	local self = setmetatable({}, Walker)
	self.rootPart = character:WaitForChild("HumanoidRootPart")
	self.model = Instance.new("Model")
	self.model.Name = "SpiderRig_" .. plr.Name
	self.pos = self.rootPart.Position
	self.up = WORLD_UP
	self.forward = tangentOn(WORLD_UP, self.rootPart.CFrame.LookVector)
	self.lastPilotPos = self.pos
	self.lastForward = self.forward
	self.wasGrounded = true
	self.bodySpring = Spring.new(self.pos, CONFIG.PosFreq)   -- render body chases the logical body
	self.upSpring = Spring.new(WORLD_UP, CONFIG.UpFreq)      -- visual roll onto new surfaces
	self.headSpring = Spring.new(0, 3)
	self.renderCF = self.rootPart.CFrame
	self.renderUp = WORLD_UP

	-- torso + armor plate + head + eyes. just enough silhouette to sell it
	self.body = mkPart(self.model, CONFIG.BodySize, CHARCOAL)
	self.plate = mkPart(self.model, Vector3.new(CONFIG.BodySize.X * 0.78, 0.5, CONFIG.BodySize.Z * 0.85), STEEL)
	self.plateOffset = CFrame.new(0, CONFIG.BodySize.Y * 0.5 + 0.2, 0)
	self.head = mkPart(self.model, Vector3.new(2.6, 1.7, 2.2), CHARCOAL)
	self.headOffset = CFrame.new(0, 0.45, -(CONFIG.BodySize.Z * 0.5 + 0.9))
	self.eyeL = mkBall(self.model, 0.55, EMBER, Enum.Material.Neon)
	self.eyeR = mkBall(self.model, 0.55, EMBER, Enum.Material.Neon)
	local glow = Instance.new("PointLight")
	glow.Color = EMBER
	glow.Range = 11
	glow.Brightness = 1.5
	glow.Parent = self.head
	self.model.PrimaryPart = self.body

	self.limbs = {
		Limb.new(self.model, -1, -1), Limb.new(self.model, 1, -1),
		Limb.new(self.model, -1, 1), Limb.new(self.model, 1, 1),
	}
	-- trot pairing, diagonal legs move together. they cancel each others
	-- torque about the center, same reason real quadrupeds trot
	self.gaitPairs = { { self.limbs[1], self.limbs[4] }, { self.limbs[2], self.limbs[3] } }

	self.model.Parent = rigFolder

	-- plant everything now so frame 1 is a stance, not a fold-out from origin
	local logical = CFrame.lookAt(self.pos, self.pos + self.forward, self.up)
	for _, limb in self.limbs do
		local home = limb:homeWorld(logical)
		limb.plantPos = limb:probe(home, self.up) or home - self.up * CONFIG.Hover
		limb.footPos = limb.plantPos
	end
	return self
end

-- shared per frame core (gait, dynamics, render) given a logical frame,
-- either from the local climb solver or someone elses replicated root
function Walker:frameUpdate(dt, now, grounded, vel, logical)
	self.pos = logical.Position
	self.up = logical.UpVector
	self.forward = logical.LookVector
	local up = self.up
	local renderUpTarget = up

	if grounded then
		if not self.wasGrounded or self.forceReplant then
			-- touchdown / surface swap frame. replant every foot at once, no
			-- arcs (a spider regripping a new surface instantly looks right,
			-- feet arcing toward a plane the body just rotated onto looks
			-- broken) and let the body spring inherit the approach velocity
			-- so hooke turns the landing into a dip and rebound for free
			self.forceReplant = false
			self.bodySpring.v = vel
			for _, limb in self.limbs do
				limb.swing = nil
				local hit = limb:probeNear(limb:homeWorld(logical), up, self.pos)
				if hit then
					limb.plantPos = hit
					limb.footPos = hit
				end
			end
		end
		local landed = 0
		-- airborne feet first so a pair finishing this frame frees the gate,
		-- and retarget every frame so landings track a turning body
		for _, limb in self.limbs do
			if limb.swing then
				local fresh = limb:probeNear(limb:homeWorld(logical) + vel * CONFIG.LeadTime, up, self.pos)
				if fresh then
					limb.swing.target = fresh
				end
				if limb:updateSwing(dt, up) then
					landed += 1
				end
			end
		end
		-- trot gate, one diagonal pair in the air at a time. when both are
		-- down, the pair whose worst foot drifted furthest goes next
		local aBusy = self.gaitPairs[1][1].swing or self.gaitPairs[1][2].swing
		local bBusy = self.gaitPairs[2][1].swing or self.gaitPairs[2][2].swing
		if not aBusy and not bBusy then
			-- ask each foot where it COULD land (the cascade may pull the
			-- spot inboard on narrow surfaces) and measure against that.
			-- measuring drift against the raw home instead strobes forever on
			-- anything narrower than the stance, aka every pillar climb
			local bestPair, bestErr = nil, CONFIG.StepTrigger
			for _, pair in self.gaitPairs do
				for _, limb in pair do
					local cand = limb:probeNear(limb:homeWorld(logical) + vel * CONFIG.LeadTime, up, self.pos)
					limb.stepCandidate = cand
					if cand then
						-- drift is tangent plane only, distance along the
						-- normal is the hover springs problem not the gaits
						local err = flatOn(cand - limb.plantPos, up).Magnitude
						if err > bestErr then
							bestErr, bestPair = err, pair
						end
					end
				end
			end
			if bestPair then
				for _, limb in bestPair do
					if limb.stepCandidate then -- nil means a hole or a rim, staying planted beats stepping into air
						limb:beginStep(limb.stepCandidate)
					end
				end
			end
		end
		if landed > 0 then
			self.bodySpring.v -= up * (CONFIG.ImpactKick * landed) -- stride weight, dip along the normal + spring back
		end
		-- cheap plane fit: cross the foot quads diagonals, flip to our side of
		-- the surface. feeding it into the render normal lets the torso ride
		-- bumps the logical frame doesnt know about
		local p1, p2 = self.limbs[1].plantPos, self.limbs[2].plantPos
		local p3, p4 = self.limbs[3].plantPos, self.limbs[4].plantPos
		local raw = (p2 - p3):Cross(p1 - p4)
		if raw:Dot(up) < 0 then
			raw = -raw
		end
		if raw.Magnitude > 1e-3 then
			renderUpTarget = (up * 2 + raw.Unit).Unit -- mostly the climb solvers normal, seasoned with foot terrain
		end
	else
		-- airborne. no gait, just ease every foot into the pounce tuck
		for _, limb in self.limbs do
			limb.swing = nil
			local tuck = limb:homeWorld(logical) - up * CONFIG.AirTuck
			limb.footPos = limb.footPos:Lerp(tuck, math.min(dt * CONFIG.AirTuckRate, 1))
			limb.plantPos = limb.footPos -- keeps drift + the support plane sane for the landing frame
		end
		self.bodySpring.x = self.pos -- body rides the ballistic path exactly
		self.bodySpring.v = vel      -- and remembers its velocity for the touchdown squash
	end
	self.wasGrounded = grounded

	-- render layers: springed position, springed normal, cosmetic angles on top
	local renderPos = grounded and self.bodySpring:step(self.pos, dt) or self.pos
	local su = self.upSpring:step(renderUpTarget, dt)
	self.renderUp = su.Magnitude > 0.05 and su.Unit or up
	-- signed yaw rate about the normal, from how far forward swung since last
	-- frame. sin of the angle is the cross products normal component
	local yawRate = math.asin(math.clamp(self.lastForward:Cross(self.forward):Dot(up), -1, 1)) / math.max(dt, 1e-3)
	self.lastForward = self.forward
	local speed = flatOn(vel, up).Magnitude
	local breathe = math.sin(now * CONFIG.BreathRate) * CONFIG.BreathAmp -- always alive, even at a dead stop
	local roll = math.clamp(-yawRate * speed * CONFIG.BankGain, -0.22, 0.22)
	local pitch = -speed * CONFIG.LeanGain
	if not grounded then
		pitch += math.clamp(vel.Y * CONFIG.AirPitchGain, -0.35, 0.3) -- nose up on the rise, down into the fall
	end
	local frame = CFrame.lookAt(renderPos, renderPos + tangentOn(self.renderUp, self.forward), self.renderUp)
	local render = frame * CFrame.new(0, breathe, 0) * CFrame.Angles(pitch, 0, roll)
	self.renderCF = render
	self.body.CFrame = render
	self.plate.CFrame = render * self.plateOffset
	-- head leads the turn a little, smoothed so glances feel deliberate
	local lookYaw = self.headSpring:step(math.clamp(yawRate * 0.1, -0.5, 0.5), dt)
	local headCF = render * self.headOffset * CFrame.Angles(0, lookYaw, 0)
	self.head.CFrame = headCF
	self.eyeL.CFrame = headCF * CFrame.new(-0.62, 0.2, -1.0)
	self.eyeR.CFrame = headCF * CFrame.new(0.62, 0.2, -1.0)
	-- pose all 8 bones off the SWAYING frame, so the legs visibly soak up
	-- every bob, bank and impact while the feet stay glued down
	for _, limb in self.limbs do
		limb:solve(render)
	end
end

-- remote spiders. their pilot already solved climbing on their machine and
-- the answer showed up in the root cframe. rebuild everything visual from
-- that one transform
function Walker:mirrorStep(dt, now)
	local logical = self.rootPart.CFrame
	local vel = (logical.Position - self.lastPilotPos) / math.max(dt, 1e-3)
	self.lastPilotPos = logical.Position
	-- grounded = some surface within hover reach of the belly
	local hit = workspace:Raycast(logical.Position, -logical.UpVector * (CONFIG.Hover + 2.5), GAIT_PARAMS)
	self:frameUpdate(dt, now, hit ~= nil, vel, logical)
end

------------------------------- piloting (local) -------------------------------

local mouseDX, mouseDY, wheel = 0, 0, 0
UserInputService.InputChanged:Connect(function(input)
	if input.UserInputType == Enum.UserInputType.MouseMovement then
		mouseDX += input.Delta.X
		mouseDY += input.Delta.Y
	elseif input.UserInputType == Enum.UserInputType.MouseWheel then
		wheel += input.Position.Z
	end
end)

-- every pilot side surface query goes through this. rigs and avatars are
-- excluded by the params, water gets refused here. a spider standing on the
-- open sea kills the whole thing instantly
local function surfaceCast(origin, dir)
	local hit = workspace:Raycast(origin, dir, GAIT_PARAMS)
	if hit and hit.Material ~= Enum.Material.Water then
		return hit
	end
	return nil
end

-- initial attach, one long world-down ray so spawning doesnt free fall
function Walker:seed()
	self.camPitch = -0.45
	self.camDist = CONFIG.CamDist
	self.camCurDist = CONFIG.CamDist
	self.camUp = WORLD_UP
	self.camFocus = self.pos
	self.grabLock = 0
	self.forceReplant = false
	self.velAir = Vector3.zero
	self.prevJump = false
	self.lastMove = self.forward
	local hit = surfaceCast(self.pos + WORLD_UP * 5, -WORLD_UP * 80)
	self.attached = hit ~= nil
	if hit then
		self.pos = hit.Position + WORLD_UP * CONFIG.Hover
	end
	self.spawnPos = self.pos + WORLD_UP * 2
end

function Walker:pilot(dt, now)
	-- IsKeyDown cant tell chat from input, so gate everything while a
	-- textbox has focus
	local typing = UserInputService:GetFocusedTextBox() ~= nil

	-- 1) look: mouse yaw spins the WHOLE spider around its current surface
	-- normal, so steering feels identical on floor/wall/ceiling. pitch only
	-- orbits the camera, never touches the creature
	if not typing then
		self.forward = rotAround(self.forward, self.up, -mouseDX * CONFIG.MouseSens)
		self.camPitch = math.clamp(self.camPitch - mouseDY * CONFIG.MouseSens, -1.25, 0.75)
		self.camDist = math.clamp(self.camDist - wheel * 3, 8, 40)
	end
	mouseDX, mouseDY, wheel = 0, 0, 0

	-- 2) move intent, wasd on the tangent plane of whatever were stuck to
	local right = self.forward:Cross(self.up) -- matches cframes RightVector = look x up
	local move = Vector3.zero
	if not typing then
		if UserInputService:IsKeyDown(Enum.KeyCode.W) then move += self.forward end
		if UserInputService:IsKeyDown(Enum.KeyCode.S) then move -= self.forward end
		if UserInputService:IsKeyDown(Enum.KeyCode.D) then move += right end
		if UserInputService:IsKeyDown(Enum.KeyCode.A) then move -= right end
	end
	if move.Magnitude > 0.01 then
		move = move.Unit
		self.lastMove = move
	else
		move = Vector3.zero
	end

	-- 3) pounce. spiders dont politely step off ledges. impulse leaves along
	-- the surface normal, so leaping off a wall throws you AWAY from the
	-- wall, which is what your brain expects
	local jumping = not typing and UserInputService:IsKeyDown(Enum.KeyCode.Space)
	if jumping and not self.prevJump and self.attached then
		self.attached = false
		self.velAir = self.up * CONFIG.LeapUp + move * CONFIG.LeapFwd
	end
	self.prevJump = jumping

	if self.attached then
		-- 4a) climb solver, three rays decide everything
		if move ~= Vector3.zero then
			-- ray 1, wall grab. a face blocking the path becomes the new
			-- floor, and forward becomes "up the wall" (the old up projected
			-- onto it) since the old forward projects to zero there
			local ahead = now >= self.grabLock and surfaceCast(self.pos, move * CONFIG.GrabDist) or nil
			if ahead and ahead.Normal:Dot(self.up) < 0.55 and ahead.Normal:Dot(move) < -0.2 then
				local oldUp = self.up
				self.up = ahead.Normal
				self.forward = tangentOn(self.up, oldUp, self.forward)
				self.grabLock = now + CONFIG.GrabLock -- corner seams alternate faces, rate limit or it strobes
				self.forceReplant = true -- all four feet regrip the new floor this frame
			else
				self.pos += move * CONFIG.MoveSpeed * dt
			end
		end
		-- ray 2, stick. re-find the surface under the moved body. BLEND the
		-- normal instead of snapping, terrain normals are noisy
		local ghit = surfaceCast(self.pos + self.up * CONFIG.ProbeUp, -self.up * (CONFIG.ProbeUp + CONFIG.ProbeDown))
		if ghit then
			self.up = blendDir(self.up, ghit.Normal, dt * CONFIG.UpBlend)
			self.pos = ghit.Position + self.up * CONFIG.Hover
		else
			-- ray 3, edge wrap. nothing under us, so look BACK under the rim
			-- we just walked over. its far face becomes the floor and forward
			-- now points down around the corner (minus old up projected)
			local back = self.lastMove
			local whit = surfaceCast(self.pos - self.up * (CONFIG.Hover + 1.5), -back * CONFIG.WrapDist)
			if whit then
				local oldUp = self.up
				self.up = whit.Normal
				self.forward = tangentOn(self.up, -oldUp, self.forward)
				self.pos = whit.Position + self.up * CONFIG.Hover
				self.forceReplant = true -- feet chase the body around the corner immediately
			else
				self.attached = false -- actual void. fall, keep tangent momentum
				self.velAir = move * CONFIG.MoveSpeed - self.up * 2
			end
		end
		self.forward = tangentOn(self.up, self.forward, self.lastMove)
	else
		-- 4b) ballistic. integrate our own gravity (the capsules physics is
		-- off), steer a little, attach to the FIRST face the motion ray
		-- crosses no matter its orientation, floor wall or ceiling
		self.velAir += Vector3.new(0, -CONFIG.LeapGravity * dt, 0)
		local step = self.velAir * dt + move * (CONFIG.MoveSpeed * 0.35 * dt)
		if step.Magnitude > 1e-4 then
			local ahit = surfaceCast(self.pos, step + step.Unit * 1.6)
			if ahit then
				local oldUp = self.up
				self.attached = true
				self.up = ahit.Normal
				self.forward = tangentOn(self.up, self.velAir, self.forward, oldUp)
				self.pos = ahit.Position + self.up * CONFIG.Hover
			else
				self.pos += step
				self.up = blendDir(self.up, WORLD_UP, dt * 3) -- roll upright while flying
				self.forward = tangentOn(self.up, self.forward)
			end
		end
		if self.pos.Y < CONFIG.VoidY then
			-- swam too deep or fell off the world, back to the island
			self.pos = self.spawnPos
			self.velAir = Vector3.zero
			self.up = WORLD_UP
			self.attached = false
		end
	end

	-- 5) commit. the character root carries the whole pose to the server and
	-- every other client through plain character replication. this line is
	-- the entire netcode
	local logical = CFrame.lookAt(self.pos, self.pos + self.forward, self.up)
	self.rootPart.CFrame = logical
	self.rootPart.AssemblyLinearVelocity = Vector3.zero -- dont let physics accumulate gravity between our writes
	self.rootPart.AssemblyAngularVelocity = Vector3.zero
	local vel = (self.pos - self.lastPilotPos) / math.max(dt, 1e-3)
	self.lastPilotPos = self.pos
	self:frameUpdate(dt, now, self.attached, self.attached and vel or self.velAir, logical)

	-- 6) camera, derived from its OWN smoothed copy of the spider frame. the
	-- creature is allowed to snap onto surfaces, breathe, thump and bank,
	-- but none of that reaches the viewport raw. the cam up sweeps onto a
	-- new surface over a few tenths of a second (a wall still becomes the
	-- floor, it just rolls there instead of teleporting) and the focus
	-- chases the logical body while ignoring the cosmetic bob layers
	local cam = workspace.CurrentCamera
	cam.CameraType = Enum.CameraType.Scriptable
	-- reassert every frame, focus changes reset it. cursor gets released
	-- while typing so chat and menus stay usable
	UserInputService.MouseBehavior = typing and Enum.MouseBehavior.Default or Enum.MouseBehavior.LockCenter
	UserInputService.MouseIconEnabled = typing
	self.camUp = blendDir(self.camUp, self.up, dt * CONFIG.CamUpBlend)
	local camFwd = tangentOn(self.camUp, self.forward, self.lastMove)
	self.camFocus = self.camFocus:Lerp(self.pos + self.camUp * CONFIG.CamFocusUp, math.min(dt * CONFIG.CamFocusRate, 1))
	local dir = camFwd * math.cos(self.camPitch) + self.camUp * math.sin(self.camPitch)
	-- occlusion at two speeds. pull IN instantly so geometry never clips the
	-- view, relax back OUT slow so the zoom doesnt pop
	local wantDist = self.camDist
	local block = workspace:Raycast(self.camFocus, -dir * self.camDist, GAIT_PARAMS)
	if block then
		wantDist = math.max((block.Position - self.camFocus).Magnitude - 0.8, 2.5)
	end
	if wantDist < self.camCurDist then
		self.camCurDist = wantDist
	else
		self.camCurDist += (wantDist - self.camCurDist) * math.min(dt * CONFIG.CamRelaxRate, 1)
	end
	local eye = self.camFocus - dir * self.camCurDist
	cam.CFrame = CFrame.lookAt(eye, self.camFocus, self.camUp)
end

------------------------------- character wiring -------------------------------

local walkers = {} -- player -> Walker

-- avatars vanish CLIENT side. property writes on instances you dont own
-- never replicate, but every client runs this same file, so everyone hides
-- everyone locally and all anyone ever sees is spiders
local function hideVisual(inst)
	if inst:IsA("BasePart") then
		inst.Transparency = 1 -- local-only write on stuff we dont own
		inst.LocalTransparencyModifier = 1 -- and the compositing override, in case something refreshes Transparency
		inst.CastShadow = false -- invisible avatar shouldnt cast a humanoid shaped shadow
	elseif inst:IsA("Decal") then
		inst.Transparency = 1
	end
end

local function hideCharacter(character)
	for _, d in character:GetDescendants() do
		hideVisual(d)
	end
	character.DescendantAdded:Connect(hideVisual) -- accessories stream in late, catch those too
end

-- the local capsule becomes pure cargo. the climb solver owns position, so
-- physics forces and collisions would only fight it. we simulate our own
-- character, so these client side changes are authoritative for motion
local function prepareOwnCharacter(character)
	local humanoid = character:WaitForChild("Humanoid")
	humanoid.PlatformStand = true -- no balancing, no walk forces, no state flapping
	humanoid.AutoRotate = false
	local animate = character:FindFirstChild("Animate")
	if animate then
		animate.Enabled = false -- no point animating an invisible rig
	end
	for _, d in character:GetDescendants() do
		if d:IsA("BasePart") then
			d.CanCollide = false
		end
	end
end

local function onCharacter(plr, character)
	refreshExclusion()
	if walkers[plr] then
		walkers[plr].model:Destroy()
	end
	hideCharacter(character)
	local w = Walker.new(plr, character)
	if plr == player then
		prepareOwnCharacter(character)
		w:seed()
	end
	walkers[plr] = w
end

local function drop(plr)
	local w = walkers[plr]
	if w then
		walkers[plr] = nil
		w.model:Destroy()
	end
	refreshExclusion()
end

local function bind(plr)
	plr.CharacterAdded:Connect(function(character)
		onCharacter(plr, character)
	end)
	plr.CharacterRemoving:Connect(function()
		drop(plr)
	end)
	if plr.Character then
		onCharacter(plr, plr.Character)
	end
end

Players.PlayerAdded:Connect(bind)
Players.PlayerRemoving:Connect(drop)
for _, plr in Players:GetPlayers() do
	bind(plr)
end

-- camera priority + 1 so our write lands AFTER the default camera scripts
-- every frame and ours is the one that sticks. dt is clamped because the
-- spring integrator goes unstable once omega*dt gets past about 2
RunService:BindToRenderStep("ProceduralIKSpider", Enum.RenderPriority.Camera.Value + 1, function(dt)
	local now = os.clock()
	local step = math.min(dt, 1 / 20)
	for plr, w in walkers do
		if w.rootPart.Parent then
			if plr == player then
				w:pilot(step, now)
			else
				w:mirrorStep(step, now)
			end
		end
	end
end)
