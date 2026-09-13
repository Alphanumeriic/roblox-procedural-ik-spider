-- Connected Discord-GitHub | Discord: yousif.dev | Roblox: Yousxif
--==============================================================================
-- PROCEDURAL DYNAMIC IK SPIDER: YOU ARE THE SPIDER
-- One file. Every player's avatar is hidden and replaced by a quadruped spider
-- mech built from primitives at runtime, driven camera first: the mouse yaws
-- the whole creature around whatever surface it clings to, WASD moves on that
-- surface's tangent plane, Space pounces, and any face you run into or land on
-- becomes the new floor: walls, cliff sides, the underside of an arch.
--   1) Analytic two bone inverse kinematics per leg (law of cosines inside a
--      pole vector bend plane), written frame relative so it works upside down.
--   2) A raycast trot gait: diagonal pairs, drift triggered steps, quadratic
--      Bezier swing arcs, predictive landings, all on arbitrary surfaces.
--   3) Spring damped body dynamics (Hooke's law): hover, stride and landing
--      impacts, support plane tilt, turn banking, breathing.
--   4) A surface attachment solver: wall grab, edge wrap, ballistic leaps.
-- Architecture: pure client, zero server code, zero RemoteEvents. Every client
-- runs this same file: it hides every avatar locally, pilots its OWN spider
-- kinematically (zero input latency) and commits the pose into its character
-- root part, which Roblox already replicates because the client owns it; each
-- client then re-derives gait and IK for everyone else from those roots. The
-- character root is the entire network protocol.
-- Usage: paste into a LocalScript in StarterPlayer > StarterPlayerScripts,
-- press Play. Mouse steers, WASD walks, Space pounces, wheel zooms.
--==============================================================================

local RunService = game:GetService("RunService")
local Players = game:GetService("Players")

local WORLD_UP = Vector3.yAxis

--=============================== TUNING =======================================
-- Every number that shapes the creature or its feel lives here and nowhere else.
local CONFIG = {
	-- skeleton
	UpperLength = 4.0,               -- femur length in studs
	LowerLength = 5.2,               -- tibia length: longer than the femur reads as arachnid
	BodySize = Vector3.new(6.5, 2.4, 8.5),
	HipX = 3.1, HipY = -0.55, HipZ = 3.4, -- hip pivots sit just inside the torso corners
	HomeX = 6.8, HomeZ = 5.4,        -- neutral stance splay, measured from the body center
	Hover = 4.5,                     -- body offset above the surface, measured along its normal

	-- gait
	StepTrigger = 2.4,               -- a step fires once a home point drifts this far from its planted foot
	SwingTime = 0.18,                -- airborne seconds per step: cadence emerges from drift, not a clock
	StepHeight = 1.9,                -- Bezier control point lift along the body normal
	LeadTime = 0.25,                 -- feet land this many seconds AHEAD of the motion (predictive placement)
	ProbeUp = 4, ProbeDown = 10,     -- foot ray span along the body normal
	MaxFootSlope = 0.3,              -- footholds must roughly agree with the body normal (dot product floor)

	-- piloting
	MoveSpeed = 14,                  -- studs per second along the tangent plane
	MouseSens = 0.0074,              -- radians of yaw or camera pitch per pixel of mouse travel
	CamDist = 22, CamFocusUp = 2,    -- default third person orbit, wheel zoomable
	CamUpBlend = 3.2,                -- per second sweep of the camera roll onto a new surface: the view never teleports
	CamFocusRate = 14, CamRelaxRate = 6, -- focus chase speed and occlusion zoom-out smoothing, both per second
	GrabLock = 0.18,                 -- seconds between wall grabs: stops corner seams from strobing the view
	GrabDist = 6,                    -- how far ahead a face can be and still be climbed onto
	WrapDist = 8,                    -- how far back under a rim the edge wrap ray searches
	UpBlend = 10,                    -- per second blend of the body normal onto noisy terrain normals
	LeapUp = 30, LeapFwd = 16,       -- pounce impulse along the surface normal and move direction
	LeapGravity = 85,                -- ballistic gravity: lighter than world gravity for readable arcs
	VoidY = -25,                     -- fell in the sea or off the world: respawn on the island

	-- body dynamics
	PosFreq = 3.4,                   -- body position spring stiffness as a frequency in Hz
	UpFreq = 2.2,                    -- render normal spring: how fast the body visually rolls onto walls
	ImpactKick = 1.4,                -- velocity injected along the normal per landing foot: stride thump
	BreathRate = 1.7, BreathAmp = 0.07,
	BankGain = 0.0022,               -- roll per (yaw rate times speed): banks into mouse turns
	LeanGain = 0.005,                -- nose down pitch per stud/s of stride speed
	AirPitchGain = 0.01,             -- nose follows vertical velocity while ballistic
	AirTuck = 2.4, AirTuckRate = 8,  -- feet pull into a pounce crouch while airborne
}

-- palette for the generated rig
local CHARCOAL = Color3.fromRGB(36, 38, 44)
local STEEL = Color3.fromRGB(86, 92, 104)
local EMBER = Color3.fromRGB(255, 70, 52)

--============================== GROUNDWORK ====================================

local UserInputService = game:GetService("UserInputService")
local player = Players.LocalPlayer

-- all rigs live under one folder so a single exclusion entry hides every
-- spider from every ray (feet, climbing, camera) at once
local rigFolder = Instance.new("Folder")
rigFolder.Name = "SpiderRigs_" .. player.Name
rigFolder.Parent = workspace

local GAIT_PARAMS = RaycastParams.new()
GAIT_PARAMS.FilterType = Enum.RaycastFilterType.Exclude
GAIT_PARAMS.RespectCanCollide = true -- decor with collision off is not a valid surface

-- characters are invisible but still raycastable, so they must be excluded
-- explicitly or feet would try to stand on ghost capsules
local function refreshExclusion()
	local list = { rigFolder }
	for _, plr in Players:GetPlayers() do
		if plr.Character then
			table.insert(list, plr.Character)
		end
	end
	GAIT_PARAMS.FilterDescendantsInstances = list
end

--============================ MATH HELPERS ====================================

-- Projects v onto the plane perpendicular to up by subtracting its normal
-- component: the workhorse that makes every rule below surface agnostic.
local function flatOn(v, up)
	return v - up * v:Dot(up)
end

-- Rotates a vector around an axis: how mouse yaw spins the creature around
-- whatever its current "up" happens to be, wall or ceiling included.
local function rotAround(v, axis, angle)
	return CFrame.fromAxisAngle(axis, angle):VectorToWorldSpace(v)
end

-- Returns the first candidate that survives projection onto the tangent plane
-- of up. Surface transitions need this: when the new up IS the old forward
-- (running straight at a wall), the naive projection collapses to zero and a
-- fallback axis has to define what "forward" now means.
local function tangentOn(up, ...)
	for _, candidate in { ... } do
		local t = flatOn(candidate, up)
		if t.Magnitude > 0.05 then
			return t.Unit
		end
	end
	local t = flatOn(Vector3.xAxis, up)
	if t.Magnitude < 0.05 then
		t = flatOn(Vector3.zAxis, up) -- no direction is parallel to both world X and Z
	end
	return t.Unit
end

-- Lerp for direction vectors with a guard: blending between near opposite
-- normals passes close to the zero vector, and normalizing that is a NaN.
local function blendDir(a, b, t)
	local m = a:Lerp(b, math.min(t, 1))
	return m.Magnitude > 0.05 and m.Unit or b
end

-- Quadratic Bezier as a lerp of lerps; with the middle point hoisted along the
-- body normal it traces the foot swing arc on floors, walls and ceilings alike.
local function bezier(p0, p1, p2, t)
	return p0:Lerp(p1, t):Lerp(p1:Lerp(p2, t), t)
end

--============================ PART FACTORY ====================================

-- Rig pieces are kinematic set dressing: Anchored so physics never fights the
-- solver, CanQuery false so rays and the camera see through them, CanCollide
-- false so swing arcs pass through terrain lips harmlessly.
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

--=============================== SPRING =======================================
-- One damped harmonic oscillator serving scalars AND Vector3s through Luau
-- operator overloads. Critically damped: fast settle, zero ringing.
local Spring = {}
Spring.__index = Spring

function Spring.new(value, freq)
	local k = (math.pi * 2 * freq) ^ 2 -- stiffness from frequency: k equals omega squared for unit mass
	return setmetatable({
		x = value,
		v = value * 0,                 -- times zero yields 0 OR Vector3.zero: type generic init
		k = k,
		damp = 2 * math.sqrt(k),       -- critical damping: c = 2*sqrt(k*m) with m = 1
	}, Spring)
end

function Spring:step(target, dt)
	-- Hooke's law accelerates toward the target (a = k * error) while damping
	-- bleeds velocity (minus c*v); semi implicit Euler (velocity first, then
	-- position) keeps the integration stable at game frame rates.
	local accel = (target - self.x) * self.k - self.v * self.damp
	self.v += accel * dt
	self.x += self.v * dt
	return self.x
end

--================================ LIMB ========================================
-- One leg: geometry (bone lengths, hip socket, stance home), gait state
-- (planted point or an in flight swing), and the two bone IK solver.
local Limb = {}
Limb.__index = Limb

-- sx and sz are -1 or 1: which torso corner the leg hangs off. All asymmetry
-- in the whole creature comes from these two signs.
function Limb.new(model, sx, sz)
	local self = setmetatable({}, Limb)
	self.L1, self.L2 = CONFIG.UpperLength, CONFIG.LowerLength
	-- rigid bones can never span less than |L1-L2| nor more than L1+L2; the
	-- margins keep acos away from its domain edges where poses snap
	self.minReach = math.abs(self.L1 - self.L2) + 0.1
	self.maxReach = self.L1 + self.L2 - 0.05
	self.hipOffset = Vector3.new(CONFIG.HipX * sx, CONFIG.HipY, CONFIG.HipZ * sz)
	self.homeOffset = Vector3.new(CONFIG.HomeX * sx, 0, CONFIG.HomeZ * sz)
	self.outDir = Vector3.new(sx, 0, 0) -- pole hint: knees must fold AWAY from the torso midline
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

-- Homes ride the body frame, so yawing in place drags them sideways and
-- provokes turn in place steps, and strafing produces a crab walk, both for
-- free from the same drift rule that drives ordinary walking.
function Limb:homeWorld(bodyCF)
	return bodyCF:PointToWorldSpace(self.homeOffset)
end

-- Foot probe along the BODY normal, not world down: on a wall these rays fire
-- horizontally into the wall, which is exactly what makes wall gait work.
-- Starting ProbeUp above the sample catches ledges higher than the foot.
function Limb:probe(at, up)
	local hit = workspace:Raycast(at + up * CONFIG.ProbeUp, -up * (CONFIG.ProbeUp + CONFIG.ProbeDown), GAIT_PARAMS)
	-- a foothold must roughly agree with the surface we cling to, and water is
	-- not floor; callers treat nil as "do not step there"
	if hit and hit.Normal:Dot(up) > CONFIG.MaxFootSlope and hit.Material ~= Enum.Material.Water then
		return hit.Position
	end
	return nil
end

-- Narrow surfaces (a pillar, a fence top, a rim) are exactly where spider legs
-- sell the fantasy, and exactly where a raw home point overhangs the surface
-- so its ray misses everything and the leg freezes. Retry the probe
-- progressively closer to the body's own support line, so feet hug whatever
-- sliver of surface actually exists instead of giving up.
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

-- Advances an in flight foot. Returns true on the touchdown frame so the body
-- can convert the landing into an impact reaction.
function Limb:updateSwing(dt, up)
	local s = self.swing
	s.age += dt
	local t = math.min(s.age / CONFIG.SwingTime, 1)
	-- smoothstep easing (3t^2 - 2t^3): zero velocity at lift off AND landing,
	-- peak speed mid flight, so plants never look teleported
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

-- Two bone analytic IK. Note there is no world axis anywhere in here: the
-- whole solve happens in the body frame, which is why the same code poses
-- legs on the floor, a wall, or hanging under an arch.
function Limb:solve(bodyCF)
	local hip = bodyCF:PointToWorldSpace(self.hipOffset)
	local toFoot = self.footPos - hip
	local d = toFoot.Magnitude
	local dir = d > 1e-4 and toFoot / d or -bodyCF.UpVector -- degenerate guard: a NaN would poison every CFrame downstream
	-- clamp the span to what rigid bones can cover; the shin terminates at the
	-- clamped point so bones NEVER stretch, even when the target is unreachable
	local reach = math.clamp(d, self.minReach, self.maxReach)
	local target = hip + dir * reach
	-- the pole vector biases the bend plane up and outward so knees fold like
	-- spider legs; the cosine rule fixes the bend ANGLE but not the bend SIDE,
	-- and this is what disambiguates it
	local pole = bodyCF.UpVector + bodyCF:VectorToWorldSpace(self.outDir) * 0.4
	local plane = CFrame.lookAt(hip, target, pole) -- minus Z runs hip to foot, plus Y leans toward the pole
	-- law of cosines rearranged for the angle at the hip corner of the
	-- hip/knee/foot triangle: cos(A) = (L1^2 + d^2 - L2^2) / (2 * L1 * d)
	local cosA = (self.L1 * self.L1 + reach * reach - self.L2 * self.L2) / (2 * self.L1 * reach)
	local hipAngle = math.acos(math.clamp(cosA, -1, 1))
	-- pitching the aim frame by the interior angle swings its look vector from
	-- the foot line onto the femur line: one rotation places the knee, and the
	-- shin is then fully determined, the entire trick of two bone IK
	local knee = hip + (plane * CFrame.Angles(hipAngle, 0, 0)).LookVector * self.L1
	self.hipBall.CFrame = CFrame.new(hip)
	self.kneeBall.CFrame = CFrame.new(knee)
	self.footBall.CFrame = CFrame.new(target)
	self.upper.CFrame = CFrame.lookAt(hip:Lerp(knee, 0.5), knee, pole)
	self.lower.CFrame = CFrame.lookAt(knee:Lerp(target, 0.5), target, pole)
end

--=============================== WALKER =======================================
-- One spider. The local player's walker also owns piloting (input, climbing,
-- camera); everyone else's walker mirrors the pose their pilot wrote into
-- their replicated character root and re-derives gait and IK locally.
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
	self.bodySpring = Spring.new(self.pos, CONFIG.PosFreq)   -- Vector3 spring: render body chases the logical body
	self.upSpring = Spring.new(WORLD_UP, CONFIG.UpFreq)      -- Vector3 spring: visual roll onto new surfaces
	self.headSpring = Spring.new(0, 3)
	self.renderCF = self.rootPart.CFrame
	self.renderUp = WORLD_UP

	-- torso, armor plate, head and eyes: enough silhouette to sell a creature
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
	-- trot pairing: diagonal legs move together because they cancel each
	-- other's torque about the body center, exactly why real quadrupeds trot
	self.gaitPairs = { { self.limbs[1], self.limbs[4] }, { self.limbs[2], self.limbs[3] } }

	self.model.Parent = rigFolder

	-- start planted so frame one is a stance, not a fold up from the origin
	local logical = CFrame.lookAt(self.pos, self.pos + self.forward, self.up)
	for _, limb in self.limbs do
		local home = limb:homeWorld(logical)
		limb.plantPos = limb:probe(home, self.up) or home - self.up * CONFIG.Hover
		limb.footPos = limb.plantPos
	end
	return self
end

-- Shared per frame core: gait, dynamics and rendering, given a logical frame
-- (from the local climb solver, or from a remote player's replicated root).
function Walker:frameUpdate(dt, now, grounded, vel, logical)
	self.pos = logical.Position
	self.up = logical.UpVector
	self.forward = logical.LookVector
	local up = self.up
	local renderUpTarget = up

	if grounded then
		if not self.wasGrounded or self.forceReplant then
			-- touchdown or surface transition frame: replant every foot at once
			-- (no arcs, a spider re-grips a new surface instantly, and arcing
			-- feet toward a plane the body just rotated onto reads as broken)
			-- and let the body spring INHERIT the approach velocity, so
			-- Hooke's law turns the landing into a dip and rebound for free
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
		-- advance airborne feet first so a pair finishing this frame frees the
		-- gate, and retarget every frame so landings track a turning body
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
		-- trot gate: one diagonal pair airborne at a time; when both are down,
		-- the pair whose worst foot has drifted furthest steps next
		local aBusy = self.gaitPairs[1][1].swing or self.gaitPairs[1][2].swing
		local bBusy = self.gaitPairs[2][1].swing or self.gaitPairs[2][2].swing
		if not aBusy and not bBusy then
			-- ask each foot where it COULD land (the cascade may pull the spot
			-- inboard on narrow surfaces) and fire the pair whose reachable
			-- target sits furthest from its plant. Measuring drift against the
			-- raw home instead would strobe forever on any surface narrower
			-- than the stance, which is exactly what a pillar climb is.
			local bestPair, bestErr = nil, CONFIG.StepTrigger
			for _, pair in self.gaitPairs do
				for _, limb in pair do
					local cand = limb:probeNear(limb:homeWorld(logical) + vel * CONFIG.LeadTime, up, self.pos)
					limb.stepCandidate = cand
					if cand then
						-- drift lives on the tangent plane: distance along the
						-- normal is the hover spring's business, not the gait's
						local err = flatOn(cand - limb.plantPos, up).Magnitude
						if err > bestErr then
							bestErr, bestPair = err, pair
						end
					end
				end
			end
			if bestPair then
				for _, limb in bestPair do
					if limb.stepCandidate then -- nil means a hole or a rim: staying planted beats stepping into air
						limb:beginStep(limb.stepCandidate)
					end
				end
			end
		end
		if landed > 0 then
			self.bodySpring.v -= up * (CONFIG.ImpactKick * landed) -- stride weight: dip along the normal, spring back
		end
		-- support plane: cross the foot quad's diagonals for a cheap plane fit,
		-- flipped to the body side of the surface; feeding it to the render
		-- normal lets the torso ride bumps the logical frame ignores
		local p1, p2 = self.limbs[1].plantPos, self.limbs[2].plantPos
		local p3, p4 = self.limbs[3].plantPos, self.limbs[4].plantPos
		local raw = (p2 - p3):Cross(p1 - p4)
		if raw:Dot(up) < 0 then
			raw = -raw
		end
		if raw.Magnitude > 1e-3 then
			renderUpTarget = (up * 2 + raw.Unit).Unit -- mostly the climb solver's normal, seasoned with foot terrain
		end
	else
		-- flight: no gait, ease every foot into the pounce tuck under its home
		for _, limb in self.limbs do
			limb.swing = nil
			local tuck = limb:homeWorld(logical) - up * CONFIG.AirTuck
			limb.footPos = limb.footPos:Lerp(tuck, math.min(dt * CONFIG.AirTuckRate, 1))
			limb.plantPos = limb.footPos -- keeps drift and the support plane fresh for the landing frame
		end
		self.bodySpring.x = self.pos -- the body rides the ballistic path exactly
		self.bodySpring.v = vel      -- and remembers its velocity for the touchdown compression
	end
	self.wasGrounded = grounded

	-- render layers: springed position, springed normal, then cosmetic angles
	local renderPos = grounded and self.bodySpring:step(self.pos, dt) or self.pos
	local su = self.upSpring:step(renderUpTarget, dt)
	self.renderUp = su.Magnitude > 0.05 and su.Unit or up
	-- signed yaw rate around the normal, from how far forward swung since the
	-- last frame: sin of the angle is the cross product's normal component
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
	-- the head leads the turn slightly: intent, smoothed so glances feel deliberate
	local lookYaw = self.headSpring:step(math.clamp(yawRate * 0.1, -0.5, 0.5), dt)
	local headCF = render * self.headOffset * CFrame.Angles(0, lookYaw, 0)
	self.head.CFrame = headCF
	self.eyeL.CFrame = headCF * CFrame.new(-0.62, 0.2, -1.0)
	self.eyeR.CFrame = headCF * CFrame.new(0.62, 0.2, -1.0)
	-- pose all eight bones off the SWAYING frame: legs visibly absorb every
	-- bob, bank and impact while the feet stay glued to the surface
	for _, limb in self.limbs do
		limb:solve(render)
	end
end

-- Remote spiders: their pilot already solved climbing on their machine and
-- wrote the answer into their root CFrame, which replicated here. Re-derive
-- everything visual from that single transform.
function Walker:mirrorStep(dt, now)
	local logical = self.rootPart.CFrame
	local vel = (logical.Position - self.lastPilotPos) / math.max(dt, 1e-3)
	self.lastPilotPos = logical.Position
	-- grounded if any surface sits within hover reach of the belly
	local hit = workspace:Raycast(logical.Position, -logical.UpVector * (CONFIG.Hover + 2.5), GAIT_PARAMS)
	self:frameUpdate(dt, now, hit ~= nil, vel, logical)
end

--============================ PILOTING (LOCAL) ================================

local mouseDX, mouseDY, wheel = 0, 0, 0
UserInputService.InputChanged:Connect(function(input)
	if input.UserInputType == Enum.UserInputType.MouseMovement then
		mouseDX += input.Delta.X
		mouseDY += input.Delta.Y
	elseif input.UserInputType == Enum.UserInputType.MouseWheel then
		wheel += input.Position.Z
	end
end)

-- Every surface query the pilot makes goes through this: rigs and avatars are
-- excluded by the params, and water is refused here, because a spider that
-- can stand on the open sea breaks the fiction instantly.
local function surfaceCast(origin, dir)
	local hit = workspace:Raycast(origin, dir, GAIT_PARAMS)
	if hit and hit.Material ~= Enum.Material.Water then
		return hit
	end
	return nil
end

-- Initial attachment: one long world-down ray so spawning does not free fall.
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
	-- while a textbox has focus, keystrokes are prose, not piloting: IsKeyDown
	-- cannot tell the difference on its own, so gate everything on this
	local typing = UserInputService:GetFocusedTextBox() ~= nil

	-- 1) LOOK: mouse yaw spins the WHOLE spider around its current surface
	-- normal, so steering is identical on floors, walls and ceilings; pitch
	-- only orbits the camera and never touches the creature
	if not typing then
		self.forward = rotAround(self.forward, self.up, -mouseDX * CONFIG.MouseSens)
		self.camPitch = math.clamp(self.camPitch - mouseDY * CONFIG.MouseSens, -1.25, 0.75)
		self.camDist = math.clamp(self.camDist - wheel * 3, 8, 40)
	end
	mouseDX, mouseDY, wheel = 0, 0, 0

	-- 2) MOVE INTENT: WASD spans the tangent plane of whatever we cling to
	local right = self.forward:Cross(self.up) -- matches CFrame's RightVector = look cross up
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

	-- 3) LEAP: spiders do not step off ledges politely, they pounce; the
	-- impulse leaves along the surface normal, so leaping off a wall throws
	-- you AWAY from the wall, which is exactly what your inner spider expects
	local jumping = not typing and UserInputService:IsKeyDown(Enum.KeyCode.Space)
	if jumping and not self.prevJump and self.attached then
		self.attached = false
		self.velAir = self.up * CONFIG.LeapUp + move * CONFIG.LeapFwd
	end
	self.prevJump = jumping

	if self.attached then
		-- 4a) CLIMB SOLVER: three rays decide everything
		if move ~= Vector3.zero then
			-- ray 1, wall grab: a face blocking the path becomes the new floor;
			-- forward continues UP the wall (the old up projected onto it),
			-- because the projection of the old forward collapses to zero there
			local ahead = now >= self.grabLock and surfaceCast(self.pos, move * CONFIG.GrabDist) or nil
			if ahead and ahead.Normal:Dot(self.up) < 0.55 and ahead.Normal:Dot(move) < -0.2 then
				local oldUp = self.up
				self.up = ahead.Normal
				self.forward = tangentOn(self.up, oldUp, self.forward)
				self.grabLock = now + CONFIG.GrabLock -- corner seams alternate faces; rate limiting stops the strobe
				self.forceReplant = true -- all four feet re-grip the new floor this frame
			else
				self.pos += move * CONFIG.MoveSpeed * dt
			end
		end
		-- ray 2, stick: re-acquire the surface under the moved body; BLEND the
		-- normal instead of snapping because voxel terrain normals are noisy
		local ghit = surfaceCast(self.pos + self.up * CONFIG.ProbeUp, -self.up * (CONFIG.ProbeUp + CONFIG.ProbeDown))
		if ghit then
			self.up = blendDir(self.up, ghit.Normal, dt * CONFIG.UpBlend)
			self.pos = ghit.Position + self.up * CONFIG.Hover
		else
			-- ray 3, edge wrap: nothing under us, so look BACK under the rim we
			-- just walked over; its far face becomes the floor and forward now
			-- points DOWN around the corner (minus old up projected onto it)
			local back = self.lastMove
			local whit = surfaceCast(self.pos - self.up * (CONFIG.Hover + 1.5), -back * CONFIG.WrapDist)
			if whit then
				local oldUp = self.up
				self.up = whit.Normal
				self.forward = tangentOn(self.up, -oldUp, self.forward)
				self.pos = whit.Position + self.up * CONFIG.Hover
				self.forceReplant = true -- feet chase the body around the corner immediately
			else
				self.attached = false -- a true void: fall, keeping tangent momentum
				self.velAir = move * CONFIG.MoveSpeed - self.up * 2
			end
		end
		self.forward = tangentOn(self.up, self.forward, self.lastMove)
	else
		-- 4b) BALLISTIC: integrate our own gravity (the capsule's physics is
		-- disabled), steer a little, and attach to the FIRST face the motion
		-- ray crosses, whatever its orientation: floor, wall or ceiling
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
				self.up = blendDir(self.up, WORLD_UP, dt * 3) -- roll the world upright while flying
				self.forward = tangentOn(self.up, self.forward)
			end
		end
		if self.pos.Y < CONFIG.VoidY then
			-- fell in the sea or off the world: back to the island
			self.pos = self.spawnPos
			self.velAir = Vector3.zero
			self.up = WORLD_UP
			self.attached = false
		end
	end

	-- 5) COMMIT: the character root carries the full pose to the server and
	-- every other client through ordinary character replication; this line is
	-- the entire netcode of the system
	local logical = CFrame.lookAt(self.pos, self.pos + self.forward, self.up)
	self.rootPart.CFrame = logical
	self.rootPart.AssemblyLinearVelocity = Vector3.zero -- physics must not accumulate gravity between our writes
	self.rootPart.AssemblyAngularVelocity = Vector3.zero
	local vel = (self.pos - self.lastPilotPos) / math.max(dt, 1e-3)
	self.lastPilotPos = self.pos
	self:frameUpdate(dt, now, self.attached, self.attached and vel or self.velAir, logical)

	-- 6) CAMERA: derived from its OWN smoothed copy of the spider frame. The
	-- creature is allowed to snap onto new surfaces, breathe, thump and bank,
	-- but none of that may reach the viewport raw: the camera up sweeps onto
	-- a new surface over a few tenths of a second (a wall still becomes the
	-- floor, it rolls there instead of teleporting), and the focus chases the
	-- logical body while deliberately ignoring the cosmetic bob layers.
	local cam = workspace.CurrentCamera
	cam.CameraType = Enum.CameraType.Scriptable
	-- reasserted every frame because focus changes reset it; while typing the
	-- cursor is released so chat and menus stay usable
	UserInputService.MouseBehavior = typing and Enum.MouseBehavior.Default or Enum.MouseBehavior.LockCenter
	UserInputService.MouseIconEnabled = typing
	self.camUp = blendDir(self.camUp, self.up, dt * CONFIG.CamUpBlend)
	local camFwd = tangentOn(self.camUp, self.forward, self.lastMove)
	self.camFocus = self.camFocus:Lerp(self.pos + self.camUp * CONFIG.CamFocusUp, math.min(dt * CONFIG.CamFocusRate, 1))
	local dir = camFwd * math.cos(self.camPitch) + self.camUp * math.sin(self.camPitch)
	-- occlusion handled at two speeds: pull IN instantly so geometry never
	-- clips into the view, relax back OUT smoothly so the zoom never pops
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

--============================ CHARACTER WIRING ================================

local walkers = {} -- player -> Walker

-- Avatars vanish CLIENT SIDE: property writes on instances this client does
-- not own never replicate, but every client runs this same file, so every
-- client hides every avatar for itself and all anyone ever sees is spiders.
local function hideVisual(inst)
	if inst:IsA("BasePart") then
		inst.Transparency = 1 -- a local only write on instances this client does not own
		inst.LocalTransparencyModifier = 1 -- and the compositing override, in case anything refreshes Transparency
		inst.CastShadow = false -- an invisible avatar must not leave a humanoid shaped shadow
	elseif inst:IsA("Decal") then
		inst.Transparency = 1
	end
end

local function hideCharacter(character)
	for _, d in character:GetDescendants() do
		hideVisual(d)
	end
	character.DescendantAdded:Connect(hideVisual) -- accessories stream in late: hide those too
end

-- The local capsule becomes pure cargo: the climb solver owns position, so
-- physics forces and collisions would only fight it. The owner simulates its
-- own character, so these client side changes are authoritative for motion.
local function prepareOwnCharacter(character)
	local humanoid = character:WaitForChild("Humanoid")
	humanoid.PlatformStand = true -- no balancing, no walk forces, no state flapping
	humanoid.AutoRotate = false
	local animate = character:FindFirstChild("Animate")
	if animate then
		animate.Enabled = false -- no point running walk animations on an invisible rig
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

-- Camera priority plus one: our write lands AFTER the default camera scripts
-- every frame, so ours is the one that sticks. dt is clamped because the
-- spring integrator destabilizes once omega * dt grows past about 2.
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
