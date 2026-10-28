--[[
    ADCornerModule
    --------------
    Sharp corners of a road network route are driven "point to point": the tractor steers at the corner
    point, saturates its steering and the trailer cuts the inside. A real driver swings wide on entry or
    exit so that his turning radius fits.

    When a network route is set, every sharp corner (CORNER_MIN_ANGLE .. CORNER_MAX_ANGLE) is replaced by
    an arc of the real turning radius of the train, chosen among candidates, in this order:
      - a wide arc without any offset, then the tight arc without offset,
      - the tight arc with the entry leg shifted to the outer side (swing wide on entry),
      - the tight arc with the exit leg shifted to the outer side (swing wide on exit),
      - both.
    A candidate is kept when its corridor (train width, plus the inner cut of the trailer on the arc) hits no
    static obstacle, stays off the fields (unless the original route already is on one), and stays on the
    road mesh where the original route is on one. When no candidate fits, the corner is left as it is.
    The inserted points are pathfinder-like points (no id), as the obstacle detour already does.
]]

ADCornerModule = {}


ADCornerModule.CORNER_MIN_ANGLE = math.rad(40)
ADCornerModule.CORNER_MAX_ANGLE = math.rad(150)
ADCornerModule.RADIUS_MAX = 14                 -- m, a larger estimate is a wheel data oddity
ADCornerModule.RADIUS_FACTOR_TIGHT = 1.15    -- a little more than the minimum radius: the steering can follow
ADCornerModule.RADIUS_FACTOR_WIDE = 1.6
ADCornerModule.ARC_STEP = 2.5                -- m between two arc points
ADCornerModule.RAMP_STEP = 2.5               -- m between two points of an offset ramp
ADCornerModule.RAMP_PER_METRE = 4            -- ramp length = offset * RAMP_PER_METRE, at least RAMP_MIN
ADCornerModule.RAMP_MIN = 6
ADCornerModule.LEG_MARGIN = 1                -- m kept between the previous / next point and the manoeuvre
ADCornerModule.STRAIGHT_ANGLE = math.rad(8)  -- points of a leg are "straight" below this angle: the manoeuvre may span them
ADCornerModule.LEG_MAX = 60                  -- m, longest leg looked at
ADCornerModule.SIDE_MARGIN = 0.5             -- m added on each side of the train width
ADCornerModule.INNER_CUT_MAX = 3             -- m, cap of the trailer inner cut on the arc
ADCornerModule.SAMPLE_STEP = 2               -- m between two field / road samples along a stretch
ADCornerModule.ROAD_RAY_HEIGHT = 3           -- m above the terrain for the road mesh raycast
ADCornerModule.OFFSETS = {
    {wide = true,  e1 = 0,   e2 = 0},
    {wide = false, e1 = 0,   e2 = 0},
    {wide = false, e1 = 1.5, e2 = 0},
    {wide = false, e1 = 0,   e2 = 1.5},
    {wide = false, e1 = 3,   e2 = 0},
    {wide = false, e1 = 0,   e2 = 3},
    {wide = false, e1 = 2,   e2 = 2},
    {wide = false, e1 = 3,   e2 = 3},
    {wide = true,  e1 = 2,   e2 = 0},
    {wide = true,  e1 = 0,   e2 = 2},
    {wide = true,  e1 = 3,   e2 = 3},
}

function ADCornerModule:log(text, ...)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "ADCornerModule: " .. text, ...)
end

function ADCornerModule:new(vehicle)
    local o = {}
    setmetatable(o, self)
    self.__index = self
    o.vehicle = vehicle
    return o
end

function ADCornerModule.isEnabled()
    local enabled = AutoDrive.getSetting("cornerSwingEnabled")
    return enabled == nil or enabled == true or enabled == 1
end

------------------------------------------------------------------------------------------------
-- train geometry
------------------------------------------------------------------------------------------------
function ADCornerModule:readTrain()
    local width = self.vehicle.size ~= nil and self.vehicle.size.width or 3
    local trailerLength = 0
    local units = AutoDrive.getAllUnits(self.vehicle)
    if units ~= nil then
        for _, unit in pairs(units) do
            if unit.size ~= nil then
                if unit.size.width ~= nil then
                    width = math.max(width, unit.size.width)
                end
                if unit ~= self.vehicle and unit.size.length ~= nil then
                    trailerLength = math.max(trailerLength, unit.size.length)
                end
            end
        end
    end
    self.halfWidth = width / 2 + ADCornerModule.SIDE_MARGIN
    self.trailerLength = trailerLength
    -- turning radius: the vehicle file value (AutoDrive.getDriverRadius) is sometimes far too small
    -- (3.9 m on some sprayers). The steering geometry of the wheels gives a second estimate: the larger one wins.
    self.fileRadius = AutoDrive.getDriverRadius(self.vehicle)
    self.wheelRadius = ADCornerModule.getWheelRadius(self.vehicle)
    self.radius = math.max(self.fileRadius or 0, self.wheelRadius or 0)
    if self.radius < 3 then
        self.radius = 6
    end
    self.radius = math.min(self.radius, ADCornerModule.RADIUS_MAX)
    self.height = PathFinderModule.getTrainHeight ~= nil and PathFinderModule.getTrainHeight(self.vehicle) or 3
end

-- turning radius from the wheels: wheelbase / tan(max steering angle) + half track (bicycle model),
-- nil when the wheel data is not available. All wheels steered (4WS): half the wheelbase counts.
function ADCornerModule.getWheelRadius(vehicle)
    local spec = vehicle.spec_wheels
    if spec == nil or spec.wheels == nil or vehicle.components == nil or vehicle.components[1] == nil then
        return nil
    end
    local root = vehicle.components[1].node
    local steerMax, steerZ, steerCount, fixedZ, fixedCount = 0, 0, 0, 0, 0
    local xMin, xMax = math.huge, -math.huge
    for _, wheel in pairs(spec.wheels) do
        local physics = wheel.physics or wheel
        if wheel.node ~= nil and physics.positionX ~= nil then
            local x, _, z = localToLocal(wheel.node, root, physics.positionX, physics.positionY or 0, physics.positionZ or 0)
            local rot = math.max(math.abs(physics.rotMax or 0), math.abs(physics.rotMin or 0))
            if rot > 0.05 then
                steerMax = math.max(steerMax, rot)
                steerZ, steerCount = steerZ + z, steerCount + 1
            else
                fixedZ, fixedCount = fixedZ + z, fixedCount + 1
            end
            xMin, xMax = math.min(xMin, x), math.max(xMax, x)
        end
    end
    if steerCount == 0 or steerMax <= 0.05 or steerMax >= math.rad(80) then
        return nil
    end
    local wheelbase
    if fixedCount > 0 then
        wheelbase = math.abs(steerZ / steerCount - fixedZ / fixedCount)
    else
        wheelbase = (vehicle.size ~= nil and vehicle.size.length or 4) * 0.3
    end
    if wheelbase < 1 then
        return nil
    end
    return wheelbase / math.tan(steerMax) + (xMax - xMin) / 2
end

-- how far the rear of the trailer cuts inside the tractor's arc
function ADCornerModule:getInnerCut(radius)
    if self.trailerLength <= 0 then
        return 0
    end
    return math.min(ADCornerModule.INNER_CUT_MAX, self.trailerLength * self.trailerLength / (2 * radius))
end

------------------------------------------------------------------------------------------------
-- terrain tests
------------------------------------------------------------------------------------------------
function ADCornerModule:collisionCallback(node)
    if node == nil or node == 0 or node == g_currentMission.terrainRootNode then
        return true
    end
    local object = g_currentMission.nodeToObject[node]
    if object == nil then
        local parent = getParent(node)
        if parent ~= nil and parent ~= 0 then
            object = g_currentMission.nodeToObject[parent]
        end
    end
    if object ~= nil then
        local root = self.vehicle:getRootVehicle()
        if object == self.vehicle or object == root or AutoDrive:checkIsConnected(root, object) then
            return true
        end
        -- vehicles move: they are not a reason to refuse a corner
        if object.ad ~= nil or object.spec_motorized ~= nil or object.spec_attachable ~= nil then
            return true
        end
    elseif not ADSensor.isElementBlockingVehicle(self, node) then
        return true
    end
    self.hits = self.hits + 1
    return false
end

function ADCornerModule:roadCallback(hitObjectId, x, y, z, distance)
    if hitObjectId ~= nil and hitObjectId ~= 0 then
        self.roadHit = true
    end
    return false
end

-- true when a road mesh (CollisionFlag.ROAD) lies under the point
function ADCornerModule:isOnRoadMesh(x, z)
    if raycastClosest == nil or CollisionFlag == nil or CollisionFlag.ROAD == nil then
        return false
    end
    self.roadHit = false
    local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 300, z)
    raycastClosest(x, y + ADCornerModule.ROAD_RAY_HEIGHT, z, 0, -1, 0, ADCornerModule.ROAD_RAY_HEIGHT * 2, "roadCallback", self, CollisionFlag.ROAD)
    return self.roadHit
end

function ADCornerModule:isOnField(x, z)
    local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 300, z)
    return AutoDrive.checkIsOnField(x, y, z) == true
end

-- one stretch of the candidate: obstacle box (shifted to the inner side on the arc), field and road samples.
-- reference: the original leg point the samples are compared with (on field / on road allowed there)
function ADCornerModule:isStretchClear(a, b, inner, innerCut, reference)
    local vx, vz = b.x - a.x, b.z - a.z
    local length = math.sqrt(vx * vx + vz * vz)
    if length < 0.05 then
        return true
    end
    local ry = math.atan2(vx, vz)
    local s, c = vx / length, vz / length
    local midX, midZ = a.x + vx / 2, a.z + vz / 2
    local halfWidth = self.halfWidth
    if inner ~= nil and innerCut > 0 then
        halfWidth = halfWidth + innerCut / 2
        midX = midX + inner.x * innerCut / 2
        midZ = midZ + inner.z * innerCut / 2
    end
    local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, midX, 300, midZ)
    self.hits = 0
    overlapBox(midX, y + self.height / 2, midZ, 0, ry, 0, halfWidth, self.height / 2, length / 2 + 0.5, "collisionCallback", self, AutoDrive.collisionMaskTerrain, true, true, true, true)
    if self.hits > 0 then
        return false, "obstacle"
    end
    -- field and road samples: centre line and both edges of the corridor. The trailer cut is not sampled here:
    -- the trailer cuts the inside of a raw corner even more, only obstacles count for it (box above)
    local nx, nz = c, -s -- left normal of the stretch
    local edge = self.halfWidth - ADCornerModule.SIDE_MARGIN
    local lateral = {{x = 0, z = 0}, {x = nx * edge, z = nz * edge}, {x = -nx * edge, z = -nz * edge}}
    local samples = math.max(1, math.ceil(length / ADCornerModule.SAMPLE_STEP))
    for k = 0, samples do
        local px, pz = a.x + vx * k / samples, a.z + vz * k / samples
        for _, l in ipairs(lateral) do
            if not reference.onField and self:isOnField(px + l.x, pz + l.z) then
                return false, "field"
            end
            if reference.onRoad and not self:isOnRoadMesh(px + l.x, pz + l.z) then
                return false, "off the road"
            end
        end
    end
    return true
end

------------------------------------------------------------------------------------------------
-- geometry of one candidate
------------------------------------------------------------------------------------------------
local function unit(from, to)
    local dx, dz = to.x - from.x, to.z - from.z
    local l = math.sqrt(dx * dx + dz * dz)
    if l < 0.01 then
        return nil, 0
    end
    return {x = dx / l, z = dz / l}, l
end

local function newPoint(x, z)
    return {x = x, z = z, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 300, z), isPathFinderPoint = true, isCornerPoint = true}
end

local function smoothStep(t)
    return t * t * (3 - 2 * t)
end

-- builds the points replacing the corner P between P0 and P1, nil when the legs are too short
function ADCornerModule:buildCandidate(P0, P, P1, d1, d2, theta, cross, radius, e1, e2)
    local sign = cross > 0 and 1 or -1
    -- perpendicular towards the centre of the turn, for each leg
    local c1 = {x = -d1.z * sign, z = d1.x * sign}
    local c2 = {x = -d2.z * sign, z = d2.x * sign}
    -- shifted legs: outer side = away from the centre
    local A = {x = P0.x - c1.x * e1, z = P0.z - c1.z * e1}      -- a point of the shifted entry line
    local B = {x = P.x - c2.x * e2, z = P.z - c2.z * e2}        -- a point of the shifted exit line
    local denom = d1.x * d2.z - d1.z * d2.x
    if math.abs(denom) < 0.001 then
        return nil
    end
    local wx, wz = B.x - A.x, B.z - A.z
    local s = (wx * d2.z - wz * d2.x) / denom
    local V = {x = A.x + d1.x * s, z = A.z + d1.z * s}          -- virtual corner of the shifted legs
    local tangent = radius * math.tan(theta / 2)
    local T1 = {x = V.x - d1.x * tangent, z = V.z - d1.z * tangent}
    local T2 = {x = V.x + d2.x * tangent, z = V.z + d2.z * tangent}

    local ramp1 = e1 > 0 and math.max(ADCornerModule.RAMP_MIN, e1 * ADCornerModule.RAMP_PER_METRE) or 0
    local ramp2 = e2 > 0 and math.max(ADCornerModule.RAMP_MIN, e2 * ADCornerModule.RAMP_PER_METRE) or 0
    -- room on the legs, measured along the original lines
    local roomBefore = (T1.x - P0.x) * d1.x + (T1.z - P0.z) * d1.z
    local roomAfter = (P1.x - T2.x) * d2.x + (P1.z - T2.z) * d2.z
    if roomBefore < ramp1 + ADCornerModule.LEG_MARGIN or roomAfter < ramp2 + ADCornerModule.LEG_MARGIN then
        return nil
    end

    local points = {}
    -- entry ramp: from the original line to the shifted line, ending at T1
    if ramp1 > 0 then
        local n = math.max(2, math.ceil(ramp1 / ADCornerModule.RAMP_STEP))
        for k = 0, n - 1 do
            local f = k / n
            local along = -ramp1 * (1 - f)
            local offset = -e1 * (1 - smoothStep(f))
            points[#points + 1] = newPoint(T1.x + d1.x * along - c1.x * offset, T1.z + d1.z * along - c1.z * offset)
        end
    end
    -- arc from T1 to T2
    local C = {x = T1.x + c1.x * radius, z = T1.z + c1.z * radius}
    local a0 = math.atan2(T1.z - C.z, T1.x - C.x)
    local sweep = sign * theta
    local steps = math.max(2, math.ceil(theta * radius / ADCornerModule.ARC_STEP))
    local arcStart = #points + 1
    points[#points + 1] = newPoint(T1.x, T1.z)
    for n = 1, steps do
        local a = a0 + sweep * n / steps
        points[#points + 1] = newPoint(C.x + math.cos(a) * radius, C.z + math.sin(a) * radius)
    end
    local arcEnd = #points
    -- exit ramp: from the shifted line back to the original line
    if ramp2 > 0 then
        local n = math.max(2, math.ceil(ramp2 / ADCornerModule.RAMP_STEP))
        for k = 1, n do
            local f = k / n
            local along = ramp2 * f
            local offset = -e2 * (1 - smoothStep(f))
            points[#points + 1] = newPoint(T2.x + d2.x * along - c2.x * offset, T2.z + d2.z * along - c2.z * offset)
        end
    end
    return {points = points, arcStart = arcStart, arcEnd = arcEnd, centre = C, tangent = tangent}
end

function ADCornerModule:isCandidateClear(candidate, P0, P1, radius, reference)
    local innerCut = self:getInnerCut(radius)
    local pts = candidate.points
    local prev = P0
    for n = 1, #pts + 1 do
        local p = pts[n] or P1
        local inner = nil
        if n > candidate.arcStart and n <= candidate.arcEnd then
            local midX, midZ = (prev.x + p.x) / 2, (prev.z + p.z) / 2
            local tx, tz = candidate.centre.x - midX, candidate.centre.z - midZ
            local l = math.sqrt(tx * tx + tz * tz)
            if l > 0.01 then
                inner = {x = tx / l, z = tz / l}
            end
        end
        local clear, reason = self:isStretchClear(prev, p, inner, innerCut, reference)
        if not clear then
            return false, reason
        end
        prev = p
    end
    return true
end

------------------------------------------------------------------------------------------------
-- route processing
------------------------------------------------------------------------------------------------
local function isNetworkPoint(wp)
    return wp ~= nil and wp.id ~= nil and not wp.isPathFinderPoint
end

local function isStraight(dA, dB)
    local dot = math.max(-1, math.min(1, dA.x * dB.x + dA.z * dB.z))
    return math.acos(dot) < ADCornerModule.STRAIGHT_ANGLE
end

-- start of the straight run ending at out[#out] -> P: index in out, and the point
local function straightStart(out, P)
    local k = #out
    local dir = unit(out[k], P)
    local length = 0
    while k > 1 and dir ~= nil do
        local a, b = out[k - 1], out[k]
        if not isNetworkPoint(a) or not isNetworkPoint(b) or b.isCornerPoint or ADGraphManager:isReverseRoad(a, b) then
            break
        end
        local d, l = unit(a, b)
        if d == nil or not isStraight(d, dir) or length + l > ADCornerModule.LEG_MAX then
            break
        end
        length = length + l
        k = k - 1
    end
    return k
end

-- end of the straight run starting at P -> wayPoints[i + 1]: index in wayPoints
local function straightEnd(wayPoints, i, P)
    local k = i + 1
    local dir = unit(P, wayPoints[k])
    local length = 0
    while wayPoints[k + 1] ~= nil and dir ~= nil do
        local a, b = wayPoints[k], wayPoints[k + 1]
        if not isNetworkPoint(a) or not isNetworkPoint(b) or ADGraphManager:isReverseRoad(a, b) then
            break
        end
        local d, l = unit(a, b)
        if d == nil or not isStraight(d, dir) or length + l > ADCornerModule.LEG_MAX then
            break
        end
        length = length + l
        k = k + 1
    end
    return k
end

local function along(from, to, dir)
    return (to.x - from.x) * dir.x + (to.z - from.z) * dir.z
end

-- returns the route with its sharp corners replaced; firstIndex: first corner index that may be changed
function ADCornerModule:smoothRoute(wayPoints, firstIndex)
    if not self.vehicle.isServer or wayPoints == nil or #wayPoints < 3 or not ADCornerModule.isEnabled() then
        return wayPoints
    end
    self:readTrain()
    local radiusTight = self.radius * ADCornerModule.RADIUS_FACTOR_TIGHT
    local radiusWide = self.radius * ADCornerModule.RADIUS_FACTOR_WIDE
    local out = {}
    for i = 1, math.max(1, (firstIndex or 3) - 1) do
        out[#out + 1] = wayPoints[i]
    end
    local i = #out + 1
    local changed, kept = 0, 0
    while i <= #wayPoints do
        local P0, P, P1 = out[#out], wayPoints[i], wayPoints[i + 1]
        local done = false
        if P1 ~= nil and isNetworkPoint(P0) and isNetworkPoint(P) and isNetworkPoint(P1)
            and not ADGraphManager:isReverseRoad(P0, P) and not ADGraphManager:isReverseRoad(P, P1) then
            local d1 = unit(P0, P)
            local d2 = unit(P, P1)
            if d1 ~= nil and d2 ~= nil then
                local dot = math.max(-1, math.min(1, d1.x * d2.x + d1.z * d2.z))
                local theta = math.acos(dot)
                local cross = d1.x * d2.z - d1.z * d2.x
                if theta >= ADCornerModule.CORNER_MIN_ANGLE and theta <= ADCornerModule.CORNER_MAX_ANGLE and math.abs(cross) > 0.01 then
                    -- the manoeuvre may span the straight run of points before and after the corner
                    local startIndex = math.max(straightStart(out, P), math.max(1, (firstIndex or 3) - 1))
                    local endIndex = straightEnd(wayPoints, i, P)
                    local S, E = out[startIndex], wayPoints[endIndex]
                    local reference = {onField = self:isOnField(P.x, P.z) or self:isOnField(P0.x, P0.z) or self:isOnField(P1.x, P1.z),
                                       onRoad = self:isOnRoadMesh(P0.x, P0.z) and self:isOnRoadMesh(P1.x, P1.z)}
                    local chosen, chosenOffsets, lastReason = nil, nil, "legs too short"
                    for _, o in ipairs(ADCornerModule.OFFSETS) do
                        local radius = o.wide and radiusWide or radiusTight
                        local candidate = self:buildCandidate(S, P, E, d1, d2, theta, cross, radius, o.e1, o.e2)
                        if candidate ~= nil then
                            local clear, reason = self:isCandidateClear(candidate, S, E, radius, reference)
                            if clear then
                                chosen, chosenOffsets = candidate, {radius = radius, e1 = o.e1, e2 = o.e2}
                                break
                            end
                            lastReason = reason
                        end
                    end
                    if chosen ~= nil then
                        -- drop the straight points covered by the manoeuvre, before (in out) and after (in wayPoints)
                        local first, last = chosen.points[1], chosen.points[#chosen.points]
                        while #out > startIndex and along(first, out[#out], d1) > -ADCornerModule.LEG_MARGIN do
                            out[#out] = nil
                        end
                        while i + 1 < endIndex and along(last, wayPoints[i + 1], d2) < ADCornerModule.LEG_MARGIN do
                            i = i + 1
                        end
                        for _, p in ipairs(chosen.points) do
                            out[#out + 1] = p
                        end
                        changed = changed + 1
                        self:log("corner %d (%s) %.0f deg at %.0f,%.0f: arc radius %.1f m, swing entry %.1f m / exit %.1f m (%d points)",
                            i, tostring(P.id), math.deg(theta), P.x, P.z, chosenOffsets.radius, chosenOffsets.e1, chosenOffsets.e2, #chosen.points)
                        done = true
                    else
                        kept = kept + 1
                        self:log("corner %d (%s) %.0f deg at %.0f,%.0f: kept as it is (%s)", i, tostring(P.id), math.deg(theta), P.x, P.z, tostring(lastReason))
                    end
                end
            end
        end
        if not done then
            out[#out + 1] = P
        end
        i = i + 1
    end
    if changed > 0 or kept > 0 then
        self:log("route of %d points: %d corner(s) rounded, %d kept, radius %.1f m (vehicle file %.1f, wheels %s), train %.1f m wide, trailer %.1f m",
            #wayPoints, changed, kept, self.radius, self.fileRadius or -1, self.wheelRadius ~= nil and string.format("%.1f", self.wheelRadius) or "?",
            self.halfWidth * 2 - 2 * ADCornerModule.SIDE_MARGIN, self.trailerLength)
    end
    return out
end
