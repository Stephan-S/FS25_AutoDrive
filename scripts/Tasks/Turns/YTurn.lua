-- Y manoeuvre at the end of a row (his drawing: red forward, blue reverse, green forward): forward on
-- a diagonal away from the new row (red), a long straight reverse back along that diagonal into the open field (blue),
-- then forward at full lock into the new row beside the harvester and straight on to settle the trailer (green).
-- Several diagonals, leads and reverse lengths are tried: the candidates are sorted by how close they end to the place
-- beside the harvester's new row, and the first one on field ground without crop and clear of fixed obstacles is kept.
YTurn = ADInheritsFrom(AbstractTurn)

YTurn.ANGLES = {0, 15, 30, 45, 60}       -- deg turned first (the diagonal), away from the new row or toward it
YTurn.LEADS = {0, 5, 10}                 -- m straight on along the diagonal
YTurn.BACKS = {15, 20, 25, 30, 40, 50}   -- m straight back along the diagonal
YTurn.MIN_TURN = 10                      -- deg: the last arc must turn at least this much
YTurn.MAX_TURN = 300                     -- deg: ... and at most this much
YTurn.MAX_CANDIDATES = 15                -- best candidates checked on the ground, one per frame
YTurn.END_AHEAD_MAX = 40                 -- m past the place beside the new row still fine (the reverse to the pipe covers it)
YTurn.END_MAX_ERROR = 25                 -- m: a manoeuvre ending farther than this from the place is no use, a route is better
YTurn.POINT_STEP = 1                     -- m between two path points
YTurn.STRAIGHTEN = 0.75                  -- share of the train length driven straight at the end
YTurn.OBSTACLE_MARGIN = 0.3              -- m added to the train half width for the obstacle boxes
YTurn.MIN_RADIUS = 5                     -- m

function YTurn:new(turnParameters)
    local o = YTurn:create()
    o.generationStep = 0
    o.turnGenerated = false
    o.turnParameters = turnParameters
    o.path = {}
    o.task = o.turnParameters.handleHarvesterTask
    o.failed = false
    o.candidates = nil
    o.index = 0
    o.reasons = {}
    return o
end

function YTurn.checkValidity(turnParameters)
    return turnParameters.angle > 90 and turnParameters.angle < 270
end

function YTurn:update(dt)
    if self.failed then
        return false
    end
    if not self.turnGenerated then
        self:generateTurn()
    end
    return self.turnGenerated
end

function YTurn:getWayPoints()
    return self.path
end

local function wrapAngle(a)
    while a > math.pi do
        a = a - 2 * math.pi
    end
    while a < -math.pi do
        a = a + 2 * math.pi
    end
    return a
end

-- frame of the start: f = forward, l = left (AutoDrive: a vehicle's local +X is its left side), angles from f toward l
function YTurn:getFrame()
    local p = self.turnParameters
    local f = {x = p.startDir.z.x, z = p.startDir.z.z}
    local l = {x = p.startDir.x.x, z = p.startDir.x.z}
    local g = {x = p.targetDir.z.x, z = p.targetDir.z.z}
    local goal = math.atan2(g.x * l.x + g.z * l.z, g.x * f.x + g.z * f.z)
    return f, l, goal
end

function YTurn:getRadius()
    return math.max(self.turnParameters.minRadius or 8, YTurn.MIN_RADIUS)
end

-- the place beside the harvester's new row (world), or the new row entry itself
function YTurn:getIdealEnd()
    local p = self.turnParameters
    local ok, points = pcall(self.task.generateReverseToCombineSection, self.task, true)
    if ok and points ~= nil and points[1] ~= nil then
        return {x = points[1].x, z = points[1].z}
    end
    return {x = p.targetPosition.x, z = p.targetPosition.z}
end

-- the pose after the three legs in the start frame, without building the points: u along f, v along l, heading h (rad)
-- s: sense of the first arc (+1 left), t: sense of the last arc, delta: angle of the last arc (rad). nil if the last arc
-- would be too short or too long
function YTurn:endPose(angleDeg, s, lead, back, t, goal)
    local R = self:getRadius()
    local a = math.rad(angleDeg) * s
    -- red arc from the origin heading 0, centre at s * R on the left axis
    local u = R * math.sin(math.abs(a))
    local v = s * R * (1 - math.cos(a))
    -- straight lead, then the straight reverse along the same heading
    u, v = u + (lead - back) * math.cos(a), v + (lead - back) * math.sin(a)
    -- green arc in sense t until the goal heading
    local delta = (goal - a) * t
    delta = delta % (2 * math.pi)
    if delta < math.rad(YTurn.MIN_TURN) or delta > math.rad(YTurn.MAX_TURN) then
        return nil
    end
    local cu, cv = u - t * R * math.sin(a), v + t * R * math.cos(a)
    local h = a + t * delta
    u, v = cu + t * R * math.sin(h), cv - t * R * math.cos(h)
    -- straight on to settle the trailer
    local straight = YTurn.STRAIGHTEN * AutoDrive.getTractorTrainLength(self.turnParameters.vehicle, true, true)
    u, v = u + straight * math.cos(h), v + straight * math.sin(h)
    local length = R * math.abs(a) + lead + back + R * delta + straight
    return {u = u, v = v, h = h, a = a, delta = delta, straight = straight, length = length}
end

function YTurn:toWorld(u, v)
    local p = self.turnParameters
    local f, l = self:getFrame()
    return p.startPosition.x + u * f.x + v * l.x, p.startPosition.z + u * f.z + v * l.z
end

function YTurn:buildCandidates()
    local p = self.turnParameters
    local f, l, goal = self:getFrame()
    local side = p.targetLeft and 1 or -1
    local ideal = self:getIdealEnd()
    local g = {x = p.targetDir.z.x, z = p.targetDir.z.z}
    local gl = {x = p.targetDir.x.x, z = p.targetDir.x.z}
    local list = {}
    for _, angle in ipairs(YTurn.ANGLES) do
        for _, s in ipairs(angle == 0 and {1} or {-side, side}) do
            for _, lead in ipairs(YTurn.LEADS) do
                for _, back in ipairs(YTurn.BACKS) do
                    for _, t in ipairs({side, -side}) do
                        local e = self:endPose(angle, s, lead, back, t, goal)
                        if e ~= nil then
                            local wx, wz = self:toWorld(e.u, e.v)
                            local dx, dz = wx - ideal.x, wz - ideal.z
                            local along = dx * g.x + dz * g.z
                            local lat = dx * gl.x + dz * gl.z
                            local error = math.sqrt(dx * dx + dz * dz)
                            if error <= YTurn.END_MAX_ERROR then
                                local score = math.abs(lat) + 3 * math.max(0, -along) + 2 * math.max(0, along - YTurn.END_AHEAD_MAX)
                                    + 0.05 * math.max(0, along) + 0.02 * e.length
                                list[#list + 1] = {angle = angle, s = s, lead = lead, back = back, t = t, e = e, score = score,
                                    along = along, lat = lat, error = error, x = wx, z = wz}
                            end
                        end
                    end
                end
            end
        end
    end
    table.sort(list, function(a, b)
        return a.score < b.score
    end)
    while #list > YTurn.MAX_CANDIDATES do
        table.remove(list, #list)
    end
    self.candidates = list
    self.ideal = ideal
end

local function addPoint(points, x, z, reverse)
    local y = AutoDrive:getTerrainHeightAtWorldPos(x, z) or 0
    local wp = {x = x, y = y, z = z}
    if reverse then
        wp.isReverse = true
    else
        wp.isForward = true
    end
    points[#points + 1] = wp
end

-- the path points of a candidate (world), every POINT_STEP m
function YTurn:buildPoints(c)
    local R = self:getRadius()
    local step = YTurn.POINT_STEP
    local points = {}
    local e = c.e
    local a = e.a
    -- red arc
    local arc = R * math.abs(a)
    local n = math.ceil(arc / step)
    for i = 1, n do
        local phi = a * i / n
        local u, v = R * math.sin(math.abs(phi)), c.s * R * (1 - math.cos(phi))
        local x, z = self:toWorld(u, v)
        addPoint(points, x, z, false)
    end
    local u0, v0 = R * math.sin(math.abs(a)), c.s * R * (1 - math.cos(a))
    -- red lead
    n = math.ceil(c.lead / step)
    for i = 1, n do
        local d = c.lead * i / n
        local x, z = self:toWorld(u0 + d * math.cos(a), v0 + d * math.sin(a))
        addPoint(points, x, z, false)
    end
    u0, v0 = u0 + c.lead * math.cos(a), v0 + c.lead * math.sin(a)
    -- blue: straight back
    n = math.ceil(c.back / step)
    for i = 1, n do
        local d = c.back * i / n
        local x, z = self:toWorld(u0 - d * math.cos(a), v0 - d * math.sin(a))
        addPoint(points, x, z, true)
    end
    u0, v0 = u0 - c.back * math.cos(a), v0 - c.back * math.sin(a)
    -- green arc
    local cu, cv = u0 - c.t * R * math.sin(a), v0 + c.t * R * math.cos(a)
    arc = R * e.delta
    n = math.ceil(arc / step)
    for i = 1, n do
        local h = a + c.t * e.delta * i / n
        local x, z = self:toWorld(cu + c.t * R * math.sin(h), cv - c.t * R * math.cos(h))
        addPoint(points, x, z, false)
    end
    local h = e.h
    u0, v0 = cu + c.t * R * math.sin(h), cv - c.t * R * math.cos(h)
    -- straight on
    n = math.ceil(e.straight / step)
    for i = 1, n do
        local d = e.straight * i / n
        local x, z = self:toWorld(u0 + d * math.cos(h), v0 + d * math.sin(h))
        addPoint(points, x, z, false)
    end
    -- harvester AI turn (not the Courseplay end of row, where it gives nothing): the reverse to its pipe
    local ok, section = pcall(self.task.generateReverseToCombineSection, self.task)
    if ok and section ~= nil then
        for _, node in ipairs(section) do
            points[#points + 1] = node
        end
    end
    return points
end

-- first fixed obstacle (hedge, trees, wall, learned blocked zone) along the points; vehicles are left to the task's own
-- collision check. Returns x, z of the segment or nil
function YTurn.findObstacle(vehicle, points)
    if g_currentMission == nil or g_currentMission.terrainRootNode == nil or FollowCombineTask == nil or overlapBox == nil
        or vehicle.ad == nil or vehicle.ad.pathFinderModule == nil then
        return nil
    end
    local pf = vehicle.ad.pathFinderModule
    local halfWidth = pf:getTrainWidth() / 2 + YTurn.OBSTACLE_MARGIN
    local height = PathFinderModule.getTrainHeight(vehicle) + PathFinderModule.OBSTACLE_HEIGHT_MARGIN
    for i = 2, #points do
        local a, b = points[i - 1], points[i]
        local dx, dz = b.x - a.x, b.z - a.z
        local len = math.sqrt(dx * dx + dz * dz)
        if len > 0.1 then
            dx, dz = dx / len, dz / len
            local angleRad = AutoDrive.normalizeAngle(math.atan2(-dz, dx))
            local midX, midZ = (a.x + b.x) / 2, (a.z + b.z) / 2
            local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, midX, 1, midZ)
            local probe = {hits = 0, callback = FollowCombineTask.lineCollisionCallback}
            overlapBox(midX, y + height / 2, midZ, 0, angleRad, 0, len / 2 + 0.5, height / 2, halfWidth, "callback", probe, AutoDrive.collisionMaskTerrain, true, true, true, true)
            if probe.hits > 0 or pf:boxHitsBlockedZone(midX, midZ, angleRad, len / 2 + 0.5, halfWidth) then
                return midX, midZ
            end
        end
    end
    return nil
end

local function reasonKey(why)
    why = tostring(why)
    for _, key in ipairs({"off the field", "crop", "obstacle"}) do
        if string.find(why, key, 1, true) ~= nil then
            return key
        end
    end
    return why
end

function YTurn:generateTurn()
    local vehicle = self.turnParameters.vehicle
    local name = tostring(vehicle:getName())
    if self.candidates == nil then
        self:buildCandidates()
        if #self.candidates == 0 then
            self.failed = true
            return
        end
    end
    -- one candidate per frame
    self.index = self.index + 1
    local c = self.candidates[self.index]
    if c == nil then
        self.failed = true
        local parts = {}
        for key, n in pairs(self.reasons) do
            parts[#parts + 1] = string.format("%s x%d", key, n)
        end
        return
    end
    local points = self:buildPoints(c)
    if #points < 3 then
        self.reasons["too short"] = (self.reasons["too short"] or 0) + 1
        return
    end
    local ok, why = self.task:checkGround(points)
    if ok then
        local ox, oz = YTurn.findObstacle(vehicle, points)
        if ox ~= nil then
            ok, why = false, string.format("obstacle at %.0f,%.0f", ox, oz)
        end
    end
    if not ok then
        local key = reasonKey(why)
        self.reasons[key] = (self.reasons[key] or 0) + 1
        return
    end
    self.path = points
    self.turnGenerated = true
end
