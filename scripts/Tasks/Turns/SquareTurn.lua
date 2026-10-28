-- Square manoeuvre at the end of a row, in two variants.
-- Tractor beside the harvester: forward through a quarter turn toward the harvester's side, straight on until the
-- trailer is aligned, straight reverse to free the nose, forward through a quarter turn to end in the axis of the new
-- row behind the harvester.
-- Tractor right behind the harvester: a straight reverse of a few metres first, then the same legs.
-- The harvester has already made its U-turn: the place is beside its new row, on its pipe side, as
-- generateReverseToCombineSection measures it.
-- Built like the other AutoDrive turns: fixed legs (arcs at full lock and straights), several lengths tried, candidates
-- sorted by how close they end to the place behind the harvester's new row, the first one on field ground without crop
-- and clear of fixed obstacles is kept. The arcs are driven forward and the reverse legs are straight, after a straight
-- run long enough to align the trailer.
SquareTurn = ADInheritsFrom(AbstractTurn)

SquareTurn.BACKS_FIRST = {0, 8, 14, 20}     -- m straight reverse first (0 = beside the harvester, more = right behind it)
SquareTurn.CLEAR_DISTANCE = 5               -- m: closer than this to the harvester, the reverse first comes first
SquareTurn.SIDE_STEPS = {0}                 -- m side step (kept at 0)
SquareTurn.FORWARDS = {0, 4, 8, 12, 16, 20, 24, 28} -- m straight forward before the first arc (passing the harvester first)
SquareTurn.ANGLES = {90, 110}               -- deg of the first forward arc; 90 preferred (reverse along the headland)
SquareTurn.LEAD_EXTRAS = {0, 6, 12, 18}     -- m straight on after the first arc, added to the train length
SquareTurn.BACKS = {6, 10, 15, 20, 25, 30}  -- m straight reverse, plus the length that lands on the place
SquareTurn.BACK_MIN = 5                     -- m
SquareTurn.BACK_MAX = 30                    -- m
SquareTurn.MIN_TURN = 20                    -- deg: the last arc must turn at least this much
SquareTurn.MAX_TURN = 200                   -- deg: ... and at most this much
SquareTurn.MAX_CANDIDATES = 15              -- best candidates checked on the ground, one per frame
SquareTurn.END_MAX_LAT = 8                  -- m beside the place still fine
SquareTurn.END_MAX_SHORT = 40               -- m short of the place still fine (the chase drives on)
SquareTurn.END_MAX_AHEAD = 10               -- m past the place still fine
SquareTurn.POINT_STEP = 1                   -- m between two path points
SquareTurn.STRAIGHTEN = 0.5                 -- share of the train length driven straight at the end, at least
SquareTurn.STRAIGHT_MAX_EXTRA = 25          -- m more straight at the end to reach the place beside the harvester
SquareTurn.OBSTACLE_MARGIN = 0.3            -- m added to the train half width for the obstacle boxes
SquareTurn.MIN_RADIUS = 5                   -- m
SquareTurn.MIN_LEAD = 8                     -- m straight on at least before a reverse
SquareTurn.COMBINE_MARGIN = 0.8             -- m kept around the harvester and its header

function SquareTurn:new(turnParameters)
    local o = SquareTurn:create()
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

function SquareTurn.checkValidity(turnParameters)
    return turnParameters.angle > 90 and turnParameters.angle < 270
end

function SquareTurn:update(dt)
    if self.failed then
        return false
    end
    if not self.turnGenerated then
        self:generateTurn()
    end
    return self.turnGenerated
end

function SquareTurn:getWayPoints()
    return self.path
end

-- frame of the start: f = forward, l = left (AutoDrive: a vehicle's local +X is its left side), angles from f toward l
function SquareTurn:getFrame()
    local p = self.turnParameters
    local f = {x = p.startDir.z.x, z = p.startDir.z.z}
    local l = {x = p.startDir.x.x, z = p.startDir.x.z}
    local g = {x = p.targetDir.z.x, z = p.targetDir.z.z}
    local goal = math.atan2(g.x * l.x + g.z * l.z, g.x * f.x + g.z * f.z)
    return f, l, goal
end

function SquareTurn:getRadius()
    return math.max(self.turnParameters.minRadius or 8, SquareTurn.MIN_RADIUS)
end

function SquareTurn:getTrainLength()
    local ok, len = pcall(AutoDrive.getTractorTrainLength, self.turnParameters.vehicle, true, true)
    if ok and type(len) == "number" and len > 0 then
        return len
    end
    return 12
end

-- the place behind the harvester's new row (world), or the new row entry itself
function SquareTurn:getIdealEnd()
    local p = self.turnParameters
    local ok, points = pcall(self.task.generateReverseToCombineSection, self.task, true)
    if ok and points ~= nil and points[1] ~= nil then
        return {x = points[1].x, z = points[1].z}
    end
    return {x = p.targetPosition.x, z = p.targetPosition.z}
end

-- the harvester body and its header as two boxes in the start frame: centre (u, v), heading angle (rad from f toward
-- l), half length and half width widened by the train half width. nil when it cannot be read
function SquareTurn:getCombineBoxes()
    local combine = self.task ~= nil and self.task.combine or nil
    if combine == nil or combine.components == nil or combine.components[1] == nil or getWorldTranslation == nil or localDirectionToWorld == nil then
        return nil
    end
    local ok, box = pcall(function()
        local node = combine.components[1].node
        local cx, _, cz = getWorldTranslation(node)
        local dx, _, dz = localDirectionToWorld(node, 0, 0, 1)
        local f, l = self:getFrame()
        local p = self.turnParameters
        if self.task.endRow and p.targetPosition ~= nil and p.targetDir ~= nil and p.targetDir.z ~= nil then
            -- end of row: the harvester is still turning when the manoeuvre is planned; it is counted where it will stand, at the entry of its new row
            cx, cz = p.targetPosition.x, p.targetPosition.z
            dx, dz = p.targetDir.z.x, p.targetDir.z.z
        end
        local rx, rz = cx - p.startPosition.x, cz - p.startPosition.z
        local u, v = rx * f.x + rz * f.z, rx * l.x + rz * l.z
        local h = math.atan2(dx * l.x + dz * l.z, dx * f.x + dz * f.z)
        local length = combine.size ~= nil and combine.size.length ~= nil and math.abs(combine.size.length) or 8
        local width = combine.size ~= nil and combine.size.width ~= nil and math.abs(combine.size.width) or 3
        local toolLength, toolWidth = 0, 0
        if AutoDrive.getFrontToolLength ~= nil then
            local okL, len = pcall(AutoDrive.getFrontToolLength, combine)
            toolLength = okL and type(len) == "number" and len or 0
        end
        if AutoDrive.getFrontToolWidth ~= nil then
            local okW, w = pcall(AutoDrive.getFrontToolWidth, combine)
            toolWidth = okW and type(w) == "number" and w or 0
        end
        local trainHalf = 1.5
        if p.vehicle.ad ~= nil and p.vehicle.ad.pathFinderModule ~= nil and p.vehicle.ad.pathFinderModule.getTrainWidth ~= nil then
            local okT, tw = pcall(p.vehicle.ad.pathFinderModule.getTrainWidth, p.vehicle.ad.pathFinderModule)
            if okT and type(tw) == "number" then
                trainHalf = tw / 2
            end
        end
        local pad = trainHalf + SquareTurn.COMBINE_MARGIN
        local boxes = {{u = u, v = v, h = h, halfLen = length / 2 + pad, halfWidth = width / 2 + pad}}
        if toolLength > 0.5 and toolWidth > width then
            -- the header sits in front of the root
            local shift = length / 2 + toolLength / 2
            boxes[2] = {u = u + shift * math.cos(h), v = v + shift * math.sin(h), h = h, halfLen = toolLength / 2 + pad, halfWidth = toolWidth / 2 + pad}
        end
        return boxes
    end)
    if ok then
        return box
    end
    return nil
end

local function insideBox(boxes, u, v)
    for _, box in ipairs(boxes) do
        local du, dv = u - box.u, v - box.v
        local c, s = math.cos(box.h), math.sin(box.h)
        local along = du * c + dv * s
        local across = -du * s + dv * c
        if math.abs(along) <= box.halfLen and math.abs(across) <= box.halfWidth then
            return true
        end
    end
    return false
end

-- distance from (u, v) to the nearest box (0 inside)
local function boxDistance(boxes, u, v)
    local best = math.huge
    for _, box in ipairs(boxes) do
        local du, dv = u - box.u, v - box.v
        local c, s = math.cos(box.h), math.sin(box.h)
        local along = math.max(0, math.abs(du * c + dv * s) - box.halfLen)
        local across = math.max(0, math.abs(-du * s + dv * c) - box.halfWidth)
        best = math.min(best, math.sqrt(along * along + across * across))
    end
    return best
end

function SquareTurn:toWorld(u, v)
    local p = self.turnParameters
    local f, l = self:getFrame()
    return p.startPosition.x + u * f.x + v * l.x, p.startPosition.z + u * f.z + v * l.z
end

-- the legs of a candidate in the start frame. A leg: {arc = true, sense = +-1 (+1 left), angle = rad, reverse = bool}
-- or {len = m, reverse = bool}. The last arc angle is computed from the goal heading, nil when out of range
function SquareTurn:buildLegs(c, goal)
    local legs = {}
    local h = 0
    if c.back0 > 0 then
        legs[#legs + 1] = {len = c.back0, reverse = true}
    end
    local R = self:getRadius()
    if c.step > 0 then
        -- side step away from the harvester's side: two opposite arcs, heading unchanged
        local theta = math.acos(math.max(-1, 1 - c.step / (2 * R)))
        legs[#legs + 1] = {arc = true, sense = -c.s, angle = theta}
        legs[#legs + 1] = {arc = true, sense = c.s, angle = theta}
    end
    if c.fwd0 > 0 then
        legs[#legs + 1] = {len = c.fwd0}
    end
    legs[#legs + 1] = {arc = true, sense = c.s, angle = math.rad(c.angle)}
    h = c.s * math.rad(c.angle)
    legs[#legs + 1] = {len = c.lead}
    legs[#legs + 1] = {len = c.back, reverse = true}
    local delta = ((goal - h) * c.t) % (2 * math.pi)
    if delta < math.rad(SquareTurn.MIN_TURN) or delta > math.rad(SquareTurn.MAX_TURN) then
        return nil
    end
    legs[#legs + 1] = {arc = true, sense = c.t, angle = delta}
    legs[#legs + 1] = {len = SquareTurn.STRAIGHTEN * self:getTrainLength()}
    return legs, delta
end

-- walks the legs from (0, 0) heading 0; visit(u, v, reverse) is called every POINT_STEP m when given
local function walk(legs, R, visit)
    local u, v, h, length = 0, 0, 0, 0
    local step = SquareTurn.POINT_STEP
    for _, leg in ipairs(legs) do
        if leg.arc then
            local arc = R * leg.angle
            local n = visit ~= nil and math.max(1, math.ceil(arc / step)) or 1
            local cu, cv = u - leg.sense * R * math.sin(h), v + leg.sense * R * math.cos(h)
            local h0 = h
            for i = 1, n do
                local hi = h0 + leg.sense * leg.angle * i / n
                u, v = cu + leg.sense * R * math.sin(hi), cv - leg.sense * R * math.cos(hi)
                if visit ~= nil then
                    visit(u, v, leg.reverse == true)
                end
            end
            h = h0 + leg.sense * leg.angle
            length = length + arc
        else
            local n = visit ~= nil and math.max(1, math.ceil(leg.len / step)) or 1
            local sign = leg.reverse and -1 or 1
            local u0, v0 = u, v
            for i = 1, n do
                local d = leg.len * i / n
                u, v = u0 + sign * d * math.cos(h), v0 + sign * d * math.sin(h)
                if visit ~= nil then
                    visit(u, v, leg.reverse == true)
                end
            end
            length = length + leg.len
        end
    end
    return u, v, h, length
end

function SquareTurn:buildCandidates()
    local p = self.turnParameters
    local f, l, goal = self:getFrame()
    local side = p.targetLeft and 1 or -1
    local ideal = self:getIdealEnd()
    local g = {x = p.targetDir.z.x, z = p.targetDir.z.z}
    local gl = {x = p.targetDir.x.x, z = p.targetDir.x.z}
    local R = self:getRadius()
    local train = math.max(SquareTurn.MIN_LEAD, self:getTrainLength())
    local box = self:getCombineBoxes()
    -- standing close to the harvester, the tractor backs up first to clear it
    local close = box ~= nil and boxDistance(box, 0, 0) < SquareTurn.CLEAR_DISTANCE
    self.closeToCombine = close
    local list = {}
    local throughCombine = 0
    for _, back0 in ipairs(SquareTurn.BACKS_FIRST) do
      for _, stepSide in ipairs(SquareTurn.SIDE_STEPS) do
        for _, fwd0 in ipairs(SquareTurn.FORWARDS) do
            for _, angle in ipairs(SquareTurn.ANGLES) do
                for _, s in ipairs({side, -side}) do
                    for _, extra in ipairs(SquareTurn.LEAD_EXTRAS) do
                        for _, t in ipairs({s, -s}) do
                            -- the end moves linearly with the reverse length: the length that lands beside the place is tried too
                            local backs = {}
                            for _, b in ipairs(SquareTurn.BACKS) do
                                backs[#backs + 1] = b
                            end
                            local c0 = {back0 = back0, step = stepSide, fwd0 = fwd0, angle = angle, s = s, lead = train + extra, back = 0, t = t}
                            local legs0 = self:buildLegs(c0, goal)
                            if legs0 ~= nil then
                                local u0, v0 = walk(legs0, R)
                                local wx0, wz0 = self:toWorld(u0, v0)
                                local h1 = s * math.rad(angle)
                                local du, dv = math.cos(h1), math.sin(h1)
                                local dirLat = (du * f.x + dv * l.x) * gl.x + (du * f.z + dv * l.z) * gl.z
                                if math.abs(dirLat) > 0.2 then
                                    local lat0 = (wx0 - ideal.x) * gl.x + (wz0 - ideal.z) * gl.z
                                    local solved = lat0 / dirLat
                                    if solved >= SquareTurn.BACK_MIN and solved <= SquareTurn.BACK_MAX then
                                        backs[#backs + 1] = math.floor(solved * 2 + 0.5) / 2
                                    end
                                end
                            end
                            for _, back in ipairs(backs) do
                                local c = {back0 = back0, step = stepSide, fwd0 = fwd0, angle = angle, s = s, lead = train + extra, back = back, t = t}
                                local legs, delta = self:buildLegs(c, goal)
                                if legs ~= nil then
                                    local u, v, h, length = walk(legs, R)
                                    local wx, wz = self:toWorld(u, v)
                                    local dx, dz = wx - ideal.x, wz - ideal.z
                                    local along = dx * g.x + dz * g.z
                                    local lat = dx * gl.x + dz * gl.z
                                    if along < -1 then
                                        -- short of the place: the last straight leg (heading = new row) is extended up to it
                                        local extra = math.min(-along, SquareTurn.STRAIGHT_MAX_EXTRA)
                                        legs[#legs].len = legs[#legs].len + extra
                                        along = along + extra
                                        length = length + extra
                                        u, v = u + extra * math.cos(h), v + extra * math.sin(h)
                                        wx, wz = self:toWorld(u, v)
                                        dx, dz = wx - ideal.x, wz - ideal.z
                                    end
                                    local fits = math.abs(lat) <= SquareTurn.END_MAX_LAT and along >= -SquareTurn.END_MAX_SHORT and along <= SquareTurn.END_MAX_AHEAD
                                    if fits and box ~= nil then
                                        -- the manoeuvre goes around the harvester, never through it or its header
                                        local hit, left = false, not insideBox(box, 0, 0)
                                        walk(legs, R, function(pu, pv)
                                            local inside = insideBox(box, pu, pv)
                                            -- a start already within the widened box (right behind it) counts once out
                                            if not inside then
                                                left = true
                                            elseif left then
                                                hit = true
                                            end
                                        end)
                                        if hit then
                                            fits = false
                                            throughCombine = throughCombine + 1
                                        end
                                    end
                                    if fits then
                                        -- preferred first: arcs toward the harvester's side, no first reverse nor side step
                                        local score = math.abs(lat) + 0.3 * math.max(0, -along - 10) + 2 * math.max(0, along - 3)
                                            + 0.02 * length + (s ~= side and 3 or 0) + (t ~= s and 2 or 0) + (close and (back0 == 0 and 20 or 0.1 * back0) or 0.3 * back0) + 0.5 * stepSide + 0.05 * fwd0 + 0.15 * math.abs(angle - 90)
                                        c.legs, c.delta, c.length = legs, delta, length
                                        c.score, c.along, c.lat, c.error = score, along, lat, math.sqrt(dx * dx + dz * dz)
                                        c.x, c.z = wx, wz
                                        list[#list + 1] = c
                                    end
                                end
                            end
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
    while #list > SquareTurn.MAX_CANDIDATES do
        table.remove(list, #list)
    end
    self.candidates = list
    self.ideal = ideal
    self.throughCombine = throughCombine
end

-- the path points of a candidate (world), every POINT_STEP m
function SquareTurn:buildPoints(c)
    local points = {}
    walk(c.legs, self:getRadius(), function(u, v, reverse)
        local x, z = self:toWorld(u, v)
        local y = AutoDrive:getTerrainHeightAtWorldPos(x, z) or 0
        local wp = {x = x, y = y, z = z}
        if reverse then
            wp.isReverse = true
        else
            wp.isForward = true
        end
        points[#points + 1] = wp
    end)
    -- harvester AI turn (not the Courseplay end of row, where it gives nothing): the reverse to its pipe
    local ok, section = pcall(self.task.generateReverseToCombineSection, self.task)
    if ok and section ~= nil then
        for _, node in ipairs(section) do
            points[#points + 1] = node
        end
    end
    return points
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

function SquareTurn:generateTurn()
    local vehicle = self.turnParameters.vehicle
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
        return
    end
    local points = self:buildPoints(c)
    if #points < 3 then
        self.reasons["too short"] = (self.reasons["too short"] or 0) + 1
        return
    end
    local ok, why = self.task:checkGround(points)
    if ok and YTurn ~= nil and YTurn.findObstacle ~= nil then
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
