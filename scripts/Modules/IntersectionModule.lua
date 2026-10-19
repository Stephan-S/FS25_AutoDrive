-- "Intersection" road type (red points, AutoDrive.FLAG_INTERSECTION):
-- before entering an intersection section, AutoDrive looks at the AutoDrive lanes
-- this section crosses or joins, going back up each lane for 80 m.
-- If a vehicle is coming (game traffic, tractor, player), it stops just before the
-- first red point, waits until the lanes have been clear for 2 s, then crosses without stopping.
-- For route calculation an intersection point counts as a normal point.

ADIntersectionModule = {}

ADIntersectionModule.DEBUG = false

ADIntersectionModule.WATCH_DIST = 80          -- m of lane watched upstream
ADIntersectionModule.LOOK_AHEAD = 60          -- m of route checked ahead of the vehicle
ADIntersectionModule.STOP_BEFORE = 1          -- m: the vehicle front stops this far before the first red point
ADIntersectionModule.APPROACH_DIST = 25       -- m before the stop point: reduced speed to look
ADIntersectionModule.APPROACH_KMH = 20
ADIntersectionModule.BRAKE_DECEL = 1.5        -- m/s²
ADIntersectionModule.COMMIT_DIST = 2          -- m before the stop point: if clear, it goes
ADIntersectionModule.CLEAR_MS = 2000          -- clear for 2 s after a wait
ADIntersectionModule.CHECK_MS = 250
ADIntersectionModule.CLOSE_DIST = 8           -- m: vehicle on the crossing, even stopped
ADIntersectionModule.ARRIVAL_S = 8            -- s: vehicle arriving within 8 s
ADIntersectionModule.NEAR_DIST = 40           -- m: vehicle moving towards the crossing
ADIntersectionModule.MOVING_MS = 0.5          -- m/s
ADIntersectionModule.SEARCH_RADIUS = 40       -- m around the red section to find the lanes
ADIntersectionModule.BOX_HALF_WIDTH = 2.0
ADIntersectionModule.BOX_HALF_HEIGHT = 1.5
ADIntersectionModule.BOX_MAX_LENGTH = 10

ADIntersectionModule.zones = {}               -- cache: id of the first red point -> zone
ADIntersectionModule.tracks = {}

function ADIntersectionModule.isIntersectionPoint(wp)
    if wp == nil then
        return false
    end
    local flags = wp.flags
    if wp.id ~= nil then
        local g = ADGraphManager:getWayPointById(wp.id)
        if g ~= nil then
            flags = g.flags
        end
    end
    return flags ~= nil and bit32.band(flags, AutoDrive.FLAG_INTERSECTION) > 0
end

-------------------------------------------------------------------------------
-- Zone: the red section and the lanes to watch
-------------------------------------------------------------------------------

-- 2D segment intersection: returns x, z or nil
local function segmentsCross(ax, az, bx, bz, cx, cz, dx, dz)
    local rx, rz = bx - ax, bz - az
    local sx, sz = dx - cx, dz - cz
    local den = rx * sz - rz * sx
    if math.abs(den) < 1e-6 then
        return nil
    end
    local qx, qz = cx - ax, cz - az
    local t = (qx * sz - qz * sx) / den
    local u = (qx * rz - qz * rx) / den
    if t < 0 or t > 1 or u < 0 or u > 1 then
        return nil
    end
    return ax + t * rx, az + t * rz
end

-- lane segments upstream of a point, up to WATCH_DIST (non red lanes)
local function collectUpstream(startId, boxes, seen)
    local queue = { { id = startId, dist = 0 } }
    local head = 1
    while queue[head] ~= nil do
        local item = queue[head]
        head = head + 1
        local wp = ADGraphManager:getWayPointById(item.id)
        if wp ~= nil and wp.incoming ~= nil then
            for _, inId in pairs(wp.incoming) do
                local prev = ADGraphManager:getWayPointById(inId)
                if prev ~= nil and not ADIntersectionModule.isIntersectionPoint(prev) then
                    local key = inId .. ">" .. item.id
                    if not seen[key] then
                        seen[key] = true
                        local len = MathUtil.vector2Length(wp.x - prev.x, wp.z - prev.z)
                        table.insert(boxes, { ax = prev.x, ay = prev.y, az = prev.z, bx = wp.x, by = wp.y, bz = wp.z })
                        if item.dist + len < ADIntersectionModule.WATCH_DIST then
                            table.insert(queue, { id = inId, dist = item.dist + len })
                        end
                    end
                end
            end
        end
    end
end

-- splits the segments into boxes of at most BOX_MAX_LENGTH
local function makeBoxes(segments)
    local out = {}
    for _, s in ipairs(segments) do
        local len = MathUtil.vector2Length(s.bx - s.ax, s.bz - s.az)
        if len > 0.1 then
            local n = math.max(1, math.ceil(len / ADIntersectionModule.BOX_MAX_LENGTH))
            local ry = math.atan2(s.bx - s.ax, s.bz - s.az)
            for k = 0, n - 1 do
                local t0, t1 = k / n, (k + 1) / n
                local tm = (t0 + t1) * 0.5
                table.insert(out, {
                    x = s.ax + (s.bx - s.ax) * tm,
                    y = s.ay + (s.by - s.ay) * tm,
                    z = s.az + (s.bz - s.az) * tm,
                    ry = ry,
                    half = len / n * 0.5 + 0.5
                })
            end
        end
    end
    return out
end

-- builds the zone from the red points of the route, firstIndex..lastIndex
function ADIntersectionModule.buildZone(wayPoints, firstIndex, lastIndex)
    local red = {}
    for i = math.max(1, firstIndex - 1), math.min(#wayPoints, lastIndex + 1) do
        table.insert(red, wayPoints[i])
    end
    local endPoint = wayPoints[lastIndex + 1]

    local cx, cz = 0, 0
    for _, p in ipairs(red) do
        cx, cz = cx + p.x, cz + p.z
    end
    cx, cz = cx / #red, cz / #red

    local points = {}      -- crossing / joining points (x, z)
    local segments = {}
    local seen = {}
    local r2 = ADIntersectionModule.SEARCH_RADIUS ^ 2

    -- lanes crossed by the red section
    for _, p in pairs(ADGraphManager:getWayPoints()) do
        if p.out ~= nil and (p.x - cx) ^ 2 + (p.z - cz) ^ 2 < r2 and not ADIntersectionModule.isIntersectionPoint(p) then
            for _, outId in pairs(p.out) do
                local q = ADGraphManager:getWayPointById(outId)
                if q ~= nil and not ADIntersectionModule.isIntersectionPoint(q) then
                    for i = 1, #red - 1 do
                        local a, b = red[i], red[i + 1]
                        -- segments sharing a point with the red section are not crossed lanes
                        if p.id ~= a.id and p.id ~= b.id and q.id ~= a.id and q.id ~= b.id then
                            local x, z = segmentsCross(a.x, a.z, b.x, b.z, p.x, p.z, q.x, q.z)
                            if x ~= nil then
                                table.insert(points, { x = x, z = z })
                                local key = p.id .. ">" .. q.id
                                if not seen[key] then
                                    seen[key] = true
                                    table.insert(segments, { ax = p.x, ay = p.y, az = p.z, bx = q.x, by = q.y, bz = q.z })
                                end
                                collectUpstream(p.id, segments, seen)
                                -- two way lane: traffic also comes from the other side
                                if q.out ~= nil and table.contains(q.out, p.id) then
                                    collectUpstream(q.id, segments, seen)
                                end
                            end
                        end
                    end
                end
            end
        end
    end

    -- lane joined at the end of the red section
    if endPoint ~= nil and endPoint.id ~= nil and not ADIntersectionModule.isIntersectionPoint(endPoint) then
        table.insert(points, { x = endPoint.x, z = endPoint.z })
        collectUpstream(endPoint.id, segments, seen)
    end

    local zone = {
        boxes = makeBoxes(segments),
        points = points,
        lanes = #segments
    }
    return zone
end

-------------------------------------------------------------------------------
-- Traffic
-------------------------------------------------------------------------------

local function getMask()
    local m = 0
    for _, name in ipairs({ "VEHICLE", "TRAFFIC_VEHICLE", "TRAFFIC_VEHICLE_BLOCKING" }) do
        if CollisionFlag[name] ~= nil then
            m = m + CollisionFlag[name]
        end
    end
    return m
end

function ADIntersectionModule:overlapCallback(node)
    if node == nil or node == 0 or node == g_currentMission.terrainRootNode then
        return true
    end
    local object = g_currentMission:getNodeObject(node)
    if object == nil then
        local parent = getParent(node)
        if parent ~= nil and parent ~= 0 then
            object = g_currentMission:getNodeObject(parent)
        end
    end
    local me = self.scanVehicle
    if object ~= nil and me ~= nil then
        local root = me:getRootVehicle()
        if object == me or object == root or AutoDrive:checkIsConnected(root, object) then
            return true
        end
    end
    local key = object ~= nil and object or node
    if not self.scanSeen[key] then
        self.scanSeen[key] = true
        table.insert(self.scanHits, { key = key, node = node, object = object })
    end
    return true
end

function ADIntersectionModule.isTrafficComing(vehicle, zone)
    local self = ADIntersectionModule
    self.scanVehicle = vehicle
    self.scanHits = {}
    self.scanSeen = {}
    local mask = getMask()
    for _, b in ipairs(zone.boxes) do
        overlapBox(b.x, b.y + self.BOX_HALF_HEIGHT + 0.2, b.z, 0, b.ry, 0, self.BOX_HALF_WIDTH, self.BOX_HALF_HEIGHT, b.half,
            "overlapCallback", self, mask, true, true, true, true)
        if ADIntersectionModule.DEBUG then
            DebugUtil.drawOverlapBox(b.x, b.y + self.BOX_HALF_HEIGHT + 0.2, b.z, 0, b.ry, 0, self.BOX_HALF_WIDTH, self.BOX_HALF_HEIGHT, b.half, 1, 0, 0)
        end
    end
    self.scanVehicle = nil

    local now = g_time
    local tracks = self.tracks
    local danger, reason = false, nil
    for _, h in ipairs(self.scanHits) do
        local hx, _hy, hz
        if h.object ~= nil and h.object.rootNode ~= nil then
            hx, _hy, hz = getWorldTranslation(h.object.rootNode)
        else
            hx, _hy, hz = getWorldTranslation(h.node)
        end
        local d = math.huge
        for _, p in ipairs(zone.points) do
            d = math.min(d, MathUtil.vector2Length(hx - p.x, hz - p.z))
        end
        local speed
        if h.object ~= nil and h.object.lastSpeedReal ~= nil then
            speed = h.object.lastSpeedReal * 1000
        end
        local t = tracks[h.key]
        local closing = nil
        if t ~= nil and now - t.time > 100 and now - t.time < 2000 then
            closing = (t.d - d) / ((now - t.time) / 1000)
        end
        if t == nil or now - t.time >= 100 then
            tracks[h.key] = { d = d, time = now }
        end
        if speed == nil and closing ~= nil then
            speed = math.abs(closing)
        end

        local name = h.object ~= nil and h.object.getFullName ~= nil and h.object:getFullName() or "traffic car"
        if d <= self.CLOSE_DIST then
            danger, reason = true, string.format("%s on the crossing (%.0f m)", name, d)
        elseif closing == nil then
            if d < self.NEAR_DIST and (speed == nil or speed > self.MOVING_MS) then
                danger, reason = true, string.format("%s at %.0f m", name, d)
            end
        elseif closing > self.MOVING_MS and (d / closing < self.ARRIVAL_S or d < self.NEAR_DIST) then
            danger, reason = true, string.format("%s coming (%.0f m, %.0f km/h)", name, d, closing * 3.6)
        end
        if danger then
            break
        end
    end

    for k, t in pairs(tracks) do
        if now - t.time > 5000 then
            tracks[k] = nil
        end
    end
    return danger, reason
end

-------------------------------------------------------------------------------
-- Driving
-------------------------------------------------------------------------------

local function getFrontOffset(vehicle)
    local size = vehicle.size
    if size == nil then
        return 3
    end
    return size.length * 0.5 + (size.lengthOffset or 0)
end

local function getTrainLength(vehicle)
    local total = 0
    local root = vehicle:getRootVehicle()
    if root.childVehicles ~= nil then
        for _, v in pairs(root.childVehicles) do
            if v.size ~= nil then
                total = total + v.size.length
            end
        end
    end
    if total <= 0 then
        total = vehicle.size ~= nil and vehicle.size.length or 6
    end
    return total
end

-- first red section on the route ahead: {dist = m from the front to the first red point, endDist, key, zone}
local function findIntersection(vehicle)
    local dpm = vehicle.ad.drivePathModule
    local wayPoints, index = dpm:getWayPoints()
    if wayPoints == nil or index == nil or wayPoints[index] == nil then
        return nil
    end
    local x, _, z = getWorldTranslation(vehicle.components[1].node)
    local front = getFrontOffset(vehicle)
    local cum = -front
    local px, pz = x, z
    local i = index
    while wayPoints[i] ~= nil and cum < ADIntersectionModule.LOOK_AHEAD do
        local wp = wayPoints[i]
        cum = cum + MathUtil.vector2Length(wp.x - px, wp.z - pz)
        if ADIntersectionModule.isIntersectionPoint(wp) then
            -- the red section may already be started: go back to its first point
            local first = i
            while first > 1 and ADIntersectionModule.isIntersectionPoint(wayPoints[first - 1]) do
                first = first - 1
            end
            local last = i
            local endDist = cum
            local lx, lz = wp.x, wp.z
            while wayPoints[last + 1] ~= nil and ADIntersectionModule.isIntersectionPoint(wayPoints[last + 1]) do
                last = last + 1
                endDist = endDist + MathUtil.vector2Length(wayPoints[last].x - lx, wayPoints[last].z - lz)
                lx, lz = wayPoints[last].x, wayPoints[last].z
            end
            if wayPoints[last + 1] ~= nil then
                endDist = endDist + MathUtil.vector2Length(wayPoints[last + 1].x - lx, wayPoints[last + 1].z - lz)
            end
            local key = tostring(wayPoints[first].id or (math.floor(wayPoints[first].x) .. ":" .. math.floor(wayPoints[first].z)))
            local zone = ADIntersectionModule.zones[key]
            if zone == nil then
                zone = ADIntersectionModule.buildZone(wayPoints, first, last)
                ADIntersectionModule.zones[key] = zone
            end
            local dist = first < index and -1 or cum
            return { dist = dist, endDist = endDist, key = key, zone = zone }
        end
        px, pz = wp.x, wp.z
        i = i + 1
    end
    return nil
end

function ADIntersectionModule.reset(vehicle)
    local c = vehicle.ad.intersection
    if c ~= nil and c.holding and vehicle.ad.specialDrivingModule ~= nil then
        vehicle.ad.specialDrivingModule:releaseVehicle()
    end
    vehicle.ad.intersection = nil
end

-- requested max speed (km/h) or math.huge
function ADIntersectionModule.getSpeedLimit(vehicle)
    local c = vehicle.ad ~= nil and vehicle.ad.intersection or nil
    if c == nil or c.speed == nil then
        return math.huge
    end
    return c.speed
end

local function hold(vehicle, dt)
    local sdm = vehicle.ad.specialDrivingModule
    sdm.motorShouldNotBeStopped = true
    sdm:stopVehicle()
    sdm:update(dt)
    sdm.motorShouldNotBeStopped = false
end

-- called every frame by ADDrivePathModule:followWaypoints; returns true while the vehicle has to stay stopped
function ADIntersectionModule.handle(vehicle, dt)
    if vehicle == nil or vehicle.ad == nil or not vehicle.isServer then
        return false
    end
    local dpm = vehicle.ad.drivePathModule
    if dpm == nil or dpm:getIsReversing() then
        return false
    end
    local c = vehicle.ad.intersection
    if c == nil then
        c = { nextCheck = 0, clearTime = 0 }
        vehicle.ad.intersection = c
    end

    -- committed: cross without looking until the whole train has passed
    if c.committed ~= nil then
        c.travel = c.travel + (vehicle.lastMovedDistance or 0)
        if c.travel > c.committed.span then
            c.done = { key = c.committed.key, time = g_time }
            c.committed = nil
        end
        c.speed = nil
        return false
    end

    if g_time < c.nextCheck then
        if c.holding then
            hold(vehicle, dt)
            return true
        end
        return false
    end
    c.nextCheck = g_time + ADIntersectionModule.CHECK_MS

    local inter = findIntersection(vehicle)
    if c.done ~= nil and g_time - c.done.time > 60000 then
        c.done = nil
    end
    if inter ~= nil and c.done ~= nil and c.done.key == inter.key then
        inter = nil
    end
    if inter == nil then
        if c.holding then
            vehicle.ad.specialDrivingModule:releaseVehicle()
        end
        c.holding, c.waited, c.clearTime, c.speed = false, false, 0, nil
        return false
    end

    local toStop = inter.dist - ADIntersectionModule.STOP_BEFORE
    local danger = ADIntersectionModule.isTrafficComing(vehicle, inter.zone)

    local function commit()
        if c.holding then
            vehicle.ad.specialDrivingModule:releaseVehicle()
        end
        c.committed = { key = inter.key, span = math.max(0, inter.endDist) + getTrainLength(vehicle) + 2 }
        c.travel = 0
        c.holding, c.waited, c.clearTime, c.speed = false, false, 0, nil
    end

    if danger then
        c.clearTime = 0
        c.waited = true
        if toStop <= 1 then
            if toStop < -1 and not c.holding then
                -- already on the red section: too late to stop, go on
                commit()
                return false
            end
            c.holding = true
        end
        local v = math.sqrt(2 * ADIntersectionModule.BRAKE_DECEL * math.max(0, toStop)) * 3.6
        c.speed = math.max(3, math.min(ADIntersectionModule.APPROACH_KMH, v))
    else
        if c.waited and c.clearTime == 0 then
            c.clearTime = g_time
        end
        local clearLongEnough = not c.waited or g_time - c.clearTime >= ADIntersectionModule.CLEAR_MS
        if toStop <= ADIntersectionModule.COMMIT_DIST and clearLongEnough then
            commit()
            return false
        end
        if c.waited and toStop <= 1 then
            c.holding = true
        end
        if toStop < ADIntersectionModule.APPROACH_DIST then
            c.speed = ADIntersectionModule.APPROACH_KMH
        else
            c.speed = nil
        end
    end

    if c.holding then
        hold(vehicle, dt)
        return true
    end
    return false
end

-- the network changed (editor): zones have to be rebuilt
function ADIntersectionModule.clearCache()
    ADIntersectionModule.zones = {}
end
