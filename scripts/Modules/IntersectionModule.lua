-- "Intersection" road type (red points, AutoDrive.FLAG_INTERSECTION):
-- before entering an intersection section, AutoDrive looks at the AutoDrive lanes
-- this section crosses or joins, going back up each lane for 80 m.
-- If a vehicle is coming (game traffic, tractor, player), it stops just before the
-- first red point, waits until the lanes have been clear for 2 s, then crosses without stopping.
-- For route calculation an intersection point counts as a normal point.

ADIntersectionModule = {}

ADIntersectionModule.DEBUG = false

ADIntersectionModule.WATCH_DIST = 120         -- m of lane watched upstream (80 m is only 4 s at 73 km/h, too short for a train starting off)
ADIntersectionModule.LOOK_AHEAD = 60          -- m of route checked ahead of the vehicle
ADIntersectionModule.STOP_BEFORE = 1          -- m: the vehicle front stops this far before the first red point
ADIntersectionModule.APPROACH_DIST = 25       -- m before the stop point: reduced speed to look
ADIntersectionModule.APPROACH_KMH = 20
ADIntersectionModule.LOOK_KMH = 8              -- km/h at the edge of the red section, even with the road clear (look before crossing)
ADIntersectionModule.BRAKE_DECEL = 1.5        -- m/s²
ADIntersectionModule.COMMIT_DIST = 2          -- m before the stop point: if clear, it goes
ADIntersectionModule.CLEAR_MS = 2000          -- clear for 2 s after a wait
ADIntersectionModule.CHECK_MS = 250
ADIntersectionModule.CLOSE_DIST = 8           -- m: vehicle on the crossing, even stopped
ADIntersectionModule.PARKED_DIST = 4          -- m: further away, a stopped vehicle stands beside the crossing, not on it
ADIntersectionModule.ARRIVAL_S = 8            -- s: vehicle arriving within 8 s (minimum)
ADIntersectionModule.ARRIVAL_MAX_S = 14       -- s: longest horizon, from the time the train needs to cross
-- the crossing time depends on the train (a light van does not need the gap of a big train), then the patience
-- lowers the gap asked for
ADIntersectionModule.LIGHT_TRAIN_M = 8        -- m: a single short vehicle is light
ADIntersectionModule.ACCEL_LIGHT = 1.5        -- m/s² from a stop
ADIntersectionModule.VMAX_LIGHT = 10          -- m/s (36 km/h) while crossing
ADIntersectionModule.ACCEL_TRAIN = 0.7
ADIntersectionModule.VMAX_TRAIN = 8
ADIntersectionModule.PATIENCE_MS = 30000      -- ms waited at the line before a shorter gap is accepted
ADIntersectionModule.PATIENCE_STEP_MS = 10000 -- ms: 1 s less per step
ADIntersectionModule.PATIENCE_MIN_S = 5       -- s: shortest gap accepted (the game traffic brakes in front of a vehicle)
ADIntersectionModule.TRACK_MAX_MS = 5000      -- ms: oldest track used to get an approach speed
ADIntersectionModule.NEAR_DIST = 40           -- m: vehicle moving towards the crossing
ADIntersectionModule.ASSUMED_MS = 14          -- m/s (50 km/h) assumed for a vehicle seen for the first time
ADIntersectionModule.MOVING_MS = 0.5          -- m/s
ADIntersectionModule.SEARCH_RADIUS = 40       -- m around the red section to find the lanes
ADIntersectionModule.BOX_HALF_WIDTH = 2.0
ADIntersectionModule.BOX_HALF_HEIGHT = 1.5
ADIntersectionModule.BOX_MAX_LENGTH = 10
ADIntersectionModule.RETREAT_STUCK_MS = 1500   -- ms stopped on the red section before backing out of it
ADIntersectionModule.RETREAT_TIMEOUT_MS = 40000
ADIntersectionModule.RETREAT_BLOCKED_MS = 6000 -- ms blocked behind while backing out: the back-off is given up
ADIntersectionModule.COMMIT_STUCK_MS = 20000  -- ms committed and stopped without a possible back-off: the crossing lets go
ADIntersectionModule.EXIT_MIN_LENGTH = 25     -- m of free lane needed after the red section (at least the train + 8 m)

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

function ADIntersectionModule.getTracks(vehicle)
    local c = vehicle.ad ~= nil and vehicle.ad.intersection or nil
    if c == nil then
        return ADIntersectionModule.tracks
    end
    if c.tracks == nil then
        c.tracks = {}
    end
    return c.tracks
end

function ADIntersectionModule.isTrafficComing(vehicle, zone, horizon)
    local self = ADIntersectionModule
    local arrival = horizon or self.ARRIVAL_S
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
    -- tracks per vehicle: two vehicles at the same crossing refreshing one shared track 50 ms apart never got a
    -- speed out of it, the second one saw a stopped car as a danger for minutes
    local tracks = ADIntersectionModule.getTracks(vehicle)
    local danger, reason = false, nil
    -- the vehicle that waits for me before a single track (TrafficYield plan, stopped on purpose): not a danger,
    -- waiting for it would block us both
    local yieldModule = vehicle.ad ~= nil and vehicle.ad.trafficYieldModule or nil
    local waitingForMe = nil
    if yieldModule ~= nil and yieldModule.plan ~= nil and yieldModule.plan.parked and yieldModule.plan.waitBeforeSection then
        waitingForMe = yieldModule.partner
    end
    for _, h in ipairs(self.scanHits) do
      local isWaitingForMe = waitingForMe ~= nil and h.object ~= nil and (h.object == waitingForMe
            or (h.object.getRootVehicle ~= nil and h.object:getRootVehicle() == waitingForMe) or AutoDrive:checkIsConnected(waitingForMe, h.object))
      if not isWaitingForMe then
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
        -- a track up to TRACK_MAX_MS old (a stopped car seen now and then never got a speed)
        if t ~= nil and now - t.time > 100 and now - t.time < self.TRACK_MAX_MS then
            closing = (t.d - d) / ((now - t.time) / 1000)
        end
        if t == nil or now - t.time >= 100 then
            tracks[h.key] = { d = d, time = now }
        end
        if speed == nil and closing ~= nil then
            speed = math.abs(closing)
        end

        local name = h.object ~= nil and h.object.getFullName ~= nil and h.object:getFullName() or "traffic car"
        -- a stopped AutoDrive vehicle waiting at its own line (or backing off) does not occupy the crossing unless it
        -- really stands on the points; my sensor keeps me off its train
        local root = h.object ~= nil and h.object.getRootVehicle ~= nil and h.object:getRootVehicle() or h.object
        local stopped = (speed == nil or speed < self.MOVING_MS) and (closing == nil or math.abs(closing) < self.MOVING_MS)
        local waitingAtItsLine = root ~= nil and root.ad ~= nil and root.ad.intersection ~= nil and stopped
            and (root.ad.intersection.holding or root.ad.intersection.retreat ~= nil)
        -- a stopped AutoDrive vehicle held by its face to face plan (parked, backed off, waiting) waits for somebody to
        -- pass: not a danger
        local parkedByYield = root ~= nil and root.ad ~= nil and root.ad.trafficYieldModule ~= nil and stopped
            and root.ad.trafficYieldModule.isControlling ~= nil and root.ad.trafficYieldModule:isControlling()
        -- (two vehicles waiting at their lines 3 m from a point would wait for each other: the limit is 1.5 m)
        -- Every track is updated on each pass (no break at the first danger), else a car seen behind another one
        -- keeps an old track and counts as unknown again
        if danger then
            -- already a danger: track updated, nothing else
        elseif (waitingAtItsLine or parkedByYield) and d > 1.5 then
            -- not a danger
        elseif d <= self.CLOSE_DIST and (d <= self.PARKED_DIST or not (stopped and (speed ~= nil or closing ~= nil))) then
            -- a stopped vehicle (known speed) more than PARKED_DIST from a point of the crossing stands beside it, my
            -- sensor deals with it; a moving vehicle stays a danger up to CLOSE_DIST
            danger, reason = true, string.format("%s on the crossing (%.0f m)", name, d)
        elseif closing == nil then
            -- first seen (no approach speed yet): assumed coming toward the crossing at 50 km/h (or at its known
            -- speed), seen again 250 ms later. A car seen for the first time far away counts too, so that a vehicle
            -- at its line does not pull out right in front of it
            local assumed = speed or self.ASSUMED_MS
            if (speed == nil or speed > self.MOVING_MS) and (d < self.NEAR_DIST or d / assumed < arrival) then
                danger, reason = true, string.format("%s at %.0f m (speed unknown)", name, d)
            end
        elseif closing > self.MOVING_MS and (d / closing < arrival or d < self.NEAR_DIST) then
            danger, reason = true, string.format("%s coming (%.0f m, %.0f km/h)", name, d, closing * 3.6)
        end
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

-- time (s) to cross 'span' m from a stop, from the train: a single short vehicle starts quickly, a train (trailer,
-- implement) slowly
local function crossingTime(vehicle, span)
    local self = ADIntersectionModule
    local root = vehicle:getRootVehicle()
    local count = 0
    if root.childVehicles ~= nil then
        for _ in pairs(root.childVehicles) do
            count = count + 1
        end
    end
    local light = count <= 1 and getTrainLength(vehicle) <= self.LIGHT_TRAIN_M
    local accel = light and self.ACCEL_LIGHT or self.ACCEL_TRAIN
    local vmax = light and self.VMAX_LIGHT or self.VMAX_TRAIN
    local accelDist = vmax * vmax / (2 * accel)
    if span <= accelDist then
        return math.sqrt(2 * span / accel), light
    end
    return vmax / accel + (span - accelDist) / vmax, light
end

-- the lane after the red section: boxes along MY route after the last red point, over 'length' m
local function buildExitBoxes(wayPoints, lastIndex, length)
    local segments = {}
    local cum = 0
    local i = lastIndex + 1
    while wayPoints[i] ~= nil and wayPoints[i + 1] ~= nil and cum < length do
        local a, b = wayPoints[i], wayPoints[i + 1]
        table.insert(segments, { ax = a.x, ay = a.y, az = a.z, bx = b.x, by = b.y, bz = b.z })
        cum = cum + MathUtil.vector2Length(b.x - a.x, b.z - a.z)
        i = i + 1
    end
    return makeBoxes(segments), wayPoints[lastIndex + 1]
end

-- a stopped (or nearly stopped) vehicle on the exit lane: if I go, I stay across the crossing (cars queuing behind a
-- stopped vehicle)
function ADIntersectionModule.isExitBlocked(vehicle, wayPoints, lastIndex)
    local self = ADIntersectionModule
    local length = math.max(self.EXIT_MIN_LENGTH, getTrainLength(vehicle) + 8)
    local boxes, exitPoint = buildExitBoxes(wayPoints, lastIndex, length)
    if #boxes == 0 or exitPoint == nil then
        return false
    end
    self.scanVehicle = vehicle
    self.scanHits = {}
    self.scanSeen = {}
    local mask = getMask()
    for _, b in ipairs(boxes) do
        overlapBox(b.x, b.y + self.BOX_HALF_HEIGHT + 0.2, b.z, 0, b.ry, 0, self.BOX_HALF_WIDTH, self.BOX_HALF_HEIGHT, b.half,
            "overlapCallback", self, mask, true, true, true, true)
    end
    self.scanVehicle = nil
    local now = g_time
    local tracks = ADIntersectionModule.getTracks(vehicle)
    -- the vehicle that waits for me (face to face plan) does not occupy my exit, my sensor sees it
    local yieldModule = vehicle.ad ~= nil and vehicle.ad.trafficYieldModule or nil
    local waitingForMe = nil
    if yieldModule ~= nil and yieldModule.plan ~= nil and yieldModule.plan.parked and yieldModule.plan.waitBeforeSection then
        waitingForMe = yieldModule.partner
    end
    for _, h in ipairs(self.scanHits) do
      local isWaitingForMe = waitingForMe ~= nil and h.object ~= nil and (h.object == waitingForMe
            or (h.object.getRootVehicle ~= nil and h.object:getRootVehicle() == waitingForMe) or AutoDrive:checkIsConnected(waitingForMe, h.object))
      if not isWaitingForMe then
        local hx, _hy, hz
        if h.object ~= nil and h.object.rootNode ~= nil then
            hx, _hy, hz = getWorldTranslation(h.object.rootNode)
        else
            hx, _hy, hz = getWorldTranslation(h.node)
        end
        local d = MathUtil.vector2Length(hx - exitPoint.x, hz - exitPoint.z)
        local speed
        if h.object ~= nil and h.object.lastSpeedReal ~= nil then
            speed = math.abs(h.object.lastSpeedReal) * 1000
        end
        local key = "exit:" .. tostring(h.key)
        local t = tracks[key]
        if speed == nil and t ~= nil and now - t.time > 100 and now - t.time < 2000 then
            speed = math.abs(d - t.d) / ((now - t.time) / 1000)
        end
        if t == nil or now - t.time >= 100 then
            tracks[key] = { d = d, time = now }
        end
        -- unknown speed (game traffic car seen for the first time): taken as stopped, seen again 250 ms later
        if speed == nil or speed < self.MOVING_MS then
            local name = h.object ~= nil and h.object.getFullName ~= nil and h.object:getFullName() or "traffic car"
            return true, string.format("exit occupied: %s stopped %.0f m after the crossing", name, d)
        end
      end
    end
    return false
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
            return { dist = dist, endDist = endDist, key = key, zone = zone, lastIndex = last, wayPoints = wayPoints }
        end
        px, pz = wp.x, wp.z
        i = i + 1
    end
    return nil
end

function ADIntersectionModule.reset(vehicle)
    local c = vehicle.ad.intersection
    if c ~= nil and (c.holding or c.retreat ~= nil) and vehicle.ad.specialDrivingModule ~= nil then
        vehicle.ad.specialDrivingModule:releaseVehicle()
    end
    vehicle.ad.intersection = nil
end

-- true while the crossing holds the vehicle: slowed approach, stop before the line or backing out (the obstacle
-- detour must not take the car it waits for as an obstacle)
function ADIntersectionModule.isWaiting(vehicle)
    local c = vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.intersection or nil
    if c == nil then
        return false
    end
    return c.holding == true or c.retreat ~= nil or c.speed ~= nil
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

    -- missed: stopped on the red section, it backs off until it is before the line again, then waits
    if c.retreat ~= nil then
        return ADIntersectionModule.updateRetreat(vehicle, c, dt)
    end

    -- while the face to face module drives the vehicle (back-off, wait before a single track), the crossing decides
    -- nothing (neither "clear, cross" nor backing out)
    local yieldModule = vehicle.ad.trafficYieldModule
    if yieldModule ~= nil and yieldModule.isControlling ~= nil and yieldModule:isControlling() and c.committed == nil then
        return false
    end

    -- committed: cross without looking until the whole train has passed
    if c.committed ~= nil then
        c.travel = c.travel + (vehicle.lastMovedDistance or 0)
        if c.travel > c.committed.span then
            c.done = { key = c.committed.key, time = g_time }
            c.committed = nil
            c.stuckMs = 0
            c.speed = nil
            c.noRetreat = nil
            return false
        end
        -- a vehicle must not wait in the middle of the crossing: stopped on the red section (sensor, vehicle in front)
        -- before getting out of it, it backs off at once to the line
        local kmh = math.abs(vehicle.lastSpeedReal or 0) * 3600
        if kmh < 0.5 and c.travel > 0.5 then
            c.stuckMs = (c.stuckMs or 0) + dt
            if c.stuckMs >= ADIntersectionModule.RETREAT_STUCK_MS then
                -- a vehicle right in front of me going my way is a queue, not a miss: I wait until it moves on. No
                -- back-off either when it is impossible or was already done twice here
                local queued, queuedName = ADIntersectionModule.isQueuedBehindVehicle(vehicle)
                c.retreatCount = c.retreatCount or {}
                -- committed and blocked for long without a possible back-off: the crossing lets go (the face to face
                -- may take over)
                if c.stuckMs >= ADIntersectionModule.COMMIT_STUCK_MS and (queued or c.noRetreat == c.committed.key or (c.retreatCount[c.committed.key] or 0) >= 2) then
                    c.done = { key = c.committed.key, time = g_time }
                    c.committed = nil
                    c.stuckMs = 0
                    c.speed = nil
                    c.noRetreat = nil
                    return false
                end
                if not queued and c.noRetreat ~= c.committed.key and (c.retreatCount[c.committed.key] or 0) < 2 then
                    c.retreatCount[c.committed.key] = (c.retreatCount[c.committed.key] or 0) + 1
                    ADIntersectionModule.startRetreat(vehicle, c)
                    return true
                end
            end
        else
            c.stuckMs = 0
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
        c.waitStart = nil
        return false
    end

    local toStop = inter.dist - ADIntersectionModule.STOP_BEFORE
    -- the horizon is the time the train needs to cross the whole red section from a stop (accelerating, top speed),
    -- ARRIVAL_S to ARRIVAL_MAX_S
    local crossSpan = math.max(0, inter.endDist) + getTrainLength(vehicle) + 2
    local horizon = math.min(ADIntersectionModule.ARRIVAL_MAX_S, math.max(ADIntersectionModule.ARRIVAL_S, crossingTime(vehicle, crossSpan) + 2))
    -- patience: after PATIENCE_MS at the line, the gap asked for in the traffic drops by 1 s every PATIENCE_STEP_MS,
    -- down to PATIENCE_MIN_S (on a road with regular traffic a long gap never comes)
    if c.waited and c.waitStart ~= nil and g_time - c.waitStart > ADIntersectionModule.PATIENCE_MS then
        local relaxed = math.max(ADIntersectionModule.PATIENCE_MIN_S,
            horizon - math.floor((g_time - c.waitStart - ADIntersectionModule.PATIENCE_MS) / ADIntersectionModule.PATIENCE_STEP_MS) - 1)
        horizon = math.min(horizon, relaxed)
    end
    local danger, reason = ADIntersectionModule.isTrafficComing(vehicle, inter.zone, horizon)
    if not danger and toStop > -1 then
        danger, reason = ADIntersectionModule.isExitBlocked(vehicle, inter.wayPoints, inter.lastIndex)
    end

    local function commit()
        if c.holding then
            vehicle.ad.specialDrivingModule:releaseVehicle()
        end
        c.committed = { key = inter.key, span = math.max(0, inter.endDist) + getTrainLength(vehicle) + 2 }
        c.travel = 0
        c.holding, c.waited, c.clearTime, c.speed = false, false, 0, nil
        c.waitStart = nil
    end

    if danger then
        c.clearTime = 0
        if not c.waited then
            c.waited = true
            c.waitStart = g_time
        end
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
            local look = ADIntersectionModule.LOOK_KMH / 3.6
            local v = math.sqrt(look * look + 2 * ADIntersectionModule.BRAKE_DECEL * math.max(0, toStop - ADIntersectionModule.COMMIT_DIST)) * 3.6
            c.speed = math.min(ADIntersectionModule.APPROACH_KMH, v)
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

-- stopped right behind a vehicle (less than 8 m on my route) going my way, or whose way is unknown: a queue, not a
-- miss. Also returns the name of that vehicle
function ADIntersectionModule.isQueuedBehindVehicle(vehicle)
    local am = vehicle.ad ~= nil and vehicle.ad.approachModule or nil
    if am == nil or am.getHit == nil then
        return false
    end
    local hit = am:getHit()
    if hit == nil or not hit.isVehicle or (hit.distance or 99) > 8 then
        return false
    end
    local node = nil
    if hit.object ~= nil then
        node = hit.object.rootNode or hit.object.node
    end
    if node ~= nil and entityExists ~= nil and entityExists(node) and localDirectionToWorld ~= nil then
        local ox, _, oz = localDirectionToWorld(node, 0, 0, 1)
        local mx, _, mz = localDirectionToWorld(vehicle.components[1].node, 0, 0, 1)
        local sameWay = ox * mx + oz * mz > 0.5
        return sameWay, hit.name
    end
    return true, hit.name
end

-- backing out of the red section (the crossing has no reverse of its own: the one of the face to face module, along
-- my own route)
function ADIntersectionModule.startRetreat(vehicle, c)
    local yieldModule = vehicle.ad.trafficYieldModule
    local x, _, z = getWorldTranslation(vehicle.components[1].node)
    local needed = c.travel + ADIntersectionModule.STOP_BEFORE + 1
    local targets = {}
    if yieldModule ~= nil and yieldModule.buildReverseTargets ~= nil then
        targets = yieldModule:buildReverseTargets(needed + 10)
    end
    c.retreat = { needed = needed, targets = targets, index = 1, startX = x, startZ = z, startTime = g_time, blockedMs = 0, key = c.committed.key, span = c.committed.span, travel = c.travel }
    c.committed = nil
    c.stuckMs = 0
    c.speed = nil
    if vehicle.ad.specialDrivingModule ~= nil then
        vehicle.ad.specialDrivingModule:releaseVehicle()
    end
end

function ADIntersectionModule.updateRetreat(vehicle, c, dt)
    local r = c.retreat
    local yieldModule = vehicle.ad.trafficYieldModule
    local x, _, z = getWorldTranslation(vehicle.components[1].node)
    local reversed = MathUtil.vector2Length(x - r.startX, z - r.startZ)
    local target = r.targets[r.index]
    local blocked = vehicle.ad.collisionDetectionModule ~= nil and vehicle.ad.collisionDetectionModule:checkReverseCollision()
    if blocked then
        r.blockedMs = r.blockedMs + dt
    else
        r.blockedMs = 0
    end
    local finished = reversed >= r.needed or target == nil or yieldModule == nil or yieldModule.reverseTowards == nil
        or g_time - r.startTime > ADIntersectionModule.RETREAT_TIMEOUT_MS
    -- a vehicle right behind: backing off is impossible, no use waiting for the timeout and acting as if I were before
    -- the line: I stay committed and wait until the way ahead is free (no new back-off at this crossing)
    if not finished and r.blockedMs >= ADIntersectionModule.RETREAT_BLOCKED_MS then
        c.retreat = nil
        c.committed = { key = r.key, span = r.span }
        c.travel = math.max(0, r.travel - reversed)
        c.noRetreat = r.key
        c.stuckMs = 0
        c.speed = nil
        if ADTrafficYieldModule ~= nil and ADTrafficYieldModule.resyncWayPoints ~= nil then
            ADTrafficYieldModule.resyncWayPoints(vehicle, r.reachedRouteIndex)
        end
        if vehicle.ad.specialDrivingModule ~= nil then
            vehicle.ad.specialDrivingModule:releaseVehicle()
        end
        return false
    end
    if finished then
        c.retreat = nil
        c.done = nil
        c.holding, c.waited, c.clearTime, c.speed = true, true, 0, nil
        c.waitStart = g_time
        c.nextCheck = 0
        if ADTrafficYieldModule ~= nil and ADTrafficYieldModule.resyncWayPoints ~= nil then
            ADTrafficYieldModule.resyncWayPoints(vehicle, r.reachedRouteIndex)
        end
        hold(vehicle, dt)
        return true
    end
    if blocked then
        -- a car right behind: let it go (the game traffic drives around or backs off in the end), then try again
        local sdm = vehicle.ad.specialDrivingModule
        sdm:stopAndHoldVehicle(dt)
        return true
    end
    if yieldModule:reverseTowards(dt, target) then
        if target.routeIndex ~= nil then
            r.reachedRouteIndex = target.routeIndex
        end
        r.index = r.index + 1
    end
    return true
end

-- the network changed (editor): zones have to be rebuilt
function ADIntersectionModule.clearCache()
    ADIntersectionModule.zones = {}
end
