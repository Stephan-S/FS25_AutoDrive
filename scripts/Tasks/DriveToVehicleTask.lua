DriveToVehicleTask = ADInheritsFrom(AbstractTask)

-- the 2nd unloader called behind the 1st.
-- It waits until the 1st is in place beside the harvester with crumbs laid, aims the crumb trail (never "35 m behind the
-- 1st" when crumbs exist), with the grid route first, then a route traced in the cut strips (band flood), then a straight
-- line; no route: it waits and tries again, and never gives up the follow (that made the harvester call it again every
-- 13 s). A route far longer than the straight distance is refused (it waits instead). A crumb close ahead: it follows at once
DriveToVehicleTask.TARGET_DISTANCE = 35        -- m behind the 1st unloader, only when it has no crumbs
DriveToVehicleTask.CRUMB_FROM_END = 40         -- m from the end of the crumb trail: the crumb aimed at (the follow keeps 40 m)
DriveToVehicleTask.ATTACH_DISTANCE = 15        -- m: a crumb this close ahead, the follow starts at once (FollowVehicleTask rule)
DriveToVehicleTask.MIN_CRUMBS = 3
DriveToVehicleTask.WAIT_FIRST_MAX = 120000     -- ms waiting for the 1st to be in place before the old target is used anyway
DriveToVehicleTask.STRAIGHT_MAX = 60           -- m: straight line to the target when no route is found
DriveToVehicleTask.STRAIGHT_ANGLE = 60         -- deg: the target this far off the heading at most for a straight line
DriveToVehicleTask.STRAIGHT_SPEED = 10         -- km/h
DriveToVehicleTask.STRAIGHT_TIMEOUT = 60000
DriveToVehicleTask.LONG_ROUTE_RATIO = 3        -- a route longer than this times the straight distance ...
DriveToVehicleTask.LONG_ROUTE_MIN = 150        -- ... and longer than this is refused (wait, plan again)
DriveToVehicleTask.LONG_ROUTE_WAIT = 15000
DriveToVehicleTask.RETRY_WAIT = 4000           -- ms before the next plan after a failure (doubles up to RETRY_WAIT_MAX)
DriveToVehicleTask.RETRY_WAIT_MAX = 15000
DriveToVehicleTask.BAND_WAIT = 10000           -- ms waiting for the flood of the cut strips
DriveToVehicleTask.REPLAN_TARGET_MOVED = 40    -- m the 1st moved since the route was planned (no-crumb target): new route
DriveToVehicleTask.REPLAN_MIN_AGE = 15000      -- ms between two plans
DriveToVehicleTask.REPLAN_MIN_LEFT = 25        -- m left on the route to still plan again
DriveToVehicleTask.ARRIVALS_MAX = 3            -- arrivals without a crumb close by before waiting in place
DriveToVehicleTask.WAIT_HERE_CHECK = 5000

DriveToVehicleTask.STATE_WAIT_FOR_FIRST = 0
DriveToVehicleTask.STATE_PATHPLANNING = 1
DriveToVehicleTask.STATE_DRIVING = 2
DriveToVehicleTask.STATE_BAND_WAIT = 3
DriveToVehicleTask.STATE_STRAIGHT = 4
DriveToVehicleTask.STATE_RETRY_WAIT = 5
DriveToVehicleTask.STATE_WAIT_HERE = 6
-- called while the 1st is not in place yet and standing in the entrance or on the road, the follower first drives to
-- a field park place (same rules as the waiting places) and waits there, the entrance stays free
DriveToVehicleTask.STATE_PARK_SEARCH = 7
DriveToVehicleTask.STATE_PARK_PLAN = 8
DriveToVehicleTask.STATE_PARK_DRIVE = 9
DriveToVehicleTask.PARK_PLAN_TIMEOUT = 30000 -- ms of pathfinding to one place at most
-- after PARK_AFTER_FAILURES failed plans in a row the follower goes to a field park place first (the entrance stays
-- free), and plans again from there
DriveToVehicleTask.PARK_AFTER_FAILURES = 2
-- the way the 1st drove (its track to the harvester, kept on the harvester, then its crumbs) is a coherent way through
-- the crop: it is taken before any search, joined where it passes near the train, every stretch checked (crop at the
-- tyres, fixed obstacles)
DriveToVehicleTask.TRACK_JOIN_DISTANCE = 25  -- m: the track must pass this close ahead of the tractor
DriveToVehicleTask.TRACK_JOIN_AHEAD = 3      -- m: joined at a point at least this far ahead
DriveToVehicleTask.TRACK_MIN_LENGTH = 10
DriveToVehicleTask.TRACK_MIN_GAIN = 5        -- m closer to the target at least
DriveToVehicleTask.TRACK_MAX_AGE = 1200000   -- ms: the 1st's track to the harvester this old at most
DriveToVehicleTask.TRACK_STOP_OVERRUN = 4    -- m cut back before a blocked spot

local function logName(vehicle)
    return vehicle ~= nil and vehicle.getName ~= nil and tostring(vehicle:getName()) or "?"
end

function DriveToVehicleTask:new(vehicle, targetVehicle, why)
    local o = DriveToVehicleTask:create()
    o.vehicle = vehicle
    o.targetVehicle = targetVehicle
    o.why = why or "call"
    o.state = DriveToVehicleTask.STATE_WAIT_FOR_FIRST
    o.wayPoints = nil
    o.trailers = nil
    o.failures = 0
    o.arrivals = 0
    o.retryDelay = DriveToVehicleTask.RETRY_WAIT
    o.retryTime = 0
    o.waitStart = nil
    o.taskType = "DriveToVehicleTask"
    return o
end

function DriveToVehicleTask:setUp()
    self.trailers, _ = AutoDrive.getAllUnits(self.vehicle)
    AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, false)
    self.waitStart = g_time or 0
    if self:isFirstReady() then
        self:startPlanning(self.why)
    else
        self.state = DriveToVehicleTask.STATE_WAIT_FOR_FIRST
        self:parkInFieldFirst()
    end
end

-- ---------------------------------------------------------------- waiting for the 1st: out of the entrance

-- standing in the entrance or on the road while waiting: a field park place is looked for (ADFieldPark rules)
function DriveToVehicleTask:parkInFieldFirst()
    if ADFieldPark == nil or ADFieldPark.isEnabled == nil or not ADFieldPark.isEnabled() or self.vehicle.ad.trafficYieldModule == nil then
        return false
    end
    local good, why = ADFieldPark.isGoodWaitingPlace(self.vehicle)
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local onField = AutoDrive.checkIsOnField(x, y, z)
    if good or (why ~= "road" and why ~= "off field" and onField) then
        return false
    end
    self.parkSearch = ADFieldPark.newSearch(self.vehicle)
    self.state = DriveToVehicleTask.STATE_PARK_SEARCH
    return true
end

function DriveToVehicleTask:parkGiveUp()
    ADFieldPark.setParked(self.vehicle, nil)
    self.parkSpot = nil
    self.state = DriveToVehicleTask.STATE_WAIT_FOR_FIRST
end

function DriveToVehicleTask:planToParkSpot()
    self.parkIndex = (self.parkIndex or 0) + 1
    local spot = self.parkSpots ~= nil and self.parkSpots[self.parkIndex] or nil
    if spot == nil or self.parkIndex > ADFieldPark.MAX_TRIES then
        self:parkGiveUp()
        return
    end
    self.parkSpot = spot
    ADFieldPark.setParked(self.vehicle, spot, true)
    local wayPoints = ADFieldPark.getMemoPath(self.vehicle, spot)
    if wayPoints ~= nil then
        self.vehicle.ad.drivePathModule:setWayPoints(wayPoints)
        self.state = DriveToVehicleTask.STATE_PARK_DRIVE
        return
    end
    local target, leadIn = ADFieldPark.getLeadIn(self.vehicle, spot)
    self.parkLeadIn = leadIn
    wayPoints = ADFieldPark.findDirectPath(self.vehicle, target)
    if wayPoints ~= nil then
        self.vehicle.ad.drivePathModule:setWayPoints(ADFieldPark.appendLeadIn(wayPoints, leadIn))
        self.state = DriveToVehicleTask.STATE_PARK_DRIVE
        return
    end
    if spot.roadside then
        self:planToParkSpot()
        return
    end
    self.parkPlanTime = 0
    local pfm = self.vehicle.ad.pathFinderModule
    pfm:reset()
    pfm:startPathPlanningTo({x = target.x, y = target.y, z = target.z}, {x = target.dirX, z = target.dirZ})
    self.state = DriveToVehicleTask.STATE_PARK_PLAN
end

function DriveToVehicleTask:updatePark(dt)
    if self.state == DriveToVehicleTask.STATE_PARK_SEARCH then
        self:stopHere(dt)
        local spots = ADFieldPark.searchStep(self.parkSearch)
        if spots ~= nil then
            self.parkSearch = nil
            self.parkSpots = spots
            self.parkIndex = 0
            self:planToParkSpot()
        end
    elseif self.state == DriveToVehicleTask.STATE_PARK_PLAN then
        local pfm = self.vehicle.ad.pathFinderModule
        self.parkPlanTime = (self.parkPlanTime or 0) + dt
        if pfm:hasFinished() then
            local wayPoints = pfm:getPath()
            local crosses = wayPoints ~= nil and #wayPoints > 1 and WaitForCallTask ~= nil and WaitForCallTask.pathCrossesCrop ~= nil and WaitForCallTask.pathCrossesCrop(self, wayPoints, pfm)
            if wayPoints == nil or #wayPoints <= 1 or pfm.fallBackMode3 or crosses then
                self:planToParkSpot()
            else
                self.vehicle.ad.drivePathModule:setWayPoints(ADFieldPark.appendLeadIn(wayPoints, self.parkLeadIn))
                self.state = DriveToVehicleTask.STATE_PARK_DRIVE
            end
        elseif self.parkPlanTime > DriveToVehicleTask.PARK_PLAN_TIMEOUT then
            pfm:reset()
            self:planToParkSpot()
        else
            pfm:update(dt)
            self:stopHere(dt)
        end
    elseif self.state == DriveToVehicleTask.STATE_PARK_DRIVE then
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            ADFieldPark.setParked(self.vehicle, self.parkSpot, false)
            self.parkedInField = true
            self.state = DriveToVehicleTask.STATE_WAIT_FOR_FIRST
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end
    end
end

function DriveToVehicleTask:leaveParkSpot()
    if self.parkSpot ~= nil and ADFieldPark ~= nil then
        ADFieldPark.setParked(self.vehicle, nil)
    end
    self.parkSpot = nil
    self.parkedInField = nil
end

-- ---------------------------------------------------------------- the 1st unloader and its crumbs

function DriveToVehicleTask:getFirstMode()
    local t = self.targetVehicle
    return t ~= nil and t.ad ~= nil and t.ad.modes ~= nil and t.ad.modes[AutoDrive.MODE_UNLOAD] or nil
end

function DriveToVehicleTask:getCrumbs()
    local mode = self:getFirstMode()
    local crumbs = mode ~= nil and mode.getBreadCrumbs ~= nil and mode:getBreadCrumbs() or nil
    return crumbs ~= nil and crumbs.items or nil
end

-- the 1st unloader is beside the harvester (active unloading) and has laid crumbs to follow
function DriveToVehicleTask:isFirstReady()
    local mode = self:getFirstMode()
    if mode == nil then
        return false
    end
    local items = self:getCrumbs()
    return mode.state == CombineUnloaderMode.STATE_ACTIVE_UNLOAD_COMBINE and items ~= nil and #items >= DriveToVehicleTask.MIN_CRUMBS
end

-- the nearest crumb ahead of the tractor (index, distance)
function DriveToVehicleTask:getNearestCrumbAhead()
    local items = self:getCrumbs()
    if items == nil then
        return nil, nil, 0
    end
    local vx, vy, vz = getWorldTranslation(self.vehicle.components[1].node)
    local nearest, nearestD = nil, math.huge
    for i, c in ipairs(items) do
        local _, _, lz = AutoDrive.worldToLocal(self.vehicle, c.x, c.y or vy, c.z)
        if lz > 1 then
            local d = MathUtil.vector2Length(c.x - vx, c.z - vz)
            if d < nearestD then
                nearest, nearestD = i, d
            end
        end
    end
    return nearest, nearestD, #items
end

-- target point, arrival vector, what it is, crumb index: the crumb CRUMB_FROM_END m before the end of the trail (or the
-- nearest one ahead when the tractor is already further along it); without crumbs, TARGET_DISTANCE m behind the 1st
function DriveToVehicleTask:chooseTarget()
    local items = self:getCrumbs()
    local nearest, nearestD = self:getNearestCrumbAhead()
    if items ~= nil and #items >= 2 and not (self.forceOldTarget and #items < DriveToVehicleTask.MIN_CRUMBS) then
        if nearest ~= nil and nearestD < DriveToVehicleTask.ATTACH_DISTANCE then
            return items[nearest], nil, "near crumb", nearest
        end
        local index = #items
        local dist = 0
        while index > 1 do
            local a, b = items[index - 1], items[index]
            dist = dist + MathUtil.vector2Length(b.x - a.x, b.z - a.z)
            index = index - 1
            if dist >= DriveToVehicleTask.CRUMB_FROM_END then
                break
            end
        end
        if nearest ~= nil and nearest > index then
            index = nearest
        end
        local p = items[index]
        local q = items[index + 1]
        local dx, dz
        if q ~= nil then
            dx, dz = q.x - p.x, q.z - p.z
        else
            dx, dz = p.x - items[index - 1].x, p.z - items[index - 1].z
        end
        local l = math.sqrt(dx * dx + dz * dz)
        if l < 0.1 then
            local rx, _, rz = AutoDrive.localDirectionToWorld(self.targetVehicle, 0, 0, 1)
            dx, dz, l = rx, rz, 1
        end
        return {x = p.x, y = p.y, z = p.z}, {x = dx / l, z = dz / l}, string.format("crumb %d/%d", index, #items), index
    end
    local tx, ty, tz = getWorldTranslation(self.targetVehicle.components[1].node)
    local rx, _, rz = AutoDrive.localDirectionToWorld(self.targetVehicle, 0, 0, 1)
    return {x = tx - DriveToVehicleTask.TARGET_DISTANCE * rx, y = ty, z = tz - DriveToVehicleTask.TARGET_DISTANCE * rz}, {x = rx, z = rz},
        string.format("%d m behind", DriveToVehicleTask.TARGET_DISTANCE), nil
end

-- ---------------------------------------------------------------- planning

function DriveToVehicleTask:startPlanning(why)
    local target, vector, kind, index = self:chooseTarget()
    if target == nil then
        self:startRetryWait(DriveToVehicleTask.RETRY_WAIT, "no target")
        return
    end
    local name, targetName = logName(self.vehicle), logName(self.targetVehicle)
    if kind == "near crumb" then
        local _, d = self:getNearestCrumbAhead()
        self:startFollowing()
        return
    end
    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
    self.target = target
    self.targetKind = kind
    self.targetIsCrumb = index ~= nil
    local tx, _, tz = getWorldTranslation(self.targetVehicle.components[1].node)
    self.targetPos = {x = tx, z = tz, time = g_time or 0}
    self.straight = MathUtil.vector2Length(target.x - vx, target.z - vz)
    -- order: the 1st's track, the crop edge, the cut strips, the (fine) grid, a straight line
    if self:tryTrack(target, index, why) then
        return
    end
    local pfm = self.vehicle.ad.pathFinderModule
    pfm:reset()
    pfm:startPathPlanningToFollow(self.targetVehicle, target, vector)
    self.bandTried = false
    self.state = DriveToVehicleTask.STATE_PATHPLANNING
end

-- ---------------------------------------------------------------- the 1st's track

-- the way the 1st drove: its track to the harvester (shared on the harvester for the next ones, TRACK_MAX_AGE at most)
-- then its crumbs up to the one aimed at. Returns the points and how many of them belong to the track to the harvester
function DriveToVehicleTask:getFirstTrack(targetIndex)
    local points = {}
    local mode = self:getFirstMode()
    local combine = mode ~= nil and mode.combine or nil
    local shared = combine ~= nil and combine.ad ~= nil and combine.ad.sharedLane or nil
    if shared ~= nil and shared.points ~= nil and (g_time or 0) - (shared.time or 0) <= DriveToVehicleTask.TRACK_MAX_AGE then
        for _, p in ipairs(shared.points) do
            points[#points + 1] = {x = p.x, y = p.y, z = p.z}
        end
    end
    local laneCount = #points
    local items = self:getCrumbs()
    if items ~= nil then
        local last = math.min(targetIndex or #items, #items)
        for i = 1, last do
            local c = items[i]
            points[#points + 1] = {x = c.x, y = c.y, z = c.z}
        end
    end
    return points, laneCount
end

-- one stretch of the 1st's track: crop at the tyres and fixed obstacles. The field test is left out on purpose: the
-- 1st drove there, and the entrance where the follower stands is road
function DriveToVehicleTask:trackStretchClear(pfm, a, b, cropHalf)
    local dx, dz = b.x - a.x, b.z - a.z
    local l = math.sqrt(dx * dx + dz * dz)
    if l < 0.05 then
        return true
    end
    local nx, nz = -dz / l, dx / l
    local n = math.max(1, math.ceil(l / 0.5))
    local samples = math.ceil(cropHalf * 4)
    for i = 0, n do
        local px, pz = a.x + dx * i / n, a.z + dz * i / n
        for k = 0, samples do
            local w = -cropHalf + 2 * cropHalf * k / samples
            if pfm:edgeCropAt(px + nx * w, pz + nz * w) then
                return false, "crop"
            end
        end
    end
    if not pfm:isStretchClear(a, b, nil, true, true) then
        return false, "obstacle"
    end
    return true
end

-- the 1st's track joined where it passes ahead of the train and followed up to the crumb aimed at
function DriveToVehicleTask:tryTrack(target, targetIndex, why)
    local pfm = self.vehicle.ad.pathFinderModule
    if pfm == nil or pfm.isStretchClear == nil or pfm.edgeCropAt == nil or pfm.getCropHalfWidth == nil then
        return false
    end
    local track = self:getFirstTrack(targetIndex)
    if #track < 2 then
        return false
    end
    local vx, vy, vz = getWorldTranslation(self.vehicle.components[1].node)
    local hx, _, hz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local hl = math.sqrt(hx * hx + hz * hz)
    if hl < 0.01 then
        return false
    end
    hx, hz = hx / hl, hz / hl
    -- join point: the nearest track point ahead of the tractor, the track running roughly our way there
    local best, bestD = nil, DriveToVehicleTask.TRACK_JOIN_DISTANCE
    for i = 1, #track - 1 do
        local p, q = track[i], track[i + 1]
        local ahead = (p.x - vx) * hx + (p.z - vz) * hz
        local d = MathUtil.vector2Length(p.x - vx, p.z - vz)
        local ldx, ldz = q.x - p.x, q.z - p.z
        local ll = math.sqrt(ldx * ldx + ldz * ldz)
        if ahead >= DriveToVehicleTask.TRACK_JOIN_AHEAD and d < bestD and ll > 0.1 and (ldx * hx + ldz * hz) / ll > 0.3 then
            best, bestD = i, d
        end
    end
    if best == nil then
        return false
    end
    local cropHalf = pfm:getCropHalfWidth()
    local points = {{x = vx, y = vy, z = vz}}
    local length, blocked = 0, false
    local i = best
    while i <= #track do
        local a, b = points[#points], track[i]
        if not self:trackStretchClear(pfm, a, b, cropHalf) then
            blocked = true
            break
        end
        length = length + MathUtil.vector2Length(b.x - a.x, b.z - a.z)
        points[#points + 1] = {x = b.x, y = b.y, z = b.z}
        i = i + 1
    end
    if blocked then
        -- stopped short of the blocked spot: the train overruns its last point
        local cut = 0
        while #points > 2 and cut < DriveToVehicleTask.TRACK_STOP_OVERRUN do
            local p1, p2 = points[#points - 1], points[#points]
            local l = MathUtil.vector2Length(p2.x - p1.x, p2.z - p1.z)
            cut = cut + l
            length = length - l
            table.remove(points, #points)
        end
    end
    if length < DriveToVehicleTask.TRACK_MIN_LENGTH then
        return false
    end
    local last = points[#points]
    local startD = MathUtil.vector2Length(target.x - vx, target.z - vz)
    local endD = MathUtil.vector2Length(target.x - last.x, target.z - last.z)
    if endD > startD - DriveToVehicleTask.TRACK_MIN_GAIN then
        return false
    end
    self.wayPoints = points
    self.vehicle.ad.drivePathModule:setWayPoints(points)
    self.state = DriveToVehicleTask.STATE_DRIVING
    self.failures = 0
    self.retryDelay = DriveToVehicleTask.RETRY_WAIT
    return true
end

function DriveToVehicleTask:getRouteLength(wayPoints)
    local length = 0
    for i = 2, #wayPoints do
        length = length + MathUtil.vector2Length(wayPoints[i].x - wayPoints[i - 1].x, wayPoints[i].z - wayPoints[i - 1].z)
    end
    return length
end

function DriveToVehicleTask:startRetryWait(delay, why)
    self.state = DriveToVehicleTask.STATE_RETRY_WAIT
    self.retryTime = delay
    self.retryWhy = why
end

-- the 1st unloader moved on since the route was planned (no-crumb target): the route ends far behind it
function DriveToVehicleTask:targetMovedAway()
    if self.targetIsCrumb or self.targetPos == nil or self.targetVehicle == nil or self.targetVehicle.components == nil then
        return false
    end
    if (g_time or 0) - self.targetPos.time < DriveToVehicleTask.REPLAN_MIN_AGE then
        return false
    end
    local tx, _, tz = getWorldTranslation(self.targetVehicle.components[1].node)
    local moved = MathUtil.vector2Length(tx - self.targetPos.x, tz - self.targetPos.z)
    if moved < DriveToVehicleTask.REPLAN_TARGET_MOVED then
        return false
    end
    local left = self.vehicle.ad.drivePathModule.getDistanceToLastWaypoint ~= nil and self.vehicle.ad.drivePathModule:getDistanceToLastWaypoint(1000) or 0
    if left < DriveToVehicleTask.REPLAN_MIN_LEFT then
        return false
    end
    return true
end

-- no grid route: the cut strips (ADBandRoute flood, maybe still running), then a straight line, else wait and plan again
function DriveToVehicleTask:noRoute()
    local pfm = self.vehicle.ad.pathFinderModule
    if pfm.bandFlood == nil and ADBandRoute ~= nil and self.target ~= nil and pfm.getCropHalfWidth ~= nil then
        pfm.bandFlood = ADBandRoute.request(self.target.x, self.target.z, pfm:getCropHalfWidth())
    end
    if pfm.bandFlood ~= nil and not pfm.bandFlood.done then
        self.state = DriveToVehicleTask.STATE_BAND_WAIT
        self.bandWaitTime = 0
        return
    end
    self:tryBandOrStraight()
end

function DriveToVehicleTask:isStraightPossible()
    if self.target == nil or self.straight == nil or self.straight > DriveToVehicleTask.STRAIGHT_MAX then
        return false, string.format("%.0f m, too far", self.straight or -1)
    end
    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
    local fx, _, fz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local angle = math.abs(AutoDrive.angleBetween({x = fx, z = fz}, {x = self.target.x - vx, z = self.target.z - vz}))
    if angle > DriveToVehicleTask.STRAIGHT_ANGLE then
        return false, string.format("target %.0f deg off my heading", angle)
    end
    local firstMode = self:getFirstMode()
    local harvester = (firstMode ~= nil and firstMode.combine) or self.targetVehicle
    local clear, reason = FollowCombineTask.isChaseLineClear(self.vehicle, harvester, self.target, AutoDrive.CHASEPOS_REAR)
    if not clear then
        return false, tostring(reason)
    end
    return true
end

function DriveToVehicleTask:tryBandOrStraight()
    local pfm = self.vehicle.ad.pathFinderModule
    local name, targetName = logName(self.vehicle), logName(self.targetVehicle)
    local path, bandWhy = nil, "no strip flood"
    if self.bandTried then
        bandWhy = "already tried before the grid"
    elseif pfm.getBandPath ~= nil then
        path, bandWhy = pfm:getBandPath()
    end
    if path ~= nil and #path >= 2 then
        self.wayPoints = path
        self.vehicle.ad.drivePathModule:setWayPoints(path)
        self.state = DriveToVehicleTask.STATE_DRIVING
        self.failures = 0
        self.retryDelay = DriveToVehicleTask.RETRY_WAIT
        return
    end
    local straightOk, straightWhy = self:isStraightPossible()
    if straightOk then
        self.state = DriveToVehicleTask.STATE_STRAIGHT
        self.straightTime = 0
        self.lineCheckTime = 0
        return
    end
    self.failures = self.failures + 1
    if self.failures >= DriveToVehicleTask.PARK_AFTER_FAILURES and not self.failParkTried then
        self.failParkTried = true
        if self:parkInFieldFirst() then
            return
        end
    end
    self:startRetryWait(self.retryDelay, "no route")
    self.retryDelay = math.min(self.retryDelay * 2, DriveToVehicleTask.RETRY_WAIT_MAX)
end

-- ---------------------------------------------------------------- arrival

function DriveToVehicleTask:startFollowing()
    self.vehicle.ad.taskModule:addTask(FollowVehicleTask:new(self.vehicle, self.targetVehicle))
    self.vehicle.ad.taskModule:setCurrentTaskFinished(ADTaskModule.DONT_PROPAGATE)
end

function DriveToVehicleTask:arrived(how)
    local name, targetName = logName(self.vehicle), logName(self.targetVehicle)
    local nearest, nearestD, crumbCount = self:getNearestCrumbAhead()
    if nearest ~= nil and nearestD < DriveToVehicleTask.ATTACH_DISTANCE then
        self:startFollowing()
        return
    end
    self.arrivals = self.arrivals + 1
    local tx, _, tz = getWorldTranslation(self.targetVehicle.components[1].node)
    self.arrivedFirstPos = {x = tx, z = tz}
    local where = nearest ~= nil and string.format("nearest ahead at %.0f m", nearestD) or "none ahead of me"
    if crumbCount >= 2 and self.arrivals < DriveToVehicleTask.ARRIVALS_MAX then
        self:startPlanning("crumbs not reached")
        return
    end
    self.state = DriveToVehicleTask.STATE_WAIT_HERE
    self.waitHereTime = 0
end

-- ---------------------------------------------------------------- update

function DriveToVehicleTask:signalPromotion()
    self.promoted = true
end

function DriveToVehicleTask:stopHere(dt)
    self.vehicle.ad.specialDrivingModule:stopVehicle()
    self.vehicle.ad.specialDrivingModule:update(dt)
end

function DriveToVehicleTask:update(dt)
    if self.targetVehicle == nil or self.targetVehicle.components == nil or g_currentMission.nodeToObject[self.targetVehicle.components[1].node] == nil then
        self:finished()
        return
    end
    if self.promoted then
        -- the 1st unloader left while this one was still on its way to it: straight to the harvester
        if self.state == DriveToVehicleTask.STATE_PATHPLANNING or self.state == DriveToVehicleTask.STATE_BAND_WAIT or self.state == DriveToVehicleTask.STATE_PARK_PLAN then
            self.vehicle.ad.pathFinderModule:reset()
        end
        self:leaveParkSpot()
        self.targetVehicle.ad.modes[AutoDrive.MODE_UNLOAD]:unregisterFollowingUnloader()
        self.vehicle.ad.taskModule:setCurrentTaskFinished()
        return
    end
    if self.state == DriveToVehicleTask.STATE_PARK_SEARCH or self.state == DriveToVehicleTask.STATE_PARK_PLAN or self.state == DriveToVehicleTask.STATE_PARK_DRIVE then
        self:updatePark(dt)
        return
    end
    -- the harvester or the 1st comes toward this follower: it steps aside (StepAsideTask) and comes back
    if StepAsideTask ~= nil and StepAsideTask.hook(self.vehicle, self.targetVehicle, dt) then
        return
    end
    local state = self.state
    if state == DriveToVehicleTask.STATE_WAIT_FOR_FIRST then
        self:stopHere(dt)
        if self:isFirstReady() then
            self:leaveParkSpot()
            self:startPlanning(self.why .. ", the 1st is in place")
        elseif (g_time or 0) - (self.waitStart or 0) > DriveToVehicleTask.WAIT_FIRST_MAX then
            self.forceOldTarget = true
            self:leaveParkSpot()
            self:startPlanning("waited too long")
        end
    elseif state == DriveToVehicleTask.STATE_PATHPLANNING then
        local pfm = self.vehicle.ad.pathFinderModule
        if pfm:hasFinished() then
            self.wayPoints = pfm:getPath()
            if self.wayPoints ~= nil and #self.wayPoints > 0 then
                local length = self:getRouteLength(self.wayPoints)
                if length > DriveToVehicleTask.LONG_ROUTE_MIN and length > DriveToVehicleTask.LONG_ROUTE_RATIO * (self.straight or 0) then
                    self:startRetryWait(DriveToVehicleTask.LONG_ROUTE_WAIT, "route too long")
                    return
                end
                self.vehicle.ad.drivePathModule:setWayPoints(self.wayPoints)
                self.state = DriveToVehicleTask.STATE_DRIVING
                self.failures = 0
                self.retryDelay = DriveToVehicleTask.RETRY_WAIT
                return
            end
            self:noRoute()
        else
            pfm:update(dt)
            self:stopHere(dt)
            -- the cut strips before the grid (the grid waits for the same flood anyway)
            if not self.bandTried and pfm.bandFlood ~= nil and pfm.bandFlood.done and pfm.getBandPath ~= nil then
                self.bandTried = true
                local path, bandWhy = pfm:getBandPath()
                if path ~= nil and #path >= 2 then
                    pfm:reset()
                    self.wayPoints = path
                    self.vehicle.ad.drivePathModule:setWayPoints(path)
                    self.state = DriveToVehicleTask.STATE_DRIVING
                    self.failures = 0
                    self.retryDelay = DriveToVehicleTask.RETRY_WAIT
                end
            end
        end
    elseif state == DriveToVehicleTask.STATE_BAND_WAIT then
        self:stopHere(dt)
        self.bandWaitTime = (self.bandWaitTime or 0) + dt
        local flood = self.vehicle.ad.pathFinderModule.bandFlood
        local done = flood == nil or ADBandRoute.step(flood, ADBandRoute.BUDGET)
        if done or self.bandWaitTime > DriveToVehicleTask.BAND_WAIT then
            self:tryBandOrStraight()
        end
    elseif state == DriveToVehicleTask.STATE_DRIVING then
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            self:arrived("route")
        elseif self:targetMovedAway() then
            self:startPlanning("the 1st moved")
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end
    elseif state == DriveToVehicleTask.STATE_STRAIGHT then
        self.straightTime = (self.straightTime or 0) + dt
        local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
        local d = MathUtil.vector2Length(vx - self.target.x, vz - self.target.z)
        local _, _, lz = AutoDrive.worldToLocal(self.vehicle, self.target.x, self.target.y or 0, self.target.z)
        if d < 4 or lz < -1 or self.straightTime > DriveToVehicleTask.STRAIGHT_TIMEOUT then
            self:arrived("straight line")
            return
        end
        self.lineCheckTime = (self.lineCheckTime or 0) + dt
        if self.lineCheckTime > 1000 then
            self.lineCheckTime = 0
            local ok, why = self:isStraightPossible()
            if not ok then
                self:startRetryWait(DriveToVehicleTask.RETRY_WAIT, "straight line blocked")
                return
            end
        end
        self.vehicle.ad.specialDrivingModule:driveToPoint(dt, self.target, DriveToVehicleTask.STRAIGHT_SPEED, true, 1, DriveToVehicleTask.STRAIGHT_SPEED, true)
    elseif state == DriveToVehicleTask.STATE_RETRY_WAIT then
        self:stopHere(dt)
        self.retryTime = self.retryTime - dt
        if self.retryTime <= 0 then
            if self:isFirstReady() or self.forceOldTarget then
                self:startPlanning("new try after " .. tostring(self.retryWhy))
            else
                self.state = DriveToVehicleTask.STATE_WAIT_FOR_FIRST
            end
        end
    elseif state == DriveToVehicleTask.STATE_WAIT_HERE then
        self:stopHere(dt)
        self.waitHereTime = (self.waitHereTime or 0) + dt
        if self.waitHereTime > DriveToVehicleTask.WAIT_HERE_CHECK then
            self.waitHereTime = 0
            local nearest, nearestD = self:getNearestCrumbAhead()
            if nearest ~= nil and nearestD < DriveToVehicleTask.ATTACH_DISTANCE then
                self:startFollowing()
                return
            end
            local tx, _, tz = getWorldTranslation(self.targetVehicle.components[1].node)
            if self.arrivedFirstPos ~= nil and MathUtil.vector2Length(tx - self.arrivedFirstPos.x, tz - self.arrivedFirstPos.z) > DriveToVehicleTask.REPLAN_TARGET_MOVED then
                self.arrivals = 0
                self:startPlanning("the 1st moved while I waited")
            end
        end
    end
end

function DriveToVehicleTask:abort()
    if self.state == DriveToVehicleTask.STATE_PATHPLANNING or self.state == DriveToVehicleTask.STATE_BAND_WAIT or self.state == DriveToVehicleTask.STATE_PARK_PLAN then
        self.vehicle.ad.pathFinderModule:reset()
    end
    self:leaveParkSpot()
    self.targetVehicle.ad.modes[AutoDrive.MODE_UNLOAD]:unregisterFollowingUnloader()
end

function DriveToVehicleTask:finished(propagate)
    self:leaveParkSpot()
    if self.targetVehicle ~= nil and self.targetVehicle.ad ~= nil and self.targetVehicle.ad.modes ~= nil then
        self.targetVehicle.ad.modes[AutoDrive.MODE_UNLOAD]:unregisterFollowingUnloader()
    end
    self.vehicle.ad.taskModule:setCurrentTaskFinished(propagate)
end

function DriveToVehicleTask:getInfoText()
    if self.state == DriveToVehicleTask.STATE_PATHPLANNING then
        local actualState, maxStates = self.vehicle.ad.pathFinderModule:getCurrentState()
        return g_i18n:getText("AD_task_pathfinding") .. string.format(" %d / %d ", actualState, maxStates)
    else
        return g_i18n:getText("AD_task_drive_to_vehicle")
    end
end

function DriveToVehicleTask:getI18nInfo()
    if self.state == DriveToVehicleTask.STATE_PATHPLANNING then
        return self:getPlanningI18n()
    else
        return "$l10n_AD_task_drive_to_vehicle;"
    end
end
