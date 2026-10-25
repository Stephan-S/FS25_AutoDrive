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
    end
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
    local pfm = self.vehicle.ad.pathFinderModule
    pfm:reset()
    pfm:startPathPlanningToFollow(self.targetVehicle, target, vector)
    self.target = target
    self.targetKind = kind
    self.targetIsCrumb = index ~= nil
    local tx, _, tz = getWorldTranslation(self.targetVehicle.components[1].node)
    self.targetPos = {x = tx, z = tz, time = g_time or 0}
    self.straight = MathUtil.vector2Length(target.x - vx, target.z - vz)
    self.state = DriveToVehicleTask.STATE_PATHPLANNING
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
    if pfm.getBandPath ~= nil then
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
        if self.state == DriveToVehicleTask.STATE_PATHPLANNING or self.state == DriveToVehicleTask.STATE_BAND_WAIT then
            self.vehicle.ad.pathFinderModule:reset()
        end
        self.targetVehicle.ad.modes[AutoDrive.MODE_UNLOAD]:unregisterFollowingUnloader()
        self.vehicle.ad.taskModule:setCurrentTaskFinished()
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
            self:startPlanning(self.why .. ", the 1st is in place")
        elseif (g_time or 0) - (self.waitStart or 0) > DriveToVehicleTask.WAIT_FIRST_MAX then
            self.forceOldTarget = true
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
    if self.state == DriveToVehicleTask.STATE_PATHPLANNING or self.state == DriveToVehicleTask.STATE_BAND_WAIT then
        self.vehicle.ad.pathFinderModule:reset()
    end
    self.targetVehicle.ad.modes[AutoDrive.MODE_UNLOAD]:unregisterFollowingUnloader()
end

function DriveToVehicleTask:finished(propagate)
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
