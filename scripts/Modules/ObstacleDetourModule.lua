--[[
    ADObstacleDetourModule
    ----------------------
    Gets a vehicle out of a static obstacle (fallen object, wall, tree...) that blocks its route forever.

    Situation: the front collision sensor sees an obstacle that is not a vehicle. AutoDrive stops and
    waits for the way to clear, without any time limit. A static obstacle never moves, so the vehicle
    waits forever. This happens on the road network (object on the road) and on pathfinder routes
    (e.g. leaving a field: the pathfinder starts 7 m ahead of the vehicle without checking that spot,
    so a vehicle standing close to a wall gets a route through the wall).

    Resolution:
      1. The vehicle is blocked by a static obstacle for <obstacleDetourDelay> seconds.
      2. It reverses straight back up to <obstacleDetourReverse> metres. The rear sensors (tractor and
         last trailer / implement) are watched: it stops earlier if something is behind it.
      3. The pathfinder computes a route around the obstacle to a road network point further along
         the current route, which is spliced into the route.
      4. If no route is found, the vehicle stays stopped (no forced manoeuvre) and does not try again
         at the same place.
    Everything runs on the server (called from ADDrivePathModule:followWaypoints).
]]

ADObstacleDetourModule = {}

ADObstacleDetourModule.STATE_IDLE = 0
ADObstacleDetourModule.STATE_REVERSING = 1
ADObstacleDetourModule.STATE_PLANNING = 2
ADObstacleDetourModule.STATE_NAMES = {[0] = "IDLE", [1] = "REVERSING", [2] = "PLANNING"}

ADObstacleDetourModule.REVERSE_SPEED = 6              -- km/h
ADObstacleDetourModule.REVERSE_TIMEOUT = 25000        -- ms
ADObstacleDetourModule.REAR_BLOCKED_TIME = 1500       -- ms the rear sensor must stay blocked to stop reversing
ADObstacleDetourModule.TARGET_MIN_DISTANCE = 25       -- m from the vehicle to the network point to rejoin
ADObstacleDetourModule.DESTINATION_MIN_DISTANCE = 20  -- m, closer to the end of the route: obstacle is the destination itself
ADObstacleDetourModule.SAME_PLACE_DISTANCE = 15       -- m, no new attempt where an attempt already failed
ADObstacleDetourModule.MAX_ATTEMPTS_SAME_PLACE = 2
ADObstacleDetourModule.BLOCKED_ZONE_AHEAD = 1         -- m in front of the vehicle nose: center of the zone the route must avoid
ADObstacleDetourModule.BLOCKED_ZONE_RADIUS = 3        -- m (the pathfinder may not see the obstacle, e.g. a low wall)

function ADObstacleDetourModule:log(text, ...)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "ADObstacleDetourModule [%s]: " .. text,
        ADObstacleDetourModule.STATE_NAMES[self.state] or tostring(self.state), ...)
end

function ADObstacleDetourModule:new(vehicle)
    local o = {}
    setmetatable(o, self)
    self.__index = self
    o.vehicle = vehicle
    o.blockTimer = AutoDriveTON:new()
    o.stateTimer = AutoDriveTON:new()
    o.rearBlockedTimer = AutoDriveTON:new()
    o.state = ADObstacleDetourModule.STATE_IDLE
    o.lastAttemptPos = nil
    o.attemptsHere = 0
    return o
end

function ADObstacleDetourModule:reset()
    if self.state == ADObstacleDetourModule.STATE_PLANNING then
        self.vehicle.ad.pathFinderModule.blockedZones = nil
        self.vehicle.ad.pathFinderModule:abort()
    end
    self.state = ADObstacleDetourModule.STATE_IDLE
    self.blockTimer:timer(false)
    self.stateTimer:timer(false)
    self.rearBlockedTimer:timer(false)
    self.targetIndex = nil
end

function ADObstacleDetourModule.isEnabled()
    local enabled = AutoDrive.getSetting("obstacleDetourEnabled")
    return enabled == true or enabled == 1
end

function ADObstacleDetourModule:isControlling()
    return self.state ~= ADObstacleDetourModule.STATE_IDLE
end

function ADObstacleDetourModule:setState(newState)
    if newState ~= self.state then
        self.state = newState
        self.stateTimer:timer(false)
        self:log("new state")
    end
end

function ADObstacleDetourModule:getPosition()
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    return x, y, z
end

function ADObstacleDetourModule:ensureMotorStarted()
    if self.vehicle.startMotor ~= nil and not self.vehicle:getIsMotorStarted() and self.vehicle:getCanMotorRun() then
        self.vehicle:startMotor()
    end
end

function ADObstacleDetourModule:holdVehicle(dt)
    self.vehicle.ad.specialDrivingModule:stopVehicle(true)
    self.vehicle.ad.specialDrivingModule:update(dt)
end

------------------------------------------------------------------------------------------------
-- entry point, called every frame from ADDrivePathModule:followWaypoints (server)
-- returns true when this module controls the vehicle for this frame
------------------------------------------------------------------------------------------------
function ADObstacleDetourModule:handle(dt, obstacleDetected)
    if not self.vehicle.isServer then
        return false
    end
    if self.state == ADObstacleDetourModule.STATE_REVERSING then
        self:updateReversing(dt)
        return true
    elseif self.state == ADObstacleDetourModule.STATE_PLANNING then
        self:updatePlanning(dt)
        return true
    end

    local blockedByStaticObstacle = obstacleDetected and ADObstacleDetourModule.isEnabled() and self:isBlockedByStaticObstacle()
    local delay = (AutoDrive.getSetting("obstacleDetourDelay") or 15) * 1000
    if self.blockTimer:timer(blockedByStaticObstacle, delay, dt) then
        self.blockTimer:timer(false)
        -- a vehicle in front (also a non AD one, e.g. parked): keep waiting, it may move
        if self:isVehicleInFront() then
            self:log("obstacle is a vehicle, keep waiting")
            return false
        end
        if not self:mayTryHere() then
            return false
        end
        if self:findTarget() == nil then
            self:log("no network point to rejoin (or obstacle at the destination), keep waiting")
            return false
        end
        self:start()
        return true
    end
    return false
end

-- front short sensor blocked, but not by AD traffic, a reverse section or a head-on manoeuvre
function ADObstacleDetourModule:isBlockedByStaticObstacle()
    local ad = self.vehicle.ad
    if ad.trafficYieldModule ~= nil and ad.trafficYieldModule:isBusy() then
        return false
    end
    if ad.drivePathModule:getIsReversing() then
        return false
    end
    local cdm = ad.collisionDetectionModule
    if cdm.lastAdTrafficOnRoute or cdm.lastReverseSectionBlocked then
        return false
    end
    return ad.sensors ~= nil and ad.sensors.frontSensorDynamicShort ~= nil and ad.sensors.frontSensorDynamicShort:pollInfo() == true
end

function ADObstacleDetourModule:isVehicleInFront()
    local ad = self.vehicle.ad
    local excludedList = {}
    local activeTask = ad.taskModule:getActiveTask()
    if activeTask ~= nil and activeTask.getExcludedVehiclesForCollisionCheck ~= nil then
        excludedList = activeTask:getExcludedVehiclesForCollisionCheck()
    end
    local box = ad.sensors.frontSensorDynamicLong:getBoxShape()
    local boundingBox = {box.topLeft, box.topRight, box.downRight, box.downLeft}
    boundingBox[1].y = box.y
    return AutoDrive:checkForVehicleCollision(self.vehicle, boundingBox, excludedList) == true
end

function ADObstacleDetourModule:mayTryHere()
    local x, _, z = self:getPosition()
    if self.lastAttemptPos ~= nil and MathUtil.vector2Length(x - self.lastAttemptPos.x, z - self.lastAttemptPos.z) < ADObstacleDetourModule.SAME_PLACE_DISTANCE then
        if self.attemptsHere >= ADObstacleDetourModule.MAX_ATTEMPTS_SAME_PLACE then
            return false
        end
    else
        self.attemptsHere = 0
    end
    return true
end

-- first road network point of the current route far enough ahead, not too close to the destination
function ADObstacleDetourModule:findTarget()
    local dpm = self.vehicle.ad.drivePathModule
    local wayPoints, currentIndex = dpm:getWayPoints()
    if wayPoints == nil or currentIndex == nil or currentIndex < 1 or #wayPoints < 2 then
        return nil
    end
    local x, _, z = self:getPosition()
    local last = wayPoints[#wayPoints]
    if MathUtil.vector2Length(x - last.x, z - last.z) < ADObstacleDetourModule.DESTINATION_MIN_DISTANCE then
        return nil
    end
    for i = currentIndex, #wayPoints do
        local wp = wayPoints[i]
        if wp.id ~= nil and not wp.isPathFinderPoint and MathUtil.vector2Length(x - wp.x, z - wp.z) >= ADObstacleDetourModule.TARGET_MIN_DISTANCE then
            return i
        end
    end
    return nil
end

function ADObstacleDetourModule:start()
    local x, y, z = self:getPosition()
    if self.lastAttemptPos == nil or MathUtil.vector2Length(x - self.lastAttemptPos.x, z - self.lastAttemptPos.z) >= ADObstacleDetourModule.SAME_PLACE_DISTANCE then
        self.attemptsHere = 0
        self.blockedZones = {}
    end
    -- where the front sensor stopped: the detour route must not go there again
    local length = self.vehicle.size ~= nil and self.vehicle.size.length or 5
    local bx, _, bz = AutoDrive.localToWorld(self.vehicle, 0, 0, length / 2 + ADObstacleDetourModule.BLOCKED_ZONE_AHEAD)
    self.blockedZones = self.blockedZones or {}
    table.insert(self.blockedZones, {x = bx, z = bz, radius = ADObstacleDetourModule.BLOCKED_ZONE_RADIUS})
    self.lastAttemptPos = {x = x, z = z}
    self.attemptsHere = self.attemptsHere + 1
    self.startPos = {x = x, y = y, z = z}
    self.maxReverse = AutoDrive.getSetting("obstacleDetourReverse") or 10
    local tx, ty, tz = AutoDrive.localToWorld(self.vehicle, 0, 0, -100)
    self.reverseTarget = {x = tx, y = ty, z = tz}
    self.rearBlockedTimer:timer(false)
    self.vehicle.ad.specialDrivingModule:reset()
    AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.INFO, "$l10n_AD_Driver_of; %s $l10n_AD_obstacleDetour_start;", 5000, self.vehicle.ad.stateModule:getName())
    self:log("blocked by a static obstacle, attempt %d, reverse up to %d m", self.attemptsHere, self.maxReverse)
    if self.maxReverse <= 0 then
        self.reversedDistance = 0
        self:startPlanning()
    else
        self:setState(ADObstacleDetourModule.STATE_REVERSING)
    end
end

function ADObstacleDetourModule:updateReversing(dt)
    local x, _, z = self:getPosition()
    self.reversedDistance = MathUtil.vector2Length(x - self.startPos.x, z - self.startPos.z)
    local timedOut = self.stateTimer:timer(true, ADObstacleDetourModule.REVERSE_TIMEOUT, dt)
    local rearBlocked = self.vehicle.ad.collisionDetectionModule:checkReverseCollision() == true
    local rearStop = self.rearBlockedTimer:timer(rearBlocked, ADObstacleDetourModule.REAR_BLOCKED_TIME, dt)

    if self.reversedDistance >= self.maxReverse or rearStop or timedOut then
        self:log("reverse done: %.1f m (rear blocked %s, timeout %s)", self.reversedDistance, tostring(rearStop), tostring(timedOut))
        self.vehicle.ad.specialDrivingModule:reset()
        self:startPlanning()
        self:holdVehicle(dt)
        return
    end

    local sdm = self.vehicle.ad.specialDrivingModule
    if rearBlocked then
        sdm:stopAndHoldVehicle(dt)
        return
    end
    sdm:releaseVehicle()
    self:ensureMotorStarted()
    if sdm:reverseToTargetLocation(dt, self.reverseTarget, ADObstacleDetourModule.REVERSE_SPEED) then
        self.vehicle.ad.specialDrivingModule:reset()
        self:startPlanning()
    end
end

function ADObstacleDetourModule:startPlanning()
    local targetIndex = self:findTarget()
    if targetIndex == nil then
        self:fail("no network point to rejoin")
        return
    end
    local wayPoints = self.vehicle.ad.drivePathModule:getWayPoints()
    local target = wayPoints[targetIndex]
    local vector
    local nextWp = wayPoints[targetIndex + 1]
    if nextWp ~= nil then
        vector = {x = nextWp.x - target.x, z = nextWp.z - target.z}
    else
        local prevWp = wayPoints[targetIndex - 1]
        vector = {x = target.x - prevWp.x, z = target.z - prevWp.z}
    end
    self.targetIndex = targetIndex
    self.targetId = target.id

    local pf = self.vehicle.ad.pathFinderModule
    pf:reset()
    -- the pathfinder starts 7 m ahead of the vehicle without checking that spot: start closer when
    -- the vehicle could not reverse that far, so the route does not begin inside the obstacle
    local reversed = self.reversedDistance or 0
    if reversed < PathFinderModule.PATHFINDER_START_DISTANCE + 3 then
        pf.PATHFINDER_START_DISTANCE = math.max(2, reversed - 3)
    end
    pf.blockedZones = self.blockedZones
    pf:startPathPlanningTo(target, vector)
    pf.PATHFINDER_START_DISTANCE = nil -- back to the class value
    -- keep the rejoined network point a network point (see PathFinderModule:createWayPoints)
    pf.goingToNetwork = true
    self:log("planning to network point %s (route index %d), %d zone(s) forbidden", tostring(target.id), targetIndex, #self.blockedZones)
    self:setState(ADObstacleDetourModule.STATE_PLANNING)
end

function ADObstacleDetourModule:updatePlanning(dt)
    local pf = self.vehicle.ad.pathFinderModule
    self:holdVehicle(dt)
    if not pf:hasFinished() then
        pf:update(dt)
        return
    end
    local path = pf:getPath()
    pf.blockedZones = nil
    if path == nil or #path < 2 or path[#path].id ~= self.targetId then
        self:fail("no route around the obstacle")
        return
    end

    local dpm = self.vehicle.ad.drivePathModule
    local wayPoints = dpm:getWayPoints()
    -- the route must start with a point behind the first target: AutoDrive looks at the previous point
    -- (checkForReverseSection) and fails on index 1. Like setWayPoints, the vehicle drives from index 2.
    local vx, vy, vz = self:getPosition()
    local newWayPoints = {{x = vx, y = vy, z = vz, isPathFinderPoint = true}}
    for _, wp in ipairs(path) do
        table.insert(newWayPoints, wp)
    end
    for i = self.targetIndex + 1, #wayPoints do
        table.insert(newWayPoints, wayPoints[i])
    end
    dpm.wayPoints = newWayPoints
    dpm:setCurrentWayPointIndex(2)
    dpm.minDistanceToNextWp = math.huge
    dpm.minDistanceTimer:timer(false)
    dpm:setDirtyFlag()
    self.vehicle.ad.specialDrivingModule:releaseVehicle()

    AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.INFO, "$l10n_AD_Driver_of; %s $l10n_AD_obstacleDetour_found;", 5000, self.vehicle.ad.stateModule:getName())
    self:log("route around the obstacle found: %d points, rejoining at %s", #path, tostring(self.targetId))
    self:setState(ADObstacleDetourModule.STATE_IDLE)
end

function ADObstacleDetourModule:fail(reason)
    self:log("failed: %s", reason)
    -- no new attempt at this place: it stays stopped like before
    self.attemptsHere = ADObstacleDetourModule.MAX_ATTEMPTS_SAME_PLACE
    self.vehicle.ad.pathFinderModule.blockedZones = nil
    self.vehicle.ad.pathFinderModule:abort()
    AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_obstacleDetour_failed;", 5000, self.vehicle.ad.stateModule:getName())
    self:setState(ADObstacleDetourModule.STATE_IDLE)
end
