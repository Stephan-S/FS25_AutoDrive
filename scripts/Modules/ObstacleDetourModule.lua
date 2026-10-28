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
ADObstacleDetourModule.QUICK_DELAY = 3000             -- ms: first attempt, without reversing, comes quickly
ADObstacleDetourModule.VEHICLE_STILL_TIME = 45000     -- ms a vehicle in front must stand still to be driven around
ADObstacleDetourModule.STATIC_HIT_DISTANCE = 6        -- m, static object this close on the route while stopped = blocked
ADObstacleDetourModule.DRIVERLESS_STILL_TIME = 5000   -- ms for a parked vehicle (nobody inside, engine off)
ADObstacleDetourModule.BLOCKED_ZONE_AHEAD = 2.5       -- m in front of the vehicle nose: center of the zone the route must avoid (no route scan hit)
ADObstacleDetourModule.BLOCKED_ZONE_RADIUS = 2        -- m around the obstacle (the pathfinder may not see it, e.g. a low wall); cells by their center
ADObstacleDetourModule.TRAFFIC_CAR_DISTANCE = 8       -- m: a traffic car this close on the route counts as a vehicle in front (it drives off), not as a wall
ADObstacleDetourModule.GATE_PATTERNS = {"gate", "portail", "hoftor", "gatter", "barrier", "schranke", "barriere"} -- object names: a gate opens, no route around it

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
    o.vehicleStillTimer = AutoDriveTON:new()
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
    -- a lane change beside a stopped traffic car (ADAvoidModule) comes first
    if self.vehicle.ad.avoidModule ~= nil and self.vehicle.ad.avoidModule:isActive() then
        blockedByStaticObstacle = false
    end
    -- escalation: first attempt quickly and without reversing, the next one after the set delay with reversing
    local delay = (AutoDrive.getSetting("obstacleDetourDelay") or 15) * 1000
    if self:attemptsAtThisPlace() == 0 then
        delay = math.min(delay, ADObstacleDetourModule.QUICK_DELAY)
    end
    -- a vehicle in front (also a non AD one, e.g. parked): wait, it may move; driven around once it stood still long enough
    local vehicleInFront = blockedByStaticObstacle and self:isVehicleInFront()
    -- a parked vehicle (nobody inside, no AutoDrive / helper, engine off) will not move: driven around quickly
    local stillTime = ADObstacleDetourModule.VEHICLE_STILL_TIME
    if vehicleInFront and self:isDriverlessVehicleInFront() then
        stillTime = ADObstacleDetourModule.DRIVERLESS_STILL_TIME
    end
    local vehicleStill = self.vehicleStillTimer:timer(vehicleInFront, stillTime, dt)
    if self.blockTimer:timer(blockedByStaticObstacle, delay, dt) then
        self.blockTimer:timer(false)
        if vehicleInFront and not vehicleStill then
            self:log("obstacle is a vehicle, keep waiting (%.0f s)", stillTime / 1000)
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
    -- waiting at an intersection on purpose: not blocked
    if ADIntersectionModule ~= nil and ADIntersectionModule.isWaiting ~= nil and ADIntersectionModule.isWaiting(self.vehicle) then
        return false
    end
    -- a gate in front opens by itself: wait, there is no route around a gate
    if self:isGateInFront() then
        return false
    end
    if ad.drivePathModule:getIsReversing() then
        return false
    end
    local cdm = ad.collisionDetectionModule
    if cdm.lastAdTrafficOnRoute or cdm.lastReverseSectionBlocked then
        return false
    end
    if ad.approachModule ~= nil and ad.approachModule:shouldStop() then
        return true -- stopped at the gap behind another AutoDrive vehicle
    end
    if ad.sensors ~= nil and ad.sensors.frontSensorDynamicShort ~= nil and ad.sensors.frontSensorDynamicShort:pollInfo() == true then
        return true
    end
    -- stopped by the long sensor in front of a static object a few metres ahead (service kit, pallet on the road):
    -- the short sensor does not reach it, the route scan does
    if ad.approachModule ~= nil and math.abs(self.vehicle.lastSpeedReal or 0) * 3600 < 0.5 then
        local hit = ad.approachModule:getHit()
        local distance = ad.approachModule:getEstimatedDistance()
        if hit ~= nil and not hit.isVehicle and distance ~= nil and distance < ADObstacleDetourModule.STATIC_HIT_DISTANCE then
            return true
        end
    end
    return false
end

function ADObstacleDetourModule:isVehicleInFront()
    local ad = self.vehicle.ad
    if ad.approachModule ~= nil and ad.approachModule:shouldStop() then
        return true
    end
    -- a traffic car (no object behind its node, the sensors take it for a wall) stopped on the route drives off
    -- by itself most of the time: treated as a vehicle (wait first, lane change by ADAvoidModule meanwhile)
    if ad.approachModule ~= nil then
        local hit = ad.approachModule:getHit()
        local distance = ad.approachModule:getEstimatedDistance()
        if hit ~= nil and hit.isTraffic and distance ~= nil and distance <= ADObstacleDetourModule.TRAFFIC_CAR_DISTANCE then
            return true
        end
    end
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

-- the object seen by the route scan right in front is a gate or a barrier (by its name)
function ADObstacleDetourModule:isGateInFront()
    local approach = self.vehicle.ad.approachModule
    if approach == nil then
        return false
    end
    local hit = approach:getHit()
    if hit == nil or hit.isVehicle or hit.name == nil then
        return false
    end
    local name = string.lower(tostring(hit.name))
    for _, pattern in ipairs(ADObstacleDetourModule.GATE_PATTERNS) do
        if string.find(name, pattern, 1, true) then
            return true
        end
    end
    return false
end

-- called by the "stuck" safety of ADDrivePathModule: reverse, then plan a route around from here
function ADObstacleDetourModule:startFromStuck()
    local ad = self.vehicle.ad
    if not self.vehicle.isServer or not ADObstacleDetourModule.isEnabled() or self.state ~= ADObstacleDetourModule.STATE_IDLE then
        return false
    end
    if ad.drivePathModule:getIsReversing() or (ad.trafficYieldModule ~= nil and ad.trafficYieldModule:isBusy()) then
        return false
    end
    if ADIntersectionModule ~= nil and ADIntersectionModule.isWaiting ~= nil and ADIntersectionModule.isWaiting(self.vehicle) then
        return false
    end
    if self:isGateInFront() then
        return false
    end
    if not self:mayTryHere() or self:findTarget() == nil then
        return false
    end
    self.forceReverse = true
    self:start()
    self.forceReverse = nil
    return true
end

-- the vehicle seen by the route scan right in front, nil when unknown
function ADObstacleDetourModule:getVehicleInFront()
    local approach = self.vehicle.ad.approachModule
    if approach == nil then
        return nil
    end
    local hit = approach:getHit()
    if hit == nil or not hit.isVehicle or hit.object == nil or (approach:getEstimatedDistance() or 99) > 8 then
        return nil
    end
    return hit.object.getRootVehicle ~= nil and hit.object:getRootVehicle() or hit.object
end

function ADObstacleDetourModule:isDriverlessVehicleInFront()
    local root = self:getVehicleInFront()
    if root == nil then
        return false
    end
    if root.ad ~= nil and root.ad.stateModule ~= nil and root.ad.stateModule.isActive ~= nil and root.ad.stateModule:isActive() then
        return false
    end
    if (root.getIsControlled ~= nil and root:getIsControlled()) or (root.getIsEntered ~= nil and root:getIsEntered()) then
        return false
    end
    if root.getIsAIActive ~= nil and root:getIsAIActive() then
        return false
    end
    if root.getIsMotorStarted ~= nil and root:getIsMotorStarted() then
        return false
    end
    return true
end

function ADObstacleDetourModule:attemptsAtThisPlace()
    local x, _, z = self:getPosition()
    if self.lastAttemptPos ~= nil and MathUtil.vector2Length(x - self.lastAttemptPos.x, z - self.lastAttemptPos.z) < ADObstacleDetourModule.SAME_PLACE_DISTANCE then
        return self.attemptsHere
    end
    return 0
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
    -- where the front sensor stopped: the detour route must not go there again. The zone sits on the obstacle seen
    -- by the route scan when there is one; a zone centred 1 m past the nose with 3 m radius would cover the start
    -- cell and all its neighbours, so the first attempt would never find a route
    local length = self.vehicle.size ~= nil and self.vehicle.size.length or 5
    local bx, _, bz = AutoDrive.localToWorld(self.vehicle, 0, 0, length / 2 + ADObstacleDetourModule.BLOCKED_ZONE_AHEAD)
    local approach = self.vehicle.ad.approachModule
    local hit = approach ~= nil and approach:getHit() or nil
    if hit ~= nil and hit.x ~= nil and hit.z ~= nil and not hit.isVehicle then
        bx, bz = hit.x, hit.z
    end
    self.blockedZones = self.blockedZones or {}
    table.insert(self.blockedZones, {x = bx, z = bz, radius = ADObstacleDetourModule.BLOCKED_ZONE_RADIUS, byCenter = true})
    self.lastAttemptPos = {x = x, z = z}
    self.attemptsHere = self.attemptsHere + 1
    self.startPos = {x = x, y = y, z = z}
    self.maxReverse = AutoDrive.getSetting("obstacleDetourReverse") or 10
    if self.attemptsHere == 1 and not self.forceReverse then
        -- first attempt: a route around from here, without reversing
        self.maxReverse = 0
    end
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
    self:log("failed: %s (attempt %d of %d here)", reason, self.attemptsHere or 0, ADObstacleDetourModule.MAX_ATTEMPTS_SAME_PLACE)
    -- the quick attempt without reversing may fail: the next one reverses first; after the last one it stays stopped like before
    if (self.attemptsHere or 0) >= ADObstacleDetourModule.MAX_ATTEMPTS_SAME_PLACE then
        self.attemptsHere = ADObstacleDetourModule.MAX_ATTEMPTS_SAME_PLACE
    end
    self.vehicle.ad.pathFinderModule.blockedZones = nil
    self.vehicle.ad.pathFinderModule:abort()
    AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_obstacleDetour_failed;", 5000, self.vehicle.ad.stateModule:getName())
    self:setState(ADObstacleDetourModule.STATE_IDLE)
end
