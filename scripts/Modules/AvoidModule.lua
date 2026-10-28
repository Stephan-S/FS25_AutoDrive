--[[
    ADAvoidModule
    -------------
    Light obstacle on one side of the route (pole, bale, sign, parked car...): the vehicle shifts its followed
    line a little to the other side and passes without stopping, then comes back.

    Uses the route scan of ADApproachModule: the obstacle ahead, its distance and the side of the corridor it is
    on. The offset candidates (1 .. 2.5 m away from the obstacle) are tested on a shifted corridor over
    CLEAR_RANGE metres, vehicles included (nothing may come the other way). The first clear one is applied as
    ADDrivePathModule.lateralOffset, ramped in and out with the distance driven, held until the whole train
    has passed the obstacle. Speed is capped meanwhile. Only on the road network, never on a field, never while a
    head-on plan (TrafficYield) or the obstacle detour is running. A vehicle counts as an obstacle only once it
    stood still for VEHICLE_STILL_TIME.

    AI traffic car stopped in front, across the corridor (nose to nose: it waits for us and we wait for it):
    lane change. The vehicle backs up just enough room (STATE_BACKING, like the obstacle detour), then shifts a
    whole lane (LANE_OFFSETS, right side first) and passes; the car drives on once its lane is free.
    Everything runs on the server (called from ADDrivePathModule:followWaypoints).
]]

ADAvoidModule = {}


ADAvoidModule.STATE_IDLE = 0
ADAvoidModule.STATE_SHIFTING = 1
ADAvoidModule.STATE_RETURNING = 2
ADAvoidModule.STATE_BACKING = 3
ADAvoidModule.STATE_NAMES = {[0] = "IDLE", [1] = "SHIFTING", [2] = "RETURNING", [3] = "BACKING"}

ADAvoidModule.OFFSETS = {1.0, 1.5, 2.0, 2.5}  -- m, tried in this order
ADAvoidModule.LANE_OFFSETS = {2.5, 3.0, 3.5}  -- m, lane change beside a stopped traffic car
ADAvoidModule.TRIGGER_DISTANCE = 25           -- m, obstacle closer than this: start the manoeuvre
ADAvoidModule.MIN_DISTANCE = 2                -- m, closer than this: too late, the sensors stop the vehicle
ADAvoidModule.CLEAR_RANGE = 60                -- m, the shifted corridor must be free that far (oncoming traffic)
ADAvoidModule.HOLD_RANGE = 15                 -- m, the shifted corridor is watched that far while passing
ADAvoidModule.RATE = 0.3                      -- m of offset per metre driven
ADAvoidModule.LANE_RATE = 0.5                 -- m of offset per metre driven for a lane change
ADAvoidModule.PASS_MARGIN = 4                 -- m driven past the obstacle before coming back
ADAvoidModule.SPEED = 12                      -- km/h cap while shifted
ADAvoidModule.START_SPEED = 20                -- km/h, the manoeuvre starts only below this
ADAvoidModule.VEHICLE_STILL_TIME = 5000       -- ms a vehicle must stand still to be avoided
ADAvoidModule.VEHICLE_STILL_SPEED = 0.5       -- km/h
ADAvoidModule.TRAFFIC_STILL_TIME = 2000       -- ms a traffic car must stand still (it only waits for us)
ADAvoidModule.TRAFFIC_STILL_MOVE = 0.3        -- m, a traffic car that moved more than this is driving
ADAvoidModule.BACK_MARGIN = 3                 -- m of room kept in front of the car after the lane change
ADAvoidModule.BACK_MAX = 15                   -- m, never back up more than this
ADAvoidModule.BACK_TIMEOUT = 20000            -- ms
ADAvoidModule.REAR_BLOCKED_TIME = 1500        -- ms the rear sensor must stay blocked to stop backing
ADAvoidModule.REVERSE_SPEED = 6               -- km/h
ADAvoidModule.CHECK_FRAMES = 8
ADAvoidModule.LOG_INTERVAL = 5000             -- ms between two "cannot avoid" logs for the same obstacle
ADAvoidModule.COOLDOWN = 2000                 -- ms after a manoeuvre before the next

function ADAvoidModule:log(text, ...)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "ADAvoidModule [%s]: " .. text,
        ADAvoidModule.STATE_NAMES[self.state] or tostring(self.state), ...)
end

function ADAvoidModule:new(vehicle)
    local o = {}
    setmetatable(o, self)
    self.__index = self
    o.vehicle = vehicle
    o.state = ADAvoidModule.STATE_IDLE
    o.offset = 0
    o.target = 0
    o.rate = ADAvoidModule.RATE
    o.stillObject = nil
    o.stillSince = nil
    o.lastLogTime = -math.huge
    o.cooldownUntil = 0
    o.backTimer = AutoDriveTON:new()
    o.rearBlockedTimer = AutoDriveTON:new()
    return o
end

function ADAvoidModule:reset()
    if self.state == ADAvoidModule.STATE_BACKING and self.vehicle.ad ~= nil and self.vehicle.ad.specialDrivingModule ~= nil then
        self.vehicle.ad.specialDrivingModule:reset()
    end
    self.state = ADAvoidModule.STATE_IDLE
    self.offset = 0
    self.target = 0
    self.rate = ADAvoidModule.RATE
    self.travelled = 0
    self.passTravel = nil
    self.stillObject = nil
    self.stillSince = nil
    self.backNeeded = nil
    self.backTimer:timer(false)
    self.rearBlockedTimer:timer(false)
    if self.vehicle.ad ~= nil and self.vehicle.ad.drivePathModule ~= nil then
        self.vehicle.ad.drivePathModule.lateralOffset = 0
    end
end

function ADAvoidModule.isEnabled()
    local enabled = AutoDrive.getSetting("avoidEnabled")
    return (enabled == nil or enabled == true or enabled == 1) and (AutoDrive.getSetting("enableTrafficDetection") or 0) >= 1
end

function ADAvoidModule:isActive()
    return self.state ~= ADAvoidModule.STATE_IDLE or self.offset ~= 0
end

function ADAvoidModule:isControlling()
    return self.state == ADAvoidModule.STATE_BACKING
end

function ADAvoidModule:getOffset()
    return self.offset
end

-- how far ADApproachModule must scan while a manoeuvre runs
function ADAvoidModule:getScanRange()
    if self:isActive() then
        return ADAvoidModule.HOLD_RANGE
    end
    return 0
end

function ADAvoidModule:getTrainLength()
    local length = self.vehicle.size ~= nil and self.vehicle.size.length or 6
    local units = AutoDrive.getAllUnits(self.vehicle)
    if units ~= nil then
        for _, unit in pairs(units) do
            if unit ~= self.vehicle and unit.size ~= nil and unit.size.length ~= nil then
                length = length + unit.size.length
            end
        end
    end
    return length
end

------------------------------------------------------------------------------------------------
-- may the obstacle be avoided ?
------------------------------------------------------------------------------------------------
function ADAvoidModule:isOnField()
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    return AutoDrive.checkIsOnField(x, y, z) == true
end

function ADAvoidModule:isBusyElsewhere()
    local ad = self.vehicle.ad
    if ad.drivePathModule:getIsReversing() or not ad.drivePathModule:isOnRoadNetwork() then
        return true
    end
    if ad.trafficYieldModule ~= nil and ad.trafficYieldModule.isBusy ~= nil and ad.trafficYieldModule:isBusy() then
        return true
    end
    if ad.obstacleDetourModule ~= nil and ad.obstacleDetourModule:isControlling() then
        return true
    end
    -- waiting at an intersection: the stopped car in front is the one we wait for
    if ADIntersectionModule ~= nil and ADIntersectionModule.isWaiting ~= nil and ADIntersectionModule.isWaiting(self.vehicle) then
        return true
    end
    return false
end

-- a vehicle is an obstacle once it stood still long enough and is not waiting for me in a head-on plan
function ADAvoidModule:isVehicleStill(hit, dt)
    local object = hit.object
    if object == nil then
        return self:isTrafficStill(hit)
    end
    local root = object.getRootVehicle ~= nil and object:getRootVehicle() or object
    local speed = math.abs(root.lastSpeedReal or 0) * 3600
    local still = speed < ADAvoidModule.VEHICLE_STILL_SPEED
    if root.ad ~= nil and root.ad.trafficYieldModule ~= nil and root.ad.trafficYieldModule.partner == self.vehicle then
        still = false
    end
    if object ~= self.stillObject or not still then
        self.stillObject = object
        self.stillSince = still and g_time or nil
        return false
    end
    return self.stillSince ~= nil and (g_time - self.stillSince) >= ADAvoidModule.VEHICLE_STILL_TIME
end

-- an AI traffic car has no game object: it stands still when its node no longer moves
function ADAvoidModule:isTrafficStill(hit)
    local node = hit.node
    if node == nil or node == 0 or not hit.isTraffic then
        return false
    end
    local x, _, z = getWorldTranslation(node)
    if node ~= self.stillObject or self.stillX == nil or MathUtil.vector2Length(x - self.stillX, z - self.stillZ) > ADAvoidModule.TRAFFIC_STILL_MOVE then
        self.stillObject = node
        self.stillX, self.stillZ = x, z
        self.stillSince = g_time
        return false
    end
    return (g_time - self.stillSince) >= ADAvoidModule.TRAFFIC_STILL_TIME
end

function ADAvoidModule:tryStart(dt)
    local approach = self.vehicle.ad.approachModule
    if approach == nil then
        return
    end
    local hit = approach:getHit()
    local distance = approach:getEstimatedDistance()
    if hit == nil or distance == nil then
        -- the scan may lose the obstacle for a moment: the "stands still" clock is kept for 2 s
        if self.lastHitTime ~= nil and g_time - self.lastHitTime > 2000 then
            self.stillObject = nil
            self.stillSince = nil
        end
        return
    end
    self.lastHitTime = g_time
    if hit.isVehicle and not self:isVehicleStill(hit, dt) then
        return
    end
    if distance > ADAvoidModule.TRIGGER_DISTANCE then
        return
    end
    local speed = math.abs(self.vehicle.lastSpeedReal or 0) * 3600
    if speed > ADAvoidModule.START_SPEED then
        return
    end
    if hit.side == 0 and (hit.isTraffic or not hit.isVehicle) then
        -- traffic car stopped across my line (it waits for me), or an object across the corridor (service kit,
        -- pallet, bale on the road): a lane change, after backing up when too close. A wall or a gate across the
        -- road fails the corridor test and goes to the detour as before
        if not self:isBusyElsewhere() and not self:isOnField() then
            self:tryLaneChange(hit, distance, speed)
        end
        return
    end
    if distance < ADAvoidModule.MIN_DISTANCE then
        return
    end
    if hit.side == 0 then
        if not hit.isVehicle then
            self:logLimited(hit, "obstacle across the corridor, cannot pass beside it")
        end
        return
    end
    if self:isBusyElsewhere() or self:isOnField() then
        return
    end
    local sign = -hit.side
    local range = math.max(ADAvoidModule.CLEAR_RANGE, distance + self:getTrainLength() + ADAvoidModule.PASS_MARGIN)
    for _, e in ipairs(ADAvoidModule.OFFSETS) do
        local clear, reason = approach:isCorridorClear(sign * e, range)
        if clear then
            self.target = sign * e
            self.rate = ADAvoidModule.RATE
            self.travelled = 0
            self.passTravel = distance + self:getTrainLength() + ADAvoidModule.PASS_MARGIN
            self.state = ADAvoidModule.STATE_SHIFTING
            self:log("'%s' %.1f m ahead on side %d: shifting %.1f m to the other side, back after %.0f m (%.0f km/h)",
                tostring(hit.name), distance, hit.side, self.target, self.passTravel, speed)
            return
        else
            self.lastReason = reason
        end
    end
    self:logLimited(hit, string.format("no free corridor beside it (%s)", tostring(self.lastReason)))
end

-- lane change beside a stopped traffic car: right side first (local +x is left in GIANTS: right is a negative offset)
function ADAvoidModule:findLane(hit, distance)
    local approach = self.vehicle.ad.approachModule
    local range = math.max(ADAvoidModule.CLEAR_RANGE, distance + self:getTrainLength() + ADAvoidModule.PASS_MARGIN)
    for _, sign in ipairs({-1, 1}) do
        for _, e in ipairs(ADAvoidModule.LANE_OFFSETS) do
            local clear, reason = approach:isCorridorClear(sign * e, range)
            if clear then
                return sign * e
            end
            self.lastReason = reason
        end
    end
    return nil
end

function ADAvoidModule:tryLaneChange(hit, distance, speed)
    local target = self:findLane(hit, distance)
    if target == nil then
        self:logLimited(hit, string.format("%s across my line, no free lane beside it (%s)", hit.isTraffic and "traffic car" or "obstacle", tostring(self.lastReason)))
        return
    end
    -- room needed in front of the car to reach the offset: offset / rate, plus a margin
    local needed = math.abs(target) / ADAvoidModule.LANE_RATE + ADAvoidModule.BACK_MARGIN
    self.target = target
    self.rate = ADAvoidModule.LANE_RATE
    self.travelled = 0
    self.hitDistance = distance
    self.hitName = hit.name
    self.passTravel = distance + self:getTrainLength() + ADAvoidModule.PASS_MARGIN
    if distance >= needed then
        self.state = ADAvoidModule.STATE_SHIFTING
        self:log("%s '%s' %.1f m ahead across my line: lane change %.1f m, back after %.0f m (%.0f km/h)",
            hit.isTraffic and "traffic car stopped" or "obstacle", tostring(hit.name), distance, self.target, self.passTravel, speed)
        return
    end
    self.backNeeded = math.min(ADAvoidModule.BACK_MAX, needed - distance)
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    self.startPos = {x = x, z = z}
    local tx, ty, tz = AutoDrive.localToWorld(self.vehicle, 0, 0, -100)
    self.reverseTarget = {x = tx, y = ty, z = tz}
    self.backTimer:timer(false)
    self.rearBlockedTimer:timer(false)
    self.vehicle.ad.specialDrivingModule:reset()
    self.state = ADAvoidModule.STATE_BACKING
    self:log("%s '%s' %.1f m ahead across my line: backing up %.1f m, then lane change %.1f m",
        hit.isTraffic and "traffic car stopped" or "obstacle", tostring(hit.name), distance, self.backNeeded, self.target)
end

function ADAvoidModule:logLimited(hit, text)
    if g_time - self.lastLogTime > ADAvoidModule.LOG_INTERVAL then
        self.lastLogTime = g_time
        self:log("'%s' %.1f m ahead: %s", tostring(hit.name), hit.distance or -1, text)
    end
end

------------------------------------------------------------------------------------------------
-- backing up before a lane change: takes control of the vehicle (called from followWaypoints,
-- returns true while this module drives)
------------------------------------------------------------------------------------------------
function ADAvoidModule:handle(dt)
    if self.state ~= ADAvoidModule.STATE_BACKING then
        return false
    end
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    local reversed = MathUtil.vector2Length(x - self.startPos.x, z - self.startPos.z)
    local timedOut = self.backTimer:timer(true, ADAvoidModule.BACK_TIMEOUT, dt)
    local cdm = self.vehicle.ad.collisionDetectionModule
    local rearBlocked = cdm ~= nil and cdm.checkReverseCollision ~= nil and cdm:checkReverseCollision() == true
    local rearStop = self.rearBlockedTimer:timer(rearBlocked, ADAvoidModule.REAR_BLOCKED_TIME, dt)
    local sdm = self.vehicle.ad.specialDrivingModule

    if reversed >= self.backNeeded or rearStop or timedOut then
        sdm:reset()
        local distance = (self.hitDistance or 0) + reversed
        -- the lane must still be free from here
        local target = self:findLane({name = self.hitName}, distance)
        if target == nil then
            self:log("backed up %.1f m (rear blocked %s, timeout %s): no free lane any more, giving up", reversed, tostring(rearStop), tostring(timedOut))
            self:reset()
            self.cooldownUntil = g_time + ADAvoidModule.COOLDOWN
            return false
        end
        self.target = target
        self.travelled = 0
        self.passTravel = distance + self:getTrainLength() + ADAvoidModule.PASS_MARGIN
        self.state = ADAvoidModule.STATE_SHIFTING
        self:log("backed up %.1f m (rear blocked %s, timeout %s): lane change %.1f m, car %.1f m ahead, back after %.0f m",
            reversed, tostring(rearStop), tostring(timedOut), self.target, distance, self.passTravel)
        return false
    end
    if rearBlocked then
        sdm:stopAndHoldVehicle(dt)
        return true
    end
    sdm:releaseVehicle()
    if self.vehicle.startMotor ~= nil and not self.vehicle:getIsMotorStarted() and self.vehicle:getCanMotorRun() then
        self.vehicle:startMotor()
    end
    sdm:reverseToTargetLocation(dt, self.reverseTarget, ADAvoidModule.REVERSE_SPEED)
    return true
end

------------------------------------------------------------------------------------------------
-- entry point, called every frame from ADDrivePathModule:followWaypoints (server)
-- returns the speed limit (km/h) while a manoeuvre runs, math.huge otherwise
------------------------------------------------------------------------------------------------
function ADAvoidModule:update(dt)
    if not self.vehicle.isServer or not ADAvoidModule.isEnabled() then
        if self.offset ~= 0 or self.state ~= ADAvoidModule.STATE_IDLE then
            self:reset()
        end
        return math.huge
    end
    local dpm = self.vehicle.ad.drivePathModule
    if self.state == ADAvoidModule.STATE_BACKING then
        dpm.lateralOffset = 0
        return math.huge -- handle() drives
    end
    local step = math.abs(self.vehicle.lastSpeedReal or 0) * dt -- metres driven this frame
    self.travelled = (self.travelled or 0) + step

    if self.state == ADAvoidModule.STATE_IDLE then
        if g_time >= self.cooldownUntil and ((g_updateLoopIndex + self.vehicle.id) % ADAvoidModule.CHECK_FRAMES) == 0 then
            self:tryStart(dt)
        end
        if self.state == ADAvoidModule.STATE_IDLE or self.state == ADAvoidModule.STATE_BACKING then
            dpm.lateralOffset = 0
            return math.huge
        end
    end

    -- the detour or a head-on plan took over: back on the line at once
    if self:isBusyElsewhere() then
        self:log("another manoeuvre took over, offset dropped")
        self:reset()
        self.cooldownUntil = g_time + ADAvoidModule.COOLDOWN
        return math.huge
    end

    if self.state == ADAvoidModule.STATE_SHIFTING then
        if ((g_updateLoopIndex + self.vehicle.id) % ADAvoidModule.CHECK_FRAMES) == 0 then
            local clear, reason = self.vehicle.ad.approachModule:isCorridorClear(self.target, ADAvoidModule.HOLD_RANGE)
            if not clear then
                self:log("shifted corridor no longer free (%s): coming back", tostring(reason))
                self.target = 0
                self.state = ADAvoidModule.STATE_RETURNING
            end
        end
        if self.passTravel ~= nil and self.travelled >= self.passTravel then
            self:log("obstacle passed (%.0f m driven): coming back on the line", self.travelled)
            self.target = 0
            self.state = ADAvoidModule.STATE_RETURNING
        end
    end

    -- ramp the offset with the distance driven
    local maxChange = step * (self.rate or ADAvoidModule.RATE)
    if self.offset < self.target then
        self.offset = math.min(self.target, self.offset + maxChange)
    elseif self.offset > self.target then
        self.offset = math.max(self.target, self.offset - maxChange)
    end
    dpm.lateralOffset = self.offset

    if self.state == ADAvoidModule.STATE_RETURNING and math.abs(self.offset) < 0.05 then
        self.offset = 0
        dpm.lateralOffset = 0
        self.state = ADAvoidModule.STATE_IDLE
        self.rate = ADAvoidModule.RATE
        self.cooldownUntil = g_time + ADAvoidModule.COOLDOWN
        self:log("back on the line")
        return math.huge
    end
    return ADAvoidModule.SPEED
end
