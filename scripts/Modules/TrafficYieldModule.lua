--[[
    ADTrafficYieldModule
    --------------------
    Resolves head-on deadlocks between two AutoDrive vehicles on two-way roads.

    Situation: two AD vehicles meet face to face on a dual road. Both detect each other with
    their collision sensors and stop forever (detectAdTrafficOnRoute() is skipped on dual roads).

    Resolution:
      1. After both vehicles are blocked for <trafficYieldDelay> seconds, a shared plan is created.
      2. Roles are chosen deterministically (same result whichever vehicle detects first):
           - REVERSER : the vehicle easiest to reverse (empty first, then no trailer, then lowest id).
                        It backs up along its own route to make room.
           - PARKER   : the other vehicle. It searches a free spot on the roadside (right side first,
                        then left), drives into it and waits.
      3. Once the parker is parked, the reverser drives on and passes. When it has passed,
         the parker returns to its route.
    Everything runs on the server (AutoDrive:onUpdate is server side), so it is multiplayer safe.
]]

ADTrafficYieldModule = {}

ADTrafficYieldModule.DEBUG = false -- set to true to print [AD-Yield] diagnostic logs

function ADTrafficYieldModule:log(text, ...)
    if ADTrafficYieldModule.DEBUG then
        print(string.format("[AD-Yield] %s (%s) [%s]: " .. text, ADTrafficYieldModule.getName(self.vehicle), tostring(self.vehicle.id),
            ADTrafficYieldModule.STATE_NAMES[self.state] or tostring(self.state), ...))
    end
end

ADTrafficYieldModule.STATE_IDLE = 0
ADTrafficYieldModule.STATE_REVERSER_RETREAT = 1
ADTrafficYieldModule.STATE_REVERSER_WAIT = 2
ADTrafficYieldModule.STATE_REVERSER_PASS = 3
ADTrafficYieldModule.STATE_REVERSER_BRANCH = 4
ADTrafficYieldModule.STATE_REVERSER_HIDE = 5
ADTrafficYieldModule.STATE_PARKER_SEARCH = 11
ADTrafficYieldModule.STATE_PARKER_MOVE = 12
ADTrafficYieldModule.STATE_PARKER_WAIT = 13
ADTrafficYieldModule.STATE_PARKER_PASS = 14
ADTrafficYieldModule.STATE_YIELDER_APPROACH = 21
ADTrafficYieldModule.STATE_PRIORITY_APPROACH = 22
ADTrafficYieldModule.STATE_WAIT_DETACHED = 31
ADTrafficYieldModule.STATE_SECTION_CLEAR = 41

ADTrafficYieldModule.STATE_NAMES = {
    [0] = "IDLE",
    [1] = "REVERSER_RETREAT",
    [2] = "REVERSER_WAIT",
    [3] = "REVERSER_PASS",
    [4] = "REVERSER_BRANCH",
    [5] = "REVERSER_HIDE",
    [11] = "PARKER_SEARCH",
    [12] = "PARKER_MOVE",
    [13] = "PARKER_WAIT",
    [14] = "PARKER_PASS",
    [21] = "YIELDER_APPROACH",
    [22] = "PRIORITY_APPROACH",
    [31] = "WAIT_DETACHED",
    [41] = "SECTION_CLEAR"
}

ADTrafficYieldModule.ANTICIPATION_DISTANCE = 250 -- m of route looked ahead to find an oncoming AD vehicle
ADTrafficYieldModule.ANTICIPATION_INTERVAL = 250 -- ms between two route comparisons
ADTrafficYieldModule.ANTICIPATION_DECEL = 1.2    -- m/s2 comfortable deceleration used to compute the speed limit
ADTrafficYieldModule.ANTICIPATION_GAP = 10       -- m free between both fronts when stopped
ADTrafficYieldModule.ANTICIPATION_START_DELAY = 1500 -- ms both stopped before the manoeuvre starts
ADTrafficYieldModule.PLAN_TIMEOUT = 180000      -- ms, whole manoeuvre
ADTrafficYieldModule.COOLDOWN = 20000           -- ms, no new detection after a plan ended
ADTrafficYieldModule.PATH_STEP = 2               -- m between two points of the parallel path
ADTrafficYieldModule.PATH_EXTRA = 8              -- m of parallel path after the parking point (alignment, aim)
ADTrafficYieldModule.CANDIDATE_STEP = 5          -- m between two leaving points tested along the route
ADTrafficYieldModule.SECTION_WAIT_CLEAR = 15     -- m kept free before the start of a single track when the yielder waits there
ADTrafficYieldModule.SECTION_BACKOFF_MAX = 50    -- m, longest reverse of a yielder already in the mouth of the single track
ADTrafficYieldModule.JUNCTION_GAP = 5           -- m between my front and the node where my route leaves the road, when I wait before the turn-off
ADTrafficYieldModule.SECTION_BACKOFF_MIN = 6     -- m, a back-off stopped before (vehicle queue behind) falls back to the standard manoeuvre
ADTrafficYieldModule.SECTION_CLEAR_MAX = 150     -- m, longest reverse of the vehicle in the single track that gives way to one stuck in the mouth
ADTrafficYieldModule.SECTION_BACKOFF_INSIDE = 8  -- m, front already this far inside the single track: too deep, no back-off
ADTrafficYieldModule.YIELDER_STOPPED_FALLBACK = 10000 -- ms stopped without a spot nor a hold of its own: standard manoeuvre
ADTrafficYieldModule.YIELDER_SHARE = 0.65        -- part of the free distance given to the vehicle that pulls over
ADTrafficYieldModule.DETOUR_MAX = 400            -- m, longest detour taken to avoid meeting an oncoming vehicle
ADTrafficYieldModule.DETOUR_MIN_AHEAD = 5        -- m, the detour must leave the route before the head-on section
ADTrafficYieldModule.DETOUR_PENALTY = 1000000    -- added weight of the head-on section waypoints when looking for a detour
ADTrafficYieldModule.SPOT_CLEARANCE = 6          -- m kept free between the oncoming vehicle and the end of the roadside path
ADTrafficYieldModule.COOLDOWN_SUCCESS = 3000    -- ms, after a successful plan (another conflict may follow at once)
ADTrafficYieldModule.GIVE_UP_TIME = 300000      -- ms, after two failed attempts the pair is left alone this long
ADTrafficYieldModule.HISTORY_TIMEOUT = 600000   -- ms, a failure older than this is forgotten

-- failed attempts per pair of vehicles: key "idA-idB" -> {failures, time}
ADTrafficYieldModule.pairHistory = {}
ADTrafficYieldModule.SEARCH_INTERVAL = 500      -- ms between two roadside searches
ADTrafficYieldModule.MAX_DETECTION_DISTANCE = 45 -- m between the two vehicle centres
ADTrafficYieldModule.REVERSE_STEP = 5           -- m between two reverse targets
ADTrafficYieldModule.REVERSE_SPEED = 7          -- km/h
ADTrafficYieldModule.PARK_SPEED = 7             -- km/h
ADTrafficYieldModule.LATERAL_MARGIN = 0.6       -- m free space between the two vehicles when passing
ADTrafficYieldModule.MAX_SLOPE_DIFF = 1.2       -- m height difference allowed inside the parking footprint
-- map collision shapes that never block a vehicle in practice (field border collisions of the maps)
ADTrafficYieldModule.IGNORED_SHAPE_NAMES = {
    borderCol = true
}
ADTrafficYieldModule.MAX_ROAD_HEIGHT_DIFF = 1.0 -- m height difference allowed between road and parking spot

function ADTrafficYieldModule:new(vehicle)
    local o = {}
    setmetatable(o, self)
    self.__index = self
    o.vehicle = vehicle
    o.blockedTimer = AutoDriveTON:new()
    ADTrafficYieldModule.reset(o)
    o.cooldownUntil = 0
    return o
end

function ADTrafficYieldModule:reset()
    self.state = ADTrafficYieldModule.STATE_IDLE
    self.plan = nil
    self.partner = nil
    self.reverseTargets = nil
    self.reverseTargetIndex = 0
    self.reverseStart = nil
    self.escapeBranch = nil
    self.branchTargetIndex = 0
    self.reachedRouteIndex = nil
    self.pathIndex = nil
    self.reverseBlockedTimer = AutoDriveTON:new()
    self.stateTimer = AutoDriveTON:new()
    self.yielderStoppedTimer = AutoDriveTON:new()
    self.lastSearchTime = 0
    self.alignmentDone = nil
    self.alignmentTime = 0
    self.blockedTimer:timer(false)
end

------------------------------------------------------------------------------------------------
-- helpers
------------------------------------------------------------------------------------------------

function ADTrafficYieldModule.getName(vehicle)
    if vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.stateModule ~= nil then
        return vehicle.ad.stateModule:getName()
    end
    return "?"
end

-- side used when pulling over: the right side only (never against the traffic; without a free spot on that
-- side, the vehicle reverses or the other one pulls over instead).
-- Local +x is left in GIANTS, so right is negative x
function ADTrafficYieldModule.getSides()
    return {{name = "right", sign = -1}}
end

-- straight run along the roadside after the swerve, for the towed units to line up behind the tractor:
-- a trailer only converges onto the tractor's track after about twice its own length
function ADTrafficYieldModule.getRunOut(trailLength)
    if trailLength <= 0 then
        return 2
    end
    return trailLength * 2 + 4
end

function ADTrafficYieldModule.isEnabled()
    return AutoDrive.getSetting("trafficYieldEnabled") == true or AutoDrive.getSetting("trafficYieldEnabled") == 1
end

function ADTrafficYieldModule.getPosition(vehicle)
    return getWorldTranslation(vehicle.components[1].node)
end

function ADTrafficYieldModule.getDirection(vehicle)
    local dx, _, dz = AutoDrive.localDirectionToWorld(vehicle, 0, 0, 1)
    local length = MathUtil.vector2Length(dx, dz)
    if length < 0.0001 then
        return 0, 1
    end
    return dx / length, dz / length
end

function ADTrafficYieldModule.getSpeedKmh(vehicle)
    return math.abs(vehicle.lastSpeedReal or 0) * 3600
end

-- measures the real dimensions of every unit of the train in its current (transport) state, with the
-- AutoDrive dimension sensor; done once when a plan starts, the results are cached in vehicle.ad.adDimensions
function ADTrafficYieldModule.measureTrain(vehicle)
    local ok = pcall(function()
        AutoDrive.getAllVehicleDimensions(vehicle, true)
    end)
    if not ok then
        Logging.error("[AD] ADTrafficYieldModule: could not measure the dimensions of %s", ADTrafficYieldModule.getName(vehicle))
    end
end

-- real width / length of one unit: measured values when available, otherwise the vehicle xml values
function ADTrafficYieldModule.getUnitDimensions(unit)
    local width = (unit.size ~= nil and unit.size.width) or 3
    local length = (unit.size ~= nil and unit.size.length) or 5
    local dims = unit.ad ~= nil and unit.ad.adDimensions or nil
    if dims ~= nil then
        if dims.maxWidthLeft ~= nil and dims.maxWidthRight ~= nil and (dims.maxWidthLeft + dims.maxWidthRight) > 0.5 then
            width = dims.maxWidthLeft + dims.maxWidthRight
        elseif dims.width ~= nil and dims.width > 0.5 then
            width = dims.width
        end
        if dims.maxLengthFront ~= nil and dims.maxLengthBack ~= nil and (dims.maxLengthFront + dims.maxLengthBack) > 0.5 then
            length = dims.maxLengthFront + dims.maxLengthBack
        elseif dims.length ~= nil and dims.length > 0.5 then
            length = dims.length
        end
    end
    return width, length
end

-- widest unit of the train (tractor, front / rear implements, trailers)
function ADTrafficYieldModule.getWidth(vehicle)
    local maxWidth = ADTrafficYieldModule.getUnitDimensions(vehicle)
    local units, _ = AutoDrive.getAllUnits(vehicle)
    if units ~= nil then
        for _, unit in pairs(units) do
            local width = ADTrafficYieldModule.getUnitDimensions(unit)
            maxWidth = math.max(maxWidth, width)
        end
    end
    return math.min(maxWidth, 8)
end

-- distances from the vehicle origin to the front-most and rear-most points of the train
-- (front implements included, e.g. a front cultivator or a front weight)
function ADTrafficYieldModule.getTrainExtents(vehicle)
    local _, tractorLength = ADTrafficYieldModule.getUnitDimensions(vehicle)
    local front, rear = tractorLength / 2, tractorLength / 2
    local units, _ = AutoDrive.getAllUnits(vehicle)
    if units ~= nil then
        for _, unit in pairs(units) do
            if unit ~= vehicle and unit.components ~= nil then
                local _, unitLength = ADTrafficYieldModule.getUnitDimensions(unit)
                local ux, uy, uz = getWorldTranslation(unit.components[1].node)
                local _, _, localZ = AutoDrive.worldToLocal(vehicle, ux, uy, uz)
                front = math.max(front, localZ + unitLength / 2)
                rear = math.max(rear, -(localZ - unitLength / 2))
            end
        end
    end
    return front, rear
end

-- length of the whole train (sum of all units)
function ADTrafficYieldModule.getTotalLength(vehicle)
    local length = 0
    local units, count = AutoDrive.getAllUnits(vehicle)
    if units ~= nil and count ~= nil and count > 0 then
        for _, unit in pairs(units) do
            local _, unitLength = ADTrafficYieldModule.getUnitDimensions(unit)
            length = length + unitLength
        end
    end
    if length <= 0 then
        local _, tractorLength = ADTrafficYieldModule.getUnitDimensions(vehicle)
        length = tractorLength
    end
    return length
end

function ADTrafficYieldModule.hasTrailer(vehicle)
    local _, trailerCount = AutoDrive.getAllUnits(vehicle)
    return trailerCount ~= nil and trailerCount > 1
end

function ADTrafficYieldModule.isLoaded(vehicle)
    local trailers, _ = AutoDrive.getAllUnits(vehicle)
    if trailers == nil or #trailers == 0 then
        return false
    end
    local fillLevel, fillCapacity = AutoDrive.getAllFillLevels(trailers)
    if fillCapacity == nil or fillCapacity <= 0 then
        return false
    end
    return (fillLevel / fillCapacity) > 0.1
end

-- the lower the score, the more the vehicle should be the one reversing
function ADTrafficYieldModule.getPriorityScore(vehicle)
    local score = 0
    if ADTrafficYieldModule.isLoaded(vehicle) then
        score = score + 2
    end
    if ADTrafficYieldModule.hasTrailer(vehicle) then
        score = score + 1
    end
    return score
end

function ADTrafficYieldModule.isAdVehicleUsable(vehicle)
    return vehicle ~= nil and vehicle.ad ~= nil and vehicle.components ~= nil and vehicle.ad.stateModule ~= nil
        and vehicle.ad.stateModule:isActive() and vehicle.ad.trafficYieldModule ~= nil and vehicle.ad.drivePathModule ~= nil
end

function ADTrafficYieldModule:isControlling()
    if self.state == ADTrafficYieldModule.STATE_YIELDER_APPROACH then
        -- backing off from the mouth of a single track or holding before it: the module drives (otherwise the
        -- "stuck" safety of DrivePathModule resets the vehicle in the middle of its back-off)
        local plan = self.plan
        return plan ~= nil and (plan.sectionBackOff ~= nil or (plan.parked == true and not plan.straightOn))
    end
    return self.state ~= ADTrafficYieldModule.STATE_IDLE and self.state ~= ADTrafficYieldModule.STATE_REVERSER_PASS and self.state ~= ADTrafficYieldModule.STATE_PARKER_PASS
        and self.state ~= ADTrafficYieldModule.STATE_PRIORITY_APPROACH
end

function ADTrafficYieldModule:isBusy()
    return self.state ~= ADTrafficYieldModule.STATE_IDLE or g_time < self.cooldownUntil
end

function ADTrafficYieldModule:setState(newState)
    if newState ~= self.state then
        self:log("-> %s (partner %s)", ADTrafficYieldModule.STATE_NAMES[newState] or tostring(newState), ADTrafficYieldModule.getName(self.partner))
        local oldName = ADTrafficYieldModule.STATE_NAMES[self.state] or "?"
        self.state = newState
        self.stateTimer:timer(false)
    end
end

function ADTrafficYieldModule:ensureMotorStarted()
    if self.vehicle.startMotor ~= nil and not self.vehicle:getIsMotorStarted() and self.vehicle:getCanMotorRun() then
        self.vehicle:startMotor()
    end
end

function ADTrafficYieldModule:holdVehicle(dt)
    -- a planned hold, not a blocked vehicle: isBlocked would make the unloader mode think it is stuck
    -- after 10 s and reverse out of the road (CombineUnloaderMode:monitorTasks)
    self.vehicle.ad.specialDrivingModule:stopVehicle(false)
    self.vehicle.ad.specialDrivingModule:update(dt)
end

------------------------------------------------------------------------------------------------
-- entry point, called every frame from ADDrivePathModule:followWaypoints (server)
-- returns true when this module controls the vehicle for this frame
------------------------------------------------------------------------------------------------

function ADTrafficYieldModule:handle(dt, obstacleDetected)
    if not self.vehicle.isServer then
        return false
    end
    -- the approach module stops the vehicle a few metres behind an AD vehicle on its route, before the front sensors
    -- see it: that stop counts as "blocked by a vehicle" for the face to face detection
    if not obstacleDetected then
        local approach = self.vehicle.ad.approachModule
        if approach ~= nil and approach.shouldStop ~= nil and approach:shouldStop() then
            obstacleDetected = true
        end
    end

    -- parked / hidden vehicle waiting for a given vehicle to pass, no longer linked to a plan
    if self.state == ADTrafficYieldModule.STATE_WAIT_DETACHED then
        return self:updateDetachedWait(dt)
    end

    if self.state == ADTrafficYieldModule.STATE_IDLE then
        if not ADTrafficYieldModule.isEnabled() then
            self.blockedTimer:timer(false)
            return false
        end
        -- anticipation: oncoming AD vehicle on my route -> slow down and stop at a safe distance
        local holding = self:updateAnticipation(dt)
        if self.state == ADTrafficYieldModule.STATE_IDLE then
            if holding then
                return true
            end
            self:checkForHeadOnDeadlock(dt, obstacleDetected)
        end
        -- a plan may have just started
        if self.state == ADTrafficYieldModule.STATE_IDLE then
            return false
        end
    end

    if not self:checkPlanStillValid() then
        return false
    end

    if self.state == ADTrafficYieldModule.STATE_REVERSER_RETREAT then
        self:updateReverserRetreat(dt)
    elseif self.state == ADTrafficYieldModule.STATE_REVERSER_WAIT then
        self:updateReverserWait(dt)
    elseif self.state == ADTrafficYieldModule.STATE_REVERSER_PASS then
        self:updateReverserPass(dt)
    elseif self.state == ADTrafficYieldModule.STATE_YIELDER_APPROACH then
        -- holding at the stop point while still approaching: the module drives this frame
        if self:updateYielderApproach(dt) and self.state == ADTrafficYieldModule.STATE_YIELDER_APPROACH then
            return true
        end
    elseif self.state == ADTrafficYieldModule.STATE_PRIORITY_APPROACH then
        if self:updatePriorityApproach(dt) and self.state == ADTrafficYieldModule.STATE_PRIORITY_APPROACH then
            return true
        end
    elseif self.state == ADTrafficYieldModule.STATE_REVERSER_BRANCH then
        self:updateReverserBranch(dt)
    elseif self.state == ADTrafficYieldModule.STATE_REVERSER_HIDE then
        self:updateReverserHide(dt)
    elseif self.state == ADTrafficYieldModule.STATE_PARKER_PASS then
        self:updateParkerPass(dt)
    elseif self.state == ADTrafficYieldModule.STATE_PARKER_SEARCH then
        self:updateParkerSearch(dt)
    elseif self.state == ADTrafficYieldModule.STATE_PARKER_MOVE then
        self:updateParkerMove(dt)
    elseif self.state == ADTrafficYieldModule.STATE_PARKER_WAIT then
        self:updateParkerWait(dt)
    elseif self.state == ADTrafficYieldModule.STATE_SECTION_CLEAR then
        self:updateSectionClear(dt)
    end

    -- while driving inside a plan (passing / approaching), keep anticipating any OTHER oncoming vehicle
    if self:holdForOtherOncoming(dt) then
        return true
    end

    -- stopped face to face by a vehicle that the route comparison could not see
    if self:checkBlockedWhilePassing(dt, obstacleDetected) then
        return self:isControlling()
    end

    return self:isControlling()
end

-- a vehicle passing inside a plan is stopped by its sensor face to face with another AD vehicle that is
-- not part of the plan (e.g. started between both vehicles, already too close to be seen on the route):
-- hand over to it with a standard plan instead of waiting for the pass timeout
function ADTrafficYieldModule:checkBlockedWhilePassing(dt, obstacleDetected)
    local passing = self.state == ADTrafficYieldModule.STATE_REVERSER_PASS or self.state == ADTrafficYieldModule.STATE_PARKER_PASS
    local isStopped = ADTrafficYieldModule.getSpeedKmh(self.vehicle) < 1
    local delay = (AutoDrive.getSetting("trafficYieldDelay") or 5) * 1000
    if not self.blockedTimer:timer(passing and obstacleDetected and isStopped, delay, dt) then
        return false
    end
    if (g_updateLoopIndex % AutoDrive.PERF_FRAMES) ~= 0 then
        return false
    end
    local other = self:findHeadOnPartner(self.partner)
    if other == nil then
        return false
    end
    self:log("blocked face to face by %s while passing", ADTrafficYieldModule.getName(other))
    self:handOverTo(other, true)
    return true
end

-- a vehicle that drives during a plan (passing or approaching its partner) must still slow down and stop
-- for a third oncoming vehicle; returns true while held
function ADTrafficYieldModule:holdForOtherOncoming(dt)
    -- only once passing: during the approach both already brake for each other, and the partner must be
    -- allowed to finish pulling over before a third vehicle is dealt with (else it is left half parked)
    local drivingStates = {
        [ADTrafficYieldModule.STATE_REVERSER_PASS] = true,
        [ADTrafficYieldModule.STATE_PARKER_PASS] = true
    }
    if not drivingStates[self.state] then
        self.otherConflict = nil
        return false
    end
    if g_time >= (self.nextOtherCheck or 0) then
        self.nextOtherCheck = g_time + ADTrafficYieldModule.ANTICIPATION_INTERVAL
        self.otherConflict, self.otherGap = self:findOncomingConflict(self.partner)
        self.otherGapTime = g_time
        if self.otherConflict ~= nil then
            local myFront = ADTrafficYieldModule.getTrainExtents(self.vehicle)
            local otherFront = ADTrafficYieldModule.getTrainExtents(self.otherConflict)
            self.otherStopGap = myFront + otherFront + ADTrafficYieldModule.ANTICIPATION_GAP
        end
    end
    local other = self.otherConflict
    if other == nil or not ADTrafficYieldModule.isAdVehicleUsable(other) then
        self.otherHoldTimer = 0
        return false
    end
    local elapsed = (g_time - (self.otherGapTime or g_time)) / 1000
    local closing = (ADTrafficYieldModule.getSpeedKmh(self.vehicle) + ADTrafficYieldModule.getSpeedKmh(other)) / 3.6 * elapsed
    local stopDistance = (self.otherGap - closing - self.otherStopGap) / 2
    if stopDistance > 0.5 then
        local allowedSpeed = math.sqrt(2 * ADTrafficYieldModule.ANTICIPATION_DECEL * stopDistance) * 3.6
        local drivePathModule = self.vehicle.ad.drivePathModule
        if drivePathModule.speedLimit ~= nil then
            drivePathModule.speedLimit = math.min(drivePathModule.speedLimit, math.max(allowedSpeed, 3))
        end
        self.otherHoldTimer = 0
        return false
    end
    self:holdVehicle(dt)

    -- held for a third vehicle while still inside a plan: hand over, so a new plan can start with it
    self.otherHoldTimer = (self.otherHoldTimer or 0) + dt
    local otherModule = other.ad.trafficYieldModule
    if self.otherHoldTimer >= ADTrafficYieldModule.ANTICIPATION_START_DELAY and otherModule.state == ADTrafficYieldModule.STATE_IDLE
        and ADTrafficYieldModule.getSpeedKmh(self.vehicle) < 1 then
        self.otherHoldTimer = 0
        self:handOverTo(other)
    end
    return true
end

-- leaves the current plan to start a new one with another oncoming vehicle; the partner of the current
-- plan, if parked or hidden, keeps waiting on its own until this vehicle has passed it
function ADTrafficYieldModule:handOverTo(other, faceToFace)
    local partner = self.partner
    if partner ~= nil and partner.ad ~= nil and partner.ad.trafficYieldModule ~= nil then
        local partnerModule = partner.ad.trafficYieldModule
        if partnerModule.plan == self.plan then
            local waiting = partnerModule.state == ADTrafficYieldModule.STATE_PARKER_WAIT or partnerModule.state == ADTrafficYieldModule.STATE_PARKER_MOVE
                or partnerModule.state == ADTrafficYieldModule.STATE_REVERSER_HIDE or partnerModule.state == ADTrafficYieldModule.STATE_REVERSER_BRANCH
            local reachedRouteIndex = partnerModule.reachedRouteIndex
            partnerModule:reset()
            if waiting then
                local partnerSpot = self.plan ~= nil and self.plan.parkSpot or nil
                partnerModule.waitDirX = partnerSpot and partnerSpot.dirX
                partnerModule.waitDirZ = partnerSpot and partnerSpot.dirZ
                partnerModule.waitFor = self.vehicle
                partnerModule.waitSpot = partnerSpot
                partnerModule.waitRouteIndex = reachedRouteIndex
                partnerModule:setState(ADTrafficYieldModule.STATE_WAIT_DETACHED)
            else
                partnerModule.cooldownUntil = g_time + ADTrafficYieldModule.COOLDOWN_SUCCESS
                if partner.ad.specialDrivingModule ~= nil then
                    partner.ad.specialDrivingModule:releaseVehicle()
                end
                ADTrafficYieldModule.resyncWayPoints(partner, reachedRouteIndex)
            end
        end
    end
    self:log("hand over to %s", ADTrafficYieldModule.getName(other))
    local myRouteIndex = self.reachedRouteIndex
    self:reset()
    ADTrafficYieldModule.resyncWayPoints(self.vehicle, myRouteIndex)
    if faceToFace then
        -- already nose to nose: no room left to pull over in advance
        ADTrafficYieldModule.startPlan(self.vehicle, other)
    else
        ADTrafficYieldModule.startEarlyPlan(self.vehicle, other)
    end
end

function ADTrafficYieldModule:updateDetachedWait(dt)
    if not self:finishAlignment(dt, self.waitSpot, self.waitFor) then
        self:holdVehicle(dt)
    end
    local done = self:hasPassedMe(self.waitFor, self.waitDirX, self.waitDirZ)
    if done or self.stateTimer:timer(true, 180000, dt) then
        local routeIndex = self.waitRouteIndex
        self:reset()
        self.waitFor = nil
        self.waitSpot = nil
        self.waitRouteIndex = nil
        self.cooldownUntil = g_time + ADTrafficYieldModule.COOLDOWN_SUCCESS
        self.vehicle.ad.specialDrivingModule:releaseVehicle()
        ADTrafficYieldModule.resyncWayPoints(self.vehicle, routeIndex)
        return false
    end
    return true
end

------------------------------------------------------------------------------------------------
-- anticipation: compare the routes of the AD vehicles to see an oncoming one long before the sensors
------------------------------------------------------------------------------------------------

-- route ahead of each vehicle computed once per frame and shared by every vehicle that looks at it
-- (the tables returned are only read, never changed)
ADTrafficYieldModule.routeCache = setmetatable({}, {__mode = "k"})

-- graph ids of the next waypoints of a route with the distance along the route to each of them

function ADTrafficYieldModule.collectRouteAhead(vehicle, maxDistance)
    local wayPoints, currentIndex = vehicle.ad.drivePathModule:getWayPoints()
    local cached = ADTrafficYieldModule.routeCache[vehicle]
    if cached ~= nil and cached.loop == g_updateLoopIndex and cached.maxDistance == maxDistance
        and cached.wayPoints == wayPoints and cached.currentIndex == currentIndex then
        return cached.list, cached.distById
    end
    local list, distById = ADTrafficYieldModule.buildRouteAhead(vehicle, wayPoints, currentIndex, maxDistance)
    ADTrafficYieldModule.routeCache[vehicle] = {loop = g_updateLoopIndex, maxDistance = maxDistance, wayPoints = wayPoints,
                                                currentIndex = currentIndex, list = list, distById = distById}
    return list, distById
end

function ADTrafficYieldModule.buildRouteAhead(vehicle, wayPoints, currentIndex, maxDistance)
    local list, distById = {}, {}
    if wayPoints == nil or currentIndex == nil then
        return list, distById
    end
    local lastX, _, lastZ = ADTrafficYieldModule.getPosition(vehicle)
    local distance = 0
    for i = math.max(1, currentIndex), #wayPoints do
        local wp = wayPoints[i]
        if wp == nil then
            break
        end
        distance = distance + MathUtil.vector2Length(wp.x - lastX, wp.z - lastZ)
        lastX, lastZ = wp.x, wp.z
        if distance > maxDistance then
            break
        end
        if wp.id ~= nil and distById[wp.id] == nil then
            table.insert(list, wp.id)
            distById[wp.id] = distance
        end
    end
    return list, distById
end

-- returns the oncoming AD vehicle that will use a segment of my route in the opposite direction,
-- and the distance between both vehicles along the route
-- another AD vehicle drives ahead of me on my route, in my direction, before 'beforeDistance' metres:
-- I am not the head of the convoy, the leader deals with the oncoming vehicle first
ADTrafficYieldModule.ON_ROUTE_STATES = {
    [ADTrafficYieldModule.STATE_IDLE] = true,
    [ADTrafficYieldModule.STATE_YIELDER_APPROACH] = true,
    [ADTrafficYieldModule.STATE_PRIORITY_APPROACH] = true,
    [ADTrafficYieldModule.STATE_REVERSER_PASS] = true,
    [ADTrafficYieldModule.STATE_PARKER_PASS] = true
}

function ADTrafficYieldModule:hasLeaderAhead(myDist, beforeDistance, excluded)
    for _, other in pairs(AutoDrive.getAllVehicles()) do
        if other ~= self.vehicle and other ~= excluded and other.ad ~= self.vehicle.ad and ADTrafficYieldModule.isAdVehicleUsable(other)
            and ADTrafficYieldModule.ON_ROUTE_STATES[other.ad.trafficYieldModule.state] -- a parked vehicle is not in the way
            and not AutoDrive:checkIsConnected(self.vehicle, other) then
            local wayPoints, currentIndex = other.ad.drivePathModule:getWayPoints()
            if wayPoints ~= nil and currentIndex ~= nil and wayPoints[currentIndex] ~= nil and wayPoints[currentIndex + 1] ~= nil then
                local here, nextId = wayPoints[currentIndex].id, wayPoints[currentIndex + 1].id
                local dHere, dNext = myDist[here], myDist[nextId]
                if dHere ~= nil and dNext ~= nil and dNext > dHere and dHere < beforeDistance then
                    return true
                end
            end
        end
    end
    return false
end

-- another AD vehicle follows me closely on my route, in my direction (it would be in the way if I reversed)
function ADTrafficYieldModule.hasFollowerBehind(vehicle, maxDistance, excluded)
    local wayPoints, currentIndex = vehicle.ad.drivePathModule:getWayPoints()
    if wayPoints == nil or currentIndex == nil or wayPoints[currentIndex] == nil or wayPoints[currentIndex + 1] == nil then
        return false
    end
    local myId, myNextId = wayPoints[currentIndex].id, wayPoints[currentIndex + 1].id
    if myId == nil or myNextId == nil then
        return false
    end
    for _, other in pairs(AutoDrive.getAllVehicles()) do
        if other ~= vehicle and other ~= excluded and other.ad ~= vehicle.ad and ADTrafficYieldModule.isAdVehicleUsable(other)
            and not AutoDrive:checkIsConnected(vehicle, other) then
            local _, otherDist = ADTrafficYieldModule.collectRouteAhead(other, maxDistance)
            if otherDist[myId] ~= nil and otherDist[myNextId] ~= nil and otherDist[myNextId] > otherDist[myId] then
                return true
            end
        end
    end
    return false
end

-- head-on section between my route and the route of another vehicle: the consecutive segments I drive a -> b
-- that it drives b -> a (a two-way road, the other one really drives that same segment backwards).
-- Returns the distance along my route to the start of the section and along its route to the same point,
-- nil when there is no such section or when one of them leaves it before the other one enters it
-- (the first one simply goes first, nobody has to give way)
function ADTrafficYieldModule.findHeadOnSection(vehicle, myList, myDist, other, otherList, otherDist)
    local otherNext = {}
    for k = 1, #otherList - 1 do
        otherNext[otherList[k]] = otherList[k + 1]
    end
    for i = 1, #myList - 1 do
        local a, b = myList[i], myList[i + 1]
        if otherNext[b] == a then
            -- extend the section as long as the other one drives my next segments backwards
            local j = i + 1
            while j < #myList and otherNext[myList[j + 1]] == myList[j] do
                j = j + 1
            end
            local last = myList[j]
            local myFront, myRear = ADTrafficYieldModule.getTrainExtents(vehicle)
            local otherFront, otherRear = ADTrafficYieldModule.getTrainExtents(other)
            local margin = ADTrafficYieldModule.ANTICIPATION_GAP
            -- travel times at the current speeds (at least 2 m/s, a vehicle held for the other one restarts)
            local mySpeed = math.max(ADTrafficYieldModule.getSpeedKmh(vehicle) / 3.6, 2)
            local otherSpeed = math.max(ADTrafficYieldModule.getSpeedKmh(other) / 3.6, 2)
            -- the other one is out of the section (its rear past a) before my front reaches a
            if (otherDist[a] + otherRear + margin) / otherSpeed < (myDist[a] - myFront) / mySpeed then
                return nil, nil
            end
            -- I am out of the section (my rear past 'last') before its front reaches 'last'
            if (myDist[last] + myRear + margin) / mySpeed < (otherDist[last] - otherFront) / otherSpeed then
                return nil, nil
            end
            local section = {}
            for k = i, j do
                table.insert(section, myList[k])
            end
            return myDist[a], otherDist[a], section
        end
    end
    return nil, nil
end

function ADTrafficYieldModule:findOncomingConflict(excluded)
    if not self.vehicle.ad.drivePathModule:isOnRoadNetwork() then
        return nil, nil
    end
    local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
    local myList, myDist = ADTrafficYieldModule.collectRouteAhead(self.vehicle, ADTrafficYieldModule.ANTICIPATION_DISTANCE)
    if #myList < 2 then
        return nil, nil
    end

    local bestOther, bestGap = nil, math.huge
    for _, other in pairs(AutoDrive.getAllVehicles()) do
        if other ~= self.vehicle and other ~= excluded and other.ad ~= self.vehicle.ad and ADTrafficYieldModule.isAdVehicleUsable(other)
            and other.ad.drivePathModule:isOnRoadNetwork() and not AutoDrive:checkIsConnected(self.vehicle, other) then
            local ox, _, oz = ADTrafficYieldModule.getPosition(other)
            if MathUtil.vector2Length(ox - x, oz - z) < 2 * ADTrafficYieldModule.ANTICIPATION_DISTANCE then
                local otherList, otherDist = ADTrafficYieldModule.collectRouteAhead(other, ADTrafficYieldModule.ANTICIPATION_DISTANCE)
                local mine, da = ADTrafficYieldModule.findHeadOnSection(self.vehicle, myList, myDist, other, otherList, otherDist)
                if mine ~= nil then
                    local gap = mine + da
                    -- only the heads of the convoys: nobody of mine in front of me, nobody of its own in front of it
                    if gap < bestGap and not self:hasLeaderAhead(myDist, mine, other)
                        and not other.ad.trafficYieldModule:hasLeaderAhead(otherDist, da, self.vehicle) then
                        bestGap = gap
                        bestOther = other
                    end
                end
            end
        end
    end
    if bestOther == nil then
        return nil, nil
    end
    return bestOther, bestGap
end

-- slows the vehicle down so that both oncoming vehicles stop at a safe distance, then starts the
-- passing manoeuvre. Returns true while the vehicle is held at the stop point.
-- the crossing has priority over the face to face (otherwise a vehicle may switch from one to the other): the
-- crossing has priority: a vehicle committed on a red section (or backing out of it) either waits or goes, no face to face
-- plan starts for it or with it until it is through
function ADTrafficYieldModule.intersectionHasPriority(vehicle)
    local c = vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.intersection or nil
    return c ~= nil and (c.committed ~= nil or c.retreat ~= nil)
end

function ADTrafficYieldModule:updateAnticipation(dt)
    if g_time >= (self.nextAnticipationCheck or 0) then
        -- every vehicle checks in the same frame, so each route ahead is computed once and shared (routeCache)
        self.nextAnticipationCheck = (math.floor(g_time / ADTrafficYieldModule.ANTICIPATION_INTERVAL) + 1) * ADTrafficYieldModule.ANTICIPATION_INTERVAL
        self.conflictPartner, self.conflictGap = self:findOncomingConflict()
        if self.conflictPartner ~= nil then
            local myFront = ADTrafficYieldModule.getTrainExtents(self.vehicle)
            local otherFront = ADTrafficYieldModule.getTrainExtents(self.conflictPartner)
            self.conflictStopGap = myFront + otherFront + ADTrafficYieldModule.ANTICIPATION_GAP
            self.conflictGapTime = g_time
        end
    end

    local partner = self.conflictPartner
    if partner == nil or not ADTrafficYieldModule.isAdVehicleUsable(partner) then
        self.anticipationTimer = nil
        return false
    end

    -- the crossing first: no plan while one of us is engaged on a red section
    if ADTrafficYieldModule.intersectionHasPriority(self.vehicle) or ADTrafficYieldModule.intersectionHasPriority(partner) then
        self.anticipationTimer = nil
        return false
    end
    -- first choice: one of them pulls over in advance on its own route (early yield)
    local partnerModuleEarly = partner.ad.trafficYieldModule
    if not self:isBusy() and not partnerModuleEarly:isBusy() then
        -- better still: another way to my destination that avoids the head-on section, if the detour is short.
        -- Both vehicles try, the one with an alternative lane takes it
        if self:tryDetour(partner) or partnerModuleEarly:tryDetour(self.vehicle) then
            self.conflictPartner = nil
            return false
        end
        local history = ADTrafficYieldModule.pairHistory[ADTrafficYieldModule.getPairKey(self.vehicle, partner)]
        local earlyRecentlyFailed = history ~= nil and history.earlyTried ~= nil and g_time - history.earlyTried < 60000
        if not earlyRecentlyFailed then
            ADTrafficYieldModule.startEarlyPlan(self.vehicle, partner)
            self.conflictPartner = nil
            return false
        end
    end

    -- distance driven by both since the last route comparison (both approach each other)
    local elapsed = (g_time - (self.conflictGapTime or g_time)) / 1000
    local closing = (ADTrafficYieldModule.getSpeedKmh(self.vehicle) + ADTrafficYieldModule.getSpeedKmh(partner)) / 3.6 * elapsed
    local gap = self.conflictGap - closing

    -- each vehicle brakes for its half of the remaining free distance
    local stopDistance = (gap - self.conflictStopGap) / 2
    if stopDistance > 0.5 then
        local allowedSpeed = math.sqrt(2 * ADTrafficYieldModule.ANTICIPATION_DECEL * stopDistance) * 3.6
        local drivePathModule = self.vehicle.ad.drivePathModule
        if drivePathModule.speedLimit ~= nil then
            drivePathModule.speedLimit = math.min(drivePathModule.speedLimit, math.max(allowedSpeed, 3))
        end
        self.anticipationTimer = nil
        return false
    end

    -- at the stop point: hold, and start the manoeuvre once both are stopped
    self:holdVehicle(dt)
    local partnerModule = partner.ad.trafficYieldModule
    local bothStopped = ADTrafficYieldModule.getSpeedKmh(self.vehicle) < 1 and ADTrafficYieldModule.getSpeedKmh(partner) < 1
    if bothStopped and not self:isBusy() and not partnerModule:isBusy() then
        self.anticipationTimer = (self.anticipationTimer or 0) + dt
        if self.anticipationTimer >= ADTrafficYieldModule.ANTICIPATION_START_DELAY then
            self.anticipationTimer = nil
            self.conflictPartner = nil
            ADTrafficYieldModule.startPlan(self.vehicle, partner)
        end
    else
        self.anticipationTimer = nil
    end
    return true
end

------------------------------------------------------------------------------------------------
-- early yield: as soon as an oncoming vehicle is seen on the route, the yielding vehicle looks for a
-- roadside spot ahead on its own route and pulls over there in advance; the other one does not stop
------------------------------------------------------------------------------------------------

-- length of a list of waypoints from index 'from'
function ADTrafficYieldModule.getPathLength(points, from)
    local length = 0
    for k = math.max(from, 1), #points - 1 do
        length = length + MathUtil.vector2Length(points[k + 1].x - points[k].x, points[k + 1].z - points[k].z)
    end
    return length
end

-- looks for another way to my destination that avoids the head-on section with 'partner'; when one exists and
-- is at most DETOUR_MAX longer, my route is changed to it and the passing is not needed. Tried once per partner
function ADTrafficYieldModule:tryDetour(partner)
    if self.detourPartner == partner and g_time < (self.detourRetryTime or 0) then
        return false
    end
    self.detourPartner = partner
    self.detourRetryTime = g_time + 20000
    local drivePathModule = self.vehicle.ad.drivePathModule
    local wayPoints, currentIndex = drivePathModule:getWayPoints()
    if wayPoints == nil or currentIndex == nil or wayPoints[currentIndex] == nil or wayPoints[currentIndex + 1] == nil then
        return false
    end
    local startWp, nextWp, lastWp = wayPoints[currentIndex], wayPoints[currentIndex + 1], wayPoints[#wayPoints]
    if startWp.id == nil or nextWp.id == nil or lastWp.id == nil or ADPathCalculator == nil then
        return false
    end
    local myList, myDist = ADTrafficYieldModule.collectRouteAhead(self.vehicle, ADTrafficYieldModule.ANTICIPATION_DISTANCE)
    local otherList, otherDist = ADTrafficYieldModule.collectRouteAhead(partner, ADTrafficYieldModule.ANTICIPATION_DISTANCE)
    local mine, _, section = ADTrafficYieldModule.findHeadOnSection(self.vehicle, myList, myDist, partner, otherList, otherDist)
    if mine == nil then
        return false
    end
    if mine < ADTrafficYieldModule.DETOUR_MIN_AHEAD then
        self:log("no other way to avoid %s: the shared section starts %.0f m ahead", ADTrafficYieldModule.getName(partner), mine)
        return false
    end
    local avoid = {}
    for _, id in ipairs(section) do
        avoid[id] = true
    end
    if avoid[startWp.id] then
        self:log("no other way to avoid %s: I am already on the shared section", ADTrafficYieldModule.getName(partner))
        return false
    end

    -- AutoDrive route calculation with the head-on section made very expensive
    local originalWeights = ADPathCalculator.getDetourWeights
    ADPathCalculator.getDetourWeights = function(calculator)
        local weights = originalWeights(calculator)
        for id in pairs(avoid) do
            weights[id] = (weights[id] or 0) + ADTrafficYieldModule.DETOUR_PENALTY
        end
        return weights
    end
    local ok, path = pcall(ADGraphManager.pathFromTo, ADGraphManager, startWp.id, lastWp.id, {nextWp.id})
    ADPathCalculator.getDetourWeights = originalWeights
    if not ok or path == nil or #path < 2 or path[1].id ~= startWp.id or path[#path].id ~= lastWp.id then
        self:log("no other way to avoid %s: no route found", ADTrafficYieldModule.getName(partner))
        return false
    end
    for _, wp in ipairs(path) do
        if avoid[wp.id] then
            self:log("no other way to avoid %s", ADTrafficYieldModule.getName(partner))
            return false
        end
    end
    local detour = ADTrafficYieldModule.getPathLength(path, 1) - ADTrafficYieldModule.getPathLength(wayPoints, currentIndex)
    if detour > ADTrafficYieldModule.DETOUR_MAX then
        self:log("other way to avoid %s is %.0f m longer, too long", ADTrafficYieldModule.getName(partner), detour)
        return false
    end

    -- keep the waypoints already driven, then the new way from the current waypoint on
    local newWayPoints = {}
    for k = 1, currentIndex - 1 do
        newWayPoints[k] = wayPoints[k]
    end
    for _, wp in ipairs(path) do
        table.insert(newWayPoints, wp)
    end
    drivePathModule.wayPoints = newWayPoints
    drivePathModule:setCurrentWayPointIndex(currentIndex)
    self:log("detour of %.0f m to avoid meeting %s", math.max(detour, 0), ADTrafficYieldModule.getName(partner))
    return true
end

-- distance between me and a given vehicle along both routes when it comes towards me, nil otherwise
function ADTrafficYieldModule:getOncomingGap(other)
    if not ADTrafficYieldModule.isAdVehicleUsable(other) then
        return nil
    end
    local myList, myDist = ADTrafficYieldModule.collectRouteAhead(self.vehicle, ADTrafficYieldModule.ANTICIPATION_DISTANCE)
    local otherList, otherDist = ADTrafficYieldModule.collectRouteAhead(other, ADTrafficYieldModule.ANTICIPATION_DISTANCE)
    local mine, da, section = ADTrafficYieldModule.findHeadOnSection(self.vehicle, myList, myDist, other, otherList, otherDist)
    if mine == nil then
        return nil
    end
    return mine + da, mine, section
end

-- speed limit / hold to stop before the oncoming partner (same rule as the anticipation)
-- returns true while held at the stop point
-- share: part of the free distance this vehicle may drive (the other one drives the rest)
function ADTrafficYieldModule:limitSpeedTowards(dt, partner, share)
    share = share or 0.5
    if g_time >= (self.nextGapCheck or 0) then
        self.nextGapCheck = g_time + ADTrafficYieldModule.ANTICIPATION_INTERVAL
        self.earlyGap, self.earlySectionMine, self.earlySection = self:getOncomingGap(partner)
        self.earlyGapTime = g_time
        local myFront = ADTrafficYieldModule.getTrainExtents(self.vehicle)
        local otherFront = ADTrafficYieldModule.getTrainExtents(partner)
        self.earlyStopGap = myFront + otherFront + ADTrafficYieldModule.ANTICIPATION_GAP
    end
    if self.earlyGap == nil then
        return false, nil
    end
    local elapsed = (g_time - (self.earlyGapTime or g_time)) / 1000
    local closing = (ADTrafficYieldModule.getSpeedKmh(self.vehicle) + ADTrafficYieldModule.getSpeedKmh(partner)) / 3.6 * elapsed
    local stopDistance = (self.earlyGap - closing - self.earlyStopGap) * share
    if stopDistance > 0.5 then
        local allowedSpeed = math.sqrt(2 * ADTrafficYieldModule.ANTICIPATION_DECEL * stopDistance) * 3.6
        local drivePathModule = self.vehicle.ad.drivePathModule
        if drivePathModule.speedLimit ~= nil then
            drivePathModule.speedLimit = math.min(drivePathModule.speedLimit, math.max(allowedSpeed, 3))
        end
        return false, stopDistance
    end
    self:holdVehicle(dt)
    return true, stopDistance
end

function ADTrafficYieldModule.startEarlyPlan(vehicleA, vehicleB)
    local scoreA = ADTrafficYieldModule.getPriorityScore(vehicleA)
    local scoreB = ADTrafficYieldModule.getPriorityScore(vehicleB)
    local yielder, priority
    if scoreA < scoreB then
        yielder, priority = vehicleA, vehicleB
    elseif scoreB < scoreA then
        yielder, priority = vehicleB, vehicleA
    elseif (vehicleA.id or 0) < (vehicleB.id or 0) then
        yielder, priority = vehicleA, vehicleB
    else
        yielder, priority = vehicleB, vehicleA
    end

    ADTrafficYieldModule.measureTrain(yielder)
    ADTrafficYieldModule.measureTrain(priority)

    local plan = {
        mode = "early",
        pairKey = ADTrafficYieldModule.getPairKey(vehicleA, vehicleB),
        firstReverser = priority,
        reverser = priority, -- the vehicle that drives on (same role as the reverser once the other is parked)
        parker = yielder,
        startTime = g_time,
        parkSpot = nil,
        reverserMayStop = true,
        reverserAtLimit = false,
        parked = false,
        passed = false,
        hidden = false
    }
    local yModule = yielder.ad.trafficYieldModule
    local pModule = priority.ad.trafficYieldModule
    yModule:log("EARLY plan: I pull over for %s", ADTrafficYieldModule.getName(priority))
    yModule:reset()
    pModule:reset()
    yModule.plan, yModule.partner = plan, priority
    pModule.plan, pModule.partner = plan, yielder
    yModule:setState(ADTrafficYieldModule.STATE_YIELDER_APPROACH)
    pModule:setState(ADTrafficYieldModule.STATE_PRIORITY_APPROACH)
end

-- builds a path parallel to the route, starting at route index 'startIndex' (point sx, sz on the route):
-- the lateral offset grows smoothly from 0 to 'lateral' over 'swerve' metres, then stays constant.
-- One point every PATH_STEP metres, 'length' metres long. Each point: x, z, dirX, dirZ (road direction), s, offset
function ADTrafficYieldModule.buildOffsetPath(wayPoints, startIndex, lateral, swerve, length)
    local step = ADTrafficYieldModule.PATH_STEP
    local points = {}
    local s = 0
    local index = startIndex
    local ax, az = wayPoints[index].x, wayPoints[index].z
    local nextS = 0
    while index < #wayPoints and s <= length do
        local b = wayPoints[index + 1]
        local segLength = MathUtil.vector2Length(b.x - ax, b.z - az)
        if segLength > 0.01 then
            local dx, dz = (b.x - ax) / segLength, (b.z - az) / segLength
            while nextS <= s + segLength and nextS <= length do
                local t = nextS - s
                local px, pz = ax + dx * t, az + dz * t
                local ramp = math.min(1, nextS / swerve)
                ramp = ramp * ramp * (3 - 2 * ramp) -- smoothstep: soft leaving of the road
                local offset = lateral * ramp
                -- GIANTS local +x (left) of the road direction is (dz, -dx)
                table.insert(points, {x = px + dz * offset, z = pz - dx * offset, dirX = dx, dirZ = dz, s = nextS, offset = offset})
                nextS = nextS + step
            end
            s = s + segLength
        end
        ax, az = b.x, b.z
        index = index + 1
    end
    if #points < 2 or points[#points].s < length - step then
        return nil -- route too short
    end
    return points
end

-- checks a parallel path: terrain, water, field and obstacles along it (the vehicle body width)
-- returns free, onField
function ADTrafficYieldModule:isPathFree(points, width, roadY, py)
    local halfW = width / 2 + 0.2
    local onField = false
    -- terrain / water / field on the points first (cheap)
    for _, p in ipairs(points) do
        local h = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, p.x, py, p.z)
        if math.abs(h - roadY) > ADTrafficYieldModule.MAX_ROAD_HEIGHT_DIFF then
            return false, false
        end
        -- lateral slope of the strip
        local lx, lz = p.x + p.dirZ * halfW, p.z - p.dirX * halfW
        local rx, rz = p.x - p.dirZ * halfW, p.z + p.dirX * halfW
        local hl = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, lx, py, lz)
        local hr = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, rx, py, rz)
        if math.abs(hl - hr) > ADTrafficYieldModule.MAX_SLOPE_DIFF then
            return false, false
        end
        if g_currentMission.environmentAreaSystem ~= nil and g_currentMission.environmentAreaSystem.getWaterYAtWorldPosition ~= nil then
            local waterY = g_currentMission.environmentAreaSystem:getWaterYAtWorldPosition(p.x, h, p.z)
            if waterY ~= nil and waterY > h - 0.2 then
                return false, false
            end
        end
        if p.offset ~= 0 and AutoDrive.checkIsOnField(p.x, h, p.z) then
            onField = true
        end
    end
    -- obstacles: one box per path segment, oriented along it
    for k = 1, #points - 1 do
        local a, b = points[k], points[k + 1]
        if math.abs(b.offset) > halfW then -- the part still on the road is the normal lane
            local dx, dz = b.x - a.x, b.z - a.z
            local segLength = math.max(0.1, MathUtil.vector2Length(dx, dz))
            local mx, mz = (a.x + b.x) / 2, (a.z + b.z) / 2
            local h = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, mx, py, mz)
            if self:overlapHit(mx, math.max(h, roadY) + 1.9, mz, math.atan2(dx, dz), halfW, 1.3, segLength / 2 + 0.2) then
                return false, onField
            end
        end
    end
    return true, onField
end

-- the whole stretch on the driving side (see getSides)
function ADTrafficYieldModule:searchSpotBySides(maxAhead, keepClearOfPartner)
    for _, side in ipairs(ADTrafficYieldModule.getSides()) do
        local spot = self:searchSpotAlongRoute(maxAhead, side.name, keepClearOfPartner)
        if spot ~= nil then
            return spot
        end
    end
    return nil
end

-- searches, along my route ahead, a point where I can leave the road and follow a path parallel to the
-- route on the roadside, before reaching maxAhead metres. sideFilter: "right" / "left" / nil
-- keepClearOfPartner: the partner is still driving towards me (early plan), the end of the path must stay
-- out of its way, braking distance included (otherwise I would stop half way with my trailer on the road)
function ADTrafficYieldModule:searchSpotAlongRoute(maxAhead, sideFilter, keepClearOfPartner)
    local wayPoints, currentIndex = self.vehicle.ad.drivePathModule:getWayPoints()
    if wayPoints == nil or currentIndex == nil then
        return nil
    end
    local parker = self.vehicle
    local reverser = self.partner
    local parkerWidth = ADTrafficYieldModule.getWidth(parker)
    local reverserWidth = ADTrafficYieldModule.getWidth(reverser)
    local _, tractorLength = ADTrafficYieldModule.getUnitDimensions(parker)
    local parkerFront, parkerRear = ADTrafficYieldModule.getTrainExtents(parker)
    local margin = AutoDrive.getSetting("trafficYieldMargin") or ADTrafficYieldModule.LATERAL_MARGIN
    local baseLateral = (parkerWidth + reverserWidth) / 2 + margin
    local trailLength = math.max(0, parkerRear - tractorLength / 2)
    local runOut = ADTrafficYieldModule.getRunOut(trailLength)
    local swerve = math.max(8, tractorLength * 1.5, baseLateral * 2.5, trailLength)
    -- the tractor stops at swerve + runOut; the path continues a bit for the alignment and the aim point
    local pathLength = swerve + runOut + ADTrafficYieldModule.PATH_EXTRA
    local needed = swerve + runOut + parkerFront
    local laterals = {baseLateral, baseLateral + 1.0}
    local sides = ADTrafficYieldModule.getSides()

    local lastX, py, lastZ = ADTrafficYieldModule.getPosition(parker)
    local roadY = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, lastX, py, lastZ)

    local partnerX, partnerZ, partnerClearance
    if keepClearOfPartner and reverser ~= nil then
        local px, _, pz = ADTrafficYieldModule.getPosition(reverser)
        local partnerFront = ADTrafficYieldModule.getTrainExtents(reverser)
        local v = ADTrafficYieldModule.getSpeedKmh(reverser) / 3.6
        partnerX, partnerZ = px, pz
        partnerClearance = partnerFront + ADTrafficYieldModule.SPOT_CLEARANCE + v * v / (2 * ADTrafficYieldModule.ANTICIPATION_DECEL)
    end

    for _, side in ipairs(sides) do
        if sideFilter == nil or side.name == sideFilter then
            local fieldCandidate = nil
            local distance = 0
            local lastCheckedDistance = -math.huge
            lastX, _, lastZ = ADTrafficYieldModule.getPosition(parker)
            for i = currentIndex, #wayPoints - 1 do
                local wp = wayPoints[i]
                if wp == nil then
                    break
                end
                distance = distance + MathUtil.vector2Length(wp.x - lastX, wp.z - lastZ)
                lastX, lastZ = wp.x, wp.z
                if distance + needed > maxAhead then
                    break
                end
                if distance >= 5 and distance - lastCheckedDistance >= ADTrafficYieldModule.CANDIDATE_STEP then
                    lastCheckedDistance = distance
                    for _, lateral in ipairs(laterals) do
                        local lat = side.sign * lateral
                        local path = ADTrafficYieldModule.buildOffsetPath(wayPoints, i, lat, swerve, pathLength)
                        local tooCloseToPartner = false
                        if path ~= nil and partnerX ~= nil then
                            local last = path[#path]
                            tooCloseToPartner = MathUtil.vector2Length(last.x - partnerX, last.z - partnerZ) < partnerClearance
                        end
                        if path ~= nil and not tooCloseToPartner then
                            local free, onField = self:isPathFree(path, parkerWidth, roadY, py)
                            if free then
                                local spot = ADTrafficYieldModule.makePathSpot(path, wp, distance, side.name, lat, swerve, runOut, onField, roadY)
                                if not onField then
                                    return spot
                                end
                                fieldCandidate = fieldCandidate or spot
                            end
                        end
                    end
                end
            end
            -- no grass spot on this side: a spot partly on a field of this side before trying the other side
            if fieldCandidate ~= nil then
                return fieldCandidate
            end
        end
    end
    return nil
end

function ADTrafficYieldModule.makePathSpot(path, wp, distance, sideName, lat, swerve, runOut, onField, y)
    local function pointAt(s)
        local best = path[#path]
        for _, p in ipairs(path) do
            if p.s >= s then
                best = p
                break
            end
        end
        return best
    end
    local entry = path[1]
    local swervePoint = pointAt(swerve)
    local finalPoint = pointAt(swerve + runOut)
    local last = path[#path]
    return {
        path = path,
        finalS = swerve + runOut,
        entryX = entry.x, entryZ = entry.z, entryDistance = distance,
        onField = onField,
        side = sideName,
        forward = swerve,
        lateral = lat,
        lateralAbs = math.abs(lat),
        x = swervePoint.x, y = y, z = swervePoint.z,
        finalX = finalPoint.x, finalZ = finalPoint.z,
        aimX = last.x, aimZ = last.z,
        originX = wp.x, originZ = wp.z,
        dirX = finalPoint.dirX, dirZ = finalPoint.dirZ
    }
end

-- index of the path point closest to a world position, searching from 'fromIndex'
function ADTrafficYieldModule.getClosestPathIndex(path, x, z, fromIndex)
    local bestIndex, bestDistance = fromIndex or 1, math.huge
    for k = math.max(1, (fromIndex or 1) - 2), #path do
        local d = MathUtil.vector2Length(path[k].x - x, path[k].z - z)
        if d < bestDistance then
            bestIndex, bestDistance = k, d
        end
    end
    return bestIndex, bestDistance
end

-- rear of the last towed unit lies on the parallel path (close to it and parallel)
function ADTrafficYieldModule:isRearOnPath(path)
    local units, count = AutoDrive.getAllUnits(self.vehicle)
    local unit = self.vehicle
    if units ~= nil and count ~= nil and count > 1 and units[count] ~= nil then
        unit = units[count]
    end
    local _, length = ADTrafficYieldModule.getUnitDimensions(unit)
    local rx, _, rz = localToWorld(unit.components[1].node, 0, 0, -length / 2)
    local index, distance = ADTrafficYieldModule.getClosestPathIndex(path, rx, rz, 1)
    if distance > 0.6 then
        return false
    end
    -- the rear must be on the constant offset part of the path, not on the ramp
    local p = path[index]
    if math.abs(p.offset) < math.abs(path[#path].offset) - 0.3 then
        return false
    end
    local ux, _, uz = localDirectionToWorld(unit.components[1].node, 0, 0, 1)
    local l = MathUtil.vector2Length(ux, uz)
    return l < 0.01 or (ux * p.dirX + uz * p.dirZ) / l > 0.985
end

-- follows the parallel path with a look-ahead point; returns true when the train is parked
function ADTrafficYieldModule:followParkPath(dt, spot)
    local path = spot.path
    local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
    self.pathIndex = ADTrafficYieldModule.getClosestPathIndex(path, x, z, self.pathIndex or 1)
    local current = path[self.pathIndex]

    local reachedFinal = current.s >= spot.finalS
    local atPathEnd = self.pathIndex >= #path - 1
    if (reachedFinal and self:isRearOnPath(path)) or atPathEnd then
        self.pathIndex = nil
        return true
    end

    -- look-ahead target on the path
    local lookAhead = 5 + ADTrafficYieldModule.getSpeedKmh(self.vehicle) / 3.6
    local target = path[#path]
    for k = self.pathIndex, #path do
        if MathUtil.vector2Length(path[k].x - x, path[k].z - z) >= lookAhead then
            target = path[k]
            break
        end
    end
    self.vehicle.ad.specialDrivingModule:releaseVehicle()
    self:ensureMotorStarted()
    self.vehicle.ad.specialDrivingModule:driveToPoint(dt, {x = target.x, y = spot.y, z = target.z}, ADTrafficYieldModule.PARK_SPEED, false, 0.5, ADTrafficYieldModule.PARK_SPEED)
    return false
end


-- the yielder is already in the mouth of the single track (closer than SECTION_WAIT_CLEAR): it reverses along its
-- own route until its front is SECTION_WAIT_CLEAR m before the start of the track, then waits there.
-- Returns true while the module drives the vehicle
function ADTrafficYieldModule:updateSectionBackOff(dt)
    local plan = self.plan
    local backOff = plan.sectionBackOff
    if self.reverseTargets == nil then
        self.reverseTargets = self:buildReverseTargets()
        self.reverseTargetIndex = 1
        local x, y, z = ADTrafficYieldModule.getPosition(self.vehicle)
        self.reverseStart = {x = x, y = y, z = z}
        -- the back-off follows my own route behind me, never a straight line into the unknown (a train that has just
        -- started its route would end in the grass)
        self.backOffLine = {{x = x, z = z}}
        for _, t in ipairs(self.reverseTargets) do
            if t.straight then
                break
            end
            table.insert(self.backOffLine, {x = t.x, z = t.z})
        end
    end
    local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
    local reversed = MathUtil.vector2Length(x - self.reverseStart.x, z - self.reverseStart.z)
    local blockedBehind = self.vehicle.ad.collisionDetectionModule:checkReverseCollision()
    local blockedTooLong = self.reverseBlockedTimer:timer(blockedBehind, 8000, dt)
    local target = self.reverseTargets[self.reverseTargetIndex]
    if target ~= nil and target.straight then
        target = nil -- end of my route behind me: no further
    end
    -- drifted off my route (trailer): stop here
    local offRoute = #self.backOffLine >= 2 and ADTrafficYieldModule.distanceToPolyline(self.backOffLine, x, z) > 3.5
    if offRoute and not backOff.offRouteLogged then
        backOff.offRouteLogged = true
        self:log("back-off left my route (%.1f m aside), I stop here", ADTrafficYieldModule.distanceToPolyline(self.backOffLine, x, z))
    end
    if #self.backOffLine < 2 and not backOff.noRouteLogged then
        backOff.noRouteLogged = true
        self:log("no route behind me to back off along")
    end
    backOff.time = backOff.time or g_time
    -- the way of the other one may run along my own route before its first network point
    -- (pathfinder lead-in, field entrance, same dirt track): the wait line is only good once my whole train is off its
    -- remaining route, else I keep reversing up to SECTION_BACKOFF_MAX
    if g_time >= (backOff.nextClearCheck or 0) then
        backOff.nextClearCheck = g_time + 500
        backOff.clear = not self:isOnPartnerOffNetworkWay()
    end
    local reached = reversed >= backOff.distance
    if reached and not backOff.clear and not backOff.extended and not backOff.extra and reversed < ADTrafficYieldModule.SECTION_BACKOFF_MAX and target ~= nil then
        backOff.extended = true
        self:log("backed off %.0f m but still on the way of %s, I keep reversing (%d m at most)", reversed, ADTrafficYieldModule.getName(self.partner), ADTrafficYieldModule.SECTION_BACKOFF_MAX)
    end
    local done = (reached and (backOff.clear or reversed >= ADTrafficYieldModule.SECTION_BACKOFF_MAX)) or target == nil or blockedTooLong or offRoute or g_time - backOff.time > 60000
    if done then
        plan.sectionBackOff = nil
        self.backOffLine = nil
        -- far enough: at the wait line (3 m), or, when the wait line is only the single track itself, at least 8 m
        -- before it, and off the way of the other one; still in the mouth, across the turn-off or on its way: the other
        -- one gives way instead
        local remaining = backOff.distance - reversed
        local farEnough = (remaining <= 3 or (backOff.where ~= "road" and remaining <= ADTrafficYieldModule.SECTION_WAIT_CLEAR - 8)) and backOff.clear == true
        if not farEnough and backOff.clear ~= true then
            self:log("back-off stopped after %.0f m, still on the way of %s", reversed, ADTrafficYieldModule.getName(self.partner))
        end
        if not farEnough then
            -- blocked behind (vehicle queue) before getting out of the mouth: classic manoeuvre, I park on the roadside
            self:log("back-off before the single track stopped after %.0f m of %.0f m", reversed, backOff.distance)
            -- a vehicle right behind me: I go first and the other one gives way, the vehicle in the single track
            -- backs out of my way, I drive in first
            if self:startSectionClear() then
                return true
            end
            local history = ADTrafficYieldModule.pairHistory[plan.pairKey] or {failures = 0, time = g_time}
            history.earlyTried = g_time
            ADTrafficYieldModule.pairHistory[plan.pairKey] = history
            self:endPlan("back-off blocked, fall back to the standard manoeuvre", nil)
            return false
        end
        -- off the shared way: wait here like a yielder that stopped before the single track
        plan.waitBeforeSection = true
        plan.parked = true
        self.vehicle.ad.specialDrivingModule:releaseVehicle()
        self:log("backed off %.0f m of %.0f m before the single track%s, %s may come out", reversed, backOff.distance,
            blockedTooLong and " (blocked behind)" or "", ADTrafficYieldModule.getName(self.partner))
        self:holdVehicle(dt)
        return true
    end
    -- stop first (it may still roll into the mouth), then reverse
    if not backOff.stopped then
        self:holdVehicle(dt)
        if ADTrafficYieldModule.getSpeedKmh(self.vehicle) < 1 then
            backOff.stopped = true
        end
        return true
    end
    if self:reverseTowards(dt, target) then
        if target.routeIndex ~= nil then
            self.reachedRouteIndex = target.routeIndex
        end
        self.reverseTargetIndex = self.reverseTargetIndex + 1
    end
    return true
end

-- the node where my route turns off (a node with two ways out) last before the single track, as a signed
-- distance along my route from my position (negative: already behind me, searched up to 40 m back), nil if none
function ADTrafficYieldModule:findTurnOffBeforeSection()
    local section = self.earlySection
    local wayPoints, currentIndex = self.vehicle.ad.drivePathModule:getWayPoints()
    if section == nil or section[1] == nil or wayPoints == nil or currentIndex == nil or ADGraphManager == nil then
        return nil
    end
    local function isBranch(wp)
        if wp == nil or wp.id == nil or wp.id == section[1] then
            return false
        end
        local node = ADGraphManager:getWayPointById(wp.id)
        return node ~= nil and node.out ~= nil and #node.out >= 2
    end
    local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
    -- ahead, up to the start of the single track
    local found, foundIndex = nil, nil
    local lastX, lastZ, distance = x, z, 0
    for i = math.max(1, currentIndex), #wayPoints do
        local wp = wayPoints[i]
        if wp == nil then
            break
        end
        distance = distance + MathUtil.vector2Length(wp.x - lastX, wp.z - lastZ)
        lastX, lastZ = wp.x, wp.z
        if wp.id == section[1] then
            break
        end
        if isBranch(wp) then
            found, foundIndex = distance, i
        end
        if distance > ADTrafficYieldModule.ANTICIPATION_DISTANCE then
            break
        end
    end
    if found ~= nil then
        return found, foundIndex
    end
    -- behind me (I am already in the turn)
    lastX, lastZ, distance = x, z, 0
    for i = currentIndex - 1, 1, -1 do
        local wp = wayPoints[i]
        if wp == nil then
            break
        end
        distance = distance + MathUtil.vector2Length(wp.x - lastX, wp.z - lastZ)
        lastX, lastZ = wp.x, wp.z
        if distance > 40 then
            break
        end
        if isBranch(wp) then
            return -distance, i
        end
    end
    return nil
end

-- rather than reversing, the yielder is at the turn-off into the single track and cannot wait before it: it drives straight on along the
-- other branch of the junction, AutoDrive brings it back to its destination afterwards. Returns true when the route
-- was changed
function ADTrafficYieldModule:tryStraightOn(turnOffIndex)
    local drivePathModule = self.vehicle.ad.drivePathModule
    local wayPoints, currentIndex = drivePathModule:getWayPoints()
    if turnOffIndex == nil or wayPoints == nil or currentIndex == nil or ADGraphManager == nil or self.partner == nil then
        return false
    end
    local junction, towardsTrack, before, lastWp = wayPoints[turnOffIndex], wayPoints[turnOffIndex + 1], wayPoints[turnOffIndex - 1], wayPoints[#wayPoints]
    if junction == nil or junction.id == nil or towardsTrack == nil or lastWp == nil or lastWp.id == nil then
        return false
    end
    -- the junction must still be ahead of my front axle (otherwise I am already turning in)
    local myFront = ADTrafficYieldModule.getTrainExtents(self.vehicle)
    local _, _, junctionLocalZ = AutoDrive.worldToLocal(self.vehicle, junction.x, junction.y, junction.z)
    if junctionLocalZ < myFront - 1 then
        return false
    end
    local node = ADGraphManager:getWayPointById(junction.id)
    if node == nil or node.out == nil then
        return false
    end
    local _, otherDist = ADTrafficYieldModule.collectRouteAhead(self.partner, ADTrafficYieldModule.ANTICIPATION_DISTANCE)
    -- incoming direction at the junction
    local inX, inZ
    if before ~= nil then
        local l = math.max(0.01, MathUtil.vector2Length(junction.x - before.x, junction.z - before.z))
        inX, inZ = (junction.x - before.x) / l, (junction.z - before.z) / l
    else
        inX, inZ = ADTrafficYieldModule.getDirection(self.vehicle)
    end
    -- the most straight-on branch that is neither the track nor the way I came from nor the way of the partner
    local best, bestDot = nil, -math.huge
    for _, outId in pairs(node.out) do
        local candidate = ADGraphManager:getWayPointById(outId)
        if candidate ~= nil and outId ~= towardsTrack.id and (before == nil or outId ~= before.id) and otherDist[outId] == nil then
            local l = math.max(0.01, MathUtil.vector2Length(candidate.x - junction.x, candidate.z - junction.z))
            local dot = ((candidate.x - junction.x) * inX + (candidate.z - junction.z) * inZ) / l
            if dot > bestDot then
                best, bestDot = candidate, dot
            end
        end
    end
    if best == nil or bestDot < 0.3 then
        return false
    end
    -- follow that branch for 40 m (single way out at each step), never on the way of the partner
    local chain = {best}
    local length = MathUtil.vector2Length(best.x - junction.x, best.z - junction.z)
    local current = best
    while length < 40 do
        local graphNode = ADGraphManager:getWayPointById(current.id)
        if graphNode == nil or graphNode.out == nil or #graphNode.out < 1 then
            break
        end
        local nextWp = ADGraphManager:getWayPointById(graphNode.out[1])
        if nextWp == nil or otherDist[nextWp.id] ~= nil or nextWp.id == junction.id then
            break
        end
        table.insert(chain, nextWp)
        length = length + MathUtil.vector2Length(nextWp.x - current.x, nextWp.z - current.z)
        current = nextWp
    end
    if length < 15 then
        return false
    end
    -- way back to my destination from the end of that stretch
    local ok, path = pcall(ADGraphManager.pathFromTo, ADGraphManager, current.id, lastWp.id, {})
    if not ok or path == nil or #path < 2 or path[1].id ~= current.id or path[#path].id ~= lastWp.id then
        self:log("straight on at the junction: no way back to my destination")
        return false
    end
    local newWayPoints = {}
    for k = 1, turnOffIndex do
        newWayPoints[k] = wayPoints[k]
    end
    for _, wp in ipairs(chain) do
        table.insert(newWayPoints, wp)
    end
    for k = 2, #path do
        table.insert(newWayPoints, path[k])
    end
    local extra = ADTrafficYieldModule.getPathLength(newWayPoints, currentIndex) - ADTrafficYieldModule.getPathLength(wayPoints, currentIndex)
    drivePathModule.wayPoints = newWayPoints
    drivePathModule:setCurrentWayPointIndex(currentIndex)
    drivePathModule.minDistanceToNextWp = math.huge
    self:log("at the turn-off into the single track: I drive straight on and come back later (%.0f m more), %s comes out first", math.max(extra, 0), ADTrafficYieldModule.getName(self.partner))
    return true
end

-- point of my route 'distance' metres ahead of my position (world x, z), nil when the route is shorter
function ADTrafficYieldModule:getRoutePointAhead(distance)
    local wayPoints, currentIndex = self.vehicle.ad.drivePathModule:getWayPoints()
    if wayPoints == nil or currentIndex == nil then
        return nil
    end
    local lastX, _, lastZ = ADTrafficYieldModule.getPosition(self.vehicle)
    local travelled = 0
    for i = math.max(1, currentIndex), #wayPoints do
        local wp = wayPoints[i]
        if wp == nil then
            break
        end
        local seg = MathUtil.vector2Length(wp.x - lastX, wp.z - lastZ)
        if travelled + seg >= distance and seg > 0.01 then
            local t = (distance - travelled) / seg
            return {x = lastX + (wp.x - lastX) * t, y = wp.y, z = lastZ + (wp.z - lastZ) * t}
        end
        travelled = travelled + seg
        lastX, lastZ = wp.x, wp.z
    end
    return nil
end

-- first node of the head-on section on my route (world position), nil when not found
function ADTrafficYieldModule:findSectionStart()
    local section = self.earlySection
    local wayPoints, currentIndex = self.vehicle.ad.drivePathModule:getWayPoints()
    if section == nil or section[1] == nil or wayPoints == nil or currentIndex == nil then
        return nil
    end
    for i = math.max(1, currentIndex), #wayPoints do
        local wp = wayPoints[i]
        if wp ~= nil and wp.id == section[1] then
            return {x = wp.x, z = wp.z}
        end
    end
    return nil
end

-- I already drive on the way of the partner (the two waypoints behind me are on its route): I am inside the
-- single track, not in its mouth, a short reverse would not get me out of it
function ADTrafficYieldModule:isInsideSection()
    local wayPoints, currentIndex = self.vehicle.ad.drivePathModule:getWayPoints()
    if wayPoints == nil or currentIndex == nil or self.partner == nil then
        return false
    end
    local _, otherDist = ADTrafficYieldModule.collectRouteAhead(self.partner, ADTrafficYieldModule.ANTICIPATION_DISTANCE)
    local behind1, behind2 = wayPoints[currentIndex - 1], wayPoints[currentIndex - 2]
    return behind1 ~= nil and behind2 ~= nil and behind1.id ~= nil and behind2.id ~= nil and otherDist[behind1.id] ~= nil and otherDist[behind2.id] ~= nil
end

-- the partner (the vehicle coming out of the single track) is clear of its entrance, trailer included
function ADTrafficYieldModule:isPartnerClearOfSection()
    local start = self.plan.sectionStart
    if start == nil or self.partner == nil then
        return true
    end
    local px, _, pz = ADTrafficYieldModule.getPosition(self.partner)
    local _, partnerRear = ADTrafficYieldModule.getTrainExtents(self.partner)
    -- past the entrance in its own direction of travel (still inside the track: the entrance is ahead of it)
    local dx, dz = ADTrafficYieldModule.getDirection(self.partner)
    return (px - start.x) * dx + (pz - start.z) * dz > partnerRear + 8
end

-- the part of the remaining route of the partner that is off the network (pathfinder lead-in: exit of a field, detour):
-- those points have no id, the route comparison cannot see them, the single track may begin before its first network
-- point and run over the place where I wait (e.g. a vehicle leaving its field). True when my train (front, centre, rear) is within passing distance of one of those segments.
-- (network lanes side by side are never compared here: two lanes of a road are closer than that and are fine)
-- extra: more room asked (m), e.g. before ending the plan (the sensors stop both earlier than the routes touch)
function ADTrafficYieldModule:isOnPartnerOffNetworkWay(extra)
    local partner = self.partner
    if partner == nil or partner.ad == nil or partner.ad.drivePathModule == nil then
        return false
    end
    local wayPoints, currentIndex = partner.ad.drivePathModule:getWayPoints()
    if wayPoints == nil or currentIndex == nil then
        return false
    end
    local px, _, pz = ADTrafficYieldModule.getPosition(partner)
    local points = {{x = px, z = pz, off = not partner.ad.drivePathModule:isOnRoadNetwork()}}
    local distance = 0
    for i = math.max(1, currentIndex), #wayPoints do
        local wp = wayPoints[i]
        if wp == nil then
            break
        end
        local last = points[#points]
        distance = distance + MathUtil.vector2Length(wp.x - last.x, wp.z - last.z)
        table.insert(points, {x = wp.x, z = wp.z, off = wp.id == nil})
        if distance > 150 then
            break
        end
    end
    local segments = {}
    for k = 1, #points - 1 do
        if points[k].off or points[k + 1].off then
            table.insert(segments, {points[k], points[k + 1]})
        end
    end
    if #segments == 0 then
        return false
    end
    local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
    local dirX, dirZ = ADTrafficYieldModule.getDirection(self.vehicle)
    local myFront, myRear = ADTrafficYieldModule.getTrainExtents(self.vehicle)
    local margin = AutoDrive.getSetting("trafficYieldMargin") or ADTrafficYieldModule.LATERAL_MARGIN
    local needed = (ADTrafficYieldModule.getWidth(self.vehicle) + ADTrafficYieldModule.getWidth(partner)) / 2 + margin + 1 + (extra or 0)
    for _, f in ipairs({myFront, 0, -myRear}) do
        local tx, tz = x + dirX * f, z + dirZ * f
        for _, seg in ipairs(segments) do
            if ADTrafficYieldModule.distanceToPolyline(seg, tx, tz) < needed then
                return true
            end
        end
    end
    return false
end

-- the partner came out and stopped right in front of me (its sensor sees me): I am on its way, whatever the routes say
function ADTrafficYieldModule:isPartnerStoppedInFront(dt)
    local partner = self.partner
    if partner == nil then
        return false
    end
    local px, py, pz = ADTrafficYieldModule.getPosition(partner)
    local localX, _, localZ = AutoDrive.worldToLocal(self.vehicle, px, py, pz)
    local myFront = ADTrafficYieldModule.getTrainExtents(self.vehicle)
    local partnerFront = ADTrafficYieldModule.getTrainExtents(partner)
    -- a cone ahead of me, not a corridor (a partner stopped at an angle in a bend is in front too)
    local gap = MathUtil.vector2Length(localX, localZ) - myFront - partnerFront
    local inFront = localZ > 0 and gap < 12 and math.abs(localX) < math.max(4, localZ * 0.7) and ADTrafficYieldModule.getSpeedKmh(partner) < 1
    self.partnerStoppedTimer = self.partnerStoppedTimer or AutoDriveTON:new()
    return self.partnerStoppedTimer:timer(inFront, 3000, dt)
end

-- the yielder waits on its road SECTION_WAIT_CLEAR m before the single track until the other one is out of it
function ADTrafficYieldModule:updateSectionWait(dt)
    local plan = self.plan
    local myFront = ADTrafficYieldModule.getTrainExtents(self.vehicle)
    local remaining
    if plan.waitPoint ~= nil then
        local _, _, localZ = AutoDrive.worldToLocal(self.vehicle, plan.waitPoint.x, plan.waitPoint.y or 0, plan.waitPoint.z)
        remaining = localZ - myFront
        -- (its way out may still run over my place after its last network point of the single track: wait until it is past)
        if self.earlyGap == nil and self:isPartnerClearOfSection() and not self:isOnPartnerOffNetworkWay(8) then
            self:endPlan("partner out of the single track", true)
            return false
        end
    elseif self.earlyGap ~= nil then
        local elapsed = (g_time - (self.earlyGapTime or g_time)) / 1000
        remaining = (self.earlySectionMine or 0) - myFront - ADTrafficYieldModule.SECTION_WAIT_CLEAR - ADTrafficYieldModule.getSpeedKmh(self.vehicle) / 3.6 * elapsed
    else
        local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
        remaining = MathUtil.vector2Length(plan.sectionStart.x - x, plan.sectionStart.z - z) - myFront - ADTrafficYieldModule.SECTION_WAIT_CLEAR
        if self:isPartnerClearOfSection() and not self:isOnPartnerOffNetworkWay(8) then
            self:endPlan("partner out of the single track", true)
            return false
        end
    end
    -- stopped short of the wait line by something else (sensor: the other one already at the mouth):
    -- I am waiting anyway, go on as if parked (the nose to nose rule below may back me off)
    self.waitStoppedTimer = self.waitStoppedTimer or AutoDriveTON:new()
    local stoppedShort = self.waitStoppedTimer:timer(not plan.parked and remaining > 0.5 and ADTrafficYieldModule.getSpeedKmh(self.vehicle) < 1, 3000, dt)
    if remaining > 0.5 and not plan.parked and not stoppedShort then
        local allowedSpeed = math.sqrt(2 * ADTrafficYieldModule.ANTICIPATION_DECEL * remaining) * 3.6
        local drivePathModule = self.vehicle.ad.drivePathModule
        if drivePathModule.speedLimit ~= nil then
            drivePathModule.speedLimit = math.min(drivePathModule.speedLimit, math.max(allowedSpeed, 3))
        end
        return false
    end
    -- stopped before the single track: off the shared way, the other one may drive through
    if not plan.parked then
        self:log("waiting before the single track%s (%.1f m to the wait point%s), %s may come out", stoppedShort and string.format(" (stopped %.0f m short of the line)", remaining) or "",
            remaining, plan.waitPoint ~= nil and "" or ", no wait point", ADTrafficYieldModule.getName(self.partner))
    end
    plan.parked = true
    -- the wait line comes from the network points, the other one may still have to drive over my place
    -- (pathfinder lead-in before its first network point): once it is close, check that my train is off its remaining route,
    -- else back off some more (once)
    do
        local reason = nil
        -- nose to nose: at most twice (it may be stopped by something else, a crossing, a third vehicle)
        if self:isPartnerStoppedInFront(dt) then
            if (plan.noseBackOffs or 0) < 2 then
                plan.noseBackOffs = (plan.noseBackOffs or 0) + 1
                reason = "stopped nose to nose with me"
            end
        elseif g_time >= (plan.nextWaitClearCheck or 0) then
            plan.nextWaitClearCheck = g_time + 1000
            local px, _, pz = ADTrafficYieldModule.getPosition(self.partner)
            local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
            if MathUtil.vector2Length(px - x, pz - z) < 60 and self:isOnPartnerOffNetworkWay() then
                reason = "its way out runs over my place"
            end
        end
        if reason ~= nil then
            plan.extraBackOff = (plan.extraBackOff or 0) + 10
            if plan.extraBackOff > ADTrafficYieldModule.SECTION_BACKOFF_MAX then
                -- reversing does not get me off its way: it backs out of the single track instead, or the standard manoeuvre
                self:log("still on the way of %s after %d m more: I give up backing off", ADTrafficYieldModule.getName(self.partner), plan.extraBackOff - 10)
                if self:startSectionClear() then
                    return true
                end
                local history = ADTrafficYieldModule.pairHistory[plan.pairKey] or {failures = 0, time = g_time}
                history.earlyTried = g_time
                ADTrafficYieldModule.pairHistory[plan.pairKey] = history
                self:endPlan("still on the way of the partner after backing off, fall back to the standard manoeuvre", nil)
                return false
            end
            plan.sectionBackOff = {distance = 10, where = plan.waitWhere or "track", extra = true}
            self.reverseTargets = nil
            self:log("%s %s: I back off 10 m more", ADTrafficYieldModule.getName(self.partner), reason)
            return self:updateSectionBackOff(dt)
        end
    end
    self:holdVehicle(dt)
    if self.stateTimer:timer(true, 120000, dt) then
        self:endPlan("wait before the single track: timeout", false)
    end
    return true
end

-- I am stuck in the mouth of the single track (vehicle queue behind me): roles swapped, the partner in the
-- single track reverses along its own route until it is off my way (or parks beside the track), I drive in first
function ADTrafficYieldModule:startSectionClear()
    local plan = self.plan
    local other = self.partner
    if other == nil or plan == nil or plan.sectionCleared then
        return false
    end
    if ADTrafficYieldModule.hasFollowerBehind(other, 20, self.vehicle) then
        return false
    end
    plan.sectionCleared = true
    plan.mode = "sectionClear"
    plan.reverser, plan.parker = self.vehicle, other -- I drive on (REVERSER_PASS), it gets off the way
    plan.parked = false
    plan.waitBeforeSection = nil
    plan.reverserMayStop = true
    self:log("back-off blocked behind me: %s backs out of the single track, I go first", ADTrafficYieldModule.getName(other))
    local otherModule = other.ad.trafficYieldModule
    self.vehicle.ad.specialDrivingModule:releaseVehicle()
    other.ad.specialDrivingModule:releaseVehicle()
    otherModule.reverseTargets = nil
    self:setState(ADTrafficYieldModule.STATE_REVERSER_WAIT)
    self:holdVehicle(0)
    otherModule:setState(ADTrafficYieldModule.STATE_SECTION_CLEAR)
    return true
end

-- the remaining route of a vehicle as a polyline (its position first), at most maxDistance metres
function ADTrafficYieldModule.getRoutePolyline(vehicle, maxDistance)
    local x, _, z = ADTrafficYieldModule.getPosition(vehicle)
    local points = {{x = x, z = z}}
    local wayPoints, currentIndex = vehicle.ad.drivePathModule:getWayPoints()
    if wayPoints == nil or currentIndex == nil then
        return points
    end
    local distance = 0
    for i = math.max(1, currentIndex), #wayPoints do
        local wp = wayPoints[i]
        if wp == nil then
            break
        end
        local last = points[#points]
        distance = distance + MathUtil.vector2Length(wp.x - last.x, wp.z - last.z)
        table.insert(points, {x = wp.x, z = wp.z})
        if distance > maxDistance then
            break
        end
    end
    return points
end

function ADTrafficYieldModule.distanceToPolyline(points, x, z)
    local best = math.huge
    for k = 1, #points - 1 do
        local a, b = points[k], points[k + 1]
        local dx, dz = b.x - a.x, b.z - a.z
        local l2 = dx * dx + dz * dz
        local t = 0
        if l2 > 0.0001 then
            t = math.max(0, math.min(1, ((x - a.x) * dx + (z - a.z) * dz) / l2))
        end
        best = math.min(best, MathUtil.vector2Length(x - (a.x + dx * t), z - (a.z + dz * t)))
    end
    if #points == 1 then
        best = MathUtil.vector2Length(x - points[1].x, z - points[1].z)
    end
    return best
end

-- my whole train (front, centre, rear) is far enough from the route the partner still has to drive
function ADTrafficYieldModule:isClearOfPartnerRoute()
    local partner = self.partner
    local route = ADTrafficYieldModule.getRoutePolyline(partner, ADTrafficYieldModule.SECTION_CLEAR_MAX + 100)
    local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
    local dirX, dirZ = ADTrafficYieldModule.getDirection(self.vehicle)
    local myFront, myRear = ADTrafficYieldModule.getTrainExtents(self.vehicle)
    local margin = AutoDrive.getSetting("trafficYieldMargin") or ADTrafficYieldModule.LATERAL_MARGIN
    local needed = (ADTrafficYieldModule.getWidth(self.vehicle) + ADTrafficYieldModule.getWidth(partner)) / 2 + margin + 1
    -- its route may end at the field entrance and go on into the field: keep 20 m more around that end
    local last = route[#route]
    for _, f in ipairs({myFront, 0, -myRear}) do
        local px, pz = x + dirX * f, z + dirZ * f
        if ADTrafficYieldModule.distanceToPolyline(route, px, pz) < needed or MathUtil.vector2Length(px - last.x, pz - last.z) < needed + 20 then
            return false
        end
    end
    return true
end

-- the vehicle in the single track backs out of the way of the one stuck in the mouth, then waits until it passed
function ADTrafficYieldModule:updateSectionClear(dt)
    local plan = self.plan
    if plan.parked then
        self:holdVehicle(dt)
        local partner = self.partner
        if self:hasPassedMe(partner) or not partner.ad.drivePathModule:isOnRoadNetwork() then
            plan.passed = true
            self:endPlan("the other one went through the single track", true)
            return
        end
        if self.stateTimer:timer(true, 120000, dt) then
            self:endPlan("single track clear: wait timeout", false)
        end
        return
    end
    if self.reverseTargets == nil then
        self.reverseTargets = self:buildReverseTargets(ADTrafficYieldModule.SECTION_CLEAR_MAX)
        self.reverseTargetIndex = 1
        local x, y, z = ADTrafficYieldModule.getPosition(self.vehicle)
        self.reverseStart = {x = x, y = y, z = z}
        self.clearStartTime = g_time
    end
    -- a free place beside the track (field): park there, the other one passes (roles of the standard plan)
    if self:tryOwnParkingSpot(dt) then
        return
    end
    local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
    local reversed = MathUtil.vector2Length(x - self.reverseStart.x, z - self.reverseStart.z)
    local blockedBehind = self.vehicle.ad.collisionDetectionModule:checkReverseCollision()
    local blockedTooLong = self.reverseBlockedTimer:timer(blockedBehind, 8000, dt)
    local target = self.reverseTargets[self.reverseTargetIndex]
    local clear = self:isClearOfPartnerRoute()
    if clear or target == nil or blockedTooLong or reversed >= ADTrafficYieldModule.SECTION_CLEAR_MAX or g_time - self.clearStartTime > 150000 then
        plan.parked = true
        self.vehicle.ad.specialDrivingModule:releaseVehicle()
        self:holdVehicle(dt)
        self:log("backed %.0f m out of the single track (%s), %s goes through", reversed, clear and "off its way" or (blockedTooLong and "blocked behind" or "limit"),
            ADTrafficYieldModule.getName(self.partner))
        return
    end
    if self:reverseTowards(dt, target) then
        if target.routeIndex ~= nil then
            self.reachedRouteIndex = target.routeIndex
        end
        self.reverseTargetIndex = self.reverseTargetIndex + 1
    end
end

function ADTrafficYieldModule:updateYielderApproach(dt)
    local plan = self.plan
    if plan.sectionBackOff ~= nil then
        return self:updateSectionBackOff(dt)
    end
    if plan.straightOn then
        -- driving away along the road: done once the other one is out of the single track (or no longer oncoming)
        self.vehicle.ad.specialDrivingModule:releaseVehicle()
        if self:isPartnerClearOfSection() then
            self:endPlan("partner out of the single track (straight on)", true)
        elseif self.stateTimer:timer(true, 120000, dt) then
            self:endPlan("straight on: timeout", true)
        end
        return false
    end
    -- the yielder gets most of the free distance to find a spot (the other one stops earlier)
    local holding, stopDistance = self:limitSpeedTowards(dt, self.partner, ADTrafficYieldModule.YIELDER_SHARE)
    if self.earlyGap == nil then
        -- waiting before the single track: the partner may still be in the mouth (its last node passed but its trailer
        -- not out yet): keep waiting until it is clear of the entrance
        if plan.waitBeforeSection and plan.sectionStart ~= nil then
            return self:updateSectionWait(dt)
        end
        -- not oncoming any more (route changed): nothing to do
        self:endPlan("no more oncoming", nil)
        return false
    end

    -- the single track starts ahead of me: I wait on my road SECTION_WAIT_CLEAR m before it, so that my nose
    -- does not block its mouth, the other one comes out first, then I go
    if plan.waitBeforeSection == nil and self.earlySectionMine ~= nil then
        local myFront, myRear = ADTrafficYieldModule.getTrainExtents(self.vehicle)
        plan.sectionStart = self:findSectionStart()
        -- where my front stops: SECTION_WAIT_CLEAR m before the single track. When the single track starts shortly
        -- after the turn-off, the train would stand across it with its trailer on the road: when my whole train cannot wait between the turn-off and the single track, I wait on my road before the turn-off
        local waitLine = self.earlySectionMine - ADTrafficYieldModule.SECTION_WAIT_CLEAR
        local turnOff, turnOffIndex = self:findTurnOffBeforeSection()
        local where = "track"
        if turnOff ~= nil and self.earlySectionMine - turnOff < myFront + myRear + ADTrafficYieldModule.SECTION_WAIT_CLEAR + 3 then
            local beforeTurnOff = turnOff - ADTrafficYieldModule.JUNCTION_GAP
            if beforeTurnOff < waitLine then
                waitLine = beforeTurnOff
                where = "road"
            end
        end
        plan.waitBeforeSection = waitLine - myFront > 2
        -- cannot wait before the turn-off any more: straight on along the road rather than reversing
        if not plan.waitBeforeSection and where == "road" and self:tryStraightOn(turnOffIndex) then
            plan.straightOn = true
            plan.waitBeforeSection = false
            plan.parked = true -- the other one drives on
            self.vehicle.ad.specialDrivingModule:releaseVehicle()
            return false
        end
        plan.waitWhere = where
        if plan.waitBeforeSection then
            plan.waitPoint = self:getRoutePointAhead(waitLine)
            self:log("single track %.0f m ahead (turn-off %s m): I wait with my front %.0f m ahead (%s), %s comes out first", self.earlySectionMine,
                turnOff ~= nil and string.format("%.0f", turnOff) or "-", waitLine, where, ADTrafficYieldModule.getName(self.partner))
        else
            -- already in the mouth (too close for the other one to come out): I back off along my route to the wait line
            local frontDistance = self.earlySectionMine - myFront
            local distance = myFront - waitLine
            if plan.sectionStart ~= nil and frontDistance > -ADTrafficYieldModule.SECTION_BACKOFF_INSIDE and distance <= ADTrafficYieldModule.SECTION_BACKOFF_MAX
                and not self:isInsideSection() then
                plan.sectionBackOff = {distance = math.max(distance, 3), where = where}
                self:log("single track %.0f m ahead (front at %.0f m): too close, I back off %.0f m, %s comes out first", self.earlySectionMine, frontDistance,
                    plan.sectionBackOff.distance, ADTrafficYieldModule.getName(self.partner))
                return self:updateSectionBackOff(dt)
            end
        end
    end
    if plan.waitBeforeSection and plan.parkSpot == nil then
        return self:updateSectionWait(dt)
    end

    if plan.parkSpot == nil then
        if g_time - self.lastSearchTime >= ADTrafficYieldModule.SEARCH_INTERVAL and stopDistance ~= nil then
            self.lastSearchTime = g_time
            -- the whole stretch on the right first, the left side only as a last resort
            plan.parkSpot = self:searchSpotBySides(stopDistance + 2, true)
            if plan.parkSpot ~= nil then
                self:log("early spot found: side=%s at %.0f m, field=%s", plan.parkSpot.side, plan.parkSpot.entryDistance, tostring(plan.parkSpot.onField))
            end
        end
        -- stopped by something else (sensor, crossing) without any spot ahead: waiting for the plan timeout
        -- would block both for 3 min
        local stoppedTooLong = self.yielderStoppedTimer:timer(ADTrafficYieldModule.getSpeedKmh(self.vehicle) < 1, ADTrafficYieldModule.YIELDER_STOPPED_FALLBACK, dt)
        if plan.parkSpot == nil and (holding or stoppedTooLong) then
            -- stopped without any spot ahead: classic manoeuvre (reverse + roadside / side branch)
            local history = ADTrafficYieldModule.pairHistory[plan.pairKey] or {failures = 0, time = g_time}
            history.earlyTried = g_time
            ADTrafficYieldModule.pairHistory[plan.pairKey] = history
            self:endPlan("no spot ahead, fall back to the standard manoeuvre", nil)
            return false
        end
        return holding
    end

    -- drive normally along the route until the leaving point, then swerve into the spot
    local spot = plan.parkSpot
    local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
    local _, _, entryLocalZ = AutoDrive.worldToLocal(self.vehicle, spot.entryX, spot.y, spot.entryZ)
    if entryLocalZ < 2 or MathUtil.vector2Length(spot.entryX - x, spot.entryZ - z) < 3 then
        self.vehicle.ad.specialDrivingModule:releaseVehicle()
        self:setState(ADTrafficYieldModule.STATE_PARKER_MOVE)
        return false
    end
    -- slow down so the swerve starts at park speed (no skid into the ditch at full speed)
    local parkSpeed = ADTrafficYieldModule.PARK_SPEED / 3.6
    local allowedSpeed = math.sqrt(parkSpeed * parkSpeed + 2 * ADTrafficYieldModule.ANTICIPATION_DECEL * math.max(entryLocalZ - 2, 0)) * 3.6
    local drivePathModule = self.vehicle.ad.drivePathModule
    if drivePathModule.speedLimit ~= nil then
        drivePathModule.speedLimit = math.min(drivePathModule.speedLimit, allowedSpeed)
    end
    if holding then
        -- the spot is known and the partner waits until we are parked: never stop before reaching it
        self.vehicle.ad.specialDrivingModule:releaseVehicle()
        self:ensureMotorStarted()
    end
    return false
end

function ADTrafficYieldModule:updatePriorityApproach(dt)
    local plan = self.plan
    if plan.parked then
        self.vehicle.ad.specialDrivingModule:releaseVehicle()
        self:setState(ADTrafficYieldModule.STATE_REVERSER_PASS)
        return false
    end
    -- keep the safe stopping distance until the other one has pulled over
    local holding = self:limitSpeedTowards(dt, self.partner, 1 - ADTrafficYieldModule.YIELDER_SHARE)
    -- once the roadside spot is known, stop before the end of its path so the other train can line up completely
    local spot = plan.parkSpot
    if not holding and spot ~= nil and spot.aimX ~= nil then
        local _, _, aimLocalZ = AutoDrive.worldToLocal(self.vehicle, spot.aimX, spot.y, spot.aimZ)
        local myFront = ADTrafficYieldModule.getTrainExtents(self.vehicle)
        local free = aimLocalZ - myFront - ADTrafficYieldModule.SPOT_CLEARANCE
        if aimLocalZ > 0 and free <= 0.5 then
            self:holdVehicle(dt)
            return true
        elseif aimLocalZ > 0 then
            local allowedSpeed = math.sqrt(2 * ADTrafficYieldModule.ANTICIPATION_DECEL * free) * 3.6
            local drivePathModule = self.vehicle.ad.drivePathModule
            if drivePathModule.speedLimit ~= nil then
                drivePathModule.speedLimit = math.min(drivePathModule.speedLimit, math.max(allowedSpeed, 3))
            end
        end
    end
    return holding
end

------------------------------------------------------------------------------------------------
-- detection
------------------------------------------------------------------------------------------------

function ADTrafficYieldModule:checkForHeadOnDeadlock(dt, obstacleDetected)
    local isStopped = ADTrafficYieldModule.getSpeedKmh(self.vehicle) < 1
    local onRoad = self.vehicle.ad.drivePathModule:isOnRoadNetwork()
    local delay = (AutoDrive.getSetting("trafficYieldDelay") or 5) * 1000

    local blockedLongEnough = self.blockedTimer:timer(obstacleDetected and isStopped and onRoad and g_time >= self.cooldownUntil, delay, dt)
    if not blockedLongEnough then
        return
    end
    if (g_updateLoopIndex % AutoDrive.PERF_FRAMES) ~= 0 then
        return
    end

    if ADTrafficYieldModule.intersectionHasPriority(self.vehicle) then
        return -- engaged on a red section: the crossing decides (wait or go)
    end
    local other = self:findHeadOnPartner()
    if other ~= nil and not ADTrafficYieldModule.intersectionHasPriority(other) then
        -- nose to nose, before anybody reverses, either of us takes another lane when there is one
        local otherModule = other.ad.trafficYieldModule
        if self:tryDetour(other) or (otherModule ~= nil and otherModule.tryDetour ~= nil and otherModule:tryDetour(self.vehicle)) then
            self.blockedTimer:timer(false)
            return
        end
        ADTrafficYieldModule.startPlan(self.vehicle, other)
    end
end

function ADTrafficYieldModule:findHeadOnPartner(excluded)
    local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
    local dirX, dirZ = ADTrafficYieldModule.getDirection(self.vehicle)
    local myLength = ADTrafficYieldModule.getTotalLength(self.vehicle)

    local bestOther = nil
    local bestDistance = math.huge

    for _, other in pairs(AutoDrive.getAllVehicles()) do
        if other ~= self.vehicle and other ~= excluded and other.ad ~= self.vehicle.ad and ADTrafficYieldModule.isAdVehicleUsable(other)
            and not AutoDrive:checkIsConnected(self.vehicle, other) then
            local ox, _, oz = ADTrafficYieldModule.getPosition(other)
            local distance = MathUtil.vector2Length(ox - x, oz - z)
            local maxDistance = ADTrafficYieldModule.MAX_DETECTION_DISTANCE + myLength / 2

            if distance < maxDistance and distance < bestDistance then
                local odirX, odirZ = ADTrafficYieldModule.getDirection(other)
                local headingDot = dirX * odirX + dirZ * odirZ
                -- other in front of me and me in front of other
                local _, _, otherLocalZ = AutoDrive.worldToLocal(self.vehicle, ox, 0, oz)
                local myX, myY, myZ = ADTrafficYieldModule.getPosition(self.vehicle)
                local _, _, meLocalZInOther = AutoDrive.worldToLocal(other, myX, myY, myZ)

                local otherModule = other.ad.trafficYieldModule
                local otherBlocked = other.ad.specialDrivingModule ~= nil and other.ad.specialDrivingModule:isStoppingVehicle()
                local otherStopped = ADTrafficYieldModule.getSpeedKmh(other) < 1
                local otherOnRoad = other.ad.drivePathModule:isOnRoadNetwork()


                if headingDot < -0.5 and otherLocalZ > 0 and meLocalZInOther > 0 and otherBlocked and otherStopped and otherOnRoad and not otherModule:isBusy() then
                    bestOther = other
                    bestDistance = distance
                end
            end
        end
    end
    return bestOther
end

------------------------------------------------------------------------------------------------
-- plan handling
------------------------------------------------------------------------------------------------

function ADTrafficYieldModule.getPairKey(vehicleA, vehicleB)
    local a, b = vehicleA.id or 0, vehicleB.id or 0
    if a > b then
        a, b = b, a
    end
    return tostring(a) .. "-" .. tostring(b)
end

function ADTrafficYieldModule.startPlan(vehicleA, vehicleB)
    local scoreA = ADTrafficYieldModule.getPriorityScore(vehicleA)
    local scoreB = ADTrafficYieldModule.getPriorityScore(vehicleB)

    local reverser, parker
    if scoreA < scoreB then
        reverser, parker = vehicleA, vehicleB
    elseif scoreB < scoreA then
        reverser, parker = vehicleB, vehicleA
    elseif (vehicleA.id or 0) < (vehicleB.id or 0) then
        reverser, parker = vehicleA, vehicleB
    else
        reverser, parker = vehicleB, vehicleA
    end

    -- previous failed attempts of this pair: 1 failure -> swap the roles (the other one reverses along
    -- its own route, where there may be a free roadside or a junction), 2 failures -> give up for a while
    local key = ADTrafficYieldModule.getPairKey(vehicleA, vehicleB)
    local history = ADTrafficYieldModule.pairHistory[key]
    if history ~= nil and g_time - history.time > ADTrafficYieldModule.HISTORY_TIMEOUT then
        ADTrafficYieldModule.pairHistory[key] = nil
        history = nil
    end
    if history ~= nil then
        if history.failures >= 2 then
            for _, vehicle in pairs({vehicleA, vehicleB}) do
                vehicle.ad.trafficYieldModule.cooldownUntil = g_time + ADTrafficYieldModule.GIVE_UP_TIME
                vehicle.ad.trafficYieldModule.blockedTimer:timer(false)
            end
            AutoDriveMessageEvent.sendMessageOrNotification(vehicleA, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_trafficYield_giveUp;", 10000, ADTrafficYieldModule.getName(vehicleA))
            return
        end
        if history.failures == 1 and history.lastReverser == reverser then
            reverser, parker = parker, reverser
        end
    end

    -- a vehicle with another one right behind it cannot reverse: it parks, the other one reverses
    local maxReverse = (AutoDrive.getSetting("trafficYieldMaxReverse") or 50) + 10
    if ADTrafficYieldModule.hasFollowerBehind(reverser, maxReverse, parker) and not ADTrafficYieldModule.hasFollowerBehind(parker, maxReverse, reverser) then
        reverser.ad.trafficYieldModule:log("a vehicle follows me, %s reverses instead", ADTrafficYieldModule.getName(parker))
        reverser, parker = parker, reverser
    end

    local plan = {
        pairKey = key,
        firstReverser = reverser, -- roles may be swapped during the plan, keep the initial one for the history
        reverser = reverser,
        parker = parker,
        startTime = g_time,
        parkSpot = nil,      -- set by parker when a free roadside spot is found
        reverserMayStop = false,
        reverserAtLimit = false,
        parked = false,
        passed = false,
        mode = "park",       -- "park": parker parks on the roadside / "branch": reverser hides in a side branch
        hidden = false
    }

    local rModule = reverser.ad.trafficYieldModule
    local pModule = parker.ad.trafficYieldModule
    rModule:log("STANDARD plan: I reverse, %s parks", ADTrafficYieldModule.getName(parker))

    -- measure both trains in their current state so the roadside spot fits them exactly
    ADTrafficYieldModule.measureTrain(reverser)
    ADTrafficYieldModule.measureTrain(parker)

    rModule:reset()
    pModule:reset()
    rModule.plan, rModule.partner = plan, parker
    pModule.plan, pModule.partner = plan, reverser


    rModule:setState(ADTrafficYieldModule.STATE_REVERSER_RETREAT)
    pModule:setState(ADTrafficYieldModule.STATE_PARKER_SEARCH)

    AutoDriveMessageEvent.sendMessageOrNotification(reverser, ADMessagesManager.messageTypes.INFO, "$l10n_AD_Driver_of; %s $l10n_AD_trafficYield_giveWay;", 5000, ADTrafficYieldModule.getName(reverser))
end

-- ends the plan for both vehicles
function ADTrafficYieldModule:endPlan(reason, success)
    local plan = self.plan
    self:log("end of plan: %s (success=%s)", tostring(reason), tostring(success))

    if plan ~= nil and plan.pairKey ~= nil and success ~= nil then
        if success then
            ADTrafficYieldModule.pairHistory[plan.pairKey] = nil
        else
            local history = ADTrafficYieldModule.pairHistory[plan.pairKey] or {failures = 0}
            history.failures = history.failures + 1
            history.time = g_time
            history.lastReverser = plan.firstReverser or plan.reverser
            if plan.mode == "early" then
                -- the same early plan would start again at once and block both again
                history.earlyTried = g_time
            end
            ADTrafficYieldModule.pairHistory[plan.pairKey] = history
        end
    end

    local vehicles = {self.vehicle}
    if plan ~= nil then
        vehicles = {plan.reverser, plan.parker}
    end

    for _, vehicle in pairs(vehicles) do
        if vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.trafficYieldModule ~= nil then
            local module = vehicle.ad.trafficYieldModule
            if module.plan == plan then
                local wasControlling = module.state ~= ADTrafficYieldModule.STATE_IDLE
                local reachedRouteIndex = module.reachedRouteIndex
                module:reset()
                if success == true then
                    module.cooldownUntil = g_time + ADTrafficYieldModule.COOLDOWN_SUCCESS
                elseif success == false then
                    module.cooldownUntil = g_time + ADTrafficYieldModule.COOLDOWN
                end
                if wasControlling and vehicle.ad.specialDrivingModule ~= nil then
                    vehicle.ad.specialDrivingModule:releaseVehicle()
                end
                ADTrafficYieldModule.resyncWayPoints(vehicle, reachedRouteIndex)
            end
        end
    end

    if success == false and plan ~= nil and plan.reverser ~= nil then
        AutoDriveMessageEvent.sendMessageOrNotification(plan.reverser, ADMessagesManager.messageTypes.WARN, "$l10n_AD_Driver_of; %s $l10n_AD_trafficYield_failed;", 5000, ADTrafficYieldModule.getName(plan.reverser))
    end
end

function ADTrafficYieldModule:checkPlanStillValid()
    local plan = self.plan
    if plan == nil then
        self:reset()
        return false
    end
    local partner = self.partner
    if not ADTrafficYieldModule.isAdVehicleUsable(partner) or partner.ad.trafficYieldModule.plan ~= plan then
        self:endPlan("partner not available any more", false)
        return false
    end
    if plan.mode == "sectionClear" and plan.parked and not self.vehicle.ad.drivePathModule:isOnRoadNetwork() and self.state == ADTrafficYieldModule.STATE_REVERSER_PASS then
        -- through the single track and into the field: done
        plan.passed = true
        self:endPlan("through the single track", true)
        return false
    end
    if not self.vehicle.ad.drivePathModule:isOnRoadNetwork() and self.state ~= ADTrafficYieldModule.STATE_PARKER_MOVE and self.state ~= ADTrafficYieldModule.STATE_PARKER_WAIT
        and self.state ~= ADTrafficYieldModule.STATE_SECTION_CLEAR then
        self:endPlan("vehicle left the road network", false)
        return false
    end
    if g_time - plan.startTime > ADTrafficYieldModule.PLAN_TIMEOUT then
        self:endPlan("timeout", false)
        return false
    end
    local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
    local px, _, pz = ADTrafficYieldModule.getPosition(partner)
    local maxApart = 120
    if plan.mode == "early" then
        maxApart = 3 * ADTrafficYieldModule.ANTICIPATION_DISTANCE
    elseif plan.mode == "sectionClear" then
        maxApart = ADTrafficYieldModule.SECTION_CLEAR_MAX + 100
    end
    if MathUtil.vector2Length(px - x, pz - z) > maxApart then
        self:endPlan("vehicles too far apart", plan.passed)
        return false
    end
    return true
end

-- after a manoeuvre the current waypoint may be behind the vehicle (parker drove past it on the roadside)
-- skip every waypoint that is no longer in front of the vehicle
function ADTrafficYieldModule.resyncWayPoints(vehicle, restartRouteIndex)
    local drivePathModule = vehicle.ad.drivePathModule
    if drivePathModule == nil then
        return
    end
    -- the reverser went back along its route: restart from the last route waypoint it reversed past
    local wps, idx = drivePathModule:getWayPoints()
    -- AutoDrive never drives with index 1 (it always needs the previous waypoint), so clamp to 2
    if restartRouteIndex ~= nil and wps ~= nil and idx ~= nil then
        restartRouteIndex = math.max(2, restartRouteIndex)
        if restartRouteIndex < idx and wps[restartRouteIndex] ~= nil and wps[restartRouteIndex - 1] ~= nil then
            drivePathModule:setCurrentWayPointIndex(restartRouteIndex)
        end
    end
    local wayPoints, currentIndex = drivePathModule:getWayPoints()
    local skipped = 0
    if wayPoints ~= nil and currentIndex ~= nil then
        while skipped < 30 do
            local wp = drivePathModule:getCurrentWayPoint()
            if wp == nil or drivePathModule:getNextWayPoint() == nil then
                break
            end
            local _, _, localZ = AutoDrive.worldToLocal(vehicle, wp.x, wp.y, wp.z)
            if localZ > 2 then
                break
            end
            drivePathModule:switchToNextWayPoint()
            skipped = skipped + 1
        end
    end
    drivePathModule.minDistanceToNextWp = math.huge
    if drivePathModule.minDistanceTimer ~= nil then
        drivePathModule.minDistanceTimer:timer(false)
    end
end

------------------------------------------------------------------------------------------------
-- REVERSER
------------------------------------------------------------------------------------------------

function ADTrafficYieldModule:buildReverseTargets(maxDistance)
    local targets = {}
    local x, y, z = ADTrafficYieldModule.getPosition(self.vehicle)
    local maxReverse = maxDistance or (AutoDrive.getSetting("trafficYieldMaxReverse") or 50) + 5
    local wayPoints, currentIndex = self.vehicle.ad.drivePathModule:getWayPoints()

    local lastX, lastZ = x, z
    local travelled = 0
    if wayPoints ~= nil and currentIndex ~= nil then
        local index = currentIndex - 1
        while index >= 1 and travelled < maxReverse do
            local wp = wayPoints[index]
            if wp == nil then
                break
            end
            local _, _, localZ = AutoDrive.worldToLocal(self.vehicle, wp.x, wp.y, wp.z)
            if localZ < -1 then
                local d = MathUtil.vector2Length(wp.x - lastX, wp.z - lastZ)
                local isJunction = self.escapeBranch ~= nil and self.escapeBranch.routeIndex == index
                if d >= ADTrafficYieldModule.REVERSE_STEP or (#targets == 0 and d >= 3) or (isJunction and d >= 1) then
                    travelled = travelled + d
                    table.insert(targets, {x = wp.x, y = wp.y, z = wp.z, routeIndex = index, isJunction = isJunction})
                    lastX, lastZ = wp.x, wp.z
                end
            end
            index = index - 1
        end
    end

    -- no route behind (start of the route): reverse straight
    if travelled < maxReverse then
        local dirX, dirZ
        if #targets >= 2 then
            local a, b = targets[#targets - 1], targets[#targets]
            local l = math.max(0.01, MathUtil.vector2Length(b.x - a.x, b.z - a.z))
            dirX, dirZ = (b.x - a.x) / l, (b.z - a.z) / l
        else
            local fx, fz = ADTrafficYieldModule.getDirection(self.vehicle)
            dirX, dirZ = -fx, -fz
        end
        while travelled < maxReverse do
            lastX = lastX + dirX * ADTrafficYieldModule.REVERSE_STEP
            lastZ = lastZ + dirZ * ADTrafficYieldModule.REVERSE_STEP
            travelled = travelled + ADTrafficYieldModule.REVERSE_STEP
            local ty = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, lastX, y, lastZ)
            table.insert(targets, {x = lastX, y = ty, z = lastZ, straight = true})
        end
    end

    return targets
end

-- collects the graph ids of the next waypoints of a vehicle route
function ADTrafficYieldModule.getUpcomingIds(vehicle, fromOffset, count)
    local ids = {}
    local wayPoints, currentIndex = vehicle.ad.drivePathModule:getWayPoints()
    if wayPoints ~= nil and currentIndex ~= nil then
        for i = math.max(1, currentIndex + fromOffset), currentIndex + fromOffset + count do
            local wp = wayPoints[i]
            if wp ~= nil and wp.id ~= nil then
                ids[wp.id] = true
            end
        end
    end
    return ids
end

-- searches, behind the reverser on its own route, a junction with a side branch that is not used by
-- the parker: the reverser can back into it and let the parker pass
function ADTrafficYieldModule:findEscapeBranch()
    local wayPoints, currentIndex = self.vehicle.ad.drivePathModule:getWayPoints()
    if wayPoints == nil or currentIndex == nil or ADGraphManager == nil then
        return nil
    end
    local maxReverse = AutoDrive.getSetting("trafficYieldMaxReverse") or 50
    local myLength = ADTrafficYieldModule.getTotalLength(self.vehicle)
    local neededDepth = myLength + 4

    -- nodes that must stay free: route of the parker ahead, and my own route around me
    local forbidden = ADTrafficYieldModule.getUpcomingIds(self.partner, -5, 150)
    for id, _ in pairs(ADTrafficYieldModule.getUpcomingIds(self.vehicle, -80, 160)) do
        forbidden[id] = true
    end

    local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
    local lastX, lastZ = x, z
    local travelled = 0
    local index = currentIndex - 1
    -- a waiting lane (dead end of the AutoDrive network) anywhere within reach is preferred to the
    -- first ordinary side branch
    local firstBranch = nil
    while index >= 2 and travelled < maxReverse do
        local wp = wayPoints[index]
        if wp == nil then
            break
        end
        travelled = travelled + MathUtil.vector2Length(wp.x - lastX, wp.z - lastZ)
        lastX, lastZ = wp.x, wp.z
        if wp.id ~= nil then
            local node = ADGraphManager:getWayPointById(wp.id)
            if node ~= nil then
                local neighbours = {}
                for _, id in pairs(node.out or {}) do neighbours[id] = true end
                for _, id in pairs(node.incoming or {}) do neighbours[id] = true end
                for neighbourId, _ in pairs(neighbours) do
                    if not forbidden[neighbourId] then
                        local branch = self:walkBranch(node, neighbourId, forbidden, neededDepth)
                        if branch ~= nil then
                            branch.routeIndex = index
                            branch.junctionId = wp.id
                            branch.isWaitingLane = ADTrafficYieldModule.isDeadEnd(node, neighbourId, forbidden)
                            if branch.isWaitingLane then
                                return branch
                            end
                            firstBranch = firstBranch or branch
                        end
                    end
                end
            end
        end
        index = index - 1
    end
    return firstBranch
end

ADTrafficYieldModule.WAITING_LANE_MAX_LENGTH = 80 -- m, a dead end longer than this is a road, not a waiting lane

-- waiting lane: the side branch starting at junction -> firstId ends in a dead end (no other way out)
-- within WAITING_LANE_MAX_LENGTH metres
function ADTrafficYieldModule.isDeadEnd(junction, firstId, forbidden)
    local visited = {[junction.id] = true}
    local current = ADGraphManager:getWayPointById(firstId)
    local length = 0
    local previous = junction
    while current ~= nil do
        if visited[current.id] or forbidden[current.id] then
            return false -- loop back or back to the route: it is a way through
        end
        visited[current.id] = true
        length = length + MathUtil.vector2Length(current.x - previous.x, current.z - previous.z)
        if length > ADTrafficYieldModule.WAITING_LANE_MAX_LENGTH then
            return false
        end
        local nextIds = {}
        for _, id in pairs(current.out or {}) do
            if not visited[id] then nextIds[id] = true end
        end
        for _, id in pairs(current.incoming or {}) do
            if not visited[id] then nextIds[id] = true end
        end
        local nextId, count = nil, 0
        for id, _ in pairs(nextIds) do
            nextId, count = id, count + 1
        end
        if count == 0 then
            return true
        end
        if count > 1 then
            return false -- the branch forks: a small network, not a simple waiting lane
        end
        previous = current
        current = ADGraphManager:getWayPointById(nextId)
    end
    return false
end

function ADTrafficYieldModule:walkBranch(junction, firstId, forbidden, neededDepth)
    local nodes = {}
    local visited = {[junction.id] = true}
    local current = ADGraphManager:getWayPointById(firstId)
    local previous = junction
    local depth = 0
    while current ~= nil and #nodes < 12 do
        if forbidden[current.id] or visited[current.id] then
            return nil
        end
        visited[current.id] = true
        depth = MathUtil.vector2Length(current.x - junction.x, current.z - junction.z)
        table.insert(nodes, {x = current.x, y = current.y, z = current.z, id = current.id})
        if depth >= neededDepth then
            return {nodes = nodes, depth = depth}
        end
        -- continue with the straightest following node
        local dirX, dirZ = current.x - previous.x, current.z - previous.z
        local best, bestDot = nil, -2
        local candidates = {}
        for _, id in pairs(current.out or {}) do candidates[id] = true end
        for _, id in pairs(current.incoming or {}) do candidates[id] = true end
        for id, _ in pairs(candidates) do
            if not visited[id] then
                local n = ADGraphManager:getWayPointById(id)
                if n ~= nil then
                    local vx, vz = n.x - current.x, n.z - current.z
                    local l1 = math.max(0.01, MathUtil.vector2Length(dirX, dirZ))
                    local l2 = math.max(0.01, MathUtil.vector2Length(vx, vz))
                    local dot = (dirX * vx + dirZ * vz) / (l1 * l2)
                    if dot > bestDot then
                        best, bestDot = n, dot
                    end
                end
            end
        end
        previous = current
        current = best
    end
    -- dead end: accept it if deep enough to hide most of the vehicle
    if depth >= neededDepth * 0.8 then
        return {nodes = nodes, depth = depth}
    end
    return nil
end

-- reverse towards a world point; own reached check (distance only) so sharp turns into a branch are possible
function ADTrafficYieldModule:reverseTowards(dt, target, reachedDistance)
    local sdm = self.vehicle.ad.specialDrivingModule
    sdm:releaseVehicle()
    self:ensureMotorStarted()
    sdm.reverseNode = sdm:getReverseNode()
    if sdm.reverseNode == nil then
        return true
    end
    sdm.reverseTarget = target
    sdm.currentWayPointIndex = 0
    sdm.wayPoints = {}
    sdm:getBasicStates()
    local distance = MathUtil.vector2Length(target.x - sdm.rNx, target.z - sdm.rNz)
    if distance < (reachedDistance or 2.5) then
        return true
    end
    -- target already in front of the reverse node: consider it reached
    local _, _, localZ = worldToLocal(sdm.reverseNode, target.x, target.y, target.z)
    if localZ > 1 and distance < 6 then
        return true
    end
    if self.vehicle.ad.collisionDetectionModule:checkReverseCollision() then
        sdm:stopAndHoldVehicle(dt)
    else
        sdm:reverseToPoint(dt, ADTrafficYieldModule.REVERSE_SPEED)
    end
    return false
end

function ADTrafficYieldModule:updateReverserRetreat(dt)
    local plan = self.plan
    if self.reverseTargets == nil then
        self.escapeBranch = self:findEscapeBranch()
        if self.escapeBranch ~= nil and self.escapeBranch.isWaitingLane then
            -- a waiting lane behind me: I back into it, the other one does not look for a roadside spot
            plan.mode = "branch"
            self:log("waiting lane (dead end) behind me, I back into it")
        end
        self.reverseTargets = self:buildReverseTargets()
        self.reverseTargetIndex = 1
        local x, y, z = ADTrafficYieldModule.getPosition(self.vehicle)
        self.reverseStart = {x = x, y = y, z = z}
    end

    local room = plan.moreRoom
    local extra = room ~= nil and room.pending and room.started
    if plan.reverserMayStop and not extra then
        self:setState(ADTrafficYieldModule.STATE_REVERSER_WAIT)
        self:holdVehicle(dt)
        return
    end

    -- the reverser may have a free roadside spot next to it: if so it parks itself and the other one passes
    if self:tryOwnParkingSpot(dt) then
        return
    end

    local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
    local reversed = MathUtil.vector2Length(x - self.reverseStart.x, z - self.reverseStart.z)
    local maxReverse = AutoDrive.getSetting("trafficYieldMaxReverse") or 50

    local blockedBehind = self.vehicle.ad.collisionDetectionModule:checkReverseCollision()
    local blockedTooLong = self.reverseBlockedTimer:timer(blockedBehind, 8000, dt)

    local target = self.reverseTargets[self.reverseTargetIndex]
    if extra then
        local more = MathUtil.vector2Length(x - room.x, z - room.z)
        if more >= ADTrafficYieldModule.MORE_ROOM or target == nil or blockedTooLong or reversed >= maxReverse + 5 then
            room.pending = false
            room.failed = more < ADTrafficYieldModule.MORE_ROOM_MIN
            self:log("backed off %.0f m more%s", more, room.failed and " (no room behind me)" or "")
            self:setState(ADTrafficYieldModule.STATE_REVERSER_WAIT)
            self:holdVehicle(dt)
            return
        end
    end
    if reversed >= maxReverse + 5 or target == nil or blockedTooLong then
        plan.reverserAtLimit = true
        self:setState(ADTrafficYieldModule.STATE_REVERSER_WAIT)
        self:holdVehicle(dt)
        return
    end

    if self:reverseTowards(dt, target) then
        if target.routeIndex ~= nil then
            self.reachedRouteIndex = target.routeIndex
        end
        self.reverseTargetIndex = self.reverseTargetIndex + 1
        -- at the junction and still no roadside spot for the parker: hide in the side branch
        if target.isJunction and self.escapeBranch ~= nil and (plan.parkSpot == nil or self.escapeBranch.isWaitingLane) then
            plan.mode = "branch"
            self.branchTargetIndex = 1
            self:setState(ADTrafficYieldModule.STATE_REVERSER_BRANCH)
        end
    end
end

function ADTrafficYieldModule:updateReverserBranch(dt)
    local plan = self.plan
    local blockedBehind = self.vehicle.ad.collisionDetectionModule:checkReverseCollision()
    local blockedTooLong = self.reverseBlockedTimer:timer(blockedBehind, 8000, dt)
    local timedOut = self.stateTimer:timer(true, 45000, dt)
    local target = self.escapeBranch.nodes[self.branchTargetIndex]

    if target == nil or blockedTooLong or timedOut then
        plan.hidden = true
        self:setState(ADTrafficYieldModule.STATE_REVERSER_HIDE)
        self:holdVehicle(dt)
        return
    end
    if self:reverseTowards(dt, target, 2) then
        self.branchTargetIndex = self.branchTargetIndex + 1
    end
end

function ADTrafficYieldModule:updateReverserHide(dt)
    self:holdVehicle(dt)
    -- the parker ends the plan once it has passed the junction
    if self.stateTimer:timer(true, 120000, dt) then
        self:endPlan("hide timeout", false)
    end
end

function ADTrafficYieldModule:updateReverserWait(dt)
    local plan = self.plan
    self:holdVehicle(dt)
    if plan.moreRoom ~= nil and plan.moreRoom.pending and not plan.moreRoom.started and not plan.parked then
        local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
        plan.moreRoom.started = true
        plan.moreRoom.x, plan.moreRoom.z = x, z
        self:log("backing off %d m more for %s", ADTrafficYieldModule.MORE_ROOM, ADTrafficYieldModule.getName(self.partner))
        self.vehicle.ad.specialDrivingModule:releaseVehicle()
        self:setState(ADTrafficYieldModule.STATE_REVERSER_RETREAT)
        return
    end
    if plan.parked then
        self:setState(ADTrafficYieldModule.STATE_REVERSER_PASS)
        self.vehicle.ad.specialDrivingModule:releaseVehicle()
        ADTrafficYieldModule.resyncWayPoints(self.vehicle, self.reachedRouteIndex)
        return
    end
    -- reverser stopped before the parker had a spot (limit reached): give both a last chance, then give up
    if plan.reverserAtLimit and plan.parkSpot == nil then
        if self:tryOwnParkingSpot(dt) then
            return
        end
        if self.stateTimer:timer(true, 6000, dt) then
            -- no roadside spot on either side (both would stay there): rather than giving up, I keep reversing along my route until I am off the
            -- way of the other one (junction, end of the single track, up to SECTION_CLEAR_MAX), it follows and passes
            if self:startClearAsReverser() then
                return
            end
            self:endPlan("no free roadside spot found", false)
        end
    end
end

-- the reverser of the standard plan reached its limit and the parker found no spot: the plan goes on as a
-- "single track clear" (the reverser backs until it is off the way of the other one, which then drives on)
function ADTrafficYieldModule:startClearAsReverser()
    local plan = self.plan
    local other = self.partner
    if other == nil or plan == nil or plan.sectionCleared or plan.mode == "branch" then
        return false
    end
    if ADTrafficYieldModule.hasFollowerBehind(self.vehicle, 20, other) then
        return false
    end
    plan.sectionCleared = true
    plan.mode = "sectionClear"
    plan.reverser, plan.parker = other, self.vehicle -- it drives on (REVERSER_PASS), I get off its way
    plan.parked = false
    plan.waitBeforeSection = nil
    plan.reverserMayStop = true
    plan.reverserAtLimit = false
    self:log("no roadside spot on either side: I keep reversing until I am off the way of %s (%d m at most)", ADTrafficYieldModule.getName(other), ADTrafficYieldModule.SECTION_CLEAR_MAX)
    local otherModule = other.ad.trafficYieldModule
    self.vehicle.ad.specialDrivingModule:releaseVehicle()
    other.ad.specialDrivingModule:releaseVehicle()
    self.reverseTargets = nil
    otherModule:setState(ADTrafficYieldModule.STATE_REVERSER_WAIT)
    otherModule:holdVehicle(0)
    self:setState(ADTrafficYieldModule.STATE_SECTION_CLEAR)
    return true
end

-- the reverser searches a roadside spot from its own position (towards the other vehicle). When it
-- finds one first, the roles are swapped: the reverser parks there and the other vehicle drives on.
function ADTrafficYieldModule:tryOwnParkingSpot(dt)
    local plan = self.plan
    if plan.parkSpot ~= nil or plan.mode == "branch" then
        return false
    end
    if g_time - self.lastSearchTime < ADTrafficYieldModule.SEARCH_INTERVAL then
        return false
    end
    self.lastSearchTime = g_time

    local spot = self:searchParkingSpot()
    if spot == nil then
        return false
    end

    -- swap the roles in the running plan
    self:log("own spot found (side=%s) -> roles swapped, I park", tostring(spot.side))
    local other = self.partner
    local otherModule = other.ad.trafficYieldModule
    plan.reverser, plan.parker = other, self.vehicle
    plan.parkSpot = spot
    plan.reverserMayStop = true
    plan.reverserAtLimit = false

    self.vehicle.ad.specialDrivingModule:releaseVehicle()
    self:setState(ADTrafficYieldModule.STATE_PARKER_MOVE)
    otherModule:setState(ADTrafficYieldModule.STATE_REVERSER_WAIT)
    return true
end

function ADTrafficYieldModule:updateReverserPass(dt)
    -- normal driving (module does not control the vehicle), only watch when we have passed the parker
    local plan = self.plan
    local parker = self.partner
    local px, py, pz = ADTrafficYieldModule.getPosition(parker)
    local _, _, parkerLocalZ = AutoDrive.worldToLocal(self.vehicle, px, py, pz)
    local parkerLength = ADTrafficYieldModule.getTotalLength(parker)
    local myLength = ADTrafficYieldModule.getTotalLength(self.vehicle)

    -- the parker centre is behind us by more than the length of both trains
    if parkerLocalZ < -(parkerLength + myLength / 2 + 2) then
        plan.passed = true
        self:endPlan("passed", true)
        return
    end
    if self.stateTimer:timer(true, 90000, dt) then
        self:endPlan("pass timeout", false)
    end
end

------------------------------------------------------------------------------------------------
-- PARKER
------------------------------------------------------------------------------------------------

function ADTrafficYieldModule:updateParkerSearch(dt)
    local plan = self.plan
    self:holdVehicle(dt)

    if plan.mode == "branch" then
        if plan.hidden then
            self.vehicle.ad.specialDrivingModule:releaseVehicle()
            ADTrafficYieldModule.resyncWayPoints(self.vehicle)
            self:setState(ADTrafficYieldModule.STATE_PARKER_PASS)
        end
        return
    end

    if g_time - self.lastSearchTime < ADTrafficYieldModule.SEARCH_INTERVAL then
        return
    end
    self.lastSearchTime = g_time

    local spot = self:searchParkingSpot()
    if spot ~= nil then
        self:log("spot found: side=%s field=%s", tostring(spot.side), tostring(spot.onField))
        plan.parkSpot = spot
        plan.reverserMayStop = true
        self:setState(ADTrafficYieldModule.STATE_PARKER_MOVE)
        self.vehicle.ad.specialDrivingModule:releaseVehicle()
        return
    end

    -- if the reverser cannot make more room, it ends the plan itself after a last search delay
end

-- searches a free spot on the roadside in front of the parker, inside the gap left by the reverser.
-- The road geometry of the route is followed (curves), the right side first, the left side as a last resort;
-- the straight search from the vehicle axis is only used when the route ahead is too short.
function ADTrafficYieldModule:searchParkingSpot()
    local rx, ry, rz = ADTrafficYieldModule.getPosition(self.partner)
    local _, _, reverserLocalZ = AutoDrive.worldToLocal(self.vehicle, rx, ry, rz)
    local reverserFront = ADTrafficYieldModule.getTrainExtents(self.partner)
    local gapEnd = reverserLocalZ - reverserFront - 2
    if gapEnd <= 0 then
        return nil
    end
    return self:searchSpotBySides(gapEnd, false) or self:searchParkingSpotStraight()
end

function ADTrafficYieldModule:searchParkingSpotStraight()
    local parker = self.vehicle
    local reverser = self.partner

    local px, py, pz = ADTrafficYieldModule.getPosition(parker)
    local dirX, dirZ = ADTrafficYieldModule.getDirection(parker)

    local parkerWidth = ADTrafficYieldModule.getWidth(parker)
    local reverserWidth = ADTrafficYieldModule.getWidth(reverser)
    local _, tractorLength = ADTrafficYieldModule.getUnitDimensions(parker)
    local parkerFront, parkerRear = ADTrafficYieldModule.getTrainExtents(parker)
    local totalLength = parkerFront + parkerRear
    local reverserFront = ADTrafficYieldModule.getTrainExtents(reverser)

    -- free gap in front of the parker up to the front of the reverser (front implements included)
    local rx, ry, rz = ADTrafficYieldModule.getPosition(reverser)
    local _, _, reverserLocalZ = AutoDrive.worldToLocal(parker, rx, ry, rz)
    local gapEnd = reverserLocalZ - reverserFront - 2

    local margin = AutoDrive.getSetting("trafficYieldMargin") or ADTrafficYieldModule.LATERAL_MARGIN
    local baseLateral = (parkerWidth + reverserWidth) / 2 + margin
    -- towed units need a straight run along the roadside after the swerve to line up behind the tractor
    local trailLength = math.max(0, parkerRear - tractorLength / 2)
    local runOut = ADTrafficYieldModule.getRunOut(trailLength)
    local minForward = math.max(8, tractorLength * 1.5, baseLateral * 2.5, trailLength)
    local maxForward = gapEnd - parkerFront - runOut

    if maxForward < minForward then
        return nil
    end

    -- local +x is left in GIANTS, so right side is negative x
    -- driving side only (see getSides)
    local sides = ADTrafficYieldModule.getSides()
    local laterals = {baseLateral, baseLateral + 1.0, baseLateral + 2.0}
    -- first pass: spot outside the fields (grass strip), second pass: spot may overlap a field
    -- every option on the driving side: grass first, then field
    for _, side in ipairs(sides) do
    for pass = 1, 2 do
    local allowField = pass == 2
        for _, lateral in ipairs(laterals) do
            local forward = minForward
            while forward <= maxForward do
                if self:isSpotFree(px, py, pz, dirX, dirZ, forward, side.sign * lateral, parkerWidth, parkerFront, totalLength, allowField, runOut) then
                    local wx, wy, wz = AutoDrive.localToWorld(parker, side.sign * lateral, 0, forward)
                    local finalX, _, finalZ = AutoDrive.localToWorld(parker, side.sign * lateral, 0, forward + runOut)
                    local ax, _, az = AutoDrive.localToWorld(parker, side.sign * lateral, 0, forward + runOut + 8)
                    return {
                        finalX = finalX, finalZ = finalZ,
                        originX = px, originZ = pz,
                        lateralAbs = lateral,
                        onField = allowField,
                        side = side.name,
                        forward = forward,
                        lateral = side.sign * lateral,
                        x = wx, y = wy, z = wz,
                        aimX = ax, aimZ = az,
                        dirX = dirX, dirZ = dirZ
                    }
                end
                forward = forward + 2
            end
        end
    end
    end
    return nil
end

function ADTrafficYieldModule:isSpotFree(px, py, pz, dirX, dirZ, forward, lateral, width, frontExtent, totalLength, allowField, runOut)
    local ry = math.atan2(dirX, dirZ)
    -- frame of the road at (px, pz) heading (dirX, dirZ); GIANTS local +x is left = (dirZ, -dirX)
    local function toWorld(lateralOffset, forwardOffset)
        return px + dirX * forwardOffset + dirZ * lateralOffset, py, pz + dirZ * forwardOffset - dirX * lateralOffset
    end
    local sign = lateral >= 0 and 1 or -1

    -- 1. final footprint of the whole train (tractor front at forward + tractorLength / 2)
    -- final position: tractor at the end of the straight run, whole train lined up behind it
    local frontZ = forward + (runOut or 0) + frontExtent
    local centerZ = frontZ - totalLength / 2
    local fx, _, fz = toWorld(lateral, centerZ)
    local roadY = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, px, py, pz)

    -- terrain check on the footprint corners
    local minH, maxH = math.huge, -math.huge
    local halfW = width / 2 + 0.2
    local halfL = totalLength / 2 + 0.3
    local onField = false
    for _, corner in ipairs({{-1, -1}, {-1, 1}, {1, -1}, {1, 1}, {0, 0}}) do
        local cx, _, cz = toWorld(lateral + corner[1] * halfW, centerZ + corner[2] * halfL)
        local h = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, cx, py, cz)
        minH = math.min(minH, h)
        maxH = math.max(maxH, h)
        if not allowField and AutoDrive.checkIsOnField(cx, h, cz) then
            onField = true
        end
    end
    if onField then
        return false
    end
    if (maxH - minH) > ADTrafficYieldModule.MAX_SLOPE_DIFF or math.abs(maxH - roadY) > ADTrafficYieldModule.MAX_ROAD_HEIGHT_DIFF or math.abs(minH - roadY) > ADTrafficYieldModule.MAX_ROAD_HEIGHT_DIFF then
        return false
    end

    -- water check
    if g_currentMission.environmentAreaSystem ~= nil and g_currentMission.environmentAreaSystem.getWaterYAtWorldPosition ~= nil then
        local waterY = g_currentMission.environmentAreaSystem:getWaterYAtWorldPosition(fx, roadY, fz)
        if waterY ~= nil and waterY > minH - 0.2 then
            return false
        end
    end

    -- box from 0.6 m to 3.2 m above the ground: low road borders / curbs are ignored
    local fy = math.max(maxH, roadY) + 1.9
    if self:overlapHit(fx, fy, fz, ry, halfW, 1.3, halfL) then
        return false
    end

    -- 2. swerve corridor, roadside part only (from the side of my own lane to the spot)
    local corridorStart = frontExtent
    local corridorEnd = frontZ
    local corridorHalfL = (corridorEnd - corridorStart) / 2
    local innerEdge = width / 2 + 0.2
    local outerEdge = math.abs(lateral) + width / 2
    if corridorHalfL > 0.5 and outerEdge > innerEdge then
        local corridorCenterX = sign * (innerEdge + outerEdge) / 2
        local corridorHalfW = (outerEdge - innerEdge) / 2
        local cx, _, cz = toWorld(corridorCenterX, corridorStart + corridorHalfL)
        if self:overlapHit(cx, fy, cz, ry, corridorHalfW, 1.3, corridorHalfL) then
            return false
        end
    end
    return true
end

function ADTrafficYieldModule:overlapHit(x, y, z, ry, halfX, halfY, halfZ)
    self.overlapHitCount = 0
    local mask = CollisionFlag.STATIC_OBJECT + CollisionFlag.DYNAMIC_OBJECT + CollisionFlag.VEHICLE + CollisionFlag.TREE + CollisionFlag.BUILDING + CollisionFlag.TRAFFIC_VEHICLE + CollisionFlag.TRAFFIC_VEHICLE_BLOCKING
    overlapBox(x, y, z, 0, ry, 0, halfX, halfY, halfZ, "overlapCallback", self, mask, true, true, true, true) -- dynamics, kinematics, statics, exactTest (real geometry, not the bounding box)
    return self.overlapHitCount > 0
end

function ADTrafficYieldModule:overlapCallback(transformId)
    if transformId == nil or transformId == 0 or transformId == g_currentMission.terrainRootNode then
        return true
    end
    local object = g_currentMission:getNodeObject(transformId)
    if object == nil then
        local parent = getParent(transformId)
        if parent ~= nil and parent ~= 0 then
            object = g_currentMission:getNodeObject(parent)
        end
    end
    if object ~= nil and (object == self.vehicle or AutoDrive:checkIsConnected(self.vehicle:getRootVehicle(), object)) then
        return true -- ignore myself
    end
    if object == nil then
        if ADTrafficYieldModule.IGNORED_SHAPE_NAMES[getName(transformId)] then
            return true -- field border
        end
        if ADSensor ~= nil and ADSensor.isElementBlockingVehicle ~= nil and not ADSensor:isElementBlockingVehicle(transformId) then
            return true -- traffic sign etc.
        end
        -- only keep the shapes the vehicle would physically collide with
        if not self:isShapeBlockingVehicle(transformId) then
            return true
        end
    end

    self.overlapHitCount = self.overlapHitCount + 1
    return false -- blocked, no need to look further
end

-- compares the collision filter of the shape with the one of the vehicle (FS25 physics rule:
-- two shapes collide when groupA & maskB ~= 0 and groupB & maskA ~= 0)
function ADTrafficYieldModule:isShapeBlockingVehicle(shapeId)
    if getCollisionFilterGroup == nil or getCollisionFilterMask == nil or bitAND == nil then
        return true
    end
    local ok, shapeGroup, shapeMask, vehGroup, vehMask = pcall(function()
        local vehicleNode = self.vehicle.components[1].node
        return getCollisionFilterGroup(shapeId), getCollisionFilterMask(shapeId), getCollisionFilterGroup(vehicleNode), getCollisionFilterMask(vehicleNode)
    end)
    if not ok or shapeGroup == nil or shapeMask == nil or vehGroup == nil or vehMask == nil then
        return true
    end
    return bitAND(shapeGroup, vehMask) ~= 0 and bitAND(vehGroup, shapeMask) ~= 0
end

-- lateral distance of the rear of the last towed unit from the original road line
-- last towed unit heading close to the road direction (less than about 10 degrees)
function ADTrafficYieldModule:isLastUnitParallel(spot)
    local units, count = AutoDrive.getAllUnits(self.vehicle)
    local unit = self.vehicle
    if units ~= nil and count ~= nil and count > 1 and units[count] ~= nil then
        unit = units[count]
    end
    local ux, _, uz = localDirectionToWorld(unit.components[1].node, 0, 0, 1)
    local l = MathUtil.vector2Length(ux, uz)
    if l < 0.01 then
        return true
    end
    return (ux * spot.dirX + uz * spot.dirZ) / l > 0.985
end

function ADTrafficYieldModule:getRearLateralOffset(spot)
    local units, count = AutoDrive.getAllUnits(self.vehicle)
    local unit = self.vehicle
    if units ~= nil and count ~= nil and count > 1 and units[count] ~= nil then
        unit = units[count]
    end
    local _, length = ADTrafficYieldModule.getUnitDimensions(unit)
    local rx, _, rz = localToWorld(unit.components[1].node, 0, 0, -length / 2)
    return math.abs((rx - spot.originX) * spot.dirZ - (rz - spot.originZ) * spot.dirX)
end

-- the parker could reach the reverser before being off the road and call itself parked, so the reverser could not
-- pass. Now the reverser backs off ADTrafficYieldModule.MORE_ROOM more meters (twice at most), the parker waits
-- meanwhile, then finishes pulling over.
ADTrafficYieldModule.MORE_ROOM = 20           -- m backed off once more by the reverser
ADTrafficYieldModule.MORE_ROOM_TRIES = 2
ADTrafficYieldModule.MORE_ROOM_MIN = 5        -- m: backed off less than this, no room behind it, the parker stays where it is

-- true while the parker waits for the reverser to make room (the caller returns)
function ADTrafficYieldModule:requestMoreRoom(dt)
    local plan = self.plan
    local room = plan.moreRoom
    if room ~= nil and room.pending then
        self:holdVehicle(dt)
        return true
    end
    if room ~= nil and room.failed then
        return false
    end
    if (plan.moreRoomCount or 0) >= ADTrafficYieldModule.MORE_ROOM_TRIES or plan.mode == "branch" then
        return false
    end
    plan.moreRoomCount = (plan.moreRoomCount or 0) + 1
    plan.moreRoom = {pending = true}
    self:log("%s too close in front before I am off the road: it backs off %d m more (%d/%d)", ADTrafficYieldModule.getName(self.partner),
        ADTrafficYieldModule.MORE_ROOM, plan.moreRoomCount, ADTrafficYieldModule.MORE_ROOM_TRIES)
    self:holdVehicle(dt)
    return true
end

-- the move timer does not run while the parker waits for room
function ADTrafficYieldModule:parkTimeout(dt, limit)
    local room = self.plan.moreRoom
    return self.stateTimer:timer(true, limit, room ~= nil and room.pending and 0 or dt)
end

function ADTrafficYieldModule:updateParkerMove(dt)
    local plan = self.plan
    local spot = plan.parkSpot
    local _, _, spotLocalZ = AutoDrive.worldToLocal(self.vehicle, spot.x, spot.y, spot.z)
    local _, _, finalLocalZ = AutoDrive.worldToLocal(self.vehicle, spot.finalX, spot.y, spot.finalZ)

    -- keep a safety distance to the waiting reverser
    local reverser = self.partner
    local rx, ry, rz = ADTrafficYieldModule.getPosition(reverser)
    local reverserLocalX, _, reverserLocalZ = AutoDrive.worldToLocal(self.vehicle, rx, ry, rz)
    local myFront = ADTrafficYieldModule.getTrainExtents(self.vehicle)
    local reverserFrontExtent = ADTrafficYieldModule.getTrainExtents(reverser)
    local sideBySide = math.abs(reverserLocalX) > (ADTrafficYieldModule.getWidth(self.vehicle) + ADTrafficYieldModule.getWidth(reverser)) / 2
    local tooCloseToReverser = not sideBySide and (reverserLocalZ - reverserFrontExtent) - myFront < 3

    -- parallel path along the route (curves followed)
    if spot.path ~= nil then
        if tooCloseToReverser and self:requestMoreRoom(dt) then
            return
        end
        if tooCloseToReverser then
            self:log("parked early: %s too close in front, the train may not be aligned", ADTrafficYieldModule.getName(reverser))
        end
        if tooCloseToReverser or self:followParkPath(dt, spot) or self:parkTimeout(dt, 60000) then
            self.pathIndex = nil
            plan.parked = true
            self:setState(ADTrafficYieldModule.STATE_PARKER_WAIT)
            self:holdVehicle(dt)
        end
        return
    end

    local rearAligned = self:getRearLateralOffset(spot) >= math.abs(spot.lateralAbs) - 0.2 and self:isLastUnitParallel(spot)
    local reachedFinal = finalLocalZ <= 0.5

    if tooCloseToReverser and not (reachedFinal and rearAligned) and self:requestMoreRoom(dt) then
        return
    end
    if (reachedFinal and rearAligned) or finalLocalZ <= -6 or tooCloseToReverser then
        self:log("parked: %s", (reachedFinal and rearAligned) and "aligned on the spot" or (finalLocalZ <= -6 and "spot passed")
            or string.format("%s too close in front, may still be on the road", ADTrafficYieldModule.getName(reverser)))
        plan.parked = true
        self:setState(ADTrafficYieldModule.STATE_PARKER_WAIT)
        self:holdVehicle(dt)
        return
    end

    if self:parkTimeout(dt, 45000) then
        self:log("parked: 45 s timeout, may still be on the road")
        plan.parked = true
        self:setState(ADTrafficYieldModule.STATE_PARKER_WAIT)
        self:holdVehicle(dt)
        return
    end

    -- obstacle in front while moving to the spot (pedestrian, AI traffic...)
    if self.vehicle.ad.sensors ~= nil and self.vehicle.ad.sensors.frontSensor ~= nil and self.vehicle.ad.sensors.frontSensor:pollInfo() and spotLocalZ > 4 then
        self:holdVehicle(dt)
        return
    end

    -- first swerve to the roadside line, then follow it straight so the towed units line up
    local aimTarget
    if spotLocalZ > 6 then
        aimTarget = {x = spot.x, y = spot.y, z = spot.z}
    else
        aimTarget = {x = spot.aimX, y = spot.y, z = spot.aimZ}
    end
    self.vehicle.ad.specialDrivingModule:releaseVehicle()
    self:ensureMotorStarted()
    self.vehicle.ad.specialDrivingModule:driveToPoint(dt, aimTarget, ADTrafficYieldModule.PARK_SPEED, false, 0.5, ADTrafficYieldModule.PARK_SPEED)
end

-- true when 'watched' has driven past me. Measured along the road direction (not my own axis, which can
-- be turned after pulling over), plus a fallback: it no longer comes towards me and is far enough.
function ADTrafficYieldModule:hasPassedMe(watched, dirX, dirZ)
    if watched == nil or watched.components == nil then
        return true
    end
    local px, _, pz = ADTrafficYieldModule.getPosition(self.vehicle)
    local wx, _, wz = ADTrafficYieldModule.getPosition(watched)
    if dirX == nil or dirZ == nil then
        dirX, dirZ = ADTrafficYieldModule.getDirection(self.vehicle)
    end
    local myFront, myRear = ADTrafficYieldModule.getTrainExtents(self.vehicle)
    local _, watchedRear = ADTrafficYieldModule.getTrainExtents(watched)
    local along = (wx - px) * dirX + (wz - pz) * dirZ
    if along < -(myRear + watchedRear + 2) then
        return true
    end
    -- the watched vehicle drives away from me and is past my front
    local distance = MathUtil.vector2Length(wx - px, wz - pz)
    if along < -myFront and distance > myFront + myRear + watchedRear + 5 then
        local wdx, wdz = ADTrafficYieldModule.getDirection(watched)
        if (wdx * (wx - px) + wdz * (wz - pz)) > 0 then
            return true
        end
    end
    return false
end

-- a train stopped before being lined up on its roadside path (other vehicle too close in front) still blocks
-- the road with its trailer: as soon as the way ahead is clear, it drives on along the path until aligned.
-- Returns true while it drives (the caller must not hold the vehicle)
function ADTrafficYieldModule:finishAlignment(dt, spot, watched)
    if spot == nil or spot.path == nil or self.alignmentDone then
        return false
    end
    if self:isRearOnPath(spot.path) then
        self.alignmentDone = true
        self.pathIndex = nil
        return false
    end
    if watched ~= nil and watched.components ~= nil then
        local wx, wy, wz = ADTrafficYieldModule.getPosition(watched)
        local watchedLocalX, _, watchedLocalZ = AutoDrive.worldToLocal(self.vehicle, wx, wy, wz)
        local myFront = ADTrafficYieldModule.getTrainExtents(self.vehicle)
        local watchedFront = ADTrafficYieldModule.getTrainExtents(watched)
        local sideBySide = math.abs(watchedLocalX) > (ADTrafficYieldModule.getWidth(self.vehicle) + ADTrafficYieldModule.getWidth(watched)) / 2
        if not sideBySide and watchedLocalZ > 0 and (watchedLocalZ - watchedFront) - myFront < 3 then
            return false
        end
    end
    if self.vehicle.ad.sensors ~= nil and self.vehicle.ad.sensors.frontSensor ~= nil and self.vehicle.ad.sensors.frontSensor:pollInfo() then
        return false
    end
    if (self.alignmentTime or 0) == 0 then
        self:log("finishing the alignment on the roadside path")
    end
    self.alignmentTime = (self.alignmentTime or 0) + dt
    if self.alignmentTime > 30000 or self:followParkPath(dt, spot) then
        self.alignmentDone = true
        self.pathIndex = nil
        return false
    end
    return true
end

function ADTrafficYieldModule:updateParkerWait(dt)
    local plan = self.plan
    if not self:finishAlignment(dt, plan.parkSpot, self.partner) then
        self:holdVehicle(dt)
    end
    if plan.passed then
        self:endPlan("reverser passed", true)
        return
    end
    -- double check from parker side: the reverser is behind us (along the road)
    local spot = plan.parkSpot
    if self:hasPassedMe(self.partner, spot and spot.dirX, spot and spot.dirZ) then
        plan.passed = true
        self:endPlan("reverser passed", true)
        return
    end
    if self.stateTimer:timer(true, 120000, dt) then
        self:endPlan("parker wait timeout", false)
    end
end

function ADTrafficYieldModule:updateParkerPass(dt)
    -- normal driving, watch until the hidden reverser is behind us
    local reverser = self.partner
    local rx, ry, rz = ADTrafficYieldModule.getPosition(reverser)
    local _, _, reverserLocalZ = AutoDrive.worldToLocal(self.vehicle, rx, ry, rz)
    local myLength = ADTrafficYieldModule.getTotalLength(self.vehicle)
    if reverserLocalZ < -(myLength + 3) then
        self.plan.passed = true
        self:endPlan("parker passed the side branch", true)
        return
    end
    if self.stateTimer:timer(true, 90000, dt) then
        self:endPlan("parker pass timeout", false)
    end
end

