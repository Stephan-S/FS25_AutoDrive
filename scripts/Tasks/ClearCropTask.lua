ClearCropTask = ADInheritsFrom(AbstractTask)

ClearCropTask.debug = false
ClearCropTask.TARGET_DISTANCE_SIDE = 10
ClearCropTask.TARGET_DISTANCE_FRONT_STEP = 10
ClearCropTask.MAX_CLEAR_STEPS = 4
ClearCropTask.ESCAPE_BASE_RADIUS = 80
ClearCropTask.ESCAPE_RADIUS_STEP = 30
ClearCropTask.ESCAPE_BASE_CELLS = 600
ClearCropTask.ESCAPE_LAST_RESORT_FRONT_PENALTY = 1000
ClearCropTask.TARGET_FRUIT_CLEARANCE = 4 -- m around complete train at final parking corridor
ClearCropTask.MAX_HARVESTER_DISTANCE = 50
ClearCropTask.COMBINE_EXCLUSION_FACTOR = 2 -- clear target must stay outside 2x the normal combine exclusion zone
ClearCropTask.WAIT_TIME = 10000
ClearCropTask.DRIVE_TIME = 30000
ClearCropTask.STALL_TIME = 15000 -- standstill time while driving before reversing out
ClearCropTask.STUCK_TIME = 60000
ClearCropTask.STATE_WAITING = {}
ClearCropTask.STATE_PLANNING = {}
ClearCropTask.STATE_ESCAPE_PLANNING = {}
ClearCropTask.STATE_DRIVING = {}
ClearCropTask.STATE_REVERSING = {}

ClearCropTask.LEFT = 1
ClearCropTask.RIGHT = -1

function ClearCropTask:new(vehicle, harvester)
    local o = ClearCropTask:create()
    o.vehicle = vehicle
    o.harvester = harvester
    o.waitTimer = AutoDriveTON:new()
    o.driveTimer = AutoDriveTON:new()
    o.stuckTimer = AutoDriveTON:new()
    o.state = ClearCropTask.STATE_WAITING
    o.reverseStartLocation = nil
    o.clearStep = 1
    o.vehicleTrainLength = AutoDrive.getTractorTrainLength(vehicle, true, false)
    ClearCropTask.setStateNames(o)
    return o
end

function ClearCropTask:setUp()
    ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:setUp")
    self.clearStep = 1
end

-- Which side to clear towards and which vehicle (own or harvester) to measure the escape hop
-- from - independent of exclusion-zone/reachability checks.
function ClearCropTask:pickClearSide()
    local leftBlocked = self.vehicle.ad.sensors.leftSensorFruit:pollInfo() or self.vehicle.ad.sensors.leftSensor:pollInfo()
    local rightBlocked = self.vehicle.ad.sensors.rightSensorFruit:pollInfo() or self.vehicle.ad.sensors.rightSensor:pollInfo()

    local leftFrontBlocked = self.vehicle.ad.sensors.leftFrontSensorFruit:pollInfo() or self.vehicle.ad.sensors.leftFrontSensor:pollInfo()
    local rightFrontBlocked = self.vehicle.ad.sensors.rightFrontSensorFruit:pollInfo() or self.vehicle.ad.sensors.rightFrontSensor:pollInfo()

    leftBlocked = leftBlocked or leftFrontBlocked
    rightBlocked = rightBlocked or rightFrontBlocked

    local cleartowards = ClearCropTask.RIGHT
    if leftBlocked and rightBlocked then
        cleartowards = ClearCropTask.RIGHT
    elseif leftBlocked then
        cleartowards = ClearCropTask.RIGHT
    elseif rightBlocked then
        cleartowards = ClearCropTask.LEFT
    end

    if self.harvester ~= nil and AutoDrive.getDistanceBetween(self.vehicle, self.harvester) < ClearCropTask.MAX_HARVESTER_DISTANCE then
        return self.harvester, cleartowards
    end
    return self.vehicle, cleartowards
end

function ClearCropTask:getDoubledCombineExclusionZone()
    local exclusionZone = AutoDrive.getCombineExclusionZone(self.harvester)
    if exclusionZone ~= nil then
        exclusionZone.radius = exclusionZone.radius * ClearCropTask.COMBINE_EXCLUSION_FACTOR
    end
    return exclusionZone
end

-- Retry Dijkstra with wider limits. After MAX_CLEAR_STEPS, wait and restart; never blind-drive.
function ClearCropTask:retryPlanning()
    if self.clearStep < ClearCropTask.MAX_CLEAR_STEPS then
        self.clearStep = self.clearStep + 1
        self:startPlanningStep()
    else
        -- Never replace failed collision-checked planning with a blind straight hop. Wait and
        -- restart strict search; vehicle stays stopped until a real course exists.
        ClearCropTask.debugMsg(self.vehicle, "ClearCropTask - no Dijkstra escape course found, waiting before retry")
        self.clearStep = 1
        self.escapeJob = nil
        self:resetAllTimers()
        self.stuckTimer:timer(false)
        self.state = ClearCropTask.STATE_WAITING
    end
end

function ClearCropTask:startPlanningStep()
    -- Primary: local collision-checked escape. Fruit, harvester exclusion zone and the very
    -- high front-of-harvester penalty decide its route.
    self:startEscapePlanning()
end

function ClearCropTask:startEscapePlanning()
    -- Generate a local collision-checked course. Each retry widens search and raises cell budget;
    -- this task never substitutes a network-target course for the local crop escape.
    local maxRadius = ClearCropTask.ESCAPE_BASE_RADIUS + (self.clearStep - 1) * ClearCropTask.ESCAPE_RADIUS_STEP
    local maxCells = ClearCropTask.ESCAPE_BASE_CELLS * self.clearStep
    local combineFrontPenalty = ADEscapeCourseGenerator.COMBINE_FRONT_CELL_PENALTY
    if self.clearStep == ClearCropTask.MAX_CLEAR_STEPS then
        -- Only after all strict searches failed, allow ground ahead of the harvester at a still
        -- very high cost. This makes "front" reachable as genuine last Dijkstra option.
        combineFrontPenalty = ClearCropTask.ESCAPE_LAST_RESORT_FRONT_PENALTY
    end
    self.escapeJob = ADEscapeCourseGenerator.begin(self.vehicle, ADEscapeCourseGenerator.TARGET_FRUIT_FREE, {
        exclusionZone = self:getDoubledCombineExclusionZone(),
        combine = self.harvester,
        combineFrontPenalty = combineFrontPenalty,
        targetFruitClearance = ClearCropTask.TARGET_FRUIT_CLEARANCE,
        maxRadius = maxRadius,
        maxCells = maxCells
    })
    ClearCropTask.debugMsg(self.vehicle, "ClearCropTask - starting Dijkstra escape attempt %d/%d, radius %dm, cells %d, frontPenalty %d", self.clearStep, ClearCropTask.MAX_CLEAR_STEPS, maxRadius, maxCells, combineFrontPenalty)
    self.state = ClearCropTask.STATE_ESCAPE_PLANNING
end

function ClearCropTask:update(dt)
    if self.lastState ~= self.state then
        ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:update %s -> %s", tostring(self:getStateName(self.lastState)), tostring(self:getStateName()))
        self.lastState = self.state
    end

    -- only counts while actually standing still - a long (but moving) escape course must not
    -- trip this and abort the task mid-way
    self.stuckTimer:timer(self.vehicle.lastSpeedReal <= 0.0002, ClearCropTask.STUCK_TIME, dt)
    if self.stuckTimer:done() then
        ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:update stuckTimer:done")
        self:finished()
        return
    end

    if self.state == ClearCropTask.STATE_WAITING then
        self.waitTimer:timer(true, ClearCropTask.WAIT_TIME, dt)
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        if self.waitTimer:done() then
            ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:update STATE_WAITING - done waiting - plan a path out now...")
            self:resetAllTimers()
            self:startPlanningStep()
            return
        end
    elseif self.state == ClearCropTask.STATE_PLANNING then
        self:startPlanningStep()
    elseif self.state == ClearCropTask.STATE_ESCAPE_PLANNING then
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        if self.escapeJob ~= nil then
            self.escapeJob:update()
            if self.escapeJob:isFinished() then
                local escapeCourse = self.escapeJob:getCourse()
                self.escapeJob = nil
                if escapeCourse ~= nil then
                    ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:update - using escape course with %d waypoints", #escapeCourse)
                    self.vehicle.ad.drivePathModule:setWayPoints(escapeCourse)
                    self:resetAllTimers()
                    self.state = ClearCropTask.STATE_DRIVING
                else
                    self:retryPlanning()
                end
            end
        else
            -- Job lost (should not happen) - retry Dijkstra with wider limits.
            self:retryPlanning()
        end
    elseif self.state == ClearCropTask.STATE_DRIVING then
        -- reverse out only on real standstill (blocked), never just because the course takes a
        -- while - a fixed drive timeout used to fire mid-course and shove the rig back into crop
        self.driveTimer:timer(self.vehicle.lastSpeedReal <= 0.0002, ClearCropTask.STALL_TIME, dt)
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:update STATE_DRIVING isTargetReached")
            if AutoDrive.isVehicleOrTrailerInCrop(self.vehicle, true) and self.clearStep < ClearCropTask.MAX_CLEAR_STEPS then
                -- still not clear enough - the reached point was apparently in/next to fruit
                -- itself, move on to the next candidate/step instead of repeating this one
                self:resetAllTimers()
                self:retryPlanning()
            else
                self:finished()
            end
            return
        elseif self.driveTimer:done() then
            ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:update STATE_DRIVING stalled -> reversing")
            self:resetAllTimers()
            local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
            self.reverseStartLocation = {x = x, y = y, z = z}
            self.state = ClearCropTask.STATE_REVERSING
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end
    elseif self.state == ClearCropTask.STATE_REVERSING then
        local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
        local distanceToReversStart = MathUtil.vector2Length(x - self.reverseStartLocation.x, z - self.reverseStartLocation.z)
        if (g_updateLoopIndex % 60 == 0) then
            ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:update distanceToReversStart %.0f"
            , distanceToReversStart
            )
        end
        if distanceToReversStart > 20 then
            ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:update distanceToReversStart > 20")
            self:resetAllTimers()
            self.clearStep = 1
            self.state = ClearCropTask.STATE_PLANNING
        else
            self.vehicle.ad.specialDrivingModule:driveReverse(dt, 15, 1, self.vehicle.ad.trailerModule:canBeHandledInReverse())
        end
    end
end

function ClearCropTask:abort()
    ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:abort")
end

function ClearCropTask:finished()
    ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:finished")
    self.vehicle.ad.taskModule:setCurrentTaskFinished()
end

function ClearCropTask:setStateNames()
    if self.statesToNames == nil then
        self.statesToNames = {}
        for name, id in pairs(ClearCropTask) do
            if string.sub(name, 1, 6) == "STATE_" then
                self.statesToNames[id] = name
            end
        end
    end
end

function ClearCropTask:getStateName(state)
    local requestedState = state
    if requestedState == nil then
        requestedState = self.state
    end
    if requestedState == nil then
        Logging.error("[AD] ClearCropTask: Could not find name for state ->%s<- !", tostring(requestedState))
    end
    return self.statesToNames[requestedState] or ""
end

function ClearCropTask:resetAllTimers()
    -- self.stuckTimer:timer(false) -- stuckTimer reset by speed changes
    self.waitTimer:timer(false)
    self.driveTimer:timer(false)
end

function ClearCropTask:getI18nInfo()
    local text = "$l10n_AD_task_clearcrop;"
    if self.state == ClearCropTask.STATE_ESCAPE_PLANNING then
        text = text .. string.format(" - Dijkstra %d/%d", self.clearStep, ClearCropTask.MAX_CLEAR_STEPS)
    elseif self.state == ClearCropTask.STATE_DRIVING then
        text = text .. string.format(" - %d/%d", self.clearStep, ClearCropTask.MAX_CLEAR_STEPS)
    elseif self.state == ClearCropTask.STATE_REVERSING then
        text = text .. " - " .. "$l10n_AD_task_reversing_from_combine;"
    elseif self.state == ClearCropTask.STATE_WAITING then
        text = text .. " - " .. "$l10n_AD_task_waiting_for_room;"
    end
    return text
end

function ClearCropTask.debugMsg(vehicle, debugText, ...)
    if ClearCropTask.debug == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    else
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_COMBINEINFO, debugText, ...)
    end
end
