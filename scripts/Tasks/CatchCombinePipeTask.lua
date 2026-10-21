CatchCombinePipeTask = ADInheritsFrom(AbstractTask)
CatchCombinePipeTask.debug = false

CatchCombinePipeTask.TARGET_DISTANCE = 15

CatchCombinePipeTask.STATE_PATHPLANNING = {}
CatchCombinePipeTask.STATE_DRIVING = {}
CatchCombinePipeTask.STATE_REVERSING = {}
CatchCombinePipeTask.STATE_DELAY_PATHPLANNING = {}
CatchCombinePipeTask.STATE_WAIT_BEFORE_FINISH = {}
CatchCombinePipeTask.STATE_FINISHED = {}

CatchCombinePipeTask.MAX_REVERSE_DISTANCE = 18
CatchCombinePipeTask.MIN_COMBINE_DISTANCE = 25
CatchCombinePipeTask.MAX_REVERSE_TIME = 5000
CatchCombinePipeTask.MAX_STUCK_TIME = 60000
CatchCombinePipeTask.MAX_COUNT_NEW_PATHFINDING = 20
-- waiting for a route while the harvester comes towards the unloader: get out of its way
CatchCombinePipeTask.STATE_CLEAR_WAY = {}
CatchCombinePipeTask.STATE_WAIT_PASS_BY = {}
CatchCombinePipeTask.CLEAR_WAY_DISTANCE = 40      -- m in front of the harvester watched
CatchCombinePipeTask.CLEAR_WAY_MARGIN = 2         -- m added on each side of the cutting width
CatchCombinePipeTask.CLEAR_WAY_MIN_SPEED = 0.0005 -- harvester speed (lastSpeedReal) counted as driving forward
CatchCombinePipeTask.CLEAR_WAY_MAX_REVERSE = 20   -- m
CatchCombinePipeTask.CLEAR_WAY_REVERSE_SPEED = 6  -- km/h
CatchCombinePipeTask.CLEAR_WAY_TIMEOUT = 25000    -- ms
CatchCombinePipeTask.CLEAR_WAY_STUCK_TIME = 3000  -- ms without moving while reversing
CatchCombinePipeTask.CLEAR_WAY_REAR_CHECK = 3     -- m behind the last unit that must be field ground without crop
CatchCombinePipeTask.PASS_BY_TIMEOUT = 30000      -- ms
CatchCombinePipeTask.PASS_BY_BEHIND = 5           -- m: the unloader is behind the harvester, it has passed

function CatchCombinePipeTask:new(vehicle, combine)
    local o = CatchCombinePipeTask:create()
    o.vehicle = vehicle
    o.combine = combine
    o.state = CatchCombinePipeTask.STATE_DELAY_PATHPLANNING
    o.wayPoints = nil
    o.stuckTimer = AutoDriveTON:new()
    o.reverseTimer = AutoDriveTON:new()
    o.waitTimer = AutoDriveTON:new()
    o.waitForCheckTimer = AutoDriveTON:new()
    o.waitForCheckTimer.elapsedTime = 4000
    o.taskType = "CatchCombinePipeTask"
    o.newPathFindingCounter = 0
    o.trailers = nil
    CatchCombinePipeTask.setStateNames(o)
    return o
end

function CatchCombinePipeTask:setUp()
    CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:setUp()")
    local angleToCombineHeading = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getAngleToCombineHeading()
    local angleToCombine = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getAngleToCombine()

    if angleToCombineHeading < 35 and angleToCombine < 90 and AutoDrive.getDistanceBetween(self.vehicle, self.combine) < 60
        and not self.combine.ad.isFixedPipeChopper then
        -- close and aligned: chase straight away, but only if the straight line is free, otherwise plan a route
        local chasePos, chaseSide = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getPipeChasePosition()
        local clear, reason = FollowCombineTask.isChaseLineClear(self.vehicle, self.combine, chasePos, chaseSide)
        if clear then
            self:finished()
        else
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_COMBINEINFO, "CatchCombinePipeTask: harvester close but the straight line is blocked (%s), path search", tostring(reason))
        end
    end
    self.trailers, _ = AutoDrive.getAllUnits(self.vehicle)
    AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, true)
end

function CatchCombinePipeTask:update(dt)
    if self.combine ~= nil and g_currentMission.nodeToObject[self.combine.components[1].node] == nil then
        self:finished()
        return
    end
    if self.lastState ~= self.state then
        CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update %s -> %s", tostring(self:getStateName(self.lastState)), tostring(self:getStateName()))
        self.lastState = self.state
        self:resetAllTimers()
    end

    local checkForStuck = (self.vehicle.lastSpeedReal <= 0.0002) and (
        self.state == CatchCombinePipeTask.STATE_DRIVING
    )

    self.stuckTimer:timer(checkForStuck, self.MAX_STUCK_TIME, dt)
    if self.stuckTimer:done()
        -- or AutoDrive.getDistanceBetween(self.vehicle, self.combine) < self.MIN_COMBINE_DISTANCE 
        then
        -- got stuck or to close to combine -> reverse
        CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - STATE_DRIVING stuck")
        self.stuckTimer:timer(false)
        local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
        self.reverseStartLocation = {x = x, y = y, z = z}
        self.state = CatchCombinePipeTask.STATE_REVERSING
        return
    end

    if (self.state == CatchCombinePipeTask.STATE_PATHPLANNING or self.state == CatchCombinePipeTask.STATE_DELAY_PATHPLANNING)
        and self:isInCombinePath() then
        self:startClearWay()
        return
    end

    if self.state == CatchCombinePipeTask.STATE_CLEAR_WAY then
        self:updateClearWay(dt)
        return
    elseif self.state == CatchCombinePipeTask.STATE_WAIT_PASS_BY then
        self:updateWaitPassBy(dt)
        return
    end

    if self.state == CatchCombinePipeTask.STATE_PATHPLANNING then
        if self.vehicle.ad.pathFinderModule:hasFinished() then
            CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - STATE_PATHPLANNING finished")
            self.newPathFindingCounter = 0
            self.wayPoints = self.vehicle.ad.pathFinderModule:getPath()
            if self.wayPoints == nil or #self.wayPoints < 1 then
                --restart
                self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:notifyAboutFailedPathfinder()
                --AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.WARN, "$l10n_AD_Driver_of; %s $l10n_AD_cannot_find_path; %s", 5000, self.vehicle.ad.stateModule:getName(), self.combine.ad.stateModule:getName())
                CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - STATE_PATHPLANNING restarting path finder - with delay")
                self.state = CatchCombinePipeTask.STATE_DELAY_PATHPLANNING
                return
            else
                self.vehicle.ad.drivePathModule:setWayPoints(self.wayPoints)
                self.state = CatchCombinePipeTask.STATE_DRIVING
                return
            end
        else
            self.vehicle.ad.pathFinderModule:update(dt)
            self.vehicle.ad.specialDrivingModule:stopVehicle()
            self.vehicle.ad.specialDrivingModule:update(dt)
            return
        end
    elseif self.state == CatchCombinePipeTask.STATE_DELAY_PATHPLANNING then
        CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - STATE_DELAY_PATHPLANNING")
        if self.waitForCheckTimer:timer(true, 1000, dt) then
            if self.newPathFindingCounter > self.MAX_COUNT_NEW_PATHFINDING then
                -- prevent deadlock
                self.state = CatchCombinePipeTask.STATE_FINISHED
                return
            end
            if self:startNewPathFinding() then
                self.state = CatchCombinePipeTask.STATE_PATHPLANNING
                return
            end
        end        
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return
    elseif self.state == CatchCombinePipeTask.STATE_DRIVING then
        -- check if this is still a clever path to follow
        -- do this by distance of the combine to the last location pathfinder started at
        local x, y, z = getWorldTranslation(self.combine.components[1].node)
        local combineTravelDistance = MathUtil.vector2Length(x - self.combinesStartLocation.x, z - self.combinesStartLocation.z)

        if combineTravelDistance > 85 then
            CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - combine travelled - recalculate path")
            self.waitForCheckTimer.elapsedTime = 4000
            self.state = CatchCombinePipeTask.STATE_DELAY_PATHPLANNING
            return
        else
            if self.vehicle.ad.drivePathModule:isTargetReached() then
                -- check if we have actually reached the target or not
                -- accept current location if we are in a good position to start chasing: distance and angle are important here
                CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - STATE_DRIVING TargetReached")
                local angleToCombine = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getAngleToCombineHeading()
                local isCorrectSide = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:isUnloaderOnCorrectSide()

                if angleToCombine < 35 and AutoDrive.getDistanceBetween(self.vehicle, self.combine) < 80 and isCorrectSide then
                    CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - STATE_DRIVING -> STATE_FINISHED")
                    self.state = CatchCombinePipeTask.STATE_FINISHED
                    return
                else
                    CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - angle or distance to combine too high - recalculate path now")
                    self.state = CatchCombinePipeTask.STATE_DELAY_PATHPLANNING
                    return
                end
            else
                self.vehicle.ad.drivePathModule:update(dt)
            end
        end
    elseif self.state == CatchCombinePipeTask.STATE_REVERSING then
        local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
        local distanceToReverseStart = MathUtil.vector2Length(x - self.reverseStartLocation.x, z - self.reverseStartLocation.z)
        self.reverseTimer:timer(true, self.MAX_REVERSE_TIME, dt)
        if distanceToReverseStart > self.MAX_REVERSE_DISTANCE or self.reverseTimer:done() then
            self.state = CatchCombinePipeTask.STATE_WAIT_BEFORE_FINISH
            return
        else
            self.vehicle.ad.specialDrivingModule:driveReverse(dt, 15, 1, self.vehicle.ad.trailerModule:canBeHandledInReverse())
        end
    elseif self.state == CatchCombinePipeTask.STATE_WAIT_BEFORE_FINISH then
        CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - STATE_WAIT_BEFORE_FINISH")
        self.waitTimer:timer(true, self.MAX_REVERSE_TIME, dt)
        if self.waitTimer:done() then
            self.state = CatchCombinePipeTask.STATE_FINISHED
            return
        else
            self.vehicle.ad.specialDrivingModule:stopVehicle()
            self.vehicle.ad.specialDrivingModule:update(dt)
            return
        end
    elseif self.state == CatchCombinePipeTask.STATE_FINISHED then
        CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - STATE_FINISHED")
        self:finished()
        return
    end
end

function CatchCombinePipeTask:abort()
end

-- tractor and towed units of the unloader
function CatchCombinePipeTask:getTrainUnits()
    local units = {self.vehicle}
    local all = AutoDrive.getAllUnits(self.vehicle)
    if all ~= nil then
        for _, unit in ipairs(all) do
            if unit ~= self.vehicle and unit.components ~= nil then
                table.insert(units, unit)
            end
        end
    end
    return units
end

-- the harvester drives forward and a unit of the unloader stands in its way: in front of it, within its cutting width
function CatchCombinePipeTask:isInCombinePath()
    local combine = self.combine
    if combine == nil or combine.ad == nil or combine.ad.ADRootNode == nil then
        return false
    end
    local movingDirection = combine.movingDirection or 1
    if combine.ad.isReverseAttached then
        movingDirection = -movingDirection
    end
    if (combine.lastSpeedReal or 0) * movingDirection < CatchCombinePipeTask.CLEAR_WAY_MIN_SPEED then
        return false
    end
    local laneHalf = math.max(AutoDrive.getFrontToolWidth(combine) or 0, combine.size.width) / 2 + CatchCombinePipeTask.CLEAR_WAY_MARGIN
    for _, unit in ipairs(self:getTrainUnits()) do
        local x, y, z = getWorldTranslation(unit.components[1].node)
        local diffX, _, diffZ = AutoDrive.worldToLocal(combine, x, y, z, combine.ad.ADRootNode)
        local unitHalfWidth = unit.size ~= nil and unit.size.width / 2 or 1.5
        if diffZ > 0 and diffZ < CatchCombinePipeTask.CLEAR_WAY_DISTANCE and math.abs(diffX) < laneHalf + unitHalfWidth then
            return true
        end
    end
    return false
end

-- ground behind the last unit: field ground without standing crop, across the width of the train
function CatchCombinePipeTask:isRearClear()
    local units = self:getTrainUnits()
    local last = units[#units]
    local dx, _, dz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local x, _, z = getWorldTranslation(last.components[1].node)
    local back = (last.size ~= nil and last.size.length / 2 or 3) + CatchCombinePipeTask.CLEAR_WAY_REAR_CHECK
    local half = self.vehicle.ad.pathFinderModule:getTrainWidth() / 2
    local rx, rz = x - dx * back, z - dz * back
    for _, side in ipairs({0, -1, 1}) do
        local px, pz = rx - dz * half * side, rz + dx * half * side
        if not AutoDrive.checkIsOnField(px, 0, pz) or PathFinderModule.getMapCropAt(px, pz) ~= 0 then
            return false
        end
    end
    return true
end

function CatchCombinePipeTask:startClearWay()
    if self.state == CatchCombinePipeTask.STATE_PATHPLANNING then
        self.vehicle.ad.pathFinderModule:reset()
    end
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    self.clearWayStart = {x = x, z = z}
    self.clearWayTime = 0
    self.clearWayStuckTime = 0
    self.clearWayLast = {x = x, z = z}
    if self:isRearClear() then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_COMBINEINFO, "CatchCombinePipeTask: the harvester is coming towards the unloader, reversing out of its way")
        self.state = CatchCombinePipeTask.STATE_CLEAR_WAY
    else
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_COMBINEINFO, "CatchCombinePipeTask: the harvester is coming towards the unloader, cannot reverse (crop or field edge behind), waiting")
        self.passByTime = 0
        self.state = CatchCombinePipeTask.STATE_WAIT_PASS_BY
    end
end

function CatchCombinePipeTask:updateClearWay(dt)
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    local reversed = MathUtil.vector2Length(x - self.clearWayStart.x, z - self.clearWayStart.z)
    self.clearWayTime = self.clearWayTime + dt
    if MathUtil.vector2Length(x - self.clearWayLast.x, z - self.clearWayLast.z) > 0.3 then
        self.clearWayLast = {x = x, z = z}
        self.clearWayStuckTime = 0
    else
        self.clearWayStuckTime = self.clearWayStuckTime + dt
    end
    local reason = nil
    if not self:isInCombinePath() then
        reason = "out of its way"
    elseif reversed >= CatchCombinePipeTask.CLEAR_WAY_MAX_REVERSE then
        reason = "maximum reverse distance"
    elseif not self:isRearClear() then
        reason = "crop or field edge behind"
    elseif self.clearWayTime > CatchCombinePipeTask.CLEAR_WAY_TIMEOUT or self.clearWayStuckTime > CatchCombinePipeTask.CLEAR_WAY_STUCK_TIME then
        reason = "reverse blocked"
    end
    if reason ~= nil then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_COMBINEINFO, "CatchCombinePipeTask: reversing stopped after %.0f m (%s), waiting for the harvester to pass", reversed, reason)
        self.passByTime = 0
        self.state = CatchCombinePipeTask.STATE_WAIT_PASS_BY
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return
    end
    self.vehicle.ad.specialDrivingModule:driveReverse(dt, CatchCombinePipeTask.CLEAR_WAY_REVERSE_SPEED, 1, self.vehicle.ad.trailerModule:canBeHandledInReverse())
end

function CatchCombinePipeTask:updateWaitPassBy(dt)
    self.vehicle.ad.specialDrivingModule:stopVehicle()
    self.vehicle.ad.specialDrivingModule:update(dt)
    self.passByTime = self.passByTime + dt
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local _, _, diffZ = AutoDrive.worldToLocal(self.combine, x, y, z, self.combine.ad.ADRootNode)
    local passed = diffZ < -CatchCombinePipeTask.PASS_BY_BEHIND
    if passed or (not self:isInCombinePath() and self.passByTime > 2000) or self.passByTime > CatchCombinePipeTask.PASS_BY_TIMEOUT then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_COMBINEINFO, "CatchCombinePipeTask: %s, new path search",
            passed and "the harvester has passed" or (self.passByTime > CatchCombinePipeTask.PASS_BY_TIMEOUT and "waited too long" or "out of its way"))
        self.waitForCheckTimer.elapsedTime = 4000
        self.state = CatchCombinePipeTask.STATE_DELAY_PATHPLANNING
    end
end

function CatchCombinePipeTask:finished(propagate)
    CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:finished()")
    self.vehicle.ad.taskModule:setCurrentTaskFinished(propagate)
end

function CatchCombinePipeTask:startNewPathFinding()
    CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:startNewPathFinding()")
    local pipeChasePos, pipeChaseSide = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getPipeChasePosition(true)
    local x, _, z = getWorldTranslation(self.combine.components[1].node)
    local targetFieldId = g_farmlandManager:getFarmlandIdAtWorldPosition(pipeChasePos.x, pipeChasePos.z)
    local combineFieldId = g_farmlandManager:getFarmlandIdAtWorldPosition(x, z)

    -- Only chase the rear on low fill levels of the combine. This should prevent getting into unneccessarily tight spots for the final approach to the pipe.
    -- Also for small fields, there is often no purpose in chasing so far behind the combine as it will already start a turn soon

    local cfillLevel, cfillCapacity, _, cfillFreeCapacity = AutoDrive.getObjectFillLevels(self.combine)
    local cFillRatio = cfillCapacity > 0 and cfillLevel / cfillCapacity or 0

    if cFillRatio > 0.91 or cfillFreeCapacity < 0.1 then
        CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:startNewPathFinding() - Combine is almost full - dont chase for active unloading anymore")
        self:finished(ADTaskModule.DONT_PROPAGATE)
        self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:setToWaitForCall()
        return false
    end

    self.newPathFindingCounter = self.newPathFindingCounter + 1 -- used to prevent deadlock

    if self.combine.ad.isChopper or (pipeChaseSide ~= AutoDrive.CHASEPOS_REAR or (targetFieldId == combineFieldId and cFillRatio <= 0.85)) then
    -- if self.combine:getIsBufferCombine() or (pipeChaseSide ~= AutoDrive.CHASEPOS_REAR and targetFieldId == combineFieldId and cFillRatio <= 0.85) then
        -- is chopper or chase not rear and harvester on correct field and filled < 85% - i.e. combine pipe not in fruit
        CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:startNewPathFinding() - chase pos looks good - calculate path to it...")
        self.vehicle.ad.pathFinderModule:reset()
        self.vehicle.ad.pathFinderModule:startPathPlanningToPipe(self.combine, false)
        -- use false to enable pathfinder fallback
        -- no clean route in strict: drive part of the way, this task plans again on arrival
        self.vehicle.ad.pathFinderModule.allowPartialPath = true
        self.combinesStartLocation = {}
        self.combinesStartLocation.x, self.combinesStartLocation.y, self.combinesStartLocation.z = getWorldTranslation(self.combine.components[1].node)
        return true
    else
        CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:startNewPathFinding() - chase pos looks bad, is not on the same field or combine's fill level is approaching limit - aborting for now")
        self.waitForCheckTimer:timer(false)
    end
    return false
end

function CatchCombinePipeTask:getExcludedVehiclesForCollisionCheck()
    local excludedVehicles = {}
    if self.state == CatchCombinePipeTask.STATE_DRIVING then
        table.insert(excludedVehicles, self.combine:getRootVehicle())
    end
    return excludedVehicles
end

function CatchCombinePipeTask:setStateNames()
    if self.statesToNames == nil then
        self.statesToNames = {}
        for name, id in pairs(CatchCombinePipeTask) do
            if string.sub(name, 1, 6) == "STATE_" then
                self.statesToNames[id] = name
            end
        end
    end
end

function CatchCombinePipeTask:getStateName(state)
    local requestedState = state
    if requestedState == nil then
        requestedState = self.state
    end
    if requestedState == nil then
        Logging.error("[AD] CatchCombinePipeTask: Could not find name for state ->%s<- !", tostring(requestedState))
    end
    return self.statesToNames[requestedState] or ""
end

function CatchCombinePipeTask:resetAllTimers()
    -- self.stuckTimer:timer(false) -- stuckTimer reset by speed changes
    self.reverseTimer:timer(false)
    self.waitTimer:timer(false)
    self.waitForCheckTimer:timer(false)
end

function CatchCombinePipeTask:getI18nInfo()
    local text = "$l10n_AD_task_catch_up_with_combine;"
    if self.state == CatchCombinePipeTask.STATE_PATHPLANNING then
        local actualState, maxStates, steps, max_pathfinder_steps = self.vehicle.ad.pathFinderModule:getCurrentState()
        text = text .. " - " .. "$l10n_AD_task_pathfinding;" .. string.format(" %d / %d - %d / %d", actualState, maxStates, steps, max_pathfinder_steps)
    elseif self.state == CatchCombinePipeTask.STATE_DELAY_PATHPLANNING then
        text = text .. " - " .. "$l10n_AD_task_unload_area_in_fruit;"
    elseif self.state == CatchCombinePipeTask.STATE_REVERSING then
        text = text .. " - " .. "$l10n_AD_task_reversing_from_combine;"
    elseif self.state == CatchCombinePipeTask.STATE_WAIT_BEFORE_FINISH then
        text = text .. " - " .. "$l10n_AD_task_waiting_for_room;"
    end
    return text 
end

function CatchCombinePipeTask.debugMsg(vehicle, debugText, ...)
    if CatchCombinePipeTask.debug == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    else
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_COMBINEINFO, debugText, ...)
    end
end
