ExitFieldTask = ADInheritsFrom(AbstractTask)
ExitFieldTask.debug = false

ExitFieldTask.STATE_PATHPLANNING = 1
ExitFieldTask.STATE_DRIVING = 2
ExitFieldTask.STATE_DELAY_PATHPLANNING = 3
ExitFieldTask.STATE_ESCAPE_DRIVING = 4
ExitFieldTask.STATE_FINISHED = 5
ExitFieldTask.STATE_ESCAPE_PLANNING = 6

ExitFieldTask.STRATEGY_START = 0
ExitFieldTask.STRATEGY_BEHIND_START = 1
ExitFieldTask.STRATEGY_CLOSEST = 2

function ExitFieldTask:new(vehicle, combine)
    local o = ExitFieldTask:create()
    o.vehicle = vehicle
    o.combine = combine
    o.trailers = nil
    o.failedPathFinder = 0
    o.waitForCheckTimer = AutoDriveTON:new()
    return o
end

function ExitFieldTask:setUp()
    self.state = ExitFieldTask.STATE_DELAY_PATHPLANNING
    self.nextExitStrategy = AutoDrive.getSetting("exitField", self.vehicle)
    self.exitCandidateIndex = 1
    self.exitCandidates = nil
    self.escapeCourseTried = false
    self.trailers, _ = AutoDrive.getAllUnits(self.vehicle)
    AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, false)

end

-- Last-resort free-space exit when no reachable network/Fanglinie entry remains. The normal
-- path must target the network first; otherwise TARGET_OFF_FIELD would always choose the nearest
-- arbitrary field edge and bypass recorded field exits.
function ExitFieldTask:tryEscapeCourse()
    if self.escapeCourseTried then
        return false
    end
    self.escapeCourseTried = true
    ExitFieldTask.debugMsg(self.vehicle, "ExitFieldTask - network entries failed, starting Dijkstra fallback")
    self.escapeJob = ADEscapeCourseGenerator.begin(self.vehicle, ADEscapeCourseGenerator.TARGET_OFF_FIELD, {
        exclusionZone = AutoDrive.getCombineExclusionZone(self.combine),
        combine = self.combine
    })
    self.state = ExitFieldTask.STATE_ESCAPE_PLANNING
    return true
end

function ExitFieldTask:update(dt)
    if self.state == ExitFieldTask.STATE_PATHPLANNING then
        if self.vehicle.ad.pathFinderModule:hasFinished() then
            self.wayPoints = self.vehicle.ad.pathFinderModule:getPath()
            if self.wayPoints == nil or #self.wayPoints == 0 then
                self.failedPathFinder = self.failedPathFinder + 1
                if self.nextExitStrategy == ExitFieldTask.STRATEGY_CLOSEST and self:selectNextClosestCandidate() then
                    self:startPathPlanning()
                elseif self:tryEscapeCourse() then
                    -- free-space escape course set, drive it and replan afterwards
                    return
                elseif self.failedPathFinder > 5 then
                    self.failedPathFinder = 0
                    self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:notifyAboutFailedPathfinder()
                    self:selectNextStrategy()
                elseif self.vehicle.ad.pathFinderModule:isTargetBlocked() then
                    -- If the selected field exit isn't reachable, try the next strategy and restart without delay
                    self:startPathPlanning()
                elseif self.vehicle.ad.pathFinderModule:timedOut() or self.vehicle.ad.pathFinderModule:isBlocked() then
                    -- Add some delay to give the situation some room to clear itself
                    self:startPathPlanning()
                    self.vehicle.ad.pathFinderModule:addDelayTimer(10000)
                else
                    self:startPathPlanning()
                    self.vehicle.ad.pathFinderModule:addDelayTimer(10000)
                end
            else
                self.vehicle.ad.drivePathModule:setWayPoints(self.wayPoints)
                self.state = ExitFieldTask.STATE_DRIVING
            end
        else
            self.vehicle.ad.pathFinderModule:update(dt)
            self.vehicle.ad.specialDrivingModule:stopVehicle()
            self.vehicle.ad.specialDrivingModule:update(dt)
        end
    elseif self.state == ExitFieldTask.STATE_DELAY_PATHPLANNING then
        ExitFieldTask.debugMsg(self.vehicle, "ExitFieldTask:update - STATE_DELAY_PATHPLANNING")
        if self.waitForCheckTimer:timer(true, 1000, dt) then
            if self:startPathPlanning() then
                self.vehicle.ad.pathFinderModule:addDelayTimer(6000)
                self.state = ExitFieldTask.STATE_PATHPLANNING
                return
            end
        end
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return
    elseif self.state == ExitFieldTask.STATE_DRIVING then
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            self.state = ExitFieldTask.STATE_FINISHED
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end
    elseif self.state == ExitFieldTask.STATE_ESCAPE_PLANNING then
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        if self.escapeJob ~= nil then
            self.escapeJob:update()
            if self.escapeJob:isFinished() then
                local escapeCourse = self.escapeJob:getCourse()
                self.escapeJob = nil
                if escapeCourse ~= nil then
                    ExitFieldTask.debugMsg(self.vehicle, "ExitFieldTask - driving escape course with %d waypoints", #escapeCourse)
                    self.vehicle.ad.drivePathModule:setWayPoints(escapeCourse)
                    self.state = ExitFieldTask.STATE_ESCAPE_DRIVING
                else
                    -- no escape course found - continue with the next exit strategy
                    self:selectNextStrategy()
                    self.state = ExitFieldTask.STATE_DELAY_PATHPLANNING
                end
            end
        else
            self:selectNextStrategy()
            self.state = ExitFieldTask.STATE_DELAY_PATHPLANNING
        end
    elseif self.state == ExitFieldTask.STATE_ESCAPE_DRIVING then
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            -- off the field now (or as close as the course got us) - replan from this position
            self.exitCandidates = nil
            self.exitCandidateIndex = 1
            self.failedPathFinder = 0
            self.state = ExitFieldTask.STATE_DELAY_PATHPLANNING
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end
    elseif self.state == ExitFieldTask.STATE_FINISHED then
        self:finished()
    end
end

function ExitFieldTask:abort()
end

function ExitFieldTask:finished()
    self.vehicle.ad.taskModule:setCurrentTaskFinished()
end

function ExitFieldTask:startPathPlanning()
    ExitFieldTask.debugMsg(self.vehicle, "ExitFieldTask:startPathPlanning")
    local closest, closestDistance = self.vehicle:getClosestWayPoint()
    if self.nextExitStrategy == ExitFieldTask.STRATEGY_CLOSEST then
        if self.exitCandidates == nil then
            self.exitCandidates = ADGraphManager:getReachableNetworkEntryCandidates(self.vehicle, self.vehicle.ad.stateModule:getSecondWayPoint(), ADGraphManager.NETWORK_ENTRY_SEARCH_RADIUS, 12, true, AutoDrive.getCombineExclusionZone(self.combine))
        end
        closest = self.exitCandidates[self.exitCandidateIndex]
        if closest == nil then
            self:selectNextStrategy()
            return self:startPathPlanning()
        end
        local vehicleX, _, vehicleZ = getWorldTranslation(self.vehicle.components[1].node)
        local closestNode = ADGraphManager:getWayPointById(closest)
        closestDistance = MathUtil.vector2Length(closestNode.x - vehicleX, closestNode.z - vehicleZ)
        local wayPoints = ADGraphManager:pathFromTo(closest, self.vehicle.ad.stateModule:getSecondWayPoint())
        if wayPoints ~= nil and #wayPoints > 1 then
            if closestDistance > ADGraphManager.MAX_DIRECT_NETWORK_ENTRY_DISTANCE then
                -- Initiate pathfinder unless the vehicle is already directly on the network.
                local vecToNextPoint = {x = wayPoints[2].x - closestNode.x, z = wayPoints[2].z - closestNode.z}
                self.vehicle.ad.pathFinderModule:reset()
                self.vehicle.ad.pathFinderModule:startPathPlanningTo(closestNode, vecToNextPoint)
                return true
            else
                -- close to network, set task finished
                self:finished()
            end
        else
            AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.WARN, "$l10n_AD_Driver_of; %s $l10n_AD_cannot_find_path;", 5000, self.vehicle.ad.stateModule:getName())
            self.vehicle.ad.taskModule:abortAllTasks()
            self.vehicle.ad.taskModule:addTask(StopAndDisableADTask:new(self.vehicle))
        end
    else
        local targetNode = ADGraphManager:getWayPointById(self.vehicle.ad.stateModule:getFirstWayPoint())
        local wayPoints = ADGraphManager:pathFromTo(self.vehicle.ad.stateModule:getFirstWayPoint(), self.vehicle.ad.stateModule:getSecondWayPoint())
        if wayPoints ~= nil and #wayPoints > 1 then
            local vecToNextPoint = {x = wayPoints[2].x - targetNode.x, z = wayPoints[2].z - targetNode.z}
            if AutoDrive.getSetting("exitField", self.vehicle) == 1 and #wayPoints > 6 then
                targetNode = wayPoints[5]
                vecToNextPoint = {x = wayPoints[6].x - targetNode.x, z = wayPoints[6].z - targetNode.z}
            end
            self.vehicle.ad.pathFinderModule:reset()
            self.vehicle.ad.pathFinderModule:startPathPlanningTo(targetNode, vecToNextPoint)
            return true
        else
            AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.WARN, "$l10n_AD_Driver_of; %s $l10n_AD_cannot_find_path;", 5000, self.vehicle.ad.stateModule:getName())
            self.vehicle.ad.taskModule:abortAllTasks()
            self.vehicle.ad.taskModule:addTask(StopAndDisableADTask:new(self.vehicle))
        end
    end
    return false
end

function ExitFieldTask:selectNextStrategy()
    self.nextExitStrategy = (self.nextExitStrategy + 1) % (ExitFieldTask.STRATEGY_CLOSEST + 1)
    self.exitCandidateIndex = 1
    self.exitCandidates = nil
end

function ExitFieldTask:selectNextClosestCandidate()
    self.exitCandidateIndex = self.exitCandidateIndex + 1
    return self.exitCandidates ~= nil and self.exitCandidates[self.exitCandidateIndex] ~= nil
end

function ExitFieldTask:continue()
end

function ExitFieldTask:getInfoText()
    if self.state == ExitFieldTask.STATE_PATHPLANNING then
        local actualState, maxStates = self.vehicle.ad.pathFinderModule:getCurrentState()
        return g_i18n:getText("AD_task_pathfinding") .. string.format(" %d / %d ", actualState, maxStates)
    else
        return g_i18n:getText("AD_task_exiting_field")
    end
end

function ExitFieldTask:getI18nInfo()
    if self.state == ExitFieldTask.STATE_PATHPLANNING then
        local actualState, maxStates, steps, max_pathfinder_steps = self.vehicle.ad.pathFinderModule:getCurrentState()
        return "$l10n_AD_task_pathfinding;" .. string.format(" %d / %d - %d / %d", actualState, maxStates, steps, max_pathfinder_steps)
    else
        return "$l10n_AD_task_exiting_field;"
    end
end

function ExitFieldTask.debugMsg(vehicle, debugText, ...)
    if ExitFieldTask.debug == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    else
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_COMBINEINFO, debugText, ...)
    end
end
