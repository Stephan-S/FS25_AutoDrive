UnloadAtDestinationTask = ADInheritsFrom(AbstractTask)

UnloadAtDestinationTask.STATE_PATHPLANNING = 1
UnloadAtDestinationTask.STATE_DRIVING = 2
UnloadAtDestinationTask.STATE_WAIT_FOR_AL_UNLOAD = 3
UnloadAtDestinationTask.WAIT_FOR_BALE_UNLOAD = 20000
UnloadAtDestinationTask.BALE_UNLOAD_DISTANCE = 5

function UnloadAtDestinationTask:new(vehicle, destinationID)
    local o = UnloadAtDestinationTask:create()
    o.taskType = "UnloadAtDestinationTask"
    o.vehicle = vehicle
    o.destinationID = destinationID
    o.isContinued = false
    o.waitForALUnloadTimer = AutoDriveTON:new()
    o.waitForALUnload = false
    o.waitForBaleUnloadTimer = AutoDriveTON:new()
    o.isReverseTriggerReached = false
    o.trailers = nil
    return o
end

function UnloadAtDestinationTask:setUp()
    if self.vehicle.spec_locomotive and self.vehicle.ad and self.vehicle.ad.trainModule then
        self.state = UnloadAtDestinationTask.STATE_DRIVING
        self.vehicle.ad.trainModule:setPathTo(self.destinationID)
    elseif ADGraphManager:getDistanceFromNetwork(self.vehicle) > 30 then
        self.state = UnloadAtDestinationTask.STATE_PATHPLANNING
        if self.vehicle.ad.stateModule:getCanRestartHelper() then
            if self.vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER then
                self.vehicle.ad.pathFinderModule:reset()
                self.vehicle.ad.pathFinderModule:startPathPlanningToWayPoint(self.vehicle.ad.stateModule:getFirstWayPoint(), self.destinationID)
            else
                self.vehicle.ad.pathFinderModule:reset()
                self.vehicle.ad.pathFinderModule:startPathPlanningToWayPoint(self.vehicle.ad.stateModule:getSecondWayPoint(), self.destinationID)
            end
        else
            self.vehicle.ad.pathFinderModule:reset()
            self.vehicle.ad.pathFinderModule:startPathPlanningToNetwork(self.destinationID)
        end
    else
        self.state = UnloadAtDestinationTask.STATE_DRIVING
        self.vehicle.ad.drivePathModule:setPathTo(self.destinationID)
    end
    self.trailers, _ = AutoDrive.getAllUnits(self.vehicle)
    self.vehicle.ad.trailerModule:reset()
    self.waitForALUnload = false
    self.waitForBaleUnload = false
    self.baleTrailer = nil
    for _, trailer in pairs(self.trailers) do
        local spec = trailer.spec_baleLoader
        if spec and trailer.startAutomaticBaleUnloading then
            self.baleTrailer = trailer
            break
        end
    end
    self.baleUnloadForwardsTarget = nil
    self.isReverseTriggerReached = false
end

function UnloadAtDestinationTask:update(dt)
    self.vehicle.ad.specialDrivingModule.motorShouldNotBeStopped = false
    if self.state == UnloadAtDestinationTask.STATE_PATHPLANNING then
        if self.vehicle.ad.pathFinderModule:hasFinished() then
            self.wayPoints = self.vehicle.ad.pathFinderModule:getPath()
            if self.wayPoints == nil or #self.wayPoints == 0 then
                if self.vehicle.ad.pathFinderModule:isTargetBlocked() then
                    -- If the selected field exit isn't reachable, try the closest one
                    self.vehicle.ad.pathFinderModule:reset()
                    self.vehicle.ad.pathFinderModule:startPathPlanningToNetwork(self.vehicle.ad.stateModule:getSecondWayPoint())
                elseif self.vehicle.ad.pathFinderModule:timedOut() or self.vehicle.ad.pathFinderModule:isBlocked() then
                    -- Add some delay to give the situation some room to clear itself
                    self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:notifyAboutFailedPathfinder()
                    self.vehicle.ad.pathFinderModule:reset()
                    self.vehicle.ad.pathFinderModule:startPathPlanningToNetwork(self.vehicle.ad.stateModule:getSecondWayPoint())
                    self.vehicle.ad.pathFinderModule:addDelayTimer(10000)
                else
                    self.vehicle.ad.pathFinderModule:reset()
                    self.vehicle.ad.pathFinderModule:startPathPlanningToNetwork(self.vehicle.ad.stateModule:getSecondWayPoint())
                end

                Logging.error("[AutoDrive] Could not calculate path - shutting down")
                self.vehicle.ad.taskModule:abortAllTasks()
                self.vehicle:stopAutoDrive()
                AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_cannot_find_path;", 5000, self.vehicle.ad.stateModule:getName())
            else
                self.vehicle.ad.drivePathModule:setWayPoints(self.wayPoints)
                --self.vehicle.ad.drivePathModule:appendPathTo(self.wayPoints[#self.wayPoints], self.destinationID)
                self.state = UnloadAtDestinationTask.STATE_DRIVING
            end
        else
            self.vehicle.ad.pathFinderModule:update(dt)
            self.vehicle.ad.specialDrivingModule:stopVehicle()
            self.vehicle.ad.specialDrivingModule:update(dt)
        end
    elseif self.state == UnloadAtDestinationTask.STATE_DRIVING then
        if not self.isContinued then
            self.vehicle.ad.trailerModule:update(dt)
        end
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "UnloadAtDestinationTask:update isTargetReached")
            if not self.vehicle.ad.trailerModule:isActiveAtTrigger() then
                --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "UnloadAtDestinationTask:update isTargetReached isActiveAtTrigger")
                AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, true)
                local fillLevel = AutoDrive.getAllFillLevels(self.trailers)
                if
                    fillLevel < 0.1
                    or self.isContinued
                    or (
                        ((AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYDELIVER or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders")) 
                        and (not (self.vehicle.ad.drivePathModule:getIsReversing() and self.vehicle.ad.trailerModule:getBunkerTrigger() ~= nil))
                        and self.vehicle.ad.trailerModule:getHasAL() ~= true
                    )
                    then
                    --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "UnloadAtDestinationTask:update fillLevel < 0.1")
                    AutoDrive.setAugerPipeOpen(self.trailers, false)
                    self:finished()
                else
                   -- AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "UnloadAtDestinationTask:update Wait at unload point until unloaded somehow")
                    -- Wait at unload point until unloaded somehow
                    --Keep motor running. Maybe we could check if the current trailer need Power to be unloaded or not
                    self.vehicle.ad.specialDrivingModule.motorShouldNotBeStopped = true

                    self.vehicle.ad.specialDrivingModule:stopVehicle()
                    self.vehicle.ad.specialDrivingModule:update(dt)
                    if self.vehicle.ad.trailerModule:getHasAL() == true then
                    	-- AutoLoad
                        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "UnloadAtDestinationTask:update unloadALAll start")
                        if AutoDrive.getSetting("ALUnloadWaitTime", self.vehicle) > 0 and AutoDrive.getSetting("ALUnload", self.vehicle) > 0 then
                            -- wait only if unload is not disabled and wait time > 0
                            self.waitForALUnload = true
                            self.waitForALUnloadTimer:timer(false)
                            self.state = UnloadAtDestinationTask.STATE_WAIT_FOR_AL_UNLOAD
                        end
                        AutoDrive:unloadALAll(self.vehicle)
                    elseif self.baleTrailer and self.baleTrailer.startAutomaticBaleUnloading then
                        -- bale unload
                        local spec = self.baleTrailer.spec_baleLoader
                        if spec.emptyState == BaleLoader.EMPTY_NONE then
                            self.waitForBaleUnload = true
                            self.baleTrailer:startAutomaticBaleUnloading()
                            self.state = UnloadAtDestinationTask.STATE_WAIT_FOR_AL_UNLOAD
                        end
                    end
                end
            else
                --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "UnloadAtDestinationTask:update isTargetReached NOT isActiveAtTrigger")
                if self.vehicle.ad.trailerModule:isUnloadingToBunkerSilo() then
                    --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "UnloadAtDestinationTask:update isTargetReached NOT isActiveAtTrigger isUnloadingToBunkerSilo")
                    self.vehicle.ad.drivePathModule:update(dt)
                else
                    --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "UnloadAtDestinationTask:update isTargetReached NOT isActiveAtTrigger stopVehicle")
                    self.vehicle.ad.specialDrivingModule.motorShouldNotBeStopped = true
                    self.vehicle.ad.specialDrivingModule:stopVehicle()
                    self.vehicle.ad.specialDrivingModule:update(dt)
                end
            end
        else
            if self.vehicle.ad.trailerModule:isActiveAtTrigger() then
                --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "UnloadAtDestinationTask:update isActiveAtTrigger")
                if self.vehicle.ad.trailerModule:isUnloadingToBunkerSilo() then
                    --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "UnloadAtDestinationTask:update isActiveAtTrigger isUnloadingToBunkerSilo")
                    self.vehicle.ad.drivePathModule:update(dt)
                else
                    --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "UnloadAtDestinationTask:update isActiveAtTrigger stopVehicle")
                    self.isReverseTriggerReached = self.vehicle.ad.drivePathModule:getIsReversing()
                    self.vehicle.ad.specialDrivingModule:stopVehicle()
                    self.vehicle.ad.specialDrivingModule:update(dt)
                end
            else
                --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "UnloadAtDestinationTask:update isActiveAtTrigger drivePathModule:update")
                if self.isReverseTriggerReached then
                    self:finished()
                else
                    self.vehicle.ad.drivePathModule:update(dt)
                end
            end
        end
    else -- UnloadAtDestinationTask.STATE_WAIT_FOR_AL_UNLOAD
        local waitForALUnloadTime = AutoDrive.getSetting("ALUnloadWaitTime", self.vehicle)
        if self.waitForALUnload and (waitForALUnloadTime >= 0 and self.waitForALUnloadTimer:timer(self.waitForALUnload, waitForALUnloadTime, dt)) or self.isContinued then
            -- used to wait for AutoLoader to unload
            self.waitForALUnloadTimer:timer(false)
            AutoDrive.setAugerPipeOpen(self.trailers, false)
            self:finished()
            self.waitForALUnload = false
        end
        if self.waitForALUnload then
            -- AutoLoad - stop driving for the wait time, let autoloaded objects disappear
            self.vehicle.ad.specialDrivingModule:stopVehicle()
            self.vehicle.ad.specialDrivingModule:update(dt)
            return
        end
        if self.waitForBaleUnload then
            if self:isBaleUnloadFinished(dt) then
                self.waitForBaleUnload = false
                self:finished()
            else
                -- avoid further actions
                return
            end
        end
    end
end

function UnloadAtDestinationTask:abort()
    AutoDrive.resetFoldState(self.vehicle)
    AutoDrive.closeAllCurtains(self.trailers, true) -- close curtain at UAL trailers
end

function UnloadAtDestinationTask:continue()
    if not self.waitForBaleUnload then
        self.vehicle.ad.trailerModule:stopUnloading()
        self.isContinued = true
    end
    AutoDrive.resetFoldState(self.vehicle)
    AutoDrive.closeAllCurtains(self.trailers, true) -- close curtain at UAL trailers
end

function UnloadAtDestinationTask:finished()
    AutoDrive.resetFoldState(self.vehicle)
    AutoDrive.closeAllCurtains(self.trailers, true) -- close curtain at UAL trailers
    self.vehicle.ad.taskModule:setCurrentTaskFinished()
end

function UnloadAtDestinationTask:isBaleUnloadFinished(dt)
    if self.baleTrailer and self.baleTrailer.getIsAutomaticBaleUnloadingInProgress then
        local unloading = self.baleTrailer:getIsAutomaticBaleUnloadingInProgress()
        if self.baleUnloadForwardsTarget == nil then
            local x, y, z = AutoDrive.localToWorld(self.vehicle, 0, 0 , UnloadAtDestinationTask.BALE_UNLOAD_DISTANCE)
            self.baleUnloadForwardsTarget = {x=x, y=y, z=z}
        end
        if self.waitForBaleUnloadTimer:timer(true, UnloadAtDestinationTask.WAIT_FOR_BALE_UNLOAD, dt) then
            -- wait for pusher
            local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
            local distance = MathUtil.vector2Length(x - self.baleUnloadForwardsTarget.x, z - self.baleUnloadForwardsTarget.z)
            if distance >= 1 then
                -- move some meters
                self.vehicle.ad.specialDrivingModule:driveToPoint(dt, self.baleUnloadForwardsTarget, 1, false, 0.5, 1)
                return false
            end
        end
        if not unloading then
            local spec = self.baleTrailer.spec_baleLoader
            -- as long as Giants is not able to synchronize their machines, there will be strange appearances
            if spec and not spec.isInWorkPosition then
                self.baleTrailer:doStateChange(BaleLoader.CHANGE_BUTTON_WORK_TRANSPORT)
                return false
            end
            if spec and spec.isInWorkPosition then
                self.baleTrailer:doStateChange(BaleLoader.CHANGE_BUTTON_WORK_TRANSPORT)
                return true
            end
        end
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return false
    end
    return true
end

function UnloadAtDestinationTask:getI18nInfo()
    if self.state == UnloadAtDestinationTask.STATE_PATHPLANNING then
        local actualState, maxStates, steps, max_pathfinder_steps = self.vehicle.ad.pathFinderModule:getCurrentState()
        return "$l10n_AD_task_pathfinding;" .. string.format(" %d / %d - %d / %d", actualState, maxStates, steps, max_pathfinder_steps)
    else
        return "$l10n_AD_task_drive_to_unload_point;"
    end
end
