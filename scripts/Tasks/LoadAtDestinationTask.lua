LoadAtDestinationTask = ADInheritsFrom(AbstractTask)

LoadAtDestinationTask.STATE_PATHPLANNING = 1
LoadAtDestinationTask.STATE_DRIVING = 2

LoadAtDestinationTask.LOAD_RETRY_TIME = 3000

function LoadAtDestinationTask:new(vehicle, destinationID)
    local o = LoadAtDestinationTask:create()
    o.vehicle = vehicle
    o.destinationID = destinationID
    o.trailers = nil
    o.waitForALLoadTimer = AutoDriveTON:new()
    o.activatedUALLoading = false
    o.isReverseTriggerReached = false
    return o
end

function LoadAtDestinationTask:setUp()
    if self.vehicle.spec_locomotive and self.vehicle.ad and self.vehicle.ad.trainModule then
        self.state = LoadAtDestinationTask.STATE_DRIVING
        self.vehicle.ad.trainModule:setPathTo(self.destinationID)
    elseif ADGraphManager:getDistanceFromNetwork(self.vehicle) > 30 then
        self.state = LoadAtDestinationTask.STATE_PATHPLANNING
        self.vehicle.ad.pathFinderModule:reset()
        self.vehicle.ad.pathFinderModule:startPathPlanningToNetwork(self.destinationID)
    else
        self.state = LoadAtDestinationTask.STATE_DRIVING
        self.vehicle.ad.drivePathModule:setPathTo(self.destinationID)
    end
    if self.loadRetryTimer == nil then
        self.loadRetryTimer = AutoDriveTON:new()
    else
        self.loadRetryTimer:timer(false)      -- clear timer
    end
    self.trailers, _ = AutoDrive.getAllUnits(self.vehicle)
    self.vehicle.ad.trailerModule:reset()
    self.activatedUALLoading = false
    self.fillLevel = 0
    self.fillFreeCapacity = 0
    self.lastFillLevel = 0
    self.filledToUnload = false
    self.retryTime = LoadAtDestinationTask.LOAD_RETRY_TIME
    self.isReverseTriggerReached = false
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "LoadAtDestinationTask:setUp end self.state %s", tostring(self.state))
end

function LoadAtDestinationTask:update(dt)
    if self.state == LoadAtDestinationTask.STATE_PATHPLANNING then
        if self.vehicle.ad.pathFinderModule:hasFinished() then
            self.wayPoints = self.vehicle.ad.pathFinderModule:getPath()
            if self.wayPoints == nil or #self.wayPoints == 0 then
                Logging.error("[AutoDrive] Could not calculate path - shutting down")
                self.vehicle.ad.taskModule:abortAllTasks()
                self.vehicle:stopAutoDrive()
                AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_cannot_find_path;", 5000, self.vehicle.ad.stateModule:getName())
            else
                self.vehicle.ad.drivePathModule:setWayPoints(self.wayPoints)
                --self.vehicle.ad.drivePathModule:appendPathTo(self.wayPoints[#self.wayPoints], self.destinationID)
                self.state = LoadAtDestinationTask.STATE_DRIVING
            end
        else
            self.vehicle.ad.pathFinderModule:update(dt)
            self.vehicle.ad.specialDrivingModule:stopVehicle()
            self.vehicle.ad.specialDrivingModule:update(dt)
        end
    else
        -- STATE_DRIVING
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "LoadAtDestinationTask:update self.state %s", tostring(self.state))
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            --Check if we have actually loaded / tried to load
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "LoadAtDestinationTask:update isTargetReached")
            AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, true)

            if (self.vehicle.ad.stateModule:getCanRestartHelper() and self.vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER) then
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "LoadAtDestinationTask:update stopAutoDrive")
                -- pass over to CP
                self.vehicle:stopAutoDrive()
            else
                self.vehicle.ad.specialDrivingModule:stopVehicle()
                self.vehicle.ad.specialDrivingModule:update(dt)
                -- local waitForALUnloadTime = AutoDrive.getSetting("ALUnloadWaitTime", self.vehicle)

                if self.vehicle.ad.hasAL then
                    -- UAL special handling - loading only possible if vehicle not moving -> self.lastSpeedReal < 0.0005
                    -- assume no influence on aPalletAutoLoader
                    if not self.activatedUALLoading then
                        if self.vehicle.lastSpeedReal < 0.0005 then
                            -- start loading
                            self.activatedUALLoading = true
                            self.ualIterations = 0
                            self.fillLevel, _, _, _ = AutoDrive.getAllFillLevels(self.trailers)
                            self.lastFillLevel = self.fillLevel
                            self.numberOfSelectedFillTypes = #self.vehicle.ad.stateModule:getSelectedFillTypes()
                            AutoDrive.activateALTrailers(self.vehicle, self.trailers)
                        end
                        self.retryTime = LoadAtDestinationTask.LOAD_RETRY_TIME
                    end
                else
                    if self.vehicle.ad.trailerModule:wasAtSuitableTrigger() or ((AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYPICKUP or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders")) then
                        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "LoadAtDestinationTask:update wasAtSuitableTrigger -> self:finished")
                        self:finished()
                    end
                    self.retryTime = LoadAtDestinationTask.LOAD_RETRY_TIME
                end
                if self.vehicle.ad.trailerModule:isActiveAtTrigger() then
                    -- update to catch if no longer active at trigger
                    self.vehicle.ad.trailerModule:update(dt)
                else
                    -- try to load somehow while standing at destination
                    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "LoadAtDestinationTask:update not isActiveAtTrigger")
                    self.loadRetryTimer:timer(true, self.retryTime, dt)
                    if self.loadRetryTimer:done() then
                        -- performance: avoid to initiate loading while standing at destination to often
                        self.loadRetryTimer:timer(false)      -- clear timer
                        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "LoadAtDestinationTask:update try loading somehow")

                        self.fillLevel, _, self.filledToUnload, self.fillFreeCapacity = AutoDrive.getAllFillLevels(self.trailers)
                        if self.activatedUALLoading == true then
                            -- AL loading activated
                            if self.numberOfSelectedFillTypes > 1 then
                                -- multiple selected
                                if self.lastFillLevel ~= self.fillLevel and self.fillFreeCapacity > 0 and not self.filledToUnload then
                                    -- objects loaded and not full - set retryTime to AL wait time, min LOAD_RETRY_TIME to try load more objects of the selected fillType
                                    self.lastFillLevel = self.fillLevel
                                    self.retryTime = math.max(LoadAtDestinationTask.LOAD_RETRY_TIME, AutoDrive.getSetting("ALUnloadWaitTime", self.vehicle))
                                else
                                    -- no objects loaded - continue with next fillType or end
                                    local objectsAvailable = AutoDrive.objectsToLoadAvailable(self.vehicle, self.trailers)
                                    if objectsAvailable and (self.ualIterations < self.numberOfSelectedFillTypes) then
                                        self.ualIterations = self.ualIterations + 1
                                        self.vehicle.ad.stateModule:nextSelectedFillType()
                                        self.retryTime = LoadAtDestinationTask.LOAD_RETRY_TIME
                                    elseif self.filledToUnload then
                                        self:finished()
                                    end
                                end
                            elseif self.filledToUnload then
                                self:finished()
                            end
                        else
                            self.vehicle.ad.trailerModule:update(dt)
                            if not self.vehicle.ad.trailerModule:isActiveAtTrigger() then
                                -- check fill levels only if not still filling something
                                if self.filledToUnload then
                                    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "LoadAtDestinationTask:update leftCapacity <= -> self:finished")
                                    self:finished()
                                end
                            end
                        end
                    end
                end
            end
        else
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "LoadAtDestinationTask:update NOT isTargetReached")

            if not (self.vehicle.ad.stateModule:getCanRestartHelper() and self.vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER) then
                -- need to try loading if CP is not active
                self.vehicle.ad.trailerModule:update(dt)
            end
            if self.vehicle.ad.trailerModule:isActiveAtTrigger() then
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "LoadAtDestinationTask:update 2 isActiveAtTrigger -> specialDrivingModule:stopVehicle")
                self.isReverseTriggerReached = self.vehicle.ad.drivePathModule:getIsReversing()
                self.vehicle.ad.specialDrivingModule:stopVehicle()
                self.vehicle.ad.specialDrivingModule:update(dt)
            else
                if self.vehicle.ad.trailerModule:wasAtSuitableTrigger() and AutoDrive.checkForContinueOnEmptyLoadTrigger(self.vehicle) then
                    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "LoadAtDestinationTask:update not isActiveAtTrigger + wasAtSuitableTrigger + should continue -> finished")
                    self:finished()
                    return
                end
                if self.isReverseTriggerReached then
                    self.fillLevel, _, self.filledToUnload, self.fillFreeCapacity = AutoDrive.getAllFillLevels(self.trailers)
                    if self.filledToUnload then
                        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "LoadAtDestinationTask:update not isActiveAtTrigger isReverseTriggerReached -> finished")
                        self:finished()
                        return
                    end
                end
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "LoadAtDestinationTask:update not isActiveAtTrigger -> drivePathModule:update")
                self.vehicle.ad.drivePathModule:update(dt)
            end
        end
    end
end

function LoadAtDestinationTask:continue()
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "LoadAtDestinationTask:continue -> trailerModule:stopLoading")
    self.vehicle.ad.trailerModule:stopLoading()
    AutoDrive.deactivateALTrailers(self.vehicle, self.trailers)
    AutoDrive.resetFoldState(self.vehicle)
    AutoDrive.closeAllCurtains(self.trailers, true) -- close curtain at UAL trailers
end

function LoadAtDestinationTask:abort()
    AutoDrive.deactivateALTrailers(self.vehicle, self.trailers)
    AutoDrive.resetFoldState(self.vehicle)
    AutoDrive.closeAllCurtains(self.trailers, true) -- close curtain at UAL trailers
end

function LoadAtDestinationTask:finished()
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "LoadAtDestinationTask:finished -> specialDrivingModule:releaseVehicle / setCurrentTaskFinished")
    self.vehicle.ad.specialDrivingModule:releaseVehicle()
    AutoDrive.deactivateALTrailers(self.vehicle, self.trailers)
    AutoDrive.resetFoldState(self.vehicle)
    AutoDrive.closeAllCurtains(self.trailers, true) -- close curtain at UAL trailers
    self.vehicle.ad.taskModule:setCurrentTaskFinished()
end

function LoadAtDestinationTask:getI18nInfo()
    if self.state == LoadAtDestinationTask.STATE_PATHPLANNING then
        local actualState, maxStates, steps, max_pathfinder_steps = self.vehicle.ad.pathFinderModule:getCurrentState()
        return "$l10n_AD_task_pathfinding;" .. string.format(" %d / %d - %d / %d", actualState, maxStates, steps, max_pathfinder_steps)
    else
        return "$l10n_AD_task_drive_to_load_point;"
    end
end
