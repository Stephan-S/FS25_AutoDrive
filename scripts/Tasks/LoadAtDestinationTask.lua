LoadAtDestinationTask = ADInheritsFrom(AbstractTask)
LoadAtDestinationTask.debug = false

LoadAtDestinationTask.STATE_PATHPLANNING = 1
LoadAtDestinationTask.STATE_DRIVING = 2

LoadAtDestinationTask.LOAD_RETRY_TIME = 3000

function LoadAtDestinationTask:new(vehicle, destinationID)
    local o = LoadAtDestinationTask:create()
    o.vehicle = vehicle
    o.destinationID = destinationID
    o.trailers = nil
    o.waitForUALLoadTimer = AutoDriveTON:new()
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
    self.UALAttempts = 0
    self.retryTime = LoadAtDestinationTask.LOAD_RETRY_TIME
    self.isReverseTriggerReached = false
    LoadAtDestinationTask.debugMsg(self.vehicle, "LoadAtDestinationTask:setUp end self.state %s", tostring(self.state))
end

function LoadAtDestinationTask:update(dt)
    if self.state == LoadAtDestinationTask.STATE_PATHPLANNING then
        if self.vehicle.ad.pathFinderModule:hasFinished() then
            self.wayPoints = self.vehicle.ad.pathFinderModule:getPath()
            if self.wayPoints == nil or #self.wayPoints == 0 then
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
        LoadAtDestinationTask.debugMsg(self.vehicle, "LoadAtDestinationTask:update self.state %s", tostring(self.state))
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            --Check if we have actually loaded / tried to load
            LoadAtDestinationTask.debugMsg(self.vehicle, "LoadAtDestinationTask:update isTargetReached")
            AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, true)

            if (self.vehicle.ad.stateModule:getCanRestartHelper() and self.vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER) then
                LoadAtDestinationTask.debugMsg(self.vehicle, "LoadAtDestinationTask:update stopAutoDrive")
                -- pass over to CP
                self.vehicle:stopAutoDrive()
            else
                self.vehicle.ad.specialDrivingModule:stopVehicle()
                self.vehicle.ad.specialDrivingModule:update(dt)
                -- local waitForALUnloadTime = AutoDrive.getSetting("ALUnloadWaitTime", self.vehicle)

                if self.vehicle.ad.hasAL then
                    -- UAL special handling - loading only possible if vehicle not moving -> self.lastSpeedReal < 0.0005
                    if not self.activatedUALLoading then
                        if self.vehicle.lastSpeedReal < 0.0005 then
                            local objectsAvailable = AutoDrive.objectsToLoadAvailable(self.vehicle, self.trailers)
                            if not objectsAvailable then
                                self.UALAttempts = self.UALAttempts + 1
                                self:spawnObjects(self.UALAttempts)
                            end
                            -- start loading
                            self.activatedUALLoading = true
                            self.ualIterations = 0
                            self.fillLevel, _, _, _ = AutoDrive.getAllFillLevels(self.trailers)
                            self.lastFillLevel = self.fillLevel
                            self.numberOfSelectedFillTypes = #self.vehicle.ad.stateModule:getSelectedFillTypes()
                            AutoDrive.activateALTrailers(self.vehicle, self.trailers)
                            self.retryTime = LoadAtDestinationTask.LOAD_RETRY_TIME
                        end
                    end
                else
                    if ((AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYPICKUP or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders"))
                        or (AutoDrive.getSetting("unloadFillLevel", self.vehicle) == 0)
                    then
                        LoadAtDestinationTask.debugMsg(self.vehicle, "LoadAtDestinationTask:update -> self:finished")
                        self:finished()
                        return
                    end
                    self.retryTime = LoadAtDestinationTask.LOAD_RETRY_TIME
                end
                if self.vehicle.ad.trailerModule:isActiveAtTrigger() then
                    -- update to catch if no longer active at trigger
                    self.vehicle.ad.trailerModule:update(dt)
                else
                    -- try to load somehow while standing at destination
                    LoadAtDestinationTask.debugMsg(self.vehicle, "LoadAtDestinationTask:update not isActiveAtTrigger")
                    self.loadRetryTimer:timer(true, self.retryTime, dt)
                    if self.loadRetryTimer:done() then
                        -- performance: avoid to initiate loading while standing at destination to often
                        self.loadRetryTimer:timer(false)      -- clear timer
                        LoadAtDestinationTask.debugMsg(self.vehicle, "LoadAtDestinationTask:update try loading somehow")

                        self.fillLevel, _, self.filledToUnload, self.fillFreeCapacity = AutoDrive.getAllFillLevels(self.trailers)
                        if self.activatedUALLoading == true then
                            -- UAL loading activated
                            if self.filledToUnload then
                                -- fill level reached or continue
                                self:finished()
                                return
                            end

                            if self.numberOfSelectedFillTypes > 1 then
                                -- multiple selected
                                local objectsAvailable = AutoDrive.objectsToLoadAvailable(self.vehicle, self.trailers)
                                if not objectsAvailable then
                                    local supportedFillTypeIDs = nil
                                    local objectStorage = AutoDrive.getNearestObjectStorage(self.vehicle)
                                    if objectStorage then
                                        supportedFillTypeIDs = AutoDrive.getObjectStorageFillTypes(self.vehicle, objectStorage)
                                    else
                                        local productionSpawner = AutoDrive.getNearestProductionSpawner(self.vehicle)
                                        if productionSpawner then
                                            supportedFillTypeIDs = AutoDrive.getProductionSpawnerFillTypes(self.vehicle, productionSpawner)
                                        end
                                    end

                                    if supportedFillTypeIDs and #supportedFillTypeIDs > 0 then
                                        local count = 0
                                        repeat
                                            -- find and select next avaiable fillType in ObjectStorage
                                            self.vehicle.ad.stateModule:nextSelectedFillType()
                                            count = count + 1
                                        until table.contains(supportedFillTypeIDs, self.vehicle.ad.stateModule:getFillType())
                                            or (count > self.numberOfSelectedFillTypes)
                                        self.ualIterations = self.ualIterations + 1
                                    end
                                end
                            end
                            -- request spawn objects
                            self.UALAttempts = self.UALAttempts + 1
                            self:spawnObjects(self.UALAttempts)
                            self.retryTime = math.max(LoadAtDestinationTask.LOAD_RETRY_TIME, AutoDrive.getSetting("ALUnloadWaitTime", self.vehicle))
                        else
                            self.vehicle.ad.trailerModule:update(dt)
                            if not self.vehicle.ad.trailerModule:isActiveAtTrigger() then
                                -- check fill levels only if not still filling something
                                if self.filledToUnload or (AutoDrive.getSetting("unloadFillLevel", self.vehicle) == 0) then
                                    LoadAtDestinationTask.debugMsg(self.vehicle, "LoadAtDestinationTask:update leftCapacity <= -> self:finished")
                                    self:finished()
                                    return
                                end
                            end
                        end
                    end
                end
            end
        else
            LoadAtDestinationTask.debugMsg(self.vehicle, "LoadAtDestinationTask:update NOT isTargetReached")

            if not (self.vehicle.ad.stateModule:getCanRestartHelper() and self.vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER) then
                -- need to try loading if CP is not active
                self.vehicle.ad.trailerModule:update(dt)
            end
            if self.vehicle.ad.trailerModule:isActiveAtTrigger() then
                LoadAtDestinationTask.debugMsg(self.vehicle, "LoadAtDestinationTask:update 2 isActiveAtTrigger -> specialDrivingModule:stopVehicle")
                self.isReverseTriggerReached = self.vehicle.ad.drivePathModule:getIsReversing()
                self.vehicle.ad.specialDrivingModule:stopVehicle()
                self.vehicle.ad.specialDrivingModule:update(dt)
            else
                if self.isReverseTriggerReached then
                    self.fillLevel, _, self.filledToUnload, self.fillFreeCapacity = AutoDrive.getAllFillLevels(self.trailers)
                    if self.filledToUnload then
                        LoadAtDestinationTask.debugMsg(self.vehicle, "LoadAtDestinationTask:update not isActiveAtTrigger isReverseTriggerReached -> finished")
                        self:finished()
                        return
                    end
                end
                LoadAtDestinationTask.debugMsg(self.vehicle, "LoadAtDestinationTask:update not isActiveAtTrigger -> drivePathModule:update")
                self.vehicle.ad.drivePathModule:update(dt)
            end
        end
    end
end

function LoadAtDestinationTask:spawnObjects(attempts)
    local numObjectsToSpawn = AutoDrive.getSetting("ALspawnObjects", self.vehicle)
    LoadAtDestinationTask.debugMsg(self.vehicle, "spawnObjects start attempts %s numObjectsToSpawn %s", tostring(attempts), tostring(numObjectsToSpawn))
    if numObjectsToSpawn > 0 then
        local productionSpawner = nil
        local objectStorage = AutoDrive.getNearestObjectStorage(self.vehicle)
        if objectStorage == nil then
            productionSpawner = AutoDrive.getNearestProductionSpawner(self.vehicle)
        end
        local fillTypeIndex = self.vehicle.ad.stateModule:getFillType()
        if (objectStorage or productionSpawner) and fillTypeIndex ~= FillType.UNKNOWN then
            if objectStorage then
                LoadAtDestinationTask.debugMsg(self.vehicle, "objectStorageSpawn spawnObjects fillTypeIndex %s", tostring(fillTypeIndex))
                AutoDrive.objectStorageSpawn(self.vehicle, objectStorage, fillTypeIndex, numObjectsToSpawn)
            elseif productionSpawner then
                LoadAtDestinationTask.debugMsg(self.vehicle, "productionSpawnerSpawn spawnObjects fillTypeIndex %s", tostring(fillTypeIndex))
                AutoDrive.productionSpawnerSpawn(self.vehicle, productionSpawner, fillTypeIndex, numObjectsToSpawn)
            end
        end
    end
    LoadAtDestinationTask.debugMsg(self.vehicle, "spawnObjects end attempts %d", attempts)
end

function LoadAtDestinationTask:continue()
    LoadAtDestinationTask.debugMsg(self.vehicle, "LoadAtDestinationTask:continue -> trailerModule:stopLoading")
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
    LoadAtDestinationTask.debugMsg(self.vehicle, "LoadAtDestinationTask:finished -> specialDrivingModule:releaseVehicle / setCurrentTaskFinished")
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

function LoadAtDestinationTask.debugMsg(vehicle, debugText, ...)
    if LoadAtDestinationTask.debug == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    else
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_VEHICLEINFO, debugText, ...)
    end
end
