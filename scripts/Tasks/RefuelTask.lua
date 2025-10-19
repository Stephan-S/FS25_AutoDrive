RefuelTask = ADInheritsFrom(AbstractTask)

RefuelTask.STATE_PATHPLANNING = 1
RefuelTask.STATE_DRIVING = 2

function RefuelTask:new(vehicle, destinationID)
    local o = RefuelTask:create()
    o.vehicle = vehicle
    o.destinationID = destinationID
    o.trailers = nil
    return o
end

function RefuelTask:setUp()
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "RefuelTask:setUp ")
    self.refuelTrigger = nil
    self.wasRefuelling = false
    self.matchingFillTypes = {}
    if ADGraphManager:getDistanceFromNetwork(self.vehicle) > 30 then
        self.state = RefuelTask.STATE_PATHPLANNING
        self.vehicle.ad.pathFinderModule:reset()
        self.vehicle.ad.pathFinderModule:startPathPlanningToNetwork(self.destinationID)
    else
        self.state = RefuelTask.STATE_DRIVING
        self.vehicle.ad.drivePathModule:setPathTo(self.destinationID)
    end
    AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.WARN, "$l10n_AD_Driver_of; %s $l10n_AD_task_drive_to_refuel_point;", 5000, self.vehicle.ad.stateModule:getName())
    self.trailers, _ = AutoDrive.getAllUnits(self.vehicle)
    AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, false)
end

function RefuelTask:update(dt)
    local inRefuelRange = false
    if self.refuelTrigger ~= nil and self.refuelTrigger.stoppedTimer ~= nil then
        -- update timer
        self.refuelTrigger.stoppedTimer:timer(not self.refuelTrigger.isLoading,300,dt)
    end

    if self.state == RefuelTask.STATE_PATHPLANNING then
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
                self.state = RefuelTask.STATE_DRIVING
            end
        else
            self.vehicle.ad.pathFinderModule:update(dt)
            self.vehicle.ad.specialDrivingModule:stopVehicle()
            self.vehicle.ad.specialDrivingModule:update(dt)
        end
    else
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            self:finished()
        else
            local refuelOngoing = self.refuelTrigger ~= nil and (self.refuelTrigger.isLoading or (self.refuelTrigger.stoppedTimer ~= nil and not self.refuelTrigger.stoppedTimer:done())) 
            if refuelOngoing then
                self.vehicle.ad.specialDrivingModule:stopVehicle()
                self.vehicle.ad.specialDrivingModule:update(dt)
            else
                if self.wasRefuelling then
                    -- allow loading at the refuel destination
                    self:finished()
                end
                self.vehicle.ad.drivePathModule:update(dt)
            end

            inRefuelRange = self:isInRefuelRange()
            if inRefuelRange and not refuelOngoing then
                if self:getMatchingFillTypes() then

                    -- if self.refuelTrigger ~= nil and inRefuelRange and not (self.refuelTrigger.isLoading or (self.refuelTrigger.stoppedTimer ~= nil and not self.refuelTrigger.stoppedTimer:done())) then
                    if self.refuelTrigger ~= nil and not (self.refuelTrigger.isLoading) then
                        self:startRefueling()
                    end

                end
            end
        end
    end
end

function RefuelTask:abort()
    self.vehicle.ad.onRouteToRefuel = false
    self.refuelTrigger = nil
    self.wasRefuelling = false
end

function RefuelTask:finished()
    self.vehicle.ad.onRouteToRefuel = #AutoDrive.getRequiredRefuels(self.vehicle, self.vehicle.ad.onRouteToRefuel) > 0
    self.refuelTrigger = nil
    self.wasRefuelling = false
    self.vehicle:stopAutoDrive()
    self.vehicle.ad.stateModule:getCurrentMode():start()
    self.vehicle.ad.taskModule:setCurrentTaskFinished(ADTaskModule.DONT_PROPAGATE)
end

function RefuelTask:isInRefuelRange()
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    local refuelX, refuelZ = ADGraphManager:getWayPointById(self.destinationID).x, ADGraphManager:getWayPointById(self.destinationID).z
    local distance = MathUtil.vector2Length(refuelX - x, refuelZ - z)       -- vehicle to destination

    if distance <= AutoDrive.MAX_REFUEL_TRIGGER_DISTANCE then
        if self.refuelTrigger == nil then

            local refuelTrigger = ADTriggerManager.getClosestRefuelTrigger(self.vehicle, self.vehicle.ad.onRouteToRefuel)
            if refuelTrigger then
                for _, fillableObject in pairs(refuelTrigger.fillableObjects) do
                    if fillableObject == self.vehicle or (fillableObject.object ~= nil and fillableObject.object == self.vehicle) then
                        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "RefuelTask:isInRefuelRange return true")
                        self.refuelTrigger = refuelTrigger
                        return true
                    end
                end
            end
        end
    end
    return self.refuelTrigger ~= nil
end

function RefuelTask:getMatchingFillTypes()
    local ret = false
    self.matchingFillTypes = {}
    if self.refuelTrigger ~= nil then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "RefuelTask:getMatchingFillTypes Start ")

        local requiredRefuelFillTypes = AutoDrive.getRequiredRefuels(self.vehicle, self.vehicle.ad.onRouteToRefuel)
        if requiredRefuelFillTypes and #requiredRefuelFillTypes > 0 then
            local spec = self.vehicle.spec_motorized
            if spec ~= nil and spec.consumers ~= nil then

                for index, consumer in pairs(spec.consumers) do
                    for _, fillType in pairs(requiredRefuelFillTypes) do
                        local refuelFillTypeTitle = g_fillTypeManager:getFillTypeByIndex(fillType) and g_fillTypeManager:getFillTypeByIndex(fillType).title or "unknown"
                        if AutoDrive.fillTypesMatch(self.vehicle, self.refuelTrigger, self.vehicle, {fillType}, consumer.fillUnitIndex) then
                            local item = {fillType = fillType, wasLoaded = false, refuelTrigger = self.refuelTrigger}
                            table.insert(self.matchingFillTypes, item)
                            ret = true
                        end
                    end
                end
            end
        end
    end
    return ret
end

function RefuelTask:startRefueling()
    if self.refuelTrigger ~= nil and (not self.refuelTrigger.isLoading) then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "RefuelTask:startRefueling Start refueling")

        local spec = self.vehicle.spec_motorized
        if spec ~= nil and spec.consumers ~= nil then
            for index, consumer in pairs(spec.consumers) do
                for _, item in pairs(self.matchingFillTypes) do
                    if not item.wasLoaded then
                        local refuelFillTypeTitle = g_fillTypeManager:getFillTypeByIndex(item.fillType) and g_fillTypeManager:getFillTypeByIndex(item.fillType).title or "unknown"
                        if AutoDrive.fillTypesMatch(self.vehicle, self.refuelTrigger, self.vehicle, {item.fillType}, consumer.fillUnitIndex) then
                        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "RefuelTask:startRefueling fillTypesMatch -> refuelFillTypeTitle %s", refuelFillTypeTitle)

                            self.refuelTrigger.autoStart = true
                            self.refuelTrigger.selectedFillType = item.fillType
                            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "RefuelTask:startRefueling Start onFillTypeSelection")
                            self.refuelTrigger:onFillTypeSelection(item.fillType)
                            if self.refuelTrigger.isLoading then    
                                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "RefuelTask:startRefueling isLoading")
                                self.refuelTrigger.selectedFillType = item.fillType
                                self.refuelTrigger.autoStart = true
                                g_effectManager:setEffectTypeInfo(self.refuelTrigger.effects, self.refuelTrigger.selectedFillType)
                            end
                            if self.refuelTrigger.stoppedTimer == nil then
                                self.refuelTrigger.stoppedTimer = AutoDriveTON:new()
                            end
                            self.refuelTrigger.stoppedTimer:timer(false, 500)

							if self.refuelTrigger.isLoading then
                                self.wasRefuelling = true
                            	item.wasLoaded = true
                            	return
							end
                        end
                    end
                end
            end
        end
    end
end

function RefuelTask:getInfoText()
    if self.state == RefuelTask.STATE_PATHPLANNING then
        local actualState, maxStates = self.vehicle.ad.pathFinderModule:getCurrentState()
        return g_i18n:getText("AD_task_pathfinding") .. string.format(" %d / %d ", actualState, maxStates)
    else
        return g_i18n:getText("AD_task_drive_to_refuel_point")
    end
end

function RefuelTask:getI18nInfo()
    if self.state == RefuelTask.STATE_PATHPLANNING then
        local actualState, maxStates, steps, max_pathfinder_steps = self.vehicle.ad.pathFinderModule:getCurrentState()
        return "$l10n_AD_task_pathfinding;" .. string.format(" %d / %d - %d / %d", actualState, maxStates, steps, max_pathfinder_steps)
    else
        return "$l10n_AD_task_drive_to_refuel_point;"
    end
end
