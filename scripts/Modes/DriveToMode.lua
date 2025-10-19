DriveToMode = ADInheritsFrom(AbstractMode)

function DriveToMode:new(vehicle)
    local o = DriveToMode:create()
    o.vehicle = vehicle
    DriveToMode.reset(o)
    return o
end

function DriveToMode:reset()
    self.driveToDestinationTask = nil
    self.destinationID = nil
    self.vehicle.ad.trailerModule:reset()
end

function DriveToMode:start(user)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "DriveToMode:start self.vehicle.ad.onRouteToRefuel %s", tostring(self.vehicle.ad.onRouteToRefuel))
    if not self.vehicle.ad.stateModule:isActive() then
        self.vehicle:startAutoDrive()
    end

    if self.vehicle.ad.stateModule:getFirstMarker() == nil then
        return
    end

    self:reset()
    self.destinationID = self.vehicle.ad.stateModule:getFirstMarker().id
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "DriveToMode:start self.destinationID %s", tostring(self.destinationID))

    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "DriveToMode:start add DriveToDestinationTask")
    self.driveToDestinationTask = DriveToDestinationTask:new(self.vehicle, self.destinationID)

    self.vehicle.ad.taskModule:addTask(self.driveToDestinationTask)
end

function DriveToMode:monitorTasks(dt)
end

function DriveToMode:handleFinishedTask()
    if self.driveToDestinationTask ~= nil then
        self.driveToDestinationTask = nil
        self.vehicle.ad.taskModule:addTask(StopAndDisableADTask:new(self.vehicle), ADTaskModule.DONT_PROPAGATE)
        local target = self.vehicle.ad.stateModule:getFirstMarker().name
        local mapMarker = ADGraphManager:getMapMarkerByWayPointId(self.destinationID)
        if mapMarker ~= nil and mapMarker.name ~= nil then
            target = mapMarker.name
        end
        if self.vehicle.ad.isStoppingWithError == false then
            AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.INFO, "$l10n_AD_Driver_of; %s $l10n_AD_has_reached; %s", 5000, self.vehicle.ad.stateModule:getName(), target)
        end
    end
end

function DriveToMode:stop()
end
