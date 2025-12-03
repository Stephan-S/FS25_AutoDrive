ParkTask = ADInheritsFrom(AbstractTask)

ParkTask.STATE_PATHPLANNING = 1
ParkTask.STATE_DRIVING = 2

function ParkTask:new(vehicle, destinationID)
    local o = ParkTask:create()
    o.vehicle = vehicle
    o.destinationID = destinationID
    o.actualParkDestinationName = nil
    o.trailers = nil
    return o
end

function ParkTask:setUp()
    self.vehicle.ad.onRouteToPark = false
    local targetParkParkDestination = nil

    if AutoDrive.getSetting("enableParkAtJobFinished", self.vehicle) then
        local actualParkDestination = self.vehicle.ad.stateModule:getParkDestinationAtJobFinished()
        if actualParkDestination >= 1 and ADGraphManager:getMapMarkerById(actualParkDestination) ~= nil then
            targetParkParkDestination = ADGraphManager:getMapMarkerById(actualParkDestination).id
            self.actualParkDestinationName = ADGraphManager:getMapMarkerById(actualParkDestination).name
            self.vehicle.ad.onRouteToPark = true
        else
            AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_parkVehicle_noPosSet;", 5000, self.vehicle.ad.stateModule:getName())
        end
    end

    if (self.destinationID == targetParkParkDestination) or (targetParkParkDestination == nil) then
        -- park destination reached or not available - do not drive to it
        self:finished()
        return
    end

    if self.vehicle.spec_locomotive and self.vehicle.ad and self.vehicle.ad.trainModule then
        self.state = ParkTask.STATE_DRIVING
        self.vehicle.ad.trainModule:setPathTo(targetParkParkDestination)
    elseif ADGraphManager:getDistanceFromNetwork(self.vehicle) > 30 then
        self.state = ParkTask.STATE_PATHPLANNING
        self.vehicle.ad.pathFinderModule:reset()
        self.vehicle.ad.pathFinderModule:startPathPlanningToNetwork(targetParkParkDestination)
    else
        self.state = ParkTask.STATE_DRIVING
        self.vehicle.ad.drivePathModule:setPathTo(targetParkParkDestination)
    end
    self.trailers, _ = AutoDrive.getAllUnits(self.vehicle)
    AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, false)
end

function ParkTask:update(dt)
    if self.state == ParkTask.STATE_PATHPLANNING then
        if self.vehicle.ad.pathFinderModule:hasFinished() then
            self.wayPoints = self.vehicle.ad.pathFinderModule:getPath()
            if self.wayPoints == nil or #self.wayPoints == 0 then
                Logging.error("[AutoDrive] Could not calculate path - shutting down")
                self:finished(ADTaskModule.DONT_PROPAGATE)
                self.vehicle:stopAutoDrive()
                AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_cannot_find_path;", 5000, self.vehicle.ad.stateModule:getName())
            else
                self.vehicle.ad.drivePathModule:setWayPoints(self.wayPoints)
                self.state = ParkTask.STATE_DRIVING
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
            self.vehicle.ad.drivePathModule:update(dt)
        end
    end
end

function ParkTask:abort()
    self.vehicle.ad.onRouteToPark = false
end

function ParkTask:finished(propagate)
    self.vehicle.ad.onRouteToPark = false
    -- avoid activate helper when park position is reached
    self.vehicle.ad.stateModule:setStartHelper(false)
    self.vehicle.ad.taskModule:setCurrentTaskFinished(propagate)
    if self.actualParkDestinationName ~= nil then
        AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.INFO, "$l10n_AD_Driver_of; %s $l10n_AD_has_reached; %s", 5000, self.vehicle.ad.stateModule:getName(), self.actualParkDestinationName)
    else
        AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.INFO, "$l10n_AD_Driver_of; %s $l10n_AD_has_reached; %s", 5000, self.vehicle.ad.stateModule:getName(), ADGraphManager:getMapMarkerByWayPointId(self.destinationID).name)
    end
end

function ParkTask:getI18nInfo()
    if self.state == ParkTask.STATE_PATHPLANNING then
        local actualState, maxStates, steps, max_pathfinder_steps = self.vehicle.ad.pathFinderModule:getCurrentState()
        return "$l10n_AD_task_pathfinding;" .. string.format(" %d / %d - %d / %d", actualState, maxStates, steps, max_pathfinder_steps)
    elseif self.vehicle.ad.onRouteToPark == true then
        return "$l10n_AD_task_drive_to_park;"
    else
        return "$l10n_AD_task_drive_to_destination;"
    end
end
