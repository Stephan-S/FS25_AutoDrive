DriveToDestinationTask = ADInheritsFrom(AbstractTask)

DriveToDestinationTask.STATE_PATHPLANNING = 1
DriveToDestinationTask.STATE_DRIVING = 2

function DriveToDestinationTask:new(vehicle, destinationID)
    local o = DriveToDestinationTask:create()
    o.vehicle = vehicle
    o.destinationID = destinationID
    o.trailers = nil
    o.gsRoadBlockMs = 0
    o.gsRoadReplanUsed = false
    return o
end

function DriveToDestinationTask:setUp()
    if self.vehicle.spec_locomotive and self.vehicle.ad and self.vehicle.ad.trainModule then
        self.state = DriveToDestinationTask.STATE_DRIVING
        self.vehicle.ad.trainModule:setPathTo(self.destinationID)
    elseif self.gsForceDeparturePath or ADGraphManager:getDistanceFromNetwork(self.vehicle) > 30 then
        self.state = DriveToDestinationTask.STATE_PATHPLANNING
        self.vehicle.ad.pathFinderModule:reset()
        self.vehicle.ad.pathFinderModule:startPathPlanningToNetwork(self.destinationID)
    else
        self.state = DriveToDestinationTask.STATE_DRIVING
        self.vehicle.ad.drivePathModule:setPathTo(self.destinationID)
    end
    self.trailers, _ = AutoDrive.getAllUnits(self.vehicle)
    AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, false)
end

function DriveToDestinationTask:update(dt)
    if self.state == DriveToDestinationTask.STATE_PATHPLANNING then
        if self.vehicle.ad.pathFinderModule:hasFinished() then
            self.wayPoints = self.vehicle.ad.pathFinderModule:getPath()
            if self.wayPoints == nil or #self.wayPoints == 0 then
                Logging.error("[AutoDrive] Could not calculate path - shutting down")
                self:finished(ADTaskModule.DONT_PROPAGATE)
                self.vehicle:stopAutoDrive()
                AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_cannot_find_path;", 5000, self.vehicle.ad.stateModule:getName())
            else
                self.vehicle.ad.drivePathModule:setWayPoints(self.wayPoints)
                --self.vehicle.ad.drivePathModule:appendPathTo(self.wayPoints[#self.wayPoints], self.destinationID)
                self.state = DriveToDestinationTask.STATE_DRIVING
            end
        else
            self.vehicle.ad.pathFinderModule:update(dt)
            self.vehicle.ad.specialDrivingModule:stopVehicle()
            self.vehicle.ad.specialDrivingModule:update(dt)
        end
    else
        local ad = self.vehicle.ad
        local sensor = ad.collisionDetectionModule
        local shortBlocked = sensor ~= nil and sensor.gsShortObstacle == true
        local blocked = sensor ~= nil
            and (sensor.trafficVehicle ~= nil or sensor.gsLongObstacle == true or shortBlocked)
        if blocked and math.abs(self.vehicle.lastSpeedReal or 0) < 0.00012
            and ad.drivePathModule:isOnRoadNetwork() then
            self.gsRoadBlockMs = (self.gsRoadBlockMs or 0) + dt
        else
            self.gsRoadBlockMs = 0
        end
        local roadTimeout = shortBlocked and 15000 or 45000
        if not self.gsRoadReplanUsed and self.gsRoadBlockMs > roadTimeout then
            self.gsRoadReplanUsed = true
            ad.pathFinderModule:reset()
            ad.pathFinderModule:startPathPlanningToNetwork(self.destinationID)
            self.state = DriveToDestinationTask.STATE_PATHPLANNING
            return
        end
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            self:finished()
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end
    end
end

function DriveToDestinationTask:abort()
end

function DriveToDestinationTask:finished(propagate)
    self.vehicle.ad.taskModule:setCurrentTaskFinished(propagate)
end

function DriveToDestinationTask:getI18nInfo()
    if self.state == DriveToDestinationTask.STATE_PATHPLANNING then
        local actualState, maxStates, steps, max_pathfinder_steps = self.vehicle.ad.pathFinderModule:getCurrentState()
        return "$l10n_AD_task_pathfinding;" .. string.format(" %d / %d - %d / %d", actualState, maxStates, steps, max_pathfinder_steps)
    else
        return "$l10n_AD_task_drive_to_destination;"
    end
end
