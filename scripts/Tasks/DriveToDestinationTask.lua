DriveToDestinationTask = ADInheritsFrom(AbstractTask)

DriveToDestinationTask.STATE_PATHPLANNING = 1
DriveToDestinationTask.STATE_DRIVING = 2

function DriveToDestinationTask:new(vehicle, destinationID)
    local o = DriveToDestinationTask:create()
    o.vehicle = vehicle
    o.destinationID = destinationID
    o.trailers = nil
    return o
end

function DriveToDestinationTask:setUp()
    if self.vehicle.spec_locomotive and self.vehicle.ad and self.vehicle.ad.trainModule then
        self.state = DriveToDestinationTask.STATE_DRIVING
        self.vehicle.ad.trainModule:setPathTo(self.destinationID)
    elseif ADGraphManager:getDistanceFromNetwork(self.vehicle) > 30 then
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
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            self:finished()
        elseif self:isEntranceHold(dt) then
            return
        elseif self:isFieldFullStop(dt) then
            return
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end
    end
end

DriveToDestinationTask.FIELD_FULL_STOP_DISTANCE = 60 -- m before the field point: the field being full, it stops on the road here
DriveToDestinationTask.FIELD_FULL_STOP_MIN = 25     -- m: closer than this it goes on to the field point as before
DriveToDestinationTask.FIELD_FULL_CHECK = 1000      -- ms between two looks

-- empty unloader back to a field that already holds its unloaders (ADFieldPark.FIELD_CAPACITY): it does not drive up
-- to the entrance (where it blocks it, nose in the field, with no clean way to the roadside), it stops on the road
-- before the entrance and WaitForCallTask parks it beside the road from there
function DriveToDestinationTask:isFieldFullStop(dt)
    local mode = self.vehicle.ad.modes ~= nil and self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD] or nil
    if mode == nil or ADFieldPark == nil or not ADFieldPark.isEnabled() or self.vehicle.ad.stateModule:getMode() ~= AutoDrive.MODE_UNLOAD
        or mode.state ~= mode.STATE_DRIVE_TO_START then
        return false
    end
    self.fieldFullCheck = (self.fieldFullCheck or 0) + dt
    if self.fieldFullCheck < DriveToDestinationTask.FIELD_FULL_CHECK then
        return false
    end
    self.fieldFullCheck = 0
    local marker = self.vehicle.ad.stateModule:getFirstMarker()
    local wp = marker ~= nil and ADGraphManager:getWayPointById(marker.id) or nil
    if wp == nil then
        return false
    end
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local distance = MathUtil.vector2Length(wp.x - x, wp.z - z)
    if distance > DriveToDestinationTask.FIELD_FULL_STOP_DISTANCE or distance < DriveToDestinationTask.FIELD_FULL_STOP_MIN
        or AutoDrive.checkIsOnField(x, y, z) then
        return false
    end
    local harvester = ADFieldPark.findHarvester(self.vehicle, wp.x, wp.z)
    if harvester == nil or ADFieldPark.countInField(self.vehicle, harvester) < ADFieldPark.FIELD_CAPACITY then
        return false
    end
    if self.vehicle.ad.trafficYieldModule ~= nil then
        self.vehicle.ad.trafficYieldModule:log("field park: field full (%d unloaders), stopping on the road %.0f m before the entrance", ADFieldPark.FIELD_CAPACITY, distance)
    end
    self.vehicle.ad.specialDrivingModule:stopVehicle()
    self.vehicle.ad.specialDrivingModule:update(dt)
    self:finished()
    return true
end

-- empty unloader back to the field while a full one drives out by the same entrance: it stops on the road before
-- the entrance until the full one is out
function DriveToDestinationTask:isEntranceHold(dt)
    local mode = self.vehicle.ad.modes ~= nil and self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD] or nil
    if mode == nil or ADFieldPark == nil or self.vehicle.ad.stateModule:getMode() ~= AutoDrive.MODE_UNLOAD
        or mode.state ~= mode.STATE_DRIVE_TO_START then
        return false
    end
    return ADFieldPark.holdForExit(self.vehicle, dt, "on the road")
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
