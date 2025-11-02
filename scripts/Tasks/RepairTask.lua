RepairTask = ADInheritsFrom(AbstractTask)

RepairTask.STATE_PATHPLANNING = 1
RepairTask.STATE_DRIVING = 2

function RepairTask:new(vehicle, destinationID)
    local o = RepairTask:create()
    o.vehicle = vehicle
    o.isRepaired = false
    o.destinationID = destinationID
    o.trailers = nil
    return o
end

function RepairTask:setUp()
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "RepairTask:setUp ")
    self.repairTrigger = nil
    if ADGraphManager:getDistanceFromNetwork(self.vehicle) > 30 then
        self.state = RepairTask.STATE_PATHPLANNING
        self.vehicle.ad.pathFinderModule:reset()
        self.vehicle.ad.pathFinderModule:startPathPlanningToNetwork(self.destinationID)
    else
        self.state = RepairTask.STATE_DRIVING
        self.vehicle.ad.drivePathModule:setPathTo(self.destinationID)
    end
    AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.WARN, "$l10n_AD_Driver_of; %s $l10n_AD_task_drive_to_repair_point;", 5000, self.vehicle.ad.stateModule:getName())
    self.trailers, _ = AutoDrive.getAllUnits(self.vehicle)
    AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, false)
end

function RepairTask:update(dt)
    if self.state == RepairTask.STATE_PATHPLANNING then
        if self.vehicle.ad.pathFinderModule:hasFinished() then
            self.wayPoints = self.vehicle.ad.pathFinderModule:getPath()
            if self.wayPoints == nil or #self.wayPoints == 0 then
                Logging.error("[AutoDrive] Could not calculate path - shutting down")
                self.vehicle.ad.taskModule:abortAllTasks()
                self.vehicle:stopAutoDrive()
                AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_cannot_find_path;", 5000, self.vehicle.ad.stateModule:getName())
            else
                self.vehicle.ad.drivePathModule:setWayPoints(self.wayPoints)
                self.state = RepairTask.STATE_DRIVING
            end
        else
            self.vehicle.ad.pathFinderModule:update(dt)
            self.vehicle.ad.specialDrivingModule:stopVehicle()
            self.vehicle.ad.specialDrivingModule:update(dt)
        end
    else
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            -- Do the actual repair here
            if g_server ~= nil then
                local implements = AutoDrive.getAllImplements(self.vehicle, true)
                for _, implement in pairs(implements) do
                    if implement ~= nil and implement.repairVehicle ~= nil then
                        implement:repairVehicle()
                        --g_server:broadcastEvent(self)
                        --g_messageCenter:publish(MessageType.VEHICLE_REPAIRED, self.vehicle, self.atSellingPoint)
                        -- repair also transported implements, vehicles
                        local implementFarmId = implement.getOwnerFarmId and implement:getOwnerFarmId()
                        if  implementFarmId ~= nil and implementFarmId ~= 0 then
                            local implementPosX, implementPosY, implementPosZ = getWorldTranslation(implement.components[1].node)
                            for _, otherVehicle in pairs(AutoDrive.getAllVehicles()) do
                                local otherVehicleFarmId = otherVehicle.getOwnerFarmId and otherVehicle:getOwnerFarmId()
                                if otherVehicleFarmId ~= nil and otherVehicleFarmId == implementFarmId then
                                    local otherPosX, otherPosY, otherPosZ = getWorldTranslation(otherVehicle.components[1].node)
                                    local distance = MathUtil.vector2Length(otherPosX - implementPosX, otherPosZ - implementPosZ)
                                    if  distance < 5 and otherVehicle.repairVehicle then
                                        otherVehicle:repairVehicle()
                                    end
                                end
                            end
                        end
                    end
                end
            end
            self:finished()
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end
    end
end

function RepairTask:abort()
    self.vehicle.ad.onRouteToRepair = false
end

function RepairTask:finished()    
    self.vehicle.ad.onRouteToRepair = false

    self.vehicle:stopAutoDrive()
    self.vehicle.ad.stateModule:getCurrentMode():start()
    self.vehicle.ad.taskModule:setCurrentTaskFinished(ADTaskModule.DONT_PROPAGATE)
end

function RepairTask:getInfoText()
    if self.state == RepairTask.STATE_PATHPLANNING then
        local actualState, maxStates = self.vehicle.ad.pathFinderModule:getCurrentState()
        return g_i18n:getText("AD_task_pathfinding") .. string.format(" %d / %d ", actualState, maxStates)
    else
        return g_i18n:getText("AD_task_drive_to_repair_point")
    end
end

function RepairTask:getI18nInfo()
    if self.state == RepairTask.STATE_PATHPLANNING then
        local actualState, maxStates, steps, max_pathfinder_steps = self.vehicle.ad.pathFinderModule:getCurrentState()
        return "$l10n_AD_task_pathfinding;" .. string.format(" %d / %d - %d / %d", actualState, maxStates, steps, max_pathfinder_steps)
    else
        return "$l10n_AD_task_drive_to_repair_point;"
    end
end
