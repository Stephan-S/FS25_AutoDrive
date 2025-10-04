ADUnloadManager = {}

ADUnloadManager.UPDATE_TIME = 1000

function ADUnloadManager:load()
    self.bunkerSilos = {}
    self.unloadingTargets = {}
    self.lastUpdateTime = 0
end

function ADUnloadManager:update(dt)

    if g_time < self.lastUpdateTime + ADUnloadManager.UPDATE_TIME then
        return
    end
    self.lastUpdateTime = g_time
    local range = AutoDrive.getSetting("UMRange") or 0
    if range == 0 then
        return
    end

    -- first check for bunker silos
    self.bunkerSilos = {}
    for _, bunkerSilo in pairs(ADTriggerManager.getBunkerSilos()) do
        if bunkerSilo and bunkerSilo.bunkerSiloArea then
            bunkerSilo.adVehicles = {}
            table.insert(self.bunkerSilos, bunkerSilo)
        end
    end

    for _, bunkerSilo in pairs(self.bunkerSilos) do
        local minDistance = math.huge
        bunkerSilo.adClosestVehicle = nil
        for _, vehicle in pairs(AutoDrive.getAllVehicles()) do
            if vehicle and vehicle.ad and vehicle.ad.stateModule and vehicle.ad.stateModule:isActive() then
                if self:isDestinationInBunkerSilo(vehicle, bunkerSilo) then
                    table.insert(bunkerSilo.adVehicles, vehicle)
                    vehicle.ad.isUnloadManaged = true
                    local vehicleX, _, vehicleZ = getWorldTranslation(vehicle.components[1].node)
                    local triggerX, _, triggerZ = ADTriggerManager.getTriggerPos(bunkerSilo)
                    if triggerX ~= nil then
                        local distance = MathUtil.vector2Length(triggerX - vehicleX, triggerZ - vehicleZ)
                        if minDistance > distance then
                            minDistance = distance
                            bunkerSilo.adClosestVehicle = vehicle
                        end
                    end
                end
            end
        end
    end

    for _, bunkerSilo in pairs(self.bunkerSilos) do
        for _, vehicle in pairs(bunkerSilo.adVehicles) do
            local vehicleX, _, vehicleZ = getWorldTranslation(vehicle.components[1].node)
            local triggerX, _, triggerZ = ADTriggerManager.getTriggerPos(bunkerSilo)
            if triggerX ~= nil then
                local distance = MathUtil.vector2Length(triggerX - vehicleX, triggerZ - vehicleZ)
                if distance < range then
                    local fillLevel, _, _, _ = AutoDrive.getAllFillLevels(AutoDrive.getAllUnits(vehicle))
                    if AutoDrive.isVehicleInBunkerSiloArea(vehicle)
                    or bunkerSilo.adClosestVehicle == vehicle
                    or fillLevel < 0.1
                    then
                        -- IMPORTANT: DO NOT SET setUnPaused to avoid crash with CP silo compacter !!!
                        -- vehicle.ad.drivePathModule:setUnPaused()
                    else
                        vehicle.ad.drivePathModule:setPaused()
                    end
                end
            end
        end
    end

    self.unloadingTargets = {}
    for _, vehicle in pairs(AutoDrive.getAllVehicles()) do
        if vehicle and vehicle.ad and vehicle.ad.stateModule and vehicle.ad.stateModule:isActive() and not vehicle.ad.isUnloadManaged then
            -- check vehicles not going into bunker silo
            local destination = nil
            if vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER or vehicle.ad.stateModule:getMode() == AutoDrive.MODE_UNLOAD then
                destination = vehicle.ad.stateModule:getSecondWayPoint()
            elseif vehicle.ad.stateModule:getMode() == AutoDrive.MODE_DELIVERTO then
                destination = vehicle.ad.stateModule:getFirstWayPoint()
            end
            if destination and destination > 0 then
                if self.unloadingTargets[destination] == nil then
                    self.unloadingTargets[destination] = {}
                end
                local wayPoint = ADGraphManager:getWayPointById(destination)
                local vehicleX, _, vehicleZ = getWorldTranslation(vehicle.components[1].node)
                local distance = MathUtil.vector2Length(wayPoint.x - vehicleX, wayPoint.z - vehicleZ)
                if distance < range then
                    local fillLevel, _, _, _ = AutoDrive.getAllFillLevels(AutoDrive.getAllUnits(vehicle))
                    if fillLevel > 0.1 then
                        if self.unloadingTargets[destination][1] == nil or (self.unloadingTargets[destination][1] and distance < self.unloadingTargets[destination][1].minDistance) then
                            table.insert(self.unloadingTargets[destination], 1, vehicle)
                            self.unloadingTargets[destination][1].minDistance = distance
                        else
                            table.insert(self.unloadingTargets[destination], vehicle)
                        end
                    end
                end
            end
        end
        if vehicle and vehicle.ad and vehicle.ad.stateModule then
            -- reset the state
            vehicle.ad.isUnloadManaged = false
        end
    end
    if table.count(self.unloadingTargets) > 0 then
        for destination, _ in pairs(self.unloadingTargets) do
            if table.count(self.unloadingTargets[destination]) > 1 then
                for i, vehicle in pairs(self.unloadingTargets[destination]) do
                    if i > 1 and vehicle and vehicle.ad and vehicle.ad.drivePathModule then
                        vehicle.ad.drivePathModule:setPaused()
                    end
                end
            end
        end
    end
end

function ADUnloadManager:isDestinationInBunkerSilo(vehicle, bunkerSilo)
    local network = ADGraphManager:getWayPoints()
    local destination = nil
    local destinationInBunkerSilo = false
    if vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER or vehicle.ad.stateModule:getMode() == AutoDrive.MODE_UNLOAD then
        destination = vehicle.ad.stateModule:getSecondWayPoint()
    elseif vehicle.ad.stateModule:getMode() == AutoDrive.MODE_DELIVERTO then
        destination = vehicle.ad.stateModule:getFirstWayPoint()
    end
    if destination and destination > 0 then
        local wp = network[destination]
        if wp then
            destinationInBunkerSilo = MathUtil.isPointInParallelogram(wp.x, wp.z, bunkerSilo.bunkerSiloArea.sx, bunkerSilo.bunkerSiloArea.sz, 
                bunkerSilo.bunkerSiloArea.dwx, bunkerSilo.bunkerSiloArea.dwz, bunkerSilo.bunkerSiloArea.dhx, bunkerSilo.bunkerSiloArea.dhz)
        end
    end
    return destinationInBunkerSilo
end
