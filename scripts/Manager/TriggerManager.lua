ADTriggerManager = {}

ADTriggerManager.tipTriggers = {}
ADTriggerManager.bunkerSilos = {}
ADTriggerManager.bunkerSilosResult = {}
ADTriggerManager.siloTriggers = {}
ADTriggerManager.repairTriggers = {}

ADTriggerManager.searchedForTriggers = false
ADTriggerManager.maxBunkerSiloLength = 0

function ADTriggerManager.load()
end

function ADTriggerManager:update(dt)
end

function ADTriggerManager.addItems(items)
    if items == nil then
        return
    end
    if table.count(items) > 0 then
        for _, item in pairs(items) do
            local spec = nil
            local loadingStation = nil
            local unloadingStation = nil

-- Loading
            loadingStation = (item.spec_silo and item.spec_silo.loadingStation)
            or (item.spec_husbandry and item.spec_husbandry.loadingStation)
            or (item.spec_manureHeap and item.spec_manureHeap.loadingStation)
            or (item.spec_buyingStation and item.spec_buyingStation.buyingStation)
            or (item.spec_chargingStation and item.spec_chargingStation.buyingStation)
            or (item.spec_productionPoint and item.spec_productionPoint.productionPoint and item.spec_productionPoint.productionPoint.loadingStation)
            or item.loadingStation

            if loadingStation and loadingStation.loadTriggers then
                for _, loadTrigger in pairs(loadingStation.loadTriggers) do
                    if not table.contains(ADTriggerManager.siloTriggers, loadTrigger) then
                        table.insert(ADTriggerManager.siloTriggers, loadTrigger)
                    end
                end
            end

            if item.spec_chargingStation and item.spec_chargingStation.loadTrigger then
                if not table.contains(ADTriggerManager.siloTriggers, item.spec_chargingStation.loadTrigger) then
                    table.insert(ADTriggerManager.siloTriggers, item.spec_chargingStation.loadTrigger)
                end
            end

-- Unloading
--[[
            unloadingStation = (item.spec_silo and item.spec_silo.unloadingStation)
            or (item.spec_husbandry and item.spec_husbandry.unloadingStation)
            or (item.spec_sellingStation and item.spec_sellingStation.sellingStation)
            or (item.spec_productionPoint and item.spec_productionPoint.productionPoint and item.spec_productionPoint.productionPoint.unloadingStation)
            or item.unloadingStation

            if unloadingStation and unloadingStation.unloadTriggers then
                for _, unloadTrigger in pairs(unloadingStation.unloadTriggers) do
                    if not table.contains(ADTriggerManager.tipTriggers, unloadTrigger) then
                        table.insert(ADTriggerManager.tipTriggers, unloadTrigger)
                    end
                end
            end

            if item.spec_husbandryFood and item.spec_husbandryFood.feedingTrough then
                if not table.contains(ADTriggerManager.tipTriggers, item.spec_husbandryFood.feedingTrough) then
                    table.insert(ADTriggerManager.tipTriggers, item.spec_husbandryFood.feedingTrough)
                end
            end
]]

            if item.spec_bunkerSilo then
                ADTriggerManager.addBunkerSilo(item.spec_bunkerSilo.bunkerSilo)
            end

-- Repair
            if item.spec_workshop and item.spec_workshop.sellingPoint then
                if item.spec_workshop.sellingPoint.sellTriggerNode then
                    table.insert(ADTriggerManager.repairTriggers, {node=item.spec_workshop.sellingPoint.sellTriggerNode, owner=item.ownerFarmId })
                end
            end

        end
    end
end

function ADTriggerManager.loadAllTriggers()
    ADTriggerManager.searchedForTriggers = true
    ADTriggerManager.tipTriggers = {}
    ADTriggerManager.bunkerSilos = {}
    ADTriggerManager.bunkerSilosResult = {}
    ADTriggerManager.siloTriggers = {}
    ADTriggerManager.repairTriggers = {}

    if g_currentMission.placeableSystem.placeables ~= nil then
        ADTriggerManager.addItems(g_currentMission.placeableSystem.placeables)
    end

    if g_currentMission.placeables ~= nil then
        ADTriggerManager.addItems(g_currentMission.placeables)
    end

    if g_currentMission.placeableSystem.bunkerSilos ~= nil then
        ADTriggerManager.addItems(g_currentMission.placeableSystem.bunkerSilos)
    end

    if g_currentMission.bunkerSilos ~= nil then
        ADTriggerManager.addItems(g_currentMission.bunkerSilos)
    end

    if g_currentMission.ownedItems ~= nil then
        for _, ownedItem in pairs(g_currentMission.ownedItems) do
            ADTriggerManager.addItems(ownedItem.items)
        end
    end

    -- lump sum whatever is not catched before
    if g_currentMission.nodeToObject ~= nil then
        for _, object in pairs(g_currentMission.nodeToObject) do
            if object.triggerNode ~= nil then
                if not table.contains(ADTriggerManager.siloTriggers, object) then
                    table.insert(ADTriggerManager.siloTriggers, object)
                end
            end
            -- if object.exactFillRootNode ~= nil then
            --     if not table.contains(ADTriggerManager.tipTriggers, object) then
            --         table.insert(ADTriggerManager.tipTriggers, object)
            --     end
            -- end
        end
    end

    for _, trigger in pairs(ADTriggerManager.siloTriggers) do
        if trigger.stoppedTimer == nil then
            trigger.stoppedTimer = AutoDriveTON:new()
        end
    end
end

function ADTriggerManager.getUnloadTriggers()
    if not ADTriggerManager.searchedForTriggers then
        ADTriggerManager.loadAllTriggers()
    end
    return ADTriggerManager.tipTriggers
end

function ADTriggerManager.getBunkerSilos()
    if not ADTriggerManager.searchedForTriggers then
        ADTriggerManager.loadAllTriggers()
    end
    return ADTriggerManager.bunkerSilosResult
end

function ADTriggerManager.getLoadTriggers()
    if not ADTriggerManager.searchedForTriggers then
        ADTriggerManager.loadAllTriggers()
    end
    return ADTriggerManager.siloTriggers
end

function ADTriggerManager.getRepairTriggers()
    if not ADTriggerManager.searchedForTriggers then
        ADTriggerManager.loadAllTriggers()
    end
    return ADTriggerManager.repairTriggers
end

-- returns only suitable fuel triggers according to required fuel types
function ADTriggerManager.getRefuelTriggers(vehicle, ignoreFillLevel)
    local refuelTriggers = {}
    local refuelFillTypes = AutoDrive.getRequiredRefuels(vehicle, ignoreFillLevel)
    if #refuelFillTypes > 0 then

        for _, trigger in pairs(ADTriggerManager.getLoadTriggers()) do
            local fillLevels = {}

            if trigger.source and trigger.source.getAllFillLevels then
                fillLevels, _ = trigger.source:getAllFillLevels(vehicle:getOwnerFarmId())
            end

            if table.count(fillLevels) > 0 then
                local hasFill = false
                for _, refuelFillType in pairs(refuelFillTypes) do
                    if trigger.fillTypes and trigger.fillTypes[refuelFillType] then
                        hasFill = hasFill or (fillLevels[refuelFillType] and fillLevels[refuelFillType] > 0)
                        if hasFill then
                            local isVehicleTrigger = trigger.triggerNode and CollisionFlag.getHasMaskFlagSet(trigger.triggerNode, CollisionFlag.FILLABLE) and CollisionFlag.getHasGroupFlagSet(trigger.triggerNode, CollisionFlag.TRIGGER)
                            AutoDrive.debugPrint(vehicle, AutoDrive.DC_VEHICLEINFO, "ADTriggerManager.getRefuelTriggers hasFill %s isVehicleTrigger %s", tostring(hasFill), tostring(isVehicleTrigger))
                            if isVehicleTrigger then
                                local triggerX, _, triggerZ = ADTriggerManager.getTriggerPos(trigger)
                                AutoDrive.debugPrint(vehicle, AutoDrive.DC_VEHICLEINFO, "ADTriggerManager.getRefuelTriggers Pos: %s,%s", tostring(triggerX), tostring(triggerZ))
                            end
                            if isVehicleTrigger and not table.contains(refuelTriggers, trigger) then
                                table.insert(refuelTriggers, trigger)
                            end
                        end
                    end
                end
            end
        end
    end

    return refuelTriggers
end

function ADTriggerManager.getClosestRefuelTrigger(vehicle, ignoreFillLevel)
    local refuelTriggers = ADTriggerManager.getRefuelTriggers(vehicle, ignoreFillLevel)
    local x, _, z = getWorldTranslation(vehicle.components[1].node)

    local closestRefuelTrigger = nil
    local closestDistance = math.huge

    for _, refuelTrigger in pairs(refuelTriggers) do
        local triggerX, _, triggerZ = ADTriggerManager.getTriggerPos(refuelTrigger)
        if triggerX then
            local distance = MathUtil.vector2Length(triggerX - x, triggerZ - z)

            if distance < closestDistance then
                closestDistance = distance
                closestRefuelTrigger = refuelTrigger
            end
        end
    end
    return closestRefuelTrigger
end

function ADTriggerManager.getRefuelDestinations(vehicle, ignoreFillLevel)
    local refuelDestinations = {}

    local refuelTriggers = ADTriggerManager.getRefuelTriggers(vehicle, ignoreFillLevel)

    for mapMarkerID, mapMarker in pairs(ADGraphManager:getMapMarkers()) do
        for _, refuelTrigger in pairs(refuelTriggers) do
            local triggerX, _, triggerZ = ADTriggerManager.getTriggerPos(refuelTrigger)
            if triggerX then
                local distance = MathUtil.vector2Length(triggerX - ADGraphManager:getWayPointById(mapMarker.id).x, triggerZ - ADGraphManager:getWayPointById(mapMarker.id).z)
                if distance < AutoDrive.MAX_REFUEL_TRIGGER_DISTANCE then
                    table.insert(refuelDestinations, {mapMarkerID = mapMarkerID, refuelTrigger = refuelTrigger, distance = distance})
                end
            end
        end
    end

    return refuelDestinations
end

function ADTriggerManager.getClosestRefuelDestination(vehicle, ignoreFillLevel)
    local refuelDestinations = ADTriggerManager.getRefuelDestinations(vehicle, ignoreFillLevel)

    local x, _, z = getWorldTranslation(vehicle.components[1].node)
    local closestRefuelDestination = nil
    local closestDistance = math.huge
    local closestRefuelTrigger = nil

    -- for _, refuelDestination in pairs(refuelDestinations) do
    for _, item in pairs(refuelDestinations) do
        local refuelX, refuelZ = ADGraphManager:getWayPointById(ADGraphManager:getMapMarkerById(item.mapMarkerID).id).x, ADGraphManager:getWayPointById(ADGraphManager:getMapMarkerById(item.mapMarkerID).id).z
        local distance = MathUtil.vector2Length(refuelX - x, refuelZ - z)       -- vehicle to destination
        if distance <= closestDistance then
            closestRefuelDestination = item.mapMarkerID
            closestRefuelTrigger = item.refuelTrigger
            closestDistance = distance
        end
    end
    if closestRefuelTrigger ~= nil then
        -- now find the closest mapMarker for the found refuel trigger
        local closestDistance2 = math.huge
        for _, item in pairs(refuelDestinations) do
            if item.refuelTrigger == closestRefuelTrigger and item.distance < closestDistance2 then
                closestRefuelTrigger = item.refuelTrigger
                closestDistance2 = item.distance
                closestRefuelDestination = item.mapMarkerID
            end
        end
    end

    return closestRefuelDestination
end

function ADTriggerManager.getTriggerPos(trigger)
    local x, y, z = 0, 0, 0
    if trigger.triggerNode ~= nil and g_currentMission.nodeToObject[trigger.triggerNode] ~= nil and entityExists(trigger.triggerNode) then
        x, y, z = getWorldTranslation(trigger.triggerNode)
    end
    if trigger.exactFillRootNode ~= nil and g_currentMission.nodeToObject[trigger.exactFillRootNode] ~= nil and entityExists(trigger.exactFillRootNode) then
        x, y, z = getWorldTranslation(trigger.exactFillRootNode)
    end
    if trigger.baleTrigger ~= nil then
        local node = trigger.baleTrigger.triggerNode
        if node ~= nil and g_currentMission.nodeToObject[node] ~= nil and entityExists(node) then
            x, y, z = getWorldTranslation(node)
        end
    end
    if trigger.woodTrigger ~= nil then
        local node = trigger.woodTrigger.triggerNode
        if node ~= nil and g_currentMission.nodeToObject[node] ~= nil and entityExists(node) then
            x, y, z = getWorldTranslation(node)
        end
    end
    if trigger.bunkerSiloArea and trigger.interactionTriggerNode ~= nil and g_currentMission.nodeToObject[trigger.interactionTriggerNode] ~= nil and entityExists(trigger.interactionTriggerNode) then
        x, y, z = getWorldTranslation(trigger.interactionTriggerNode)
    end
    return x, y, z
end

function ADTriggerManager.getMaxBunkerSiloLength()
    return ADTriggerManager.maxBunkerSiloLength
end

function ADTriggerManager:loadTriggerLoad(superFunc, ...)
    local result = superFunc(self, ...)

    if ADTriggerManager ~= nil and ADTriggerManager.siloTriggers ~= nil then
        if not table.contains(ADTriggerManager.siloTriggers, self) then
            table.insert(ADTriggerManager.siloTriggers, self)
        end
    end

    return result
end

function ADTriggerManager:loadTriggerDelete(superFunc)
    if ADTriggerManager ~= nil and ADTriggerManager.siloTriggers ~= nil then
        table.removeValue(ADTriggerManager.siloTriggers, self)
    end
    superFunc(self)
end

function ADTriggerManager:onPlaceableBuy()
    ADTriggerManager.searchedForTriggers = false
    ADTriggerManager.maxBunkerSiloLength = 0
    ADTriggerManager.loadAllTriggers()
end

function ADTriggerManager:onPlaceableSell()
    local bunkerSilo = self.spec_bunkerSilo and self.spec_bunkerSilo.bunkerSilo
    if bunkerSilo then
        if bunkerSilo.ad == nil then
            bunkerSilo.ad = {}
        end
        bunkerSilo.ad.isSold = true
    end
    ADTriggerManager.searchedForTriggers = false
    ADTriggerManager.maxBunkerSiloLength = 0
    ADTriggerManager.loadAllTriggers()
end

function ADTriggerManager.triggerSupportsFillType(trigger, fillType)
    if fillType > 0 then
        if trigger ~= nil and trigger.getIsFillTypeSupported then
            return trigger:getIsFillTypeSupported(fillType)
        end
    end
    return false
end

function ADTriggerManager.getAllTriggersForFillType(fillType)
    local triggers = {}

    for _, trigger in pairs(ADTriggerManager.getUnloadTriggers()) do
        if ADTriggerManager.triggerSupportsFillType(trigger, fillType) then
            table.insert(triggers, trigger)
        end
    end

    return triggers
end

function ADTriggerManager:getHighestPayingSellStation(fillType)
    local bestSellingStation = nil
    local bestPrice = -1

    for _, station in pairs(g_currentMission.storageSystem:getUnloadingStations()) do
        --AutoDrive.dumpTable(station, "Station:", 1)
		if station:isa(SellingStation) and not station.hideFromPricesMenu and station.isSellingPoint and station.supportedFillTypes[fillType] and station.unloadTriggers ~= nil and #station.unloadTriggers > 0 then
			station.uiName = station:getName()
            local price = station:getEffectiveFillTypePrice(fillType)
            if price > bestPrice then
                bestPrice = price
                bestSellingStation = station
            end
		end
	end

    return bestSellingStation
end

function ADTriggerManager:getBestPickupLocationFor(vehicle, trailer, fillType)
    local farmId = -1
    if vehicle.spec_enterable ~= nil and vehicle.spec_enterable.controllerFarmId ~= nil and vehicle.spec_enterable.controllerFarmId ~= 0 then
        farmId = vehicle.spec_enterable.controllerFarmId
    elseif vehicle.spec_aiVehicle ~= nil and vehicle.spec_aiVehicle.startedFarmId ~= nil and vehicle.spec_aiVehicle.startedFarmId ~= 0 then
        farmId = vehicle.spec_aiVehicle.startedFarmId
    end

    if farmId <= 0 then
        return
    end

    local validLoadingStations = {}
    local closestDistance = math.huge

	for _, loadingStation in pairs(g_currentMission.storageSystem:getLoadingStations()) do
		if g_currentMission.accessHandler:canFarmAccess(farmId, loadingStation) then
            local aifillTypes = loadingStation:getAISupportedFillTypes()
			if aifillTypes[fillType] and loadingStation:getFillLevel(fillType, farmId) > 0 then
                if loadingStation.getAITargetPositionAndDirection ~= nil then
                    local x, z, xDir, zDir = loadingStation:getAITargetPositionAndDirection(FillType.UNKNOWN)

                    table.insert(validLoadingStations, loadingStation)
                end

			end
		end
	end

    -- Todo: Sort by owned first and then by distance
    if #validLoadingStations > 0 then
        local vehicleX, _, vehicleZ = getWorldTranslation(vehicle.components[1].node)
        local closestLoadingStation = validLoadingStations[1]
        local closestDistance = math.huge
        for _, loadingStation in pairs(validLoadingStations) do
            if loadingStation.getAITargetPositionAndDirection ~= nil then
                local x, z, xDir, zDir = loadingStation:getAITargetPositionAndDirection(FillType.UNKNOWN)
                local dis = MathUtil.vector2Length(vehicleX - x, vehicleZ - z)
                if dis < closestDistance then
                    closestDistance = dis
                    closestLoadingStation = loadingStation
                end
            end
        end
        return closestLoadingStation
    end
end

function ADTriggerManager:getMarkerAtStation(sellingStation, vehicle, maxTriggerDistance)
    local maxTriggerDis = maxTriggerDistance or 6
    local closest = -1
    if sellingStation ~= nil then
        local x, z, xDir, zDir = 0,0,0,0

        if sellingStation.getAITargetPositionAndDirection ~= nil and sellingStation:getAITargetPositionAndDirection(FillType.UNKNOWN) ~= nil then
            x, z, xDir, zDir = sellingStation:getAITargetPositionAndDirection(FillType.UNKNOWN)
        elseif sellingStation.unloadTriggers ~= nil and #sellingStation.unloadTriggers > 0 then
            if sellingStation.unloadTriggers[1].supportsAIUnloading then
                x, z, xDir, zDir = sellingStation.unloadTriggers[1]:getAITargetPositionAndDirection()
            elseif sellingStation.unloadTriggers[1].exactFillRootNode ~= nil then
                x, _, z = getWorldTranslation(sellingStation.unloadTriggers[1].exactFillRootNode)
            elseif sellingStation.unloadTriggers[1].baleTrigger ~= nil then
                local node = sellingStation.unloadTriggers[1].baleTrigger.triggerNode
                x, _, z = getWorldTranslation(node)
            else
                return -1
            end
        else
            return -1
        end

        -- Now find suitable node in the network
        local minDistance = AutoDrive.getTractorTrainLength(vehicle, true, false) + 3
        local distance = minDistance + 10

        --First look for suitable marker
        for mapMarkerID, mapMarker in pairs(ADGraphManager:getMapMarkers()) do
            local dis = MathUtil.vector2Length(ADGraphManager:getWayPointById(mapMarker.id).x - x, ADGraphManager:getWayPointById(mapMarker.id).z - z)
            if dis < distance and dis > minDistance then
                -- check if this is in the right direction
                local wp = ADGraphManager:getWayPointById(mapMarker.id)
                local isOnPathOverTrigger = AutoDrive:checkIfPathTraversedOverPosition(wp, {x=x, z=z}, maxTriggerDis, 20)

                if wp.incoming ~= nil and #wp.incoming > 0 and isOnPathOverTrigger then
                    local disIncoming = MathUtil.vector2Length(ADGraphManager:getWayPointById(wp.incoming[1]).x - x, ADGraphManager:getWayPointById(wp.incoming[1]).z - z)
                    if disIncoming < dis then
                        closest = mapMarker.id
                        distance = dis
                    end
                end
            end
        end

        if closest == -1 then
            -- Else look for waypoint and create marker
            -- Todo: first check for a closest point and then traverse until one meets the requirements

            local closestNode = nil
            local closestNodeDistance = math.huge
            for i in pairs(ADGraphManager:getWayPoints()) do
                local dis = MathUtil.vector2Length(ADGraphManager:getWayPointById(i).x - x, ADGraphManager:getWayPointById(i).z - z)
                if dis < closestNodeDistance and dis < maxTriggerDis then
                    closestNode = i
                    closestNodeDistance = dis
                end
            end

            if closestNode ~= nil then
                local pointWithEnoughDistance = AutoDrive:getNodeWithMinDistanceTo(ADGraphManager:getWayPointById(closestNode), {x=x, z=z}, minDistance, 20)
                if pointWithEnoughDistance ~= nil then
                    closest = pointWithEnoughDistance.id
                end
            end

            if closest >= 0 then
                local markerName = "NoName"
                if sellingStation.uiName ~= nil then
                    markerName = sellingStation.uiName
                elseif sellingStation.getName ~= nil then
                    markerName = sellingStation:getName()
                end
                ADGraphManager:createMapMarker(closest, markerName)
            end
        end
    end
    return closest
end

function ADTriggerManager.addBunkerSiloAreaV(bunkerSilo)
    if bunkerSilo and bunkerSilo.bunkerSiloArea and bunkerSilo.bunkerSiloArea.sx then
        local sx, sy, sz = bunkerSilo.bunkerSiloArea.sx, bunkerSilo.bunkerSiloArea.sy, bunkerSilo.bunkerSiloArea.sz
        local wx, wy, wz = bunkerSilo.bunkerSiloArea.wx, bunkerSilo.bunkerSiloArea.wy, bunkerSilo.bunkerSiloArea.wz
        local hx, hy, hz = bunkerSilo.bunkerSiloArea.hx, bunkerSilo.bunkerSiloArea.hy, bunkerSilo.bunkerSiloArea.hz
        local vx = hx + (wx - sx)
        local vy = hy + (wy - sy)
        local vz = hz + (wz - sz)
        bunkerSilo.bunkerSiloArea.vx = vx
        bunkerSilo.bunkerSiloArea.vy = vy
        bunkerSilo.bunkerSiloArea.vz = vz
        return true
    else
        return false
    end
end

function ADTriggerManager.getBunkerSiloAreasConnectionType(bunkerSilo1, bunkerSilo2)
    if bunkerSilo1 and bunkerSilo1.bunkerSiloArea and bunkerSilo1.bunkerSiloArea.vx and
        bunkerSilo2 and bunkerSilo2.bunkerSiloArea and bunkerSilo2.bunkerSiloArea.vx then
        if math.abs(bunkerSilo1.bunkerSiloArea.sx - bunkerSilo2.bunkerSiloArea.hx) < AutoDrive.BUNKERSILO_CONNECTED_DISTANCE and
            math.abs(bunkerSilo1.bunkerSiloArea.sz - bunkerSilo2.bunkerSiloArea.hz) < AutoDrive.BUNKERSILO_CONNECTED_DISTANCE and
            math.abs(bunkerSilo1.bunkerSiloArea.wx - bunkerSilo2.bunkerSiloArea.vx) < AutoDrive.BUNKERSILO_CONNECTED_DISTANCE and
            math.abs(bunkerSilo1.bunkerSiloArea.wz - bunkerSilo2.bunkerSiloArea.vz) < AutoDrive.BUNKERSILO_CONNECTED_DISTANCE then
            return 1
        end
        if math.abs(bunkerSilo1.bunkerSiloArea.sx - bunkerSilo2.bunkerSiloArea.wx) < AutoDrive.BUNKERSILO_CONNECTED_DISTANCE and
            math.abs(bunkerSilo1.bunkerSiloArea.sz - bunkerSilo2.bunkerSiloArea.wz) < AutoDrive.BUNKERSILO_CONNECTED_DISTANCE and
            math.abs(bunkerSilo1.bunkerSiloArea.wx - bunkerSilo2.bunkerSiloArea.sx) < AutoDrive.BUNKERSILO_CONNECTED_DISTANCE and
            math.abs(bunkerSilo1.bunkerSiloArea.wz - bunkerSilo2.bunkerSiloArea.sz) < AutoDrive.BUNKERSILO_CONNECTED_DISTANCE then
            return 2
        end
        if math.abs(bunkerSilo1.bunkerSiloArea.vx - bunkerSilo2.bunkerSiloArea.hx) < AutoDrive.BUNKERSILO_CONNECTED_DISTANCE and
            math.abs(bunkerSilo1.bunkerSiloArea.vz - bunkerSilo2.bunkerSiloArea.hz) < AutoDrive.BUNKERSILO_CONNECTED_DISTANCE and
            math.abs(bunkerSilo1.bunkerSiloArea.hx - bunkerSilo2.bunkerSiloArea.vx) < AutoDrive.BUNKERSILO_CONNECTED_DISTANCE and
            math.abs(bunkerSilo1.bunkerSiloArea.hz - bunkerSilo2.bunkerSiloArea.vz) < AutoDrive.BUNKERSILO_CONNECTED_DISTANCE then
            return 3
        end
        if math.abs(bunkerSilo1.bunkerSiloArea.vx - bunkerSilo2.bunkerSiloArea.wx) < AutoDrive.BUNKERSILO_CONNECTED_DISTANCE and
            math.abs(bunkerSilo1.bunkerSiloArea.vz - bunkerSilo2.bunkerSiloArea.wz) < AutoDrive.BUNKERSILO_CONNECTED_DISTANCE and
            math.abs(bunkerSilo1.bunkerSiloArea.hx - bunkerSilo2.bunkerSiloArea.sx) < AutoDrive.BUNKERSILO_CONNECTED_DISTANCE and
            math.abs(bunkerSilo1.bunkerSiloArea.hz - bunkerSilo2.bunkerSiloArea.sz) < AutoDrive.BUNKERSILO_CONNECTED_DISTANCE then
            return 4
        end
    end
    return 0
end

function ADTriggerManager.connectBunkerSilos(bunkerSilo1, bunkerSilo2, bunkerSiloConnectionType)
    local bunkerSilo = {}
    local sx, sy, sz, wx, wy, wz, hx, hy, hz, vx, vy, vz, dwx, dwy, dwz, dhx, dhy, dhz
    if bunkerSiloConnectionType == 1 then
        sx = bunkerSilo2.bunkerSiloArea.sx
        sy = bunkerSilo2.bunkerSiloArea.sy
        sz = bunkerSilo2.bunkerSiloArea.sz
        wx = bunkerSilo2.bunkerSiloArea.wx
        wy = bunkerSilo2.bunkerSiloArea.wy
        wz = bunkerSilo2.bunkerSiloArea.wz
        hx = bunkerSilo1.bunkerSiloArea.hx
        hy = bunkerSilo1.bunkerSiloArea.hy
        hz = bunkerSilo1.bunkerSiloArea.hz
        vx = bunkerSilo1.bunkerSiloArea.vx
        vy = bunkerSilo1.bunkerSiloArea.vy
        vz = bunkerSilo1.bunkerSiloArea.vz
    elseif bunkerSiloConnectionType == 2 then
        sx = bunkerSilo2.bunkerSiloArea.vx
        sy = bunkerSilo2.bunkerSiloArea.vy
        sz = bunkerSilo2.bunkerSiloArea.vz
        wx = bunkerSilo2.bunkerSiloArea.hx
        wy = bunkerSilo2.bunkerSiloArea.hy
        wz = bunkerSilo2.bunkerSiloArea.hz
        hx = bunkerSilo1.bunkerSiloArea.hx
        hy = bunkerSilo1.bunkerSiloArea.hy
        hz = bunkerSilo1.bunkerSiloArea.hz
        vx = bunkerSilo1.bunkerSiloArea.vx
        vy = bunkerSilo1.bunkerSiloArea.vy
        vz = bunkerSilo1.bunkerSiloArea.vz
    elseif bunkerSiloConnectionType == 3 then
        sx = bunkerSilo2.bunkerSiloArea.sx
        sy = bunkerSilo2.bunkerSiloArea.sy
        sz = bunkerSilo2.bunkerSiloArea.sz
        wx = bunkerSilo2.bunkerSiloArea.wx
        wy = bunkerSilo2.bunkerSiloArea.wy
        wz = bunkerSilo2.bunkerSiloArea.wz
        hx = bunkerSilo1.bunkerSiloArea.wx
        hy = bunkerSilo1.bunkerSiloArea.wy
        hz = bunkerSilo1.bunkerSiloArea.wz
        vx = bunkerSilo1.bunkerSiloArea.sx
        vy = bunkerSilo1.bunkerSiloArea.sy
        vz = bunkerSilo1.bunkerSiloArea.sz
    elseif bunkerSiloConnectionType == 4 then
        sx = bunkerSilo2.bunkerSiloArea.vx
        sy = bunkerSilo2.bunkerSiloArea.vy
        sz = bunkerSilo2.bunkerSiloArea.vz
        wx = bunkerSilo2.bunkerSiloArea.hx
        wy = bunkerSilo2.bunkerSiloArea.hy
        wz = bunkerSilo2.bunkerSiloArea.hz
        hx = bunkerSilo1.bunkerSiloArea.wx
        hy = bunkerSilo1.bunkerSiloArea.wy
        hz = bunkerSilo1.bunkerSiloArea.wz
        vx = bunkerSilo1.bunkerSiloArea.sx
        vy = bunkerSilo1.bunkerSiloArea.sy
        vz = bunkerSilo1.bunkerSiloArea.sz
    else
        return nil
    end
    dwx = wx - sx
    dwy = wy - sy
    dwz = wz - sz
    dhx = hx - sx
    dhy = hy - sy
    dhz = hz - sz
    bunkerSilo.bunkerSiloArea = {sx = sx, sy = sy, sz = sz, wx = wx, wy =wy, wz = wz, hx = hx, hy = hy, hz = hz, vx = vx, vy = vy, vz = vz, dwx = dwx, dwy = dwy, dwz = dwz, dhx = dhx, dhy = dhy, dhz = dhz}
    bunkerSilo.interactionTriggerNode = bunkerSilo1.interactionTriggerNode
    return bunkerSilo
end

function ADTriggerManager.addBunkerSilo(bunkerSilo)
    local isSold = bunkerSilo.ad and bunkerSilo.ad.isSold
    if table.contains(ADTriggerManager.bunkerSilos, bunkerSilo) or isSold then
        -- bunkerSilo already added or sold
        return
    end

    ADTriggerManager.addBunkerSiloAreaV(bunkerSilo)
    table.insert(ADTriggerManager.bunkerSilos, bunkerSilo)

    local isConnected = false
    if #ADTriggerManager.bunkerSilosResult > 0 then
        for _, bunkerSiloResult in ipairs(ADTriggerManager.bunkerSilosResult) do
            local isOppositeConnected = false
            local bunkerSiloConnectionType = ADTriggerManager.getBunkerSiloAreasConnectionType(bunkerSilo, bunkerSiloResult)
            if bunkerSiloConnectionType == 0 then
                -- test the opposite direction
                bunkerSiloConnectionType = ADTriggerManager.getBunkerSiloAreasConnectionType(bunkerSiloResult, bunkerSilo)
                if bunkerSiloConnectionType > 0 then
                    isOppositeConnected = true
                end
            end
            if bunkerSiloConnectionType > 0 then
                local bunkerSiloConnected
                if not isOppositeConnected then
                    bunkerSiloConnected = ADTriggerManager.connectBunkerSilos(bunkerSilo, bunkerSiloResult, bunkerSiloConnectionType)
                else
                    bunkerSiloConnected = ADTriggerManager.connectBunkerSilos(bunkerSiloResult, bunkerSilo, bunkerSiloConnectionType)
                end
                if bunkerSiloConnected then
                    local length = MathUtil.vector2Length(bunkerSiloConnected.bunkerSiloArea.hx - bunkerSiloConnected.bunkerSiloArea.sx, bunkerSiloConnected.bunkerSiloArea.hz - bunkerSiloConnected.bunkerSiloArea.sz)
                    ADTriggerManager.maxBunkerSiloLength = math.max(ADTriggerManager.maxBunkerSiloLength, length)
                    table.removeValue(ADTriggerManager.bunkerSilosResult, bunkerSiloResult)
                    table.insert(ADTriggerManager.bunkerSilosResult, bunkerSiloConnected)
                    isConnected = true
                    break
                end
            end
        end
    end

    if not isConnected then
        local length = MathUtil.vector2Length(bunkerSilo.bunkerSiloArea.hx - bunkerSilo.bunkerSiloArea.sx, bunkerSilo.bunkerSiloArea.hz - bunkerSilo.bunkerSiloArea.sz)
        ADTriggerManager.maxBunkerSiloLength = math.max(ADTriggerManager.maxBunkerSiloLength, length)
        table.insert(ADTriggerManager.bunkerSilosResult, bunkerSilo)
    end
end

function AutoDrive:checkIfPathTraversedOverPosition(wayPoint, targetPosition, radius, maxSteps)
    local maxSearchSteps = maxSteps or 30
    if maxSearchSteps <= 0 then
        return false
    end
    local distance = MathUtil.vector2Length(wayPoint.x - targetPosition.x, wayPoint.z - targetPosition.z)
    if distance < radius then
        return true
    end
    for _, incomingId in pairs(wayPoint.incoming) do
        if AutoDrive:checkIfPathTraversedOverPosition(ADGraphManager:getWayPointById(incomingId), targetPosition, radius, maxSearchSteps - 1) then
            return true
        end
    end
    return false
end

function AutoDrive:getNodeWithMinDistanceTo(wayPoint, targetPosition, minDistance, maxSteps)
    local maxSearchSteps = maxSteps or 30
    if maxSearchSteps <= 0 then
        return nil
    end
    local distance = MathUtil.vector2Length(wayPoint.x - targetPosition.x, wayPoint.z - targetPosition.z)
    if distance > minDistance then
        return wayPoint
    end
    for _, outId in pairs(wayPoint.out) do
        local result = AutoDrive:getNodeWithMinDistanceTo(ADGraphManager:getWayPointById(outId), targetPosition, minDistance, maxSearchSteps - 1)
        if result ~= nil then
            return result
        end
    end
    return nil
end

function AutoDrive:getClosestRepairTrigger(vehicle)
    local x, y, z = getWorldTranslation(vehicle.components[1].node)
    local distance = math.huge
    local maxDistance = 15
    local closest = nil

    -- Check ownerFarmId
    local repairMarkers = {}
    local ownedRepairMarkers = {}
    for _, repairTrigger in pairs(ADTriggerManager.getRepairTriggers()) do
        local triggerX, _, triggerZ = getWorldTranslation(repairTrigger.node)

        --First look for suitable marker
        for mapMarkerID, mapMarker in pairs(ADGraphManager:getMapMarkers()) do
            local dis = MathUtil.vector2Length(ADGraphManager:getWayPointById(mapMarker.id).x - triggerX, ADGraphManager:getWayPointById(mapMarker.id).z - triggerZ)
            if dis < distance and dis < maxDistance then
                closest = mapMarker.id
                distance = dis
            end
        end

        if closest ~= nil then
            table.insert(repairMarkers, {marker=closest, distance=MathUtil.vector2Length(ADGraphManager:getWayPointById(closest).x - x, ADGraphManager:getWayPointById(closest).z - z)})
            if vehicle.getOwnerFarmId ~= nil and vehicle:getOwnerFarmId() == repairTrigger.owner then
                table.insert(ownedRepairMarkers, {marker=closest, distance=MathUtil.vector2Length(ADGraphManager:getWayPointById(closest).x - x, ADGraphManager:getWayPointById(closest).z - z)})
            end
        end

        distance = math.huge
        closest = nil
    end

    if #ownedRepairMarkers > 0 then
        repairMarkers = ownedRepairMarkers
    end

    for _, repairMarker in pairs(repairMarkers) do
        if repairMarker.distance < distance then
            closest = repairMarker
            distance = repairMarker.distance
        end
    end

    return closest
end
