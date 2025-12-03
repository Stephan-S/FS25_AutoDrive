AutoDrive.destinationListeners = {}

--startX, startZ: World location
--startYRot: rotation in rad
--destinationID: ID of marker to find path to
--options (optional): options.minDistance, options.maxDistance (default 1m, 20m) define boundaries between the first AutoDrive waypoint and the starting location.
function AutoDrive:GetPath(startX, startZ, startYRot, destinationID, options)
    AutoDrive.debugPrint(nil, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:GetPath(%s, %s, %s, %s, %s)", startX, startZ, startYRot, destinationID, options)
    if startX == nil or startZ == nil or startYRot == nil or destinationID == nil or ADGraphManager:getMapMarkerById(destinationID) == nil then
        return
    end
    startYRot = AutoDrive.normalizeAngleToPlusMinusPI(startYRot)
    local markerName = ADGraphManager:getMapMarkerById(destinationID).name
    local startPoint = {x = startX, z = startZ}
    local minDistance = 1
    local maxDistance = 20
    if options ~= nil and options.minDistance ~= nil then
        minDistance = options.minDistance
    end
    if options ~= nil and options.maxDistance ~= nil then
        maxDistance = options.maxDistance
    end
    local directionVec = {x = math.sin(startYRot), z = math.cos(startYRot)}
    local bestPoint = ADGraphManager:findMatchingWayPoint(startPoint, directionVec, ADGraphManager:getWayPointsInRange(startPoint, minDistance, maxDistance))

    if bestPoint == -1 then
        bestPoint = AutoDrive:GetClosestPointToLocation(startX, startZ, minDistance)
        if bestPoint == -1 then
            return
        end
    end

    return ADGraphManager:FastShortestPath(bestPoint, markerName, ADGraphManager:getMapMarkerById(destinationID).id)
end

function AutoDrive:GetPathVia(startX, startZ, startYRot, viaID, destinationID, options)
    AutoDrive.debugPrint(nil, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:GetPathVia(%s, %s, %s, %s, %s, %s)", startX, startZ, startYRot, viaID, destinationID, options)
    if startX == nil or startZ == nil or startYRot == nil or destinationID == nil or ADGraphManager:getMapMarkerById(destinationID) == nil or viaID == nil or ADGraphManager:getMapMarkerById(viaID) == nil then
        return
    end
    startYRot = AutoDrive.normalizeAngleToPlusMinusPI(startYRot)

    local markerName = ADGraphManager:getMapMarkerById(viaID).name
    local startPoint = {x = startX, z = startZ}
    local minDistance = 1
    local maxDistance = 20
    if options ~= nil and options.minDistance ~= nil then
        minDistance = options.minDistance
    end
    if options ~= nil and options.maxDistance ~= nil then
        maxDistance = options.maxDistance
    end
    local directionVec = {x = math.sin(startYRot), z = math.cos(startYRot)}
    local bestPoint = ADGraphManager:findMatchingWayPoint(startPoint, directionVec, ADGraphManager:getWayPointsInRange(startPoint, minDistance, maxDistance))

    if bestPoint == -1 then
        bestPoint = AutoDrive:GetClosestPointToLocation(startX, startZ, minDistance)
        if bestPoint == -1 then
            return
        end
    end

    local toViaID = ADGraphManager:FastShortestPath(bestPoint, markerName, ADGraphManager:getMapMarkerById(viaID).id)

    if toViaID == nil or #toViaID < 1 then
        return
    end

    local fromViaID = ADGraphManager:FastShortestPath(toViaID[#toViaID].id, ADGraphManager:getMapMarkerById(destinationID).name, ADGraphManager:getMapMarkerById(destinationID).id)

    for i, wayPoint in pairs(fromViaID) do
        if i > 1 then
            table.insert(toViaID, wayPoint)
        end
    end

    return toViaID
end

function AutoDrive:GetDriverName(vehicle)
    return vehicle.ad.stateModule:getName()
end

function AutoDrive:GetAvailableDestinations()
    AutoDrive.debugPrint(nil, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:GetAvailableDestinations()")
    local destinations = {}
    for markerID, marker in pairs(ADGraphManager:getMapMarkers()) do
        local point = ADGraphManager:getWayPointById(marker.id)
        if point ~= nil then
            destinations[markerID] = {name = marker.name, x = point.x, y = point.y, z = point.z, id = markerID}
        end
    end
    return destinations
end

function AutoDrive:GetClosestPointToLocation(x, z, minDistance)
    AutoDrive.debugPrint(nil, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:GetClosestPointToLocation(%s, %s, %s)", x, z, minDistance)
    local closest = -1
    if ADGraphManager:getWayPointsCount() < 1 then
        local distance = math.huge

        for i in pairs(ADGraphManager:getWayPoints()) do
            local dis = MathUtil.vector2Length(ADGraphManager:getWayPointById(i).x - x, ADGraphManager:getWayPointById(i).z - z)
            if dis < distance and dis >= minDistance then
                closest = i
                distance = dis
            end
        end
    end

    return closest
end

function AutoDrive:StartDriving(vehicle, destinationID, unloadDestinationID, callBackObject, callBackFunction, callBackArg)
    AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:StartDriving(%s, %s, %s, %s, %s)", destinationID, unloadDestinationID, callBackObject, callBackFunction, callBackArg)
    if vehicle ~= nil and vehicle.ad ~= nil and not vehicle.ad.stateModule:isActive() then
        vehicle.ad.callBackObject = callBackObject
        vehicle.ad.callBackFunction = callBackFunction
        vehicle.ad.callBackArg = callBackArg

        if destinationID ~= nil and destinationID >= 0 and ADGraphManager:getMapMarkerById(destinationID) ~= nil 
            and unloadDestinationID ~= nil and unloadDestinationID >= 0 and ADGraphManager:getMapMarkerById(unloadDestinationID) ~= nil then
            vehicle.ad.stateModule:setFirstMarker(destinationID)
        end
        if unloadDestinationID ~= nil then
            if unloadDestinationID >= 0 and ADGraphManager:getMapMarkerById(unloadDestinationID) ~= nil then
                vehicle.ad.stateModule:setSecondMarker(unloadDestinationID)
                vehicle.ad.stateModule:getCurrentMode():start()
            elseif unloadDestinationID == -3 then --park
                local parkDestinationAtJobFinished = vehicle.ad.stateModule:getParkDestinationAtJobFinished()
                if parkDestinationAtJobFinished >= 1 then
                    local trailers, _ = AutoDrive.getAllUnits(vehicle)
                    local fillLevel, _, _ = AutoDrive.getAllFillLevels(trailers)
                    if vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER and fillLevel > 0 then
                        -- unload before going to park
                        AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:StartDriving unload before going to park")
                        vehicle.ad.stateModule:setMode(AutoDrive.MODE_DELIVERTO)
                        vehicle.ad.stateModule:setFirstMarker(vehicle.ad.stateModule:getSecondMarkerId())
                    else
                        vehicle.ad.stateModule:setMode(AutoDrive.MODE_DRIVETO)
                        vehicle.ad.stateModule:setFirstMarker(parkDestinationAtJobFinished)
                        vehicle.ad.onRouteToPark = true
                    end
                    vehicle.ad.stateModule:getCurrentMode():start()
                else
                    AutoDriveMessageEvent.sendMessage(vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_parkVehicle_noPosSet;", 5000)
                    -- stop vehicle movement
                    vehicle.ad.trailerModule:handleTrailerReversing(false)
                    AutoDrive.driveInDirection(vehicle, 16, 30, 0, 0.2, 20, false, false, 0, 0, 0, 1)
                    vehicle:setCruiseControlState(Drivable.CRUISECONTROL_STATE_OFF)
                    if vehicle.stopMotor ~= nil then
                        vehicle:stopMotor()
                    end
                end
            else --unloadDestinationID == -2 refuel
                -- vehicle.ad.stateModule:setMode(AutoDrive.MODE_DRIVETO) -- should fix #1477
                vehicle.ad.stateModule:getCurrentMode():start()
            end
        end
    end
end

function AutoDrive:StartDrivingWithPathFinder(vehicle, destinationID, unloadDestinationID, callBackObject, callBackFunction, callBackArg)
    AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:StartDrivingWithPathFinder(%s, %s, %s, %s, %s)", destinationID, unloadDestinationID, callBackObject, callBackFunction, callBackArg)
    if vehicle ~= nil and vehicle.ad ~= nil and not vehicle.ad.stateModule:isActive() then
        if unloadDestinationID < -1 then
            if unloadDestinationID == -3 then --park
                AutoDrive:StartDriving(vehicle, destinationID, unloadDestinationID, callBackObject, callBackFunction, callBackArg)
            elseif unloadDestinationID == -2 then --refuel
                AutoDrive:StartDriving(vehicle, vehicle.ad.stateModule:getFirstMarkerId(), unloadDestinationID, callBackObject, callBackFunction, callBackArg)
            end
        else
            AutoDrive:StartDriving(vehicle, destinationID, unloadDestinationID, callBackObject, callBackFunction, callBackArg)
        end
    end
end

function AutoDrive:GetParkDestination(vehicle)
    AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:GetParkDestination()")
    if vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.stateModule ~= nil then
        local parkDestinationAtJobFinished = vehicle.ad.stateModule:getParkDestinationAtJobFinished()
        if parkDestinationAtJobFinished >= 1 then
            return parkDestinationAtJobFinished
        end
    end
    return nil
end

function AutoDrive:registerDestinationListener(callBackObject, callBackFunction)
    AutoDrive.debugPrint(nil, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:registerDestinationListener(%s, %s)", callBackObject, callBackFunction)
    if AutoDrive.destinationListeners[callBackObject] == nil then
        AutoDrive.destinationListeners[callBackObject] = callBackFunction
    end
end

function AutoDrive:unRegisterDestinationListener(callBackObject)
    AutoDrive.debugPrint(nil, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:unRegisterDestinationListener(%s)", callBackObject)
    if AutoDrive.destinationListeners[callBackObject] ~= nil then
        AutoDrive.destinationListeners[callBackObject] = nil
    end
end

function AutoDrive:notifyDestinationListeners()
    AutoDrive.debugPrint(nil, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:notifyDestinationListeners()")
    for object, callBackFunction in pairs(AutoDrive.destinationListeners) do
        callBackFunction(object, true)
    end
end

function AutoDrive:combineIsCallingDriver(combine)	--only for CoursePlay
    AutoDrive.debugPrint(combine, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:combineIsCallingDriver hasHarvesterAvailableUnloader %s getIsCPWaitingForUnload %s"
    , tostring(ADHarvestManager:hasHarvesterAvailableUnloader(combine))
    , tostring(AutoDrive:getIsCPWaitingForUnload(combine))
    )
    return combine ~=nil and AutoDrive:getIsCPWaitingForUnload(combine) and ADHarvestManager:hasHarvesterAvailableUnloader(combine)
end

function AutoDrive:getCombineOpenPipePercent(combine)	--for AIVE
	local _, pipePercent = ADHarvestManager.getOpenPipePercent(combine)
	return pipePercent
end

-- start CP at first wayPoint
function AutoDrive:StartCP(vehicle)
    if vehicle == nil then 
        return 
    end
    AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:StartCP...")
    -- if vehicle.startCpAtFirstWp ~= nil then
        -- vehicle:startCpAtFirstWp()
    if vehicle.startCpAtLastWp ~= nil then
        vehicle:startCpAtLastWp()
    elseif vehicle.startCpALastWp ~= nil then
        vehicle:startCpALastWp()
    else
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:StartCP - Not possible. CP interface not found")
    end
end

-- restart CP to continue
function AutoDrive:RestartCP(vehicle)
    if vehicle == nil then 
        return 
    end
    AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:RestartCP...")
    if vehicle.startCpAtLastWp ~= nil then
        vehicle:startCpAtLastWp()
    elseif vehicle.startCpALastWp ~= nil then
        vehicle:startCpALastWp()
    else
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:RestartCP - Not possible. CP interface not found")
    end
end

-- stop CP if it is active
function AutoDrive:StopCP(vehicle)
    if vehicle == nil then 
        return 
    end
    local vehicleToCheck = vehicle.getRootVehicle and vehicle:getRootVehicle()

    AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:StopCP...")

    if vehicleToCheck ~= nil then
        if vehicleToCheck.cpStartStopDriver ~= nil then
            if vehicleToCheck:getIsCpActive() then
                AutoDrive.debugPrint(vehicleToCheck, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:StopCP - cpStartStopDriver")
                vehicleToCheck:cpStartStopDriver()
            end
            if vehicleToCheck.ad ~= nil and vehicleToCheck.ad.stateModule ~= nil and vehicleToCheck.ad.stateModule:getUsedHelper() == ADStateModule.HELPER_CP and  vehicleToCheck.ad.stateModule:getStartHelper() then
                -- CP button active
                -- deactivate CP button
                AutoDrive.debugPrint(vehicleToCheck, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:StopCP - deactivate CP button")
                vehicleToCheck.ad.stateModule:setStartHelper(false)
            end
            vehicleToCheck.ad.restartCP = false -- do not continue CP course
        else
            AutoDrive.debugPrint(vehicleToCheck, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:StopCP - Not possible. CP interface not found")
        end
    end
end

function AutoDrive:HoldDriving(vehicle)
    if vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.stateModule:isActive() then
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:HoldDriving should set setPaused")
        vehicle.ad.drivePathModule:setPaused()
    end
end

function AutoDrive:logCPStatus(vehicle, functionName)
    if vehicle == nil then
        return false
    end
    if (g_updateLoopIndex % 60) == 0 then
        local vehicleToCheck = vehicle.getRootVehicle and vehicle:getRootVehicle()
        if vehicleToCheck then
            if vehicleToCheck.getIsCpActive then
                AutoDrive.debugPrint(vehicleToCheck, AutoDrive.DC_EXTERNALINTERFACEINFO, "%s getIsCpActive %s", functionName, tostring(vehicleToCheck:getIsCpActive()))
            end
            if vehicleToCheck.getIsCpHarvesterWaitingForUnload then
                AutoDrive.debugPrint(vehicleToCheck, AutoDrive.DC_EXTERNALINTERFACEINFO, "%s getIsCpHarvesterWaitingForUnload %s", functionName, tostring(vehicleToCheck:getIsCpHarvesterWaitingForUnload()))
            end
            if  vehicleToCheck.getIsCpHarvesterWaitingForUnloadInPocket then
                AutoDrive.debugPrint(vehicleToCheck, AutoDrive.DC_EXTERNALINTERFACEINFO, "%s getIsCpHarvesterWaitingForUnloadInPocket %s", functionName, tostring(vehicleToCheck:getIsCpHarvesterWaitingForUnloadInPocket()))
            end
            if vehicleToCheck.getIsCpHarvesterWaitingForUnloadAfterPulledBack then
                AutoDrive.debugPrint(vehicleToCheck, AutoDrive.DC_EXTERNALINTERFACEINFO, "%s getIsCpHarvesterWaitingForUnloadAfterPulledBack %s", functionName, tostring(vehicleToCheck:getIsCpHarvesterWaitingForUnloadAfterPulledBack()))
            end
            if  vehicleToCheck.getIsCpHarvesterManeuvering then
                AutoDrive.debugPrint(vehicleToCheck, AutoDrive.DC_EXTERNALINTERFACEINFO, "%s getIsCpHarvesterManeuvering %s", functionName, tostring(vehicleToCheck:getIsCpHarvesterManeuvering()))
            end
        end
    end
end

function AutoDrive:getIsCPActive(vehicle)
    if vehicle == nil then
        return false
    end
    local vehicleToCheck = vehicle.getRootVehicle and vehicle:getRootVehicle()

    -- AutoDrive:logCPStatus(vehicleToCheck, "holdCPCombine") -- enable only if required

    if vehicleToCheck and vehicleToCheck.getIsCpActive and vehicleToCheck:getIsCpActive() then
        return true
    else
        return false
    end

end

function AutoDrive:holdCPCombine(vehicle)
    if vehicle == nil then
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:holdCPCombine start ERROR: vehicle == nil")
        return false
    end
    local vehicleToCheck = vehicle.getRootVehicle and vehicle:getRootVehicle()

    AutoDrive:logCPStatus(vehicleToCheck, "holdCPCombine")

    if vehicleToCheck and AutoDrive:getIsCPActive(vehicleToCheck) 
        and
        (
            (vehicleToCheck.holdCpHarvesterTemporarily)
        )
    then
        if (g_updateLoopIndex % 60) == 0 then
            AutoDrive.debugPrint(vehicleToCheck, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:holdCPCombine ")
        end
        vehicleToCheck:holdCpHarvesterTemporarily(200) -- hold 200 ms
    end
end

function AutoDrive:getIsCPCombineInPocket(vehicle)
    if vehicle == nil then
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:getIsCPCombineInPocket start ERROR: vehicle == nil")
        return false
    end
    local vehicleToCheck = vehicle.getRootVehicle and vehicle:getRootVehicle()

    AutoDrive:logCPStatus(vehicleToCheck, "getIsCPCombineInPocket")

    if vehicleToCheck and AutoDrive:getIsCPActive(vehicleToCheck) 
        and
        (
            (vehicleToCheck.getIsCpHarvesterWaitingForUnloadInPocket and vehicleToCheck:getIsCpHarvesterWaitingForUnloadInPocket())
            or 
            (vehicleToCheck.getIsCpHarvesterWaitingForUnloadAfterPulledBack and vehicleToCheck:getIsCpHarvesterWaitingForUnloadAfterPulledBack())
        )
    then
        AutoDrive.debugPrint(vehicleToCheck, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:getIsCPCombineInPocket AutoDrive:getIsCPCombineInPocket return true")
        return true
    else
        return false
    end

end

function AutoDrive:getIsCPWaitingForUnload(vehicle)
    if vehicle == nil then
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:getIsCPWaitingForUnload start ERROR: vehicle == nil")
        return false
    end

    local vehicleToCheck = vehicle.getRootVehicle and vehicle:getRootVehicle()

    AutoDrive:logCPStatus(vehicleToCheck, "getIsCPWaitingForUnload")

    if vehicleToCheck and AutoDrive:getIsCPActive(vehicleToCheck)
        and
        (
            (vehicleToCheck.getIsCpHarvesterWaitingForUnload and vehicleToCheck:getIsCpHarvesterWaitingForUnload())
        )
    then
        if (g_updateLoopIndex % 60) == 0 then
            AutoDrive.debugPrint(vehicleToCheck, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:getIsCPWaitingForUnload return true")
        end
        return true
    else
        return false
    end

end

function AutoDrive:getIsCPTurning(vehicle)
    if vehicle == nil then
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:getIsCPTurning start ERROR: vehicle == nil")
        return false
    end
    local vehicleToCheck = vehicle.getRootVehicle and vehicle:getRootVehicle()

    AutoDrive:logCPStatus(vehicleToCheck, "getIsCPTurning")

    if vehicleToCheck and AutoDrive:getIsCPActive(vehicleToCheck) 
        and 
        (
            (vehicleToCheck.getIsCpHarvesterManeuvering and vehicleToCheck:getIsCpHarvesterManeuvering())
        )
    then
        if (g_updateLoopIndex % 60) == 0 then
            AutoDrive.debugPrint(vehicleToCheck, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:getIsCPTurning return true")
        end
        return true
    else
        return false
    end

end

-- CP event handler
function AutoDrive:onCpFinished()
    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:onCpFinished start... enableParkAtJobFinished %s", AutoDrive.getSetting("enableParkAtJobFinished", self))
    if self.isServer then
        if self.ad and self.ad.stateModule and self.startAutoDrive and AutoDrive.getSetting("enableParkAtJobFinished", self) then
            self.ad.onRouteToRefuel = false
            self.ad.onRouteToRepair = false
            self.ad.restartCP = false
            self.ad.stateModule:setStartHelper(false)
            local parkDestinationAtJobFinished = self.ad.stateModule:getParkDestinationAtJobFinished()

            if parkDestinationAtJobFinished >= 1 then
                local trailers, _ = AutoDrive.getAllUnits(self)
                local fillLevel, _, _ = AutoDrive.getAllFillLevels(trailers)
                if self.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER and fillLevel > 0 then
                    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:onCpFinished fillLevel > 0 %s", tostring(fillLevel))
                    -- unload before going to park
                    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:onCpFinished unload before going to park - fillLevel %s", tostring(fillLevel))
                    self.ad.stateModule:setMode(AutoDrive.MODE_DELIVERTO)
                    self.ad.stateModule:setFirstMarker(self.ad.stateModule:getSecondMarkerId())
                else
                    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:onCpFinished drive to park position")
                    self.ad.onRouteToPark = true
                    self.ad.stateModule:setMode(AutoDrive.MODE_DRIVETO)
                    self.ad.stateModule:setFirstMarker(parkDestinationAtJobFinished)
                end
                self.ad.stateModule:getCurrentMode():start()
            else
                AutoDriveMessageEvent.sendMessageOrNotification(self, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_parkVehicle_noPosSet;", 5000, self.ad.stateModule:getName())
                -- stop vehicle movement
                self.ad.trailerModule:handleTrailerReversing(false)
                -- AutoDrive.driveInDirection(self, 16, 30, 0, 0.2, 20, false, false, 0, 0, 0, 1) -- not to call here #222!
                self:setCruiseControlState(Drivable.CRUISECONTROL_STATE_OFF)
                if self.stopMotor ~= nil then
                    self:stopMotor()
                end
            end
        end
    else
        Logging.devError("AutoDrive:onCpFinished() must be called only on the server.")
    end
    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:onCpFinished end")
end

function AutoDrive:handleCPFieldWorker(vehicle)
    AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:handleCPFieldWorker start...")
    if vehicle.isServer then
        if vehicle.ad and vehicle.ad.stateModule and vehicle.startAutoDrive then
            -- restart CP
            vehicle.ad.restartCP = false
            if not vehicle.ad.stateModule:isActive() then
                if vehicle.ad.stateModule:getStartHelper() and vehicle.ad.stateModule:getUsedHelper() == ADStateModule.HELPER_CP then
                    -- CP button active
                    if table.contains(AutoDrive.modesToStartFromCP, vehicle.ad.stateModule:getMode()) then
                        -- mode allowed to activate
                        vehicle.ad.restartCP = true
                        AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:handleCPFieldWorker start AD")
                        vehicle.ad.stateModule:getCurrentMode():start()
                    else
                        -- deactivate CP button
                        AutoDriveMessageEvent.sendMessageOrNotification(vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s: $l10n_AD_Wrong_Mode_takeover_from_CP;", 5000, vehicle.ad.stateModule:getName())
                        vehicle.ad.restartCP = false -- do not continue CP course
                        vehicle.ad.stateModule:setStartHelper(false)
                    end
                end
            else
                AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:handleCPFieldWorker - AD already active, should not happen")
            end
        end
    else
        Logging.devError("AutoDrive:handleCPFieldWorker() must be called only on the server.")
    end
    AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:handleCPFieldWorker end")
end

function AutoDrive:onCpEmpty()
    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:onCpEmpty start...")
    local rootVehicle = self.getRootVehicle and self:getRootVehicle()
    if rootVehicle then
        if rootVehicle.ad == nil then
            rootVehicle.ad = {}
        end
        rootVehicle.ad.isCpEmpty = true
    end
    AutoDrive:handleCPFieldWorker(self)
end

function AutoDrive:onCpFull()
    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:onCpFull start...")
    local rootVehicle = self.getRootVehicle and self:getRootVehicle()
    if rootVehicle then
        if rootVehicle.ad == nil then
            rootVehicle.ad = {}
        end
        rootVehicle.ad.isCpFull = true
    end
    AutoDrive:handleCPFieldWorker(self)
end

function AutoDrive:onCpFuelEmpty() -- refuel
    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:onCpFuelEmpty start... autoRefuel %s", AutoDrive.getSetting("autoRefuel", self))
    if self.isServer then
        self.ad.restartCP = false
        if self.ad and self.ad.stateModule and self.startAutoDrive and AutoDrive.getSetting("autoRefuel", self) then

            local refuelDestination = ADTriggerManager.getClosestRefuelDestination(self, true)
            AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:onCpFuelEmpty refuelDestination %s", tostring(refuelDestination))

            if refuelDestination ~= nil and refuelDestination >= 1 then
                if not self.ad.stateModule:isActive() then
                    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:onCpFuelEmpty getCurrentMode() start %s", tostring(self.ad.stateModule:getCurrentMode()))
                    self.ad.onRouteToRefuel = true
                    self.ad.restartCP = true
                    self.ad.stateModule:getCurrentMode():start()
                else
                    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:onCpFuelEmpty - AD already active, should not happen")
                end
            else
                local refuelFillTypes = AutoDrive.getRequiredRefuels(self, true)
                AutoDriveMessageEvent.sendMessageOrNotification(self, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_No_Refuel_Station;", 5000, self.ad.stateModule:getName())
            end
        end
    else
        Logging.devError("AutoDrive:onCpFuelEmpty() must be called only on the server.")
    end
    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:onCpFuelEmpty end")
end

function AutoDrive:onCpBroken() -- repair
    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:onCpBroken start... autoRepair %s", tostring(AutoDrive.getSetting("autoRepair", self)))

    if self.isServer then
        self.ad.restartCP = false
        if self.ad and self.ad.stateModule and self.startAutoDrive and AutoDrive.getSetting("autoRepair", self) then
            local repairDestinationMarkerNodeID = AutoDrive:getClosestRepairTrigger(self)
            AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO,"AutoDrive:onCpBroken repairDestinationMarkerNodeID %s", tostring(repairDestinationMarkerNodeID))
            if repairDestinationMarkerNodeID ~= nil then
                if not self.ad.stateModule:isActive() then
                    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:onCpBroken getCurrentMode() start %s", tostring(self.ad.stateModule:getCurrentMode()))
                    self.ad.onRouteToRepair = true
                    self.ad.restartCP = true
                    self.ad.stateModule:getCurrentMode():start()
                else
                    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:onCpBroken - AD already active, should not happen")
                end
            else
                AutoDriveMessageEvent.sendMessageOrNotification(self, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_No_Repair_Station;", 5000, self.ad.stateModule:getName())
            end
        end
    else
        Logging.devError("AutoDrive:onCpBroken() must be called only on the server.")
    end
    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:onCpBroken end")
end

--- Disables click to switch, if the user clicks on the hud or the editor mode is active.
function AutoDrive:enterVehicleRaycastClickToSwitch(superFunc, x, y)

    if AutoDrive.isEditorModeEnabled() then 
        return
    end

    --- Checks if the mouse is over a hud element.
    if AutoDriveHud:isMouseOverHud(x, y) then 
        return
    end

    superFunc(self, x, y)
end

-- currently used by CP
function AutoDrive:getCanAdTakeControl()
    if self.isServer then
        if self.ad and self.ad.stateModule and not self.ad.stateModule:isActive() then
            return self.ad.stateModule:getStartHelper() and self.ad.stateModule:getUsedHelper() == ADStateModule.HELPER_CP
        end
    end
    return false
end

-- Autoloader
--[[
APalletAutoLoader:
]]
function AutoDrive:hasAL(object)
    if object == nil then
        return false
    end
    local ret = false
    ret = ret or (object.spec_aPalletAutoLoader ~= nil and object.spec_aPalletAutoLoader.loadArea ~= nil and object.spec_aPalletAutoLoader.loadArea["baseNode"] ~= nil)
    if object.spec_universalAutoload ~= nil then
        local rootVehicle = object.getRootVehicle and object:getRootVehicle()
        if rootVehicle and not rootVehicle.spec_locomotive then
            ret = ret or (object.spec_universalAutoload.isAutoloadAvailable and not object.spec_universalAutoload.autoloadDisabled)
        end
    end
    return ret
end

--[[
    STOPPED = 1,
    RUNNING = 2
]]

function AutoDrive:setALOn(object)
    if not AutoDrive:hasAL(object) then
        return false
    end
    local spec = object.spec_aPalletAutoLoader
    if spec and object.SetLoadingState then
        -- set loading state On
        AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:setALOn SetLoadingState 2")
        object:SetLoadingState(2)
    end
    spec = object.spec_universalAutoload
    if spec and object.ualStartLoad then
        local rootVehicle = object.getRootVehicle and object:getRootVehicle()
        if rootVehicle and rootVehicle.lastSpeedReal < 0.0005 then
            -- loading not possible during movement
            -- set loading state On
            AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:setALOn ualStartLoad")
            if object.ualSetMaterialTypeIndex and object.ualStartLoad then
                local fillTypeID = rootVehicle.ad.stateModule:getFillType()
                object:ualSetMaterialTypeIndex(fillTypeID)
                -- need to set the fillType before ualStartLoad as it resets always to ALL
                object:ualStartLoad()
            end
        end
    end
end

function AutoDrive:setALOff(object)
    if not AutoDrive:hasAL(object) then
        return false
    end

    local spec = object.spec_aPalletAutoLoader
    if spec and object.SetLoadingState then
        -- set loading state off
        AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:setALOff SetLoadingState 1")
        object:SetLoadingState(1)
    end
    local spec = object.spec_universalAutoload
    if spec and object.ualStopLoad then
        -- set loading state off
        AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:setALOff ualStopLoad")
        if object.ualStopLoad then
            object:ualStopLoad()
        end
    end
end

function AutoDrive.activateALTrailers(vehicle, trailers)
    if vehicle == nil or trailers == nil then
        return false
    end
    AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:activateALTrailers")
    if #trailers > 0 then
        for _, trailer in ipairs(trailers) do
            AutoDrive:setALOn(trailer)
        end
    end
end

function AutoDrive.deactivateALTrailers(vehicle, trailers)
    if vehicle == nil or trailers == nil then
        return false
    end
    AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:deactivateTrailerAL")
    if #trailers > 0 then
        for _, trailer in ipairs(trailers) do
            AutoDrive:setALOff(trailer)
        end
    end
end

--[[
APalletAutoLoaderTipsides = {
    LEFT = 1,
    RIGHT = 2,
    MIDDLE = 3,
    BACK = 4
}
    values = {0, 1, 2, 3, 4},
    texts = {"gui_ad_AL_off", "gui_ad_AL_center", "gui_ad_AL_left", "gui_ad_AL_behind", "gui_ad_AL_right"},

]]
-- TODO: unload in rear
function AutoDrive:unloadAL(object)
    if object == nil or not AutoDrive:hasAL(object) then
        return false
    end
    AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:unloadAL start")
    local rootVehicle = object:getRootVehicle()
    -- spec_aPalletAutoLoader
    local spec = object.spec_aPalletAutoLoader
    if spec and object.SetTipside and object.unloadAll then
        local unloadPositions = {
            3,
            1,
            4,
            2
        }

        local unloadPositionSetting = AutoDrive.getSetting("ALUnload", rootVehicle)
        AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:unloadAL spec_aPalletAutoLoader unloadPositionSetting %s", tostring(unloadPositionSetting))
        if unloadPositionSetting ~= nil and unloadPositionSetting > 0 then
            local unloadPosition = unloadPositions[unloadPositionSetting]
            AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:unloadAL spec_aPalletAutoLoader unloadPosition %s", tostring(unloadPosition))
            if unloadPosition ~= nil then
                -- should unload
                AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:unloadAL spec_aPalletAutoLoader should unload")
                if object.setAllTensionBeltsActive ~= nil then
                    object:setAllTensionBeltsActive(false, false)
                end
                if object.SetTipside and object.unloadAll then
                    AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:unloadAL spec_aPalletAutoLoader SetTipside unloadPosition %s", tostring(unloadPosition))
                    if unloadPosition > 0 then
                        object:SetTipside(unloadPosition)
                    end
                    -- set loading state off
                    object:unloadAll()
                end
            end
        end
    end
    -- spec_universalAutoload
    local spec = object.spec_universalAutoload
    if spec and object.ualUnload then
        local unloadPositions = {
            "center",
            "left",
            "behind",
            "right"
        }
        local unloadPositionSetting = AutoDrive.getSetting("ALUnload", rootVehicle)
        AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:unloadAL spec_universalAutoload unloadPositionSetting %s", tostring(unloadPositionSetting))
        if unloadPositionSetting ~= nil and unloadPositionSetting > 0 then
            local unloadPosition = unloadPositions[unloadPositionSetting]
            AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:unloadAL spec_universalAutoload unloadPosition %s", tostring(unloadPosition))
            if unloadPosition ~= nil and string.len(unloadPosition) > 0 then
                if unloadPositionSetting == 1 then
                    -- center unload - unfasten belts only
                    if object.setAllTensionBeltsActive ~= nil then
                        AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:unloadAL spec_universalAutoload setAllTensionBeltsActive")
                        object:setAllTensionBeltsActive(false, false)
                    end
                else
                    AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:unloadAL spec_universalAutoload ualUnload")
                    if object.ualSetUnloadPosition and object.ualUnload then
                        object:ualSetUnloadPosition(unloadPosition)
                        object:ualUnload()
                    end
                end
            end
        end
    end
    AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:unloadAL end")
end

function AutoDrive:unloadALAll(vehicle) -- used by UnloadAtDestinationTask
    if vehicle == nil then
        return false
    end
    local trailers, trailerCount = AutoDrive.getAllUnits(vehicle)
    AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:unloadALAll trailerCount %s", tostring(trailerCount))
    if trailers and trailerCount > 0 then
        for i=1, trailerCount do
            AutoDrive:unloadAL(trailers[i])
        end
    end
end

function AutoDrive:getALObjectFillLevels(object) -- used by getIsFillUnitEmpty, getIsFillUnitFull, getObjectFillLevels, getAllFillLevels
    if object == nil then
        Logging.error("[AD] AutoDrive.getALObjectFillLevels object == nil")
        return 0, 0, false, 0
    end
    local rootVehicle = object:getRootVehicle()
    if rootVehicle == nil then
        Logging.error("[AD] AutoDrive.getALObjectFillLevels rootVehicle == nil")
        return 0, 0, false, 0
    end
    AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:getALObjectFillLevels object.spec_aPalletAutoLoader %s object.spec_universalAutoload %s", tostring(object.spec_aPalletAutoLoader), tostring(object.spec_universalAutoload))
    local fillCapacity = 0
    local fillLevel = 0
    local fillFreeCapacity = 0
    if AutoDrive:hasAL(object) then
        if object.spec_aPalletAutoLoader and object.getFillUnitCapacity and object.getFillUnitFillLevel and object.getFillUnitFreeCapacity then
            fillCapacity = object:getFillUnitCapacity()
            fillLevel = object:getFillUnitFillLevel()
            fillFreeCapacity = object:getFillUnitFreeCapacity()
        elseif object.spec_universalAutoload and object.ualGetFillUnitCapacity and object.ualGetFillUnitFillLevel and object.ualGetFillUnitFreeCapacity then
            fillCapacity = object:ualGetFillUnitCapacity()
            fillLevel = object:ualGetFillUnitFillLevel()
            fillFreeCapacity = object:ualGetFillUnitFreeCapacity()
        end
        AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:getALObjectFillLevels fillCapacity %s fillLevel %s fillFreeCapacity %s", tostring(fillCapacity), tostring(fillLevel), tostring(fillFreeCapacity))
    end
    local filledToUnload = AutoDrive.isUnloadFillLevelReached(rootVehicle, fillLevel, fillFreeCapacity, fillCapacity)
    AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:getALObjectFillLevels filledToUnload %s ", tostring(filledToUnload))
    return fillLevel, fillCapacity, filledToUnload, fillFreeCapacity
end

function AutoDrive:getALFillTypes(object) -- used by PullDownList, getSupportedFillTypesOfAllUnitsAlphabetically
    if object == nil then
        return {}
    end
    local fillTypes = {}

    -- spec_aPalletAutoLoader
    local spec = object.spec_aPalletAutoLoader
    if spec and AutoDrive:hasAL(object) and object.GetAutoloadTypes then
        local autoLoadTypes = object:GetAutoloadTypes()
        if autoLoadTypes and table.count(autoLoadTypes) > 0 then
            for i = 1, table.count(autoLoadTypes) do
                if autoLoadTypes[i].nameTranslated then
                    table.insert(fillTypes, autoLoadTypes[i].nameTranslated)
                end
            end
        end
    end
    -- spec_universalAutoload
    local spec = object.spec_universalAutoload
    if spec and AutoDrive:hasAL(object) then
        AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:getALFillTypes spec_universalAutoload retrieve fillTypes...")
        for index, fillType in ipairs(g_fillTypeManager:getFillTypes()) do
            if fillType.isBaleType or fillType.isPalletType or (fillType.palletFilename ~= nil) then
                table.insert(fillTypes, {fillTypeID = index, fillTypeName = fillType.title})
            end
        end
    end
    AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:getALFillTypes #fillTypes %s", tostring(#fillTypes))
    return fillTypes
end

function AutoDrive:getALCurrentFillType(object) -- used by onEnterVehicle, onPostAttachImplement
    if object == nil then
        return nil
    end
    -- spec_aPalletAutoLoader
    local spec = object.spec_aPalletAutoLoader
    if spec and AutoDrive:hasAL(object) and spec.currentautoLoadTypeIndex then
        return spec.currentautoLoadTypeIndex
    end
    -- spec_universalAutoload
    local spec = object.spec_universalAutoload
    if spec and AutoDrive:hasAL(object) then
        AutoDrive.debugPrint(object, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:getALCurrentFillType spec_universalAutoload function not supported!")
        local rootVehicle = object.getRootVehicle and object:getRootVehicle()
        if rootVehicle then
            return rootVehicle.ad.stateModule:getFillType()
        else
            return AutoDrive.UAL_FILLTYPE_ALL -- assume all fillTypes in UAL
        end
    end
    return nil
end

function AutoDrive:setALFillType(vehicle, fillTypeID) -- used by PullDownList
    if vehicle == nil or fillTypeID == nil then
        return false
    end
    AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:setALFillType")
    local trailers, trailerCount = AutoDrive.getAllUnits(vehicle)
    if trailers and trailerCount > 0 then
        local rootVehicle = trailers[1].getRootVehicle and trailers[1]:getRootVehicle()
        if rootVehicle then
            rootVehicle.ad.ALCurrentFillTypeID = fillTypeID
        end

        for _, trailer in ipairs(trailers) do
            -- spec_aPalletAutoLoader
            local spec = trailer.spec_aPalletAutoLoader
            if spec and AutoDrive:hasAL(trailer) and trailer.SetLoadingState and trailer.SetAutoloadType then
                AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:setALFillType spec_aPalletAutoLoader fillTypeID %s", tostring(fillTypeID))
                -- set loading state off
                trailer:SetLoadingState(1)
                trailer:SetAutoloadType(fillTypeID)
            end
            -- spec_universalAutoload
            local spec = trailer.spec_universalAutoload
            if spec and AutoDrive:hasAL(trailer) then
                AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:setALFillType spec_universalAutoload function not supported!")
                if trailer.ualSetMaterialTypeIndex then
                    trailer:ualSetMaterialTypeIndex(fillTypeID)
                end
            end
        end
    end
end

function AutoDrive.objectsToLoadAvailable(vehicle, trailers)
    if vehicle == nil or trailers == nil then
        return false
    end
    local objectsToLoadAvailable = false
    AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:objectsToLoadAvailable")
    if #trailers > 0 then
        for _, trailer in ipairs(trailers) do
            -- spec_universalAutoload
            local spec = trailer.spec_universalAutoload
            if spec and AutoDrive:hasAL(trailer) then
                if trailer.ualGetTotalAvailableCount then
                    objectsToLoadAvailable = objectsToLoadAvailable or trailer:ualGetTotalAvailableCount() > 0
                end
            end
        end
    end
    return objectsToLoadAvailable
end

function AutoDrive:handleAIFinished(vehicle)
    AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:handleAIFinished start... enableParkAtJobFinished %s", AutoDrive.getSetting("enableParkAtJobFinished", vehicle))
    if vehicle.isServer then
        if vehicle.ad and vehicle.ad.stateModule and vehicle.startAutoDrive and AutoDrive.getSetting("enableParkAtJobFinished", vehicle) then
            vehicle.ad.onRouteToRefuel = false
            vehicle.ad.onRouteToRepair = false
            vehicle.ad.stateModule:setStartHelper(false) -- do not continue AI job
            local parkDestinationAtJobFinished = vehicle.ad.stateModule:getParkDestinationAtJobFinished()

            if parkDestinationAtJobFinished >= 1 then
                AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:handleAIFinished drive to park position")
                vehicle.ad.onRouteToPark = true
                vehicle.ad.stateModule:setMode(AutoDrive.MODE_DRIVETO)
                vehicle.ad.stateModule:setFirstMarker(parkDestinationAtJobFinished)
                vehicle.ad.stateModule:getCurrentMode():start()
            else
                AutoDriveMessageEvent.sendMessageOrNotification(vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_parkVehicle_noPosSet;", 5000, vehicle.ad.stateModule:getName())
                -- stop vehicle movement
                vehicle.ad.trailerModule:handleTrailerReversing(false)
                AutoDrive.driveInDirection(vehicle, 16, 30, 0, 0.2, 20, false, false, 0, 0, 0, 1)
                vehicle:setCruiseControlState(Drivable.CRUISECONTROL_STATE_OFF)
                if vehicle.stopMotor ~= nil then
                    vehicle:stopMotor()
                end
            end
        end
    else
        Logging.devError("AutoDrive:handleAIFinished() must be called only on the server.")
    end
    AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:handleAIFinished end")
end

function AutoDrive:handleAIFieldWorker(vehicle)
    AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:handleAIFieldWorker start...")
    if vehicle.isServer then
        if vehicle.ad and vehicle.ad.stateModule and vehicle.startAutoDrive then
            -- restart AI
            if not vehicle.ad.stateModule:isActive() then
                if vehicle.ad.stateModule:getStartHelper() then
                    -- AI button active
                    if table.contains(AutoDrive.modesToStartFromCP, vehicle.ad.stateModule:getMode()) then
                        -- mode allowed to activate
                        AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:handleAIFieldWorker start AD")
                        vehicle.ad.stateModule:getCurrentMode():start()
                    else
                        -- deactivate AI button
                        AutoDriveMessageEvent.sendMessageOrNotification(vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s: $l10n_AD_Wrong_Mode_takeover_from_CP;", 5000, vehicle.ad.stateModule:getName())
                        vehicle.ad.stateModule:setStartHelper(false) -- do not continue AI job
                    end
                end
            else
                AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:handleAIFieldWorker - AD already active, should not happen")
            end
        end
    else
        Logging.devError("AutoDrive:handleAIFieldWorker() must be called only on the server.")
    end
    AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:handleAIFieldWorker end")
end

function AutoDrive:onAIJobFinished()
    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:onAIJobFinished start...")
    local rootVehicle = self.getRootVehicle and self:getRootVehicle()
    if rootVehicle then
        if AutoDrive:getIsCPActive(rootVehicle) then
            -- ignore this event for CP helpers to react on the separate CP events
            AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive:onAIJobFinished ignore this event for CP helpers")
            return
        end
        if rootVehicle.ad == nil then
            rootVehicle.ad = {}
        end
        rootVehicle.ad.isAIJobFinished = true
    end

    local trailers, _ = AutoDrive.getAllUnits(self)
    local fillLevel, _, _ = AutoDrive.getAllFillLevels(trailers)
    if (self.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER and fillLevel > 0) -- something to unload
        or (self.ad.stateModule:getMode() == AutoDrive.MODE_UNLOAD and fillLevel > 0) -- something to unload
        or (self.ad.stateModule:getMode() == AutoDrive.MODE_LOAD and fillLevel < 0.01) -- something to load
    then
        AutoDrive:handleAIFieldWorker(self)
    else
        AutoDrive:handleAIFinished(self) -- stop or park
    end
end
