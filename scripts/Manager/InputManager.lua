ADInputManager = {}

-- same order as in modDesc.xml
-- {ActionName, InputName, allowedInHelperScreen, executedOnServer, ActionEventTextVisibility, ActionEventTextPriority}
ADInputManager.actionsToInputs = {
    {"ADToggleMouse", "input_toggleMouse", true, false, true, 1},
    {"ADSilomode", "input_silomode", true, true},
    {"ADPreviousMode", "input_previousMode", true, true},
    {"ADRecord", "input_record", false, true},
    {"ADRecord_Dual", "input_record_dual", false, true},
    {"ADRecord_SubPrio", "input_record_subPrio", false, true},
    {"ADRecord_SubPrioDual", "input_record_subPrioDual", false, true},
    {"ADRecord_TwoWay", "input_record_twoWay", false, true},
    {"ADRecord_Dual_TwoWay", "input_record_dualTwoWay", false, true},
    {"ADRecord_SubPrio_TwoWay", "input_record_subPrioTwoWay", false, true},
    {"ADRecord_SubPrioDual_TwoWay", "input_record_subPrioDualTwoWay", false, true},
    {"ADEnDisable", "input_start_stop", true, true, true, 1},
    {"ADSelectTarget", "input_nextTarget", true, true},
    {"ADSelectPreviousTarget", "input_previousTarget", true, true},
    {"ADSelectTargetUnload", "input_nextTarget_Unload", true, true},
    {"ADSelectPreviousTargetUnload", "input_previousTarget_Unload", true, true},
    {"ADActivateDebug", "input_debug", false, false},
    {"ADDisplayMapPoints", "input_displayMapPoints", false, false},
    {"ADDebugSelectNeighbor", "input_showNeighbor", false, false},
    {"ADDebugCreateConnection", "input_toggleConnection", false, false},
    {"ADDebugToggleConnectionInverted", "input_toggleConnectionInverted", false, false},
    {"ADDebugChangeNeighbor", "input_nextNeighbor", false, false},
    {"ADDebugPreviousNeighbor", "input_previousNeighbor", false, false},
    {"ADDebugCreateMapMarker", "input_createMapMarker", false, false},
    {"ADRenameMapMarker", "input_editMapMarker", false, false},
    {"ADDebugDeleteDestination", "input_removeMapMarker", false, false},
    {"ADNameDriver", "input_nameDriver", true, false},
    {"ADSetDestinationFilter", "input_setDestinationFilter", true, false},
    {"AD_Speed_up", "input_increaseSpeed", true, true},
    {"AD_Speed_down", "input_decreaseSpeed", true, true},
    {"AD_FieldSpeed_up", "input_increaseFieldSpeed", true, true},
    {"AD_FieldSpeed_down", "input_decreaseFieldSpeed", true, true},
    {"ADToggleHud", "input_toggleHud", false, false, true, 1},
    -- {"COURSEPLAY_MOUSEACTION_SECONDARY", "input_toggleMouse", false, false},
    {"ADDebugDeleteWayPoint", "input_removeWaypoint", false, false},
    {"AD_routes_manager", "input_routesManager", false, false},
    {"ADSelectNextFillType", "input_nextFillType", true, true},
    {"ADSelectPreviousFillType", "input_previousFillType", true, true},
    {"ADOpenGUI", "input_openGUI", true, false, true, 2},
    {"ADCallDriver", "input_callDriver", false, true},
    {"ADGoToVehicle", "input_goToVehicle", false, false},
    {"ADIncLoopCounter", "input_incLoopCounter", true, true},
    {"ADDecLoopCounter", "input_decLoopCounter", true, true},
    {"ADSwapTargets", "input_swapTargets", true, true},
    {"ADStartHelper", "input_startHelper", true, true},
    {"ADToggleUsedHelper", "input_toggleUsedHelper", true, true},
    {"AD_open_notification_history", "input_openNotificationHistory", false, false},
    -- {"AD_open_colorSettings", "input_openColorSettings", false},
    {"AD_continue", "input_continue", true, true},
    {"ADParkVehicle", "input_parkVehicle", true, true},
    {"ADSetParkDestination", "input_setParkDestination", false, false},
    {"AD_devAction", "input_devAction", false, false},
    {"ADRefuelVehicle", "input_refuelVehicle", false, true},
    {"ADToggleHudExtension", "input_toggleHudExtension", true, false, true, 1},
    {"ADToggleAutomaticUnloadTarget", "input_toggleAutomaticUnloadTarget", true, true},
    {"ADToggleAutomaticPickupTarget", "input_toggleAutomaticPickupTarget", true, true},
    {"ADToggleLoadByFillLevel", "input_toggleLoadByFillLevel", true, true},
    {"ADRepairVehicle", "input_repairVehicle", false, true}
}

--[[
tool selection not proper on dedi servers as known!
That's why the following event is only taken on clients and send as event in the network
input_setParkDestination
]]
-- inputs to send to server
ADInputManager.inputsToIds = {}
ADInputManager.idsToInputs = {}

function ADInputManager:load()
    for k, v in pairs(self.actionsToInputs) do
        if v[4] then
            self.inputsToIds[self.actionsToInputs[k][2]] = k
        end
    end
    for k, v in pairs(self.inputsToIds) do
        self.idsToInputs[v] = k
    end
end

function ADInputManager.onActionCall(vehicle, actionName)
    local controlledVehicle = AutoDrive.getControlledVehicle()

    for k, v in pairs(ADInputManager.actionsToInputs) do
        local action = ADInputManager.actionsToInputs[k][1]
        local allowed = ADInputManager.actionsToInputs[k][3] or (controlledVehicle ~= nil and controlledVehicle == vehicle)

        if action == actionName and allowed then
            local input = ADInputManager.actionsToInputs[k][2]
            if type(input) ~= "string" or input == "" then
                Logging.devError("[AutoDrive] Action '%s' = '%s'", actionName, input)
                return
            end
            ADInputManager:onInputCall(vehicle, input)
            break
        end
    end
end

function ADInputManager:onInputCall(vehicle, input, farmId, sendEvent)
    local actualFarmId = farmId or 0
    if actualFarmId == 0 then
        actualFarmId = AutoDrive:getAIFrameFarmId() or 0
    end
    if actualFarmId == 0 and vehicle.ad and vehicle.ad.stateModule then
        actualFarmId = vehicle.ad.stateModule:getActualFarmId()
    end

    local controlledVehicle = AutoDrive.getControlledVehicle()

    for k, v in pairs(ADInputManager.actionsToInputs) do
        local allowed = ADInputManager.actionsToInputs[k][3] or ((controlledVehicle ~= nil and controlledVehicle == vehicle) or v[4])
        if input == ADInputManager.actionsToInputs[k][2] and allowed then
            local func = self[input]
            if type(func) ~= "function" then
                Logging.devError("[AutoDrive] Input '%s' = '%s'", input, type(func))
                return
            end

            if sendEvent == nil or sendEvent == true then
                local inputId = self.inputsToIds[input]
                if inputId ~= nil then
                    AutoDriveInputEventEvent.sendEvent(vehicle, inputId, actualFarmId)
                    return
                end
            end

            func(ADInputManager, vehicle, actualFarmId)
            break
        end
    end


end

-- Sender only events

function ADInputManager:input_openNotificationHistory(vehicle)
    AutoDrive.onOpenNotificationsHistory()
end

function ADInputManager:input_openColorSettings()
    AutoDrive.onOpenColorSettings()
end

function ADInputManager:input_editMapMarker(vehicle)
    if AutoDrive.isEditorModeEnabled() then
        -- This can be triggered both from 'Edit Target' keyboard shortcut and right click on 'Create Target' hud button
        if ADGraphManager:getWayPointById(1) == nil or vehicle.ad.stateModule:getFirstMarker() == nil then
            return
        end
        AutoDrive.editSelectedMapMarker = true
        AutoDrive.onOpenEnterTargetName()
    end
end

function ADInputManager:input_removeWaypoint(vehicle)
    if AutoDrive.isEditorModeEnabled() then
        local closestWayPoint, _ = vehicle:getClosestWayPoint()
        if ADGraphManager:getWayPointById(closestWayPoint) ~= nil then
            -- This can be triggered both from 'Remove Waypoint' keyboard shortcut and left click on 'Remove Waypoint' hud button
            ADGraphManager:removeWayPoint(closestWayPoint)
        end
    end
end

function ADInputManager:input_removeMapMarker(vehicle)
    if AutoDrive.isEditorModeEnabled() then
        local closestWayPoint, _ = vehicle:getClosestWayPoint()
        if ADGraphManager:getWayPointById(closestWayPoint) ~= nil then
            -- This can be triggered both from 'Remove Target' keyboard shortcut and right click on 'Remove Waypoint' hud button
            ADGraphManager:removeMapMarkerByWayPoint(closestWayPoint)
        end
    end
end

function ADInputManager:input_createMapMarker(vehicle)
    if AutoDrive.isEditorModeEnabled() then
        local closestWayPoint, _ = vehicle:getClosestWayPoint()
        if ADGraphManager:getWayPointById(closestWayPoint) ~= nil then
            -- This can be triggered both from 'Create Target' keyboard shortcut and left click on 'Create Target' hud button
            AutoDrive.editSelectedMapMarker = false
            AutoDrive.onOpenEnterTargetName()
        end
    end
end

function ADInputManager:input_toggleConnection(vehicle)
    if AutoDrive.isEditorModeEnabled() then
        local closestWayPoint, _ = vehicle:getClosestWayPoint()
        if ADGraphManager:getWayPointById(closestWayPoint) ~= nil then
            if vehicle.ad.stateModule:getSelectedNeighbourPoint() ~= nil then
                ADGraphManager:toggleConnectionBetween(ADGraphManager:getWayPointById(closestWayPoint), vehicle.ad.stateModule:getSelectedNeighbourPoint(), false, false)
            end
        end
    end
end

function ADInputManager:input_toggleConnectionInverted(vehicle)
    if AutoDrive.isEditorModeEnabled() then
        local closestWayPoint, _ = vehicle:getClosestWayPoint()
        if ADGraphManager:getWayPointById(closestWayPoint) ~= nil then
            if vehicle.ad.stateModule:getSelectedNeighbourPoint() ~= nil then
                ADGraphManager:toggleConnectionBetween(vehicle.ad.stateModule:getSelectedNeighbourPoint(), ADGraphManager:getWayPointById(closestWayPoint), false, false)
            end
        end
    end
end

function ADInputManager:input_nameDriver()
    AutoDrive.onOpenEnterDriverName()
end

function ADInputManager:input_setDestinationFilter()
    AutoDrive.onOpenEnterDestinationFilter()
end

function ADInputManager:input_openGUI()
    AutoDrive.onOpenSettings()
end

function ADInputManager:input_toggleHud(vehicle)
    AutoDrive.Hud:toggleHud(vehicle)
end

function ADInputManager:input_toggleHudExtension(vehicle)
    AutoDrive.Hud:toggleHudExtension(vehicle)
end

function ADInputManager:input_toggleMouse()
    g_inputBinding:setShowMouseCursor(not g_inputBinding:getShowMouseCursor())
end

function ADInputManager:input_routesManager()
    if (AutoDrive.getSetting("enableRoutesManagerOnDediServer") and g_dedicatedServer ~= nil) or g_dedicatedServer == nil then
        AutoDrive.onOpenRoutesManager()
    end
end

function ADInputManager:input_goToVehicle()
    ADMessagesManager:goToVehicle()
end

function ADInputManager:input_showNeighbor(vehicle)
    vehicle.ad.stateModule:togglePointToNeighbor()
end

function ADInputManager:input_nextNeighbor(vehicle)
    vehicle.ad.stateModule:changeNeighborPoint(1)
end

function ADInputManager:input_previousNeighbor(vehicle)
    vehicle.ad.stateModule:changeNeighborPoint(-1)
end

-- Sender and server events

function ADInputManager:input_start_stop(vehicle, farmId)
    if ADGraphManager:getWayPointById(1) == nil or vehicle.ad.stateModule:getFirstMarker() == nil then
        return
    end
    if vehicle.ad.stateModule:isActive() then
        vehicle.ad.isStoppingWithError = true
        vehicle.ad.stateModule:setLoopsDone(0)
        vehicle:stopAutoDrive()
    else
        if farmId and farmId > 0 then
            -- set the farmId for the vehicle controlled by AD
            vehicle.ad.stateModule:setActualFarmId(farmId) -- input_start_stop
        end
        vehicle.ad.stateModule:setLoopsDone(0)
        vehicle.ad.stateModule:getCurrentMode():start(AutoDrive.USER_PLAYER)
    end
    if vehicle.isServer then
        SpecializationUtil.raiseEvent(vehicle, "onUpdate", 16, false, false, false)
    end
end

function ADInputManager:input_incLoopCounter(vehicle)
    vehicle.ad.stateModule:changeLoopCounter(true)
end

function ADInputManager:input_decLoopCounter(vehicle)
    vehicle.ad.stateModule:changeLoopCounter(false)
end

function ADInputManager:input_setParkDestination(vehicle)
    AutoDrive.setActualParkDestination(vehicle)
end

function ADInputManager:input_silomode(vehicle)
    vehicle.ad.stateModule:nextMode()
end

function ADInputManager:input_previousMode(vehicle)
    vehicle.ad.stateModule:previousMode()
end

function ADInputManager:input_record(vehicle)
    if not vehicle.ad.stateModule:isInCreationMode() then
        if not AutoDrive.isInExtendedEditorMode() then
            AutoDrive.setEditorMode(AutoDrive.EDITOR_EXTENDED)
        end
        vehicle.ad.stateModule:startNormalCreationMode()
    else
        vehicle.ad.stateModule:disableCreationMode()
    end
end

function ADInputManager:input_record_dual(vehicle)
    if not vehicle.ad.stateModule:isInCreationMode() then
        if not AutoDrive.isInExtendedEditorMode() then
            AutoDrive.setEditorMode(AutoDrive.EDITOR_EXTENDED)
        end
        vehicle.ad.stateModule:startDualCreationMode()
    else
        vehicle.ad.stateModule:disableCreationMode()
    end
end

function ADInputManager:input_record_subPrio(vehicle)
    if not vehicle.ad.stateModule:isInCreationMode() then
        if not AutoDrive.isInExtendedEditorMode() then
            AutoDrive.setEditorMode(AutoDrive.EDITOR_EXTENDED)
        end
        vehicle.ad.stateModule:startSubPrioCreationMode()
    else
        vehicle.ad.stateModule:disableCreationMode()
    end
end

function ADInputManager:input_record_subPrioDual(vehicle)
    if not vehicle.ad.stateModule:isInCreationMode() then
        if not AutoDrive.isInExtendedEditorMode() then
            AutoDrive.setEditorMode(AutoDrive.EDITOR_EXTENDED)
        end
        vehicle.ad.stateModule:startSubPrioDualCreationMode()
    else
        vehicle.ad.stateModule:disableCreationMode()
    end
end

function ADInputManager:input_record_twoWay(vehicle)
    if not vehicle.ad.stateModule:isInCreationMode() then
        if not AutoDrive.isInExtendedEditorMode() then
            AutoDrive.setEditorMode(AutoDrive.EDITOR_EXTENDED)
        end
        vehicle.ad.stateModule:startNormalTwoWayCreationMode()
    else
        vehicle.ad.stateModule:disableCreationMode()
    end
end

function ADInputManager:input_record_dualTwoWay(vehicle)
    if not vehicle.ad.stateModule:isInCreationMode() then
        if not AutoDrive.isInExtendedEditorMode() then
            AutoDrive.setEditorMode(AutoDrive.EDITOR_EXTENDED)
        end
        vehicle.ad.stateModule:startDualTwoWayCreationMode()
    else
        vehicle.ad.stateModule:disableCreationMode()
    end
end

function ADInputManager:input_record_subPrioTwoWay(vehicle)
    if not vehicle.ad.stateModule:isInCreationMode() then
        if not AutoDrive.isInExtendedEditorMode() then
            AutoDrive.setEditorMode(AutoDrive.EDITOR_EXTENDED)
        end
        vehicle.ad.stateModule:startSubPrioTwoWayCreationMode()
    else
        vehicle.ad.stateModule:disableCreationMode()
    end
end

function ADInputManager:input_record_subPrioDualTwoWay(vehicle)
    if not vehicle.ad.stateModule:isInCreationMode() then
        if not AutoDrive.isInExtendedEditorMode() then
            AutoDrive.setEditorMode(AutoDrive.EDITOR_EXTENDED)
        end
        vehicle.ad.stateModule:startSubPrioDualTwoWayCreationMode()
    else
        vehicle.ad.stateModule:disableCreationMode()
    end
end

function ADInputManager:input_debug(vehicle)
    AutoDrive.cycleEditMode()
end

function ADInputManager:input_displayMapPoints(vehicle)
    AutoDrive.cycleEditorShowMode()
end

function ADInputManager:input_increaseSpeed(vehicle)
    vehicle.ad.stateModule:increaseSpeedLimit()
end

function ADInputManager:input_decreaseSpeed(vehicle)
    vehicle.ad.stateModule:decreaseSpeedLimit()
end

function ADInputManager:input_increaseFieldSpeed(vehicle)
    vehicle.ad.stateModule:increaseFieldSpeedLimit()
end

function ADInputManager:input_decreaseFieldSpeed(vehicle)
    vehicle.ad.stateModule:decreaseFieldSpeedLimit()
end

function ADInputManager:input_nextTarget(vehicle)
    if ADGraphManager:getMapMarkerById(1) ~= nil and ADGraphManager:getWayPointById(1) ~= nil then
        local currentTarget = vehicle.ad.stateModule:getFirstMarkerId()
        local nextTarget = ADGraphManager:getNextTargetAlphabetically(currentTarget)
        vehicle.ad.stateModule:setFirstMarker(nextTarget)
        if not vehicle.ad.hasCombine then
            -- not stop / change CP for harvesters
            AutoDrive:StopCP(vehicle)
        end
    end
end

function ADInputManager:input_previousTarget(vehicle)
    if ADGraphManager:getMapMarkerById(1) ~= nil and ADGraphManager:getWayPointById(1) ~= nil then
        local currentTarget = vehicle.ad.stateModule:getFirstMarkerId()
        local previousTarget = ADGraphManager:getPreviousTargetAlphabetically(currentTarget)
        vehicle.ad.stateModule:setFirstMarker(previousTarget)
        if not vehicle.ad.hasCombine then
            -- not stop / change CP for harvesters
            AutoDrive:StopCP(vehicle)
        end
    end
end

function ADInputManager:input_nextTarget_Unload(vehicle)
    if ADGraphManager:getMapMarkerById(1) ~= nil and ADGraphManager:getWayPointById(1) ~= nil then
        local currentTarget = vehicle.ad.stateModule:getSecondMarkerId()
        local nextTarget = ADGraphManager:getNextTargetAlphabetically(currentTarget)
        vehicle.ad.stateModule:setSecondMarker(nextTarget)
    end
end

function ADInputManager:input_previousTarget_Unload(vehicle)
    if ADGraphManager:getMapMarkerById(1) ~= nil and ADGraphManager:getWayPointById(1) ~= nil then
        local currentTarget = vehicle.ad.stateModule:getSecondMarkerId()
        local previousTarget = ADGraphManager:getPreviousTargetAlphabetically(currentTarget)
        vehicle.ad.stateModule:setSecondMarker(previousTarget)
    end
end

function ADInputManager:input_nextFillType(vehicle)
    vehicle.ad.stateModule:nextFillType()
end

function ADInputManager:input_previousFillType(vehicle)
    vehicle.ad.stateModule:previousFillType()
end

function ADInputManager:input_continue(vehicle)
    vehicle.ad.stateModule:getCurrentMode():continue()
end

function ADInputManager:input_callDriver(vehicle)
    if vehicle.spec_combine ~= nil and vehicle.spec_pipe ~= nil and vehicle.spec_enterable ~= nil then
        ADHarvestManager:assignUnloaderToHarvester(vehicle)
    elseif vehicle.ad.attachableCombine ~= nil then
        ADHarvestManager:assignUnloaderToHarvester(vehicle.ad.attachableCombine)
    end
end

function ADInputManager:input_parkVehicle(vehicle, farmId)
    local actualParkDestination = vehicle.ad.stateModule:getParkDestinationAtJobFinished()
    if actualParkDestination >= 1 then
        vehicle.ad.stateModule:setFirstMarker(actualParkDestination)
        AutoDrive:StopCP(vehicle)
        if vehicle.ad.stateModule:isActive() then
            self:input_start_stop(vehicle, farmId) --disable if already active
        end
        vehicle.ad.stateModule:setMode(AutoDrive.MODE_DRIVETO)
        self:input_start_stop(vehicle, farmId)
        vehicle.ad.onRouteToPark = true
    else
        vehicle.ad.onRouteToPark = false
        AutoDriveMessageEvent.sendMessageOrNotification(vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_parkVehicle_noPosSet;", 5000, vehicle.ad.stateModule:getName())
    end
end

function ADInputManager:input_swapTargets(vehicle)
    local currentFirstMarker = vehicle.ad.stateModule:getFirstMarkerId()
    vehicle.ad.stateModule:setFirstMarker(vehicle.ad.stateModule:getSecondMarkerId())
    vehicle.ad.stateModule:setSecondMarker(currentFirstMarker)
    if not vehicle.ad.hasCombine then
        -- not stop / change CP for harvesters
        AutoDrive:StopCP(vehicle)
    end
end

function ADInputManager:input_startHelper(vehicle) -- enable / disable helper passthrough
    vehicle.ad.stateModule:toggleStartHelper()
end

function ADInputManager:input_toggleUsedHelper(vehicle) -- toggle the available helpers
    vehicle.ad.stateModule:toggleUsedHelper()
    vehicle.ad.stateModule:setStartHelper(false) -- disable helper if changed
end

function ADInputManager:input_toggleAutomaticUnloadTarget(vehicle)
    if AutoDrive.automaticUnloadTarget then
        vehicle.ad.stateModule:toggleAutomaticUnloadTarget()
    end
end

function ADInputManager:input_toggleAutomaticPickupTarget(vehicle)
    if AutoDrive.automaticPickupTarget then
        vehicle.ad.stateModule:toggleAutomaticPickupTarget()
    end
end

function ADInputManager:input_toggleLoadByFillLevel(vehicle)
    vehicle.ad.stateModule:toggleLoadByFillLevel()
end

function ADInputManager:input_devAction(vehicle)
    if AutoDrive.devAction ~= nil then
        AutoDrive.devAction(vehicle)
    end
end

function ADInputManager:input_bunkerUnloadType(vehicle)    
    vehicle.ad.stateModule:nextBunkerUnloadType()
end

function ADInputManager:input_refuelVehicle(vehicle, farmId)
    AutoDrive.debugPrint(vehicle, AutoDrive.DC_VEHICLEINFO, "ADInputManager:input_refuelVehicle ")
    local refuelDestination = ADTriggerManager.getClosestRefuelDestination(vehicle, true)
    if refuelDestination ~= nil and refuelDestination >= 1 then
        -- vehicle.ad.stateModule:setFirstMarker(refuelDestination)
        AutoDrive:StopCP(vehicle)
        if vehicle.ad.stateModule:isActive() then
            self:input_start_stop(vehicle, farmId) --disable if already active
        end
        vehicle.ad.stateModule:setMode(AutoDrive.MODE_DRIVETO)
        vehicle.ad.onRouteToRefuel = true
        self:input_start_stop(vehicle, farmId)
    else
        local refuelFillTypes = AutoDrive.getRequiredRefuels(vehicle, true)
        local refuelFillTypeTitle = ""
        if #refuelFillTypes > 0 then
            refuelFillTypeTitle = g_fillTypeManager:getFillTypeByIndex(refuelFillTypes[1]).title
        end
        AutoDriveMessageEvent.sendMessageOrNotification(vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_No_Refuel_Station; %s", 5000, vehicle.ad.stateModule:getName(), refuelFillTypeTitle)
    end
end

function ADInputManager:input_repairVehicle(vehicle, farmId)
    AutoDrive.debugPrint(vehicle, AutoDrive.DC_VEHICLEINFO, "ADInputManager:input_repairVehicle ")
    local repairDestinationMarkerNodeID = AutoDrive:getClosestRepairTrigger(vehicle)
    if repairDestinationMarkerNodeID ~= nil then
        AutoDrive:StopCP(vehicle)
        if vehicle.ad.stateModule:isActive() then
            self:input_start_stop(vehicle, farmId) --disable if already active
        end
        vehicle.ad.onRouteToRepair = true
        vehicle.ad.stateModule:setMode(AutoDrive.MODE_DRIVETO)
        self:input_start_stop(vehicle, farmId)
    else
        AutoDriveMessageEvent.sendMessageOrNotification(vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_No_Repair_Station;", 5000, vehicle.ad.stateModule:getName())
    end
end