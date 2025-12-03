ADHudButton = ADInheritsFrom(ADGenericHudElement)

function ADHudButton:new(posX, posY, width, height, primaryAction, secondaryAction, tertiaryAction, quatenaryAction, fithAction, sixthAction, seventhAction, eightthAction, toolTip, state, visible)
    local o = ADHudButton:create()
    o:init(posX, posY, width, height)
    o.primaryAction = primaryAction
    o.secondaryAction = secondaryAction
    o.tertiaryAction = tertiaryAction
    o.quatenaryAction = quatenaryAction
    o.fithAction = fithAction
    o.sixthAction = sixthAction
    o.seventhAction = seventhAction
    o.eightthAction = eightthAction
    o.toolTip = toolTip
    o.state = state
    o.isVisible = visible

    o.layer = 5

    o.images = o:readImages()

    o.ov = g_overlayManager:createOverlay(o.images[o.state], o.position.x, o.position.y, o.size.width, o.size.height)

    return o
end

function ADHudButton:readImages()
    local images = {}
    local counter = 1

    local path = self.primaryAction
    if self.primaryAction == "input_toggleAutomaticPickupTarget" then
        path = "input_toggleAutomaticUnloadTarget"
    end

    local adTextureConfig = g_overlayManager.textureConfigs["ad_gui"]
    while counter <= 20 do
        -- we can't exit early here, because we have gaps e.g. input_startHelper
        local sliceId = path .. "_" .. counter
        if adTextureConfig.slices[sliceId] ~= nil then
            images[counter] = "ad_gui." .. sliceId
        end
        counter = counter + 1
    end
    return images
end

function ADHudButton:onDraw(vehicle, uiScale)
    self:updateState(vehicle)
    if self.isVisible then
        self.ov:render()
    end
end

function ADHudButton:updateState(vehicle)
    local newState = self:getNewState(vehicle)
    self.ov:setSliceId(self.images[newState])
    self.state = newState
end

function ADHudButton:getNewState(vehicle)
    local newState = self.state
    if self.primaryAction == "input_silomode" then
        if vehicle.ad.stateModule:getMode() == AutoDrive.MODE_DELIVERTO then
            newState = 2
        elseif vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER then
            newState = 3
        elseif vehicle.ad.stateModule:getMode() == AutoDrive.MODE_UNLOAD then
            newState = 5
        elseif vehicle.ad.stateModule:getMode() == AutoDrive.MODE_LOAD then
            newState = 4
        elseif vehicle.ad.stateModule:getMode() == AutoDrive.MODE_BGA then
            newState = 6
        else
            newState = 1
        end
    end

    if self.primaryAction == "input_record" then
        if vehicle.ad.stateModule:isInCreationMode() then
            newState = 2
            if vehicle.ad.stateModule:isInDualCreationMode() then
                newState = 3
            end
            if vehicle.ad.stateModule:isInSubPrioCreationMode() then
                newState = 4
            end
            if vehicle.ad.stateModule:isInSubPrioDualCreationMode() then
                newState = 5
            end
            if vehicle.ad.stateModule:isInNormalTwoWayCreationMode() then
                newState = 6
            end
            if vehicle.ad.stateModule:isInDualTwoWayCreationMode() then
                newState = 7
            end
            if vehicle.ad.stateModule:isInSubPrioTwoWayCreationMode() then
                newState = 8
            end
            if vehicle.ad.stateModule:isInSubPrioDualTwoWayCreationMode() then
                newState = 9
            end
        else
            newState = 1
        end
        self.isVisible = AutoDrive.isEditorModeEnabled()
    end

    if self.primaryAction == "input_start_stop" then
        if vehicle.ad.stateModule:isActive() then
            newState = 2
        else
            newState = 1
        end
    end

    if self.primaryAction == "input_debug" then
        if AutoDrive.isEditorModeEnabled() then
            newState = 2
        else
            newState = 1
        end
    end

    if self.primaryAction == "input_showNeighbor" then
        self.isVisible = AutoDrive.isEditorModeEnabled()

        if vehicle.ad.showSelectedDebugPoint == true then
            newState = 2
        else
            newState = 1
        end
    end

    if self.primaryAction == "input_toggleConnection" then
        self.isVisible = AutoDrive.isEditorModeEnabled()
    end

    if self.primaryAction == "input_nextNeighbor" then
        self.isVisible = AutoDrive.isEditorModeEnabled()
    end

    if self.primaryAction == "input_createMapMarker" then
        self.isVisible = AutoDrive.isEditorModeEnabled()
    end

    if self.primaryAction == "input_routesManager" then
        if (AutoDrive.getSetting("enableRoutesManagerOnDediServer") and g_dedicatedServer ~= nil) or g_dedicatedServer == nil then
            self.isVisible = AutoDrive.isEditorModeEnabled()
        end
    end

    if self.primaryAction == "input_removeWaypoint" then
        self.isVisible = AutoDrive.isEditorModeEnabled()
    end

    if self.primaryAction == "input_editMapMarker" then
        self.isVisible = AutoDrive.isEditorModeEnabled()
    end

    if self.primaryAction == "input_removeMapMarker" then
        self.isVisible = AutoDrive.isEditorModeEnabled()
    end

    if self.primaryAction == "input_parkVehicle" then
        local actualParkDestination = vehicle.ad.stateModule:getParkDestinationAtJobFinished()

        if actualParkDestination >= 1 then
            newState = 1
        else
            newState = 2
        end
    end

    if self.primaryAction == "input_startHelper" then
        local usedHelper = vehicle.ad.stateModule:getUsedHelper()
        if vehicle.ad.stateModule:getStartHelper() then
            newState = usedHelper * 2
        else
            newState = (usedHelper * 2) - 1
        end
        self.isVisible = (not AutoDrive.isEditorModeEnabled()) or (AutoDrive.getSetting("wideHUD") and AutoDrive.getSetting("addSettingsToHUD"))
    end

    if self.primaryAction == "input_bunkerUnloadType" then
        if vehicle.ad.stateModule:getBunkerUnloadTypeIsTrigger() then
            newState = 1
        else
            newState = 2
        end
    end

    if self.primaryAction == "input_toggleAutomaticUnloadTarget" then
        self.isVisible = (vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER or vehicle.ad.stateModule:getMode() == AutoDrive.MODE_UNLOAD or vehicle.ad.stateModule:getMode() == AutoDrive.MODE_LOAD)

        if vehicle.ad.stateModule:getMode() == AutoDrive.MODE_LOAD then            
            if vehicle.ad.stateModule:getAutomaticPickupTarget() then
                newState = 5
            else
                newState = 4
            end
        elseif vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER then
            if vehicle.ad.stateModule:getAutomaticUnloadTarget() then
                newState = 3
            else
                newState = 2
            end
        elseif vehicle.ad.stateModule:getMode() == AutoDrive.MODE_UNLOAD then
            newState = 2
        end
    end

    if self.primaryAction == "input_toggleAutomaticPickupTarget" then
        if vehicle.ad.stateModule:getMode() == AutoDrive.MODE_LOAD
            or vehicle.ad.stateModule:getMode() == AutoDrive.MODE_UNLOAD
            or vehicle.ad.stateModule:getMode() == AutoDrive.MODE_DRIVETO
            then
            newState = 1
        elseif vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER then
            if vehicle.ad.stateModule:getAutomaticPickupTarget() then
                newState = 5
            else
                newState = 4
            end
        elseif vehicle.ad.stateModule:getMode() == AutoDrive.MODE_DELIVERTO then
            if vehicle.ad.stateModule:getAutomaticUnloadTarget() then
                newState = 3
            else
                newState = 2
            end
        end
    end

    if self.primaryAction == "input_toggleLoadByFillLevel" then
        self.isVisible = vehicle.ad.stateModule:getMode() == AutoDrive.MODE_LOAD or vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER
        if vehicle.ad.hasAL and #vehicle.ad.stateModule:getSelectedFillTypes() > 1 then
            newState = 3
        elseif #vehicle.ad.stateModule:getSelectedFillTypes() <= 1 then
            newState = 1
        elseif vehicle.ad.stateModule:getLoadByFillLevel() then
            newState = 2
        else
            newState = 3
        end
    end

    return newState
end

function ADHudButton:act(vehicle, posX, posY, isDown, isUp, button)
    if self.isVisible then
        vehicle.ad.sToolTip = self.toolTip
        vehicle.ad.nToolTipWait = 5
        vehicle.ad.sToolTipInfo = nil
        vehicle.ad.toolTipIsSetting = false

        if self.primaryAction == "input_parkVehicle" then
            local actualParkDestination = vehicle.ad.stateModule:getParkDestinationAtJobFinished()
            if actualParkDestination >= 1 and ADGraphManager:getMapMarkerById(actualParkDestination) ~= nil then
                vehicle.ad.sToolTipInfo = ADGraphManager:getMapMarkerById(actualParkDestination).name
            end

        end

        if self.primaryAction == "input_toggleAutomaticUnloadTarget" or self.primaryAction == "input_toggleAutomaticPickupTarget" then
            self:actOnIcons(vehicle, posX, posY, isDown, isUp, button)
            return
        end

        if button == 1 and isUp and not AutoDrive.leftLSHIFTmodifierKeyPressed and not AutoDrive.leftCTRLmodifierKeyPressed then
            ADInputManager:onInputCall(vehicle, self.primaryAction)
            return true
        elseif (button == 3 or button == 2) and isUp and not AutoDrive.leftLSHIFTmodifierKeyPressed and not AutoDrive.leftCTRLmodifierKeyPressed then
            ADInputManager:onInputCall(vehicle, self.secondaryAction)
            return true
        elseif button == 1 and isUp and AutoDrive.leftLSHIFTmodifierKeyPressed and not AutoDrive.leftCTRLmodifierKeyPressed then
            ADInputManager:onInputCall(vehicle, self.tertiaryAction)
            return true
        elseif (button == 3 or button == 2) and isUp and AutoDrive.leftLSHIFTmodifierKeyPressed and not AutoDrive.leftCTRLmodifierKeyPressed then
            ADInputManager:onInputCall(vehicle, self.quatenaryAction)
            return true
        elseif button == 1 and isUp and not AutoDrive.leftLSHIFTmodifierKeyPressed and AutoDrive.leftCTRLmodifierKeyPressed then
            ADInputManager:onInputCall(vehicle, self.fithAction)
            return true
        elseif (button == 3 or button == 2) and isUp and not AutoDrive.leftLSHIFTmodifierKeyPressed and AutoDrive.leftCTRLmodifierKeyPressed then
            ADInputManager:onInputCall(vehicle, self.sixthAction)
            return true
        elseif button == 1 and isUp and AutoDrive.leftLSHIFTmodifierKeyPressed and AutoDrive.leftCTRLmodifierKeyPressed then
            ADInputManager:onInputCall(vehicle, self.seventhAction)
            return true
        elseif (button == 3 or button == 2) and isUp and AutoDrive.leftLSHIFTmodifierKeyPressed and AutoDrive.leftCTRLmodifierKeyPressed then
            ADInputManager:onInputCall(vehicle, self.eightthAction)
            return true
        end

        if button > 0 and button < 4 and isDown then
            return true, true
        end
    end

    return false
end

function ADHudButton:actOnIcons(vehicle, posX, posY, isDown, isUp, button)
    if button == 1 and isUp and not AutoDrive.leftLSHIFTmodifierKeyPressed then
        if self.primaryAction == "input_toggleAutomaticUnloadTarget" then
            if vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER then
                ADInputManager:onInputCall(vehicle, self.primaryAction)
            elseif vehicle.ad.stateModule:getMode() == AutoDrive.MODE_LOAD then
                ADInputManager:onInputCall(vehicle, "input_toggleAutomaticPickupTarget")
            end
        end

        if self.primaryAction == "input_toggleAutomaticPickupTarget" then
            if vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER then
                ADInputManager:onInputCall(vehicle, self.primaryAction)
            elseif vehicle.ad.stateModule:getMode() == AutoDrive.MODE_UNLOAD then
                ADInputManager:onInputCall(vehicle, "input_toggleAutomaticUnloadTarget")
            end
        end

        return true
    end

    return false
end
