-- positive X -> left
-- negative X -> right
function AutoDrive.createWayPointRelativeToVehicle(vehicle, offsetX, offsetZ)
    local wayPoint = {}
    wayPoint.x, wayPoint.y, wayPoint.z = AutoDrive.localToWorld(vehicle, offsetX, 0, offsetZ, vehicle.ad.ADRootNode)
    return wayPoint
end

function AutoDrive.createWayPointRelativeToDischargeNode(vehicle, offsetX, offsetZ)
    if not vehicle.ad.isCombine or AutoDrive.getDischargeNode(vehicle) == nil then
        return AutoDrive.createWayPointRelativeToVehicle(vehicle, offsetX, offsetZ)
    end
    local wayPoint = {}
    local referenceAxis = vehicle.components[1].node
    if vehicle.components[2] ~= nil and vehicle.components[2].node ~= nil then
        referenceAxis = vehicle.components[2].node
    end
    local node = AutoDrive.getDischargeNode(vehicle)
    local worldOffsetX, worldOffsetY, worldOffsetZ = AutoDrive.localDirectionToWorld(vehicle, offsetX, 0, offsetZ, referenceAxis)
    local x, y, z = getWorldTranslation(node)
    wayPoint.x, wayPoint.y, wayPoint.z = x + worldOffsetX, y + worldOffsetY, z + worldOffsetZ
    return wayPoint
end

function AutoDrive.isTrailerInCrop(vehicle, enlargeDetectionArea)
    local widthFactor = 1
    if enlargeDetectionArea then
        widthFactor = 1.5
    end

    local trailers, trailerCount = AutoDrive.getAllUnits(vehicle)
    local trailer = trailers and trailers[trailerCount]
    local inCrop = false
    if trailer ~= nil then
        if trailer.ad == nil then
            trailer.ad = {}
        end
        ADSensor:handleSensors(trailer, 0)
        inCrop = trailer.ad.sensors.centerSensorFruit:pollInfo(true, widthFactor)
    end
    return inCrop
end

function AutoDrive.isVehicleOrTrailerInCrop(vehicle, enlargeDetectionArea)
    local widthFactor = 1
    if enlargeDetectionArea then
        widthFactor = 1.5
    end

    return AutoDrive.isTrailerInCrop(vehicle, enlargeDetectionArea) or vehicle.ad.sensors.centerSensorFruit:pollInfo(true, widthFactor)
end

function AutoDrive:checkIsConnected(toCheck, other)
    local isAttachedToMe = false
    if toCheck == nil or other == nil then
        return false
    end
    if toCheck == other then
        return true
    end
    for _, implement in pairs(AutoDrive.getAllImplements(toCheck, true)) do
        if implement == other then
            return true
        end

        if implement.spec_baleGrab ~= nil and implement.spec_baleGrab.dynamicMountedObjects ~= nil and table.contains(implement.spec_baleGrab.dynamicMountedObjects, other) then
            return true
        end

        if implement.spec_dynamicMountAttacher ~= nil and implement.spec_dynamicMountAttacher.dynamicMountedObjects ~= nil and table.contains(implement.spec_dynamicMountAttacher.dynamicMountedObjects, other) then
            return true
        end
    end

    return isAttachedToMe
end

function AutoDrive.defineMinDistanceByVehicleType(vehicle, reverse)
    local min_distance = vehicle.size.length / 2
    if vehicle.getAISteeringNode ~= nil then
        local _, _, diffZ = localToLocal(vehicle:getAISteeringNode(), vehicle.components[1].node, 0, 0, 0)
        
        -- diffZ > 0 -> steering node ahead of vehicle center
        if reverse ~= true then
            min_distance = min_distance - diffZ
        else
            min_distance = min_distance + diffZ
        end
        -- print(string.format("Min distance for %s is %.2f with diffZ %.2f", vehicle:getName(), min_distance, diffZ))
    end

    
    return math.max(min_distance, 0.1)
end

function AutoDrive.defineMinDistanceByVehicleTypeOld(vehicle)
    local min_distance = 1.8
    if
        vehicle.typeDesc == "combine" or vehicle.typeDesc == "harvester" or vehicle.typeName == "combineDrivable" or vehicle.typeName == "selfPropelledMower" or vehicle.typeName == "woodHarvester" or vehicle.typeName == "combineCutterFruitPreparer" or vehicle.typeName == "drivableMixerWagon" or
            vehicle.typeName == "cottonHarvester" or
            vehicle.typeName == "pdlc_claasPack.combineDrivableCrawlers"
     then
        min_distance = 6
    elseif vehicle.typeDesc == "telehandler" or vehicle.spec_crabSteering ~= nil then --If vehicle has 4 steering wheels like xerion or hardi self Propelled sprayer then also min_distance = 3;
        min_distance = 3
    elseif vehicle.typeDesc == "truck" then
        min_distance = 3
    end
    -- If vehicle is quadtrack then also min_distance = 6;
    if vehicle.spec_articulatedAxis ~= nil and vehicle.spec_articulatedAxis.rotSpeed ~= nil then
        min_distance = 6
    end
    return min_distance
end

function AutoDrive.getVehicleMaxSpeed(vehicle)
    -- 255 is the max value to prevent errors with MP sync
    if vehicle ~= nil and vehicle.spec_motorized ~= nil and vehicle.spec_motorized.motor ~= nil then
        local motor = vehicle.spec_motorized.motor
        local maxSpeed = math.max(motor:getMaximumForwardSpeed() * 3.6, motor:getMaximumBackwardSpeed() * 3.6)
        return math.min(maxSpeed, 255)
    end
    return 255
end

function AutoDrive.renameDriver(vehicle, name, sendEvent)
    if name:len() > 1 and vehicle ~= nil and vehicle.ad ~= nil then
        if sendEvent == nil or sendEvent == true then
            -- Propagating driver rename all over the network
            AutoDriveRenameDriverEvent.sendEvent(vehicle, name)
        else
            vehicle.ad.stateModule:setName(name)
        end
    end
end

-- return fillType to refuel or nil if no refuel required
--[[
ignoreFillLevel: if true the fuel type is captured of level < 90%
]]
function AutoDrive.getRequiredRefuels(vehicle, ignoreFillLevel)
    local ret = {}
    local minFillLevel = 0.90   -- to prevent cycles between different fuel fillTypes we take 90% as full level
    if vehicle ~= nil then
        local spec = vehicle.spec_motorized
        if spec ~= nil and spec.consumersByFillTypeName ~= nil then

            for fillTypeName, consumer in pairs(spec.consumersByFillTypeName) do
                local currentFillLevelPercentage = vehicle:getFillUnitFillLevelPercentage(consumer.fillUnitIndex)
                local needFuel = (currentFillLevelPercentage < AutoDrive.REFUEL_LEVEL or (ignoreFillLevel and (currentFillLevelPercentage < minFillLevel)))
                if needFuel then
                    if not table.contains(AutoDrive.nonFillableFillTypes, fillTypeName) then
                        table.insert(ret, consumer.fillType)
                    end
                end
            end
        end
    end
    return ret
end

function AutoDrive.combineIsTurning(combine)
    local cpIsTurning = AutoDrive:getIsCPTurning(combine)
    if cpIsTurning then
        -- CP turn maneuver might get noMovementTimer expired, so return here already
        return true
    end
    local aiIsTurning = false
    local rootVehicle = nil
    if combine.getRootVehicle ~= nil then
        rootVehicle = combine:getRootVehicle()
        if rootVehicle ~= nil then
            aiIsTurning = rootVehicle.getAIFieldWorkerIsTurning ~= nil and rootVehicle:getAIFieldWorkerIsTurning()
        end
    end
    local combineIsTurning = cpIsTurning or aiIsTurning

    --Check if we are close to the field borders and about to turn
    local fieldLengthInFront = AutoDrive.getLengthOfFieldInFront(combine, true, 50, 5)
    local fieldLengthBehind = math.abs(AutoDrive.getLengthOfFieldInFront(combine, false, 50, -5))

    if (fieldLengthInFront <= 20 or fieldLengthBehind <= 20) and combine.ad.noMovementTimer.elapsedTime < 5000 and not combine.ad.isChopper then
        combineIsTurning = true
    end

    if not combineIsTurning then --(combine.ad.driveForwardTimer:done() and (not combine:getIsBufferCombine()))
        return false
    end
    if combine.ad.noMovementTimer.elapsedTime > 3000 then
        return false
    end
    return true
end

function AutoDrive.mouseIsAtPos(position, radius)
    local x, y, _ = project(position.x, position.y + AutoDrive.drawHeight + AutoDrive.getSetting("lineHeight"), position.z)

    if g_lastMousePosX < (x + radius) and g_lastMousePosX > (x - radius) then
        if g_lastMousePosY < (y + radius) and g_lastMousePosY > (y - radius) then
            return true
        end
    end

    return false
end

function AutoDrive.isVehicleInBunkerSiloArea(vehicle)
    if not (vehicle.ad.stateModule:getCurrentMode():shouldUnloadAtTrigger() == true) then
        -- check only for bunker silo if should unload to improve performance
        return false
    end
    for _, trigger in pairs(ADTriggerManager.getBunkerSilos()) do
        local x, y, z = getWorldTranslation(vehicle.components[1].node)
        local tx, _, tz = x, y, z + 1
        if trigger ~= nil and trigger.bunkerSiloArea ~= nil then
            local x1, z1 = trigger.bunkerSiloArea.sx, trigger.bunkerSiloArea.sz
            local x2, z2 = trigger.bunkerSiloArea.wx, trigger.bunkerSiloArea.wz
            local x3, z3 = trigger.bunkerSiloArea.hx, trigger.bunkerSiloArea.hz
            if MathUtil.hasRectangleLineIntersection2D(x1, z1, x2 - x1, z2 - z1, x3 - x1, z3 - z1, x, z, tx - x, tz - z) then
                return true
            end

            local trailers, trailerCount = AutoDrive.getAllUnits(vehicle)
            if trailers and trailerCount > 0 then
                for _, trailer in pairs(trailers) do
                    if AutoDrive.isTrailerInBunkerSiloArea(trailer, trigger) then
                        return true
                    end
                end
            end
        end
    end

    return false
end

function AutoDrive.isEditorModeEnabled()
    return (AutoDrive.getSetting("EditorMode") == AutoDrive.EDITOR_ON) or (AutoDrive.getSetting("EditorMode") == AutoDrive.EDITOR_EXTENDED)
end

function AutoDrive.isEditorShowEnabled()
    return (AutoDrive.getSetting("EditorMode") == AutoDrive.EDITOR_SHOW)
end

function AutoDrive.isInExtendedEditorMode()
    return (AutoDrive.getSetting("EditorMode") == AutoDrive.EDITOR_EXTENDED)
end

function AutoDrive.getEditorMode()
    return (AutoDrive.getSetting("EditorMode"))
end

function AutoDrive.setEditorMode(editorMode)
    AutoDrive.setSettingState("EditorMode", editorMode)
    ADUserDataManager:sendToServer()
end

function AutoDrive.cycleEditMode()
    local vehicle = AutoDrive.getControlledVehicle()
    if g_client ~= nil and vehicle ~= nil then

        if vehicle ~= nil and vehicle.ad ~= nil then
            AutoDrive.resetMouseSelections(vehicle)
        end
        if (AutoDrive.getSetting("EditorMode") == AutoDrive.EDITOR_OFF) then
            AutoDrive.setEditorMode(AutoDrive.EDITOR_EXTENDED)
        else
            AutoDrive.setEditorMode(AutoDrive.EDITOR_OFF)
            if g_server ~= nil and g_client ~= nil and g_dedicatedServer == nil then
                -- in SP always delete color selection wayPoints if there are any
                ADGraphManager:deleteColorSelectionWayPoints()
            end
            if vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.stateModule ~= nil then
                if not AutoDrive.getSetting("RecordWhileNotInVehicle") then
                    vehicle.ad.stateModule:disableCreationMode()
                end
            end
        end
    end
end

function AutoDrive.cycleEditorShowMode()
    local vehicle = AutoDrive.getControlledVehicle()

    if g_client ~= nil and vehicle ~= nil then
        if (AutoDrive.getSetting("EditorMode") == AutoDrive.EDITOR_OFF) then
            AutoDrive.setEditorMode(AutoDrive.EDITOR_SHOW)
        else
            AutoDrive.setEditorMode(AutoDrive.EDITOR_OFF)
            if vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.stateModule ~= nil then
                if not AutoDrive.getSetting("RecordWhileNotInVehicle") then
                    vehicle.ad.stateModule:disableCreationMode()
                end
            end
        end
    end
end

function AutoDrive.isInConstructionMode()
    if not g_gui:getIsGuiVisible() or g_gui.currentGuiName ~= "ConstructionScreen" or g_constructionScreen == nil then
        -- not in construction screen
        return false
    end
    if g_constructionScreen.categorySelector == nil  or g_constructionScreen.categories == nil or g_constructionScreen.camera == nil then
        return false
    end
    local index = g_constructionScreen.categorySelector.selectedIndex
    if index == nil or g_constructionScreen.categories[index] == nil then
        return false
    end
    return true
end

function AutoDrive.isInConstructionModeLandScaping()
    local index = g_constructionScreen.categorySelector.selectedIndex
    -- consider landscaping category
    return AutoDrive.isInConstructionMode() and index and g_constructionScreen.categories[index].name == "LANDSCAPING"
end

function AutoDrive.isMouseActiveForHud()
    return not g_gui:getIsGuiVisible() or (g_gui.currentGuiName == "InGameMenu" and AutoDrive.aiFrameOpen)
end

function AutoDrive.isMouseActiveForEditor()
    return not g_gui:getIsGuiVisible() or AutoDrive.isInConstructionModeLandScaping()
end


function AutoDrive.getSelectedWorkTool(vehicle)
    local selectedWorkTool = nil

    for _, implement in pairs(AutoDrive.getAllImplements(vehicle)) do
        if implement.getIsSelected ~= nil and implement:getIsSelected() then
            selectedWorkTool = implement
        end
    end

    return selectedWorkTool
end

function AutoDrive.getVehicleLeadingEdge(vehicle)
    local leadingEdge = 0
    local implements = AutoDrive.getAllImplements(vehicle)
    for _, implement in pairs(implements) do
        local implementX, implementY, implementZ = getWorldTranslation(implement.components[1].node)
        local _, _, diffZ = AutoDrive.worldToLocal(vehicle, implementX, implementY, implementZ)
        if diffZ > 0 and implement.size.length ~= nil then
            leadingEdge = math.max(leadingEdge, diffZ + (implement.size.length / 2) - (vehicle.size.length / 2))
        end
    end
    return leadingEdge
end

function AutoDrive.getAllImplements(vehicle, includeVehicle)
    local allImp = {}

    if vehicle ~= nil and vehicle.getAttachedImplements and #vehicle:getAttachedImplements() > 0 then

        local function addAllAttached(obj)
            if obj.getAttachedImplements ~= nil then
                for _, imp in pairs(obj:getAttachedImplements()) do
                    addAllAttached(imp.object)
                    table.insert(allImp, imp.object)
                end
            end
        end

        addAllAttached(vehicle)
    end

    if includeVehicle then
        table.insert(allImp, vehicle)
    end

    return allImp
end

function AutoDrive.getFrontImplements(vehicle)
    if vehicle == nil then
        return nil, nil
    end
    local rootVehicle = vehicle.getRootVehicle and vehicle:getRootVehicle()
    if rootVehicle == nil then
        return nil, nil
    end
    local frontImplements = nil
    local mostFrontImplement = nil
    local frontDistance = 0
    if rootVehicle == vehicle then
        -- consider only attachments to root vehicle
        for _, implement in pairs(AutoDrive.getAllImplements(rootVehicle)) do
            if implement ~= nil and implement ~= rootVehicle then
                local implementX, implementY, implementZ = getWorldTranslation(implement.components[1].node)
                local _, _, diffZ = AutoDrive.worldToLocal(rootVehicle, implementX, implementY, implementZ)
                if diffZ > 0 then
                    if frontImplements == nil then
                        frontImplements = {}
                    end
                    table.insert(frontImplements,implement)
                end
                if diffZ > frontDistance then
                    frontDistance = diffZ
                    mostFrontImplement = implement
                end
            end
        end
    end
    return frontImplements, mostFrontImplement
end

function AutoDrive.foldAllImplements(vehicle)
    local implements = AutoDrive.getAllImplements(vehicle, true)
    local spec
    AutoDrive.setAugerPipeOpen(implements, false) -- close all pipes first
    AutoDrive.closeAllCurtains(implements, true) -- close curtain at UAL trailers
    for _, implement in pairs(implements) do
        local specs = implement.spec_baleLoader and implement.spec_baler
        if specs then
            if not implement.spec_baleLoader.fullAutomaticUnloading then
                -- TODO: check in case an implement without fullAutomaticUnloading comes into picture
                -- at the moment only Krone BiG Pack 1290 HDP VC use this configuration of baler and baleunloader with fullAutomaticUnloading
                if implement:getFillUnitFillLevel(implement.spec_baleLoader.fillUnitIndex) > 0 then
                    -- BaleLoader
                    if implement:getIsAutomaticBaleUnloadingAllowed() then
                        if not implement:getIsAutomaticBaleUnloadingInProgress() then
                            implement:startAutomaticBaleUnloading()
                        end
                    end
                else
                    -- Baler
                    if implement:isUnloadingAllowed() then
                        implement:dropBaleFromPlatform()
                    end
                end
            end
        else
            spec = implement.spec_baleLoader
            if spec and spec.doStateChange then
                if spec.isInWorkPosition and spec.emptyState == BaleLoader.EMPTY_NONE then
                    spec:doStateChange(BaleLoader.CHANGE_BUTTON_WORK_TRANSPORT)
                end
            end
        end
        spec = implement.spec_plow
        if spec then
            if spec.getIsPlowRotationAllowed and spec:getIsPlowRotationAllowed() and spec.rotationMax ~= false then
                spec:setRotationMax(false)
            end
        end
        spec = implement.spec_foldable
        if spec then
            local shouldFold = true
            if implement.spec_baler then
                -- delay folding for baler with and without collector to ensure all unloadable bales are unloaded before folding
                if implement.ad == nil then
                    implement.ad = {}
                end
                if implement:getIsBaleUnloading() then
                    implement.ad.foldDelay = g_time
                end
                if implement.ad.foldDelay and implement.ad.foldDelay + 1000 > g_time then
                    shouldFold = false
                end
            end
            if shouldFold and not AutoDrive.isVehicleFolded(implement) then
                if spec ~= nil and implement.getToggledFoldDirection then
                    if implement:getToggledFoldDirection() ~= spec.turnOnFoldDirection then
                        local toggledFoldDirection = implement:getToggledFoldDirection()
                        if implement.getIsFoldAllowed and toggledFoldDirection and implement:getIsFoldAllowed(toggledFoldDirection) and implement.setFoldState then
                            implement:setFoldState(toggledFoldDirection, false)
                        end
                    end
                end
            end
        end
        -- combine handle ladder separate when enter or leave combine
        AutoDrive.foldLadder(implement)
    end
end

function AutoDrive.getAllImplementsFolded(vehicle)
    local ret = true
    local implements = AutoDrive.getAllImplements(vehicle, true)
    local spec
    for _, implement in pairs(implements) do
        -- check if all is set to transport position
        if not implement.spec_aiDrivable and implement.getIsAIReadyToDrive then
            -- spec_aiDrivable might cause infinite loop
            ret = ret and implement:getIsAIReadyToDrive()
        end

        spec = implement.spec_baleLoader
        if spec then
            -- bale loader
            ret = ret and not implement:getIsAutomaticBaleUnloadingInProgress()
            ret = ret and not implement:getIsBaleLoaderFoldingPlaying()
            ret = ret and spec.emptyState == BaleLoader.EMPTY_NONE
        end

        spec = implement.spec_baler
        if spec then
            -- baler
            ret = ret and implement:getIsFoldAllowed() and not implement:getIsBaleUnloading()
        end

        spec = implement.spec_foldable
        if spec then
            ret = ret and AutoDrive.isVehicleFolded(implement)
        end

        spec = implement.spec_pipe
        if spec and spec.hasMovablePipe then
            ret = ret and spec.currentState == 1
        end

        spec = implement.spec_plow
        if spec then
            ret = ret and not spec.rotationMax
            ret = ret and not (spec.rotationPart.turnAnimation and implement:getIsAnimationPlaying(spec.rotationPart.turnAnimation))
        end

        ret = ret and AutoDrive.isCurtainClosed(implement)

        -- combine handle ladder separate when enter or leave combine
        ret = ret and AutoDrive.isLadderFolded(implement)
    end
    return ret
end

function AutoDrive.resetFoldState(vehicle)
    if vehicle and vehicle.ad then
        vehicle.ad.foldStartTime = g_time
    end
end

function AutoDrive.foldLadder(vehicle)
    local spec = vehicle.spec_combine
    if spec and not AutoDrive.isLadderFolded(vehicle) then
        local ladder = spec.ladder
        if ladder and ladder.animName and ladder.foldDirection and vehicle.getAnimationTime then
            if not vehicle:getIsAnimationPlaying(ladder.animName) then
                vehicle:playAnimation(ladder.animName, -ladder.foldDirection, vehicle:getAnimationTime(ladder.animName), true)
            end
        end
    end
end

function AutoDrive.isLadderFolded(vehicle)
    local ret = true
    local spec = vehicle.spec_combine
    if spec then
        local ladder = spec.ladder
        if ladder and ladder.animName and vehicle.getAnimationTime then
            local foldAnimTime = vehicle:getAnimationTime(ladder.animName)
            if foldAnimTime then
                if ladder.foldDirection == 1 then
                    ret = ret and (foldAnimTime < 0.01)
                else
                    ret = ret and (foldAnimTime >= 1)
                end
            end
        end
    end
    return ret
end

function AutoDrive.closeCurtain(vehicle)
    local leftDone, rightDone = false, false
    local spec = vehicle.spec_trailer
    if spec and not AutoDrive.isCurtainClosed(vehicle) then
        for _, tipSide in pairs(spec.tipSides) do
            if tipSide and tipSide.manualTipToggle and tipSide.animation and tipSide.animation.name then
                if vehicle.getIsAnimationPlaying and vehicle.playAnimation then
                    if not vehicle:getIsAnimationPlaying(tipSide.animation.name) then
                        vehicle:playAnimation(tipSide.animation.name, tipSide.animation.closeSpeedScale, vehicle:getAnimationTime(tipSide.animation.name), true)
                    end
                end
            end
        end
    end
end

function AutoDrive.closeAllCurtains(trailers, onlyUAL)
    if trailers and #trailers > 0 then
        for _, trailer in ipairs(trailers) do
            -- if (not onlyUAL) or (onlyUAL and AutoDrive:hasAL(trailer)) then
                AutoDrive.closeCurtain(trailer)
            -- end
        end
    end
end

function AutoDrive.openCurtain(vehicle)
    local leftDone, rightDone = false, false
    local spec = vehicle.spec_trailer
    if spec and AutoDrive.isCurtainClosed(vehicle) then
        for _, tipSide in pairs(spec.tipSides) do
            if tipSide and tipSide.manualTipToggle and tipSide.animation and tipSide.animation.name then
                if vehicle.getIsAnimationPlaying and vehicle.playAnimation then
                    if not leftDone and string.find(tipSide.animation.name, "Left") then
                        leftDone = true
                        if not vehicle:getIsAnimationPlaying(tipSide.animation.name) then
                            vehicle:playAnimation(tipSide.animation.name, tipSide.animation.speedScale, vehicle:getAnimationTime(tipSide.animation.name), true)
                        end
                    end
                    if not rightDone and string.find(tipSide.animation.name, "Right") then
                        rightDone = true
                        if not vehicle:getIsAnimationPlaying(tipSide.animation.name) then
                            vehicle:playAnimation(tipSide.animation.name, tipSide.animation.speedScale, vehicle:getAnimationTime(tipSide.animation.name), true)
                        end
                    end
                end
            end
        end
    end
end

function AutoDrive.openAllCurtains(trailers, onlyUAL)
    if trailers and #trailers > 0 then
        for _, trailer in ipairs(trailers) do
            if (not onlyUAL) or (onlyUAL and AutoDrive:hasAL(trailer)) then
                AutoDrive.openCurtain(trailer)
            end
        end
    end
end

function AutoDrive.isCurtainClosed(vehicle)
    local leftDone, rightDone = false, false
    local ret = true
    local spec = vehicle.spec_trailer
    if spec then
        for _, tipSide in pairs(spec.tipSides) do
            if tipSide and tipSide.manualTipToggle and tipSide.animation and tipSide.animation.name then
                if tipSide.animation.closeSpeedScale ~= 0 and vehicle.getAnimationDuration and vehicle:getAnimationDuration(tipSide.animation.name) > 1 then
                    local animationTime = vehicle:getAnimationTime(tipSide.animation.name)
                    ret = ret and animationTime <= 0.01
                end
            end
        end
    end
    return ret
end

function AutoDrive.getAllCurtainsClosed(trailers, onlyUAL)
    local ret = true
    if trailers and #trailers > 0 then
        for _, trailer in ipairs(trailers) do
            if (not onlyUAL) or (onlyUAL and AutoDrive:hasAL(trailer)) then
                ret = ret and AutoDrive.isCurtainClosed(trailer)
            end
        end
    end
    return ret
end

function AutoDrive.isVehicleFolded(vehicle)
    local spec = vehicle.spec_foldable
    if spec ~= nil and #spec.foldingParts > 0 then
        return spec.turnOnFoldDirection == -1 and spec.foldAnimTime >= 0.99 or spec.turnOnFoldDirection == 1 and spec.foldAnimTime <= 0.01
    end
    return true
end

-- set or delete park destination for selected vehicle, tool from user input action, client mode!
function AutoDrive.setActualParkDestination(vehicle)
    local actualParkDestination = -1
    local selectedWorkTool = nil

    if vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.stateModule ~= nil and vehicle.ad.stateModule:getFirstMarker() ~= nil then
        local firstMarkerID = vehicle.ad.stateModule:getFirstMarkerId()
        if firstMarkerID > 0 then
            local mapMarker = ADGraphManager:getMapMarkerById(firstMarkerID)
            -- do not allow to set debug marker as park destination
            if mapMarker ~= nil and mapMarker.isADDebug ~= true then
                selectedWorkTool = AutoDrive.getSelectedWorkTool(vehicle)

                if selectedWorkTool == nil then
                    -- no attachment selected, so use the vehicle itself
                    selectedWorkTool = vehicle
                end

                if selectedWorkTool ~= nil then
                    if AutoDrive.isInExtendedEditorMode() and AutoDrive.leftCTRLmodifierKeyPressed and not AutoDrive.leftALTmodifierKeyPressed then
                        -- assign park destination
                        if vehicle.advd ~= nil then
                            vehicle.advd:setParkDestination(selectedWorkTool, firstMarkerID)
                        end

                        -- on client sendMessage is not allowed, so add the message to ADMessagesManager to show it
                        local messageText = "$l10n_AD_parkVehicle_selected; %s"
                        local messageArg = vehicle.ad.stateModule:getFirstMarker().name
                        -- localization
                        messageText = AutoDrive.localize(messageText)
                        -- formatting
                        messageText = string.format(messageText, messageArg)
                        ADMessagesManager:addMessage(vehicle, ADMessagesManager.messageTypes.INFO, messageText, 5000)

                    elseif AutoDrive.isInExtendedEditorMode() and not AutoDrive.leftCTRLmodifierKeyPressed and AutoDrive.leftALTmodifierKeyPressed then
                        -- delete park destination
                        if vehicle.advd ~= nil then
                            vehicle.advd:setParkDestination(selectedWorkTool, -1)
                        end

                        -- on client sendMessage is not allowed, so add the message to ADMessagesManager to show it
                        local messageText = "$l10n_AD_parkVehicle_deleted; %s"
                        local messageArg = vehicle.ad.stateModule:getFirstMarker().name
                        -- localization
                        messageText = AutoDrive.localize(messageText)
                        -- formatting
                        messageText = string.format(messageText, messageArg)
                        ADMessagesManager:addMessage(vehicle, ADMessagesManager.messageTypes.INFO, messageText, 5000)
                    end
                end
            end
        end
    end
end

-- MP: Important! vehicle:getIsEntered() is not working as expected on server!
-- This is the alternative MP approach
function AutoDrive:getIsEntered(vehicle)
    local user = nil
    if g_dedicatedServer ~= nil and vehicle ~= nil and g_currentMission.userManager ~= nil and g_currentMission.userManager.getUserByConnection ~= nil and vehicle.getOwner ~= nil then
        -- MP
        user = g_currentMission.userManager:getUserByConnection(vehicle:getOwner())
    else
        -- SP
        if vehicle ~= nil and vehicle.getIsEntered ~= nil then
            if vehicle.getIsControlled ~= nil then
                return vehicle:getIsEntered() or vehicle:getIsControlled()
            else
                return vehicle:getIsEntered()
            end
        end
    end
    return user ~= nil
end

function AutoDrive:getAIFrameFarmId()
    local actualFarmId = nil
    if AutoDrive.aiFrameOpen and AutoDrive.aiFrameVehicle and AutoDrive.aiFrameVehicle.ad and AutoDrive.aiFrameVehicle.ad.stateModule then
        local player = AutoDrive.getPlayer()
        if player and player.farmId and player.farmId > 0 then
            actualFarmId = player.farmId
        end
    end
    return actualFarmId
end

function AutoDrive:getColorKeyNames()
    local colorKeyNames = {}
	for k, v in pairs(AutoDrive.colors) do
		local tagName = string.format("%s",k)
        table.insert(colorKeyNames, tagName)
    end
    return colorKeyNames
end

function AutoDrive:getColorAssignment(colorKey)
    if AutoDrive.currentColors[colorKey] ~= nil then
        local r = AutoDrive.currentColors[colorKey][1]
        local g = AutoDrive.currentColors[colorKey][2]
        local b = AutoDrive.currentColors[colorKey][3]
        local a = AutoDrive.currentColors[colorKey][4]
        return {r, g, b, a}
    else
        return {AutoDrive.currentColors.ad_color_default[1], AutoDrive.currentColors.ad_color_default[2], AutoDrive.currentColors.ad_color_default[3], AutoDrive.currentColors.ad_color_default[4]}
    end
end

function AutoDrive:setColorAssignment(colorKey, r, g, b, a)
    if AutoDrive.currentColors[colorKey] ~= nil then
        AutoDrive.currentColors[colorKey][1] = r
        AutoDrive.currentColors[colorKey][2] = g
        AutoDrive.currentColors[colorKey][3] = b
        if a ~= nil then
            AutoDrive.currentColors[colorKey][4] = a
        end
    end
end

function AutoDrive:resetColorAssignment(colorKey, all)
    if all ~= nil and all == true then
        AutoDrive.currentColors = {}
        for k, v in pairs(AutoDrive.colors) do
            AutoDrive.currentColors[k] = {}
            for i = 1, 4 do
                AutoDrive.currentColors[k][i] = AutoDrive.colors[k][i]
            end
        end
    elseif AutoDrive.currentColors[colorKey] ~= nil then
        for i = 1, 4 do
            AutoDrive.currentColors[colorKey][i] = AutoDrive.colors[colorKey][i]
        end
    end
end

-- return the vehicle which can be AD controlled
function AutoDrive.getADFocusVehicle(debug)
	local vehicle = nil
    local controlledVehicle = AutoDrive.getControlledVehicle()
    if AutoDrive.aiFrameOpen and AutoDrive.aiFrameVehicle ~= nil and AutoDrive.aiFrameVehicle.ad ~= nil and AutoDrive.aiFrameVehicle.ad.stateModule ~= nil then
        vehicle = AutoDrive.aiFrameVehicle
    elseif controlledVehicle ~= nil and controlledVehicle.ad ~= nil and controlledVehicle.ad.stateModule ~= nil then
        vehicle = controlledVehicle
    end
    return vehicle
end

function AutoDrive.getSupportedFillTypesOfAllUnitsAlphabetically(vehicle, excludedFillType)
    local supportedFillTypes = {}
    local autoLoadFillTypes = nil -- AutoLoad

    if vehicle ~= nil then
        local trailers, _ = AutoDrive.getAllUnits(vehicle)
        if trailers then
            -- AutoLoad
            for _, trailer in ipairs(trailers) do
                if AutoDrive:hasAL(trailer) then
                    local alFillTypes = AutoDrive:getALFillTypes(trailer)
                    if alFillTypes ~= nil and #alFillTypes > 0 then
                        -- self.autoLoadFillTypes is either nil or it contains items. It is never empty.
                        autoLoadFillTypes = alFillTypes
                        break
                    end
                end
            end
        end
        if autoLoadFillTypes ~= nil then
            for _, fillType in ipairs(autoLoadFillTypes) do
                if fillType.fillTypeID ~= excludedFillType then
                    table.insert(supportedFillTypes, fillType.fillTypeID)
                end
            end
        else
            local dischargeableUnits = AutoDrive.getAllDischargeableUnits(vehicle, true)
            if dischargeableUnits and #dischargeableUnits > 0 then
                for i = 1, #dischargeableUnits do
                    local dischargeableUnit = dischargeableUnits[i]
                    if dischargeableUnit.object and dischargeableUnit.object.getFillUnitSupportedFillTypes ~= nil then
                        if dischargeableUnit.fillUnitIndex and dischargeableUnit.fillUnitIndex > 0 then
                            for fillType, supported in pairs(dischargeableUnit.object:getFillUnitSupportedFillTypes(dischargeableUnit.fillUnitIndex)) do
                                if supported and not table.contains(supportedFillTypes, fillType) then
                                    table.insert(supportedFillTypes, fillType)
                                end
                            end
                        end
                    end
                end
            end
        end
    end

	local sort_func = function(a, b)
        a = string.lower(tostring(g_fillTypeManager:getFillTypeByIndex(a).title))
        b = string.lower(tostring(g_fillTypeManager:getFillTypeByIndex(b).title))
        local patt = "^(.-)%s*(%d+)$"
        local _, _, col1, num1 = a:find(patt)
        local _, _, col2, num2 = b:find(patt)
        if (col1 and col2) and col1 == col2 then
            return tonumber(num1) < tonumber(num2)
        end
        return a < b
    end

    if supportedFillTypes and #supportedFillTypes > 0 then
        table.sort(supportedFillTypes, sort_func)
    end

    return supportedFillTypes
end

-- rotLimit is only available on server, so asume there are Y rotateable parts
function AutoDrive.hasVehicleRotatingYComponents(vehicle)
    local ret = true
    if vehicle.isServer then
        ret = false
        if vehicle then
            if #vehicle.componentJoints >= 1 then
                for k, componentJoint in ipairs(vehicle.componentJoints) do
                    if vehicle.componentJoints[k].rotLimit and vehicle.componentJoints[k].rotLimit[2] ~= 0 then
                        ret = true
                    end
                end
            end
        end
    end
    return ret
end

AutoDrive.implementsAllowedForReverseDriving = {
"trailer"
,"trailerlow"
,"trailerSaddled"
,"semitrailerCar"
}

function AutoDrive.isImplementAllowedForReverseDriving(vehicle,implement)
-- return true for implements allowed move reverse
    local ret = false

    if implement ~= nil and implement.spec_attachable ~= nil and implement.spec_attachable.attacherJoint ~= nil and implement.spec_attachable.attacherJoint.jointType ~= nil then
        for i, name in ipairs(AutoDrive.implementsAllowedForReverseDriving) do
            local key = "JOINTTYPE_"..string.upper(name)

            if AttacherJoints[key] ~= nil and AttacherJoints[key] == implement.spec_attachable.attacherJoint.jointType then
                -- Logging.info("[AD] isImplementAllowedForReverseDriving implement allowed %s ", tostring(key))
                return true
            end
        end
    end

    if implement ~= nil and implement.spec_attachable ~= nil
        and AttacherJoints.JOINTTYPE_IMPLEMENT == implement.spec_attachable.attacherJoint.jointType
    then
        local breakforce = implement.spec_attachable:getBrakeForce()
        -- Logging.info("[AD] isImplementAllowedForReverseDriving implement breakforce %s ", tostring(breakforce))
        if breakforce ~= nil and breakforce > 0.07 * 10
            and not (implement ~= nil and implement.getName ~= nil and implement:getName() == "GL 420")     -- Grimme GL 420 needs special handling, as it has breakforce >0.07, but no trailed wheel
        then
            return true
        end
    end

    if implement ~= nil and implement.spec_attachable ~= nil
        and AttacherJoints.JOINTTYPE_SEMITRAILER == implement.spec_attachable.attacherJoint.jointType
    then
        local implementX, implementY, implementZ = getWorldTranslation(implement.components[1].node)
        local _, _, diffZ = AutoDrive.worldToLocal(vehicle, implementX, implementY, implementZ)
        if diffZ < -3 then
            return true
        end
    end
    return ret
end

function AutoDrive.getAllVehicles()
    return (g_currentMission and g_currentMission.vehicleSystem and g_currentMission.vehicleSystem.vehicles) or {}
end

function AutoDrive:autostartHelpers()
    if g_server == nil or g_client == nil or g_dedicatedServer ~= nil then
        -- only in SP mode
        return
    end

    if not AutoDrive.getSetting("autostartHelpers") then
        return
    end

    for _, vehicle in pairs(AutoDrive.getAllVehicles()) do
        if vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.stateModule ~= nil then
            if vehicle.ad.stateModule.activeBeforeSave then
                vehicle.ad.stateModule:getCurrentMode():start(AutoDrive.USER_PLAYER)
            end
        end
    end
end

function AutoDrive.getControlledVehicle()
    return g_localPlayer and g_localPlayer.getCurrentVehicle and g_localPlayer:getCurrentVehicle()
end

function AutoDrive.setEnteredVehicle(vehicle)
    if vehicle and g_currentMission.vehicleSystem and g_currentMission.vehicleSystem.setEnteredVehicle then
        g_currentMission.vehicleSystem:setEnteredVehicle(vehicle)
    end
end

function AutoDrive.getPlayer()
    if g_currentMission and g_currentMission.playerSystem and g_currentMission.playerSystem.getLocalPlayer then
        return g_currentMission.playerSystem:getLocalPlayer()
    end 
    return nil
end

function AutoDrive.getPriceMultiplier()
    return (g_currentMission and g_currentMission.economyManager and g_currentMission.economyManager.getPriceMultiplier 
    and g_currentMission.economyManager:getPriceMultiplier()) or 1
end

function AutoDrive.drawTripod(node, offset)
    if offset == nil then
        offset = {x=0,y=0,z=0}
    end
    local nodeX, nodeY, nodeZ = getWorldTranslation(node)
    local targetX, targetY, targetZ = localToWorld(node, 2, 0, 0)
    ADDrawingManager:addLineTask(nodeX + offset.x, nodeY + offset.y, nodeZ + offset.z, targetX + offset.x, targetY + offset.y, targetZ + offset.z, 1, 1, 0, 0)
    targetX, targetY, targetZ = localToWorld(node, 0, 2, 0)
    ADDrawingManager:addLineTask(nodeX + offset.x, nodeY + offset.y, nodeZ + offset.z, targetX + offset.x, targetY + offset.y, targetZ + offset.z, 1, 0, 1, 0)
    targetX, targetY, targetZ = localToWorld(node, 0, 0, 2)
    ADDrawingManager:addLineTask(nodeX + offset.x, nodeY + offset.y, nodeZ + offset.z, targetX + offset.x, targetY + offset.y, targetZ + offset.z, 1, 0, 0, 1)
end

function AutoDrive.requestToEnterVehicle(vehicle)
    local player = AutoDrive.getPlayer()
    if player then
        player:requestToEnterVehicle(vehicle)
    end
end
