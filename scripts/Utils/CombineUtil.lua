
-- harvester types
function AutoDrive.setCombineType(vehicle)
    if vehicle == nil then
        return false
    end
    if vehicle.ad == nil then
        vehicle.ad = {}
    end
    local rootVehicle = vehicle:getRootVehicle()
    local rootVehicleName = rootVehicle and rootVehicle:getName() or "no rootVehicle!!!"
    vehicle.ad.hasCombine = false -- vehicle is any combine or has attached one
    vehicle.ad.isAutoAimingChopper = false -- chopper with flexible self targeting pipe
    vehicle.ad.isFixedPipeChopper = false -- chopper with fixed pipe
    vehicle.ad.isChopper = false -- chopper, without bunker
    vehicle.ad.isHarvester = false -- harvester with bunker

    if (vehicle.spec_combine ~= nil and vehicle.spec_pipe ~= nil and vehicle.spec_combine.isBufferCombine == true and vehicle.spec_pipe.numAutoAimingStates > 0) then
        vehicle.ad.isAutoAimingChopper = true
    end
    if (vehicle.spec_combine ~= nil and vehicle.spec_pipe ~= nil and vehicle.spec_combine.isBufferCombine == true and vehicle.spec_pipe.numAutoAimingStates == 0) then
        vehicle.ad.isFixedPipeChopper = true
    end
    if (vehicle.spec_combine ~= nil and vehicle.spec_pipe ~= nil and vehicle.spec_combine.isBufferCombine == true) then
        vehicle.ad.isChopper = true
    end
    if (vehicle.spec_combine ~= nil and vehicle.spec_pipe ~= nil and not vehicle.spec_combine.isBufferCombine) then
        vehicle.ad.isHarvester = true
    end

    rootVehicle.ad.hasCombine = vehicle.ad.isAutoAimingChopper or vehicle.ad.isFixedPipeChopper or vehicle.ad.isChopper or vehicle.ad.isHarvester
    return rootVehicle.ad.hasCombine
end


function AutoDrive.getDischargeNode(combine)
    local dischargeNode = nil
    if combine.spec_dischargeable ~= nil then
        for _, dischargeNodeIter in pairs(combine.spec_dischargeable.dischargeNodes) do
            dischargeNode = dischargeNodeIter
        end
        if combine.getPipeDischargeNodeIndex ~= nil then
            dischargeNode = combine.spec_dischargeable.dischargeNodes[combine:getPipeDischargeNodeIndex()]
        end
        return dischargeNode.node
    else
        return nil
    end
end

function AutoDrive.validateCachedPipeData(combine)
    -- check if the harvester has been reconfigured, invalidate cache
    if combine.ad ~= nil and combine.ad.pipeRoot ~= nil and not entityExists(combine.ad.pipeRoot) then
        combine.ad.pipeRoot = nil
        combine.ad.storedPipeSide = nil
        combine.ad.storedPipeLength = nil
    end
end

function AutoDrive.getPipeRoot(combine)
    if combine.ad ~= nil and combine.ad.pipeRoot ~= nil then
        return combine.ad.pipeRoot
    end
    local dischargeNode = AutoDrive.getDischargeNode(combine)
    local pipeRoot = nil

    if dischargeNode then
        for _, component in ipairs(combine.components) do

            local node = dischargeNode
            local count = 0
            repeat
                node = getParent(node)
                count = count + 1
            until ((node == component.node) or (node == 0) or (node == nil) or count >= 100)

            if node and node ~= 0 then
                -- found
                pipeRoot = node
                break
            end
        end
    end
    if pipeRoot == nil or pipeRoot == 0 then
        -- fallback
        pipeRoot = combine.components[1].node
    end

    if combine.ad ~= nil then
        combine.ad.pipeRoot = pipeRoot
    end
    return pipeRoot
end

-- ret: -1 right, 1 left, 0 behind
-- not for sugarcane harvesters, choppers!!!
function AutoDrive.getPipeSide(combine)
    if combine.ad ~= nil and combine.ad.storedPipeSide ~= nil then
        return combine.ad.storedPipeSide
    end
    local combineNode = AutoDrive.getPipeRoot(combine)
    local dischargeNode = AutoDrive.getDischargeNode(combine)
    local dischargeX, dichargeY, dischargeZ = getWorldTranslation(dischargeNode)
    local diffX, _, _ = AutoDrive.worldToLocal(combine, dischargeX, dichargeY, dischargeZ, combineNode)
    if combine.ad ~= nil and (combine.ad.isFixedPipeChopper or combine.ad.isHarvester) and AutoDrive.isPipeOut(combine) then
        combine.ad.storedPipeSide = AutoDrive.sign(diffX)
    end
    return AutoDrive.sign(diffX)
end

function AutoDrive.getPipeLength(combine)
    if combine.ad ~= nil and combine.ad.storedPipeLength ~= nil then
        return combine.ad.storedPipeLength
    end

    local pipeRootX, _ , pipeRootZ = getWorldTranslation(AutoDrive.getPipeRoot(combine))
    local dischargeX, dischargeY, dischargeZ = getWorldTranslation(AutoDrive.getDischargeNode(combine))
    local length = MathUtil.vector3Length(pipeRootX - dischargeX,
                                        0, 
                                        pipeRootZ - dischargeZ)
    --AutoDrive.debugPrint(combine, AutoDrive.DC_COMBINEINFO, "AutoDrive.getPipeLength - " .. length)
    if (combine.ad.isFixedPipeChopper or combine.ad.isHarvester) and AutoDrive.isPipeOut(combine) then
        local combineNode = AutoDrive.getPipeRoot(combine)
        local dischargeX, dichargeY, dischargeZ = getWorldTranslation(AutoDrive.getDischargeNode(combine))
        local diffX, _, _ = AutoDrive.worldToLocal(combine, dischargeX, dichargeY, dischargeZ, combineNode)
        length = math.abs(diffX)

        -- Store pipe length for 'normal' harvesters
        if combine.ad ~= nil then
            combine.ad.storedPipeLength = length
        end
    end

    return length
end

function AutoDrive.isPipeOut(combine)

    local function isPipeOut(combine)
        if (combine.spec_combine ~= nil and combine.spec_pipe ~= nil and combine.spec_dischargeable ~= nil) then
            if combine.getIsDischargeNodeActive ~= nil and combine.getPipeDischargeNodeIndex ~= nil then
                local pipeDischargeNodeIndex = combine:getPipeDischargeNodeIndex()
                if pipeDischargeNodeIndex and pipeDischargeNodeIndex > 0 then
                    local spec_dischargeable = combine.spec_dischargeable
                    if spec_dischargeable and spec_dischargeable.dischargeNodes and #spec_dischargeable.dischargeNodes > 0 then
                        local dischargeNode = spec_dischargeable.dischargeNodes[pipeDischargeNodeIndex]
                        if dischargeNode and combine:getIsDischargeNodeActive(dischargeNode) then
                            return true
                        end
                    end
                end
            end
        end
        return false
    end

    if combine.spec_combine ~= nil then
        return isPipeOut(combine)
    else        
        for _, implement in pairs(AutoDrive.getAllImplements(combine)) do
            if implement and isPipeOut(implement) then
                return true
            end
        end
    end
    return false
end

function AutoDrive.getAIMarkerWidth(object)
    local retWidth = 0
    if object ~= nil and object.getAIMarkers ~= nil then
        local aiLeftMarker, aiRightMarker = object:getAIMarkers()
        if aiLeftMarker ~= nil and aiRightMarker ~= nil then
            local left, _, _ = localToLocal(aiLeftMarker, object.rootNode, 0, 0, 0)
            local right, _, _ = localToLocal(aiRightMarker, object.rootNode, 0, 0, 0)
            if left < right then
                left, right = right, left
            end
            retWidth = left - right
        end
    end
    return retWidth
end

function AutoDrive.getAISizeMarkerWidth(object)
    local retWidth = 0
    if object ~= nil and object.getAISizeMarkers ~= nil then
        local aiLeftMarker, aiRightMarker = object:getAISizeMarkers()
        if aiLeftMarker ~= nil and aiRightMarker ~= nil then
            local left, _, _ = localToLocal(aiLeftMarker, object.rootNode, 0, 0, 0)
            local right, _, _ = localToLocal(aiRightMarker, object.rootNode, 0, 0, 0)
            if left < right then
                left, right = right, left
            end
            retWidth = left - right
        end
    end
    return retWidth
end

function AutoDrive.getFrontToolWidth(vehicle, forced)
    if vehicle.ad ~= nil and vehicle.ad.frontToolWidth ~= nil and not (forced == true) then
        return vehicle.ad.frontToolWidth
    end
    local widthOfFrontTool = 0

    -- check for AIMarkers
    local implements = AutoDrive.getAllImplements(vehicle, false)
    for _, implement in pairs(implements) do
        --Check if tool is in front of vehicle
        local toolX, toolY, toolZ = getWorldTranslation(implement.components[1].node)
        local _, _, offsetZ =  AutoDrive.worldToLocal(vehicle, toolX, toolY, toolZ)
        if offsetZ > 0 then
            widthOfFrontTool = math.max(widthOfFrontTool, AutoDrive.getAIMarkerWidth(implement))
        end
    end

    if widthOfFrontTool == 0 then
        -- check for AISizeMarkers
        for _, implement in pairs(implements) do
            --Check if tool is in front of vehicle
            local toolX, toolY, toolZ = getWorldTranslation(implement.components[1].node)
            local _, _, offsetZ =  AutoDrive.worldToLocal(vehicle, toolX, toolY, toolZ)
            if offsetZ > 0 then
                widthOfFrontTool = math.max(widthOfFrontTool, AutoDrive.getAISizeMarkerWidth(implement))
            end
        end
    end

    if widthOfFrontTool == 0 then
        -- if AIMarkers not available or returned 0, get tool size as defined in vehicle XML - the worst case, see rsmDS900.xml
        for _, implement in pairs(implements) do
            if implement.size.width ~= nil then
                --Check if tool is in front of vehicle
                local toolX, toolY, toolZ = getWorldTranslation(implement.components[1].node)
                local _, _, offsetZ =  AutoDrive.worldToLocal(vehicle, toolX, toolY, toolZ)
                if offsetZ > 0 then
                    widthOfFrontTool = math.abs(implement.size.width)
                end
            end
        end
    end

    if vehicle.ad ~= nil then
        vehicle.ad.frontToolWidth = widthOfFrontTool
    end

    return widthOfFrontTool
end

function AutoDrive.getFrontToolLength(vehicle)
    if vehicle.ad ~= nil and vehicle.ad.frontToolLength ~= nil then
        return vehicle.ad.frontToolLength
    end
    local lengthOfFrontTool = 0

    local implements = AutoDrive.getAllImplements(vehicle, false)
    for _, implement in pairs(implements) do
        if implement.size.width ~= nil then
            --Check if tool is in front of vehicle
            local toolX, toolY, toolZ = getWorldTranslation(implement.components[1].node)
            local _, _, offsetZ =  AutoDrive.worldToLocal(vehicle, toolX, toolY, toolZ)
            if offsetZ > 0 then
                lengthOfFrontTool = math.abs(implement.size.length)
            end
        end
    end

    if vehicle.ad ~= nil then
        vehicle.ad.frontToolLength = lengthOfFrontTool
    end

    return lengthOfFrontTool
end

function AutoDrive.getLengthOfFieldInFront(vehicle, onlyWithFruit, maxRange, stepLength)
    local maxSearchRange = maxRange or 50
    local acceptOnlyWithFruit = onlyWithFruit or false
    local stepLength = stepLength or 5
    
    local length = 10
    local foundField = true
    local fruitType = nil
    while foundField do
        local worldPosX, _, worldPosZ = AutoDrive.localToWorld(vehicle, 0, 0, length + stepLength)
        foundField = AutoDrive.checkIsOnField(worldPosX, 0, worldPosZ)

        if acceptOnlyWithFruit then
            local foundFruit = false
            local corners = AutoDrive.getCornersForAreaRelativeToVehicle(vehicle, 0, length, 3, stepLength)
            if fruitType == nil then
                foundFruit, fruitType = AutoDrive.checkForUnknownFruitInArea(corners)
            else
                foundFruit = AutoDrive.checkForFruitTypeInArea(corners, fruitType)
            end
            foundField = foundField and foundFruit        
        end

        length = length + stepLength
        if math.abs(length) >= maxSearchRange then
            foundField = false
        end
    end

    return length
end

-- 3
-- |
-- |
-- 1 ---- 2
--
--
--    v
function AutoDrive.getCornersForAreaRelativeToVehicle(vehicle, xOffset, zOffset, width, length)
    local corners = {}
    local worldPosX, _, worldPosZ = AutoDrive.localToWorld(vehicle, xOffset - width/2, 0, zOffset)
    corners[1] = { x = worldPosX, z = worldPosZ}
    worldPosX, _, worldPosZ = AutoDrive.localToWorld(vehicle, xOffset + width/2, 0, zOffset)
    corners[2] = { x = worldPosX, z = worldPosZ}
    worldPosX, _, worldPosZ = AutoDrive.localToWorld(vehicle, xOffset - width/2, 0, zOffset + length)
    corners[3] = { x = worldPosX, z = worldPosZ}

    return corners
end
