ADFruitSensor = ADInheritsFrom(ADSensor)

function ADFruitSensor:new(vehicle, sensorParameters)
    local o = ADFruitSensor:create()
    o:init(vehicle, ADSensor.TYPE_FRUIT, sensorParameters)
    o.fruitType = 0

    if sensorParameters.fruitType ~= nil then
        o.fruitType = sensorParameters.fruitType
    end

    o.SWATH_TYPES = {
        [FillType.GRASS_WINDROW] = true,
        [FillType.BARLEY_CUT] = true,
        [FillType.CANOLA_CUT] = true,
        [FillType.OAT_CUT] = true,
        [FillType.SOYBEAN_CUT] = true,
        [FillType.WHEAT_CUT] = true,
    }
    return o
end

function ADFruitSensor:onUpdate(dt)
    local box = self:getBoxShape()
    local corners = self:getCorners(box)
    self:setTriggerType(0)

    local foundFruit = false
    if not foundFruit then
        if self.fruitType == nil or self.fruitType == 0 then
            foundFruit, _ = AutoDrive.checkForUnknownFruitInArea(corners)
        else
            foundFruit = AutoDrive.checkForFruitTypeInArea(corners, self.fruitType)
        end
    end

    if AutoDrive.getSetting("detectSwath") then
        local fillType = DensityMapHeightUtil.getFillTypeAtArea(corners[1].x, corners[1].z, corners[2].x, corners[2].z, corners[3].x, corners[3].z)
        if fillType ~= nil and self.SWATH_TYPES[fillType] then
            local fillLevel, _, _ = DensityMapHeightUtil.getFillLevelAtArea(fillType, corners[1].x, corners[1].z, corners[2].x, corners[2].z, corners[3].x, corners[3].z)
            if (fillLevel and fillLevel > 0.1) then
                local value = DensityMapHeightUtil.getValueAtArea(corners[1].x, corners[1].z, corners[2].x, corners[2].z, corners[3].x, corners[3].z, true)
                if (value and value > 0.1) then
                    self:setTriggerType(ADSensor.TYPE_SWATH)
                    foundFruit = true
                end
            end
        end
    end

    self:setTriggered(foundFruit)

    self:onDrawDebug(box)
end

function AutoDrive.checkForUnknownFruitInArea(corners)
    for _, fruitType in pairs(g_fruitTypeManager:getFruitTypes()) do
        if not (fruitType == g_fruitTypeManager:getFruitTypeByName("MEADOW")) then
            local fruitTypeIndex = fruitType.index
            if AutoDrive.checkForFruitTypeInArea(corners, fruitTypeIndex) then
                return true, fruitTypeIndex
            end
        end
    end
    return false
end

function AutoDrive.getFruitValue(fruitTypeIndex, corner1X, corner1Z, corner2X, corner2Z, corner3X, corner3Z)
    local fruitToIgnore = { FruitType.GRASS, FruitType.MEADOW }
    if table.contains(fruitToIgnore, fruitTypeIndex) then
        return 0
    end

    local fruitValue, _, _, growthState = FSDensityMapUtil.getFruitArea(fruitTypeIndex, corner1X, corner1Z, corner2X, corner2Z, corner3X, corner3Z, true, true)

    -- we need to check the growth state for these fruit types
    local fruit = g_fruitTypeManager:getFruitTypeByIndex(fruitTypeIndex)
    if fruit ~= nil and growthState ~= nil and (fruit:getIsCut(growthState) or fruit:getIsWithered(growthState)) then
        fruitValue = 0
    end
    return fruitValue
end

function AutoDrive.checkForFruitTypeInArea(corners, fruitTypeIndex)
    local fruitValue = AutoDrive.getFruitValue(fruitTypeIndex, corners[1].x, corners[1].z, corners[2].x, corners[2].z, corners[3].x, corners[3].z)
    return (fruitValue > 10)
end
