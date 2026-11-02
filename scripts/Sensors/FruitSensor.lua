ADFruitSensor = ADInheritsFrom(ADSensor)

function ADFruitSensor:new(vehicle, sensorParameters)
    local o = ADFruitSensor:create()
    o:init(vehicle, ADSensor.TYPE_FRUIT, sensorParameters)
    o.fruitType = 0
    o.trackFruitAmount = false
    o.fruitAmount = 0

    if sensorParameters.fruitType ~= nil then
        o.fruitType = sensorParameters.fruitType
    end
    return o
end

function ADFruitSensor:setTrackFruitAmount(enabled)
    enabled = enabled == true
    if self.trackFruitAmount ~= enabled then
        self.trackFruitAmount = enabled
        self.fruitAmount = 0
        -- Die erste Abfrage soll sofort mit der neuen Messoption aktualisiert werden.
        self.executionDelay = 0
    end
end

function ADFruitSensor:onUpdate(dt)
    local box = self:getBoxShape()
    local corners = self:getCorners(box)
    self:setTriggerType(0)

    local foundFruit = false
    local fruitTypeIndex
    local fruitAmount
    if self.fruitType == nil or self.fruitType == 0 then
        foundFruit, fruitTypeIndex, fruitAmount = AutoDrive.checkForUnknownFruitInArea(corners, self.trackFruitAmount)
    elseif self.trackFruitAmount then
        fruitTypeIndex = self.fruitType
        fruitAmount = AutoDrive.getFruitValue(fruitTypeIndex, corners[1].x, corners[1].z, corners[2].x, corners[2].z, corners[3].x, corners[3].z)
        foundFruit = fruitAmount > 10
    else
        foundFruit = AutoDrive.checkForFruitTypeInArea(corners, self.fruitType)
    end

    -- Die Fruchtmenge wird nur fuer den Seitenvergleich beim Zuckerrohrernter gespeichert.
    self.fruitAmount = self.trackFruitAmount and (fruitAmount or 0) or 0

    if self.swathDetection == true then
        local fillTypeIndex = DensityMapHeightUtil.getFillTypeAtArea(corners[1].x, corners[1].z, corners[2].x, corners[2].z, corners[3].x, corners[3].z)
        if fillTypeIndex ~= nil and table.contains(AutoDrive.windrowFillTypes, fillTypeIndex) then
            local fillLevel, _, _ = DensityMapHeightUtil.getFillLevelAtArea(fillTypeIndex, corners[1].x, corners[1].z, corners[2].x, corners[2].z, corners[3].x, corners[3].z)
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

function AutoDrive.checkForUnknownFruitInArea(corners, trackFruitAmount)
    local highestFruitAmount = 0
    local highestFruitTypeIndex
    -- Fuer den Seitenvergleich alle Fruchtarten pruefen und die groesste Menge erfassen.
    for _, fruitType in pairs(g_fruitTypeManager:getFruitTypes()) do
        if not (fruitType == g_fruitTypeManager:getFruitTypeByName("MEADOW")) then
            local fruitTypeIndex = fruitType.index
            if trackFruitAmount then
                local fruitAmount = AutoDrive.getFruitValue(fruitTypeIndex, corners[1].x, corners[1].z, corners[2].x, corners[2].z, corners[3].x, corners[3].z)
                if fruitAmount > highestFruitAmount then
                    highestFruitAmount = fruitAmount
                    highestFruitTypeIndex = fruitTypeIndex
                end
            elseif AutoDrive.checkForFruitTypeInArea(corners, fruitTypeIndex) then
                return true, fruitTypeIndex
            end
        end
    end
    if trackFruitAmount then
        return highestFruitAmount > 10, highestFruitTypeIndex, highestFruitAmount
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
