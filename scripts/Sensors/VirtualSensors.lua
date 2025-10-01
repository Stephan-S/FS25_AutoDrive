ADSensor = {}
ADSensor_mt = {__index = ADSensor}
ADSensor.debug = false

ADSensor.TYPE_COLLISION = 1
ADSensor.TYPE_FRUIT = 2
ADSensor.TYPE_TRIGGER = 3
ADSensor.TYPE_FIELDBORDER = 4
ADSensor.TYPE_SWATH = 5

ADSensor.POS_FRONT = 1
ADSensor.POS_REAR = 2
ADSensor.POS_LEFT = 3
ADSensor.POS_RIGHT = 4
ADSensor.POS_FRONT_LEFT = 5
ADSensor.POS_FRONT_RIGHT = 6
ADSensor.POS_REAR_LEFT = 7
ADSensor.POS_REAR_RIGHT = 8
ADSensor.POS_FIXED = 9
ADSensor.POS_CENTER = 10

ADSensor.WIDTH_FACTOR = 0.7

ADSensor.EXECUTION_DELAY = 10

--
--          <x>
--       ^  o-o
--       z  |||
--       v  O-O
--

function ADSensor:handleSensors(vehicle, dt)
    if vehicle.ad.sensors == nil then
        ADSensor:addSensorsToVehicle(vehicle)
    end

    for _, sensor in pairs(vehicle.ad.sensors) do
        sensor:updateSensor(dt)
    end
end

function ADSensor:addSensorsToVehicle(vehicle)
    vehicle.ad.sensors = {}
    local sensorParameters = {}
    sensorParameters.dynamicLength = true
    sensorParameters.minDynamicLength = 1
    sensorParameters.position = ADSensor.POS_FRONT
    sensorParameters.width = vehicle.size.width * 0.75
    local frontSensorDynamicShort = ADCollSensorSplit:new(vehicle, sensorParameters)
    --frontSensorDynamicShort.drawDebug = true --test
    --frontSensorDynamicShort.enabled = true --test
    vehicle.ad.sensors["frontSensorDynamicShort"] = frontSensorDynamicShort

    sensorParameters.minDynamicLength = 2
    local frontSensorDynamicLong = ADCollSensor:new(vehicle, sensorParameters)
    vehicle.ad.sensors["frontSensorDynamicLong"] = frontSensorDynamicLong

    sensorParameters.dynamicLength = false
    sensorParameters.width = vehicle.size.width * 0.65
    sensorParameters.length = 0.1
    local frontSensor = ADCollSensor:new(vehicle, sensorParameters)
    vehicle.ad.sensors["frontSensor"] = frontSensor

    sensorParameters.dynamicLength = false
    sensorParameters.width = vehicle.size.width * 1.3
    sensorParameters.length = vehicle.size.length * 2
    local frontSensorLong = ADCollSensor:new(vehicle, sensorParameters)
    vehicle.ad.sensors["frontSensorLong"] = frontSensorLong

    sensorParameters.dynamicLength = false
    sensorParameters.width = vehicle.size.width * 1.3
    sensorParameters.length = vehicle.size.length * 2
    local frontSensorLongFruit = ADFruitSensor:new(vehicle, sensorParameters)
    vehicle.ad.sensors["frontSensorLongFruit"] = frontSensorLongFruit

    sensorParameters.dynamicLength = false
    sensorParameters.length = vehicle.size.length
    sensorParameters.width = vehicle.size.width * 2
    local frontSensorFruit = ADFruitSensor:new(vehicle, sensorParameters)
    vehicle.ad.sensors["frontSensorFruit"] = frontSensorFruit

    sensorParameters.dynamicLength = false
    sensorParameters.length = 2
    sensorParameters.width = vehicle.size.width * 0.5
    local frontSensorField = ADFieldSensor:new(vehicle, sensorParameters)
    vehicle.ad.sensors["frontSensorField"] = frontSensorField

    sensorParameters = {}
    sensorParameters.dynamicLength = true
    sensorParameters.minDynamicLength = 0.3
    sensorParameters.width = vehicle.size.width * 0.75
    sensorParameters.position = ADSensor.POS_REAR
    local rearSensor = ADCollSensor:new(vehicle, sensorParameters)
    local rearSensorFruit = ADFruitSensor:new(vehicle, sensorParameters)
    vehicle.ad.sensors["rearSensor"] = rearSensor
    vehicle.ad.sensors["rearSensorFruit"] = rearSensorFruit

    sensorParameters = {}
    sensorParameters.position = ADSensor.POS_LEFT
    sensorParameters.dynamicLength = false
    sensorParameters.dynamicRotation = false
    sensorParameters.width = 6.5
    sensorParameters.length = vehicle.size.length * 0.8
    local leftSensor = ADCollSensor:new(vehicle, sensorParameters)
    local leftSensorFruit = ADFruitSensor:new(vehicle, sensorParameters)
    sensorParameters.width = 6.5
    local leftSensorField = ADFieldSensor:new(vehicle, sensorParameters)
    vehicle.ad.sensors["leftSensor"] = leftSensor
    vehicle.ad.sensors["leftSensorFruit"] = leftSensorFruit
    vehicle.ad.sensors["leftSensorField"] = leftSensorField

    sensorParameters.position = ADSensor.POS_RIGHT
    sensorParameters.dynamicLength = false
    sensorParameters.dynamicRotation = false
    sensorParameters.width = 6.5
    sensorParameters.length = vehicle.size.length * 0.8
    local rightSensor = ADCollSensor:new(vehicle, sensorParameters)
    local rightSensorFruit = ADFruitSensor:new(vehicle, sensorParameters)
    sensorParameters.width = 6.5
    local rightSensorField = ADFieldSensor:new(vehicle, sensorParameters)
    vehicle.ad.sensors["rightSensor"] = rightSensor
    vehicle.ad.sensors["rightSensorFruit"] = rightSensorFruit
    vehicle.ad.sensors["rightSensorField"] = rightSensorField

    sensorParameters.position = ADSensor.POS_FRONT_LEFT
    sensorParameters.dynamicLength = false
    sensorParameters.dynamicRotation = false
    sensorParameters.width = 4
    sensorParameters.length = vehicle.size.length * 1
    local leftFrontSensor = ADCollSensor:new(vehicle, sensorParameters)
    local leftFrontSensorFruit = ADFruitSensor:new(vehicle, sensorParameters)
    vehicle.ad.sensors["leftFrontSensor"] = leftFrontSensor
    vehicle.ad.sensors["leftFrontSensorFruit"] = leftFrontSensorFruit

    sensorParameters.position = ADSensor.POS_FRONT_RIGHT
    sensorParameters.dynamicLength = false
    sensorParameters.dynamicRotation = false
    sensorParameters.width = 4
    sensorParameters.length = vehicle.size.length * 1
    local rightFrontSensor = ADCollSensor:new(vehicle, sensorParameters)
    local rightFrontSensorFruit = ADFruitSensor:new(vehicle, sensorParameters)
    vehicle.ad.sensors["rightFrontSensor"] = rightFrontSensor
    vehicle.ad.sensors["rightFrontSensorFruit"] = rightFrontSensorFruit

    sensorParameters.position = ADSensor.POS_CENTER
    sensorParameters.dynamicLength = false
    sensorParameters.dynamicRotation = false
    sensorParameters.width = vehicle.size.width * 0.95
    sensorParameters.length = vehicle.size.length * 1.4
    local centerSensorFruit = ADFruitSensor:new(vehicle, sensorParameters)
    vehicle.ad.sensors["centerSensorFruit"] = centerSensorFruit
end

function ADSensor:init(vehicle, sensorType, sensorParameters)
    --o = {}
    --setmetatable(o, self)
    --self.__index = self
    self.vehicle = vehicle
    self.sensorType = sensorType
    self.sensorParameters = sensorParameters
    self.enabled = false
    self.triggered = false
    self.triggerType = 0
    self.initialized = false
    self.drawDebug = false
    self.executionDelay = 0

    self:loadBaseParameters()
    self:loadDynamicParameters(sensorParameters)
end

function ADSensor:loadBaseParameters()
    local vehicle = self.vehicle
    if vehicle ~= nil and vehicle.size.length ~= nil and vehicle.size.width ~= nil then
        self.dynamicLength = true
        self.minDynamicLength = 2
        self.dynamicRotation = true
        self.length = vehicle.size.length
        self.width = vehicle.size.width * ADSensor.WIDTH_FACTOR
        -- self.collisionMask = AIVehicleUtil.COLLISION_MASK
        self.position = ADSensor.POS_FRONT
        self.location = self:getLocationByPosition()
        self.initialized = true
        self.frontFactor = 1
        self.sideFactor = 1
    end
end

function ADSensor:loadDynamicParameters(sensorParameters)
    if sensorParameters == nil then
        return
    end

    if sensorParameters.dynamicLength ~= nil then
        self.dynamicLength = sensorParameters.dynamicLength == true
    end
    if sensorParameters.minDynamicLength ~= nil then
        self.minDynamicLength = sensorParameters.minDynamicLength
    end
    if sensorParameters.dynamicRotation ~= nil then
        self.dynamicRotation = sensorParameters.dynamicRotation == true
    end
    if sensorParameters.length ~= nil then
        self.length = sensorParameters.length
    end
    if sensorParameters.width ~= nil then
        self.width = sensorParameters.width
    end
    if sensorParameters.collisionMask ~= nil then
        self.collisionMask = sensorParameters.collisionMask
    end
    if sensorParameters.position ~= nil then
        if sensorParameters.position >= ADSensor.POS_FRONT and sensorParameters.position <= ADSensor.POS_CENTER then
            self.position = sensorParameters.position
        end
    end
    if sensorParameters.location ~= nil then
        if self.position == ADSensor.POS_FIXED then
            self.location = sensorParameters.location
        end
    end
    self.location = self:getLocationByPosition()
end

function ADSensor:getLocationByPosition()
    local vehicle = self.vehicle
    local location = {x = 0, z = 0}

    if self.position == ADSensor.POS_FRONT then
        --location.z = vehicle.size.length / 2 + lengthOffset
        --local lengthOffset = 1
        --if self.dynamicLength then
        --local frontToolLength = 0 --AutoDrive.getFrontToolLength(self.vehicle)
        --lengthOffset = frontToolLength / 2
        --end

        if vehicle.ad and vehicle.ad.adDimensions and vehicle.ad.adDimensions.maxLengthFront and vehicle.ad.adDimensions.maxLengthFront > 0 then
            location = {x = 0, z = vehicle.ad.adDimensions.maxLengthFront}
        else
            local front = self:getRotatedFront()
            if front == nil then
                location = {x = 0, z = vehicle.size.length / 2}
                location.z = location.z + 0.5
            else
                location = front
                location.z = location.z + 0.5
            end
        end
    elseif self.position == ADSensor.POS_REAR then
        location.z = - vehicle.size.length / 2
        self.frontFactor = -1
    elseif self.position == ADSensor.POS_RIGHT then
        location.x = -vehicle.size.width / 2 - 1 - self.width / 2
        location.z = -vehicle.size.length / 2 - 2
        self.sideFactor = -1
    elseif self.position == ADSensor.POS_LEFT then
        location.x = vehicle.size.width / 2 + 1 + self.width / 2
        location.z = -vehicle.size.length / 2 - 2
    elseif self.position == ADSensor.POS_FRONT_LEFT then
        local frontToolWidth = AutoDrive.getFrontToolWidth(vehicle)
        if frontToolWidth ~= nil and frontToolWidth > 0 then
            location.x = frontToolWidth / 2 + 0.5 + self.width / 2
        else
            location.x = vehicle.size.width + 1.5 + self.width / 2
        end
        location.z = vehicle.size.length * 0.3 - 2
    elseif self.position == ADSensor.POS_FRONT_RIGHT then
        local frontToolWidth = AutoDrive.getFrontToolWidth(vehicle)
        if frontToolWidth ~= nil and frontToolWidth > 0 then
            location.x = -frontToolWidth / 2 - 0.5 - self.width / 2
        else
            location.x = -vehicle.size.width - 1.5 - self.width / 2
        end
        location.z = vehicle.size.length * 0.3 - 2
    elseif self.position == ADSensor.POS_FIXED and self.location ~= nil then
        return self.location
    elseif self.position == ADSensor.POS_CENTER then
        location.z = -self.length / 2
    end

    return location
end

function ADSensor:getBoxShape()
    local vehicle = self.vehicle
    self.location = self:getLocationByPosition()
    local lookAheadDistance = self.length
    if self.dynamicLength then
        lookAheadDistance = math.clamp(vehicle.lastSpeedReal * 3600 * 15.5 / 40, self.minDynamicLength, 50)
    end

    local vecZ = {x = 0, z = 1}
    if self.dynamicRotation then
        vecZ.x, vecZ.z = math.sin(vehicle.rotatedTime), math.cos(vehicle.rotatedTime)
    end
    local vecX = {x = vecZ.z, z = -vecZ.x}

    if self.frontFactor == -1 then
        -- ADSensor.POS_REAR
        vecZ = {x = vecZ.x, z = -vecZ.z}
    end

    local boxYPos = 2
    local boxHeight = 0.75

    if self.position == ADSensor.POS_FRONT_LEFT or self.position == ADSensor.POS_FRONT_RIGHT then
        boxYPos = 2.25
    end
    if (self.position == ADSensor.POS_FRONT or self.position == ADSensor.POS_REAR) and self.dynamicLength == true then
        boxYPos = AutoDrive.getSetting("collisionHeigth", self.vehicle) or 2
    end

    local box = {}
    box.offset = {}
    box.size = {}
    box.center = {}
    box.size[1] = self.width * 0.5
    box.size[2] = boxHeight * 0.5
    box.size[3] = lookAheadDistance * 0.5
    box.offset[1] = self.location.x
    box.offset[2] = boxYPos
    box.offset[3] = self.location.z
    box.center[1] = box.offset[1] + vecZ.x * box.size[3]
    box.center[2] = box.offset[2] + box.size[2]
    box.center[3] = box.offset[3] + vecZ.z * box.size[3]

    box.topLeft = {}
    box.topLeft[1] = box.center[1] - vecX.x * box.size[1] + vecZ.x * box.size[3]
    box.topLeft[2] = box.center[2]
    box.topLeft[3] = box.center[3] - vecX.z * box.size[1] + vecZ.z * box.size[3]

    box.topRight = {}
    box.topRight[1] = box.center[1] + vecX.x * box.size[1] + vecZ.x * box.size[3]
    box.topRight[2] = box.center[2]
    box.topRight[3] = box.center[3] + vecX.z * box.size[1] + vecZ.z * box.size[3]

    box.downRight = {}
    box.downRight[1] = box.center[1] + vecX.x * box.size[1] - vecZ.x * box.size[3]
    box.downRight[2] = box.center[2]
    box.downRight[3] = box.center[3] + vecX.z * box.size[1] - vecZ.z * box.size[3]

    box.downLeft = {}
    box.downLeft[1] = box.center[1] - vecX.x * box.size[1] - vecZ.x * box.size[3]
    box.downLeft[2] = box.center[2]
    box.downLeft[3] = box.center[3] - vecX.z * box.size[1] - vecZ.z * box.size[3]

    if self.sideFactor == -1 then
        vecX = {x = -vecX.x, z = -vecX.z}
    end

    box.dirX, box.dirY, box.dirZ =  AutoDrive.localDirectionToWorld(vehicle, 0, 0, 1)
    box.zx, box.zy, box.zz =  AutoDrive.localDirectionToWorld(vehicle, vecZ.x, 0, vecZ.z)
    box.ry = math.atan2(box.zx, box.zz)
    local angleOffset = 4
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    if not AutoDrive.checkIsOnField(x, y, z) and self.vehicle.ad.stateModule ~= nil and self.vehicle.ad.stateModule:isActive() then
        local heightDiff = self.vehicle.ad.drivePathModule:getApproachingHeightDiff()
        if heightDiff < 1.5 and heightDiff > -1 then
            angleOffset = 0
        end
    end
    box.rx = -MathUtil.getYRotationFromDirection(box.dirY, 1) * self.frontFactor - math.rad(angleOffset)
    box.x, box.y, box.z = AutoDrive.localToWorld(vehicle, box.center[1], box.center[2], box.center[3])

    box.topLeft.x, box.topLeft.y, box.topLeft.z = AutoDrive.localToWorld(vehicle, box.topLeft[1], box.topLeft[2], box.topLeft[3])
    box.topRight.x, box.topRight.y, box.topRight.z = AutoDrive.localToWorld(vehicle, box.topRight[1], box.topRight[2], box.topRight[3])
    box.downRight.x, box.downRight.y, box.downRight.z = AutoDrive.localToWorld(vehicle, box.downRight[1], box.downRight[2], box.downRight[3])
    box.downLeft.x, box.downLeft.y, box.downLeft.z = AutoDrive.localToWorld(vehicle, box.downLeft[1], box.downLeft[2], box.downLeft[3])

    return box
end

function ADSensor:getCorners(box)
    --local box = box
    if box == nil then
        box = self:getBoxShape()
    end

    local corners = {}
    corners[1] = {x = box.downLeft.x, z = box.downLeft.z}
    corners[2] = {x = box.topLeft.x, z = box.topLeft.z}
    corners[3] = {x = box.downRight.x, z = box.downRight.z}
    corners[4] = {x = box.topRight.x, z = box.topRight.z}

    return corners
end

function ADSensor:isElementBlockingVehicle(nodeId)
    -- we want to ignore some elements and keep driving instead.
    if nodeId == nil then
        return false  -- no an element
    end
    if g_currentMission:getNodeObject(nodeId) ~= nil then
        return true  -- not handled here
    end

    local isShapeClass = getHasClassId(nodeId, ClassIds.SHAPE)
    local isRigidBodyTypeDynamic = getRigidBodyType(nodeId) == RigidBodyType.DYNAMIC
    local isDynamicCollision = CollisionFlag.getHasMaskFlagSet(nodeId, CollisionFlag.DYNAMIC_OBJECT)

    -- traffic signs return true for all three checks above, this might require tweaking
    if isShapeClass and isRigidBodyTypeDynamic and isDynamicCollision then
        return false  -- probably a traffic sign
    end

    -- unknown element
    return true
end

function ADSensor:updateSensor(dt)
    --Logging.info("updateSensor called")
    if self:isEnabled() then
        self:onUpdate(dt)
    else
        self:setTriggered(false)
    end
end

function ADSensor:onUpdate(dt)
    Logging.warning("[AutoDrive] ADSensor:onUpdate() called - Please override this in instance class")
end


--[[
red box outline
green TYPE_FIELDBORDER
blue TYPE_FRUIT
]]
function ADSensor:onDrawDebug(box)
    if self.drawDebug or AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
        local red = 1
        local green = 0
        local blue = 0
        local isTriggered = self:isTriggered()
        if isTriggered then
            if self.sensorType == ADSensor.TYPE_FRUIT then
                -- red = 1
                -- green = 1
                blue = 1
            end
            if self.sensorType == ADSensor.TYPE_FIELDBORDER then
                -- red = 1
                green = 1
                -- blue = 1
            end
        end

        if self.sensorType == ADSensor.TYPE_FRUIT or self.sensorType == ADSensor.TYPE_FIELDBORDER then
            local corners = self:getCorners(box)
            corners[1].y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, corners[1].x, 1, corners[1].z) + 0.5 -- bottom right
            corners[2].y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, corners[2].x, 1, corners[2].z) + 0.5 -- top right
            corners[3].y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, corners[3].x, 1, corners[3].z) + 0.5 -- bottom left
            corners[4].y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, corners[4].x, 1, corners[4].z) + 0.5 -- top left

            local AutoDriveDM = ADDrawingManager
            AutoDriveDM:addLineTask(corners[2].x, corners[2].y, corners[2].z, corners[4].x, corners[4].y, corners[4].z, 1, red, green, blue) -- top
            AutoDriveDM:addLineTask(corners[3].x, corners[3].y, corners[3].z, corners[4].x, corners[4].y, corners[4].z, 1, red, green, blue) -- left
            AutoDriveDM:addLineTask(corners[1].x, corners[1].y, corners[1].z, corners[2].x, corners[2].y, corners[2].z, 1, red, green, blue) -- right
            AutoDriveDM:addLineTask(corners[1].x, corners[1].y, corners[1].z, corners[3].x, corners[3].y, corners[3].z, 1, red, green, blue) -- bottom
            if isTriggered and self.sensorType == ADSensor.TYPE_FRUIT then
                if self.triggerType == ADSensor.TYPE_SWATH then
                    AutoDriveDM:addLineTask(corners[1].x, corners[1].y, corners[1].z, corners[4].x, corners[4].y, corners[4].z, 1, red, green, blue)
                end
                AutoDriveDM:addLineTask(corners[3].x, corners[3].y, corners[3].z, corners[2].x, corners[2].y, corners[2].z, 1, 0, green, blue)
            end
            if isTriggered and self.sensorType == ADSensor.TYPE_FIELDBORDER then
                AutoDriveDM:addLineTask(corners[1].x, corners[1].y, corners[1].z, corners[4].x, corners[4].y, corners[4].z, 1, 0, green, blue)
            end
        else
            if isTriggered then
                DebugUtil.drawOverlapBox(box.x, box.y, box.z, box.rx, box.ry, 0, box.size[1], box.size[2], box.size[3], 1, 0, 0)
            else
                DebugUtil.drawOverlapBox(box.x, box.y, box.z, box.rx, box.ry, 0, box.size[1], box.size[2], box.size[3], 1, 1, 1)
            end
        end
    end
end

function ADSensor:pollInfo(forced, widthFactor, lengthFactor)
    self.executionDelay = self.executionDelay -1
    if self.executionDelay <= 0 or forced or AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
        local storedWidth = self.width
        if widthFactor ~= nil then
            self.width = self.width * widthFactor
        end
        local storedLength = self.length
        if lengthFactor ~= nil then
            self.length = self.length * lengthFactor
        end
        local wasEnabled = self.enabled
        self:setEnabled(true)
        if not wasEnabled then
            self:setEnabled(false)
        end
        self.lastTriggered = self:isTriggered()
        self.executionDelay = ADSensor.EXECUTION_DELAY
        self.width = storedWidth
        self.length = storedLength
    end

    return self.lastTriggered
end

function ADSensor:setEnabled(enabled)
    if enabled ~= nil and enabled == true then
        if self.enabled == false then
            self.enabled = true
            self:updateSensor(16)
        end
    else
        self.enabled = false
    end
end

function ADSensor:isEnabled()
    return self.enabled and self.initialized
end

function ADSensor:setTriggered(triggered)
    if triggered ~= nil and triggered == true then
        self.triggered = true
    else
        self.triggered = false
    end
end

function ADSensor:setTriggerType(triggerType)
    if triggerType ~= nil then
        self.triggerType = triggerType
    else
        self.triggerType = 0
    end
end

function ADSensor:isTriggered()
    return self.triggered
end

function ADSensor:getRotatedFront()
    --if self.frontAxle == nil then
        local frontWheel = nil
        local pairWheel = nil
        local frontDistance = math.huge
        local spec = self.vehicle.spec_wheels
        for _, wheel in pairs(spec.wheels) do
            local wheelNode = wheel.driveNode
            local sx, sy, sz = getWorldTranslation(wheelNode)
            local _, _, diffZ = AutoDrive.worldToLocal(self.vehicle, sx, sy, sz)
            if diffZ > 0 and diffZ < frontDistance and math.abs(frontDistance - diffZ) > 0.5 then
                frontWheel = wheel
                frontDistance = diffZ
            end
            if diffZ > 0 and (math.abs(frontDistance - diffZ) < 0.2) and wheel ~= frontWheel then
                pairWheel = wheel
            end
        end

        if frontWheel ~= nil and pairWheel ~= nil then
            local frontWheelX, frontWheelY, frontWheelZ = getWorldTranslation(frontWheel.driveNode)
            local pairWheelX, pairWheelY, pairWheelZ = getWorldTranslation(pairWheel.driveNode)
            local axleCenterX = frontWheelX + 0.5 * (pairWheelX - frontWheelX)
            local axleCenterY = frontWheelY + 0.5 * (pairWheelY - frontWheelY)
            local axleCenterZ = frontWheelZ + 0.5 * (pairWheelZ - frontWheelZ)
            local _, _, diffZ = AutoDrive.worldToLocal(self.vehicle, axleCenterX, axleCenterY, axleCenterZ)
            local wheelBaseToFront = self.vehicle.size.length / 2 - diffZ
            local leadingEdge = AutoDrive.getVehicleLeadingEdge(self.vehicle)
            self.frontAxleLength = wheelBaseToFront + leadingEdge
            self.frontAxle = {}
            self.frontAxle.x, self.frontAxle.y, self.frontAxle.z = AutoDrive.worldToLocal(self.vehicle, axleCenterX, axleCenterY, axleCenterZ)
        end

        if self.frontAxle ~= nil then
            local rx, _, rz =  AutoDrive.localDirectionToWorld(self.vehicle, math.sin(self.vehicle.rotatedTime), 0, math.cos(self.vehicle.rotatedTime))
            local frontPoint = {x = self.frontAxle.x + self.frontAxleLength * math.sin(self.vehicle.rotatedTime), y = self.frontAxle.y, z = self.frontAxle.z + self.frontAxleLength * math.cos(self.vehicle.rotatedTime)}

            return frontPoint
        end

    return nil
end

function ADSensor.debugMsg(vehicle, debugText, ...)
    if ADSensor.debug == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    end
end
