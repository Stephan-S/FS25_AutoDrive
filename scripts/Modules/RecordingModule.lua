ADRecordingModule = {}

function ADRecordingModule:new(vehicle)
    local o = {}
    setmetatable(o, self)
    self.__index = self
    o.vehicle = vehicle
    if not vehicle.ad.ADLeftNode then
        vehicle.ad.ADLeftNode = createTransformGroup("ADLeftNode")
        link(vehicle.rootNode, vehicle.ad.ADLeftNode)
        setRotation(vehicle.ad.ADLeftNode, 0, 0, 0)
        setTranslation(vehicle.ad.ADLeftNode, 0, 0, 0)
        vehicle.ad.ADRightNode = createTransformGroup("ADRightNode")
        link(vehicle.rootNode, vehicle.ad.ADRightNode)
        setRotation(vehicle.ad.ADRightNode, 0, 0, 0)
        setTranslation(vehicle.ad.ADRightNode, 0, 0, 0)
    end
    ADRecordingModule.reset(o)
    return o
end

function ADRecordingModule:reset()
    self.isDual = false
    self.isSubPrio = false
    self.isTwoWay = false
    self.trailerCount = 0
    self.flags = 0
    self.isRecording = false
    self.isRecordingReverse = false
    self.drivingReverse = false
    self.lastWp = nil
    self.secondLastWp = nil
    self.lastWp2 = nil -- 2 road recording
    self.secondLastWp2 = nil -- 2 road recording
end

function ADRecordingModule:start(dual, subPrio, twoWay)
    self.isDual = dual
    self.isSubPrio = subPrio
    self.isTwoWay = twoWay
    self.vehicle:stopAutoDrive()
    if self.isTwoWay then
        if math.abs(AutoDrive.getSetting("RecordDriveDirectionOffset") - AutoDrive.getSetting("RecordOppositeDriveDirectionOffset")) < 0.1 then
            -- disable recording if twoWay selected but distance difference between them < 0.1
            self.vehicle.ad.stateModule:disableCreationMode()
            AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_gui_ad_RecordOppositeDriveDirectionOffset;", 1000)
            return
        elseif self.vehicle.ad.ADRightNode then
            AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.INFO, "$l10n_gui_ad_RecordDriveDirectionOffset;", 1000)
        end
    end

    local startNodeId, _ = self.vehicle:getClosestWayPoint()
    local startNode = ADGraphManager:getWayPointById(startNodeId)

    if self.isSubPrio then
        self.flags = self.flags + AutoDrive.FLAG_SUBPRIO
    end

    local rearOffset = -2

    self.drivingReverse = (self.vehicle.lastSpeedReal * self.vehicle.movingDirection) < 0
    local firstNode, secondNode = self:getRecordingNodes()
    local x1, y1, z1 = getWorldTranslation(firstNode)
    if self.drivingReverse then
        -- no 2 road recording in reverse driving
        self.isTwoWay = false
        x1, y1, z1 = AutoDrive.localToWorld(self.vehicle, 0, 0, rearOffset, self.vehicle.ad.specialDrivingModule:getReverseNode())
    end
    if self.isTwoWay then
        if math.abs(AutoDrive.getSetting("RecordDriveDirectionOffset") - AutoDrive.getSetting("RecordOppositeDriveDirectionOffset")) >= 0.1 then
            self.lastWp = ADGraphManager:recordWayPoint(x1, y1, z1, false, false, self.drivingReverse, 0, self.flags)
            local x2, y2, z2 = getWorldTranslation(secondNode)
            self.lastWp2 = ADGraphManager:recordWayPoint(x2, y2, z2, false, false, self.drivingReverse, 0, self.flags)
        end
    else
        self.lastWp = ADGraphManager:recordWayPoint(x1, y1, z1, false, false, self.drivingReverse, 0, self.flags)
    end

    if not self.isTwoWay and AutoDrive.getSetting("autoConnectStart") then
        -- no autoconnect for 2 road recording
        if startNode ~= nil then
            if ADGraphManager:getDistanceBetweenNodes(startNodeId, self.lastWp.id) < 12 then
                ADGraphManager:toggleConnectionBetween(startNode, self.lastWp, self.drivingReverse, self.isDual)
            end
        end
    end
    self.isRecording = true
    self.isRecordingReverse = self.drivingReverse
    self.wasRecordingTwoRoads = self.isTwoWay
end

function ADRecordingModule:stop()
    if self.isRecording and not (self.isTwoWay or self.wasRecordingTwoRoads ~= self.isTwoWay) and AutoDrive.getSetting("autoConnectEnd") then
        -- no autoconnect for 2 road recording or if changed between single and two road recording
        if self.lastWp ~= nil then
            local targetId = ADGraphManager:findMatchingWayPointForVehicle(self.vehicle)
            local targetNode = ADGraphManager:getWayPointById(targetId)
            if targetNode ~= nil then
                ADGraphManager:toggleConnectionBetween(self.lastWp, targetNode, false, self.isDual)
            end
        end
    end
    self:reset()
end

function ADRecordingModule:updateTick(dt, isActiveForInput, isActiveForInputIgnoreSelection, isSelected)
    if not AutoDrive.getSetting("RecordWhileNotInVehicle") then
        if not self.vehicle.ad.stateModule:isInCreationMode() then
            return
        end
    end
    if not self.isRecording then
        return
    end

    local firstNode, secondNode = self:getRecordingNodes()
    local diffZ = 0
    if self.lastWp then
        _, _, diffZ = AutoDrive.worldToLocal(self.vehicle, self.lastWp.x, self.lastWp.y, self.lastWp.z, firstNode)
    elseif self.lastWp2 then
        _, _, diffZ = AutoDrive.worldToLocal(self.vehicle, self.lastWp2.x, self.lastWp2.y, self.lastWp2.z, firstNode)
    end
    self.drivingReverse = self.isRecordingReverse
    if self.isRecordingReverse and (diffZ < -1) then
        self.drivingReverse = false
    elseif not self.isRecordingReverse and (diffZ > 1) then
        self.drivingReverse = true
    end

    if self.isTwoWay then
        if self.drivingReverse or (self.wasRecordingTwoRoads and math.abs(AutoDrive.getSetting("RecordDriveDirectionOffset") - AutoDrive.getSetting("RecordOppositeDriveDirectionOffset")) < 0.1) then
            -- no 2 road recording in reverse driving, changed offsets improper or distance set to 0 - stop recording
            self.vehicle.ad.stateModule:disableCreationMode()
            AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_gui_ad_RecordOppositeDriveDirectionOffset;", 1000)
            return
        else
            -- 2 road recording
            self:twoRoadRecording()
        end
    else
        -- 1 road recording
        self:singleRecording()
    end
    self.wasRecordingTwoRoads = self.isTwoWay
end

function ADRecordingModule:singleRecording()
    local rearOffset = -2
    local vehicleX, _, vehicleZ = getWorldTranslation(self.vehicle.components[1].node)
    local reverseX, _, reverseZ = AutoDrive.localToWorld(self.vehicle, 0, 0, rearOffset, self.vehicle.ad.specialDrivingModule:getReverseNode())
    local firstNode, secondNode = self:getRecordingNodes()
    local x, y, z = getWorldTranslation(firstNode)

    if self.drivingReverse then
        x, y, z = AutoDrive.localToWorld(self.vehicle, 0, 0, rearOffset, self.vehicle.ad.specialDrivingModule:getReverseNode())
    end

    local minDistanceToLastWayPoint = true
    if self.isRecordingReverse ~= self.drivingReverse then
        --now we want a minimum distance from the last recording position to the last recorded point
        if self.isRecordingReverse then
            minDistanceToLastWayPoint = (MathUtil.vector2Length(reverseX - self.lastWp.x, reverseZ - self.lastWp.z) > 1)
        else
            if not self.isDual then
                minDistanceToLastWayPoint = (MathUtil.vector2Length(vehicleX - self.lastWp.x, vehicleZ - self.lastWp.z) > 1)
            else
                minDistanceToLastWayPoint = false
            end
        end
    end

    local speedMatchesRecording = (self.vehicle.lastSpeedReal * self.vehicle.movingDirection) > 0
    if self.drivingReverse then
        speedMatchesRecording = (self.vehicle.lastSpeedReal * self.vehicle.movingDirection) < 0
    end

    if self.secondLastWp == nil then
        if MathUtil.vector2Length(x - self.lastWp.x, z - self.lastWp.z) > 3 and MathUtil.vector2Length(vehicleX - self.lastWp.x, vehicleZ - self.lastWp.z) > 3 then
            self.secondLastWp = self.lastWp
            self.lastWp = ADGraphManager:recordWayPoint(x, y, z, true, self.isDual, self.drivingReverse, self.secondLastWp.id, self.flags)
            self.isRecordingReverse = self.drivingReverse
        end
    else
        local angle = math.abs(AutoDrive.angleBetween({x = x - self.secondLastWp.x, z = z - self.secondLastWp.z}, {x = self.lastWp.x - self.secondLastWp.x, z = self.lastWp.z - self.secondLastWp.z}))
        local max_distance = 6
        if angle < 0.5 then
            max_distance = 12
        elseif angle < 1 then
            max_distance = 6
        elseif angle < 2 then
            max_distance = 4
        elseif angle < 4 then
            max_distance = 3
        elseif angle < 7 then
            max_distance = 2
        elseif angle < 14 then
            max_distance = 1
        elseif angle < 27 then
            max_distance = 0.5
        else
            max_distance = 0.25
        end

        if self.drivingReverse then
            max_distance = math.min(max_distance, 2)
        end

        if MathUtil.vector2Length(x - self.lastWp.x, z - self.lastWp.z) > max_distance and minDistanceToLastWayPoint and speedMatchesRecording then
            self.secondLastWp = self.lastWp
            self.lastWp = ADGraphManager:recordWayPoint(x, y, z, true, self.isDual, self.drivingReverse, self.secondLastWp.id, self.flags)
            self.isRecordingReverse = self.drivingReverse
        end
    end
end

function ADRecordingModule:twoRoadRecording()
    if math.abs(AutoDrive.getSetting("RecordDriveDirectionOffset") - AutoDrive.getSetting("RecordOppositeDriveDirectionOffset")) >= 0.1 then
        local firstNode, secondNode = self:getRecordingNodes()

        self.speedMatchesRecording = (self.vehicle.lastSpeedReal * self.vehicle.movingDirection) > 0
        self.steeringAngle = math.deg(self.vehicle.rotatedTime)

        self.lastWp, self.secondLastWp = self:recordTwoRoad(firstNode, self.lastWp, self.secondLastWp, true)
        self.lastWp2, self.secondLastWp2 = self:recordTwoRoad(secondNode, self.lastWp2, self.secondLastWp2, false)
    end
end

function ADRecordingModule:recordTwoRoad(node, lastWp, secondLastWp, right)
    if node == nil then
        return nil, nil
    end
    local max_distance1 = 6
    local x1, y1, z1 = getWorldTranslation(node)
    local diffZ = 0
    if lastWp then
        _, _, diffZ = AutoDrive.worldToLocal(self.vehicle, lastWp.x, lastWp.y, lastWp.z, node)
    end
    if secondLastWp == nil then
        if math.abs(self.steeringAngle) > 10 then
            max_distance1 = 1
        else
            max_distance1 = 3
        end
        if MathUtil.vector2Length(x1 - lastWp.x, z1 - lastWp.z) > max_distance1 then
            secondLastWp = lastWp
            lastWp = ADGraphManager:recordWayPoint(x1, y1, z1, false, self.isDual, self.drivingReverse, secondLastWp.id, self.flags)
            if right then
                ADGraphManager:toggleConnectionBetween(secondLastWp, lastWp, self.drivingReverse, self.isDual)
            else
                ADGraphManager:toggleConnectionBetween(lastWp, secondLastWp, self.drivingReverse, self.isDual)
            end
        end
    else
        local angle1 = math.abs(AutoDrive.angleBetween({x = x1 - secondLastWp.x, z = z1 - secondLastWp.z}, {x = lastWp.x - secondLastWp.x, z = lastWp.z - secondLastWp.z}))
        if angle1 < 0.5 then
            max_distance1 = 12
        elseif angle1 < 1 then
            max_distance1 = 6
        elseif angle1 < 2 then
            max_distance1 = 4
        elseif angle1 < 4 then
            max_distance1 = 3
        elseif angle1 < 7 then
            max_distance1 = 2
        elseif angle1 < 14 then
            max_distance1 = 1
        elseif angle1 < 27 then
            max_distance1 = 0.5
        else
            max_distance1 = 0.25
        end
        if (self.steeringAngle < -15 and right)
            or (self.steeringAngle > 15 and not right) then
            -- steering right / left inner cicle for RHD / LHD
            max_distance1 = 1
        end
        if MathUtil.vector2Length(x1 - lastWp.x, z1 - lastWp.z) > max_distance1 and diffZ < -0.2 and self.speedMatchesRecording then
            secondLastWp = lastWp
            lastWp = ADGraphManager:recordWayPoint(x1, y1, z1, false, self.isDual, self.drivingReverse, secondLastWp.id, self.flags)
            if right then
                ADGraphManager:toggleConnectionBetween(secondLastWp, lastWp, self.drivingReverse, self.isDual)
            else
                ADGraphManager:toggleConnectionBetween(lastWp, secondLastWp, self.drivingReverse, self.isDual)
            end
        end
    end
    return lastWp, secondLastWp
end

function ADRecordingModule:update(dt)
end

function ADRecordingModule:getRecordingNodes()
    local firstNode = self.vehicle.components[1].node
    local secondNode = nil
    if self.drivingReverse then
        firstNode = self.vehicle.ad.specialDrivingModule:getReverseNode()
    elseif self.isTwoWay and self.vehicle.ad.ADRightNode then
        firstNode = self.vehicle.ad.ADRightNode
        setTranslation(self.vehicle.ad.ADRightNode, -AutoDrive.getSetting("RecordDriveDirectionOffset"), 0, 0)
        secondNode = self.vehicle.ad.ADLeftNode
        setTranslation(self.vehicle.ad.ADLeftNode, -AutoDrive.getSetting("RecordOppositeDriveDirectionOffset"), 0, 0)
    end

    return firstNode, secondNode
end
