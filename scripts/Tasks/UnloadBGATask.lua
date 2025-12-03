UnloadBGATask = ADInheritsFrom(AbstractTask)

UnloadBGATask.STATE_IDLE = 0
UnloadBGATask.STATE_INIT = 1
UnloadBGATask.STATE_INIT_AXIS = 2
UnloadBGATask.STATE_ACTIVE = 3
UnloadBGATask.STATE_WAITING_FOR_RESTART = 4

UnloadBGATask.ACTION_DRIVETOSILO_COMMON_POINT = 0
UnloadBGATask.ACTION_DRIVETOSILO_CLOSE_POINT = 1
UnloadBGATask.ACTION_DRIVETOSILO_REVERSE_POINT = 2
UnloadBGATask.ACTION_DRIVETOSILO_REVERSE_STRAIGHT = 3
UnloadBGATask.ACTION_LOAD_ALIGN = 4
UnloadBGATask.ACTION_LOAD = 5
UnloadBGATask.ACTION_REVERSEFROMLOAD = 6
UnloadBGATask.ACTION_DRIVETOUNLOAD_INIT = 7
UnloadBGATask.ACTION_DRIVETOUNLOAD = 8
UnloadBGATask.ACTION_UNLOAD = 9
UnloadBGATask.ACTION_REVERSEFROMUNLOAD = 10

UnloadBGATask.SHOVELSTATE_UNKNOWN = 0
UnloadBGATask.SHOVELSTATE_LOW = 1
UnloadBGATask.SHOVELSTATE_LOADING = 2
UnloadBGATask.SHOVELSTATE_TRANSPORT = 3
UnloadBGATask.SHOVELSTATE_BEFORE_UNLOAD = 4
UnloadBGATask.SHOVELSTATE_UNLOAD = 5

UnloadBGATask.DRIVESTRATEGY_REVERSE_LEFT = 0
UnloadBGATask.DRIVESTRATEGY_REVERSE_RIGHT = 1
UnloadBGATask.DRIVESTRATEGY_FORWARD_LEFT = 2
UnloadBGATask.DRIVESTRATEGY_FORWARD_RIGHT = 3
UnloadBGATask.DRIVESTRATEGY_FORWARDS = 4
UnloadBGATask.DRIVESTRATEGY_REVERSE = 5

UnloadBGATask.INITAXIS_STATE_INIT = 0
UnloadBGATask.INITAXIS_STATE_ARM_INIT = 1
UnloadBGATask.INITAXIS_STATE_ARM_STEER = 2
UnloadBGATask.INITAXIS_STATE_ARM_CHECK = 3
UnloadBGATask.INITAXIS_STATE_EXTENDER_INIT = 4
UnloadBGATask.INITAXIS_STATE_EXTENDER_STEER = 5
UnloadBGATask.INITAXIS_STATE_EXTENDER_CHECK = 6
UnloadBGATask.INITAXIS_STATE_ROTATOR_INIT = 7
UnloadBGATask.INITAXIS_STATE_ROTATOR_STEER = 8
UnloadBGATask.INITAXIS_STATE_ROTATOR_CHECK = 9
UnloadBGATask.INITAXIS_STATE_DONE = 10

UnloadBGATask.SHOVEL_WIDTH_OFFSET = 0.8

function UnloadBGATask:new(vehicle)
    local o = UnloadBGATask:create()
    o.vehicle = vehicle
    return o
end

function UnloadBGATask:setUp()
    self.state = self.STATE_INIT
end

function UnloadBGATask:update(dt)
    if self.state == self.STATE_IDLE then
        self.isActive = false
        self.shovel = nil
        return
    else
        self.isActive = true
    end

    if self.targetUnloadTrigger == nil then
        self.targetUnloadTrigger = self:getTargetUnloadPoint()
    end

    self:getCurrentStates()

    if self.state == self.STATE_INIT then
        self:initializeBGA()
    elseif self.state == self.STATE_INIT_AXIS then
        if self:handleInitAxis(dt) then
            self.state = self.STATE_ACTIVE
        end
    elseif self.state == self.STATE_ACTIVE then
        if self.action == self.ACTION_DRIVETOSILO_COMMON_POINT then
            self:driveToSiloCommonPoint(dt)
        elseif self.action == self.ACTION_DRIVETOSILO_CLOSE_POINT then
            self:driveToSiloClosePoint(dt)
        elseif self.action == self.ACTION_DRIVETOSILO_REVERSE_POINT then
            self:driveToSiloReversePoint(dt)
        elseif self.action == self.ACTION_DRIVETOSILO_REVERSE_STRAIGHT then
            self:driveToSiloReverseStraight(dt)
        elseif self.action == self.ACTION_LOAD_ALIGN then
            self:alignLoadFromBGA(dt)
        elseif self.action == self.ACTION_LOAD then
            self:loadFromBGA(dt)
        elseif self.action == self.ACTION_REVERSEFROMLOAD then
            self:reverseFromBGALoad(dt)
        elseif self.action == self.ACTION_DRIVETOUNLOAD_INIT then
            self:driveToBGAUnloadInit(dt)
        elseif self.action == self.ACTION_DRIVETOUNLOAD then
            self:driveToBGAUnload(dt)
        elseif self.action == self.ACTION_UNLOAD then
            self:handleBGAUnload(dt)
        elseif self.action == self.ACTION_REVERSEFROMUNLOAD then
            self:reverseFromBGAUnload(dt)
        end
    elseif self.state == self.STATE_WAITING_FOR_RESTART then
        if self:checkIfPossibleToRestart(dt) then
            self.state = self.STATE_ACTIVE
        else
            self.vehicle.ad.specialDrivingModule:stopVehicle()
            self.vehicle.ad.specialDrivingModule:update(dt)
        end
    end

    self:handleShovel(dt)

    if (self.lastAction ~= nil) and (self.lastAction ~= self.action) then
        self.strategyActiveTimer.elapsedTime = math.huge
        self.storedDirection = nil
        self.lastAngleStrategyChange = nil
        self.checkedCurrentRow = false
    end

    self.lastState = self.state
    self.lastAction = self.action

    if self.targetDriver ~= nil then
        self.targetDriver.ad.noMovementTimer:timer(self.targetDriver.lastSpeedReal < 0.0004, 3000, dt)
    end
    
    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_BGA_MODE) then
        self:drawDebug()
    end
end

function UnloadBGATask:abort()
end

function UnloadBGATask:finished()
    self.vehicle.ad.taskModule:setCurrentTaskFinished(ADTaskModule.DONT_PROPAGATE)
end

function UnloadBGATask:getCurrentStates()
    self.shovelFillLevel = self:getShovelFillLevel()
    local fillLevel, fillCapacity, filledToUnload, fillFreeCapacity = AutoDrive.getObjectNonFuelFillLevels(self.targetTrailer)
    self.trailerLeftCapacity = fillFreeCapacity
    self.bunkerFillLevel = 10000 --self:getBunkerFillLevel();

    self.targetUnloadTriggerFree = false
    if self.targetUnloadTrigger ~= nil then
        local fillType = self.shovel:getFillUnitFillType(1)
        if fillType > 0 then
            self.targetUnloadTriggerFree = self.targetUnloadTrigger:getFillUnitFreeCapacity(1, fillType) >= self.shovelFilledCapacity
            --print("targetUnloadTriggerFree: " .. tostring(targetUnloadTriggerFree) .. " capacity: " .. self.targetUnloadTrigger:getFillUnitFreeCapacity(1, fillType))
        end        
    end

    if not self:checkCurrentTrailerStillValid() then
        self.targetTrailer = nil
        self.targetDriver = nil
    end
end

function UnloadBGATask:checkIfPossibleToRestart()
    if self.targetTrailer == nil then
        self.targetTrailer, self.targetDriver = self:findCloseTrailer()
        local fillLevel, fillCapacity, filledToUnload, fillFreeCapacity = AutoDrive.getObjectNonFuelFillLevels(self.targetTrailer)
        self.trailerLeftCapacity = fillFreeCapacity
    end
    if self.targetBunker == nil then
        self.targetBunker = self:getTargetBunker()
    end
    if self.targetUnloadTrigger == nil then
        self.targetUnloadTrigger = self:getTargetUnloadPoint()
    end

    if (self.targetUnloadTrigger and self.targetUnloadTriggerFree) or (not self.targetUnloadTrigger and self.targetTrailer ~= nil and self.trailerLeftCapacity >= 1 and self.targetBunker ~= nil and self.bunkerFillLevel > 0) then
        return true
    end
end

function UnloadBGATask:getShovelFillLevel()
    if self.shovel ~= nil then
        local fillLevel = 0
        local capacity = 0
        local fillUnitCount = 0
        for _, shovelNode in pairs(self.shovel.spec_shovel.shovelNodes) do
            fillLevel = fillLevel + self.shovel:getFillUnitFillLevel(shovelNode.fillUnitIndex)
            capacity = capacity + self.shovel:getFillUnitCapacity(shovelNode.fillUnitIndex)
            fillUnitCount = fillUnitCount + 1
            --print("Detected shovelNode width: " .. shovelNode.width)
            if self.shovelWidthTool == nil or self.shovelWidthTool < shovelNode.width then
                self.shovelWidthTool = shovelNode.width
            end
        end
        if self.shovelWidthTool ~= nil then
            self.shovelWidth = self.shovelWidthTool + AutoDrive.getSetting("shovelWidth", self.vehicle)
        else
            self.shovelWidth = 3.0 + AutoDrive.getSetting("shovelWidth", self.vehicle)
        end

        if self.targetBunker ~= nil then
            self:determineHighestShovelOffset()
        end
        self.shovelFilledCapacity = fillLevel

        return fillLevel / capacity
    end

    return 0
end

function UnloadBGATask:initializeBGA()
    self.state = self.STATE_INIT_AXIS
    self.action = self.ACTION_DRIVETOSILO_COMMON_POINT
    self.shovelTarget = self.SHOVELSTATE_LOW
    self.targetTrailer, self.targetDriver = self:findCloseTrailer()
    self.targetBunker = self:getTargetBunker()
    self.targetUnloadTrigger = self:getTargetUnloadPoint()
    self.unloadToTrigger = self.vehicle.ad.stateModule:getBunkerUnloadTypeIsTrigger()

    self.inShovelRangeTimer = AutoDriveTON:new()
    self.strategyActiveTimer = AutoDriveTON:new()
    self.shovelActiveTimer = AutoDriveTON:new()
    self.wheelsOnGround = AutoDriveTON:new()
    self.wheelsOffGround = AutoDriveTON:new()
    self.strategyActiveTimer.elapsedTime = math.huge
    self.shovelOffsetCounter = 0
    self.highestShovelOffsetCounter = 0
    self.reachedPreTargetLoadPoint = false
    self.shovelInBunkerArea = false

    if self.shovel == nil then
        self:getVehicleShovel()
        if self.shovel == nil then
            AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_No_Shovel;", 5000, self.vehicle.ad.stateModule:getName())
            self.state = self.STATE_IDLE
            self.vehicle:stopAutoDrive()
            return
        end
    end

    if self.targetBunker == nil then
        AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_No_Bunker;", 5000, self.vehicle.ad.stateModule:getName())
        self.state = self.STATE_IDLE
        self.vehicle:stopAutoDrive()
    end

    if self:checkForUnloadCondition() then
        self.action = self.ACTION_DRIVETOUNLOAD
    end
end

function UnloadBGATask:handleInitAxis(dt)
    self:handleShovel(dt)
    if self.shovel ~= nil then
        local rotationObject
        local rotationTarget = 0
        local translationObject
        local translationTarget = 0
        if self.initAxisState == nil then
            self.initAxisState = self.INITAXIS_STATE_ARM_INIT
        elseif self.initAxisState == self.INITAXIS_STATE_ARM_INIT then
            if self.armMain ~= nil then
                rotationObject = self.armMain
                self.initAxisStartHeight = self:getShovelHeight()
                self.initAxisStartRotation = rotationObject.curRot[1]
                rotationTarget = (rotationObject.rotMax - rotationObject.rotMin) / 2 + rotationObject.rotMin
                if math.abs(rotationTarget - rotationObject.curRot[1]) <= 0.1 then
                    rotationTarget = rotationObject.rotMin
                end
                self.armMain.rotationTarget = rotationTarget
                self.initAxisState = self.INITAXIS_STATE_ARM_STEER
            else
                self.initAxisState = self.INITAXIS_STATE_EXTENDER_INIT
            end
        elseif self.initAxisState == self.INITAXIS_STATE_ARM_STEER then
            rotationObject = self.armMain
            rotationTarget = self.armMain.rotationTarget
        elseif self.initAxisState == self.INITAXIS_STATE_ARM_CHECK then
            rotationObject = self.armMain
            rotationTarget = self.armMain.rotationTarget
            local newHeight = self:getShovelHeight()
            if (newHeight > self.initAxisStartHeight) == (rotationTarget > self.initAxisStartRotation) then
                self.armMain.moveUpSign = 1
                self.armMain.moveDownSign = -1
            else
                self.armMain.moveUpSign = -1
                self.armMain.moveDownSign = 1
            end
            self.initAxisState = self.INITAXIS_STATE_EXTENDER_INIT
        elseif self.initAxisState == self.INITAXIS_STATE_EXTENDER_INIT then
            if self.armExtender ~= nil then
                translationObject = self.armExtender
                self.initAxisStartHeight = self:getShovelHeight()
                self.initAxisStartTranslation = translationObject.curTrans[translationObject.translationAxis]
                translationTarget = (translationObject.transMax - translationObject.transMin) / 2 + translationObject.transMin
                if math.abs(translationTarget - translationObject.curTrans[translationObject.translationAxis]) <= 0.1 then
                    translationTarget = translationObject.transMin
                end
                self.armExtender.translationTarget = translationTarget
                self.initAxisState = self.INITAXIS_STATE_EXTENDER_STEER
            else
                self.initAxisState = self.INITAXIS_STATE_ROTATOR_INIT
            end
        elseif self.initAxisState == self.INITAXIS_STATE_EXTENDER_STEER then
            translationObject = self.armExtender
            translationTarget = self.armExtender.translationTarget
        elseif self.initAxisState == self.INITAXIS_STATE_EXTENDER_CHECK then
            translationObject = self.armExtender
            translationTarget = self.armExtender.translationTarget
            local newHeight = self:getShovelHeight()
            if (newHeight > self.initAxisStartHeight) == (translationTarget > self.initAxisStartTranslation) then
                self.armExtender.moveUpSign = 1
                self.armExtender.moveDownSign = -1
            else
                self.armExtender.moveUpSign = -1
                self.armExtender.moveDownSign = 1
            end
            self.initAxisState = self.INITAXIS_STATE_ROTATOR_INIT
        elseif self.initAxisState == self.INITAXIS_STATE_ROTATOR_INIT then
            if self.shovelRotator ~= nil then
                rotationObject = self.shovelRotator
                self.initAxisStartHeight = self:getShovelHeight()
                self.initAxisStartRotation = rotationObject.curRot[1]
                local _, dy, _ = localDirectionToWorld(self.shovel.spec_shovel.shovelDischargeInfo.node, 0, 0, 1)
                local angle = math.acos(dy)
                self.initAxisStartShovelRotation = angle
                rotationTarget = (rotationObject.rotMax - rotationObject.rotMin) / 2 + rotationObject.rotMin

                if math.abs(rotationTarget - rotationObject.curRot[1]) <= 0.1 then
                    rotationTarget = rotationObject.rotMin
                end
                self.shovelRotator.rotationTarget = rotationTarget

                self.shovelRotator.horizontalPosition = math.pi / 2.0

                self.initAxisState = self.INITAXIS_STATE_ROTATOR_STEER
            else
                self.initAxisState = self.INITAXIS_STATE_DONE
            end
        elseif self.initAxisState == self.INITAXIS_STATE_ROTATOR_STEER then
            rotationObject = self.shovelRotator
            rotationTarget = self.shovelRotator.rotationTarget
        elseif self.initAxisState == self.INITAXIS_STATE_ROTATOR_CHECK then
            rotationObject = self.shovelRotator
            rotationTarget = self.shovelRotator.rotationTarget
            local _, dy, _ = localDirectionToWorld(self.shovel.spec_shovel.shovelDischargeInfo.node, 0, 0, 1)
            local newAngle = math.acos(dy)
            if (newAngle > self.initAxisStartShovelRotation) == (rotationTarget > self.initAxisStartRotation) then
                self.shovelRotator.moveUpSign = 1
                self.shovelRotator.moveDownSign = -1
            else
                self.shovelRotator.moveUpSign = -1
                self.shovelRotator.moveDownSign = 1
            end
            self.initAxisState = self.INITAXIS_STATE_DONE
        elseif self.initAxisState == self.INITAXIS_STATE_DONE then
            return true
        end

        if rotationObject ~= nil then
            if self:steerAxisTo(rotationObject, rotationTarget, 100, dt) then
                if self.initAxisState == self.INITAXIS_STATE_ARM_STEER then
                    self.initAxisState = self.INITAXIS_STATE_ARM_CHECK
                elseif self.initAxisState == self.INITAXIS_STATE_EXTENDER_STEER then
                    self.initAxisState = self.INITAXIS_STATE_EXTENDER_CHECK
                elseif self.initAxisState == self.INITAXIS_STATE_ROTATOR_STEER then
                    self.initAxisState = self.INITAXIS_STATE_ROTATOR_CHECK
                end
            end
        end
        if translationObject ~= nil then
            if self:steerAxisToTrans(translationObject, translationTarget, 100, dt) then
                if self.initAxisState == self.INITAXIS_STATE_ARM_STEER then
                    self.initAxisState = self.INITAXIS_STATE_ARM_CHECK
                elseif self.initAxisState == self.INITAXIS_STATE_EXTENDER_STEER then
                    self.initAxisState = self.INITAXIS_STATE_EXTENDER_CHECK
                elseif self.initAxisState == self.INITAXIS_STATE_ROTATOR_STEER then
                    self.initAxisState = UnloadBGATask.INITAXIS_STATE_ROTATOR_CHECK
                end
            end
        end
    end
end

function UnloadBGATask:getShovelHeight()
    local x, y, z = getWorldTranslation(self.shovel.spec_shovel.shovelDischargeInfo.node)
    local height = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 1, z)
    return y - height
end

function UnloadBGATask:steerAxisTo(rotationObject, rotationTarget, targetFactor, dt)
    local reachedTarget = false
    if rotationObject ~= nil then
        local curRot = rotationObject.curRot[1]
        if curRot ~= rotationTarget then
            if math.abs(rotationTarget - curRot) < (dt * rotationObject.rotSpeed * (targetFactor * 0.01)) then
                curRot = rotationTarget
                reachedTarget = true
            else
                if curRot > rotationTarget then
                    curRot = curRot - (dt * rotationObject.rotSpeed * (targetFactor * 0.01))
                else
                    curRot = curRot + (dt * rotationObject.rotSpeed * (targetFactor * 0.01))
                end
            end
            curRot = math.min(math.max(curRot, rotationObject.rotMin), rotationObject.rotMax)
            rotationObject.curRot[1] = curRot
            setRotation(rotationObject.node, unpack(rotationObject.curRot))
            SpecializationUtil.raiseEvent(self.vehicle, "onMovingToolChanged", rotationObject, rotationObject.rotSpeed, dt)

            Cylindered.setDirty(self.vehicle, rotationObject)
            self.vehicle:raiseDirtyFlags(self.vehicle.spec_cylindered.cylinderedDirtyFlag)
        end
    end
    return reachedTarget
end

function UnloadBGATask:steerAxisToTrans(translationObject, translationTarget, targetFactor, dt)
    local reachedTarget = false
    if translationObject ~= nil then
        local curTrans = translationObject.curTrans[translationObject.translationAxis]
        if curTrans ~= translationTarget then
            if math.abs(translationTarget - curTrans) < (dt * translationObject.transSpeed * (targetFactor * 0.01)) then
                curTrans = translationTarget
                reachedTarget = true
            else
                if curTrans > translationTarget then
                    curTrans = curTrans - (dt * translationObject.transSpeed * (targetFactor * 0.01))
                else
                    curTrans = curTrans + (dt * translationObject.transSpeed * (targetFactor * 0.01))
                end
            end
            curTrans = math.min(math.max(curTrans, translationObject.transMin), translationObject.transMax)
            translationObject.curTrans[translationObject.translationAxis] = curTrans
            setTranslation(translationObject.node, unpack(translationObject.curTrans))
            SpecializationUtil.raiseEvent(self.vehicle, "onMovingToolChanged", translationObject, translationObject.transSpeed, dt)

            Cylindered.setDirty(self.vehicle, translationObject)
            self.vehicle:raiseDirtyFlags(self.vehicle.spec_cylindered.cylinderedDirtyFlag)
        end
    end
    return reachedTarget
end

function UnloadBGATask:checkForUnloadCondition() --can unload if shovel is filled and trailer available
    if self.action == self.ACTION_DRIVETOSILO_COMMON_POINT then
        return self.shovelFillLevel > 0 and ((self.targetTrailer ~= nil and self.trailerLeftCapacity > 1) or (self.unloadToTrigger and self.targetUnloadTriggerFree))
    elseif self.action == self.ACTION_LOAD then
        return self.shovelFillLevel >= 0.98 and ((self.targetTrailer ~= nil and self.trailerLeftCapacity > 1) or (self.unloadToTrigger and self.targetUnloadTriggerFree))
    end
    return false
end

function UnloadBGATask:checkForStopLoading() --stop loading when shovel is filled
    return self.shovelFillLevel >= 0.98
end

function UnloadBGATask:checkForIdleCondition() --idle if shovel filled and no trailer available to fill;
    if self.shovelFillLevel >= 0.98 and ((((self.targetTrailer ~= nil or self.trailerLeftCapacity <= 1) or self.targetTrailer == nil) and not self.self.unloadToTrigger) or (self.unloadToTrigger and not self.targetUnloadTriggerFree)) then
        return true
    end
    return false
end

function UnloadBGATask:handleShovel(dt)
    if self.shovelState == nil then
        self.shovelState = self.SHOVELSTATE_UNKNOWN
    end

    self.shovelActiveTimer:timer(((self.shovelState ~= self.shovelTarget) and (self.state > self.STATE_INIT_AXIS)), 7000, dt)

    if self.state > self.STATE_INIT_AXIS then
        if self.shovelState == self.SHOVELSTATE_UNKNOWN then
            if not self.shovelActiveTimer:done() then
                self:moveShovelToTarget(self.SHOVELSTATE_LOW, dt)
            else
                --After timeout, assume we reached desired position as good as possible
                self.shovelState = self.shovelTarget
            end
        else
            if self.shovelState == self.shovelTarget and self.shovelInBunkerArea ~= self:isShovelInBunkerArea() then
                self.shovelState = self.SHOVELSTATE_UNKNOWN
            end
            if self.shovelState ~= self.shovelTarget then
                if not self.shovelActiveTimer:done() then
                    self:moveShovelToTarget(self.shovelTarget, dt)
                else
                    --After timeout, assume we reached desired position as good as possible
                    self.shovelState = self.shovelTarget
                end
            else
                --make sure shovel hasnt't lifted wheels
                local allWheelsOnGround = self:checkIfAllWheelsOnGround()
                --local onGroundForLongTime = self.wheelsOnGround:timer(allWheelsOnGround, 300, dt)
                local liftedForLongTime = self.wheelsOffGround:timer(not allWheelsOnGround, 300, dt)
                if liftedForLongTime and self.armMain ~= nil then --or (not onGroundForLongTime)
                    self:steerAxisTo(self.armMain, self.armMain.moveUpSign * math.pi, 33, dt)
                end
            end
        end
    end

    if self.shovel ~= nil then
        self.shovelInBunkerArea = self:isShovelInBunkerArea()
    end
end

function UnloadBGATask:moveShovelToTarget(_, dt)
    if self.shovelTarget == self.SHOVELSTATE_LOADING then
        if self:isShovelInBunkerArea() then
            self.shovelTargetHeight = -0.25 + AutoDrive.getSetting("shovelHeight", self.vehicle)
        else
            self.shovelTargetHeight = -0.20 + AutoDrive.getSetting("shovelHeight", self.vehicle)
        end
        self.shovelTargetAngle = self.shovelRotator.horizontalPosition + self.shovelRotator.moveUpSign * 0.11
        if self.armExtender ~= nil then
            self.shovelTargetExtension = self.armExtender.transMin
        end
    elseif self.shovelTarget == self.SHOVELSTATE_LOW then
        self.shovelTargetHeight = 1.1
        self.shovelTargetAngle = self.shovelRotator.horizontalPosition - self.shovelRotator.moveUpSign * 0.3
        if self.armExtender ~= nil then
            self.shovelTargetExtension = self.armExtender.transMin
        end
    elseif self.shovelTarget == self.SHOVELSTATE_TRANSPORT then
        self.shovelTargetHeight = 2.1
        self.shovelTargetAngle = self.shovelRotator.horizontalPosition - self.shovelRotator.moveUpSign * 0.3
        if self.armExtender ~= nil then
            self.shovelTargetExtension = self.armExtender.transMin
        end
    elseif self.shovelTarget == self.SHOVELSTATE_BEFORE_UNLOAD then
        self.shovelTargetHeight = 4.7
        self.shovelTargetAngle = self.shovelRotator.horizontalPosition - self.shovelRotator.moveUpSign * 0.1
        if self.armExtender ~= nil then
            self.shovelTargetExtension = self.armExtender.transMax
        end
    elseif self.shovelTarget == self.SHOVELSTATE_UNLOAD then
        self.shovelTargetHeight = 4.7
        self.shovelTargetAngle = self.shovelRotator.horizontalPosition + self.shovelRotator.moveUpSign * 0.5
        if self.armExtender ~= nil then
            self.shovelTargetExtension = self.armExtender.transMax
        end
    end

    local targetFactorHeight = math.max(5, math.min((math.abs(self:getShovelHeight() - self.shovelTargetHeight) * 200), 100))
    local targetFactorExtender = 0
    --local extenderTargetReached = true
    if self.armExtender ~= nil then
        --if math.abs(self.shovelTargetExtension - self.armExtender.curTrans[self.armExtender.translationAxis]) >= 0.01 then
        --extenderTargetReached = false
        --end
        targetFactorExtender = math.max(5, math.min((math.abs(self.shovelTargetExtension - self.armExtender.curTrans[self.armExtender.translationAxis]) * 100), 70))
    end

    local _, dy, _ = localDirectionToWorld(self.shovel.spec_shovel.shovelDischargeInfo.node, 0, 0, 1)
    local angle = math.acos(dy)
    local shovelTargetAngleReached = false
    if math.abs(angle - self.shovelTargetAngle) <= 0.05 then
        shovelTargetAngleReached = true
    end
    if self.shovelTarget == self.SHOVELSTATE_UNLOAD then
        if (math.abs(self.shovelRotator.curRot[1] - self.shovelRotator.rotMax) <= 0.01 or math.abs(self.shovelRotator.curRot[1] - self.shovelRotator.rotMin) <= 0.01) then
            shovelTargetAngleReached = true
        end
    end
    local targetFactorHorizontal = math.max(1, math.min(self:getAngleBetweenTwoRadValues(angle, self.shovelTargetAngle) * 100, 100))

    --keep shovel in targetPosition
    if not shovelTargetAngleReached then
        local targetRotation = self.shovelRotator.moveUpSign * math.pi
        if (angle - self.shovelTargetAngle) >= 0 then
            targetRotation = self.shovelRotator.moveDownSign * math.pi
        end
        self:steerAxisTo(self.shovelRotator, targetRotation, targetFactorHorizontal, dt)
    end

    if shovelTargetAngleReached and (self.action ~= self.ACTION_UNLOAD) then
        if self:getShovelHeight() >= self.shovelTargetHeight then
            self:steerAxisTo(self.armMain, self.armMain.moveDownSign * math.pi, targetFactorHeight, dt)
            if self.armExtender ~= nil then
                self:steerAxisToTrans(self.armExtender, self.armExtender.moveDownSign * math.pi, targetFactorExtender, dt)
            end
        else
            self:steerAxisTo(self.armMain, self.armMain.moveUpSign * math.pi, targetFactorHeight, dt)
            if self.armExtender ~= nil then
                self:steerAxisToTrans(self.armExtender, self.armExtender.moveUpSign * math.pi, targetFactorExtender, dt)
            end
        end
    end

    local allAxisFullyExtended = false
    if
        (self.armMain ~= nil and (math.abs(self.armMain.curRot[1] - self.armMain.rotMax) <= 0.01 or math.abs(self.armMain.curRot[1] - self.armMain.rotMin) <= 0.01)) and
            (self.armExtender == nil or math.abs(self.armExtender.curTrans[self.armExtender.translationAxis] - self.armExtender.transMax) <= 0.01)
     then
        allAxisFullyExtended = true
    end

    if ((math.abs(self:getShovelHeight() - self.shovelTargetHeight) <= 0.01) or (allAxisFullyExtended and (self.shovelTargetHeight > 4 or self.shovelTargetHeight < 0.5)) or (self.action == self.ACTION_UNLOAD)) and shovelTargetAngleReached then
        self.shovelState = self.shovelTarget
    end
end

function UnloadBGATask:checkIfAllWheelsOnGround()
    local spec = self.vehicle.spec_wheels    
    for _, wheel in pairs(spec.wheels) do
        if not wheel.physics.hasGroundContact then
            return false
        end
    end
    return true
end

function UnloadBGATask:getAngleBetweenTwoRadValues(valueOne, valueTwo)
    local abs = math.abs(valueOne - valueTwo)
    if abs > math.pi then
        abs = math.abs(abs - (2 * math.pi))
    end
    return abs
end

function UnloadBGATask:getVehicleShovel()    
    for _, implement in pairs(AutoDrive.getAllImplements(self.vehicle, true)) do
        if implement.spec_shovel ~= nil then
            self.shovelAxisOne = self.vehicle.spec_cylindered.movingTools
            self.shovelAxisTwo = implement.spec_cylindered.movingTools
            self.shovel = implement
        end
    end

    if self.shovel ~= nil then
        --split into axis
        for _, axis in pairs(self.shovelAxisOne) do
            if axis.axis == "AXIS_FRONTLOADER_ARM" then
                self.armMain = axis
            elseif axis.axis == "AXIS_FRONTLOADER_ARM2" then
                self.armExtender = axis
            elseif axis.axis == "AXIS_FRONTLOADER_TOOL" then
                self.shovelRotator = axis
            end
        end
        for _, axis in pairs(self.shovelAxisTwo) do
            if axis.axis == "AXIS_FRONTLOADER_ARM" then
                self.armMain = axis
            elseif axis.axis == "AXIS_FRONTLOADER_ARM2" then
                self.armExtender = axis
            elseif axis.axis == "AXIS_FRONTLOADER_TOOL" then
                self.shovelRotator = axis
            end
        end
    end
end

function UnloadBGATask:findCloseTrailer()
    local closestDistance = 50
    local closest = nil
    local closestTrailer = nil
    for _, vehicle in pairs(AutoDrive.getAllVehicles()) do
        if vehicle ~= self.vehicle and self:vehicleHasTrailersAttached(vehicle) and vehicle.ad ~= nil and vehicle.ad.noMovementTimer ~= nil then
            if AutoDrive.getDistanceBetween(vehicle, self.vehicle) < closestDistance and vehicle.ad.noMovementTimer:timer(vehicle.lastSpeedReal < 0.0004, 3000, 16) and (not vehicle.ad.trailerModule:isActiveAtTrigger()) then
                local _, trailers = self:vehicleHasTrailersAttached(vehicle)
                for _, trailer in pairs(trailers) do
                    if trailer ~= nil then
                        local fillLevel, fillCapacity, filledToUnload, trailerLeftCapacity = AutoDrive.getObjectNonFuelFillLevels(trailer)

                        if trailerLeftCapacity >= 10 then
                            closestDistance = AutoDrive.getDistanceBetween(trailer, self.vehicle)
                            closest = vehicle
                            closestTrailer = trailer
                        end
                    end
                end
            end
        end
    end
    if closest ~= nil then
        return closestTrailer, closest
    end
    return
end

function UnloadBGATask:vehicleHasTrailersAttached(vehicle)
    local trailers, trailerCount = AutoDrive.getAllUnits(vehicle)
    local tipTrailers = {}
    if trailers ~= nil then
        for _, trailer in pairs(trailers) do
            local _, maxCapacity, _ = AutoDrive.getObjectNonFuelFillLevels(trailer)
            
            if trailer.typeName == "trailer" or (maxCapacity >= 7000) then
                table.insert(tipTrailers, trailer)
            end
        end
    end

    return (#tipTrailers > 0), tipTrailers
end

function UnloadBGATask:checkCurrentTrailerStillValid()
    if self.targetTrailer ~= nil and self.targetDriver ~= nil then
        local tooFast = math.abs(self.targetDriver.lastSpeedReal) > 0.002
        local fillLevel, fillCapacity, filledToUnload, fillFreeCapacity = AutoDrive.getObjectNonFuelFillLevels(self.targetTrailer)
        local tooFull = fillFreeCapacity < 1

        return not (tooFull or tooFast)
    end

    return false
end

function UnloadBGATask:getTargetBunker()
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    local closestDistance = math.huge
    local closest = nil
    for _, trigger in pairs(ADTriggerManager.getUnloadTriggers()) do
        if trigger.bunkerSiloArea ~= nil then
            local centerX, centerZ = self:getBunkerCenter(trigger)
            local distance = math.sqrt(math.pow(centerX - x, 2) + math.pow(centerZ - z, 2))
            if distance < closestDistance and distance < 100 then
                closest = trigger
                closestDistance = distance
            end
        end
    end

    return closest
end

function UnloadBGATask:getTargetUnloadPoint()
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    local closestDistance = math.huge
    local closest = nil
    if self.shovel ~= nil and self.shovel.getFillUnitFillType ~= nil then
        local fillType = self.shovel:getFillUnitFillType(1)
        if fillType ~= nil then
            for _, trigger in pairs(ADTriggerManager.getAllTriggersForFillType(fillType)) do                
                local triggerX, _, triggerZ = ADTriggerManager.getTriggerPos(trigger)
                local distance = math.sqrt(math.pow(triggerX - x, 2) + math.pow(triggerZ - z, 2))
                if distance < closestDistance and distance < 1000 then
                    closest = trigger
                    closestDistance = distance
                end
            end
        end
        --print("Found closest trigger with distance: " .. closestDistance)
    end   

    return closest
end

function UnloadBGATask:getTargetBunkerLoadingSide()
    if self.targetBunker == nil then
        self:getTargetBunker()
        if self.targetBunker == nil then
            return
        end
    end

    if self.loadingSideP1 ~= nil then
        return self.loadingSideP1, self.loadingSideP2
    end

    local trigger = self.targetBunker
                                                                        --        vecW
    local x1, z1 = trigger.bunkerSiloArea.sx, trigger.bunkerSiloArea.sz --      1 ---- 2
    local x2, z2 = trigger.bunkerSiloArea.wx, trigger.bunkerSiloArea.wz -- vecH | ---- |
    local x3, z3 = trigger.bunkerSiloArea.hx, trigger.bunkerSiloArea.hz --      | ---- |
    local x4, z4 = x2 + (x3 - x1), z2 + (z3 - z1)                       --      3 ---- 4    4 = 2 + vecH

    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)

    self.vecW = {x = (x2 - x1), z = (z2 - z1)}
    self.vecH = {x = (x3 - x1), z = (z3 - z1)}
    self.vecWLength = MathUtil.vector2Length(self.vecW.x, self.vecW.z)
    self.vecHLength = MathUtil.vector2Length(self.vecH.x, self.vecH.z)

    if self.vecWLength < self.vecHLength then
        if MathUtil.vector2Length(x - x1, z - z1) <= MathUtil.vector2Length(x - x3, z - z3) then
            self.loadingSideP1 = {x = x1, z = z1}
            self.loadingSideP2 = {x = x2, z = z2}
        else
            self.vecH.x = -self.vecH.x
            self.vecH.z = -self.vecH.z
            self.loadingSideP1 = {x = x3, z = z3}
            self.loadingSideP2 = {x = x4, z = z4}
        end
    else
        if MathUtil.vector2Length(x - x1, z - z1) <= MathUtil.vector2Length(x - x2, z - z2) then
            self.loadingSideP1 = {x = x1, z = z1}
            self.loadingSideP2 = {x = x3, z = z3}
        else
            self.vecW.x = -self.vecW.x
            self.vecW.z = -self.vecW.z
            self.loadingSideP1 = {x = x2, z = z2}
            self.loadingSideP2 = {x = x4, z = z4}
        end
    end

    return self.loadingSideP1, self.loadingSideP2
end

function UnloadBGATask:getBunkerCenter(trigger)
    local x1, z1 = trigger.bunkerSiloArea.sx, trigger.bunkerSiloArea.sz
    local x2, z2 = trigger.bunkerSiloArea.wx, trigger.bunkerSiloArea.wz
    local x3, z3 = trigger.bunkerSiloArea.hx, trigger.bunkerSiloArea.hz

    return x1 + 0.5 * ((x2 - x1) + (x3 - x1)), z1 + 0.5 * ((z2 - z1) + (z3 - z1))
end

function UnloadBGATask:isAlmostInBunkerSiloArea(distanceToCheck)
    local trigger = self.targetBunker
    local x1, z1 = trigger.bunkerSiloArea.sx, trigger.bunkerSiloArea.sz
    local x2, z2 = trigger.bunkerSiloArea.wx, trigger.bunkerSiloArea.wz
    local x3, z3 = trigger.bunkerSiloArea.hx, trigger.bunkerSiloArea.hz

    local otherBoundingBox = {{x = x1, z = z1}, {x = x2, z = z2}, {x = x3, z = z3}, {x = x2 + (x3 - x1), z = z2 + (z3 - z1)}}

    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    --create bounding box to check for vehicle
    local rx, _, rz =  AutoDrive.localDirectionToWorld(self.vehicle, math.sin(self.vehicle.rotatedTime), 0, math.cos(self.vehicle.rotatedTime))
    local vehicleVector = {x = rx, z = rz}
    local width = self.vehicle.size.width
    local length = self.vehicle.size.length
    local ortho = {x = -vehicleVector.z, z = vehicleVector.x}
    local boundingBox = {}
    boundingBox[1] = {
        x = x + (width / 2) * ortho.x,
        y = y + 2,
        z = z + (width / 2) * ortho.z
    }
    boundingBox[2] = {
        x = x - (width / 2) * ortho.x,
        y = y + 2,
        z = z - (width / 2) * ortho.z
    }
    boundingBox[3] = {
        x = x - (width / 2) * ortho.x + (length / 2 + distanceToCheck) * vehicleVector.x,
        y = y + 2,
        z = z - (width / 2) * ortho.z + (length / 2 + distanceToCheck) * vehicleVector.z
    }
    boundingBox[4] = {
        x = x + (width / 2) * ortho.x + (length / 2 + distanceToCheck) * vehicleVector.x,
        y = y + 2,
        z = z + (width / 2) * ortho.z + (length / 2 + distanceToCheck) * vehicleVector.z
    }

    return AutoDrive.boxesIntersect(boundingBox, otherBoundingBox)
end

function UnloadBGATask:isShovelInBunkerArea()
    if self.shovel ~= nil and self.targetBunker ~= nil then        
        local x, y, z = getWorldTranslation(self.shovel.spec_shovel.shovelDischargeInfo.node)
        local tx, _, tz = localToWorld(self.shovel.spec_shovel.shovelDischargeInfo.node, 0, 0, -1)
        --local tx, _, tz = x, y, z + 1
        if self.targetBunker ~= nil and self.targetBunker.bunkerSiloArea ~= nil then
            local x1, z1 = self.targetBunker.bunkerSiloArea.sx, self.targetBunker.bunkerSiloArea.sz
            local x2, z2 = self.targetBunker.bunkerSiloArea.wx, self.targetBunker.bunkerSiloArea.wz
            local x3, z3 = self.targetBunker.bunkerSiloArea.hx, self.targetBunker.bunkerSiloArea.hz
            if MathUtil.hasRectangleLineIntersection2D(x1, z1, x2 - x1, z2 - z1, x3 - x1, z3 - z1, x, z, tx - x, tz - z) then
                return true
            end
        end
    end
    
    return false
end

function UnloadBGATask:driveToSiloCommonPoint(dt)
    if (self.checkedCurrentRow == nil or self.checkedCurrentRow == false) then
        self:setShovelOffsetToNonEmptyRow()
        self.checkedCurrentRow = true
    end

    --self.targetPoint = self:getTargetForShovelOffset(14)
    self.targetPoint = self:getTargetForShovelOffset(AutoDrive.getVehicleLeadingEdge(vehicle) + 6)
    local angleToSilo = self:getAngleToTarget() -- in +/- 180°

    if self.storedDirection == nil then
        self.storedDirection = true
        if math.abs(angleToSilo) > 90 then
            self.storedDirection = false
        end
    end
    self.driveStrategy = self:getDriveStrategyByAngle(angleToSilo, self.storedDirection, dt)

    self.shovelTarget = self.SHOVELSTATE_LOW

    if self:getDistanceToTarget() <= 4 then
        self.action = self.ACTION_DRIVETOSILO_CLOSE_POINT
    end

    self:handleDriveStrategy(dt)
end

function UnloadBGATask:driveToSiloClosePoint(dt)
    self.targetPoint = self:getTargetForShovelOffset(6)
    self.driveStrategy = self:getDriveStrategyToTarget(true, dt)

    self.shovelTarget = self.SHOVELSTATE_LOW

    if self:getDistanceToTarget() <= 4 then
        self.action = self.ACTION_DRIVETOSILO_REVERSE_POINT
    end

    self:handleDriveStrategy(dt)
end

function UnloadBGATask:driveToSiloReversePoint(dt)
    --self.targetPoint = self:getTargetForShovelOffset(18)
    self.targetPoint = self:getTargetForShovelOffset(AutoDrive.getVehicleLeadingEdge(vehicle) + 6)
    self.driveStrategy = self:getDriveStrategyToTarget(false, dt)

    self.shovelTarget = self.SHOVELSTATE_LOW

    local angleToSilo = self:getAngleToTarget()

    if self:getDistanceToTarget() <= 9 or (math.abs(angleToSilo) >= 177) then
        self.action = self.ACTION_DRIVETOSILO_REVERSE_STRAIGHT
    end

    self:handleDriveStrategy(dt)
end

function UnloadBGATask:driveToSiloReverseStraight(dt)
    self.targetPoint = self:getTargetForShovelOffset(68)
    self.driveStrategy = self:getDriveStrategyToTarget(false, dt)

    self.shovelTarget = self.SHOVELSTATE_LOW

    local angleToSilo = self:getAngleToTarget()
    if self:getDistanceToTarget() <= 53 or (math.abs(angleToSilo) >= 177) then
        self.action = self.ACTION_LOAD_ALIGN
    end

    self:handleDriveStrategy(dt)
end

function UnloadBGATask:alignLoadFromBGA(dt)
    self.targetPoint = self:getTargetForShovelOffset(5)
    self.driveStrategy = self:getDriveStrategyToTarget(true, dt)

    self.shovelTarget = self.SHOVELSTATE_LOADING

    if self.shovelState ~= self.shovelTarget then
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return
    end

    if self:getDistanceToTarget() <= 4 then
        self.action = self.ACTION_LOAD
    end

    self:handleDriveStrategy(dt)
end

function UnloadBGATask:handleDriveStrategy(dt)
    if self.driveStrategy == self.DRIVESTRATEGY_REVERSE_LEFT or self.driveStrategy == self.DRIVESTRATEGY_REVERSE_RIGHT then
        local finalSpeed = 8
        local acc = 0.4
        local allowedToDrive = true

        local offsetZ = -5
        local offsetX = 5
        if self.driveStrategy == self.DRIVESTRATEGY_REVERSE_LEFT then
            offsetX = -5
        end
        local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
        local rx, _, rz = AutoDrive.localDirectionToWorld(self.vehicle, offsetX, 0, offsetZ)
        x = x + rx
        z = z + rz
        local lx, lz = AutoDrive.getDriveDirection(self.vehicle, x, y, z)
        self:driveInDirection(dt, 30, acc, 0.2, 20, allowedToDrive, false, lx, lz, finalSpeed, 1)
    elseif self.driveStrategy == self.DRIVESTRATEGY_FORWARD_LEFT or self.driveStrategy == self.DRIVESTRATEGY_FORWARD_RIGHT then
        local finalSpeed = 8
        local acc = 0.4
        local allowedToDrive = true

        local offsetZ = 5
        local offsetX = 5
        if self.driveStrategy == self.DRIVESTRATEGY_FORWARD_LEFT then
            offsetX = -5
        end
        local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
        local rx, _, rz = AutoDrive.localDirectionToWorld(self.vehicle, offsetX, 0, offsetZ)
        x = x + rx
        z = z + rz
        local lx, lz = AutoDrive.getDriveDirection(self.vehicle, x, y, z)
        self:driveInDirection(dt, 30, acc, 0.2, 20, allowedToDrive, true, lx, lz, finalSpeed, 1)
    else
        local finalSpeed = 10
        local acc = 0.6
        local allowedToDrive = true

        local _, y, _ = getWorldTranslation(self.vehicle.components[1].node)
        local lx, lz = AutoDrive.getDriveDirection(self.vehicle, self.targetPoint.x, y, self.targetPoint.z)
        local driveForwards = true
        if self.driveStrategy == self.DRIVESTRATEGY_REVERSE then
            lx = -lx
            lz = -lz
            driveForwards = false
            finalSpeed = 20
        end
        self:driveInDirection(dt, 30, acc, 0.2, 20, allowedToDrive, driveForwards, lx, lz, finalSpeed, 1)
    end

    if self.vehicle.isServer then
        if self.vehicle.startMotor and self.vehicle.stopMotor then
            if not self.vehicle:getIsMotorStarted() and self.vehicle:getCanMotorRun() and not self.vehicle.ad.specialDrivingModule:shouldStopMotor() then
                self.vehicle:startMotor()
            end
        end
    end
end

function UnloadBGATask:getDriveStrategyToTarget(drivingForward, dt)
    local angleToSilo = self:getAngleToTarget() -- in +/- 180°

    return self:getDriveStrategyByAngle(angleToSilo, drivingForward, dt)
end

function UnloadBGATask:getDriveStrategyToTrailerInit(dt)
    local xT, _, zT = getWorldTranslation(self.targetTrailer.components[1].node)

    local rx, _, rz =  AutoDrive.localDirectionToWorld(self.targetTrailer, 1, 0, 0)
    local offSideLeft = {x = xT + rx * 10, z = zT + rz * 10}

    local lx, _, lz =  AutoDrive.localDirectionToWorld(self.targetTrailer, -1, 0, 0)
    local offSideRight = {x = xT + lx * 10, z = zT + lz * 10}

    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)

    local distanceToLeft = math.sqrt(math.pow(offSideLeft.x - x, 2) + math.pow(offSideLeft.z - z, 2))
    local distanceToRight = math.sqrt(math.pow(offSideRight.x - x, 2) + math.pow(offSideRight.z - z, 2))

    if distanceToLeft <= distanceToRight then
        self.targetPoint = {x = offSideLeft.x, z = offSideLeft.z}
    else
        self.targetPoint = {x = offSideRight.x, z = offSideRight.z}
    end

    local angleToTrailer = self:getAngleToTarget() -- in +/- 180°

    return self:getDriveStrategyByAngle(angleToTrailer, true, dt)
end

function UnloadBGATask:getDriveStrategyToTrigger(dt)
    local triggerX, _, triggerZ = ADTriggerManager.getTriggerPos(self.targetUnloadTrigger)    
    self.targetPoint = {x = triggerX, z = triggerZ} 

    return self:getDriveStrategyByAngle(self:getAngleToTarget(), true, dt)
end

function UnloadBGATask:getDriveStrategyToTrailer(dt)
    local xT, _, zT = getWorldTranslation(self.targetTrailer.components[1].node)

    self.targetPoint = {x = xT, z = zT}

    local angleToTrailer = self:getAngleToTarget() -- in +/- 180°

    return self:getDriveStrategyByAngle(angleToTrailer, true, dt)
end

function UnloadBGATask:getDriveStrategyByAngle(angleToTarget, drivingForward, dt)
    if self.lastAngleStrategyChange == nil then
        self.lastAngleStrategyChange = math.huge
    end

    local angleDiffToLast = math.deg(self:getAngleBetweenTwoRadValues(math.rad(self.lastAngleStrategyChange), math.rad(angleToTarget)))

    local time = 3000
    local timeToChange = self.strategyActiveTimer:timer(true, time, dt)
    local angleToCheckFor = 30
    local newStrategy = self.lastAngleStrategy
    local minimumAngleDiff = 9
    if self.vehicle.spec_articulatedAxis ~= nil and self.vehicle.spec_articulatedAxis.rotSpeed ~= nil then
        minimumAngleDiff = 40
    end

    if timeToChange or (angleDiffToLast > minimumAngleDiff) or (math.abs(angleToTarget) < 10) or (math.abs(angleToTarget) > 170) then
        if drivingForward then
            if angleToTarget < -angleToCheckFor then
                newStrategy = self.DRIVESTRATEGY_REVERSE_RIGHT
            elseif angleToTarget > angleToCheckFor then
                newStrategy = self.DRIVESTRATEGY_REVERSE_LEFT
            elseif (math.abs(angleToTarget) <= angleToCheckFor) then
                newStrategy = self.DRIVESTRATEGY_FORWARDS
            end
        else
            angleToCheckFor = 180 - angleToCheckFor
            if ((angleToTarget < angleToCheckFor) and (angleToTarget >= 0)) then
                newStrategy = self.DRIVESTRATEGY_FORWARD_RIGHT
            elseif ((angleToTarget > -angleToCheckFor) and (angleToTarget < 0)) then
                newStrategy = self.DRIVESTRATEGY_FORWARD_LEFT
            elseif (math.abs(angleToTarget) >= angleToCheckFor) then
                newStrategy = self.DRIVESTRATEGY_REVERSE
            end
        end
    end

    if self.lastAngleStrategy ~= newStrategy then
        self.lastAngleStrategyChange = angleToTarget
        self.strategyActiveTimer:timer(false)
    end
    self.lastAngleStrategy = newStrategy

    return self.lastAngleStrategy
end

function UnloadBGATask:getAngleToTarget()
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    local rx, _, rz =  AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    if self.vehicle.spec_articulatedAxis ~= nil and self.vehicle.spec_articulatedAxis.rotSpeed ~= nil then
        rx, _, rz =  AutoDrive.localDirectionToWorld(self.vehicle, MathUtil.sign(self.vehicle.spec_articulatedAxis.rotSpeed) * math.sin(self.vehicle.rotatedTime), 0, math.cos(self.vehicle.rotatedTime))
        rx, _, rz =  AutoDrive.localDirectionToWorld(self.vehicle, MathUtil.sign(self.vehicle.spec_articulatedAxis.rotSpeed) * math.sin(self.vehicle.rotatedTime) / 2, 0, (1 + math.cos(self.vehicle.rotatedTime)) / 2)
    end
    local vehicleVector = {x = rx, z = rz}

    local vecToTrailer = {x = self.targetPoint.x - x, z = self.targetPoint.z - z}

    return AutoDrive.angleBetween(vehicleVector, vecToTrailer)
end

function UnloadBGATask:loadFromBGA(dt)
    self.shovelTarget = self.SHOVELSTATE_LOADING

    if self:checkForStopLoading() then
        self.action = self.ACTION_REVERSEFROMLOAD
    end

    self.targetPoint = self:getTargetForShovelOffset(-MathUtil.vector2Length(self.vecH.x, self.vecH.z))
    self.driveStrategy = self.DRIVESTRATEGY_FORWARDS

    if self:getDistanceToTarget() <= 4 then
        self.action = self.ACTION_REVERSEFROMLOAD
    end

    self:handleDriveStrategy(dt)
end

function UnloadBGATask:getTargetForShovelOffset(inFront)
    local offsetToUse = self.shovelOffsetCounter
    local fromOtherSide = false
    if self.shovelOffsetCounter > self.highestShovelOffsetCounter then
        offsetToUse = 0
        fromOtherSide = true
    end
    local offset = (self.shovelWidth * (0.5 + offsetToUse)) + self.SHOVEL_WIDTH_OFFSET
    return self:getPointXInFrontAndYOffsetFromBunker(inFront, offset, fromOtherSide)
end

function UnloadBGATask:getPointXInFrontAndYOffsetFromBunker(inFront, offset, fromOtherSide)
    local p1, p2 = self:getTargetBunkerLoadingSide()
    if fromOtherSide ~= nil and fromOtherSide == true then
        p1, p2 = p2, p1
    end
    local normalizedVec = {x = (p2.x - p1.x) / (math.abs(p2.x - p1.x) + math.abs(p2.z - p1.z)), z = (p2.z - p1.z) / (math.abs(p2.x - p1.x) + math.abs(p2.z - p1.z))}
    --get ortho for 'inFront' parameter
    local ortho = {x = -normalizedVec.z, z = normalizedVec.x}
    local factor = math.sqrt(math.pow(ortho.x, 2) + math.pow(ortho.z, 2))
    ortho.x = ortho.x / factor
    ortho.z = ortho.z / factor
    
    --get shovel offset correct position on silo line
    local targetPoint = {x = p1.x + normalizedVec.x * offset, z = p1.z + normalizedVec.z * offset}

    local pointPositive = {x = targetPoint.x + ortho.x * inFront, z = targetPoint.z + ortho.z * inFront}
    local pointNegative = {x = targetPoint.x - ortho.x * inFront, z = targetPoint.z - ortho.z * inFront}
    local bunkerCenter = {}
    bunkerCenter.x, bunkerCenter.z = self:getBunkerCenter(self.targetBunker)

    local result = pointNegative
    if inFront < 0 then --we want a point inside the bunker. So use the closer one
        result = pointPositive
    end
    if math.sqrt(math.pow(bunkerCenter.x - pointPositive.x, 2) + math.pow(bunkerCenter.z - pointPositive.z, 2)) >= math.sqrt(math.pow(bunkerCenter.x - pointNegative.x, 2) + math.pow(bunkerCenter.z - pointNegative.z, 2)) then
        result = pointPositive
        if inFront < 0 then --we want a point inside the bunker. So use the closer one
            result = pointNegative
        end
    end

    return result
end

function UnloadBGATask:reverseFromBGALoad(dt)
    self.shovelTarget = self.SHOVELSTATE_LOW

    self.targetPoint = self:getTargetForShovelOffset(200)
    --self.targetPointClose = self:getTargetForShovelOffset(16)
    self.targetPointClose = self:getTargetForShovelOffset(AutoDrive.getVehicleLeadingEdge(vehicle) + 10)

    local finalSpeed = 30
    local acc = 1
    local allowedToDrive = true

    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local lx, lz = AutoDrive.getDriveDirection(self.vehicle, self.targetPoint.x, y, self.targetPoint.z)
    AutoDrive.driveInDirection(self.vehicle, dt, 30, acc, 0.2, 20, allowedToDrive, false, -lx, -lz, finalSpeed, 1)

    if math.sqrt(math.pow(x - self.targetPointClose.x, 2) + math.pow(z - self.targetPointClose.z, 2)) < 5 then
        self.action = self.ACTION_DRIVETOUNLOAD_INIT
        if self.shovelFillLevel <= 0.01 then
            self.action = self.ACTION_DRIVETOSILO_COMMON_POINT
        end
        if self.shovelOffsetCounter > self.highestShovelOffsetCounter then
            self.shovelOffsetCounter = 0
        else
            self.shovelOffsetCounter = self.shovelOffsetCounter + 1
        end
    end
end

function UnloadBGATask:driveToBGAUnloadInit(dt)
    if self.targetTrailer == nil and not self.unloadToTrigger then
        self:getVehicleToPause()
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return
    end

    self.shovelTarget = self.SHOVELSTATE_BEFORE_UNLOAD

    if not self.unloadToTrigger then
        self.driveStrategy = self:getDriveStrategyToTrailerInit(dt)
    else
        self.driveStrategy = self:getDriveStrategyToTrigger(dt)
    end
    self:handleDriveStrategy(dt)

    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)

    if math.sqrt(math.pow(self.targetPoint.x - x, 2) + math.pow(self.targetPoint.z - z, 2)) <= 4 then
        self.action = self.ACTION_DRIVETOUNLOAD
    end
    if ((self.targetTrailer == nil or (self.trailerLeftCapacity <= 0.001)) and not self.unloadToTrigger) or ((self.unloadToTrigger and not self.targetUnloadTriggerFree)) then
        self.action = self.ACTION_REVERSEFROMUNLOAD
        self.shovelTarget = self.SHOVELSTATE_BEFORE_UNLOAD
        self.shovel:setDischargeState(Dischargeable.DISCHARGE_STATE_OFF, true)
    end
end

function UnloadBGATask:driveToBGAUnload(dt)
    if self.targetTrailer == nil and not self.unloadToTrigger then
        AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_No_Trailer;", 5000, self.vehicle.ad.stateModule:getName())
        self:getVehicleToPause()
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return
    end
    if self.unloadToTrigger or (AutoDrive.getDistanceBetween(self.vehicle, self.targetTrailer) <= 10) then
        self.shovelTarget = self.SHOVELSTATE_BEFORE_UNLOAD
    elseif AutoDrive.getDistanceBetween(self.vehicle, self.targetTrailer) > 20 then
        self.shovelTarget = self.SHOVELSTATE_TRANSPORT
    end
    if self.shovelState ~= self.shovelTarget then
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return
    end

    if not self.unloadToTrigger then
        self.driveStrategy = self:getDriveStrategyToTrailer(dt)
    else
        self.driveStrategy = self:getDriveStrategyToTrigger(dt)
    end

    self:handleDriveStrategy(dt)

    if self.inShovelRangeTimer:timer(self:getShovelInTrailerRange(), 350, dt) then
        self.action = self.ACTION_UNLOAD
    end
    if (not self.unloadToTrigger and (self.targetTrailer == nil or (self.trailerLeftCapacity <= 0.1))) or (self.unloadToTrigger and not self.targetUnloadTriggerFree) then
        self.action = self.ACTION_REVERSEFROMUNLOAD
        self.shovelTarget = self.SHOVELSTATE_BEFORE_UNLOAD
        self.shovel:setDischargeState(Dischargeable.DISCHARGE_STATE_OFF, true)
    end
end

function UnloadBGATask:handleBGAUnload(dt)
    self.vehicle.ad.specialDrivingModule:stopVehicle()
    self.vehicle.ad.specialDrivingModule:update(dt)
    self.shovelTarget = self.SHOVELSTATE_UNLOAD
    local xV, _, zV = getWorldTranslation(self.vehicle.components[1].node)
    self.shovelUnloadPosition = {x = xV, z = zV}

    if self.shovelFillLevel <= 0.01 then
        self.strategyActiveTimer.elapsedTime = math.huge
        self.shovelState = self.SHOVELSTATE_UNLOAD
        self.action = self.ACTION_REVERSEFROMUNLOAD
        self.shovel:setDischargeState(Dischargeable.DISCHARGE_STATE_OFF, true)
    end
    if (not self.unloadToTrigger and (self.targetTrailer == nil or (self.trailerLeftCapacity <= 0.1))) or (self.unloadToTrigger and not self.targetUnloadTriggerFree) then
        self.action = self.ACTION_REVERSEFROMUNLOAD
        self.shovelState = self.SHOVELSTATE_UNLOAD
        self.shovelTarget = self.SHOVELSTATE_BEFORE_UNLOAD
        self.shovel:setDischargeState(Dischargeable.DISCHARGE_STATE_OFF, true)
    end
end

function UnloadBGATask:reverseFromBGAUnload(dt)
    self.shovelTarget = self.SHOVELSTATE_BEFORE_UNLOAD
    self.shovel:setDischargeState(Dischargeable.DISCHARGE_STATE_OFF, true)

    for _, shovelNode in pairs(self.shovel.spec_shovel.shovelNodes) do
        shovelNode.litersToDrop = 0
    end

    if self.shovelState ~= self.shovelTarget then
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return
    end

    local finalSpeed = 9
    local acc = 1
    local allowedToDrive = true

    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    local rx, _, rz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, -1)
    x = x + rx
    z = z + rz
    --local lx, lz = AutoDrive.getDriveDirection(self.vehicle, x, y, z)
    AutoDrive.driveInDirection(self.vehicle, dt, 30, acc, 0.2, 20, allowedToDrive, false, nil, nil, finalSpeed, 1)

    if self.shovelUnloadPosition ~= nil then
        if MathUtil.vector2Length(x - self.shovelUnloadPosition.x, z - self.shovelUnloadPosition.z) >= 6 then
            self.shovelTarget = self.SHOVELSTATE_LOW
        else
            self.shovelTarget = self.SHOVELSTATE_BEFORE_UNLOAD
        end

        if MathUtil.vector2Length(x - self.shovelUnloadPosition.x, z - self.shovelUnloadPosition.z) >= 8 then
            self.shovel:setDischargeState(Dischargeable.DISCHARGE_STATE_OFF, true)
            self.action = self.ACTION_DRIVETOSILO_COMMON_POINT
        end
    end    
end

function UnloadBGATask:getVehicleToPause()
    self.state = self.STATE_WAITING_FOR_RESTART
end

function UnloadBGATask:getShovelInTrailerRange()
    local dischargeNode = self.shovel:getCurrentDischargeNode()
    if dischargeNode ~= nil then
        local dischargeTarget = dischargeNode.dischargeObject
        if dischargeTarget ~= nil then
            local result = self.shovel:getDischargeState() == Dischargeable.DISCHARGE_STATE_OBJECT --and dischargeTarget == self.targetTrailer
            return result
        end
    end
    return false
end

function UnloadBGATask:determineHighestShovelOffset()
    local width = self.shovelWidth
    local p1, p2 = self:getTargetBunkerLoadingSide()
    local sideLength = MathUtil.vector2Length(p1.x - p2.x, p1.z - p2.z)
    self.highestShovelOffsetCounter = math.floor((sideLength - 2 * self.SHOVEL_WIDTH_OFFSET) / width) - 1
end

function UnloadBGATask:getDistanceToTarget()
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    return MathUtil.vector2Length(x - self.targetPoint.x, z - self.targetPoint.z)
end

function UnloadBGATask:stateToText()
    local text = nil
    if self.state == self.STATE_INIT or self.state == self.STATE_INIT_AXIS then
        text = g_i18n:getText("ad_bga_init")
    elseif self.state == self.STATE_WAITING_FOR_RESTART then
        text = g_i18n:getText("ad_bga_waiting")
    elseif self.state == self.STATE_ACTIVE then
        text = g_i18n:getText("ad_bga_active")
    end

    return text
end

function UnloadBGATask:checkForFillLevelInCurrentRow()
    local offsetToUse = self.shovelOffsetCounter
    local fromOtherSide = false
    if self.shovelOffsetCounter > self.highestShovelOffsetCounter then
        offsetToUse = 0
        fromOtherSide = true
    end
    --local inFront = 0

    local p1, p2 = self:getTargetBunkerLoadingSide()
    if fromOtherSide ~= nil and fromOtherSide == true then
        p1, p2 = p2, p1
    end
    local normalizedVec = {x = (p2.x - p1.x) / (math.abs(p2.x - p1.x) + math.abs(p2.z - p1.z)), z = (p2.z - p1.z) / (math.abs(p2.x - p1.x) + math.abs(p2.z - p1.z))}
    --get ortho for 'inFront' parameter
    --local ortho = {x = -normalizedVec.z, z = normalizedVec.x}
    --get shovel offset correct position on silo line
    local offset = (self.shovelWidth * (0.0 + offsetToUse))
    local targetPoint = {x = p1.x + normalizedVec.x * offset, z = p1.z + normalizedVec.z * offset}
    offset = (self.shovelWidth * (1.0 + offsetToUse))
    local targetPoint2 = {x = p1.x + normalizedVec.x * offset, z = p1.z + normalizedVec.z * offset}

    local pointPositive = {x = targetPoint.x + self.vecH.x, z = targetPoint.z + self.vecH.z}
    local pointNegative = {x = targetPoint.x - self.vecH.x, z = targetPoint.z - self.vecH.z}
    local bunkerCenter = {}
    bunkerCenter.x, bunkerCenter.z = self:getBunkerCenter(self.targetBunker)

    local result = pointNegative
    if math.sqrt(math.pow(bunkerCenter.x - pointPositive.x, 2) + math.pow(bunkerCenter.z - pointPositive.z, 2)) <= math.sqrt(math.pow(bunkerCenter.x - pointNegative.x, 2) + math.pow(bunkerCenter.z - pointNegative.z, 2)) then
        result = pointPositive
    end

    local innerFillLevel1 = 0 --DensityMapHeightUtil.getFillLevelAtArea(self.targetBunker.fermentingFillType, targetPoint.x,targetPoint.z, targetPoint2.x,targetPoint2.z, result.x,result.z)
    local innerFillLevel2 = DensityMapHeightUtil.getFillLevelAtArea(self.targetBunker.outputFillType, targetPoint.x, targetPoint.z, targetPoint2.x, targetPoint2.z, result.x, result.z)
    local innerFillLevel = innerFillLevel1 + innerFillLevel2

    return innerFillLevel
end

function UnloadBGATask:setShovelOffsetToNonEmptyRow()
    local currentFillLevel = self:checkForFillLevelInCurrentRow()
    local iterations = self.highestShovelOffsetCounter + 1
    while ((currentFillLevel == 0) and (iterations >= 0)) do
        iterations = iterations - 1
        if self.shovelOffsetCounter > self.highestShovelOffsetCounter then
            self.shovelOffsetCounter = 0
        else
            self.shovelOffsetCounter = self.shovelOffsetCounter + 1
        end
        currentFillLevel = self:checkForFillLevelInCurrentRow()
    end

    if ((currentFillLevel == 0) and (iterations < 0)) then
        AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_No_Bunker;", 5000, self.vehicle.ad.stateModule:getName())
        self.state = self.STATE_IDLE
        self.vehicle:stopAutoDrive()
    end
end

function UnloadBGATask:driveInDirection(dt, steeringAngleLimit, acceleration, slowAcceleration, slowAngleLimit, allowedToDrive, moveForwards, lx, lz, maxSpeed, slowDownFactor)
    if lx ~= nil and lz ~= nil then
        local dot = lz
        local angle = math.deg(math.acos(dot))
        if angle < 0 then
            angle = angle + 180
        end
        local turnLeft = lx > 0.00001
        if not moveForwards then
            turnLeft = not turnLeft
        end
        local targetRotTime = 0
        if turnLeft then
            --rotate to the left
            targetRotTime = self.vehicle.maxRotTime * math.min(angle / steeringAngleLimit, 1)
        else
            --rotate to the right
            targetRotTime = self.vehicle.minRotTime * math.min(angle / steeringAngleLimit, 1)
        end

        if math.abs(targetRotTime - self.vehicle.rotatedTime) >= 0.1 then
            maxSpeed = 1
        end

        AutoDrive.driveInDirection(self.vehicle, dt, steeringAngleLimit, acceleration, slowAcceleration, slowAngleLimit, allowedToDrive, moveForwards, lx, lz, maxSpeed, slowDownFactor)
    end
end

function UnloadBGATask:drawDebug()
    local node = self.vehicle.components[1].node
    local x, y, z = getWorldTranslation(node)

    -- line to trailer or trigger
    if self.targetUnloadTrigger ~= nil then
        local triggerX, triggerY, triggerZ = ADTriggerManager.getTriggerPos(self.targetUnloadTrigger)
        ADDrawingManager:addLineTask(x, y, z, triggerX, triggerY, triggerZ, 1, 0, 1, 0)
    end
    -- line to current target
    if self.targetPoint ~= nil then
        ADDrawingManager:addLineTask(x, y, z, self.targetPoint.x, y, self.targetPoint.z, 1, 1, 0, 0)
    end
    
    -- Bunker size
    if self.targetBunker ~= nil then
        local trigger = self.targetBunker
        local x1, z1 = trigger.bunkerSiloArea.sx, trigger.bunkerSiloArea.sz
        local x2, z2 = trigger.bunkerSiloArea.wx, trigger.bunkerSiloArea.wz
        local x3, z3 = trigger.bunkerSiloArea.hx, trigger.bunkerSiloArea.hz

        local corners = {{x = x1, z = z1}, {x = x2, z = z2}, {x = x3, z = z3}, {x = x2 + (x3 - x1), z = z2 + (z3 - z1)}}

        ADDrawingManager:addLineTask(corners[1].x, y, corners[1].z, corners[2].x, y, corners[2].z, 1, 1, 0, 0)
        ADDrawingManager:addLineTask(corners[2].x, y, corners[2].z, corners[3].x, y, corners[3].z, 1, 1, 0, 0)
        ADDrawingManager:addLineTask(corners[3].x, y, corners[3].z, corners[4].x, y, corners[4].z, 1, 1, 0, 0)
        ADDrawingManager:addLineTask(corners[4].x, y, corners[4].z, corners[1].x, y, corners[1].z, 1, 1, 0, 0)
    end
end