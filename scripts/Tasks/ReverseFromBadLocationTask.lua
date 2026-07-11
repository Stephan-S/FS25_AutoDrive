ReverseFromBadLocationTask = ADInheritsFrom(AbstractTask)

ReverseFromBadLocationTask.STATE_REVERSING = 1

function ReverseFromBadLocationTask:new(vehicle)
    local o = ReverseFromBadLocationTask:create()
    o.vehicle = vehicle
    o.trailers = nil
    return o
end

function ReverseFromBadLocationTask:setUp()
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    self.startLocation = {x=x, y=y, z=z}
    self.timeOut = AutoDriveTON:new()
    self.frontFreeTimer = AutoDriveTON:new()
    self.trailers, _ = AutoDrive.getAllUnits(self.vehicle)
    AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, false)
end

function ReverseFromBadLocationTask:update(dt)
    local timedOut = self.timeOut:timer(true, 20000, dt)
    local frontBlocked = self.vehicle.ad.sensors.frontSensorLong:pollInfo() or self.vehicle.ad.sensors.frontSensorLongFruit:pollInfo()
    local leftFrontBlocked = self.vehicle.ad.sensors.leftFrontSensorFruit:pollInfo() or self.vehicle.ad.sensors.leftFrontSensor:pollInfo()
    local rightFrontBlocked = self.vehicle.ad.sensors.rightFrontSensorFruit:pollInfo() or self.vehicle.ad.sensors.rightFrontSensor:pollInfo()
    local frontCleared = self.frontFreeTimer:timer(not frontBlocked, 4000, dt)

    if frontBlocked or (self.frontFreeTimer.elapsedTime > 0 and not frontCleared) then
        --reverse straight
        
        if self.reverseTarget == nil then
            --print("ReverseFromBadLocationTask:update(dt) - frontBlocked")
            local xOffset = 0
            --reverse left (to turn front right)
            if leftFrontBlocked and not rightFrontBlocked then
                --print("ReverseFromBadLocationTask:update(dt) - leftFrontBlocked and not rightFrontBlocked")
                xOffset = 50
            --reverse right (to turn front left)
            elseif rightFrontBlocked and not leftFrontBlocked then
                --print("ReverseFromBadLocationTask:update(dt) - rightFrontBlocked and not leftFrontBlocked")
                xOffset = -50
            end

            local x, y, z = AutoDrive.localToWorld(self.vehicle, xOffset, 0 , -100)
            self.reverseTarget = {x=x, y=y, z=z}
        end

        self.vehicle.ad.specialDrivingModule:reverseToTargetLocation(dt, self.reverseTarget)
    elseif leftFrontBlocked or rightFrontBlocked then
        --print("ReverseFromBadLocationTask:update(dt) - leftFrontBlocked or rightFrontBlocked")
        local xOffset = 0
        if leftFrontBlocked and not rightFrontBlocked then
            xOffset = -70
        elseif rightFrontBlocked and not leftFrontBlocked then
            xOffset = 70
        end
        local x, y, z = AutoDrive.localToWorld(self.vehicle, xOffset, 0 , 30)
        self.forwardsTarget = {x=x, y=y, z=z}
        self.vehicle.ad.specialDrivingModule:driveToPoint(dt, self.forwardsTarget, 8, true, 0.5, 8)
    else
        self:finished()
    end

    local vehicleX, _, vehicleZ = getWorldTranslation(self.vehicle.components[1].node)
    if timedOut or MathUtil.vector2Length(vehicleX - self.startLocation.x, vehicleZ - self.startLocation.z) > 30 then
        --print("ReverseTask timed out or went too far from start")
        self:finished()
    end
end

function ReverseFromBadLocationTask:abort()
end

function ReverseFromBadLocationTask:finished()
    self.vehicle.ad.taskModule:setCurrentTaskFinished()
end

function ReverseFromBadLocationTask:getInfoText()
    return g_i18n:getText("AD_task_reversing_from_collision")
end

function ReverseFromBadLocationTask:getI18nInfo()
    return "$l10n_AD_task_reversing_from_collision;"
end
