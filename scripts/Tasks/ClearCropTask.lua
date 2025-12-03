ClearCropTask = ADInheritsFrom(AbstractTask)

ClearCropTask.debug = false
ClearCropTask.TARGET_DISTANCE_SIDE = 10
ClearCropTask.TARGET_DISTANCE_FRONT_STEP = 10
ClearCropTask.MAX_HARVESTER_DISTANCE = 50
ClearCropTask.WAIT_TIME = 10000
ClearCropTask.DRIVE_TIME = 30000
ClearCropTask.STUCK_TIME = 60000
ClearCropTask.STATE_CLEARING_FIRST = {}
ClearCropTask.STATE_CLEARING_SECOND = {}
ClearCropTask.STATE_REVERSING = {}
ClearCropTask.STATE_WAITING = {}

ClearCropTask.LEFT = 1
ClearCropTask.RIGHT = -1

function ClearCropTask:new(vehicle, harvester)
    local o = ClearCropTask:create()
    o.vehicle = vehicle
    o.harvester = harvester
    o.waitTimer = AutoDriveTON:new()
    o.driveTimer = AutoDriveTON:new()
    o.stuckTimer = AutoDriveTON:new()
    o.state = ClearCropTask.STATE_WAITING
    o.reverseStartLocation = nil
    o.vehicleTrainLength = AutoDrive.getTractorTrainLength(vehicle, true, false)
    ClearCropTask.setStateNames(o)
    return o
end

function ClearCropTask:setUp()
    ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:setUp")
    local leftBlocked = self.vehicle.ad.sensors.leftSensorFruit:pollInfo() or self.vehicle.ad.sensors.leftSensor:pollInfo()
    local rightBlocked = self.vehicle.ad.sensors.rightSensorFruit:pollInfo() or self.vehicle.ad.sensors.rightSensor:pollInfo()

    local leftFrontBlocked = self.vehicle.ad.sensors.leftFrontSensorFruit:pollInfo() or self.vehicle.ad.sensors.leftFrontSensor:pollInfo()
    local rightFrontBlocked = self.vehicle.ad.sensors.rightFrontSensorFruit:pollInfo() or self.vehicle.ad.sensors.rightFrontSensor:pollInfo()

    leftBlocked = leftBlocked or leftFrontBlocked
    rightBlocked = rightBlocked or rightFrontBlocked

    local cleartowards = ClearCropTask.RIGHT
    if leftBlocked and rightBlocked then
        cleartowards = ClearCropTask.RIGHT
    elseif leftBlocked then
        cleartowards = ClearCropTask.RIGHT
    elseif rightBlocked then
        cleartowards = ClearCropTask.LEFT
    end

    self.wayPoints = {}

    if self.harvester then
        local distance = AutoDrive.getDistanceBetween(self.vehicle, self.harvester)
        ClearCropTask.debugMsg(self.harvester, "ClearCropTask:setUp distance %.0f"
            , distance
        )
    end
    if self.harvester and AutoDrive.getDistanceBetween(self.vehicle, self.harvester) < ClearCropTask.MAX_HARVESTER_DISTANCE then
        local leftSensorFruit = self.harvester.ad.sensors.leftSensorFruit:pollInfo()
        local leftFrontSensorFruit = self.harvester.ad.sensors.leftFrontSensorFruit:pollInfo()
        local leftFree = not (leftSensorFruit or leftFrontSensorFruit)
        local rightFrontSensorFruit = self.harvester.ad.sensors.rightFrontSensorFruit:pollInfo()
        local rightSensorFruit = self.harvester.ad.sensors.rightSensorFruit:pollInfo()
        local rightFree = not (rightSensorFruit or rightFrontSensorFruit)
        local offfsetX = 0
        if leftFree then
            offfsetX = 3
        elseif rightFree  then
            offfsetX = -3
        end
        table.insert(self.wayPoints, AutoDrive.createWayPointRelativeToVehicle(self.harvester, offfsetX, self.vehicleTrainLength * 1))
        table.insert(self.wayPoints, AutoDrive.createWayPointRelativeToVehicle(self.harvester, offfsetX, self.vehicleTrainLength * 2))
        table.insert(self.wayPoints, AutoDrive.createWayPointRelativeToVehicle(self.harvester, offfsetX, self.vehicleTrainLength * 3))
        table.insert(self.wayPoints, AutoDrive.createWayPointRelativeToVehicle(self.harvester, offfsetX, self.vehicleTrainLength * 4))
    else
        table.insert(self.wayPoints, AutoDrive.createWayPointRelativeToVehicle(self.vehicle, (ClearCropTask.TARGET_DISTANCE_SIDE / 2) * cleartowards, ClearCropTask.TARGET_DISTANCE_FRONT_STEP * 0.5))
        table.insert(self.wayPoints, AutoDrive.createWayPointRelativeToVehicle(self.vehicle, ClearCropTask.TARGET_DISTANCE_SIDE * cleartowards, ClearCropTask.TARGET_DISTANCE_FRONT_STEP * 1))
        table.insert(self.wayPoints, AutoDrive.createWayPointRelativeToVehicle(self.vehicle, ClearCropTask.TARGET_DISTANCE_SIDE * cleartowards, ClearCropTask.TARGET_DISTANCE_FRONT_STEP * 2))
        table.insert(self.wayPoints, AutoDrive.createWayPointRelativeToVehicle(self.vehicle, ClearCropTask.TARGET_DISTANCE_SIDE * cleartowards, ClearCropTask.TARGET_DISTANCE_FRONT_STEP * 3))
        table.insert(self.wayPoints, AutoDrive.createWayPointRelativeToVehicle(self.vehicle, ClearCropTask.TARGET_DISTANCE_SIDE * cleartowards, ClearCropTask.TARGET_DISTANCE_FRONT_STEP * 4))
    end
    self.vehicle.ad.drivePathModule:setWayPoints(self.wayPoints)
end

function ClearCropTask:update(dt)
    if self.lastState ~= self.state then
        ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:update %s -> %s", tostring(self:getStateName(self.lastState)), tostring(self:getStateName()))
        self.lastState = self.state
    end

    -- Check if the driver and trailers have left the crop yet
    -- TODO: due to missing doku for FSDensityMapUtil.getFruitArea this is not working properly, so deactivated until a suitable solution is found
    -- if not AutoDrive.isVehicleOrTrailerInCrop(self.vehicle, true) then
    --     ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:update not isVehicleOrTrailerInCrop")
    --     self:finished()
    --     return
    -- end
    self.stuckTimer:timer(true, ClearCropTask.STUCK_TIME, dt)
    if self.stuckTimer:done() then
        ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:update stuckTimer:done")
        self:finished()
        return
    end

    if self.state == ClearCropTask.STATE_WAITING then
        self.waitTimer:timer(true, ClearCropTask.WAIT_TIME, dt)
        if self.waitTimer:done() then
            ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:update STATE_WAITING - done waiting - clear now...")
            self:resetAllTimers()
            self.vehicle.ad.drivePathModule:setWayPoints(self.wayPoints)
            self.state = ClearCropTask.STATE_CLEARING_FIRST
            return
        end
    elseif self.state == ClearCropTask.STATE_CLEARING_FIRST then
        self.driveTimer:timer(true, ClearCropTask.DRIVE_TIME, dt)
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:update 1 isTargetReached")
            self:finished()
            return
        elseif self.driveTimer:done() then
            ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:update 1 driveTimer:done")
            self:resetAllTimers()
            local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
            self.reverseStartLocation = {x = x, y = y, z = z}
            self.state = ClearCropTask.STATE_REVERSING
            return
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end
    elseif self.state == ClearCropTask.STATE_REVERSING then
        local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
        local distanceToReversStart = MathUtil.vector2Length(x - self.reverseStartLocation.x, z - self.reverseStartLocation.z)
        if (g_updateLoopIndex % 60 == 0) then
            ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:update distanceToReversStart %.0f"
            , distanceToReversStart
            )
        end
        if distanceToReversStart > 20 then
            ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:update distanceToReversStart > 20")
            self:resetAllTimers()
            self.vehicle.ad.drivePathModule:setWayPoints(self.wayPoints)
            self.state = ClearCropTask.STATE_CLEARING_SECOND
            return
        else
            self.vehicle.ad.specialDrivingModule:driveReverse(dt, 15, 1, self.vehicle.ad.trailerModule:canBeHandledInReverse())
        end
    elseif self.state == ClearCropTask.STATE_CLEARING_SECOND then
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:update 2 isTargetReached")
            self:finished()
            return
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end
    end
end

function ClearCropTask:abort()
    ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:abort")
end

function ClearCropTask:finished()
    ClearCropTask.debugMsg(self.vehicle, "ClearCropTask:finished")
    self.vehicle.ad.taskModule:setCurrentTaskFinished()
end

function ClearCropTask:setStateNames()
    if self.statesToNames == nil then
        self.statesToNames = {}
        for name, id in pairs(ClearCropTask) do
            if string.sub(name, 1, 6) == "STATE_" then
                self.statesToNames[id] = name
            end
        end
    end
end

function ClearCropTask:getStateName(state)
    local requestedState = state
    if requestedState == nil then
        requestedState = self.state
    end
    if requestedState == nil then
        Logging.error("[AD] ClearCropTask: Could not find name for state ->%s<- !", tostring(requestedState))
    end
    return self.statesToNames[requestedState] or ""
end

function ClearCropTask:resetAllTimers()
    -- self.stuckTimer:timer(false) -- stuckTimer reset by speed changes
    self.waitTimer:timer(false)
    self.driveTimer:timer(false)
end

function ClearCropTask:getI18nInfo()
    local text = "$l10n_AD_task_clearcrop;"
    if self.state == ClearCropTask.STATE_CLEARING_FIRST then
        text = text .. " - 1/2"
    elseif self.state == ClearCropTask.STATE_CLEARING_SECOND then
        text = text .. " - 2/2"
    elseif self.state == ClearCropTask.STATE_REVERSING then
        text = text .. " - " .. "$l10n_AD_task_reversing_from_combine;"
    elseif self.state == ClearCropTask.STATE_WAITING then
        text = text .. " - " .. "$l10n_AD_task_waiting_for_room;"
    end
    return text
end

function ClearCropTask.debugMsg(vehicle, debugText, ...)
    if ClearCropTask.debug == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    else
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_COMBINEINFO, debugText, ...)
    end
end
