ADTrainModule = {}

ADTrainModule.MIN_TARGET_DISTANCE = 2
ADTrainModule.LOAD_UNLOAD_SPEED = 10
ADTrainModule.BRAKE_FACTOR = 5
ADTrainModule.TRAINLENGTH_ADDITION = 20 -- consider length of locomotive and last trailer

function ADTrainModule:new(vehicle)
    AutoDrive.debugPrint(vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:new")
    local o = {}
    setmetatable(o, self)
    self.__index = self
    o.vehicle = vehicle
    ADTrainModule.init(o)
    return o
end

function ADTrainModule:init()
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:init")
    self.lastDistance = math.huge
end

function ADTrainModule:reset()
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:reset")

    self:init()
    local spec = self.vehicle.spec_locomotive
    if AutoDrive:getIsEntered(self.vehicle) then
        if spec and spec.state ~= Locomotive.STATE_MANUAL_TRAVEL_ACTIVE then
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:reset setLocomotiveState STATE_MANUAL_TRAVEL_ACTIVE from %s", tostring(spec.state))
            self.vehicle:setLocomotiveState(Locomotive.STATE_MANUAL_TRAVEL_ACTIVE)
        end
    else
        if spec and spec.state ~= Locomotive.STATE_MANUAL_TRAVEL_INACTIVE then
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:reset setLocomotiveState STATE_MANUAL_TRAVEL_INACTIVE from %s", tostring(spec.state))
            self.vehicle:setLocomotiveState(Locomotive.STATE_MANUAL_TRAVEL_INACTIVE)
        end
    end
    
    if self.vehicle:getIsMotorStarted() and not AutoDrive:getIsEntered(self.vehicle) then
        if self.setCruiseControlState then
            self:setCruiseControlState(Drivable.CRUISECONTROL_STATE_OFF)
            self:updateVehiclePhysics(0, 0, 0, 16)
            self.vehicle:raiseActive()
        end
        self.vehicle:stopMotor()
    end
    self.vehicle:raiseActive()
end

function ADTrainModule:setUp()
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:setUp")
    self.trailers = AutoDrive.getAllImplements(self.vehicle, true)
    self.lastTrailer, self.trainLength = self:getLastTrailer()
    -- disable train vehicles to be entered to unload
    for i, trailer in ipairs(self.trailers) do
        local spec = trailer.spec_dischargeable
        if spec then
            local dischargeNode = spec.currentDischargeNode
            if dischargeNode and dischargeNode.needsIsEntered then
                dischargeNode.needsIsEntered = false
            end
        end
    end
end

function ADTrainModule:setPathTo(destinationID)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:setPathTo destinationID %s", tostring(destinationID))

    local destination = ADGraphManager:getMapMarkerByWayPointId(destinationID)
    self.vehicle.ad.stateModule:setCurrentDestination(destination)
    self:setUp()
    self.vehicle:raiseActive()
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:setPathTo self.destinationID %s", tostring(self.destinationID))
end

function ADTrainModule:update(dt)
    if (g_updateLoopIndex % (60) == 0) then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:update")
    end

    local currentDestination = self.vehicle.ad.stateModule:getCurrentDestination()
    local currentDestinationID = currentDestination and currentDestination.id
    local wayPoint = ADGraphManager:getWayPointById(currentDestinationID)
    if not wayPoint then
        return
    end
    local spec = self.vehicle.spec_locomotive
    local speedReal = spec.speed * 3.6
    local brakeDistance = speedReal * 2

    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local distance = MathUtil.vector2Length(wayPoint.x - x, wayPoint.z - z)
    local shouldBrake = false

    self.vehicle.ad.specialDrivingModule:releaseVehicle()
    if self.vehicle.startMotor then
        if not self.vehicle:getIsMotorStarted() and self.vehicle:getCanMotorRun() and not self.vehicle.ad.specialDrivingModule:shouldStopMotor() then
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:update startMotor")
            self.vehicle:startMotor()
        end
    end

    if self.vehicle.spec_locomotive.state ~= Locomotive.STATE_MANUAL_TRAVEL_ACTIVE then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:update setLocomotiveState")
        self.vehicle:setLocomotiveState(Locomotive.STATE_MANUAL_TRAVEL_ACTIVE)
    end

    if distance < self.lastDistance then
        -- slow down when approaching to target
        if distance < brakeDistance then
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:update shouldBrake distance %s", tostring(distance))
            shouldBrake = true
        end
    end
    self.lastDistance = distance

    if distance < self.trainLength + ADTrainModule.TRAINLENGTH_ADDITION and speedReal > ADTrainModule.LOAD_UNLOAD_SPEED then
        -- slow down in destination range
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:update shouldBrake destination range distance %s", tostring(distance))
        shouldBrake = true
    end

    if shouldBrake then
        if (g_updateLoopIndex % (60) == 0) then
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:update shouldBrake speedReal %s", tostring(speedReal))
        end

        if self.vehicle.movingDirection > 0 then
            if math.abs(speedReal) > (2 * ADTrainModule.LOAD_UNLOAD_SPEED) then
                self.vehicle:updateVehiclePhysics(-ADTrainModule.BRAKE_FACTOR, 0, 0, dt)
            elseif math.abs(speedReal) > ADTrainModule.LOAD_UNLOAD_SPEED then
                self.vehicle:updateVehiclePhysics(-1, 0, 0, dt)
            end
        else
            -- it happens that the movingDirection becomes 0 or -1, so move away
            self.vehicle:updateVehiclePhysics(1, 0, 0, dt)
        end
    else
        if (g_updateLoopIndex % (60) == 0) then
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:update drive forward speedReal %s", tostring(speedReal))
        end
        -- drive forward
        self.vehicle:updateVehiclePhysics(1, 0, 0, dt)
    end
    self.vehicle:raiseActive()
end

function ADTrainModule:stopAndHoldVehicle(dt)
    local spec = self.vehicle.spec_locomotive
    local speedReal = spec.speed * 3.6
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node) 
    local distance = 12345
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:stopAndHoldVehicle speedReal %s", tostring(speedReal))

    local currentDestination = self.vehicle.ad.stateModule:getCurrentDestination()
    local currentDestinationID = currentDestination and currentDestination.id
    local wayPoint = ADGraphManager:getWayPointById(currentDestinationID)
    if wayPoint then
        if not self.lastTrailer then
            self.lastTrailer, self.trainLength = self:getLastTrailer()
        end
        if self.lastTrailer then
            local x, y, z = getWorldTranslation(self.lastTrailer.components[1].node)
            distance = MathUtil.vector2Length(wayPoint.x - x, wayPoint.z - z)
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:stopAndHoldVehicle distance %s", tostring(distance))
        end
    end

    if math.abs(speedReal) > 0.001 then
        if self.vehicle.movingDirection > 0 then
            self.vehicle:updateVehiclePhysics(-ADTrainModule.BRAKE_FACTOR, 0, 0, dt)
        end
    end
    if self.vehicle.ad and self.vehicle.ad.specialDrivingModule then
        self.vehicle.ad.specialDrivingModule.stoppedTimer:timer(math.abs(speedReal) < 1 and (self.vehicle.ad.trailerModule:getCanStopMotor()), 10000, dt)
        if self.vehicle.ad.specialDrivingModule.stoppedTimer:done() then
            self.vehicle.ad.specialDrivingModule.motorShouldBeStopped = true
            if self.vehicle.ad.specialDrivingModule:shouldStopMotor() and self.vehicle:getIsMotorStarted() and (not g_currentMission.missionInfo.automaticMotorStartEnabled) then
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:stopAndHoldVehicle stopMotor")

                if self.setCruiseControlState then
                    self:setCruiseControlState(Drivable.CRUISECONTROL_STATE_OFF)
                    self:updateVehiclePhysics(0, 0, 0, 16)
                    self:raiseActive()
                end
                self.vehicle:stopMotor()
            end
        end
    end
    self.vehicle:raiseActive()
end


function ADTrainModule:isTargetReached()
    if (g_updateLoopIndex % (60) == 0) then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:isTargetReached")
    end

    local ret = true
    local currentDestination = self.vehicle.ad.stateModule:getCurrentDestination()
    local currentDestinationID = currentDestination and currentDestination.id
    local wayPoint = ADGraphManager:getWayPointById(currentDestinationID)
    if wayPoint then
        if not self.lastTrailer then
            self.lastTrailer, self.trainLength = self:getLastTrailer()
        end
        if self.lastTrailer then
            local x, y, z = getWorldTranslation(self.lastTrailer.components[1].node)
            local distance = MathUtil.vector2Length(wayPoint.x - x, wayPoint.z - z)
            if (g_updateLoopIndex % (60) == 0) then
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:isTargetReached distance %s", tostring(distance))
            end
            ret = distance < ADTrainModule.MIN_TARGET_DISTANCE
            if ret then
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:isTargetReached ADTrainModule.MIN_TARGET_DISTANCE")
            end
        end
    end
    return ret
end

function ADTrainModule:isInRangeToLoadUnloadTarget()
    if (g_updateLoopIndex % (60) == 0) then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:isInRangeToLoadUnloadTarget")
    end

    if self.vehicle == nil or self.vehicle.ad == nil or self.vehicle.ad.stateModule == nil or self.vehicle.ad.drivePathModule == nil then
        return false
    end
    if not self.lastTrailer then
        self.lastTrailer, self.trainLength = self:getLastTrailer()
    end
    local ret = false
    ret =
            (
                ((self.vehicle.ad.stateModule:getCurrentMode():shouldLoadOnTrigger() == true) and AutoDrive.getDistanceToTargetPosition(self.lastTrailer) <= self.trainLength + ADTrainModule.TRAINLENGTH_ADDITION)
                or
                ((self.vehicle.ad.stateModule:getCurrentMode():shouldUnloadAtTrigger() == true) and AutoDrive.getDistanceToUnloadPosition(self.lastTrailer) <= self.trainLength + ADTrainModule.TRAINLENGTH_ADDITION)
            )

    if (g_updateLoopIndex % (60) == 0) then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:isInRangeToLoadUnloadTarget ret %s", tostring(ret))
    end

    return ret
end

function ADTrainModule:getLastTrailer()
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:getLastTrailer")
    local lastTrailer = nil
    local trainLength = 0
    if self.trailers == nil then
        self.trailers = AutoDrive.getAllImplements(self.vehicle, true)
    end
    if self.trailers then
        for i, trailer in ipairs(self.trailers) do
            if i == 1 then
                lastTrailer = trailer
            end
            trainLength = trainLength + trailer.size.length * 1.5
        end
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAINS, "ADTrainModule:getLastTrailer trainLength %s", tostring(trainLength))
    end
    return lastTrailer, trainLength
end
