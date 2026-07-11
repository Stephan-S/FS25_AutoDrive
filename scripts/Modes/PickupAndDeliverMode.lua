PickupAndDeliverMode = ADInheritsFrom(AbstractMode)

PickupAndDeliverMode.STATE_INIT = 1
PickupAndDeliverMode.STATE_PICKUP = 2
PickupAndDeliverMode.STATE_DELIVER = 3
PickupAndDeliverMode.STATE_RETURN_TO_START = 4
PickupAndDeliverMode.STATE_FINISHED = 5
PickupAndDeliverMode.STATE_EXIT_FIELD = 6
PickupAndDeliverMode.STATE_DELIVER_TO_NEXT_TARGET = 7
PickupAndDeliverMode.STATE_PICKUP_FROM_NEXT_TARGET = 8
PickupAndDeliverMode.STATE_PARK = 9

function PickupAndDeliverMode:new(vehicle)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:new")
    local o = PickupAndDeliverMode:create()
    o.vehicle = vehicle
    o.trailers = nil
    PickupAndDeliverMode.reset(o)
    return o
end

function PickupAndDeliverMode:reset()
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:reset set STATE_INIT")
    self.state = PickupAndDeliverMode.STATE_INIT
    self.activeTask = nil
    self.trailers, self.trailerCount = AutoDrive.getAllUnits(self.vehicle)
    self.vehicle.ad.trailerModule:reset()
end

function PickupAndDeliverMode:start(user)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:start start self.state %s", tostring(self.state))

    if not self.vehicle.ad.stateModule:isActive() then
        self.vehicle:startAutoDrive()
    end

    if self.vehicle.ad.stateModule:getFirstMarker() == nil or self.vehicle.ad.stateModule:getSecondMarker() == nil then
        return
    end

    self:reset()
    local forced = not (user == AutoDrive.USER_PLAYER) -- force target delivery change if not started by player, i.e. CP
    self.activeTask = self:getNextTask(forced)
    if self.activeTask ~= nil then
        self.vehicle.ad.taskModule:addTask(self.activeTask)
    end
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:start end self.state %s", tostring(self.state))
end

function PickupAndDeliverMode:monitorTasks(dt)
end

function PickupAndDeliverMode:handleFinishedTask()
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode: handleFinishedTask")
    self.vehicle.ad.trailerModule:reset()
    self.activeTask = self:getNextTask(true)
    if self.activeTask ~= nil then
        self.vehicle.ad.taskModule:addTask(self.activeTask)
    end
end

function PickupAndDeliverMode:stop()
end

function PickupAndDeliverMode:continue()
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:continue")
    if self.activeTask ~= nil and (self.state == PickupAndDeliverMode.STATE_DELIVER or self.state == PickupAndDeliverMode.STATE_DELIVER_TO_NEXT_TARGET or self.state == PickupAndDeliverMode.STATE_PICKUP or self.state == PickupAndDeliverMode.STATE_PICKUP_FROM_NEXT_TARGET or self.state == PickupAndDeliverMode.STATE_EXIT_FIELD) then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:continue activeTask:continue")
        -- self.activeTask:continue()
        if self.activeTask ~= nil then
            self.vehicle.ad.taskModule:abortCurrentTask()
        end
        self.vehicle.ad.trailerModule:reset()
        self.activeTask = self:getNextTask(true)
        if self.activeTask ~= nil then
            self.vehicle.ad.taskModule:addTask(self.activeTask)
        end
    end
end

function PickupAndDeliverMode:getNextTask(forced)
    local nextTask
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask start forced %s self.state %s", tostring(forced), tostring(self.state))

    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local point = nil
    local distanceToStart = 0
    if self.vehicle.ad ~= nil and ADGraphManager.getWayPointById ~= nil and self.vehicle.ad.stateModule ~= nil and self.vehicle.ad.stateModule.getFirstMarker ~= nil and self.vehicle.ad.stateModule:getFirstMarker() ~= nil and self.vehicle.ad.stateModule:getFirstMarker() ~= 0 and self.vehicle.ad.stateModule:getFirstMarker().id ~= nil then
        point = ADGraphManager:getWayPointById(self.vehicle.ad.stateModule:getFirstMarker().id)
        if point ~= nil then
            distanceToStart = MathUtil.vector2Length(x - point.x, z - point.z)
        end
    end
    local fillLevel, _, filledToUnload = AutoDrive.getAllFillLevels(self.trailers)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask filledToUnload %s isCpFull %s", tostring(filledToUnload), tostring(self.vehicle.ad.isCpFull))
    filledToUnload = filledToUnload or self.vehicle.ad.isCpFull

    local setPickupTarget = function()
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:setPickupTarget")
        if ((AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYPICKUP or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders")) then
            -- multiple targets to handle
            -- get the next target to pickup from
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:setPickupTarget")
            local nextTarget = ADMultipleTargetsManager:getNextPickup(self.vehicle, forced)
            if nextTarget ~= nil then
                self.vehicle.ad.stateModule:setFirstMarker(nextTarget)
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:setPickupTarget setFirstMarker -> nextTarget getFirstMarkerName() %s", tostring(self.vehicle.ad.stateModule:getFirstMarkerName()))
            end
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:setPickupTarget getFirstMarkerName() %s", tostring(self.vehicle.ad.stateModule:getFirstMarkerName()))
        elseif self.vehicle.ad.stateModule:getAutomaticPickupTarget() then
            local pickupStation = ADTriggerManager:getBestPickupLocationFor(self.vehicle, self.trailers, self.vehicle.ad.stateModule:getFillType())
            if pickupStation ~= nil then
                local wpId = ADTriggerManager:getMarkerAtStation(pickupStation, self.vehicle)
                if wpId > 0 then                    
                    self.vehicle.ad.stateModule:setFirstMarkerByWayPointId(wpId)
                end
            end
        end
    end

    local setDeliverTarget = function()
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:setDeliverTarget")
        if ((AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYDELIVER or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders")) then
            -- multiple targets to handle
            -- get the next target to deliver to
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:setDeliverTarget")
            local nextTarget = ADMultipleTargetsManager:getNextTarget(self.vehicle, forced)
            if nextTarget ~= nil then
                self.vehicle.ad.stateModule:setSecondMarker(nextTarget)
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:setDeliverTarget setSecondMarker -> nextTarget getSecondMarkerName() %s", tostring(self.vehicle.ad.stateModule:getSecondMarkerName()))
            end
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:setDeliverTarget getSecondMarkerName() %s", tostring(self.vehicle.ad.stateModule:getSecondMarkerName()))
        else
            if self.vehicle.ad.stateModule:getAutomaticUnloadTarget() then
                local sellingStation = ADTriggerManager:getHighestPayingSellStation(self.vehicle.ad.stateModule:getFillType())
                if sellingStation ~= nil then
                    local wpId = ADTriggerManager:getMarkerAtStation(sellingStation, self.vehicle)
                    if wpId > 0 then                    
                        self.vehicle.ad.stateModule:setSecondMarkerByWayPointId(wpId)
                    end
                end
            end
        end
    end

    if self.state == PickupAndDeliverMode.STATE_INIT then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask STATE_INIT self.state %s distanceToStart %s", tostring(self.state), tostring(distanceToStart))
        if (AutoDrive.checkIsOnField(x, y, z) and ADGraphManager:requiresPathFinderToNetwork(self.vehicle)) or AutoDrive:getIsCPActive(self.vehicle) then
            -- is activated on a field - use ExitFieldTask to leave field according to setting
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask set STATE_EXIT_FIELD")
            self.state = PickupAndDeliverMode.STATE_EXIT_FIELD
        elseif filledToUnload then
            -- fill level above setting unload level
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask set STATE_PICKUP")
            self.state = PickupAndDeliverMode.STATE_PICKUP
        else
            -- fill capacity left - go to pickup
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask set STATE_DELIVER")
            self.state = PickupAndDeliverMode.STATE_DELIVER
        end
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask STATE_INIT end self.state %s", tostring(self.state))
    end

    if self.state == PickupAndDeliverMode.STATE_PICKUP_FROM_NEXT_TARGET then
        -- STATE_PICKUP_FROM_NEXT_TARGET - load at multiple targets
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask STATE_PICKUP_FROM_NEXT_TARGET")
        -- by default - go to unload
        self.vehicle.ad.stateModule:setLoopsDone(self.vehicle.ad.stateModule:getLoopsDone() + 1)
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask set STATE_PICKUP")
        self.state = PickupAndDeliverMode.STATE_PICKUP
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask STATE_PICKUP_FROM_NEXT_TARGET end")
    end

    if self.state == PickupAndDeliverMode.STATE_DELIVER_TO_NEXT_TARGET then
        -- STATE_DELIVER_TO_NEXT_TARGET - unload at multiple targets
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask STATE_DELIVER_TO_NEXT_TARGET")
        if fillLevel > 0.1 and ((AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYDELIVER or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders")) then
            -- if fill material left and multiple unload active - go to unload
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask set STATE_PICKUP")
            self.state = PickupAndDeliverMode.STATE_PICKUP
        else
            -- no fill material - go to load
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask set STATE_DELIVER")
            self.state = PickupAndDeliverMode.STATE_DELIVER
        end
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask STATE_DELIVER_TO_NEXT_TARGET end")
    end

    if self.state == PickupAndDeliverMode.STATE_DELIVER then
        -- STATE_DELIVER - drive to load destination
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask STATE_DELIVER loopsDone %s", tostring(self.vehicle.ad.stateModule:getLoopsDone()))
        -- if self.vehicle.ad.stateModule:getLoopCounter() == 0 or self.vehicle.ad.stateModule:getLoopsDone() < self.vehicle.ad.stateModule:getLoopCounter() or ((AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYPICKUP or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders")) then
        if self.vehicle.ad.stateModule:getLoopCounter() == 0 or self.vehicle.ad.stateModule:getLoopsDone() < self.vehicle.ad.stateModule:getLoopCounter() then
            -- until loops not finished or 0 - drive to load destination
            setPickupTarget()   -- if rotateTargets is set, set the next pickup target
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask LoadAtDestinationTask... getFirstMarkerName() %s", tostring(self.vehicle.ad.stateModule:getFirstMarkerName()))
            nextTask = LoadAtDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getFirstMarker().id)
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask set STATE_PICKUP_FROM_NEXT_TARGET")
            self.state = PickupAndDeliverMode.STATE_PICKUP_FROM_NEXT_TARGET
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask loopsDone %s", tostring(self.vehicle.ad.stateModule:getLoopsDone()))
        else
            -- if loops are finished - drive to park destination and stop AD
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask DriveToDestinationTask...")
            nextTask = ParkTask:new(self.vehicle, self.vehicle.ad.stateModule:getSecondMarker().id)
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask set STATE_PARK")
            self.state = PickupAndDeliverMode.STATE_PARK
        end
    elseif self.state == PickupAndDeliverMode.STATE_PICKUP then
        -- STATE_PICKUP - drive to unload destination
        setDeliverTarget()      -- if rotateTargets is set, set the next deliver target
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask STATE_PICKUP UnloadAtDestinationTask... getSecondMarkerName() %s", tostring(self.vehicle.ad.stateModule:getSecondMarkerName()))
        nextTask = UnloadAtDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getSecondMarker().id)
        self.vehicle.ad.isCpFull = false
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask set STATE_DELIVER_TO_NEXT_TARGET")
        self.state = PickupAndDeliverMode.STATE_DELIVER_TO_NEXT_TARGET
    elseif self.state == PickupAndDeliverMode.STATE_EXIT_FIELD then
        -- is activated on a field - use ExitFieldTask to leave field according to setting
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask STATE_EXIT_FIELD ExitFieldTask...")
        nextTask = ExitFieldTask:new(self.vehicle)
        if filledToUnload then
            -- fill level above setting unload level - drive to unload destination
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask set STATE_PICKUP")
            self.state = PickupAndDeliverMode.STATE_PICKUP
        else
            -- fill capacity left - go to pickup
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask set STATE_DELIVER")
            self.state = PickupAndDeliverMode.STATE_DELIVER
        end
    elseif self.state == PickupAndDeliverMode.STATE_RETURN_TO_START then
        -- job done - stop AD and tasks
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask STATE_RETURN_TO_START StopAndDisableADTask...")
        self.vehicle.ad.stateModule:setLoopsDone(0)
        nextTask = StopAndDisableADTask:new(self.vehicle, ADTaskModule.DONT_PROPAGATE)
        self.state = PickupAndDeliverMode.STATE_FINISHED
        AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.INFO, "$l10n_AD_Driver_of; %s $l10n_AD_has_reached; %s", 5000, self.vehicle.ad.stateModule:getName(), self.vehicle.ad.stateModule:getFirstMarkerName())
    elseif self.state == PickupAndDeliverMode.STATE_PARK then
        -- job done - drive to park position and stop AD and tasks
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask STATE_RETURN_TO_START StopAndDisableADTask...")
        self.vehicle.ad.stateModule:setLoopsDone(0)
        nextTask = StopAndDisableADTask:new(self.vehicle, ADTaskModule.DONT_PROPAGATE)
        self.state = PickupAndDeliverMode.STATE_FINISHED
        -- message for reached park position send by ParkTask as only there the correct destination is known
    else
        -- error path - should never appear!
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask self.state %s NO nextTask assigned !!!", tostring(self.state))
        self.vehicle.ad.stateModule:setLoopsDone(0)
        nextTask = StopAndDisableADTask:new(self.vehicle, ADTaskModule.DONT_PROPAGATE)
        self.state = PickupAndDeliverMode.STATE_FINISHED
        AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_has_reached; %s", 5000, self.vehicle.ad.stateModule:getName(), "???")
    end

    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:getNextTask end loopsDone %s self.state %s", tostring(self.vehicle.ad.stateModule:getLoopsDone()), tostring(self.state))
    return nextTask
end

function PickupAndDeliverMode:shouldUnloadAtTrigger()
    -- AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:shouldUnloadAtTrigger self.state %s ", tostring(self.state))
    return ((self.state == PickupAndDeliverMode.STATE_DELIVER) or (self.state == PickupAndDeliverMode.STATE_DELIVER_TO_NEXT_TARGET))
end

function PickupAndDeliverMode:shouldLoadOnTrigger()
    -- AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PickupAndDeliverMode:shouldLoadOnTrigger self.state %s distance to target %s", tostring(self.state), tostring(AutoDrive.getDistanceToTargetPosition(self.vehicle)))
    return ((self.state == PickupAndDeliverMode.STATE_PICKUP) or (self.state == PickupAndDeliverMode.STATE_PICKUP_FROM_NEXT_TARGET))
end

