ADTaskModule = {}

ADTaskModule.DONT_PROPAGATE = 1

function ADTaskModule:new(vehicle)
    local o = {}
    setmetatable(o, self)
    self.__index = self
    o.vehicle = vehicle
    o.tasks = Queue:new()
    ADTaskModule.reset(o)
    return o
end

function ADTaskModule:reset()
    while self.tasks:Count() > 0 do
        local task = self.tasks:Dequeue()
        if task.doRestart ~= nil then
            task:doRestart()
            break
        end
    end

    self.tasks:Clear()
    self.activeTask = nil
    self:onTaskChange()
    self.lastTaskInfo = ""
end

function ADTaskModule:addTask(newTask)
    self.tasks:Enqueue(newTask)
end

function ADTaskModule:getActiveTask()
    return self.activeTask
end

function ADTaskModule:setCurrentTaskFinished(stoppedFlag)
    if stoppedFlag == nil or stoppedFlag ~= ADTaskModule.DONT_PROPAGATE then
        if self.currentMode == nil then
            self.currentMode = self.vehicle.ad.stateModule:getCurrentMode()
        end
        if self.currentMode and self.currentMode.handleFinishedTask then
            -- call handleFinishedTask for the mode it was initiated
            self.currentMode:handleFinishedTask()
        end
    end

    self.activeTask = nil

    if not self.vehicle.spec_locomotive then
        self:RefuelIfNeeded()
        self:RepairIfNeeded()
    end

    -- No refuel needed or no refuel trigger available
    if self.activeTask == nil then
        self.activeTask = self.tasks:Dequeue()
    end

    if self.activeTask ~= nil then
        self:onTaskChange()
        self.activeTask:setUp()
        self.currentMode = self.vehicle.ad.stateModule:getCurrentMode()
    end
end

function ADTaskModule:abortCurrentTask(abortMessage)
    if abortMessage ~= nil then
        AutoDrive.printMessage(self.vehicle, abortMessage)
    end
    if self.activeTask ~= nil then
        self.activeTask:abort()
    end
    self.activeTask = nil
    self:onTaskChange()
end

function ADTaskModule:abortAllTasks()
    if self.activeTask ~= nil then
        self.activeTask:abort()
    end
    self.tasks:Clear()
    self.activeTask = nil
    self:onTaskChange()
end

function ADTaskModule:stopAndRestartAD()
    self:abortAllTasks()
    self:addTask(StopAndDisableADTask:new(self.vehicle, ADTaskModule.DONT_PROPAGATE, true))
end

function ADTaskModule:update(dt)
    if self.activeTask ~= nil and self.activeTask.update ~= nil then
        local taskInfo = self.activeTask:getI18nInfo()
        self.activeTask:update(dt)
        self.vehicle.ad.stateModule:getCurrentMode():monitorTasks(dt)
        if self.lastTaskInfo ~= taskInfo then
            self:onTaskInfoChange(taskInfo)
        end
    else
        if not self.vehicle.spec_locomotive then
            self:RefuelIfNeeded()
            self:RepairIfNeeded()
        end
    
        -- No refuel needed or no refuel trigger available
        if self.activeTask == nil then
            self.activeTask = self.tasks:Dequeue()
        end
        
        if self.activeTask ~= nil then
            self.activeTask:setUp()
            self.currentMode = self.vehicle.ad.stateModule:getCurrentMode()
        end
        self:onTaskChange()
    end
end

function ADTaskModule:hasToRefuel()
    local ret = false
    if AutoDrive.getSetting("autoRefuel", self.vehicle) or self.vehicle.ad.onRouteToRefuel then
        local refuelFillTypes = AutoDrive.getRequiredRefuels(self.vehicle, self.vehicle.ad.onRouteToRefuel)
        if #refuelFillTypes > 0 then
            ret = true
        end
    end
    return ret
end

function ADTaskModule:RefuelIfNeeded()
    if self:hasToRefuel() then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "ADTaskModule:RefuelIfNeeded hasToRefuel")
        local refuelDestinationMarkerID = ADTriggerManager.getClosestRefuelDestination(self.vehicle, self.vehicle.ad.onRouteToRefuel)
        if refuelDestinationMarkerID ~= nil then
            ADHarvestManager:unregisterAsUnloader(self.vehicle)
            self.followingUnloader = nil
            self.combine = nil
            self.vehicle.ad.onRouteToRefuel = true
            self.activeTask = RefuelTask:new(self.vehicle, ADGraphManager:getMapMarkerById(refuelDestinationMarkerID).id)
        else
            self.vehicle.ad.isStoppingWithError = true
            self.vehicle:stopAutoDrive()
            AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_No_Refuel_Station;", 5000, self.vehicle.ad.stateModule:getName())
        end
    end
end

function ADTaskModule:hasToRepair()
    local repairNeeded = false
    if self.vehicle.ad.onRouteToRepair then
        -- repair is forced by user or CP, so send vehicle to workshop independent of damage level
        return true
    end
    if AutoDrive.getSetting("autoRepair", self.vehicle) then
        local attachedObjects = AutoDrive.getAllImplements(self.vehicle, true)
        for _, attachedObject in pairs(attachedObjects) do
            repairNeeded = repairNeeded or (attachedObject.spec_wearable ~= nil and attachedObject.spec_wearable.damage > 0.6)
        end
    end

    return repairNeeded
end

function ADTaskModule:RepairIfNeeded()
    if self:hasToRepair() then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "ADTaskModule:RepairIfNeeded hasToRepair")
        local repairDestinationMarkerNodeID = AutoDrive:getClosestRepairTrigger(self.vehicle)
        if repairDestinationMarkerNodeID ~= nil then
            ADHarvestManager:unregisterAsUnloader(self.vehicle)
            self.followingUnloader = nil
            self.combine = nil
            self.activeTask = RepairTask:new(self.vehicle, repairDestinationMarkerNodeID.marker)
        else
            --self.vehicle.ad.isStoppingWithError = true
            --self.vehicle:stopAutoDrive()
            AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_No_Repair_Station;", 5000, self.vehicle.ad.stateModule:getName())
        end
    end
end
    

function ADTaskModule:onTaskChange()
    local taskInfo = ""
    if self.activeTask ~= nil then
        taskInfo = self.activeTask:getI18nInfo()
    end
    if self.lastTaskInfo ~= taskInfo then
        self:onTaskInfoChange(taskInfo)
    end
end

function ADTaskModule:onTaskInfoChange(taskInfo)
    self.vehicle.ad.stateModule:setCurrentTaskInfo(taskInfo)
    self.lastTaskInfo = taskInfo
end

function ADTaskModule:getNumberOfTasks()
    return self.tasks:Count()
end
