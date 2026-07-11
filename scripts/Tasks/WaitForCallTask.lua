WaitForCallTask = ADInheritsFrom(AbstractTask)
WaitForCallTask.HELPER_ZONE_CHECK_TIME = 5000

function WaitForCallTask:new(vehicle)
    local o = WaitForCallTask:create()
    o.vehicle = vehicle
    o.helperZoneCheckTimer = AutoDriveTON:new()
    return o
end

function WaitForCallTask:setUp()
    ADHarvestManager:registerAsUnloader(self.vehicle)
    self.vehicle.ad.specialDrivingModule:stopVehicle()
end

function WaitForCallTask:findBlockingHarvester()
    local checked = {}
    local lists = {ADHarvestManager.harvesters or {}}
    for _, harvesters in pairs(lists) do
        for _, harvester in pairs(harvesters) do
            if harvester ~= nil and not checked[harvester] and harvester.components ~= nil and harvester.components[1] ~= nil then
                checked[harvester] = true
                local helperZone = AutoDrive.getCombineHelperZone(harvester)
                if AutoDrive.isVehicleTrainInHelperZone(self.vehicle, helperZone) then
                    return harvester, helperZone
                end
            end
        end
    end
    return nil, nil
end

function WaitForCallTask:update(dt)
    self.vehicle.ad.specialDrivingModule:stopVehicle()
    self.vehicle.ad.specialDrivingModule:update(dt)
    if self.helperZoneCheckTimer:timer(true, WaitForCallTask.HELPER_ZONE_CHECK_TIME, dt) then
        self.helperZoneCheckTimer:timer(false)
        local harvester, helperZone = self:findBlockingHarvester()
        if harvester ~= nil then
            -- Isolated task chain: clear helper zone, then resume waiting without advancing mode.
            ADHarvestManager:unregisterAsUnloader(self.vehicle)
            self.vehicle.ad.taskModule:addTask(ClearHarvesterZoneTask:new(self.vehicle, harvester, helperZone))
            self.vehicle.ad.taskModule:addTask(WaitForCallTask:new(self.vehicle))
            self.vehicle.ad.taskModule:setCurrentTaskFinished(ADTaskModule.DONT_PROPAGATE)
        end
    end
end

function WaitForCallTask:abort()
end

function WaitForCallTask:finished()
    self.vehicle.ad.taskModule:setCurrentTaskFinished(self.propagate)
end

function WaitForCallTask:getInfoText()
    return g_i18n:getText("AD_task_wait_for_call")
end

function WaitForCallTask:getI18nInfo()
    return "$l10n_AD_task_wait_for_call;"
end
