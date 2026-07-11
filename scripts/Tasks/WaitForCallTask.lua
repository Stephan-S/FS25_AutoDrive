WaitForCallTask = ADInheritsFrom(AbstractTask)
WaitForCallTask.HELPER_ZONE_CHECK_TIME = 1000
WaitForCallTask.POST_CLEAR_COOLDOWN = 10000

-- zoneCheckDelay (optional, ms): suppresses the helper-zone check for a settling period. Used
-- for the follow-up wait task after a ClearHarvesterZoneTask so a parked spot near the zone
-- edge doesn't re-trigger the next escape the moment the harvester inches forward - without a
-- cooldown the pair escaped in a loop, chasing the moving zone across the field.
function WaitForCallTask:new(vehicle, zoneCheckDelay)
    local o = WaitForCallTask:create()
    o.vehicle = vehicle
    o.helperZoneCheckTimer = AutoDriveTON:new()
    o.zoneCheckDelay = zoneCheckDelay or 0
    return o
end

function WaitForCallTask:setUp()
    ADHarvestManager:registerAsUnloader(self.vehicle)
    self.vehicle.ad.specialDrivingModule:stopVehicle()
end

function WaitForCallTask:findBlockingHarvester()
    local checked = {}
    local assignedHarvester = nil
    local unloadMode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
    if unloadMode ~= nil then
        assignedHarvester = unloadMode.combine
    end
    -- Mode assignment survives some registry transitions. Check it first so helper-zone safety
    -- cannot disappear merely because HarvestManager currently rebuilt/unregistered its list.
    local lists = {{assignedHarvester}, ADHarvestManager.harvesters or {}, ADHarvestManager.idleHarvesters or {}}
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
    if self.zoneCheckDelay > 0 then
        self.zoneCheckDelay = self.zoneCheckDelay - dt
        return
    end
    if self.helperZoneCheckTimer:timer(true, WaitForCallTask.HELPER_ZONE_CHECK_TIME, dt) then
        self.helperZoneCheckTimer:timer(false)
        local harvester, helperZone = self:findBlockingHarvester()
        if harvester ~= nil then
            -- Isolated task chain: clear helper zone, then resume waiting without advancing mode.
            ADHarvestManager:unregisterAsUnloader(self.vehicle)
            self.vehicle.ad.taskModule:addTask(ClearHarvesterZoneTask:new(self.vehicle, harvester, helperZone))
            self.vehicle.ad.taskModule:addTask(WaitForCallTask:new(self.vehicle, WaitForCallTask.POST_CLEAR_COOLDOWN))
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
