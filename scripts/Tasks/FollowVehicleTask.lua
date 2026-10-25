FollowVehicleTask = ADInheritsFrom(AbstractTask)

FollowVehicleTask.TARGET_DISTANCE = 40

FollowVehicleTask.STATE_DRIVING = 1

function FollowVehicleTask:new(vehicle, targetVehicle)
    --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_COMBINEINFO, "FollowVehicleTask:new()")
    local o = FollowVehicleTask:create()
    o.vehicle = vehicle
    o.targetVehicle = targetVehicle
    o.state = FollowVehicleTask.STATE_DRIVING
    o.setFinishedNext = false
    o.trailers = nil
    return o
end

function FollowVehicleTask:setUp()
    --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_COMBINEINFO, "FollowVehicleTask:setUp()")
    if self.targetVehicle ~= nil and self.targetVehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getBreadCrumbs() ~= nil then
        local breadCrumbs = self.targetVehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getBreadCrumbs()
        local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
        local indexToAttachTo = nil
        for index, breadCrumb in ipairs(breadCrumbs.items) do
            local _, _, diffZ = AutoDrive.worldToLocal(self.vehicle, breadCrumb.x, breadCrumb.y, breadCrumb.z)
            if diffZ > 1 and MathUtil.vector2Length(x - breadCrumb.x, z - breadCrumb.z) < 15 then
                indexToAttachTo = index
                --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_COMBINEINFO, "FollowVehicleTask:setUp() Found breadcrumb to attach to during setup! Index: " .. index)
                break
            end
        end
        if indexToAttachTo ~= nil then
            --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_COMBINEINFO, "FollowVehicleTask:setUp() - removing items from Queue: " .. indexToAttachTo)
            for i = 1, (indexToAttachTo - 1) do
                breadCrumbs:Dequeue()
            end
            local path = self:getBreadCrumbsUntilFollowDistance()
            self.vehicle.ad.drivePathModule:setWayPoints(path)
        end
    end
    self.trailers, _ = AutoDrive.getAllUnits(self.vehicle)
    AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, false)
end

function FollowVehicleTask:getBreadCrumbsUntilFollowDistance()
    local toFollow = {}
    if self.targetVehicle ~= nil and self.targetVehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getBreadCrumbs() ~= nil then
        local breadCrumbs = self.targetVehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getBreadCrumbs()
        local index = 1
        while self:getBreadCrumbDistanceToEnd(index) > self.TARGET_DISTANCE do
            table.insert(toFollow, breadCrumbs:Dequeue())
        end
    end

    --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_COMBINEINFO, "FollowVehicleTask:getBreadCrumbsUntilFollowDistance() - #toFollow: " .. #toFollow)

    return toFollow
end

function FollowVehicleTask:getBreadCrumbDistanceToEnd(index)
    local distance = 0
    if self.targetVehicle ~= nil and self.targetVehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getBreadCrumbs() ~= nil then
        local breadCrumbs = self.targetVehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getBreadCrumbs()

        while index < #breadCrumbs.items do
            local wpStart = breadCrumbs.items[index]
            local wpNext = breadCrumbs.items[index + 1]
            distance = distance + MathUtil.vector2Length(wpStart.x - wpNext.x, wpStart.z - wpNext.z)
            index = index + 1
        end
    end

    return distance
end

function FollowVehicleTask:update(dt)
    -- the harvester or the 1st comes toward this follower: it steps aside (StepAsideTask) and comes back
    if not self.setFinishedNext and StepAsideTask ~= nil and StepAsideTask.hook(self.vehicle, self.targetVehicle, dt) then
        return
    end
    if self.vehicle.ad.drivePathModule:isTargetReached() or self.vehicle.ad.drivePathModule:getWayPoints() == nil or #self.vehicle.ad.drivePathModule:getWayPoints() == 0 then
        --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_COMBINEINFO, "FollowVehicleTask:FollowVehicleTask:update() - drivePathModule signals target reached")
        local newPath = self:getBreadCrumbsUntilFollowDistance()
        if newPath ~= nil and #newPath > 0 then
            --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_COMBINEINFO, "FollowVehicleTask:FollowVehicleTask:update() - got new path of length: " .. #newPath)
            self.vehicle.ad.drivePathModule:setWayPoints(newPath)
        else
            self.vehicle.ad.specialDrivingModule:stopVehicle()
            self.vehicle.ad.specialDrivingModule:update(dt)
            if self.vehicle.lastSpeedReal <= 0.0008 and (self.setFinishedNext or self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD].combine ~= nil) then
                --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_COMBINEINFO, "FollowVehicleTask:update() - stopped and received promotion -> finished")
                self:finished()
            end
        end
    else
        self.vehicle.ad.drivePathModule:update(dt)
    end
end

function FollowVehicleTask:signalPromotion()
    --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_COMBINEINFO, "FollowVehicleTask:signalPromotion()")
    self.setFinishedNext = true
end

function FollowVehicleTask:abort()
    self.targetVehicle.ad.modes[AutoDrive.MODE_UNLOAD]:unregisterFollowingUnloader()
end

function FollowVehicleTask:finished(propagate)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_COMBINEINFO, "FollowVehicleTask: FollowVehicleTask:finished()")
    self.targetVehicle.ad.modes[AutoDrive.MODE_UNLOAD]:unregisterFollowingUnloader()
    self.vehicle.ad.taskModule:setCurrentTaskFinished(propagate)
end

function FollowVehicleTask:getInfoText()
    return g_i18n:getText("AD_task_drive_to_vehicle")
end

function FollowVehicleTask:getI18nInfo()
    return "$l10n_AD_task_drive_to_vehicle;"
end
