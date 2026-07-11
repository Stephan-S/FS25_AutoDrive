ClearHarvesterZoneTask = ADInheritsFrom(AbstractTask)

ClearHarvesterZoneTask.STATE_PLANNING = {}
ClearHarvesterZoneTask.STATE_DRIVING = {}
ClearHarvesterZoneTask.STATE_PRE_REVERSING = {}

ClearHarvesterZoneTask.MAX_RADIUS = 170
ClearHarvesterZoneTask.MAX_CELLS = 2400
ClearHarvesterZoneTask.TARGET_FRUIT_CLEARANCE = 2
ClearHarvesterZoneTask.PRE_REVERSE_DISTANCE = 8
ClearHarvesterZoneTask.STALL_TIME = 8000

function ClearHarvesterZoneTask:new(vehicle, harvester, helperZone)
    local o = ClearHarvesterZoneTask:create()
    o.vehicle = vehicle
    o.harvester = harvester
    o.helperZone = helperZone
    o.escapeJob = nil
    o.reverseStartLocation = nil
    o.stallTimer = AutoDriveTON:new()
    o.state = ClearHarvesterZoneTask.STATE_PLANNING
    return o
end

function ClearHarvesterZoneTask:startPlanning()
    -- Zone is snapshotted when infringement is detected. This keeps search geometry stable even
    -- while the harvester attempts to resume work.
    self.escapeJob = ADEscapeCourseGenerator.begin(self.vehicle, ADEscapeCourseGenerator.TARGET_FRUIT_FREE, {
        exclusionZone = AutoDrive.getCombineExclusionZone(self.harvester),
        combine = self.harvester,
        helperZone = self.helperZone,
        targetFruitClearance = ClearHarvesterZoneTask.TARGET_FRUIT_CLEARANCE,
        maxRadius = ClearHarvesterZoneTask.MAX_RADIUS,
        maxCells = ClearHarvesterZoneTask.MAX_CELLS
    })
    self.state = ClearHarvesterZoneTask.STATE_PLANNING
    self.stallTimer:timer(false)
end

function ClearHarvesterZoneTask:setUp()
    local vehicleX, _, vehicleZ = getWorldTranslation(self.vehicle.components[1].node)
    local localX = AutoDrive.getHelperZoneLocalPosition(self.helperZone, vehicleX, vehicleZ)
    local sideSign = localX >= 0 and 1 or -1
    local vehicleForwardX, _, vehicleForwardZ = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    if math.abs(localX) < 0.5 then
        local rightAlignment = vehicleForwardX * self.helperZone.rightX + vehicleForwardZ * self.helperZone.rightZ
        sideSign = rightAlignment >= 0 and 1 or -1
    end
    local sideDirectionX = self.helperZone.rightX * sideSign
    local sideDirectionZ = self.helperZone.rightZ * sideSign
    local sideAlignment = vehicleForwardX * sideDirectionX + vehicleForwardZ * sideDirectionZ
    local canReverse = self.vehicle.ad.trailerModule:canBeHandledInReverse()
    local rearBlocked = self.vehicle.ad.sensors.rearSensor:pollInfo()

    if sideAlignment < -0.5 and canReverse and not rearBlocked then
        self.reverseStartLocation = {x = vehicleX, z = vehicleZ}
        self.state = ClearHarvesterZoneTask.STATE_PRE_REVERSING
    else
        self:startPlanning()
    end
end

function ClearHarvesterZoneTask:update(dt)
    if self.harvester == nil or self.harvester.components == nil or self.harvester.components[1] == nil
        or g_currentMission.nodeToObject[self.harvester.components[1].node] == nil then
        self:finished()
        return
    end

    if self.state == ClearHarvesterZoneTask.STATE_PRE_REVERSING then
        local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
        local reverseDistance = MathUtil.vector2Length(x - self.reverseStartLocation.x, z - self.reverseStartLocation.z)
        if reverseDistance >= ClearHarvesterZoneTask.PRE_REVERSE_DISTANCE
            or not AutoDrive.isVehicleTrainInHelperZone(self.vehicle, self.helperZone)
            or self.vehicle.ad.sensors.rearSensor:pollInfo() then
            self:startPlanning()
        else
            self.vehicle.ad.specialDrivingModule:driveReverse(dt, 8, 1, true)
        end
    elseif self.state == ClearHarvesterZoneTask.STATE_PLANNING then
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        self.escapeJob:update()
        if self.escapeJob:isFinished() then
            local course = self.escapeJob:getCourse()
            self.escapeJob = nil
            if course ~= nil then
                self.vehicle.ad.drivePathModule:setWayPoints(course)
                self.state = ClearHarvesterZoneTask.STATE_DRIVING
            else
                -- Stay safe: return to waiting and let its delayed zone check retry later.
                self:finished()
            end
        end
    elseif self.state == ClearHarvesterZoneTask.STATE_DRIVING then
        self.stallTimer:timer(self.vehicle.lastSpeedReal <= 0.0002, ClearHarvesterZoneTask.STALL_TIME, dt)
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            self:finished()
        elseif self.stallTimer:done() then
            self.stallTimer:timer(false)
            local rearBlocked = self.vehicle.ad.sensors.rearSensor:pollInfo()
            if self.vehicle.ad.trailerModule:canBeHandledInReverse() and not rearBlocked then
                local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
                self.reverseStartLocation = {x = x, z = z}
                self.state = ClearHarvesterZoneTask.STATE_PRE_REVERSING
            else
                self:finished()
            end
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end
    end
end

function ClearHarvesterZoneTask:abort()
    self.escapeJob = nil
end

function ClearHarvesterZoneTask:finished()
    self.vehicle.ad.taskModule:setCurrentTaskFinished(ADTaskModule.DONT_PROPAGATE)
end

function ClearHarvesterZoneTask:getI18nInfo()
    if self.state == ClearHarvesterZoneTask.STATE_PRE_REVERSING then
        return "$l10n_AD_task_reversing_from_combine; - Helferzone"
    end
    if self.state == ClearHarvesterZoneTask.STATE_PLANNING then
        return "$l10n_AD_task_pathfinding; - Helferzone"
    end
    return "$l10n_AD_task_clearcrop; - Helferzone"
end
