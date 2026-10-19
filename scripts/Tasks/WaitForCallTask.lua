WaitForCallTask = ADInheritsFrom(AbstractTask)

-- states used to park inside the field before waiting (see ADFieldPark, scripts/Modules/FieldParkModule.lua)
WaitForCallTask.STATE_CHECK = 1
WaitForCallTask.STATE_PATHPLANNING = 2
WaitForCallTask.STATE_DRIVING = 3
WaitForCallTask.STATE_WAITING = 4
WaitForCallTask.STATE_SEARCHING = 5
WaitForCallTask.STATE_BACKING_OFF = 6

-- no spot in the field yet and the nose in the field (the AutoDrive waiting point can be at the very corner):
-- back off a little, out of the way of the harvester, a bit more on the next try
WaitForCallTask.BACK_OFF_DISTANCES = {2, 3}
WaitForCallTask.BACK_OFF_TIMEOUT = 15000

function WaitForCallTask:new(vehicle)
    local o = WaitForCallTask:create()
    o.vehicle = vehicle
    return o
end

function WaitForCallTask:setUp()
    ADHarvestManager:registerAsUnloader(self.vehicle)
    self.vehicle.ad.specialDrivingModule:stopVehicle()
    self.state = WaitForCallTask.STATE_WAITING
    self.failedPathFinder = 0
    self.backOffCount = 0
    -- not when Courseplay drives this vehicle: it only passes through and Courseplay takes it over in the field
    local cpDriven = AutoDrive:getIsCPActive(self.vehicle) or self.vehicle.ad.restartCP == true
    if ADFieldPark ~= nil and ADFieldPark.isEnabled() and self.vehicle.ad.trafficYieldModule ~= nil and not cpDriven then
        self.state = WaitForCallTask.STATE_CHECK
        self.checkTimer = AutoDriveTON:new()
    end
end

function WaitForCallTask:update(dt)
    if self.state == WaitForCallTask.STATE_CHECK then
        -- let the vehicle come to a stop before looking around
        if self.checkTimer:timer(true, 1000, dt) then
            local module = self.vehicle.ad.trafficYieldModule
            if self:hasLoad() then
                -- field parking is only for the arrival at the field (empty trailers). After unloading a harvester the
                -- unloader already stands on the harvested side: stay here
                module:log("field park: loaded, back from a harvester, staying here")
                self.state = WaitForCallTask.STATE_WAITING
            elseif ADFieldPark.isGoodWaitingPlace(self.vehicle) then
                module:log("field park: already a good waiting place")
                self.state = WaitForCallTask.STATE_WAITING
            else
                self.search = ADFieldPark.newSearch(self.vehicle)
                self.state = WaitForCallTask.STATE_SEARCHING
            end
        end
        self:hold(dt)
    elseif self.state == WaitForCallTask.STATE_SEARCHING then
        -- one step of the search per frame (see ADFieldPark.searchStep)
        local spots = ADFieldPark.searchStep(self.search)
        if spots ~= nil then
            if self.search.result ~= nil then
                self.vehicle.ad.trafficYieldModule:log("field park: %s", self.search.result)
            end
            self.search = nil
            self.spots = spots
            self.spotIndex = 0
            self:planToNextSpot()
        end
        self:hold(dt)
    elseif self.state == WaitForCallTask.STATE_PATHPLANNING then
        local pathFinder = self.vehicle.ad.pathFinderModule
        if pathFinder:hasFinished() then
            local wayPoints = pathFinder:getPath()
            if wayPoints == nil or #wayPoints <= 1 then
                self.failedPathFinder = self.failedPathFinder + 1
                self.vehicle.ad.trafficYieldModule:log("field park: no path to spot %d", self.spotIndex)
                self:planToNextSpot()
            elseif pathFinder.fallBackMode3 then
                -- the pathfinder only got there by driving through the crop: not worth it. Lifting the field
                -- restriction (fallback modes 1 and 2) is normal here: the unloader waits outside the field
                self.failedPathFinder = self.failedPathFinder + 1
                self.vehicle.ad.trafficYieldModule:log("field park: path to spot %d crosses the crop, rejected", self.spotIndex)
                self:planToNextSpot()
            else
                self.vehicle.ad.drivePathModule:setWayPoints(wayPoints)
                self.state = WaitForCallTask.STATE_DRIVING
            end
            if self.state ~= WaitForCallTask.STATE_DRIVING then
                self:hold(dt)
            end
        elseif self.planningTime > ADFieldPark.PLANNING_TIMEOUT then
            -- the pathfinder can search for a very long time: give up this spot
            pathFinder:reset()
            self.vehicle.ad.trafficYieldModule:log("field park: no path to spot %d in time", self.spotIndex)
            self:planToNextSpot()
            self:hold(dt)
        else
            self.planningTime = self.planningTime + dt
            pathFinder:update(dt)
            self:hold(dt)
        end
    elseif self.state == WaitForCallTask.STATE_BACKING_OFF then
        local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
        self.backOffTime = self.backOffTime + dt
        if MathUtil.vector2Length(x - self.backOffStart.x, z - self.backOffStart.z) >= self.backOffDistance
            or self.backOffTime > WaitForCallTask.BACK_OFF_TIMEOUT then
            self.vehicle.ad.trafficYieldModule:log("field park: backed off %.1f m", MathUtil.vector2Length(x - self.backOffStart.x, z - self.backOffStart.z))
            self.state = WaitForCallTask.STATE_WAITING
            self.retryTime = 0
            self:hold(dt)
        else
            self.vehicle.ad.specialDrivingModule:driveReverse(dt, 4, 1, false)
        end
    elseif self.state == WaitForCallTask.STATE_DRIVING then
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            ADFieldPark.setParked(self.vehicle, self.spots ~= nil and self.spots[self.spotIndex] or nil)
            self.state = WaitForCallTask.STATE_WAITING
            self:hold(dt)
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end
    else
        -- no spot reachable yet (crop still standing at the entrance, harvester not started...): look again later
        if self.retryTime ~= nil then
            self.retryTime = self.retryTime + dt
            if self.retryTime > ADFieldPark.RETRY_DELAY then
                self.retryTime = nil
                self.state = WaitForCallTask.STATE_CHECK
            end
        end
        self:hold(dt)
    end
end

-- starts the path planning to the next candidate spot, or waits here when none is left
function WaitForCallTask:planToNextSpot()
    local module = self.vehicle.ad.trafficYieldModule
    self.spotIndex = self.spotIndex + 1
    local spot = self.spots ~= nil and self.spots[self.spotIndex] or nil
    if spot == nil or self.spotIndex > ADFieldPark.MAX_TRIES then
        if self.spotIndex == 1 then
            module:log("field park: no free ground found, waiting here")
        else
            module:log("field park: no clean path to a free spot, waiting here")
        end
        self.state = WaitForCallTask.STATE_WAITING
        self.retryTime = 0
        self:startBackOff()
        return
    end
    module:log("field park: spot %d found at x=%.0f z=%.0f heading %.0f°%s", self.spotIndex, spot.x, spot.z, math.deg(math.atan2(spot.dirX, spot.dirZ)), spot.queued and " (in line with a parked unloader)" or "")
    local wayPoints = ADFieldPark.findDirectPath(self.vehicle, spot)
    if wayPoints ~= nil then
        module:log("field park: direct path to spot %d, %d points", self.spotIndex, #wayPoints)
        self.vehicle.ad.drivePathModule:setWayPoints(wayPoints)
        self.state = WaitForCallTask.STATE_DRIVING
        return
    end
    self.planningTime = 0
    self.vehicle.ad.pathFinderModule:reset()
    self.vehicle.ad.pathFinderModule:startPathPlanningTo({x = spot.x, y = spot.y, z = spot.z}, {x = spot.dirX, z = spot.dirZ})
    self.state = WaitForCallTask.STATE_PATHPLANNING
end

-- something in the trailers: the unloader comes from a harvester, not from the road
function WaitForCallTask:hasLoad()
    local trailers = AutoDrive.getAllUnits(self.vehicle)
    if trailers == nil or #trailers == 0 then
        return false
    end
    local fillLevel = AutoDrive.getAllFillLevels(trailers)
    return fillLevel ~= nil and fillLevel > 0.1
end

-- backs off straight when the nose of the tractor stands in the field, at most twice
function WaitForCallTask:startBackOff()
    local distance = WaitForCallTask.BACK_OFF_DISTANCES[self.backOffCount + 1]
    if distance == nil then
        return
    end
    local x, py, z = ADTrafficYieldModule.getPosition(self.vehicle)
    local dx, dz = ADTrafficYieldModule.getDirection(self.vehicle)
    local front = ADTrafficYieldModule.getTrainExtents(self.vehicle)
    local noseX, noseZ = x + dx * math.max(front - 0.5, 0), z + dz * math.max(front - 0.5, 0)
    if not AutoDrive.checkIsOnField(noseX, py, noseZ) then
        return
    end
    self.backOffCount = self.backOffCount + 1
    self.backOffDistance = distance
    self.backOffStart = {x = x, z = z}
    self.backOffTime = 0
    self.vehicle.ad.trafficYieldModule:log("field park: nose in the field, backing off %d m", distance)
    self.state = WaitForCallTask.STATE_BACKING_OFF
end

function WaitForCallTask:hold(dt)
    self.vehicle.ad.specialDrivingModule:stopVehicle()
    self.vehicle.ad.specialDrivingModule:update(dt)
end

function WaitForCallTask:abort()
    if ADFieldPark ~= nil then
        ADFieldPark.setParked(self.vehicle, nil)
    end
    -- called by a harvester while still planning: free the pathfinder for the next task
    if self.state == WaitForCallTask.STATE_PATHPLANNING then
        self.vehicle.ad.pathFinderModule:reset()
    end
end

function WaitForCallTask:finished()
    if ADFieldPark ~= nil then
        ADFieldPark.setParked(self.vehicle, nil)
    end
    self.vehicle.ad.taskModule:setCurrentTaskFinished(self.propagate)
end

function WaitForCallTask:getInfoText()
    if self.state == WaitForCallTask.STATE_PATHPLANNING or self.state == WaitForCallTask.STATE_DRIVING then
        return g_i18n:getText("AD_task_park_in_field")
    end
    return g_i18n:getText("AD_task_wait_for_call")
end

function WaitForCallTask:getI18nInfo()
    if self.state == WaitForCallTask.STATE_PATHPLANNING or self.state == WaitForCallTask.STATE_DRIVING then
        return "$l10n_AD_task_park_in_field;"
    end
    return "$l10n_AD_task_wait_for_call;"
end
