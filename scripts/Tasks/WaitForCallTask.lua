WaitForCallTask = ADInheritsFrom(AbstractTask)

-- states used to park inside the field before waiting (see ADFieldPark, scripts/Modules/FieldParkModule.lua)
WaitForCallTask.STATE_CHECK = 1
WaitForCallTask.STATE_PATHPLANNING = 2
WaitForCallTask.STATE_DRIVING = 3
WaitForCallTask.STATE_WAITING = 4
WaitForCallTask.STATE_SEARCHING = 5
WaitForCallTask.STATE_BACKING_OFF = 6
WaitForCallTask.STATE_TO_ENTRANCE = 7 -- beside the road, a place is free in the field: back on the network to the field point first

-- no spot in the field yet and the nose in the field (the AutoDrive waiting point can be at the very corner):
-- back off a little, out of the way of the harvester, a bit more on the next try
WaitForCallTask.BACK_OFF_DISTANCES = {2, 3}
WaitForCallTask.BACK_OFF_TIMEOUT = 15000
-- field full and no clean way from the entrance to a roadside place (nose in the field, the road
-- behind): back off along the route it came by, then look again from the road
WaitForCallTask.ROUTE_BACK_OFF = 40        -- m backed off along the route
WaitForCallTask.ROUTE_BACK_OFF_TIMEOUT = 60000
WaitForCallTask.ENTRANCE_NEAR = 15         -- m: beside the road closer than this to the field point, the field search starts here

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

-- already standing at a good place (parked before the savegame was saved): stay, and hold this place for the
-- others, which queue behind
function WaitForCallTask:stayParked(module)
    local good, spot = ADFieldPark.isGoodWaitingPlace(self.vehicle)
    if not good then
        module:log("field park: current place not kept: %s", tostring(spot))
        return false
    end
    module:log("field park: already a good waiting place, staying here")
    self.parkedSpot = spot
    ADFieldPark.setParked(self.vehicle, spot, false)
    return true
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
                self.loaded = true
                self.state = WaitForCallTask.STATE_WAITING
            elseif self:stayParked(module) then
                self.state = WaitForCallTask.STATE_WAITING
            elseif self:goRoadsideIfFieldFull() then
                -- the field already holds its unloaders: wait beside the road
            else
                -- backed off along the road because the field had no place yet: looks again from the entrance
                self.search = ADFieldPark.newSearch(self.vehicle, self.noPlaceRoadside and self.noPlaceOrigin or nil)
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
            local search = self.search
            self.search = nil
            if self.probeFromRoadside then
                self.probeFromRoadside = nil
                if #spots > 0 and not search.waitSense then
                    self.noPlaceRoadside = nil
                    self:leaveRoadside()
                else
                    self.vehicle.ad.trafficYieldModule:log("field park: still no place in the field, staying beside the road")
                    self.state = WaitForCallTask.STATE_WAITING
                end
            elseif search.waitSense and #spots == 0 then
                -- the sense of the first headland is not known yet: wait beside the road until it is
                self.vehicle.ad.trafficYieldModule:log("field park: sense of the first headland not known yet, waiting beside the road")
                self:goRoadside(search.harvester)
            else
                self.spots = spots
                self.spotIndex = 0
                self:planToNextSpot()
            end
        end
        self:hold(dt)
    elseif self.state == WaitForCallTask.STATE_PATHPLANNING then
        local pathFinder = self.vehicle.ad.pathFinderModule
        if pathFinder:hasFinished() then
            local wayPoints = pathFinder:getPath()
            self.vehicle.ad.trafficYieldModule:log("field park: pathfinder for spot %d: search %.1f s, smoothing %.1f s",
                self.spotIndex, self.planningTime / 1000, (self.smoothingTime or 0) / 1000)
            if wayPoints == nil or #wayPoints <= 1 then
                self.failedPathFinder = self.failedPathFinder + 1
                self.vehicle.ad.trafficYieldModule:log("field park: no path to spot %d", self.spotIndex)
                self:planToNextSpot()
            elseif pathFinder.fallBackMode3 or self:pathCrossesCrop(wayPoints, pathFinder) then
                -- the pathfinder only got there by driving through the crop: not worth it. Lifting the field
                -- restriction (fallback modes 1 and 2) is normal here: the unloader waits outside the field.
                -- Its grid is coarse and follows the avoidFruit setting: the route is checked again every meter
                -- (a place on the far side must not be reached through the crop)
                self.failedPathFinder = self.failedPathFinder + 1
                self.vehicle.ad.trafficYieldModule:log("field park: path to spot %d crosses the crop, rejected", self.spotIndex)
                self:planToNextSpot()
            else
                wayPoints = ADFieldPark.appendLeadIn(wayPoints, self.leadIn)
                -- remembered for the next unloaders only when it ends along the border
                self.plannedPath = self.leadIn ~= nil and wayPoints or nil
                self.vehicle.ad.drivePathModule:setWayPoints(wayPoints)
                self.state = WaitForCallTask.STATE_DRIVING
            end
            if self.state ~= WaitForCallTask.STATE_DRIVING then
                self:hold(dt)
            end
        elseif pathFinder.isFinished and self.smoothingTime ~= nil and self.smoothingTime > ADFieldPark.SMOOTHING_TIMEOUT then
            -- path found, its smoothing does not end: give up this spot
            pathFinder:reset()
            self.vehicle.ad.trafficYieldModule:log("field park: path to spot %d found in %.1f s, smoothing not done after %.1f s, next spot",
                self.spotIndex, self.planningTime / 1000, self.smoothingTime / 1000)
            self:planToNextSpot()
            self:hold(dt)
        elseif not pathFinder.isFinished and self.planningTime > ADFieldPark.PLANNING_TIMEOUT then
            -- the pathfinder can search for a very long time: give up this spot
            pathFinder:reset()
            self.vehicle.ad.trafficYieldModule:log("field park: no path to spot %d in time", self.spotIndex)
            self:planToNextSpot()
            self:hold(dt)
        elseif pathFinder.isFinished then
            -- path found: the smoothing has its own time
            self.smoothingTime = (self.smoothingTime or 0) + dt
            pathFinder:update(dt)
            self:hold(dt)
        else
            self.planningTime = self.planningTime + dt
            pathFinder:update(dt)
            self:hold(dt)
        end
    elseif self.state == WaitForCallTask.STATE_BACKING_OFF then
        local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
        self.backOffTime = self.backOffTime + dt
        local backed = MathUtil.vector2Length(x - self.backOffStart.x, z - self.backOffStart.z)
        local target = self.backOffTargets ~= nil and self.backOffTargets[self.backOffIndex] or nil
        if backed >= self.backOffDistance or self.backOffTime > self.backOffTimeout
            or (self.backOffTargets ~= nil and target == nil) then
            self.vehicle.ad.trafficYieldModule:log("field park: backed off %.1f m%s", backed, self.backOffTargets ~= nil and " along the route" or "")
            if self.backOffTargets ~= nil then
                -- back on the road: the roadside places are looked for again from here
                self.backOffTargets = nil
                self.vehicle.ad.specialDrivingModule:releaseVehicle()
                self.state = WaitForCallTask.STATE_CHECK
                self.checkTimer = AutoDriveTON:new()
            else
                self.state = WaitForCallTask.STATE_WAITING
                self.retryTime = 0
            end
            self:hold(dt)
        elseif self.backOffTargets ~= nil and backed > WaitForCallTask.ROUTE_BACK_OFF_MIN
            and AutoDrive.checkIsOnField(x, getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 0, z), z) then
            -- backing along the route with a trailer drifts; as soon as the train is on a field again (the one opposite the entrance), it stops there
            self.vehicle.ad.trafficYieldModule:log("field park: backed off %.1f m along the route, on a field: stopping here", backed)
            self.backOffTargets = nil
            self.vehicle.ad.specialDrivingModule:releaseVehicle()
            self.state = WaitForCallTask.STATE_CHECK
            self.checkTimer = AutoDriveTON:new()
            self:hold(dt)
        elseif self.backOffTargets ~= nil then
            if self.vehicle.ad.trafficYieldModule:reverseTowards(dt, target) then
                self.backOffIndex = self.backOffIndex + 1
            end
        else
            self.vehicle.ad.specialDrivingModule:driveReverse(dt, 4, 1, false)
        end
    elseif self.state == WaitForCallTask.STATE_TO_ENTRANCE then
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            self.state = WaitForCallTask.STATE_CHECK
            self.checkTimer = AutoDriveTON:new()
            self:hold(dt)
        elseif ADFieldPark.holdForExit(self.vehicle, dt, "sur la route") then
            -- a full one drives out by this entrance: it passes first
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end
    elseif self.state == WaitForCallTask.STATE_DRIVING then
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            self.parkedSpot = self.spots ~= nil and self.spots[self.spotIndex] or nil
            ADFieldPark.setParked(self.vehicle, self.parkedSpot, false)
            ADFieldPark.rememberSpot(self.vehicle, self.parkedSpot, self.plannedPath, self.planStart)
            self.moveUpTimer = 0
            self.state = WaitForCallTask.STATE_WAITING
            self:hold(dt)
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end
    else
        -- beside the road: into the field when a place is free there, else closer to the entrance
        if self.parkedSpot ~= nil and self.parkedSpot.roadside and self.noPlaceRoadside then
            -- beside the road because the field had no place yet: the search runs again from the entrance it came
            -- to, without driving there; it goes in only when a place exists
            self.retryTime = nil
            self.noPlaceTimer = (self.noPlaceTimer or 0) + dt
            if self.noPlaceTimer > WaitForCallTask.NO_PLACE_RETRY then
                self.noPlaceTimer = 0
                self.search = ADFieldPark.newSearch(self.vehicle, self.noPlaceOrigin)
                self.probeFromRoadside = true
                self.state = WaitForCallTask.STATE_SEARCHING
                self:hold(dt)
                return
            end
        elseif self.parkedSpot ~= nil and self.parkedSpot.roadside then
            self.moveUpTimer = (self.moveUpTimer or 0) + dt
            if self.moveUpTimer > WaitForCallTask.MOVE_UP_CHECK then
                self.moveUpTimer = 0
                if self:moveUpRoadside() then
                    self:hold(dt)
                    return
                end
            end
        end
        -- parked in line behind another unloader in the field: takes its place when it leaves
        if self.parkedSpot ~= nil and self.parkedSpot.aheadVehicle ~= nil then
            self.moveUpTimer = (self.moveUpTimer or 0) + dt
            if self.moveUpTimer > WaitForCallTask.MOVE_UP_CHECK then
                self.moveUpTimer = 0
                if self:moveUp() then
                    self:hold(dt)
                    return
                end
            end
        end
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

WaitForCallTask.MOVE_UP_CHECK = 2000 -- ms between two checks of the place ahead in the line

-- the unloader ahead in the line left its place: take it (the whole train centred on the same footprint)
function WaitForCallTask:moveUp()
    local spot = self.parkedSpot
    local aheadSpot = spot.ahead
    if ADFieldPark.parked[spot.aheadVehicle] == aheadSpot or aheadSpot == nil or aheadSpot.cx == nil then
        return false
    end
    local front, rear = ADTrafficYieldModule.getTrainExtents(self.vehicle)
    local length = front + rear
    local halfW = ADTrafficYieldModule.getWidth(self.vehicle) / 2 + 0.5
    local probe = {vehicle = self.vehicle, halfW = halfW, halfL = length / 2 + 0.5}
    if ADFieldPark.overlapsTakenSpot(probe, aheadSpot.cx, aheadSpot.cz, aheadSpot.dirX, aheadSpot.dirZ) then
        -- someone else is already going there
        spot.aheadVehicle = nil
        return false
    end
    local shift = length / 2 - front
    local tx, tz = aheadSpot.cx + aheadSpot.dirX * shift, aheadSpot.cz + aheadSpot.dirZ * shift
    local newSpot = {
        x = tx, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, tx, 0, tz), z = tz,
        dirX = aheadSpot.dirX, dirZ = aheadSpot.dirZ,
        cx = aheadSpot.cx, cz = aheadSpot.cz, halfL = probe.halfL,
        queued = true, ahead = aheadSpot.ahead, aheadVehicle = aheadSpot.aheadVehicle, harvester = spot.harvester,
        memo = aheadSpot.memo, sense = aheadSpot.sense
    }
    self.vehicle.ad.trafficYieldModule:log("field park: the unloader in front has left, moving up to its place (%.0f, %.0f)", tx, tz)
    self.parkedSpot = nil
    self.spots = {newSpot}
    self.spotIndex = 0
    self:planToNextSpot()
    return true
end

-- ADFieldPark.FIELD_CAPACITY unloaders already in the field for this harvester: park beside the road near the
-- entrance instead, the next ones behind (true when it does)
function WaitForCallTask:goRoadsideIfFieldFull()
    local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
    local harvester = ADFieldPark.findHarvester(self.vehicle, x, z)
    if harvester == nil or ADFieldPark.countInField(self.vehicle, harvester) < ADFieldPark.FIELD_CAPACITY then
        return false
    end
    if self.ignoreCapacity then
        -- no roadside place reachable (see planToNextSpot): a place in the field behind the others rather than the entrance
        self.vehicle.ad.trafficYieldModule:log("field park: field full but no roadside place reachable, clearing the entrance, place in the field behind the others")
        return false
    end
    self.vehicle.ad.trafficYieldModule:log("field park: field full (%d unloaders), waiting beside the road", ADFieldPark.FIELD_CAPACITY)
    return self:goRoadside(harvester)
end

-- the field point (entrance) of the route, when the unloader stands on the road near it; else where it stands
function WaitForCallTask:getEntrance()
    local x, y, z = ADTrafficYieldModule.getPosition(self.vehicle)
    local marker = self.vehicle.ad.stateModule ~= nil and self.vehicle.ad.stateModule:getFirstMarker() or nil
    local wp = marker ~= nil and ADGraphManager:getWayPointById(marker.id) or nil
    if wp ~= nil and not AutoDrive.checkIsOnField(x, y, z) and MathUtil.vector2Length(wp.x - x, wp.z - z) <= ADFieldPark.ROADSIDE_RANGE then
        return wp.x, wp.z
    end
    return x, z
end

-- parks beside the road near the entrance, behind the ones already waiting there
function WaitForCallTask:goRoadside(harvester)
    local x, z = self:getEntrance()
    local module = self.vehicle.ad.trafficYieldModule
    self.spots = ADFieldPark.findRoadsideSpots(self.vehicle, harvester, x, z)
    module:log("field park: waiting beside the road, %d places found (entrance %.0f,%.0f)", #self.spots, x, z)
    self.spotIndex = 0
    if #self.spots == 0 then
        self.state = WaitForCallTask.STATE_WAITING
        self.retryTime = 0
        return true
    end
    self:planToNextSpot()
    return true
end

-- at the entrance, nose in the field, no clean way to a roadside place: back off along the route it came
-- by (the road behind), once; the places are looked for again from there. Returns true when it backs off
function WaitForCallTask:startRouteBackOff()
    if self.routeBackOffDone then
        return false
    end
    self.routeBackOffDone = true
    local module = self.vehicle.ad.trafficYieldModule
    module.escapeBranch = nil
    local targets = module:buildReverseTargets()
    if targets == nil or #targets == 0 then
        return false
    end
    local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
    self.backOffTargets = targets
    self.backOffIndex = 1
    self.backOffDistance = WaitForCallTask.ROUTE_BACK_OFF
    self.backOffTimeout = WaitForCallTask.ROUTE_BACK_OFF_TIMEOUT
    self.backOffStart = {x = x, z = z}
    self.backOffTime = 0
    self.vehicle.ad.trafficYieldModule:log("field park: no roadside place reachable from the entrance, backing off %d m along the route", WaitForCallTask.ROUTE_BACK_OFF)
    self.state = WaitForCallTask.STATE_BACKING_OFF
    return true
end

WaitForCallTask.ROUTE_BACK_OFF_MIN = 8 -- m backed along the route before a field under the train stops the back-off (the entrance itself may be on the field)
WaitForCallTask.NO_PLACE_RETRY = 45000 -- ms between two looks for a place in the field from beside the road

-- leaves its roadside place for the field: back on the network to the field point first, the search of a place
-- in the field starts there as for any arrival
function WaitForCallTask:leaveRoadside()
    self.vehicle.ad.trafficYieldModule:log("field park: free place in the field, going there")
    self.parkedSpot = nil
    -- keeps its roadside place until the search reserves one in the field
    self.state = WaitForCallTask.STATE_CHECK
    self.checkTimer = AutoDriveTON:new()
    self.backOffCount = 0
    local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
    local marker = self.vehicle.ad.stateModule ~= nil and self.vehicle.ad.stateModule:getFirstMarker() or nil
    local wp = marker ~= nil and ADGraphManager:getWayPointById(marker.id) or nil
    if wp ~= nil and MathUtil.vector2Length(wp.x - x, wp.z - z) > WaitForCallTask.ENTRANCE_NEAR then
        self.vehicle.ad.trafficYieldModule:log("field park: back on the network to the field point (%.0f m)", MathUtil.vector2Length(wp.x - x, wp.z - z))
        self.vehicle.ad.specialDrivingModule:releaseVehicle()
        self.vehicle.ad.drivePathModule:setPathTo(marker.id)
        self.state = WaitForCallTask.STATE_TO_ENTRANCE
    end
end

-- waiting beside the road: the first one in the line goes into the field as soon as a place is free there; the
-- others move to a free place closer to the entrance
function WaitForCallTask:moveUpRoadside()
    local spot = self.parkedSpot
    if ADFieldPark.countInField(self.vehicle, spot.harvester) < ADFieldPark.FIELD_CAPACITY
        and ADFieldPark.isFirstOnRoadside(self.vehicle, spot.harvester) and ADFieldPark.canParkFor(self.vehicle, spot.harvester) then
        self:leaveRoadside()
        return true
    end
    local spots = ADFieldPark.findRoadsideSpots(self.vehicle, spot.harvester, spot.entrance.x, spot.entrance.z)
    local best = spots[1]
    if best ~= nil and best.distance < spot.distance - 5 then
        self.vehicle.ad.trafficYieldModule:log("field park: a roadside place closer to the entrance is free, moving up (%.0f m)", best.distance)
        self.parkedSpot = nil
        self.spots = spots
        self.spotIndex = 0
        self:planToNextSpot()
        return true
    end
    return false
end

-- starts the path planning to the next candidate spot, or waits here when none is left
function WaitForCallTask:planToNextSpot()
    local module = self.vehicle.ad.trafficYieldModule
    self.spotIndex = self.spotIndex + 1
    local spot = self.spots ~= nil and self.spots[self.spotIndex] or nil
    if spot == nil or self.spotIndex > ADFieldPark.MAX_TRIES then
        ADFieldPark.setParked(self.vehicle, nil)
        if self.spotIndex == 1 then
            module:log("field park: no free ground found, waiting here")
        else
            module:log("field park: no clean path to a free spot, waiting here")
        end
        self.state = WaitForCallTask.STATE_WAITING
        self.retryTime = 0
        local roadside = self.spots ~= nil and self.spots[1] ~= nil and self.spots[1].roadside
        if self.spotIndex == 1 and not roadside and self.noPlaceRoadside then
            -- already backed off on the road for that reason: just wait and look again later, no more backing off
            return
        end
        if self.spotIndex == 1 and not roadside and not self.ignoreCapacity and not self.loaded then
            -- the job has just started, no harvested
            -- ground to park on yet: it does not wait at the entrance in everybody's way, it waits beside the road
            -- before the entrance and looks again from there (see probeFromRoadside)
            local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
            local harvester = ADFieldPark.findHarvester(self.vehicle, x, z)
            if harvester ~= nil then
                local dx, dz = ADTrafficYieldModule.getDirection(self.vehicle)
                self.noPlaceOrigin = {x = x, z = z, dx = dx, dz = dz}
                self.noPlaceRoadside = true
                self.noPlaceTimer = 0
                self:goRoadside(harvester)
                if self.spots == nil or #self.spots == 0 then
                    if not self:startRouteBackOff() then
                        self:startBackOff()
                    end
                end
                return
            end
        end
        if roadside and self:startRouteBackOff() then
            return
        elseif roadside and not self.ignoreCapacity then
            -- backed off already and still nothing: the entrance stays free, a place in the field behind the others
            self.ignoreCapacity = true
            self.state = WaitForCallTask.STATE_CHECK
            self.checkTimer = AutoDriveTON:new()
            return
        end
        self:startBackOff()
        return
    end
    module:log("field park: spot %d found at x=%.0f z=%.0f heading %.0f°%s", self.spotIndex, spot.x, spot.z, math.deg(math.atan2(spot.dirX, spot.dirZ)), spot.queued and " (in line with a parked unloader)" or "")
    -- taken for the others while this unloader drives there (order of arrival along the border)
    ADFieldPark.setParked(self.vehicle, spot, true)
    local x, _, z = ADTrafficYieldModule.getPosition(self.vehicle)
    local dx, dz = ADTrafficYieldModule.getDirection(self.vehicle)
    self.planStart = {x = x, z = z, dx = dx, dz = dz}
    self.plannedPath = nil
    -- same start and same stop as the first unloader of this field: its path
    local wayPoints = ADFieldPark.getMemoPath(self.vehicle, spot)
    if wayPoints ~= nil then
        module:log("field park: remembered path to spot %d, %d points", self.spotIndex, #wayPoints)
        self.plannedPath = wayPoints
        self.vehicle.ad.drivePathModule:setWayPoints(wayPoints)
        self.state = WaitForCallTask.STATE_DRIVING
        return
    end
    -- the last part follows the border: the whole train, trailers included, stops aligned with it
    local target, leadIn = ADFieldPark.getLeadIn(self.vehicle, spot)
    self.leadIn = leadIn
    if leadIn ~= nil then
        module:log("field park: lead-in along the border, %d points to spot %d", #leadIn, self.spotIndex)
    end
    wayPoints = ADFieldPark.findDirectPath(self.vehicle, target)
    if wayPoints ~= nil then
        wayPoints = ADFieldPark.appendLeadIn(wayPoints, leadIn)
        self.plannedPath = leadIn ~= nil and wayPoints or nil
        module:log("field park: direct path to spot %d, %d points", self.spotIndex, #wayPoints)
        self.vehicle.ad.drivePathModule:setWayPoints(wayPoints)
        self.state = WaitForCallTask.STATE_DRIVING
        return
    end
    if spot.roadside then
        -- the AutoDrive pathfinder may turn in the field: beside the road only with a direct path
        module:log("field park: no direct path to roadside spot %d without the field, next one", self.spotIndex)
        self:planToNextSpot()
        return
    end
    self.planningTime = 0
    self.smoothingTime = nil
    self.vehicle.ad.pathFinderModule:reset()
    self.vehicle.ad.pathFinderModule:startPathPlanningTo({x = target.x, y = target.y, z = target.z}, {x = target.dirX, z = target.dirZ})
    self.state = WaitForCallTask.STATE_PATHPLANNING
end

-- the route of the AutoDrive pathfinder checked every meter over the width of the train: standing crop
-- (small forgotten patches and the border fringe excepted) is never driven through to park
function WaitForCallTask:pathCrossesCrop(wayPoints, pathFinder)
    if wayPoints == nil or #wayPoints < 2 or ADFieldPark.isPathClear == nil then
        return false
    end
    -- the same width as the pathfinder's own crop check (the tyres, no margin, behind a forage harvester):
    -- twice narrower than the body with its margin, which read the edge rows beside the route as crossed
    -- (every route would be refused a few metres from the entrance)
    local halfW = ADTrafficYieldModule.getWidth(self.vehicle) / 2 + (ADFieldPark.CROP_MARGIN or 0.7)
    if pathFinder ~= nil and pathFinder.getCropHalfWidth ~= nil then
        halfW = math.min(halfW, pathFinder:getCropHalfWidth())
    end
    local path = {}
    for _, wp in ipairs(wayPoints) do
        local last = path[#path]
        if last == nil or MathUtil.vector2Length(wp.x - last.x, wp.z - last.z) <= 1 then
            if last == nil then
                table.insert(path, {x = wp.x, y = wp.y, z = wp.z})
            end
        else
            -- a point every meter at most: the check reads the crop at the points of the path
            local dx, dz = wp.x - last.x, wp.z - last.z
            local length = MathUtil.vector2Length(dx, dz)
            local steps = math.ceil(length)
            for i = 1, steps do
                table.insert(path, {x = last.x + dx * i / steps, y = wp.y, z = last.z + dz * i / steps})
            end
        end
    end
    -- the first meters are where the train already stands (it came in that way): the edge rows at the
    -- entrance are not a crossing
    local startX, startZ = path[1].x, path[1].z
    local firstFree = #path
    for i = 1, #path do
        if MathUtil.vector2Length(path[i].x - startX, path[i].z - startZ) > (ADFieldPark.START_FREE or 15) then
            firstFree = i
            break
        end
    end
    if firstFree > 1 then
        local trimmed = {}
        for i = firstFree, #path do
            table.insert(trimmed, path[i])
        end
        path = trimmed
    end
    if #path < 2 then
        return false
    end
    local blocked, bx, bz, index = ADFieldPark.isPathClear(self.vehicle, path, halfW, nil, false)
    if blocked == "crop" then
        self.vehicle.ad.trafficYieldModule:log("field park: pathfinder route crosses the crop at x=%.0f z=%.0f (point %d/%d)", bx, bz, index, #path)
        return true
    end
    return false
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
    self.backOffTimeout = WaitForCallTask.BACK_OFF_TIMEOUT
    self.backOffTargets = nil
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
