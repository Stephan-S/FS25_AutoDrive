ExitFieldTask = ADInheritsFrom(AbstractTask)
ExitFieldTask.debug = false

ExitFieldTask.STATE_PATHPLANNING = 1
ExitFieldTask.STATE_DRIVING = 2
ExitFieldTask.STATE_DELAY_PATHPLANNING = 3

ExitFieldTask.STRATEGY_START = 0
ExitFieldTask.STRATEGY_BEHIND_START = 1
ExitFieldTask.STRATEGY_CLOSEST = 2

function ExitFieldTask:new(vehicle)
    local o = ExitFieldTask:create()
    o.vehicle = vehicle
    o.trailers = nil
    o.failedPathFinder = 0
    o.waitForCheckTimer = AutoDriveTON:new()
    return o
end

function ExitFieldTask:setUp()
    self.state = ExitFieldTask.STATE_DELAY_PATHPLANNING
    self.nextExitStrategy = AutoDrive.getSetting("exitField", self.vehicle)
    self.trailers, _ = AutoDrive.getAllUnits(self.vehicle)
    AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, false)
end

function ExitFieldTask:update(dt)
    -- the point it drives out to is published: an empty unloader coming in waits before the entrance (field park)
    if ADFieldPark ~= nil and ADFieldPark.setFieldExit ~= nil then
        if self.state == ExitFieldTask.STATE_DRIVING and self.exitPoint ~= nil then
            ADFieldPark.setFieldExit(self.vehicle, self.exitPoint.x, self.exitPoint.z)
        else
            ADFieldPark.setFieldExit(self.vehicle, nil)
        end
    end
    if self.state == ExitFieldTask.STATE_PATHPLANNING then
        if self.vehicle.ad.pathFinderModule:hasFinished() then
            self.wayPoints = self.vehicle.ad.pathFinderModule:getPath()
            if self.wayPoints == nil or #self.wayPoints == 0 then
                -- no clean path to this network point: try the next one right away
                if not self:startPathPlanning() then
                    return
                end
            else
                self.vehicle.ad.drivePathModule:setWayPoints(self.wayPoints)
                self.state = ExitFieldTask.STATE_DRIVING
            end
        else
            self.vehicle.ad.pathFinderModule:update(dt)
            self.vehicle.ad.specialDrivingModule:stopVehicle()
            self.vehicle.ad.specialDrivingModule:update(dt)
        end
    elseif self.state == ExitFieldTask.STATE_DELAY_PATHPLANNING then
        ExitFieldTask.debugMsg(self.vehicle, "ExitFieldTask:update - STATE_DELAY_PATHPLANNING")
        if self.waitForCheckTimer:timer(true, 1000, dt) then
            if self:startPathPlanning() then
                self.vehicle.ad.pathFinderModule:addDelayTimer(6000)
                self.state = ExitFieldTask.STATE_PATHPLANNING
                return
            end
        end
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return
    elseif self.state == ExitFieldTask.STATE_DRIVING then
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            self.state = ExitFieldTask.STATE_FINISHED
        elseif self:isExitPointBlocked(dt) then
            -- an unloader waits on the point: the next point instead
            self.state = ExitFieldTask.STATE_DELAY_PATHPLANNING
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end
    elseif self.state == ExitFieldTask.STATE_FINISHED then
        self:finished()
    end
end

function ExitFieldTask:abort()
    self:clearFieldExit()
end

function ExitFieldTask:finished()
    self:clearFieldExit()
    self.vehicle.ad.taskModule:setCurrentTaskFinished()
end

function ExitFieldTask:clearFieldExit()
    if ADFieldPark ~= nil and ADFieldPark.setFieldExit ~= nil then
        ADFieldPark.setFieldExit(self.vehicle, nil)
    end
end

ExitFieldTask.POINT_CHECK_INTERVAL = 2000 -- ms between two looks at the exit point
ExitFieldTask.POINT_CHECK_MIN_DISTANCE = 20 -- m: closer to the point, it goes on (the other one sees it coming)

-- an empty unloader stands on the network point this one drives out to (it waits for its call there):
-- the next point of the list is tried, if there is one
function ExitFieldTask:isExitPointBlocked(dt)
    if ADFieldPark == nil or ADFieldPark.isExitPointTaken == nil then
        return false
    end
    self.pointCheckTime = (self.pointCheckTime or 0) + dt
    if self.exitPoint == nil or self.pointCheckTime < ExitFieldTask.POINT_CHECK_INTERVAL then
        return false
    end
    self.pointCheckTime = 0
    if self.candidates == nil or self.candidateIndex == nil or self.candidateIndex >= #self.candidates then
        return false
    end
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    if MathUtil.vector2Length(x - self.exitPoint.x, z - self.exitPoint.z) < ExitFieldTask.POINT_CHECK_MIN_DISTANCE then
        return false
    end
    local other = ADFieldPark.isExitPointTaken(self.vehicle, self.exitPoint.x, self.exitPoint.z)
    if other == nil then
        return false
    end
    ExitFieldTask.debugMsg(self.vehicle, "ExitFieldTask:isExitPointBlocked - %s waits on network point %s, next point", tostring(other:getName()), tostring(self.exitPoint.id))
    return true
end

ExitFieldTask.CANDIDATE_MAX_DISTANCE = 200   -- m, network points considered to leave the field
ExitFieldTask.CANDIDATE_SPACING = 15         -- m between two tried points
ExitFieldTask.CANDIDATE_COUNT = 6
ExitFieldTask.FIELD_POINT_MAX_DISTANCE = 500 -- m, field entrance point tried first
ExitFieldTask.BEHIND_PENALTY = 60            -- m added to points behind the vehicle

-- network points that lead to the destination, easiest first: the closest ones in front of the vehicle,
-- then the others, the field entrance (route start) included. Each is tried without ever driving
-- through crops; the first clean path wins, even if it is further away.
function ExitFieldTask:buildCandidates()
    local candidates = {}
    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
    local dx, _, dz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local minDistance = AutoDrive.getDriverRadius(self.vehicle)
    local destination = self.vehicle.ad.stateModule:getSecondWayPoint()
    local routes = {}
    local closest = self.vehicle:getClosestWayPoint()
    if closest ~= nil then
        table.insert(routes, ADGraphManager:pathFromTo(closest, destination))
    end
    local fieldRoute = ADGraphManager:pathFromTo(self.vehicle.ad.stateModule:getFirstWayPoint(), destination)
    table.insert(routes, fieldRoute)
    -- the field's own network point (where the vehicle came in) is tried first, whatever its distance
    if fieldRoute ~= nil and #fieldRoute > 1 then
        local fieldWp, nextWp = fieldRoute[1], fieldRoute[2]
        local distance = MathUtil.vector2Length(fieldWp.x - vx, fieldWp.z - vz)
        if distance >= minDistance and distance <= ExitFieldTask.FIELD_POINT_MAX_DISTANCE then
            table.insert(candidates, {wp = fieldWp, vector = {x = nextWp.x - fieldWp.x, z = nextWp.z - fieldWp.z}, score = 0, distance = distance})
        end
    end

    local pool = {}
    local seen = {}
    for _, route in ipairs(routes) do
        if route ~= nil and #route > 1 then
            for i = 1, #route - 1 do
                local wp, nextWp = route[i], route[i + 1]
                if wp.id ~= nil and not seen[wp.id] then
                    seen[wp.id] = true
                    local distance = MathUtil.vector2Length(wp.x - vx, wp.z - vz)
                    if distance >= minDistance and distance <= ExitFieldTask.CANDIDATE_MAX_DISTANCE then
                        local score = distance
                        if (wp.x - vx) * dx + (wp.z - vz) * dz < 0 then
                            score = score + ExitFieldTask.BEHIND_PENALTY
                        end
                        table.insert(pool, {wp = wp, vector = {x = nextWp.x - wp.x, z = nextWp.z - wp.z}, score = score, distance = distance})
                    end
                end
            end
        end
    end
    table.sort(pool, function(a, b) return a.score < b.score end)
    for _, c in ipairs(pool) do
        local farEnough = true
        for _, chosen in ipairs(candidates) do
            if MathUtil.vector2Length(c.wp.x - chosen.wp.x, c.wp.z - chosen.wp.z) < ExitFieldTask.CANDIDATE_SPACING then
                farEnough = false
                break
            end
        end
        if farEnough then
            table.insert(candidates, c)
            if #candidates >= ExitFieldTask.CANDIDATE_COUNT then
                break
            end
        end
    end
    return candidates
end

-- returns true when a path planning was started, false when the task ended
function ExitFieldTask:startPathPlanning()
    ExitFieldTask.debugMsg(self.vehicle, "ExitFieldTask:startPathPlanning")
    if self.candidates == nil then
        local _, closestDistance = self.vehicle:getClosestWayPoint()
        if closestDistance ~= nil and closestDistance <= AutoDrive.getDriverRadius(self.vehicle) then
            -- close to network, set task finished
            self:finished()
            return false
        end
        self.candidates = self:buildCandidates()
        self.candidateIndex = 0
    end
    self.candidateIndex = self.candidateIndex + 1
    local candidate = self.candidates[self.candidateIndex]
    if candidate == nil then
        ExitFieldTask.debugMsg(self.vehicle, "ExitFieldTask:startPathPlanning - no clean path to the network (%d point(s) tried), stopping", #self.candidates)
        AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.WARN, "$l10n_AD_Driver_of; %s $l10n_AD_cannot_find_path;", 5000, self.vehicle.ad.stateModule:getName())
        self.vehicle.ad.taskModule:abortAllTasks()
        self.vehicle.ad.taskModule:addTask(StopAndDisableADTask:new(self.vehicle))
        return false
    end
    ExitFieldTask.debugMsg(self.vehicle, "ExitFieldTask:startPathPlanning - trying network point %s (%d/%d, %.0f m)", tostring(candidate.wp.id), self.candidateIndex, #self.candidates, candidate.distance)
    local pf = self.vehicle.ad.pathFinderModule
    pf:reset()
    pf.noFallback = true
    -- the end of the route is judged for obstacles too (a wall between the field and the road)
    pf.exitToNetwork = true
    local approach, appended = self:buildApproach(candidate)
    if approach ~= nil then
        -- the point is reached along the axis of the network (the gate), the grid only has to bring the train
        -- to an approach point in the open field, aligned with that axis
        ExitFieldTask.debugMsg(self.vehicle, "ExitFieldTask:startPathPlanning - point %s reached along the network axis: approach %.0f m before it", tostring(candidate.wp.id), approach.distance)
        pf:startPathPlanningTo(approach, {x = candidate.vector.x, z = candidate.vector.z})
        pf.appendWayPoints = appended
    else
        pf:startPathPlanningTo(candidate.wp, {x = candidate.vector.x, z = candidate.vector.z})
    end
    self.exitPoint = {x = candidate.wp.x, z = candidate.wp.z, id = candidate.wp.id}
    self.state = ExitFieldTask.STATE_PATHPLANNING
    return true
end

ExitFieldTask.APPROACH_DISTANCES = {20, 12} -- m before the network point, along the axis of the network, tried in this order
ExitFieldTask.APPROACH_STEP = 3             -- m between the points of the straight stretch to the network point
ExitFieldTask.APPROACH_MIN_VEHICLE_DISTANCE = 12 -- m: closer than approach + this, the point is driven to directly

-- a field gate is a lane between a post and a wall, narrower than the train plus the margins of the grid: the grid
-- cannot find it, or finds it with a corner too sharp. The network does go through it, so the point is reached
-- along the axis of the network: an approach point in the open field, on that axis, then straight on. Returns the
-- approach point (target of the grid) and the points appended after it, or nil when the axis does not fit here
-- (approach point off the field, crop on the straight stretch, vehicle too close).
function ExitFieldTask:buildApproach(candidate)
    local wp, vector = candidate.wp, candidate.vector
    if wp == nil or vector == nil then
        return nil
    end
    local length = MathUtil.vector2Length(vector.x, vector.z)
    if length < 0.5 then
        return nil
    end
    local ux, uz = vector.x / length, vector.z / length
    local halfWidth = self:getTrainHalfWidth()
    local wy = wp.y or getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, wp.x, 1, wp.z)
    for _, distance in ipairs(ExitFieldTask.APPROACH_DISTANCES) do
        if (candidate.distance or 0) >= distance + ExitFieldTask.APPROACH_MIN_VEHICLE_DISTANCE then
            local ax, az = wp.x - ux * distance, wp.z - uz * distance
            local ay = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, ax, 1, az)
            if AutoDrive.checkIsOnField(ax, ay, az) and not ExitFieldTask.hasCropOnStretch(ax, az, wp.x, wp.z, halfWidth) then
                local approach = {x = ax, y = ay, z = az, distance = distance}
                local appended = {approach}
                local d = ExitFieldTask.APPROACH_STEP
                while d < distance - ExitFieldTask.APPROACH_STEP / 2 do
                    local px, pz = ax + ux * d, az + uz * d
                    table.insert(appended, {x = px, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, px, 1, pz), z = pz})
                    d = d + ExitFieldTask.APPROACH_STEP
                end
                table.insert(appended, {x = wp.x, y = wy, z = wp.z})
                return approach, appended
            end
        end
    end
    return nil
end

function ExitFieldTask:getTrainHalfWidth()
    local width = self.vehicle.size ~= nil and self.vehicle.size.width or 3
    for _, trailer in pairs(self.trailers or {}) do
        if trailer.size ~= nil and trailer.size.width ~= nil then
            width = math.max(width, trailer.size.width)
        end
    end
    return width / 2
end

-- standing crop on the straight band (train width) between two points, every meter
function ExitFieldTask.hasCropOnStretch(x1, z1, x2, z2, halfWidth)
    local dx, dz = x2 - x1, z2 - z1
    local length = math.sqrt(dx * dx + dz * dz)
    if length < 0.5 then
        return false
    end
    dx, dz = dx / length, dz / length
    local d = 0
    while d <= length do
        local cx, cz = x1 + dx * d, z1 + dz * d
        local corners = {
            {x = cx - dz * halfWidth, z = cz + dx * halfWidth},
            {x = cx + dz * halfWidth, z = cz - dx * halfWidth},
            {x = cx - dz * halfWidth + dx, z = cz + dx * halfWidth + dz}
        }
        if AutoDrive.checkForUnknownFruitInArea(corners) then
            return true
        end
        d = d + 1
    end
    return false
end

function ExitFieldTask:selectNextStrategy()
    self.nextExitStrategy = (self.nextExitStrategy + 1) % (ExitFieldTask.STRATEGY_CLOSEST + 1)
end

function ExitFieldTask:continue()
end

function ExitFieldTask:getInfoText()
    if self.state == ExitFieldTask.STATE_PATHPLANNING then
        local actualState, maxStates = self.vehicle.ad.pathFinderModule:getCurrentState()
        return g_i18n:getText("AD_task_pathfinding") .. string.format(" %d / %d ", actualState, maxStates)
    else
        return g_i18n:getText("AD_task_exiting_field")
    end
end

function ExitFieldTask:getI18nInfo()
    if self.state == ExitFieldTask.STATE_PATHPLANNING then
        local actualState, maxStates, steps, max_pathfinder_steps = self.vehicle.ad.pathFinderModule:getCurrentState()
        return "$l10n_AD_task_pathfinding;" .. string.format(" %d / %d - %d / %d", actualState, maxStates, steps, max_pathfinder_steps)
    else
        return "$l10n_AD_task_exiting_field;"
    end
end

function ExitFieldTask.debugMsg(vehicle, debugText, ...)
    if ExitFieldTask.debug == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    else
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_COMBINEINFO, debugText, ...)
    end
end
