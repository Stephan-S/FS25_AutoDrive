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
                self.routeStart = g_time
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
        elseif self.vehicle.ad.routeBlockedTime ~= nil and self.routeStart ~= nil and self.vehicle.ad.routeBlockedTime > self.routeStart then
            -- the front sensor kept it stopped on this route (wall, post): the place is a forbidden zone now,
            -- the exit is planned again around it instead of waiting there
            self.routeStart = nil
            self.candidates = nil
            self.state = ExitFieldTask.STATE_DELAY_PATHPLANNING
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end
    elseif self.state == ExitFieldTask.STATE_FINISHED then
        self:finished()
    end
end

function ExitFieldTask:abort()
end

function ExitFieldTask:finished()
    self.vehicle.ad.taskModule:setCurrentTaskFinished()
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
        table.insert(routes, ADGraphManager:pathFromTo(closest, destination, nil, self.vehicle))
    end
    local fieldRoute = ADGraphManager:pathFromTo(self.vehicle.ad.stateModule:getFirstWayPoint(), destination, nil, self.vehicle)
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
    pf:startPathPlanningTo(candidate.wp, {x = candidate.vector.x, z = candidate.vector.z})
    self.state = ExitFieldTask.STATE_PATHPLANNING
    return true
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
