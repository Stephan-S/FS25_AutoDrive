CombineUnloaderMode = ADInheritsFrom(AbstractMode)
CombineUnloaderMode.debug = false

CombineUnloaderMode.STATE_INIT = {}
CombineUnloaderMode.STATE_WAIT_TO_BE_CALLED = {}
CombineUnloaderMode.STATE_DRIVE_TO_COMBINE = {}
CombineUnloaderMode.STATE_DRIVE_TO_PIPE = {}
CombineUnloaderMode.STATE_LEAVE_CROP = {}
CombineUnloaderMode.STATE_DRIVE_TO_START = {}
CombineUnloaderMode.STATE_DRIVE_TO_UNLOAD = {}
CombineUnloaderMode.STATE_FOLLOW_COMBINE = {}
CombineUnloaderMode.STATE_ACTIVE_UNLOAD_COMBINE = {}
CombineUnloaderMode.STATE_FOLLOW_CURRENT_UNLOADER = {}
CombineUnloaderMode.STATE_EXIT_FIELD = {}
CombineUnloaderMode.STATE_REVERSE_FROM_BAD_LOCATION = {}
CombineUnloaderMode.STATE_PARK = {} -- harvester job done, driving to the park destination

CombineUnloaderMode.MAX_COMBINE_FILLLEVEL_CHASING = 101
CombineUnloaderMode.STATIC_X_OFFSET_FROM_HEADER = 1 -- m beyond the header tip (0 m is impossible)

function CombineUnloaderMode:new(vehicle)
    local o = CombineUnloaderMode:create()
    o.vehicle = vehicle
    o.trailers = nil
    o.trailerCount = 0
    o.tractorTrainLength = 0
    CombineUnloaderMode.reset(o)
    CombineUnloaderMode.setStateNames(o)
    return o
end

function CombineUnloaderMode:reset()
    self.state = self.STATE_INIT
    self.activeTask = nil
    self.vehicle.ad.unloaderQueueTime = nil
    ADHarvestManager.releaseStickyUnloader(self.vehicle, "AutoDrive stopped or reset")
    ADHarvestManager:unregisterAsUnloader(self.vehicle)
    self.combine = nil
    self.followingUnloader = nil
    self.breadCrumbs = Queue:new()
    self.lastBreadCrumb = nil
    self.failedPathFinder = 0
    self.trailers, self.trailerCount = AutoDrive.getAllUnits(self.vehicle)
    self.tractorTrainLength = AutoDrive.getTractorTrainLength(self.vehicle, true, false)
    self.vehicle.ad.trailerModule:reset()
    AutoDrive.getAllDischargeableUnits(self.vehicle, true) -- force initialisation
end

function CombineUnloaderMode:start(user)
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:start start self.state %s", tostring(self:getStateName()))
    if not self.vehicle.ad.stateModule:isActive() then
        self.vehicle:startAutoDrive()
    end

    if self.vehicle.ad.stateModule:getFirstMarker() == nil or self.vehicle.ad.stateModule:getSecondMarker() == nil then
        return
    end

    self:reset()
    self.vehicle.ad.harvestJobDone = nil -- a new start: the former harvester's end of job is forgotten

    self.activeTask = self:getNextTask()
    if self.activeTask ~= nil then
        self.vehicle.ad.taskModule:addTask(self.activeTask)
    end
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:start end self.state %s", tostring(self:getStateName()))
end

-- the harvester's job is done. Whatever this unloader was doing, it leaves to
-- unload what it carries (even a little), then drives to its park destination; an empty one goes to park at once
CombineUnloaderMode.JOB_DONE_MIN_LOAD = 50 -- liters: less is "empty"

function CombineUnloaderMode:getParkAfterJobMarker()
    local parkId = self.vehicle.ad.stateModule.getParkDestinationAtJobFinished ~= nil and self.vehicle.ad.stateModule:getParkDestinationAtJobFinished() or -1
    local marker = parkId ~= nil and parkId >= 1 and ADGraphManager:getMapMarkerById(parkId) or nil
    if marker ~= nil and marker.id ~= nil then
        return marker
    end
    return nil
end

function CombineUnloaderMode:onHarvesterJobDone(harvester)
    if self.vehicle.ad.harvestJobDone then
        return
    end
    local trailers = (self.trailers ~= nil and #self.trailers > 0) and self.trailers or AutoDrive.getAllUnits(self.vehicle)
    local level = 0
    if trailers ~= nil and #trailers > 0 then
        level = AutoDrive.getAllFillLevels(trailers)
    end
    local marker = self:getParkAfterJobMarker()
    local loaded = level >= CombineUnloaderMode.JOB_DONE_MIN_LOAD
    -- no park destination and nothing to unload: it stays where it is and waits, as before
    if marker == nil and not loaded then
        return
    end
    self.vehicle.ad.harvestJobDone = true
    self.vehicle.ad.unloaderQueueTime = nil
    self.vehicle.ad.taskModule:abortAllTasks()
    ADHarvestManager.releaseStickyUnloader(self.vehicle, "harvester job done")
    ADHarvestManager:unregisterAsUnloader(self.vehicle, true) -- no promotion: every unloader leaves
    self.combine = nil
    self.followingUnloader = nil
    self.targetUnloader = nil
    local nextTask
    if loaded then
        local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
        local start = self.vehicle.ad.stateModule:getFirstMarker()
        local point = start ~= nil and start.id ~= nil and ADGraphManager:getWayPointById(start.id) or nil
        local distanceToStart = point ~= nil and MathUtil.vector2Length(x - point.x, z - point.z) or 0
        if AutoDrive.checkIsOnField(x, y, z) and distanceToStart > 30 then
            nextTask = ExitFieldTask:new(self.vehicle)
            self.state = self.STATE_EXIT_FIELD
        else
            nextTask = UnloadAtDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getSecondMarker().id)
            self.state = self.STATE_DRIVE_TO_UNLOAD
        end
    else
        nextTask = self:getParkAfterJobTask()
    end
    self.activeTask = nextTask
    if nextTask ~= nil then
        self.vehicle.ad.taskModule:addTask(nextTask)
    end
end

-- the drive to the park destination (AutoDrive's "P" of the vehicle). Without one: back to the field target and
-- waiting for a call, as before; nil = the usual way
function CombineUnloaderMode:getParkAfterJobTask()
    local marker = self:getParkAfterJobMarker()
    if marker == nil then
        self.vehicle.ad.harvestJobDone = nil
        self.parkAfterJobName = nil
        return nil
    end
    self.state = self.STATE_PARK
    self.vehicle.ad.onRouteToPark = true
    self.parkAfterJobName = marker.name
    return DriveToDestinationTask:new(self.vehicle, marker.id)
end

-- AutoDrive's "stuck" safety (DrivePathModule.handleBeingStuck, 8 s without getting closer to the
-- next waypoint) reset the unloader: stop + restart, queue place and harvester lost, the next unloader called from far
-- away. An unloader on the field or busy with a harvester keeps everything:
-- it backs out as for a blocked vehicle and plans again, the reset only after STUCK_MAX_RECOVERIES in a row
CombineUnloaderMode.STUCK_MAX_RECOVERIES = 3
CombineUnloaderMode.STUCK_RECOVERY_RESET = 180000 -- ms without a stuck: the count starts again

function CombineUnloaderMode:onDriveStuck(info)
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local busy = self.combine ~= nil or self.targetUnloader ~= nil or self.state == self.STATE_WAIT_TO_BE_CALLED
    if not busy and not AutoDrive.checkIsOnField(x, y, z) then
        return false
    end
    if self.lastStuckTime ~= nil and (g_time or 0) - self.lastStuckTime > CombineUnloaderMode.STUCK_RECOVERY_RESET then
        self.stuckRecoveries = 0
    end
    self.lastStuckTime = g_time or 0
    self.stuckRecoveries = (self.stuckRecoveries or 0) + 1
    if self.stuckRecoveries > CombineUnloaderMode.STUCK_MAX_RECOVERIES then
        self.stuckRecoveries = 0
        return false
    end
    self.vehicle.ad.specialDrivingModule:releaseVehicle()
    self.vehicle.ad.taskModule:abortAllTasks()
    self.targetUnloader = nil
    self.activeTask = ReverseFromBadLocationTask:new(self.vehicle)
    self.state = self.STATE_REVERSE_FROM_BAD_LOCATION
    self.vehicle.ad.taskModule:addTask(self.activeTask)
    self.failedPathFinder = 0
    return true
end

function CombineUnloaderMode:monitorTasks(dt)
    if self.combine ~= nil and (self.state == self.STATE_DRIVE_TO_START or self.state == self.STATE_DRIVE_TO_UNLOAD or self.state == self.STATE_EXIT_FIELD) then
        if AutoDrive.getDistanceBetween(self.vehicle, self.combine) > 25 then
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:monitorTasks -> unregisterAsUnloader")
            ADHarvestManager:unregisterAsUnloader(self.vehicle)
            self.followingUnloader = nil
            self.combine = nil
        end
    end
    if self.combine ~= nil and self.state == self.STATE_ACTIVE_UNLOAD_COMBINE then
        self:leaveBreadCrumbs()
    end
    self:recordTrack()
    self:recordCombineTrail()
    --We are stuck
    if self.failedPathFinder >= 5 or ((self.vehicle.ad.specialDrivingModule:shouldStopMotor() or self.vehicle.ad.specialDrivingModule.stoppedTimer:done()) and self.vehicle.ad.specialDrivingModule.isBlocked) then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:monitorTasks() - detected stuck vehicle - try reversing out of it now")
        self.vehicle.ad.specialDrivingModule:releaseVehicle()
        self.vehicle.ad.taskModule:abortAllTasks()
        self.reverseCount = (self.reverseCount or 0) + 1
        if self.reverseCount > CombineUnloaderMode.MAX_REVERSE_COUNT and self.combine ~= nil then
            -- already reversed twice for this harvester: stop here and let another unloader take it
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:monitorTasks() - reversed %d times - hand over to another unloader", self.reverseCount - 1)
            self.vehicle.ad.skipHarvester = self.combine
            self.vehicle.ad.skipHarvesterUntil = g_time + CombineUnloaderMode.HAND_OVER_TIME
            ADHarvestManager:unregisterAsUnloader(self.vehicle)
            self.reverseCount = 0
            self.failedPathFinder = 0
            self.followingUnloader = nil
            self:setToWaitForCall()
            return
        end
        self.activeTask = ReverseFromBadLocationTask:new(self.vehicle)
        self.state = self.STATE_REVERSE_FROM_BAD_LOCATION
        self.vehicle.ad.taskModule:addTask(self.activeTask)
        self.failedPathFinder = 0
    end

    if self.vehicle.lastSpeedReal > 0.0013 then
        self.failedPathFinder = 0
    end
end

-- forage harvester: the ground this unloader drove on in the field (driven forward, so free of crop), newest last.
-- It can back off along it to let the harvester or another unloader pass
CombineUnloaderMode.TRACK_STEP = 3
CombineUnloaderMode.TRACK_MAX_POINTS = 80
CombineUnloaderMode.TRACK_LOOK_BEHIND = 5       -- m between the reversing node and the point it aims at (6 cut the inside of the curves into the crop)
CombineUnloaderMode.TRACK_STOP_DISTANCE = 2.5   -- m between the tractor and its parking point to stop
CombineUnloaderMode.TRACK_BLOCKED_TIME = 8000   -- ms without moving while reversing: something behind, stop there
CombineUnloaderMode.TRACK_TIMEOUT = 150000

CombineUnloaderMode.TRACK_OFF_FIELD_KEEP = 15 -- m off the field before the track is wiped

-- ground the harvester drove on (its CP turn leaves the field at the row end, the tractor behind it
-- stopped on): the chase line may cross the field border there
CombineUnloaderMode.COMBINE_TRAIL_STEP = 2
CombineUnloaderMode.COMBINE_TRAIL_MAX_POINTS = 60
CombineUnloaderMode.COMBINE_TRAIL_RADIUS = 2.5

CombineUnloaderMode.LANE_KEEP_DISTANCE = 25 -- m: the way of an unloader is kept for the next ones only when it ends this close to the harvester

function CombineUnloaderMode:recordCombineTrail()
    local combine = self.combine
    if combine == nil or combine.components == nil or combine.ad == nil or not combine.ad.isChopper then
        self.combineTrail = nil
        return
    end
    if self.combineTrail == nil or self.combineTrail.combine ~= combine then
        self.combineTrail = {combine = combine, points = {}}
    end
    local points = self.combineTrail.points
    local x, _, z = getWorldTranslation(combine.components[1].node)
    local last = points[#points]
    if last == nil or MathUtil.vector2Length(x - last.x, z - last.z) >= CombineUnloaderMode.COMBINE_TRAIL_STEP then
        table.insert(points, {x = x, z = z})
        while #points > CombineUnloaderMode.COMBINE_TRAIL_MAX_POINTS do
            table.remove(points, 1)
        end
    end
end

function CombineUnloaderMode:isOnCombineTrail(x, z, radius)
    if self.combineTrail == nil then
        return false
    end
    radius = radius or CombineUnloaderMode.COMBINE_TRAIL_RADIUS
    for _, p in ipairs(self.combineTrail.points) do
        if MathUtil.vector2Length(x - p.x, z - p.z) <= radius then
            return true
        end
    end
    return false
end

-- the harvester's trail from the point closest to (x, z) up to keepBehind m before its newest point: the way it
-- drove through its own U-turn. nil when the start is not on the trail or too little is left
function CombineUnloaderMode:getCombineTrailFrom(x, z, maxStartDistance, keepBehind)
    local points = self.combineTrail ~= nil and self.combineTrail.points or nil
    if points == nil or #points < 5 then
        return nil
    end
    local first, best = nil, math.huge
    for i, p in ipairs(points) do
        local d = MathUtil.vector2Length(x - p.x, z - p.z)
        if d < best then
            first, best = i, d
        end
    end
    if first == nil or best > maxStartDistance then
        return nil, string.format("tractor %.0f m from the track", best)
    end
    if best < 3 and first < #points then
        -- standing on it: aim at the next point, never back at the one under the tractor
        first = first + 1
    end
    local last, behind = #points, 0
    while last > first and behind < keepBehind do
        behind = behind + MathUtil.vector2Length(points[last].x - points[last - 1].x, points[last].z - points[last - 1].z)
        last = last - 1
    end
    if last - first < 3 then
        return nil, "track too short ahead of me"
    end
    local out = {}
    for i = first, last do
        local p = points[i]
        table.insert(out, {x = p.x, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, p.x, 1, p.z), z = p.z})
    end
    return out
end

function CombineUnloaderMode:recordTrack()
    if self.trackReverse ~= nil then
        -- backing off along the track: its indices must not move
        return
    end
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    if not AutoDrive.isChopperUnloading(self.vehicle) or not AutoDrive.checkIsOnField(x, y, z) then
        -- a small patch off the field (power pole) keeps the track: only leaving the field wipes it
        local last = self.track ~= nil and self.track[#self.track] or nil
        if last ~= nil and not AutoDrive.checkIsOnField(x, y, z) and MathUtil.vector2Length(x - last.x, z - last.z) > CombineUnloaderMode.TRACK_OFF_FIELD_KEEP then
            self.track = {}
        end
        return
    end
    self.track = self.track or {}
    local speed = (self.vehicle.lastSpeedReal or 0) * (self.vehicle.movingDirection or 1)
    if speed <= 0.0005 then
        return
    end
    local last = self.track[#self.track]
    if last == nil or MathUtil.vector2Length(x - last.x, z - last.z) >= CombineUnloaderMode.TRACK_STEP then
        table.insert(self.track, {x = x, y = y, z = z})
        while #self.track > CombineUnloaderMode.TRACK_MAX_POINTS do
            table.remove(self.track, 1)
        end
    end
end

-- point of the track where the tractor can stand with room beside it for another trailer to pass, out of the
-- harvester's way, the whole train on the track; walking back from the newest point, up to maxDistance
-- after a reverse the newest track points lie AHEAD of the tractor. Distances back along the
-- track are counted from the point closest to the tractor: returns per index the m behind the tractor, and that index
CombineUnloaderMode.TRACK_REVERSE_MIN = 15 -- m at least when backing off to get free
-- backing off along a track that curves along the crop: the trailer must not end in the crop
CombineUnloaderMode.TRACK_CROP_MARGIN = 0.3        -- m beyond the train half width checked for standing crop along the track
CombineUnloaderMode.TRACK_CROP_STEP = 1.5          -- m between the crop samples along the track
CombineUnloaderMode.TRACK_CROP_CHECK_INTERVAL = 400 -- ms between the crop checks under the rear of the train while backing
CombineUnloaderMode.TRACK_REVERSE_TOTAL_MAX = 25   -- m backed off in a row without getting closer to the harvester: no more
CombineUnloaderMode.TRACK_REVERSE_PROGRESS = 5     -- m closer to the harvester than at the first back-off: the count restarts

-- standing crop at x, z (map reading, isolated plants ignored), nil when nothing can be read
function CombineUnloaderMode.cropAt(x, z)
    if PathFinderModule == nil then
        return nil
    end
    if PathFinderModule.getMapCropAt ~= nil then
        local ok, crop = pcall(PathFinderModule.getMapCropAt, x, z)
        if ok then
            return crop
        end
    end
    if PathFinderModule.readCropAt ~= nil then
        local ok, crop = pcall(PathFinderModule.readCropAt, x, z)
        if ok then
            return crop
        end
    end
    return nil
end

-- the track between two of its points (newer index first) runs clear of standing crop over the train width:
-- true, or false and the first point with crop beside it
function CombineUnloaderMode:isTrackStretchCropFree(fromIndex, toIndex)
    local track = self.track
    if track == nil or fromIndex == nil or toIndex == nil or fromIndex <= toIndex then
        return true
    end
    local halfW = self.vehicle.ad.pathFinderModule:getTrainWidth() / 2 + CombineUnloaderMode.TRACK_CROP_MARGIN
    for i = fromIndex, toIndex + 1, -1 do
        local a, b = track[i], track[i - 1]
        local dx, dz = b.x - a.x, b.z - a.z
        local l = math.sqrt(dx * dx + dz * dz)
        if l > 0.1 then
            dx, dz = dx / l, dz / l
            local nx, nz = -dz, dx
            local d = 0
            while d <= l do
                local px, pz = a.x + dx * d, a.z + dz * d
                for _, side in ipairs({-1, 0, 1}) do
                    local crop = CombineUnloaderMode.cropAt(px + nx * halfW * side, pz + nz * halfW * side)
                    if crop ~= nil and crop ~= 0 then
                        return false, px + nx * halfW * side, pz + nz * halfW * side
                    end
                end
                d = d + CombineUnloaderMode.TRACK_CROP_STEP
            end
        end
    end
    return true
end

-- standing crop under the rear of the train while backing toward tx, tz: the trailer left the track (inside of a curve)
function CombineUnloaderMode:isTrainRearInCrop(tx, tz)
    local reverseNode = self.vehicle.ad.specialDrivingModule:getReverseNode()
    if reverseNode == nil then
        return false
    end
    local rx, _, rz = getWorldTranslation(reverseNode)
    local dx, dz = tx - rx, tz - rz
    local l = math.sqrt(dx * dx + dz * dz)
    if l < 0.1 then
        return false
    end
    dx, dz = dx / l, dz / l
    local nx, nz = -dz, dx
    local halfW = self.vehicle.ad.pathFinderModule:getTrainWidth() / 2
    -- the rear corners of the trailer, 1 m behind the reversing node
    for _, side in ipairs({-1, 1}) do
        local px, pz = rx + dx * 1 + nx * halfW * side, rz + dz * 1 + nz * halfW * side
        local crop = CombineUnloaderMode.cropAt(px, pz)
        if crop ~= nil and crop ~= 0 then
            return true, px, pz
        end
    end
    return false
end

-- m backed off along the track in a row (several back-offs without getting closer to the harvester than at the first
-- one): more than TRACK_REVERSE_TOTAL_MAX and a new back-off is refused (not the same curve again and again)
function CombineUnloaderMode:trackReverseAllowed(harvester, distance)
    local d = nil
    if harvester ~= nil and harvester.components ~= nil then
        d = AutoDrive.getDistanceBetween(self.vehicle, harvester)
    end
    local run = self.trackReverseRun
    if run == nil or d == nil or run.ref == nil or d < run.ref - CombineUnloaderMode.TRACK_REVERSE_PROGRESS then
        run = {ref = d, total = 0}
        self.trackReverseRun = run
    end
    if run.total + distance > CombineUnloaderMode.TRACK_REVERSE_TOTAL_MAX then
        return false, run.total
    end
    run.total = run.total + distance
    return true, run.total
end

function CombineUnloaderMode:getTrackBehind()
    local track = self.track
    if track == nil or #track < 2 then
        return nil, nil
    end
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    local closest, best = #track, math.huge
    for i = #track, 1, -1 do
        local d = MathUtil.vector2Length(track[i].x - x, track[i].z - z)
        if d < best then
            best, closest = d, i
        end
    end
    local behind = {}
    behind[closest] = best
    for k = closest - 1, 1, -1 do
        behind[k] = behind[k + 1] + MathUtil.vector2Length(track[k + 1].x - track[k].x, track[k + 1].z - track[k].z)
    end
    return behind, closest
end

-- track index about distance m behind the tractor (the oldest one when shorter), nil without track
function CombineUnloaderMode:getTrackIndexBehind(distance)
    local behind, closest = self:getTrackBehind()
    if behind == nil then
        return nil, 0
    end
    for k = closest, 1, -1 do
        if behind[k] >= distance then
            return k, behind[k]
        end
    end
    return 1, behind[1]
end

CombineUnloaderMode.HARVESTER_LANE_LENGTH = 60 -- m ahead of the harvester kept free
CombineUnloaderMode.HARVESTER_LANE_MARGIN = 2 -- m added to the harvester half width and the train half width
CombineUnloaderMode.HARVESTER_MIN_DISTANCE = 15 -- m between a train standing aside and the harvester

-- forage harvester: a train footprint (centre cx, cz, half sizes) stands in its lane ahead, or too close to it
function CombineUnloaderMode.isInHarvesterWay(harvester, cx, cz, halfW, halfL)
    if harvester == nil or harvester.components == nil then
        return false
    end
    local hx, _, hz = getWorldTranslation(harvester.components[1].node)
    local rx, _, rz = AutoDrive.localDirectionToWorld(harvester, 0, 0, 1)
    local dx, dz = cx - hx, cz - hz
    if math.sqrt(dx * dx + dz * dz) - halfL < CombineUnloaderMode.HARVESTER_MIN_DISTANCE then
        return true
    end
    local ahead = dx * rx + dz * rz
    local side = math.abs(dx * rz - dz * rx)
    local width = (harvester.size ~= nil and harvester.size.width or 4) / 2 + halfW + CombineUnloaderMode.HARVESTER_LANE_MARGIN
    return ahead > -halfL and ahead < CombineUnloaderMode.HARVESTER_LANE_LENGTH + halfL and side < width
end

function CombineUnloaderMode:findTrackParkIndex(harvester, maxDistance)
    local track = self.track
    if track == nil or #track < 3 then
        return nil, "no tracks"
    end
    local pf = self.vehicle.ad.pathFinderModule
    local halfW = pf:getTrainWidth() / 2 + 0.5
    local trainLength = math.max(AutoDrive.getTractorTrainLength(self.vehicle, true, false), 8)
    local halfL = trainLength / 2 + 0.5
    -- distance from each point back to the oldest one: the train must stand on the track
    local fromStart = {0}
    for i = 2, #track do
        fromStart[i] = fromStart[i - 1] + MathUtil.vector2Length(track[i].x - track[i - 1].x, track[i].z - track[i - 1].z)
    end
    local behind, closest = self:getTrackBehind()
    local fallback = nil
    for k = math.min(closest, #track - 1), 2, -1 do
        local walked = behind[k]
        if walked > maxDistance then
            if fallback ~= nil then
                return fallback, behind[fallback]
            end
            return nil, string.format("nothing over %.0f m of tracks", maxDistance)
        end
        if fromStart[k] >= trainLength then
            local dx, dz = track[k].x - track[k - 1].x, track[k].z - track[k - 1].z
            local l = math.sqrt(dx * dx + dz * dz)
            if l > 0.1 then
                dx, dz = dx / l, dz / l
                local cx, cz = track[k].x - dx * trainLength / 2, track[k].z - dz * trainLength / 2
                if PathFinderModule.getFreeWidthBeside(cx, cz, dx, dz, halfW, halfL) >= PathFinderModule.PARK_PASS_MIN_WIDTH
                    and not CombineUnloaderMode.isInHarvesterWay(harvester, cx, cz, halfW, halfL) then
                    -- a bit more room when backing off, at least TRACK_REVERSE_MIN
                    if walked >= CombineUnloaderMode.TRACK_REVERSE_MIN then
                        return k, walked
                    end
                    fallback = fallback or k
                end
            end
        end
    end
    if fallback ~= nil then
        return fallback, behind[fallback]
    end
    return nil, "no place on the tracks"
end

-- backs off along the track to its parking point. Returns true when started
function CombineUnloaderMode:startTrackReverse(harvester, maxDistance, parkIndex)
    local name = tostring(self.vehicle:getName())
    local k, info = parkIndex, nil
    if k ~= nil and self.track ~= nil and self.track[k] ~= nil then
        local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
        info = MathUtil.vector2Length(self.track[k].x - x, self.track[k].z - z)
    else
        k, info = self:findTrackParkIndex(harvester, maxDistance)
    end
    if k == nil then
        return false
    end
    local reverseNode = self.vehicle.ad.specialDrivingModule:getReverseNode()
    if reverseNode == nil then
        return false
    end
    -- the reversing node aims at the track point closest to it, then further back
    local rx, _, rz = getWorldTranslation(reverseNode)
    local j, best = #self.track, math.huge
    for i = #self.track, 1, -1 do
        local d = MathUtil.vector2Length(self.track[i].x - rx, self.track[i].z - rz)
        if d < best then
            best, j = d, i
        end
    end
    -- after a reverse and a new approach along the same line, older track points lie ahead of
    -- the tractor: a parking point ahead cannot be reached backwards
    local vx, vy, vz = getWorldTranslation(self.vehicle.components[1].node)
    local _, _, parkAhead = AutoDrive.worldToLocal(self.vehicle, self.track[k].x, self.track[k].y or vy, self.track[k].z)
    if parkAhead > 0 then
        return false
    end
    -- the track may hug the crop (recorded 0.5 m from it): backing along it the trailer cuts the inside of each curve
    -- into the crop. No back-off along a track with crop within the train width
    if not self:isTrackStretchCropFree(j, k) then
        return false
    end
    if not self:trackReverseAllowed(harvester, info) then
        return false
    end
    self.trackReverse = {park = k, target = j, time = 0, stillTime = 0, minDistance = math.huge, sx = vx, sz = vz, cropCheck = 0}
    self.trackReverseMoved = 0
    return true
end

-- "running", "done" or "blocked"
function CombineUnloaderMode:updateTrackReverse(dt)
    local tr = self.trackReverse
    local sdm = self.vehicle.ad.specialDrivingModule
    if tr == nil or self.track == nil then
        return "done"
    end
    local name = tostring(self.vehicle:getName())
    local park = self.track[tr.park]
    if park == nil or (tr.target ~= nil and self.track[tr.target] == nil) then
        self:endTrackReverse()
        sdm:stopVehicle()
        sdm:update(dt)
        return "blocked"
    end
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    if tr.sx ~= nil then
        self.trackReverseMoved = MathUtil.vector2Length(x - tr.sx, z - tr.sz)
    end
    local toPark = MathUtil.vector2Length(park.x - x, park.z - z)
    tr.minDistance = math.min(tr.minDistance, toPark)
    if toPark < CombineUnloaderMode.TRACK_STOP_DISTANCE or (tr.minDistance < 6 and toPark > tr.minDistance + 1) then
        self:endTrackReverse()
        sdm:stopVehicle()
        sdm:update(dt)
        return "done"
    end
    tr.time = tr.time + dt
    if math.abs(self.vehicle.lastSpeedReal or 0) < 0.0003 then
        tr.stillTime = tr.stillTime + dt
    else
        tr.stillTime = 0
    end
    if tr.stillTime > CombineUnloaderMode.TRACK_BLOCKED_TIME or tr.time > CombineUnloaderMode.TRACK_TIMEOUT then
        self:endTrackReverse()
        sdm:stopVehicle()
        sdm:update(dt)
        return "blocked"
    end
    local reverseNode = sdm:getReverseNode()
    if reverseNode == nil then
        self:endTrackReverse()
        return "blocked"
    end
    local rx, _, rz = getWorldTranslation(reverseNode)
    while tr.target > 1 and MathUtil.vector2Length(self.track[tr.target].x - rx, self.track[tr.target].z - rz) < CombineUnloaderMode.TRACK_LOOK_BEHIND do
        tr.target = tr.target - 1
    end
    local t = self.track[tr.target]
    -- the trailer leaves the track on the inside of a curve: standing crop under its rear ends the back-off
    tr.cropCheck = (tr.cropCheck or 0) + dt
    if tr.cropCheck >= CombineUnloaderMode.TRACK_CROP_CHECK_INTERVAL then
        tr.cropCheck = 0
        if self:isTrainRearInCrop(t.x, t.z) then
            self:endTrackReverse()
            sdm:stopVehicle()
            sdm:update(dt)
            return "blocked"
        end
    end
    sdm:reverseToTargetLocation(dt, {x = t.x, y = t.y, z = t.z}, 6)
    return "running"
end

-- end of a track reverse: the points newer than the one the tractor stands at now lie ahead of it, they are dropped
-- (kept, a later back-off could aim at a point ahead and never move)
function CombineUnloaderMode:endTrackReverse()
    self.trackReverse = nil
    local track = self.track
    if track == nil or #track < 2 then
        return
    end
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    local closest, best = #track, math.huge
    for i = #track, 1, -1 do
        local d = MathUtil.vector2Length(track[i].x - x, track[i].z - z)
        if d < best then
            best, closest = d, i
        end
    end
    for i = #track, closest + 1, -1 do
        table.remove(track, i)
    end
end

-- after this many reversals the unloader gives the harvester up to another unloader, for HAND_OVER_TIME ms
CombineUnloaderMode.MAX_REVERSE_COUNT = 2
CombineUnloaderMode.HAND_OVER_TIME = 120000

function CombineUnloaderMode:notifyAboutFailedPathfinder()
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:notifyAboutFailedPathfinder self.failedPathFinder %s ADGraphManager:getDistanceFromNetwork(self.vehicle) > 20 %s", tostring(self.failedPathFinder), tostring(ADGraphManager:getDistanceFromNetwork(self.vehicle) > 20))
    --print("CombineUnloaderMode:notifyAboutFailedPathfinder() - blocked: " .. tostring(self.vehicle.ad.pathFinderModule.completelyBlocked) .. " distance: " .. ADGraphManager:getDistanceFromNetwork(self.vehicle))
    if self.vehicle.ad.pathFinderModule.completelyBlocked and ADGraphManager:getDistanceFromNetwork(self.vehicle) > 20 then
        self.failedPathFinder = self.failedPathFinder + 1
    --print("Increased Failed pathfinder count to: " .. self.failedPathFinder)
    end
end

function CombineUnloaderMode:leaveBreadCrumbs()
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local rx, _, rz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)

    if self.lastBreadCrumb == nil then
        self.lastBreadCrumb = {x = x, y = y, z = z, dirX = rx, dirZ = rz}
        self.breadCrumbs:Enqueue(self.lastBreadCrumb)
    else
        if (self.vehicle.lastSpeedReal * self.vehicle.movingDirection) > 0 then
            local _, _, diffZ = AutoDrive.worldToLocal(self.vehicle, self.lastBreadCrumb.x, self.lastBreadCrumb.y, self.lastBreadCrumb.z)
            local vec1 = {x = x - self.lastBreadCrumb.x, z = z - self.lastBreadCrumb.z}
            local angleToNewPoint = AutoDrive.angleBetween({x = self.lastBreadCrumb.dirX, z = self.lastBreadCrumb.dirZ}, vec1)
            local minDistance = 2.5
            if math.abs(angleToNewPoint) > 40 then
                minDistance = 15
            end
            if diffZ < -1 and MathUtil.vector2Length(x - self.lastBreadCrumb.x, z - self.lastBreadCrumb.z) > minDistance and math.abs(angleToNewPoint) < 90 then
                self.lastBreadCrumb = {x = x, y = y, z = z, dirX = vec1.x, dirZ = vec1.z}
                self.breadCrumbs:Enqueue(self.lastBreadCrumb)
            end
        end
    end
end

function CombineUnloaderMode:getBreadCrumbs()
    return self.breadCrumbs
end

function CombineUnloaderMode:promoteFollowingUnloader(combine)
    local task = self.vehicle.ad.taskModule ~= nil and self.vehicle.ad.taskModule.activeTask or nil
    self.combine = combine
    if self.vehicle.ad.taskModule.activeTask ~= nil and self.vehicle.ad.taskModule.activeTask.signalPromotion ~= nil then
        self.vehicle.ad.taskModule.activeTask:signalPromotion()
    end
end

function CombineUnloaderMode:handleFinishedTask()
    -- CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:handleFinishedTask")
    self.vehicle.ad.trailerModule:reset()
    self.lastTask = self.activeTask
    self.activeTask = self:getNextTask()
    if self.activeTask ~= nil then
        self.vehicle.ad.taskModule:addTask(self.activeTask)
    end
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:handleFinishedTask self.state %s", tostring(self:getStateName()))
end

function CombineUnloaderMode:stop()
end

function CombineUnloaderMode:continue()
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local point = nil
    local distanceToStart = 0
    if
        self.vehicle.ad ~= nil and ADGraphManager.getWayPointById ~= nil and self.vehicle.ad.stateModule ~= nil and self.vehicle.ad.stateModule.getFirstMarker ~= nil and self.vehicle.ad.stateModule:getFirstMarker() ~= nil and
            self.vehicle.ad.stateModule:getFirstMarker() ~= 0 and
            self.vehicle.ad.stateModule:getFirstMarker().id ~= nil
     then
        point = ADGraphManager:getWayPointById(self.vehicle.ad.stateModule:getFirstMarker().id)
        if point ~= nil then
            distanceToStart = MathUtil.vector2Length(x - point.x, z - point.z)
        end
    end

    -- (an unloader left to unload half full, nobody pressed anything): below the unload fill level
    -- setting, only the player's "continue" sends an unloader to unload
    if self.state ~= self.STATE_DRIVE_TO_UNLOAD and self.vehicle.ad.continueByPlayer ~= true then
        local trailers = (self.trailers ~= nil and #self.trailers > 0) and self.trailers or AutoDrive.getAllUnits(self.vehicle)
        local filled = false
        if trailers ~= nil and #trailers > 0 then
            _, _, filled = AutoDrive.getAllFillLevels(trailers)
        end
        if not filled then
            printCallstack()
            return
        end
    end

    if self.state == self.STATE_DRIVE_TO_UNLOAD then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:continue self.STATE_DRIVE_TO_UNLOAD")
        self.activeTask:continue()
    else
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:continue self.state" .. tostring(self:getStateName()))
        self.vehicle.ad.taskModule:abortCurrentTask()

        if AutoDrive.checkIsOnField(x, y, z) and distanceToStart > 30 then
            -- is activated on a field - use ExitFieldTask to leave field according to setting
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:continue ExitFieldTask...")
            self.activeTask = ExitFieldTask:new(self.vehicle)
            self.state = self.STATE_EXIT_FIELD
        else
            if (AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYDELIVER or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders") then
                local nextTarget = ADMultipleTargetsManager:getNextTarget(self.vehicle, false)
                if nextTarget ~= nil then
                    self.vehicle.ad.stateModule:setSecondMarker(nextTarget)
                end
            end
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:continue UnloadAtDestinationTask...")
            self.activeTask = UnloadAtDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getSecondMarker().id)
            self.state = self.STATE_DRIVE_TO_UNLOAD
        end

        -- leaves to unload: back at the end of the queue on return
        self.vehicle.ad.unloaderQueueTime = nil
    ADHarvestManager.releaseStickyUnloader(self.vehicle)
        ADHarvestManager:unregisterAsUnloader(self.vehicle)
        self.followingUnloader = nil
        self.combine = nil
        self.vehicle.ad.taskModule:addTask(self.activeTask)
    end
end

function CombineUnloaderMode:getNextTask()
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask start self.state %s", tostring(self:getStateName()))
    local nextTask

    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local point = nil
    local _, _, filledToUnload, _ = AutoDrive.getAllFillLevels(self.trailers)

    if self.state == self.STATE_INIT then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask STATE_INIT filledToUnload %s", tostring(filledToUnload))
        if filledToUnload then -- fill level above setting unload level
            if (AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYDELIVER or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders") then
                local nextTarget = ADMultipleTargetsManager:getNextTarget(self.vehicle, false)
                if nextTarget ~= nil then
                    self.vehicle.ad.stateModule:setSecondMarker(nextTarget)
                end
            end

            nextTask = self:getTaskAfterUnload(filledToUnload)

            -- leaves to unload: back at the end of the queue on return
            self.vehicle.ad.unloaderQueueTime = nil
    ADHarvestManager.releaseStickyUnloader(self.vehicle)
            ADHarvestManager:unregisterAsUnloader(self.vehicle)
            self.followingUnloader = nil
            self.combine = nil
        else
            if not AutoDrive.checkIsOnField(x, y, z) then
                nextTask = DriveToDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getFirstMarker().id)
                self.state = self.STATE_DRIVE_TO_START
            else
                self:setToWaitForCall()
            end
        end
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_INIT end self.state %s", tostring(self:getStateName()))
    elseif self.state == self.STATE_DRIVE_TO_COMBINE then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_DRIVE_TO_COMBINE")
        -- we finished the precall to combine route
        -- check if we should wait / pull up to combines pipe
        if self.reversedTurnRequested then
            -- forage harvester, behind it the wrong way round: AutoDrive U-turn first, its fallbacks after
            self.reversedTurnRequested = nil
            nextTask = HandleHarvesterTurnTask:new(self.vehicle, self.combine)
            return nextTask
        end
        nextTask = FollowCombineTask:new(self.vehicle, self.combine)
        self.state = self.STATE_ACTIVE_UNLOAD_COMBINE
        -- forage harvester: the way this unloader came is clean and drivable, the next ones reuse it
        if self.combine ~= nil and self.combine.ad ~= nil and self.combine.ad.isChopper and self.track ~= nil and #self.track >= 5 then
            -- (seen in game: its own loops, U-turn and 43 m reverse kept as "the way to the harvester",
            -- ending 56 m aside of it, then followed in circles): kept only when it really ends at the harvester
            local laneEnd = AutoDrive.getDistanceBetween(self.vehicle, self.combine)
            if laneEnd <= CombineUnloaderMode.LANE_KEEP_DISTANCE then
                local lane = {}
                for i, p in ipairs(self.track) do
                    lane[i] = {x = p.x, y = p.y, z = p.z}
                end
                self.combine.ad.sharedLane = {points = lane, time = g_time}
            end
        end
        self.reverseCount = 0
        self.breadCrumbs = Queue:new()
        self.lastBreadCrumb = nil
        -- the chase takes over: the wrong-way arrivals and the extra room for the turn are forgotten
        self.vehicle.ad.turnRoomExtra = nil
        self.vehicle.ad.wrongWay = nil
    elseif self.state == self.STATE_DRIVE_TO_PIPE or self.state == self.STATE_REVERSE_FROM_BAD_LOCATION then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_DRIVE_TO_PIPE | STATE_REVERSE_FROM_BAD_LOCATION")
        --Drive to pipe can be finished when combine is emptied or when vehicle has reached 'old' pipe position and should switch to active mode
        nextTask = self:getTaskAfterUnload(filledToUnload)
    elseif self.state == self.STATE_LEAVE_CROP then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_LEAVE_CROP")
        self:setToWaitForCall()
    elseif self.state == self.STATE_DRIVE_TO_UNLOAD and self.vehicle.ad.harvestJobDone and self:getParkAfterJobMarker() ~= nil then
        -- unloaded after the harvester's end of job: to the park destination, not back to the field
        nextTask = self:getParkAfterJobTask()
    elseif self.state == self.STATE_PARK then
        self.vehicle.ad.onRouteToPark = false
        if self.parkAfterJobName ~= nil then
            AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.INFO, "$l10n_AD_Driver_of; %s $l10n_AD_has_reached; %s", 5000, self.vehicle.ad.stateModule:getName(), self.parkAfterJobName)
        end
        nextTask = StopAndDisableADTask:new(self.vehicle, ADTaskModule.DONT_PROPAGATE)
    elseif self.state == self.STATE_DRIVE_TO_START and self.vehicle.ad.harvestJobDone and self:getParkAfterJobMarker() ~= nil then
        nextTask = self:getParkAfterJobTask()
    elseif self.state == self.STATE_DRIVE_TO_UNLOAD then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_DRIVE_TO_UNLOAD")
        if self.vehicle.ad.harvestJobDone then
            self.vehicle.ad.harvestJobDone = nil
        end
        nextTask = DriveToDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getFirstMarker().id)
        if (AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYDELIVER or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders") then
            local nextTarget = ADMultipleTargetsManager:getNextTarget(self.vehicle, true)
            if nextTarget ~= nil then
                self.vehicle.ad.stateModule:setSecondMarker(nextTarget)
            end
        end
        self.state = self.STATE_DRIVE_TO_START
    elseif self.state == self.STATE_DRIVE_TO_START then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_DRIVE_TO_START")
        self:setToWaitForCall()
    elseif self.state == self.STATE_ACTIVE_UNLOAD_COMBINE then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_ACTIVE_UNLOAD_COMBINE")
        if filledToUnload then
            nextTask = self:getTaskAfterUnload(filledToUnload)
        elseif self.lastTask ~= nil and self.lastTask.lineBlocked and not self.lastTask.combineInTurn then
            -- chase given up on a blocked straight line: plan a route to the combine
            nextTask = CatchCombinePipeTask:new(self.vehicle, self.combine)
            self.state = self.STATE_DRIVE_TO_COMBINE
            -- forage harvester close by: the chase drives; after CLOSE_BLOCKED_TRIES blocked chases at the same
            -- place (it cannot reach it in a straight line), a planned route is allowed once
            local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
            if self.closeBlocked ~= nil and MathUtil.vector2Length(x - self.closeBlocked.x, z - self.closeBlocked.z) < 5 then
                self.closeBlocked.count = self.closeBlocked.count + 1
            else
                self.closeBlocked = {x = x, z = z, count = 1}
            end
            -- forage harvester: no grid route near it any more (it looped into trees and crop), the crop edge
            -- following of the catch task handles corners; the planned route is still allowed for a combine harvester
            if self.closeBlocked.count >= CatchCombinePipeTask.CLOSE_BLOCKED_TRIES and self.combine ~= nil and self.combine.ad ~= nil and self.combine.ad.isChopper then
                -- forage harvester: the straight line is blocked by the crop (a corner): follow the crop edge around it,
                -- never a grid route this close
                nextTask.closeRangePlanning = true
                nextTask.edgeOnly = true
                self.closeBlocked = nil
            elseif self.closeBlocked.count >= CatchCombinePipeTask.CLOSE_BLOCKED_TRIES then
                nextTask.closeRangePlanning = true
                self.closeBlocked = nil
            end
        else
            nextTask = HandleHarvesterTurnTask:new(self.vehicle, self.combine)
            self.state = self.STATE_DRIVE_TO_COMBINE
        end
    elseif self.state == self.STATE_FOLLOW_CURRENT_UNLOADER then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_FOLLOW_CURRENT_UNLOADER")
        if self.targetUnloader ~= nil then
            self.targetUnloader.ad.modes[AutoDrive.MODE_UNLOAD]:unregisterFollowingUnloader()
        end
        nextTask = self:getTaskAfterUnload(filledToUnload)
    elseif self.state == self.STATE_EXIT_FIELD then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_EXIT_FIELD")
        if (AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYDELIVER or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders") then
            local nextTarget = ADMultipleTargetsManager:getNextTarget(self.vehicle, false)
            if nextTarget ~= nil then
                self.vehicle.ad.stateModule:setSecondMarker(nextTarget)
            end
        end
        nextTask = UnloadAtDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getSecondMarker().id)
        self.state = self.STATE_DRIVE_TO_UNLOAD
    end

    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask end self.state %s", tostring(self:getStateName()))
    return nextTask
end

function CombineUnloaderMode:setToWaitForCall(keepCombine)
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:setToWaitForCall start self.state %s keepCombine %s", tostring(self:getStateName()), tostring(keepCombine))
    -- We just have to wait to be wait to be called (again)
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local point = nil
    local distanceToStart = 0
    if
        self.vehicle.ad ~= nil and ADGraphManager.getWayPointById ~= nil and self.vehicle.ad.stateModule ~= nil and self.vehicle.ad.stateModule.getFirstMarker ~= nil
        and self.vehicle.ad.stateModule:getFirstMarker() ~= nil and self.vehicle.ad.stateModule:getFirstMarker() ~= 0 and self.vehicle.ad.stateModule:getFirstMarker().id ~= nil
    then
        point = ADGraphManager:getWayPointById(self.vehicle.ad.stateModule:getFirstMarker().id)
        if point ~= nil then
            distanceToStart = MathUtil.vector2Length(x - point.x, z - point.z)
        end
    end

    local _, _, filledToUnload, _ = AutoDrive.getAllFillLevels(self.trailers)
    if filledToUnload then
        -- full (unload fill level setting): leaves to unload, the next unloader of the queue takes over
        self.vehicle.ad.unloaderQueueTime = nil
        ADHarvestManager.releaseStickyUnloader(self.vehicle)
        if AutoDrive.checkIsOnField(x, y, z) and distanceToStart > 30 then
            -- is activated on a field - use ExitFieldTask to leave field according to setting
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:setToWaitForCall ExitFieldTask...")
            self.vehicle.ad.taskModule:addTask(ExitFieldTask:new(self.vehicle))
            self.state = self.STATE_EXIT_FIELD
        else
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:setToWaitForCall UnloadAtDestinationTask...")
            self.vehicle.ad.taskModule:addTask(UnloadAtDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getSecondMarker().id))
            self.state = self.STATE_DRIVE_TO_UNLOAD
        end
    else
	    self.state = self.STATE_WAIT_TO_BE_CALLED
	    self.vehicle.ad.taskModule:addTask(WaitForCallTask:new(self.vehicle))
	    if self.combine ~= nil and self.combine.ad ~= nil and (keepCombine == nil or keepCombine ~= true) then
	        self.combine = nil
	    end
    end
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:setToWaitForCall end self.state %s self.combine %s", tostring(self:getStateName()), tostring(self.combine))
end

function CombineUnloaderMode:assignToHarvester(harvester)
    if self.state == self.STATE_WAIT_TO_BE_CALLED then

        self.vehicle.ad.taskModule:abortCurrentTask()
        self.combine = harvester
        -- kept after the unloader leaves the harvester (exit task: let the next unloader pass)
        self.vehicle.ad.lastHarvester = harvester
        self.vehicle.ad.workingForChopper = harvester.ad ~= nil and harvester.ad.isChopper == true
        self.combineRootVehicle = self.combine:getRootVehicle()
        AutoDrive.validateCachedPipeData(harvester)

        -- if combine has extended pipe, aim for that. Otherwise DriveToVehicle and choose from there
        if AutoDrive.isPipeOut(self.combine) then

            local cfillLevel, cfillCapacity, _, cleftCapacity = AutoDrive.getObjectFillLevels(self.combine)
            local cFillRatio = cfillCapacity > 0 and cfillLevel / cfillCapacity or 0

            if (self.combine.ad.isHarvester) 
                and (self.combine.ad.noMovementTimer.elapsedTime > 500 or cleftCapacity < 0.1 or cFillRatio > 0.945)
            then
                -- default unloading - no movement
                self.state = self.STATE_DRIVE_TO_PIPE
                self.vehicle.ad.taskModule:addTask(EmptyHarvesterTask:new(self.vehicle, self.combine))
            else
                -- Probably active unloading for choppers and moving combines
                self.state = self.STATE_DRIVE_TO_COMBINE
                self.vehicle.ad.taskModule:addTask(CatchCombinePipeTask:new(self.vehicle, self.combine))
            end
        else
            self.state = self.STATE_DRIVE_TO_COMBINE
            self.vehicle.ad.taskModule:addTask(CatchCombinePipeTask:new(self.vehicle, self.combine))
        end
    end
end

function CombineUnloaderMode:driveToUnloader(unloader)
    if self.state == self.STATE_WAIT_TO_BE_CALLED then
        self.vehicle.ad.taskModule:abortCurrentTask()
        self.vehicle.ad.taskModule:addTask(DriveToVehicleTask:new(self.vehicle, unloader))
        unloader.ad.modes[AutoDrive.MODE_UNLOAD]:registerFollowingUnloader(self.vehicle)
        self.targetUnloader = unloader
        self.state = self.STATE_FOLLOW_CURRENT_UNLOADER
    end
end

function CombineUnloaderMode:getTaskAfterUnload(filledToUnload)
    local nextTask
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getTaskAfterUnload start filledToUnload %s", tostring(filledToUnload))

    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local point = nil
    local distanceToStart = 0
    if
        self.vehicle.ad ~= nil and ADGraphManager.getWayPointById ~= nil and self.vehicle.ad.stateModule ~= nil and self.vehicle.ad.stateModule.getFirstMarker ~= nil and self.vehicle.ad.stateModule:getFirstMarker() ~= nil and
            self.vehicle.ad.stateModule:getFirstMarker() ~= 0 and
            self.vehicle.ad.stateModule:getFirstMarker().id ~= nil
     then
        point = ADGraphManager:getWayPointById(self.vehicle.ad.stateModule:getFirstMarker().id)
        if point ~= nil then
            distanceToStart = MathUtil.vector2Length(x - point.x, z - point.z)
        end
    end

    if filledToUnload then
        self.vehicle.ad.unloaderQueueTime = nil
        ADHarvestManager.releaseStickyUnloader(self.vehicle)
        --ADHarvestManager:unregisterAsUnloader(self.vehicle)
        --self.followingUnloader = nil
        --self.combine = nil
        if AutoDrive.checkIsOnField(x, y, z) and distanceToStart > 30 then
            -- is activated on a field - use ExitFieldTask to leave field according to setting
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getTaskAfterUnload ExitFieldTask...")
            nextTask = ExitFieldTask:new(self.vehicle)
            self.state = self.STATE_EXIT_FIELD
        else
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getTaskAfterUnload UnloadAtDestinationTask...")
            nextTask = UnloadAtDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getSecondMarker().id)
            self.state = self.STATE_DRIVE_TO_UNLOAD
        end
    else
        -- Should we park in the field?
        if (self.combine and self.combine.ad.isChopper) or (AutoDrive.getSetting("parkInField", self.vehicle) or (self.lastTask ~= nil and self.lastTask.stayOnField)) then
            -- If we are in fruit, we should clear it
            if AutoDrive.isVehicleOrTrailerInCrop(self.vehicle, true) then
                CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getTaskAfterUnload ClearCropTask...")
                nextTask = ClearCropTask:new(self.vehicle, self.combine)
                self.state = self.STATE_LEAVE_CROP
            else
                CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getTaskAfterUnload setToWaitForCall")
                self:setToWaitForCall()
            end
        else
            ADHarvestManager:unregisterAsUnloader(self.vehicle)
            nextTask = DriveToDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getFirstMarker().id)
            self.state = self.STATE_DRIVE_TO_START
        end
    end
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getTaskAfterUnload end")
    return nextTask
end

function CombineUnloaderMode:shouldLoadOnTrigger()
    return false
end

function CombineUnloaderMode:shouldUnloadAtTrigger()
    return self.state == self.STATE_DRIVE_TO_UNLOAD
end

function CombineUnloaderMode:getUnloaderOnSide()
    local vehicleX, vehicleY, vehicleZ = getWorldTranslation(self.vehicle.components[1].node)
    local diffX, _, diffZ = AutoDrive.worldToLocal(self.combine, vehicleX, vehicleY, vehicleZ, self.combine.ad.ADRootNode)
    local leftright = AutoDrive.CHASEPOS_UNKNOWN
    local frontback = AutoDrive.CHASEPOS_FRONT
    -- If we're not clearly on one side or the other of the combine, we don't
    -- give a clear answer
    if math.abs(diffX) > self.combine.size.width / 2 then
        leftright = AutoDrive.sign(diffX)
    end

    local maxZ = self.tractorTrainLength
    if diffZ < maxZ then
        frontback = AutoDrive.CHASEPOS_REAR
    end

    return leftright, frontback
end

function CombineUnloaderMode:isUnloaderOnCorrectSide(chaseSide)
    local sideIndex = chaseSide

    if sideIndex == nil and self.chasePosIndex == nil then
        return false
    elseif sideIndex == nil then
        sideIndex = self.chasePosIndex
    end

    local leftRight, frontBack = self:getUnloaderOnSide()

    if (leftRight == sideIndex and frontBack == AutoDrive.CHASEPOS_REAR) or (leftRight == AutoDrive.CHASEPOS_UNKNOWN and frontBack == AutoDrive.CHASEPOS_REAR) or frontBack == sideIndex then
        return true
    else
        return false
    end
end

function CombineUnloaderMode:getPipeSlopeCorrection2()
    local dischargeX, dichargeY, dischargeZ = getWorldTranslation(AutoDrive.getDischargeNode(self.combine))
    local diffX, diffY, _ = AutoDrive.worldToLocal(self.combine, dischargeX, dichargeY, dischargeZ, self.combine.ad.ADRootNode)
    if math.abs(diffX) < self.combine.size.width / 2 then
        -- Some pipes curl up so tight they cause a collisions.
        -- We just don't try to correct in this case.
        return 0
    end

    local heightUnderPipe = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, dischargeX, dichargeY, dischargeZ)
    -- It would be nice if we could use the discharge node direction and the terrain under the harveser,
    -- but discharge node rotations are untrustworthy.
    local wux, wuy, wuz = getTerrainNormalAtWorldPos(g_currentMission.terrainRootNode, dischargeX, dichargeY - heightUnderPipe, dischargeZ)
    local ux, uy, uz = AutoDrive.worldDirectionToLocal(self.combine, wux, wuy, wuz, self.combine.ad.ADRootNode)

    -- This is backwards from the usual order of these variables, but I need deviation from 0 and pi, not
    -- pi/2 and 3pi/4, so we adjust the coordinate system
    local theta = math.atan(ux / uy)

    local pipePosition = diffX * math.cos(theta) + diffY * math.sin(theta)
    local currentElevationCorrection = pipePosition - diffX

    if math.abs(currentElevationCorrection) > 1 then
        -- Assume something has gone very wrong if the correction gets too large.
        return 0
    end

    return currentElevationCorrection
end

function CombineUnloaderMode:getPipeSlopeCorrection1()
    self.combineX, self.combineY, self.combineZ = getWorldTranslation(self.combineNode)
    local nodeX, nodeY, nodeZ = getWorldTranslation(AutoDrive.getDischargeNode(self.combine))
    local heightUnderCombine = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, self.combineX, self.combineY, self.combineZ)
    local heightUnderPipe = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, nodeX, nodeY, nodeZ)
    -- want this to be negative if the ground is lower under the pipe
    local dh = heightUnderPipe - heightUnderCombine
    local hyp = MathUtil.vector3Length(self.combineX - nodeX, heightUnderCombine - heightUnderPipe, self.combineZ - nodeZ)
    local run = math.sqrt(hyp * hyp - dh * dh)
    local elevationCorrection = (hyp + (nodeY - heightUnderPipe) * (dh / hyp)) - run
    return elevationCorrection * self.pipeSide
end

function CombineUnloaderMode:getPipeSlopeCorrection()
    return self:getPipeSlopeCorrection2()
end

function CombineUnloaderMode:getSideChaseOffsetX()
    -- NB: We cannot apply slope correction until after we have chosen which side
    -- we are chasing on! This function only finds the base X offset "to the left".
    -- Slope and side correction MUST be applied in CombineUnloaderMode:getPipeChasePosition
    -- AFTER determining the chase side. Or this function needs to be rewritten.
    local pipeOffset = AutoDrive.getSetting("pipeOffset", self.vehicle)
    local unloaderWidest = self.vehicle.size.width
    local headerExtra = math.max((AutoDrive.getFrontToolWidth(self.combine) - self.combine.size.width) / 2, 0)

    local sideChaseTermPipeIn = self.combine.size.width / 2 + unloaderWidest / 2 + headerExtra
    local sideChaseTermPipeOut = AutoDrive.getPipeLength(self.combine)
    -- Some combines fold up their pipe so tight that targeting it could cause a collision.
    -- So, choose the max between the two to avoid a collison
    local sideChaseTermX = math.max(sideChaseTermPipeIn, sideChaseTermPipeOut)

    if self.combine.ad.isSugarcaneHarvester then
        -- check for SugarcaneHarvester has to be first, as it also is IsBufferCombine!
        sideChaseTermX = AutoDrive.getPipeLength(self.combine)
    elseif self.combine.ad.isAutoAimingChopper then
        sideChaseTermX = sideChaseTermPipeIn + CombineUnloaderMode.STATIC_X_OFFSET_FROM_HEADER
    elseif (self.combine.ad ~= nil and self.combine.ad.storedPipeLength ~= nil) or AutoDrive.isPipeOut(self.combine) then
        -- If the pipe is extended, though, target it regardless
        sideChaseTermX = sideChaseTermPipeOut
    end

    return sideChaseTermX + pipeOffset
end

function CombineUnloaderMode:getSideChaseOffsetX_new()
    -- NB: We cannot apply slope correction until after we have chosen which side
    -- we are chasing on! This function only finds the base X offset "to the left".
    -- Slope and side correction MUST be applied in CombineUnloaderMode:getPipeChasePosition
    -- AFTER determining the chase side. Or this function needs to be rewritten.
    local pipeOffset = AutoDrive.getSetting("pipeOffset", self.vehicle)
    local unloaderWidest = self.vehicle.size.width
    local headerExtra = math.max((AutoDrive.getFrontToolWidth(self.combine) - self.combine.size.width) / 2, 0)

    local sideChaseTermPipeIn = self.combine.size.width / 2 + unloaderWidest / 2 + headerExtra
    -- Some combines fold up their pipe so tight that targeting it could cause a collision.
    -- So, choose the max between the two to avoid a collison
    local sideChaseTermX = 0

    if self.combine.ad.isSugarcaneHarvester then
        -- check for SugarcaneHarvester has to be first, as it also is IsBufferCombine!
        sideChaseTermX = AutoDrive.getPipeLength(self.combine)
    elseif self.combine.ad.isAutoAimingChopper then
        sideChaseTermX = sideChaseTermPipeIn + CombineUnloaderMode.STATIC_X_OFFSET_FROM_HEADER
    elseif (self.combine.ad ~= nil and self.combine.ad.storedPipeLength ~= nil) or AutoDrive.isPipeOut(self.combine) then
        -- If the pipe is extended, though, target it regardless
        sideChaseTermX = 0
    end

    return sideChaseTermX + pipeOffset
end

function CombineUnloaderMode:getPipeChaseWayPoint(offsetX, offsetZ)
    local wayPoint = {x = 0, y = 0, z = 0}
    local trailer = g_currentMission.nodeToObject[self.targetFillNode]
    if trailer == nil then
        CombineUnloaderMode.debugMsg(self.vehicle, "ERROR: CombineUnloaderMode:getPipeChaseWayPoint trailer == nil")
        trailer = self.vehicle
    end
    local _, trailerDistY, _ = localToLocal(self.targetFillNode, trailer.components[1].node, 0, 0, 0)
    local dischargeNode = AutoDrive.getDischargeNode(self.combine)
    if dischargeNode == nil then
        CombineUnloaderMode.debugMsg(self.vehicle, "ERROR: CombineUnloaderMode:getPipeChaseWayPoint dischargeNode == nil")
        dischargeNode = self.combine.components[1].node
    end
    local combinePipeRootNode = AutoDrive.getPipeRoot(self.combine)
    if combinePipeRootNode == nil then
        CombineUnloaderMode.debugMsg(self.vehicle, "ERROR: CombineUnloaderMode:getPipeChaseWayPoint combinePipeRootNode == nil")
        combinePipeRootNode = self.combine.components[1].node
    end
    local targetX, targetY, targetZ = getWorldTranslation(dischargeNode)
    local combineX, combineY, combineZ = getWorldTranslation(combinePipeRootNode)
    local _, combineDistY, _ = localToLocal(dischargeNode, combinePipeRootNode, 0, 0, 0)
    local diffY = combineDistY - trailerDistY -- distance discharge to fill point
    local pipeSide = self.pipeSide
    if pipeSide == 0 then
        pipeSide = 1
    end
    local pipeOffset = AutoDrive.getSetting("pipeOffset", self.vehicle)
    local worldOffsetX1, worldOffsetY1, worldOffsetZ1 = AutoDrive.localDirectionToWorld(self.combine, 0, -diffY, 0, dischargeNode)
    local targetDistX, _, targetDistZ = AutoDrive.worldToLocal(self.combine, targetX + worldOffsetX1, targetY + worldOffsetY1, targetZ + worldOffsetZ1, combinePipeRootNode)
    local worldOffsetX2, worldOffsetY2, worldOffsetZ2 = AutoDrive.localDirectionToWorld(self.combine, targetDistX + (offsetX or 0) + (pipeSide * pipeOffset), 0, targetDistZ + (offsetZ or 0), combinePipeRootNode)
    wayPoint.x = combineX + worldOffsetX2
    wayPoint.y = combineY + worldOffsetY2
    wayPoint.z = combineZ + worldOffsetZ2
    return wayPoint
end

-- the trailer fill node was only read in getPipeChasePosition (the chase). The AutoDrive
-- U-turn task measures the place at the pipe (ForwardOffsetTurn) and can now run before any chase, e.g. an unloader called
-- 10 m from the stopped harvester right after a game load: node nil, 599 Lua errors in 19 s. Read it when missing or gone.
function CombineUnloaderMode:ensureTargetFillNode()
    local node = self.targetFillNode
    if node ~= nil and (entityExists == nil or entityExists(node)) then
        return node
    end
    if self.vehicle ~= nil and AutoDrive.getNextFreeDischargeableUnit ~= nil then
        self.targetFillUnit, self.targetFillNode, self.targetAutoAimTargetNode = AutoDrive.getNextFreeDischargeableUnit(self.vehicle)
    end
    node = self.targetFillNode
    if node ~= nil and entityExists ~= nil and not entityExists(node) then
        node = nil
    end
    return node
end

function CombineUnloaderMode:getDynamicSideChaseOffsetZ()
    local targetFillNode = self:ensureTargetFillNode()
    local aimNode = self.targetAutoAimTargetNode
    if self.combine.ad.isAutoAimingChopper and aimNode ~= nil and (entityExists == nil or entityExists(aimNode)) then
        targetFillNode = aimNode
    end
    local dischargeNode = AutoDrive.getDischargeNode(self.combine)
    if targetFillNode == nil or dischargeNode == nil then
        -- same value as with the dynamic chase distance off (origin AutoDrive)
        return self:getSideChaseOffsetZ(false)
    end
    local nodeX, nodeY, nodeZ = getWorldTranslation(dischargeNode)
    local _, _, pipeZOffsetToCombine = AutoDrive.worldToLocal(self.combine, nodeX, nodeY, nodeZ, self.combine.ad.ADRootNode)
    local targetX, targetY, targetZ = getWorldTranslation(targetFillNode)
    local _, _, vehicleZOffsetToTarget = AutoDrive.worldToLocal(self.vehicle, targetX, targetY, targetZ)
    if pipeZOffsetToCombine == nil or vehicleZOffsetToTarget == nil then
        return self:getSideChaseOffsetZ(false)
    end
    local sideChaseTermZ = pipeZOffsetToCombine - vehicleZOffsetToTarget
    return sideChaseTermZ
end

function CombineUnloaderMode:getDynamicSideChaseOffsetZ_fromDischargeNode(planningPhase)
    local targetFillNode = self:ensureTargetFillNode()
    local vehicleZOffsetToTarget = nil
    if targetFillNode ~= nil then
        local targetX, targetY, targetZ = getWorldTranslation(targetFillNode)
        local _, _, lz = AutoDrive.worldToLocal(self.vehicle, targetX, targetY, targetZ)
        vehicleZOffsetToTarget = lz
    end
    if vehicleZOffsetToTarget == nil then
        -- no node: the fill point is taken at the middle of the train behind the tractor
        vehicleZOffsetToTarget = -((self.tractorTrainLength ~= nil and self.tractorTrainLength > 0) and self.tractorTrainLength or self.vehicle.size.length) / 2
    end
    local offset = -vehicleZOffsetToTarget
    if planningPhase then
        offset = offset + (self.vehicle.size.length/2)
    end
    return offset
end

function CombineUnloaderMode:getDynamicSideChaseOffsetZ_FS19()
    -- The default maximum will place the front of the unloader at the back of the header
    local pipeZOffsetToCombine = 0
    if AutoDrive.isPipeOut(self.combine) then
        local nodeX, nodeY, nodeZ = getWorldTranslation(AutoDrive.getDischargeNode(self.combine))
        local _, _, diffZ = AutoDrive.worldToLocal(self.combine, nodeX, nodeY, nodeZ, self.combine.ad.ADRootNode)
        pipeZOffsetToCombine = diffZ
    end
    local vehicleX, vehicleY, vehicleZ = getWorldTranslation(self.vehicle.components[1].node)

--[[
    -- AimTargetNode - not used as it not always fit to the fill area of trailers
    if self.targetTrailer.getFillUnitAutoAimTargetNode ~= nil then
        local node = self.targetTrailer:getFillUnitAutoAimTargetNode(1)
        if node ~= nil then
            -- use dynamic fill in trailer if available, to better follow unloading the harvester
            targetNode = node
        end
    end
]]
    local _, _, diffZ = AutoDrive.worldToLocal(self.targetTrailer, vehicleX, vehicleY, vehicleZ)

    -- We gradually move the chase node forward as a function of fill level because it's pretty and
    -- helps the sugarcane harvester. We start at at the front of the trailer. We use an exponential
    -- to increase dwell time towards the front of the trailer, since loads migrate towards the back.

    -- The constant additions should put at precisely at the joint of the vehicle and trailer, then correct for
    -- only moving the midpoint of the tractor
    local constantAdditionsZ = 1 + self.vehicle.size.length / 2 - self.targetTrailer.size.length / 2
    -- We then gradually move back, but don't use the last part of trailer for cosmetic reasons
    local dynamicAdditionsZ = diffZ + pipeZOffsetToCombine
    dynamicAdditionsZ = dynamicAdditionsZ + math.max((self.targetTrailer.size.length * 0.5 - 2) ^ self.targetTrailerFillRatio, 0)
    local sideChaseTermZ = constantAdditionsZ + dynamicAdditionsZ
    return sideChaseTermZ
end

function CombineUnloaderMode:getSideChaseOffsetZ(dynamic)
    if dynamic then
        return self:getDynamicSideChaseOffsetZ()
    else
        return (self.combine.size.length - self.vehicle.size.length + AutoDrive.getFrontToolLength(self.combine)) / 2
    end
end

function CombineUnloaderMode:getRearChaseOffsetX(leftBlocked, rightBlocked)
    local rearChaseOffset = (self.combine.size.width / 2 + self.vehicle.size.width / 2) + 1

    if self.combine.ad.isAutoAimingChopper and not self.combine.ad.isSugarcaneHarvester then
        return 0
    elseif rightBlocked and leftBlocked then
        return 0
    elseif leftBlocked then
        return -rearChaseOffset
    else
        return rearChaseOffset
    end
end

CombineUnloaderMode.REAR_CHASE_GAP = 0.5 -- m between the tractor nose and the forage harvester rear when chasing behind
function CombineUnloaderMode:getRearChaseOffsetZ(planningPhase)
    local followDistance = AutoDrive.getSetting("followDistance", self.vehicle)
    local rearChaseOffset = -followDistance - (self.combine.size.length / 2)
    if self.combine.ad.isAutoAimingChopper and not self.combine.ad.isSugarcaneHarvester then
        if planningPhase then
            rearChaseOffset = - (self.combine.size.length / 2)
        else
            -- behind it, it often pushed the harvester. The chase place is the tractor root (rear axle),
            -- its nose is further ahead: front of the tractor REAR_CHASE_GAP behind the harvester rear (0.5 m)
            local size = self.vehicle.size or {}
            local front = (size.length or 6) / 2 + (size.lengthOffset or 0)
            local csize = self.combine.size or {}
            local rear = (csize.length or 10) / 2 - (csize.lengthOffset or 0)
            rearChaseOffset = math.min(rearChaseOffset, -(rear + front + CombineUnloaderMode.REAR_CHASE_GAP))
        end
    else
        -- math.sqrt(2) ensures the trailer could straighten if it was turned 90 degrees, and it makes this point further
        -- back than the pathfinder (straightening) target in PathFinderModule:startPathPlanningToPipe
        -- math.sqrt(2) gives the hypotenuse of an isosceles right trangle with side length equal to the length
        -- of the trailer
        if self.combine.ad.isSugarcaneHarvester then
            rearChaseOffset = -self.combine.size.length / 2 - self.tractorTrainLength * math.sqrt(2)
        else
            local combineSensors = self.combine.ad.sensors or self.combineRootVehicle.ad.sensors
            if self.combine.lastSpeedReal > 0.002 and combineSensors.frontSensorFruit:pollInfo() then
                rearChaseOffset = -10
            else
                --there is no need to be close to the rear of the harvester here. We can make it hard on the pathfinder since we have no strong desire to chase there anyway for normal harvesters
                --Especially when they are CP driven, we have to be prepared for that massive reverse maneuver when the combine is filled and wants to avoid the crop.
                rearChaseOffset = -45
            end
        end
    end

    return rearChaseOffset
end

-- forage harvester in a curve toward the unloader: the unloader drives at its 3/4 rear, so that the
-- harvester can turn; back beside it once the curve ends. The curve is read on the harvester's own track: lateral
-- offset of its position CURVE_BACK m ago (on the side it turns to)
CombineUnloaderMode.CURVE_BACK = 5
CombineUnloaderMode.CURVE_IN = 1.3 -- m, about 15 deg (0.6 took the 3/4 rear on a small kink)
CombineUnloaderMode.CURVE_OUT = 0.2 -- m (about 4-5 deg)
CombineUnloaderMode.CURVE_REAR_GAP = 0 -- m behind the harvester rear (further back the pipe cannot reach, it stops)
CombineUnloaderMode.CP_TURN_MIN_DEG = 20 -- Courseplay course: bend toward the trailer that starts the 3/4 rear place
CombineUnloaderMode.CP_TURN_MAX_DEG = 70 -- and below this only
CombineUnloaderMode.CP_TURN_WITHIN = 15 -- m ahead of the harvester
CombineUnloaderMode.CP_UTURN_DEG = 150 -- a Courseplay U-turn is 90 + 90 deg
CombineUnloaderMode.CP_UTURN_AHEAD = 50 -- m of course read for it
CombineUnloaderMode.CURVE_MIN_TIME = 10000 -- ms the 3/4 rear place is kept at least once taken
-- Courseplay's own course ahead of the harvester (it drives straight, turn, straight, turn): heading change
-- over the next `ahead` m and the side (1 left, -1 right in the harvester frame), distance to the first bend. nil if unknown
-- Courseplay course: where the next row starts after the end-of-row U-turn (start our own U-turn at the
-- same time). First waypoint whose heading is reversed from the current row and stays so for ROW_ENTRY_STRAIGHT m.
-- Returns x, z of that waypoint and the new row direction, or nil
CombineUnloaderMode.ROW_ENTRY_REVERSED = 165 -- deg from the current row
CombineUnloaderMode.ROW_ENTRY_STRAIGHT = 10 -- m
CombineUnloaderMode.ROW_ENTRY_MAX = 150 -- m of course read
function CombineUnloaderMode.getCpRowEntry(combine)
    CombineUnloaderMode.cpRowEntryWhy = nil
    local ok, ex, ez, dx, dz = pcall(function()
        local strategy = combine.getCpDriveStrategy ~= nil and combine:getCpDriveStrategy() or nil
        if strategy == nil or strategy.getCurrentCourse == nil or strategy.ppc == nil then
            return nil
        end
        local course = strategy:getCurrentCourse()
        local ix = strategy.ppc:getCurrentWaypointIx()
        if course == nil or ix == nil then
            return nil
        end
        local n = course:getNumberOfWaypoints()
        local px, _, pz = course:getWaypointPosition(ix)
        local h0, dist, maxD = nil, 0, 0
        local cand, candDist = nil, 0
        for i = ix + 1, n do
            local x, _, z = course:getWaypointPosition(i)
            local seg = MathUtil.vector2Length(x - px, z - pz)
            if seg > 0.1 then
                local h = math.atan2(x - px, z - pz)
                if h0 == nil then
                    h0 = h
                end
                local d = math.abs(math.deg(AutoDrive.normalizeAngle(h - h0)))
                if d > 180 then
                    d = 360 - d
                end
                maxD = math.max(maxD, d)
                if d >= CombineUnloaderMode.ROW_ENTRY_REVERSED then
                    if cand == nil then
                        cand = {x = px, z = pz, dx = (x - px) / seg, dz = (z - pz) / seg}
                        candDist = 0
                    end
                    candDist = candDist + seg
                    if candDist >= CombineUnloaderMode.ROW_ENTRY_STRAIGHT then
                        return cand.x, cand.z, cand.dx, cand.dz
                    end
                else
                    cand = nil
                end
                dist = dist + seg
                px, pz = x, z
                if dist > CombineUnloaderMode.ROW_ENTRY_MAX then
                    break
                end
            end
        end
        if cand ~= nil and (dist <= CombineUnloaderMode.ROW_ENTRY_MAX) then
            -- the course (Courseplay turn course) ends already reversed: its end is the row entry
            return cand.x, cand.z, cand.dx, cand.dz
        end
        CombineUnloaderMode.cpRowEntryWhy = string.format("no row entry found (point %d/%d, %.0f m read, max deviation %.0f deg)", ix, n, dist, maxD)
        return nil
    end)
    if not ok then
        CombineUnloaderMode.cpRowEntryWhy = "error reading the course: " .. tostring(ex)
        return nil
    end
    return ex, ez, dx, dz
end

function CombineUnloaderMode.getCpTurnAhead(combine, ahead)
    local ok, angle, side, bendAt = pcall(function()
        local strategy = combine.getCpDriveStrategy ~= nil and combine:getCpDriveStrategy() or nil
        if strategy == nil or strategy.getCurrentCourse == nil or strategy.ppc == nil then
            return nil
        end
        local course = strategy:getCurrentCourse()
        local ix = strategy.ppc:getCurrentWaypointIx()
        if course == nil or ix == nil then
            return nil
        end
        local n = course:getNumberOfWaypoints()
        local x0, _, z0 = course:getWaypointPosition(ix)
        local px, pz, dist = x0, z0, 0
        local h0 = nil
        local maxTurn, firstBend = 0, nil
        local endX, endZ = x0, z0
        for i = ix + 1, n do
            local x, _, z = course:getWaypointPosition(i)
            local seg = MathUtil.vector2Length(x - px, z - pz)
            if seg > 0.1 then
                local h = math.atan2(x - px, z - pz)
                if h0 == nil then
                    h0 = h
                end
                local d = math.abs(math.deg(AutoDrive.normalizeAngle(h - h0)))
                if d > 180 then
                    d = 360 - d
                end
                if d > maxTurn then
                    maxTurn = d
                end
                if firstBend == nil and d >= 20 then
                    firstBend = dist
                end
                dist = dist + seg
                px, pz = x, z
                endX, endZ = x, z
                if dist >= ahead then
                    break
                end
            end
        end
        local cx, cy, cz = getWorldTranslation(combine.components[1].node)
        local lx = AutoDrive.worldToLocal(combine, endX, cy, endZ, combine.ad.ADRootNode)
        return maxTurn, lx >= 0 and 1 or -1, firstBend
    end)
    if not ok then
        return nil
    end
    return angle, side, bendAt
end

-- jog in the course: Courseplay bends hard and comes back to its
-- heading within JOG_AHEAD m. Not a U-turn: the unloader goes behind the harvester until the course is straight
CombineUnloaderMode.JOG_AHEAD = 50 -- m of course read
CombineUnloaderMode.JOG_MIN_DEG = 45 -- deviation from the harvester heading that makes a jog
CombineUnloaderMode.JOG_END_DEG = 35 -- end of the read course back within this of the harvester heading
CombineUnloaderMode.JOG_WITHIN = 20 -- m: first bend that close
CombineUnloaderMode.JOG_STRAIGHT_AHEAD = 15 -- m straight ahead again: back beside it
CombineUnloaderMode.JOG_STRAIGHT_DEG = 15
CombineUnloaderMode.JOG_MIN_TIME = 4000 -- ms behind at least

-- max deviation and end deviation (deg) of the course ahead from the harvester heading, distance to the first bend
function CombineUnloaderMode.getCpCourseShape(combine, ahead)
    local ok, maxDev, endDev, bendAt = pcall(function()
        local strategy = combine.getCpDriveStrategy ~= nil and combine:getCpDriveStrategy() or nil
        if strategy == nil or strategy.getCurrentCourse == nil or strategy.ppc == nil then
            return nil
        end
        local course = strategy:getCurrentCourse()
        local ix = strategy.ppc:getCurrentWaypointIx()
        if course == nil or ix == nil then
            return nil
        end
        local hx, _, hz = AutoDrive.localDirectionToWorld(combine, 0, 0, 1, combine.ad.ADRootNode)
        local hc = math.atan2(hx, hz)
        local n = course:getNumberOfWaypoints()
        local px, _, pz = course:getWaypointPosition(ix)
        local dist, maxD, lastD, firstBend = 0, 0, 0, nil
        for i = ix + 1, n do
            local x, _, z = course:getWaypointPosition(i)
            local seg = MathUtil.vector2Length(x - px, z - pz)
            if seg > 0.1 then
                local d = math.abs(math.deg(AutoDrive.normalizeAngle(math.atan2(x - px, z - pz) - hc)))
                if d > 180 then
                    d = 360 - d
                end
                maxD = math.max(maxD, d)
                lastD = d
                if firstBend == nil and d >= 20 then
                    firstBend = dist
                end
                dist = dist + seg
                px, pz = x, z
                if dist >= ahead then
                    break
                end
            end
        end
        return maxD, lastD, firstBend
    end)
    if not ok then
        return nil
    end
    return maxDev, endDev, bendAt
end

function CombineUnloaderMode.isCpJogAhead(combine)
    local maxDev, endDev, bendAt = CombineUnloaderMode.getCpCourseShape(combine, CombineUnloaderMode.JOG_AHEAD)
    return maxDev ~= nil and maxDev >= CombineUnloaderMode.JOG_MIN_DEG and endDev <= CombineUnloaderMode.JOG_END_DEG
        and bendAt ~= nil and bendAt <= CombineUnloaderMode.JOG_WITHIN, maxDev, endDev, bendAt
end

-- Courseplay turn points (capture: red point = start of the end-of-row U-turn, green
-- point = its end), read on its fieldwork course with its own functions (isTurnStartAtIx, isHeadlandTurnAtIx, the
-- turn context while it turns). A jog or a loop around a power pole has none. nil and the reason when they cannot be
-- read (the course shape is used then), else a table: ix (current waypoint), turning (Courseplay in a turn), startIx
-- and endIx (red and green points of the next turn or of the one being driven), dist (m of course before the red
-- point, 0 in the turn), deg (heading change of that turn), headland (headland corner), uTurn (end-of-row U-turn)
CombineUnloaderMode.CP_ROW_TURN_READ = 120 -- m of course read for the next red point
CombineUnloaderMode.CP_ROW_TURN_UTURN_DEG = 150 -- heading change of a turn counted as an end-of-row U-turn
function CombineUnloaderMode.getCpRowTurn(combine)
    if combine == nil or combine.ad == nil then
        return nil, "no harvester"
    end
    local now = g_time or 0
    local cache = combine.ad.cpRowTurnCache
    if cache ~= nil and cache.t == now then
        return cache.res, cache.why
    end
    local ok, res, why = pcall(CombineUnloaderMode.readCpRowTurn, combine)
    if not ok then
        why = "error reading the Courseplay points: " .. tostring(res)
        res = nil
    end
    combine.ad.cpRowTurnCache = {t = now, res = res, why = why}
    return res, why
end

-- heading (rad) of the course next to waypoint i: arriving at it (step -1) or leaving it (step 1), measured over
-- CP_ROW_TURN_BASE m of course (the row before the red point, the row after the green point)
CombineUnloaderMode.CP_ROW_TURN_BASE = 8
function CombineUnloaderMode.getCpHeadingAt(course, i, step, n)
    local x0, _, z0 = course:getWaypointPosition(i)
    local lx, lz = nil, nil
    for k = 1, 10 do
        local j = i + step * k
        if j < 1 or j > n then
            break
        end
        local x, _, z = course:getWaypointPosition(j)
        if MathUtil.vector2Length(x - x0, z - z0) > 0.5 then
            lx, lz = x, z
        end
        if MathUtil.vector2Length(x - x0, z - z0) >= CombineUnloaderMode.CP_ROW_TURN_BASE then
            break
        end
    end
    if lx == nil then
        return nil
    end
    if step < 0 then
        return math.atan2(x0 - lx, z0 - lz)
    end
    return math.atan2(lx - x0, lz - z0)
end

function CombineUnloaderMode.isCpRowEndAt(course, i)
    if course.getWaypoint == nil then
        return false
    end
    local wp = course:getWaypoint(i)
    return wp ~= nil and wp.isRowEnd ~= nil and wp:isRowEnd() == true
end

function CombineUnloaderMode.readCpRowTurn(combine)
    local strategy = combine.getCpDriveStrategy ~= nil and combine:getCpDriveStrategy() or nil
    if strategy == nil then
        return nil, "no Courseplay driver"
    end
    local course = strategy.getFieldworkCourse ~= nil and strategy:getFieldworkCourse() or nil
    course = course or strategy.fieldWorkCourse
    if course == nil or course.isTurnStartAtIx == nil or course.getNumberOfWaypoints == nil or course.getWaypointPosition == nil
        or course.getCurrentWaypointIx == nil then
        return nil, "Courseplay course without turn points"
    end
    local n = course:getNumberOfWaypoints()
    local ix = course:getCurrentWaypointIx()
    if n == nil or n < 3 or ix == nil then
        return nil, "current point of the Courseplay course unknown"
    end
    local res = {ix = ix, turning = strategy.isTurning ~= nil and strategy:isTurning() == true, uTurn = false}
    local tc = strategy.turnContext
    if res.turning and tc ~= nil and tc.turnStartWpIx ~= nil and tc.turnEndWpIx ~= nil then
        -- in its turn: the points of that turn (a row finish only has start = end, no U-turn)
        res.startIx, res.endIx, res.dist = tc.turnStartWpIx, tc.turnEndWpIx, 0
    else
        local px, _, pz = course:getWaypointPosition(ix)
        local dist = 0
        for i = math.max(1, ix), n - 1 do
            if i > ix then
                local x, _, z = course:getWaypointPosition(i)
                dist = dist + MathUtil.vector2Length(x - px, z - pz)
                px, pz = x, z
            end
            if dist > CombineUnloaderMode.CP_ROW_TURN_READ then
                break
            end
            -- red point; or a row end handed to its pathfinder (next row starting at an island): the same U-turn
            local rowEnd = not course:isTurnStartAtIx(i) and CombineUnloaderMode.isCpRowEndAt(course, i)
            if rowEnd or course:isTurnStartAtIx(i) then
                res.startIx, res.endIx, res.dist, res.byPathfinder = i, i + 1, dist, rowEnd
                break
            end
        end
    end
    if res.startIx == nil then
        res.why = string.format("no red point over %d m (point %d/%d)", CombineUnloaderMode.CP_ROW_TURN_READ, ix, n)
        return res
    end
    if res.endIx <= res.startIx and res.startIx < n and CombineUnloaderMode.isCpRowEndAt(course, res.startIx) then
        -- row finish before its pathfinder: the turn goes to the next waypoint (next row start)
        res.endIx, res.byPathfinder = res.startIx + 1, true
    end
    if res.endIx <= res.startIx or res.endIx > n then
        res.why = string.format("row end without U-turn (point %d)", res.startIx)
        return res
    end
    res.headland = course.isHeadlandTurnAtIx ~= nil and course:isHeadlandTurnAtIx(res.endIx) == true
    local hIn = CombineUnloaderMode.getCpHeadingAt(course, res.startIx, -1, n)
    local hOut = CombineUnloaderMode.getCpHeadingAt(course, res.endIx, 1, n)
    if hIn ~= nil and hOut ~= nil then
        local d = math.abs(math.deg(hOut - hIn)) % 360
        if d > 180 then
            d = 360 - d
        end
        res.deg = d
    end
    res.uTurn = not res.headland and res.deg ~= nil and res.deg >= CombineUnloaderMode.CP_ROW_TURN_UTURN_DEG
    res.sx, _, res.sz = course:getWaypointPosition(res.startIx)
    res.ex, _, res.ez = course:getWaypointPosition(res.endIx)
    res.why = string.format("red point %d %s, green point %d, %s%s%s", res.startIx, res.turning and "passed (turn in progress)" or string.format("at %.0f m", res.dist),
        res.endIx, res.deg ~= nil and string.format("%.0f deg", res.deg) or "heading unknown", res.headland and ", headland corner" or "",
        res.byPathfinder and ", row end by a planned route" or "")
    return res
end

-- behind the harvester through a jog, kept until the course is straight again
--: real crop on the straight line to the side place
CombineUnloaderMode.ISLAND_REAR_TIME = 20000 -- ms behind the harvester
function CombineUnloaderMode:isIslandRear()
    return self.islandRearUntil ~= nil and (g_time or 0) < self.islandRearUntil
end

-- (seen in game: 50 hand-overs along a stone wall): a side place
-- whose straight line or place is taken by a fixed obstacle (wall, tree, building) or a learned blocked zone is
-- condemned for SIDE_BLOCK_TIME: the chase takes the other side or the rear instead of planning the same place again
CombineUnloaderMode.SIDE_BLOCK_TIME = 90000
CombineUnloaderMode.PLACE_CHECK_INTERVAL = 1000 -- ms between two obstacle checks of a place
CombineUnloaderMode.PLACE_CLOSER_STEP = 1.5     -- m closer to the harvester tried when the place is obstructed
CombineUnloaderMode.PLACE_MIN_GAP = 0.4         -- m between the harvester side and the train side at least
function CombineUnloaderMode:blockSide(side, why)
    self.blockedSides = self.blockedSides or {}
    self.blockedSides[side] = {till = (g_time or 0) + CombineUnloaderMode.SIDE_BLOCK_TIME, why = why}
end

function CombineUnloaderMode:isSideBlocked(side)
    local b = self.blockedSides ~= nil and self.blockedSides[side] or nil
    return b ~= nil and (g_time or 0) < b.till
end

-- fixed obstacle in the box the train takes at a place (aligned with the harvester, the trailer behind the tractor)
function CombineUnloaderMode:isPlaceObstructed(pos)
    if pos == nil or self.combine == nil or self.combine.components == nil or FollowCombineTask == nil then
        return false
    end
    local rx, _, rz = AutoDrive.localDirectionToWorld(self.combine, 0, 0, 1, self.combine.ad.ADRootNode)
    local length = math.max(6, AutoDrive.getTractorTrainLength(self.vehicle, true, false))
    local halfWidth = self.vehicle.ad.pathFinderModule:getTrainWidth() / 2 + 0.3
    local height = PathFinderModule.getTrainHeight(self.vehicle) + PathFinderModule.OBSTACLE_HEIGHT_MARGIN
    local angleRad = AutoDrive.normalizeAngle(math.atan2(-rz, rx))
    -- the place is where the tractor stands, the train reaches 3 m ahead of it and the rest behind
    local cx, cz = pos.x - rx * (length / 2 - 3), pos.z - rz * (length / 2 - 3)
    local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, cx, 1, cz)
    local probe = {hits = 0, callback = FollowCombineTask.lineCollisionCallback}
    overlapBox(cx, y + height / 2, cz, 0, angleRad, 0, length / 2, height / 2, halfWidth, "callback", probe, AutoDrive.collisionMaskTerrain, true, true, true, true)
    if probe.hits > 0 then
        return true
    end
    return self.vehicle.ad.pathFinderModule:boxHitsBlockedZone(cx, cz, angleRad, length / 2, halfWidth)
end

-- a side place of a forage harvester: obstructed, it is brought closer to the harvester when that frees it (the spout
-- reaches), else the side counts as blocked. Checked every PLACE_CHECK_INTERVAL. Returns the place, blocked
function CombineUnloaderMode:checkPlaceObstacle(side, pos, blocked, offsetX, offsetZ)
    if blocked or pos == nil or self.combine == nil or self.combine.ad == nil or not self.combine.ad.isChopper then
        return pos, blocked
    end
    local now = g_time or 0
    self.placeCheck = self.placeCheck or {}
    local c = self.placeCheck[side]
    if c == nil or now - c.t >= CombineUnloaderMode.PLACE_CHECK_INTERVAL or c.offsetZ ~= offsetZ then
        c = {t = now, blocked = false, offsetX = nil, offsetZ = offsetZ}
        local name = tostring(self.vehicle:getName())
        local sideName = side == AutoDrive.CHASEPOS_LEFT and "left" or "right"
        if self:isPlaceObstructed(pos) then
            local sign = offsetX >= 0 and 1 or -1
            local minOffset = (self.combine.size ~= nil and self.combine.size.width or 3) / 2 + self.vehicle.ad.pathFinderModule:getTrainWidth() / 2 + CombineUnloaderMode.PLACE_MIN_GAP
            local closer = math.abs(offsetX) - CombineUnloaderMode.PLACE_CLOSER_STEP
            while closer >= minOffset do
                local p = AutoDrive.createWayPointRelativeToVehicle(self.combine, sign * closer, offsetZ)
                if not self:isPlaceObstructed(p) then
                    c.offsetX = sign * closer
                    break
                end
                closer = closer - CombineUnloaderMode.PLACE_CLOSER_STEP
            end
            if c.offsetX == nil then
                c.blocked = true
            end
        end
        self.placeCheck[side] = c
    end
    if c.blocked then
        return pos, true
    end
    if c.offsetX ~= nil then
        return AutoDrive.createWayPointRelativeToVehicle(self.combine, c.offsetX, offsetZ), false
    end
    return pos, false
end

-- rear place held on purpose (jog or island): standing while the harvester moves past is no stuck chase
function CombineUnloaderMode:isRearChaseHeld()
    return self.jogRearStart ~= nil or self:isIslandRear()
end

function CombineUnloaderMode:updateJogRear()
    local now = g_time or 0
    local name = tostring(self.vehicle:getName())
    -- the jog place stayed on into the end-of-row U-turn, the trailer followed the harvester into
    -- it. A Courseplay U-turn (red point) close ahead or being driven ends it, the end-of-row wait takes over
    local row = CombineUnloaderMode.getCpRowTurn(self.combine)
    local near = FollowCombineTask ~= nil and FollowCombineTask.END_ROW_BEND_WITHIN or 15
    if row ~= nil and row.uTurn and (row.turning or (row.dist ~= nil and row.dist <= near)) then
        if self.jogRearStart ~= nil then
            self.jogRearStart = nil
        end
        return false
    end
    local jog, maxDev, endDev, bendAt = CombineUnloaderMode.isCpJogAhead(self.combine)
    if jog then
        if self.jogRearStart == nil then
            self.jogRearStart = now
        end
        self.jogRearLast = now
        return true
    end
    if self.jogRearStart == nil then
        return false
    end
    local straight = CombineUnloaderMode.getCpCourseShape(self.combine, CombineUnloaderMode.JOG_STRAIGHT_AHEAD)
    if now - self.jogRearStart < CombineUnloaderMode.JOG_MIN_TIME or straight == nil or straight > CombineUnloaderMode.JOG_STRAIGHT_DEG then
        return true
    end
    self.jogRearStart = nil
    return false
end

function CombineUnloaderMode:getChopperCurveOffset()
    local trail = self.combine.ad ~= nil and self.combine.ad.chopperTrail or nil
    if trail == nil or #trail < 2 then
        return 0
    end
    local cx, cy, cz = getWorldTranslation(self.combine.components[1].node)
    for i = #trail, 1, -1 do
        local p = trail[i]
        if MathUtil.vector2Length(p.x - cx, p.z - cz) >= CombineUnloaderMode.CURVE_BACK then
            local lx, _, lz = AutoDrive.worldToLocal(self.combine, p.x, cy, p.z, self.combine.ad.ADRootNode)
            if lz < 0 then
                return lx
            end
            return 0
        end
    end
    return 0
end

function CombineUnloaderMode:getChopperCurveSide(side)
    local offset = self:getChopperCurveOffset()
    local toward = offset * side
    local was = self.curveRearSide == side
    -- a turn announced by Courseplay: 3/4 rear at once, whichever way it turns
    -- no longer from the Courseplay turn flag (raised often on straight lines, the 3/4 stop slowed the trailer)
    local announced = false
    -- Courseplay drives: its course tells the next bend and its side before the harvester turns
    local cpAngle, cpSide, cpBend = CombineUnloaderMode.getCpTurnAhead(self.combine, 30)
    -- the 3/4 only helps in bends under CP_TURN_MAX_DEG
    local tooSharp = cpAngle ~= nil and cpAngle >= CombineUnloaderMode.CP_TURN_MAX_DEG
    local cpTurn = cpAngle ~= nil and not tooSharp and cpAngle >= CombineUnloaderMode.CP_TURN_MIN_DEG and cpSide == side and cpBend ~= nil and cpBend <= CombineUnloaderMode.CP_TURN_WITHIN
    -- end-of-row U-turn: no 3/4 there, the normal turn handling drives (the 3/4 stop left the trailer 16 m behind)
    local uAngle = CombineUnloaderMode.getCpTurnAhead(self.combine, CombineUnloaderMode.CP_UTURN_AHEAD)
    local uTurn = uAngle ~= nil and uAngle >= CombineUnloaderMode.CP_UTURN_DEG
    local now = (announced and not uTurn) or cpTurn and not uTurn or toward > (was and CombineUnloaderMode.CURVE_OUT or CombineUnloaderMode.CURVE_IN)
    if uTurn or tooSharp or self.vehicle.ad.endRowTurnActive then
        now = false
    end
    -- kept CURVE_MIN_TIME at least (it switched on and off every 2 s, never reached)
    if was and not now and not uTurn and not tooSharp and not self.vehicle.ad.endRowTurnActive and (g_time or 0) - (self.curveRearStart or 0) < CombineUnloaderMode.CURVE_MIN_TIME then
        now = true
    end
    if now and not was then
        self.curveRearStart = g_time or 0
    end
    if now ~= was then
        self.curveRearSide = now and side or nil
    end
    return now
end

-- (an unloader stopped in front of leftover maize): the
-- harvester's side fruit sensors trip on any plant (more than 10 pixels) and send the place behind the harvester,
-- the trailer beside it then stops. For a forage harvester the crop counts only when it is real crop: not isolated
-- plants or a narrow strip (same rule as the chase line), not on the harvester's own trail
CombineUnloaderMode.SENSOR_CROP_STEP = 1 -- m between the samples read in the sensor box
CombineUnloaderMode.SENSOR_CROP_CACHE = 500 -- ms
function CombineUnloaderMode:isSideFruitBlocking(sensor)
    if sensor == nil or not (AutoDrive.getSetting("avoidFruit", self.vehicle) and sensor:pollInfo()) then
        return false
    end
    if not self.combine.ad.isChopper or sensor.triggerType == ADSensor.TYPE_SWATH then
        return true
    end
    local now = g_time or 0
    self.sensorCropCache = self.sensorCropCache or {}
    local cached = self.sensorCropCache[sensor]
    if cached ~= nil and now >= cached.t and now - cached.t < CombineUnloaderMode.SENSOR_CROP_CACHE then
        return cached.v
    end
    local corners = sensor:getCorners()
    local c1, c2, c3 = corners[1], corners[2], corners[3]
    local ux, uz = c2.x - c1.x, c2.z - c1.z
    local vx, vz = c3.x - c1.x, c3.z - c1.z
    local nu = math.max(1, math.ceil(math.sqrt(ux * ux + uz * uz) / CombineUnloaderMode.SENSOR_CROP_STEP))
    local nv = math.max(1, math.ceil(math.sqrt(vx * vx + vz * vz) / CombineUnloaderMode.SENSOR_CROP_STEP))
    local real, seen = false, false
    for i = 0, nu do
        for j = 0, nv do
            local x, z = c1.x + ux * i / nu + vx * j / nv, c1.z + uz * i / nu + vz * j / nv
            if PathFinderModule.getMapCropRawAt(x, z) ~= 0 then
                seen = true
                if not PathFinderModule.isIsolatedCrop(x, z) and not self:isOnCombineTrail(x, z) then
                    real = true
                    break
                end
            end
        end
        if real then
            break
        end
    end
    self.sensorCropCache[sensor] = {t = now, v = real}
    return real
end

function CombineUnloaderMode:getPipeChasePosition(planningPhase)
    local chaseNode
    local sideIndex = AutoDrive.CHASEPOS_REAR

    -- in case of rotated attached combine use the trailing vehicle sensors
    local combineSensors = self.combine.ad.sensors or self.combineRootVehicle.ad.sensors

    local swathDetection =  AutoDrive.getSetting("detectSwath", self.vehicle) or false

    combineSensors.leftSensorFruit:setSwathDetection(swathDetection)
    local leftBlocked = self:isSideFruitBlocking(combineSensors.leftSensorFruit)
    or combineSensors.leftSensor:pollInfo()
    or (AutoDrive.getSetting("followOnlyOnField", self.vehicle) and (not combineSensors.leftSensorField:pollInfo()))

    combineSensors.leftFrontSensorFruit:setSwathDetection(swathDetection)
    local leftFrontBlocked = self:isSideFruitBlocking(combineSensors.leftFrontSensorFruit)
    or combineSensors.leftFrontSensor:pollInfo()

    combineSensors.rightSensorFruit:setSwathDetection(swathDetection)
    local rightBlocked = self:isSideFruitBlocking(combineSensors.rightSensorFruit)
    or combineSensors.rightSensor:pollInfo()
    or (AutoDrive.getSetting("followOnlyOnField", self.vehicle) and (not combineSensors.rightSensorField:pollInfo()))

    combineSensors.rightFrontSensorFruit:setSwathDetection(swathDetection)
    local rightFrontBlocked = self:isSideFruitBlocking(combineSensors.rightFrontSensorFruit)
    or combineSensors.rightFrontSensor:pollInfo()

    rightBlocked = rightBlocked or rightFrontBlocked
    leftBlocked = leftBlocked or leftFrontBlocked

-- TODO: does this make sense, i.e. leftBlocked = leftBlocked or leftFrontBlocked <-> elseif leftFrontBlocked and (not rightFrontBlocked) then
    -- prefer side where front is also free
    if (not leftBlocked) and (not rightBlocked) then
        if (not leftFrontBlocked) and rightFrontBlocked then
            rightBlocked = true
        elseif leftFrontBlocked and (not rightFrontBlocked) then
            leftBlocked = true
        end
    end

    self.pipeSide = AutoDrive.getPipeSide(self.combine)
    self.targetFillUnit, self.targetFillNode, self.targetAutoAimTargetNode = AutoDrive.getNextFreeDischargeableUnit(self.vehicle)

    local sideChaseTermX = self:getSideChaseOffsetX()
    local sideChaseTermZ = self:getSideChaseOffsetZ(AutoDrive.dynamicChaseDistance or self.combine.ad.isHarvester)
    local rearChaseTermZ = self:getRearChaseOffsetZ(planningPhase)

    if self.combine.ad.isChopper then
        -- any chopper
        --CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getPipeChasePosition=IsBufferCombine")
        local leftZ, rightZ = sideChaseTermZ - 2, sideChaseTermZ - 2
        local curveLeft = false
        if not planningPhase and AutoDrive.isChopperUnloading(self.vehicle) then
            local curveZ = math.min(sideChaseTermZ - 2, -((self.combine.size ~= nil and self.combine.size.length or 10) / 2 + CombineUnloaderMode.CURVE_REAR_GAP))
            local unloaderSide = self:getUnloaderOnSide()
            if unloaderSide == AutoDrive.CHASEPOS_LEFT and self:getChopperCurveSide(AutoDrive.CHASEPOS_LEFT) then
                leftZ = curveZ
                curveLeft = true
            elseif unloaderSide == AutoDrive.CHASEPOS_RIGHT and self:getChopperCurveSide(AutoDrive.CHASEPOS_RIGHT) then
                rightZ = curveZ
            end
        end
        local leftChasePos = AutoDrive.createWayPointRelativeToVehicle(self.combine, sideChaseTermX + self:getPipeSlopeCorrection(), leftZ)
        local rightChasePos = AutoDrive.createWayPointRelativeToVehicle(self.combine, -(sideChaseTermX + self:getPipeSlopeCorrection()), rightZ)
        local rearChasePos = AutoDrive.createWayPointRelativeToVehicle(self.combine, 0, rearChaseTermZ)
        -- a side condemned by an obstacle on the chase line (blockSide),
        -- and a wall, tree or building at a side place: the place is brought closer to the harvester when that frees it,
        -- else that side counts as blocked
        if self:isSideBlocked(AutoDrive.CHASEPOS_LEFT) then
            leftBlocked = true
        end
        if self:isSideBlocked(AutoDrive.CHASEPOS_RIGHT) then
            rightBlocked = true
        end
        leftChasePos, leftBlocked = self:checkPlaceObstacle(AutoDrive.CHASEPOS_LEFT, leftChasePos, leftBlocked, sideChaseTermX + self:getPipeSlopeCorrection(), leftZ)
        rightChasePos, rightBlocked = self:checkPlaceObstacle(AutoDrive.CHASEPOS_RIGHT, rightChasePos, rightBlocked, -(sideChaseTermX + self:getPipeSlopeCorrection()), rightZ)
        local angleToLeftChaseSide = self:getAngleToChasePos(leftChasePos)
        local angleToRearChaseSide = self:getAngleToChasePos(rearChasePos)
        local chaseSideSetting = AutoDrive.getSetting("chaseSide", self.combine)

        -- Default to the side of the harvester the unloader is already on
        -- then check if there is a better side
        chaseNode = leftChasePos
        sideIndex = AutoDrive.CHASEPOS_LEFT
        local unloaderPos, _ = self:getUnloaderOnSide()
        if unloaderPos == AutoDrive.CHASEPOS_RIGHT then
            chaseNode = rightChasePos
            sideIndex = AutoDrive.CHASEPOS_RIGHT
        end
        -- following order is important!
        if self.combine.ad.isFixedPipeChopper then
            -- chopper with fixed unloading
            local sideOffsetZ = self:getDynamicSideChaseOffsetZ_fromDischargeNode(planningPhase)
            chaseNode = self:getPipeChaseWayPoint(0, sideOffsetZ)
            sideIndex = self.pipeSide
        elseif AutoDrive:getIsCPActive(self.combine) and AutoDrive.combineIsTurning(self.combine)
            and not (not planningPhase and self.vehicle.ad.cpTurnFlagIgnored) then
            -- (the chase ignores the Courseplay turn flag on a straight line, heading steady: the place
            -- stays beside, behind the harvester it stopped the trailer 40 m before the jog)
            chaseNode = rearChasePos
            sideIndex = AutoDrive.CHASEPOS_REAR
        elseif not planningPhase and AutoDrive.isChopperUnloading(self.vehicle) and AutoDrive:getIsCPActive(self.combine)
            and not self.vehicle.ad.endRowTurnActive and (self:updateJogRear() or self:isIslandRear()) then
            -- jog in the course, behind it makes the turn easier; crop on the line to the side place:
            -- behind it, before the island
            chaseNode = rearChasePos
            sideIndex = AutoDrive.CHASEPOS_REAR
        elseif chaseSideSetting ~= AutoDrive.CHASEPOS_AUTO then
            -- consider chaseSide setting
            sideIndex = chaseSideSetting
            if chaseSideSetting == AutoDrive.CHASEPOS_LEFT then
                chaseNode = leftChasePos
            elseif chaseSideSetting == AutoDrive.CHASEPOS_RIGHT then
                chaseNode = rightChasePos
            else
                chaseNode = rearChasePos
            end
        elseif (not leftBlocked) and ((self:isUnloaderOnCorrectSide(AutoDrive.CHASEPOS_LEFT) and (curveLeft or angleToLeftChaseSide < angleToRearChaseSide)) or planningPhase) then
            chaseNode = leftChasePos
            sideIndex = AutoDrive.CHASEPOS_LEFT
        elseif (not rightBlocked) and (self:isUnloaderOnCorrectSide(AutoDrive.CHASEPOS_RIGHT) or planningPhase) then
            chaseNode = rightChasePos
            sideIndex = AutoDrive.CHASEPOS_RIGHT
        elseif not self.combine.ad.isSugarcaneHarvester then
            chaseNode = rearChasePos
            sideIndex = AutoDrive.CHASEPOS_REAR
        end
    else
        -- harveser with bunker
        --CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getPipeChasePosition:IsNormalCombine")
        local rearChaseTermX = self:getRearChaseOffsetX(leftBlocked, rightBlocked)

        local sideChasePos = AutoDrive.createWayPointRelativeToVehicle(self.combine, self.pipeSide * (sideChaseTermX + self:getPipeSlopeCorrection()), sideChaseTermZ)
        local rearChasePos = AutoDrive.createWayPointRelativeToVehicle(self.combine, rearChaseTermX, rearChaseTermZ)
        local angleToSideChaseSide = self:getAngleToChasePos(sideChasePos)
        local angleToRearChaseSide = self:getAngleToChasePos(rearChasePos)

        AutoDrive.useNewPipeOffsets = true

        if AutoDrive.useNewPipeOffsets and AutoDrive.isPipeOut(self.combine) then
            local sideOffsetZ = self:getDynamicSideChaseOffsetZ_fromDischargeNode(planningPhase)
            -- local sideOffsetX = self.pipeSide * self:getSideChaseOffsetX_new()
            -- sideChasePos = AutoDrive.createWayPointRelativeToDischargeNode(self.combine, sideOffsetX, sideOffsetZ)
            sideChasePos = self:getPipeChaseWayPoint(0, sideOffsetZ)
            angleToSideChaseSide = self:getAngleToChasePos(sideChasePos)
        end

        if
            (
                (
                    (self.pipeSide == AutoDrive.CHASEPOS_LEFT and not leftBlocked) 
                    or (self.pipeSide == AutoDrive.CHASEPOS_RIGHT and not rightBlocked)
                ) 
                and
                (
                    (
                        self:isUnloaderOnCorrectSide(self.pipeSide) 
                        -- and math.abs(angleToSideChaseSide) < math.abs(angleToRearChaseSide)
                    )
                    or (planningPhase == true)
                )
            )
            or
            (
                (planningPhase == true) and (self.combine.ad.noMovementTimer.elapsedTime > 1000)
            )
         then
            -- Take into account a right sided harvester, e.g. potato harvester.
            chaseNode = sideChasePos
            sideIndex = self.pipeSide
         else
            sideIndex = AutoDrive.CHASEPOS_REAR
            chaseNode = rearChasePos
         end
    end

    self.chasePosIndex = sideIndex
    return chaseNode, self.chasePosIndex
end

function CombineUnloaderMode:getAngleToCombineHeading()
    if self.vehicle == nil or self.combine == nil then
        return math.huge
    end

    local combineRx, _, combineRz = AutoDrive.localDirectionToWorld(self.combine, 0, 0, 1, self.combine.ad.ADRootNode)
    local rx, _, rz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)

    return math.abs(AutoDrive.angleBetween({x = rx, z = rz}, {x = combineRx, z = combineRz}))
end

function CombineUnloaderMode:getAngleToCombine()
    if self.vehicle == nil or self.combine == nil then
        return math.huge
    end

    local vehicleX, _, vehicleZ = getWorldTranslation(self.vehicle.components[1].node)
    local combineX, _, combineZ = getWorldTranslation(self.combine.components[1].node)
    local rx, _, rz =  AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1, self.combine.ad.ADRootNode)

    return math.abs(AutoDrive.angleBetween({x = rx, z = rz}, {x = combineX - vehicleX, z = combineZ - vehicleZ}))
end

function CombineUnloaderMode:getAngleToChasePos(chasePos)
    local worldX, _, worldZ = getWorldTranslation(self.vehicle.components[1].node)
    local rx, _, rz =  AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local angle = AutoDrive.angleBetween({x = rx, z = rz}, {x = chasePos.x - worldX, z = chasePos.z - worldZ})

    return angle
end

function CombineUnloaderMode:getFollowingUnloader()
    return self.followingUnloader
end

function CombineUnloaderMode:registerFollowingUnloader(followingUnloader)
    self.followingUnloader = followingUnloader
end

function CombineUnloaderMode:unregisterFollowingUnloader()
    self.followingUnloader = nil
end

function CombineUnloaderMode:setStateNames()
    if self.statesToNames == nil then
        self.statesToNames = {}
        for name, id in pairs(CombineUnloaderMode) do
            if string.sub(name, 1, 6) == "STATE_" then
                self.statesToNames[id] = name
            end
        end
    end
end

function CombineUnloaderMode:getStateName(state)
    local requestedState = state
    if requestedState == nil then
        requestedState = self.state
    end
    if requestedState == nil then
        Logging.error("[AD] CombineUnloaderMode: Could not find name for state ->%s<- !", tostring(requestedState))
    end
    return self.statesToNames[requestedState] or ""
end

function CombineUnloaderMode.debugMsg(vehicle, debugText, ...)
    if CombineUnloaderMode.debug == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    else
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_COMBINEINFO, debugText, ...)
    end
end
