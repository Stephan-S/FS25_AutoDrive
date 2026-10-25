-- the 2nd unloader follows the 1st on its crumbs (DriveToVehicleTask, FollowVehicleTask). When
-- the forage harvester or the 1st comes toward it (ahead of it on its lane, head-on or from behind), it pulls aside to
-- the free side (forward diagonal), stops, lets them pass, then plans a new route behind the 1st. No room, or they are
-- already too close: it stops where it is and waits.
StepAsideTask = ADInheritsFrom(AbstractTask)

StepAsideTask.DETECT_DISTANCE = 100     -- m: a mover further away is no threat yet
StepAsideTask.MIN_SPEED = 1.5           -- km/h: a standing mover is no threat
StepAsideTask.LANE_MARGIN = 2           -- m beside the mover's track still counted as its lane
StepAsideTask.HEADER_HALF_MIN = 4.5     -- m: half width counted for a forage harvester at least (its header)
StepAsideTask.CLEAR_MARGIN = 2          -- m between the mover's side and the train's side after the step
StepAsideTask.MIN_MOVE = 1.5            -- m: a smaller step is not worth it, they pass beside (the follower only stops)
StepAsideTask.FORWARD_MIN = 12            -- m driven ahead at least (the train must turn, then straighten)
StepAsideTask.FORWARD_MIN_FACING = 7      -- m ahead at least when the mover comes head-on (less time, the train stays bent)
StepAsideTask.FORWARD_STEP = 5            -- m: the forward runs tried (FORWARD_MIN, + STEP, + 2 STEP), the smallest step aside wins
StepAsideTask.SIDE_MAX = 14             -- m aside at most
StepAsideTask.NO_CROSS_DISTANCE = 60    -- m: the track of a mover closer than this is never crossed
StepAsideTask.NEAR_FACING = 35          -- m: a mover coming head-on closer than this leaves no time to move, stop
StepAsideTask.SPEED = 10                -- km/h
StepAsideTask.ARRIVE_DISTANCE = 2.5
StepAsideTask.MOVE_TIMEOUT = 30000
StepAsideTask.STILL_TIME = 4000         -- ms without moving while stepping aside: stay there
StepAsideTask.WAIT_TIMEOUT = 90000
StepAsideTask.PASSED_STAND_TIME = 20000 -- ms the movers stand without coming closer: they wait for something else, go on
StepAsideTask.PASSED_QUIET_TIME = 3000  -- ms nobody comes any more before the follower goes on
StepAsideTask.COOLDOWN = 8000           -- ms after a step before the next check
StepAsideTask.CHECK_INTERVAL = 500
StepAsideTask.MAX_MOVES = 2             -- steps aside in one episode

StepAsideTask.STATE_MOVING = 1
StepAsideTask.STATE_WAITING = 2

local function logName(vehicle)
    return vehicle ~= nil and vehicle.getName ~= nil and tostring(vehicle:getName()) or "?"
end

-- 2D cross product: sign tells the side of (bx, bz) the point (ax, az) lies on
local function cross(ax, az, bx, bz)
    return ax * bz - az * bx
end

function StepAsideTask:new(vehicle, targetUnloader, info)
    local o = StepAsideTask:create()
    o.vehicle = vehicle
    o.targetUnloader = targetUnloader
    o.info = info
    o.state = StepAsideTask.STATE_WAITING
    o.moves = 0
    o.time = 0
    o.waitTime = 0
    o.standTime = 0
    o.quietTime = 0
    o.checkTime = 0
    o.taskType = "StepAsideTask"
    return o
end

function StepAsideTask:setUp()
    self.trailers, _ = AutoDrive.getAllUnits(self.vehicle)
    AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, false)
    self:startWaiting()
    if self.info ~= nil then
        self:planAside(self.info)
    end
end

-- ---------------------------------------------------------------- detection (static, used by the follow tasks)

-- the vehicles that may come toward the follower: the harvester of the 1st (forage harvester) and the 1st itself
function StepAsideTask.getMovers(targetUnloader)
    local movers = {}
    if targetUnloader == nil or targetUnloader.ad == nil or targetUnloader.components == nil then
        return movers
    end
    local mode = targetUnloader.ad.modes ~= nil and targetUnloader.ad.modes[AutoDrive.MODE_UNLOAD] or nil
    local harvester = mode ~= nil and mode.combine or nil
    if harvester ~= nil and harvester.ad ~= nil and harvester.ad.isChopper and harvester.components ~= nil then
        movers[#movers + 1] = harvester
    end
    movers[#movers + 1] = targetUnloader
    return movers
end

-- real motion of a mover (positions over time, as the chase does for the harvester): known only after a second sample
function StepAsideTask.getMotion(vehicle, mover)
    local now = g_time or 0
    vehicle.ad.asideMotion = vehicle.ad.asideMotion or {}
    local m = vehicle.ad.asideMotion[mover]
    local x, _, z = getWorldTranslation(mover.components[1].node)
    if m == nil or now - m.t > 3000 then
        m = {x = x, z = z, t = now, vx = 0, vz = 0, known = false}
        vehicle.ad.asideMotion[mover] = m
        return m
    end
    if now - m.t >= StepAsideTask.CHECK_INTERVAL then
        local dt = (now - m.t) / 1000
        m.vx, m.vz = (x - m.x) / dt, (z - m.z) / dt
        m.x, m.z, m.t = x, z, now
        m.known = true
    end
    return m
end

function StepAsideTask.getHalfWidth(vehicle)
    local w = vehicle.size ~= nil and vehicle.size.width or 3
    if vehicle.ad ~= nil and vehicle.ad.pathFinderModule ~= nil and vehicle.ad.pathFinderModule.getTrainWidth ~= nil then
        w = math.max(w, vehicle.ad.pathFinderModule:getTrainWidth())
    end
    return w / 2
end

function StepAsideTask.getMoverHalfWidth(mover)
    local half = (mover.size ~= nil and mover.size.width or 3) / 2
    if mover.ad ~= nil and mover.ad.isChopper then
        local tool = 0
        if AutoDrive.getFrontToolWidth ~= nil then
            tool = AutoDrive.getFrontToolWidth(mover) or 0
        end
        half = math.max(half, StepAsideTask.HEADER_HALF_MIN, tool / 2)
    end
    return half
end

-- the mover's motion as a lane: {mover, mx, mz, rx, rz, speed, needed} or nil when it does not move
function StepAsideTask.getLane(vehicle, mover)
    if mover == nil or mover == vehicle or mover.components == nil then
        return nil
    end
    local m = StepAsideTask.getMotion(vehicle, mover)
    if not m.known then
        return nil
    end
    local speed = math.sqrt(m.vx * m.vx + m.vz * m.vz) * 3.6
    if speed < StepAsideTask.MIN_SPEED then
        return nil
    end
    local mx, _, mz = getWorldTranslation(mover.components[1].node)
    local lane = {mover = mover, mx = mx, mz = mz, rx = m.vx / (speed / 3.6), rz = m.vz / (speed / 3.6), speed = speed}
    lane.needed = StepAsideTask.getMoverHalfWidth(mover) + StepAsideTask.getHalfWidth(vehicle) + StepAsideTask.CLEAR_MARGIN
    return lane
end

-- is this mover coming toward the vehicle? Returns info {mover, distance, ahead, lateral, laneHalf, speed, headOn, lane}
function StepAsideTask.checkMover(vehicle, mover)
    local lane = StepAsideTask.getLane(vehicle, mover)
    if lane == nil then
        return nil
    end
    local vx, _, vz = getWorldTranslation(vehicle.components[1].node)
    local dx, dz = vx - lane.mx, vz - lane.mz
    local distance = math.sqrt(dx * dx + dz * dz)
    if distance > StepAsideTask.DETECT_DISTANCE then
        return nil
    end
    local ahead = dx * lane.rx + dz * lane.rz
    if ahead <= 0 then
        return nil
    end
    local lateral = math.abs(cross(dx, dz, lane.rx, lane.rz))
    local laneHalf = StepAsideTask.getMoverHalfWidth(mover) + StepAsideTask.getHalfWidth(vehicle) + StepAsideTask.LANE_MARGIN
    if lateral > laneHalf then
        return nil
    end
    local fx, _, fz = AutoDrive.localDirectionToWorld(vehicle, 0, 0, 1)
    return {mover = mover, distance = distance, ahead = ahead, lateral = lateral, laneHalf = laneHalf, speed = lane.speed,
        headOn = fx * lane.rx + fz * lane.rz < 0, lane = lane}
end

-- the closest mover coming toward the vehicle, nil when none (evaluated every CHECK_INTERVAL, nil in between)
function StepAsideTask.check(vehicle, targetUnloader)
    local now = g_time or 0
    if vehicle.ad.asideCooldownUntil ~= nil and now < vehicle.ad.asideCooldownUntil then
        return nil
    end
    if vehicle.ad.asideCheckAt ~= nil and now - vehicle.ad.asideCheckAt < StepAsideTask.CHECK_INTERVAL then
        return nil
    end
    vehicle.ad.asideCheckAt = now
    local best = nil
    for _, mover in ipairs(StepAsideTask.getMovers(targetUnloader)) do
        local info = StepAsideTask.checkMover(vehicle, mover)
        if info ~= nil and (best == nil or info.distance < best.distance) then
            best = info
        end
    end
    return best
end

-- called each frame by the follow tasks (DriveToVehicleTask, FollowVehicleTask): when a mover comes, the task is
-- replaced by a StepAsideTask and true is returned (the caller returns at once)
function StepAsideTask.hook(vehicle, targetUnloader, dt)
    if vehicle == nil or vehicle.ad == nil or targetUnloader == nil or targetUnloader.ad == nil or vehicle.ad.taskModule == nil then
        return false
    end
    local info = StepAsideTask.check(vehicle, targetUnloader)
    if info == nil then
        return false
    end
    if vehicle.ad.pathFinderModule ~= nil then
        vehicle.ad.pathFinderModule:reset()
    end
    vehicle.ad.taskModule:addTask(StepAsideTask:new(vehicle, targetUnloader, info))
    vehicle.ad.taskModule:setCurrentTaskFinished(ADTaskModule.DONT_PROPAGATE)
    return true
end

-- ---------------------------------------------------------------- the step aside

-- smallest step S (m) on the side (sx, sz) that takes the tractor out of every lane, the point being
-- P = V + f * F + side * S (F m ahead along the heading, S m aside). The lateral offset to a lane is linear in S:
-- a + b * S, forbidden while |a + b S| < needed. nil: no step on this side (runs along a lane, too far, crosses a close lane)
function StepAsideTask.getSideStep(lanes, vx, vz, fx, fz, sx, sz, F)
    local f0 = F or StepAsideTask.FORWARD_MIN
    local bands = {}
    for _, m in ipairs(lanes) do
        local a = cross(vx - m.mx + fx * f0, vz - m.mz + fz * f0, m.rx, m.rz)
        local b = cross(sx, sz, m.rx, m.rz)
        if math.abs(b) < 0.05 then
            if math.abs(a) < m.needed then
                return nil, "the step runs along its path"
            end
        else
            local s1, s2 = (-m.needed - a) / b, (m.needed - a) / b
            bands[#bands + 1] = {lo = math.min(s1, s2), hi = math.max(s1, s2), a = a, b = b, lane = m,
                lat0 = cross(vx - m.mx, vz - m.mz, m.rx, m.rz)}
        end
    end
    local S = 0
    for _ = 1, #bands + 1 do
        local moved = false
        for _, band in ipairs(bands) do
            if S > band.lo and S < band.hi then
                S = band.hi
                moved = true
            end
        end
        if not moved then
            break
        end
    end
    if S > StepAsideTask.SIDE_MAX then
        return nil, string.format("%.0f m sideways needed", S)
    end
    -- never across the track of a mover close by (the step would pass in front of it): the tractor and the point
    -- must lie on the same side of its track
    for _, band in ipairs(bands) do
        local before, after = band.lat0, band.a + band.b * S
        if before * after < 0 then
            local d = MathUtil.vector2Length(vx - band.lane.mx, vz - band.lane.mz)
            if d < StepAsideTask.NO_CROSS_DISTANCE then
                return nil, string.format("would cross the path of %s at %.0f m", logName(band.lane.mover), d)
            end
        end
    end
    return S
end

-- a place out of the lanes of all the movers, forward and aside: the side needing the smaller step first; the line to
-- it must be free (no crop, obstacle, field border). Returns true when the tractor drives there
function StepAsideTask:planAside(info)
    local vehicle = self.vehicle
    if self.moves >= StepAsideTask.MAX_MOVES then
        return false
    end
    if info.headOn and info.distance < StepAsideTask.NEAR_FACING then
        self.held = true
        return false
    end
    local vx, _, vz = getWorldTranslation(vehicle.components[1].node)
    local fx, _, fz = AutoDrive.localDirectionToWorld(vehicle, 0, 0, 1)
    local lx, _, lz = AutoDrive.localDirectionToWorld(vehicle, 1, 0, 0)
    local lanes = {}
    for _, mover in ipairs(StepAsideTask.getMovers(self.targetUnloader)) do
        local lane = StepAsideTask.getLane(vehicle, mover)
        if lane ~= nil then
            lanes[#lanes + 1] = lane
        end
    end
    if #lanes == 0 and info.lane ~= nil then
        lanes[1] = info.lane
    end
    local candidates = {}
    local refused = {}
    -- three forward runs per side (a mover head-on leaves less time: shorter runs), the smallest step aside wins
    local forwardMin = info.headOn and StepAsideTask.FORWARD_MIN_FACING or StepAsideTask.FORWARD_MIN
    for _, side in ipairs({1, -1}) do
        local best, bestWhy = nil, nil
        for i = 0, 2 do
            local F = forwardMin + i * StepAsideTask.FORWARD_STEP
            local S, why = StepAsideTask.getSideStep(lanes, vx, vz, fx, fz, side * lx, side * lz, F)
            if S ~= nil and (best == nil or S < best.S - 0.01) then
                best = {side = side, S = S, F = F, x = vx + fx * F + side * lx * S, z = vz + fz * F + side * lz * S}
            elseif S == nil and bestWhy == nil then
                bestWhy = why
            end
        end
        if best ~= nil then
            candidates[#candidates + 1] = best
        else
            refused[#refused + 1] = string.format("%s : %s", side > 0 and "left" or "right", tostring(bestWhy))
        end
    end
    table.sort(candidates, function(a, b) return a.S < b.S or (a.S == b.S and a.F < b.F) end)
    if #candidates > 0 and candidates[1].S < StepAsideTask.MIN_MOVE then
        self.held = true
        return false
    end
    local firstMode = self.targetUnloader.ad.modes[AutoDrive.MODE_UNLOAD]
    local harvester = (firstMode ~= nil and firstMode.combine) or self.targetUnloader
    for _, c in ipairs(candidates) do
        local sideName = c.side > 0 and "left" or "right"
        if not AutoDrive.checkIsOnField(c.x, 0, c.z) then
            refused[#refused + 1] = sideName .. ": off the field"
        else
            c.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, c.x, 0, c.z)
            local clear, why = FollowCombineTask.isChaseLineClear(vehicle, harvester, c, AutoDrive.CHASEPOS_REAR)
            if clear then
                self.target = c
                self.moves = self.moves + 1
                self.state = StepAsideTask.STATE_MOVING
                self.time = 0
                self.stillTime = 0
                self.moved = false
                self.held = false
                return true
            end
            refused[#refused + 1] = sideName .. " : " .. tostring(why)
        end
    end
    self.held = true
    return false
end

function StepAsideTask:startWaiting()
    self.state = StepAsideTask.STATE_WAITING
    self.waitTime = 0
    self.standTime = 0
    self.quietTime = 0
    self.checkTime = 0
end

function StepAsideTask:updateMoving(dt)
    local t = self.target
    self.time = self.time + dt
    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
    local d = MathUtil.vector2Length(vx - t.x, vz - t.z)
    local _, _, lz = AutoDrive.worldToLocal(self.vehicle, t.x, t.y, t.z)
    local speed = (self.vehicle.lastSpeedReal or 0) * 3600
    if speed > 1 then
        self.moved = true
    end
    if speed < 0.5 and self.moved then
        self.stillTime = self.stillTime + dt
    else
        self.stillTime = 0
    end
    if d < StepAsideTask.ARRIVE_DISTANCE or lz < -1 then
        self:startWaiting()
        return
    end
    if self.time > StepAsideTask.MOVE_TIMEOUT or self.stillTime > StepAsideTask.STILL_TIME then
        self:startWaiting()
        return
    end
    self.vehicle.ad.specialDrivingModule:driveToPoint(dt, t, StepAsideTask.SPEED, true, 1, StepAsideTask.SPEED, true)
end

-- movers still coming (info of the closest), and movers standing ahead within reach
function StepAsideTask:getThreat()
    local coming, standing = nil, false
    local vx, vy, vz = getWorldTranslation(self.vehicle.components[1].node)
    for _, mover in ipairs(StepAsideTask.getMovers(self.targetUnloader)) do
        if mover ~= self.vehicle and mover.components ~= nil then
            local info = StepAsideTask.checkMover(self.vehicle, mover)
            if info ~= nil then
                if coming == nil or info.distance < coming.distance then
                    coming = info
                end
            else
                local m = StepAsideTask.getMotion(self.vehicle, mover)
                local speed = math.sqrt(m.vx * m.vx + m.vz * m.vz) * 3.6
                local mx, my, mz = getWorldTranslation(mover.components[1].node)
                local _, _, lz = AutoDrive.worldToLocal(self.vehicle, mx, my, mz)
                if m.known and speed < StepAsideTask.MIN_SPEED and lz > -StepAsideTask.ARRIVE_DISTANCE
                    and MathUtil.vector2Length(mx - vx, mz - vz) < StepAsideTask.DETECT_DISTANCE then
                    standing = true
                end
            end
        end
    end
    return coming, standing
end

function StepAsideTask:updateWaiting(dt)
    self.vehicle.ad.specialDrivingModule:stopVehicle()
    self.vehicle.ad.specialDrivingModule:update(dt)
    self.waitTime = self.waitTime + dt
    self.checkTime = self.checkTime + dt
    if self.checkTime < StepAsideTask.CHECK_INTERVAL then
        return
    end
    local elapsed = self.checkTime
    self.checkTime = 0
    local coming, standing = self:getThreat()
    if coming ~= nil then
        self.quietTime = 0
        self.standTime = 0
        -- still coming after a step (or none yet): one more try when there is time
        if self.moves < StepAsideTask.MAX_MOVES and self.waitTime > 2000 and not self.replanned
            and (not coming.headOn or coming.distance > StepAsideTask.NEAR_FACING) then
            self.replanned = true
            self:planAside(coming)
        end
        if self.waitTime > StepAsideTask.WAIT_TIMEOUT then
            self:resume(string.format("waited too long (%d s), %s still coming", StepAsideTask.WAIT_TIMEOUT / 1000, logName(coming.mover)))
        end
        return
    end
    if standing then
        self.standTime = self.standTime + elapsed
        if self.standTime < StepAsideTask.PASSED_STAND_TIME then
            return
        end
        self:resume("they have not moved for " .. math.floor(self.standTime / 1000) .. " s")
        return
    end
    self.quietTime = self.quietTime + elapsed
    if self.quietTime >= StepAsideTask.PASSED_QUIET_TIME then
        self:resume("ils sont passes")
    end
end

function StepAsideTask:resume(why)
    self.vehicle.ad.asideCooldownUntil = (g_time or 0) + StepAsideTask.COOLDOWN
    self.vehicle.ad.taskModule:addTask(DriveToVehicleTask:new(self.vehicle, self.targetUnloader, "after stepping aside"))
    self.vehicle.ad.taskModule:setCurrentTaskFinished(ADTaskModule.DONT_PROPAGATE)
end

function StepAsideTask:update(dt)
    if self.targetUnloader == nil or self.targetUnloader.components == nil or g_currentMission.nodeToObject[self.targetUnloader.components[1].node] == nil then
        self:finished()
        return
    end
    local mode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
    if self.promoted or (mode ~= nil and mode.combine ~= nil) then
        -- the 1st left: this one takes over at the harvester, as soon as it stands
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        if (self.vehicle.lastSpeedReal or 0) <= 0.0008 then
            self:finished()
        end
        return
    end
    if self.state == StepAsideTask.STATE_MOVING and self.target ~= nil then
        self:updateMoving(dt)
    else
        self:updateWaiting(dt)
    end
end

function StepAsideTask:signalPromotion()
    self.promoted = true
end

function StepAsideTask:abort()
    if self.targetUnloader ~= nil and self.targetUnloader.ad ~= nil and self.targetUnloader.ad.modes ~= nil then
        self.targetUnloader.ad.modes[AutoDrive.MODE_UNLOAD]:unregisterFollowingUnloader()
    end
end

function StepAsideTask:finished(propagate)
    if self.targetUnloader ~= nil and self.targetUnloader.ad ~= nil and self.targetUnloader.ad.modes ~= nil then
        self.targetUnloader.ad.modes[AutoDrive.MODE_UNLOAD]:unregisterFollowingUnloader()
    end
    self.vehicle.ad.taskModule:setCurrentTaskFinished(propagate)
end

function StepAsideTask:getInfoText()
    return g_i18n:getText("AD_task_step_aside")
end

function StepAsideTask:getI18nInfo()
    return "$l10n_AD_task_step_aside;"
end
