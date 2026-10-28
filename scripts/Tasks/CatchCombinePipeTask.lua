CatchCombinePipeTask = ADInheritsFrom(AbstractTask)
CatchCombinePipeTask.debug = false

CatchCombinePipeTask.TARGET_DISTANCE = 15

CatchCombinePipeTask.STATE_PATHPLANNING = {}
CatchCombinePipeTask.STATE_DRIVING = {}
CatchCombinePipeTask.STATE_REVERSING = {}
CatchCombinePipeTask.STATE_DELAY_PATHPLANNING = {}
CatchCombinePipeTask.STATE_WAIT_BEFORE_FINISH = {}
CatchCombinePipeTask.STATE_FINISHED = {}

CatchCombinePipeTask.MAX_REVERSE_DISTANCE = 18
CatchCombinePipeTask.MIN_COMBINE_DISTANCE = 25
CatchCombinePipeTask.MAX_REVERSE_TIME = 5000
CatchCombinePipeTask.MAX_STUCK_TIME = 60000
CatchCombinePipeTask.MAX_COUNT_NEW_PATHFINDING = 20
-- waiting for a route while the harvester comes towards the unloader: get out of its way
CatchCombinePipeTask.STATE_CLEAR_WAY = {}
CatchCombinePipeTask.STATE_WAIT_PASS_BY = {}
CatchCombinePipeTask.STATE_OPEN_REVERSE = {} -- forage harvester: backing off along its own track to open the angle
CatchCombinePipeTask.STATE_BAND_TURN = {} -- forage harvester: U-turn in the strip, the train faces the wrong way
CatchCombinePipeTask.BAND_TURN_RANGE = 100 -- m from the harvester
CatchCombinePipeTask.BAND_TURN_ANGLE = 90 -- deg: harvester heading this far from ours is behind
CatchCombinePipeTask.BAND_TURN_YAW_RATE = 6 -- deg/s: harvester turning faster than this is in its headland turn
CatchCombinePipeTask.BAND_TURN_AFTER_TURN = 25000 -- ms after the harvester's turn: no U-turn, it comes back to the train
CatchCombinePipeTask.BAND_TURN_BEHIND_RANGE = 40 -- m: harvester only placed behind (heading not reversed) counts this close
CatchCombinePipeTask.OPEN_REVERSE_DISTANCE = 15 -- m (10 was too short)
CatchCombinePipeTask.OPEN_REVERSE_FAILS = 2 -- failed plans from the same place before backing off
CatchCombinePipeTask.OPEN_REVERSE_RADIUS = 15 -- m around a place already backed off from: no new back-off there
CatchCombinePipeTask.OPEN_REVERSE_PAUSE = 120000 -- ms before a back-off is allowed again at the same place
-- no route twice near the standing harvester (headland corner): the chase takes over and may cut a tip of crop on its
-- straight line for CROP_TIP_TIME
CatchCombinePipeTask.CROP_TIP_DISTANCE = 40  -- m from the standing harvester
CatchCombinePipeTask.CROP_TIP_ANGLE = 100    -- deg off its heading at most (not the wrong way round)
CatchCombinePipeTask.CROP_TIP_TIME = 60000   -- ms
-- forage harvester: no route, nothing moved since the last try (same start, harvester still): wait longer each time
CatchCombinePipeTask.RETRY_DELAY_MIN = 1000 -- ms
CatchCombinePipeTask.RETRY_DELAY_MAX = 10000 -- ms
CatchCombinePipeTask.RETRY_SAME_VEHICLE = 1 -- m moved by the unloader counted as the same start
CatchCombinePipeTask.RETRY_SAME_COMBINE = 2 -- m moved by the harvester counted as the same target
CatchCombinePipeTask.CLEAR_WAY_DISTANCE = 40      -- m in front of the harvester watched
CatchCombinePipeTask.CLEAR_WAY_MARGIN = 2         -- m added on each side of the cutting width
CatchCombinePipeTask.CLEAR_WAY_MIN_SPEED = 0.0005 -- harvester speed (lastSpeedReal) counted as driving forward
CatchCombinePipeTask.CLEAR_WAY_MAX_REVERSE = 20   -- m
CatchCombinePipeTask.CLEAR_WAY_REVERSE_SPEED = 6  -- km/h
CatchCombinePipeTask.CLEAR_WAY_TIMEOUT = 25000    -- ms
CatchCombinePipeTask.CLEAR_WAY_STUCK_TIME = 3000  -- ms without moving while reversing
CatchCombinePipeTask.CLEAR_WAY_REAR_CHECK = 3     -- m behind the last unit that must be field ground without crop
CatchCombinePipeTask.PASS_BY_TIMEOUT = 30000      -- ms
CatchCombinePipeTask.PASS_BY_BEHIND = 5           -- m: the unloader is behind the harvester, it has passed
CatchCombinePipeTask.END_ROW_UTURN_AGE = 20000    -- ms after the end-of-row wait: own U-turn tried first
CatchCombinePipeTask.END_ROW_UTURN_ANGLE = 120     -- deg against the harvester heading
-- forage harvester: unloader already beside it (pipe side, same heading, out of its lane, not past
-- its nose): the chase takes over directly, no route, no manoeuvre
CatchCombinePipeTask.BESIDE_ANGLE = 30            -- deg between the unloader and the harvester headings
CatchCombinePipeTask.BESIDE_MAX_LATERAL = 15      -- m from the harvester centre line at most
CatchCombinePipeTask.BESIDE_MAX_BACK = 10         -- m behind the harvester rear at most

function CatchCombinePipeTask:new(vehicle, combine)
    local o = CatchCombinePipeTask:create()
    o.vehicle = vehicle
    o.combine = combine
    o.state = CatchCombinePipeTask.STATE_DELAY_PATHPLANNING
    o.wayPoints = nil
    o.stuckTimer = AutoDriveTON:new()
    o.reverseTimer = AutoDriveTON:new()
    o.waitTimer = AutoDriveTON:new()
    o.waitForCheckTimer = AutoDriveTON:new()
    o.waitForCheckTimer.elapsedTime = 4000
    o.taskType = "CatchCombinePipeTask"
    o.newPathFindingCounter = 0
    o.retryDelay = CatchCombinePipeTask.RETRY_DELAY_MIN
    o.trailers = nil
    CatchCombinePipeTask.setStateNames(o)
    return o
end

function CatchCombinePipeTask:setUp()
    CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:setUp()")
    local angleToCombineHeading = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getAngleToCombineHeading()
    local angleToCombine = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getAngleToCombine()

    if angleToCombineHeading < 35 and angleToCombine < 90 and AutoDrive.getDistanceBetween(self.vehicle, self.combine) < 60
        and not self.combine.ad.isFixedPipeChopper then
        -- close and aligned: chase straight away, but only if the straight line is free, otherwise plan a route
        local chasePos, chaseSide = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getPipeChasePosition()
        local clear, reason = FollowCombineTask.isChaseLineClear(self.vehicle, self.combine, chasePos, chaseSide)
        if clear then
            self:finished()
        end
    end
    self.trailers, _ = AutoDrive.getAllUnits(self.vehicle)
    AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, true)
end

function CatchCombinePipeTask:update(dt)
    if self.combine ~= nil and g_currentMission.nodeToObject[self.combine.components[1].node] == nil then
        self:finished()
        return
    end
    if self.lastState ~= self.state then
        CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update %s -> %s", tostring(self:getStateName(self.lastState)), tostring(self:getStateName()))
        self.lastState = self.state
        self:resetAllTimers()
    end
    self:sampleCombineHeading()

    if self.state == CatchCombinePipeTask.STATE_BAND_TURN then
        local result = ADBandTurn.update(self.vehicle, dt)
        if result == "joined" then
            self.state = CatchCombinePipeTask.STATE_FINISHED
        elseif result ~= "running" then
            if result == "done" then
                -- turned: plan at once, the earlier failures were for the other heading
                self.retryDelay = CatchCombinePipeTask.RETRY_DELAY_MIN
                self.lastFailure = nil
            end
            self.waitForCheckTimer.elapsedTime = 4000
            self.state = CatchCombinePipeTask.STATE_DELAY_PATHPLANNING
        end
        return
    end

    local checkForStuck = (self.vehicle.lastSpeedReal <= 0.0002) and (
        self.state == CatchCombinePipeTask.STATE_DRIVING
    )

    -- stuck against a vehicle (waiting trailers) on the way to the harvester: back off in its own
    -- tracks, then plan again from there (the planned route did not see them or they moved meanwhile)
    if self:updateVehicleBlocked(dt, checkForStuck) then
        return
    end

    self.stuckTimer:timer(checkForStuck, self.MAX_STUCK_TIME, dt)
    if self.stuckTimer:done()
        -- or AutoDrive.getDistanceBetween(self.vehicle, self.combine) < self.MIN_COMBINE_DISTANCE 
        then
        -- got stuck or to close to combine -> reverse
        CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - STATE_DRIVING stuck")
        self.stuckTimer:timer(false)
        local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
        self.reverseStartLocation = {x = x, y = y, z = z}
        self.state = CatchCombinePipeTask.STATE_REVERSING
        return
    end

    if (self.state == CatchCombinePipeTask.STATE_PATHPLANNING or self.state == CatchCombinePipeTask.STATE_DELAY_PATHPLANNING)
        and self:isInCombinePath() then
        self:startClearWay()
        return
    end

    if (self.state == CatchCombinePipeTask.STATE_PATHPLANNING or self.state == CatchCombinePipeTask.STATE_DELAY_PATHPLANNING
        or self.state == CatchCombinePipeTask.STATE_DRIVING) and self:isBesideInPlace() then
        if self.state == CatchCombinePipeTask.STATE_PATHPLANNING or self.preplanning then
            self.vehicle.ad.pathFinderModule:reset()
            self.preplanning = false
        end
        self:noteHandover()
        self:finished()
        return
    end

    -- end of row, unloader against the harvester heading (drawing): its own U-turn first, to the
    -- harvester side (forward, back then forward, 3 points), the planned route only if none fits
    if self.vehicle.ad.endRowUTurn ~= nil and (self.state == CatchCombinePipeTask.STATE_PATHPLANNING or self.state == CatchCombinePipeTask.STATE_DELAY_PATHPLANNING) then
        local since = (g_time or 0) - self.vehicle.ad.endRowUTurn
        self.vehicle.ad.endRowUTurn = nil
        if since < CatchCombinePipeTask.END_ROW_UTURN_AGE and self:tryEndRowUTurn() then
            return
        end
    end

    if self.state == CatchCombinePipeTask.STATE_CLEAR_WAY then
        self:updateClearWay(dt)
        return
    elseif self.state == CatchCombinePipeTask.STATE_WAIT_PASS_BY then
        self:updateWaitPassBy(dt)
        return
    end

    -- forage harvester: the chase was blocked several times at the same place (no straight line forward) and the
    -- harvester heading is behind: U-turn in the strip as a fallback
    if self.closeRangePlanning and not self.bandTurnChecked then
        self.bandTurnChecked = true
        if self:tryBandTurn("chase blocked") then
            return
        end
    end

    -- forage harvester close by: no planned route to its pipe (a route that must arrive in its heading loops
    -- around at that range), the chase takes over: strict straight line, wait for it to pass, change side.
    -- Only when the chase gave up several times at the same place, a planned route is allowed again
    -- (seen in game: 29 m behind the standing harvester, aligned, routes to a point 10 m ahead looping
    -- for 4 min): a standing harvester waits for this unloader, the chase takes over from further away
    local closeRange = CatchCombinePipeTask.CLOSE_RANGE
    if self.state ~= CatchCombinePipeTask.STATE_DRIVING and self:isCombineStanding() then
        local _, _, inCone = self:isInRearCone()
        if inCone then
            closeRange = CatchCombinePipeTask.CLOSE_RANGE_STANDING
        end
    end
    if not self.closeRangePlanning and AutoDrive.isChopperUnloading(self.vehicle)
        and (self.state == CatchCombinePipeTask.STATE_PATHPLANNING or self.state == CatchCombinePipeTask.STATE_DELAY_PATHPLANNING or self.state == CatchCombinePipeTask.STATE_DRIVING)
        and AutoDrive.getDistanceBetween(self.vehicle, self.combine) < closeRange then
        -- only in
        -- the harvester's direction. An unloader passed the standing harvester the wrong way round (177 deg) at 10 m, the
        -- chase took over and looped 4 min (chase, reverse, new route, chase...). The wrong way round: a route being
        -- driven goes on (it ends behind the harvester); without a route its own U-turn first, then a new route
        local angle = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getAngleToCombineHeading()
        if angle < CatchCombinePipeTask.CLOSE_RANGE_ANGLE then
            if self.state == CatchCombinePipeTask.STATE_PATHPLANNING or self.preplanning then
                self.vehicle.ad.pathFinderModule:reset()
                self.preplanning = false
            end
            self:noteHandover()
            self.state = CatchCombinePipeTask.STATE_FINISHED
        elseif self.state ~= CatchCombinePipeTask.STATE_DRIVING then
            if angle >= CatchCombinePipeTask.REVERSED_ANGLE and self:tryReversedUTurn(angle, true) then
                return
            end
        end
    end

    if self.state == CatchCombinePipeTask.STATE_PATHPLANNING then
        if self.vehicle.ad.pathFinderModule:hasFinished() then
            CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - STATE_PATHPLANNING finished")
            self.newPathFindingCounter = 0
            self.wayPoints = self.vehicle.ad.pathFinderModule:getPath()
            if (self.wayPoints == nil or #self.wayPoints < 1) and self.vehicle.ad.pathFinderModule:tryGuidedDeparture() then
                -- forage harvester: no route to it from here, a guided departure frees the train first (desenclavement),
                -- the next plan starts from where it stops
                self.wayPoints = self.vehicle.ad.pathFinderModule:getPath()
            end
            if self.wayPoints == nil or #self.wayPoints < 1 then
                --restart
                self.infoNoPath = true -- status text only
                self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:notifyAboutFailedPathfinder()
                self:updateRetryDelay()
                -- behind a standing harvester (it waits for this unloader) no planned route loops for ever
                if self:tryStandingHandover("no route") then
                    return
                end
                -- behind the harvester on its own trail, drive that trail
                if self:tryCombineTrail() then
                    return
                end
                -- fallback: no forward route and the harvester heading is behind, U-turn in the strip
                if self:tryBandTurn("no forward route") then
                    return
                end
                if self:tryOpenReverse() then
                    return
                end
                --AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.WARN, "$l10n_AD_Driver_of; %s $l10n_AD_cannot_find_path; %s", 5000, self.vehicle.ad.stateModule:getName(), self.combine.ad.stateModule:getName())
                CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - STATE_PATHPLANNING restarting path finder - with delay")
                self.state = CatchCombinePipeTask.STATE_DELAY_PATHPLANNING
                return
            else
                -- (seen in game: the same 12 m one-cell route every 1.3 s, ending at 101 deg from the standing
                -- harvester): a short route that cannot turn the train toward the harvester heading is useless, the
                -- wrong-way ladder runs instead (band turn toward its heading, back off, more room in the next route)
                if self:isUselessShortRoute(self.wayPoints) then
                    local pfm = self.vehicle.ad.pathFinderModule
                    local wasPartial = pfm.partialPath == true
                    pfm:reset()
                    self.preplanning = false
                    if self:tryWrongWayTurn(self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getAngleToCombineHeading()) then
                        return
                    end
                    self:updateRetryDelay()
                    -- a route cut short by the face-to-face rule and refused twice the same way: that rule is lifted for
                    -- the next plan
                    if wasPartial and self.lastFailure ~= nil and self.retryDelay > CatchCombinePipeTask.RETRY_DELAY_MIN and not pfm.partialFacingLifted then
                        pfm.partialFacingLifted = true
                    end
                    self.state = CatchCombinePipeTask.STATE_DELAY_PATHPLANNING
                    return
                end
                self.lastFailure = nil
                self.infoNoPath = false
                self.retryDelay = CatchCombinePipeTask.RETRY_DELAY_MIN
                self.vehicle.ad.drivePathModule:setWayPoints(self.wayPoints)
                self.routeIsPartial = self.vehicle.ad.pathFinderModule.partialPath == true
                self.preplanning = false
                self.state = CatchCombinePipeTask.STATE_DRIVING
                return
            end
        else
            self.vehicle.ad.pathFinderModule:update(dt)
            self.vehicle.ad.specialDrivingModule:stopVehicle()
            self.vehicle.ad.specialDrivingModule:update(dt)
            return
        end
    elseif self.state == CatchCombinePipeTask.STATE_OPEN_REVERSE then
        local result = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:updateTrackReverse(dt)
        if result ~= "running" then
            self.state = CatchCombinePipeTask.STATE_DELAY_PATHPLANNING
        end
        return
    elseif self.state == CatchCombinePipeTask.STATE_DELAY_PATHPLANNING then
        CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - STATE_DELAY_PATHPLANNING")
        if self.waitForCheckTimer:timer(true, self.retryDelay or CatchCombinePipeTask.RETRY_DELAY_MIN, dt) then
            if self.newPathFindingCounter > self.MAX_COUNT_NEW_PATHFINDING then
                -- prevent deadlock
                self.state = CatchCombinePipeTask.STATE_FINISHED
                return
            end
            if self:startNewPathFinding() then
                self.state = CatchCombinePipeTask.STATE_PATHPLANNING
                return
            end
        end        
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return
    elseif self.state == CatchCombinePipeTask.STATE_DRIVING then
        -- check if this is still a clever path to follow
        -- do this by distance of the combine to the last location pathfinder started at
        local x, y, z = getWorldTranslation(self.combine.components[1].node)
        local combineTravelDistance = MathUtil.vector2Length(x - self.combinesStartLocation.x, z - self.combinesStartLocation.z)

        if combineTravelDistance > 85 then
            CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - combine travelled - recalculate path")
            self.preplanning = false
            self.waitForCheckTimer.elapsedTime = 4000
            self.state = CatchCombinePipeTask.STATE_DELAY_PATHPLANNING
            return
        else
            if self.vehicle.ad.drivePathModule:isTargetReached() and self.preplanning then
                -- end of a partial route, the next route is still being planned: wait for it here
                self.preplanning = false
                self.state = CatchCombinePipeTask.STATE_PATHPLANNING
                return
            elseif self.vehicle.ad.drivePathModule:isTargetReached() then
                -- check if we have actually reached the target or not
                -- accept current location if we are in a good position to start chasing: distance and angle are important here
                CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - STATE_DRIVING TargetReached")
                local angleToCombine = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getAngleToCombineHeading()
                local isCorrectSide = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:isUnloaderOnCorrectSide()
                -- forage harvester in a curve: behind it on its own track and aligned with that track,
                -- the chase takes over even when the harvester itself points elsewhere
                -- not when facing it (accepted at 167 deg on the previous row's track)
                if angleToCombine >= CatchCombinePipeTask.CHOPPER_HANDOVER_ANGLE and angleToCombine < 90 and AutoDrive.isChopperUnloading(self.vehicle) and self:isBehindChopper() then
                    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
                    local tx, tz = PathFinderModule.getTrailDirectionAt(self.combine, vx, vz)
                    if tx ~= nil then
                        local rx, _, rz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
                        local trailAngle = math.abs(AutoDrive.angleBetween({x = rx, z = rz}, {x = tx, z = tz}))
                        if trailAngle < CatchCombinePipeTask.CHOPPER_HANDOVER_ANGLE then
                            angleToCombine = trailAngle
                            isCorrectSide = true
                        end
                    end
                end

                -- forage harvester: behind it, the chase takes over up to CHOPPER_HANDOVER_ANGLE (curves)
                local maxAngle = AutoDrive.isChopperUnloading(self.vehicle) and CatchCombinePipeTask.CHOPPER_HANDOVER_ANGLE or 35
                -- in the cone behind a forage harvester the planned side does not matter
                isCorrectSide = self:acceptOtherSideBehind(angleToCombine, isCorrectSide, maxAngle)
                if angleToCombine < maxAngle and AutoDrive.getDistanceBetween(self.vehicle, self.combine) < 80 and isCorrectSide and self:isBehindChopper() then
                    CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - STATE_DRIVING -> STATE_FINISHED")
                    self:noteHandover()
                    self.state = CatchCombinePipeTask.STATE_FINISHED
                    return
                else
                    -- anticipated end-of-row U-turn: in place behind the next row before the harvester,
                    -- wait there for it to come in instead of planning again
                    local ant = self.vehicle.ad.anticipatedRear
                    if ant ~= nil and AutoDrive.isChopperUnloading(self.vehicle) and (g_time or 0) - ant.t < PathFinderModule.ANTICIPATED_MAX_AGE then
                        self.infoAnticipatedWaitAt = g_time or 0 -- status text
                        self.vehicle.ad.specialDrivingModule:stopVehicle()
                        self.vehicle.ad.specialDrivingModule:update(dt)
                        return
                    end
                    -- (sim b): behind it but the wrong way round: U-turn now, the AutoDrive one first
                    -- (HandleHarvesterTurnTask, end-of-row mode), then the 3/5/7-move manoeuvre
                    if self:tryReversedUTurn(angleToCombine, true) then
                        return
                    end
                    -- (seen in game: an unloader at 101 deg, 22 m from
                    -- the standing harvester, looping every 1.3 s): from the 2nd arrival at the same place, band turn toward
                    -- the harvester heading, else back off, and more room for the turn in the next route
                    if self:tryWrongWayTurn(angleToCombine) then
                        return
                    end
                    -- forage harvester: close to it but unable to hand over (wrong heading or side, face to face):
                    -- recalculating gives the same 1-cell route forever. After HANDOVER_TRIES at the same place, the chase
                    -- takes over anyway: it waits for the harvester to pass, backs off or changes side (original procedures)
                    if AutoDrive.isChopperUnloading(self.vehicle) and AutoDrive.getDistanceBetween(self.vehicle, self.combine) < CatchCombinePipeTask.HANDOVER_DISTANCE then
                        local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
                        if self.handoverPlace ~= nil and MathUtil.vector2Length(vx - self.handoverPlace.x, vz - self.handoverPlace.z) < 2 then
                            self.handoverTries = (self.handoverTries or 0) + 1
                        else
                            self.handoverPlace = {x = vx, z = vz}
                            self.handoverTries = 1
                        end
                        if self.handoverTries >= CatchCombinePipeTask.HANDOVER_TRIES and self:tryBandTurn("near the harvester without room to line up") then
                            self.handoverPlace = nil
                            self.handoverTries = 0
                            return
                        end
                    end
                    CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - angle or distance to combine too high - recalculate path now")
                    self.state = CatchCombinePipeTask.STATE_DELAY_PATHPLANNING
                    return
                end
            else
                self.vehicle.ad.drivePathModule:update(dt)
                self:updatePreplanning(dt)
            end
        end
    elseif self.state == CatchCombinePipeTask.STATE_REVERSING then
        local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
        local distanceToReverseStart = MathUtil.vector2Length(x - self.reverseStartLocation.x, z - self.reverseStartLocation.z)
        self.reverseTimer:timer(true, self.MAX_REVERSE_TIME, dt)
        if distanceToReverseStart > self.MAX_REVERSE_DISTANCE or self.reverseTimer:done() then
            self.state = CatchCombinePipeTask.STATE_WAIT_BEFORE_FINISH
            return
        else
            self.vehicle.ad.specialDrivingModule:driveReverse(dt, 15, 1, self.vehicle.ad.trailerModule:canBeHandledInReverse())
        end
    elseif self.state == CatchCombinePipeTask.STATE_WAIT_BEFORE_FINISH then
        CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - STATE_WAIT_BEFORE_FINISH")
        self.waitTimer:timer(true, self.MAX_REVERSE_TIME, dt)
        if self.waitTimer:done() then
            self.state = CatchCombinePipeTask.STATE_FINISHED
            return
        else
            self.vehicle.ad.specialDrivingModule:stopVehicle()
            self.vehicle.ad.specialDrivingModule:update(dt)
            return
        end
    elseif self.state == CatchCombinePipeTask.STATE_FINISHED then
        CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:update - STATE_FINISHED")
        self:finished()
        return
    end
end

function CatchCombinePipeTask:abort()
    ADBandTurn.abort(self.vehicle)
end

-- tractor and towed units of the unloader
function CatchCombinePipeTask:getTrainUnits()
    local units = {self.vehicle}
    local all = AutoDrive.getAllUnits(self.vehicle)
    if all ~= nil then
        for _, unit in ipairs(all) do
            if unit ~= self.vehicle and unit.components ~= nil then
                table.insert(units, unit)
            end
        end
    end
    return units
end

-- the harvester drives forward and a unit of the unloader stands in its way: in front of it, within its cutting width
function CatchCombinePipeTask:isInCombinePath()
    local combine = self.combine
    if combine == nil or combine.ad == nil or combine.ad.ADRootNode == nil then
        return false
    end
    local movingDirection = combine.movingDirection or 1
    if combine.ad.isReverseAttached then
        movingDirection = -movingDirection
    end
    if (combine.lastSpeedReal or 0) * movingDirection < CatchCombinePipeTask.CLEAR_WAY_MIN_SPEED then
        return false
    end
    local laneHalf = math.max(AutoDrive.getFrontToolWidth(combine) or 0, combine.size.width) / 2 + CatchCombinePipeTask.CLEAR_WAY_MARGIN
    for _, unit in ipairs(self:getTrainUnits()) do
        local x, y, z = getWorldTranslation(unit.components[1].node)
        local diffX, _, diffZ = AutoDrive.worldToLocal(combine, x, y, z, combine.ad.ADRootNode)
        local unitHalfWidth = unit.size ~= nil and unit.size.width / 2 or 1.5
        if diffZ > 0 and diffZ < CatchCombinePipeTask.CLEAR_WAY_DISTANCE and math.abs(diffX) < laneHalf + unitHalfWidth then
            return true
        end
    end
    return false
end

-- ground behind the last unit: field ground without standing crop, across the width of the train
function CatchCombinePipeTask:isRearClear()
    local units = self:getTrainUnits()
    local last = units[#units]
    local dx, _, dz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local x, _, z = getWorldTranslation(last.components[1].node)
    local back = (last.size ~= nil and last.size.length / 2 or 3) + CatchCombinePipeTask.CLEAR_WAY_REAR_CHECK
    local half = self.vehicle.ad.pathFinderModule:getTrainWidth() / 2
    local rx, rz = x - dx * back, z - dz * back
    for _, side in ipairs({0, -1, 1}) do
        local px, pz = rx - dz * half * side, rz + dx * half * side
        if not AutoDrive.checkIsOnField(px, 0, pz) or PathFinderModule.getMapCropAt(px, pz) ~= 0 then
            return false
        end
    end
    return true
end

function CatchCombinePipeTask:startClearWay()
    if self.state == CatchCombinePipeTask.STATE_PATHPLANNING then
        self.vehicle.ad.pathFinderModule:reset()
    end
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    self.clearWayStart = {x = x, z = z}
    self.clearWayTime = 0
    self.clearWayStuckTime = 0
    self.clearWayLast = {x = x, z = z}
    if self:isRearClear() then
        self.state = CatchCombinePipeTask.STATE_CLEAR_WAY
    else
        self.passByTime = 0
        self.state = CatchCombinePipeTask.STATE_WAIT_PASS_BY
    end
end

function CatchCombinePipeTask:updateClearWay(dt)
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    local reversed = MathUtil.vector2Length(x - self.clearWayStart.x, z - self.clearWayStart.z)
    self.clearWayTime = self.clearWayTime + dt
    if MathUtil.vector2Length(x - self.clearWayLast.x, z - self.clearWayLast.z) > 0.3 then
        self.clearWayLast = {x = x, z = z}
        self.clearWayStuckTime = 0
    else
        self.clearWayStuckTime = self.clearWayStuckTime + dt
    end
    local reason = nil
    if not self:isInCombinePath() then
        reason = "out of the way"
    elseif reversed >= CatchCombinePipeTask.CLEAR_WAY_MAX_REVERSE then
        reason = "max reverse"
    elseif not self:isRearClear() then
        reason = "crop or field edge behind"
    elseif self.clearWayTime > CatchCombinePipeTask.CLEAR_WAY_TIMEOUT or self.clearWayStuckTime > CatchCombinePipeTask.CLEAR_WAY_STUCK_TIME then
        reason = "reverse blocked"
    end
    if reason ~= nil then
        self.passByTime = 0
        self.state = CatchCombinePipeTask.STATE_WAIT_PASS_BY
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return
    end
    self.vehicle.ad.specialDrivingModule:driveReverse(dt, CatchCombinePipeTask.CLEAR_WAY_REVERSE_SPEED, 1, self.vehicle.ad.trailerModule:canBeHandledInReverse())
end

-- forage harvester: unloader on the pipe side, same heading, outside its lane, level with it (not past its nose)
function CatchCombinePipeTask:isBesideInPlace()
    local combine = self.combine
    if not AutoDrive.isChopperUnloading(self.vehicle) or combine == nil or combine.ad == nil or combine.components == nil then
        return false
    end
    local mode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
    if mode:getAngleToCombineHeading() >= CatchCombinePipeTask.BESIDE_ANGLE or not mode:isUnloaderOnCorrectSide() then
        return false
    end
    local cx, _, cz = getWorldTranslation(combine.components[1].node)
    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
    local rx, rz = self.vehicle.ad.pathFinderModule:getWorkingDirection(combine)
    local along = (vx - cx) * rx + (vz - cz) * rz
    local lateral = math.abs((vx - cx) * rz - (vz - cz) * rx)
    local halfLength = combine.size ~= nil and combine.size.length / 2 or 5
    if along > halfLength or along < -(halfLength + CatchCombinePipeTask.BESIDE_MAX_BACK) then
        return false
    end
    local laneHalf = math.max(AutoDrive.getFrontToolWidth(combine) or 0, combine.size.width) / 2
    local unitHalfWidth = self.vehicle.size ~= nil and self.vehicle.size.width / 2 or 1.5
    return lateral >= laneHalf + unitHalfWidth and lateral <= CatchCombinePipeTask.BESIDE_MAX_LATERAL and not self:isInCombinePath()
end

function CatchCombinePipeTask:updateWaitPassBy(dt)
    self.vehicle.ad.specialDrivingModule:stopVehicle()
    self.vehicle.ad.specialDrivingModule:update(dt)
    self.passByTime = self.passByTime + dt
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local _, _, diffZ = AutoDrive.worldToLocal(self.combine, x, y, z, self.combine.ad.ADRootNode)
    local passed = diffZ < -CatchCombinePipeTask.PASS_BY_BEHIND
    if passed or (not self:isInCombinePath() and self.passByTime > 2000) or self.passByTime > CatchCombinePipeTask.PASS_BY_TIMEOUT then
        self.waitForCheckTimer.elapsedTime = 4000
        self.state = CatchCombinePipeTask.STATE_DELAY_PATHPLANNING
    end
end

function CatchCombinePipeTask:finished(propagate)
    CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:finished()")
    self.vehicle.ad.taskModule:setCurrentTaskFinished(propagate)
end

function CatchCombinePipeTask:startNewPathFinding()
    self.infoOnTrail = false -- status text
    CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:startNewPathFinding()")
    local pipeChasePos, pipeChaseSide = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getPipeChasePosition(true)
    local x, _, z = getWorldTranslation(self.combine.components[1].node)
    local targetFieldId = g_farmlandManager:getFarmlandIdAtWorldPosition(pipeChasePos.x, pipeChasePos.z)
    local combineFieldId = g_farmlandManager:getFarmlandIdAtWorldPosition(x, z)

    -- Only chase the rear on low fill levels of the combine. This should prevent getting into unneccessarily tight spots for the final approach to the pipe.
    -- Also for small fields, there is often no purpose in chasing so far behind the combine as it will already start a turn soon

    local cfillLevel, cfillCapacity, _, cfillFreeCapacity = AutoDrive.getObjectFillLevels(self.combine)
    local cFillRatio = cfillCapacity > 0 and cfillLevel / cfillCapacity or 0

    if cFillRatio > 0.91 or cfillFreeCapacity < 0.1 then
        CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:startNewPathFinding() - Combine is almost full - dont chase for active unloading anymore")
        self:finished(ADTaskModule.DONT_PROPAGATE)
        self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:setToWaitForCall()
        return false
    end

    self.newPathFindingCounter = self.newPathFindingCounter + 1 -- used to prevent deadlock

    if self.combine.ad.isChopper or (pipeChaseSide ~= AutoDrive.CHASEPOS_REAR or (targetFieldId == combineFieldId and cFillRatio <= 0.85)) then
    -- if self.combine:getIsBufferCombine() or (pipeChaseSide ~= AutoDrive.CHASEPOS_REAR and targetFieldId == combineFieldId and cFillRatio <= 0.85) then
        -- is chopper or chase not rear and harvester on correct field and filled < 85% - i.e. combine pipe not in fruit
        CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:startNewPathFinding() - chase pos looks good - calculate path to it...")
        self.vehicle.ad.pathFinderModule:reset()
        -- next piece of a route: planned from the end of the piece being driven
        self.vehicle.ad.pathFinderModule.startPose = self.nextStartPose
        self.vehicle.ad.pathFinderModule.edgeOnly = self.edgeOnly == true
        self.vehicle.ad.pathFinderModule:startPathPlanningToPipe(self.combine, false)
        -- use false to enable pathfinder fallback
        -- no clean route in strict: drive part of the way, this task plans again on arrival
        self.vehicle.ad.pathFinderModule.allowPartialPath = true
        self.combinesStartLocation = {}
        self.combinesStartLocation.x, self.combinesStartLocation.y, self.combinesStartLocation.z = getWorldTranslation(self.combine.components[1].node)
        return true
    else
        CatchCombinePipeTask.debugMsg(self.vehicle, "CatchCombinePipeTask:startNewPathFinding() - chase pos looks bad, is not on the same field or combine's fill level is approaching limit - aborting for now")
        self.waitForCheckTimer:timer(false)
    end
    return false
end

-- forage harvester: a partial route ends with a stop and a new plan. The next piece is planned as soon as the
-- tractor has started this one, from the end of this piece (its position and heading), and appended to it:
-- the tractor drives on without stopping. Relaunched if the harvester moved a lot; no route: stop and plan as before
CatchCombinePipeTask.CLOSE_RANGE = 10             -- m from the forage harvester: the chase drives, no planned route (was 4)
CatchCombinePipeTask.CLOSE_RANGE_ANGLE = 90       -- deg: only in the harvester's direction (seen in game: 177 deg, looped 4 min)
CatchCombinePipeTask.CLOSE_RANGE_STANDING = 32    -- m: harvester standing and the unloader in the cone behind it, before any route (an unloader looping 4 min at 29 m)
CatchCombinePipeTask.CLOSE_BLOCKED_TRIES = 3       -- chases given up at the same place before a planned route is allowed
CatchCombinePipeTask.HANDOVER_DISTANCE = 15        -- m from the harvester: the chase takes over when the route cannot align
CatchCombinePipeTask.HANDOVER_TRIES = 2
CatchCombinePipeTask.CHOPPER_HANDOVER_ANGLE = 50  -- deg between unloader and forage harvester headings, behind it
-- forage harvester standing (full, it waits for this unloader) and the unloader in the cone behind it: the chase takes
-- over whatever side was planned, it picks the free side itself (an unloader arrived 22 m behind the
-- standing JAGUAR at 35 deg, right of the planned left side, and planned the same point again for ever)
CatchCombinePipeTask.STANDING_HANDOVER_DISTANCE = 30 -- m from the harvester
CatchCombinePipeTask.STANDING_SPEED = 0.0003 -- lastSpeedReal (m/ms) under which the harvester stands

-- hand-overs to the chase at the same place in a short time (the chase gives the place back at once:
-- obstacle on its line): after HANDOVER_LOOP_COUNT the place behind the harvester is forced for HANDOVER_LOOP_REAR_TIME
CatchCombinePipeTask.HANDOVER_LOOP_COUNT = 4
CatchCombinePipeTask.HANDOVER_LOOP_TIME = 30000
CatchCombinePipeTask.HANDOVER_LOOP_RADIUS = 3
CatchCombinePipeTask.HANDOVER_LOOP_REAR_TIME = 60000
function CatchCombinePipeTask:noteHandover()
    local mode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
    if mode == nil or not AutoDrive.isChopperUnloading(self.vehicle) then
        return
    end
    local now = g_time or 0
    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
    local h = mode.handoverLoop
    if h == nil or now - h.t0 > CatchCombinePipeTask.HANDOVER_LOOP_TIME or MathUtil.vector2Length(vx - h.x, vz - h.z) > CatchCombinePipeTask.HANDOVER_LOOP_RADIUS then
        mode.handoverLoop = {x = vx, z = vz, t0 = now, n = 1}
        return
    end
    h.n = h.n + 1
    if h.n >= CatchCombinePipeTask.HANDOVER_LOOP_COUNT and mode.isIslandRear ~= nil and not mode:isIslandRear() then
        mode.islandRearUntil = now + CatchCombinePipeTask.HANDOVER_LOOP_REAR_TIME
        mode.handoverLoop = nil
    end
end

-- forage harvester: the unloader is behind it (its rear half), not beside or ahead; any other harvester: true
function CatchCombinePipeTask:isBehindChopper()
    if not AutoDrive.isChopperUnloading(self.vehicle) or self.combine == nil or self.combine.components == nil then
        return true
    end
    local cx, _, cz = getWorldTranslation(self.combine.components[1].node)
    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
    local rx, rz = self.vehicle.ad.pathFinderModule:getWorkingDirection(self.combine)
    return (vx - cx) * rx + (vz - cz) * rz < 0
end

-- forage harvester standing: full, it waits for this unloader
function CatchCombinePipeTask:isCombineStanding()
    local c = self.combine
    if c == nil then
        return false
    end
    if math.abs(c.lastSpeedReal or 0) < CatchCombinePipeTask.STANDING_SPEED then
        return true
    end
    return c.ad ~= nil and c.ad.noMovementTimer ~= nil and (c.ad.noMovementTimer.elapsedTime or 0) > 1000
end

-- the unloader in the cone behind the forage harvester: behind its rear, no further aside than behind.
-- Returns its lateral and longitudinal offsets (m, harvester frame) and whether it is in the cone
function CatchCombinePipeTask:isInRearCone()
    if self.combine == nil or self.combine.components == nil then
        return 0, 0, false
    end
    local vx, vy, vz = getWorldTranslation(self.vehicle.components[1].node)
    local dx, _, dz = AutoDrive.worldToLocal(self.combine, vx, vy, vz, self.combine.ad ~= nil and self.combine.ad.ADRootNode or nil)
    local rear = (self.combine.size ~= nil and self.combine.size.length or 10) / 2
    return dx, dz, dz < -rear and math.abs(dx) <= -dz
end

-- arrival behind a forage harvester on the other side than the planned one (an unloader arrived 22 m
-- behind the standing JAGUAR at 35 deg, right of the planned left side, and planned the same point for ever): in the
-- cone behind it the side does not matter, the chase picks the free side itself (or drives behind it first)
function CatchCombinePipeTask:acceptOtherSideBehind(angleToCombine, isCorrectSide, maxAngle)
    if isCorrectSide or not AutoDrive.isChopperUnloading(self.vehicle) or angleToCombine >= maxAngle then
        return isCorrectSide
    end
    local dx, dz, inCone = self:isInRearCone()
    if not inCone then
        return false
    end
    return true
end

-- no route and the forage harvester stands (full, it waits for this unloader), the unloader in the cone behind it less
-- than CHOPPER_HANDOVER_ANGLE off its heading: the chase takes over, a planned route only loops at that range
function CatchCombinePipeTask:tryStandingHandover(why)
    if not AutoDrive.isChopperUnloading(self.vehicle) or not self:isCombineStanding() then
        return false
    end
    local dx, dz, inCone = self:isInRearCone()
    if not inCone then
        return false
    end
    local distance = AutoDrive.getDistanceBetween(self.vehicle, self.combine)
    local angle = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getAngleToCombineHeading()
    if distance > CatchCombinePipeTask.STANDING_HANDOVER_DISTANCE or angle >= CatchCombinePipeTask.CHOPPER_HANDOVER_ANGLE then
        return false
    end
    self:noteHandover()
    if self.state == CatchCombinePipeTask.STATE_PATHPLANNING or self.preplanning then
        self.vehicle.ad.pathFinderModule:reset()
        self.preplanning = false
    end
    self.state = CatchCombinePipeTask.STATE_FINISHED
    return true
end
CatchCombinePipeTask.PREPLAN_START_DISTANCE = 1    -- m driven on the piece before planning the next one
CatchCombinePipeTask.PREPLAN_COMBINE_MOVED = 30    -- m the harvester may move before the next piece is planned again
CatchCombinePipeTask.PREPLAN_MIN_LEFT = 5          -- m left on the piece to still relaunch
CatchCombinePipeTask.PREPLAN_MAX_STANDING = 3      -- pieces appended at most while the tractor stays within PREPLAN_STANDING_RADIUS
CatchCombinePipeTask.PREPLAN_STANDING_RADIUS = 10  -- m

function CatchCombinePipeTask:updatePreplanning(dt)
    if not self.routeIsPartial or not AutoDrive.isChopperUnloading(self.vehicle) then
        return
    end
    local pf = self.vehicle.ad.pathFinderModule
    local dpm = self.vehicle.ad.drivePathModule
    local vx, vy, vz = getWorldTranslation(self.vehicle.components[1].node)
    local name = tostring(self.vehicle:getName())
    local wps = dpm:getWayPoints()
    if wps == nil or #wps < 2 then
        self.routeIsPartial = false
        return
    end
    local last, before = wps[#wps], wps[#wps - 1]
    local leftOnPiece = MathUtil.vector2Length(last.x - vx, last.z - vz)
    if not self.preplanning then
        if MathUtil.vector2Length(wps[1].x - vx, wps[1].z - vz) < CatchCombinePipeTask.PREPLAN_START_DISTANCE then
            return
        end
        local dx, dz = last.x - before.x, last.z - before.z
        local l = math.sqrt(dx * dx + dz * dz)
        if l < 0.1 then
            self.routeIsPartial = false
            return
        end
        -- forage harvester: the piece ends within CLOSE_RANGE of it, the chase drives from there (no grid route near it)
        local hx, _, hz = getWorldTranslation(self.combine.components[1].node)
        if AutoDrive.isChopperUnloading(self.vehicle) and MathUtil.vector2Length(last.x - hx, last.z - hz) < CatchCombinePipeTask.CLOSE_RANGE then
            self.routeIsPartial = false
            return
        end
        self.nextStartPose = {x = last.x, y = last.y or vy, z = last.z, rx = dx / l, rz = dz / l}
        if self:startNewPathFinding() then
            self.preplanning = true
            local cx, _, cz = getWorldTranslation(self.combine.components[1].node)
            self.preplanCombine = {x = cx, z = cz}
        else
            self.routeIsPartial = false
        end
        self.nextStartPose = nil
        return
    end
    pf:update(dt)
    if not pf:hasFinished() then
        return
    end
    self.preplanning = false
    local cx, _, cz = getWorldTranslation(self.combine.components[1].node)
    local moved = MathUtil.vector2Length(cx - self.preplanCombine.x, cz - self.preplanCombine.z)
    if moved > CatchCombinePipeTask.PREPLAN_COMBINE_MOVED then
        if leftOnPiece > CatchCombinePipeTask.PREPLAN_MIN_LEFT then
            -- planned toward where the harvester was: plan again from the same end of piece
            return
        end
        self.routeIsPartial = false
        return
    end
    local path = pf:getPath()
    if path == nil or #path < 2 then
        self.routeIsPartial = false
        return
    end
    -- (seen in game: 51 pieces appended in 90 s without moving, lane and grid in turn): no more than
    -- PREPLAN_MAX_STANDING pieces while the tractor stays about where it is, then it drives this piece to its end
    local anchor = self.preplanAnchor
    if anchor == nil or MathUtil.vector2Length(vx - anchor.x, vz - anchor.z) > CatchCombinePipeTask.PREPLAN_STANDING_RADIUS then
        self.preplanAnchor = {x = vx, z = vz, count = 0}
        anchor = self.preplanAnchor
    end
    anchor.count = anchor.count + 1
    if anchor.count > CatchCombinePipeTask.PREPLAN_MAX_STANDING then
        self.routeIsPartial = false
        return
    end
    -- (seen in game: pieces up to 135 m from the harvester): a partial next piece that ends farther from the
    -- harvester than the end of this piece leads away, it is not appended
    if pf.partialPath then
        local pEnd = path[#path]
        local dStart = MathUtil.vector2Length(last.x - cx, last.z - cz)
        local dEnd = MathUtil.vector2Length(pEnd.x - cx, pEnd.z - cz)
        if dEnd > dStart - 1 then
            self.routeIsPartial = false
            return
        end
    end
    -- what is left of this piece, then the next piece
    local newWps = {{x = vx, y = vy, z = vz}}
    local index = dpm:getCurrentWayPointIndex() or 2
    for i = math.max(index, 2), #wps do
        newWps[#newWps + 1] = wps[i]
    end
    for i = 1, #path do
        local p = path[i]
        local tail = newWps[#newWps]
        if MathUtil.vector2Length(p.x - tail.x, p.z - tail.z) > 0.5 then
            newWps[#newWps + 1] = p
        end
    end
    self.wayPoints = newWps
    dpm:setWayPoints(newWps)
    self.routeIsPartial = pf.partialPath == true
    self.newPathFindingCounter = 0
end

-- forage harvester: a failed plan from the same start with the harvester at the same place gives the same result,
-- the next try waits twice as long (up to RETRY_DELAY_MAX); anything moved: back to RETRY_DELAY_MIN
function CatchCombinePipeTask:updateRetryDelay()
    if not AutoDrive.isChopperUnloading(self.vehicle) then
        return
    end
    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
    local cx, _, cz = getWorldTranslation(self.combine.components[1].node)
    local f = self.lastFailure
    if f ~= nil and MathUtil.vector2Length(vx - f.vx, vz - f.vz) < CatchCombinePipeTask.RETRY_SAME_VEHICLE
        and MathUtil.vector2Length(cx - f.cx, cz - f.cz) < CatchCombinePipeTask.RETRY_SAME_COMBINE then
        self.retryDelay = math.min(CatchCombinePipeTask.RETRY_DELAY_MAX, (self.retryDelay or CatchCombinePipeTask.RETRY_DELAY_MIN) * 2)
    else
        self.retryDelay = CatchCombinePipeTask.RETRY_DELAY_MIN
    end
    self.lastFailure = {vx = vx, vz = vz, cx = cx, cz = cz}
end

-- harvester heading once per second: remembers when it last turned (headland turn)
function CatchCombinePipeTask:sampleCombineHeading()
    if self.combine == nil or g_time == nil then
        return
    end
    local last = self.combineHeading
    if last ~= nil and g_time - last.t < 1000 then
        return
    end
    local dx, _, dz = AutoDrive.localDirectionToWorld(self.combine, 0, 0, 1)
    local yaw = math.deg(math.atan2(dx, dz))
    if last ~= nil then
        local d = math.abs(yaw - last.yaw) % 360
        if d > 180 then
            d = 360 - d
        end
        if d / ((g_time - last.t) / 1000) > CatchCombinePipeTask.BAND_TURN_YAW_RATE then
            self.combineTurnedAt = g_time
        end
    end
    self.combineHeading = {yaw = yaw, t = g_time}
end

-- forage harvester, fallback only (no forward route): U-turn in the strip when its heading is behind ours
function CatchCombinePipeTask:tryBandTurn(why)
    if not AutoDrive.isChopperUnloading(self.vehicle) then
        return false
    end
    if not ADBandTurn.canTry(self.vehicle) then
        return false
    end
    local distance = AutoDrive.getDistanceBetween(self.vehicle, self.combine)
    if distance > CatchCombinePipeTask.BAND_TURN_RANGE then
        return false
    end
    -- harvester turning or just turned at the headland: its heading means nothing yet, it comes back, wait
    -- (the AutoDrive turn flag alone is also raised while it stands waiting for a trailer, its turn points decide)
    if self:isCombineReallyTurning() or (self.combineTurnedAt ~= nil and g_time - self.combineTurnedAt < CatchCombinePipeTask.BAND_TURN_AFTER_TURN) then
        if self.bandTurnTurnLogAt == nil or g_time - self.bandTurnTurnLogAt > 5000 then
            self.bandTurnTurnLogAt = g_time
        end
        return false
    end
    -- a partial route was found and only waits for a clean stop: not stuck, the next plan goes on
    local pfm = self.vehicle.ad.pathFinderModule
    if pfm.partialPath and (pfm.partialFailCount or 0) > 0 and pfm.partialFailCount < PathFinderModule.PARTIAL_FACING_LIFT_FAILURES then
        return false
    end
    -- behind: its heading reversed against ours, or the harvester itself behind us (the route must turn around)
    local behind, angle = ADBandTurn.isFacingHarvester(self.vehicle, self.combine, CatchCombinePipeTask.BAND_TURN_RANGE, CatchCombinePipeTask.BAND_TURN_ANGLE)
    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
    local cx, _, cz = getWorldTranslation(self.combine.components[1].node)
    local dx, _, dz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local bearing = math.abs(AutoDrive.angleBetween({x = dx, z = dz}, {x = cx - vx, z = cz - vz}))
    if not behind and (bearing < CatchCombinePipeTask.BAND_TURN_ANGLE or distance > CatchCombinePipeTask.BAND_TURN_BEHIND_RANGE) then
        return false
    end
    if not ADBandTurn.start(self.vehicle, string.format("fallback, %s: harvester heading %.0f deg, harvester at %.0f deg, %.0f m", why, angle or -1, bearing, distance), self.combine) then
        return false
    end
    self.vehicle.ad.pathFinderModule:reset()
    self.preplanning = false
    self.infoEndRowTurn = false -- status text
    self.state = CatchCombinePipeTask.STATE_BAND_TURN
    return true
end

CatchCombinePipeTask.TRAIL_BEARING = 60      -- deg: the harvester ahead of the tractor within this angle
CatchCombinePipeTask.TRAIL_RETRY_DISTANCE = 5
CatchCombinePipeTask.TRAIL_RETRY_TIME = 30000

-- forage harvester: no route, but the tractor stands on the harvester's trail with the harvester ahead
-- (17 m behind it, aligned, target 21 m behind it is behind the tractor): drive its trail up to just behind it
function CatchCombinePipeTask:tryCombineTrail()
    local mode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
    if not AutoDrive.isChopperUnloading(self.vehicle) or self.combine == nil or mode == nil or mode.getCombineTrailFrom == nil then
        return false
    end
    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
    if self.trailTried ~= nil and MathUtil.vector2Length(vx - self.trailTried.x, vz - self.trailTried.z) < CatchCombinePipeTask.TRAIL_RETRY_DISTANCE
        and g_time - self.trailTried.time < CatchCombinePipeTask.TRAIL_RETRY_TIME then
        return false
    end
    self.trailTried = {x = vx, z = vz, time = g_time}
    local cx, _, cz = getWorldTranslation(self.combine.components[1].node)
    local dx, _, dz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local bearing = math.abs(AutoDrive.angleBetween({x = dx, z = dz}, {x = cx - vx, z = cz - vz}))
    if bearing > CatchCombinePipeTask.TRAIL_BEARING then
        return false
    end
    local combineRear = (self.combine.size ~= nil and self.combine.size.length or 10) / 2 - (self.combine.size ~= nil and self.combine.size.lengthOffset or 0)
    local points, why = mode:getCombineTrailFrom(vx, vz, HandleHarvesterTurnTask.TRAIL_START_DISTANCE, combineRear + HandleHarvesterTurnTask.TRAIL_KEEP_BEHIND_CLOSE)
    if points == nil then
        return false
    end
    local ok, groundWhy = HandleHarvesterTurnTask.checkGround({vehicle = self.vehicle}, points)
    if not ok then
        return false
    end
    self.vehicle.ad.pathFinderModule:reset()
    self.wayPoints = points
    self.vehicle.ad.drivePathModule:setWayPoints(points)
    self.combinesStartLocation = {x = cx, z = cz}
    self.routeIsPartial = false
    self.preplanning = false
    self.infoNoPath = false
    self.infoOnTrail = true -- status text, until the next plan
    self.state = CatchCombinePipeTask.STATE_DRIVING
    return true
end

CatchCombinePipeTask.REVERSED_ANGLE = 100          -- deg against the harvester heading: the wrong way round (70 s looping at 135 deg with 150)
CatchCombinePipeTask.REVERSED_RANGE = 80
CatchCombinePipeTask.REVERSED_RETRY_DISTANCE = 10
CatchCombinePipeTask.REVERSED_RETRY_TIME = 90000

-- Courseplay turn really being driven (its turn course). The AutoDrive flag (getIsCPTurning, "maneuvering") is also
-- raised while the harvester stands waiting for a trailer: read on its turn points when possible
function CatchCombinePipeTask:isCombineReallyTurning()
    if self.combine == nil then
        return false
    end
    local row = CombineUnloaderMode.getCpRowTurn(self.combine)
    if row ~= nil then
        return row.turning == true
    end
    local standing = self.combine.ad ~= nil and self.combine.ad.noMovementTimer ~= nil and self.combine.ad.noMovementTimer.elapsedTime > 3000
    return AutoDrive:getIsCPTurning(self.combine) and not standing
end

-- aside (an unloader beside the standing harvester at 177 deg): beside or in front of it as well, while it stands
function CatchCombinePipeTask:tryReversedUTurn(angleToCombine, aside)
    local mode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
    if not AutoDrive.isChopperUnloading(self.vehicle) or self.combine == nil or mode == nil then
        return false
    end
    local distance = AutoDrive.getDistanceBetween(self.vehicle, self.combine)
    if angleToCombine < CatchCombinePipeTask.REVERSED_ANGLE or distance > CatchCombinePipeTask.REVERSED_RANGE then
        return false
    end
    local behind = self:isBehindChopper()
    local standing = self.combine.ad ~= nil and self.combine.ad.noMovementTimer ~= nil and self.combine.ad.noMovementTimer.elapsedTime > 3000
    if not behind and not (aside and standing) then
        return false
    end
    if self:isCombineReallyTurning() then
        return false
    end
    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
    local last = mode.reversedTurnTried
    if last ~= nil and MathUtil.vector2Length(vx - last.x, vz - last.z) < CatchCombinePipeTask.REVERSED_RETRY_DISTANCE
        and (g_time or 0) - last.t < CatchCombinePipeTask.REVERSED_RETRY_TIME then
        return false
    end
    mode.reversedTurnTried = {x = vx, z = vz, t = g_time or 0}
    self.vehicle.ad.endRowUTurn = g_time or 0
    mode.reversedTurnRequested = true
    self.state = CatchCombinePipeTask.STATE_FINISHED
    return true
end

-- seen in game: an unloader at 101 deg, 22 m from the standing
-- harvester: no U-turn accepts 100-150 deg and each new route is the same useless piece. From the 2nd time at the same
-- place: band turn toward the harvester heading (the 120 deg rule lifted), else back off along the own track to open the
-- angle; and every time more room for the turn in the next route (turnRoomExtra, PathFinderModule.startRearFallback)
CatchCombinePipeTask.WRONG_WAY_RANGE = 45        -- m from the harvester
CatchCombinePipeTask.WRONG_WAY_RADIUS = 6        -- m: arrivals closer than this count as the same place
CatchCombinePipeTask.WRONG_WAY_ROOM_STEP = 8     -- m added to the aimed point setback per arrival at the same place
CatchCombinePipeTask.WRONG_WAY_ROOM_MAX = 32
CatchCombinePipeTask.WRONG_WAY_FORGET = 120000   -- ms
CatchCombinePipeTask.SHORT_ROUTE_LENGTH = 25     -- m: a route this short ...
CatchCombinePipeTask.SHORT_ROUTE_END_ANGLE = 60  -- deg: ... ending this far from the harvester heading cannot hand over
function CatchCombinePipeTask:tryWrongWayTurn(angle)
    local mode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
    if not AutoDrive.isChopperUnloading(self.vehicle) or self.combine == nil or self.combine.components == nil or angle == nil
        or angle < CatchCombinePipeTask.REVERSED_ANGLE then
        return false
    end
    local distance = AutoDrive.getDistanceBetween(self.vehicle, self.combine)
    if distance > CatchCombinePipeTask.WRONG_WAY_RANGE or not self:isCombineStanding() or self:isCombineReallyTurning() then
        return false
    end
    local name = tostring(self.vehicle:getName())
    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
    local now = g_time or 0
    local w = self.vehicle.ad.wrongWay
    if w == nil or MathUtil.vector2Length(vx - w.x, vz - w.z) > CatchCombinePipeTask.WRONG_WAY_RADIUS or now - w.t > CatchCombinePipeTask.WRONG_WAY_FORGET then
        w = {x = vx, z = vz, t = now, n = 0}
        self.vehicle.ad.wrongWay = w
    end
    w.n = w.n + 1
    w.t = now
    self.vehicle.ad.turnRoomExtra = math.min(CatchCombinePipeTask.WRONG_WAY_ROOM_MAX, w.n * CatchCombinePipeTask.WRONG_WAY_ROOM_STEP)
    -- no place behind her (hedge, field border): more room for the aimed point changes nothing, the
    -- turn comes at once
    local pfm = self.vehicle.ad.pathFinderModule
    local noRearPlace = pfm ~= nil and pfm.rearPlaceNone == true
    if w.n < 2 and not noRearPlace then
        return false
    end
    local cx, _, cz = getWorldTranslation(self.combine.components[1].node)
    local dx, _, dz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local side = (cx - vx) * dz - (cz - vz) * dx >= 0 and 1 or -1
    local goal = nil
    if ADBandTurn.headingOfNode ~= nil then
        goal = ADBandTurn.headingOfNode(self.combine.ad.ADRootNode or self.combine.components[1].node)
    end
    if ADBandTurn.canTry(self.vehicle) and ADBandTurn.start(self.vehicle, string.format("the wrong way round near the standing harvester (%.0f deg, %.0f m, %d times here)", angle, distance, w.n), self.combine, side, goal) then
        if self.state == CatchCombinePipeTask.STATE_PATHPLANNING or self.preplanning then
            self.vehicle.ad.pathFinderModule:reset()
            self.preplanning = false
        end
        self.infoEndRowTurn = true -- status text
        self.state = CatchCombinePipeTask.STATE_BAND_TURN
        return true
    end
    -- once per place, as the open back-off (else it backs off again and again at the same place)
    local lastOpen = self.vehicle.ad.openReverseDone
    local openAllowed = lastOpen == nil or (g_time or 0) - lastOpen.t >= CatchCombinePipeTask.OPEN_REVERSE_PAUSE
        or MathUtil.vector2Length(vx - lastOpen.x, vz - lastOpen.z) >= CatchCombinePipeTask.OPEN_REVERSE_RADIUS
    if openAllowed and mode ~= nil and mode.track ~= nil and #mode.track >= 3 and mode.getTrackIndexBehind ~= nil then
        local index, walked = mode:getTrackIndexBehind(CatchCombinePipeTask.OPEN_REVERSE_DISTANCE)
        if index ~= nil and walked ~= nil and walked >= CombineUnloaderMode.TRACK_STOP_DISTANCE + 2 and mode:startTrackReverse(self.combine, nil, index) then
            self.vehicle.ad.openReverseDone = {x = vx, z = vz, t = g_time or 0}
            if self.state == CatchCombinePipeTask.STATE_PATHPLANNING or self.preplanning then
                self.vehicle.ad.pathFinderModule:reset()
                self.preplanning = false
            end
            self.infoReverse = {blocked = false, distance = walked} -- status text
            self.state = CatchCombinePipeTask.STATE_OPEN_REVERSE
            return true
        end
    end
    return false
end

-- a short route that ends far from the harvester heading while the train is the wrong way round: the arrival cannot
-- hand over to the chase, the same route comes again
function CatchCombinePipeTask:isUselessShortRoute(wayPoints)
    if wayPoints == nil or #wayPoints < 2 or not AutoDrive.isChopperUnloading(self.vehicle) or self.combine == nil or self.combine.components == nil then
        return false
    end
    local mode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
    local myAngle = mode:getAngleToCombineHeading()
    local distance = AutoDrive.getDistanceBetween(self.vehicle, self.combine)
    if myAngle < CatchCombinePipeTask.CLOSE_RANGE_ANGLE or distance > CatchCombinePipeTask.WRONG_WAY_RANGE then
        return false
    end
    local length = 0
    for i = 2, #wayPoints do
        length = length + MathUtil.vector2Length(wayPoints[i].x - wayPoints[i - 1].x, wayPoints[i].z - wayPoints[i - 1].z)
    end
    if length > CatchCombinePipeTask.SHORT_ROUTE_LENGTH then
        return false
    end
    local a, b = wayPoints[#wayPoints - 1], wayPoints[#wayPoints]
    local cx, _, cz = AutoDrive.localDirectionToWorld(self.combine, 0, 0, 1, self.combine.ad.ADRootNode)
    local endAngle = math.abs(AutoDrive.angleBetween({x = b.x - a.x, z = b.z - a.z}, {x = cx, z = cz}))
    if endAngle <= CatchCombinePipeTask.SHORT_ROUTE_END_ANGLE then
        return false
    end
    return true
end

-- forage harvester: no route from here several times (nose against the crop, angle too sharp): back off about
-- OPEN_REVERSE_DISTANCE along its own clean track to open the angle, then plan again. Once per place
function CatchCombinePipeTask:tryEndRowUTurn()
    if not AutoDrive.isChopperUnloading(self.vehicle) or self.combine == nil then
        return false
    end
    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
    local cx, _, cz = getWorldTranslation(self.combine.components[1].node)
    local dx, _, dz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local facing, angle
    local ant = self.vehicle.ad.anticipatedRear
    if ant ~= nil and (g_time or 0) - ant.t < PathFinderModule.ANTICIPATED_MAX_AGE then
        -- anticipated: the harvester has not turned yet, the new row (Courseplay course) gives the heading and side
        angle = math.deg(math.acos(math.max(-1, math.min(1, dx * ant.dx + dz * ant.dz))))
        facing = angle >= CatchCombinePipeTask.END_ROW_UTURN_ANGLE
        cx, cz = ant.ex, ant.ez
    else
        facing, angle = ADBandTurn.isFacingHarvester(self.vehicle, self.combine, CatchCombinePipeTask.BAND_TURN_RANGE, CatchCombinePipeTask.END_ROW_UTURN_ANGLE)
    end
    -- (an unloader at 101-105 deg, 19-22 m from the standing harvester, no turn accepted): near a standing
    -- harvester the 120 deg rule is lifted from REVERSED_ANGLE on, the turn then aims the harvester heading itself
    local goalHeading = nil
    if not facing and angle ~= nil and angle >= CatchCombinePipeTask.REVERSED_ANGLE and self:isCombineStanding()
        and AutoDrive.getDistanceBetween(self.vehicle, self.combine) <= CatchCombinePipeTask.WRONG_WAY_RANGE and ADBandTurn.headingOfNode ~= nil then
        facing = true
        if ant ~= nil and (g_time or 0) - ant.t < PathFinderModule.ANTICIPATED_MAX_AGE then
            goalHeading = math.atan2(ant.dx, ant.dz)
        else
            goalHeading = ADBandTurn.headingOfNode(self.combine.ad.ADRootNode or self.combine.components[1].node)
        end
    end
    if not facing then
        return false
    end
    -- harvester (or new row) side: +1 on the right of the tractor, -1 on the left
    local side = (cx - vx) * dz - (cz - vz) * dx >= 0 and 1 or -1
    if not ADBandTurn.start(self.vehicle, string.format("row end, harvester on the %s, harvester heading %.0f deg", side > 0 and "left" or "right", angle or -1), self.combine, side, goalHeading) then
        return false
    end
    if self.state == CatchCombinePipeTask.STATE_PATHPLANNING or self.preplanning then
        self.vehicle.ad.pathFinderModule:reset()
        self.preplanning = false
    end
    self.infoEndRowTurn = true -- status text
    self.state = CatchCombinePipeTask.STATE_BAND_TURN
    return true
end

CatchCombinePipeTask.VEHICLE_BLOCKED_TIME = 10000 -- ms standing in front of a vehicle before backing off
CatchCombinePipeTask.VEHICLE_BLOCKED_REVERSE = 15 -- m backed off along the own tracks

function CatchCombinePipeTask:updateVehicleBlocked(dt, standing)
    local detour = self.vehicle.ad.obstacleDetourModule
    if not standing or detour == nil or detour.isVehicleInFront == nil or not AutoDrive.isChopperUnloading(self.vehicle)
        or self.vehicle.ad.drivePathModule:isTargetReached() then
        self.vehicleBlockedTime = 0
        return false
    end
    self.vehicleBlockedTime = (self.vehicleBlockedTime or 0) + dt
    if self.vehicleBlockedTime < CatchCombinePipeTask.VEHICLE_BLOCKED_TIME then
        return false
    end
    self.vehicleBlockedTime = 0
    if not detour:isVehicleInFront() then
        return false
    end
    local name = tostring(self.vehicle:getName())
    local mode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
    local track = mode.track
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    local last = self.vehicle.ad.openReverseDone
    if last ~= nil and (g_time or 0) - last.t < CatchCombinePipeTask.OPEN_REVERSE_PAUSE
        and MathUtil.vector2Length(x - last.x, z - last.z) < 3 then
        return false
    end
    if track == nil or #track < 3 then
        self.vehicle.ad.openReverseDone = {x = x, z = z, t = g_time or 0}
        return false
    end
    local index, walked = mode:getTrackIndexBehind(CatchCombinePipeTask.VEHICLE_BLOCKED_REVERSE)
    if index == nil or walked < CombineUnloaderMode.TRACK_STOP_DISTANCE + 2 then
        self.vehicle.ad.openReverseDone = {x = x, z = z, t = g_time or 0}
        return false
    end
    self.vehicle.ad.openReverseDone = {x = x, z = z, t = g_time or 0}
    if mode:startTrackReverse(self.combine, nil, index) then
        self.preplanning = false
        self.vehicle.ad.pathFinderModule:reset()
        -- status text
        self.infoReverse = {blocked = true, distance = math.min(walked, CatchCombinePipeTask.VEHICLE_BLOCKED_REVERSE)}
        self.state = CatchCombinePipeTask.STATE_OPEN_REVERSE
        return true
    end
    return false
end

-- no route twice from the same place near the standing harvester (its place off the field at a corner, no grid there,
-- no edge to follow): the chase takes over and its straight line may cut a tip of crop for a while, instead of
-- backing off along the track again and again
function CatchCombinePipeTask:tryCropTipHandover(f)
    if not self:isCombineStanding() or f.cropTipDone then
        return false
    end
    local distance = AutoDrive.getDistanceBetween(self.vehicle, self.combine)
    local angle = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getAngleToCombineHeading()
    if distance > CatchCombinePipeTask.CROP_TIP_DISTANCE or angle >= CatchCombinePipeTask.CROP_TIP_ANGLE then
        return false
    end
    f.cropTipDone = true
    local mode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
    mode.cropTipUntil = (g_time or 0) + CatchCombinePipeTask.CROP_TIP_TIME
    self:noteHandover()
    if self.state == CatchCombinePipeTask.STATE_PATHPLANNING or self.preplanning then
        self.vehicle.ad.pathFinderModule:reset()
        self.preplanning = false
    end
    self.state = CatchCombinePipeTask.STATE_FINISHED
    return true
end

function CatchCombinePipeTask:tryOpenReverse()
    if not AutoDrive.isChopperUnloading(self.vehicle) then
        return false
    end
    local mode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
    local track = mode.track
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    -- once per place: remembered on the vehicle, the place of the failures moves with the back-off itself
    local last = self.vehicle.ad.openReverseDone
    if last ~= nil and (g_time or 0) - last.t < CatchCombinePipeTask.OPEN_REVERSE_PAUSE
        and MathUtil.vector2Length(x - last.x, z - last.z) < CatchCombinePipeTask.OPEN_REVERSE_RADIUS then
        return false
    end
    local f = self.openFails
    if f == nil or MathUtil.vector2Length(x - f.x, z - f.z) > 2 then
        self.openFails = {x = x, z = z, count = 1}
        return false
    end
    f.count = f.count + 1
    if f.count < CatchCombinePipeTask.OPEN_REVERSE_FAILS or f.done then
        return false
    end
    if self:tryCropTipHandover(f) then
        return true
    end
    if track == nil or #track < 3 then
        return false
    end
    -- track point about OPEN_REVERSE_DISTANCE behind the tractor
    -- counted from the track point closest to the tractor (counted from the newest one, ahead after a
    -- reverse, the 10 m back-off was 2 m and ended at once)
    local index, walked = mode:getTrackIndexBehind(CatchCombinePipeTask.OPEN_REVERSE_DISTANCE)
    if index == nil or walked < CombineUnloaderMode.TRACK_STOP_DISTANCE + 2 then
        return false
    end
    f.done = true
    if mode:startTrackReverse(self.combine, nil, index) then
        self.vehicle.ad.openReverseDone = {x = x, z = z, t = g_time or 0}
        self.infoReverse = {blocked = false, distance = walked} -- status text
        self.state = CatchCombinePipeTask.STATE_OPEN_REVERSE
        return true
    end
    return false
end

function CatchCombinePipeTask:getExcludedVehiclesForCollisionCheck()
    local excludedVehicles = {}
    if self.state == CatchCombinePipeTask.STATE_DRIVING then
        table.insert(excludedVehicles, self.combine:getRootVehicle())
    end
    return excludedVehicles
end

function CatchCombinePipeTask:setStateNames()
    if self.statesToNames == nil then
        self.statesToNames = {}
        for name, id in pairs(CatchCombinePipeTask) do
            if string.sub(name, 1, 6) == "STATE_" then
                self.statesToNames[id] = name
            end
        end
    end
end

function CatchCombinePipeTask:getStateName(state)
    local requestedState = state
    if requestedState == nil then
        requestedState = self.state
    end
    if requestedState == nil then
        Logging.error("[AD] CatchCombinePipeTask: Could not find name for state ->%s<- !", tostring(requestedState))
    end
    return self.statesToNames[requestedState] or ""
end

function CatchCombinePipeTask:resetAllTimers()
    -- self.stuckTimer:timer(false) -- stuckTimer reset by speed changes
    self.reverseTimer:timer(false)
    self.waitTimer:timer(false)
    self.waitForCheckTimer:timer(false)
end

function CatchCombinePipeTask:getI18nInfo()
    local harvester = AbstractTask.getHarvesterI18n(self.combine)
    if self.state == CatchCombinePipeTask.STATE_PATHPLANNING then
        return "$l10n_AD_task_plan_to; " .. harvester
    elseif self.state == CatchCombinePipeTask.STATE_DELAY_PATHPLANNING then
        if (self.handoverTries or 0) >= CatchCombinePipeTask.HANDOVER_TRIES
            and AutoDrive.getDistanceBetween(self.vehicle, self.combine) < CatchCombinePipeTask.HANDOVER_DISTANCE then
            return "$l10n_AD_task_near; " .. harvester .. ", $l10n_AD_task_wait_behind;"
        elseif self.infoNoPath then
            local left = math.max(0, (self.retryDelay or CatchCombinePipeTask.RETRY_DELAY_MIN) - (self.waitForCheckTimer.elapsedTime or 0))
            return "$l10n_AD_task_no_path_retry;" .. string.format(" %d s", math.ceil(left / 1000))
        end
        return "$l10n_AD_task_plan_to; " .. harvester
    elseif self.state == CatchCombinePipeTask.STATE_DRIVING then
        if self.infoAnticipatedWaitAt ~= nil and (g_time or 0) - self.infoAnticipatedWaitAt < 1000 then
            return "$l10n_AD_task_wait_row_entry; " .. harvester
        elseif self.infoOnTrail then
            return "$l10n_AD_task_follow_trail; " .. harvester
        elseif self.preplanning then
            return "$l10n_AD_task_drive_to; " .. harvester .. ", $l10n_AD_task_plan_next;"
        end
        return "$l10n_AD_task_drive_to; " .. harvester .. string.format(" (%d m)", math.floor(AutoDrive.getDistanceBetween(self.vehicle, self.combine)))
    elseif self.state == CatchCombinePipeTask.STATE_OPEN_REVERSE then
        local info = self.infoReverse or {distance = CatchCombinePipeTask.OPEN_REVERSE_DISTANCE}
        if info.blocked then
            return "$l10n_AD_task_blocked_reverse;" .. string.format(" %d m", math.floor(info.distance + 0.5))
        end
        return "$l10n_AD_task_reverse_by;" .. string.format(" %d m ", math.floor(info.distance + 0.5)) .. "$l10n_AD_task_tracks_open_angle;"
    elseif self.state == CatchCombinePipeTask.STATE_BAND_TURN then
        if self.infoEndRowTurn then
            return "$l10n_AD_task_end_row_turn;"
        end
        return "$l10n_AD_task_band_turn_rescue;"
    elseif self.state == CatchCombinePipeTask.STATE_CLEAR_WAY then
        return "$l10n_AD_task_clear_way_for; " .. harvester
    elseif self.state == CatchCombinePipeTask.STATE_WAIT_PASS_BY then
        return "$l10n_AD_task_let_pass; " .. harvester
    elseif self.state == CatchCombinePipeTask.STATE_REVERSING then
        return "$l10n_AD_task_stuck_reverse;"
    elseif self.state == CatchCombinePipeTask.STATE_WAIT_BEFORE_FINISH then
        return "$l10n_AD_task_waiting_for_room;"
    end
    return "$l10n_AD_task_drive_to; " .. harvester
end

function CatchCombinePipeTask.debugMsg(vehicle, debugText, ...)
    if CatchCombinePipeTask.debug == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    else
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_COMBINEINFO, debugText, ...)
    end
end
