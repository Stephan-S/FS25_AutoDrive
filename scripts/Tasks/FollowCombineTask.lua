FollowCombineTask = ADInheritsFrom(AbstractTask)
FollowCombineTask.debug = false

FollowCombineTask.STATE_CHASING = {}
FollowCombineTask.STATE_WAIT_FOR_TURN = {}
FollowCombineTask.STATE_TRACK_REVERSE = {} -- forage harvester: backing off along its own track, out of its way
FollowCombineTask.STATE_BACK_TO_PLACE = {} -- forage harvester standing, its place behind the tractor: a few metres back
FollowCombineTask.STATE_REVERSING = {}
FollowCombineTask.STATE_REVERSING_FROM_CHOPPER = {}
FollowCombineTask.STATE_WAIT_FOR_PASS_BY = {}
FollowCombineTask.STATE_FINISHED = {}
FollowCombineTask.STATE_WAIT_BEFORE_FINISH = {}
FollowCombineTask.STATE_WAIT_FOR_COMBINE_TO_PASS_BY = {}
FollowCombineTask.STATE_GENERATE_UTURN_PATH = {}
FollowCombineTask.STATE_DRIVE_UTURN_PATH = {}

FollowCombineTask.MAX_REVERSE_DISTANCE = 20
FollowCombineTask.MIN_COMBINE_DISTANCE = 25
FollowCombineTask.MAX_REVERSE_TIME = 30000
FollowCombineTask.MAX_TURN_TIME = 60000
FollowCombineTask.MAX_STUCK_TIME = 60000
FollowCombineTask.WAIT_BEFORE_FINISH_TIME = 8000
-- straight line to the chase point: checked every LINE_CHECK_INTERVAL, the chase is given up after LINE_BLOCKED_TIME blocked
FollowCombineTask.LINE_CHECK_INTERVAL = 500
FollowCombineTask.LINE_BLOCKED_TIME = 2000
-- checked from the vehicle itself: the crop just in front of the hood counts
FollowCombineTask.LINE_SKIP_START = 0
FollowCombineTask.LINE_SKIP_END = 3
FollowCombineTask.STRICT_LINE_MIN_LENGTH = 5 -- m checked ahead even when the chase point is closer (strict)
FollowCombineTask.STRICT_LINE_FRUIT_RATIO = 0.02 -- strict: almost no crop allowed on the chase line
FollowCombineTask.LINE_WIDTH_MARGIN = 0.5
-- combine turn: the unloader waits until the combine is aligned again and drives forward, a stop alone does not end the turn
FollowCombineTask.TURN_END_FORWARD_TIME = 3000
FollowCombineTask.TURN_END_MAX_HEADING_CHANGE = 5
FollowCombineTask.TURN_HEADING_SAMPLE_TIME = 2000
FollowCombineTask.TRACK_REVERSE_DISTANCE = 80 -- m backed off along its own track at most (forage harvester)
FollowCombineTask.TURN_STOPPED_RELEASE_TIME = 8000 -- ms standing still: the harvester waits for the trailer, no turn
FollowCombineTask.TURN_STOPPED_SPEED = 0.0002 -- lastSpeedReal (m/ms) below which the combine counts as stopped
-- forage harvester (Courseplay) turning toward the unloader's side: the unloader takes the 3/4 rear place on its side
FollowCombineTask.INNER_TURN_MIN_DEG = 3 -- heading change toward us that starts it (with the Courseplay turn flag)
FollowCombineTask.INNER_TURN_REAR_RATIO = 0.75 -- the place moves back by this part of the harvester length
FollowCombineTask.INNER_TURN_REVERSE_TOLERANCE = 1.5 -- m behind the tractor before it backs up
FollowCombineTask.INNER_TURN_REVERSE_SPEED = 8 -- km/h
FollowCombineTask.INNER_TURN_REAR_CHECK = 4 -- m behind the train free of crop to back up
FollowCombineTask.INNER_TURN_MAX_TIME = 60000

function FollowCombineTask:new(vehicle, combine)
    local o = FollowCombineTask:create()
    o.vehicle = vehicle
    o.combine = combine
    o.state = FollowCombineTask.STATE_CHASING
    o.reverseStartLocation = nil
    o.angleWrongTimer = AutoDriveTON:new()
    o.waitForTurnTimer = AutoDriveTON:new()
    o.stuckTimer = AutoDriveTON:new()
    o.dischargeTimer = AutoDriveTON:new()
    o.fillingTimer = AutoDriveTON:new()
    o.lastChaseSide = -10
    o.waitForPassByTimer = AutoDriveTON:new()
    o.chaseTimer = AutoDriveTON:new()
    o.startedChasing = false
    o.reverseTimer = AutoDriveTON:new()
    o.waitTimer = AutoDriveTON:new()
    o.chasePos, o.chaseSide = vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getPipeChasePosition()
    o.angleToCombineHeading = vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getAngleToCombineHeading()
    o.angleToCombine = vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getAngleToCombine()
    o.trailers = nil
    o.lineCheckTimer = 0
    o.lineClear = true
    o.lineBlockedTimer = AutoDriveTON:new()
    o.lineBlocked = false
    o.combineInTurn = false
    o.turnHeadingSamples = {}
    o.turnHeadingSampleTimer = 0
    o.turnFlagIgnored = false
    o.combineRootVehicle = o.combine:getRootVehicle()
    o.activeUnloading = AutoDrive.getSetting("activeUnloading", o.combineRootVehicle)
    FollowCombineTask.setStateNames(o)
    return o
end

function FollowCombineTask:setUp()
    -- chasing again: the anticipated U-turn place is used up
    self.vehicle.ad.anticipatedRear = nil
    self.vehicle.ad.cpTurnFlagIgnored = nil
    FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask setUp")
    self.lastChaseSide = self.chaseSide
    self.trailers, _ = AutoDrive.getAllUnits(self.vehicle)
    self.activeUnloading = AutoDrive.getSetting("activeUnloading", self.combineRootVehicle)
    AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, true)
end

function FollowCombineTask:update(dt)
    if self.combine == nil or g_currentMission.nodeToObject[self.combine.components[1].node] == nil then
        self:finished()
        return
    end

    self:updateStates(dt)
    self:updateCombineTurn(dt)
    self:updateInnerTurn(dt)

    if self.lastState ~= self.state then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update %s -> %s", tostring(self:getStateName(self.lastState)), tostring(self:getStateName()))
        self.lastState = self.state
        self:resetAllTimers()
    end

    -- end-of-row U-turn of a forage harvester: stop and wait until it has turned, no 3/4, no reversing,
    -- then the repositioning (with its own U-turn) or the chase
    if self.state == FollowCombineTask.STATE_FINISHED and self.vehicle.ad.endRowUTurn ~= nil then
        -- end of row handed over (U-turn, anticipated place): finish now, the chase code below must not run
        self:finished()
        return
    end
    if self:updateEndRowTurn(dt) then
        return
    end

    local checkForStuck = (self.vehicle.lastSpeedReal <= 0.0002) and (
        self.state == FollowCombineTask.STATE_CHASING
        or self.state == FollowCombineTask.STATE_WAIT_FOR_TURN
        or self.state == FollowCombineTask.STATE_WAIT_FOR_COMBINE_TO_PASS_BY
        or self.state == FollowCombineTask.STATE_GENERATE_UTURN_PATH
        or self.state == FollowCombineTask.STATE_DRIVE_UTURN_PATH
    )

    self.stuckTimer:timer(checkForStuck, self.MAX_STUCK_TIME, dt)
    if self.stuckTimer:done()
        -- or AutoDrive.getDistanceBetween(self.vehicle, self.combine) < self.MIN_COMBINE_DISTANCE 
        then
        -- got stuck or to close to combine -> reverse
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update - STATE_DRIVING stuck")
        local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
        self.reverseStartLocation = {x = x, y = y, z = z}
        if self.combine.ad.isChopper then
            self.state = FollowCombineTask.STATE_REVERSING_FROM_CHOPPER
        else
            self.state = FollowCombineTask.STATE_REVERSING -- reverse to get room from harvester
        end
    end

    if self.innerTurn and (self.state == FollowCombineTask.STATE_CHASING or self.state == FollowCombineTask.STATE_WAIT_FOR_TURN) then
        self:driveInnerTurn(dt)
        return
    end

    if self.state == FollowCombineTask.STATE_CHASING then
        self.chaseTimer:timer(true, 4000, dt)

        if self.combine.ad.isChopper then
            if self.filled and self.chaseSide ~= nil and self.chaseSide ~= AutoDrive.CHASEPOS_REAR then
                --skip reversing
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_CHASING - filled chopper")
                self.state = FollowCombineTask.STATE_FINISHED -- finish immediate
                return
            elseif self.filled and self.chaseSide ~= nil and self.chaseSide == AutoDrive.CHASEPOS_REAR then
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_CHASING CHASEPOS_REAR - filled chopper")
                local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
                self.reverseStartLocation = {x = x, y = y, z = z}
                self.state = FollowCombineTask.STATE_REVERSING -- reverse to get room from harvester
                return
            end
        elseif self.filled or (self.combine.ad.isHarvester and self.combineFillPercent <= 0.1 and (not self.activeUnloading)) then
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_CHASING - filled harvester")
            self.state = FollowCombineTask.STATE_WAIT_BEFORE_FINISH -- unload after some time to let harvester drive away
            return
        end

        local wrongChopperHeading = self.combine.ad.isChopper and (self.angleToCombineHeading > 90 and self.distanceToCombine < 30)
        if wrongChopperHeading and not self:isChopperComingAtUs() then
            -- facing
            -- the standing harvester the chase can drive nowhere, an unloader looped 4 min (chase, reverse, route, chase...):
            -- its own U-turn first, the chase after
            if self:turnBeforeChase() then
                return
            end
            -- forage harvester stopped (waiting for its unloader) or turning elsewhere at a corner: it is not coming,
            -- backing off would only make both wait for each other
            wrongChopperHeading = false
        end

        -- going behind the harvester for a jog, the place is behind us and we stand on purpose
        -- while it drives past: that is no stuck chase, no reverse in our tracks
        local mode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
        -- whatever its speed (slowed under 2 km/h, the old timer backed off 16 m)
        local waitingPassForJog = self.combine.ad.isChopper and mode ~= nil and mode:isRearChaseHeld() and not wrongChopperHeading
        if waitingPassForJog then
            self.infoJogWaitAt = g_time or 0 -- status text
        end
        -- rolling beside the harvester until the place comes ahead: moving, not stuck
        if ((self.angleWrongTimer.elapsedTime > 15000) and not waitingPassForJog and not self.rollingNow) or wrongChopperHeading then
            -- if stuck with harvester - try reverse
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_CHASING - stuck -> stuckTimer:%s angleWrongTimer:%s"
            , tostring(self.stuckTimer:done()), tostring(self.angleWrongTimer.elapsedTime > 15000))
            local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
            self.reverseStartLocation = {x = x, y = y, z = z}
            if self.combine.ad.isChopper then
                self.state = FollowCombineTask.STATE_REVERSING_FROM_CHOPPER
                return
            else
                self.state = FollowCombineTask.STATE_REVERSING -- reverse to get room from harvester
                return
            end
        end

        if not self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:isUnloaderOnCorrectSide() then
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_CHASING - not UnloaderOnCorrectSide -> finished")
            self:finished()
            return
        end

        if self.combine.ad.isHarvester and ((self.combineFillPercent > 90 and AutoDrive.getDistanceBetween(self.vehicle, self.combine) < self.MIN_COMBINE_DISTANCE) -- if to close -> reverse
            or AutoDrive:getIsCPTurning(self.combine))
            then
            -- Stop chasing and wait for a normal unload call while standing
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_CHASING - to close to harvester -> reverse")
            local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
            self.reverseStartLocation = {x = x, y = y, z = z}
            self.state = FollowCombineTask.STATE_REVERSING -- reverse to get room from harvester
            return
        end

        if (not self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:isUnloaderOnCorrectSide(self.chaseSide)) and (not self.combineInTurn) then
            if self.lastChaseSide ~= CombineUnloaderMode.CHASEPOS_REAR then
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_CHASING - switching chase side from side to elsewhere - let's wait for passby next")
                self.state = FollowCombineTask.STATE_WAIT_FOR_PASS_BY
                return
            end
        end

        local movingDirection = self.combine.movingDirection
        if self.combine.ad.isReverseAttached then
            movingDirection = -self.combine.movingDirection
        end
        if self.combineInTurn then
            -- harvester turns
            --print("Waiting for turn now - 1- t:" ..  tostring(AutoDrive.combineIsTurning(self.combine)) .. " anglewrongtimer: " .. tostring(self.angleWrongTimer.elapsedTime > 10000))      
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_CHASING - combineIsTurning")
            self.state = FollowCombineTask.STATE_WAIT_FOR_TURN
            return
        elseif ((self.combine.lastSpeedReal *  movingDirection) <= -0.00005) then
            self.vehicle.ad.specialDrivingModule:driveReverse(dt, self.combine.lastSpeedReal * 3600 * 1.3, 1, self.vehicle.ad.trailerModule:canBeHandledInReverse())
        else
            self:followChasePoint(dt)
        end
    elseif self.state == FollowCombineTask.STATE_WAIT_FOR_TURN then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN")
        self.waitForTurnTimer:timer(true, self.MAX_TURN_TIME, dt)
        if self.waitForTurnTimer:done() then
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - combine turn took to long - set finished now")
            self.state = FollowCombineTask.STATE_FINISHED
            return
        end

        if self.combineInTurn then
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - combineIsTurning")
            if self.combine.ad.isHarvester and (self.distanceToCombine < ((self.vehicle.size.length + self.combine.size.length) / 2 + 10))
                and FollowCombineTask.isCombineReversing(self.combine) then
                -- harvester
                -- if combine drive reverse to turn -> reverse to keep distance
                self:reverse(dt)
            elseif self.combine.ad.isChopper and AutoDrive:getIsCPActive(self.combine) then
                -- CP chopper turn
                if self.combine.ad.isAutoAimingChopper then
                    local movingDirection = self.combine.movingDirection
                    if self.combine.ad.isReverseAttached then
                        movingDirection = -self.combine.movingDirection
                    end
                    local isdrivingReverse = ((self.combine.lastSpeedReal * movingDirection) <= -0.00051) 
                    local combineIsDriving = (self.combine.lastSpeedReal > 0.001) 

                    -- (S bend at the pole): a jog or an island makes the
                    -- trailer follow behind the harvester; CP calls the bend a turn, but this is no U-turn: keep following
                    local unloadMode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
                    local rearHeld = unloadMode ~= nil and unloadMode.isRearChaseHeld ~= nil and unloadMode:isRearChaseHeld()
                    -- a Courseplay turn that is no end-of-row U-turn on its turn points, nor a
                    -- headland corner (loop or jog around a power pole, row finish), is followed behind as well
                    local cpRow = CombineUnloaderMode.getCpRowTurn(self.combine)
                    local cpNoUTurn = cpRow ~= nil and cpRow.turning and not cpRow.uTurn and not cpRow.headland
                        and (cpRow.deg ~= nil or (cpRow.endIx ~= nil and cpRow.startIx ~= nil and cpRow.endIx <= cpRow.startIx))
                    rearHeld = rearHeld or cpNoUTurn
                    -- and the harvester standing there waits for this trailer: its place ahead is driven to
                    local _, _, placeAhead = AutoDrive.worldToLocal(self.vehicle, self.chasePos.x, self.chasePos.y, self.chasePos.z)
                    if isdrivingReverse then
                        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN -> self:reverse")
                        self:reverse(dt)
                    elseif rearHeld and AutoDrive.isChopperUnloading(self.vehicle) then
                        self:followChasePoint(dt)
                    elseif not combineIsDriving and placeAhead > 0.5 and AutoDrive.isChopperUnloading(self.vehicle) then
                        self:followChasePoint(dt)
                    elseif not combineIsDriving and self:getAngleToCobine() > 45 and AutoDrive.isChopperUnloading(self.vehicle) then
                        -- the harvester stands and waits for the trailer, backing off only put the
                        -- train nose in the crop: stand still too (the end of the turn wait sees it waiting)
                        self.vehicle.ad.specialDrivingModule:stopVehicle()
                        self.vehicle.ad.specialDrivingModule:update(dt)
                    elseif (not combineIsDriving and (self:getAngleToCobine() > 45)) then
                        -- if stuck with harvester - try reverse
                        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - stuck / getAngleToCobine() > 45 -> STATE_REVERSING_FROM_CHOPPER combineIsDriving %s", tostring(combineIsDriving))
                        local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
                        self.reverseStartLocation = {x = x, y = y, z = z}
                        self.state = FollowCombineTask.STATE_REVERSING_FROM_CHOPPER -- reverse to get room from harvester
                        return
                    elseif combineIsDriving then
                        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - combineIsDriving -> stopVehicle")
                        self.vehicle.ad.specialDrivingModule:stopVehicle()
                        self.vehicle.ad.specialDrivingModule:update(dt)
                    else
                        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN -> followChasePoint self.chaseSide %s", tostring(self.chaseSide))
                        self:followChasePoint(dt)
                    end
                else
                    -- isFixedPipeChopper
                    FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - noMovementTimer %d", self.combine.ad.noMovementTimer.elapsedTime)
                    local dischargeState = self.combine:getDischargeState()
                    self.fillingTimer:timer(not self.combine.spec_combine.isFilling, 100, dt)
                    if self.fillingTimer:done() and self.combine.ad.noMovementTimer.elapsedTime < 5000 then
                        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - fillingTimer:done")
                        -- harvested to end of row
                        AutoDrive:holdCPCombine(self.combine)
                        self.vehicle.ad.specialDrivingModule:stopVehicle()
                        self.vehicle.ad.specialDrivingModule:update(dt)
                    else
                        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN -> followChasePoint no AutoAimingChopper")
                        self:followChasePoint(dt)
                    end
                    self.dischargeTimer:timer(dischargeState ~= Dischargeable.DISCHARGE_STATE_OBJECT , 500, dt)
                    if self.dischargeTimer:done() and self.fillingTimer:done() and self.combine.ad.noMovementTimer.elapsedTime < 5000 then
                        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - dischargeTimer:done")
                        local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
                        self.reverseStartLocation = {x = x, y = y, z = z}
                        self.state = FollowCombineTask.STATE_REVERSING_FROM_CHOPPER -- reverse to get room from harvester
                        return
                    end
                end
            else
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN -> stopVehicle")
                -- stop while combine is turning
                self.vehicle.ad.specialDrivingModule:stopVehicle()
                self.vehicle.ad.specialDrivingModule:update(dt)
            end
        end

        -- check if we could continue
        local combineSensors = self.combine.ad.sensors or self.combineRootVehicle.ad.sensors
        -- the turn ends only when the combine is aligned again and drives forward (no more 15 s escape)
        if not self.combineInTurn and
            (
                combineSensors.frontSensorFruit:pollInfo() or
                self.combine.ad.isChopper or
                self.combine.ad.driveForwardTimer.elapsedTime > 8000
            ) then
            if (self.angleToCombineHeading + self.angleToCombine) < 180 and self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:isUnloaderOnCorrectSide(self.chaseSide) then
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - combine turn finished - Heading looks good - start chasing again")
                self.state = FollowCombineTask.STATE_CHASING
                return
            elseif self.angleToCombineHeading > 150 and self.angleToCombineHeading < 210 and self.distanceToCombine < 80 and self.combine.ad.isHarvester then
                -- Instead of directly trying a long way around to get behind the harvester, let's wait for him to pass us by and then U-turn
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - combine turn finished - Heading inverted - wait for passby, then U-turn")
                self.state = FollowCombineTask.STATE_WAIT_FOR_COMBINE_TO_PASS_BY
                return
            else
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - combine turn finished - Heading looks bad - stop to be able to start pathfinder")
                self.stayOnField = true
                self.state = FollowCombineTask.STATE_FINISHED
                return
            end
        end
    elseif self.state == FollowCombineTask.STATE_WAIT_FOR_PASS_BY then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_PASS_BY")
        self.waitForPassByTimer:timer(true, 2200, dt)
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        if self.waitForPassByTimer:done() then
            if (self.angleToCombineHeading + self.angleToCombine) < 180 and self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:isUnloaderOnCorrectSide() then
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_PASS_BY - passby timer elapsed - heading looks good - chasing again")
                self.state = FollowCombineTask.STATE_CHASING
                return
            else
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_PASS_BY - passby timer elapsed - heading looks bad - set finished now")
                self.stayOnField = true
                self.state = FollowCombineTask.STATE_WAIT_BEFORE_FINISH
                return
            end
        end
    elseif self.state == FollowCombineTask.STATE_BACK_TO_PLACE then
        self:updateBackToPlace(dt)
        return
    elseif self.state == FollowCombineTask.STATE_REVERSING then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_REVERSING")
        local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
        local distanceToReverseStart = MathUtil.vector2Length(x - self.reverseStartLocation.x, z - self.reverseStartLocation.z)
        self.reverseTimer:timer(true, self.MAX_REVERSE_TIME, dt)
        local doneReversing = distanceToReverseStart > self.MAX_REVERSE_DISTANCE or (not self.startedChasing)
        if doneReversing or self.reverseTimer:done() then
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_REVERSING - done reversing - set finished")
            self.state = FollowCombineTask.STATE_WAIT_BEFORE_FINISH
            return
        else
            self:reverse(dt)
        end
    elseif self.state == FollowCombineTask.STATE_TRACK_REVERSE then
        local mode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
        local result = mode:updateTrackReverse(dt)
        if result ~= "running" then
            -- (seen in game: 8 times without moving a metre): nothing to gain by
            -- waiting here facing the standing harvester, its own U-turn forward instead
            if result == "blocked" and (mode.trackReverseMoved or 0) < 2 and self:turnBeforeChase("reverse impossible") then
                return
            end
            -- the harvester stands and waits for this trailer, its place now well ahead: the chase
            -- goes there at once instead of 8 s of waiting and a new call
            if self.chasePos ~= nil and math.abs(self.combine.lastSpeedReal or 0) <= FollowCombineTask.TURN_STOPPED_SPEED then
                local _, _, ahead = AutoDrive.worldToLocal(self.vehicle, self.chasePos.x, self.chasePos.y or 0, self.chasePos.z)
                if ahead >= FollowCombineTask.BACK_AHEAD then
                    self.state = FollowCombineTask.STATE_CHASING
                    return
                end
            end
            -- out of its way (or as far as possible): wait, the harvester passes, then a new call
            self.stayOnField = true
            self.state = FollowCombineTask.STATE_WAIT_BEFORE_FINISH
        end
        return
    elseif self.state == FollowCombineTask.STATE_REVERSING_FROM_CHOPPER and AutoDrive.isChopperUnloading(self.vehicle) and not self.trackReverseTried then
        -- forage harvester: back off along its own track (crop free) to a place out of its way, not straight back
        self.trackReverseTried = true
        if self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:startTrackReverse(self.combine, FollowCombineTask.TRACK_REVERSE_DISTANCE) then
            self.state = FollowCombineTask.STATE_TRACK_REVERSE
        end
        return
    elseif self.state == FollowCombineTask.STATE_REVERSING_FROM_CHOPPER then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_REVERSING_FROM_CHOPPER")
        local cx, _, cz = getWorldTranslation(self.combine.components[1].node)
        local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
        local distanceToReverseStart = MathUtil.vector2Length(vx - self.reverseStartLocation.x, vz - self.reverseStartLocation.z)
        local distanceToCombine = MathUtil.vector2Length(cx - vx, cz - vz)
        self.reverseTimer:timer(true, self.MAX_REVERSE_TIME, dt)
        local doneReversing = distanceToReverseStart > self.MAX_REVERSE_DISTANCE or distanceToCombine > self.MAX_REVERSE_DISTANCE
        if doneReversing or self.reverseTimer:done() then
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_REVERSING_FROM_CHOPPER - done reversing - set finished")
            if self.combine.ad.isFixedPipeChopper and AutoDrive:getIsCPActive(self.combine) then
                -- wait for CP to finish a turn maneuver before invoke pathfinder
                self.state = FollowCombineTask.STATE_WAIT_BEFORE_FINISH
                return
            else
                self.state = FollowCombineTask.STATE_FINISHED
                return
            end
        else
            if self.combine.ad.isFixedPipeChopper then
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_REVERSING_FROM_CHOPPER - not AutoAimingChopper -> holdCPCombine")
                AutoDrive:holdCPCombine(self.combine)
            else
                if self:getAngleToCobine() > 30 then
                    FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_REVERSING_FROM_CHOPPER - AngleToCobine > 30 -> holdCPCombine")
                    AutoDrive:holdCPCombine(self.combine)
                end
            end
            self:reverse(dt)
        end
    elseif self.state == FollowCombineTask.STATE_WAIT_BEFORE_FINISH then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_BEFORE_FINISH")
        -- wait for CP to finish a turn maneuver before invoke pathfinder
        -- TODO: check if following is useful!
        local combineIsDriving = self.combine.ad.isFixedPipeChopper and AutoDrive:getIsCPActive(self.combine) and (self.combine.lastSpeedReal > 0.001)
        self.waitTimer:timer(not combineIsDriving, self.WAIT_BEFORE_FINISH_TIME, dt)
        if self.waitTimer:done() then
            self.state = FollowCombineTask.STATE_FINISHED
            return
        else
            self.vehicle.ad.specialDrivingModule:stopVehicle()
            self.vehicle.ad.specialDrivingModule:update(dt)
        end
    elseif self.state == FollowCombineTask.STATE_WAIT_FOR_COMBINE_TO_PASS_BY then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_COMBINE_TO_PASS_BY")
        self.waitForPassByTimer:timer(true, 15000, dt)
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        if self.waitForPassByTimer:done() then
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_COMBINE_TO_PASS_BY - passby timer elapsed - heading looks bad - set finished now")
            self.stayOnField = true
            self.state = FollowCombineTask.STATE_WAIT_BEFORE_FINISH
            return
        else
            local cx, cy, cz = getWorldTranslation(self.combine.components[1].node)
            local _, _, offsetZ = AutoDrive.worldToLocal(self.vehicle, cx, cy, cz)
            if offsetZ <= -10 then
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_COMBINE_TO_PASS_BY - combine passed us. Calculate U-turn now")
                local cx, cy, cz = getWorldTranslation(self.combine.components[1].node)
                local offsetX, _, _ = AutoDrive.worldToLocal(self.vehicle, cx, cy, cz)
                self.vehicle:generateUTurn(offsetX > 0)
                self.state = FollowCombineTask.STATE_GENERATE_UTURN_PATH
                return
            end
        end
    elseif self.state == FollowCombineTask.STATE_GENERATE_UTURN_PATH then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_GENERATE_UTURN_PATH")
        if self.vehicle.ad.uTurn ~= nil and self.vehicle.ad.uTurn.inProgress then
            self.vehicle:generateUTurn(true)
        elseif self.vehicle.ad.uTurn ~= nil and not self.vehicle.ad.uTurn.inProgress then
            if self.vehicle.ad.uTurn.colliFound or self.vehicle.ad.uTurn.points == nil or #self.vehicle.ad.uTurn.points < 5 then
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_GENERATE_UTURN_PATH - U-Turn generation failed due to collision - set finished now")
                self.stayOnField = true
                self.state = FollowCombineTask.STATE_WAIT_BEFORE_FINISH
                return
            else
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_GENERATE_UTURN_PATH - U-Turn generation finished - passing points to drivePathModule now")
                self.vehicle.ad.drivePathModule:setWayPoints(self.vehicle.ad.uTurn.points)
                self.state = FollowCombineTask.STATE_DRIVE_UTURN_PATH
                return
            end
        end
    elseif self.state == FollowCombineTask.STATE_DRIVE_UTURN_PATH then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_DRIVE_UTURN_PATH")
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_DRIVE_UTURN_PATH - U-Turn finished")
            if (self.angleToCombineHeading + self.angleToCombine) < 180 and self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:isUnloaderOnCorrectSide() then
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_DRIVE_UTURN_PATH - passby timer elapsed - heading looks good - chasing again")
                self.state = FollowCombineTask.STATE_CHASING
                return
            else
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_DRIVE_UTURN_PATH - passby timer elapsed - heading looks bad - set finished now")
                self.stayOnField = true
                self.state = FollowCombineTask.STATE_WAIT_BEFORE_FINISH
                return
            end
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end
    elseif self.state == FollowCombineTask.STATE_FINISHED then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_FINISHED")
        self:finished()
        return
    end
end

function FollowCombineTask:startPathPlanningForCircling()
    local sideOffset = 0
    if self.chaseSide ~= nil and self.chaseSide == AutoDrive.CHASEPOS_LEFT then
        sideOffset = 8
    elseif self.chaseSide ~= nil and self.chaseSide == AutoDrive.CHASEPOS_RIGHT then
        sideOffset = -8
    end

    local targetPos = AutoDrive.createWayPointRelativeToVehicle(self.vehicle, sideOffset, 0)
    local directionX, directionY, directionZ = AutoDrive.localToWorld(self.vehicle, 0, 0, 0, self.vehicle.ad.ADRootNode)
    local direction = {x = directionX - targetPos.x, z = directionZ - targetPos.z}
    self.vehicle.ad.pathFinderModule:reset()
    self.vehicle.ad.pathFinderModule:startPathPlanningTo(targetPos, direction)
end

function FollowCombineTask:updateStates(dt)
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local cx, cy, cz = getWorldTranslation(self.combine.components[1].node)

    self.chasePos, self.chaseSide = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getPipeChasePosition()

    self.angleToCombineHeading = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getAngleToCombineHeading()
    self.angleToCombine = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getAngleToCombine()

    self.lastChaseSide = self.chaseSide

    self.distanceToCombine = MathUtil.vector2Length(x - cx, z - cz)

    if (g_updateLoopIndex  % AutoDrive.PERF_FRAMES == 0) or self.updateStatesFirst ~= true then
        self.updateStatesFirst = true

        local cmaxCapacity = 0
        local cfillLevel = 0
        cfillLevel, cmaxCapacity, _ = AutoDrive.getObjectFillLevels(self.combine)
        self.combineFillPercent = cmaxCapacity > 0 and (cfillLevel / cmaxCapacity) * 100 or 0

        local fillFreeCapacity = 0
        _, _, self.filledToUnload, fillFreeCapacity = AutoDrive.getAllFillLevels(self.trailers)
        self.filled = fillFreeCapacity <= 0.1
        
        self.activeUnloading = AutoDrive.getSetting("activeUnloading", self.combineRootVehicle)
    end
    self:shouldWaitForChasePos(dt)
end

function FollowCombineTask:reverse(dt)
    self.vehicle.ad.specialDrivingModule:driveReverse(dt, 15, 1, self.vehicle.ad.trailerModule:canBeHandledInReverse())
end

function FollowCombineTask:followChasePoint(dt)
    self.rollingNow = false
    if self.combineInTurn then
        -- combine turning: no line check, a blocked line must not end the chase during the turn
        self.lineCheckTimer = 0
        self.lineClear = true
        self.lineBlockedTimer:timer(false, FollowCombineTask.LINE_BLOCKED_TIME, dt)
    else
        self.lineCheckTimer = self.lineCheckTimer - dt
    end
    if self.lineCheckTimer < 0 then
        self.lineCheckTimer = FollowCombineTask.LINE_CHECK_INTERVAL
        local clear, reason = FollowCombineTask.isChaseLineClear(self.vehicle, self.combine, self.chasePos, self.chaseSide)
        self.lineBlockReason = reason
        self.lineClear = clear
    end
    -- crop on the line to the side place: go behind the harvester first, before the island
    local mode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
    if not self.lineClear and self.lineBlockReason ~= nil and string.sub(self.lineBlockReason, 1, 4) == "crop"
        and self.combine.ad.isChopper and mode ~= nil and mode.isIslandRear ~= nil and not mode:isRearChaseHeld()
        and self.chaseSide ~= AutoDrive.CHASEPOS_REAR then
        mode.islandRearUntil = (g_time or 0) + CombineUnloaderMode.ISLAND_REAR_TIME
        self.lineClear = true
        self.lineCheckTimer = 0
        self.lineBlockedTimer:timer(false, FollowCombineTask.LINE_BLOCKED_TIME, dt)
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return
    end
    -- a fixed obstacle (wall, tree, building) or a learned blocked zone on
    -- the line to a side place: that side is condemned for a while and the place changes (other side or behind) instead
    -- of handing over to a new route to the same place every 2 s (an unloader along the stone wall)
    if not self.lineClear and (self.lineBlockReason == "obstacle" or self.lineBlockReason == "blocked zone")
        and self.combine.ad.isChopper and mode ~= nil and mode.blockSide ~= nil
        and (self.chaseSide == AutoDrive.CHASEPOS_LEFT or self.chaseSide == AutoDrive.CHASEPOS_RIGHT) and not mode:isSideBlocked(self.chaseSide) then
        mode:blockSide(self.chaseSide, self.lineBlockReason)
        self.lineClear = true
        self.lineCheckTimer = 0
        self.lineBlockedTimer:timer(false, FollowCombineTask.LINE_BLOCKED_TIME, dt)
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return
    end
    self.lineBlockedTimer:timer(not self.lineClear, FollowCombineTask.LINE_BLOCKED_TIME, dt)
    if not self.lineClear then
        if self.lineBlockedTimer:done() then
            self.lineBlocked = true
            self.state = FollowCombineTask.STATE_FINISHED
        end
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return
    end
    -- the harvester stands and waits for this trailer, its place lies behind the tractor: back up to it
    if self:tryBackToPlace(dt) then
        return
    end
    if self:shouldWaitForChasePos(dt) then
        self.rollingNow = self:rollInsteadOfWait(dt)
        if self.rollingNow then
            return
        end
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:followChasePoint getAngleToChasePos %.0f -> stopVehicle", self:getAngleToChasePos())
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
    else
        self.rollingNow = false
        self.startedChasing = true
        local combineSpeed = self.combine.lastSpeedReal * 3600
        local acc = 1
        local totalSpeedLimit = 40
        -- Let's start driving a little slower when we are switching sides
        if not self.chaseTimer:done() or not self:isCaughtCurrentChaseSide() then
            acc = 1
            totalSpeedLimit = math.max(combineSpeed + 20, 10)
        end
        -- forage harvester in a curve toward us: the 3/4 rear place lies behind the tractor, it stops and lets the
        -- harvester move on, no reversing
        local unloadMode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
        -- (not while it stands: it may be waiting for this trailer under its pipe)
        local combineStands = self.combine.ad.noMovementTimer ~= nil and self.combine.ad.noMovementTimer.elapsedTime > 3000
        if AutoDrive.isChopperUnloading(self.vehicle) and unloadMode.curveRearSide ~= nil and self.chasePos ~= nil and not combineStands then
            local _, _, lz = AutoDrive.worldToLocal(self.vehicle, self.chasePos.x, self.chasePos.y or 0, self.chasePos.z)
            if lz < -1 then
                -- no stop any more: it drives on slower than the harvester, aiming its line ahead of
                -- the place, until the harvester has brought the place up beside it
                local slow = math.max(combineSpeed - FollowCombineTask.CURVE_SLOWER_KMH, FollowCombineTask.CURVE_MIN_KMH)
                local fx, _, fz = AutoDrive.localDirectionToWorld(self.combine, 0, 0, 1)
                local ahead = -lz + FollowCombineTask.CURVE_AIM_AHEAD
                local aim = {x = self.chasePos.x + fx * ahead, y = self.chasePos.y, z = self.chasePos.z + fz * ahead}
                self.vehicle.ad.specialDrivingModule:driveToPoint(dt, aim, slow, false, acc, slow)
                return
            end
        end
        self.vehicle.ad.specialDrivingModule:driveToPoint(dt, self.chasePos, combineSpeed, false, acc, totalSpeedLimit)
    end
end

-- the harvester stands (it waits for this trailer) and the chase place lies behind the tractor
-- (behind the harvester through a jog, or beside it after an overshoot): rolling on brings nothing (it does not move),
-- waiting makes both wait. The tractor backs up straight until the place is a few metres ahead, then the chase drives
-- to it. Never in the harvester's lane ahead of it (its header), never into the crop, once per place
FollowCombineTask.BACK_STAND_TIME = 2000 -- ms the harvester stands before the tractor backs up
FollowCombineTask.BACK_MIN_BEHIND = 3 -- m the place is behind the tractor at least
FollowCombineTask.BACK_AHEAD = 4 -- m the place must be ahead of the tractor at the end
FollowCombineTask.BACK_AHEAD_PER_SIDE = 1.2 -- m ahead more per m the place is aside (room to steer to it)
FollowCombineTask.BACK_MAX_DISTANCE = 16 -- m backed up at most
FollowCombineTask.BACK_MAX_TIME = 25000
FollowCombineTask.BACK_BLOCKED_TIME = 3000 -- ms without moving: something behind, stop there
FollowCombineTask.BACK_SPEED = 6 -- km/h
FollowCombineTask.BACK_COOLDOWN = 20000 -- ms before another one
FollowCombineTask.BACK_MIN_SIDE = 2 -- m aside at least for a place ahead to count as out of reach
FollowCombineTask.BACK_ASIDE_RATIO = 1.0 -- place less far ahead than this times its side offset (45 deg or more aside): back up first
function FollowCombineTask:tryBackToPlace(dt)
    local combine = self.combine
    if not AutoDrive.isChopperUnloading(self.vehicle) or self.chasePos == nil or self.filled or self.innerTurn or self.endRowTurn ~= nil then
        return false
    end
    local stoppedTime = combine.ad.noMovementTimer ~= nil and combine.ad.noMovementTimer.elapsedTime or 0
    if stoppedTime < FollowCombineTask.BACK_STAND_TIME or math.abs(combine.lastSpeedReal or 0) > FollowCombineTask.TURN_STOPPED_SPEED then
        return false
    end
    local lx, _, lz = AutoDrive.worldToLocal(self.vehicle, self.chasePos.x, self.chasePos.y or 0, self.chasePos.z)
    -- the place behind us, or aside and too close ahead to steer to it (seen: jog place 0.5 m ahead and 7 m aside, the trailer stood 60 s
    -- beside the standing harvester before anything backed up)
    local aside = math.abs(lx) >= FollowCombineTask.BACK_MIN_SIDE and lz < math.abs(lx) * FollowCombineTask.BACK_ASIDE_RATIO
    if lz > -FollowCombineTask.BACK_MIN_BEHIND and not aside then
        return false
    end
    local where = lz < 0 and string.format("%.0f m behind me", -lz) or string.format("only %.1f m ahead of me, %.0f m sideways", lz, math.abs(lx))
    local now = g_time or 0
    if self.backToPlaceLast ~= nil and now - self.backToPlaceLast < FollowCombineTask.BACK_COOLDOWN then
        return false
    end
    local name = FollowCombineTask.getLogName(self.vehicle)
    -- the train beside the harvester (clear of its header) or behind it: never backing up in its lane ahead of it
    local vx, vy, vz = getWorldTranslation(self.vehicle.components[1].node)
    local ox, _, oz = AutoDrive.worldToLocal(combine, vx, vy, vz, combine.ad.ADRootNode)
    local combineHalf = math.max(combine.size ~= nil and combine.size.width or 4, AutoDrive.getFrontToolWidth(combine) or 0) / 2
    local trainHalf = (self.vehicle.ad.pathFinderModule ~= nil and self.vehicle.ad.pathFinderModule:getTrainWidth() or 3) / 2
    local combineRear = (combine.size ~= nil and combine.size.length or 10) / 2 - (combine.size ~= nil and combine.size.lengthOffset or 0)
    local tractorFront = (self.vehicle.size ~= nil and self.vehicle.size.length or 6) / 2 + (self.vehicle.size ~= nil and self.vehicle.size.lengthOffset or 0)
    local beside = math.abs(ox) >= combineHalf + trainHalf + FollowCombineTask.ROLL_SIDE_ROOM
    local behind = oz + tractorFront < -combineRear
    if not beside and not behind then
        self.backToPlaceLast = now
        return false
    end
    if self:isCropBehindTrain() then
        self.backToPlaceLast = now
        return false
    end
    self.backToPlaceLast = now
    -- room to steer to it: BACK_AHEAD plus BACK_AHEAD_PER_SIDE per metre aside (7 m aside: about 12.5 m ahead, 30 deg)
    local need = FollowCombineTask.BACK_AHEAD + math.abs(lx) * FollowCombineTask.BACK_AHEAD_PER_SIDE
    self.backToPlace = {sx = vx, sz = vz, time = 0, still = 0, need = need}
    self.state = FollowCombineTask.STATE_BACK_TO_PLACE
    return true
end

function FollowCombineTask:updateBackToPlace(dt)
    local b = self.backToPlace
    if b == nil or self.chasePos == nil then
        self.backToPlace = nil
        self.state = FollowCombineTask.STATE_CHASING
        return
    end
    b.time = b.time + dt
    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
    local reversed = MathUtil.vector2Length(vx - b.sx, vz - b.sz)
    local _, _, lz = AutoDrive.worldToLocal(self.vehicle, self.chasePos.x, self.chasePos.y or 0, self.chasePos.z)
    if b.time > 1500 and math.abs(self.vehicle.lastSpeedReal or 0) < 0.0003 then
        b.still = b.still + dt
    else
        b.still = 0
    end
    local why
    if lz >= b.need then
        why = "place ahead"
    elseif reversed >= FollowCombineTask.BACK_MAX_DISTANCE then
        why = string.format("%d m recules", FollowCombineTask.BACK_MAX_DISTANCE)
    elseif math.abs(self.combine.lastSpeedReal or 0) > FollowCombineTask.TURN_STOPPED_SPEED then
        why = "the harvester drives on"
    elseif b.still > FollowCombineTask.BACK_BLOCKED_TIME then
        why = "something behind"
    elseif b.time > FollowCombineTask.BACK_MAX_TIME then
        why = "too long"
    elseif self:isCropBehindTrain() then
        why = "crop behind the trailer"
    end
    if why ~= nil then
        self.backToPlace = nil
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        self.state = FollowCombineTask.STATE_CHASING
        return
    end
    self.vehicle.ad.specialDrivingModule:driveReverse(dt, FollowCombineTask.BACK_SPEED, 1, self.vehicle.ad.trailerModule:canBeHandledInReverse())
end

-- the train
-- faces the standing harvester (the wrong way round) and no Courseplay turn of hers will bring it round: the chase
-- hands over to its own U-turn (AutoDrive's end-of-row U-turn first, then the strip manoeuvre, then a route)
FollowCombineTask.FACING_UTURN_DEG = 100 -- deg against the harvester heading (as the catch task)
FollowCombineTask.FACING_STOPPED_TIME = 3000 -- ms the harvester stands
FollowCombineTask.FACING_MAX_DISTANCE = 80 -- m
function FollowCombineTask:turnBeforeChase(why)
    local combine = self.combine
    if not AutoDrive.isChopperUnloading(self.vehicle) or self.filled or (self.angleToCombineHeading or 0) < FollowCombineTask.FACING_UTURN_DEG
        or (self.distanceToCombine or math.huge) > FollowCombineTask.FACING_MAX_DISTANCE then
        return false
    end
    local stoppedTime = combine.ad.noMovementTimer ~= nil and combine.ad.noMovementTimer.elapsedTime or 0
    if stoppedTime < FollowCombineTask.FACING_STOPPED_TIME then
        return false
    end
    -- a real Courseplay turn being driven brings it round (the maneuvering flag is also raised while it stands)
    local row = CombineUnloaderMode.getCpRowTurn(combine)
    if row ~= nil and row.turning then
        return false
    end
    local mode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
    if mode ~= nil and mode:isRearChaseHeld() then
        return false
    end
    self.vehicle.ad.endRowUTurn = g_time or 0
    self.stayOnField = true
    self.state = FollowCombineTask.STATE_FINISHED
    return true
end

-- (an unloader standing 6 s, 40 m before a jog): the place went behind the tractor (behind the
-- harvester or 3/4 rear) or more than 50 deg aside while the forage harvester drives on. The tractor no longer stops
-- and waits: it rolls on slower than the harvester, parallel to it at its present distance aside, until the
-- harvester has brought the place up ahead. Same line check as the chase (islands, bands, trail); a real block stops.
FollowCombineTask.ROLL_SLOWER_KMH = 3
FollowCombineTask.ROLL_MIN_KMH = 2
FollowCombineTask.ROLL_MIN_COMBINE_KMH = 4 -- the harvester slower than this is stopping: the tractor stops too
FollowCombineTask.ROLL_AIM_AHEAD = 8 -- m ahead of the tractor along the harvester
FollowCombineTask.ROLL_SIDE_ROOM = 0.5 -- m between the header side and the train side at least
FollowCombineTask.ROLL_CATCH_TIME = 3 -- s: the speed lets the harvester bring a place behind up in about this time
function FollowCombineTask:rollInsteadOfWait(dt)
    if not AutoDrive.isChopperUnloading(self.vehicle) or self.chasePos == nil or self.combineInTurn
        or FollowCombineTask.isCombineReversing(self.combine) then
        return false
    end
    -- a jog in its course (power pole): the tractor waits behind on purpose, as decided
    local mode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
    if mode ~= nil and mode.jogRearStart ~= nil then
        return false
    end
    local combineSpeed = math.abs(self.combine.lastSpeedReal or 0) * 3600
    if combineSpeed < FollowCombineTask.ROLL_MIN_COMBINE_KMH then
        return false
    end
    -- tractor in the harvester frame: beside it (clear of the header) or behind it, never in its lane ahead
    local vx, vy, vz = getWorldTranslation(self.vehicle.components[1].node)
    local ox, _, oz = AutoDrive.worldToLocal(self.combine, vx, vy, vz, self.combine.ad.ADRootNode)
    local combineHalf = math.max(self.combine.size ~= nil and self.combine.size.width or 4, AutoDrive.getFrontToolWidth(self.combine) or 0) / 2
    local trainHalf = (self.vehicle.ad.pathFinderModule ~= nil and self.vehicle.ad.pathFinderModule:getTrainWidth() or 3) / 2
    local combineRear = (self.combine.size ~= nil and self.combine.size.length or 10) / 2 - (self.combine.size ~= nil and self.combine.size.lengthOffset or 0)
    local tractorFront = (self.vehicle.size ~= nil and self.vehicle.size.length or 6) / 2 + (self.vehicle.size ~= nil and self.vehicle.size.lengthOffset or 0)
    local beside = math.abs(ox) >= combineHalf + trainHalf + FollowCombineTask.ROLL_SIDE_ROOM
    local behind = oz + tractorFront < -combineRear
    if not beside and not behind then
        return false
    end
    local ax, ay, az = AutoDrive.localToWorld(self.combine, ox, 0, oz + FollowCombineTask.ROLL_AIM_AHEAD, self.combine.ad.ADRootNode)
    local aim = {x = ax, y = ay, z = az}
    self.rollCheckTimer = (self.rollCheckTimer or 0) - dt
    if self.rollCheckTimer < 0 then
        self.rollCheckTimer = FollowCombineTask.LINE_CHECK_INTERVAL
        local clear, reason = FollowCombineTask.isChaseLineClear(self.vehicle, self.combine, aim, self.chaseSide)
        self.rollClear = clear
    end
    if not self.rollClear then
        return false
    end
    -- slower the further the place is behind (3 km/h slower only, the place 8 m behind never came up):
    -- the harvester brings it up in about ROLL_CATCH_TIME, never faster than the harvester minus ROLL_SLOWER_KMH
    local _, _, lz = AutoDrive.worldToLocal(self.vehicle, self.chasePos.x, self.chasePos.y or 0, self.chasePos.z)
    local behind = math.max(0, -lz)
    local speed = math.max(math.min(combineSpeed - FollowCombineTask.ROLL_SLOWER_KMH, combineSpeed - behind / FollowCombineTask.ROLL_CATCH_TIME * 3.6), FollowCombineTask.ROLL_MIN_KMH)
    self.vehicle.ad.specialDrivingModule:driveToPoint(dt, aim, speed, false, 1, speed, true)
    return true
end

FollowCombineTask.CURVE_SLOWER_KMH = 3
FollowCombineTask.CURVE_MIN_KMH = 2
FollowCombineTask.CURVE_AIM_AHEAD = 8 -- m

FollowCombineTask.END_ROW_TURN_DEG = 150 -- Courseplay course turning that much over 50 m (90 + 90): end-of-row U-turn
FollowCombineTask.END_ROW_BEND_WITHIN = 15 -- m ahead of the harvester
FollowCombineTask.END_ROW_STOP_ADVANCE = 2 -- m the harvester drives on (trailer under the pipe) after the red point is within reach, before the stop
FollowCombineTask.END_ROW_DONE_DEG = 120 -- heading change of the harvester before the wait can end
FollowCombineTask.END_ROW_WAIT_STOPPED = 2000 -- ms: turned and standing = done
FollowCombineTask.END_ROW_PAUSE = 30000 -- ms after a wait before another one
FollowCombineTask.END_ROW_STRAIGHT_AHEAD = 30 -- m of Courseplay course read to see the harvester in its row again
FollowCombineTask.END_ROW_STRAIGHT_DEG = 20 -- deg: course ahead counted as straight below this
-- never leave before the harvester, wait until it has freed the place, then the own U-turn
FollowCombineTask.ANTICIPATE = false
FollowCombineTask.END_ROW_FIRST_TURN = 80 -- deg turned by the harvester (its first 90 deg): the own manoeuvre starts
FollowCombineTask.ANTICIPATED_EXTRA = 5 -- m more behind the row entry: room for the harvester to finish its turn
FollowCombineTask.END_ROW_INVERTED_DEG = 150 -- original AutoDrive: harvester heading against ours, wait for the pass-by
FollowCombineTask.END_ROW_PASS_BY_RANGE = 80
FollowCombineTask.END_ROW_PASSED_BY = -10 -- m, harvester behind us along our heading: passed
FollowCombineTask.END_ROW_PASS_BY_TIME = 15000
FollowCombineTask.END_ROW_ALIGNED_DEG = 60 -- tractor within this of the harvester heading afterwards: chase on
-- row change, the harvester comes back facing us and waits for the trailer: drive along its side
-- until the trailer is behind it, then the U-turn there (AutoDrive one, else the manoeuvre in several moves)
FollowCombineTask.PASS_ALONG_MARGIN = 5 -- m between the trailer rear and the harvester rear at the stop
-- harvester at the field end, 5 m more behind it was off the field: stop earlier, down to 1 m
FollowCombineTask.PASS_ALONG_MIN_MARGIN = 1
FollowCombineTask.PASS_ALONG_SIDE_ROOM = 1.5 -- m between the harvester (header) side and the train side
FollowCombineTask.PASS_ALONG_S_MIN = 12
FollowCombineTask.PASS_ALONG_S_MAX = 25
FollowCombineTask.PASS_ALONG_S_PER_M = 3
FollowCombineTask.PASS_ALONG_BACKS = {0, 8, 15} -- m reversed in the own track first
FollowCombineTask.PASS_ALONG_STRAIGHTS = {0, 5, 10, 15, 20, 30} -- m straight before the S
FollowCombineTask.PASS_ALONG_S_LENGTHS = {12, 18, 25} -- m, longest S allowed
FollowCombineTask.PASS_ALONG_EXTRA_SIDE = {0, 3} -- m more room beside the harvester
FollowCombineTask.PASS_ALONG_TIME = 90000
FollowCombineTask.PASS_ALONG_BLOCKED = 6000

-- forward points along the harvester line (its heading reversed), side gap taken up by a gentle S, checked for
-- Strict field / crop and for vehicles or obstacles under the tractor and the trailer; nil, why when it does not fit
function FollowCombineTask:buildPassAlong(combine)
    local hx, _, hz = AutoDrive.localDirectionToWorld(combine, 0, 0, 1)
    local len = math.sqrt(hx * hx + hz * hz)
    if len < 0.01 then
        return nil, "harvester heading unknown"
    end
    hx, hz = hx / len, hz / len
    local nx, nz = hz, -hx
    local cx, _, cz = getWorldTranslation(combine.components[1].node)
    local tx, _, tz = getWorldTranslation(self.vehicle.components[1].node)
    local along = (tx - cx) * hx + (tz - cz) * hz
    local gap = (tx - cx) * nx + (tz - cz) * nz
    local trainWidth = self.vehicle.ad.pathFinderModule ~= nil and self.vehicle.ad.pathFinderModule:getTrainWidth() or 3
    local combineWidth = math.max(combine.size ~= nil and combine.size.width or 4, AutoDrive.getFrontToolWidth(combine) or 0)
    local side = combineWidth / 2 + trainWidth / 2 + FollowCombineTask.PASS_ALONG_SIDE_ROOM
    local combineRear = (combine.size ~= nil and combine.size.length or 10) / 2 - (combine.size ~= nil and combine.size.lengthOffset or 0)
    local tractorLength = self.vehicle.size ~= nil and self.vehicle.size.length or 6
    local tractorOffset = self.vehicle.size ~= nil and self.vehicle.size.lengthOffset or 0
    local d0 = math.max(0, AutoDrive.getTractorTrainLength(self.vehicle, true, false) - tractorLength / 2)
    local aEnd = -(combineRear + FollowCombineTask.PASS_ALONG_MARGIN + d0)
    if along - aEnd < 10 then
        return nil, string.format("already behind it (%.0f m)", along)
    end
    local probe = {vehicle = self.vehicle}
    local head = string.format("harvester %.0f,%.0f, tractor %.0f,%.0f, %.0f m ahead / %.0f m sideways", cx, cz, tx, tz, along, gap)
    local function at(s0, lateral, isReverse)
        local x, z = cx + hx * s0 + nx * lateral, cz + hz * s0 + nz * lateral
        return {x = x, y = AutoDrive:getTerrainHeightAtWorldPos(x, z) or 0, z = z, isReverse = isReverse}
    end
    -- one S only crossed the grass patch of a power pole. Several layouts, the simplest first:
    -- a short reverse in the own track first, a straight stretch, then the S (length, side room)
    local best = nil
    local tried = 0
    local signs = gap >= 0 and {1, -1} or {-1, 1}
    for _, back in ipairs(FollowCombineTask.PASS_ALONG_BACKS) do
        for _, straight in ipairs(FollowCombineTask.PASS_ALONG_STRAIGHTS) do
            for _, sMax in ipairs(FollowCombineTask.PASS_ALONG_S_LENGTHS) do
                for _, extra in ipairs(FollowCombineTask.PASS_ALONG_EXTRA_SIDE) do
                    for _, sign in ipairs(signs) do
                        local target = sign * math.max(side + extra, sign * gap > 0 and math.abs(gap) or 0)
                        local a0 = along + back
                        local total = a0 - aEnd
                        local sLen = math.max(FollowCombineTask.PASS_ALONG_S_MIN, math.min(sMax, math.abs(target - gap) * FollowCombineTask.PASS_ALONG_S_PER_M))
                        if straight + sLen <= total - 2 then
                            tried = tried + 1
                            local points = {}
                            -- reverse in the own track (the train came that way), checked at the end only
                            if back > 0 then
                                for d = 0, back, 1 do
                                    table.insert(points, at(along + d, gap, true))
                                end
                            end
                            local fwdStart = #points
                            local n = math.max(2, math.floor(total))
                            for i = 0, n do
                                local d = total * i / n
                                local u = math.max(0, math.min(1, (d - straight) / math.max(sLen, 0.1)))
                                local lateral = gap + (target - gap) * (3 * u * u - 2 * u * u * u)
                                table.insert(points, at(a0 - d, lateral, false))
                            end
                            local keep, why = #points, nil
                            for i = fwdStart + 2, #points do
                                local good, w = HandleHarvesterTurnTask.checkGround(probe, {points[i - 1], points[i]}, 1)
                                if not good then
                                    keep = i - 1
                                    why = string.format("%s a %.0f m", tostring(w), total * (i - 1 - fwdStart - 1) / n)
                                    break
                                end
                            end
                            local ok = true
                            if keep < #points then
                                local sKeep = (points[keep].x - cx) * hx + (points[keep].z - cz) * hz
                                local margin = -sKeep - d0 - combineRear
                                if margin < FollowCombineTask.PASS_ALONG_MIN_MARGIN then
                                    ok = false
                                    -- the failure that went furthest is the one reported
                                    if best == nil or keep > best.keep then
                                        best = {keep = keep, why = string.format("%s (reverse %d, straight %d, S %d, side %s)", why, back, straight, sMax, sign * gap >= 0 and "actuel" or "oppose")}
                                    end
                                else
                                    for i = #points, keep + 1, -1 do
                                        table.remove(points, i)
                                    end
                                end
                            end
                            if ok then
                                local hit = self:passAlongObstacle(points, trainWidth, tractorLength, tractorOffset, d0)
                                if hit == nil then
                                    return points, target
                                end
                                if best == nil then
                                    best = {keep = 0, why = hit}
                                end
                            end
                        end
                    end
                end
            end
        end
    end
    return nil, string.format("%s, %d tries, farthest: %s", head, tried, best ~= nil and best.why or "none")
end

-- tractor body every 2 m (the harvester is NOT left out: we must not touch it), trailer body at the stop
function FollowCombineTask:passAlongObstacle(points, trainWidth, tractorLength, tractorOffset, d0)
    local module = self.vehicle.ad.trafficYieldModule
    if module == nil or module.overlapHit == nil then
        return nil
    end
    local halfW = trainWidth / 2
    local last = 0
    for i = 2, #points do
        local a, b = points[i - 1], points[i]
        if not b.isReverse and (i == #points or i - last >= 2) then
            last = i
            local dx, dz = b.x - a.x, b.z - a.z
            local l = math.max(0.01, math.sqrt(dx * dx + dz * dz))
            dx, dz = dx / l, dz / l
            local x, z = b.x + dx * tractorOffset, b.z + dz * tractorOffset
            local y = AutoDrive:getTerrainHeightAtWorldPos(x, z) or 0
            if module:overlapHit(x, y + 2, z, math.atan2(dx, dz), halfW, 1.5, tractorLength / 2) then
                return string.format("vehicle or obstacle at %.0f,%.0f", x, z)
            end
            if i == #points and d0 > 1 then
                local mx, mz = b.x - dx * (tractorLength / 2 + d0 / 2), b.z - dz * (tractorLength / 2 + d0 / 2)
                local my = AutoDrive:getTerrainHeightAtWorldPos(mx, mz) or 0
                if module:overlapHit(mx, my + 2, mz, math.atan2(dx, dz), halfW, 1.5, d0 / 2) then
                    return string.format("trailer on a vehicle or obstacle at %.0f,%.0f", mx, mz)
                end
            end
        end
    end
    return nil
end

function FollowCombineTask:updateEndRowTurn(dt)
    local combine = self.combine
    if self.endRowPassAlong ~= nil then
        local p = self.endRowPassAlong
        local name = FollowCombineTask.getLogName(self.vehicle)
        p.time = p.time + dt
        self.stuckTimer:timer(false)
        p.blocked = math.abs(self.vehicle.lastSpeedReal or 0) < 0.0002 and p.blocked + dt or 0
        local reached = self.vehicle.ad.drivePathModule:isTargetReached()
        if not reached and p.time < FollowCombineTask.PASS_ALONG_TIME and p.blocked < FollowCombineTask.PASS_ALONG_BLOCKED then
            self.vehicle.ad.drivePathModule:update(dt)
            return true
        end
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.endRowPassAlong = nil
        self.vehicle.ad.endRowUTurn = g_time or 0
        self.stayOnField = true
        self.state = FollowCombineTask.STATE_FINISHED
        return true
    end
    -- original AutoDrive condition: turn over,
    -- harvester coming back against us: let it pass by (10 m behind us, 15 s at most), then the U-turn toward its side
    if self.endRowPassBy ~= nil then
        local p = self.endRowPassBy
        p.time = p.time + dt
        self.stuckTimer:timer(false)
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        local cx, cy, cz = getWorldTranslation(combine.components[1].node)
        local _, _, offsetZ = AutoDrive.worldToLocal(self.vehicle, cx, cy, cz)
        if offsetZ > FollowCombineTask.END_ROW_PASSED_BY and p.time < FollowCombineTask.END_ROW_PASS_BY_TIME then
            return true
        end
        self.endRowPassBy = nil
        self.vehicle.ad.endRowUTurn = g_time or 0
        self.stayOnField = true
        self.state = FollowCombineTask.STATE_FINISHED
        return true
    end
    if not self.endRowTurn then
        if (self.state ~= FollowCombineTask.STATE_CHASING and self.state ~= FollowCombineTask.STATE_WAIT_FOR_TURN)
            or not AutoDrive.isChopperUnloading(self.vehicle) or not combine.ad.isChopper or not AutoDrive:getIsCPActive(combine) then
            return false
        end
        -- kept on the tractor: a new chase task after the wait must not start a second one
        if self.vehicle.ad.endRowTurnPause ~= nil and (g_time or 0) < self.vehicle.ad.endRowTurnPause then
            return false
        end
        -- Courseplay turn points first
        -- the wait starts only before the red point of an end-of-row U-turn, a loop or a jog around a power
        -- pole has none. The course shape only when the points cannot be read
        local angle, src
        local row, rowWhy = CombineUnloaderMode.getCpRowTurn(combine)
        -- the wait ended because the harvester stood before its turn (it waited for this
        -- trailer): it starts again only once the harvester drives and really turns (heading changed since then)
        if not self:updateEndRowHold() then
            return false
        end
        if row ~= nil then
            if not row.uTurn or (not row.turning and (row.dist == nil or row.dist > FollowCombineTask.END_ROW_BEND_WITHIN)) then
                self.endRowArm = nil
                return false
            end
            -- the trailer stays under the pipe END_ROW_STOP_ADVANCE more metres of the
            -- harvester's way before the stop, as long as it drives on without turning
            if not row.turning and math.abs(combine.lastSpeedReal or 0) > FollowCombineTask.TURN_STOPPED_SPEED then
                local cx, _, cz = getWorldTranslation(combine.components[1].node)
                if self.endRowArm == nil then
                    self.endRowArm = {x = cx, z = cz}
                end
                if MathUtil.vector2Length(cx - self.endRowArm.x, cz - self.endRowArm.z) < FollowCombineTask.END_ROW_STOP_ADVANCE then
                    return false
                end
            end
            self.endRowArm = nil
            angle, src = row.deg, "Courseplay points, " .. tostring(row.why)
        else
            local bend
            angle, _, bend = CombineUnloaderMode.getCpTurnAhead(combine, CombineUnloaderMode.CP_UTURN_AHEAD)
            if angle == nil or angle < FollowCombineTask.END_ROW_TURN_DEG or bend == nil or bend > FollowCombineTask.END_ROW_BEND_WITHIN then
                return false
            end
            -- a jog (the course comes back to its heading) is no end-of-row U-turn, the trailer goes behind
            if CombineUnloaderMode.isCpJogAhead(combine) then
                return false
            end
            src = string.format("%.0f deg on the Courseplay course, unreadable points: %s", angle, tostring(rowWhy))
        end
        self.endRowTurn = {start = FollowCombineTask.getCombineHeading(combine), time = 0, cpEndIx = row ~= nil and row.endIx or nil}
        self.vehicle.ad.endRowTurnActive = true
        -- the place behind (jog or island) was still on, the trailer followed the harvester into its
        -- U-turn at 17 km/h
        local unloadMode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
        if unloadMode ~= nil and (unloadMode.jogRearStart ~= nil or unloadMode.islandRearUntil ~= nil) then
            unloadMode.jogRearStart = nil
            unloadMode.islandRearUntil = nil
        end
        if self.innerTurn then
            self:endInnerTurn("end-of-row U-turn, stop")
        end
        -- anticipated: Courseplay tells where the next row starts, our own U-turn starts now toward
        -- a place behind that entry, both turn at the same time
        local ex, ez, dx, dz = CombineUnloaderMode.getCpRowEntry(combine)
        if ex == nil and self.heldRowEntry ~= nil then
            -- read before the wait was released on the standing harvester, unreadable now in its turn
            ex, ez, dx, dz = self.heldRowEntry.ex, self.heldRowEntry.ez, self.heldRowEntry.dx, self.heldRowEntry.dz
        end
        self.heldRowEntry = nil
        if ex ~= nil and not self.filled then
            -- new row kept for later: own U-turn toward it once the harvester has done its first 90 deg
            local back0 = (combine.size ~= nil and combine.size.length or 10) / 2 + PathFinderModule.REAR_FALLBACK_GAP
                + AutoDrive.getTractorTrainLength(self.vehicle, true, false) / 2 + FollowCombineTask.ANTICIPATED_EXTRA
            self.vehicle.ad.anticipatedRear = {ex = ex, ez = ez, dx = dx, dz = dz, back = back0, t = g_time or 0}
        end
        if FollowCombineTask.ANTICIPATE and ex ~= nil and not self.filled then
            local back = (combine.size ~= nil and combine.size.length or 10) / 2 + PathFinderModule.REAR_FALLBACK_GAP
                + AutoDrive.getTractorTrainLength(self.vehicle, true, false) / 2 + FollowCombineTask.ANTICIPATED_EXTRA
            self.vehicle.ad.anticipatedRear = {ex = ex, ez = ez, dx = dx, dz = dz, back = back, t = g_time or 0}
            self.endRowTurn = nil
            self.vehicle.ad.endRowTurnActive = nil
            self.vehicle.ad.endRowTurnPause = (g_time or 0) + FollowCombineTask.END_ROW_PAUSE
            -- its own U-turn first (forward, back then forward, 3 points), then the place behind the entry
            self.vehicle.ad.endRowUTurn = g_time or 0
            self.stayOnField = true
            self.state = FollowCombineTask.STATE_FINISHED
            -- nothing else this frame: the chase must not take the hand back
            return true
        end
    end
    local t = self.endRowTurn
    t.time = t.time + dt
    self.stuckTimer:timer(false)
    self.vehicle.ad.specialDrivingModule:stopVehicle()
    self.vehicle.ad.specialDrivingModule:update(dt)
    local heading = FollowCombineTask.getCombineHeading(combine)
    local turned = math.abs(heading - t.start) % 360
    if turned > 180 then
        turned = 360 - turned
    end
    local steady = #self.turnHeadingSamples > FollowCombineTask.TURN_HEADING_SAMPLE_TIME / 500
    if steady then
        local diff = math.abs(heading - self.turnHeadingSamples[1]) % 360
        if diff > 180 then
            diff = 360 - diff
        end
        steady = diff <= FollowCombineTask.TURN_END_MAX_HEADING_CHANGE
    end
    -- held here, the harvester reached the row end
    -- without a trailer under its pipe and stopped; both waited 60 s. Standing that long before its first quarter
    -- turn it waits for this trailer: the wait ends, the chase brings the trailer under the pipe, and the wait
    -- starts again once it really turns (updateEndRowHold)
    local stoppedTime = combine.ad.noMovementTimer ~= nil and combine.ad.noMovementTimer.elapsedTime or 0
    if stoppedTime >= FollowCombineTask.TURN_STOPPED_RELEASE_TIME and turned < FollowCombineTask.END_ROW_FIRST_TURN then
        self.endRowTurn = nil
        self.vehicle.ad.endRowTurnActive = nil
        self.vehicle.ad.endRowHold = {heading = heading, t = g_time or 0, entry = self.vehicle.ad.anticipatedRear}
        self.vehicle.ad.anticipatedRear = nil
        self.combineInTurn = false
        self.turnReleasedStopped = true
        self.state = FollowCombineTask.STATE_CHASING
        return true
    end
    -- the harvester has done its first 90 deg (place freed): the own manoeuvre starts now
    local ant = self.vehicle.ad.anticipatedRear
    if ant ~= nil and (g_time or 0) - ant.t < PathFinderModule.ANTICIPATED_MAX_AGE and turned >= FollowCombineTask.END_ROW_FIRST_TURN then
        self.endRowTurn = nil
        self.vehicle.ad.endRowTurnActive = nil
        self.vehicle.ad.endRowTurnPause = (g_time or 0) + FollowCombineTask.END_ROW_PAUSE
        self.combineInTurn = false
        self.vehicle.ad.endRowUTurn = g_time or 0
        self.stayOnField = true
        self.state = FollowCombineTask.STATE_FINISHED
        return true
    end
    local forward = combine.ad.driveForwardTimer.elapsedTime >= FollowCombineTask.TURN_END_FORWARD_TIME
    -- turned and standing: it waits for this trailer in its new row (both waited for each other)
    local waiting = combine.ad.noMovementTimer ~= nil and combine.ad.noMovementTimer.elapsedTime > FollowCombineTask.END_ROW_WAIT_STOPPED
    -- standing at the row entrance it still counts as turning for Courseplay (60 s lost): not asked then
    -- headland to row: it may align on its row with less than END_ROW_DONE_DEG turned; the turn is
    -- over too once Courseplay is no longer turning and its course ahead is straight again
    local cpTurning = AutoDrive:getIsCPTurning(combine)
    local turnOver, overWhy = false, nil
    if t.cpEndIx ~= nil then
        -- Courseplay turn points: over once it no longer turns and drives its course from the green
        -- point on. No shortcut on a straight course ahead any more (seen in game: it ended at 23 deg, before its turn)
        local row = CombineUnloaderMode.getCpRowTurn(combine)
        turnOver = row ~= nil and not row.turning and row.ix ~= nil and row.ix >= t.cpEndIx
        if turnOver then
            overWhy = string.format("green point %d passed, Courseplay course point %d", t.cpEndIx, row.ix)
        end
    else
        local cpAhead = CombineUnloaderMode.getCpTurnAhead(combine, FollowCombineTask.END_ROW_STRAIGHT_AHEAD)
        turnOver = not cpTurning and cpAhead ~= nil and cpAhead < FollowCombineTask.END_ROW_STRAIGHT_DEG
        if turnOver then
            overWhy = string.format("Courseplay course straight ahead: %.0f deg over %d m", cpAhead, FollowCombineTask.END_ROW_STRAIGHT_AHEAD)
        end
    end
    local done = (turned >= FollowCombineTask.END_ROW_DONE_DEG or turnOver) and (waiting or (steady and forward and not cpTurning))
    if not done and t.time < FollowCombineTask.MAX_TURN_TIME then
        return true
    end
    self.endRowTurn = nil
    self.vehicle.ad.endRowTurnActive = nil
    self.vehicle.ad.endRowTurnPause = (g_time or 0) + FollowCombineTask.END_ROW_PAUSE
    self.combineInTurn = false
    local rx, _, rz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local cx, _, cz = AutoDrive.localDirectionToWorld(combine, 0, 0, 1)
    local aligned = math.deg(math.acos(math.max(-1, math.min(1, rx * cx + rz * cz))))
    if done and aligned <= FollowCombineTask.END_ROW_ALIGNED_DEG and self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:isUnloaderOnCorrectSide() then
        self.state = FollowCombineTask.STATE_CHASING
    else
        -- original AutoDrive (STATE_WAIT_FOR_COMBINE_TO_PASS_BY): heading inverted and close, it comes back along us:
        -- wait until it has passed, the U-turn then ends behind it instead of crossing its way
        local cx, cy, cz = getWorldTranslation(combine.components[1].node)
        local _, _, offsetZ = AutoDrive.worldToLocal(self.vehicle, cx, cy, cz)
        if done and aligned >= FollowCombineTask.END_ROW_INVERTED_DEG and AutoDrive.getDistanceBetween(self.vehicle, combine) < FollowCombineTask.END_ROW_PASS_BY_RANGE
            and offsetZ > FollowCombineTask.END_ROW_PASSED_BY then
            local points, target = self:buildPassAlong(combine)
            if points ~= nil then
                local dpm = self.vehicle.ad.drivePathModule
                dpm:setWayPoints(points)
                -- starts with a reverse in its own track: driven in reverse from the start (the drive module begins
                -- every path forward and turned round toward a point behind the trailer)
                if points[2] ~= nil and points[2].isReverse then
                    dpm.isReversing = true
                    self.vehicle.ad.specialDrivingModule:reset()
                    self.vehicle.ad.specialDrivingModule.currentWayPointIndex = dpm:getCurrentWayPointIndex()
                end
                self.endRowPassAlong = {time = 0, blocked = 0}
                return true
            end
            self.endRowPassBy = {time = 0}
            return true
        end
        -- the unloader turns by itself (forward U-turn, back then U-turn, 3 points) before any route
        self.vehicle.ad.endRowUTurn = g_time or 0
        self.stayOnField = true
        self.state = FollowCombineTask.STATE_FINISHED
        return true
    end
    return false
end

-- after a wait ended on a standing harvester: true once the wait may start again (it drives and has turned
-- END_ROW_REARM_DEG since), false while it is held
FollowCombineTask.END_ROW_REARM_DEG = 10
FollowCombineTask.END_ROW_HOLD_MAX = 180000 -- ms, safety
function FollowCombineTask:updateEndRowHold()
    local hold = self.vehicle.ad.endRowHold
    if hold == nil then
        return true
    end
    local heading = FollowCombineTask.getCombineHeading(self.combine)
    local turnedSince = math.abs(heading - hold.heading) % 360
    if turnedSince > 180 then
        turnedSince = 360 - turnedSince
    end
    local driving = math.abs(self.combine.lastSpeedReal or 0) > FollowCombineTask.TURN_STOPPED_SPEED
    if driving and turnedSince >= FollowCombineTask.END_ROW_REARM_DEG then
        -- the new row entry read before the release serves again (not readable any more once it is in its turn)
        self.heldRowEntry = hold.entry
        self.vehicle.ad.endRowHold = nil
        return true
    end
    if (g_time or 0) - hold.t > FollowCombineTask.END_ROW_HOLD_MAX then
        self.vehicle.ad.endRowHold = nil
        return true
    end
    return false
end

function FollowCombineTask.getCombineHeading(combine)
    local dx, _, dz = localDirectionToWorld(combine.components[1].node, 0, 0, 1)
    if combine.ad ~= nil and combine.ad.isReverseAttached then
        dx, dz = -dx, -dz
    end
    return math.deg(math.atan2(dx, dz))
end

-- latched turn state: starts when the original detection (AutoDrive.combineIsTurning) sees a turn,
-- ends only when the combine drives forward for TURN_END_FORWARD_TIME with a steady heading
function FollowCombineTask:updateCombineTurn(dt)
    local heading = FollowCombineTask.getCombineHeading(self.combine)
    self.turnHeadingSampleTimer = self.turnHeadingSampleTimer - dt
    if self.turnHeadingSampleTimer <= 0 then
        self.turnHeadingSampleTimer = 500
        table.insert(self.turnHeadingSamples, heading)
        local maxSamples = FollowCombineTask.TURN_HEADING_SAMPLE_TIME / 500 + 1
        while #self.turnHeadingSamples > maxSamples do
            table.remove(self.turnHeadingSamples, 1)
        end
    end
    if not AutoDrive.combineIsTurning(self.combine) then
        self.turnFlagIgnored = false
    end
    -- read by the place choice (CombineUnloaderMode:getPipeChasePosition): flag ignored, the place stays beside
    self.vehicle.ad.cpTurnFlagIgnored = self.turnFlagIgnored and not self.combineInTurn
    local chopper = AutoDrive.isChopperUnloading(self.vehicle)
    -- forage harvester only: standing still that long it is not turning, it waits for this trailer: end the wait
    -- (and do not start it again while it stands)
    local stoppedTime = self.combine.ad.noMovementTimer ~= nil and self.combine.ad.noMovementTimer.elapsedTime or 0
    if not chopper or stoppedTime < FollowCombineTask.TURN_STOPPED_RELEASE_TIME then
        self.turnReleasedStopped = false
    elseif self.combineInTurn then
        -- safety net: the chase goes on, with its strict line check (a blocked line hands over to a planned route)
        self.combineInTurn = false
        self.turnReleasedStopped = true
    end
    if AutoDrive.combineIsTurning(self.combine) then
        if not self.combineInTurn then
            -- the original detection also says "turning" when less than 20 m of crop is left in front, while the
            -- combine still harvests straight on: the turn only starts when it really stops, reverses or turns
            local reason, flagIgnored = FollowCombineTask.getTurnStartReason(self.combine, heading, self.turnHeadingSamples, chopper)
            if self.turnReleasedStopped then
                return
            end
            if reason == nil then
                if flagIgnored ~= nil and not self.turnFlagIgnored then
                    self.turnFlagIgnored = true
                end
                self.vehicle.ad.cpTurnFlagIgnored = self.turnFlagIgnored
                return
            end
            self.turnFlagIgnored = false
            self.vehicle.ad.cpTurnFlagIgnored = false
        end
        self.combineInTurn = true
        return
    end
    if not self.combineInTurn then
        return
    end
    local forward = self.combine.ad.driveForwardTimer.elapsedTime >= FollowCombineTask.TURN_END_FORWARD_TIME
    local steady = #self.turnHeadingSamples > FollowCombineTask.TURN_HEADING_SAMPLE_TIME / 500
    if steady then
        local diff = math.abs(heading - self.turnHeadingSamples[1]) % 360
        if diff > 180 then
            diff = 360 - diff
        end
        steady = diff <= FollowCombineTask.TURN_END_MAX_HEADING_CHANGE
    end
    if forward and steady then
        self.combineInTurn = false
    end
end

-- forage harvester driven by Courseplay turning toward our side: we are inside its turn. Instead of staying beside
-- the cab (it comes at us) the unloader takes the 3/4 rear place on its side, backing up only as much as needed.
-- Ends when the harvester drives forward again on a steady heading. Outside of its turn nothing changes.
function FollowCombineTask:updateInnerTurn(dt)
    local combine = self.combine
    local usable = AutoDrive.isChopperUnloading(self.vehicle) and combine.ad.isChopper and combine.ad.isAutoAimingChopper
        and not combine.ad.isFixedPipeChopper and AutoDrive:getIsCPActive(combine) and not self.filled
    if not usable then
        if self.innerTurn then
            self:endInnerTurn("no longer applicable")
        end
        return
    end
    local samples = self.turnHeadingSamples
    if #samples < 2 then
        return
    end
    local h = math.rad(FollowCombineTask.getCombineHeading(combine))
    local h0 = math.rad(samples[1])
    local fx, fz = math.sin(h), math.cos(h)
    local diff = math.deg(math.abs(AutoDrive.normalizeAngle(h - h0)))
    if diff > 180 then
        diff = 360 - diff
    end
    if self.innerTurn then
        self.innerTurnTime = self.innerTurnTime + dt
        local steady = #samples > FollowCombineTask.TURN_HEADING_SAMPLE_TIME / 500 and diff <= FollowCombineTask.TURN_END_MAX_HEADING_CHANGE
        local forward = combine.ad.driveForwardTimer.elapsedTime >= FollowCombineTask.TURN_END_FORWARD_TIME
        if steady and forward and not AutoDrive:getIsCPTurning(combine) then
            self:endInnerTurn("harvester aligned and driving forward, back to the normal point")
        elseif self.innerTurnTime > FollowCombineTask.INNER_TURN_MAX_TIME then
            self:endInnerTurn("too long, back to the normal behaviour")
        end
        return
    end
    local turning = self.combineInTurn or AutoDrive:getIsCPTurning(combine)
    if self.endRowTurn ~= nil or not turning or diff < FollowCombineTask.INNER_TURN_MIN_DEG then
        return
    end
    -- 3/4 only in bends under CP_TURN_MAX_DEG
    local cpAngle = CombineUnloaderMode.getCpTurnAhead(combine, 30)
    if cpAngle ~= nil and cpAngle >= CombineUnloaderMode.CP_TURN_MAX_DEG then
        return
    end
    -- our side of the harvester and the way it turns, both as 2D cross products (same sign = toward us)
    local cx, _, cz = getWorldTranslation(combine.components[1].node)
    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
    local dx, dz = vx - cx, vz - cz
    local along = dx * fx + dz * fz
    local lateral = fx * dz - fz * dx
    local turn = math.sin(h0) * fz - math.cos(h0) * fx
    local halfWidth = (combine.size ~= nil and combine.size.width or 4) / 2
    local length = combine.size ~= nil and combine.size.length or 8
    if math.abs(lateral) < halfWidth or along < -length or lateral * turn <= 0 then
        return
    end
    self.innerTurn = true
    self.innerTurnTime = 0
end

function FollowCombineTask:endInnerTurn(reason)
    self.innerTurn = false
    self.angleWrongTimer:timer(false)
    self.lineClear = true
    self.lineBlockedTimer:timer(false)
end

-- 3/4 rear place on our side: the usual side place moved back by INNER_TURN_REAR_RATIO of the harvester length
function FollowCombineTask:getInnerTurnPoint()
    local combine = self.combine
    local mode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
    local h = math.rad(FollowCombineTask.getCombineHeading(combine))
    local fx, fz = math.sin(h), math.cos(h)
    local cx, cy, cz = getWorldTranslation(combine.components[1].node)
    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
    local dx, dz = vx - cx, vz - cz
    local along = dx * fx + dz * fz
    local ux, uz = dx - fx * along, dz - fz * along
    local ul = math.sqrt(ux * ux + uz * uz)
    if ul < 0.1 then
        return nil
    end
    ux, uz = ux / ul, uz / ul
    local length = combine.size ~= nil and combine.size.length or 8
    local side = mode:getSideChaseOffsetX()
    local back = mode:getSideChaseOffsetZ(AutoDrive.dynamicChaseDistance or combine.ad.isHarvester) - 2 - length * FollowCombineTask.INNER_TURN_REAR_RATIO
    local x, z = cx + fx * back + ux * side, cz + fz * back + uz * side
    return {x = x, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, cy, z), z = z}
end

-- standing crop just behind the train (the strip it would back into)
function FollowCombineTask:isCropBehindTrain()
    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
    local rx, _, rz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local trainLength = math.max(AutoDrive.getTractorTrainLength(self.vehicle, true, false), 8)
    local half = self.vehicle.ad.pathFinderModule:getTrainWidth() / 2
    local sx, sz = -rz * half, rx * half
    local ax, az = vx - rx * trainLength, vz - rz * trainLength
    local bx, bz = ax - rx * FollowCombineTask.INNER_TURN_REAR_CHECK, az - rz * FollowCombineTask.INNER_TURN_REAR_CHECK
    for _, fruitType in pairs(g_fruitTypeManager:getFruitTypes()) do
        if fruitType.index ~= FruitType.MEADOW and fruitType.index ~= FruitType.GRASS then
            local ratio = PathFinderModule.getFruitRatio(fruitType.index, ax + sx, az + sz, bx + sx, bz + sz, ax - sx, az - sz)
            if ratio > FollowCombineTask.STRICT_LINE_FRUIT_RATIO then
                -- isolated plants, a narrow strip or the harvester's trail do not stop it (same rule
                -- as the chase line)
                if not FollowCombineTask.isOnlyIgnoredCropIn(self.vehicle, ax + sx, az + sz, bx + sx, bz + sz, ax - sx, az - sz) then
                    return true
                end
            end
        end
    end
    return false
end

-- crop in the parallelogram only made of isolated plants, narrow strips or crop on the harvester's own trail
-- (none of them stops the chase)
function FollowCombineTask.isOnlyIgnoredCropIn(vehicle, c1X, c1Z, c2X, c2Z, c3X, c3Z)
    local unloadMode = vehicle.ad.modes ~= nil and vehicle.ad.modes[AutoDrive.MODE_UNLOAD] or nil
    local ux, uz = c2X - c1X, c2Z - c1Z
    local vx, vz = c3X - c1X, c3Z - c1Z
    local nu = math.max(1, math.ceil(math.sqrt(ux * ux + uz * uz) / PathFinderModule.ISLAND_SAMPLE))
    local nv = math.max(1, math.ceil(math.sqrt(vx * vx + vz * vz) / PathFinderModule.ISLAND_SAMPLE))
    for i = 0, nu do
        for j = 0, nv do
            local x, z = c1X + ux * i / nu + vx * j / nv, c1Z + uz * i / nu + vz * j / nv
            if PathFinderModule.getMapCropRawAt(x, z) ~= 0 and not PathFinderModule.isIsolatedCrop(x, z)
                and not (unloadMode ~= nil and unloadMode.isOnCombineTrail ~= nil and unloadMode:isOnCombineTrail(x, z)) then
                return false
            end
        end
    end
    return true
end

function FollowCombineTask:driveInnerTurn(dt)
    local point = self:getInnerTurnPoint()
    if point == nil then
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return
    end
    self.chasePos = point
    local _, _, diffZ = AutoDrive.worldToLocal(self.vehicle, point.x, point.y, point.z)
    if diffZ < -FollowCombineTask.INNER_TURN_REVERSE_TOLERANCE then
        -- the place is behind: back up just as much as needed, never into the crop
        if self:isCropBehindTrain() then
            self.vehicle.ad.specialDrivingModule:stopVehicle()
            self.vehicle.ad.specialDrivingModule:update(dt)
            return
        end
        self.vehicle.ad.specialDrivingModule:driveReverse(dt, FollowCombineTask.INNER_TURN_REVERSE_SPEED, 1, self.vehicle.ad.trailerModule:canBeHandledInReverse())
        return
    end
    -- forward: same line check as the chase, a blocked line only stops the unloader here
    self.lineCheckTimer = self.lineCheckTimer - dt
    if self.lineCheckTimer < 0 then
        self.lineCheckTimer = FollowCombineTask.LINE_CHECK_INTERVAL
        local clear, reason = FollowCombineTask.isChaseLineClear(self.vehicle, self.combine, point, self.chaseSide)
        self.lineClear = clear
    end
    if not self.lineClear then
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return
    end
    local combineSpeed = self.combine.lastSpeedReal * 3600
    self.vehicle.ad.specialDrivingModule:driveToPoint(dt, point, combineSpeed, false, 1, math.max(combineSpeed + 10, 8))
end

-- combine driving backwards
function FollowCombineTask.isCombineReversing(combine)
    local movingDirection = combine.movingDirection or 1
    if combine.ad ~= nil and combine.ad.isReverseAttached then
        movingDirection = -movingDirection
    end
    return (combine.lastSpeedReal or 0) * movingDirection <= -0.00005
end

-- why the turn really starts, nil while the combine still drives straight on
function FollowCombineTask.getTurnStartReason(combine, heading, samples, chopper)
    -- Courseplay / helper may report a maneuver while the combine still harvests straight on:
    -- their flag only counts when the combine really reverses, stops or changes heading
    local flag = nil
    if AutoDrive:getIsCPTurning(combine) then
        flag = "Courseplay turning"
    else
        local rootVehicle = combine.getRootVehicle ~= nil and combine:getRootVehicle() or nil
        if rootVehicle ~= nil and rootVehicle.getAIFieldWorkerIsTurning ~= nil and rootVehicle:getAIFieldWorkerIsTurning() then
            flag = "helper turning"
        end
    end
    if flag ~= nil and not chopper then
        -- combine harvester: the flag alone starts the wait (behaviour before the forage harvester changes)
        return flag
    end
    local prefix = flag ~= nil and (flag .. ", ") or ""
    if FollowCombineTask.isCombineReversing(combine) then
        return prefix .. "reversing"
    end
    -- forage harvester standing still in a straight line: it waits for the trailer, not a turn
    if (combine.lastSpeedReal or 0) < FollowCombineTask.TURN_STOPPED_SPEED and not chopper then
        return prefix .. "arretee"
    end
    if samples ~= nil and #samples > 0 then
        local diff = math.abs(heading - samples[1]) % 360
        if diff > 180 then
            diff = 360 - diff
        end
        if diff > FollowCombineTask.TURN_END_MAX_HEADING_CHANGE then
            return prefix .. string.format("turned %.0f deg", diff)
        end
    end
    return nil, flag
end

function FollowCombineTask.getLogName(vehicle)
    return vehicle ~= nil and vehicle.getName ~= nil and vehicle:getName() or "?"
end

-- the chase drives straight to the chase point without a route: the straight line must be free of
-- buildings, trees and learned blocked zones (vehicles are ignored), never leave the field nor drive onto another
-- field, nor cross standing crop, whatever the setting. Returns clear, reason
-- small patch off the field inside it (power pole): field ground within ENCLAVE_RADIUS in every direction.
-- The pole itself stays an obstacle for the box check
FollowCombineTask.ENCLAVE_RADIUS = 10
FollowCombineTask.BORDER_BITE = 3 -- m: field ground this close beside an off-field point of the line = the border bitten

-- off-field point of the chase line with field ground within BORDER_BITE m across the line: the line only bites the
-- border . Returns the distance to the field ground,
-- nil when there is none
function FollowCombineTask.getBorderBite(x, z, nx, nz)
    for d = 1, FollowCombineTask.BORDER_BITE do
        for _, side in ipairs({-1, 1}) do
            if AutoDrive.checkIsOnField(x + nx * d * side, 0, z + nz * d * side) then
                return d
            end
        end
    end
    return nil
end

function FollowCombineTask.isFieldEnclave(x, z)
    for k = 0, 7 do
        local a = k * math.pi / 4
        local dx, dz = math.sin(a), math.cos(a)
        local found = false
        for d = 2, FollowCombineTask.ENCLAVE_RADIUS, 2 do
            if AutoDrive.checkIsOnField(x + dx * d, 0, z + dz * d) then
                found = true
                break
            end
        end
        if not found then
            return false
        end
    end
    return true
end

function FollowCombineTask.isChaseLineClear(vehicle, combine, chasePos, chaseSide)
    if vehicle == nil or combine == nil or chasePos == nil then
        return true
    end
    local vx, _, vz = getWorldTranslation(vehicle.components[1].node)
    local dirX, dirZ = chasePos.x - vx, chasePos.z - vz
    local length = math.sqrt(dirX * dirX + dirZ * dirZ)
    -- strict: the whole line up to the chase point is checked, at least STRICT_LINE_MIN_LENGTH ahead, any real crop stops
    local strict = AutoDrive.getSetting("avoidFruit", vehicle) == "strict" and AutoDrive.isChopperUnloading(vehicle)
    local skipEnd = strict and 0 or FollowCombineTask.LINE_SKIP_END
    if strict and length < FollowCombineTask.STRICT_LINE_MIN_LENGTH then
        if length < 0.1 then
            local rx, _, rz = AutoDrive.localDirectionToWorld(vehicle, 0, 0, 1)
            dirX, dirZ, length = rx, rz, 1
        end
        dirX, dirZ = dirX / length * FollowCombineTask.STRICT_LINE_MIN_LENGTH, dirZ / length * FollowCombineTask.STRICT_LINE_MIN_LENGTH
        length = FollowCombineTask.STRICT_LINE_MIN_LENGTH
    end
    local maxRatio = strict and FollowCombineTask.STRICT_LINE_FRUIT_RATIO or PathFinderModule.FRUIT_MAX_RATIO
    local checkLength = length - FollowCombineTask.LINE_SKIP_START - skipEnd
    if checkLength < 1 then
        return true
    end
    dirX, dirZ = dirX / length, dirZ / length
    local a = {x = vx + dirX * FollowCombineTask.LINE_SKIP_START, z = vz + dirZ * FollowCombineTask.LINE_SKIP_START}
    local b = {x = a.x + dirX * checkLength, z = a.z + dirZ * checkLength}
    local pathFinder = vehicle.ad.pathFinderModule
    local halfWidth = pathFinder:getTrainWidth() / 2 + FollowCombineTask.LINE_WIDTH_MARGIN
    local height = PathFinderModule.getTrainHeight(vehicle) + PathFinderModule.OBSTACLE_HEIGHT_MARGIN

    -- obstacles: same box as the pathfinder's route check
    local angleRad = AutoDrive.normalizeAngle(math.atan2(-dirZ, dirX))
    local midX, midZ = (a.x + b.x) / 2, (a.z + b.z) / 2
    local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, midX, 1, midZ)
    local probe = {hits = 0}
    probe.callback = FollowCombineTask.lineCollisionCallback
    overlapBox(midX, y + height / 2, midZ, 0, angleRad, 0, checkLength / 2, height / 2, halfWidth, "callback", probe, AutoDrive.collisionMaskTerrain, true, true, true, true)
    if probe.hits > 0 then
        return false, "obstacle"
    end
    if pathFinder:boxHitsBlockedZone(midX, midZ, angleRad, checkLength / 2, halfWidth) then
        return false, "blocked zone"
    end

    local cx, _, cz = getWorldTranslation(combine.components[1].node)
    local combineField = PathFinderModule.getFieldAtPosition(cx, cz)
    -- field of the harvester: its connected field ground, as the pathfinder does in strict mode (map outlines of
    -- two neighbour fields can overlap at the border)
    local combineArea = PathFinderModule.getConnectedFieldArea(cx, cz)
    if combineArea ~= nil and combineArea.capped then
        combineArea = nil
    end
    local function isOtherField(x, z)
        if combineArea ~= nil then
            return not PathFinderModule.isInFieldArea(combineArea, x, z)
        end
        local field = combineField ~= nil and PathFinderModule.getFieldAtPosition(x, z) or nil
        return field ~= nil and field ~= combineField
    end
    local unloadMode = vehicle.ad.modes ~= nil and vehicle.ad.modes[AutoDrive.MODE_UNLOAD] or nil
    local sideX, sideZ = -dirZ * (halfWidth - FollowCombineTask.LINE_WIDTH_MARGIN), dirX * (halfWidth - FollowCombineTask.LINE_WIDTH_MARGIN)
    local slices = math.max(1, math.ceil(checkLength / 4))
    local cropSlices, cropName = {}, nil
    for k = 0, slices do
        local px, pz = a.x + (b.x - a.x) * k / slices, a.z + (b.z - a.z) * k / slices
        -- same rule as the pathfinder along the border (strict): the centre stays on the field of the harvester,
        -- the sides of the train may be on the grass beside it, never on another field
        -- the harvester drove there: same ground, the tractor may follow (CP turn beyond the border, or the outline
        -- of a neighbour field overlapping its track)
        local onTrail = unloadMode ~= nil and unloadMode.isOnCombineTrail ~= nil and unloadMode:isOnCombineTrail(px, pz)
        local borderBite = false
        if not AutoDrive.checkIsOnField(px, 0, pz) and not FollowCombineTask.isFieldEnclave(px, pz) then
            if not onTrail then
                --: the line may bite the border (field
                -- ground within BORDER_BITE m beside the point; the obstacles were checked above). An unloader stood
                -- 40 m behind the standing harvester, aligned, and gave the chase up for an axis 1-2 m over the border
                local bite = FollowCombineTask.getBorderBite(px, pz, -dirZ, dirX)
                if bite == nil then
                    return false, "off the field"
                end
                borderBite = true
            end
        end
        if not onTrail and not borderBite and isOtherField(px, pz) then
            return false, "other field"
        end
        for _, side in ipairs({-1, 1}) do
            local sx, sz = px + sideX * side, pz + sideZ * side
            if not onTrail and AutoDrive.checkIsOnField(sx, 0, sz) and isOtherField(sx, sz) then
                return false, "other field"
            end
        end
        -- standing crop: same rule as the route check, more than FRUIT_MAX_RATIO of a 4 m stretch at the train width
        -- crop on the harvester's own trail does not stop the tractor
        if k < slices and not onTrail then
            local ex, ez = a.x + (b.x - a.x) * (k + 1) / slices, a.z + (b.z - a.z) * (k + 1) / slices
            for _, fruitType in pairs(g_fruitTypeManager:getFruitTypes()) do
                if fruitType.index ~= FruitType.MEADOW and fruitType.index ~= FruitType.GRASS then
                    local ratio = PathFinderModule.getFruitRatio(fruitType.index, px + sideX, pz + sideZ, ex + sideX, ez + sideZ, px - sideX, pz - sideZ)
                    if ratio > maxRatio and PathFinderModule.isOnlyIsolatedCropIn(px + sideX, pz + sideZ, ex + sideX, ez + sideZ, px - sideX, pz - sideZ) then
                        -- isolated plants / leftover strip on the chase line: ignored
                        ratio = 0
                    end
                    if ratio > maxRatio then
                        table.insert(cropSlices, {k = k, ratio = ratio})
                        cropName = fruitType.name
                        break
                    end
                end
            end
        end
    end
    if #cropSlices > 0 then
        -- the tip of a right-angle corner may be cut. Only that: crop on at most CORNER_CUT_SLICES consecutive
        -- 4 m slices, each partly covered (the corner point, not a crop block across the line)
        local tip = #cropSlices <= FollowCombineTask.CORNER_CUT_SLICES
        for i, c in ipairs(cropSlices) do
            if c.ratio > FollowCombineTask.CORNER_CUT_MAX_RATIO or (i > 1 and c.k ~= cropSlices[i - 1].k + 1) then
                tip = false
            end
        end
        --: a narrow band of crop across
        -- the line (one 4 m slice, clear after it up to the place) is crossed
        local band = #cropSlices == 1 and cropSlices[1].k < slices - 1
        if not tip and band then
            return true
        end
        if not tip then
            return false, "crop " .. tostring(cropName)
        end
    end
    return true
end

-- the forage harvester drives toward this unloader: moving, and the unloader stands ahead of it in its lane
FollowCombineTask.COMING_MIN_SPEED = 1 -- km/h
FollowCombineTask.COMING_LANE_MARGIN = 2 -- m beside the harvester and the train
function FollowCombineTask:isChopperComingAtUs()
    local combine = self.combine
    if combine == nil or combine.components == nil then
        return true
    end
    -- real motion of the harvester (positions over time): its node axes are not reliable for every harvester
    local cx, _, cz = getWorldTranslation(combine.components[1].node)
    local now = g_time or 0
    local m = self.chopperMotion
    if m == nil or now - m.t > 3000 then
        self.chopperMotion = {x = cx, z = cz, t = now, vx = 0, vz = 0}
        return false
    end
    if now - m.t >= 500 then
        local dt = (now - m.t) / 1000
        m.vx, m.vz = (cx - m.x) / dt, (cz - m.z) / dt
        m.x, m.z, m.t = cx, cz, now
    end
    local speed = math.sqrt(m.vx * m.vx + m.vz * m.vz) * 3.6
    if speed < FollowCombineTask.COMING_MIN_SPEED then
        return false
    end
    local rx, rz = m.vx / (speed / 3.6), m.vz / (speed / 3.6)
    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
    local dx, dz = vx - cx, vz - cz
    local ahead = dx * rx + dz * rz
    if ahead <= 0 then
        return false
    end
    local lateral = math.abs(dx * rz - dz * rx)
    local combineHalf = (combine.size ~= nil and combine.size.width or 4) / 2
    local trainHalf = (self.vehicle.size ~= nil and self.vehicle.size.width or 3) / 2
    return lateral <= combineHalf + trainHalf + FollowCombineTask.COMING_LANE_MARGIN
end

FollowCombineTask.CORNER_CUT_SLICES = 2 -- 4 m slices of crop allowed on the chase line (tip of a corner)
FollowCombineTask.CORNER_CUT_MAX_RATIO = 0.5 -- each partly covered only

function FollowCombineTask.lineCollisionCallback(probe, transformId)
    if transformId ~= 0 and transformId ~= g_currentMission.terrainRootNode then
        -- vehicles are left to the chase as before (combine, other unloaders), only fixed objects count
        local object = g_currentMission:getNodeObject(transformId)
        if object == nil or object.rootVehicle == nil then
            probe.hits = probe.hits + 1
        end
    end
    return true
end

function FollowCombineTask:shouldWaitForChasePos(dt)
    -- jog, the place behind the harvester came ahead of us but 7 m aside (> 50 deg): drive to it
    local mode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
    if self.combine.ad.isChopper and mode ~= nil and mode:isRearChaseHeld() then
        local _, _, ahead = AutoDrive.worldToLocal(self.vehicle, self.chasePos.x, self.chasePos.y, self.chasePos.z)
        if ahead > 0.5 then
            self.angleWrongTimer:timer(false)
            return false
        end
    end
    -- (3 times at the same place): the harvester stands and waits for this trailer, its place is
    -- ahead but more than 50 deg aside: waiting here waits for each other forever, drive to it
    if self.combine.ad.isChopper and math.abs(self.combine.lastSpeedReal or 0) < 0.0003 then
        local _, _, ahead = AutoDrive.worldToLocal(self.vehicle, self.chasePos.x, self.chasePos.y, self.chasePos.z)
        if ahead > 0.5 then
            self.angleWrongTimer:timer(false)
            return false
        end
    end
    local angle = self:getAngleToChasePos()
    self.angleWrongTimer:timer(angle > 50, 3000, dt)
    local _, _, diffZ = AutoDrive.worldToLocal(self.vehicle, self.chasePos.x, self.chasePos.y, self.chasePos.z)
    return self.angleWrongTimer:done() or  diffZ <= -1 --or (not self.combine.ad.sensors.frontSensorFruit:pollInfo())
end

function FollowCombineTask:isCaughtCurrentChaseSide()
    local caught = false
    local angle = self:getAngleToChasePos()
    local vehicleX, vehicleY, vehicleZ = getWorldTranslation(self.vehicle.components[1].node)
    local _, _, diffZ = AutoDrive.worldToLocal(self.vehicle, self.chasePos.x, self.chasePos.y, self.chasePos.z)

    local diffX, _, _ = AutoDrive.worldToLocal(self.combine, vehicleX, vehicleY, vehicleZ, self.combine.ad.ADRootNode)
    if ((angle < 15 and diffZ >= 0) or (angle > 165 and diffZ < 0)) and (self.angleToCombineHeading < 15) and (AutoDrive.sign(diffX) == self.chaseSide or self.chaseSide == AutoDrive.CHASEPOS_REAR) then
        caught = true
    end
    return caught
end


function FollowCombineTask:getAngleToCombineHeading()
    if self.vehicle == nil or self.combine == nil then
        return math.huge
    end

    local combineRx, _, combineRz = AutoDrive.localDirectionToWorld(self.combine, 0, 0, 1, self.combine.ad.ADRootNode)
    local rx, _, rz =  AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)

    return math.abs(AutoDrive.angleBetween({x = rx, z = rz}, {x = combineRx, z = combineRz}))
end

function FollowCombineTask:getAngleToChasePos()
    local worldX, _, worldZ = getWorldTranslation(self.vehicle.components[1].node)
    local rx, _, rz =  AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local angle = math.abs(AutoDrive.angleBetween({x = rx, z = rz}, {x = self.chasePos.x - worldX, z = self.chasePos.z - worldZ}))
    return angle
end

function FollowCombineTask:getAngleToCobine()
    local worldX, _, worldZ = getWorldTranslation(self.vehicle.components[1].node)
    local rx, _, rz =  AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local referenceAxis = self.combine.components[1].node
    if self.combine.components[2] ~= nil and self.combine.components[2].node ~= nil then
        referenceAxis = self.combine.components[2].node
    end

    local combineX, _, combineZ = getWorldTranslation(referenceAxis)
    local angle = math.abs(AutoDrive.angleBetween({x = rx, z = rz}, {x = combineX - worldX, z = combineZ - worldZ}))
    return angle
end

function FollowCombineTask:abort()
    self.vehicle.ad.endRowTurnActive = nil
    self.vehicle.ad.cpTurnFlagIgnored = nil
end

function FollowCombineTask:finished()
    self.vehicle.ad.endRowTurnActive = nil
    self.vehicle.ad.cpTurnFlagIgnored = nil
    FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:finished()")
    self.vehicle.ad.taskModule:setCurrentTaskFinished()
end

function FollowCombineTask:getExcludedVehiclesForCollisionCheck()
    local excludedVehicles = {}
    if self.state == FollowCombineTask.STATE_CHASING or self.state == FollowCombineTask.STATE_WAIT_FOR_TURN then
        table.insert(excludedVehicles, self.combine:getRootVehicle())
    end
    return excludedVehicles
end

function FollowCombineTask:setStateNames()
    if self.statesToNames == nil then
        self.statesToNames = {}
        for name, id in pairs(FollowCombineTask) do
            if string.sub(name, 1, 6) == "STATE_" then
                self.statesToNames[id] = name
            end
        end
    end
end

function FollowCombineTask:getStateName(state)
    local requestedState = state
    if requestedState == nil then
        requestedState = self.state
    end
    if requestedState == nil then
        Logging.error("[AD] FollowCombineTask: Could not find name for state ->%s<- !", tostring(requestedState))
    end
    return self.statesToNames[requestedState] or ""
end

function FollowCombineTask:resetAllTimers()
    -- self.stuckTimer:timer(false) -- stuckTimer reset by speed changes
    self.angleWrongTimer:timer(false)
    self.waitForTurnTimer:timer(false)
    self.dischargeTimer:timer(false)
    self.fillingTimer:timer(false)
    self.waitForPassByTimer:timer(false)
    self.chaseTimer:timer(false)
    self.reverseTimer:timer(false)
    self.waitTimer:timer(false)
end


function FollowCombineTask:getI18nInfo()
    local harvester = AbstractTask.getHarvesterI18n(self.combine)
    if self.state == FollowCombineTask.STATE_CHASING then
        if self.infoJogWaitAt ~= nil and (g_time or 0) - self.infoJogWaitAt < 1000 then
            return "$l10n_AD_task_wait_overtake_1; " .. harvester .. " $l10n_AD_task_wait_overtake_2;"
        end
        if not self:isCaughtCurrentChaseSide() then
            if self.chaseSide == AutoDrive.CHASEPOS_LEFT then
                return "$l10n_AD_task_join_side_left; " .. harvester
            elseif self.chaseSide == AutoDrive.CHASEPOS_RIGHT then
                return "$l10n_AD_task_join_side_right; " .. harvester
            end
            return "$l10n_AD_task_join_side_rear; " .. harvester
        end
        local side = "$l10n_AD_task_side_rear;"
        if self.chaseSide == AutoDrive.CHASEPOS_LEFT then
            side = "$l10n_AD_task_side_left;"
        elseif self.chaseSide == AutoDrive.CHASEPOS_RIGHT then
            side = "$l10n_AD_task_side_right;"
        end
        return "$l10n_AD_task_chase; " .. harvester .. " " .. side
    elseif self.state == FollowCombineTask.STATE_REVERSING_FROM_CHOPPER then
        return "$l10n_AD_task_backoff; " .. harvester .. " $l10n_AD_task_is_turning;"
    elseif self.state == FollowCombineTask.STATE_WAIT_FOR_TURN then
        return "$l10n_AD_task_waiting_comma; " .. harvester .. " $l10n_AD_task_is_turning;"
    elseif self.state == FollowCombineTask.STATE_TRACK_REVERSE then
        return "$l10n_AD_task_reverse_tracks_clear;"
    elseif self.state == FollowCombineTask.STATE_BACK_TO_PLACE then
        return "$l10n_AD_task_back_to_place;"
    elseif self.state == FollowCombineTask.STATE_REVERSING then
        return "$l10n_AD_task_reversing_from_combine;"
    elseif self.state == FollowCombineTask.STATE_WAIT_FOR_PASS_BY or self.state == FollowCombineTask.STATE_WAIT_FOR_COMBINE_TO_PASS_BY then
        return "$l10n_AD_task_let_pass; " .. harvester
    elseif self.state == FollowCombineTask.STATE_GENERATE_UTURN_PATH or self.state == FollowCombineTask.STATE_DRIVE_UTURN_PATH then
        return "$l10n_AD_task_uturn;"
    elseif self.state == FollowCombineTask.STATE_WAIT_BEFORE_FINISH then
        return "$l10n_AD_task_waiting_for_room;"
    end
    return "$l10n_AD_task_chase; " .. harvester
end

function FollowCombineTask.debugMsg(vehicle, debugText, ...)
    if FollowCombineTask.debug == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    else
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_COMBINEINFO, debugText, ...)
    end
end

