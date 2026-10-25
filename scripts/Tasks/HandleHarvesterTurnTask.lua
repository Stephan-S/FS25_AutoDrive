HandleHarvesterTurnTask = ADInheritsFrom(AbstractTask)

HandleHarvesterTurnTask.STATE_INITIAL_REVERSING = 1
HandleHarvesterTurnTask.STATE_GENERATE_U_TURN = 2
HandleHarvesterTurnTask.STATE_EXECUTE_U_TURN = 3
HandleHarvesterTurnTask.STATE_STRAIGHTEN_TRAILER = 4
HandleHarvesterTurnTask.REVERSE_TO_COMBINE = 5
HandleHarvesterTurnTask.STATE_FINISHED = 6

HandleHarvesterTurnTask.STATE_GENERATE_TURN = 10
HandleHarvesterTurnTask.STATE_EXECUTE_TURN = 11

HandleHarvesterTurnTask.MAX_INITIAL_REVERSING_DISTANCE = 18

-- end of row: after the U-turn, harvester behind on the same heading: straight back to its pipe
HandleHarvesterTurnTask.STATE_REVERSE_TO_PIPE = 12
HandleHarvesterTurnTask.PIPE_REVERSE_ANGLE = 30 -- deg between tractor and harvester headings
HandleHarvesterTurnTask.PIPE_REVERSE_RANGE = 90 -- m
HandleHarvesterTurnTask.PIPE_REVERSE_TIME = 90000
HandleHarvesterTurnTask.PIPE_REVERSE_BLOCKED = 6000
HandleHarvesterTurnTask.PIPE_REVERSE_SPEED = 12
HandleHarvesterTurnTask.PIPE_WAIT_TIME = 60000 -- ms waiting for the harvester where the reverse stopped
HandleHarvesterTurnTask.PIPE_REVERSE_WIDTH = 0.7 -- share of the axle width checked on the straight line back
--: started while the harvester was still turning (aligned for a moment),
-- its pipe place then jumped to its other side and the reverse ended 56 m short. The reverse waits for the end of
-- the harvester's turn, keeps the place on its starting side, ends only beside it, and starts again if the
-- harvester turns once more
HandleHarvesterTurnTask.STATE_WAIT_COMBINE_TURN = 13
HandleHarvesterTurnTask.TURN_WAIT_MAX = 60000 -- ms waiting for the end of the harvester's turn
HandleHarvesterTurnTask.TURN_STEADY_TIME = 2000 -- ms of steady heading (and no Courseplay turn) = turn finished
HandleHarvesterTurnTask.TURN_STEADY_DEG = 4
HandleHarvesterTurnTask.PIPE_BESIDE_GAP = 6 -- m aside at most to count as beside the pipe place
HandleHarvesterTurnTask.PIPE_REVERSE_TURNED = 20 -- deg the harvester turns during the reverse: stop and plan again
HandleHarvesterTurnTask.PIPE_REVERSE_RESTARTS = 3

function HandleHarvesterTurnTask:new(vehicle, combine)
    local o = HandleHarvesterTurnTask:create()
    o.vehicle = vehicle
    o.combine = combine
    -- the mode reads the trailer fill node in the chase only, this task can come first
    local mode = vehicle.ad ~= nil and vehicle.ad.modes ~= nil and vehicle.ad.modes[AutoDrive.MODE_UNLOAD] or nil
    if mode ~= nil and mode.ensureTargetFillNode ~= nil then
        mode:ensureTargetFillNode()
    end
    o.state = HandleHarvesterTurnTask.STATE_GENERATE_TURN
    o.taskType = "HandleHarvesterTurnTask"
    o.nextTurnToTry = 0
    o.turnGenerated = false
    o.triedAllTurnsAfterTurnEnded = false
    -- Courseplay end of row: set when the end-of-row wait hands over
    local flag = vehicle.ad.endRowUTurn
    o.endRow = flag ~= nil and (g_time or 0) - flag < CatchCombinePipeTask.END_ROW_UTURN_AGE
    if o.endRow then
        -- order of the drawing: forward U-turn, back then U-turn, forward offset U-turn
        o.nextTurnToTry = -1
        o.endRowTries = 0
    end

    local x, y, z = getWorldTranslation(vehicle.components[1].node)
    local diffX, _, diffZ = AutoDrive.worldToLocal(combine, x, y, z, combine.ad.ADRootNode)
    local turnLeft = AutoDrive.sign(diffX) >= 0
    local targetPosition = o:getHarvesterEndTurnPosition()

    if targetPosition == nil then
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_COMBINEINFO, "HandleHarvesterTurnTask:new - Could not determine end position - aborting")
        o.aborting = true
        return o
    end

    if AutoDrive.combineIsTurning(o.combine) or o.cpRow ~= nil then
        diffX, _, diffZ = AutoDrive.worldToLocal(vehicle, targetPosition.x, targetPosition.y, targetPosition.z)
        turnLeft = AutoDrive.sign(diffX) >= 0
    end

    local dirX, _, dirZ =  AutoDrive.localDirectionToWorld(combine, 0, 0, -1)
    local targetDirZ = {x=dirX, z=dirZ}
    dirX, _, dirZ =  AutoDrive.localDirectionToWorld(combine, -1, 0, 0)
    local targetDirX = {x=dirX, z=dirZ}
    if o.cpRow ~= nil then
        -- Courseplay: the new row heading read on its course, not the harvester axes
        targetDirZ = {x = o.cpRow.dx, z = o.cpRow.dz}
        targetDirX = {x = o.cpRow.dz, z = -o.cpRow.dx}
    end

    local startPosX, startPosY, startPosZ = AutoDrive.localToWorld(vehicle, 0, 0, 0.5 + vehicle.size.length / 2)
    local startPos = { x=startPosX, y=startPosY, z=startPosZ}
    dirX, _, dirZ =  AutoDrive.localDirectionToWorld(vehicle, 0, 0, 1)
    local startDirZ = {x=dirX, z=dirZ}
    dirX, _, dirZ =  AutoDrive.localDirectionToWorld(vehicle, 1, 0, 0)
    local startDirX = {x=dirX, z=dirZ}

    o.turnParameters = {
        vehicle = vehicle,
        combine = combine,
        targetLeft = turnLeft,
        targetPosition = targetPosition,
        startPosition = startPos, -- ~0.5m ahead of vehicle,
        targetDir = { x= targetDirX, z = targetDirZ},
        startDir = { x = startDirX, z = startDirZ },
        minRadius = AutoDrive.getDriverRadius(vehicle),
        handleHarvesterTask = o,
        angle = math.abs(AutoDrive.angleBetween({x = startDirZ.x, z = startDirZ.z}, {x = targetDirZ.x, z = targetDirZ.z}))
    }

    o.currentTurn = nil

    AutoDrive.debugPrint(vehicle, AutoDrive.DC_COMBINEINFO, "HandleHarvesterTurnTask  - turnleft: " .. tostring(turnLeft) .. " angle: " .. o.turnParameters.angle)
    return o
end

function HandleHarvesterTurnTask:setUp()
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_COMBINEINFO, "HandleHarvesterTurnTask:setUp()")
    AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, true)
end

function HandleHarvesterTurnTask:reset()
    self.state = HandleHarvesterTurnTask.STATE_GENERATE_TURN
    self.generationStep = 0
    self.turnGenerated = false
end

function HandleHarvesterTurnTask:update(dt)
    if self.aborting then
        self:finished(ADTaskModule.DONT_PROPAGATE)
        self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:setToWaitForCall(true)
        return
    end
    if self.combine ~= nil and g_currentMission.nodeToObject[self.combine.components[1].node] == nil then
        self:finished()
        return
    end
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)

    if self.state == HandleHarvesterTurnTask.STATE_GENERATE_TURN then
        if self.currentTurn == nil then
            self:tryNextTurn()
            if self.endRowGaveUp then
                return
            end
        end
        local turnReady = false
        if self.currentTurn ~= nil then
            -- a Lua error while generating a turn must not stop
            -- AutoDrive every frame, the turn is dropped and the next one tried
            local ok, result = pcall(self.currentTurn.update, self.currentTurn)
            if ok then
                turnReady = result
                if not result and self.currentTurn.failed then
                    self.currentTurn = nil
                end
            else
                self.currentTurn = nil
            end
        end
        if turnReady then
            local points = self.currentTurn:getWayPoints()
            if points ~= nil and #points > 0 then
                self:cleanPoints(points)
                local finished, foundCollision = self:doCollisionCheck(points)
                if finished then
                    local groundOk, groundWhy = true, nil
                    if not foundCollision then
                        groundOk, groundWhy = self:checkGround(points)
                    end
                    if foundCollision or not groundOk then
                        self.currentTurn = nil
                    else
                        if self.endRow then
                            self.vehicle.ad.endRowUTurn = nil
                        end
                        self:continueWith(points)
                    end
                end
            else
                self.currentTurn = nil
            end
        end
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
    elseif self.state == HandleHarvesterTurnTask.STATE_EXECUTE_TURN then
        local allowedToSkip = self.vehicle.ad.drivePathModule:getCurrentWayPoint() ~= nil and self.vehicle.ad.drivePathModule:getCurrentWayPoint().allowedToSkip
        local movingDirection = self.combine.movingDirection
        if self.combine.ad.isReverseAttached then
            movingDirection = -self.combine.movingDirection
        end
        local combineMovingAgain = ((self.combine.lastSpeedReal * movingDirection) >= 0.0004)

        if self.vehicle.ad.drivePathModule:isTargetReached() or (allowedToSkip and combineMovingAgain) then
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_COMBINEINFO, "Turn finished")
            self.state = HandleHarvesterTurnTask.STATE_FINISHED
            if self.endRow then
                self:startWaitCombineTurn()
            end
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end

        if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_COMBINEINFO) then
            local r,g,b = 0,1,0
            if self.colliFound then
                r = 1
                g = 0
            end

            for i, p in ipairs(self.points) do
                if i > 1 then
                    ADDrawingManager:addLineTask(self.points[i-1].x, self.points[i-1].y, self.points[i-1].z, p.x, p.y, p.z, 1, r, g, b)
                    ADDrawingManager:addArrowTask(self.points[i-1].x, self.points[i-1].y, self.points[i-1].z, p.x, p.y, p.z, 1, ADDrawingManager.arrows.position.middle, unpack(AutoDrive.currentColors.ad_color_subPrioSingleConnection))
                end
            end
        end
    elseif self.state == HandleHarvesterTurnTask.STATE_WAIT_COMBINE_TURN then
        self:updateWaitCombineTurn(dt)
    elseif self.state == HandleHarvesterTurnTask.STATE_REVERSE_TO_PIPE then
        if self:updateReverseToPipe(dt) then
            self.state = HandleHarvesterTurnTask.STATE_FINISHED
        end
    elseif self.state == HandleHarvesterTurnTask.STATE_FINISHED then
        self:finished()
        return
    end
end

-- pipe place in the world: during the reverse on the side it started on (the side choice of the chase can jump
-- to the other side while the harvester pivots), else the chase place
function HandleHarvesterTurnTask:getPipePlace()
    local mode = self.vehicle.ad.modes ~= nil and self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD] or nil
    if mode == nil or mode.getPipeChasePosition == nil or mode.combine == nil then
        return nil
    end
    if self.pipeSide ~= nil and mode.getSideChaseOffsetX ~= nil then
        local x = self.pipeSide * (mode:getSideChaseOffsetX() + mode:getPipeSlopeCorrection())
        local z = mode:getSideChaseOffsetZ(AutoDrive.dynamicChaseDistance or self.combine.ad.isHarvester) - 2
        return AutoDrive.createWayPointRelativeToVehicle(self.combine, x, z)
    end
    return mode:getPipeChasePosition()
end

function HandleHarvesterTurnTask:getCombineHeadingDeg()
    local dx, _, dz = AutoDrive.localDirectionToWorld(self.combine, 0, 0, 1)
    return math.deg(math.atan2(dx, dz))
end

function HandleHarvesterTurnTask.headingDiff(a, b)
    local d = math.abs(a - b) % 360
    if d > 180 then
        d = 360 - d
    end
    return d
end

function HandleHarvesterTurnTask:startWaitCombineTurn()
    self.turnWait = {time = 0, steady = 0, heading = self:getCombineHeadingDeg(), sample = 0}
    self.state = HandleHarvesterTurnTask.STATE_WAIT_COMBINE_TURN
end

-- stands where the U-turn ended until the harvester has finished its own turn, then the reverse beside it
function HandleHarvesterTurnTask:updateWaitCombineTurn(dt)
    local w = self.turnWait
    local name = tostring(self.vehicle:getName())
    w.time = w.time + dt
    w.sample = w.sample + dt
    if w.sample >= 500 then
        w.sample = 0
        -- steady over the whole window (4 deg per half second let a harvester still turning pass)
        local h = self:getCombineHeadingDeg()
        if HandleHarvesterTurnTask.headingDiff(h, w.heading) <= HandleHarvesterTurnTask.TURN_STEADY_DEG then
            w.steady = w.steady + 500
        else
            w.steady = 0
            w.heading = h
        end
    end
    -- the Courseplay turn flag is also raised on straight lines: with it, a longer steady heading
    local need = AutoDrive:getIsCPTurning(self.combine) and 2 * HandleHarvesterTurnTask.TURN_STEADY_TIME or HandleHarvesterTurnTask.TURN_STEADY_TIME
    if w.steady >= need or w.time > HandleHarvesterTurnTask.TURN_WAIT_MAX then
        if self:startReverseToPipe() then
            self.state = HandleHarvesterTurnTask.STATE_REVERSE_TO_PIPE
        else
            self.state = HandleHarvesterTurnTask.STATE_FINISHED
        end
        return
    end
    self.vehicle.ad.specialDrivingModule:stopVehicle()
    self.vehicle.ad.specialDrivingModule:update(dt)
end

-- pipe position seen from the tractor (x left, z ahead), nil if unknown
function HandleHarvesterTurnTask:getPipeLocal()
    local chasePos = self:getPipePlace()
    if chasePos == nil then
        return nil
    end
    local x, _, z = AutoDrive.worldToLocal(self.vehicle, chasePos.x, chasePos.y or 0, chasePos.z)
    return x, z
end

function HandleHarvesterTurnTask:startReverseToPipe()
    local name = tostring(self.vehicle:getName())
    -- the pipe place stays on the side the tractor is on now (place chosen by the chase, else the tractor's side)
    self.pipeSide = nil
    local mode = self.vehicle.ad.modes ~= nil and self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD] or nil
    if mode ~= nil and mode.getPipeChasePosition ~= nil and mode.combine ~= nil then
        local _, chaseSide = mode:getPipeChasePosition()
        if chaseSide == AutoDrive.CHASEPOS_LEFT or chaseSide == AutoDrive.CHASEPOS_RIGHT then
            self.pipeSide = chaseSide
        else
            local vx, vy, vz = getWorldTranslation(self.vehicle.components[1].node)
            local ox = AutoDrive.worldToLocal(self.combine, vx, vy, vz, self.combine.ad.ADRootNode)
            self.pipeSide = ox >= 0 and AutoDrive.CHASEPOS_LEFT or AutoDrive.CHASEPOS_RIGHT
        end
    end
    self.pipeReverseHeading = self:getCombineHeadingDeg()
    local rx, _, rz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local cx, _, cz = AutoDrive.localDirectionToWorld(self.combine, 0, 0, 1)
    local angle = math.deg(math.acos(math.max(-1, math.min(1, (rx * cx + rz * cz) / math.max(0.001, math.sqrt((rx * rx + rz * rz) * (cx * cx + cz * cz)))))))
    local px, pz = self:getPipeLocal()
    if px == nil then
        return false
    end
    if angle > HandleHarvesterTurnTask.PIPE_REVERSE_ANGLE or pz > -2 or -pz > HandleHarvesterTurnTask.PIPE_REVERSE_RANGE then
        return false
    end
    -- reverse aligned on the harvester, with the AutoDrive tools (reverse waypoints driven by
    -- the drive module, as the original reverse-to-combine section): parallel to it at the pipe distance, a
    -- gentle S at the start takes up the side gap
    self.rowLine = self:getRowLine()
    local aligned, alignedWhy = self:buildAlignedReverse()
    if aligned == nil and alignedWhy ~= nil and string.sub(alignedWhy, 1, 4) == "crop" then
        --: the original
        -- reverse-to-combine section does not look at the crop, the parallel reverse is kept (field still checked)
        aligned, alignedWhy = self:buildAlignedReverse(true)
    end
    if aligned ~= nil then
        local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
        self.pipeReverse = {x = x, z = z, time = 0, blocked = 0, aligned = true}
        self.vehicle.ad.specialDrivingModule:reset()
        local dpm = self.vehicle.ad.drivePathModule
        dpm:setWayPoints(aligned)
        --: the drive module starts every path forward and only switches to
        -- reverse when it reaches the 2nd point, which lies behind the trailer: the tractor turned round forward
        -- toward it. This path is reverse from its first point: driven in reverse from the start
        dpm.isReversing = true
        self.vehicle.ad.specialDrivingModule:reset()
        self.vehicle.ad.specialDrivingModule.currentWayPointIndex = dpm:getCurrentWayPointIndex()
        return true
    end
    -- (seen in game: with a side gap of 55 m; an unloader: 87 m, ended 14 m aside):
    -- straight back only when it really ends at the pipe: side gap plus the drift of the heading angle small, way short
    local drift = math.abs(px) + (-pz) * math.sin(math.rad(math.min(90, angle)))
    if drift > HandleHarvesterTurnTask.STRAIGHT_BACK_MAX_SIDE or -pz > HandleHarvesterTurnTask.STRAIGHT_BACK_MAX_DISTANCE then
        return false
    end
    -- straight back on its own axis (original AutoDrive behaviour: the diagonal to the pipe cut the crop edge),
    -- the chase corrects the side gap afterwards. The line must respect the field and crop settings too
    local points = {}
    local steps = math.max(2, math.ceil(-pz / 2))
    for i = 0, steps do
        local f = i / steps
        local wx, wy, wz = AutoDrive.localToWorld(self.vehicle, 0, 0, pz * f)
        table.insert(points, {x = wx, y = wy, z = wz})
    end
    -- its own cut lane: the wheels already stand there, a small margin under the axles against the crop edge
    local ok, why = self:checkGround(points, HandleHarvesterTurnTask.PIPE_REVERSE_WIDTH)
    if not ok then
        return false
    end
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    self.pipeReverse = {x = x, z = z, time = 0, blocked = 0}
    self.vehicle.ad.specialDrivingModule:reset()
    return true
end

HandleHarvesterTurnTask.STRAIGHT_BACK_MAX_SIDE = 6 -- m beside the pipe at the end of a straight reverse at most
HandleHarvesterTurnTask.STRAIGHT_BACK_MAX_DISTANCE = 45 -- m straight back at most
HandleHarvesterTurnTask.ALIGNED_S_MIN = 12 -- m of reverse to take up the side gap
HandleHarvesterTurnTask.ALIGNED_S_MAX = 25
HandleHarvesterTurnTask.ALIGNED_S_PER_M = 3 -- m of reverse per m of side gap
HandleHarvesterTurnTask.ALIGNED_MAX_GAP = 12

-- (capture): the reverse line is the Courseplay course of the new row, offset by
-- half the header width + REVERSE_HEADER_GAP up to the side edge of the train. It was taken from the harvester itself,
-- still finishing its turn: the train ended in front of the header tip and the harvester stopped
HandleHarvesterTurnTask.REVERSE_HEADER_GAP = 1 -- m between the header tip line and the side edge of the train
HandleHarvesterTurnTask.ROW_LINE_AHEAD = 80 -- m of Courseplay course read from its current waypoint
HandleHarvesterTurnTask.ROW_LINE_DEG = 6 -- deg from the row heading for a course segment to belong to the row
HandleHarvesterTurnTask.ROW_LINE_MIN = 10 -- m of straight course at least
HandleHarvesterTurnTask.ROW_LINE_BEHIND = 30 -- m: the harvester stands at most this far from the ends of the line read
-- the reverse goes on until the tractor stands at the pipe place (the trailer under the pipe): the waypoints are
-- followed by the trailer wheels, the last one is reached 3 m early
HandleHarvesterTurnTask.REVERSE_END_PAST = 2 -- m

-- half width of the train at its widest (body or outer tyre edge)
function HandleHarvesterTurnTask:getTrainHalfWidth()
    local half = 0
    local units = AutoDrive.getAllUnits(self.vehicle)
    for _, unit in pairs(units or {}) do
        if unit.size ~= nil and unit.size.width ~= nil then
            half = math.max(half, math.abs(unit.size.width) / 2)
        end
    end
    local pfm = self.vehicle.ad.pathFinderModule
    if pfm ~= nil and pfm.getWheelHalfWidth ~= nil then
        local ok, wheelHalf = pcall(pfm.getWheelHalfWidth, pfm)
        if ok and wheelHalf ~= nil and wheelHalf < 5 then
            half = math.max(half, wheelHalf)
        end
    end
    if half < 0.5 then
        half = 1.5
    end
    return half
end

-- header width of the harvester (its front tool), else its own width
function HandleHarvesterTurnTask:getHeaderWidth()
    local width = AutoDrive.getFrontToolWidth(self.combine) or 0
    if width < 1 and self.combine.size ~= nil and self.combine.size.width ~= nil then
        width = math.abs(self.combine.size.width)
    end
    return width
end

-- distance from the tractor root back to the node that follows the reverse waypoints (trailer wheels), and that node
function HandleHarvesterTurnTask:getReverseNodeBehind()
    local sdm = self.vehicle.ad.specialDrivingModule
    if sdm == nil or sdm.getReverseNode == nil then
        return nil
    end
    local ok, node = pcall(sdm.getReverseNode, sdm)
    if not ok or node == nil then
        return nil
    end
    local x, y, z = getWorldTranslation(node)
    local _, _, lz = AutoDrive.worldToLocal(self.vehicle, x, y, z)
    return math.max(0, -lz), x, z
end

-- line of the straight course part matching the row heading, read on the Courseplay course of the harvester. With
-- fromIx, from its current waypoint on; otherwise the whole course, the part the harvester stands on
function HandleHarvesterTurnTask:readCpRowLine(refX, refZ, fromIx, maxOff)
    local ok, line = pcall(function()
        local strategy = self.combine.getCpDriveStrategy ~= nil and self.combine:getCpDriveStrategy() or nil
        if strategy == nil or strategy.getCurrentCourse == nil or strategy.ppc == nil then
            return nil
        end
        local course = strategy:getCurrentCourse()
        local ix = strategy.ppc:getCurrentWaypointIx()
        if course == nil or ix == nil then
            return nil
        end
        local n = course:getNumberOfWaypoints()
        local cx, _, cz = getWorldTranslation(self.combine.components[1].node)
        local first = fromIx and math.max(1, ix - 2) or 1
        local cosMax = math.cos(math.rad(HandleHarvesterTurnTask.ROW_LINE_DEG))
        local best, run = nil, nil
        local function closeRun()
            if run ~= nil and run.len >= HandleHarvesterTurnTask.ROW_LINE_MIN then
                local dx, dz = run.x2 - run.x1, run.z2 - run.z1
                local len = math.sqrt(dx * dx + dz * dz)
                if len > 1 then
                    dx, dz = dx / len, dz / len
                    local along = (cx - run.x1) * dx + (cz - run.z1) * dz
                    local off = math.abs((cx - run.x1) * dz - (cz - run.z1) * dx)
                    if off <= maxOff and along >= -HandleHarvesterTurnTask.ROW_LINE_BEHIND and along <= len + HandleHarvesterTurnTask.ROW_LINE_BEHIND
                        and (best == nil or off < best.off) then
                        best = {x = run.x1, z = run.z1, dx = dx, dz = dz, len = len, off = off}
                    end
                end
            end
            run = nil
        end
        local px, _, pz = course:getWaypointPosition(first)
        local dist = 0
        for i = first + 1, n do
            local x, _, z = course:getWaypointPosition(i)
            local seg = MathUtil.vector2Length(x - px, z - pz)
            if seg > 0.1 then
                local dx, dz = (x - px) / seg, (z - pz) / seg
                if dx * refX + dz * refZ >= cosMax then
                    if run == nil then
                        run = {x1 = px, z1 = pz, len = 0}
                    end
                    run.x2, run.z2 = x, z
                    run.len = run.len + seg
                else
                    closeRun()
                    if fromIx and best ~= nil then
                        break
                    end
                end
                dist = dist + seg
                px, pz = x, z
                if fromIx and dist > HandleHarvesterTurnTask.ROW_LINE_AHEAD then
                    break
                end
            end
        end
        closeRun()
        return best
    end)
    if ok then
        return line
    end
    return nil
end

-- the new row: point, unit driving direction and source. Ways tried in turn: the Courseplay course from its current
-- waypoint, the whole course, the row entry read at the end of the row, the harvester itself
function HandleHarvesterTurnTask:getRowLine()
    local hx, _, hz = AutoDrive.localDirectionToWorld(self.combine, 0, 0, 1)
    local len = math.sqrt(hx * hx + hz * hz)
    if len < 0.01 then
        return nil
    end
    hx, hz = hx / len, hz / len
    local refX, refZ = hx, hz
    if self.cpRow ~= nil and self.cpRow.dx ~= nil and self.cpRow.dx * hx + self.cpRow.dz * hz > 0.7 then
        refX, refZ = self.cpRow.dx, self.cpRow.dz
    end
    local maxOff = math.max(3, self:getHeaderWidth() / 2)
    local line = self:readCpRowLine(refX, refZ, true, maxOff)
    if line ~= nil then
        line.src = string.format("Courseplay course ahead of the harvester (%.0f m read, harvester %.1f m from it)", line.len, line.off)
        return line
    end
    line = self:readCpRowLine(refX, refZ, false, maxOff)
    if line ~= nil then
        line.src = string.format("Courseplay course of the row (whole course, %.0f m, harvester %.1f m from it)", line.len, line.off)
        return line
    end
    local cx, _, cz = getWorldTranslation(self.combine.components[1].node)
    if self.cpRow ~= nil and self.cpRow.dx ~= nil and refX == self.cpRow.dx then
        local off = math.abs((cx - self.cpRow.x) * self.cpRow.dz - (cz - self.cpRow.z) * self.cpRow.dx)
        if off <= maxOff then
            return {x = self.cpRow.x, z = self.cpRow.z, dx = self.cpRow.dx, dz = self.cpRow.dz,
                src = string.format("row entry read at the row end (harvester %.1f m away)", off)}
        end
    end
    return {x = cx, z = cz, dx = hx, dz = hz, src = "harvester position and heading (Courseplay course unreadable)"}
end

-- reverse waypoints from the train to the pipe place, along the row line; nil, reason if refused
function HandleHarvesterTurnTask:buildAlignedReverse(ignoreCrop)
    local pipe = self:getPipePlace()
    if pipe == nil then
        return nil, "pipe unknown"
    end
    local line = self.rowLine or self:getRowLine()
    if line == nil then
        return nil, "harvester heading unknown"
    end
    local hx, hz = line.dx, line.dz
    local nx, nz = hz, -hx -- left of the row
    local tx, _, tz = getWorldTranslation(self.vehicle.components[1].node)
    local sT = (tx - line.x) * hx + (tz - line.z) * hz
    local sP = (pipe.x - line.x) * hx + (pipe.z - line.z) * hz
    local along = sT - sP -- m from the pipe place up to the tractor
    -- side of the train: the pipe side, else the side it stands on
    local side = self.pipeSide
    if side ~= AutoDrive.CHASEPOS_LEFT and side ~= AutoDrive.CHASEPOS_RIGHT then
        side = ((tx - line.x) * nx + (tz - line.z) * nz) >= 0 and AutoDrive.CHASEPOS_LEFT or AutoDrive.CHASEPOS_RIGHT
    end
    local header = self:getHeaderWidth()
    local trainHalf = self:getTrainHalfWidth()
    local offset = header / 2 + HandleHarvesterTurnTask.REVERSE_HEADER_GAP + trainHalf
    local targetLat = side * offset
    -- the waypoints are followed by the trailer wheels: start behind the train, end when the tractor stands at the place
    local tractorHalf = (self.vehicle.size ~= nil and self.vehicle.size.length or 6) / 2
    local d0 = math.max(0, AutoDrive.getTractorTrainLength(self.vehicle, true, false) - tractorHalf)
    local dRev, rx, rz = self:getReverseNodeBehind()
    dRev = dRev or d0
    local startLat
    if rx ~= nil then
        startLat = (rx - line.x) * nx + (rz - line.z) * nz
    else
        startLat = (tx - line.x) * nx + (tz - line.z) * nz
    end
    d0 = math.max(d0, dRev + 1)
    local dEnd = along + dRev + HandleHarvesterTurnTask.REVERSE_END_PAST
    if dEnd - d0 < 4 then
        return nil, string.format("pipe too close (%.0f m, trailer %.0f m)", along, dRev)
    end
    local gap = startLat - targetLat
    if math.abs(gap) > HandleHarvesterTurnTask.ALIGNED_MAX_GAP then
        return nil, string.format("side offset too large (%.0f m)", gap)
    end
    local sLen = math.min(dEnd - d0 - 2, math.max(HandleHarvesterTurnTask.ALIGNED_S_MIN,
        math.min(HandleHarvesterTurnTask.ALIGNED_S_MAX, math.abs(gap) * HandleHarvesterTurnTask.ALIGNED_S_PER_M)))
    local points = {}
    local n = math.max(2, math.floor(dEnd - d0))
    for i = 0, n do
        local d = d0 + (dEnd - d0) * i / n -- m behind the tractor root, along the row
        local u = math.min(1, (d - d0) / math.max(sLen, 0.1))
        local lateral = targetLat + gap * (1 - (3 * u * u - 2 * u * u * u))
        local s = sT - d
        local x, z = line.x + hx * s + nx * lateral, line.z + hz * s + nz * lateral
        local y = AutoDrive:getTerrainHeightAtWorldPos(x, z) or 0
        table.insert(points, {x = x, y = y, z = z, isReverse = true})
    end
    local ok, why = self:checkGround(points, HandleHarvesterTurnTask.PIPE_REVERSE_WIDTH, ignoreCrop)
    if not ok then
        return nil, why
    end
    self.alignedGap = math.abs(gap)
    self.alignedS = sLen
    self.alignedLen = dEnd - d0
    self.alignedSrc = line.src
    self.alignedOffset = offset
    self.alignedHeader = header
    self.alignedTrainHalf = trainHalf
    return points
end

-- straight back toward the pipe, the side gap corrected along the way; done once beside it
function HandleHarvesterTurnTask:updateReverseToPipe(dt)
    local name = tostring(self.vehicle:getName())
    local r = self.pipeReverse
    local sdm = self.vehicle.ad.specialDrivingModule
    local px, pz = self:getPipeLocal()
    if px == nil then
        sdm:stopVehicle()
        return true
    end
    if pz > -2 and (not r.aligned or math.abs(px) <= HandleHarvesterTurnTask.PIPE_BESIDE_GAP) then
        sdm:stopVehicle()
        return true
    end
    -- the harvester turns again (its turn was not finished): stop, wait for it, plan the reverse again from here
    if self.pipeReverseHeading ~= nil and HandleHarvesterTurnTask.headingDiff(self:getCombineHeadingDeg(), self.pipeReverseHeading) > HandleHarvesterTurnTask.PIPE_REVERSE_TURNED
        and (self.pipeRestarts or 0) < HandleHarvesterTurnTask.PIPE_REVERSE_RESTARTS then
        self.pipeRestarts = (self.pipeRestarts or 0) + 1
        sdm:stopVehicle()
        sdm:update(dt)
        self:startWaitCombineTurn()
        return false
    end
    r.time = r.time + dt
    -- held 27 m short (probably its rear sensor on the harvester header), then the chase from
    -- ahead of it planned a route forward. Blocked: stop there and wait for the harvester to come alongside
    if r.waiting then
        sdm:stopVehicle()
        sdm:update(dt)
        if r.time > HandleHarvesterTurnTask.PIPE_REVERSE_TIME + HandleHarvesterTurnTask.PIPE_WAIT_TIME then
            return true
        end
        return false
    end
    r.blocked = math.abs(self.vehicle.lastSpeedReal or 0) < 0.0002 and r.blocked + dt or 0
    if r.time > HandleHarvesterTurnTask.PIPE_REVERSE_TIME or r.blocked > HandleHarvesterTurnTask.PIPE_REVERSE_BLOCKED then
        local sensor = self.vehicle.ad.collisionDetectionModule ~= nil and self.vehicle.ad.collisionDetectionModule:checkReverseCollision()
        r.waiting = true
        r.time = HandleHarvesterTurnTask.PIPE_REVERSE_TIME
        sdm:stopVehicle()
        return false
    end
    if r.aligned then
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            r.waiting = true
            r.time = HandleHarvesterTurnTask.PIPE_REVERSE_TIME
            return false
        end
        self.vehicle.ad.drivePathModule:update(dt)
        return false
    end
    local tx, ty, tz = AutoDrive.localToWorld(self.vehicle, 0, 0, pz - 6)
    sdm:reverseToTargetLocation(dt, {x = tx, y = ty, z = tz}, HandleHarvesterTurnTask.PIPE_REVERSE_SPEED)
    return false
end

function HandleHarvesterTurnTask:continueWith(points)
    self.points = points
    self.vehicle.ad.drivePathModule:setWayPoints(self.points)
    self.state = HandleHarvesterTurnTask.STATE_EXECUTE_TURN
end

function HandleHarvesterTurnTask:tryNextTurn()
    if self.endRow then
        if self.endRowTries >= 4 and not self.trailTried then
            -- 4th way: drive its own trail
            self.trailTried = true
            if self:tryCombineTrail() then
                return
            end
        end
        if self.endRowTries >= 4 then
            -- fresh flag: the catch task tries the own U-turn (3 points...) before any route
            self.vehicle.ad.endRowUTurn = g_time or 0
            self:finished(ADTaskModule.DONT_PROPAGATE)
            self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:setToWaitForCall(true)
            self.endRowGaveUp = true
            return
        end
        self.endRowTries = self.endRowTries + 1
    end
    -- 4 turns: the Y manoeuvre of his drawing added to the AutoDrive list
    self.nextTurnToTry = (self.nextTurnToTry + 1) % 4
    if self.nextTurnToTry == 0 and self.endRow then
        self.currentTurn = OffsetTurn:new(self.turnParameters)
    elseif self.nextTurnToTry == 0 then
        self.currentTurn = OffsetTurn:new(self.turnParameters)
        if not AutoDrive.combineIsTurning(self.combine) then
            if self.triedAllTurnsAfterTurnEnded == true then
                self:finished(ADTaskModule.DONT_PROPAGATE)
                self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:setToWaitForCall()
            else
                self.triedAllTurnsAfterTurnEnded = true
            end
        end
    elseif self.nextTurnToTry == 1 then
        self.currentTurn = ReverseOffsetTurn:new(self.turnParameters)
    elseif self.nextTurnToTry == 2 then
        self.currentTurn = ForwardOffsetTurn:new(self.turnParameters)
    elseif self.nextTurnToTry == 3 then
        self.currentTurn = YTurn:new(self.turnParameters)
    end

    -- end of row: 150-210 deg accepted, the reverse to the pipe takes up the rest (30 deg allowed)
    -- the Y manoeuvre turns until the new row heading: 90-270 deg (an unloader was at 103 deg, nothing passed)
    local valid
    local window = "150-210"
    if self.endRow and self.nextTurnToTry == 3 then
        valid = YTurn.checkValidity(self.turnParameters)
        window = "90-270"
    elseif self.endRow then
        valid = self.turnParameters.angle > 150 and self.turnParameters.angle < 210
    else
        valid = self.currentTurn.checkValidity(self.turnParameters)
    end
    if not valid then
        self.currentTurn = nil
    end

    self.lastCollisionCheckIndex = nil
    self.expectedColliCallbacks = 0
    self.colliFound = false
end

HandleHarvesterTurnTask.TRAIL_GROUND_RADIUS = 4    -- m around the harvester's trail counted as drivable ground
HandleHarvesterTurnTask.TRAIL_START_DISTANCE = 6   -- m at most between the tractor and the trail
HandleHarvesterTurnTask.TRAIL_KEEP_BEHIND = 8      -- m kept behind the harvester's rear at the end of the trail
HandleHarvesterTurnTask.TRAIL_KEEP_BEHIND_CLOSE = 2 -- same, when the tractor is already close behind it

function HandleHarvesterTurnTask:tryCombineTrail()
    local mode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
    if mode == nil or mode.getCombineTrailFrom == nil then
        return false
    end
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    local combine = self.combine
    -- its trail leads to it: only when it is ahead of the tractor (not when the tractor faces away from it)
    local cx, _, cz = getWorldTranslation(combine.components[1].node)
    local fx, _, fz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    if math.abs(AutoDrive.angleBetween({x = fx, z = fz}, {x = cx - x, z = cz - z})) > 90 then
        return false
    end
    local combineRear = (combine.size ~= nil and combine.size.length or 10) / 2 - (combine.size ~= nil and combine.size.lengthOffset or 0)
    local points, why = mode:getCombineTrailFrom(x, z, HandleHarvesterTurnTask.TRAIL_START_DISTANCE,
        combineRear + HandleHarvesterTurnTask.TRAIL_KEEP_BEHIND + (self.vehicle.size ~= nil and self.vehicle.size.length / 2 or 3))
    if points == nil and why == "track too short ahead of me" then
        -- already close behind it (16 m, tractor at 68 deg): drive its trail up to just behind its rear,
        -- the tractor ends aligned with it and the chase takes over
        points, why = mode:getCombineTrailFrom(x, z, HandleHarvesterTurnTask.TRAIL_START_DISTANCE, combineRear + HandleHarvesterTurnTask.TRAIL_KEEP_BEHIND_CLOSE)
    end
    if points == nil then
        return false
    end
    local ok, groundWhy = self:checkGround(points)
    if not ok then
        return false
    end
    self.vehicle.ad.endRowUTurn = nil
    self:continueWith(points)
    return true
end

HandleHarvesterTurnTask.turnNames = {[0] = "forward U-turn", [1] = "reverse then U-turn", [2] = "forward offset U-turn", [3] = "Y manoeuvre (diagonal forward, long straight reverse, forward U-turn)"}

-- field and crop under the whole width of the train along the turn, by the AutoDrive settings:
-- Strict refuses, otherwise allowed. The last reverse points toward the harvester (its own cut track) are kept
function HandleHarvesterTurnTask:checkGround(points, widthFactor, ignoreCrop)
    local strictField = AutoDrive.getSetting("restrictToField", self.vehicle) == "strict"
    local strictCrop = AutoDrive.getSetting("avoidFruit", self.vehicle) == "strict" and not ignoreCrop
    if not strictField and not strictCrop then
        return true
    end
    local half = (self.vehicle.ad.pathFinderModule ~= nil and self.vehicle.ad.pathFinderModule:getTrainWidth() or 3) / 2 * (widthFactor or 1)
    local pf = PathFinderModule
    -- ground the harvester drove on is drivable, even beyond the field border (its CP turn)
    local mode = self.vehicle.ad.modes ~= nil and self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD] or nil
    local stored = pf.ignoreIsolatedCrop
    pf.ignoreIsolatedCrop = true
    local why = nil
    for i = 2, #points do
        local a, b = points[i - 1], points[i]
        local dx, dz = b.x - a.x, b.z - a.z
        local len = math.sqrt(dx * dx + dz * dz)
        if len > 0.01 then
            dx, dz = dx / len, dz / len
            for _, w in ipairs({-half, 0, half}) do
                local px, pz = b.x - dz * w, b.z + dx * w
                if strictField and not pf.getMapOnFieldAt(px, pz) and not (mode ~= nil and mode.isOnCombineTrail ~= nil
                    and mode:isOnCombineTrail(px, pz, HandleHarvesterTurnTask.TRAIL_GROUND_RADIUS)) then
                    why = string.format("off the field at %.0f,%.0f", px, pz)
                    break
                end
                local crop = pf.getMapCropAt(px, pz)
                -- on the harvester's own trail the crop under it does not count (its U-turn with
                -- the header lifted)
                if strictCrop and crop ~= nil and crop ~= 0 and not (mode ~= nil and mode.isOnCombineTrail ~= nil and mode:isOnCombineTrail(px, pz)) then
                    why = string.format("crop at %.0f,%.0f", px, pz)
                    break
                end
            end
        end
        if why ~= nil then
            break
        end
    end
    pf.ignoreIsolatedCrop = stored
    return why == nil, why
end

function HandleHarvesterTurnTask:abort()
end

function HandleHarvesterTurnTask:finished(propagate)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_COMBINEINFO, "HandleHarvesterTurnTask:finished()")
    self.vehicle.ad.taskModule:setCurrentTaskFinished(propagate)
end

function HandleHarvesterTurnTask:cleanPoints(points)
    for i, point in pairs(points) do
        point.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, point.x,point.y,point.z)
    end
end

function HandleHarvesterTurnTask:getUturnOffsetValues(left)
    local radius = AutoDrive.getDriverRadius(self.vehicle)
    local offsetTurn = (radius*2) > (AutoDrive.getFrontToolWidth(self.combine) + 2)
    if offsetTurn then
        local offset = (radius*2) - AutoDrive.getFrontToolWidth(self.combine)

        local offsetX = offset
        local offsetZ = 5 + ((offset - 3)/1.5) * 1
        if left then
            offsetX = -offsetX
        end
        return offsetX, offsetZ
    end

    return 0, 0
end

function HandleHarvesterTurnTask:generateUTurn(startPos, startDirX, startDirZ, left, offsetValues)
    local radius = AutoDrive.getDriverRadius(self.vehicle)
    local vehX, vehY, vehZ = getWorldTranslation(self.vehicle.components[1].node)
    local resolution = 20

    local offsetX, offsetZ = self:getUturnOffsetValues(left)
    if offsetValues ~= nil then
        offsetX, offsetZ = offsetValues[1], offsetValues[2]
    end
    local points = {}

    if math.abs(offsetX) > 0 then
        local endPoint = { x = startPos.x + offsetX * startDirX.x + offsetZ * startDirZ.x, y=vehY, z = startPos.z + offsetX * startDirX.z + offsetZ * startDirZ.z}

        offsetZ = offsetZ + 2
        local p3 = { x = startPos.x + offsetX * startDirX.x + offsetZ * startDirZ.x, y=vehY, z = startPos.z + offsetX * startDirX.z + offsetZ * startDirZ.z}
        local p0 = { x = startPos.x - 1 * startDirZ.x, y=vehY, z = startPos.z -1 * startDirZ.z}

        AutoDrive.splineInterpolationUserCurvature = 5
        local splinePoints = AutoDrive:createSplineWithControlPoints(startPos, p0, endPoint, p3)
        AutoDrive.splineInterpolation = {
            valid = false
        }

        if splinePoints ~= nil and #splinePoints > 2 then
            for _, wp in pairs(splinePoints) do
                wp.isForward = true
                table.insert(points, wp)
                startPos = wp
            end
        end
    end

    if left then
        for i = 3, (resolution + 1) do
            local circlePoint = {   x = -math.cos((i-1) * math.pi / resolution) * radius + radius,
                                    y = math.sin((i-1) * math.pi / resolution) * radius }

            local point = { x = startPos.x + circlePoint.x * startDirX.x + circlePoint.y * startDirZ.x, y=vehY, z = startPos.z + circlePoint.x * startDirX.z + circlePoint.y * startDirZ.z}
            local rayCastResult = AutoDrive:getTerrainHeightAtWorldPos(point.x, point.z)
            point.y = rayCastResult or point.y
            local dummy = 1
            for i = 1, 1000 do
                dummy = dummy + i
            end
            point.y = AutoDrive.raycastHeight or point.y
            point.isForward = true

            table.insert(points, point)
        end
    else
        for i = 3, (resolution + 1) do
            local circlePoint = {   x = math.cos((i-1) * math.pi / resolution) * radius - radius,
                                    y = math.sin((i-1) * math.pi / resolution) * radius }

            local point = { x = startPos.x + circlePoint.x * startDirX.x + circlePoint.y * startDirZ.x, y=vehY, z = startPos.z + circlePoint.x * startDirX.z + circlePoint.y * startDirZ.z}
            local rayCastResult = AutoDrive:getTerrainHeightAtWorldPos(point.x, point.z)
            point.y = rayCastResult or point.y
            local dummy = 1
            for i = 1, 1000 do
                dummy = dummy + i
            end
            point.y = AutoDrive.raycastHeight or point.y
            point.isForward = true

            table.insert(points, point)
        end
    end

    return points
end

function HandleHarvesterTurnTask:generateStraight(startPos, startDir, length, isReverse)
    local nodes = {}
    local stepLength = 1
    local currentLength = 1
    while currentLength < length do
        local pos = { x = startPos.x + currentLength * startDir.x, z = startPos.z + currentLength * startDir.z}
        local rayCastResult = AutoDrive:getTerrainHeightAtWorldPos(pos.x, pos.z)
        pos.y = rayCastResult or startPos.y
        local dummy = 1
        for i = 1, 1000 do
            dummy = dummy + i
        end
        pos.y = AutoDrive.raycastHeight or pos.y
        if isReverse then
            pos.isReverse = true
        else
            pos.isForward = true
        end
        table.insert(nodes, pos)
        currentLength = currentLength + stepLength
    end

    return nodes
end

function HandleHarvesterTurnTask:generateReverseToCombineSection(measureOnly)
    if self.endRow and measureOnly ~= true then
        return {}
    end
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local diffX, _, diffZ = AutoDrive.worldToLocal(self.combine, x, y, z, self.combine.ad.ADRootNode)

    local nodes = {}

    local sideChaseTermX = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getSideChaseOffsetX()
    local sideChaseTermZ = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getSideChaseOffsetZ(AutoDrive.dynamicChaseDistance or not self.combine.ad.isChopper)

    if not AutoDrive.combineIsTurning(self.combine) and not self.endRow then
        local chasePos = AutoDrive.createWayPointRelativeToVehicle(self.combine, -(sideChaseTermX + self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getPipeSlopeCorrection()), sideChaseTermZ - 2)
        local chasePosFar = AutoDrive.createWayPointRelativeToVehicle(self.combine, -(sideChaseTermX + self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getPipeSlopeCorrection()), sideChaseTermZ - 2 - 20)

        if self.turnParameters.targetLeft then --left
            chasePos = AutoDrive.createWayPointRelativeToVehicle(self.combine, sideChaseTermX + self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getPipeSlopeCorrection(), sideChaseTermZ - 2)
            chasePosFar = AutoDrive.createWayPointRelativeToVehicle(self.combine, sideChaseTermX + self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getPipeSlopeCorrection(), sideChaseTermZ - 2 - 20)
        end

        chasePos.isReverse = true
        chasePosFar.isReverse = true
        chasePos.allowedToSkip = true
        chasePosFar.allowedToSkip = true

        table.insert(nodes, chasePos)
        table.insert(nodes, chasePosFar)

    else
        -- For now, assume this position is 180° reversed to combines start position
        local harvesterTargetPosition = self:getHarvesterEndTurnPosition()
        local nearPosZ = 2
        local farPosZ = 5 + nearPosZ + AutoDrive.getTractorTrainLength(self.vehicle, true, false)
        if harvesterTargetPosition ~= nil then
            local chasePos =    {   x = harvesterTargetPosition.x - self.turnParameters.targetDir.x.x * sideChaseTermX + self.turnParameters.targetDir.z.x * (sideChaseTermZ - nearPosZ),
                                    z = harvesterTargetPosition.z - self.turnParameters.targetDir.x.z * sideChaseTermX + self.turnParameters.targetDir.z.z * (sideChaseTermZ - nearPosZ) }
            local chasePosFar = {   x = harvesterTargetPosition.x - self.turnParameters.targetDir.x.x * sideChaseTermX + self.turnParameters.targetDir.z.x * (sideChaseTermZ - farPosZ),
                                    z = harvesterTargetPosition.z - self.turnParameters.targetDir.x.z * sideChaseTermX + self.turnParameters.targetDir.z.z * (sideChaseTermZ - farPosZ) }

            if self.turnParameters.targetLeft then
                chasePos =    {     x = harvesterTargetPosition.x + self.turnParameters.targetDir.x.x * sideChaseTermX + self.turnParameters.targetDir.z.x * (sideChaseTermZ - nearPosZ),
                                    z = harvesterTargetPosition.z + self.turnParameters.targetDir.x.z * sideChaseTermX + self.turnParameters.targetDir.z.z * (sideChaseTermZ - nearPosZ) }
                chasePosFar =    {  x = harvesterTargetPosition.x + self.turnParameters.targetDir.x.x * sideChaseTermX + self.turnParameters.targetDir.z.x * (sideChaseTermZ - farPosZ),
                                    z = harvesterTargetPosition.z + self.turnParameters.targetDir.x.z * sideChaseTermX + self.turnParameters.targetDir.z.z * (sideChaseTermZ - farPosZ) }
            end

            chasePos.y = AutoDrive:getTerrainHeightAtWorldPos(chasePos.x, chasePos.z)
            chasePosFar.y = AutoDrive:getTerrainHeightAtWorldPos(chasePosFar.x, chasePosFar.z)

            chasePos.isReverse = true
            chasePosFar.isReverse = true
            chasePos.allowedToSkip = true
            chasePosFar.allowedToSkip = true

            table.insert(nodes, chasePos)
            table.insert(nodes, chasePosFar)
        end
    end

    return nodes
end

-- Returns bool finished, bool foundCollision
function HandleHarvesterTurnTask:doCollisionCheck(waypoints)
    if self.lastCollisionCheckIndex == nil then
        self.lastCollisionCheckIndex = 1
    end

    --print("HandleHarvesterTurnTask:doCollisionCheck: index: " .. self.lastCollisionCheckIndex .. " / " .. #waypoints)

    --- Coll check:
    local widthX = self.vehicle.size.width / 1.75
    local height = 2.3
    local mask = AutoDrive.collisionMaskTerrain

    for i, wp in pairs(waypoints) do
        if i > self.lastCollisionCheckIndex and i <= (#waypoints - 1) and self.expectedColliCallbacks == 0 then
            local wpLast = waypoints[i - 1]
            local deltaX, deltaY, deltaZ = wp.x - wpLast.x, wp.y - wpLast.y, wp.z - wpLast.z
            local centerX, centerY, centerZ = wpLast.x + deltaX/2,  wpLast.y + deltaY/2,  wpLast.z + deltaZ/2
            local angleRad = math.atan2(deltaX, deltaZ)
            angleRad = AutoDrive.normalizeAngle(angleRad)
            local length = MathUtil.vector2Length(deltaX, deltaZ) / 2

            local angleX = -MathUtil.getYRotationFromDirection(deltaY, length*2)

            local shapes = overlapBox(centerX, centerY+3, centerZ, angleX, angleRad, 0, widthX, height, length, "collisionTestCallback", self, mask, true, true, true, true)
            if shapes > 0 then
                self.expectedColliCallbacks = self.expectedColliCallbacks + 1
                --print("Expecting collisionTestCallbacks: " .. self.expectedColliCallbacks)
            end
            if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_COMBINEINFO) then
                local r,g,b = 0,1,0
                if shapes > 0 then
                    r = 1
                    g = 0
                    DebugUtil.drawOverlapBox(centerX, centerY+3, centerZ, angleX, angleRad, 0, widthX, height, length, r, g, b)
                end
                DebugUtil.drawOverlapBox(centerX, centerY+3, centerZ, angleX, angleRad, 0, widthX, height, length, r, g, b)
            end
            self.lastCollisionCheckIndex = i
        end
    end

    return (self.lastCollisionCheckIndex == (#waypoints - 1) and self.expectedColliCallbacks == 0) or self.colliFound, self.colliFound
end

function HandleHarvesterTurnTask:collisionTestCallback(transformId, x, y, z, distance)
    self.expectedColliCallbacks = math.max(-1, self.expectedColliCallbacks - 1)
    --print("Received collisionTestCallback. Outstanding: " .. self.expectedColliCallbacks)
    if transformId ~= 0 and transformId ~= g_currentMission.terrainRootNode then
        if g_currentMission.nodeToObject[transformId] ~= nil then
            local object = g_currentMission.nodeToObject[transformId]
            local isHarvester = self.endRow and self.combine ~= nil and (object == self.combine or AutoDrive:checkIsConnected(self.combine, object))
            if object ~= self.vehicle and not AutoDrive:checkIsConnected(self.vehicle, object) and not isHarvester then
                self.colliFound = true
            end
        else
            self.colliFound = true
        end
    end
end

function HandleHarvesterTurnTask:getHarvesterEndTurnPosition()
    if self.combine.spec_aiFieldWorker ~= nil then
        local spec = self.combine.spec_aiFieldWorker
        if spec.lastTurnStrategy ~= nil then
            if spec.lastTurnStrategy.turnSegments ~= nil then
                local segments = spec.lastTurnStrategy.turnSegments
                if segments[#segments] ~= nil then
                    if segments[#segments].endPoint ~= nil then
                        local endPoint = segments[#segments].endPoint
                        local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
                        return {x = endPoint[1], y = endPoint[2], z = endPoint[3]}
                    end
                end
            end
        end
    end
    -- Courseplay harvester: entry of the new row read on its course (kept at the end-of-row wait)
    if self.cpRow == nil and self.endRow then
        local ant = self.vehicle.ad.anticipatedRear
        if ant ~= nil and (g_time or 0) - ant.t < PathFinderModule.ANTICIPATED_MAX_AGE then
            self.cpRow = {x = ant.ex, z = ant.ez, dx = ant.dx, dz = ant.dz}
        else
            -- its turn is over (or the row was not read): it stands on its new row, its own heading gives it
            local cx, _, cz = getWorldTranslation(self.combine.components[1].node)
            local dx, _, dz = AutoDrive.localDirectionToWorld(self.combine, 0, 0, 1)
            local len = math.sqrt(dx * dx + dz * dz)
            if len > 0.01 then
                self.cpRow = {x = cx, z = cz, dx = dx / len, dz = dz / len}
            end
        end
    end
    if self.cpRow ~= nil then
        return {x = self.cpRow.x, y = AutoDrive:getTerrainHeightAtWorldPos(self.cpRow.x, self.cpRow.z) or 0, z = self.cpRow.z}
    end
    return nil
end

function HandleHarvesterTurnTask:getI18nInfo()
    local harvester = AbstractTask.getHarvesterI18n(self.combine)
    if self.state == HandleHarvesterTurnTask.STATE_WAIT_COMBINE_TURN then
        return "$l10n_AD_task_waiting_comma; " .. harvester .. " $l10n_AD_task_is_turning;"
    elseif self.state == HandleHarvesterTurnTask.STATE_REVERSE_TO_PIPE then
        return "$l10n_AD_task_reverse_to; " .. harvester
    elseif self.endRow then
        return "$l10n_AD_task_end_row_turn;"
    end
    return "$l10n_AD_task_uturn;"
end