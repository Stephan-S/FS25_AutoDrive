ClearHarvesterZoneTask = ADInheritsFrom(AbstractTask)

ClearHarvesterZoneTask.STATE_PLANNING = {}
ClearHarvesterZoneTask.STATE_DRIVING = {}
ClearHarvesterZoneTask.STATE_PRE_REVERSING = {}
ClearHarvesterZoneTask.STATE_RETRY_WAIT = {}
ClearHarvesterZoneTask.STATE_LOOP_WAIT = {}
ClearHarvesterZoneTask.STATE_STRAIGHTENING = {}

ClearHarvesterZoneTask.MIN_RADIUS = 70
ClearHarvesterZoneTask.MAX_RADIUS = 110
ClearHarvesterZoneTask.MIN_PATH_LENGTH = 80
ClearHarvesterZoneTask.MAX_PATH_LENGTH = 140
ClearHarvesterZoneTask.MAX_CELLS = 2400
ClearHarvesterZoneTask.FRUIT_CELL_COST = 3
ClearHarvesterZoneTask.TARGET_FRUIT_CLEARANCE = 2
ClearHarvesterZoneTask.PRE_REVERSE_DISTANCE = 8
ClearHarvesterZoneTask.STALL_TIME = 8000
ClearHarvesterZoneTask.RETRY_TIME = 3000
ClearHarvesterZoneTask.LOOP_MIN_WAIT = 1500
ClearHarvesterZoneTask.MAX_TARGET_CONTINUATION = 50
ClearHarvesterZoneTask.CONTINUATION_MARGIN_FACTOR = 1.25
ClearHarvesterZoneTask.CLEAR_CHECK_TIME = 1000
ClearHarvesterZoneTask.MAX_PLAN_ATTEMPTS = 6
ClearHarvesterZoneTask.STRAIGHTEN_MARGIN = 2
ClearHarvesterZoneTask.STRAIGHT_DOT_LIMIT = 0.985 -- ~10 degrees per unit
ClearHarvesterZoneTask.HELPER_ZONE_PARK_MARGIN = 8

function ClearHarvesterZoneTask:new(vehicle, harvester, helperZone)
    local o = ClearHarvesterZoneTask:create()
    o.vehicle = vehicle
    o.harvester = harvester
    o.helperZone = helperZone
    o.escapeJob = nil
    o.course = nil
    o.reverseStartLocation = nil
    o.stallTimer = AutoDriveTON:new()
    o.retryTimer = AutoDriveTON:new()
    o.loopWaitTimer = AutoDriveTON:new()
    o.clearCheckTimer = AutoDriveTON:new()
    o.loopEscape = false
    o.planAttempts = 0
    o.straightened = false
    o.state = ClearHarvesterZoneTask.STATE_PLANNING
    return o
end

function ClearHarvesterZoneTask:isTrainStraight()
    local units = AutoDrive.getAllUnits(self.vehicle)
    if units == nil then
        return true
    end
    local forwardX, _, forwardZ = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    for _, unit in pairs(units) do
        if unit ~= self.vehicle and unit.components ~= nil and unit.components[1] ~= nil then
            local unitForwardX, _, unitForwardZ = localDirectionToWorld(unit.components[1].node, 0, 0, 1)
            if forwardX * unitForwardX + forwardZ * unitForwardZ < ClearHarvesterZoneTask.STRAIGHT_DOT_LIMIT then
                return false
            end
        end
    end
    return true
end

-- Once the clear/park condition is met the trailers may still stand at an angle right at the
-- crop edge (early exit, relaxed planning without continuation, loop escapes). Pull the train
-- straight for at least its own length before finishing - unless it already is straight.
function ClearHarvesterZoneTask:finishOrStraighten()
    if self.straightened or self:isTrainStraight() then
        self:finished()
        return
    end
    self:beginStraightening()
end

function ClearHarvesterZoneTask:beginStraightening()
    self.escapeJob = nil -- may still be set when the early exit fires during STATE_PLANNING
    local trainLength = AutoDrive.getTractorTrainLength(self.vehicle, true, false) or self.vehicle.size.length
    local distance = trainLength + ClearHarvesterZoneTask.STRAIGHTEN_MARGIN
    local wayPoints = {}
    local step = 6
    local pointDistance = step
    while pointDistance < distance + step do
        table.insert(wayPoints, AutoDrive.createWayPointRelativeToVehicle(self.vehicle, 0, pointDistance))
        pointDistance = pointDistance + step
    end
    self.course = wayPoints
    self.vehicle.ad.drivePathModule:setWayPoints(wayPoints)
    self.stallTimer:timer(false)
    self.state = ClearHarvesterZoneTask.STATE_STRAIGHTENING
end

-- The task is done as soon as the whole train is outside the harvester's current helper zone
-- and out of standing crop - regardless of how far the planned course still runs. Checked
-- periodically so a course that has already served its purpose is not driven to the bitter end
-- and no new planning is started when the condition is already met.
-- Deliberately asymmetric to the completion check at target reached: cutting a course short is
-- only allowed with the ENLARGED crop boxes clear, so a trailer that still hangs diagonally at
-- the crop edge keeps being pulled straight along the planned fruit-free continuation. The
-- target-reached check uses the strict boxes instead, where the enlarged ones would re-trigger
-- planning forever on crop edges the train has already left.
function ClearHarvesterZoneTask:isTrainClear()
    local currentHelperZone = AutoDrive.getCombineHelperZone(self.harvester)
    -- Zone standoff included: cutting the course short right at the zone boundary would let the
    -- next small harvester movement sweep the zone over the train again and restart the escape.
    if currentHelperZone ~= nil
        and AutoDrive.isVehicleTrainInHelperZone(self.vehicle, currentHelperZone, ClearHarvesterZoneTask.HELPER_ZONE_PARK_MARGIN) then
        return false
    end
    return not AutoDrive.isVehicleTrainInCrop(self.vehicle, true)
end

-- All (re)planning funnels through here. After too many attempts the task gives up quietly:
-- the WaitForCallTask that follows re-checks the zone every second and spawns a fresh task
-- (with a fresh zone snapshot) if the vehicle still infringes - that restart is more likely to
-- succeed than another attempt from an unchanged local state.
-- Giving up is only allowed while the train is NOT standing in crop: WaitForCallTask only
-- watches the helper zone, so finishing mid-crop would strand the vehicle there for good.
-- While in crop the task keeps planning instead - startPlanning relaxes its target
-- requirements with rising attempt count, so these retries get easier, not identical.
function ClearHarvesterZoneTask:replanOrFinish(allowFruitAtTarget)
    self.planAttempts = self.planAttempts + 1
    if self.planAttempts > ClearHarvesterZoneTask.MAX_PLAN_ATTEMPTS
        and not AutoDrive.isVehicleTrainInCrop(self.vehicle, false) then
        self:finished()
        return
    end
    self:startPlanning(allowFruitAtTarget)
end

-- Straight fruit-free continuation required beyond the target so trailers get pulled straight
-- and out of crop. getTractorTrainLength sums unit lengths only - drawbars and hitch offsets
-- are not included, and isTargetReached stops the tractor with some tolerance before the final
-- waypoint - hence the margin on top. The full margin is the ideal end pose though, and on
-- small or cluttered fields no such straight may exist at all: with rising attempt count the
-- requirement is relaxed progressively, trading end-pose quality for actually finding a
-- fruit-free spot instead of retrying the same impossible search until the give-up limit.
function ClearHarvesterZoneTask:getTargetContinuationDistance(trainLength, cellSize)
    if self.planAttempts > 4 then
        return 0
    end
    if self.planAttempts > 2 then
        return math.min(trainLength, ClearHarvesterZoneTask.MAX_TARGET_CONTINUATION)
    end
    return math.min(trainLength * ClearHarvesterZoneTask.CONTINUATION_MARGIN_FACTOR + cellSize,
        ClearHarvesterZoneTask.MAX_TARGET_CONTINUATION)
end

function ClearHarvesterZoneTask:startPlanning(allowFruitAtTarget)
    self.loopEscape = allowFruitAtTarget == true
    local trainLength = AutoDrive.getTractorTrainLength(self.vehicle, true, false) or self.vehicle.size.length
    local cellSize = math.clamp(math.max(self.vehicle.size.width, 3) + 1,
        ADEscapeCourseGenerator.MIN_CELL_SIZE, ADEscapeCourseGenerator.MAX_CELL_SIZE)
    -- Standoff from the zone boundary so the parked train doesn't get swept over again by the
    -- next small harvester movement. Dropped on late attempts - escaping at all beats parking
    -- pretty (the follow-up WaitForCallTask cooldown still damps immediate re-triggers).
    local parkMargin = self.planAttempts > 4 and 0 or ClearHarvesterZoneTask.HELPER_ZONE_PARK_MARGIN
    local shortestZoneExit = math.min(self.helperZone.halfWidth, self.helperZone.halfLength)
    local requiredDistance = shortestZoneExit + trainLength + cellSize * 2 + parkMargin
    local maxRadius = math.clamp(requiredDistance + 10,
        ClearHarvesterZoneTask.MIN_RADIUS, ClearHarvesterZoneTask.MAX_RADIUS)
    local maxPathLength = math.clamp(requiredDistance + 25,
        ClearHarvesterZoneTask.MIN_PATH_LENGTH, ClearHarvesterZoneTask.MAX_PATH_LENGTH)

    -- Zone is snapshotted when infringement is detected. This keeps search geometry stable even
    -- while the harvester attempts to resume work.
    self.escapeJob = ADEscapeCourseGenerator.begin(self.vehicle, ADEscapeCourseGenerator.TARGET_FRUIT_FREE, {
        exclusionZone = AutoDrive.getCombineExclusionZone(self.harvester),
        combine = self.harvester,
        helperZone = self.helperZone,
        allowFruitAtTarget = self.loopEscape,
        targetFruitClearance = ClearHarvesterZoneTask.TARGET_FRUIT_CLEARANCE,
        helperZoneTargetMargin = parkMargin,
        targetContinuationDistance = self.loopEscape and 0 or self:getTargetContinuationDistance(trainLength, cellSize),
        fruitCellCost = ClearHarvesterZoneTask.FRUIT_CELL_COST,
        maxPathLength = maxPathLength,
        maxRadius = maxRadius,
        maxCells = ClearHarvesterZoneTask.MAX_CELLS
    })
    self.state = ClearHarvesterZoneTask.STATE_PLANNING
    self.stallTimer:timer(false)
    self.retryTimer:timer(false)
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
        self.stallTimer:timer(false)
        self.state = ClearHarvesterZoneTask.STATE_PRE_REVERSING
    else
        self:replanOrFinish(false)
    end
end

function ClearHarvesterZoneTask:update(dt)
    if self.harvester == nil or self.harvester.components == nil or self.harvester.components[1] == nil
        or g_currentMission.nodeToObject[self.harvester.components[1].node] == nil then
        self:finished()
        return
    end

    -- Not while straightening: the clear condition is already met there by definition, the
    -- remaining forward pull must not be restarted or cut short by this check.
    if self.state ~= ClearHarvesterZoneTask.STATE_STRAIGHTENING
        and self.clearCheckTimer:timer(true, ClearHarvesterZoneTask.CLEAR_CHECK_TIME, dt) then
        self.clearCheckTimer:timer(false)
        if self:isTrainClear() then
            self:finishOrStraighten()
            return
        end
    end

    if self.state ~= ClearHarvesterZoneTask.STATE_PLANNING then
        ADEscapeCourseGenerator.drawHelperZone(self.helperZone)
        ADEscapeCourseGenerator.drawCourse(self.course, 0, 1, 1)
    end

    if self.state == ClearHarvesterZoneTask.STATE_PRE_REVERSING then
        local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
        local reverseDistance = MathUtil.vector2Length(x - self.reverseStartLocation.x, z - self.reverseStartLocation.z)
        self.stallTimer:timer(self.vehicle.lastSpeedReal <= 0.0002, ClearHarvesterZoneTask.STALL_TIME, dt)
        if reverseDistance >= ClearHarvesterZoneTask.PRE_REVERSE_DISTANCE
            or not AutoDrive.isVehicleTrainInHelperZone(self.vehicle, self.helperZone)
            or self.vehicle.ad.sensors.rearSensor:pollInfo()
            or self.stallTimer:done() then
            self:replanOrFinish(false)
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
                self.course = course
                self.vehicle.ad.drivePathModule:setWayPoints(course)
                self.state = ClearHarvesterZoneTask.STATE_DRIVING
            else
                if not self.loopEscape then
                    -- No local fruit-free parking position: perform one short side escape, then
                    -- continue forward from there toward a fruit-free local parking position.
                    self:replanOrFinish(true)
                else
                    self.state = ClearHarvesterZoneTask.STATE_RETRY_WAIT
                    self.retryTimer:timer(false)
                end
            end
        end
    elseif self.state == ClearHarvesterZoneTask.STATE_RETRY_WAIT then
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        if self.retryTimer:timer(true, ClearHarvesterZoneTask.RETRY_TIME, dt) then
            self.helperZone = AutoDrive.getCombineHelperZone(self.harvester)
            self:replanOrFinish(false)
        end
    elseif self.state == ClearHarvesterZoneTask.STATE_DRIVING then
        self.stallTimer:timer(self.vehicle.lastSpeedReal <= 0.0002, ClearHarvesterZoneTask.STALL_TIME, dt)
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            if self.loopEscape then
                self.state = ClearHarvesterZoneTask.STATE_LOOP_WAIT
                self.loopWaitTimer:timer(false)
                return
            end
            -- Completing a course is real progress - grant a fresh planning budget for the
            -- follow-up legs below instead of counting them toward the give-up limit.
            self.planAttempts = 0
            local currentHelperZone = AutoDrive.getCombineHelperZone(self.harvester)
            if AutoDrive.isVehicleTrainInCrop(self.vehicle, false) then
                -- Dijkstra's center-line corridor is only an approximation for articulated
                -- trains. Verify actual tractor and every trailer (strict boxes - the enlarged
                -- check trips on crop edges the train has already left), then continue forward
                -- until the real train is completely clear of crop.
                self.helperZone = currentHelperZone
                self:replanOrFinish(false)
            elseif AutoDrive.isVehicleTrainInHelperZone(self.vehicle, currentHelperZone) then
                -- Harvester may have moved while this snapshot course was driven. Never park in
                -- its current helper zone; replan locally from the new relative position.
                self.helperZone = currentHelperZone
                self:replanOrFinish(false)
            else
                self:finishOrStraighten()
            end
        elseif self.stallTimer:done() then
            self.stallTimer:timer(false)
            local rearBlocked = self.vehicle.ad.sensors.rearSensor:pollInfo()
            if self.vehicle.ad.trailerModule:canBeHandledInReverse() and not rearBlocked then
                local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
                self.reverseStartLocation = {x = x, z = z}
                self.state = ClearHarvesterZoneTask.STATE_PRE_REVERSING
            else
                -- Blocked ahead and unable to reverse: replan around the obstacle from the
                -- current position (fresh zone in case the harvester moved) instead of giving
                -- up - the obstacle is usually another vehicle that the new course can avoid.
                self.helperZone = AutoDrive.getCombineHelperZone(self.harvester) or self.helperZone
                self:replanOrFinish(false)
            end
        else
            -- Escape courses legitimately spend long stretches without closing on the next
            -- waypoint (tight trailer turns between coarse Dijkstra cells), which would trip
            -- the generic "got stuck" watchdog and abort AD entirely. Real blockages are
            -- covered by the stall handling above, so keep the watchdog quiet here.
            self.vehicle.ad.drivePathModule.minDistanceTimer:timer(false)
            self.vehicle.ad.drivePathModule.minDistanceToNextWp = math.huge
            self.vehicle.ad.drivePathModule:update(dt)
        end
    elseif self.state == ClearHarvesterZoneTask.STATE_STRAIGHTENING then
        self.stallTimer:timer(self.vehicle.lastSpeedReal <= 0.0002, ClearHarvesterZoneTask.STALL_TIME, dt)
        local aheadPoint = AutoDrive.createWayPointRelativeToVehicle(self.vehicle, 0, 10)
        local currentHelperZone = AutoDrive.getCombineHelperZone(self.harvester)
        -- Best effort: stop straightening rather than drive into crop, an obstacle or back
        -- toward the harvester - whatever straightness has been reached by then has to do.
        if self.vehicle.ad.drivePathModule:isTargetReached()
            or self:isTrainStraight()
            or self.stallTimer:done()
            or self.vehicle.ad.sensors.frontSensorFruit:pollInfo()
            or self.vehicle.ad.sensors.frontSensor:pollInfo()
            or AutoDrive.isPointInHelperZone(currentHelperZone, aheadPoint.x, aheadPoint.z, 0) then
            self.straightened = true
            self:finished()
        else
            -- same watchdog suppression as STATE_DRIVING
            self.vehicle.ad.drivePathModule.minDistanceTimer:timer(false)
            self.vehicle.ad.drivePathModule.minDistanceToNextWp = math.huge
            self.vehicle.ad.drivePathModule:update(dt)
        end
    elseif self.state == ClearHarvesterZoneTask.STATE_LOOP_WAIT then
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        local currentHelperZone = AutoDrive.getCombineHelperZone(self.harvester)
        if self.loopWaitTimer:timer(true, ClearHarvesterZoneTask.LOOP_MIN_WAIT, dt) then
            -- Do not drive the outward waypoint list backwards. A reversed point list is not a
            -- valid trailer manoeuvre and caused the stuck detection. Continue forward from the
            -- escape point toward a locally fruit-free parking position instead.
            self.helperZone = currentHelperZone
            self:replanOrFinish(false)
        end
    end
end

function ClearHarvesterZoneTask:abort()
    self.escapeJob = nil
    self.course = nil
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
    if self.state == ClearHarvesterZoneTask.STATE_RETRY_WAIT then
        return "$l10n_AD_task_pathfinding; - Helferzone (erneut)"
    end
    if self.state == ClearHarvesterZoneTask.STATE_LOOP_WAIT then
        return "$l10n_AD_task_clearcrop; - Helferzone (suche Parkposition)"
    end
    if self.state == ClearHarvesterZoneTask.STATE_STRAIGHTENING then
        return "$l10n_AD_task_clearcrop; - Helferzone (ziehe gerade)"
    end
    return "$l10n_AD_task_clearcrop; - Helferzone"
end
