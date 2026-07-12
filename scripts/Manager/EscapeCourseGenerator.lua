-- ADEscapeCourseGenerator
--
-- Generates short, safe escape courses on open ground (no waypoint network required) by running
-- a Dijkstra search over a locally sampled grid of the world around the vehicle. Every cell on
-- the resulting course has been checked for static collisions and other vehicles before it is
-- returned, and the course is connected by construction - so unlike blind fixed-offset "hop"
-- targets it can neither lead into an unreachable pocket nor through an obstacle.
--
-- Costs are soft for fruit and the harvester exclusion zone (the vehicle usually starts inside
-- both, so they must be crossable - just expensive), hard for static collisions and other
-- vehicles; off-field ground is impassable except as an explicit TARGET_OFF_FIELD goal.
--
-- The search runs incrementally: ADEscapeCourseGenerator.begin() returns a job whose update()
-- evaluates at most CELLS_PER_FRAME grid cells per call (each evaluation costs one overlapBox
-- and one density-map read - doing all of them in a single frame caused visible stutter).
-- Callers poll update() once per frame until isFinished(), then read getCourse().
--
-- Consumers:
--   - ClearCropTask: TARGET_FRUIT_FREE - nearest fruit-free spot on the field, clear of harvester
--   - ExitFieldTask: TARGET_OFF_FIELD - nearest spot off the field (fallback when no network
--     entry candidate can be reached)
--   - EmptyHarvesterTask: TARGET_CLEAR_OF_ZONE - safe-distance spot away from the harvester

ADEscapeCourseGenerator = {}

ADEscapeCourseGenerator.MAX_CELLS = 600 -- default cap on evaluated cells per job (search budget)
ADEscapeCourseGenerator.CELLS_PER_FRAME = 25 -- max cell evaluations per update() call (perf guard)
ADEscapeCourseGenerator.MIN_CELL_SIZE = 4 -- m
ADEscapeCourseGenerator.MAX_CELL_SIZE = 8 -- m
ADEscapeCourseGenerator.FRUIT_CELL_COST = 20 -- multiplier: crossing fruit is possible but expensive - prefer detours up to ~20x the direct distance around standing crop
ADEscapeCourseGenerator.ZONE_CELL_COST = 6 -- multiplier: crossing the harvester exclusion zone is possible but expensive
ADEscapeCourseGenerator.COMBINE_FRONT_CELL_PENALTY = 1000000 -- use ground ahead of harvester only when no other course exists
ADEscapeCourseGenerator.WRONG_SIDE_CELL_COST = 4 -- multiplier: cells opposite the preferred (pipe) side of the combine cost extra - below FRUIT_CELL_COST so avoiding crop still wins over side preference
ADEscapeCourseGenerator.HELPER_ZONE_CENTER_COST = 250 -- increasingly expensive toward rectangle center
ADEscapeCourseGenerator.HELPER_ZONE_DEEPER_PENALTY = 100000 -- entering deeper than start is last resort
ADEscapeCourseGenerator.DEBUG_MAX_CELLS = 400

ADEscapeCourseGenerator.TARGET_FRUIT_FREE = 1 -- reach a fruit-free cell (stay on field)
ADEscapeCourseGenerator.TARGET_OFF_FIELD = 2 -- reach a cell just off the field
ADEscapeCourseGenerator.TARGET_CLEAR_OF_ZONE = 3 -- reach a fruit-free cell outside the exclusion zone (leave-harvester maneuver)

local sqrt2 = math.sqrt(2)

-- Neighbor offsets: 4-connected first (cheaper), then diagonals
local NEIGHBORS = {
    {dx = 1, dz = 0, cost = 1}, {dx = -1, dz = 0, cost = 1},
    {dx = 0, dz = 1, cost = 1}, {dx = 0, dz = -1, cost = 1},
    {dx = 1, dz = 1, cost = sqrt2}, {dx = 1, dz = -1, cost = sqrt2},
    {dx = -1, dz = 1, cost = sqrt2}, {dx = -1, dz = -1, cost = sqrt2}
}

local EscapeJob = {}
EscapeJob.__index = EscapeJob

function ADEscapeCourseGenerator.isDebugEnabled()
    return AutoDrive.isEditorModeEnabled() and AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO)
end

function ADEscapeCourseGenerator.drawCourse(course, r, g, b)
    if not ADEscapeCourseGenerator.isDebugEnabled() or course == nil then
        return
    end
    r, g, b = r or 0, g or 1, b or 1
    for index = 2, #course do
        local previous = course[index - 1]
        local current = course[index]
        ADDrawingManager:addLineTask(previous.x, previous.y, previous.z, current.x, current.y, current.z, 2, r, g, b)
        ADDrawingManager:addArrowTask(previous.x, previous.y, previous.z, current.x, current.y, current.z, 2, ADDrawingManager.arrows.position.start, r, g, b)
    end
    local target = course[#course]
    if target ~= nil then
        ADDrawingManager:addSphereTask(target.x, target.y, target.z, 2, r, g, b, 0.35)
    end
end

function ADEscapeCourseGenerator.drawHelperZone(zone)
    if not ADEscapeCourseGenerator.isDebugEnabled() or zone == nil then
        return
    end
    local corners = {}
    local signs = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}
    for index, sign in ipairs(signs) do
        local x = zone.x + zone.rightX * zone.halfWidth * sign[1] + zone.forwardX * zone.halfLength * sign[2]
        local z = zone.z + zone.rightZ * zone.halfWidth * sign[1] + zone.forwardZ * zone.halfLength * sign[2]
        local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 0, z)
        corners[index] = {x = x, y = y, z = z}
    end
    for index = 1, 4 do
        local nextIndex = index % 4 + 1
        ADDrawingManager:addLineTask(corners[index].x, corners[index].y, corners[index].z, corners[nextIndex].x, corners[nextIndex].y, corners[nextIndex].z, 3, 1, 0, 1)
    end
end

-- Starts a new incremental escape course search.
--   vehicle: the escaping vehicle
--   targetType: TARGET_FRUIT_FREE, TARGET_OFF_FIELD or TARGET_CLEAR_OF_ZONE
--   options (all optional): exclusionZone = {x, z, radius}, combine = vehicle,
--                           helperZone, combineFrontPenalty, targetFruitClearance (m),
--                           targetContinuationDistance (m), allowFruitAtTarget, fruitCellCost,
--                           helperZoneTargetMargin (m), maxPathLength (m), maxRadius (m), maxCells,
--                           preferredSide (-1 right / 1 left of the combine, e.g. its pipe side:
--                           cells on the opposite side of the combine's long axis cost extra, so the
--                           course curves away toward that side; requires combine)
-- Returns a job. Call job:update() once per frame until job:isFinished(), then job:getCourse()
-- returns a waypoint list ({x,y,z}, ...) or nil if no valid course was found within budget.
function ADEscapeCourseGenerator.begin(vehicle, targetType, options)
    options = options or {}
    local job = setmetatable({}, EscapeJob)
    job.vehicle = vehicle
    job.targetType = targetType
    job.exclusionZone = options.exclusionZone
    job.combine = options.combine
    job.helperZone = options.helperZone
    job.startedInHelperZone = job.helperZone ~= nil and AutoDrive.isVehicleTrainInHelperZone(vehicle, job.helperZone)
    job.combineFrontPenalty = options.combineFrontPenalty or ADEscapeCourseGenerator.COMBINE_FRONT_CELL_PENALTY
    job.fruitCellCost = options.fruitCellCost or ADEscapeCourseGenerator.FRUIT_CELL_COST
    job.helperZoneTargetMargin = options.helperZoneTargetMargin or 0
    job.allowFruitAtTarget = options.allowFruitAtTarget == true
    job.maxPathLength = options.maxPathLength
    job.targetFruitClearance = options.targetFruitClearance or 0
    job.targetContinuationDistance = options.targetContinuationDistance or 0
    job.maxCells = options.maxCells or ADEscapeCourseGenerator.MAX_CELLS
    job.cells = {}
    job.evaluatedCells = 0
    job.collisionHits = 0
    job.finished = false
    job.course = nil

    local width = math.max(vehicle.size.width, 3)
    job.cellSize = math.clamp(width + 1, ADEscapeCourseGenerator.MIN_CELL_SIZE, ADEscapeCourseGenerator.MAX_CELL_SIZE)
    job.trainLength = AutoDrive.getTractorTrainLength(vehicle, true, false) or vehicle.size.length
    local maxRadius = options.maxRadius or 150
    job.maxGridRadius = math.ceil(maxRadius / job.cellSize)

    local vehicleX, _, vehicleZ = getWorldTranslation(vehicle.components[1].node)
    job.originX = vehicleX
    job.originZ = vehicleZ
    if job.helperZone ~= nil then
        local startLocalX, startLocalZ = AutoDrive.getHelperZoneLocalPosition(job.helperZone, vehicleX, vehicleZ)
        job.helperStartProgress = math.max(math.abs(startLocalX) / job.helperZone.halfWidth, math.abs(startLocalZ) / job.helperZone.halfLength)
    end

    local rx, _, rz = AutoDrive.localDirectionToWorld(vehicle, 0, 0, 1)
    job.headingX = rx
    job.headingZ = rz

    -- Side preference: bias the search toward one side of the combine (usually its pipe side).
    -- Snapshotted here - the combine may move while the incremental search runs.
    local preferredSide = options.preferredSide or 0
    if preferredSide ~= 0 and job.combine ~= nil and job.combine.components ~= nil and job.combine.components[1] ~= nil then
        local combineX, _, combineZ = getWorldTranslation(job.combine.components[1].node)
        local sideX, _, sideZ = AutoDrive.localDirectionToWorld(job.combine, preferredSide, 0, 0)
        job.preferredSideRefX = combineX
        job.preferredSideRefZ = combineZ
        job.preferredSideDirX = sideX
        job.preferredSideDirZ = sideZ
    end

    local startCell = job:evaluateCell(0, 0)
    -- the start cell is where the vehicle already stands - accept it even if it reads as
    -- blocked/fruit (a nearby harvester may trip the collision check), it is never part of the
    -- returned course. If it was blocked, the field/fruit/zone flags were skipped - fill them in.
    if startCell.blocked then
        startCell.blocked = false
        startCell.onField = job:cellIsOnField(startCell.worldX, startCell.worldZ)
        startCell.fruit = job:cellHasFruit(startCell.worldX, startCell.worldZ, job.cellSize / 2)
        startCell.inZone = job.exclusionZone ~= nil
            and MathUtil.vector2Length(startCell.worldX - job.exclusionZone.x, startCell.worldZ - job.exclusionZone.z) < job.exclusionZone.radius
    end
    startCell.cost = 0
    startCell.pathDistance = 0
    startCell.parent = nil

    job.startCell = startCell
    job.openList = {startCell}
    job.closed = {}
    return job
end

function EscapeJob:isFinished()
    return self.finished
end

function EscapeJob:getCourse()
    return self.course
end

function EscapeJob:drawDebug()
    if not ADEscapeCourseGenerator.isDebugEnabled() then
        return
    end
    ADEscapeCourseGenerator.drawHelperZone(self.helperZone)

    local drawEvery = math.max(1, math.ceil(math.max(self.evaluatedCells, 1) / ADEscapeCourseGenerator.DEBUG_MAX_CELLS))
    local cellIndex = 0
    for key, cell in pairs(self.cells) do
        cellIndex = cellIndex + 1
        if cellIndex % drawEvery == 0 then
            local r, g, b = 0, 1, 0
            if cell.blocked then
                r, g, b = 1, 0, 0
            elseif cell.inHelperZone then
                r, g, b = 1, 0, 1
            elseif cell.inZone then
                r, g, b = 1, 0.45, 0
            elseif cell.fruit then
                r, g, b = 1, 0.9, 0
            elseif self.closed[key] then
                r, g, b = 0.1, 0.4, 1
            end
            local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, cell.worldX, 0, cell.worldZ)
            ADDrawingManager:addSmallSphereTask(cell.worldX, y, cell.worldZ, r, g, b)
            if cell.parent ~= nil and not cell.blocked then
                local parentY = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, cell.parent.worldX, 0, cell.parent.worldZ)
                ADDrawingManager:addLineTask(cell.parent.worldX, parentY, cell.parent.worldZ, cell.worldX, y, cell.worldZ, 0.35, r, g, b)
            end
        end
    end
    ADEscapeCourseGenerator.drawCourse(self.course)
    if self.course ~= nil and self.course[1] ~= nil then
        local originY = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, self.originX, 0, self.originZ)
        ADDrawingManager:addLineTask(self.originX, originY, self.originZ, self.course[1].x, self.course[1].y, self.course[1].z, 2, 0, 1, 1)
        ADDrawingManager:addArrowTask(self.originX, originY, self.originZ, self.course[1].x, self.course[1].y, self.course[1].z, 2, ADDrawingManager.arrows.position.start, 0, 1, 1)
    end
end

-- overlapBox callback: counts shapes that don't belong to the searching vehicle's train.
-- The harvester is deliberately NOT excluded - driving into it is exactly what we're avoiding.
-- Overrunnable shapes (knock-over props like traffic signs, light pushable objects) are
-- filtered out via the shared sensor logic, they pose no risk of getting stuck.
function EscapeJob:collisionTestCallback(transformId)
    if transformId ~= 0 and transformId ~= g_currentMission.terrainRootNode then
        if self.vehicle.ad.sensors == nil then
            ADSensor:addSensorsToVehicle(self.vehicle)
        end
        local sensor = self.vehicle.ad.sensors.frontSensorDynamicShort
        if sensor:isElementBlockingVehicle(transformId) then
            local collisionObject = g_currentMission:getNodeObject(transformId)
            if collisionObject == nil or (collisionObject ~= self.vehicle and not AutoDrive:checkIsConnected(self.vehicle, collisionObject)) then
                self.collisionHits = self.collisionHits + 1
            end
        end
    end
end

function EscapeJob:cellHasStaticCollision(worldX, worldZ, halfSize)
    self.collisionHits = 0
    local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, worldX, 0, worldZ)
    overlapBox(worldX, y + 3, worldZ, 0, 0, 0, halfSize, 2.65, halfSize, "collisionTestCallback", self, AutoDrive.collisionMaskTerrain, true, true, true, true)
    return self.collisionHits > 0
end

function EscapeJob:cellHasFruit(worldX, worldZ, halfSize)
    local corners = {
        {x = worldX - halfSize, z = worldZ - halfSize},
        {x = worldX + halfSize, z = worldZ - halfSize},
        {x = worldX - halfSize, z = worldZ + halfSize}
    }
    local hasFruit, _ = AutoDrive.checkForUnknownFruitInArea(corners)
    return hasFruit
end

function EscapeJob:cellHasFruitAtTargetClearance(cell)
    if self.targetFruitClearance <= 0 then
        return cell.fruit
    end
    if cell.fruitAtTargetClearance == nil then
        cell.fruitAtTargetClearance = self:cellHasFruit(cell.worldX, cell.worldZ, self.cellSize / 2 + self.targetFruitClearance)
    end
    return cell.fruitAtTargetClearance
end

function EscapeJob:cellIsOnField(worldX, worldZ)
    local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, worldX, 0, worldZ)
    return AutoDrive.checkIsOnField(worldX, y, worldZ)
end

function EscapeJob:cellIsInFrontOfCombine(worldX, worldZ)
    if self.combine == nil or self.combine.components == nil or self.combine.components[1] == nil then
        return false
    end
    local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, worldX, 0, worldZ)
    local rootNode = self.combine.components[1].node
    if self.combine.ad ~= nil and self.combine.ad.ADRootNode ~= nil then
        rootNode = self.combine.ad.ADRootNode
    end
    local _, _, localZ = AutoDrive.worldToLocal(self.combine, worldX, y, worldZ, rootNode)
    return localZ > 0
end

-- Evaluates (and caches) a grid cell: blocked (hard), fruit/inZone (soft cost), onField
function EscapeJob:evaluateCell(gridX, gridZ)
    local key = gridX .. "|" .. gridZ
    local cached = self.cells[key]
    if cached ~= nil then
        return cached
    end

    local worldX = self.originX + gridX * self.cellSize
    local worldZ = self.originZ + gridZ * self.cellSize
    local halfSize = self.cellSize / 2

    local cell = {gridX = gridX, gridZ = gridZ, worldX = worldX, worldZ = worldZ}

    if self:cellHasStaticCollision(worldX, worldZ, halfSize) then
        -- overlapBox with the terrain collision mask also reports other vehicles' physics shapes,
        -- so parked/working vehicles (incl. the harvester) block cells without a separate check
        cell.blocked = true
    else
        cell.blocked = false
        cell.onField = self:cellIsOnField(worldX, worldZ)
        cell.fruit = self:cellHasFruit(worldX, worldZ, halfSize)
        -- The exclusion zone is soft, not a hard block: the vehicle usually STARTS inside it
        -- (right next to the harvester), so the course must be allowed to lead through/out of
        -- it - it just must not end there, and crossing it costs extra. Actual contact with the
        -- harvester is prevented by the static collision check above.
        cell.inZone = self.exclusionZone ~= nil
            and MathUtil.vector2Length(worldX - self.exclusionZone.x, worldZ - self.exclusionZone.z) < self.exclusionZone.radius
        if self.helperZone ~= nil then
            cell.helperLocalX, cell.helperLocalZ = AutoDrive.getHelperZoneLocalPosition(self.helperZone, worldX, worldZ)
            cell.inHelperZone = math.abs(cell.helperLocalX) <= self.helperZone.halfWidth + halfSize
                and math.abs(cell.helperLocalZ) <= self.helperZone.halfLength + halfSize
            local isSideExit = math.abs(cell.helperLocalX) > self.helperZone.halfWidth + halfSize
                and math.abs(cell.helperLocalZ) <= self.helperZone.halfLength + halfSize
            local isRearExit = cell.helperLocalZ < -self.helperZone.halfLength - halfSize
                and math.abs(cell.helperLocalX) <= self.helperZone.halfWidth + halfSize
            cell.isHelperSafeExit = isSideExit or isRearExit
        else
            cell.inHelperZone = false
            cell.isHelperSafeExit = true
        end
    end

    self.evaluatedCells = self.evaluatedCells + 1
    self.evaluationsThisUpdate = (self.evaluationsThisUpdate or 0) + 1
    self.cells[key] = cell
    return cell
end

function EscapeJob:isTargetCell(cell)
    local invalidHelperTarget = self.helperZone ~= nil
        and (cell.inHelperZone or (self.startedInHelperZone and not cell.isHelperSafeExit))
    -- Optional standoff: don't park right at the helper-zone boundary, where the next small
    -- harvester movement would sweep the zone over the vehicle again and restart the escape.
    if not invalidHelperTarget and self.helperZone ~= nil and self.helperZoneTargetMargin > 0
        and cell.helperLocalX ~= nil then
        invalidHelperTarget = math.abs(cell.helperLocalX) <= self.helperZone.halfWidth + self.helperZoneTargetMargin
            and math.abs(cell.helperLocalZ) <= self.helperZone.halfLength + self.helperZoneTargetMargin
    end
    if cell.blocked or cell.inZone or invalidHelperTarget then
        return false
    end
    if self.targetType == ADEscapeCourseGenerator.TARGET_FRUIT_FREE
        or self.targetType == ADEscapeCourseGenerator.TARGET_CLEAR_OF_ZONE then
        if not cell.onField or (cell.fruit and not self.allowFruitAtTarget) then
            return false
        end
        -- Require a train-length clear corridor at the end of the actual approach path. A square
        -- clearance margin rejected every normal harvested strip, while this proves the whole
        -- train (not only tractor origin) is clear without demanding unused space at both sides.
        local clearDistance = 0
        local corridorCell = cell
        while corridorCell ~= nil and clearDistance < self.trainLength do
            if corridorCell.blocked or (not self.allowFruitAtTarget and self:cellHasFruitAtTargetClearance(corridorCell))
                or corridorCell.inZone or corridorCell.inHelperZone then
                return false
            end
            local parent = corridorCell.parent
            if parent == nil then
                return false
            end
            clearDistance = clearDistance + MathUtil.vector2Length(corridorCell.worldX - parent.worldX, corridorCell.worldZ - parent.worldZ)
            corridorCell = parent
        end

        -- Require and remember a straight clear continuation beyond the target. Normally one
        -- cell is enough. Articulated trains may request more so trailers straighten and leave
        -- crop without starting another turning Dijkstra course at the nominal target.
        local parent = cell.parent
        if parent == nil then
            return false
        end
        local forwardDx = cell.gridX - parent.gridX
        local forwardDz = cell.gridZ - parent.gridZ
        local stepDistance = MathUtil.vector2Length(forwardDx * self.cellSize, forwardDz * self.cellSize)
        local requiredContinuation = math.max(stepDistance, self.targetContinuationDistance)
        local continuationDistance = 0
        local continuationStep = 1
        local continuation = nil
        while continuationDistance < requiredContinuation do
            local continuationGridX = cell.gridX + forwardDx * continuationStep
            local continuationGridZ = cell.gridZ + forwardDz * continuationStep
            continuation = self.cells[continuationGridX .. "|" .. continuationGridZ]
            if continuation == nil then
                if self.evaluatedCells >= self.maxCells then
                    return false
                end
                continuation = self:evaluateCell(continuationGridX, continuationGridZ)
            end
            if continuation.blocked or (not self.allowFruitAtTarget and self:cellHasFruitAtTargetClearance(continuation))
                or continuation.inZone or continuation.inHelperZone or not continuation.onField then
                return false
            end
            continuationDistance = continuationDistance + stepDistance
            continuationStep = continuationStep + 1
        end
        if self.targetContinuationDistance > 0 then
            cell.targetContinuationEnd = continuation
        end
        return true
    elseif self.targetType == ADEscapeCourseGenerator.TARGET_OFF_FIELD then
        return not cell.onField
    end
    return false
end

-- Traversable as an intermediate course cell (target validity is checked separately)
function EscapeJob:isTraversable(cell)
    if cell.blocked then
        return false
    end
    if self.helperZone ~= nil and not self.startedInHelperZone and cell.inHelperZone then
        -- Course started outside: never enter helper clearance rectangle.
        return false
    end
    if self.targetType == ADEscapeCourseGenerator.TARGET_OFF_FIELD then
        -- off-field cells are only valid as the final cell; on the way we stay on field
        return cell.onField or self:isTargetCell(cell)
    end
    -- fruit-free / clear-of-zone targets: must stay on the field the whole way
    return cell.onField
end

function EscapeJob:cellStepCost(cell, baseCost)
    local cost = baseCost
    if cell.fruit then
        cost = cost * self.fruitCellCost
    end
    if cell.inZone then
        cost = cost * ADEscapeCourseGenerator.ZONE_CELL_COST
    end
    if self.helperZone ~= nil and cell.inHelperZone then
        local progress = math.max(math.abs(cell.helperLocalX) / self.helperZone.halfWidth, math.abs(cell.helperLocalZ) / self.helperZone.halfLength)
        cost = cost + (1 - math.clamp(progress, 0, 1)) * ADEscapeCourseGenerator.HELPER_ZONE_CENTER_COST
        if self.helperStartProgress ~= nil and progress + 0.05 < self.helperStartProgress then
            cost = cost + ADEscapeCourseGenerator.HELPER_ZONE_DEEPER_PENALTY
        end
    end
    if self.preferredSideDirX ~= nil then
        local lateral = (cell.worldX - self.preferredSideRefX) * self.preferredSideDirX + (cell.worldZ - self.preferredSideRefZ) * self.preferredSideDirZ
        if lateral < 0 then
            cost = cost * ADEscapeCourseGenerator.WRONG_SIDE_CELL_COST
        end
    end
    if self:cellIsInFrontOfCombine(cell.worldX, cell.worldZ) then
        -- Default penalty dominates normal routes. Callers may lower it only for a final pass
        -- after strict behind/beside searches found no course.
        cost = cost + self.combineFrontPenalty
    end
    return cost
end

-- Builds the waypoint list from the parent chain, dropping collinear intermediate points.
function EscapeJob:buildCourse(targetCell)
    local reversedCells = {}
    local current = targetCell
    while current ~= nil do
        table.insert(reversedCells, current)
        current = current.parent
    end

    local wayPoints = {}
    local count = #reversedCells
    -- skip the start cell itself (index count) - the vehicle already stands there
    for index = count - 1, 1, -1 do
        local cell = reversedCells[index]
        local previous = wayPoints[#wayPoints]
        local nextCell = index > 1 and reversedCells[index - 1] or nil
        local keep = true
        if previous ~= nil and nextCell ~= nil then
            local dirInX, dirInZ = cell.worldX - previous.x, cell.worldZ - previous.z
            local dirOutX, dirOutZ = nextCell.worldX - cell.worldX, nextCell.worldZ - cell.worldZ
            -- drop point if directions are collinear (cross product ~ 0)
            keep = math.abs(dirInX * dirOutZ - dirInZ * dirOutX) > 0.1
        end
        if keep then
            local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, cell.worldX, 0, cell.worldZ)
            table.insert(wayPoints, {x = cell.worldX, y = y, z = cell.worldZ})
        end
    end
    local continuation = targetCell.targetContinuationEnd
    if continuation ~= nil and continuation ~= targetCell then
        local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, continuation.worldX, 0, continuation.worldZ)
        table.insert(wayPoints, {x = continuation.worldX, y = y, z = continuation.worldZ})
    end
    if #wayPoints == 0 then
        return nil
    end
    return wayPoints
end

-- Advances the Dijkstra search by at most CELLS_PER_FRAME cell evaluations. Call once per frame.
function EscapeJob:update()
    if self.finished then
        self:drawDebug()
        return
    end
    self:drawDebug()
    self.evaluationsThisUpdate = 0

    while #self.openList > 0 do
        -- extract cheapest (list stays small - bounded by job.maxCells)
        local bestIndex = 1
        for index = 2, #self.openList do
            if self.openList[index].cost < self.openList[bestIndex].cost then
                bestIndex = index
            end
        end
        local currentCell = table.remove(self.openList, bestIndex)
        local currentKey = currentCell.gridX .. "|" .. currentCell.gridZ

        if not self.closed[currentKey] then
            self.closed[currentKey] = true

            if currentCell ~= self.startCell and self:isTargetCell(currentCell) then
                self.course = self:buildCourse(currentCell)
                self.finished = true
                self:drawDebug()
                return
            end

            for _, neighborOffset in pairs(NEIGHBORS) do
                local neighborGridX = currentCell.gridX + neighborOffset.dx
                local neighborGridZ = currentCell.gridZ + neighborOffset.dz
                local withinRadius = math.abs(neighborGridX) <= self.maxGridRadius and math.abs(neighborGridZ) <= self.maxGridRadius
                if withinRadius and self.evaluatedCells < self.maxCells then
                    -- from the start cell, don't expand straight backwards - the course is driven
                    -- forwards and shouldn't begin with a full turnaround
                    local backwards = false
                    if currentCell == self.startCell then
                        local dot = neighborOffset.dx * self.headingX + neighborOffset.dz * self.headingZ
                        backwards = dot < -0.5
                    end
                    if not backwards then
                        local neighbor = self:evaluateCell(neighborGridX, neighborGridZ)
                        local neighborKey = neighborGridX .. "|" .. neighborGridZ
                        if not self.closed[neighborKey] and self:isTraversable(neighbor) then
                            local newPathDistance = (currentCell.pathDistance or 0) + neighborOffset.cost * self.cellSize
                            local newCost = currentCell.cost + self:cellStepCost(neighbor, neighborOffset.cost)
                            if (self.maxPathLength == nil or newPathDistance <= self.maxPathLength)
                                and (neighbor.cost == nil or newCost < neighbor.cost) then
                                neighbor.cost = newCost
                                neighbor.pathDistance = newPathDistance
                                neighbor.parent = currentCell
                                table.insert(self.openList, neighbor)
                            end
                        end
                    end
                end
            end
        end

        if self.evaluatedCells >= self.maxCells then
            break
        end
        if self.evaluationsThisUpdate >= ADEscapeCourseGenerator.CELLS_PER_FRAME then
            -- frame budget spent - continue next update()
            return
        end
    end

    -- open list exhausted or cell budget spent without reaching a target
    self.finished = true
    self.course = nil
end
