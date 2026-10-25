--[[
New to Pathfinder:
- 3 Settings are considered: restrictToField, avoidFruit, pathFinderTime

1. restrictToField
The Pathfinder tries to find a path to target in the limit of the current field borders.
This is only possible if the vehicle is located inside a field.
If disabled, only setting avoidFruit limits or not, the shortest path to target will be calculated!

2. avoidFruit
The Pathfinder tries to find a path to target without passing through fruit.
This is effective if vehicle is inside a field.
NEW: Working also outside of a field, i.e. if possible a path around a field to network will be searched for!
If disabled, the shortest path to target will be calculated!

3. pathFinderTime
Is more a factor for the range the Pathfinder will search for path to target, default is 1.
In case fruit avoid on large fields with sufficient headland is not working, you may try to increase.
But be aware the calculation time will increase as well!

- 3 fallback scenario will automatic take effect:
If setting restrictToField is enabled, fallback 1 and 2 are possible:
fallback 1:
The first fallback will extend the field border by 30+ meters around the field pathfinding was started.
With this the vehicle will search for a path also around the field border!
30+ meters depend on the vehicle + trailer turn radius, if greater then extended wider.
fallback 2:
Second fallback will deactivate all field border restrictions, means setting avoidFruit will limit or not the search range for path.

fallback 3:
Third fallback will take effect only if setting avoidFruit is enabled.
It will disable fruit avoid automatically if no path was found.

Inside informations:
This is a calculation with the worst assumption of all cells to be checked:

Number of cells:
#cells = MAX_PATHFINDER_STEPS_PER_FRAME / 2 * MAX_PATHFINDER_STEPS_TOTAL * 3 (next directions - see determineNextGridCells)

PathFinderModule.MAX_PATHFINDER_STEPS_PER_FRAME = 10
PathFinderModule.MAX_PATHFINDER_STEPS_TOTAL = 400
#cells = 6000

with minTurnRadius = 7m calculated area:

cellsize = 7m * 7m = 49m^2
overall area = #cells * cellsize * pathFinderTime

with pathFinderTime = 1:
overall area = 6000 * 49 * 1 = 294000 m^2
for quadrat field layout: side length ~ 540m

with pathFinderTime = 2: side length ~ 760m
with pathFinderTime = 3: side length ~ 940m

This is inclusive of the field border cells!
]]

PathFinderModule = {}
PathFinderModule.debug = false

PathFinderModule.PATHFINDER_MAX_RETRIES = 3
PathFinderModule.MAX_PATHFINDER_STEPS_PER_FRAME = 2
PathFinderModule.MAX_PATHFINDER_STEPS_TOTAL = 400
PathFinderModule.MAX_PATHFINDER_STEPS_COMBINE_TURN = 100
PathFinderModule.PATHFINDER_FOLLOW_DISTANCE = 45
PathFinderModule.PATHFINDER_TARGET_DISTANCE = 7
PathFinderModule.PATHFINDER_TARGET_DISTANCE_PIPE = 16
PathFinderModule.PATHFINDER_TARGET_DISTANCE_PIPE_CLOSE = 6
PathFinderModule.PATHFINDER_START_DISTANCE = 7
PathFinderModule.MAX_FIELDBORDER_CELLS = 5
PathFinderModule.PATHFINDER_MIN_DISTANCE_START_TARGET = 50

PathFinderModule.PP_MIN_DISTANCE = 20
PathFinderModule.PP_CELL_X = 9
PathFinderModule.PP_CELL_Z = 9

PathFinderModule.GRID_SIZE_FACTOR = 0.5
PathFinderModule.GRID_SIZE_FACTOR_SECOND_UNLOADER = 1.1

PathFinderModule.PP_MAX_EAGER_LOOKAHEAD_STEPS = 10

PathFinderModule.MIN_FRUIT_VALUE = 50
PathFinderModule.SLOPE_DETECTION_THRESHOLD = math.rad(20)
PathFinderModule.NEW_PF_STEP_FACTOR = 20
-- A*: check fruit along the segment really driven between two cells, with the width of the train,
-- instead of a square of the turn radius size, so harvested strips narrower than the turn radius stay usable
PathFinderModule.FRUIT_CHECK_TRAIN_CORRIDOR = true
PathFinderModule.FRUIT_CHECK_CORRIDOR_MARGIN = 0.7
-- smaller grid cells when no path is found: 9 m -> 6 m -> 4.5 m -> 2 m (turns of the final path keep the real radius)
PathFinderModule.GRID_SCALES = {1, 2 / 3, 1 / 2}
PathFinderModule.GRID_FINEST_SIZE = 2
PathFinderModule.GRID_STEPS = #PathFinderModule.GRID_SCALES + 1

function PathFinderModule:getGridScale(step)
    return PathFinderModule.GRID_SCALES[step] or math.min(1, PathFinderModule.GRID_FINEST_SIZE / self.realTurnRadius)
end
--[[
from Giants Engine:
AITurnStrategy.SLOPE_DETECTION_THRESHOLD  = 0.5235987755983
]]

PathFinderModule.OBSTACLE_HEIGHT_MARGIN = 0.5
PathFinderModule.OBSTACLE_MIN_HEIGHT = 3

function PathFinderModule.getTrainHeight(vehicle)
    local height = PathFinderModule.OBSTACLE_MIN_HEIGHT
    local vehicles = {vehicle}
    if vehicle.getChildVehicles ~= nil then
        vehicles = vehicle:getChildVehicles()
    end
    for _, v in pairs(vehicles) do
        if v.size ~= nil and v.size.height ~= nil then
            height = math.max(height, v.size.height)
        end
    end
    return height
end

function PathFinderModule:new(vehicle)
    local o = {}
    setmetatable(o, self)
    self.__index = self
    o.vehicle = vehicle
    o.dubins = ADDubins:new()
    PathFinderModule.reset(o)
    return o
end

function PathFinderModule:reset()
    PathFinderModule.debugMsg(self.vehicle, "PFM:reset start")
    self.mask = AutoDrive.collisionMaskTerrain
    self.blockedZones = nil
    self.steps = 0
    self.grid = {}
    self.wayPoints = {}
    self.initNew = false
    self.path = {}
    self.diffOverallNetTime = 0
    self.retryCounter = 0
    self.delayTime = 0
    self.restrictToField = false
    self.avoidFruitSetting = false
    self.destinationId = 0
    self.fallBackMode1 = false
    self.fallBackMode2 = false
    self.fallBackMode3 = false
    self.isFinished = true
    self.smoothDone = true
    self.fruitAreas = {}
    self.goingToNetwork  = false
    self.goingToPipe = false
    self.noFallback = false -- set by the caller after reset(): never lift field / fruit restrictions, fail instead
    self.chasingVehicle = false
    self.isSecondChasingVehicle = false
    self.pathFinderTime = AutoDrive.getSetting("pathFinderTime")
    self.max_pathfinder_steps = 0
    -- obstacle check up to the real height of the train + 0.5 m: branches above it are not a wall
    self.vehicleMinHeight = PathFinderModule.getTrainHeight(self.vehicle) + PathFinderModule.OBSTACLE_HEIGHT_MARGIN

    if AutoDrive.getSetting("Pathfinder") == 1 then
        self.PP_UP = 0
        self.PP_UP_LEFT = 1
        self.PP_LEFT = 2
        self.PP_DOWN_LEFT = 3
        self.PP_DOWN = 4
        self.PP_DOWN_RIGHT = 5
        self.PP_RIGHT = 6
        self.PP_UP_RIGHT = 7
        self.direction_to_text = {
            "PP_UP",
            "PP_UP_LEFT",
            "PP_LEFT",
            "PP_DOWN_LEFT",
            "PP_DOWN",
            "PP_DOWN_RIGHT",
            "PP_RIGHT",
            "PP_UP_RIGHT",
            "unknown"
        }
        self.minTurnRadius = AutoDrive.getDriverRadius(self.vehicle)
        self.dubinsDone = false
        self.dubinsCount = 0
        self.isNewPF = true
        self.pathFinderTime = 3
    else
        self.PP_UP = 0
        self.PP_UP_RIGHT = 1
        self.PP_RIGHT = 2
        self.PP_DOWN_RIGHT = 3
        self.PP_DOWN = 4
        self.PP_DOWN_LEFT = 5
        self.PP_LEFT = 6
        self.PP_UP_LEFT = 7
        self.direction_to_text = {
            "PP_UP",
            "PP_UP_RIGHT",
            "PP_RIGHT",
            "PP_DOWN_RIGHT",
            "PP_DOWN",
            "PP_DOWN_LEFT",
            "PP_LEFT",
            "PP_UP_LEFT",
            "unknown"
        }
        self.minTurnRadius = AutoDrive.getDriverRadius(self.vehicle) * 2 / 3
        self.isNewPF = false
    end
    self.realTurnRadius = self.minTurnRadius
    self.gridScaleStep = 1
    self.allowPartialPath = false
    self.rearFallbackDone = false
    self.rearFallbackSkippedPrinted = false
    self.partialPath = false
end

function PathFinderModule:hasFinished()
    if AutoDrive.isEditorModeEnabled() and AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
        return false
    end
    if self.isFinished and self.smoothDone == true then
        return true
    end
    return false
end

function PathFinderModule:getPath()
    return self.wayPoints
end

function PathFinderModule:startPathPlanningToNetwork(destinationId)
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule:startPathPlanningToNetwork destinationId %s"
        , tostring(destinationId)
    )
    PathFinderModule.debugVehicleMsg(self.vehicle,
        string.format("PFM startPathPlanningToNetwork destinationId %s",
            tostring(destinationId)
        )
    )
    local closest = self.vehicle:getClosestWayPoint()
    local fieldExit = self:getFieldExitWayPoint(destinationId)
    if fieldExit ~= nil then
        PathFinderModule.debugMsg(self.vehicle, "PathFinderModule:startPathPlanningToNetwork - leaving the field by the destination point %s first", tostring(fieldExit))
        closest = fieldExit
    end
    self:startPathPlanningToWayPoint(closest, destinationId)
    self.goingToNetwork = true
end

PathFinderModule.FIELD_EXIT_MAX_DISTANCE = 500 -- m, destinations considered to leave the field

-- a vehicle on a field leaves it by a destination of the network instead of the closest point, which can
-- be past the crop or on another field: in harvester mode the field set as first destination, otherwise
-- the closest destination (up to FIELD_EXIT_MAX_DISTANCE) that leads to the target
function PathFinderModule:getFieldExitWayPoint(destinationId)
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    if not AutoDrive.checkIsOnField(x, y, z) then
        return nil
    end
    local function leadsToDestination(wayPointId)
        local wayPoint = wayPointId ~= nil and ADGraphManager:getWayPointById(wayPointId) or nil
        if wayPoint == nil or wayPointId == destinationId then
            return nil
        end
        local distance = MathUtil.vector2Length(wayPoint.x - x, wayPoint.z - z)
        if distance > PathFinderModule.FIELD_EXIT_MAX_DISTANCE then
            return nil
        end
        local route = ADGraphManager:pathFromTo(wayPointId, destinationId, nil, self.vehicle)
        if route == nil or #route < 2 then
            return nil
        end
        return distance
    end
    local stateModule = self.vehicle.ad.stateModule
    if stateModule:getMode() == AutoDrive.MODE_UNLOAD then
        local fieldWayPointId = stateModule:getFirstWayPoint()
        if leadsToDestination(fieldWayPointId) ~= nil then
            return fieldWayPointId
        end
    end
    -- closest destinations first, the route search only runs until one leads to the target
    local markers = {}
    for _, marker in pairs(ADGraphManager:getMapMarkers()) do
        local wayPoint = marker.id ~= nil and ADGraphManager:getWayPointById(marker.id) or nil
        if wayPoint ~= nil then
            local distance = MathUtil.vector2Length(wayPoint.x - x, wayPoint.z - z)
            if distance <= PathFinderModule.FIELD_EXIT_MAX_DISTANCE then
                table.insert(markers, {id = marker.id, distance = distance, x = wayPoint.x, z = wayPoint.z})
            end
        end
    end
    -- destinations of the field the vehicle is on come first (their point is often just off the field edge)
    local area = PathFinderModule.getConnectedFieldArea(x, z)
    if area ~= nil then
        for _, marker in ipairs(markers) do
            marker.sameField = PathFinderModule.isNearFieldArea(area, marker.x, marker.z)
        end
    end
    table.sort(markers, function(a, b)
        if a.sameField ~= b.sameField then
            return a.sameField == true
        end
        return a.distance < b.distance
    end)
    for _, marker in ipairs(markers) do
        -- never a destination lying on a neighbouring field
        local onOtherField = not marker.sameField and AutoDrive.checkIsOnField(marker.x, 0, marker.z)
        if not onOtherField and leadsToDestination(marker.id) ~= nil then
            return marker.id
        end
    end
    return nil
end

function PathFinderModule:startPathPlanningToWayPoint(wayPointId, destinationId)
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule:startPathPlanningToWayPoint destinationId %s"
        , tostring(destinationId)
    )
    PathFinderModule.debugVehicleMsg(self.vehicle,
        string.format("PFM startPathPlanningToWayPoint wayPointId %s",
            tostring(wayPointId)
        )
    )
    local targetNode = ADGraphManager:getWayPointById(wayPointId)
    local wayPoints = ADGraphManager:pathFromTo(wayPointId, destinationId, nil, self.vehicle)
    if wayPoints ~= nil and #wayPoints > 1 then
        local vecToNextPoint = {x = wayPoints[2].x - targetNode.x, z = wayPoints[2].z - targetNode.z}
        self:startPathPlanningTo(targetNode, vecToNextPoint)
        self.goingToNetwork = true
        self.destinationId = destinationId
        self.targetWayPointId = wayPointId
        self.appendWayPoints = wayPoints
    end
    return
end

function PathFinderModule:startPathPlanningToPipe(combine, chasing)
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule:startPathPlanningToPipe chasing %s"
        , tostring(chasing)
    )
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PathFinderModule:startPathPlanningToPipe")
    PathFinderModule.debugVehicleMsg(self.vehicle,
        string.format("PFM startPathPlanningToPipe combine %s",
            tostring(combine:getName())
        )
    )
    local combineX, worldY, combineZ = getWorldTranslation(combine.components[1].node)
    PathFinderModule.startFieldScan(combineX, combineZ)
    local rx, _, rz =  AutoDrive.localDirectionToWorld(combine, 0, 0, 1, combine.ad.ADRootNode)
    if combine.components[2] ~= nil and combine.components[2].node ~= nil then
        rx, _, rz =  AutoDrive.localDirectionToWorld(combine, 0, 0, 1, combine.components[2].node)
    end
    local combineVector = {x = rx, z = rz}

    local pipeChasePos, pipeChaseSide = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getPipeChasePosition(true)
    -- We use the follow distance as a proxy measure for "what works" for the size of the
    -- field being worked.
    -- local followDistance = AutoDrive.getSetting("followDistance", self.vehicle)
    -- Use the length of the tractor-trailer combo to determine how far to drive to straighten
    -- the trailer.
    -- 2*math.sin(math.pi/8)) is the third side of a 45-67.5-67.5 isosceles triangle with the
    -- equal sides being the length of the tractor train
    local lengthOffset = combine.size.length / 2 +
                            AutoDrive.getTractorTrainLength(self.vehicle, true, false) * (2 * math.sin(math.pi / 8))
    -- A bit of a sanity check, in case the vehicle is absurdly long.
    --if lengthOffset > self.PATHFINDER_FOLLOW_DISTANCE then
    --    lengthOffset = self.PATHFINDER_FOLLOW_DISTANCE
    --elseif
    if lengthOffset <= self.PATHFINDER_TARGET_DISTANCE then
        lengthOffset = self.PATHFINDER_TARGET_DISTANCE
    end

    --local target = {x = pipeChasePos.x, y = worldY, z = pipeChasePos.z}
    -- The sugarcane harvester needs extra room or it collides
    --if pipeChaseSide ~= CombineUnloaderMode.CHASEPOS_REAR or CombineUnloaderMode:isSugarcaneHarvester(combine) then
    --    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PathFinderModule:startPathPlanningToPipe?lengthOffset " .. lengthOffset)
    --    local straightenNode = {x = pipeChasePos.x - lengthOffset * rx, y = worldY, z = pipeChasePos.z - lengthOffset * rz}
    --    self:startPathPlanningTo(straightenNode, combineVector)
    --    table.insert(self.appendWayPoints, target)
    --else
    --    self:startPathPlanningTo(target, combineVector)
    --end
    if combine.ad.isAutoAimingChopper then
        -- local pathFinderTarget = {x = pipeChasePos.x - (self.PATHFINDER_TARGET_DISTANCE) * rx, y = worldY, z = pipeChasePos.z - (self.PATHFINDER_TARGET_DISTANCE) * rz}
        local pathFinderTarget = {x = pipeChasePos.x, y = worldY, z = pipeChasePos.z}
        self:startPathPlanningTo(pathFinderTarget, combineVector)

    elseif combine.ad.isFixedPipeChopper then
        local pathFinderTarget = {x = pipeChasePos.x, y = worldY, z = pipeChasePos.z}
        -- only append target points / try to straighten the driver/trailer combination if we are driving up to the pipe not the rear end
        if pipeChaseSide ~= AutoDrive.CHASEPOS_REAR then
            pathFinderTarget = {x = pipeChasePos.x - (combine.size.length) * rx, y = worldY, z = pipeChasePos.z - (combine.size.length) * rz}
        end
        local appendedNode = {x = pipeChasePos.x - (combine.size.length / 2 * rx), y = worldY, z = pipeChasePos.z - (combine.size.length / 2 * rz)}

        self:startPathPlanningTo(pathFinderTarget, combineVector)

        if pipeChaseSide ~= AutoDrive.CHASEPOS_REAR then
            table.insert(self.appendWayPoints, appendedNode)
        end
    else
        -- combine.ad.isHarvester 
        local pathFinderTarget = {x = pipeChasePos.x, y = worldY, z = pipeChasePos.z}
        -- only append target points / try to straighten the driver/trailer combination if we are driving up to the pipe not the rear end
        if pipeChaseSide ~= AutoDrive.CHASEPOS_REAR then
            pathFinderTarget = {x = pipeChasePos.x - (lengthOffset) * rx, y = worldY, z = pipeChasePos.z - (lengthOffset) * rz}
        end
        local appendedNode = {x = pipeChasePos.x - (combine.size.length / 2 * rx), y = worldY, z = pipeChasePos.z - (combine.size.length / 2 * rz)}

        self:startPathPlanningTo(pathFinderTarget, combineVector)

        if pipeChaseSide ~= AutoDrive.CHASEPOS_REAR then
            table.insert(self.appendWayPoints, appendedNode)
            table.insert(self.appendWayPoints, pipeChasePos)
        end
    end

    if combine.spec_combine ~= nil and combine.ad.isHarvester then
        if combine.spec_combine.fillUnitIndex ~= nil and combine.spec_combine.fillUnitIndex ~= 0 then
            local fillType = g_fruitTypeManager:getFruitTypeIndexByFillTypeIndex(combine:getFillUnitFillType(combine.spec_combine.fillUnitIndex))
            if fillType ~= nil then
                self.fruitToCheck = fillType
                local fruitType = g_fruitTypeManager:getFruitTypeByIndex(fillType)

                PathFinderModule.debugVehicleMsg(self.vehicle,
                    string.format("PFM startPathPlanningToPipe self.fruitToCheck %s Fruit name %s title %s",
                        tostring(self.fruitToCheck),
                        tostring(fruitType.fillType.name),
                        tostring(fruitType.fillType.title)
                    )
                )
            end
        end
    end

    self.goingToPipe = true
    self.pipeCombine = combine
    -- far from the harvester: aim at its rear first, its own cut track always leads there; the chase takes over close to it
    if not chasing and combine.ad ~= nil and not combine.ad.isChopper and (self.strictFruit or self.strictField)
        and AutoDrive.getDistanceBetween(self.vehicle, combine) > PathFinderModule.REAR_FIRST_DISTANCE then
        self:startRearFallback()
    end
    if AutoDrive.getDistanceBetween(self.vehicle, combine) < 50 then
        -- shorten path calculation for close combine
        self.max_pathfinder_steps = PathFinderModule.MAX_PATHFINDER_STEPS_COMBINE_TURN
    end
    self.chasingVehicle = chasing
end

function PathFinderModule:startPathPlanningToVehicle(targetVehicle, targetDistance)
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule:startPathPlanningToVehicle targetDistance %s"
        , tostring(targetDistance)
    )
    PathFinderModule.debugVehicleMsg(self.vehicle,
        string.format("PFM startPathPlanningToVehicle targetVehicle %s",
            tostring(targetVehicle:getName())
        )
    )
    local worldX, worldY, worldZ = getWorldTranslation(targetVehicle.components[1].node)
    local rx, _, rz =  AutoDrive.localDirectionToWorld(targetVehicle, 0, 0, 1)
    local targetVector = {x = rx, z = rz}

    local wpBehind = {x = worldX - targetDistance * rx, y = worldY, z = worldZ - targetDistance * rz}
    self:startPathPlanningTo(wpBehind, targetVector)

    self.goingToPipe = false
    self.pipeCombine = nil
    self.chasingVehicle = true
    self.isSecondChasingVehicle = true
    if targetVehicle.ad ~= nil and targetVehicle.ad.pathFinderModule ~= nil and targetVehicle.ad.pathFinderModule.fruitToCheck ~= nil then
        self.fruitToCheck = targetVehicle.ad.pathFinderModule.fruitToCheck
    end
end

function PathFinderModule:getTrainWidth()
    local width = 3
    local units = AutoDrive.getAllUnits(self.vehicle)
    if units ~= nil then
        for _, unit in pairs(units) do
            if unit.size ~= nil and unit.size.width ~= nil then
                width = math.max(width, unit.size.width)
            end
        end
    end
    return width
end

-- corners of the corridor driven from fromCell to cell, same layout as getCorners (start, width, height, opposite)
function PathFinderModule:getCorridorCorners(fromCell, cell)
    local a = self:gridLocationToWorldLocation(fromCell)
    local b = self:gridLocationToWorldLocation(cell)
    local dx, dz = b.x - a.x, b.z - a.z
    local length = math.sqrt(dx * dx + dz * dz)
    if length < 0.01 then
        return nil
    end
    local nx, nz = -dz / length * self.trainHalfWidth, dx / length * self.trainHalfWidth
    return {
        {x = a.x - nx, z = a.z - nz},
        {x = a.x + nx, z = a.z + nz},
        {x = b.x - nx, z = b.z - nz},
        {x = b.x + nx, z = b.z + nz}
    }
end

-- field outlines of the map, with a bounding box each, read once
function PathFinderModule.getFieldOutlines()
    if PathFinderModule.fieldOutlines ~= nil then
        return PathFinderModule.fieldOutlines
    end
    local outlines = {}
    if g_fieldManager ~= nil and g_fieldManager.fields ~= nil and FieldPointGenerator ~= nil then
        for _, field in pairs(g_fieldManager.fields) do
            local points = FieldPointGenerator:getFieldWorldPoints(field)
            if points ~= nil and #points >= 3 then
                local outline = {field = field, points = points, minX = math.huge, maxX = -math.huge, minZ = math.huge, maxZ = -math.huge}
                for _, p in ipairs(points) do
                    outline.minX = math.min(outline.minX, p.x)
                    outline.maxX = math.max(outline.maxX, p.x)
                    outline.minZ = math.min(outline.minZ, p.z)
                    outline.maxZ = math.max(outline.maxZ, p.z)
                end
                table.insert(outlines, outline)
            end
        end
    end
    if #outlines > 0 then
        PathFinderModule.fieldOutlines = outlines
    end
    return outlines
end

-- field whose outline contains the position, nil if none
function PathFinderModule.getFieldAtPosition(x, z)
    for _, outline in ipairs(PathFinderModule.getFieldOutlines()) do
        if x >= outline.minX and x <= outline.maxX and z >= outline.minZ and z <= outline.maxZ then
            local inside = false
            local points = outline.points
            local j = #points
            for i = 1, #points do
                local pi, pj = points[i], points[j]
                if ((pi.z > z) ~= (pj.z > z)) and (x < (pj.x - pi.x) * (z - pi.z) / (pj.z - pi.z) + pi.x) then
                    inside = not inside
                end
                j = i
            end
            if inside then
                return outline.field
            end
        end
    end
    return nil
end

-- connected field ground: flood fill of the on-field samples around a position. Two fields count as one
-- only if their ground touches, a grass strip or a road between them splits them
PathFinderModule.FIELD_AREA_STEP = 4
PathFinderModule.FIELD_AREA_MAX_SAMPLES = 60000
PathFinderModule.FIELD_AREA_CACHE_TIME = 600000
PathFinderModule.fieldAreaCache = {}

function PathFinderModule.getFieldAreaKey(ix, iz)
    return (ix + 50000) * 100000 + (iz + 50000)
end

function PathFinderModule.isInFieldArea(area, x, z)
    local step = PathFinderModule.FIELD_AREA_STEP
    local ix, iz = math.floor(x / step + 0.5), math.floor(z / step + 0.5)
    if area.cells[PathFinderModule.getFieldAreaKey(ix, iz)] then
        return true
    end
    -- position on the field edge, the nearest sample is off the field: look at the samples around it
    if not AutoDrive.checkIsOnField(ix * step, 0, iz * step) then
        for dx = -1, 1 do
            for dz = -1, 1 do
                if area.cells[PathFinderModule.getFieldAreaKey(ix + dx, iz + dz)] then
                    return true
                end
            end
        end
    end
    return false
end

PathFinderModule.FIELD_EDGE_DISTANCE = 12 -- m, a point this close to the field ground belongs to the field

function PathFinderModule.isNearFieldArea(area, x, z)
    local step = PathFinderModule.FIELD_AREA_STEP
    local range = math.ceil(PathFinderModule.FIELD_EDGE_DISTANCE / step)
    local ix, iz = math.floor(x / step + 0.5), math.floor(z / step + 0.5)
    for dx = -range, range do
        for dz = -range, range do
            if (dx * dx + dz * dz) * step * step <= PathFinderModule.FIELD_EDGE_DISTANCE ^ 2 and area.cells[PathFinderModule.getFieldAreaKey(ix + dx, iz + dz)] then
                return true
            end
        end
    end
    return false
end

function PathFinderModule.getConnectedFieldArea(x, z)
    local step = PathFinderModule.FIELD_AREA_STEP
    local getKey = PathFinderModule.getFieldAreaKey
    local sx, sz = math.floor(x / step + 0.5), math.floor(z / step + 0.5)
    local seedX, seedZ
    for r = 0, 2 do
        for dx = -r, r do
            for dz = -r, r do
                if seedX == nil and math.max(math.abs(dx), math.abs(dz)) == r and AutoDrive.checkIsOnField((sx + dx) * step, 0, (sz + dz) * step) then
                    seedX, seedZ = sx + dx, sz + dz
                end
            end
        end
        if seedX ~= nil then
            break
        end
    end
    if seedX == nil then
        return nil
    end
    local seedKey = getKey(seedX, seedZ)
    local now = g_time or 0
    local cache = PathFinderModule.fieldAreaCache
    for i = #cache, 1, -1 do
        if now - cache[i].time > PathFinderModule.FIELD_AREA_CACHE_TIME or now < cache[i].time then
            table.remove(cache, i)
        elseif cache[i].cells[seedKey] then
            return cache[i]
        end
    end
    local area = {cells = {}, count = 0, capped = false, time = now}
    local visited = {[seedKey] = true}
    local queue = {seedX, seedZ}
    local head = 1
    while head < #queue do
        local ix, iz = queue[head], queue[head + 1]
        head = head + 2
        area.cells[getKey(ix, iz)] = true
        area.count = area.count + 1
        if area.count >= PathFinderModule.FIELD_AREA_MAX_SAMPLES then
            area.capped = true
            break
        end
        for _, d in ipairs({{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) do
            local nx, nz = ix + d[1], iz + d[2]
            local key = getKey(nx, nz)
            if not visited[key] then
                visited[key] = true
                if AutoDrive.checkIsOnField(nx * step, 0, nz * step) then
                    table.insert(queue, nx)
                    table.insert(queue, nz)
                end
            end
        end
    end
    table.insert(cache, area)
    if #cache > 8 then
        table.remove(cache, 1)
    end
    return area
end

-- field restriction for one position: normal mode = only on a field, strict mode = anywhere but on another field
PathFinderModule.OFF_FIELD_NEAR_ENDS = 12
PathFinderModule.OFF_FIELD_MAX_DISTANCE = 8

-- grass strip along a field: field ground within OFF_FIELD_MAX_DISTANCE
function PathFinderModule:isNearFieldGround(x, z)
    self.nearFieldCache = self.nearFieldCache or {}
    local key = string.format("%d|%d", math.floor(x), math.floor(z))
    local cached = self.nearFieldCache[key]
    if cached ~= nil then
        return cached
    end
    local result = false
    for _, d in ipairs({PathFinderModule.OFF_FIELD_MAX_DISTANCE / 2, PathFinderModule.OFF_FIELD_MAX_DISTANCE}) do
        for i = 0, 7 do
            local a = i * math.pi / 4
            if AutoDrive.checkIsOnField(x + math.cos(a) * d, 0, z + math.sin(a) * d) then
                result = true
                break
            end
        end
        if result then
            break
        end
    end
    self.nearFieldCache[key] = result
    return result
end
function PathFinderModule:isFieldPositionAllowed(x, z, isOnField)
    if isOnField == nil then
        isOnField = PathFinderModule.getMapOnFieldAt(x, z)
    end
    if not self.strictField then
        return isOnField
    end
    if not isOnField then
        -- finer grids find gaps on the grass between trees AutoDrive does not always see: stay on field ground,
        -- except around the start and the target (the vehicle may wait on the grass beside the field)
        -- strict: the field ground only, whatever the grid, except around the start and the target
        local near = PathFinderModule.OFF_FIELD_NEAR_ENDS
        local nearStart = self.startX ~= nil and MathUtil.vector2Length(x - self.startX, z - self.startZ) < near
        local nearTarget = self.target ~= nil and MathUtil.vector2Length(x - self.target.x, z - self.target.z) < near
        return nearStart or nearTarget
    end
    if self.allowedAreas ~= nil and next(self.allowedAreas) ~= nil then
        local capped = false
        for area, _ in pairs(self.allowedAreas) do
            if PathFinderModule.isInFieldArea(area, x, z) then
                return true
            end
            capped = capped or area.capped
        end
        if not capped then
            return false
        end
    end
    if self.allowedFields ~= nil and next(self.allowedFields) ~= nil then
        local field = PathFinderModule.getFieldAtPosition(x, z)
        if field ~= nil then
            return self.allowedFields[field] == true
        end
        -- cultivated ground outside every field outline: allowed if it lies on one of our farmlands
    end
    return self.allowedFarmlands ~= nil and self.allowedFarmlands[g_farmlandManager:getFarmlandIdAtWorldPosition(x, z)] == true
end

-- side of the train along the field border (strict): the grass beside the field is allowed, only the ground of another field is refused,
-- the centre line of the route is checked by isFieldPositionAllowed. Returns allowed, onOtherField
function PathFinderModule:isTrainSideAllowed(x, z)
    local isOnField = PathFinderModule.getMapOnFieldAt(x, z)
    if self.strictField and not isOnField then
        return true, false
    end
    return self:isFieldPositionAllowed(x, z, isOnField), isOnField
end

-- small patches of standing crop left in a harvested field are ignored: a fruit hit only counts
-- if at least PathFinderModule.ISLAND_MIN_RATIO of the area around the cell holds this fruit too
PathFinderModule.ISLAND_CHECK_HALF_SIZE = 10
PathFinderModule.ISLAND_CHECK_SAMPLES = 10
PathFinderModule.ISLAND_MIN_RATIO = 0.08
-- one line in the log for each route found in strict mode, to see what was checked when a route crosses a crop
function PathFinderModule:printStrictSummary(kind)
    if not (self.strictFruit or self.strictField) then
        return
    end
    local fruitName = "all"
    if self.fruitToCheck ~= nil and not self.strictFruit then
        local fruitType = g_fruitTypeManager:getFruitTypeByIndex(self.fruitToCheck)
        fruitName = fruitType ~= nil and fruitType.name or tostring(self.fruitToCheck)
    end
    local name = self.vehicle ~= nil and self.vehicle.getName ~= nil and self.vehicle:getName() or "?"
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule strict check: path found (%s) to %.0f,%.0f, crop checked %s, cells refused for crop %d, avoid fruit %s, restrict to field %s, fruit fallback %s, to the pipe %s, field map %d cells (%d read for this search)", kind, self.target ~= nil and self.target.x or 0, self.target ~= nil and self.target.z or 0, fruitName, self.strictFruitRefusals or 0,
        tostring(self.avoidFruitSetting), tostring(AutoDrive.getSetting("restrictToField", self.vehicle)),
        tostring(self.fallBackMode3), tostring(self.goingToPipe), PathFinderModule.fieldMapCount, PathFinderModule.fieldMapReads - (self.mapReadsAtStart or 0))
end

function PathFinderModule:isFruitIsland(fruitTypeIndex, x, z)
    local key = string.format("%s_%d_%d", tostring(fruitTypeIndex), math.floor(x / 8), math.floor(z / 8))
    if self.islandCache ~= nil and self.islandCache[key] ~= nil then
        return self.islandCache[key]
    end
    -- from the field map: share of this crop in the square around the position
    if true then
        local half = PathFinderModule.ISLAND_CHECK_HALF_SIZE
        local ratio = PathFinderModule.getMapCropRatio(fruitTypeIndex, x - half, z - half, x + half, z - half, x - half, z + half)
        local isIslandMap = ratio < PathFinderModule.ISLAND_MIN_RATIO
        if self.islandCache ~= nil then
            self.islandCache[key] = isIslandMap
        end
        return isIslandMap
    end
    local half = PathFinderModule.ISLAND_CHECK_HALF_SIZE
    local samples = PathFinderModule.ISLAND_CHECK_SAMPLES
    local step = 2 * half / samples
    local hits = 0
    for i = 0, samples - 1 do
        for j = 0, samples - 1 do
            local sx = x - half + i * step
            local sz = z - half + j * step
            if AutoDrive.getFruitValue(fruitTypeIndex, sx, sz, sx + step, sz, sx, sz + step) > 0 then
                hits = hits + 1
            end
        end
    end
    local isIsland = hits < samples * samples * PathFinderModule.ISLAND_MIN_RATIO
    if self.islandCache ~= nil then
        self.islandCache[key] = isIsland
    end
    return isIsland
end

-- zones the pathfinder must avoid although overlapBox finds nothing there (learned by ADDrivePathModule:learnBlockedPlace, or set by ADObstacleDetourModule:
-- an obstacle the front sensor stopped on, which the route search does not see). {x, z, radius}
-- box as given to overlapBox (center, rotation around Y, half extents along local X and Z)
function PathFinderModule:boxHitsBlockedZone(cx, cz, ry, halfX, halfZ)
    local cosA, sinA = math.cos(ry), math.sin(ry)
    -- learned zones are tried again after LEARNED_ZONE_TTL: if the obstacle is still there the front sensor adds it back
    local now = g_time or 0
    for i = #PathFinderModule.learnedZones, 1, -1 do
        local zone = PathFinderModule.learnedZones[i]
        if now - (zone.time or now) > PathFinderModule.LEARNED_ZONE_TTL then
            table.remove(PathFinderModule.learnedZones, i)
        end
    end
    for _, zones in ipairs({self.blockedZones or {}, PathFinderModule.learnedZones}) do
        for _, zone in ipairs(zones) do
            local dx, dz = zone.x - cx, zone.z - cz
            local lx = dx * cosA - dz * sinA
            local lz = dx * sinA + dz * cosA
            if math.abs(lx) < halfX + zone.radius and math.abs(lz) < halfZ + zone.radius then
                return true
            end
        end
    end
    return false
end

-- places where the front sensor stopped a vehicle on a calculated route: kept for the whole session, for all vehicles
PathFinderModule.learnedZones = {}
PathFinderModule.LEARNED_ZONE_RADIUS = 4
PathFinderModule.LEARNED_ZONE_DELAY = 3000
PathFinderModule.LEARNED_ZONE_TTL = 300000 -- ms

function PathFinderModule.addLearnedZone(x, z, vehicleName)
    for _, zone in ipairs(PathFinderModule.learnedZones) do
        if MathUtil.vector2Length(zone.x - x, zone.z - z) < zone.radius then
            return
        end
    end
    AutoDrive.debugPrint(nil, AutoDrive.DC_PATHINFO, "PathFinderModule: %s: blocked zone added at %.0f,%.0f (front sensor blocked), %d zone(s)", tostring(vehicleName), x, z, #PathFinderModule.learnedZones)
    table.insert(PathFinderModule.learnedZones, {x = x, z = z, radius = PathFinderModule.LEARNED_ZONE_RADIUS, time = g_time or 0})
end

PathFinderModule.FRUIT_MAX_RATIO = 0.10
PathFinderModule.FRUIT_SLICE_LENGTH = 4
PathFinderModule.SMOOTH_FRUIT_MARGIN = 1
PathFinderModule.ZIGZAG_MAX_KEPT = 2
PathFinderModule.OFF_FIELD_EXTRA_MARGIN = 1.5

-- field map: standing crop kept in memory in 2 m cells (4 samples each, 1 m apart), shared by all vehicles.
-- A cell is read once from the density map, then kept; around a working harvester the cells are forgotten
-- every second and read again when needed. Every cell is read again after MAP_TTL anyway.
PathFinderModule.MAP_CELL = 2
PathFinderModule.MAP_TTL = 1800000
PathFinderModule.MAP_REFRESH_INTERVAL = 1000
PathFinderModule.MAP_HARVESTER_MARGIN = 8
PathFinderModule.MAP_SAMPLE_OFFSETS = {{-0.5, -0.5}, {0.5, -0.5}, {-0.5, 0.5}, {0.5, 0.5}}
PathFinderModule.fieldMap = {}
PathFinderModule.fieldMapCount = 0
PathFinderModule.fieldMapReads = 0
PathFinderModule.fieldMapRefreshTime = -math.huge

-- standing crop at one position: fruit type index, 0 if none (cut, withered, grass and meadow count as none)
function PathFinderModule.readCropAt(x, z)
    local fruitTypeIndex, growthState = FSDensityMapUtil.getFruitTypeIndexAtWorldPos(x, z)
    if fruitTypeIndex == nil or fruitTypeIndex == 0 or growthState == nil or growthState == 0 then
        return 0
    end
    if fruitTypeIndex == FruitType.MEADOW or fruitTypeIndex == FruitType.GRASS then
        return 0
    end
    local fruit = g_fruitTypeManager:getFruitTypeByIndex(fruitTypeIndex)
    if fruit == nil or fruit:getIsCut(growthState) or fruit:getIsWithered(growthState) then
        return 0
    end
    -- the stubble behind the harvester can still answer with the crop above: confirm with the same reading as
    -- the checks of this morning (fruit area of a 0.5 m square, cut or withered counts as none)
    if PathFinderModule.getFruitRatio(fruitTypeIndex, x - 0.25, z - 0.25, x + 0.25, z - 0.25, x - 0.25, z + 0.25) <= 0 then
        return 0
    end
    return fruitTypeIndex
end

function PathFinderModule.getMapCell(ix, iz)
    local key = (ix + 50000) * 100000 + (iz + 50000)
    local cell = PathFinderModule.fieldMap[key]
    local now = g_time or 0
    if cell ~= nil and now >= cell.t and now - cell.t < PathFinderModule.MAP_TTL then
        return cell
    end
    if cell == nil then
        PathFinderModule.fieldMapCount = PathFinderModule.fieldMapCount + 1
    end
    cell = {t = now, field = {}}
    local cx, cz = ix * PathFinderModule.MAP_CELL, iz * PathFinderModule.MAP_CELL
    for k, offset in ipairs(PathFinderModule.MAP_SAMPLE_OFFSETS) do
        cell[k] = PathFinderModule.readCropAt(cx + offset[1], cz + offset[2])
        cell.field[k] = AutoDrive.checkIsOnField(cx + offset[1], 0, cz + offset[2]) == true
        cell.onField = cell.onField or cell.field[k]
    end
    if g_currentMission ~= nil and g_currentMission.terrainRootNode ~= nil then
        cell.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, cx, 0, cz)
    end
    PathFinderModule.fieldMapReads = PathFinderModule.fieldMapReads + 1
    PathFinderModule.fieldMap[key] = cell
    return cell
end

-- crop at a position from the map (nearest of the 4 samples of its cell)
function PathFinderModule.getMapCropAt(x, z)
    local size = PathFinderModule.MAP_CELL
    local ix, iz = math.floor(x / size + 0.5), math.floor(z / size + 0.5)
    local cell = PathFinderModule.getMapCell(ix, iz)
    local k = 1
    if x >= ix * size then
        k = k + 1
    end
    if z >= iz * size then
        k = k + 2
    end
    return cell[k]
end

-- field ground at a position from the map (nearest of the 4 samples of its cell)
function PathFinderModule.getMapOnFieldAt(x, z)
    local size = PathFinderModule.MAP_CELL
    local ix, iz = math.floor(x / size + 0.5), math.floor(z / size + 0.5)
    local cell = PathFinderModule.getMapCell(ix, iz)
    local k = 1
    if x >= ix * size then
        k = k + 1
    end
    if z >= iz * size then
        k = k + 2
    end
    return cell.field[k]
end

-- share of standing crop in the parallelogram c1, c2 = c1 + u, c4 = c1 + v, sampled every meter.
-- fruitFilter nil: every crop counts. Returns ratio and the crop found most
function PathFinderModule.getMapCropRatio(fruitFilter, c1X, c1Z, c2X, c2Z, c4X, c4Z)
    local uX, uZ = c2X - c1X, c2Z - c1Z
    local vX, vZ = c4X - c1X, c4Z - c1Z
    local nu = math.max(1, math.ceil(math.sqrt(uX * uX + uZ * uZ)))
    local nv = math.max(1, math.ceil(math.sqrt(vX * vX + vZ * vZ)))
    local hits, found = 0, nil
    for i = 0, nu - 1 do
        local fu = (i + 0.5) / nu
        for j = 0, nv - 1 do
            local fv = (j + 0.5) / nv
            local crop = PathFinderModule.getMapCropAt(c1X + uX * fu + vX * fv, c1Z + uZ * fu + vZ * fv)
            if crop ~= 0 and (fruitFilter == nil or crop == fruitFilter) then
                hits = hits + 1
                found = crop
            end
        end
    end
    return hits / (nu * nv), found
end

-- crop filter of this search: every crop in strict mode or to the network, else the crop of the field
function PathFinderModule:getCropFilter()
    if self.goingToNetwork or self.strictFruit then
        return nil
    end
    return self.fruitToCheck
end

-- the harvesters cut the crop: read the map again around each one working (every MAP_REFRESH_INTERVAL)
function PathFinderModule.refreshFieldMap()
    local now = g_time or 0
    if now - PathFinderModule.fieldMapRefreshTime < PathFinderModule.MAP_REFRESH_INTERVAL and now >= PathFinderModule.fieldMapRefreshTime then
        return
    end
    PathFinderModule.fieldMapRefreshTime = now
    if PathFinderModule.fieldMapCount == 0 or g_currentMission == nil or g_currentMission.vehicleSystem == nil then
        return
    end
    local size = PathFinderModule.MAP_CELL
    for _, vehicle in pairs(g_currentMission.vehicleSystem.vehicles) do
        if vehicle.spec_combine ~= nil and vehicle.components ~= nil and vehicle.rootNode ~= nil and vehicle.getLastSpeed ~= nil and vehicle:getLastSpeed() > 0.5 then
            local x, _, z = getWorldTranslation(vehicle.rootNode)
            local vehicleSize = vehicle.size or {}
            local radius = math.max(vehicleSize.width or 6, vehicleSize.length or 10, 12) / 2 + PathFinderModule.MAP_HARVESTER_MARGIN
            local r = math.ceil(radius / size)
            local cx, cz = math.floor(x / size + 0.5), math.floor(z / size + 0.5)
            for ix = cx - r, cx + r do
                for iz = cz - r, cz + r do
                    local key = (ix + 50000) * 100000 + (iz + 50000)
                    local cell = PathFinderModule.fieldMap[key]
                    if cell ~= nil then
                        cell.t = -math.huge
                        PathFinderModule.getMapCell(ix, iz)
                    end
                end
            end
        end
    end
end

-- whole field recorded at once when an unloader goes to a harvester: flood fill over the field ground from the
-- harvester, a ring of cells around the field included, spread over the frames (MAP_SCAN_BUDGET ms each)
PathFinderModule.MAP_SCAN_BUDGET = 3
PathFinderModule.MAP_SCAN_MAX_CELLS = 400000
PathFinderModule.MAP_DRAW_RADIUS = 40
PathFinderModule.showFieldMap = false
PathFinderModule.fieldScan = nil
PathFinderModule.scannedCells = {}
PathFinderModule.searchCells = {}

local function mapClock()
    if getTimeSec ~= nil then
        return getTimeSec() * 1000
    end
    return os.clock() * 1000
end

function PathFinderModule.startFieldScan(x, z)
    local size = PathFinderModule.MAP_CELL
    local ix, iz = math.floor(x / size + 0.5), math.floor(z / size + 0.5)
    local key = (ix + 50000) * 100000 + (iz + 50000)
    local scannedAt = PathFinderModule.scannedCells[key]
    local now = g_time or 0
    if PathFinderModule.fieldScan ~= nil and PathFinderModule.fieldScan.visited[key] then
        return
    end
    if scannedAt ~= nil and now >= scannedAt and now - scannedAt < PathFinderModule.MAP_TTL then
        return
    end
    local scan = {visited = {[key] = true}, queue = {{ix, iz}}, head = 1, tail = 1, count = 0, startTime = mapClock(), startX = x, startZ = z}
    PathFinderModule.fieldScan = scan
    AutoDrive.debugPrint(nil, AutoDrive.DC_PATHINFO, "PathFinderModule field map: scanning the field at %.0f,%.0f", x, z)
end

function PathFinderModule.updateFieldScan()
    local scan = PathFinderModule.fieldScan
    if scan == nil then
        return
    end
    local now = g_time or 0
    local stopAt = mapClock() + PathFinderModule.MAP_SCAN_BUDGET
    local done = 0
    while scan.head <= scan.tail do
        local item = scan.queue[scan.head]
        scan.queue[scan.head] = nil
        scan.head = scan.head + 1
        local ix, iz = item[1], item[2]
        local key = (ix + 50000) * 100000 + (iz + 50000)
        local cell = PathFinderModule.fieldMap[key]
        if cell ~= nil and (now < cell.t or now - cell.t >= PathFinderModule.MAP_TTL or cell.field == nil) then
            cell.t = -math.huge
        end
        cell = PathFinderModule.getMapCell(ix, iz)
        PathFinderModule.scannedCells[key] = now
        scan.count = scan.count + 1
        if cell.onField and scan.count < PathFinderModule.MAP_SCAN_MAX_CELLS then
            for _, d in ipairs({{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) do
                local nx, nz = ix + d[1], iz + d[2]
                local nkey = (nx + 50000) * 100000 + (nz + 50000)
                if not scan.visited[nkey] then
                    scan.visited[nkey] = true
                    scan.tail = scan.tail + 1
                    scan.queue[scan.tail] = {nx, nz}
                end
            end
        end
        done = done + 1
        if done % 20 == 0 and mapClock() >= stopAt then
            return
        end
    end
    local crop = 0
    for key, _ in pairs(scan.visited) do
        local cell = PathFinderModule.fieldMap[key]
        if cell ~= nil and (cell[1] ~= 0 or cell[2] ~= 0 or cell[3] ~= 0 or cell[4] ~= 0) then
            crop = crop + 1
        end
    end
    AutoDrive.debugPrint(nil, AutoDrive.DC_PATHINFO, "PathFinderModule field map: field scanned, %d cells of 2 m (%.1f ha), crop %d cells, in %.1f s", scan.count, scan.count * 4 / 10000, crop, (mapClock() - scan.startTime) / 1000)
    PathFinderModule.fieldScan = nil
end

-- cells looked at by the last search, kept for the display (reason: libre, champ, culture, obstacle/pente, vehicule)
function PathFinderModule.addSearchCell(x, z, reason)
    local cells = PathFinderModule.searchCells
    if #cells < 8000 then
        cells[#cells + 1] = {x = x, z = z, reason = reason, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 0, z)}
    end
end

PathFinderModule.SEARCH_COLORS = {
    ["free"] = {0, 1, 1},
    ["field"] = {0, 0, 1},
    ["crop"] = {1, 0, 1},
    ["obstacle/slope"] = {1, 1, 0},
    ["vehicle"] = {1, 0.5, 0}
}

function PathFinderModule.toggleFieldMap()
    PathFinderModule.showFieldMap = not PathFinderModule.showFieldMap
    return "AutoDrive field map: " .. (PathFinderModule.showFieldMap and "shown" or "hidden")
end

-- the map around the player: on field, green crosses = free, red = crop; posts = cells of the last search
function PathFinderModule.drawFieldMap()
    if not PathFinderModule.showFieldMap or drawDebugLine == nil or PathFinderModule.fieldMapCount == 0 then
        return
    end
    local node = nil
    local vehicle = AutoDrive.getControlledVehicle and AutoDrive.getControlledVehicle()
    if vehicle ~= nil and vehicle.rootNode ~= nil then
        node = vehicle.rootNode
    elseif g_localPlayer ~= nil and g_localPlayer.rootNode ~= nil then
        node = g_localPlayer.rootNode
    end
    if node == nil then
        return
    end
    local px, _, pz = getWorldTranslation(node)
    local size = PathFinderModule.MAP_CELL
    local r = math.floor(PathFinderModule.MAP_DRAW_RADIUS / size)
    local cx, cz = math.floor(px / size + 0.5), math.floor(pz / size + 0.5)
    local h = size * 0.35
    for ix = cx - r, cx + r do
        for iz = cz - r, cz + r do
            local cell = PathFinderModule.fieldMap[(ix + 50000) * 100000 + (iz + 50000)]
            if cell ~= nil and cell.onField and cell.y ~= nil then
                local x, z, y = ix * size, iz * size, cell.y + 0.15
                local red, green = 0, 1
                if cell[1] ~= 0 or cell[2] ~= 0 or cell[3] ~= 0 or cell[4] ~= 0 then
                    red, green = 1, 0
                end
                drawDebugLine(x - h, y, z - h, red, green, 0, x + h, y, z + h, red, green, 0, false)
                drawDebugLine(x - h, y, z + h, red, green, 0, x + h, y, z - h, red, green, 0, false)
            end
        end
    end
    local radius = PathFinderModule.MAP_DRAW_RADIUS
    for _, c in ipairs(PathFinderModule.searchCells) do
        if math.abs(c.x - px) < radius and math.abs(c.z - pz) < radius then
            local color = PathFinderModule.SEARCH_COLORS[c.reason] or {1, 1, 1}
            local y = c.y
            drawDebugLine(c.x, y, c.z, color[1], color[2], color[3], c.x, y + 1.5, c.z, color[1], color[2], color[3], false)
        end
    end
end

-- share of the area covered by standing crop (cut or withered crop counts as none)
function PathFinderModule.getFruitRatio(fruitTypeIndex, c1X, c1Z, c2X, c2Z, c3X, c3Z)
    if fruitTypeIndex == FruitType.GRASS or fruitTypeIndex == FruitType.MEADOW then
        return 0
    end
    local fruitValue, _, totalPixels, growthState = FSDensityMapUtil.getFruitArea(fruitTypeIndex, c1X, c1Z, c2X, c2Z, c3X, c3Z, true, true)
    local fruit = g_fruitTypeManager:getFruitTypeByIndex(fruitTypeIndex)
    if fruit ~= nil and growthState ~= nil and (fruit:getIsCut(growthState) or fruit:getIsWithered(growthState)) then
        return 0
    end
    if fruitValue == nil or totalPixels == nil or totalPixels <= 0 then
        return 0
    end
    return fruitValue / totalPixels
end

function PathFinderModule:isOffFieldReduced(x, z, isOnField)
    if self.gridScaleStep == nil or self.gridScaleStep <= 1 then
        return false
    end
    if isOnField == nil then
        isOnField = PathFinderModule.getMapOnFieldAt(x, z)
    end
    return not isOnField
end

PathFinderModule.START_DISTANCES_SHORTER = {5, 3, 1.5}

-- longest start distance (up to PATHFINDER_START_DISTANCE) whose corridor, vehicle -> start point + turning radius at train width, hits no obstacle
function PathFinderModule:getFreeStartDistance(vehicleWorldX, vehicleWorldZ, dirX, dirZ)
    local maxDistance = self.PATHFINDER_START_DISTANCE
    local distances = {maxDistance}
    for _, d in ipairs(PathFinderModule.START_DISTANCES_SHORTER) do
        if d < maxDistance then
            table.insert(distances, d)
        end
    end
    local halfWidth = ADPathCalculator.getPassageWidth(self.vehicle) / 2 + 0.5
    local angleRad = math.atan2(dirX, dirZ)
    local height = self.vehicleMinHeight or 6
    -- from the front of the vehicle, so its own wheels and body are not in the box
    local frontOffset = (self.vehicle.size ~= nil and self.vehicle.size.length ~= nil and self.vehicle.size.length / 2 or 2.5) + 0.3
    -- beyond the start point the vehicle still needs about its turning radius to turn: check that too
    local turnSpace = math.max(2, self.minTurnRadius or 2)
    for _, distance in ipairs(distances) do
        local length = distance + turnSpace - frontOffset
        if length <= 0.5 then
            return distance
        end
        local cx = vehicleWorldX + dirX * (frontOffset + length / 2)
        local cz = vehicleWorldZ + dirZ * (frontOffset + length / 2)
        local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, cx, 1, cz)
        self.collisionhits = 0
        overlapBox(cx, y + height / 2 + 0.3, cz, 0, angleRad, 0, halfWidth, height / 2, length / 2, "collisionTestCallback", self, self.mask, true, true, true, true)
        if self:boxHitsBlockedZone(cx, cz, angleRad, halfWidth, length / 2) then
            self.collisionhits = self.collisionhits + 1
        end
        if self.collisionhits == 0 then
            if distance ~= maxDistance then
                PathFinderModule.debugMsg(self.vehicle, "PathFinderModule:getFreeStartDistance obstacle ahead, start %.1f m in front instead of %.1f m", distance, maxDistance)
            end
            return distance
        end
    end
    local shortest = distances[#distances]
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule:getFreeStartDistance obstacle right in front, start %.1f m in front", shortest)
    return shortest
end

function PathFinderModule:startPathPlanningTo(targetPoint, targetVector)
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule:startPathPlanningTo self.minTurnRadius %.1f"
        , self.minTurnRadius
    )

    PathFinderModule.debugVehicleMsg(self.vehicle,
        string.format("PFM startPathPlanningTo targetPoint x,z %d %d",
            math.floor(targetPoint.x),
            math.floor(targetPoint.z)
        )
    )
    ADScheduler:addPathfinderVehicle(self.vehicle)
    self.minTurnRadius = self.realTurnRadius or self.minTurnRadius
    self.gridScaleStep = 1
    self.gridScaleFailPrinted = false
    self.partialPath = false
    if math.abs(targetVector.x) < 0.001 then
        targetVector.x = 0.001
    end
    if math.abs(targetVector.z) < 0.001 then
        targetVector.z = 0.001
    end
    self.targetVector = targetVector
    local vehicleWorldX, vehicleWorldY, vehicleWorldZ = getWorldTranslation(self.vehicle.components[1].node)
    local vehicleRx, _, vehicleRz =  AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local vehicleVector = {x = vehicleRx, z = vehicleRz}
    -- the stretch from the vehicle to the start point is driven unchecked: pull the start closer if a tree, wall... is on it
    local startDistance = self:getFreeStartDistance(vehicleWorldX, vehicleWorldZ, vehicleRx, vehicleRz)
    self.startX = vehicleWorldX + startDistance * vehicleRx
    self.startZ = vehicleWorldZ + startDistance * vehicleRz

    local angleRad = math.atan2(targetVector.z, targetVector.x)
    angleRad = AutoDrive.normalizeAngle(angleRad)

    self.vectorX = {x =   math.cos(angleRad) * self.minTurnRadius, z = math.sin(angleRad) * self.minTurnRadius}
    self.vectorZ = {x = - math.sin(angleRad) * self.minTurnRadius, z = math.cos(angleRad) * self.minTurnRadius}

    --Make the target a few meters ahead of the road to the start point
    local targetX = targetPoint.x - math.cos(angleRad) * self.PATHFINDER_TARGET_DISTANCE
    local targetZ = targetPoint.z - math.sin(angleRad) * self.PATHFINDER_TARGET_DISTANCE

    self.grid = {}
    self.steps = 0
    self.retryCounter = 0
    self.isFinished = false
    self.fallBackMode1 = false  -- disable restrict to field
    self.fallBackMode2 = false  -- disable restrict to field border
    self.fallBackMode3 = false  -- disable avoid fruit
    self.max_pathfinder_steps = PathFinderModule.MAX_PATHFINDER_STEPS_TOTAL * self.pathFinderTime

    self.fruitToCheck = nil
    self.trainHalfWidth = self:getTrainWidth() / 2 + PathFinderModule.FRUIT_CHECK_CORRIDOR_MARGIN

    self.start = {x = self.startX, z = self.startZ}
    self.startCell = {x = 0, z = 0}
    self.startCell.direction = self:worldDirectionToGridDirection(vehicleVector)
    self.startCell.visited = false
    self.startCell.out = nil
    self.startCell.isRestricted = false
    self.startCell.hasCollision = false
    self.startCell.hasFruit = false
    self.startCell.steps = 0
    self.startCell.bordercells = 0
    self.currentCell = nil

    local vehicleBehindX, _, vehicleBehindZ =  AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, -self.minTurnRadius)
    local vehicleBehindVector = {x = vehicleBehindX, z = vehicleBehindZ}
    self.behindStartCell = self:worldLocationToGridLocation(vehicleWorldX + vehicleBehindX, vehicleWorldZ + vehicleBehindZ)
    self.behindStartCell.direction = self:worldDirectionToGridDirection(vehicleBehindVector, vehicleVector)
    self.behind = {x = vehicleWorldX + vehicleBehindX, z = vehicleWorldZ + vehicleBehindZ}

    -- table.insert(self.grid, self.startCell)
    local gridKey = string.format("%d|%d|%d", self.startCell.x, self.startCell.z, self.startCell.direction)
    self.grid[gridKey] = self.startCell

    self.smoothStep = 0
    self.smoothDone = false
    self.target = {x = targetX, z = targetZ}

    local targetCellZ = (((targetX - self.startX) / self.vectorX.x) * self.vectorX.z - targetZ + self.startZ) / (((self.vectorZ.x / self.vectorX.x) * self.vectorX.z) - self.vectorZ.z)
    local targetCellX = (targetZ - self.startZ - targetCellZ * self.vectorZ.z) / self.vectorX.z
    targetCellX = AutoDrive.round(targetCellX)
    targetCellZ = AutoDrive.round(targetCellZ)
    self.targetCell = {x = targetCellX, z = targetCellZ, direction = self.PP_UP}
    self.targetAhead = {x = targetX + self.vectorX.x, z = targetZ + self.vectorX.z}
    self.targetAheadCell = self:worldLocationToGridLocation(self.targetAhead.x, self.targetAhead.z)

    self:determineBlockedCells(self.targetCell)
    --self:checkGridCell(self.targetCell)

    self.appendWayPoints = {}
    self.appendWayPoints[1] = targetPoint

    self.goingToCombine = false

    self.startIsOnField = AutoDrive.checkIsOnField(vehicleWorldX, vehicleWorldY, vehicleWorldZ) and self.vehicle.ad.sensors.frontSensorField:pollInfo(true)
    self.endIsOnField = AutoDrive.checkIsOnField(targetX, vehicleWorldY, targetZ)
    self.restrictToField = AutoDrive.getSetting("restrictToField", self.vehicle) and self.startIsOnField and self.endIsOnField
    self.goingToPipe = false
    self.pipeCombine = nil
    self.chasingVehicle = false
    self.isSecondChasingVehicle = false
    self.goingToNetwork = false
    self.destinationId = nil
    self.completelyBlocked = false
    self.targetBlocked = false --self.targetCell.hasCollision or self.targetCell.isRestricted --> always false TODO: how to use this ?
    self.blockedByOtherVehicle = false
    self.avoidFruitSetting = AutoDrive.getSetting("avoidFruit", self.vehicle)
    -- strict settings are never dropped by the fallback modes, whatever the trip
    self.strictField = AutoDrive.getSetting("restrictToField", self.vehicle) == "strict"
    self.strictFruit = self.avoidFruitSetting == "strict"
    if self.strictField then
        -- strict: never drive on another field, wherever the trip starts or ends. Ground that is not a field (grass, verges) stays allowed
        self.restrictToField = true
        -- the field is identified by the map's field outline, a field can lie on several farmlands
        self.allowedFields = {}
        self.allowedFarmlands = {}
        for _, pos in ipairs({{x = vehicleWorldX, z = vehicleWorldZ, onField = self.startIsOnField}, {x = targetX, z = targetZ, onField = self.endIsOnField}}) do
            if pos.onField then
                local field = PathFinderModule.getFieldAtPosition(pos.x, pos.z)
                if field ~= nil then
                    self.allowedFields[field] = true
                end
                self.allowedFarmlands[g_farmlandManager:getFarmlandIdAtWorldPosition(pos.x, pos.z)] = true
            end
        end
        -- the field ground actually connected to the start and to the target, outlines and farmlands do not always match the worked field
        self.allowedAreas = {}
        for _, pos in ipairs({{x = vehicleWorldX, z = vehicleWorldZ}, {x = targetX, z = targetZ}}) do
            local area = PathFinderModule.getConnectedFieldArea(pos.x, pos.z)
            if area ~= nil then
                self.allowedAreas[area] = true
                PathFinderModule.debugMsg(self.vehicle, "PFM:startPathPlanningTo strict field area xz %.1f,%.1f samples %d capped %s", pos.x, pos.z, area.count, tostring(area.capped))
            end
        end
    end
    self.islandCache = {}
    self.strictFruitRefusals = 0
    self.mapReadsAtStart = PathFinderModule.fieldMapReads
    PathFinderModule.searchCells = {}
    -- self.targetFieldId = g_farmlandManager:getFarmlandIdAtWorldPosition(targetX, targetZ)   -- only has ID if vector target is onField
    -- self.targetFieldId = nil
    -- if self.restrictToField and self.targetFieldId ~= nil and self.targetFieldId > 0 then
        -- self.reachedFieldBorder = startIsOnField
        -- local targetFieldPos = {x = g_farmlandManager.farmlands[self.targetFieldId].xWorldPos, z = g_farmlandManager.farmlands[self.targetFieldId].zWorldPos}

        -- self.fieldCell = self:worldLocationToGridLocation(targetFieldPos.x, targetFieldPos.z)
    -- else
        -- self.reachedFieldBorder = true
    -- end

    self.q0 = {
        vehicleWorldX
        , -vehicleWorldZ
        , AutoDrive.normalizeAngle(math.atan2(vehicleRx, vehicleRz) + math.pi + math.pi / 2)
    }

    self.q1 = {
        targetX
        , -targetZ
        , AutoDrive.normalizeAngle(math.atan2(targetVector.x, targetVector.z) + math.pi + math.pi / 2)
    }

    self:setupNew(self.behindStartCell, self.startCell,self.targetCell)

    self.chainStartToTarget = {}

    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule:startPathPlanningTo vehicleWorldX xz %.1f,%.1f"
        , vehicleWorldX, vehicleWorldZ
    )

    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule:startPathPlanningTo targetX xz %.1f,%.1f"
        , targetX, targetZ
    )

    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule:startPathPlanningTo self.behind xz %.1f,%.1f"
        , self.behind.x, self.behind.z
    )

    local location = self:gridLocationToWorldLocation(self.behindStartCell)
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule:startPathPlanningTo behindStartCell location xz %.1f,%.1f"
        , location.x, location.z
    )

    PathFinderModule.debugVehicleMsg(self.vehicle,
        string.format("PFM startPathPlanningTo self.minTurnRadius %s vectorX.x,vectorX.z %s %s vectorZ.x,vectorZ.z %s %s",
            tostring(self.minTurnRadius),
            tostring(self.vectorX.x),
            tostring(self.vectorX.z),
            tostring(self.vectorZ.x),
            tostring(self.vectorZ.z)
        )
    )
    PathFinderModule.debugVehicleMsg(self.vehicle,
        string.format("PFM startPathPlanningTo startCell xz %d %d direction %s",
            math.floor(self.startCell.x),
            math.floor(self.startCell.z),
            tostring(self.direction_to_text[self.startCell.direction+1])
        )
    )
    PathFinderModule.debugVehicleMsg(self.vehicle,
        string.format("PFM startPathPlanningTo start targetCell xz %d %d direction %s",
            math.floor(self.targetCell.x),
            math.floor(self.targetCell.z),
            tostring(self.direction_to_text[self.targetCell.direction+1])
        )
    )
    PathFinderModule.debugVehicleMsg(self.vehicle,
        string.format("PFM startPathPlanningTo restrictToField %s endIsOnField %s",
            tostring(self.restrictToField),
            tostring(self.endIsOnField)
        )
    )
    PathFinderModule.debugVehicleMsg(self.vehicle,
        string.format("PFM startPathPlanningTo self.fruitToCheck %s getSetting avoidFruit %s",
            tostring(self.fruitToCheck),
            tostring(self.avoidFruitSetting)
        )
    )
end

function PathFinderModule:printTrip(kind)
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: path (%s, cells %.1f m) start %.0f,%.0f -> target %.0f,%.0f", kind, self.minTurnRadius or 0,
        self.startX or 0, self.startZ or 0, self.target ~= nil and self.target.x or 0, self.target ~= nil and self.target.z or 0)
end

function PathFinderModule:getLogName()
    return self.vehicle ~= nil and self.vehicle.getName ~= nil and self.vehicle:getName() or "?"
end

-- no path found: same start, target and restrictions, but a finer grid
function PathFinderModule:shrinkGrid()
    local oldSize = self.minTurnRadius
    self.gridScaleStep = self.gridScaleStep + 1
    local scale = self:getGridScale(self.gridScaleStep)
    self.minTurnRadius = self.realTurnRadius * scale
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: no path with %.1f m cells, trying %.1f m (%s)", oldSize, self.minTurnRadius, self:getDiagText())

    local angleRad = AutoDrive.normalizeAngle(math.atan2(self.targetVector.z, self.targetVector.x))
    self.vectorX = {x =   math.cos(angleRad) * self.minTurnRadius, z = math.sin(angleRad) * self.minTurnRadius}
    self.vectorZ = {x = - math.sin(angleRad) * self.minTurnRadius, z = math.cos(angleRad) * self.minTurnRadius}

    local vehicleWorldX, _, vehicleWorldZ = getWorldTranslation(self.vehicle.components[1].node)
    local vehicleRx, _, vehicleRz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local vehicleVector = {x = vehicleRx, z = vehicleRz}
    self.startCell.direction = self:worldDirectionToGridDirection(vehicleVector)

    local vehicleBehindX, _, vehicleBehindZ = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, -self.minTurnRadius)
    self.behindStartCell = self:worldLocationToGridLocation(vehicleWorldX + vehicleBehindX, vehicleWorldZ + vehicleBehindZ)
    self.behindStartCell.direction = self:worldDirectionToGridDirection({x = vehicleBehindX, z = vehicleBehindZ}, vehicleVector)
    self.behind = {x = vehicleWorldX + vehicleBehindX, z = vehicleWorldZ + vehicleBehindZ}

    local targetCell = self:worldLocationToGridLocation(self.target.x, self.target.z)
    self.targetCell = {x = AutoDrive.round(targetCell.x), z = AutoDrive.round(targetCell.z), direction = self.PP_UP}
    self.targetAhead = {x = self.target.x + self.vectorX.x, z = self.target.z + self.vectorX.z}
    self.targetAheadCell = self:worldLocationToGridLocation(self.targetAhead.x, self.targetAhead.z)

    -- smaller cells cover less ground per step
    self.max_pathfinder_steps = math.floor(self.max_pathfinder_steps / scale * self:getGridScale(self.gridScaleStep - 1))
    self:autoRestart()
end

-- straight stretch a -> b: obstacles, blocked zones, crops and field limit, as the shortcut check does
-- inner: optional unit vector towards the turn centre, the trailer cuts inside a turn
-- narrow: corridor of the train only (used to verify a whole path, hedges and trees along it are normal)
-- crop along a -> b, judged in short slices: a local bite into the crop must not vanish in the average of a long stretch
function PathFinderModule:isFruitStretchClear(a, b, halfWidth)
    local vectorX, vectorZ = b.x - a.x, b.z - a.z
    local length = math.sqrt(vectorX * vectorX + vectorZ * vectorZ)
    if length < 0.05 then
        return true
    end
    local angleRad = AutoDrive.normalizeAngle(math.atan2(-vectorZ, vectorX))
    local leftAngle = AutoDrive.normalizeAngle(angleRad + math.rad(-90))
    local rightAngle = AutoDrive.normalizeAngle(angleRad + math.rad(90))
    local slices = math.max(1, math.ceil(length / PathFinderModule.FRUIT_SLICE_LENGTH))
    local cosL, sinL = math.cos(leftAngle), math.sin(leftAngle)
    local cosR, sinR = math.cos(rightAngle), math.sin(rightAngle)
    for k = 0, slices - 1 do
        local sx, sz = a.x + vectorX * k / slices, a.z + vectorZ * k / slices
        local ex, ez = a.x + vectorX * (k + 1) / slices, a.z + vectorZ * (k + 1) / slices
        local c1X, c1Z = sx - cosL * halfWidth, sz + sinL * halfWidth
        local c2X, c2Z = ex - cosL * halfWidth, ez + sinL * halfWidth
        local c4X, c4Z = sx - cosR * halfWidth, sz + sinR * halfWidth
        local filter = self:getCropFilter()
        if filter ~= nil or self.goingToNetwork or self.strictFruit then
            local ratio, found = PathFinderModule.getMapCropRatio(filter, c1X, c1Z, c2X, c2Z, c4X, c4Z)
            -- same rule as the grid cells: the train may run along the crop edge, not into it
            if ratio > PathFinderModule.CELL_MAX_RATIO then
                local fruitType = found ~= nil and g_fruitTypeManager:getFruitTypeByIndex(found) or nil
                return false, "crop " .. tostring(fruitType ~= nil and fruitType.name or "")
            end
        end
    end
    return true
end

function PathFinderModule:isStretchClear(a, b, inner, narrow, obstaclesOnly)
    local vectorX, vectorZ = b.x - a.x, b.z - a.z
    local length = math.sqrt(vectorX * vectorX + vectorZ * vectorZ)
    if length < 0.05 then
        return true
    end
    local angleRad = AutoDrive.normalizeAngle(math.atan2(-vectorZ, vectorX))
    local leftAngle = AutoDrive.normalizeAngle(angleRad + math.rad(-90))
    local rightAngle = AutoDrive.normalizeAngle(angleRad + math.rad(90))
    local halfWidth = self.trainHalfWidth or 3
    local midX, midZ = a.x + vectorX / 2, a.z + vectorZ / 2
    -- same corridor as the shortcut check of the grid path, at least the train width
    local boxHalf = math.max(self.minTurnRadius * PathFinderModule.GRID_SIZE_FACTOR + 1.5, halfWidth + 0.5)
    if narrow then
        boxHalf = halfWidth + 0.5
    end
    -- trees on the grass are not always seen: wider corridor off the field in reduced grids
    if self:isOffFieldReduced(midX, midZ) then
        boxHalf = boxHalf + PathFinderModule.OFF_FIELD_EXTRA_MARGIN
    end
    local boxX, boxZ = midX, midZ
    if inner ~= nil then
        boxHalf = boxHalf + PathFinderModule.CORNER_INNER_MARGIN / 2
        boxX = boxX + inner.x * PathFinderModule.CORNER_INNER_MARGIN / 2
        boxZ = boxZ + inner.z * PathFinderModule.CORNER_INNER_MARGIN / 2
    end
    local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, boxX, 1, boxZ)

    self.collisionhits = 0
    overlapBox(boxX, y + self.vehicleMinHeight / 2, boxZ, 0, angleRad, 0, length / 2 + 1, self.vehicleMinHeight / 2, boxHalf, "collisionTestCallback", self, self.mask, true, true, true, true)
    if self.collisionhits > 0 or self:boxHitsBlockedZone(boxX, boxZ, angleRad, length / 2 + 1, boxHalf) then
        return false, "obstacle"
    end
    if obstaclesOnly then
        return true
    end
    if self:checkSlopeAngle(a.x, a.z, b.x, b.z) then
        return false, "slope"
    end

    if self.avoidFruitSetting and not self.fallBackMode3 then
        local clear, reason = self:isFruitStretchClear(a, b, halfWidth)
        if not clear then
            return false, reason
        end
    end

    if self.restrictToField and not (self.fallBackMode1 or self.fallBackMode2) then
        local sideHalf = halfWidth - PathFinderModule.FRUIT_CHECK_CORRIDOR_MARGIN
        local sideX, sideZ = math.cos(leftAngle) * sideHalf, -math.sin(leftAngle) * sideHalf
        for _, t in ipairs({0, 0.5, 1}) do
            local px, pz = a.x + vectorX * t, a.z + vectorZ * t
            for _, sign in ipairs({1, -1}) do
                local allowed, onOtherField = self:isTrainSideAllowed(px + sideX * sign, pz + sideZ * sign)
                if not allowed then
                    return false, onOtherField and "field edge (other field)" or "field edge (off the field)"
                end
            end
        end
    end
    return true
end

PathFinderModule.PARTIAL_MIN_GAIN = 10

function PathFinderModule:partialAllowed()
    return self.allowPartialPath and (self.strictFruit or self.strictField) and not self.partialPath and self.isNewPF
end

-- a partial route must bring the vehicle at least PARTIAL_MIN_GAIN closer to the target
function PathFinderModule:partialGain(endPoint)
    local startDistance = MathUtil.vector2Length(self.startX - self.target.x, self.startZ - self.target.z)
    local endDistance = MathUtil.vector2Length(endPoint.x - self.target.x, endPoint.z - self.target.z)
    return startDistance - endDistance
end

-- refused route: keep its clean beginning, up to the first refused stretch
function PathFinderModule:usePartialPath(badIndex)
    -- a partial route can be shortened again
    if not (self:partialAllowed() or self.partialPath) or badIndex == nil or badIndex < 2 then
        return false
    end
    local points = {}
    for i = 1, badIndex do
        points[i] = self.wayPoints[i]
    end
    local gain = self:partialGain(points[#points])
    if gain < PathFinderModule.PARTIAL_MIN_GAIN then
        return false
    end
    self.wayPoints = points
    self.partialPath = true
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: partial path to %.0f,%.0f (%.0f m closer), new search on arrival", points[#points].x, points[#points].z, gain)
    return true
end

-- end of a partial route to a harvester: never waiting face to face with it, against its way
PathFinderModule.PARTIAL_MAX_HEADING_ANGLE = math.rad(120)
PathFinderModule.PARTIAL_HEADING_LENGTH = 3

function PathFinderModule:isPartialEndFacingCombine(index)
    local wps = self.wayPoints
    if not self.goingToPipe or self.pipeCombine == nil or self.pipeCombine.components == nil or index < 2 then
        return false
    end
    local b = wps[index]
    local a = nil
    for j = index - 1, 1, -1 do
        a = wps[j]
        if MathUtil.vector2Length(b.x - a.x, b.z - a.z) >= PathFinderModule.PARTIAL_HEADING_LENGTH then
            break
        end
    end
    local dx, dz = b.x - a.x, b.z - a.z
    local length = math.sqrt(dx * dx + dz * dz)
    if length < 0.5 then
        return false
    end
    local rx, _, rz = AutoDrive.localDirectionToWorld(self.pipeCombine, 0, 0, 1)
    return (dx * rx + dz * rz) / length < math.cos(PathFinderModule.PARTIAL_MAX_HEADING_ANGLE)
end

-- shortens the partial route to its last point not facing the harvester; false: no usable point, wait
function PathFinderModule:trimPartialEnd()
    local wps = self.wayPoints
    for i = #wps, 2, -1 do
        if self:partialGain(wps[i]) < PathFinderModule.PARTIAL_MIN_GAIN then
            return false
        end
        if not self:isPartialEndFacingCombine(i) then
            if i < #wps then
                for j = #wps, i + 1, -1 do
                    table.remove(wps, j)
                end
                PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: end of the partial path faces the harvester, stopping earlier at %.0f,%.0f", wps[i].x, wps[i].z)
            end
            return self:keepPartialStopMargin()
        end
    end
    return false
end

-- a partial route stops with margin: PARTIAL_STOP_MARGIN meters back along the route, and the train standing there
-- (its width + 1 m each side) touches no crop. Returns false if nothing worth driving is left
PathFinderModule.PARTIAL_STOP_MARGIN = 10
function PathFinderModule:isPartialStopClear(i)
    local wps = self.wayPoints
    local a, b = wps[i - 1], wps[i]
    local dx, dz = b.x - a.x, b.z - a.z
    local l = math.sqrt(dx * dx + dz * dz)
    if l < 0.05 then
        return true
    end
    dx, dz = dx / l, dz / l
    local half = (self.trainHalfWidth or 2) + 1
    local back = math.max(AutoDrive.getTractorTrainLength(self.vehicle, true, false), 8)
    local c1X, c1Z = b.x - dz * half - dx * back, b.z + dx * half - dz * back
    local c2X, c2Z = b.x - dz * half + dx * 2, b.z + dx * half + dz * 2
    local c4X, c4Z = b.x + dz * half - dx * back, b.z - dx * half - dz * back
    return PathFinderModule.getMapCropRatio(nil, c1X, c1Z, c2X, c2Z, c4X, c4Z) <= 0
end

function PathFinderModule:keepPartialStopMargin()
    local wps = self.wayPoints
    local removed = 0
    while #wps > 2 do
        local last, before = wps[#wps], wps[#wps - 1]
        if removed >= PathFinderModule.PARTIAL_STOP_MARGIN and self:isPartialStopClear(#wps) then
            break
        end
        removed = removed + MathUtil.vector2Length(last.x - before.x, last.z - before.z)
        table.remove(wps, #wps)
    end
    if #wps < 2 or self:partialGain(wps[#wps]) < PathFinderModule.PARTIAL_MIN_GAIN then
        PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: partial path too short with the margin, waiting")
        return false
    end
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: partial path stopped %.0f m earlier (margin), at %.0f,%.0f", removed, wps[#wps].x, wps[#wps].z)
    return true
end

-- behind the harvester: on its own cut track, the chase takes over from there
PathFinderModule.REAR_FALLBACK_GAP = 8
PathFinderModule.REAR_FIRST_DISTANCE = 40

function PathFinderModule:startRearFallback()
    local combine = self.pipeCombine
    if self.rearFallbackDone or not self.goingToPipe or combine == nil or combine.components == nil or not (self.strictFruit or self.strictField) then
        return false
    end
    -- only to catch a harvester that drives: behind a stopped one the trailer cannot reach the pipe, nothing takes over there
    local stopped = (combine.lastSpeedReal or 0) < 0.0002
        or (combine.ad ~= nil and combine.ad.noMovementTimer ~= nil and combine.ad.noMovementTimer.elapsedTime > 1000)
    if stopped then
        if not self.rearFallbackSkippedPrinted then
            self.rearFallbackSkippedPrinted = true
            PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: harvester stopped, aiming at the pipe directly (not behind)")
        end
        return false
    end
    local x, y, z = getWorldTranslation(combine.components[1].node)
    local rx, _, rz = AutoDrive.localDirectionToWorld(combine, 0, 0, 1)
    local back = (combine.size ~= nil and combine.size.length or 10) / 2 + PathFinderModule.REAR_FALLBACK_GAP + AutoDrive.getTractorTrainLength(self.vehicle, true, false) / 2
    local target = {x = x - rx * back, y = y, z = z - rz * back}
    local allowPartial = self.allowPartialPath
    local fruitToCheck = self.fruitToCheck
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: going behind the harvester (%.0f,%.0f)", target.x, target.z)
    self:startPathPlanningTo(target, {x = rx, z = rz})
    self.goingToPipe = true
    self.pipeCombine = combine
    self.fruitToCheck = fruitToCheck
    self.allowPartialPath = allowPartial
    self.rearFallbackDone = true
    return true
end

-- no route at the finest grid: route to the explored cell closest to the target
function PathFinderModule:usePartialGridPath()
    if not self:partialAllowed() or self.closedset == nil or self.nodeGoal == nil then
        return false
    end
    local best, bestDistance = nil, math.huge
    for node, _ in pairs(self.closedset) do
        if node ~= self.nodeBehindStart and self.came_from[node] ~= nil then
            local distance = MathUtil.vector2Length(node.x - self.nodeGoal.x, node.z - self.nodeGoal.z)
            if distance < bestDistance then
                best, bestDistance = node, distance
            end
        end
    end
    if best == nil or best == self.nodeStart then
        return false
    end
    local endPoint = self:gridLocationToWorldLocation(best)
    if self:partialGain(endPoint) < PathFinderModule.PARTIAL_MIN_GAIN then
        return false
    end
    self.path = self:unwind_path({}, self.came_from, best)
    table.insert(self.path, best)
    self.partialPath = true
    self.completelyBlocked = false
    self.isFinished = true
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: no complete path, partial path to %.0f,%.0f, new search on arrival", endPoint.x, endPoint.z)
    return true
end

-- the first meters (at the vehicle) and the last meters (next to the target) are not judged
PathFinderModule.VERIFY_SKIP_START = 12
PathFinderModule.START_CROP_RADIUS = 4
PathFinderModule.VERIFY_SKIP_END = 15
function PathFinderModule:findBlockedStretch()
    local wps = self.wayPoints
    if wps == nil then
        return nil
    end
    -- distances along the route: the first and last meters are not judged (start turn, approach)
    local total = 0
    for i = 1, #wps - 1 do
        total = total + MathUtil.vector2Length(wps[i + 1].x - wps[i].x, wps[i + 1].z - wps[i].z)
    end
    local skipFrom, skipTo = PathFinderModule.VERIFY_SKIP_START, total - PathFinderModule.VERIFY_SKIP_END
    if self.partialPath then
        -- a partial route stops before the target: its end is where the vehicle waits, it is judged too
        skipTo = total
    end
    local done = 0
    for i = 1, #wps - 1 do
        local a, b = wps[i], wps[i + 1]
        local dx, dz = b.x - a.x, b.z - a.z
        local length = math.sqrt(dx * dx + dz * dz)
        local from = math.max(0, skipFrom - done)
        local to = math.min(length, skipTo - done)
        -- start turn: crop and field edge not judged, obstacles still are
        local startTo = math.min(length, skipFrom - done, skipTo - done)
        if startTo > 0.5 then
            local pb = {x = a.x + dx * startTo / length, z = a.z + dz * startTo / length}
            if not self:isStretchClear(a, pb, nil, true, true) then
                return i, "obstacle"
            end
        end
        done = done + length
        if to - from > 0.5 then
            local pa = {x = a.x + dx * from / length, z = a.z + dz * from / length}
            local pb = {x = a.x + dx * to / length, z = a.z + dz * to / length}
            local clear, reason = self:isStretchClear(pa, pb, nil, true)
            if not clear then
                return i, reason
            end
        end
    end
    return nil
end

PathFinderModule.CORNER_MIN_ANGLE = math.rad(8)
PathFinderModule.CORNER_MAX_ANGLE = math.rad(150)
PathFinderModule.CORNER_MAX_MERGE = 4
PathFinderModule.CORNER_ARC_STEP = 2.5
PathFinderModule.CORNER_INNER_MARGIN = 1.5
-- a corner that could not be rounded to the real turn radius and turns more than this cannot be driven by the train
PathFinderModule.CORNER_SHARP_ANGLE = math.rad(30)

-- the finer grid makes corners tighter than the vehicle can turn: replace them by arcs of the real radius
-- a corner too short for its arc is merged with the next corners turning the same way; no room -> corner kept
function PathFinderModule:roundCornersToTurnRadius()
    local wps = self.wayPoints
    if wps == nil or #wps < 3 then
        return
    end
    local radius = self.realTurnRadius or self.minTurnRadius
    local function unit(from, to)
        local dx, dz = to.x - from.x, to.z - from.z
        local l = math.sqrt(dx * dx + dz * dz)
        if l < 0.01 then
            return nil
        end
        return {x = dx / l, z = dz / l}, l
    end
    local function newPoint(x, z, template)
        local p = {x = x, z = z, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 1, z)}
        if template ~= nil then
            p.direction = template.direction
        end
        return p
    end

    local out = {wps[1]}
    local last = wps[1]
    local rounded, kept, sharp = 0, 0, 0
    local i = 2
    while i <= #wps - 1 do
        local d1 = unit(wps[i - 1], wps[i])
        local done = false
        if d1 ~= nil then
            local turnSign = nil
            for k = i, math.min(i + PathFinderModule.CORNER_MAX_MERGE - 1, #wps - 1) do
                local d2 = unit(wps[k], wps[k + 1])
                if d2 == nil then
                    break
                end
                local cross = d1.x * d2.z - d1.z * d2.x
                local dot = math.max(-1, math.min(1, d1.x * d2.x + d1.z * d2.z))
                local theta = math.acos(dot)
                -- every merged corner must turn the same way
                local localCross = 0
                local dPrev = unit(wps[k - 1], wps[k])
                if dPrev ~= nil then
                    localCross = dPrev.x * d2.z - dPrev.z * d2.x
                end
                if math.abs(localCross) > 0.01 then
                    local sign = localCross > 0 and 1 or -1
                    if turnSign == nil then
                        turnSign = sign
                    elseif sign ~= turnSign then
                        break
                    end
                end
                if theta < PathFinderModule.CORNER_MIN_ANGLE then
                    if k == i then
                        break
                    end
                elseif theta > PathFinderModule.CORNER_MAX_ANGLE then
                    break
                else
                    -- virtual corner: incoming line through wps[i] (d1) crosses the outgoing line through wps[k] (d2)
                    local denom = d1.x * d2.z - d1.z * d2.x
                    if math.abs(denom) > 0.001 then
                        local wx, wz = wps[k].x - wps[i].x, wps[k].z - wps[i].z
                        local s = (wx * d2.z - wz * d2.x) / denom
                        local vx, vz = wps[i].x + d1.x * s, wps[i].z + d1.z * s
                        local tangent = radius * math.tan(theta / 2)
                        local t1 = {x = vx - d1.x * tangent, z = vz - d1.z * tangent}
                        local t2 = {x = vx + d2.x * tangent, z = vz + d2.z * tangent}
                        local nextPoint = wps[k + 1]
                        -- the arc must start after the last kept point and end before the next point (half way if more corners follow)
                        local beforeOk = ((t1.x - last.x) * d1.x + (t1.z - last.z) * d1.z) >= 0
                        local _, outLength = unit({x = vx, z = vz}, nextPoint)
                        local outRoom = outLength or 0
                        if k + 1 < #wps then
                            outRoom = outRoom * 0.5
                        end
                        local afterOk = ((nextPoint.x - vx) * d2.x + (nextPoint.z - vz) * d2.z) > 0 and tangent <= outRoom
                        if beforeOk and afterOk then
                            local perpX, perpZ = -d1.z, d1.x
                            if cross < 0 then
                                perpX, perpZ = d1.z, -d1.x
                            end
                            local cx, cz = t1.x + perpX * radius, t1.z + perpZ * radius
                            local a0 = math.atan2(t1.z - cz, t1.x - cx)
                            local sweep = (cross > 0 and 1 or -1) * theta
                            local steps = math.max(2, math.ceil(theta * radius / PathFinderModule.CORNER_ARC_STEP))
                            local arc = {newPoint(t1.x, t1.z, wps[i])}
                            for n = 1, steps do
                                local a = a0 + sweep * n / steps
                                arc[#arc + 1] = newPoint(cx + math.cos(a) * radius, cz + math.sin(a) * radius, wps[k])
                            end
                            local clear = self:isStretchClear(last, arc[1]) and self:isStretchClear(arc[#arc], nextPoint)
                            for n = 2, #arc do
                                if not clear then
                                    break
                                end
                                local midX, midZ = (arc[n - 1].x + arc[n].x) / 2, (arc[n - 1].z + arc[n].z) / 2
                                local toCentreX, toCentreZ = cx - midX, cz - midZ
                                local toCentreLength = math.sqrt(toCentreX * toCentreX + toCentreZ * toCentreZ)
                                local inner = {x = toCentreX / toCentreLength, z = toCentreZ / toCentreLength}
                                clear = self:isStretchClear(arc[n - 1], arc[n], inner)
                            end
                            if clear then
                                for _, p in ipairs(arc) do
                                    out[#out + 1] = p
                                end
                                last = arc[#arc]
                                rounded = rounded + 1
                                i = k + 1
                                done = true
                                break
                            end
                        end
                    end
                end
            end
        end
        if not done then
            local d2 = unit(wps[i], wps[i + 1])
            if d1 ~= nil and d2 ~= nil then
                local theta = math.acos(math.max(-1, math.min(1, d1.x * d2.x + d1.z * d2.z)))
                if theta >= PathFinderModule.CORNER_MIN_ANGLE then
                    kept = kept + 1
                end
                if theta > PathFinderModule.CORNER_SHARP_ANGLE then
                    sharp = sharp + 1
                end
            end
            out[#out + 1] = wps[i]
            last = wps[i]
            i = i + 1
        end
    end
    out[#out + 1] = wps[#wps]
    self.wayPoints = out
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: corners rounded at %.1f m: %d, corners kept (no room): %d, too sharp (> %.0f deg): %d", radius, rounded, kept, math.deg(PathFinderModule.CORNER_SHARP_ANGLE), sharp)
    return kept, sharp
end

function PathFinderModule:restartAtNextWayPoint()
    self.targetWayPointId = self.appendWayPoints[2].id
    local targetNode = ADGraphManager:getWayPointById(self.targetWayPointId)
    local wayPoints = ADGraphManager:pathFromTo(self.targetWayPointId, self.destinationId, nil, self.vehicle)
    if wayPoints ~= nil and #wayPoints > 1 then
        local vecToNextPoint = {x = wayPoints[2].x - targetNode.x, z = wayPoints[2].z - targetNode.z}
        local storedRetryCounter = self.retryCounter
        local storedTargetWayPointId = self.targetWayPointId
        local storedDestinationId = self.destinationId
        self:startPathPlanningTo(targetNode, vecToNextPoint)
        self.retryCounter = storedRetryCounter
        self.destinationId = storedDestinationId
        self.fallBackMode1 = false  -- disable restrict to field
        self.fallBackMode2 = false  -- disable restrict to field border
        self.fallBackMode3 = false  -- disable avoid fruit
        self.targetWayPointId = storedTargetWayPointId
        if self.targetWayPointId ~= nil then
            self.appendWayPoints = ADGraphManager:pathFromTo(self.targetWayPointId, self.destinationId, nil, self.vehicle)
        end
    end
    self:autoRestart()
end

function PathFinderModule:autoRestart()
    PathFinderModule.debugMsg(self.vehicle, "PFM:autoRestart start")
    self.steps = 0
    self.grid = {}
    self.wayPoints = {}
    self.initNew = false
    self.path = {}
    self.diffOverallNetTime = 0
    self.startCell.visited = false
    self.startCell.out = nil
    self.currentCell = nil

    local gridKey = string.format("%d|%d|%d", self.startCell.x, self.startCell.z, self.startCell.direction)
    self.grid[gridKey] = self.startCell

    self:determineBlockedCells(self.targetCell)
    self.smoothStep = 0
    self.smoothDone = false
    self.completelyBlocked = false
    -- self.targetBlocked = false   --> always false TODO: how to use this ?
end

function PathFinderModule:abort()
    PathFinderModule.debugMsg(self.vehicle, "PFM:abort start")
    if self.strictField or self.strictFruit then
        -- no route respecting crop / field is not a stuck vehicle: no reversing out, the task plans again
        self.completelyBlocked = false
    end
    self.isFinished = true
    self.dubinsDone = true
    self.smoothDone = true
    self.wayPoints = {}
    ADScheduler:removePathfinderVehicle(self.vehicle)
end

function PathFinderModule:isBlocked()       --> true if no path in grid found -- used in: ExitFieldTask, UnloadAtDestinationTask
    return self.completelyBlocked -- or self.targetBlocked --> always false TODO: how to use this ?
end

function PathFinderModule:isTargetBlocked() --> always false TODO: how to use this ? -- used in: ExitFieldTask, UnloadAtDestinationTask, EmptyHarvesterTask
    return self.targetBlocked
end

-- return the actual and max number of iterations the pathfinder will perform by itself, could be used to show info in HUD
function PathFinderModule:getCurrentState()
    local maxStates = 1
    local actualState = 1
    if self.restrictToField then
        maxStates = maxStates + 2
    end
    if self.avoidFruitSetting then
        maxStates = maxStates + 1
    end
    if self.destinationId ~= nil then
        maxStates = maxStates + 3
    end

    if self.fallBackMode1 then
        actualState = actualState + 1
    end
    if self.fallBackMode2 then
        actualState = actualState + 1
    end
    if self.fallBackMode3 then
        actualState = actualState + 1
    end
    if self.destinationId ~= nil and self.retryCounter > 0 then
        actualState = actualState + 1
    end

    return actualState, maxStates, self.steps, self.max_pathfinder_steps
end

function PathFinderModule:timedOut()    --> not self:isBlocked() -- used in: ExitFieldTask, UnloadAtDestinationTask
    return not self:isBlocked()
end

function PathFinderModule:addDelayTimer(delayTime)      -- used in: ExitFieldTask, UnloadAtDestinationTask, CatchCombinePipeTask, EmptyHarvesterTask
    self.delayTime = delayTime
end

function PathFinderModule:update(dt)
    --stop if called without prior 'start' method calls
    if self.startCell == nil then
        self:abort()
    end
    PathFinderModule.refreshFieldMap()

    self.delayTime = math.max(0, self.delayTime - dt)
    if self.delayTime > 0 then
        return
    end

    if AutoDrive.isEditorModeEnabled() and AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
        if self.isNewPF then
            self:drawDebugNewPF()
            if self.isFinished and self.smoothDone and self.wayPoints ~= nil then
                local lastPoint = nil
                for index, point in ipairs(self.wayPoints) do
                    if point.isPathFinderPoint and lastPoint ~= nil then
                        ADDrawingManager:addLineTask(lastPoint.x, lastPoint.y, lastPoint.z, point.x, point.y, point.z, 1, 1, 0.09, 0.09)
                        ADDrawingManager:addArrowTask(lastPoint.x, lastPoint.y, lastPoint.z, point.x, point.y, point.z, 1, ADDrawingManager.arrows.position.start, 1, 0.09, 0.09)

                        if AutoDrive.getSettingState("lineHeight") == 1 then
                            local gy = point.y - AutoDrive.drawHeight + 4
                            local ty = lastPoint.y - AutoDrive.drawHeight + 4
                            ADDrawingManager:addLineTask(point.x, gy, point.z, point.x, point.y, point.z, 1, 1, 0.09, 0.09)
                            ADDrawingManager:addSphereTask(point.x, gy, point.z, 3, 1, 0.09, 0.09, 0.15)
                            ADDrawingManager:addLineTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, 1, 0.09, 0.09)
                            ADDrawingManager:addArrowTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, ADDrawingManager.arrows.position.start, 1, 0.09, 0.09)
                        else
                            local gy = point.y - AutoDrive.drawHeight - 4
                            local ty = lastPoint.y - AutoDrive.drawHeight - 4
                            ADDrawingManager:addLineTask(point.x, gy, point.z, point.x, point.y, point.z, 1, 1, 0.09, 0.09)
                            ADDrawingManager:addSphereTask(point.x, gy, point.z, 3, 1, 0.09, 0.09, 0.15)
                            ADDrawingManager:addLineTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, 1, 0.09, 0.09)
                            ADDrawingManager:addArrowTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, ADDrawingManager.arrows.position.start, 1, 0.09, 0.09)
                        end
                    end
                    lastPoint = point
                end
            end
        else
            if self.isFinished and self.smoothDone and self.wayPoints ~= nil and self.chainStartToTarget ~= nil and #self.chainStartToTarget > 0 and self.vehicle.ad.stateModule:getSpeedLimit() > 40 then
                self:drawDebugForCreatedRoute()
            else
                self:drawDebugForPF()
            end
        end
    end
    if self.isFinished then
        if self.dubinsDone then
            self.smoothDone = true
        end
        if not self.smoothDone then
            if self.isNewPF then
                self:createWayPointsNew()
            else
                self:createWayPoints()
            end
        end
        if self.smoothDone then
            ADScheduler:removePathfinderVehicle(self.vehicle)
            -- PathFinderModule.debugMsg(self.vehicle, "PFM:update Complete #self.path %s"
            --     , tostring(#self.path)
            -- )
        end
        return
    end

    self.steps = self.steps + 1
    if (self.steps % 100) == 0 then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PathFinderModule:update - self.steps %d #self.grid %d", self.steps, table.count(self.grid))
    end

    if self.completelyBlocked or self.targetBlocked or self.steps > (self.max_pathfinder_steps) then
        --[[ We need some better logic here.
        Some situations might be solved by the module itself by either
            a) 'fallBackMode (ignore fruit and field restrictions)'
            b) 'try next wayPoint'
        while others should be handled by the calling task, to properly assess the current situation
            c) 'retry same location - with or without prior pausing'
            d) 'update target location and reinvoke pathfinder if target has moved'
            e) 'try different field exit strategy'
        --]]
        -- Only allow fallback if we are not heading for a moving vehicle
        -- and never when driving to a harvester pipe: keep restrictToField and avoidFruit, the calling task retries later
        local keepRestrictions = self.goingToPipe or self.noFallback
        local fallBackModeAllowed1 = (not keepRestrictions) and (not self.strictField) and (not self.chasingVehicle) and (not self.isSecondChasingVehicle) and (self.restrictToField) and (not self.fallBackMode1)    -- disable restrict to field
        local fallBackModeAllowed2 = (not keepRestrictions) and (not self.strictField) and (not self.chasingVehicle) and (not self.isSecondChasingVehicle) and (self.restrictToField) and (not self.fallBackMode2)    -- disable restrict to field border
        local fallBackModeAllowed3 = (not keepRestrictions) and (not self.strictFruit) and (not self.chasingVehicle) and (not self.isSecondChasingVehicle) and (self.avoidFruitSetting) and (not self.fallBackMode3)    -- disable avoid fruit
        local increaseStepsAllowed = (not self.chasingVehicle) and (not self.isSecondChasingVehicle) and (self.max_pathfinder_steps < PathFinderModule.MAX_PATHFINDER_STEPS_TOTAL * self.pathFinderTime)    -- increase number of steps if possible

        -- Only allow auto restart when planning path to network and we can adjust target wayPoint
        local retryAllowed = self.destinationId ~= nil and self.retryCounter < self.PATHFINDER_MAX_RETRIES
        local smallerGridAllowed = (not self.chasingVehicle) and (not self.isSecondChasingVehicle) and self.gridScaleStep ~= nil and self.gridScaleStep < PathFinderModule.GRID_STEPS

        if not smallerGridAllowed and self.gridScaleStep ~= nil then
            PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: no path with %.1f m cells (%s)", self.minTurnRadius, self:getDiagText())
        end
        if smallerGridAllowed then
            self:shrinkGrid()
        elseif self:startRearFallback() then
            -- no clean route to the pipe: go behind the harvester, on its own harvested track
        elseif self:usePartialGridPath() then
            -- drives to the explored cell closest to the target, the task plans again on arrival
        elseif fallBackModeAllowed1 then
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PathFinderModule:update - error - retryAllowed: no -> fallBackModeAllowed1: yes -> going fallback now -> disable restrict to field #self.grid %d", table.count(self.grid))
            PathFinderModule.debugVehicleMsg(self.vehicle,
                string.format("PFM update - error - retryAllowed: no -> fallBackModeAllowed1: yes -> going fallback now -> disable restrict to field #self.grid %d",
                    table.count(self.grid)
                )
            )
            self.fallBackMode1 = true
            self:autoRestart()
        elseif fallBackModeAllowed2 and not self.isNewPF then
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PathFinderModule:update - error - retryAllowed: no -> fallBackModeAllowed2: yes -> going fallback now -> disable field borders #self.grid %d", table.count(self.grid))
            PathFinderModule.debugVehicleMsg(self.vehicle,
                string.format("PFM update - error - retryAllowed: no -> fallBackModeAllowed2: yes -> going fallback now -> disable field borders #self.grid %d",
                    table.count(self.grid)
                )
            )
            self.fallBackMode2 = true
            self:autoRestart()
        elseif fallBackModeAllowed3 then
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PathFinderModule:update - error - retryAllowed: no -> fallBackModeAllowed3: yes -> going fallback now -> disable avoid fruit #self.grid %d", table.count(self.grid))
            PathFinderModule.debugVehicleMsg(self.vehicle,
                string.format("PFM update - error - retryAllowed: no -> fallBackModeAllowed3: yes -> going fallback now -> disable avoid fruit #self.grid %d",
                    table.count(self.grid)
                )
            )
            self.fallBackMode3 = true
            self:autoRestart()
        elseif increaseStepsAllowed then
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PathFinderModule:update - error - retryAllowed: no -> increaseStepsAllowed: yes -> restart #self.grid %d self.max_pathfinder_steps %d", table.count(self.grid), self.max_pathfinder_steps)
            PathFinderModule.debugVehicleMsg(self.vehicle,
                string.format("PFM update - error - retryAllowed: no -> increaseStepsAllowed: yes -> restart -> disable avoid fruit #self.grid %d self.max_pathfinder_steps %d",
                    table.count(self.grid),
                    self.max_pathfinder_steps
                )
            )
            self.max_pathfinder_steps = self.max_pathfinder_steps + PathFinderModule.MAX_PATHFINDER_STEPS_COMBINE_TURN
            self.max_pathfinder_steps = math.min(self.max_pathfinder_steps, PathFinderModule.MAX_PATHFINDER_STEPS_TOTAL * self.pathFinderTime)
            self.fallBackMode1 = false
            self.fallBackMode2 = false
            self.fallBackMode3 = false
            self:autoRestart()
        elseif retryAllowed then
            self.retryCounter = self.retryCounter + 1
            --if we are going to the network and can't find a path. Just select the next waypoint for now
            if self.appendWayPoints ~= nil and #self.appendWayPoints > 2 then
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PathFinderModule:update - error - retryAllowed: yes -> retry now retryCounter %d", self.retryCounter)
                PathFinderModule.debugVehicleMsg(self.vehicle,
                    string.format("PFM update - error - retryAllowed: yes -> retry now retryCounter %d",
                        self.retryCounter
                    )
                )
                self:restartAtNextWayPoint()
            else
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PathFinderModule:update - error - retryAllowed: yes -> but no appendWayPoints")
                PathFinderModule.debugVehicleMsg(self.vehicle,
                    string.format("PFM update - error - retryAllowed: yes -> but no appendWayPoints"
                    )
                )
                self:abort()
            end
        else
            if AutoDrive.isEditorModeEnabled() and AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                return
            end
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PathFinderModule:update - error - retryAllowed: no -> fallBackModeAllowed: no -> aborting now")
            PathFinderModule.debugVehicleMsg(self.vehicle,
                string.format("PFM update - error - retryAllowed: no -> fallBackModeAllowed: no -> aborting now"
                )
            )
            self:abort()
        end
        return
    end

    if self.isNewPF then
        if not self.isFinished then
            if MathUtil.vector2Length(self.start.x - self.target.x, self.start.z - self.target.z) < PathFinderModule.PATHFINDER_MIN_DISTANCE_START_TARGET then
                -- try dubins first before full pathfinder if close to target
                if not self.dubinsDone then
                    self.dubinsCount = self.dubinsCount + 1
                    local dubinsPath = self:getDubinsPath()
                    PathFinderModule.debugMsg(self.vehicle, "PFM:update getDubinsPath dubinsPath %s"
                        , tostring(dubinsPath)
                    )
                    if dubinsPath then
                        self.dubinsDone = true
                        self.wayPoints = dubinsPath
                        self:appendWayPointsNew()
                        self.isFinished = true
                        return  -- found path
                    else
                        -- self.completelyBlocked = true
                          -- no valid path
                        -- PathFinderModule.debugMsg(self.vehicle, "PFM:update getDubinsPath self.completelyBlocked %s"
                        --     , tostring(self.completelyBlocked)
                        -- )
                    end
                    PathFinderModule.debugMsg(self.vehicle, "PFM:update getDubinsPath self.fallBackMode3 %s"
                        , tostring(self.fallBackMode3)
                    )
                    if self.fallBackMode3 or self.dubinsCount > 4 then
                        self.dubinsDone = true
                        -- self.completelyBlocked = false
                        -- self.fallBackMode1 = false  -- disable restrict to field
                        -- self.fallBackMode2 = false  -- disable restrict to field border
                        -- self.fallBackMode3 = false  -- disable avoid fruit
                    end
                    -- return
                end
            end
            if not self.initNew then
                self:setupNew(self.behindStartCell, self.startCell,self.targetCell)
                local dx, dz = self.nodeStart.x - self.nodeGoal.x, self.nodeStart.z - self.nodeGoal.z
                local diff = math.sqrt(dx * dx + dz * dz)
                local toCloseToTarget = (diff < 3)
                dx, dz = self.nodeBehindStart.x - self.nodeGoal.x, self.nodeBehindStart.z - self.nodeGoal.z
                diff = math.sqrt(dx * dx + dz * dz)
                toCloseToTarget = toCloseToTarget or (diff < 3)
                if (self.nodeBehindStart == self.nodeGoal) or toCloseToTarget then
                    self.completelyBlocked = true
                    return  -- no valid path
                end
            end
            local diffNetTime = netGetTime()

            local current
            local add_neighbor_fn = function(neighbor, cost)
                -- closed cells are never reopened, skip them before the expensive checks
                if not self.closedset[neighbor] and self:isDriveableAstar(neighbor) then
                    do
                        if not cost then cost = self:get_cost(current, neighbor) end
                        local tentative_g_score = self.g_score[current] + cost
                        local openset_idx = self.openset[neighbor]
                        if not openset_idx or tentative_g_score < self.g_score[neighbor] then
                            self.came_from[neighbor] = current
                            self.g_score[neighbor] = tentative_g_score
                            self.h_score[neighbor] = self.h_score[neighbor] or self:estimate_cost(neighbor, self.nodeGoal)
                            self.f_score[neighbor] = tentative_g_score + self.h_score[neighbor]
                            self.openset[neighbor] = true
                            self:pushOpenNode(neighbor, self.f_score[neighbor])
                        end
                    end
                end
            end
            local count = 0
            while next(self.openset) do
                count = count + 1
                if count > 10000 then
                    diffNetTime = netGetTime() - diffNetTime
                    self.diffOverallNetTime = self.diffOverallNetTime + diffNetTime

                    AutoDrive.debugMsg(self.vehicle, "PFM:find ERROR exit counter count %d self.diffOverallNetTime %d"
                        , count
                        , self.diffOverallNetTime
                    )

                    self.completelyBlocked = true
                    return  -- no valid path
                end
                current = self:pop_best_node(self.openset, self.f_score)
                if current == self.nodeGoal or self:reachedGoal(current, self.nodeGoal) then
                    self.came_from[self.nodeGoal] = current
                    self.path = self:unwind_path({}, self.came_from, self.nodeGoal)
                    table.insert(self.path, self.nodeGoal)
                    diffNetTime = netGetTime() - diffNetTime
                    self.diffOverallNetTime = self.diffOverallNetTime + diffNetTime
                    if current then
                        PathFinderModule.debugMsg(self.vehicle, "PFM:update find goal reached self.steps %d diffOverallNetTime %d self.nodeGoal xz %d,%d current xz %d,%d"
                            , self.steps
                            , self.diffOverallNetTime
                            , self.nodeGoal.x, self.nodeGoal.z
                            , current.x, current.z
                        )
                    end

                    self:printStrictSummary("grid")
                    self:printTrip("grid")
                    if self.gridScaleStep ~= nil and self.gridScaleStep > 1 then
                        PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: path found with %.1f m cells", self.minTurnRadius)
                    end
                    self.isFinished = true
                    return  -- found path
                end
                if current then self.closedset[current] = true end
                local from_node = self.came_from[current]
                self:get_neighbors(current, from_node, add_neighbor_fn)
                if count > (ADScheduler:getStepsPerFrame() * PathFinderModule.NEW_PF_STEP_FACTOR) then
                    diffNetTime = netGetTime() - diffNetTime
                    self.diffOverallNetTime = self.diffOverallNetTime + diffNetTime

                    PathFinderModule.debugMsg(self.vehicle, "PFM:find steps in frame count %d diffNetTime %d self.diffOverallNetTime %d"
                        , count
                        , diffNetTime
                        , self.diffOverallNetTime
                    )
                    return -- shedule
                end
            end
            diffNetTime = netGetTime() - diffNetTime
            self.diffOverallNetTime = self.diffOverallNetTime + diffNetTime

            PathFinderModule.debugMsg(self.vehicle, "PFM:find exit end count %d self.diffOverallNetTime %d"
                , count
                , self.diffOverallNetTime
            )

            self.completelyBlocked = true
            return -- no valid path
        end
    else
        --We should see some perfomance increase by localizing the sqrt/pow functions right here
        local sqrt = math.sqrt
        local distanceFunc = function(a, b)
            return sqrt(a * a + b * b)
        end

        for i = 1, ADScheduler:getStepsPerFrame(), 1 do
            if self.currentCell == nil then

                self.currentCell = self:findClosestCell(self.grid, math.huge)

                if self.currentCell ~= nil and distanceFunc(self.targetCell.x - self.currentCell.x, self.targetCell.z - self.currentCell.z) < 1.5 then

                    if self.currentCell.out == nil then
                        self:determineNextGridCells(self.currentCell)
                    end

                    if self:reachedTargetsNeighbor(self.currentCell.out) then
                        return
                    end
                end

                if self.currentCell == nil then
                    --Mark process stopped if we have no more cells to check
                    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PathFinderModule:update - Mark process stopped if we have no more cells to check")
                    PathFinderModule.debugVehicleMsg(self.vehicle,
                        string.format("PFM update - Mark process stopped if we have no more cells to check #self.grid %d",
                            table.count(self.grid)
                        )
                    )
                    self.completelyBlocked = true
                    break
                end
            else
                if self.currentCell.out == nil then
                    self:determineNextGridCells(self.currentCell)
                end
                self:testNextCells(self.currentCell)

                --Try shortcutting the process here. We dont have to go through the whole grid if one of the out points is viable and closer than the currenCell which was already closest
                local currentDistance = distanceFunc(self.targetCell.x - self.currentCell.x, self.targetCell.z - self.currentCell.z)

                local outCells = {}
                for _, outCell in pairs(self.currentCell.out) do
                    local gridKey = string.format("%d|%d|%d", outCell.x, outCell.z, outCell.direction)
                    if self.grid[gridKey] ~= nil then
                        table.insert(outCells, self.grid[gridKey])
                    end
                end
                local nextCell = self:findClosestCell(outCells, currentDistance)

                -- Lets again check if we have reached our target already
                if self:reachedTargetsNeighbor(self.currentCell.out) then
                    return
                end

                self.currentCell = nextCell
            end
        end
    end
end

function PathFinderModule:reachedTargetsNeighbor(cells)
    for _, outCell in pairs(cells) do
        if outCell.x == self.targetCell.x and outCell.z == self.targetCell.z then
            self.isFinished = true
            self.targetCell.incoming = self.currentCell --.incoming

            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "PathFinderModule:update - path found")
            PathFinderModule.debugVehicleMsg(self.vehicle,
                string.format("PFM update - path found #self.grid %d",
                    table.count(self.grid)
                )
            )

            return true
        end
    end
    return false
end

function PathFinderModule:findClosestCell(cells, startDistance)
    local cellsToCheck = cells
    local sqrt = math.sqrt
    local distanceFunc = function(a, b)
        return sqrt(a * a + b * b)
    end
    local minDistance = startDistance
    local bestCell = nil
    local bestSteps = math.huge

    for _, cell in pairs(cellsToCheck) do
        if (not cell.visited) and (not cell.hasCollision) and (not cell.isRestricted) and (cell.bordercells < PathFinderModule.MAX_FIELDBORDER_CELLS) then
            local distance = distanceFunc(self.targetCell.x - cell.x, self.targetCell.z - cell.z)

            if (distance < minDistance) or (distance == minDistance and cell.steps < bestSteps) then
                minDistance = distance
                bestCell = cell
                bestSteps = cell.steps
            end
        end
    end

    return bestCell
end

function PathFinderModule:testNextCells(cell)
    if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
        PathFinderModule.debugVehicleMsg(self.vehicle,
            string.format("PFM testNextCells start cell xz %d %d isRestricted %s hasFruit %s direction %s",
                math.floor(cell.x),
                math.floor(cell.z),
                tostring(cell.isRestricted),
                tostring(cell.hasFruit),
                tostring(self.direction_to_text[cell.direction+1])
            )
        )
    end
    for _, location in pairs(cell.out) do
        local createPoint = true
        local duplicatePointDirection = -1
        if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
            PathFinderModule.debugVehicleMsg(self.vehicle,
                string.format("PFM testNextCells location xz %d %d direction %s",
                    math.floor(location.x),
                    math.floor(location.z),
                    tostring(self.direction_to_text[location.direction+1])
                )
            )
        end
        for i = -1, self.PP_UP_LEFT, 1 do -- important: do not break this loop to check for all directions!
            local gridKey = string.format("%d|%d|%d", location.x, location.z, i)
            if self.grid[gridKey] ~= nil then
                -- cell is already in the grid
                if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                    PathFinderModule.debugVehicleMsg(self.vehicle,
                        string.format("PFM testNextCells gridKey %s cell.x,cell.z %s %s direction %s",
                            tostring(gridKey),
                            tostring(self.grid[gridKey].x),
                            tostring(self.grid[gridKey].z),
                            tostring(self.direction_to_text[self.grid[gridKey].direction+1])
                        )
                    )
                end

                if self.grid[gridKey].x == location.x and self.grid[gridKey].z == location.z then     -- out cell is already in grid

                    if self.grid[gridKey].direction == -1 then
                        createPoint = false
                    elseif self.grid[gridKey].direction == location.direction then
                        createPoint = false
                        if self.grid[gridKey].steps > (cell.steps + 1) then --found shortcut
                            self.grid[gridKey].incoming = cell
                            self.grid[gridKey].steps = cell.steps + 1
                        end
                    --elseif self.grid[gridKey].direction ~= location.direction then
                        --duplicatePointDirection = self.grid[gridKey].direction -- remember the grid direction
                        --if self.grid[gridKey].steps > (cell.steps + 1) then --found shortcut -> not true!!! The outgoing angles would be all wrong here. This caused issues with undrivable paths being generated!
                            --self.grid[gridKey].incoming = cell
                            --self.grid[gridKey].steps = cell.steps + 1
                        --end
                    end
                end
            end
        end
        if createPoint then
            local gridKey
            if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                PathFinderModule.debugVehicleMsg(self.vehicle,
                    string.format("PFM testNextCells location xz %d %d createPoint %s",
                        math.floor(location.x),
                        math.floor(location.z),
                        tostring(self.direction_to_text[location.direction+1])
                    )
                )
            end
            if duplicatePointDirection >= 0 then
                -- if different direction, it is not necessary to check the cell details again, just add a new entry in grid with known required restrictions
                -- Todo : Not true!! If we come from a different direction we ususally have a differently sized collision box to check. There is a difference between a 0° angle when coming from the last cell and a +/- 45° angle.
                gridKey = string.format("%d|%d|%d", location.x, location.z, duplicatePointDirection)
                location.isRestricted = self.grid[gridKey].isRestricted
                location.hasCollision = self.grid[gridKey].hasCollision
                location.bordercells = self.grid[gridKey].bordercells
                location.hasFruit = self.grid[gridKey].hasFruit
                location.fruitValue = self.grid[gridKey].fruitValue

                if not location.isRestricted and not location.hasCollision and location.incoming ~= nil then
                    -- check for up/down is to big or below water level
                    -- this is a required check as we come from different direction
                    local worldPos = self:gridLocationToWorldLocation(location)
                    local worldPosPrevious = self:gridLocationToWorldLocation(location.incoming)
                    location.hasCollision = location.hasCollision or self:checkSlopeAngle(worldPos.x, worldPos.z, worldPosPrevious.x, worldPosPrevious.z)    --> true if up/down is to big or below water level
                end

                if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                    PathFinderModule.debugVehicleMsg(self.vehicle,
                        string.format("PFM testNextCells different direction xz %d %d createPoint %s",
                            math.floor(location.x),
                            math.floor(location.z),
                            tostring(self.direction_to_text[location.direction+1])
                        )
                    )
                end
            else
                self:checkGridCell(location)
            end
            gridKey = string.format("%d|%d|%d", location.x, location.z, location.direction)
            self.grid[gridKey] = location
        end
    end

    cell.visited = true
end

function PathFinderModule:checkGridCell(cell)
    local worldPos = self:gridLocationToWorldLocation(cell)
    --Try going through the checks in a way that fast checks happen before slower ones which might then be skipped

    cell.isOnField = PathFinderModule.getMapOnFieldAt(worldPos.x, worldPos.z)
    if self.restrictToField and (self.fallBackMode1 and not self.fallBackMode2) then
        -- limit cells to field border only possible if started on field
        cell.bordercells = cell.incoming.bordercells + 1      -- by default we assume the new cell is not on field, so increase the counter

        if cell.incoming.bordercells == 0 then
            -- if incoming cell is on field we check if the new is also on field
            if cell.isOnField then
                -- still on field, so set the current cell counter to 0
                cell.bordercells = 0
            end
        end

        if cell.bordercells > 0 then
            if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                PathFinderModule.debugVehicleMsg(self.vehicle,
                    string.format("PFM checkGridCell - xz %d %d cell.bordercells %d #self.grid %d",
                        math.floor(cell.x),
                        math.floor(cell.z),
                        math.floor(cell.bordercells),
                        table.count(self.grid)
                    )
                )
            end
        end
    end

    -- check the most probable restrictions on field first to prevent unneccessary checks
    if not cell.isRestricted and self.restrictToField and not (self.fallBackMode1 and self.fallBackMode2) then
        -- in fallBackMode1 we ignore the field restriction
        cell.isRestricted = cell.isRestricted or (not cell.isOnField)

        if cell.isRestricted then
            if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                PathFinderModule.debugVehicleMsg(self.vehicle,
                    string.format("PFM checkGridCell isRestricted self.restrictToField %s self.fallBackMode1 %s isOnField %s x,z %d %d",
                        tostring(self.restrictToField),
                        tostring(self.fallBackMode1),
                        tostring(cell.isOnField),
                        math.floor(worldPos.x),
                        math.floor(worldPos.z)
                    )
                )
            end
        end
    end

    local gridFactor = PathFinderModule.GRID_SIZE_FACTOR * 1.3  --> 0.6
    if self.isSecondChasingVehicle then
        gridFactor = PathFinderModule.GRID_SIZE_FACTOR_SECOND_UNLOADER * 1.6    --> 1.7
    end
    local corners = self:getCorners(cell, {x = self.vectorX.x * gridFactor, z = self.vectorX.z * gridFactor}, {x = self.vectorZ.x * gridFactor, z = self.vectorZ.z * gridFactor})

    if not cell.isRestricted and self.avoidFruitSetting and not self.fallBackMode3 then
        -- check for fruit
        self:checkForFruitInArea(cell, corners) -- set cell.isRestricted if fruit found
    end

    if not cell.isRestricted and cell.incoming ~= nil then
        -- check for up/down is to big or below water level
        local worldPosPrevious = self:gridLocationToWorldLocation(cell.incoming)
        local angelToSlope, angle = self:checkSlopeAngle(worldPos.x, worldPos.z, worldPosPrevious.x, worldPosPrevious.z)    --> true if up/down or roll is to big or below water level
        cell.angle = angle
        cell.hasCollision = cell.hasCollision or angelToSlope
    end

    if not cell.isRestricted and not cell.hasCollision then
        -- check for obstacles
        local shapeDefinition = self:getShapeDefByDirectionType(cell)   --> return shape for the cell according to direction, on ground level, self.vehicleMinHeight
        local ignoreObstaclesUpToHeight = 0.2
        local shapes = overlapBox(shapeDefinition.x, shapeDefinition.y + ignoreObstaclesUpToHeight, shapeDefinition.z, 0, shapeDefinition.angleRad, 0, shapeDefinition.widthX, shapeDefinition.height - ignoreObstaclesUpToHeight, shapeDefinition.widthZ, "collisionTestCallbackIgnore", nil, self.mask, true, true, true, true)
        cell.hasCollision = cell.hasCollision or (shapes > 0) or self:boxHitsBlockedZone(shapeDefinition.x, shapeDefinition.z, shapeDefinition.angleRad, shapeDefinition.widthX, shapeDefinition.widthZ)
        if cell.hasCollision then
            if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                PathFinderModule.debugVehicleMsg(self.vehicle,
                    string.format("PFM checkGridCell hasCollision = (shapes > 0) %d x,z %d %d x,z %d %d ",
                        shapes,
                        math.floor(worldPos.x),
                        math.floor(worldPos.z),
                        math.floor(shapeDefinition.x),
                        math.floor(shapeDefinition.z)
                    )
                )
            end
        end
    end

    if not cell.isRestricted and not cell.hasCollision and cell.incoming ~= nil then
        local worldPosPrevious = self:gridLocationToWorldLocation(cell.incoming)
        local vectorX = worldPosPrevious.x - worldPos.x
        local vectorZ = worldPosPrevious.z - worldPos.z
        local dirVec = { x=vectorX, z = vectorZ}

        local cellUsedByVehiclePath = AutoDrive.checkForVehiclePathInBox(corners, self.minTurnRadius, self.vehicle, dirVec)
        cell.isRestricted = cell.isRestricted or cellUsedByVehiclePath
        self.blockedByOtherVehicle = self.blockedByOtherVehicle or cellUsedByVehiclePath
        if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
            PathFinderModule.debugVehicleMsg(self.vehicle,
                string.format("PFM checkGridCell cellUsedByVehiclePath %s self.blockedByOtherVehicle %s",
                    tostring(cellUsedByVehiclePath),
                    tostring(self.blockedByOtherVehicle)
                )
            )
        end
    end
end

function PathFinderModule:gridLocationToWorldLocation(cell)
    local result = {x = 0, z = 0}

    result.x = self.target.x + (cell.x - self.targetCell.x) * self.vectorX.x + (cell.z - self.targetCell.z) * self.vectorZ.x
    result.z = self.target.z + (cell.x - self.targetCell.x) * self.vectorX.z + (cell.z - self.targetCell.z) * self.vectorZ.z

    return result
end

function PathFinderModule:worldDirectionToGridDirection(vector, baseVector)
    local baseVector = baseVector or self.vectorX
    local angle = AutoDrive.angleBetween(baseVector, vector)

    local direction = math.floor(angle / 45)
    local remainder = angle % 45
    if remainder >= 22.5 then
        direction = (direction + 1)
    elseif remainder <= -22.5 then
        direction = (direction - 1)
    end

    if direction < 0 then
        direction = 8 + direction
    end

    return direction
end

function PathFinderModule:worldLocationToGridLocation(worldX, worldZ)
    local result = {x = 0, z = 0}

    result.z = (((worldX - self.startX) / self.vectorX.x) * self.vectorX.z - worldZ + self.startZ) / (((self.vectorZ.x / self.vectorX.x) * self.vectorX.z) - self.vectorZ.z)
    result.x = (worldZ - self.startZ - result.z * self.vectorZ.z) / self.vectorX.z

    result.x = AutoDrive.round(result.x)
    result.z = AutoDrive.round(result.z)

    return result
end

function PathFinderModule:determineBlockedCells(cell)
    if (math.abs(cell.x) < 2 and math.abs(cell.z) < 2) then
        return
    end
--[[
    table.insert(self.grid, {x = cell.x + 1, z = cell.z + 0, direction = -1, isRestricted = true, hasCollision = true, steps = 1000, bordercells = 0})   -- PP_UP
    table.insert(self.grid, {x = cell.x + 1, z = cell.z - 1, direction = -1, isRestricted = true, hasCollision = true, steps = 1000, bordercells = 0})   -- PP_UP_LEFT
    table.insert(self.grid, {x = cell.x + 0, z = cell.z + 1, direction = -1, isRestricted = true, hasCollision = true, steps = 1000, bordercells = 0})   -- PP_RIGHT
    table.insert(self.grid, {x = cell.x + 1, z = cell.z + 1, direction = -1, isRestricted = true, hasCollision = true, steps = 1000, bordercells = 0})   -- PP_UP_RIGHT
    table.insert(self.grid, {x = cell.x + 0, z = cell.z - 1, direction = -1, isRestricted = true, hasCollision = true, steps = 1000, bordercells = 0})   -- PP_LEFT
]]
    local gridKey = ""
    local direction = -1
    local x = 0
    local z = 0
    x = cell.x + 1
    z = cell.z + 0
    gridKey = string.format("%d|%d|%d", x, z, direction)
    self.grid[gridKey] = {x = x, z = z, direction = direction, isRestricted = true, hasCollision = true, steps = 100000, bordercells = 0}
    x = cell.x + 1
    z = cell.z - 1
    gridKey = string.format("%d|%d|%d", x, z, direction)
    self.grid[gridKey] = {x = x, z = z, direction = direction, isRestricted = true, hasCollision = true, steps = 100000, bordercells = 0}
    x = cell.x + 0
    z = cell.z + 1
    gridKey = string.format("%d|%d|%d", x, z, direction)
    self.grid[gridKey] = {x = x, z = z, direction = direction, isRestricted = true, hasCollision = true, steps = 100000, bordercells = 0}
    x = cell.x + 1
    z = cell.z + 1
    gridKey = string.format("%d|%d|%d", x, z, direction)
    self.grid[gridKey] = {x = x, z = z, direction = direction, isRestricted = true, hasCollision = true, steps = 100000, bordercells = 0}
    x = cell.x + 0
    z = cell.z - 1
    gridKey = string.format("%d|%d|%d", x, z, direction)
    self.grid[gridKey] = {x = x, z = z, direction = direction, isRestricted = true, hasCollision = true, steps = 100000, bordercells = 0}
end

function PathFinderModule:determineNextGridCells(cell)
    if cell.out == nil then
        cell.out = {}
    end
    if cell.direction == self.PP_UP then
        cell.out[1] = {x = cell.x + 1, z = cell.z - 1}
        cell.out[1].direction = self.PP_UP_LEFT
        cell.out[2] = {x = cell.x + 1, z = cell.z + 0}
        cell.out[2].direction = self.PP_UP
        cell.out[3] = {x = cell.x + 1, z = cell.z + 1}
        cell.out[3].direction = self.PP_UP_RIGHT
    elseif cell.direction == self.PP_UP_RIGHT then
        cell.out[1] = {x = cell.x + 1, z = cell.z + 0}
        cell.out[1].direction = self.PP_UP
        cell.out[2] = {x = cell.x + 1, z = cell.z + 1}
        cell.out[2].direction = self.PP_UP_RIGHT
        cell.out[3] = {x = cell.x + 0, z = cell.z + 1}
        cell.out[3].direction = self.PP_RIGHT
    elseif cell.direction == self.PP_RIGHT then
        cell.out[1] = {x = cell.x + 1, z = cell.z + 1}
        cell.out[1].direction = self.PP_UP_RIGHT
        cell.out[2] = {x = cell.x + 0, z = cell.z + 1}
        cell.out[2].direction = self.PP_RIGHT
        cell.out[3] = {x = cell.x - 1, z = cell.z + 1}
        cell.out[3].direction = self.PP_DOWN_RIGHT
    elseif cell.direction == self.PP_DOWN_RIGHT then
        cell.out[1] = {x = cell.x + 0, z = cell.z + 1}
        cell.out[1].direction = self.PP_RIGHT
        cell.out[2] = {x = cell.x - 1, z = cell.z + 1}
        cell.out[2].direction = self.PP_DOWN_RIGHT
        cell.out[3] = {x = cell.x - 1, z = cell.z + 0}
        cell.out[3].direction = self.PP_DOWN
    elseif cell.direction == self.PP_DOWN then
        cell.out[1] = {x = cell.x - 1, z = cell.z + 1}
        cell.out[1].direction = self.PP_DOWN_RIGHT
        cell.out[2] = {x = cell.x - 1, z = cell.z + 0}
        cell.out[2].direction = self.PP_DOWN
        cell.out[3] = {x = cell.x - 1, z = cell.z - 1}
        cell.out[3].direction = self.PP_DOWN_LEFT
    elseif cell.direction == self.PP_DOWN_LEFT then
        cell.out[1] = {x = cell.x - 1, z = cell.z - 0}
        cell.out[1].direction = self.PP_DOWN
        cell.out[2] = {x = cell.x - 1, z = cell.z - 1}
        cell.out[2].direction = self.PP_DOWN_LEFT
        cell.out[3] = {x = cell.x - 0, z = cell.z - 1}
        cell.out[3].direction = self.PP_LEFT
    elseif cell.direction == self.PP_LEFT then
        cell.out[1] = {x = cell.x - 1, z = cell.z - 1}
        cell.out[1].direction = self.PP_DOWN_LEFT
        cell.out[2] = {x = cell.x - 0, z = cell.z - 1}
        cell.out[2].direction = self.PP_LEFT
        cell.out[3] = {x = cell.x + 1, z = cell.z - 1}
        cell.out[3].direction = self.PP_UP_LEFT
    elseif cell.direction == self.PP_UP_LEFT then
        cell.out[1] = {x = cell.x - 0, z = cell.z - 1}
        cell.out[1].direction = self.PP_LEFT
        cell.out[2] = {x = cell.x + 1, z = cell.z - 1}
        cell.out[2].direction = self.PP_UP_LEFT
        cell.out[3] = {x = cell.x + 1, z = cell.z + 0}
        cell.out[3].direction = self.PP_UP
    end

    for _, outGoing in pairs(cell.out) do
        outGoing.visited = false
        outGoing.isRestricted = false
        outGoing.hasCollision = false
        outGoing.hasFruit = false
        outGoing.incoming = cell
        outGoing.steps = cell.steps + 1
        outGoing.bordercells = cell.bordercells
    end
end

function PathFinderModule:cellDistance(cell)
    return MathUtil.vector2Length(self.targetCell.x - cell.x, self.targetCell.z - cell.z)
end

PathFinderModule.CELL_MAX_RATIO = 0.10

-- crop of a grid cell, read from the field map: one share for every crop at once
function PathFinderModule:checkForFruitInArea(cell, corners)
    if self.goingToNetwork then
        -- on the way to network, check all fruit types
        self.fruitToCheck = nil
    end
    local filter = self.fruitToCheck
    if self.strictFruit then
        filter = nil
    end
    local ratio, found = PathFinderModule.getMapCropRatio(filter, corners[1].x, corners[1].z, corners[2].x, corners[2].z, corners[3].x, corners[3].z)
    if found ~= nil and (self.fruitToCheck == nil or self.fruitToCheck < 1) and not self.goingToNetwork then
        self.fruitToCheck = found
    end
    local hasFruit = ratio > PathFinderModule.CELL_MAX_RATIO
    if hasFruit and self.isNewPF then
        local center = cell.worldPos or self:gridLocationToWorldLocation(cell)
        if self:isFruitIsland(filter, center.x, center.z) then
            hasFruit = false
        end
    end
    local wasRestricted = cell.isRestricted
    cell.isRestricted = cell.isRestricted or hasFruit
    if not wasRestricted and cell.isRestricted then
        self.strictFruitRefusals = (self.strictFruitRefusals or 0) + 1
    end
    cell.hasFruit = hasFruit
    cell.fruitValue = ratio
    --Allow fruit in the last few grid cells
    if (self:cellDistance(cell) <= 3 and self.goingToPipe) then
        cell.isRestricted = false or wasRestricted
    end
    -- the vehicle already stands there: crop in the start cell (or within START_CROP_RADIUS) does not block the way out,
    -- any further the search has to follow the cut strip
    local nearStart = false
    if hasFruit and not wasRestricted and self.startX ~= nil then
        local center = cell.worldPos or self:gridLocationToWorldLocation(cell)
        nearStart = MathUtil.vector2Length(center.x - self.startX, center.z - self.startZ) < PathFinderModule.START_CROP_RADIUS
    end
    if hasFruit and not wasRestricted and (nearStart or (self.startCell ~= nil and cell.x ~= nil and cell.z ~= nil
        and cell.x == self.startCell.x and cell.z == self.startCell.z)) then
        cell.isRestricted = false
        cell.hasFruit = false
    end
end

-- previous check, density map read for each crop type (kept for reference, not used)
function PathFinderModule:checkForFruitInAreaDensity(cell, corners)

    if self.goingToNetwork then
        -- on the way to network, check all fruit types
        self.fruitToCheck = nil
    end
    if self.fruitToCheck == nil or self.strictFruit then
        -- strict: every standing crop counts, not only the one found in the combine tank
        for _, fruitType in pairs(g_fruitTypeManager:getFruitTypes()) do
            if not (fruitType == g_fruitTypeManager:getFruitTypeByName("MEADOW")) then
                local fruitTypeIndex = fruitType.index
                self:checkForFruitTypeInArea(cell, fruitTypeIndex, corners)
            end
            --stop if cell is already restricted and/or fruit type is now known
            if cell.isRestricted ~= false or (self.fruitToCheck ~= nil and not self.strictFruit) then
                break
            end
        end
    else
        self:checkForFruitTypeInArea(cell, self.fruitToCheck, corners)
    end
end

function PathFinderModule:checkForFruitTypeInArea(cell, fruitTypeIndex, corners)
    local fruitValue = 0
    fruitValue = AutoDrive.getFruitValue(fruitTypeIndex, corners[1].x, corners[1].z, corners[2].x, corners[2].z, corners[3].x, corners[3].z)

    if (self.fruitToCheck == nil or self.fruitToCheck < 1) and (fruitValue > PathFinderModule.MIN_FRUIT_VALUE) then
        self.fruitToCheck = fruitTypeIndex
    end
    if fruitValue > PathFinderModule.MIN_FRUIT_VALUE and self.isNewPF then
        -- Dubins cells carry their own world position, their x/z are not grid coordinates
        local center = cell.worldPos or self:gridLocationToWorldLocation(cell)
        if self:isFruitIsland(fruitTypeIndex, center.x, center.z) then
            fruitValue = 0
        end
    end
    local wasRestricted = cell.isRestricted
    cell.isRestricted = cell.isRestricted or (fruitValue > PathFinderModule.MIN_FRUIT_VALUE)
    if not wasRestricted and cell.isRestricted then
        self.strictFruitRefusals = (self.strictFruitRefusals or 0) + 1
    end

    cell.hasFruit = (fruitValue > PathFinderModule.MIN_FRUIT_VALUE)
    cell.fruitValue = fruitValue

    if cell.hasFruit then
        if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
            PathFinderModule.debugVehicleMsg(self.vehicle,
                string.format("PFM checkForFruitTypeInArea cell.hasFruit xz %d %d fruitValue %s direction %s",
                    math.floor(cell.x),
                    math.floor(cell.z),
                    tostring(fruitValue),
                    tostring(self.direction_to_text[cell.direction+1])
                )
            )
        end
    end

    --Allow fruit in the last few grid cells
    if (self:cellDistance(cell) <= 3 and self.goingToPipe) then
        cell.isRestricted = false or wasRestricted
    end
end

function PathFinderModule:drawDebugForPF()
    local AutoDriveDM = ADDrawingManager
    local pointTarget = self:gridLocationToWorldLocation(self.targetCell)
    local pointTargetUp = self:gridLocationToWorldLocation(self.targetCell)
    pointTarget.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pointTarget.x, 1, pointTarget.z) + 3
    pointTargetUp.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pointTargetUp.x, 1, pointTargetUp.z) + 20
    AutoDriveDM:addLineTask(pointTarget.x, pointTarget.y, pointTarget.z, pointTargetUp.x, pointTargetUp.y, pointTargetUp.z, 1, 0, 0, 1)
    local pointStart = {x = self.startX, z = self.startZ}
    pointStart.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pointStart.x, 1, pointStart.z) + 3
    AutoDriveDM:addLineTask(pointStart.x, pointStart.y, pointStart.z, pointStart.x, pointStart.y + 20, pointStart.z, 1, 0, 1, 0)

    local color_red = 0.1
    local color_green = 0.1
    local color_blue = 0.1
    local color_count = 0
    local index = 0
    for _, cell in pairs(self.grid) do
        index = index + 1

        color_red = math.min(color_red + 0.25, 1)
        if color_red > 0.9 then
            color_green = math.min(color_green + 0.25, 1)
        end
        if color_green > 0.9 then
            color_blue = math.min(color_blue + 0.25, 1)
        end
        color_count = color_count + 1

        local worldPos = self:gridLocationToWorldLocation(cell)

        local shapeDefinition = self:getShapeDefByDirectionType(cell, true)   --> return shape for the cell according to direction, on ground level, self.vehicleMinHeight
        local corners = self:getCornersFromShapeDefinition(shapeDefinition)
        local baseY = shapeDefinition.y + 3

        -- cell outline
        if cell.isOnField then
            ADDrawingManager:addLineTask(corners[1].x, baseY, corners[1].z, corners[3].x, baseY, corners[3].z, 1, 0, 1, 0) -- green
            ADDrawingManager:addLineTask(corners[3].x, baseY, corners[3].z, corners[2].x, baseY, corners[2].z, 1, 0, 1, 0)
            ADDrawingManager:addLineTask(corners[2].x, baseY, corners[2].z, corners[4].x, baseY, corners[4].z, 1, 0, 1, 0)
            ADDrawingManager:addLineTask(corners[4].x, baseY, corners[4].z, corners[1].x, baseY, corners[1].z, 1, 0, 1, 0)
        else
            ADDrawingManager:addLineTask(corners[1].x, baseY, corners[1].z, corners[3].x, baseY, corners[3].z, 1, 1, 0, 0) -- red
            ADDrawingManager:addLineTask(corners[3].x, baseY, corners[3].z, corners[2].x, baseY, corners[2].z, 1, 1, 0, 0)
            ADDrawingManager:addLineTask(corners[2].x, baseY, corners[2].z, corners[4].x, baseY, corners[4].z, 1, 1, 0, 0)
            ADDrawingManager:addLineTask(corners[4].x, baseY, corners[4].z, corners[1].x, baseY, corners[1].z, 1, 1, 0, 0)
        end
        
        if cell.isRestricted then
            if cell.hasFruit then
                ADDrawingManager:addLineTask(corners[1].x, baseY, corners[1].z, corners[2].x, baseY, corners[2].z, 1, 0, 1, 1) -- cyan
            else
                ADDrawingManager:addLineTask(corners[1].x, baseY, corners[1].z, corners[2].x, baseY, corners[2].z, 1, 1, 0, 0) -- red
            end
        else
            ADDrawingManager:addLineTask(corners[1].x, baseY, corners[1].z, corners[2].x, baseY, corners[2].z, 1, 0, 1, 0) -- green
        end
        if cell.hasCollision then
            if cell.hasVehicleCollision then
                ADDrawingManager:addLineTask(corners[3].x, baseY, corners[3].z, corners[4].x, baseY, corners[4].z, 1, 1, 0, 1) -- blue
            else
                ADDrawingManager:addLineTask(corners[3].x, baseY, corners[3].z, corners[4].x, baseY, corners[4].z, 1, 1, 1, 0) -- yellow
            end
        end
         
    end

    -- target cell marker
    local size = 0.3
    local pointA = self:gridLocationToWorldLocation(self.targetCell)
    pointA.x = pointA.x + self.vectorX.x * size + self.vectorZ.x * size
    pointA.z = pointA.z + self.vectorX.z * size + self.vectorZ.z * size
    pointA.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pointA.x, 1, pointA.z) + 3
    local pointB = self:gridLocationToWorldLocation(self.targetCell)
    pointB.x = pointB.x - self.vectorX.x * size - self.vectorZ.x * size
    pointB.z = pointB.z - self.vectorX.z * size - self.vectorZ.z * size
    pointB.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pointB.x, 1, pointB.z) + 3
    local pointC = self:gridLocationToWorldLocation(self.targetCell)
    pointC.x = pointC.x + self.vectorX.x * size - self.vectorZ.x * size
    pointC.z = pointC.z + self.vectorX.z * size - self.vectorZ.z * size
    pointC.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pointC.x, 1, pointC.z) + 3
    local pointD = self:gridLocationToWorldLocation(self.targetCell)
    pointD.x = pointD.x - self.vectorX.x * size + self.vectorZ.x * size
    pointD.z = pointD.z - self.vectorX.z * size + self.vectorZ.z * size
    pointD.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pointD.x, 1, pointD.z) + 3

    AutoDriveDM:addLineTask(pointA.x, pointA.y, pointA.z, pointB.x, pointB.y, pointB.z, 1, 1, 1, 1) -- white
    AutoDriveDM:addLineTask(pointC.x, pointC.y, pointC.z, pointD.x, pointD.y, pointD.z, 1, 1, 1, 1) -- white

    local pointAB = self:gridLocationToWorldLocation(self.targetCell)
    pointAB.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pointAB.x, 1, pointAB.z) + 3

    local pointTargetVector = self:gridLocationToWorldLocation(self.targetCell)
    pointTargetVector.x = pointTargetVector.x + self.targetVector.x * 10
    pointTargetVector.z = pointTargetVector.z + self.targetVector.z * 10
    pointTargetVector.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, pointTargetVector.x, 1, pointTargetVector.z) + 3
    AutoDriveDM:addLineTask(pointAB.x, pointAB.y, pointAB.z, pointTargetVector.x, pointTargetVector.y, pointTargetVector.z, 1, 1, 1, 1) -- white
end

function PathFinderModule:drawDebugForCreatedRoute()
    local AutoDriveDM = ADDrawingManager
    if self.chainStartToTarget ~= nil then
        for _, cell in pairs(self.chainStartToTarget) do
            local shape = self:getShapeDefByDirectionType(cell)
            if shape.x ~= nil then
                local pointA = {
                    x = shape.x + shape.widthX * math.cos(shape.angleRad) + shape.widthZ * math.sin(shape.angleRad),
                    y = shape.y,
                    z = shape.z + shape.widthZ * math.cos(shape.angleRad) + shape.widthX * math.sin(shape.angleRad)
                }
                local pointB = {
                    x = shape.x - shape.widthX * math.cos(shape.angleRad) - shape.widthZ * math.sin(shape.angleRad),
                    y = shape.y,
                    z = shape.z + shape.widthZ * math.cos(shape.angleRad) + shape.widthX * math.sin(shape.angleRad)
                }
                local pointC = {
                    x = shape.x - shape.widthX * math.cos(shape.angleRad) - shape.widthZ * math.sin(shape.angleRad),
                    y = shape.y,
                    z = shape.z - shape.widthZ * math.cos(shape.angleRad) - shape.widthX * math.sin(shape.angleRad)
                }
                local pointD = {
                    x = shape.x + shape.widthX * math.cos(shape.angleRad) + shape.widthZ * math.sin(shape.angleRad),
                    y = shape.y,
                    z = shape.z - shape.widthZ * math.cos(shape.angleRad) - shape.widthX * math.sin(shape.angleRad)
                }

                AutoDriveDM:addLineTask(pointA.x, pointA.y, pointA.z, pointC.x, pointC.y, pointC.z, 1, 1, 1, 1)
                AutoDriveDM:addLineTask(pointB.x, pointB.y, pointB.z, pointD.x, pointD.y, pointD.z, 1, 1, 1, 1)

                if cell.incoming ~= nil then
                    local worldPos_cell = self:gridLocationToWorldLocation(cell)
                    local worldPos_incoming = self:gridLocationToWorldLocation(cell.incoming)

                    local vectorX = worldPos_cell.x - worldPos_incoming.x
                    local vectorZ = worldPos_cell.z - worldPos_incoming.z
                    local angleRad = math.atan2(-vectorZ, vectorX)
                    angleRad = AutoDrive.normalizeAngle(angleRad)
                    local widthOfColBox = math.sqrt(math.pow(self.minTurnRadius, 2) + math.pow(self.minTurnRadius, 2))
                    local sideLength = widthOfColBox / 2

                    local leftAngle = AutoDrive.normalizeAngle(angleRad + math.rad(-90))
                    local rightAngle = AutoDrive.normalizeAngle(angleRad + math.rad(90))

                    local cornerX = worldPos_incoming.x - math.cos(leftAngle) * sideLength
                    local cornerZ = worldPos_incoming.z + math.sin(leftAngle) * sideLength

                    local corner2X = worldPos_cell.x - math.cos(leftAngle) * sideLength
                    local corner2Z = worldPos_cell.z + math.sin(leftAngle) * sideLength

                    local corner3X = worldPos_cell.x - math.cos(rightAngle) * sideLength
                    local corner3Z = worldPos_cell.z + math.sin(rightAngle) * sideLength

                    local corner4X = worldPos_incoming.x - math.cos(rightAngle) * sideLength
                    local corner4Z = worldPos_incoming.z + math.sin(rightAngle) * sideLength

                    local inY = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, worldPos_incoming.x, 1, worldPos_incoming.z) + 1
                    local currentY = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, worldPos_cell.x, 1, worldPos_cell.z) + 1

                    AutoDriveDM:addLineTask(cornerX, inY, cornerZ, corner2X, currentY, corner2Z, 1, 1, 0, 0)
                    AutoDriveDM:addLineTask(corner2X, currentY, corner2Z, corner3X, currentY, corner3Z, 1, 1, 0, 0)
                    AutoDriveDM:addLineTask(corner3X, currentY, corner3Z, corner4X, inY, corner4Z, 1, 1, 0, 0)
                    AutoDriveDM:addLineTask(corner4X, inY, corner4Z, cornerX, inY, cornerZ, 1, 1, 0, 0)
                end
            end
        end
    end

    if self.wayPoints then
        for i, waypoint in pairs(self.wayPoints) do
            Utils.renderTextAtWorldPosition(waypoint.x, waypoint.y + 4, waypoint.z, "Node " .. i, getCorrectTextSize(0.013), 0)
            if i > 1 then
                local wp = waypoint
                local pfWp = self.wayPoints[i - 1]
                AutoDriveDM:addLineTask(wp.x, wp.y, wp.z, pfWp.x, pfWp.y, pfWp.z, 1, 0, 1, 1)
            end
        end
    end
end

function PathFinderModule:getShapeDefByDirectionType_New(cell)
    local shapeDefinition = {}
    shapeDefinition.angleRad = math.atan2(-self.targetVector.z, self.targetVector.x)
    shapeDefinition.angleRad = AutoDrive.normalizeAngle(shapeDefinition.angleRad)
    local worldPos = self:gridLocationToWorldLocation(cell)
    shapeDefinition.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, worldPos.x, 1, worldPos.z)
    shapeDefinition.height = self.vehicleMinHeight

    shapeDefinition.x = worldPos.x
    shapeDefinition.z = worldPos.z
    -- a small cell is narrower than the vehicle: the obstacle check covers at least the train width
    local halfSize = math.max(self.minTurnRadius / 2, (self.trainHalfWidth or 0) + 0.5)
    if self:isOffFieldReduced(worldPos.x, worldPos.z, cell.isOnField) then
        halfSize = math.max(halfSize, (self.trainHalfWidth or 0) + 0.5 + PathFinderModule.OFF_FIELD_EXTRA_MARGIN)
    end
    shapeDefinition.widthX = halfSize
    shapeDefinition.widthZ = halfSize

    local corners = self:getCornersFromShapeDefinition(shapeDefinition)
    if corners ~= nil then
        for _, corner in pairs(corners) do
            shapeDefinition.y = math.max(shapeDefinition.y, getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, corner.x, 1, corner.z))
        end
    end

    return shapeDefinition
end


function PathFinderModule:getShapeDefByDirectionType(cell, getDefault)
    local shapeDefinition = {}
    shapeDefinition.angleRad = math.atan2(-self.targetVector.z, self.targetVector.x)
    shapeDefinition.angleRad = AutoDrive.normalizeAngle(shapeDefinition.angleRad)
    local worldPos = self:gridLocationToWorldLocation(cell)
    shapeDefinition.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, worldPos.x, 1, worldPos.z)
    shapeDefinition.height = self.vehicleMinHeight

    if cell.direction == self.PP_UP or cell.direction == self.PP_DOWN or cell.direction == self.PP_RIGHT or cell.direction == self.PP_LEFT or cell.direction == -1 or getDefault ~= nil then
        --default size:
        shapeDefinition.x = worldPos.x
        shapeDefinition.z = worldPos.z
        shapeDefinition.widthX = self.minTurnRadius / 2
        shapeDefinition.widthZ = self.minTurnRadius / 2
    elseif cell.direction == self.PP_UP_RIGHT then
        local offsetX = (-self.vectorX.x) / 2 + (-self.vectorZ.x) / 4
        local offsetZ = (-self.vectorX.z) / 2 + (-self.vectorZ.z) / 4
        shapeDefinition.x = worldPos.x + offsetX
        shapeDefinition.z = worldPos.z + offsetZ
        shapeDefinition.widthX = (self.minTurnRadius / 2) + math.abs(offsetX)
        shapeDefinition.widthZ = self.minTurnRadius / 2 + math.abs(offsetZ)
    elseif cell.direction == self.PP_UP_LEFT then
        local offsetX = (-self.vectorX.x) / 2 + (self.vectorZ.x) / 4
        local offsetZ = (-self.vectorX.z) / 2 + (self.vectorZ.z) / 4
        shapeDefinition.x = worldPos.x + offsetX
        shapeDefinition.z = worldPos.z + offsetZ
        shapeDefinition.widthX = self.minTurnRadius / 2 + math.abs(offsetX)
        shapeDefinition.widthZ = self.minTurnRadius / 2 + math.abs(offsetZ)
    elseif cell.direction == self.PP_DOWN_RIGHT then
        local offsetX = (self.vectorX.x) / 2 + (-self.vectorZ.x) / 4
        local offsetZ = (self.vectorX.z) / 2 + (-self.vectorZ.z) / 4
        shapeDefinition.x = worldPos.x + offsetX
        shapeDefinition.z = worldPos.z + offsetZ
        shapeDefinition.widthX = self.minTurnRadius / 2 + math.abs(offsetX)
        shapeDefinition.widthZ = self.minTurnRadius / 2 + math.abs(offsetZ)
    elseif cell.direction == self.PP_DOWN_LEFT then
        local offsetX = (self.vectorX.x) / 2 + (self.vectorZ.x) / 4
        local offsetZ = (self.vectorX.z) / 2 + (self.vectorZ.z) / 4
        shapeDefinition.x = worldPos.x + offsetX
        shapeDefinition.z = worldPos.z + offsetZ
        shapeDefinition.widthX = self.minTurnRadius / 2 + math.abs(offsetX)
        shapeDefinition.widthZ = self.minTurnRadius / 2 + math.abs(offsetZ)
    end

    local increaseCellFactor = 1.15
    if cell.isOnField ~= nil and cell.isOnField == true then
        increaseCellFactor = 1 --0.8
    end
    shapeDefinition.widthX = shapeDefinition.widthX * increaseCellFactor
    shapeDefinition.widthZ = shapeDefinition.widthZ * increaseCellFactor

    local corners = self:getCornersFromShapeDefinition(shapeDefinition)
    if corners ~= nil then
        for _, corner in pairs(corners) do
            shapeDefinition.y = math.max(shapeDefinition.y, getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, corner.x, 1, corner.z))
        end
    end

    return shapeDefinition
end

function PathFinderModule:getCornersFromShapeDefinition(shapeDefinition)
    local corners = {}
    corners[1] = {x = shapeDefinition.x + (-shapeDefinition.widthX), z = shapeDefinition.z + (-shapeDefinition.widthZ)}
    corners[2] = {x = shapeDefinition.x + (shapeDefinition.widthX), z = shapeDefinition.z + (shapeDefinition.widthZ)}
    corners[3] = {x = shapeDefinition.x + (-shapeDefinition.widthX), z = shapeDefinition.z + (shapeDefinition.widthZ)}
    corners[4] = {x = shapeDefinition.x + (shapeDefinition.widthX), z = shapeDefinition.z + (-shapeDefinition.widthZ)}

    return corners
end

function PathFinderModule:getCorners(cell, vectorX, vectorZ)
    local corners = {}
    local centerLocation = self:gridLocationToWorldLocation(cell)
    corners[1] = {x = centerLocation.x + (-vectorX.x - vectorZ.x), z = centerLocation.z + (-vectorX.z - vectorZ.z)}
    corners[2] = {x = centerLocation.x + (vectorX.x - vectorZ.x), z = centerLocation.z + (vectorX.z - vectorZ.z)}
    corners[3] = {x = centerLocation.x + (-vectorX.x + vectorZ.x), z = centerLocation.z + (-vectorX.z + vectorZ.z)}
    corners[4] = {x = centerLocation.x + (vectorX.x + vectorZ.x), z = centerLocation.z + (vectorX.z + vectorZ.z)}

    return corners
end

function PathFinderModule:createWayPoints()
    if self.smoothStep == 0 then
        local currentCell = self.targetCell
        self.chainTargetToStart = {}
        local index = 1
        self.chainTargetToStart[index] = currentCell
        index = index + 1
        while currentCell.x ~= 0 or currentCell.z ~= 0 do
            self.chainTargetToStart[index] = currentCell.incoming
            currentCell = currentCell.incoming
            if currentCell == nil then
                break
            end
            index = index + 1
        end
        index = index - 1

        self.chainStartToTarget = {}
        for reversedIndex = 0, index, 1 do
            self.chainStartToTarget[reversedIndex + 1] = self.chainTargetToStart[index - reversedIndex]
        end

        --Now build actual world coordinates as waypoints and include pre and append points
        self.wayPoints = {}
        for chainIndex, cell in pairs(self.chainStartToTarget) do
            self.wayPoints[chainIndex] = self:gridLocationToWorldLocation(cell)
            self.wayPoints[chainIndex].y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, self.wayPoints[chainIndex].x, 1, self.wayPoints[chainIndex].z)
            self.wayPoints[chainIndex].direction = cell.direction
        end

        -- remove zig zag line
        self:smoothResultingPPPath()
    end

    -- shortcut the path if possible
    self:smoothResultingPPPath_Refined()

    if self.smoothStep == 2 then
        -- When going to network, dont turn actual road network nodes into pathFinderPoints
        if self.goingToNetwork then
            for i = 1, #self.wayPoints, 1 do
                self.wayPoints[i].isPathFinderPoint = true
            end
        end

        if self.appendWayPoints ~= nil then
            for i = 1, #self.appendWayPoints, 1 do
                self.wayPoints[#self.wayPoints + 1] = self.appendWayPoints[i]
            end
            self.smoothStep = 3
            --PathFinderModule.debugVehicleMsg(self.vehicle,
                --string.format("PFM createWayPoints appendWayPoints %s",
                    --tostring(#self.appendWayPoints)
                --)
            --)
        end

        -- See comment above
        if not self.goingToNetwork then
            for i = 1, #self.wayPoints, 1 do
                self.wayPoints[i].isPathFinderPoint = true
            end
        end
    end
end

function PathFinderModule:smoothResultingPPPath()
    local index = 1
    local filteredIndex = 1
    local filteredWPs = {}

    while index < #self.wayPoints - 1 do
        local node = self.wayPoints[index]
        local nodeAhead = self.wayPoints[index + 1]
        local nodeTwoAhead = self.wayPoints[index + 2]

        filteredWPs[filteredIndex] = node
        filteredIndex = filteredIndex + 1

        if node.direction ~= nil and nodeAhead.direction ~= nil and nodeTwoAhead.direction ~= nil then
            if node.direction == nodeTwoAhead.direction and node.direction ~= nodeAhead.direction then
                index = index + 1 --skip next point because it is a zig zag line. Cut right through instead
            end
        end

        index = index + 1
    end

    while index <= #self.wayPoints do
        local node = self.wayPoints[index]
        filteredWPs[filteredIndex] = node
        filteredIndex = filteredIndex + 1
        index = index + 1
    end

    self.wayPoints = filteredWPs
    --PathFinderModule.debugVehicleMsg(self.vehicle,
        --string.format("PFM smoothResultingPPPath self.wayPoints %s",
            --tostring(#self.wayPoints)
        --)
    --)

end

function PathFinderModule:smoothResultingPPPath_Refined()
    if self.smoothStep == 0 then
        self.lookAheadIndex = 1
        self.smoothIndex = 1
        self.filteredIndex = 1
        self.filteredWPs = {}
        self.totalEagerSteps = 0

        --add first few without filtering
        while self.smoothIndex < #self.wayPoints and self.smoothIndex < 3 do
            self.filteredWPs[self.filteredIndex] = self.wayPoints[self.smoothIndex]
            self.filteredIndex = self.filteredIndex + 1
            self.smoothIndex = self.smoothIndex + 1
        end

        self.smoothStep = 1
    end

    local unfilteredEndPointCount = 5
    if self.smoothStep == 1 then
        local stepsThisFrame = 0
        while self.smoothIndex < #self.wayPoints - unfilteredEndPointCount and stepsThisFrame < ADScheduler:getStepsPerFrame() do

            if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                PathFinderModule.debugVehicleMsg(self.vehicle,
                    string.format("PFM smoothResultingPPPath_Refined self.smoothIndex %d ",
                        self.smoothIndex
                    )
                )
            end
            stepsThisFrame = stepsThisFrame + 1

            local node = self.wayPoints[self.smoothIndex]
            local previousNode = nil
            local worldPos = self.wayPoints[self.smoothIndex]

            if self.totalEagerSteps == nil or self.totalEagerSteps == 0 then
                if self.filteredWPs[self.filteredIndex-1].x ~= node.x and self.filteredWPs[self.filteredIndex-1].z ~= node.z then
                    self.filteredWPs[self.filteredIndex] = node
                    if self.filteredIndex > 1 then
                        previousNode = self.filteredWPs[self.filteredIndex - 1]
                    end
                    self.filteredIndex = self.filteredIndex + 1

                    self.lookAheadIndex = 1
                    self.totalEagerSteps = 0
                end
            end

            local widthOfColBox = self.minTurnRadius
            local sideLength = widthOfColBox * PathFinderModule.GRID_SIZE_FACTOR
            local foundCollision = false

            if stepsThisFrame > math.max(1, (ADScheduler:getStepsPerFrame() * 0.4)) then
                break
            end

            --local stepsOfLookAheadThisFrame = 0
            -- (foundCollision == false or self.totalEagerSteps < PathFinderModule.PP_MAX_EAGER_LOOKAHEAD_STEPS)
            while (foundCollision == false) and ((self.smoothIndex + self.totalEagerSteps) < (#self.wayPoints - unfilteredEndPointCount)) and stepsThisFrame <= math.max(1, (ADScheduler:getStepsPerFrame() * 0.4)) do

                if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                    PathFinderModule.debugVehicleMsg(self.vehicle,
                        string.format("PFM smoothResultingPPPath_Refined self.smoothIndex %d self.totalEagerSteps %d",
                            self.smoothIndex,
                            self.totalEagerSteps
                        )
                    )
                end

                local hasCollision = false
                stepsThisFrame = stepsThisFrame + 1
                local nodeAhead = self.wayPoints[self.smoothIndex + self.totalEagerSteps + 1]
                local nodeTwoAhead = self.wayPoints[self.smoothIndex + self.totalEagerSteps + 2]
                if not hasCollision and nodeAhead and nodeTwoAhead then
                    local angle = AutoDrive.angleBetween({x = nodeAhead.x - node.x, z = nodeAhead.z - node.z}, {x = nodeTwoAhead.x - nodeAhead.x, z = nodeTwoAhead.z - nodeAhead.z})
                    angle = math.abs(angle)
                    if angle > 60 then
                        hasCollision = true

                        if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                            PathFinderModule.debugVehicleMsg(self.vehicle,
                                string.format("PFM smoothResultingPPPath_Refined hasCollision %d",
                                    1
                                )
                            )
                        end
                    end
                    if previousNode ~= nil then
                        angle = AutoDrive.angleBetween({x = node.x - previousNode.x, z = node.z - previousNode.z}, {x = nodeTwoAhead.x - node.x, z = nodeTwoAhead.z - node.z})
                        angle = math.abs(angle)
                        if angle > 60 then
                            hasCollision = true

                            if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                                PathFinderModule.debugVehicleMsg(self.vehicle,
                                    string.format("PFM smoothResultingPPPath_Refined hasCollision %d",
                                        2
                                    )
                                )
                            end
                        end
                        angle = AutoDrive.angleBetween({x = node.x - previousNode.x, z = node.z - previousNode.z}, {x = nodeAhead.x - node.x, z = nodeAhead.z - node.z})
                        angle = math.abs(angle)
                        if angle > 60 then
                            hasCollision = true

                            if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                                PathFinderModule.debugVehicleMsg(self.vehicle,
                                    string.format("PFM smoothResultingPPPath_Refined hasCollision %d",
                                        3
                                    )
                                )
                            end
                        end
                    end
                end

                if not hasCollision then
                    hasCollision = hasCollision or self:checkSlopeAngle(worldPos.x, worldPos.z, nodeAhead.x, nodeAhead.z)
                    if hasCollision then

                        if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                            PathFinderModule.debugVehicleMsg(self.vehicle,
                                string.format("PFM smoothResultingPPPath_Refined hasCollision %d",
                                    4
                                )
                            )
                        end
                    end
                end

                local vectorX = nodeAhead.x - node.x
                local vectorZ = nodeAhead.z - node.z
                local angleRad = math.atan2(-vectorZ, vectorX)
                angleRad = AutoDrive.normalizeAngle(angleRad)
                local length = math.sqrt(math.pow(vectorX, 2) + math.pow(vectorZ, 2)) + widthOfColBox

                local leftAngle = AutoDrive.normalizeAngle(angleRad + math.rad(-90))
                local rightAngle = AutoDrive.normalizeAngle(angleRad + math.rad(90))

                local cornerX = node.x - math.cos(leftAngle) * sideLength
                local cornerZ = node.z + math.sin(leftAngle) * sideLength

                local corner2X = nodeAhead.x - math.cos(leftAngle) * sideLength
                local corner2Z = nodeAhead.z + math.sin(leftAngle) * sideLength

                local corner3X = nodeAhead.x - math.cos(rightAngle) * sideLength
                local corner3Z = nodeAhead.z + math.sin(rightAngle) * sideLength

                local corner4X = node.x - math.cos(rightAngle) * sideLength
                local corner4Z = node.z + math.sin(rightAngle) * sideLength
                -- fruit check of the shortcut: same corridor as the grid search (train width + margin), not the whole cell,
                -- or a strip along standing crop is never straightened and the route zigzags
                local fruitSide = self.trainHalfWidth or sideLength
                -- shortcuts keep a margin from the crop: the vehicle drives in the free strip, not along the crop edge
                if self.trainHalfWidth ~= nil then
                    fruitSide = self.trainHalfWidth + PathFinderModule.SMOOTH_FRUIT_MARGIN
                end
                local fruitCornerX = node.x - math.cos(leftAngle) * fruitSide
                local fruitCornerZ = node.z + math.sin(leftAngle) * fruitSide
                local fruitCorner2X = nodeAhead.x - math.cos(leftAngle) * fruitSide
                local fruitCorner2Z = nodeAhead.z + math.sin(leftAngle) * fruitSide
                local fruitCorner4X = node.x - math.cos(rightAngle) * fruitSide
                local fruitCorner4Z = node.z + math.sin(rightAngle) * fruitSide
                local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, nodeAhead.x, 1, nodeAhead.z)

                if not hasCollision then
                    if self.isNewPF then
                        self.collisionhits = 0
                        local shapes = overlapBox(worldPos.x + vectorX / 2, y + self.vehicleMinHeight / 2, worldPos.z + vectorZ / 2, 0, angleRad, 0, length / 2 + 2.5, self.vehicleMinHeight / 2, sideLength + 1.5, "collisionTestCallback", self, self.mask, true, true, true, true)
                        hasCollision = hasCollision or (self.collisionhits > 0)
                    else
                        local shapes = overlapBox(worldPos.x + vectorX / 2, y + self.vehicleMinHeight / 2, worldPos.z + vectorZ / 2, 0, angleRad, 0, length / 2 + 2.5, self.vehicleMinHeight / 2, sideLength + 1.5, "Ignore", nil, self.mask, true, true, true, true)
                        hasCollision = hasCollision or (shapes > 0)
                    end
                    hasCollision = hasCollision or self:boxHitsBlockedZone(worldPos.x + vectorX / 2, worldPos.z + vectorZ / 2, angleRad, length / 2 + 2.5, sideLength + 1.5)

                    if hasCollision then
                        if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                            PathFinderModule.debugVehicleMsg(self.vehicle,
                                string.format("PFM smoothResultingPPPath_Refined hasCollision %d",
                                    5
                                )
                            )
                        end
                    end
                end

                if (self.smoothIndex > 1) then
                    local worldPosPrevious = self.wayPoints[self.smoothIndex - 1]
                    length = MathUtil.vector3Length(worldPos.x - worldPosPrevious.x, worldPos.y - worldPosPrevious.y, worldPos.z - worldPosPrevious.z)
                    local angleBetween = math.atan(math.abs(worldPos.y - worldPosPrevious.y) / length)

                    if (angleBetween) > PathFinderModule.SLOPE_DETECTION_THRESHOLD then
                        hasCollision = true

                        if hasCollision then
                            if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                                PathFinderModule.debugVehicleMsg(self.vehicle,
                                    string.format("PFM smoothResultingPPPath_Refined hasCollision %d",
                                        6
                                    )
                                )
                            end
                        end
                    end
                end

                if not hasCollision and self.avoidFruitSetting and not self.fallBackMode3 then

                    local cornerWideX = node.x - math.cos(leftAngle) * sideLength * 4
                    local cornerWideZ = node.z + math.sin(leftAngle) * sideLength * 4

                    local cornerWide2X = nodeAhead.x - math.cos(leftAngle) * sideLength * 4
                    local cornerWide2Z = nodeAhead.z + math.sin(leftAngle) * sideLength * 4

                    local cornerWide4X = node.x - math.cos(rightAngle) * sideLength * 4
                    local cornerWide4Z = node.z + math.sin(rightAngle) * sideLength * 4

                    local ratio
                    if self.isSecondChasingVehicle then
                        ratio = PathFinderModule.getMapCropRatio(self:getCropFilter(), cornerWideX, cornerWideZ, cornerWide2X, cornerWide2Z, cornerWide4X, cornerWide4Z)
                    else
                        ratio = PathFinderModule.getMapCropRatio(self:getCropFilter(), fruitCornerX, fruitCornerZ, fruitCorner2X, fruitCorner2Z, fruitCorner4X, fruitCorner4Z)
                    end
                    if self:getCropFilter() ~= nil or self.goingToNetwork or self.strictFruit then
                        hasCollision = hasCollision or ratio > PathFinderModule.FRUIT_MAX_RATIO
                    end
                end

                -- same crop rule as the final check (slices of FRUIT_SLICE_LENGTH), with the margin: a shortcut the check would refuse is not made
                if not hasCollision and self.avoidFruitSetting and not self.fallBackMode3 and self.trainHalfWidth ~= nil and not self.isSecondChasingVehicle then
                    hasCollision = not self:isFruitStretchClear(node, nodeAhead, fruitSide)
                end

                -- restrict to field: the shortcut must stay on the field too, checked on both sides of the train
                if not hasCollision and self.restrictToField and not (self.fallBackMode1 or self.fallBackMode2) then
                    local halfWidth = (self.trainHalfWidth or 3) - PathFinderModule.FRUIT_CHECK_CORRIDOR_MARGIN
                    local sideX, sideZ = math.cos(leftAngle) * halfWidth, -math.sin(leftAngle) * halfWidth
                    for _, t in ipairs({0, 0.5, 1}) do
                        local px, pz = node.x + vectorX * t, node.z + vectorZ * t
                        if not self:isTrainSideAllowed(px + sideX, pz + sideZ) or not self:isTrainSideAllowed(px - sideX, pz - sideZ) then
                            hasCollision = true
                            break
                        end
                    end

                    if hasCollision then
                        if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                            PathFinderModule.debugVehicleMsg(self.vehicle,
                                string.format("PFM smoothResultingPPPath_Refined hasCollision %d",
                                    10
                                )
                            )
                        end
                    end
                end

                if not hasCollision then
                    local cellBox = AutoDrive.boundingBoxFromCorners(cornerX, cornerZ, corner2X, corner2Z, corner3X, corner3Z, corner4X, corner4Z)
                    hasCollision = hasCollision or AutoDrive.checkForVehiclePathInBox(cellBox, self.minTurnRadius, self.vehicle)

                    if hasCollision then
                        if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                            PathFinderModule.debugVehicleMsg(self.vehicle,
                                string.format("PFM smoothResultingPPPath_Refined hasCollision %d",
                                    9
                                )
                            )
                        end
                    end
                end

                foundCollision = hasCollision

                if foundCollision then
                    -- not used code removed
                else
                    self.lookAheadIndex = self.totalEagerSteps + 1
                end

                self.totalEagerSteps = self.totalEagerSteps + 1

                if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                    PathFinderModule.debugVehicleMsg(self.vehicle,
                        string.format("PFM smoothResultingPPPath_Refined self.smoothIndex %d self.totalEagerSteps %d self.filteredIndex %d foundCollision %s",
                            self.smoothIndex,
                            self.totalEagerSteps,
                            self.filteredIndex,
                            tostring(foundCollision)
                        )
                    )
                end
            end

            if foundCollision or ((self.smoothIndex + self.totalEagerSteps) >= (#self.wayPoints - unfilteredEndPointCount)) or self.totalEagerSteps > PathFinderModule.PP_MAX_EAGER_LOOKAHEAD_STEPS then
                self.smoothIndex = self.smoothIndex + math.max(1, (self.lookAheadIndex))
                self.totalEagerSteps = 0
            end
        end

        if self.smoothIndex >= #self.wayPoints - unfilteredEndPointCount then
            self.smoothStep = 2
        end
    end

    if self.smoothStep == 2 then
        --add remaining points without filtering
        while self.smoothIndex <= #self.wayPoints do
            local node = self.wayPoints[self.smoothIndex]
            self.filteredWPs[self.filteredIndex] = node
            self.filteredIndex = self.filteredIndex + 1
            self.smoothIndex = self.smoothIndex + 1
        end

        self.wayPoints = self.filteredWPs
        -- the cells can be wider than a free strip: a path found on the grid may still cross crop or an obstacle
        if self.gridScaleStep ~= nil and not self.chasingVehicle and not self.isSecondChasingVehicle then
            local badIndex, reason = self:findBlockedStretch()
            if badIndex ~= nil then
                local p = self.wayPoints[badIndex]
                if self.gridScaleStep < PathFinderModule.GRID_STEPS then
                    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: path in %.1f m cells refused (%s at %.0f,%.0f), finer search", self.minTurnRadius, tostring(reason), p.x, p.z)
                    self.isFinished = false
                    self:shrinkGrid()
                    return
                end
                if (self.strictFruit or self.strictField) and self:usePartialPath(badIndex) then
                    -- the clean beginning of the route is kept, the task plans again on arrival
                elseif self.strictFruit or self.strictField then
                    -- strict: better wait and plan again than drive through the crop, the calling task retries
                    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: no clean path (%s at %.0f,%.0f), strict: waiting and new try", tostring(reason), p.x, p.z)
                    self:abort()
                    self.smoothStep = 3
                    return
                else
                    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: path in %.1f m cells kept despite %s at %.0f,%.0f (no better one)", self.minTurnRadius, tostring(reason), p.x, p.z)
                end
            end
            if self.partialPath and not self:trimPartialEnd() then
                PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: partial path without a clean stop point (facing the harvester), waiting and new try")
                self:abort()
                self.smoothStep = 3
                return
            end
        end
        if self.gridScaleStep ~= nil and self.gridScaleStep > 1 then
            local kept, sharp = self:roundCornersToTurnRadius()
            kept, sharp = kept or 0, sharp or 0
            -- a corner too sharp for the train cannot be driven: finer grid, and on the finest one no route (strict)
            if sharp > 0 and self.gridScaleStep < PathFinderModule.GRID_STEPS then
                PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: %d corner(s) too sharp in %.1f m cells, finer search", sharp, self.minTurnRadius)
                self.isFinished = false
                self:shrinkGrid()
                return
            elseif sharp > 0 and (self.strictFruit or self.strictField) then
                PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: %d corner(s) too sharp for the train even in %.1f m cells, strict: waiting and new try", sharp, self.minTurnRadius)
                self:abort()
                self.smoothStep = 3
                return
            end
            -- zigzag: the points of a coarse grid fall badly in the free strip, a finer grid follows it
            if kept > PathFinderModule.ZIGZAG_MAX_KEPT and self.gridScaleStep < PathFinderModule.GRID_STEPS then
                PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: zigzag in %.1f m cells (%d corners), finer search", self.minTurnRadius, kept)
                self.isFinished = false
                self:shrinkGrid()
                return
            end
        end

        self.smoothDone = true

        PathFinderModule.debugVehicleMsg(self.vehicle,
            string.format("PFM smoothResultingPPPath_Refined self.wayPoints %s",
                tostring(#self.wayPoints)
            )
        )
    end
end

function PathFinderModule:checkSlopeAngle(x1, z1, x2, z2)
    local vectorFromPrevious = {x = x1 - x2, z = z1 - z2}
    local worldPosMiddle = {x = x2 + vectorFromPrevious.x / 2, z = z2 + vectorFromPrevious.z / 2}

    local terrain1 = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x1, 0, z1)
    local terrain2 = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x2, 0, z2)
    local terrain3 = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, worldPosMiddle.x, 0, worldPosMiddle.z)
    local length = MathUtil.vector3Length(x1 - x2, terrain1 - terrain2, z1 - z2)
    local lengthMiddle = MathUtil.vector3Length(worldPosMiddle.x - x2, terrain3 - terrain2, worldPosMiddle.z - z2)
    local angleBetween = math.atan(math.abs(terrain1 - terrain2) / length)
    local angleBetweenCenter = math.atan(math.abs(terrain3 - terrain2) / lengthMiddle)

    local angleLeft = 0
    local angleRight = 0

    if self.cos90 == nil then
        -- speed up the calculation
        self.cos90 = math.cos(math.rad(90))
        self.sin90 = math.sin(math.rad(90))
        self.cos270 = math.cos(math.rad(270))
        self.sin270 = math.sin(math.rad(270))
    end

    local rotX = vectorFromPrevious.x * self.cos90 - vectorFromPrevious.z * self.sin90
    local rotZ = vectorFromPrevious.x * self.sin90 + vectorFromPrevious.z * self.cos90
    local vectorLeft = {x = rotX, z = rotZ}

    local rotX = vectorFromPrevious.x * self.cos270 - vectorFromPrevious.z * self.sin270
    local rotZ = vectorFromPrevious.x * self.sin270 + vectorFromPrevious.z * self.cos270
    local vectorRight = {x = rotX, z = rotZ}

    local worldPosLeft = {x = x1 + vectorLeft.x / 2, z = z1 + vectorLeft.z / 2}
    local worldPosRight = {x = x1 + vectorRight.x / 2, z = z1 + vectorRight.z / 2}
    local terrainLeft = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, worldPosLeft.x, 0, worldPosLeft.z)
    local terrainRight = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, worldPosRight.x, 0, worldPosRight.z)
    local lengthLeft = MathUtil.vector3Length(worldPosLeft.x - x1, terrainLeft - terrain1, worldPosLeft.z - z1)
    local lengthRight = MathUtil.vector3Length(worldPosRight.x - x1, terrainRight - terrain1, worldPosRight.z - z1)
    angleLeft = math.atan(math.abs(terrainLeft - terrain1) / lengthLeft)
    angleRight = math.atan(math.abs(terrainRight - terrain1) / lengthRight)

    local waterY1 = g_currentMission.environmentAreaSystem:getWaterYAtWorldPosition(x1, terrain1, z1) or -200
    local waterY2 = g_currentMission.environmentAreaSystem:getWaterYAtWorldPosition(x2, terrain2, z2) or -200
    local waterY3 = g_currentMission.environmentAreaSystem:getWaterYAtWorldPosition(worldPosMiddle.x, terrain3, worldPosMiddle.z) or -200

    local belowGroundLevel = terrain1 < waterY1 - 0.5 or terrain2 < waterY2 - 0.5 or terrain3 < waterY3 - 0.5

    if belowGroundLevel then
        if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
            PathFinderModule.debugVehicleMsg(self.vehicle,
                string.format("PFM checkSlopeAngle belowGroundLevel x,z %d %d",
                    math.floor(x1),
                    math.floor(z1)
                )
            )
        end
        PathFinderModule.debugMsg(self.vehicle, "PFM:checkSlopeAngle belowGroundLevel xz %d,%d terrain123 %.1f %.1f %.1f waterY1 %s  waterY2 %s waterY3 %s"
        , math.floor(x1)
        , math.floor(z1)
        , terrain1
        , terrain2
        , terrain3
        , tostring(waterY1)
        , tostring(waterY2)
        , tostring(waterY3)
        )
    end

    if (angleBetween) > PathFinderModule.SLOPE_DETECTION_THRESHOLD then
        if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
            PathFinderModule.debugVehicleMsg(self.vehicle,
                string.format("PFM checkSlopeAngle (angleBetween * 1.25) > PathFinderModule.SLOPE_DETECTION_THRESHOLD  x,z %d %d",
                    math.floor(x1),
                    math.floor(z1)
                )
            )
        end
        PathFinderModule.debugMsg(self.vehicle, "PFM:checkSlopeAngle angleBetween xz %d,%d angleBetween %.1f terrain12 %.1f %.1f length %.1f "
        , math.floor(x1)
        , math.floor(z1)
        , math.deg(angleBetween)
        , terrain1
        , terrain2
        , length
        )
    end

    if (angleBetweenCenter) > PathFinderModule.SLOPE_DETECTION_THRESHOLD then
        if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
            PathFinderModule.debugVehicleMsg(self.vehicle,
                string.format("PFM checkSlopeAngle (angleBetweenCenter * 1.25) > PathFinderModule.SLOPE_DETECTION_THRESHOLD  x,z %d %d",
                    math.floor(x1),
                    math.floor(z1)
                )
            )
        end
        PathFinderModule.debugMsg(self.vehicle, "PFM:checkSlopeAngle angleBetweenCenter xz %d,%d angleBetweenCenter %.1f terrain32 %.1f %.1f lengthMiddle %.1f "
        , math.floor(x1)
        , math.floor(z1)
        , math.deg(angleBetweenCenter)
        , terrain3
        , terrain2
        , lengthMiddle
        )
    end

    if (angleLeft) > PathFinderModule.SLOPE_DETECTION_THRESHOLD then
        PathFinderModule.debugMsg(self.vehicle, "PFM:checkSlopeAngle angleLeft xz %d,%d angleLeft %.1f terrainLeft %.1f terrain1 %.1f lengthLeft %.1f "
        , math.floor(x1)
        , math.floor(z1)
        , math.deg(angleLeft)
        , terrainLeft
        , terrain1
        , lengthLeft
        )
    end

    if (angleRight) > PathFinderModule.SLOPE_DETECTION_THRESHOLD then
        PathFinderModule.debugMsg(self.vehicle, "PFM:checkSlopeAngle angleRight xz %d,%d angleRight %.1f terrainRight %.1f terrain1 %.1f lengthRight %.1f "
        , math.floor(x1)
        , math.floor(z1)
        , math.deg(angleRight)
        , terrainRight
        , terrain1
        , lengthRight
        )
    end

    if belowGroundLevel or (angleBetween) > PathFinderModule.SLOPE_DETECTION_THRESHOLD or (angleBetweenCenter) > PathFinderModule.SLOPE_DETECTION_THRESHOLD 
    or (angleLeft > PathFinderModule.SLOPE_DETECTION_THRESHOLD or angleRight > PathFinderModule.SLOPE_DETECTION_THRESHOLD)
    then
        return true, angleBetween
    end
    return false, angleBetween
end

function PathFinderModule.debugVehicleMsg(vehicle, msg)
    -- collect output for single vehicle - help to examine sequences for a single vehicle
    if vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.debug ~= nil then
        if AutoDrive.debugVehicleMsg ~= nil then
            AutoDrive.debugVehicleMsg(vehicle, msg)
        end
    end
end

function PathFinderModule:drawDebugNewPF()
    -- AStar
    if self.cachedNodes then
        for z, row in pairs(self.cachedNodes) do
            for x, node in pairs(row) do
                local shapeDefinition = node.shapeDefinition
                if shapeDefinition then
                    DebugUtil.drawOverlapBox(shapeDefinition.x, shapeDefinition.y + shapeDefinition.height / 2, shapeDefinition.z, 0, shapeDefinition.angleRad, 0, shapeDefinition.widthX, shapeDefinition.height / 2, shapeDefinition.widthZ, 1, 1, 1)
                end
                -- cell outline
                local gridFactor = PathFinderModule.GRID_SIZE_FACTOR
                if self.isSecondChasingVehicle then
                    gridFactor = PathFinderModule.GRID_SIZE_FACTOR_SECOND_UNLOADER
                end
                local corners = self:getCorners(node, {x = self.vectorX.x * gridFactor, z = self.vectorX.z * gridFactor}, {x = self.vectorZ.x * gridFactor, z = self.vectorZ.z * gridFactor})
                local tempY = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, corners[1].x, 1, corners[1].z)
                if node.isOnField then
                    ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[2].x, tempY, corners[2].z, 1, 0, 1, 0) -- green
                    ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[4].x, tempY, corners[4].z, 1, 0, 1, 0)
                    ADDrawingManager:addLineTask(corners[3].x, tempY, corners[3].z, corners[4].x, tempY, corners[4].z, 1, 0, 1, 0)
                    ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[3].x, tempY, corners[3].z, 1, 0, 1, 0)
                else
                    ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[2].x, tempY, corners[2].z, 1, 1, 0, 0) -- red
                    ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[4].x, tempY, corners[4].z, 1, 1, 0, 0)
                    ADDrawingManager:addLineTask(corners[3].x, tempY, corners[3].z, corners[4].x, tempY, corners[4].z, 1, 1, 0, 0)
                    ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[3].x, tempY, corners[3].z, 1, 1, 0, 0)
                end
                if node.isRestricted then
                    if node.hasFruit then
                        ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[3].x, tempY, corners[3].z, 1, 0, 1, 1) -- cyan
                    else
                        ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[3].x, tempY, corners[3].z, 1, 1, 0, 0) -- red
                    end
                else
                    ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[3].x, tempY, corners[3].z, 1, 0, 1, 0) -- green
                end
                if node.hasCollision then
                    if node.hasVehicleCollision then
                        ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[4].x, tempY, corners[4].z, 1, 1, 0, 1) -- blue
                    else
                        ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[4].x, tempY, corners[4].z, 1, 1, 1, 0) -- yellow
                    end
                end

                -- cell text
                if node.text == nil then
                    node.text = string.format("%d,%d", x, z)
                end
                local point = self:gridLocationToWorldLocation({x = node.x, z = node.z})
                Utils.renderTextAtWorldPosition(point.x, tempY + 3, point.z, node.text, getCorrectTextSize(0.013), 0)

                -- behind point
                if node.isBehind then
                    local text = string.format("B %d,%d", x, z)
                    Utils.renderTextAtWorldPosition(point.x, tempY + 4, point.z, text, getCorrectTextSize(0.013), 0)
                    ADDrawingManager:addSphereTask(self.behind.x, tempY + 3, self.behind.z, 6, 0, 0, 1, 0) -- blue
                end

                -- start point
                if node.isStart then
                    local text = string.format("S %d,%d", x, z)
                    Utils.renderTextAtWorldPosition(point.x, tempY + 5, point.z, text, getCorrectTextSize(0.013), 0)
                    ADDrawingManager:addSphereTask(self.startX, tempY + 3, self.startZ, 6, 0, 1, 0, 0) -- green
                end

                -- goal point            
                if node.isGoal then
                    local text = string.format("T %d,%d", x, z)
                    Utils.renderTextAtWorldPosition(point.x, tempY + 6, point.z, text, getCorrectTextSize(0.013), 0)
                    ADDrawingManager:addSphereTask(self.target.x, tempY + 3, self.target.z, 6, 1, 0, 0, 0) -- red
                end
            end
        end
    end

    -- Dubins Path
    if self.dubinsPath and #self.dubinsPath > 0 then

        local lastPoint = nil
        for index, point in ipairs(self.dubinsPath) do
            if lastPoint ~= nil then
                ADDrawingManager:addLineTask(lastPoint.x, lastPoint.y, lastPoint.z, point.x, point.y, point.z, 1, 1, 0.09, 0.09)
                ADDrawingManager:addArrowTask(lastPoint.x, lastPoint.y, lastPoint.z, point.x, point.y, point.z, 1, ADDrawingManager.arrows.position.start, 1, 0.09, 0.09)

                if AutoDrive.getSettingState("lineHeight") == 1 then
                    local gy = point.y - AutoDrive.drawHeight + 4
                    local ty = lastPoint.y - AutoDrive.drawHeight + 4
                    ADDrawingManager:addLineTask(point.x, gy, point.z, point.x, point.y, point.z, 1, 1, 0.09, 0.09)
                    ADDrawingManager:addSphereTask(point.x, gy, point.z, 3, 1, 0.09, 0.09, 0.15)
                    ADDrawingManager:addLineTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, 1, 0.09, 0.09)
                    ADDrawingManager:addArrowTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, ADDrawingManager.arrows.position.start, 1, 0.09, 0.09)
                else
                    local gy = point.y - AutoDrive.drawHeight - 4
                    local ty = lastPoint.y - AutoDrive.drawHeight - 4
                    ADDrawingManager:addLineTask(point.x, gy, point.z, point.x, point.y, point.z, 1, 1, 0.09, 0.09)
                    ADDrawingManager:addSphereTask(point.x, gy, point.z, 3, 1, 0.09, 0.09, 0.15)
                    ADDrawingManager:addLineTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, 1, 0.09, 0.09)
                    ADDrawingManager:addArrowTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, ADDrawingManager.arrows.position.start, 1, 0.09, 0.09)
                end
            end
            lastPoint = point
        end
    end
    if self.dubinsNodes then
        local i = 0
        for z, row in pairs(self.dubinsNodes) do
            for x, node in pairs(row) do
                local corners = node.corners
                i = i + 1
                local sizeMax = self.vehicle.size.width / 2
                local cell = node
                if cell.height then
                    DebugUtil.drawOverlapBox(cell.worldPos.x, cell.worldPos.y + cell.height / 2, cell.worldPos.z, 0, cell.t, 0, sizeMax, cell.height / 2, sizeMax, 0, 0, 1)
                end

                -- local text = string.format("%d",i)
                -- Utils.renderTextAtWorldPosition(x, node.worldPos.y + 3, z, text, getCorrectTextSize(0.013), 0)
                local tempY = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, node.corners[1].x, 1, node.corners[1].z)
                if node.isOnField then
                    ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[2].x, tempY, corners[2].z, 1, 0, 1, 0) -- green
                    ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[4].x, tempY, corners[4].z, 1, 0, 1, 0)
                    ADDrawingManager:addLineTask(corners[3].x, tempY, corners[3].z, corners[4].x, tempY, corners[4].z, 1, 0, 1, 0)
                    ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[3].x, tempY, corners[3].z, 1, 0, 1, 0)
                else
                    ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[2].x, tempY, corners[2].z, 1, 1, 0, 0) -- red
                    ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[4].x, tempY, corners[4].z, 1, 1, 0, 0)
                    ADDrawingManager:addLineTask(corners[3].x, tempY, corners[3].z, corners[4].x, tempY, corners[4].z, 1, 1, 0, 0)
                    ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[3].x, tempY, corners[3].z, 1, 1, 0, 0)
                end
                if node.isRestricted then
                    if node.hasFruit then
                        ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[3].x, tempY, corners[3].z, 1, 0, 1, 1) -- cyan
                    else
                        ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[3].x, tempY, corners[3].z, 1, 1, 0, 0) -- red
                    end
                else
                    ADDrawingManager:addLineTask(corners[2].x, tempY, corners[2].z, corners[3].x, tempY, corners[3].z, 1, 0, 1, 0) -- green
                end
                if node.hasCollision then
                    if node.hasVehicleCollision then
                        ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[4].x, tempY, corners[4].z, 1, 1, 0, 1) -- blue
                    else
                        ADDrawingManager:addLineTask(corners[1].x, tempY, corners[1].z, corners[4].x, tempY, corners[4].z, 1, 1, 1, 0) -- yellow
                    end
                end
                if node.fruitValue and node.fruitValue > 0 then
                    local text = string.format("%d",node.fruitValue)
                    Utils.renderTextAtWorldPosition(x, node.worldPos.y + 3, z, text, getCorrectTextSize(0.013), 0)
                end
            end
        end
    end
end

function PathFinderModule:isDriveableAstar(cell)
    cell.isRestricted = false
    cell.incoming = cell.from_node
    cell.hasCollision = false
    cell.shapeDefinition = self:getShapeDefByDirectionType_New(cell)   --> return shape for the cell according to direction, on ground level, self.vehicleMinHeight

    local worldPos = self:gridLocationToWorldLocation(cell)
    --Try going through the checks in a way that fast checks happen before slower ones which might then be skipped

    cell.isOnField = PathFinderModule.getMapOnFieldAt(worldPos.x, worldPos.z)

    -- check the most probable restrictions on field first to prevent unneccessary checks
    if not cell.isRestricted and self.restrictToField and not (self.fallBackMode1 or self.fallBackMode2) then
        -- in fallBackMode1 we ignore the field restriction
        local fieldAllowed = self:isFieldPositionAllowed(worldPos.x, worldPos.z, cell.isOnField)
        cell.isRestricted = cell.isRestricted or (not fieldAllowed)
        if not fieldAllowed then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableAstar not cell.isOnField xz %d,%d "
                , cell.x, cell.z
            )
        end
    end

    local gridFactor = PathFinderModule.GRID_SIZE_FACTOR
    if self.isSecondChasingVehicle then
        gridFactor = PathFinderModule.GRID_SIZE_FACTOR_SECOND_UNLOADER
    end
    local corners = self:getCorners(cell, {x = self.vectorX.x * gridFactor, z = self.vectorX.z * gridFactor}, {x = self.vectorZ.x * gridFactor, z = self.vectorZ.z * gridFactor})

    if not cell.isRestricted and self.avoidFruitSetting and not self.fallBackMode3 then
        -- check for fruit
        local fruitCorners = corners
        if PathFinderModule.FRUIT_CHECK_TRAIN_CORRIDOR and cell.pathFrom ~= nil and not self.isSecondChasingVehicle then
            fruitCorners = self:getCorridorCorners(cell.pathFrom, cell) or corners
        end
        self:checkForFruitInArea(cell, fruitCorners) -- set cell.isRestricted if fruit found
        table.insert(self.fruitAreas, fruitCorners)
        if cell.isRestricted then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableAstar cell.isRestricted xz %d,%d fruit found %s"
                , cell.x, cell.z
                , self.fruitToCheck
            )
        end
    end

    if not cell.isRestricted and cell.incoming ~= nil then
        -- check for up/down is to big or below water level
        local worldPosPrevious = self:gridLocationToWorldLocation(cell.incoming)
        local angelToSlope, angle = self:checkSlopeAngle(worldPos.x, worldPos.z, worldPosPrevious.x, worldPosPrevious.z)    --> true if up/down or roll is to big or below water level
        cell.angle = angle
        cell.hasCollision = cell.hasCollision or angelToSlope
        cell.isRestricted = cell.isRestricted or cell.hasCollision
        if angelToSlope then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableAstar angelToSlope xz %d,%d"
                , cell.x, cell.z
            )
        end
    end

    if not cell.isRestricted then
        -- check for obstacles
        self.collisionhits = 0
        local shapes = overlapBox(cell.shapeDefinition.x, cell.shapeDefinition.y + cell.shapeDefinition.height / 2, cell.shapeDefinition.z, 0, cell.shapeDefinition.angleRad, 0, cell.shapeDefinition.widthX, cell.shapeDefinition.height / 2, cell.shapeDefinition.widthZ, "collisionTestCallback", self, self.mask, true, true, true, true)
        cell.hasCollision = cell.hasCollision or (self.collisionhits > 0)
            or self:boxHitsBlockedZone(cell.shapeDefinition.x, cell.shapeDefinition.z, cell.shapeDefinition.angleRad, cell.shapeDefinition.widthX, cell.shapeDefinition.widthZ)
        cell.isRestricted = cell.isRestricted or cell.hasCollision
        if cell.hasCollision then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableAstar cell.hasCollision xz %d,%d collision"
                , cell.x, cell.z
            )
        end
    end

    if not cell.isRestricted and cell.incoming ~= nil then
        local worldPosPrevious = self:gridLocationToWorldLocation(cell.incoming)
        local vectorX = worldPosPrevious.x - worldPos.x
        local vectorZ = worldPosPrevious.z - worldPos.z
        local dirVec = { x=vectorX, z = vectorZ}

        local cellUsedByVehiclePath = AutoDrive.checkForVehiclePathInBox(corners, self.minTurnRadius, self.vehicle, dirVec)
        cell.isRestricted = cell.isRestricted or cellUsedByVehiclePath
        self.blockedByOtherVehicle = self.blockedByOtherVehicle or cellUsedByVehiclePath
        if cellUsedByVehiclePath then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableAstar cellUsedByVehiclePath xz %d,%d vehicle"
                , cell.x, cell.z
            )
        end
    end
    if cell.isRestricted then
        local diag = self.diag or {}
        self.diag = diag
        local reason = "vehicle"
        if cell.isOnField ~= nil and self.restrictToField and not (self.fallBackMode1 or self.fallBackMode2) and not self:isFieldPositionAllowed(worldPos.x, worldPos.z, cell.isOnField) then
            reason = "field"
        elseif cell.hasFruit then
            reason = "crop"
        elseif cell.hasCollision then
            reason = "obstacle/slope"
        end
        diag[reason] = (diag[reason] or 0) + 1
        PathFinderModule.addSearchCell(worldPos.x, worldPos.z, reason)
    else
        self.diagFree = (self.diagFree or 0) + 1
        PathFinderModule.addSearchCell(worldPos.x, worldPos.z, "free")
    end
    return not(cell.isRestricted)
end

-- one line: why the cells of the last search were refused
function PathFinderModule:getDiagText()
    local diag = self.diag or {}
    local best = math.huge
    if self.closedset ~= nil and self.nodeGoal ~= nil then
        for node, _ in pairs(self.closedset) do
            if node.x ~= nil and node.z ~= nil then
                best = math.min(best, MathUtil.vector2Length(node.x - self.nodeGoal.x, node.z - self.nodeGoal.z))
            end
        end
    end
    local targetCrop = -1
    if self.target ~= nil then
        targetCrop = PathFinderModule.getMapCropRatio(nil, self.target.x - 5, self.target.z - 5, self.target.x + 5, self.target.z - 5, self.target.x - 5, self.target.z + 5)
    end
    local text = string.format("free %d, refused: field %d, crop %d, obstacle/slope %d, vehicle %d, field map %d cells, step %d/%d, target %.0f,%.0f crop around %.0f %%, closest cell %.0f cells away, blocked %s",
        self.diagFree or 0, diag["field"] or 0, diag["crop"] or 0, diag["obstacle/slope"] or 0, diag["vehicle"] or 0, PathFinderModule.fieldMapCount,
        self.steps or 0, self.max_pathfinder_steps or 0, self.target ~= nil and self.target.x or 0, self.target ~= nil and self.target.z or 0, targetCrop * 100, best, tostring(self.completelyBlocked))
    self.diag = {}
    self.diagFree = 0
    return text
end

 -- Cost of two adjacent nodes
-- current, neighbor
function PathFinderModule:get_cost(from_node, to_node)
    local dx, dz = from_node.x - to_node.x, from_node.z - to_node.z
    return math.sqrt(dx * dx + dz * dz) + (from_node.cost + to_node.cost) * 0.5
end

-- For heuristic. Estimate cost of current node to goal node
-- neighbor, goal
function PathFinderModule:estimate_cost(node, goal_node)
    return self:get_cost(node, goal_node) * 1.5 + (node.cost + goal_node.cost) * 0.5
end

-- current = self:pop_best_node(self.openset, self.f_score)
-- return: node / nil
-- self.openset, f_score
-- the open set is mirrored in a binary heap (self.openHeap) ordered by f score,
-- entries whose score changed or which were already taken are skipped
function PathFinderModule:pushOpenNode(node, f)
    local heap = self.openHeap
    local i = #heap + 1
    heap[i] = {f = f, node = node}
    while i > 1 do
        local parent = math.floor(i / 2)
        if heap[parent].f <= heap[i].f then
            break
        end
        heap[parent], heap[i] = heap[i], heap[parent]
        i = parent
    end
end

function PathFinderModule:popOpenEntry()
    local heap = self.openHeap
    local count = #heap
    if count == 0 then
        return nil
    end
    local top = heap[1]
    heap[1] = heap[count]
    heap[count] = nil
    count = count - 1
    local i = 1
    while true do
        local left, right, smallest = i * 2, i * 2 + 1, i
        if left <= count and heap[left].f < heap[smallest].f then
            smallest = left
        end
        if right <= count and heap[right].f < heap[smallest].f then
            smallest = right
        end
        if smallest == i then
            break
        end
        heap[smallest], heap[i] = heap[i], heap[smallest]
        i = smallest
    end
    return top
end

function PathFinderModule:pop_best_node(set, score)
    local entry = self:popOpenEntry()
    while entry ~= nil do
        local node = entry.node
        if set[node] and score[node] == entry.f then
            set[node] = nil
            return node
        end
        entry = self:popOpenEntry()
    end
end

-- {}, self.came_from, self.nodeGoal
function PathFinderModule:unwind_path(flat_path, came_from, goal)
    if came_from[goal] and (came_from[goal] ~= self.nodeGoal) then
		table.insert(flat_path, 1, came_from[goal])
		return self:unwind_path(flat_path, came_from, came_from[goal])
	else
        return flat_path
	end
end

-- Node must be able to check if they are the same
-- so the example cannot directly return a different table for same coord
function PathFinderModule:get_node(x, z)
    local row = self.cachedNodes[z]
    if not row then row = {}; self.cachedNodes[z] = row end
    local node = row[x]
    if not node then node = { x = x, z = z, cost = 0 }; row[x] = node end
    return node
end

function PathFinderModule:getDirections(fromNode, node)
    if node == nil then
        AutoDrive.debugMsg(self.vehicle, "PFM:getDirections ERROR fromNode %s node %s"
            , tostring(fromNode)
            , tostring(node)
        )
        return
    end
    local directions = {}

--[[     if fromNode then
        PathFinderModule.debugMsg(self.vehicle, "PFM:getDirections fromNode %d,%d fromNode.direction %s node xz %d,%d node.direction %s"
            , fromNode.x, fromNode.z
            , tostring(self.direction_to_text[fromNode.direction+1])
            , node.x, node.z
            , tostring(self.direction_to_text[node.direction+1])
        )
    else
        PathFinderModule.debugMsg(self.vehicle, "PFM:getDirections fromNode %s node xz %d,%d node.direction %s"
            , tostring(fromNode)
            , node.x, node.z
            , tostring(self.direction_to_text[node.direction+1])
        )
    end
 ]]
    if (fromNode == nil and node.direction == self.PP_RIGHT) or (fromNode and fromNode.x == node.x and fromNode.z < node.z) then
        directions[1] = { -1, 1 }
        directions[1].direction = self.PP_DOWN_RIGHT
        directions[2] = { 0, 1 }
        directions[2].direction = self.PP_RIGHT
        directions[3] = { 1, 1 }
        directions[3].direction = self.PP_UP_RIGHT
    elseif (fromNode == nil and node.direction == self.PP_LEFT) or (fromNode and fromNode.x == node.x and fromNode.z > node.z) then
        directions[1] = { -1, -1 }
        directions[1].direction = self.PP_DOWN_LEFT
        directions[2] = { 0, -1 }
        directions[2].direction = self.PP_LEFT
        directions[3] = { 1, -1 }
        directions[3].direction = self.PP_UP_LEFT
    elseif (fromNode == nil and node.direction == self.PP_UP) or (fromNode and fromNode.x < node.x and fromNode.z == node.z) then
        directions[1] = { 1, -1 }
        directions[1].direction = self.PP_UP_LEFT
        directions[2] = { 1, 0 }
        directions[2].direction = self.PP_UP
        directions[3] = { 1, 1 }
        directions[3].direction = self.PP_UP_RIGHT
    elseif (fromNode == nil and node.direction == self.PP_DOWN) or (fromNode and fromNode.x > node.x and fromNode.z == node.z) then
        directions[1] = { -1, -1 }
        directions[1].direction = self.PP_DOWN_LEFT
        directions[2] = { -1, 0 }
        directions[2].direction = self.PP_DOWN
        directions[3] = { -1, 1 }
        directions[3].direction = self.PP_DOWN_RIGHT
    elseif (fromNode == nil and node.direction == self.PP_UP_RIGHT) or (fromNode and fromNode.x < node.x and fromNode.z < node.z) then
        directions[1] = { 1, 0 }
        directions[1].direction = self.PP_UP
        directions[2] = { 1, 1 }
        directions[2].direction = self.PP_UP_RIGHT
        directions[3] = { 0, 1 }
        directions[3].direction = self.PP_RIGHT
    elseif (fromNode == nil and node.direction == self.PP_DOWN_LEFT) or (fromNode and fromNode.x > node.x and fromNode.z > node.z) then
        directions[1] = { 0, -1 }
        directions[1].direction = self.PP_LEFT
        directions[2] = { -1, -1 }
        directions[2].direction = self.PP_DOWN_LEFT
        directions[3] = { -1, 0 }
        directions[3].direction = self.PP_DOWN
    elseif (fromNode == nil and node.direction == self.PP_UP_LEFT) or (fromNode and fromNode.x < node.x and fromNode.z > node.z) then
        directions[1] = { 0, -1 }
        directions[1].direction = self.PP_LEFT
        directions[2] = { 1, -1 }
        directions[2].direction = self.PP_UP_LEFT
        directions[3] = { 1, 0 }
        directions[3].direction = self.PP_UP
    elseif (fromNode == nil and node.direction == self.PP_DOWN_RIGHT) or (fromNode and fromNode.x > node.x and fromNode.z < node.z) then
        directions[1] = { -1, 0 }
        directions[1].direction = self.PP_DOWN
        directions[2] = { -1, 1 }
        directions[2].direction = self.PP_DOWN_RIGHT
        directions[3] = { 0, 1 }
        directions[3].direction = self.PP_RIGHT
    else
        if fromNode then
            AutoDrive.debugMsg(self.vehicle, "PFM:getDirections ERROR fromNode xz %d,%d"
                , fromNode.x
                , fromNode.z
            )
        end
        AutoDrive.debugMsg(self.vehicle, "PFM:getDirections ERROR fromNode %s node xz %d,%d node.direction %s"
            , tostring(fromNode)
            , node.x, node.z
            , tostring(node.direction)
        )
    end
--[[     
    PathFinderModule.debugMsg(self.vehicle, "PFM:getDirections %d,%d direction %s %d,%d direction %s %d,%d direction %s"
        , directions[1][1], directions[1][2]
        , tostring(self.direction_to_text[directions[1].direction+1])
        , directions[2][1], directions[2][2]
        , tostring(self.direction_to_text[directions[2].direction+1])
        , directions[3][1], directions[3][2]
        , tostring(self.direction_to_text[directions[3].direction+1])
    )
 ]]
    return directions
end

-- Return all neighbor nodes. Means a target that can be moved from the current node
-- current, from_node, add_neighbor_fn
function PathFinderModule:get_neighbors(node, fromNode, add_neighbor_fn)
    local x, z = node.x, node.z
    local directions = self:getDirections(fromNode, node)
    if directions then
        for i, offset in ipairs(directions) do
            local tnode = self:get_node(x + offset[1], z + offset[2])
            tnode.direction = offset.direction
            tnode.from_node = fromNode
            tnode.pathFrom = node
            add_neighbor_fn(tnode)
        end
    end
end

local all_neighbors_offset = {
    { -1, -1 }, { 0, -1 }, { 1, -1 },
    { -1, 0 },             { 1, 0 },
    { -1, 1 },  { 0, 1 },  { 1, 1 }
}

function PathFinderModule:setBlockedGoal()
    local x, z = self.targetAheadCell.x, self.targetAheadCell.z
    for i, offset in ipairs(all_neighbors_offset) do
        if not (self.targetCell.x == (x + offset[1]) and self.targetCell.z == (z + offset[2])) then
            local tnode = self:get_node(x + offset[1], z + offset[2])
            tnode.text = string.format("G %d,%d",x + offset[1], z + offset[2])
            tnode.isBlockedGoal = true
            self.closedset[tnode] = true
        end
    end
end

function PathFinderModule:reachedGoal(current, goal)
    if math.abs(current.x - goal.x) < 2 and math.abs(current.z - goal.z) < 2 then
        return true
    else
        return false
    end
end

function PathFinderModule:setupNew(behindStartCell, startCell, targetCell, userdata)
    PathFinderModule.debugMsg(self.vehicle, "PFM:setupNew behindStartCell %s,%s startCell %s,%s targetCell %s,%s"
        , tostring(behindStartCell.x)
        , tostring(behindStartCell.z)
        , tostring(startCell.x)
        , tostring(startCell.z)
        , tostring(targetCell.x)
        , tostring(targetCell.z)
    )
    self.cachedNodes = {}
    self.openset = {}
    self.openHeap = {}
    self.closedset = {}
    self.came_from = {}
    self.g_score = {}
    self.h_score = {}
    self.f_score = {}

    self.nodeBehindStart = self:get_node(behindStartCell.x, behindStartCell.z)
    self.nodeBehindStart.isBehind = true
    self.nodeBehindStart.direction = behindStartCell.direction

    self.nodeStart = self:get_node(startCell.x, startCell.z)
    self.nodeStart.isStart = true
    self.nodeStart.direction = startCell.direction

    self.nodeGoal = self:get_node(targetCell.x, targetCell.z)
    self.nodeGoal.isGoal = true
    self.nodeGoal.direction = targetCell.direction

    self.g_score[self.nodeBehindStart] = math.huge
    self.h_score[self.nodeBehindStart] = math.huge
    self.f_score[self.nodeBehindStart] = math.huge

    self.g_score[self.nodeStart] = 0
    self.h_score[self.nodeStart] = self:estimate_cost(self.nodeStart, self.nodeGoal)
    self.f_score[self.nodeStart] = self.h_score[self.nodeStart]
    self.came_from[self.nodeStart] = self.nodeBehindStart

    self.openset[self.nodeStart] = true
    self:pushOpenNode(self.nodeStart, self.f_score[self.nodeStart])
    self.closedset[self.nodeBehindStart] = true
    self:isDriveableAstar(self.nodeStart)
    self:setBlockedGoal()
    self.initNew = true
end

function PathFinderModule:createWayPointsNew()
    if self.smoothStep == 0 then
        self.wayPoints = {}
        for index, cell in ipairs(self.path) do
            self.wayPoints[index] = self:gridLocationToWorldLocation(cell)
            self.wayPoints[index].y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, self.wayPoints[index].x, 1, self.wayPoints[index].z)
            self.wayPoints[index].direction = cell.direction
        end
        -- remove zig zag line
        self:smoothResultingPPPath()
    end
    -- shortcut the path if possible
    self:smoothResultingPPPath_Refined()
    -- self.smoothStep = 2
    -- self.smoothDone = true

    if self.smoothStep == 2 then
        self:appendWayPointsNew()
    end
end

function PathFinderModule:appendWayPointsNew()
    -- When going to network, dont turn actual road network nodes into pathFinderPoints
    if self.goingToNetwork then
        for i = 1, #self.wayPoints, 1 do
            self.wayPoints[i].isPathFinderPoint = true
        end
    end

    if self.partialPath then
        self.smoothStep = 3
    elseif self.appendWayPoints ~= nil then
        for i = 1, #self.appendWayPoints, 1 do
            self.wayPoints[#self.wayPoints + 1] = self.appendWayPoints[i]
        end
        self.smoothStep = 3
    end

    -- See comment above
    if not self.goingToNetwork then
        for i = 1, #self.wayPoints, 1 do
            self.wayPoints[i].isPathFinderPoint = true
        end
    end
end

function PathFinderModule:isDriveableDubins(cell)
    cell.isRestricted = false
    cell.hasCollision = false
    PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableDubins start xz %d,%d "
        , cell.x, cell.z
    )

    --Try going through the checks in a way that fast checks happen before slower ones which might then be skipped

    cell.isOnField = PathFinderModule.getMapOnFieldAt(cell.worldPos.x, cell.worldPos.z)

    -- check the most probable restrictions on field first to prevent unneccessary checks
    if not cell.isRestricted and self.restrictToField and not (self.fallBackMode1 or self.fallBackMode2) then
        local fieldAllowed = self:isFieldPositionAllowed(cell.worldPos.x, cell.worldPos.z, cell.isOnField)
        cell.isRestricted = cell.isRestricted or (not fieldAllowed)
        if not fieldAllowed then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableDubins not cell.isOnField xz %d,%d "
                , cell.x, cell.z
            )
        end
    end

    local angleRad = AutoDrive.normalizeAngle(cell.t)
    local sizeMax = self.vehicle.size.width / 2
    local vectorX = {x =   math.cos(angleRad) * sizeMax, z = math.sin(angleRad) * sizeMax}
    local vectorZ = {x = - math.sin(angleRad) * sizeMax, z = math.cos(angleRad) * sizeMax}

    local corners = {}
    local centerLocation = cell.worldPos
    corners[1] = {x = centerLocation.x + (-vectorX.x - vectorZ.x), z = centerLocation.z + (-vectorX.z - vectorZ.z)}
    corners[2] = {x = centerLocation.x + (vectorX.x - vectorZ.x), z = centerLocation.z + (vectorX.z - vectorZ.z)}
    corners[3] = {x = centerLocation.x + (-vectorX.x + vectorZ.x), z = centerLocation.z + (-vectorX.z + vectorZ.z)}
    corners[4] = {x = centerLocation.x + (vectorX.x + vectorZ.x), z = centerLocation.z + (vectorX.z + vectorZ.z)}
    cell.corners = corners

    if not cell.isRestricted and self.avoidFruitSetting and not self.fallBackMode3 then
        -- check for fruit
        self:checkForFruitInArea(cell, corners) -- set cell.isRestricted if fruit found
        if cell.isRestricted then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableDubins cell.isRestricted xz %d,%d fruit found %s"
                , cell.x, cell.z
                , self.fruitToCheck
            )
        end
    end

    if not cell.isRestricted and cell.incoming ~= nil then
        -- check for up/down is to big or below water level
        local worldPosPrevious = cell.incoming.worldPos
        local angelToSlope, angle = self:checkSlopeAngle(cell.worldPos.x, cell.worldPos.z, worldPosPrevious.x, worldPosPrevious.z)    --> true if up/down or roll is to big or below water level
        cell.angle = angle
        cell.hasCollision = cell.hasCollision or angelToSlope
        cell.isRestricted = cell.isRestricted or cell.hasCollision
        if angelToSlope then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableDubins angelToSlope xz %d,%d"
                , cell.x, cell.z
            )
        end
    end

    if not cell.isRestricted then
        -- check for obstacles
        self.collisionhits = 0
        local shapes = overlapBox(cell.worldPos.x, cell.worldPos.y + cell.height / 2, cell.worldPos.z, 0, cell.t, 0, sizeMax, cell.height / 2, sizeMax, "collisionTestCallback", self, self.mask, true, true, true, true)
        cell.hasCollision = cell.hasCollision or (self.collisionhits > 0)
            or self:boxHitsBlockedZone(cell.worldPos.x, cell.worldPos.z, cell.t, sizeMax, sizeMax)
        cell.isRestricted = cell.isRestricted or cell.hasCollision
        if cell.hasCollision then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableDubins cell.hasCollision xz %d,%d collision"
                , cell.x, cell.z
            )
        end
    end

    if not cell.isRestricted and cell.incoming ~= nil then
        local worldPosPrevious = cell.incoming.worldPos
        local vectorX = worldPosPrevious.x - cell.worldPos.x
        local vectorZ = worldPosPrevious.z - cell.worldPos.z
        local dirVec = { x=vectorX, z = vectorZ}

        local cellUsedByVehiclePath = AutoDrive.checkForVehiclePathInBox(corners, self.minTurnRadius, self.vehicle, dirVec)
        cell.isRestricted = cell.isRestricted or cellUsedByVehiclePath
        self.blockedByOtherVehicle = self.blockedByOtherVehicle or cellUsedByVehiclePath
        if cellUsedByVehiclePath then
            PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableDubins cellUsedByVehiclePath xz %d,%d vehicle"
                , cell.x, cell.z
            )
        end
    end
    if cell.isRestricted then
        PathFinderModule.debugMsg(self.vehicle, "PFM:isDriveableDubins end isRestricted xz %d,%d "
            , cell.x, cell.z
        )
    end
    return not(cell.isRestricted)
end

function PathFinderModule:getDubinsPath()
    PathFinderModule.debugMsg(self.vehicle, "PFM:getDubinsPath start")
    self.dubinsPath = nil
    local result = ADDubins.EDUBNOPATH
    self.dubinsNodes  = {}
    local diffNetTime = netGetTime()

    local function get_node(x, z)
        local row = self.dubinsNodes[z]
        if not row then row = {}; self.dubinsNodes[z] = row end
        local node = row[x]
        if not node then node = { x = x, z = z, cost = 0 }; row[x] = node end
        return node
    end

    local function checkPath()
        local result = self.dubins:dubins_path_sample_many(ADDubins.DubinsPath, 1, self.dubins.createWayPoints)
        PathFinderModule.debugMsg(self.vehicle, "PFM:getDubinsPath dubins_path_sample_many result %d"
            , result
        )
        if result == ADDubins.EDUBOK then
            PathFinderModule.debugMsg(self.vehicle, "PFM:getDubinsPath dubins_path_sample_many #self.dubins.outPath %d"
            , #self.dubins.outPath
            )
            if self.dubins.outPath and #self.dubins.outPath > 0 then
                local cells = {}
                for i, wayPoint in ipairs(self.dubins.outPath) do
                    local cell = get_node(wayPoint.x, wayPoint.z)
                    cell.worldPos = {x = wayPoint.x, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, wayPoint.x, 1, wayPoint.z), z = wayPoint.z}
                    cell.t = wayPoint.t
                    cell.height = self.vehicleMinHeight
                    -- samples are 1 m apart, check the slope against the one 5 samples back
                    cell.incoming = cells[i - 5]
                    cells[i] = cell
                    if not self:isDriveableDubins(cell) then
                        result = ADDubins.EDUBNOPATH
                        break
                    end
                end
            else
                result = ADDubins.EDUBNOPATH
            end
        end
        return result
    end

    self.dubins.outPath = {}
    result = self.dubins:dubins_shortest_path(ADDubins.DubinsPath, self.q0, self.q1, self.realTurnRadius or self.minTurnRadius)
    PathFinderModule.debugMsg(self.vehicle, "PFM:getDubinsPath dubins_shortest_path result %d"
        , result
    )
    if result == ADDubins.EDUBOK then
        result = checkPath()
        if result == ADDubins.EDUBOK then
            PathFinderModule.debugMsg(self.vehicle, "PFM:getDubinsPath found shortest path #self.dubins.outPath %d result %d"
                , #self.dubins.outPath
                , result
            )
            self:printStrictSummary("direct curve")
            self:printTrip("direct curve")
            self.dubinsPath = self.dubins.outPath
            return self.dubinsPath
        end
    end

    for i = 1, 6, 1 do
        self.dubins.outPath = {}
        result = self.dubins:dubins_path(ADDubins.DubinsPath, self.q0, self.q1, self.realTurnRadius or self.minTurnRadius, i)
        PathFinderModule.debugMsg(self.vehicle, "PFM:getDubinsPath dubins_path i %d result %d"
            , i
            , result
        )
        if result == ADDubins.EDUBOK then
            result = checkPath()
        end
        if result == ADDubins.EDUBOK then
            break
        end
    end
    PathFinderModule.debugMsg(self.vehicle, "PFM:getDubinsPath #self.dubins.outPath %d result %d"
        , #self.dubins.outPath
        , result
    )
    if result == ADDubins.EDUBOK then
        self.dubinsPath = self.dubins.outPath
    end

    diffNetTime = netGetTime() - diffNetTime
    PathFinderModule.debugMsg(self.vehicle, "PFM:getDubinsPath end diffNetTime %d"
        , diffNetTime
    )
    return self.dubinsPath
end

function PathFinderModule:collisionTestCallback(transformId)
    if transformId ~= 0 and transformId ~= g_currentMission.terrainRootNode then
        local collisionObject = g_currentMission:getNodeObject(transformId)
        if (collisionObject == nil) or (collisionObject ~= nil and not (collisionObject.rootVehicle == self.vehicle)) then
            self.collisionhits = self.collisionhits + 1
            if PathFinderModule.debug == true then
                local currentCollMask = getCollisionFilterGroup(transformId)
                if currentCollMask then
                    local x, _, z = getWorldTranslation(transformId)
                    x = x + g_currentMission.mapWidth/2
                    z = z + g_currentMission.mapHeight/2

                    PathFinderModule.debugMsg(collisionObject, "PathFinderModule:collisionTestCallback transformId ->%s<- collisionObject ->%s<- getRigidBodyType->transformId %s getName->transformId %s getNodePath %s"
                        , tostring(transformId)
                        , tostring(collisionObject)
                        , tostring(getRigidBodyType(transformId))
                        , tostring(getName(transformId))
                        , tostring(I3DUtil.getNodePath(transformId))
                    )
                    if collisionObject then
                        PathFinderModule.debugMsg(collisionObject, "PathFinderModule:collisionTestCallback xmlFilename ->%s<-"
                            , tostring(collisionObject.xmlFilename)
                        )
                    end
                    PathFinderModule.debugMsg(collisionObject, "PathFinderModule:collisionTestCallback xz %.0f %.0f currentCollMask %s"
                        , x, z
                        , MathUtil.numberToSetBitsStr(currentCollMask)
                    )
                end
            end
        end
    end
end

function PathFinderModule.debugMsg(vehicle, debugText, ...)
    if PathFinderModule.debug == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    else
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_PATHINFO, debugText, ...)
    end
end
