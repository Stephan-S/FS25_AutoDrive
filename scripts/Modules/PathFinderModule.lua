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
    self.allowCrop = false -- set by the caller after reset: this one trip may cross crop (full trailer without a clean way out)
    self.edgeOnly = false -- set by the caller after reset: close to a forage harvester, crop edge following only (no grid)
    self.startPose = nil -- set by the caller after reset: plan from this pose {x, y, z, rx, rz} instead of the vehicle (next piece of a route)
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
    self.partialPath = false
    self.bandTour = nil
    self.goingBehind = false
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
    PathFinderModule.setHarvestField(combine)
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
    -- forage harvester: always from behind, at any distance, the chase takes over there
    if not chasing and combine.ad ~= nil and (self.strictFruit or self.strictField)
        and (combine.ad.isChopper or AutoDrive.getDistanceBetween(self.vehicle, combine) > PathFinderModule.REAR_FIRST_DISTANCE) then
        self:startRearFallback()
    end
    if AutoDrive.getDistanceBetween(self.vehicle, combine) < 50 then
        -- shorten path calculation for close combine
        self.max_pathfinder_steps = PathFinderModule.MAX_PATHFINDER_STEPS_COMBINE_TURN
    end
    -- forage harvester: the lane of the previous unloader first, else follow the crop edge, else the grid
    if not chasing and not self:tryLane() and not self:tryEdgeFollow() and self.edgeOnly then
        -- close to the harvester: no grid route there (it loops), wait and let the chase try again
        self:abort()
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

-- the 2nd unloader aims a crumb of the 1st (the point and the trail direction there) instead of a
-- point behind the 1st (DriveToVehicleTask)
function PathFinderModule:startPathPlanningToFollow(targetVehicle, targetPoint, targetVector)
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule:startPathPlanningToFollow %s", tostring(targetVehicle:getName()))
    self:startPathPlanningTo({x = targetPoint.x, y = targetPoint.y, z = targetPoint.z}, {x = targetVector.x, z = targetVector.z})
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

-- half width over the outer tyre edges of the whole train (crop is only run over by the wheels), nil if unknown
function PathFinderModule:getWheelHalfWidth()
    local units = AutoDrive.getAllUnits(self.vehicle)
    local half = nil
    for _, unit in pairs(units or {}) do
        local spec = unit.spec_wheels
        local root = unit.components ~= nil and unit.components[1] ~= nil and unit.components[1].node or nil
        if spec ~= nil and spec.wheels ~= nil and root ~= nil then
            for _, wheel in pairs(spec.wheels) do
                local physics = wheel.physics
                if physics ~= nil and wheel.node ~= nil and physics.positionX ~= nil then
                    local x = localToLocal(wheel.node, root, physics.positionX, physics.positionY or 0, physics.positionZ or 0)
                    local edge = math.abs(x) + (physics.width or PathFinderModule.WHEEL_DEFAULT_WIDTH) / 2
                    half = math.max(half or 0, edge)
                end
            end
        end
    end
    return half
end

-- half width used by the crop checks: forage harvester at the outer tyre edges, no margin; otherwise train width + margin
PathFinderModule.WHEEL_DEFAULT_WIDTH = 0.6
function PathFinderModule:updateCropHalfWidth()
    self.cropHalfWidth = self.trainHalfWidth
    if not self:isChopperWork() then
        return
    end
    local wheelHalf = self:getWheelHalfWidth()
    if wheelHalf ~= nil and wheelHalf > 0.5 then
        self.cropHalfWidth = math.min(wheelHalf, self.trainHalfWidth)
    end
end

function PathFinderModule:getCropHalfWidth()
    return self.cropHalfWidth or self.trainHalfWidth or 2
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
    local half = self:getCropHalfWidth()
    local nx, nz = -dz / length * half, dx / length * half
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
        return isOnField or self.parkOnGrass == true
    end
    if not isOnField and self.parkOnGrass then
        -- forage harvester, full unloader parking aside: the grass beside the field is allowed (obstacles checked)
        return true
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
    -- the unloader may stand at the entrance on ground taken for another field: the first meters from where it
    -- stands are never refused for another field
    if self.startX ~= nil and MathUtil.vector2Length(x - self.startX, z - self.startZ) < PathFinderModule.OFF_FIELD_NEAR_ENDS then
        return true
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

-- zones the pathfinder must avoid although overlapBox finds nothing there (learned by ADDrivePathModule:learnBlockedPlace:
-- an obstacle the front sensor stopped on, which the route search does not see). {x, z, radius}
-- box as given to overlapBox (center, rotation around Y, half extents along local X and Z)
function PathFinderModule:boxHitsBlockedZone(cx, cz, ry, halfX, halfZ)
    local cosA, sinA = math.cos(ry), math.sin(ry)
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

function PathFinderModule.addLearnedZone(x, z, vehicleName)
    for _, zone in ipairs(PathFinderModule.learnedZones) do
        if MathUtil.vector2Length(zone.x - x, zone.z - z) < zone.radius then
            return
        end
    end
    table.insert(PathFinderModule.learnedZones, {x = x, z = z, radius = PathFinderModule.LEARNED_ZONE_RADIUS})
    AutoDrive.debugPrint(nil, AutoDrive.DC_PATHINFO, "PathFinderModule: %s: blocked zone added at %.0f,%.0f (front sensor blocked), %d zone(s)", tostring(vehicleName), x, z, #PathFinderModule.learnedZones)
end

PathFinderModule.FRUIT_MAX_RATIO = 0.10
PathFinderModule.BAND_FLOOD_WAIT = 10000 -- ms the search waits for the flood of the cut strips (ADBandRoute)
PathFinderModule.BAND_UNREACHED_PENALTY = 1000 -- estimate added for a place out of the strips that reach the target
PathFinderModule.BAND_START_LOOKUP = 8 -- m around the start to find the strips (it may stand at the crop edge)
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
-- the 2 m cells of a recorded field are read again every 0.5 m (16 samples): FINE_BAND_WIDTH from the outer
-- border toward the middle, wider than any field = the whole field (wheel lanes through the crop
-- looked like a way in 2 m samples). The scan reads them after the field, spread over the frames as well
PathFinderModule.FINE_BAND_WIDTH = 2000
PathFinderModule.FINE_STEP = 0.5
PathFinderModule.fineBand = {}       -- key of a 2 m cell -> true when it lies in the band (set by the field scan)
PathFinderModule.fineReads = 0

-- standing crop at one position: fruit type index, 0 if none (cut, withered, grass and meadow count as none)
function PathFinderModule.readCropAt(x, z)
    local fruitTypeIndex, growthState = FSDensityMapUtil.getFruitTypeIndexAtWorldPos(x, z)
    if fruitTypeIndex == nil or fruitTypeIndex == 0 or growthState == nil or growthState == 0 then
        return 0
    end
    if fruitTypeIndex == FruitType.MEADOW or fruitTypeIndex == FruitType.GRASS then
        -- second value: grass ground, not the field of a harvester working another crop
        return 0, true
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
    cell = {t = now, field = {}, grass = {}}
    local cx, cz = ix * PathFinderModule.MAP_CELL, iz * PathFinderModule.MAP_CELL
    for k, offset in ipairs(PathFinderModule.MAP_SAMPLE_OFFSETS) do
        local crop, grass = PathFinderModule.readCropAt(cx + offset[1], cz + offset[2])
        cell[k] = crop
        cell.grass[k] = grass == true
        cell.field[k] = AutoDrive.checkIsOnField(cx + offset[1], 0, cz + offset[2]) == true
        cell.onField = cell.onField or cell.field[k]
    end
    -- field number at the cell centre (the grass strip behind the hedge is field ground for the game)
    if cell.onField then
        cell.fieldId = PathFinderModule.getFieldIdAt(cx, cz)
    end
    if g_currentMission ~= nil and g_currentMission.terrainRootNode ~= nil then
        cell.y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, cx, 0, cz)
    end
    PathFinderModule.fieldMapReads = PathFinderModule.fieldMapReads + 1
    PathFinderModule.fieldMap[key] = cell
    return cell
end

-- the 16 samples (0.5 m apart) of a cell in the border band, read once per cell reading
function PathFinderModule.getFineCell(cell, ix, iz)
    if cell.fine ~= nil then
        return cell.fine
    end
    local size, step = PathFinderModule.MAP_CELL, PathFinderModule.FINE_STEP
    local n = math.floor(size / step + 0.5)
    local x0, z0 = ix * size - size / 2 + step / 2, iz * size - size / 2 + step / 2
    local fine = {crop = {}, field = {}, grass = {}, n = n}
    for j = 0, n - 1 do
        for i = 0, n - 1 do
            local k = j * n + i + 1
            local x, z = x0 + i * step, z0 + j * step
            local crop, grass = PathFinderModule.readCropAt(x, z)
            fine.crop[k] = crop
            fine.grass[k] = grass == true
            fine.field[k] = AutoDrive.checkIsOnField(x, 0, z) == true
        end
    end
    PathFinderModule.fineReads = PathFinderModule.fineReads + 1
    cell.fine = fine
    return fine
end

-- index of the fine sample at a position, nil when the cell is not in the border band
function PathFinderModule.getFineIndex(ix, iz, x, z)
    if not PathFinderModule.fineBand[(ix + 50000) * 100000 + (iz + 50000)] then
        return nil
    end
    local size, step = PathFinderModule.MAP_CELL, PathFinderModule.FINE_STEP
    local n = math.floor(size / step + 0.5)
    local i = math.min(n - 1, math.max(0, math.floor((x - (ix * size - size / 2)) / step)))
    local j = math.min(n - 1, math.max(0, math.floor((z - (iz * size - size / 2)) / step)))
    return j * n + i + 1
end

-- isolated plants (forage harvester only, strict checks): a crop patch at most ISOLATED_PATCH_SIZE wide,
-- with no other crop within ISOLATED_CLEARANCE around it, is forgotten. A crop sample is isolated when no crop
-- is found in the ring between ISOLATED_PATCH_SIZE + 0.5 m and ISOLATED_PATCH_SIZE + ISOLATED_CLEARANCE around it
-- (sampled every 0.5 m): every point of a patch that small lies inside the ring, any standing crop edge or row
-- reaches into it. Results are kept ISOLATED_TTL ms (the harvester keeps cutting)
PathFinderModule.ISOLATED_PATCH_SIZE = 2
PathFinderModule.ISOLATED_CLEARANCE = 2.5
PathFinderModule.ISOLATED_STEP = 0.5
PathFinderModule.ISOLATED_TTL = 3000
PathFinderModule.ISOLATED_CACHE_MAX = 50000
PathFinderModule.isolatedCache = {}
PathFinderModule.isolatedCacheCount = 0
PathFinderModule.ignoreIsolatedCrop = false -- set while a forage harvester unloader searches or checks a route

function PathFinderModule.isIsolatedCrop(x, z)
    local step = PathFinderModule.ISOLATED_STEP
    local ix, iz = math.floor(x / step + 0.5), math.floor(z / step + 0.5)
    local key = (ix + 500000) * 1000000 + (iz + 500000)
    local now = g_time or 0
    local cached = PathFinderModule.isolatedCache[key]
    if cached ~= nil and now >= cached.t and now - cached.t < PathFinderModule.ISOLATED_TTL then
        return cached.v
    end
    if PathFinderModule.isolatedCacheCount > PathFinderModule.ISOLATED_CACHE_MAX then
        PathFinderModule.isolatedCache = {}
        PathFinderModule.isolatedCacheCount = 0
    end
    local cx, cz = ix * step, iz * step
    local inner = PathFinderModule.ISOLATED_PATCH_SIZE + step
    local outer = PathFinderModule.ISOLATED_PATCH_SIZE + PathFinderModule.ISOLATED_CLEARANCE
    local n = math.floor(outer / step + 0.5)
    local isolated = true
    for j = -n, n do
        local oz = j * step
        for i = -n, n do
            local ox = i * step
            if math.max(math.abs(ox), math.abs(oz)) >= inner - 0.01 and PathFinderModule.getMapCropRawAt(cx + ox, cz + oz) ~= 0 then
                isolated = false
                break
            end
        end
        if not isolated then
            break
        end
    end
    if not isolated then
        -- a long narrow strip left standing (one or two rows) counts as isolated too
        isolated = PathFinderModule.isCropStrip(cx, cz)
    end
    if cached == nil then
        PathFinderModule.isolatedCacheCount = PathFinderModule.isolatedCacheCount + 1
    end
    PathFinderModule.isolatedCache[key] = {t = now, v = isolated}
    return isolated
end

-- narrow strip: in one of the directions across it, no crop between STRIP_HALF_WIDTH + 0.5 and
-- STRIP_HALF_WIDTH + ISOLATED_CLEARANCE on both sides, here and STRIP_ALONG, 2 x STRIP_ALONG m further each way along it
-- (a single gap in a crop block is not a strip)
PathFinderModule.STRIP_HALF_WIDTH = 1.5 -- m: strips up to about 3 m wide
PathFinderModule.STRIP_DIRECTIONS = 8
PathFinderModule.STRIP_ALONG = 1.5
-- two leftover strips 2-3 m apart were seen as a crop block; 1 m clear beside a strip is enough
PathFinderModule.STRIP_CLEARANCE = 2.5 -- m clear on both sides (less and a field counts as a strip)
PathFinderModule.STRIP_MAX_WIDTH = 4 -- m of crop across a strip at most
function PathFinderModule.isCropStrip(cx, cz)
    -- thickness across the strip measured (crop run until 1 m clear on each side), up to
    -- STRIP_MAX_WIDTH, here and along it: a strip 2-3 m wide seen from its edge was not one with the old rule
    local step = PathFinderModule.ISOLATED_STEP
    local maxWidth = PathFinderModule.STRIP_MAX_WIDTH
    local clearNeed = PathFinderModule.STRIP_CLEARANCE
    local along = PathFinderModule.STRIP_ALONG
    local function run(bx, bz, px, pz)
        -- distance of crop from (bx,bz) toward p before clearNeed m without crop, nil when wider than maxWidth
        local last, r, free = 0, step, 0
        while r <= maxWidth + clearNeed + 0.01 do
            if PathFinderModule.getMapCropRawAt(bx + px * r, bz + pz * r) ~= 0 then
                last, free = r, 0
            else
                free = free + step
                if free >= clearNeed - 0.01 then
                    return last
                end
            end
            r = r + step
        end
        return nil
    end
    for k = 0, PathFinderModule.STRIP_DIRECTIONS - 1 do
        local a = math.pi * k / PathFinderModule.STRIP_DIRECTIONS
        local dx, dz = math.sin(a), math.cos(a)
        local px, pz = dz, -dx
        local ok = true
        -- the same across line here and twice along each way: a field corner seen diagonally is no strip
        for _, l in ipairs({0, -along, along, -2 * along, 2 * along}) do
            local bx, bz = cx + dx * l, cz + dz * l
            local plus = run(bx, bz, px, pz)
            local minus = plus ~= nil and run(bx, bz, -px, -pz) or nil
            if plus == nil or minus == nil or plus + minus > maxWidth then
                ok = false
                break
            end
        end
        if ok then
            return true
        end
    end
    return false
end

-- crop at a position from the map; isolated plants count as none while a forage harvester unloader checks
function PathFinderModule.getMapCropAt(x, z)
    local crop = PathFinderModule.getMapCropRawAt(x, z)
    if crop ~= 0 and PathFinderModule.ignoreIsolatedCrop and PathFinderModule.isIsolatedCrop(x, z) then
        return 0
    end
    return crop
end

-- crop at a position from the map (nearest of the 4 samples of its cell, 0.5 m sample in the border band)
function PathFinderModule.getMapCropRawAt(x, z)
    local size = PathFinderModule.MAP_CELL
    local ix, iz = math.floor(x / size + 0.5), math.floor(z / size + 0.5)
    local cell = PathFinderModule.getMapCell(ix, iz)
    local fk = PathFinderModule.getFineIndex(ix, iz, x, z)
    if fk ~= nil then
        return PathFinderModule.getFineCell(cell, ix, iz).crop[fk]
    end
    local k = 1
    if x >= ix * size then
        k = k + 1
    end
    if z >= iz * size then
        k = k + 2
    end
    return cell[k]
end

-- field ground at a position from the map (nearest of the 4 samples of its cell, 0.5 m sample in the border band)
function PathFinderModule.getMapOnFieldAt(x, z)
    local size = PathFinderModule.MAP_CELL
    local ix, iz = math.floor(x / size + 0.5), math.floor(z / size + 0.5)
    local cell = PathFinderModule.getMapCell(ix, iz)
    local fk = PathFinderModule.getFineIndex(ix, iz, x, z)
    local onField, grass
    if fk ~= nil then
        local fine = PathFinderModule.getFineCell(cell, ix, iz)
        onField, grass = fine.field[fk], fine.grass ~= nil and fine.grass[fk]
    else
        local k = 1
        if x >= ix * size then
            k = k + 1
        end
        if z >= iz * size then
            k = k + 2
        end
        onField, grass = cell.field[k], cell.grass ~= nil and cell.grass[k]
    end
    if onField and not PathFinderModule.isHarvestFieldGround(cell, grass) then
        return false
    end
    return onField
end

-- the field being harvested (an unloader drove round the hedge onto the
-- grass strip behind it, field ground for the game with no crop on it). While a forage harvester works another crop,
-- grass or meadow ground is not its field; ground of another field number is not its field either
PathFinderModule.HARVEST_FIELD_TTL = 60000 -- ms between two readings of the harvester's field
PathFinderModule.harvestField = nil
function PathFinderModule.getFieldIdAt(x, z)
    if g_fieldManager == nil or g_fieldManager.getFieldAtWorldPosition == nil then
        return nil
    end
    local ok, field = pcall(g_fieldManager.getFieldAtWorldPosition, g_fieldManager, x, z)
    if ok and type(field) == "table" then
        return field.fieldId or field.id
    end
    return nil
end
function PathFinderModule.setHarvestField(combine)
    if combine == nil or combine.components == nil or combine.ad == nil or not combine.ad.isChopper then
        return
    end
    local now = g_time or 0
    local hf = PathFinderModule.harvestField
    if hf ~= nil and hf.combine == combine and now - hf.t < PathFinderModule.HARVEST_FIELD_TTL then
        return
    end
    local x, _, z = getWorldTranslation(combine.components[1].node)
    local _, grass = PathFinderModule.readCropAt(x, z)
    -- grass is its field when it picks grass up (its tank says so), or when it stands on grass and nothing said otherwise yet
    local grassIsField = false
    if combine.spec_combine ~= nil and combine.spec_combine.fillUnitIndex ~= nil and combine.getFillUnitFillType ~= nil then
        local fillType = combine:getFillUnitFillType(combine.spec_combine.fillUnitIndex)
        if fillType ~= nil and FillType ~= nil and (fillType == FillType.GRASS_WINDROW or fillType == FillType.DRYGRASS_WINDROW) then
            grassIsField = true
        end
    end
    if not grassIsField and grass == true and (hf == nil or hf.combine ~= combine or hf.grassIsField) then
        grassIsField = true
    end
    local id = PathFinderModule.getFieldIdAt(x, z)
    local changed = hf == nil or hf.combine ~= combine or hf.id ~= id or hf.grassIsField ~= grassIsField
    PathFinderModule.harvestField = {combine = combine, id = id, grassIsField = grassIsField, t = now}
end
function PathFinderModule.isHarvestFieldGround(cell, grass)
    local hf = PathFinderModule.harvestField
    if hf == nil then
        return true
    end
    if grass and not hf.grassIsField then
        return false
    end
    if hf.id ~= nil and cell.fieldId ~= nil and cell.fieldId ~= hf.id then
        return false
    end
    return true
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
    if scan.fineKeys == nil then
        scan.bandCells = PathFinderModule.markFineBand(scan.visited)
        scan.fineKeys, scan.fineIndex = {}, 1
        for key, _ in pairs(scan.visited) do
            if PathFinderModule.fineBand[key] then
                scan.fineKeys[#scan.fineKeys + 1] = key
            end
        end
    end
    while scan.fineIndex <= #scan.fineKeys do
        local key = scan.fineKeys[scan.fineIndex]
        scan.fineIndex = scan.fineIndex + 1
        local ix, iz = math.floor(key / 100000) - 50000, key % 100000 - 50000
        PathFinderModule.getFineCell(PathFinderModule.getMapCell(ix, iz), ix, iz)
        if scan.fineIndex % 10 == 0 and mapClock() >= stopAt then
            return
        end
    end
    local bandCells = scan.bandCells
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

-- border band of a scanned field: from the cells off the field next to it (the ring read by the scan), over the
-- field cells, up to FINE_BAND_WIDTH toward the middle. Returns the number of cells in the band
function PathFinderModule.markFineBand(visited)
    local map = PathFinderModule.fieldMap
    local function decode(key)
        return math.floor(key / 100000) - 50000, key % 100000 - 50000
    end
    local function keyOf(ix, iz)
        return (ix + 50000) * 100000 + (iz + 50000)
    end
    local maxSteps = math.ceil(PathFinderModule.FINE_BAND_WIDTH / PathFinderModule.MAP_CELL)
    local dist, queue, head = {}, {}, 1
    for key, _ in pairs(visited) do
        local cell = map[key]
        if cell ~= nil and not cell.onField then
            local ix, iz = decode(key)
            for _, d in ipairs({{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) do
                local n = map[keyOf(ix + d[1], iz + d[2])]
                if n ~= nil and n.onField then
                    dist[key] = 0
                    queue[#queue + 1] = key
                    break
                end
            end
        end
    end
    while head <= #queue do
        local key = queue[head]
        head = head + 1
        local ix, iz = decode(key)
        if dist[key] < maxSteps then
            for _, d in ipairs({{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) do
                local nkey = keyOf(ix + d[1], iz + d[2])
                local n = map[nkey]
                if dist[nkey] == nil and n ~= nil and n.onField then
                    dist[nkey] = dist[key] + 1
                    queue[#queue + 1] = nkey
                end
            end
        end
    end
    local count = 0
    for key, _ in pairs(dist) do
        if not PathFinderModule.fineBand[key] then
            PathFinderModule.fineBand[key] = true
        end
        count = count + 1
    end
    return count
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
            if cell ~= nil and cell.y ~= nil and cell.fine ~= nil then
                -- border band: one small cross every 0.5 m
                local fine, step = cell.fine, PathFinderModule.FINE_STEP
                local x0, z0, y = ix * size - size / 2 + step / 2, iz * size - size / 2 + step / 2, cell.y + 0.15
                local hh = step * 0.35
                for j = 0, fine.n - 1 do
                    for i = 0, fine.n - 1 do
                        local k = j * fine.n + i + 1
                        if fine.field[k] then
                            local x, z = x0 + i * step, z0 + j * step
                            local red, green = 0, 1
                            if fine.crop[k] ~= 0 then
                                red, green = 1, 0
                            end
                            drawDebugLine(x - hh, y, z - hh, red, green, 0, x + hh, y, z + hh, red, green, 0, false)
                            drawDebugLine(x - hh, y, z + hh, red, green, 0, x + hh, y, z - hh, red, green, 0, false)
                        end
                    end
                end
            elseif cell ~= nil and cell.onField and cell.y ~= nil then
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
--: crop read from the density map over a
-- parallelogram, kept only if one of its samples is crop that is neither an isolated plant nor a narrow strip
PathFinderModule.ISLAND_SAMPLE = 1 -- m between samples
function PathFinderModule.isOnlyIsolatedCropIn(c1X, c1Z, c2X, c2Z, c3X, c3Z)
    local ux, uz = c2X - c1X, c2Z - c1Z
    local vx, vz = c3X - c1X, c3Z - c1Z
    local nu = math.max(1, math.ceil(math.sqrt(ux * ux + uz * uz) / PathFinderModule.ISLAND_SAMPLE))
    local nv = math.max(1, math.ceil(math.sqrt(vx * vx + vz * vz) / PathFinderModule.ISLAND_SAMPLE))
    local seen = false
    for i = 0, nu do
        for j = 0, nv do
            local x, z = c1X + ux * i / nu + vx * j / nv, c1Z + uz * i / nu + vz * j / nv
            if PathFinderModule.getMapCropRawAt(x, z) ~= 0 then
                seen = true
                if not PathFinderModule.isIsolatedCrop(x, z) then
                    return false
                end
            end
        end
    end
    return seen
end

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
                PathFinderModule.debugMsg(self.vehicle, "PathFinderModule:getFreeStartDistance obstacle ahead, start %.1f m in front instead of %.1f m", distance, maxDistance)
            return distance
        end
    end
    local shortest = distances[#distances]
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule:getFreeStartDistance obstacle right in front, start %.1f m in front", shortest)
    return shortest
end

-- where the route starts: the vehicle, or the pose given by the caller (end of the piece being driven)
function PathFinderModule:getStartPose()
    if self.startPose ~= nil then
        local p = self.startPose
        return p.x, p.y, p.z, p.rx, p.rz
    end
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local rx, _, rz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    return x, y, z, rx, rz
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
    local vehicleWorldX, vehicleWorldY, vehicleWorldZ, vehicleRx, vehicleRz = self:getStartPose()
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
    self:updateCropHalfWidth()

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

    local vehicleBehindX, vehicleBehindZ = -vehicleRx * self.minTurnRadius, -vehicleRz * self.minTurnRadius
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
    if self.allowCrop then
        -- crop avoided first, crossed only by the fallback (the caller lifts noFallback)
        self.strictFruit = false
    end
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
    -- forage harvester work, strict: real driving distances inside the cut strips toward the target (ADBandRoute)
    self.bandFlood = nil
    self.bandFloodStartDistance = nil
    if (self.strictFruit or self.strictField) and self:isChopperWork() and ADBandRoute ~= nil then
        self.bandFlood = ADBandRoute.request(targetX, targetZ, self:getCropHalfWidth())
        self.bandFloodWaitUntil = (g_time or 0) + PathFinderModule.BAND_FLOOD_WAIT
    end
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

    local vehicleWorldX, _, vehicleWorldZ, vehicleRx, vehicleRz = self:getStartPose()
    local vehicleVector = {x = vehicleRx, z = vehicleRz}
    self.startCell.direction = self:worldDirectionToGridDirection(vehicleVector)

    local vehicleBehindX, vehicleBehindZ = -vehicleRx * self.minTurnRadius, -vehicleRz * self.minTurnRadius
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
        local clear, reason = self:isFruitStretchClear(a, b, self.cropHalfWidth or halfWidth)
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
    local band = self:getBandGain(self.startX, self.startZ, endPoint.x, endPoint.z)
    if band ~= nil then
        return band
    end
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

-- a corner too sharp for the train: the route is kept up to it (rule: drive to the last valid point, plan
-- again from there), with the usual end checks of a partial route
PathFinderModule.SHARP_PARTIAL_MIN_LENGTH = 10

function PathFinderModule:usePartialBeforeSharp()
    local index = self.firstSharpIndex
    if not self:isChopperWork() or index == nil or index < 2 or not (self:partialAllowed() or self.partialPath) then
        return false
    end
    local length = 0
    for i = 2, index do
        length = length + MathUtil.vector2Length(self.wayPoints[i].x - self.wayPoints[i - 1].x, self.wayPoints[i].z - self.wayPoints[i - 1].z)
    end
    if length < PathFinderModule.SHARP_PARTIAL_MIN_LENGTH then
        return false
    end
    local saved, savedPartial = self.wayPoints, self.partialPath
    local p = saved[index]
    if self:usePartialPath(index) and self:trimPartialEnd() then
        return true
    end
    self.wayPoints = saved
    self.partialPath = savedPartial
    return false
end

-- the forage harvester behaviours (guided departure, short partial routes, direct curve checks...)
-- only apply when unloading a forage harvester: a combine harvester keeps the former behaviour
function PathFinderModule:isChopperWork()
    return AutoDrive.isChopperUnloading(self.vehicle)
end

-- end of a partial route to a harvester: never waiting face to face with it, against its way
PathFinderModule.PARTIAL_MAX_HEADING_ANGLE = math.rad(120)
PathFinderModule.PARTIAL_HEADING_LENGTH = 3
PathFinderModule.PARTIAL_FACING_MAX_DISTANCE = 60 -- the rule only matters near the harvester
PathFinderModule.PARTIAL_FACING_LIFT_FAILURES = 5 -- after so many waits in a row, the rule is lifted
PathFinderModule.PARTIAL_SHORT_MIN_GAIN = 2 -- a partial route shortened to a clean stop must still move this much

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
    if self:isChopperWork() then
        if self.partialFacingLifted then
            return false
        end
        local cx, _, cz = getWorldTranslation(self.pipeCombine.components[1].node)
        if MathUtil.vector2Length(b.x - cx, b.z - cz) > PathFinderModule.PARTIAL_FACING_MAX_DISTANCE then
            return false
        end
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
PathFinderModule.PARTIAL_DRIVE_SHARE = 0.8 -- forage harvester: share of a partial piece driven before the stop
PathFinderModule.STOP_OVERRUN = 4 -- clear ground in front of a stop: the train overruns its last point by ~3 m
PathFinderModule.PARTIAL_STOP_START_IGNORE = 15 -- crop this close to the start is where the train already stands
function PathFinderModule:isPartialStopClear(i)
    local wps = self.wayPoints
    local a, b = wps[i - 1], wps[i]
    local dx, dz = b.x - a.x, b.z - a.z
    local l = math.sqrt(dx * dx + dz * dz)
    if l < 0.05 then
        return true
    end
    dx, dz = dx / l, dz / l
    local chopper = self:isChopperWork()
    -- exact train width: the route itself is already checked with its margin (combine harvester: width + 1 m)
    local half = chopper and self:getCropHalfWidth() or ((self.trainHalfWidth or 2) + 1)
    local back = math.max(AutoDrive.getTractorTrainLength(self.vehicle, true, false), 8)
    -- the train already stands near the start: crop there is not run over by driving on, it is not counted
    local start = wps[1]
    if chopper and start ~= nil then
        local fromStart = MathUtil.vector2Length(b.x - start.x, b.z - start.z) - PathFinderModule.PARTIAL_STOP_START_IGNORE
        back = math.max(math.min(back, fromStart), 0.5)
    end
    local c1X, c1Z = b.x - dz * half - dx * back, b.z + dx * half - dz * back
    local ahead = chopper and PathFinderModule.STOP_OVERRUN or 2
    local c2X, c2Z = b.x - dz * half + dx * ahead, b.z + dx * half + dz * ahead
    local c4X, c4Z = b.x + dz * half - dx * back, b.z - dx * half - dz * back
    return PathFinderModule.getMapCropRatio(nil, c1X, c1Z, c2X, c2Z, c4X, c4Z) <= 0
end

-- guided departure (strict): when no clean route leaves from where the train stands (corners too sharp for it,
-- crop at a tight field entrance), short curves are tried from the real position and heading of the tractor,
-- first with its real turning radius, then tighter. The whole band swept by the train must be free of crop
-- (inner side widened in curves, the trailer cuts them) and of obstacles, and the train must stand clear at the stop
PathFinderModule.GUIDED_DISTANCES = {15, 20, 25, 30, 35, 40}
PathFinderModule.GUIDED_BEARING_MAX = 90 -- deg, stop points ahead and beside the tractor (parked askew)
PathFinderModule.GUIDED_BEARING_STEP = 15
PathFinderModule.GUIDED_HEADING_MAX = 90 -- deg, heading at the stop relative to the tractor
PathFinderModule.GUIDED_HEADING_STEP = 15
PathFinderModule.GUIDED_RADIUS_FACTORS = {1, 0.8, 0.7}
PathFinderModule.GUIDED_MIN_GAIN = 5
-- no route but with a corner too sharp, no clean stop closer: drive on to a clean stop further (rule), up to
-- GUIDED_MAX_LOSS farther from the target, the one with the most room of the strips around it, then plan again.
-- Once per place (GUIDED_LOSS_PLACE m, GUIDED_LOSS_TIME ms), GUIDED_LOSS_MAX_IN_ROW in a row, then the task's U-turn
PathFinderModule.GUIDED_MAX_LOSS = 30
PathFinderModule.GUIDED_LOSS_PLACE = 15
PathFinderModule.GUIDED_LOSS_TIME = 120000
PathFinderModule.GUIDED_LOSS_MAX_IN_ROW = 2
PathFinderModule.GUIDED_ROOM_RADIUS = 10
PathFinderModule.GUIDED_MAX_CURVES = 60 -- curves checked per radius, keeps the frame time low
PathFinderModule.GUIDED_MAX_IN_ROW = 3 -- guided departures in a row without a normal route
PathFinderModule.GUIDED_TRAILER_LENGTH = 6 -- for the cut of the trailer inside the curves
PathFinderModule.GUIDED_STOP_AHEAD = 4 -- clear ground in front of the stop (the train overruns ~3 m)
PathFinderModule.GUIDED_EXIT_AHEAD = 10 -- m ahead of the stop free of crop at the train width: never stop nose against the crop
PathFinderModule.GUIDED_POINT_SPACING = 2
PathFinderModule.GUIDED_EXACT_LENGTH = 15 -- m at the start of a guided curve checked at the exact train width (no margin)
PathFinderModule.GUIDED_START_TOLERANCE = 4 -- first meters where the train already stands: exact width, as the grid search
PathFinderModule.GUIDED_RETRY_TIME = 15000 -- no new search from the same place before this delay (ms)
-- repeated failure from the same place: the crop border lines of the map are ignored on the first meters,
-- level 1 = first line (the map spills onto the cut strip), level 2 = second line too (first crop row).
-- The stop zone stays fully clean at every level
PathFinderModule.GUIDED_BORDER_LENGTH = 30
PathFinderModule.GUIDED_BORDER_LINE = 2 -- m, one map cell
PathFinderModule.GUIDED_MAX_LEVEL = 2
PathFinderModule.GUIDED_FAILS_PER_LEVEL = 2
-- never stop in the way of the harvester: not in its lane ahead, not too close, not facing it
PathFinderModule.GUIDED_LANE_LENGTH = 60
PathFinderModule.GUIDED_LANE_MARGIN = 2
PathFinderModule.GUIDED_MIN_HARVESTER_DISTANCE = 15 -- failed searches at a level before the next one (extreme case only)

-- stop point in the way of the harvester the route goes to (nil: no harvester, nothing in the way)
function PathFinderModule:isGuidedStopInHarvesterWay(x, z, heading)
    local combine = self.pipeCombine
    if not self.goingToPipe or combine == nil or combine.components == nil then
        return false
    end
    local cx, _, cz = getWorldTranslation(combine.components[1].node)
    local rx, _, rz = AutoDrive.localDirectionToWorld(combine, 0, 0, 1)
    local dx, dz = x - cx, z - cz
    local distance = math.sqrt(dx * dx + dz * dz)
    if distance < PathFinderModule.GUIDED_MIN_HARVESTER_DISTANCE then
        return true
    end
    local ahead = dx * rx + dz * rz
    local side = math.abs(dx * rz - dz * rx)
    local width = (combine.size ~= nil and combine.size.width or 4) / 2 + (self.trainHalfWidth or 2) + PathFinderModule.GUIDED_LANE_MARGIN
    if ahead > 0 and ahead < PathFinderModule.GUIDED_LANE_LENGTH and side < width then
        return true
    end
    if distance < PathFinderModule.PARTIAL_FACING_MAX_DISTANCE
        and (math.sin(heading) * rx + math.cos(heading) * rz) < math.cos(PathFinderModule.PARTIAL_MAX_HEADING_ANGLE) then
        return true
    end
    return false
end

-- crop sample within level lines of clean cut field ground
function PathFinderModule.isGuidedBorderCrop(x, z, level)
    local maxR = level * PathFinderModule.GUIDED_BORDER_LINE
    for r = 1, maxR do
        for k = 0, 7 do
            local a = k * math.pi / 4
            local px, pz = x + math.sin(a) * r, z + math.cos(a) * r
            if PathFinderModule.getMapCropAt(px, pz) == 0 and PathFinderModule.getMapOnFieldAt(px, pz) then
                return true
            end
        end
    end
    return false
end

-- returns clear, distance along the curve of the first crop met
function PathFinderModule:isGuidedBandClear(path, radius, level)
    level = level or 0
    local along = 0
    -- crop band at the outer tyre edges (forage harvester only uses the guided departure)
    local half = self:getCropHalfWidth()
    local exactHalf = math.min(half, self:getTrainWidth() / 2)
    local cut = PathFinderModule.GUIDED_TRAILER_LENGTH
    cut = math.min(radius - math.sqrt(math.max(radius * radius - cut * cut, 0)), 3)
    for i = 2, #path do
        local a, b = path[i - 1], path[i]
        local dx, dz = b.x - a.x, b.z - a.z
        local l = math.sqrt(dx * dx + dz * dz)
        along = along + l
        if l > 0.05 then
            dx, dz = dx / l, dz / l
            local tolerant = level > 0 and along <= PathFinderModule.GUIDED_BORDER_LENGTH
            -- first GUIDED_EXACT_LENGTH meters (leaving a strip along the crop): exact train width, no comfort margin;
            -- the trailer cut inside the curves is still added after the first meters
            local base = along <= PathFinderModule.GUIDED_EXACT_LENGTH and exactHalf or half
            local left, right = base, base
            local c = path[i + 1]
            if i > PathFinderModule.GUIDED_START_TOLERANCE + 1 and c ~= nil then
                local cross = dx * (c.z - b.z) - dz * (c.x - b.x)
                if cross > 0.02 then
                    left = base + cut
                elseif cross < -0.02 then
                    right = base + cut
                end
            end
            local nx, nz = -dz, dx
            local w = -right
            while w <= left do
                for _, back in ipairs({0, 0.5}) do
                    local px, pz = b.x - dx * back + nx * w, b.z - dz * back + nz * w
                    local crop = PathFinderModule.getMapCropAt(px, pz)
                    if crop ~= nil and crop ~= 0 and not (tolerant and PathFinderModule.isGuidedBorderCrop(px, pz, level)) then
                        return false, along
                    end
                end
                w = w + 0.5
            end
        end
    end
    return true, nil
end

-- the stopped train stands on the last meters of the curve: that strip (and the overrun ahead) must be fully clean.
-- The first meters, where the train already stands, are left out as for the band
function PathFinderModule:isGuidedStopClear(path, hx, hz)
    local half = self:getCropHalfWidth()
    local trainLength = math.max(AutoDrive.getTractorTrainLength(self.vehicle, true, false), 8)
    local last = path[#path]
    -- overrun ahead of the stop, then room to drive on: a stop nose against the crop has no way out but reversing
    local ahead = math.max(PathFinderModule.GUIDED_STOP_AHEAD, PathFinderModule.GUIDED_EXIT_AHEAD)
    for f = 0.5, ahead, 0.5 do
        local w = -half
        while w <= half do
            local crop = PathFinderModule.getMapCropAt(last.x + hx * f - hz * w, last.z + hz * f + hx * w)
            if crop ~= nil and crop ~= 0 then
                return false
            end
            w = w + 0.5
        end
    end
    local covered = 0
    for i = #path, 2, -1 do
        if i <= PathFinderModule.GUIDED_START_TOLERANCE + 1 or covered >= trainLength then
            break
        end
        local a, b = path[i - 1], path[i]
        local dx, dz = b.x - a.x, b.z - a.z
        local l = math.sqrt(dx * dx + dz * dz)
        if l > 0.05 then
            dx, dz = dx / l, dz / l
            for _, back in ipairs({0, 0.5}) do
                local w = -half
                while w <= half do
                    local crop = PathFinderModule.getMapCropAt(b.x - dx * back - dz * w, b.z - dz * back + dx * w)
                    if crop ~= nil and crop ~= 0 then
                        return false
                    end
                    w = w + 0.5
                end
            end
            covered = covered + l
        end
    end
    return true
end

function PathFinderModule:tryGuidedDepartureFurther()
    local node = self.vehicle.components[1].node
    local vx, _, vz = getWorldTranslation(node)
    local now = g_time or 0
    if (self.guidedLossInRow or 0) >= PathFinderModule.GUIDED_LOSS_MAX_IN_ROW then
        return false
    end
    if self.guidedLossX ~= nil and now - (self.guidedLossTime or 0) < PathFinderModule.GUIDED_LOSS_TIME
        and MathUtil.vector2Length(vx - self.guidedLossX, vz - self.guidedLossZ) < PathFinderModule.GUIDED_LOSS_PLACE then
        return false
    end
    -- once per place, found or not
    self.guidedLossX, self.guidedLossZ, self.guidedLossTime = vx, vz, now
    -- the usual search with its limits lifted: no retry delay, no count in a row
    local failX, inRow = self.guidedFailX, self.guidedInRow
    self.guidedFailX, self.guidedInRow = nil, 0
    local level, levelFails = self.guidedLevel, self.guidedLevelFails
    self.guidedFurther = true
    local ok = self:tryGuidedDeparture()
    self.guidedFurther = false
    self.guidedFailX, self.guidedInRow = failX, inRow
    self.guidedLevel, self.guidedLevelFails = level, levelFails
    if ok then
        self.guidedLossInRow = (self.guidedLossInRow or 0) + 1
    end
    return ok
end

-- points of the strips within GUIDED_ROOM_RADIUS (2 m steps): room to turn there
function PathFinderModule:getGuidedRoom(x, z)
    if not self:isBandFloodReady() then
        return 0
    end
    local room, r = 0, PathFinderModule.GUIDED_ROOM_RADIUS
    for dz = -r, r, 2 do
        for dx = -r, r, 2 do
            if dx * dx + dz * dz <= r * r and ADBandRoute.getDistance(self.bandFlood, x + dx, z + dz, 1) ~= nil then
                room = room + 1
            end
        end
    end
    return room
end

function PathFinderModule:tryGuidedDeparture(allowLoss)
    if self.startPose ~= nil or not self:isChopperWork() or not (self.strictFruit or self.strictField) or self.target == nil or self.startX == nil then
        return false
    end
    if allowLoss then
        return self:tryGuidedDepartureFurther()
    end
    if (self.guidedInRow or 0) >= PathFinderModule.GUIDED_MAX_IN_ROW then
        return false
    end
    local node = self.vehicle.components[1].node
    local vx, _, vz = getWorldTranslation(node)
    -- nothing found from this very place a moment ago: no new search yet (it costs frame time)
    local now = g_time or 0
    if self.guidedFailX ~= nil and MathUtil.vector2Length(vx - self.guidedFailX, vz - self.guidedFailZ) < 1 then
        if now - (self.guidedFailTime or 0) < PathFinderModule.GUIDED_RETRY_TIME then
            return false
        end
    else
        -- another place: back to the clean search
        self.guidedLevel = 0
        self.guidedLevelFails = 0
    end
    local level = self.guidedLevel or 0
    local further = self.guidedFurther == true
    if further then
        level = 0
    end
    local rx, _, rz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local heading = math.atan2(rx, rz)
    local q0 = {vx, -vz, AutoDrive.normalizeAngle(heading + math.pi + math.pi / 2)}

    -- stop points on clean field ground ahead, best gain toward the target first
    local candidates = {}
    local inWay = 0
    for _, d in ipairs(PathFinderModule.GUIDED_DISTANCES) do
        for b = -PathFinderModule.GUIDED_BEARING_MAX, PathFinderModule.GUIDED_BEARING_MAX, PathFinderModule.GUIDED_BEARING_STEP do
            local a = heading + math.rad(b)
            local x, z = vx + math.sin(a) * d, vz + math.cos(a) * d
            local gain = self:partialGain({x = x, z = z})
            local crop = PathFinderModule.getMapCropAt(x, z)
            local minGain = further and -PathFinderModule.GUIDED_MAX_LOSS or PathFinderModule.GUIDED_MIN_GAIN
            if gain >= minGain and (crop == nil or crop == 0) and PathFinderModule.getMapOnFieldAt(x, z) then
                local room = further and self:getGuidedRoom(x, z) or 0
                for h = -PathFinderModule.GUIDED_HEADING_MAX, PathFinderModule.GUIDED_HEADING_MAX, PathFinderModule.GUIDED_HEADING_STEP do
                    local endHeading = heading + math.rad(h)
                    if self:isGuidedStopInHarvesterWay(x, z, endHeading) then
                        inWay = inWay + 1
                    else
                        candidates[#candidates + 1] = {x = x, z = z, d = d, heading = endHeading, gain = gain - math.abs(h) * 0.02, room = room}
                    end
                end
            end
        end
    end
    if further then
        -- the most room to turn first, then the smallest loss
        table.sort(candidates, function(c1, c2)
            if c1.room ~= c2.room then
                return c1.room > c2.room
            end
            return c1.gain > c2.gain
        end)
    else
        table.sort(candidates, function(c1, c2) return c1.gain > c2.gain end)
    end

    local tried, refusedBand, refusedStop, refusedObstacle, firstCrop = 0, 0, 0, 0, nil
    for _, factor in ipairs(PathFinderModule.GUIDED_RADIUS_FACTORS) do
        local radius = (self.realTurnRadius or self.minTurnRadius) * factor
        local checked = 0
        for _, cand in ipairs(candidates) do
            if checked >= PathFinderModule.GUIDED_MAX_CURVES then
                break
            end
            local hx, hz = math.sin(cand.heading), math.cos(cand.heading)
            local q1 = {cand.x, -cand.z, AutoDrive.normalizeAngle(cand.heading + math.pi + math.pi / 2)}
            self.dubins.outPath = {}
            if self.dubins:dubins_shortest_path(ADDubins.DubinsPath, q0, q1, radius) == ADDubins.EDUBOK
                and self.dubins:dubins_path_length(ADDubins.DubinsPath) <= cand.d * 1.6 then
                checked = checked + 1
                tried = tried + 1
                local length = self.dubins:dubins_path_length(ADDubins.DubinsPath)
                if self.dubins:dubins_path_sample_many(ADDubins.DubinsPath, 1, self.dubins.createWayPoints) == ADDubins.EDUBOK
                    and #self.dubins.outPath > 2 then
                    local path = self.dubins.outPath
                    path[#path + 1] = {x = cand.x, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, cand.x, 1, cand.z), z = cand.z, t = q1[3]}
                    local ok, cropAt = self:isGuidedBandClear(path, radius, level)
                    if not ok then
                        refusedBand = refusedBand + 1
                        if cropAt ~= nil and (firstCrop == nil or cropAt > firstCrop) then
                            firstCrop = cropAt
                        end
                    elseif not self:isGuidedStopClear(path, hx, hz) then
                        ok = false
                        refusedStop = refusedStop + 1
                    end
                    if ok then
                        -- level 2 drives over the first crop row on the first meters: the tractor's own crop check is left out there
                        local avoidFruit = self.avoidFruitSetting
                        -- obstacles, slope, field and the usual crop check of the tractor, as for any curve
                        local cells = {}
                        for i, wp in ipairs(path) do
                            local cell = {x = i, z = 0, t = wp.t, height = self.vehicleMinHeight}
                            cell.worldPos = {x = wp.x, y = wp.y, z = wp.z}
                            cell.incoming = cells[i - 5]
                            cells[i] = cell
                            -- the first meters are where the train already stands
                            if level >= 2 then
                                self.avoidFruitSetting = avoidFruit and i > PathFinderModule.GUIDED_BORDER_LENGTH
                            end
                            if i > PathFinderModule.GUIDED_START_TOLERANCE + 1 and not self:isDriveableDubins(cell) then
                                ok = false
                                refusedObstacle = refusedObstacle + 1
                                break
                            end
                        end
                        self.avoidFruitSetting = avoidFruit
                    end
                    if ok and not self:isGuidedTrailerClear(path) then
                        -- the trailer cut inside the curve into the waiting tractors
                        ok = false
                        refusedObstacle = refusedObstacle + 1
                    end
                    if ok then
                        local points = {}
                        local spacing = PathFinderModule.GUIDED_POINT_SPACING
                        for i, wp in ipairs(path) do
                            if i == 1 or i == #path or (i - 1) % spacing == 0 then
                                points[#points + 1] = {x = wp.x, y = wp.y, z = wp.z}
                            end
                        end
                        self.wayPoints = points
                        self.partialPath = true
                        if further then
                            return true
                        end
                        self.guidedInRow = (self.guidedInRow or 0) + 1
                        self.guidedLevel = 0
                        self.guidedLevelFails = 0
                        return true
                    end
                end
            end
        end
    end
    if further then
        return false
    end
    self.guidedFailX, self.guidedFailZ, self.guidedFailTime = vx, vz, now
    self.guidedLevelFails = (self.guidedLevelFails or 0) + 1
    local raise = level < PathFinderModule.GUIDED_MAX_LEVEL and self.guidedLevelFails >= PathFinderModule.GUIDED_FAILS_PER_LEVEL
    if raise then
        self.guidedLevel = level + 1
        self.guidedLevelFails = 0
    end
    return false
end

-- forage harvester, very narrow places only (a cut strip where a single train fits): the route follows the crop edge
-- at a fixed gap instead of the grid (cells cut a diagonal strip into stairs, the route zigzags or finds no way).
-- Line by line every EDGE_STEP meters: the crop edge is searched beside the line (0.5 m map), the route point is put
-- EDGE_GAP + half the tyre width away from it on the free side, turning no sharper than the train can. It ends close
-- to the harvester (the chase takes over), where the edge is lost or the strip blocked; the grid is used otherwise.
PathFinderModule.EDGE_STEP = 2
PathFinderModule.EDGE_GAP = 0.5            -- m between the tyres and the first crop row
PathFinderModule.EDGE_SCAN = 8             -- m beside the line where the crop edge is searched
PathFinderModule.EDGE_FREE_SCAN = 15       -- m on the free side to measure the strip
PathFinderModule.EDGE_NARROW_FACTOR = 2    -- narrow: strip narrower than this many trains (body width + margin)
PathFinderModule.EDGE_MAX_LENGTH = 300
PathFinderModule.EDGE_MIN_LENGTH = 10
PathFinderModule.EDGE_NO_GAIN_LENGTH = 20  -- m driven without getting closer to the harvester: stop
PathFinderModule.EDGE_STOP_OVERRUN = 4
PathFinderModule.EDGE_CORNER_MARGIN = 1 -- m added to the gap around a corner
PathFinderModule.EDGE_CORNER_MIN_ROOM = 6 -- m of cut ground behind a corner, else it is no way for the train
PathFinderModule.EDGE_TURN_MIN_DEG = 20 -- turn of a corner before the edge is looked for along the train
PathFinderModule.EDGE_TURN_FOUND = 2 -- m beyond the gap where the edge counts as found again
PathFinderModule.EDGE_BEND_EXTRA = 10 -- m looked ahead beyond the turn radius for a bend toward the crop
PathFinderModule.EDGE_BEND_MARGIN = 0.7 -- start turning away before the last moment

function PathFinderModule:edgeCropAt(x, z)
    local crop = PathFinderModule.getMapCropAt(x, z)
    return crop ~= nil and crop ~= 0
end

-- distance (0.5 m steps) from x, z along nx, nz to the first crop sample, nil if none within maxDistance
function PathFinderModule:edgeFindCrop(x, z, nx, nz, fromDistance, maxDistance)
    local w = fromDistance
    while w <= maxDistance do
        if self:edgeCropAt(x + nx * w, z + nz * w) then
            return w
        end
        w = w + 0.5
    end
    return nil
end

-- forage harvester: the track driven by the previous unloader up to the harvester (shared by the harvester), reused
-- from where it passes the train, every stretch checked again (crop at the tyres, field, obstacles such as parked trailers)
PathFinderModule.LANE_MAX_AGE = 1200000 -- ms
PathFinderModule.LANE_JOIN_DISTANCE = 6 -- m from the train to the lane
PathFinderModule.LANE_END_DISTANCE = 40 -- m: the end of the lane at most this far from the harvester

function PathFinderModule:laneStretchClear(a, b, cropHalf)
    local dx, dz = b.x - a.x, b.z - a.z
    local l = math.sqrt(dx * dx + dz * dz)
    if l < 0.05 then
        return true
    end
    local nx, nz = -dz / l, dx / l
    local n = math.max(1, math.ceil(l / 0.5))
    local samples = math.ceil(cropHalf * 4)
    for i = 0, n do
        local px, pz = a.x + dx * i / n, a.z + dz * i / n
        if not PathFinderModule.getMapOnFieldAt(px, pz) then
            return false
        end
        for k = 0, samples do
            local w = -cropHalf + 2 * cropHalf * k / samples
            if self:edgeCropAt(px + nx * w, pz + nz * w) then
                return false
            end
        end
    end
    return self:isStretchClear(a, b, nil, true, true)
end

function PathFinderModule:tryLane()
    if not self:isChopperWork() or not (self.strictFruit or self.strictField) or self.pipeCombine == nil or self.startX == nil then
        return false
    end
    local shared = self.pipeCombine.ad ~= nil and self.pipeCombine.ad.sharedLane or nil
    if shared == nil or (g_time or 0) - shared.time > PathFinderModule.LANE_MAX_AGE then
        return false
    end
    local lane = shared.points
    local x, _, z, hx, hz = self:getStartPose()
    local best, bestD = nil, PathFinderModule.LANE_JOIN_DISTANCE
    for i = 1, #lane - 1 do
        local d = MathUtil.vector2Length(lane[i].x - x, lane[i].z - z)
        local ldx, ldz = lane[i + 1].x - lane[i].x, lane[i + 1].z - lane[i].z
        local ll = math.sqrt(ldx * ldx + ldz * ldz)
        if d < bestD and ll > 0.1 and (ldx * hx + ldz * hz) / ll > 0.7 then
            best, bestD = i, d
        end
    end
    if best == nil then
        return false
    end
    local name = self:getLogName()
    local cropHalf = self:getCropHalfWidth()
    local cx, _, cz = getWorldTranslation(self.pipeCombine.components[1].node)
    local closeRange = CatchCombinePipeTask ~= nil and type(CatchCombinePipeTask.CLOSE_RANGE) == "number" and CatchCombinePipeTask.CLOSE_RANGE or 30
    local target = self.target or {x = cx, z = cz}
    local points = {{x = x, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 1, z), z = z}}
    -- join the lane a few meters ahead (not behind the tractor)
    local i = best
    while i < #lane and ((lane[i].x - x) * hx + (lane[i].z - z) * hz) < 3 do
        i = i + 1
    end
    local length, reason = 0, "end of the lane"
    while i <= #lane do
        local a, b = points[#points], lane[i]
        if not self:laneStretchClear(a, b, cropHalf) then
            reason = "lane blocked or crop"
            break
        end
        length = length + MathUtil.vector2Length(b.x - a.x, b.z - a.z)
        points[#points + 1] = {x = b.x, y = b.y, z = b.z}
        if MathUtil.vector2Length(cx - b.x, cz - b.z) < closeRange and self:isFruitStretchClear(b, target, cropHalf) then
            reason = "harvester in reach, clean straight line"
            break
        end
        i = i + 1
    end
    if reason == "lane blocked or crop" then
        local cut = 0
        while #points > 2 and cut < PathFinderModule.EDGE_STOP_OVERRUN do
            local p1, p2 = points[#points - 1], points[#points]
            local l = MathUtil.vector2Length(p2.x - p1.x, p2.z - p1.z)
            cut = cut + l
            length = length - l
            table.remove(points, #points)
        end
    end
    if length < PathFinderModule.EDGE_MIN_LENGTH then
        return false
    end
    -- (seen in game: 194 m of lane followed in circles, its end 77 m from the harvester): the lane must
    -- lead to the harvester, its end nearer to it than the start and, when the whole lane is taken, close to it
    local last = points[#points]
    local endDistance = MathUtil.vector2Length(cx - last.x, cz - last.z)
    local startDistance = MathUtil.vector2Length(cx - x, cz - z)
    if reason ~= "harvester in reach, clean straight line"
        and (endDistance > startDistance - 5 or (reason == "end of the lane" and endDistance > PathFinderModule.LANE_END_DISTANCE)) then
        return false
    end
    self.wayPoints = points
    self.partialPath = true
    self.isFinished = true
    self.smoothDone = true
    self.dubinsDone = true
    ADScheduler:removePathfinderVehicle(self.vehicle)
    return true
end

function PathFinderModule:tryEdgeFollow()
    if not self:isChopperWork() or not (self.strictFruit or self.strictField) or self.pipeCombine == nil or self.startX == nil then
        return false
    end
    local name = self:getLogName()
    local x, _, z, hx, hz = self:getStartPose()
    local l = math.sqrt(hx * hx + hz * hz)
    if l < 0.01 then
        return false
    end
    hx, hz = hx / l, hz / l
    local nx, nz = hz, -hx
    -- crop side: the nearer crop edge beside the train
    local cropSide, cropDistance = nil, math.huge
    for _, side in ipairs({1, -1}) do
        local d = self:edgeFindCrop(x, z, nx * side, nz * side, 0, PathFinderModule.EDGE_SCAN)
        if d ~= nil and d < cropDistance then
            cropSide, cropDistance = side, d
        end
    end
    if cropSide == nil then
        return false
    end
    -- free width of the strip: from the crop edge to the field border (or other crop) on the other side
    local sx, sz = -nx * cropSide, -nz * cropSide
    local free = 0
    while free < PathFinderModule.EDGE_FREE_SCAN do
        local px, pz = x + sx * (free + 0.5), z + sz * (free + 0.5)
        if not PathFinderModule.getMapOnFieldAt(px, pz) or self:edgeCropAt(px, pz) then
            break
        end
        free = free + 0.5
    end
    -- no width threshold: a crop edge beside the train is followed, the grid cuts strips into stairs.
    -- Never from inside the crop: every piece (also the ones planned ahead while driving) is checked in full
    local width = cropDistance + free
    if cropDistance < self:getCropHalfWidth() or free < 0.5 then
        return false
    end
    local cropHalf = self:getCropHalfWidth()
    local gap = cropHalf + PathFinderModule.EDGE_GAP
    local radius = self.realTurnRadius or self.minTurnRadius or 8
    local step = PathFinderModule.EDGE_STEP
    local maxShift = step * math.tan(math.min(step / radius, math.rad(30)))
    local cx, _, cz = getWorldTranslation(self.pipeCombine.components[1].node)
    local closeRange = CatchCombinePipeTask ~= nil and type(CatchCombinePipeTask.CLOSE_RANGE) == "number" and CatchCombinePipeTask.CLOSE_RANGE or 30
    local points = {{x = x, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 1, z), z = z}}
    local px, pz, dx, dz = x, z, hx, hz
    local length, best, sinceBest = 0, MathUtil.vector2Length(cx - x, cz - z), 0
    local reason = "max length"
    -- the route ends where the chase can take over: close to the harvester and a clean straight line to its target
    local target = self.target or {x = cx, z = cz}
    -- corner of the crop ahead (the edge turns away, ~90 deg): swing wide to the outer side of the strip first, then
    -- turn as tight as the train can around the corner until the edge is found again (idea)
    -- anticipate more, the train runs on the crop side all along, it must be on the outer side well before
    local lookAhead = radius + 30
    local cdx, cdz, clx, clz = nil, nil, nil, nil -- strip direction when the corner was first seen
    local cornerX, cornerZ, wideV, cornerQX, cornerQZ, cornerReach = nil, nil, nil, nil, nil, nil
    local cornerEndIndex, cutCorner = nil, false
    local corner, turned, corners = nil, 0, 0
    local bendTurns = 0
    while length < PathFinderModule.EDGE_MAX_LENGTH do
        local bx, bz = px + dx * step, pz + dz * step
        local lx, lz = dz * cropSide, -dx * cropSide -- toward the crop
        local edge = self:edgeFindCrop(bx, bz, lx, lz, -2, PathFinderModule.EDGE_SCAN)
        local shift
        if corner == nil and edge ~= nil then
            -- corner ahead: the crop edge beside the strip ends within lookAhead. The strip direction is taken over
            -- the last 10 m (one step only swings a few degrees with the 0.5 m map, far ahead that is meters)
            local sdx, sdz = dx, dz
            local back = points[math.max(cornerEndIndex or 1, #points - 5)]
            local sl = MathUtil.vector2Length(px - back.x, pz - back.z)
            if sl > 4 then
                sdx, sdz = (px - back.x) / sl, (pz - back.z) / sl
            end
            local slx, slz = sdz * cropSide, -sdx * cropSide
            local toCorner = nil
            if self:edgeFindCrop(bx + sdx * lookAhead, bz + sdz * lookAhead, slx, slz, -2, PathFinderModule.EDGE_SCAN + 4) == nil then
                for k = 1, math.ceil(lookAhead) do
                    if self:edgeFindCrop(bx + sdx * k, bz + sdz * k, slx, slz, -2, PathFinderModule.EDGE_SCAN + 4) == nil then
                        toCorner = k
                        break
                    end
                end
            end
            if toCorner ~= nil then
                corners = corners + 1
                corner = "wide"
                -- corner frame: cd along the strip, cl toward the crop; the corner point lies on the crop edge
                cdx, cdz, clx, clz = sdx, sdz, slx, slz
                dx, dz, lx, lz = sdx, sdz, slx, slz
                local free = 0
                while free < PathFinderModule.EDGE_FREE_SCAN do
                    local fx, fz = bx - lx * (free + 0.5), bz - lz * (free + 0.5)
                    if not PathFinderModule.getMapOnFieldAt(fx, fz) or self:edgeCropAt(fx, fz) then
                        break
                    end
                    free = free + 0.5
                end
                cornerX, cornerZ = bx + dx * toCorner + lx * (edge - 0.5), bz + dz * toCorner + lz * (edge - 0.5)
                -- room after the corner: depth of the strip beyond it (field, no crop), measured 3 m on the far side
                local depth = 0
                while depth < 30 do
                    local fx, fz = cornerX + dx * (depth + 0.5) + lx * 3, cornerZ + dz * (depth + 0.5) + lz * 3
                    if not PathFinderModule.getMapOnFieldAt(fx, fz) or self:edgeCropAt(fx, fz) then
                        break
                    end
                    depth = depth + 0.5
                end
                cornerReach = depth - cropHalf - 0.5
                if depth < PathFinderModule.EDGE_CORNER_MIN_ROOM then
                    -- no cut ground behind the corner for the train (a narrowing gap in the crop): not a way
                    reason = "corner without room after it"
                    break
                end
                -- swing out only as much as needed for the arc to end inside that room (not more), at most to
                -- the outer border (wheels 0.5 m inside it)
                local outer = -(free - cropHalf - 1.0) - (edge - 0.5) -- tyres 1 m inside the border (map read to 0.5 m)
                -- 1 m more than the gap: the map edge is only known to 0.5 m and the train swings a little in the arc
                local needed = -gap - PathFinderModule.EDGE_CORNER_MARGIN
                if cornerReach < radius then
                    local r2 = (radius - gap) * (radius - gap) - (radius - cornerReach) * (radius - cornerReach)
                    needed = r2 >= 0 and math.min(-gap - PathFinderModule.EDGE_CORNER_MARGIN, -radius + math.sqrt(r2)) or outer
                end
                -- on the outer side, as far as the field allows (not only what the geometry needs)
                -- (no use beyond one turn radius from the crop edge: the arc has all its room there)
                wideV = math.min(math.max(outer, -radius), -gap - PathFinderModule.EDGE_CORNER_MARGIN)
            end
        end
        if corner ~= nil then
            local u = (px - cornerX) * cdx + (pz - cornerZ) * cdz
            local v = (px - cornerX) * clx + (pz - cornerZ) * clz
            if corner == "wide" then
                -- turn centre beside the train toward the crop: the crop corner must stay inside the inner tyre arc
                -- (radius - gap); start halfway between the earliest and the latest start that keep it there
                -- earliest start keeping the corner inside the inner tyre arc, so the train ends close to the new edge
                local cv = v + radius
                local inner = radius - gap - PathFinderModule.EDGE_CORNER_MARGIN / 2
                local room = inner * inner - cv * cv
                if room >= 0 and u >= math.max(-math.sqrt(room), gap - radius) then
                    corner = "turn"
                end
            end
            local maxTurn = step / radius
            local function steer(tx, tz)
                -- new heading toward tx, tz, turning at most maxTurn: the point step meters ahead
                local want = math.atan2(tx, tz)
                local cur = math.atan2(dx, dz)
                local diff = want - cur
                while diff > math.pi do diff = diff - 2 * math.pi end
                while diff < -math.pi do diff = diff + 2 * math.pi end
                diff = math.max(-maxTurn, math.min(maxTurn, diff))
                local h = cur + diff
                return px + math.sin(h) * step, pz + math.cos(h) * step, math.abs(diff)
            end
            if corner == "wide" then
                -- out to the wide line, back along the strip direction: aim 6 m ahead on the wide line
                local ax, az = cornerX + cdx * (u + 12) + clx * wideV, cornerZ + cdz * (u + 12) + clz * wideV
                -- the crop edge is not straight: keep the aim point the gap (plus margin) away from the crop there
                local e2 = self:edgeFindCrop(ax, az, clx, clz, -6, PathFinderModule.EDGE_SCAN)
                if e2 ~= nil and e2 - 0.5 < gap + PathFinderModule.EDGE_CORNER_MARGIN then
                    local back = gap + PathFinderModule.EDGE_CORNER_MARGIN - (e2 - 0.5)
                    ax, az = ax - clx * back, az - clz * back
                end
                cornerQX, cornerQZ = steer(ax - px, az - pz)
            else
                -- turn toward the crop as tight as the train can, until heading along cl (the other side of the corner)
                local d
                cornerQX, cornerQZ, d = steer(clx, clz)
                turned = turned + d
                -- the crop edge runs along the train again (beside it here and 4 m ahead): a bend, not a right angle
                local along = false
                if turned >= math.rad(PathFinderModule.EDGE_TURN_MIN_DEG) then
                    local e1 = self:edgeFindCrop(px, pz, dz * cropSide, -dx * cropSide, 0, gap + PathFinderModule.EDGE_TURN_FOUND)
                    local e2 = self:edgeFindCrop(px + dx * 4, pz + dz * 4, dz * cropSide, -dx * cropSide, 0, gap + PathFinderModule.EDGE_TURN_FOUND)
                    along = e1 ~= nil and e2 ~= nil and e1 - 0.5 >= gap - 0.5 and e2 - 0.5 >= gap - 0.5
                end
                if along or dx * clx + dz * clz > 0.995 or turned > math.rad(120) then
                    corner, turned, cutCorner = nil, 0, false
                    cornerEndIndex = #points
                    if edge == nil then
                        edge = self:edgeFindCrop(bx, bz, lx, lz, -2, PathFinderModule.EDGE_SCAN)
                    end
                    if edge ~= nil then
                        shift = math.max(-maxShift, math.min(maxShift, edge - 0.5 - gap))
                    else
                        shift = 0
                    end
                    cornerQX = nil
                end
            end
        elseif edge ~= nil then
            -- the first crop sample is up to one map sample (0.5 m) beyond the real edge: count from the sample before
            shift = math.max(-maxShift, math.min(maxShift, edge - 0.5 - gap))
            -- bend ahead with the crop on its outer side (outside, inside, outside): the edge comes closer
            -- faster than the train can turn by hugging it. Looking one turn radius ahead, the train leaves the edge
            -- in time toward the inner side of the bend, never closer to the crop meanwhile, and follows it again after
            local k = step * 2
            while k <= radius + PathFinderModule.EDGE_BEND_EXTRA do
                local ek = self:edgeFindCrop(bx + dx * k, bz + dz * k, lx, lz, -2, PathFinderModule.EDGE_SCAN)
                if ek ~= nil then
                    -- one map sample (0.5 m) of tolerance: a straight edge read on the map is not straight
                    local need = ek - 0.5 - gap + 0.5
                    if need < 0 then
                        shift = math.min(shift, 0)
                        -- side shift reachable in k meters turning at the radius, with a margin
                        if -need >= k * k / (2 * radius) * PathFinderModule.EDGE_BEND_MARGIN then
                            shift = -maxShift
                            bendTurns = bendTurns + 1
                            break
                        end
                    end
                end
                k = k + step
            end
        else
            reason = "crop edge lost"
            break
        end
        local qx, qz
        if corner ~= nil and cornerQX ~= nil then
            qx, qz, shift = cornerQX, cornerQZ, 0
        else
            qx, qz = bx + lx * shift, bz + lz * shift
        end
        cornerQX = nil
        -- the train at the tyres: no crop across, centre on the field, body side allowed, no obstacle
        local ok = PathFinderModule.getMapOnFieldAt(qx, qz)
        local why = ok and nil or "centre off the field"
        local samples = math.ceil(cropHalf * 4)
        for k = 0, samples do
            if not ok then
                break
            end
            local w = -cropHalf + 2 * cropHalf * k / samples
            for _, t in ipairs({0, 0.5}) do
                local mx, mz = qx - dx * step * t + lx * w, qz - dz * step * t + lz * w
                if self:edgeCropAt(mx, mz) then
                    if corner == "turn" and not (self.strictFruit or self.strictField) then
                        -- a corner the train cannot take is cut (crop under the inner tyres in the turn only);
                        -- strict: never, the route stops before
                        cutCorner = true
                    else
                        ok = false
                        why = string.format("crop under the tyre at %.1f,%.1f", mx, mz)
                        break
                    end
                end
            end
        end
        if ok and self.strictField then
            local side = (self.trainHalfWidth or 2) - PathFinderModule.FRUIT_CHECK_CORRIDOR_MARGIN
            ok = self:isTrainSideAllowed(qx - lx * side, qz - lz * side) and self:isTrainSideAllowed(qx + lx * side, qz + lz * side)
            if not ok then
                why = "body off the field"
            end
        end
        if ok and not self:isStretchClear({x = px, z = pz}, {x = qx, z = qz}, nil, true, true) then
            ok = false
            reason = "obstacle"
        elseif not ok then
            reason = string.format("%s, %s, point %.0f,%.0f", corner ~= nil and "corner too sharp for the train" or "strip blocked", why or "?", qx, qz)
        end
        if not ok then
            break
        end
        local sl = MathUtil.vector2Length(qx - px, qz - pz)
        dx, dz = (qx - px) / sl, (qz - pz) / sl
        px, pz = qx, qz
        length = length + sl
        points[#points + 1] = {x = qx, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, qx, 1, qz), z = qz}
        local distance = MathUtil.vector2Length(cx - qx, cz - qz)
        if distance < closeRange and corner == nil and self:isFruitStretchClear({x = qx, z = qz}, target, cropHalf) then
            reason = "harvester in reach, clean straight line"
            break
        end
        if distance < best - 0.5 or corner ~= nil then
            best, sinceBest = math.min(best, distance), 0
        else
            sinceBest = sinceBest + sl
            if sinceBest >= PathFinderModule.EDGE_NO_GAIN_LENGTH then
                reason = "no longer getting closer"
                break
            end
        end
    end
    if reason ~= "harvester in reach, clean straight line" and reason ~= "max length" then
        -- stopped short of a blocked spot: the train overruns its last point
        local cut = 0
        while #points > 2 and cut < PathFinderModule.EDGE_STOP_OVERRUN do
            local a, b = points[#points - 1], points[#points]
            cut = cut + MathUtil.vector2Length(b.x - a.x, b.z - a.z)
            table.remove(points, #points)
            length = length - MathUtil.vector2Length(b.x - a.x, b.z - a.z)
        end
        -- never a stop nose against the crop (same rule as the guided departure): the stop is put earlier on the
        -- route, where the train still has clean ground ahead
        local trimmed = 0
        while #points > 2 do
            local a, b = points[#points - 1], points[#points]
            local sl = MathUtil.vector2Length(b.x - a.x, b.z - a.z)
            if sl > 0.05 and self:isGuidedStopClear(points, (b.x - a.x) / sl, (b.z - a.z) / sl) then
                break
            end
            trimmed = trimmed + sl
            table.remove(points, #points)
            length = length - sl
        end
    end
    if length < PathFinderModule.EDGE_MIN_LENGTH then
        return false
    end
    -- (seen in game: edge pieces up to 135 m from the harvester): a piece that ends farther from the target
    -- than it starts leads away, it is refused
    if reason ~= "harvester in reach, clean straight line" then
        local lastP = points[#points]
        local startD = MathUtil.vector2Length(target.x - x, target.z - z)
        local endD = MathUtil.vector2Length(target.x - lastP.x, target.z - lastP.z)
        if endD > startD - PathFinderModule.EDGE_MIN_GAIN then
            return false
        end
    end
    self.wayPoints = points
    self.partialPath = true
    self.isFinished = true
    self.smoothDone = true
    self.dubinsDone = true
    ADScheduler:removePathfinderVehicle(self.vehicle)
    return true
end

-- forage harvester, full unloader without a clean way out while other unloaders work the field: move aside so the
-- next unloader can pass (to the harvester). Short forward curves (as the guided departure) from the real pose, or
-- from a pose a few meters back on its own track (reverse first, then forward: angle to leave the strip). The spot is
-- on the cut strip leaving PARK_PASS_MIN_WIDTH free beside the whole train, or on the grass beside the field (strict
-- exception agreed by the only to park, no obstacle under the whole train, never another field).
-- Returns waypoints and the track index to back off to first (nil: drive from here); nil, true when already out of
-- the way; nil when no spot
PathFinderModule.PARK_DISTANCES = {6, 10, 14, 18, 22, 26, 30}
PathFinderModule.PARK_LATERAL_MAX = 12 -- m sideways from the start line
PathFinderModule.PARK_LATERAL_STEP = 1
PathFinderModule.PARK_HEADING_MAX = 30
PathFinderModule.PARK_ANGLE_STEP = 15
PathFinderModule.PARK_HARVESTER_MIN_DISTANCE = 25 -- m between a parked full trailer and the harvester
PathFinderModule.PARK_PASS_MIN_WIDTH = 4 -- m free beside the parked train for another one to pass
PathFinderModule.PARK_MAX_CURVES = 40 -- per start pose
PathFinderModule.PARK_BACK_DISTANCES = {6, 10, 15, 20} -- m backed off along the own track before the forward curve
PathFinderModule.PARK_STRAIGHT = 10 -- m driven straight after the curve: the trailer lines up behind the tractor
PathFinderModule.PARK_FIELD_TOLERANCE = 0.5 -- m of the parked train that may stay on the field ground when on the grass

-- the trailer cuts inside a curve (about L^2 / 2R): its axle is followed along the path (hitch behind the
-- tractor, axle pulled toward the hitch) and its body checked for obstacles and vehicles every meter
PathFinderModule.GUIDED_TRAILER_MARGIN = 0.3
PathFinderModule.GUIDED_FRONT_ROOM = 3 -- m free ahead of the tractor nose at the stop
function PathFinderModule:isGuidedTrailerClear(path)
    local module = self.vehicle.ad.trafficYieldModule
    if module == nil or module.overlapHit == nil or #path < 3 then
        return true
    end
    local tractorLength = (self.vehicle.size ~= nil and self.vehicle.size.length) or 6
    local trailerLength = AutoDrive.getTractorTrainLength(self.vehicle, true, false) - tractorLength
    local halfW = self:getTrainWidth() / 2 + PathFinderModule.GUIDED_TRAILER_MARGIN
    local function hitchAt(i)
        local a, b = path[math.max(1, i - 1)], path[math.min(#path, i + 1)]
        local hx, hz = b.x - a.x, b.z - a.z
        local len = math.sqrt(hx * hx + hz * hz)
        if len < 0.01 then
            return nil
        end
        hx, hz = hx / len, hz / len
        return path[i].x - hx * tractorLength / 2, path[i].z - hz * tractorLength / 2, hx, hz
    end
    -- the whole tractor body (its nose reaches well ahead of the path point) and the room ahead of the stop
    -- (the curve stopped nose against a waiting tractor, only its centre had been checked)
    local size = self.vehicle.size or {}
    local halfL = (size.length or 6) / 2
    local offset = size.lengthOffset or 0
    local name = self:getLogName()
    local strictField = AutoDrive.getSetting("restrictToField", self.vehicle) == "strict"
    local indices = {}
    for i = PathFinderModule.GUIDED_START_TOLERANCE + 2, #path - 1, 2 do
        indices[#indices + 1] = i
    end
    indices[#indices + 1] = #path
    for _, i in ipairs(indices) do
        local _, _, dx, dz = hitchAt(i)
        if dx ~= nil then
            local cx, cz = path[i].x + dx * offset, path[i].z + dz * offset
            local ahead = 0
            if i == #path then
                ahead = PathFinderModule.GUIDED_FRONT_ROOM
            end
            cx, cz = cx + dx * ahead / 2, cz + dz * ahead / 2
            -- the nose and its corners stay on the field ground (nose on the grass against the wall)
            if strictField then
                local nose = halfL + ahead / 2
                for _, w in ipairs({-halfW, 0, halfW}) do
                    local nx, nz = cx + dx * nose - dz * w, cz + dz * nose + dx * w
                    if not PathFinderModule.getMapOnFieldAt(nx, nz) then
                        return false
                    end
                end
            end
            local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, cx, 0, cz)
            if module:overlapHit(cx, y + 2, cz, math.atan2(dx, dz), halfW, 1.5, halfL + ahead / 2) then
                return false
            end
        end
    end
    local hx0, hz0, dx0, dz0 = hitchAt(1)
    if hx0 == nil or trailerLength < 3 then
        return true
    end
    local tx, tz = hx0 - dx0 * trailerLength, hz0 - dz0 * trailerLength
    for i = 2, #path do
        local hx, hz = hitchAt(i)
        if hx ~= nil then
            local vx, vz = hx - tx, hz - tz
            local len = math.sqrt(vx * vx + vz * vz)
            if len > 0.01 then
                tx, tz = hx - vx / len * trailerLength, hz - vz / len * trailerLength
                -- the first meters are where the train already stands
                if i > PathFinderModule.GUIDED_START_TOLERANCE + 1 then
                    local cx, cz = (hx + tx) / 2, (hz + tz) / 2
                    local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, cx, 0, cz)
                    if module:overlapHit(cx, y + 2, cz, math.atan2(vx, vz), halfW, 1.5, trailerLength / 2) then
                        return false
                    end
                end
            end
        end
    end
    return true
end

-- footprint of a train whose tractor stands at x, z heading hx, hz: centre and half sizes
function PathFinderModule:getParkFootprint(x, z, hx, hz)
    local trainLength = math.max(AutoDrive.getTractorTrainLength(self.vehicle, true, false), 8)
    local halfW = self:getTrainWidth() / 2 + 0.5
    return x - hx * trainLength / 2, z - hz * trainLength / 2, halfW, trainLength / 2 + 0.5
end

-- free cut ground beside the whole train (m); a train standing entirely on the grass leaves the strip free
function PathFinderModule:getPassWidth(x, z, hx, hz)
    local cx, cz, halfW, halfL = self:getParkFootprint(x, z, hx, hz)
    if self:isFootprintOffField(cx, cz, hx, hz, halfW - PathFinderModule.PARK_FIELD_TOLERANCE, halfL) then
        return math.huge, true
    end
    return PathFinderModule.getFreeWidthBeside(cx, cz, hx, hz, halfW, halfL), false
end

PathFinderModule.PASS_SEARCH = 12 -- m looked at beside a parked train for the free cut ground

-- free cut field ground beside a train footprint (centre cx, cz, direction dx, dz), widest side, in m
function PathFinderModule.getFreeWidthBeside(cx, cz, dx, dz, halfW, halfL)
    local best = 0
    for _, side in ipairs({1, -1}) do
        local nx, nz = dz * side, -dx * side
        local width = math.huge
        -- every 2 m from the rear of the trailer to the nose of the tractor: the narrowest place counts
        local steps = math.max(2, math.ceil(halfL * 2 / 2))
        for k = 0, steps do
            local f = -1 + 2 * k / steps
            local px, pz = cx + dx * halfL * f, cz + dz * halfL * f
            local w = 0
            local o = halfW + 0.5
            while o <= halfW + PathFinderModule.PASS_SEARCH do
                local x, z = px + nx * o, pz + nz * o
                local crop = PathFinderModule.getMapCropAt(x, z)
                if (crop ~= nil and crop ~= 0) or not PathFinderModule.getMapOnFieldAt(x, z) then
                    break
                end
                w = o - halfW
                o = o + 0.5
            end
            width = math.min(width, w)
        end
        best = math.max(best, width)
    end
    return best
end

-- samples every ~1 m across a footprint, edges included: fn(x, z) returns true to stop
function PathFinderModule.forEachFootprintSample(cx, cz, hx, hz, halfW, halfL, fn)
    local nl = math.max(1, math.ceil(halfL * 2))
    local nw = math.max(1, math.ceil(halfW * 2))
    for i = 0, nl do
        local l = -halfL + 2 * halfL * i / nl
        for j = 0, nw do
            local w = -halfW + 2 * halfW * j / nw
            if fn(cx + hx * l + hz * w, cz + hz * l - hx * w) then
                return true
            end
        end
    end
    return false
end

-- every 1 m sample of the footprint off the field ground (grass)
function PathFinderModule:isFootprintOffField(cx, cz, hx, hz, halfW, halfL)
    return not PathFinderModule.forEachFootprintSample(cx, cz, hx, hz, halfW, halfL, function(x, z)
        return PathFinderModule.getMapOnFieldAt(x, z)
    end)
end

-- the whole parked train: no obstacle (trees, walls, fences, vehicles), no other field, no crop
function PathFinderModule:isParkFootprintClear(x, z, hx, hz)
    local cx, cz, halfW, halfL = self:getParkFootprint(x, z, hx, hz)
    local bad = PathFinderModule.forEachFootprintSample(cx, cz, hx, hz, halfW, halfL, function(px, pz)
        local crop = PathFinderModule.getMapCropAt(px, pz)
        if crop ~= nil and crop ~= 0 then
            return true
        end
        -- another field
        return PathFinderModule.getMapOnFieldAt(px, pz) and not self:isFieldPositionAllowed(px, pz, true)
    end)
    if bad then
        return false
    end
    local module = self.vehicle.ad.trafficYieldModule
    if module ~= nil and module.overlapHit ~= nil then
        local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, cx, 0, cz)
        if module:overlapHit(cx, y + 2, cz, math.atan2(hx, hz), halfW, 1.5, halfL) then
            return false
        end
    end
    return true
end

-- one forward curve from a pose to a parking spot; waypoints or nil
function PathFinderModule:tryParkCurve(px, pz, heading, cand, radius)
    local hx, hz = math.sin(cand.heading), math.cos(cand.heading)
    local q0 = {px, -pz, AutoDrive.normalizeAngle(heading + math.pi + math.pi / 2)}
    local q1 = {cand.x, -cand.z, AutoDrive.normalizeAngle(cand.heading + math.pi + math.pi / 2)}
    self.dubins.outPath = {}
    if self.dubins:dubins_shortest_path(ADDubins.DubinsPath, q0, q1, radius) ~= ADDubins.EDUBOK
        or self.dubins:dubins_path_length(ADDubins.DubinsPath) > cand.d * 1.6 + 2 then
        return nil, false
    end
    if self.dubins:dubins_path_sample_many(ADDubins.DubinsPath, 1, self.dubins.createWayPoints) ~= ADDubins.EDUBOK or #self.dubins.outPath <= 2 then
        return nil, true
    end
    local path = self.dubins.outPath
    path[#path + 1] = {x = cand.x, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, cand.x, 1, cand.z), z = cand.z, t = q1[3]}
    -- then straight on to the stop: the trailer lines up behind the tractor (the spot is checked for a straight train)
    local straight = PathFinderModule.PARK_STRAIGHT
    for m = 1, straight do
        local x, z = cand.x + hx * m, cand.z + hz * m
        path[#path + 1] = {x = x, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 1, z), z = z, t = q1[3]}
    end
    if not self:isGuidedBandClear(path, radius, 0) then
        return nil, true
    end
    local cells = {}
    for i, wp in ipairs(path) do
        local cell = {x = i, z = 0, t = wp.t, height = self.vehicleMinHeight or 3}
        cell.worldPos = {x = wp.x, y = wp.y, z = wp.z}
        cell.incoming = cells[i - 5]
        cells[i] = cell
        if i > PathFinderModule.GUIDED_START_TOLERANCE + 1 and not self:isDriveableDubins(cell) then
            return nil, true
        end
    end
    local points = {}
    for i, wp in ipairs(path) do
        if i == 1 or i == #path or (i - 1) % PathFinderModule.GUIDED_POINT_SPACING == 0 then
            points[#points + 1] = {x = wp.x, y = wp.y, z = wp.z}
        end
    end
    return points, true
end

-- fromHere: after backing off along its track, a real spot is wanted (grass or side of the strip), from where
-- the train stands only, even if the lane beside it is already wide enough
function PathFinderModule:findParkAside(harvester, fromHere)
    if not self:isChopperWork() then
        return nil
    end
    local name = self:getLogName()
    self.trainHalfWidth = self.trainHalfWidth or (self:getTrainWidth() / 2 + PathFinderModule.FRUIT_CHECK_CORRIDOR_MARGIN)
    self:updateCropHalfWidth()
    local radius = self.realTurnRadius or self.minTurnRadius or 10
    local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
    local rx, _, rz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local savedGoing, savedCombine = self.goingToPipe, self.pipeCombine
    self.goingToPipe, self.pipeCombine = harvester ~= nil, harvester
    local here = self:getPassWidth(vx, vz, rx, rz)
    local hereNear = false
    if harvester ~= nil and harvester.components ~= nil then
        local hvx, _, hvz = getWorldTranslation(harvester.components[1].node)
        hereNear = MathUtil.vector2Length(vx - hvx, vz - hvz) < PathFinderModule.PARK_HARVESTER_MIN_DISTANCE
    end
    if not hereNear and (here == math.huge or not fromHere) and here >= PathFinderModule.PARK_PASS_MIN_WIDTH and not self:isGuidedStopInHarvesterWay(vx, vz, math.atan2(rx, rz)) then
        self.goingToPipe, self.pipeCombine = savedGoing, savedCombine
        return nil, true
    end
    -- start poses: here, then a few meters back on the own track
    local poses = {{x = vx, z = vz, heading = math.atan2(rx, rz), back = 0}}
    local mode = self.vehicle.ad.modes ~= nil and self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD] or nil
    local track = mode ~= nil and mode.track or nil
    if track ~= nil and #track > 2 and not fromHere then
        local walked = MathUtil.vector2Length(track[#track].x - vx, track[#track].z - vz)
        local wanted = 1
        for k = #track - 1, 2, -1 do
            walked = walked + MathUtil.vector2Length(track[k + 1].x - track[k].x, track[k + 1].z - track[k].z)
            local d = PathFinderModule.PARK_BACK_DISTANCES[wanted]
            if d == nil then
                break
            end
            if walked >= d then
                local dx, dz = track[k].x - track[k - 1].x, track[k].z - track[k - 1].z
                if dx * dx + dz * dz > 0.01 then
                    poses[#poses + 1] = {x = track[k].x, z = track[k].z, heading = math.atan2(dx, dz), back = walked, index = k}
                end
                wanted = wanted + 1
            end
        end
    end
    self.parkOnGrass = true
    local inWay, narrow, blocked, checked = 0, 0, 0, 0
    local result, resultIndex, info = nil, nil, nil
    for _, pose in ipairs(poses) do
        local candidates = {}
        local fx, fz = math.sin(pose.heading), math.cos(pose.heading)
        for _, d in ipairs(PathFinderModule.PARK_DISTANCES) do
            -- spots d meters ahead, shifted sideways every meter (strips run along the border)
            for lateral = -PathFinderModule.PARK_LATERAL_MAX, PathFinderModule.PARK_LATERAL_MAX, PathFinderModule.PARK_LATERAL_STEP do
                local x, z = pose.x + fx * d + fz * lateral, pose.z + fz * d - fx * lateral
                for h = -PathFinderModule.PARK_HEADING_MAX, PathFinderModule.PARK_HEADING_MAX, PathFinderModule.PARK_ANGLE_STEP do
                    local endHeading = pose.heading + math.rad(h)
                    local hx, hz = math.sin(endHeading), math.cos(endHeading)
                    -- where the tractor stops, after the straight part
                    local sx, sz = x + hx * PathFinderModule.PARK_STRAIGHT, z + hz * PathFinderModule.PARK_STRAIGHT
                    -- a train standing on the grass is out of everybody's way (they all drive on the field)
                    local pass, onGrass = self:getPassWidth(sx, sz, hx, hz)
                    -- never next to the harvester, grass included: Courseplay would take the full trailer for its
                    -- unloader and wait for it (the whole train, tractor and trailer, kept away)
                    local nearHarvester = false
                    if harvester ~= nil and harvester.components ~= nil then
                        local hvx, _, hvz = getWorldTranslation(harvester.components[1].node)
                        local trainLength = math.max(AutoDrive.getTractorTrainLength(self.vehicle, true, false), 8)
                        for _, back in ipairs({0, trainLength / 2, trainLength}) do
                            if MathUtil.vector2Length(sx - hx * back - hvx, sz - hz * back - hvz) < PathFinderModule.PARK_HARVESTER_MIN_DISTANCE then
                                nearHarvester = true
                            end
                        end
                    end
                    if nearHarvester or (not onGrass and self:isGuidedStopInHarvesterWay(sx, sz, endHeading)) then
                        inWay = inWay + 1
                    else
                        if pass < PathFinderModule.PARK_PASS_MIN_WIDTH then
                            narrow = narrow + 1
                        else
                            -- grass first (whole strip left free), then the widest lane, short manoeuvres
                            local score = (onGrass and 100 or math.min(pass, 20)) - d * 0.05 - math.abs(h) * 0.01 - pose.back * 0.2
                            candidates[#candidates + 1] = {x = x, z = z, stopX = sx, stopZ = sz, d = d, heading = endHeading, score = score, onGrass = onGrass, pass = pass}
                        end
                    end
                end
            end
        end
        table.sort(candidates, function(c1, c2) return c1.score > c2.score end)
        local checkedHere = 0
        for _, cand in ipairs(candidates) do
            if checkedHere >= PathFinderModule.PARK_MAX_CURVES then
                break
            end
            local points, tried = nil, false
            -- the whole parked train (obstacles, crop, other field), only for the spots actually tried
            if self:isParkFootprintClear(cand.stopX, cand.stopZ, math.sin(cand.heading), math.cos(cand.heading)) then
                points, tried = self:tryParkCurve(pose.x, pose.z, pose.heading, cand, radius)
            else
                blocked = blocked + 1
            end
            if tried then
                checked = checked + 1
                checkedHere = checkedHere + 1
            end
            if points ~= nil then
                result, resultIndex = points, pose.index
                info = string.format("%s, back %.0f m then %.0f m forward (%d m straight) to %.0f,%.0f", cand.onGrass and "on the grass" or string.format("%.1f m free beside", cand.pass), pose.back, cand.d + PathFinderModule.PARK_STRAIGHT, PathFinderModule.PARK_STRAIGHT, cand.stopX, cand.stopZ)
                break
            end
        end
        if result ~= nil then
            break
        end
    end
    self.parkOnGrass = false
    self.goingToPipe, self.pipeCombine = savedGoing, savedCombine
    if result ~= nil then
        return result, false, resultIndex
    end
    return nil
end

function PathFinderModule:keepPartialStopMargin()
    local wps = self.wayPoints
    local saved = {}
    for i = 1, #wps do
        saved[i] = wps[i]
    end
    local removed = 0
    local margin = PathFinderModule.PARTIAL_STOP_MARGIN
    if self:isChopperWork() then
        -- forage harvester: drive PARTIAL_DRIVE_SHARE of the planned piece (the next piece is planned meanwhile),
        -- the stop itself keeps STOP_OVERRUN of clean ground ahead (isPartialStopClear)
        local length = 0
        for i = 2, #wps do
            length = length + MathUtil.vector2Length(wps[i].x - wps[i - 1].x, wps[i].z - wps[i - 1].z)
        end
        margin = length * (1 - PathFinderModule.PARTIAL_DRIVE_SHARE)
    end
    while #wps > 2 do
        local last, before = wps[#wps], wps[#wps - 1]
        if removed >= margin and self:isPartialStopClear(#wps) then
            break
        end
        removed = removed + MathUtil.vector2Length(last.x - before.x, last.z - before.z)
        table.remove(wps, #wps)
    end
        PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: partial path too short with the margin, waiting")
    if (#wps < 2 or self:partialGain(wps[#wps]) < PathFinderModule.PARTIAL_MIN_GAIN) and not self:isChopperWork() then
        return false
    end
    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: partial path stopped %.0f m earlier (margin), at %.0f,%.0f", removed, wps[#wps].x, wps[#wps].z)
    if #wps < 2 or self:partialGain(wps[#wps]) < PathFinderModule.PARTIAL_MIN_GAIN then
        -- too short with the full margin: smaller margin, as long as the stopped train touches no crop
        for i = #wps + 1, #saved do
            wps[i] = saved[i]
        end
        removed = 0
        while #wps > 2 and not self:isPartialStopClear(#wps) do
            local last, before = wps[#wps], wps[#wps - 1]
            removed = removed + MathUtil.vector2Length(last.x - before.x, last.z - before.z)
            table.remove(wps, #wps)
        end
        if #wps < 2 or not self:isPartialStopClear(#wps) or self:partialGain(wps[#wps]) < PathFinderModule.PARTIAL_SHORT_MIN_GAIN then
            return false
        end
        return true
    end
    return true
end

-- behind the harvester: on its own cut track, the chase takes over from there
PathFinderModule.REAR_FALLBACK_GAP = 8
PathFinderModule.REAR_FIRST_DISTANCE = 40

-- working direction of a harvester (recognise it): its front faces the standing crop, its back the cut ground.
-- The node direction is checked against the field map: crop on the back side only means the node points backwards
PathFinderModule.WORK_DIR_SCAN = {2, 4, 6, 8} -- m beyond its half length
function PathFinderModule:getWorkingDirection(combine)
    local x, _, z = getWorldTranslation(combine.components[1].node)
    local rx, _, rz = AutoDrive.localDirectionToWorld(combine, 0, 0, 1, combine.ad ~= nil and combine.ad.ADRootNode or nil)
    local half = (combine.size ~= nil and combine.size.length or 10) / 2
    local function cropCount(sign)
        local n = 0
        for _, d in ipairs(PathFinderModule.WORK_DIR_SCAN) do
            for _, w in ipairs({-2, 0, 2}) do
                local px, pz = x + rx * sign * (half + d) - rz * w, z + rz * sign * (half + d) + rx * w
                local crop = PathFinderModule.getMapCropAt(px, pz)
                if crop ~= nil and crop ~= 0 then
                    n = n + 1
                end
            end
        end
        return n
    end
    -- during a Courseplay turn (end of row) the crop around it says nothing about its direction: node direction
    if AutoDrive.combineIsTurning(combine) then
        return rx, rz
    end
    local ahead, behind = cropCount(1), cropCount(-1)
    if behind > ahead then
        return -rx, -rz
    end
    return rx, rz
end

-- fixed obstacle (hedge, trees, wall, learned blocked zone) in a slice of the train's width at x, z along hx, hz
-- (the place behind a harvester standing with its back to a hedge was in the hedge)
PathFinderModule.REAR_PROBE_HALF_LENGTH = 1 -- m
function PathFinderModule:isRearPointObstructed(x, z, hx, hz)
    if g_currentMission == nil or g_currentMission.terrainRootNode == nil or FollowCombineTask == nil or overlapBox == nil then
        return false
    end
    local halfWidth = self:getTrainWidth() / 2 + 0.3
    local height = PathFinderModule.getTrainHeight(self.vehicle) + PathFinderModule.OBSTACLE_HEIGHT_MARGIN
    local angleRad = AutoDrive.normalizeAngle(math.atan2(-hz, hx))
    local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 1, z)
    local probe = {hits = 0, callback = FollowCombineTask.lineCollisionCallback}
    overlapBox(x, y + height / 2, z, 0, angleRad, 0, PathFinderModule.REAR_PROBE_HALF_LENGTH, height / 2, halfWidth, "callback", probe, AutoDrive.collisionMaskTerrain, true, true, true, true)
    if probe.hits > 0 then
        return true
    end
    return self:boxHitsBlockedZone(x, z, angleRad, PathFinderModule.REAR_PROBE_HALF_LENGTH, halfWidth)
end

-- the whole train standing at x, z heading hx, hz (tractor at the point, the trailer behind it): no fixed obstacle
function PathFinderModule:isRearTrainBoxFree(x, z, hx, hz)
    if g_currentMission == nil or g_currentMission.terrainRootNode == nil or FollowCombineTask == nil or overlapBox == nil then
        return true
    end
    local length = math.max(6, AutoDrive.getTractorTrainLength(self.vehicle, true, false))
    local halfWidth = self:getTrainWidth() / 2 + 0.3
    local height = PathFinderModule.getTrainHeight(self.vehicle) + PathFinderModule.OBSTACLE_HEIGHT_MARGIN
    local angleRad = AutoDrive.normalizeAngle(math.atan2(-hz, hx))
    local cx, cz = x - hx * (length / 2 - 3), z - hz * (length / 2 - 3)
    local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, cx, 1, cz)
    local probe = {hits = 0, callback = FollowCombineTask.lineCollisionCallback}
    overlapBox(cx, y + height / 2, cz, 0, angleRad, 0, length / 2, height / 2, halfWidth, "callback", probe, AutoDrive.collisionMaskTerrain, true, true, true, true)
    if probe.hits > 0 then
        return false
    end
    return not self:boxHitsBlockedZone(cx, cz, angleRad, length / 2, halfWidth)
end

-- the train fits at x, z: centre on the field, no crop over its half width
function PathFinderModule:isRearPointFree(x, z, half)
    if PathFinderModule.getMapOnFieldAt(x, z) ~= true then
        return false
    end
    local n = math.ceil(half / 0.5)
    for j = -n, n do
        for i = -n, n do
            local dx, dz = i * 0.5, j * 0.5
            if dx * dx + dz * dz <= half * half + 0.001 then
                local crop = PathFinderModule.getMapCropAt(x + dx, z + dz)
                if crop ~= nil and crop ~= 0 then
                    return false
                end
            end
        end
    end
    return true
end

-- the point behind a forage harvester on the track it really drove (in a curve the straight line
-- behind it falls in the crop). Its recorded track first, then the cut ground behind it meter by meter.
-- Returns x, z, the harvester direction there, the distance along the track and where it comes from; nil if none
PathFinderModule.EDGE_MIN_GAIN = 2 -- m closer to the target at least at the end of an edge piece
PathFinderModule.REAR_TRAIL_SEARCH = 25 -- m further back when the point is in crop
PathFinderModule.REAR_WALK_ANGLES = {0, 10, -10, 20, -20, 30, -30, 45, -45}
PathFinderModule.REAR_TRAIL_MAX_COS = math.cos(math.rad(45))
-- forage harvester standing (full, it waits): its trail counts only while it stays on its heading axis. Stopped right
-- after its U-turn the last 20 m of trail are the end of the arc (an unloader was sent 20 m back on
-- that arc, 13 m beside the axis and 41 deg off, arrived on the wrong side and planned the same point for ever)
PathFinderModule.REAR_TRAIL_MAX_COS_STANDING = math.cos(math.rad(15))
PathFinderModule.REAR_TRAIL_MAX_LATERAL_STANDING = 3 -- m from its heading axis
PathFinderModule.REAR_WALK_ANGLES_STANDING = {0, 10, -10} -- deg from its heading axis, not from the previous step
PathFinderModule.STANDING_SPEED = 0.0003 -- lastSpeedReal (m/ms) under which the harvester stands
function PathFinderModule.isCombineStanding(combine)
    if combine == nil then
        return false
    end
    if math.abs(combine.lastSpeedReal or 0) < PathFinderModule.STANDING_SPEED then
        return true
    end
    return combine.ad ~= nil and combine.ad.noMovementTimer ~= nil and (combine.ad.noMovementTimer.elapsedTime or 0) > 1000
end
function PathFinderModule:getRearTarget(combine, back, rx, rz)
    local x, _, z = getWorldTranslation(combine.components[1].node)
    local half = self:getCropHalfWidth()
    local maxD = back + PathFinderModule.REAR_TRAIL_SEARCH
    local points = {}
    local px, pz, d, hx, hz = x, z, 0, rx, rz
    local trail = combine.ad ~= nil and combine.ad.chopperTrail or {}
    local fromTrail = 0
    local standing = PathFinderModule.isCombineStanding(combine)
    local maxCos = standing and PathFinderModule.REAR_TRAIL_MAX_COS_STANDING or PathFinderModule.REAR_TRAIL_MAX_COS
    local trailCut = false
    local first = #trail
    while first > 0 and (trail[first].x - x) * rx + (trail[first].z - z) * rz > -1 do
        first = first - 1
    end
    for i = first, 1, -1 do
        local t = trail[i]
        local seg = MathUtil.vector2Length(px - t.x, pz - t.z)
        if seg > 0.01 then
            local sx, sz = (px - t.x) / seg, (pz - t.z) / seg
            -- the trail before its last turn (end-of-row U-turn): not the row it works now, stop there
            -- standing: off its heading axis is the end of its U-turn arc, stop there, straight on behind
            if sx * rx + sz * rz < maxCos
                or (standing and math.abs((t.x - x) * rz - (t.z - z) * rx) > PathFinderModule.REAR_TRAIL_MAX_LATERAL_STANDING) then
                if standing then
                    trailCut = true
                    hx, hz = rx, rz
                end
                break
            end
            hx, hz = sx, sz
            local n = math.ceil(seg)
            for k = 1, n do
                local f = k / n
                points[#points + 1] = {x = px + (t.x - px) * f, z = pz + (t.z - pz) * f, hx = hx, hz = hz, d = d + seg * f}
            end
            d = d + seg
            px, pz = t.x, t.z
            fromTrail = d
            if d >= maxD then
                break
            end
        end
    end
    -- short track (just started, game loaded): on over the cut ground behind
    -- standing harvester: the walk keeps its heading axis, no curve back onto its U-turn arc
    local angles = standing and PathFinderModule.REAR_WALK_ANGLES_STANDING or PathFinderModule.REAR_WALK_ANGLES
    -- the walk stops at the first fixed obstacle (hedge, trees, wall), nothing is aimed beyond it
    local obstacleD, offFieldD = nil, nil
    while d < maxD do
        local found, obstacle, offField = false, false, false
        local bhx, bhz = hx, hz
        if standing then
            bhx, bhz = rx, rz
        end
        for _, a in ipairs(angles) do
            local c, s = math.cos(math.rad(a)), math.sin(math.rad(a))
            local bx, bz = -bhx * c - -bhz * s, -bhx * s + -bhz * c
            local qx, qz = px + bx, pz + bz
            if self:isRearPointFree(qx, qz, half) then
                if self:isRearPointObstructed(qx, qz, -bx, -bz) then
                    obstacle = true
                else
                    hx, hz = -bx, -bz
                    px, pz, d = qx, qz, d + 1
                    points[#points + 1] = {x = px, z = pz, hx = hx, hz = hz, d = d}
                    found = true
                    break
                end
            elseif a == 0 and PathFinderModule.getMapOnFieldAt(qx, qz) ~= true then
                offField = true
            end
        end
        if not found then
            if obstacle then
                obstacleD = d + 1
            elseif offField then
                offFieldD = d + 1
            end
            break
        end
    end
    local minD = (combine.size ~= nil and combine.size.length or 10) / 2 + 4
    local best = nil
    for _, p in ipairs(points) do
        if p.d >= back and self:isRearPointFree(p.x, p.z, half) and self:isRearTrainBoxFree(p.x, p.z, p.hx, p.hz) then
            best = p
            break
        end
    end
    if best == nil then
        for i = #points, 1, -1 do
            local p = points[i]
            if p.d < back and p.d >= minD and self:isRearPointFree(p.x, p.z, half) and self:isRearTrainBoxFree(p.x, p.z, p.hx, p.hz) then
                best = p
                break
            end
        end
    end
    local limit = nil
    if obstacleD ~= nil then
        limit = string.format("obstacle %.0f m behind it", obstacleD)
    elseif offFieldD ~= nil then
        limit = string.format("field edge %.0f m behind it", offFieldD)
    end
    if best == nil then
        return nil, nil, nil, nil, nil, limit or string.format("no free point over %.0f m (crop or train in the way)", d)
    end
    local source = fromTrail >= best.d and "on its trail" or (fromTrail > 0 and "trail then cut ground" or "cut ground behind it")
    if trailCut then
        source = source .. " (harvester standing, its trail ends in a turn: on its heading axis)"
    end
    if best.d < back and limit ~= nil then
        source = source .. string.format(" (%s, closer place)", limit)
    end
    return best.x, best.z, best.hx, best.hz, best.d, source
end

PathFinderModule.TRAIL_ON_DISTANCE = 4
function PathFinderModule.getTrailDirectionAt(combine, x, z)
    local trail = combine.ad ~= nil and combine.ad.chopperTrail or nil
    if trail == nil or #trail < 2 then
        return nil
    end
    local best, bestD = nil, PathFinderModule.TRAIL_ON_DISTANCE
    for i = 2, #trail do
        local d = MathUtil.vector2Length(trail[i].x - x, trail[i].z - z)
        if d < bestD then
            best, bestD = i, d
        end
    end
    if best == nil then
        return nil
    end
    local a, b = trail[best - 1], trail[best]
    local l = MathUtil.vector2Length(b.x - a.x, b.z - a.z)
    if l < 0.01 then
        return nil
    end
    return (b.x - a.x) / l, (b.z - a.z) / l
end

PathFinderModule.REAR_UTURN_EXTRA = 5 -- m
PathFinderModule.ANTICIPATED_MAX_AGE = 60000 -- ms
function PathFinderModule:startRearFallback()
    local combine = self.pipeCombine
    if self.rearFallbackDone or not self.goingToPipe or combine == nil or combine.components == nil or not (self.strictFruit or self.strictField) then
        return false
    end
    -- only to catch a harvester that drives: behind a stopped one the trailer cannot reach the pipe, nothing takes over there
    local stopped = (combine.lastSpeedReal or 0) < 0.0002
        or (combine.ad ~= nil and combine.ad.noMovementTimer ~= nil and combine.ad.noMovementTimer.elapsedTime > 1000)
    -- forage harvester: behind it even when it stands, it drives on once its unloader is there
    if stopped and not (combine.ad ~= nil and combine.ad.isChopper) then
        return false
    end
    local x, y, z = getWorldTranslation(combine.components[1].node)
    local rx, rz = self:getWorkingDirection(combine)
    local back = (combine.size ~= nil and combine.size.length or 10) / 2 + PathFinderModule.REAR_FALLBACK_GAP + AutoDrive.getTractorTrainLength(self.vehicle, true, false) / 2
    local wrongWayNeed = nil
    -- tractor facing the other way (after the harvester's end-of-row U-turn): the target leaves room for
    -- its own U-turn beyond where it stands, so it arrives the right way round the first time
    if combine.ad ~= nil and combine.ad.isChopper then
        -- from where this piece starts (end of the piece being driven, the next piece began 12 m from the
        -- target and had no room to turn)
        local vx, _, vz, hx, hz = self:getStartPose()
        if hx * rx + hz * rz < 0 then
            local behind = (x - vx) * rx + (z - vz) * rz
            local radius = AutoDrive.getDriverRadius(self.vehicle) or 8
            -- more room each time the train arrived the wrong way round at the same place (CatchCombinePipeTask.tryWrongWayTurn)
            local extra = self.vehicle.ad.turnRoomExtra or 0
            local need = behind + 2 * radius + PathFinderModule.REAR_UTURN_EXTRA + extra
            if need > back then
                back = need
                wrongWayNeed = need
            end
        end
    end
    local target = {x = x - rx * back, y = y, z = z - rz * back}
    local arrive = {x = rx, z = rz}
    local where = "straight line (no free point on its trail)"
    -- end-of-row U-turn anticipated: behind the entry of the next row read on the Courseplay course,
    -- the nearest free place on the cut ground going back from the wanted distance
    local anticipated = false
    local ant = self.vehicle.ad.anticipatedRear
    if ant ~= nil and (g_time or 0) - ant.t < PathFinderModule.ANTICIPATED_MAX_AGE and combine.ad ~= nil and combine.ad.isChopper then
        local half = self:getCropHalfWidth()
        local minD = (combine.size ~= nil and combine.size.length or 10) / 2 + 4
        for d = ant.back, minD, -1 do
            local px, pz = ant.ex - ant.dx * d, ant.ez - ant.dz * d
            if self:isRearPointFree(px, pz, half) then
                target = {x = px, y = y, z = pz}
                arrive = {x = ant.dx, z = ant.dz}
                back = d
                where = string.format("anticipated U-turn, %.0f m behind the entry of the new row", d)
                anticipated = true
                break
            end
        end
        if not anticipated then
            self.vehicle.ad.anticipatedRear = nil
        end
    end
    if combine.ad ~= nil and combine.ad.isChopper and not anticipated then
        local tx, tz, thx, thz, td, source = self:getRearTarget(combine, back, rx, rz)
        -- behind a harvester standing with its back to a hedge there
        -- is no place. An unloader aimed 20 m behind it, in the hedge, for 8 minutes: without a place the side place is aimed
        if tx == nil then
            self.rearPlaceNone = true
            return false
        end
        -- facing the other way with less room behind her than the U-turn needs: not aimed either (the turn comes first)
        if wrongWayNeed ~= nil and td < wrongWayNeed - 2 then
            self.rearPlaceNone = true
            return false
        end
        self.rearPlaceNone = nil
        target = {x = tx, y = y, z = tz}
        arrive = {x = thx, z = thz}
        where = string.format("%s, %.0f m", source, td)
    end
    local allowPartial = self.allowPartialPath
    local fruitToCheck = self.fruitToCheck
    self:startPathPlanningTo(target, arrive)
    -- behind the harvester is cut ground: no crop allowed near the target, the whole route judged
    self.goingBehind = true
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
    -- a target inside an obstacle (hedge) is never approached piece by piece (an unloader crept toward the
    -- hedge, route after route). The target itself must be free of fixed obstacles
    if self.target ~= nil and self.targetVector ~= nil and self:isRearPointObstructed(self.target.x, self.target.z, self.targetVector.x, self.targetVector.z) then
        return false
    end
    local best, bestDistance = nil, math.huge
    for node, _ in pairs(self.closedset) do
        if node ~= self.nodeBehindStart and self.came_from[node] ~= nil then
            local distance = MathUtil.vector2Length(node.x - self.nodeGoal.x, node.z - self.nodeGoal.z)
            local band = self:getBandDistanceAtNode(node)
            if band ~= nil then
                -- in grid cells, as the straight distance; a node out of the strips is never the best
                distance = band ~= false and band / self.minTurnRadius or math.huge
            end
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
PathFinderModule.START_CROP_AHEAD = 1 -- m ahead of the tractor node where crop may still be (its own front)
PathFinderModule.VERIFY_CROP_START = 3 -- forage harvester, strict: crop judged from here on (12 m before)
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
    if self:isChopperWork() and (self.strictFruit or self.strictField) then
        -- strict: no turn through the crop at the start either, only the train's own place is left out
        skipFrom = PathFinderModule.VERIFY_CROP_START
    end
    if self.partialPath or self.goingBehind then
        -- a partial route stops before the target: its end is where the vehicle waits, it is judged too;
        -- behind the harvester is cut ground, nothing to leave out there
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
        -- exit to the network: the last meters are judged for obstacles too. A wall or a post
        -- between the field and the road is thinner than a cell, the coarse grid drives through it
        if self.exitToNetwork then
            local endFrom = math.max(0, skipTo - done, startTo)
            if length - endFrom > 0.5 then
                local pa = {x = a.x + dx * endFrom / length, z = a.z + dz * endFrom / length}
                if not self:isStretchClear(pa, b, nil, true, true) then
                    return i, "obstacle (approach)"
                end
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
-- forage harvester (choice): a corner kept up to this angle is driven, the train may cut it
PathFinderModule.CORNER_SHARP_ANGLE_CHOPPER = math.rad(60)

-- forage harvester, strict: a corner without room on the route line is driven as a driver would, outside, inside,
-- outside (rule): before a right turn the train first swings left, cuts the corner at the turn radius, ends
-- on the left of the next line and comes back to it. The swing angles before and after are tried from the smallest.
PathFinderModule.WIDE_SWING_ANGLES = {0, 8, 16, 24} -- deg away from the turn
PathFinderModule.WIDE_SWING_LEADS = {6, 12} -- m of the line used for the swing, before and after the normal arc
PathFinderModule.WIDE_MAX_ANGLE = math.rad(170) -- whole turn of the wide arc
PathFinderModule.WIDE_MAX_CHECKS = 40 -- full checks (obstacles) per corner, keeps the frame time low
PathFinderModule.WIDE_BAND_LOOKUP = 1.5 -- m: a point of the wide corner must have a point of the strips this close

function PathFinderModule:useWideCorners()
    return self:isChopperWork() and (self.strictFruit or self.strictField)
end

-- cheap check before the obstacle boxes: the point is in the cut strips the whole train fits in
function PathFinderModule:isWidePointInBand(x, z)
    if not self:isBandFloodReady() or self.startX == nil then
        return true
    end
    if MathUtil.vector2Length(x - self.startX, z - self.startZ) < PathFinderModule.VERIFY_SKIP_START then
        return true
    end
    return ADBandRoute.getDistance(self.bandFlood, x, z, PathFinderModule.WIDE_BAND_LOOKUP) ~= nil
end

function PathFinderModule:isWideLineInBand(a, b)
    local l = MathUtil.vector2Length(b.x - a.x, b.z - a.z)
    local n = math.max(1, math.ceil(l / 2))
    for j = 0, n do
        if not self:isWidePointInBand(a.x + (b.x - a.x) * j / n, a.z + (b.z - a.z) * j / n) then
            return false
        end
    end
    return true
end

-- corner between the line from last through pi (d1) and the line through pk to nextPoint (d2), turning theta.
-- Returns the arc points, the swing start and the swing end (points to add to the route), nil when none is clean
function PathFinderModule:findWideCorner(last, pi, pk, nextPoint, d1, d2, theta, cross, radius, moreCorners, newPoint)
    local sign = cross > 0 and 1 or -1
    local function inside(d)
        if sign > 0 then
            return {x = -d.z, z = d.x}
        end
        return {x = d.z, z = -d.x}
    end
    local n1, n2 = inside(d1), inside(d2)
    local denom = d1.x * d2.z - d1.z * d2.x
    if math.abs(denom) < 0.001 then
        return nil
    end
    local s = ((pk.x - pi.x) * d2.z - (pk.z - pi.z) * d2.x) / denom
    local vx, vz = pi.x + d1.x * s, pi.z + d1.z * s
    local baseTangent = radius * math.tan(theta / 2)
    -- room on the lines: from the last kept point to the corner, from the corner to the next point
    local inRoom = (vx - last.x) * d1.x + (vz - last.z) * d1.z
    local outRoom = (nextPoint.x - vx) * d2.x + (nextPoint.z - vz) * d2.z
    if moreCorners then
        outRoom = outRoom * 0.5
    end
    if inRoom < 1 or outRoom < 1 then
        return nil
    end
    local options = {}
    for _, a1 in ipairs(PathFinderModule.WIDE_SWING_ANGLES) do
        for _, a2 in ipairs(PathFinderModule.WIDE_SWING_ANGLES) do
            if (a1 > 0 or a2 > 0) and theta + math.rad(a1 + a2) <= PathFinderModule.WIDE_MAX_ANGLE then
                for n, l1 in ipairs(PathFinderModule.WIDE_SWING_LEADS) do
                    for m, l2 in ipairs(PathFinderModule.WIDE_SWING_LEADS) do
                        -- no swing on a side: its lead does not matter
                        if (a1 > 0 or n == 1) and (a2 > 0 or m == 1) then
                            options[#options + 1] = {a1 = a1, a2 = a2, l1 = l1, l2 = l2, cost = a1 + a2 + (l1 + l2) * 0.1}
                        end
                    end
                end
            end
        end
    end
    table.sort(options, function(o1, o2) return o1.cost < o2.cost end)
    local checks = 0
    for _, o in ipairs(options) do
        if checks >= PathFinderModule.WIDE_MAX_CHECKS then
            break
        end
        local a1, a2 = math.rad(o.a1), math.rad(o.a2)
        local back = math.min(baseTangent + o.l1, inRoom)
        local ahead = math.min(baseTangent + o.l2, outRoom)
        local q1 = {x = vx - d1.x * back, z = vz - d1.z * back}
        local q2 = {x = vx + d2.x * ahead, z = vz + d2.z * ahead}
        -- swing out before the corner (away from the turn), come back from outside after it
        local e1 = {x = d1.x * math.cos(a1) - n1.x * math.sin(a1), z = d1.z * math.cos(a1) - n1.z * math.sin(a1)}
        local e2 = {x = d2.x * math.cos(a2) + n2.x * math.sin(a2), z = d2.z * math.cos(a2) + n2.z * math.sin(a2)}
        local denomE = e1.x * e2.z - e1.z * e2.x
        if math.abs(denomE) > 0.001 then
            local t = ((q2.x - q1.x) * e2.z - (q2.z - q1.z) * e2.x) / denomE
            local wx, wz = q1.x + e1.x * t, q1.z + e1.z * t
            local theta2 = math.acos(math.max(-1, math.min(1, e1.x * e2.x + e1.z * e2.z)))
            local tangent = radius * math.tan(theta2 / 2)
            local t1 = {x = wx - e1.x * tangent, z = wz - e1.z * tangent}
            local t2 = {x = wx + e2.x * tangent, z = wz + e2.z * tangent}
            local geometryOk = t > 0 and math.abs(theta2 - (theta + a1 + a2)) < 0.05
                and ((t1.x - q1.x) * e1.x + (t1.z - q1.z) * e1.z) >= 0
                and ((q2.x - t2.x) * e2.x + (q2.z - t2.z) * e2.z) >= 0
            local arc = nil
            if geometryOk then
                local p = inside(e1)
                local cx, cz = t1.x + p.x * radius, t1.z + p.z * radius
                local a0 = math.atan2(t1.z - cz, t1.x - cx)
                local sweep = sign * theta2
                local steps = math.max(2, math.ceil(theta2 * radius / PathFinderModule.CORNER_ARC_STEP))
                arc = {newPoint(t1.x, t1.z, pi)}
                for j = 1, steps do
                    local a = a0 + sweep * j / steps
                    arc[#arc + 1] = newPoint(cx + math.cos(a) * radius, cz + math.sin(a) * radius, pk)
                end
                local inBand = self:isWideLineInBand(q1, t1) and self:isWideLineInBand(arc[#arc], q2)
                for j = 1, #arc do
                    if not inBand then
                        break
                    end
                    inBand = self:isWidePointInBand(arc[j].x, arc[j].z)
                end
                if inBand then
                    checks = checks + 1
                    local qp1, qp2 = newPoint(q1.x, q1.z, pi), newPoint(q2.x, q2.z, pk)
                    local clear = self:isStretchClear(last, qp1) and self:isStretchClear(qp1, arc[1])
                        and self:isStretchClear(arc[#arc], qp2) and self:isStretchClear(qp2, nextPoint)
                    for j = 2, #arc do
                        if not clear then
                            break
                        end
                        local midX, midZ = (arc[j - 1].x + arc[j].x) / 2, (arc[j - 1].z + arc[j].z) / 2
                        local l = math.max(0.01, MathUtil.vector2Length(cx - midX, cz - midZ))
                        clear = self:isStretchClear(arc[j - 1], arc[j], {x = (cx - midX) / l, z = (cz - midZ) / l})
                    end
                    if clear then
                        return arc, qp1, qp2
                    end
                end
            end
        end
    end
    return nil
end

function PathFinderModule:getSharpCornerAngle()
    return self:isChopperWork() and PathFinderModule.CORNER_SHARP_ANGLE_CHOPPER or PathFinderModule.CORNER_SHARP_ANGLE
end

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
    local wide = 0
    local useWide = self:useWideCorners()
    self.firstSharpIndex = nil
    local i = 2
    while i <= #wps - 1 do
        local d1 = unit(wps[i - 1], wps[i])
        local done = false
        if d1 ~= nil then
            local turnSign = nil
            local wideCandidates = {}
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
                        wideCandidates[#wideCandidates + 1] = {k = k, theta = theta, cross = cross, d2 = d2}
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
            -- no room for the arc on the route line: outside, inside, outside
            if not done and useWide then
                for _, c in ipairs(wideCandidates) do
                    local arc, q1, q2 = self:findWideCorner(last, wps[i], wps[c.k], wps[c.k + 1], d1, c.d2, c.theta, c.cross, radius, c.k + 1 < #wps, newPoint)
                    if arc ~= nil then
                        -- no double point where the swing starts at the last point or ends at the next one
                        if MathUtil.vector2Length(q1.x - last.x, q1.z - last.z) > 0.5 then
                            out[#out + 1] = q1
                        end
                        for _, p in ipairs(arc) do
                            out[#out + 1] = p
                        end
                        last = arc[#arc]
                        if MathUtil.vector2Length(q2.x - wps[c.k + 1].x, q2.z - wps[c.k + 1].z) > 0.5 then
                            out[#out + 1] = q2
                            last = q2
                        end
                        rounded = rounded + 1
                        wide = wide + 1
                        i = c.k + 1
                        done = true
                        break
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
                if theta > self:getSharpCornerAngle() then
                    sharp = sharp + 1
                    if self.firstSharpIndex == nil then
                        self.firstSharpIndex = #out + 1
                    end
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

-- forage harvester, strict, no route: the tour of the field through the cut strips, forward only, to
-- the point behind the harvester in its direction (ADBandTour). Once per place (TOUR_RETRY_PLACE m, TOUR_RETRY_TIME ms)
PathFinderModule.TOUR_RETRY_TIME = 120000
PathFinderModule.TOUR_RETRY_PLACE = 15
PathFinderModule.TOUR_POINT_SPACING = 2

function PathFinderModule:startBandTour()
    if self.bandTour ~= nil or self.startPose ~= nil or ADBandTour == nil or not self:isChopperWork() or not (self.strictFruit or self.strictField)
        or not self.goingToPipe or self.target == nil or self.targetVector == nil then
        return false
    end
    local x, _, z, hx, hz = self:getStartPose()
    local now = g_time or 0
    if self.tourFailX ~= nil and now - (self.tourFailTime or 0) < PathFinderModule.TOUR_RETRY_TIME
        and MathUtil.vector2Length(x - self.tourFailX, z - self.tourFailZ) < PathFinderModule.TOUR_RETRY_PLACE then
        return false
    end
    local tl = MathUtil.vector2Length(self.targetVector.x, self.targetVector.z)
    if tl < 0.01 then
        return false
    end
    local th = math.atan2(self.targetVector.x / tl, self.targetVector.z / tl)
    local flood = self:isBandFloodReady() and self.bandFlood or nil
    self.bandTour = ADBandTour.start(x, z, math.atan2(hx, hz), self.target.x, self.target.z, th, self.realTurnRadius or self.minTurnRadius, self:getCropHalfWidth(), flood)
    return true
end

-- no route: the tour first, else the usual end (the calling task waits and plans again)
function PathFinderModule:abortOrTour()
    if self:startBandTour() then
        self.isFinished = false
        self.smoothDone = false
        self.dubinsDone = false
        return true
    end
    self:abort()
    return false
end

function PathFinderModule:updateBandTour()
    local tour = self.bandTour
    if not ADBandTour.step(tour, ADBandTour.BUDGET) then
        return
    end
    self.bandTour = nil
    local x, _, z = self:getStartPose()
    if tour.found then
        local points = {}
        for i, p in ipairs(tour.path) do
            if i == 1 or i == #tour.path or (i - 1) % PathFinderModule.TOUR_POINT_SPACING == 0 then
                points[#points + 1] = {x = p.x, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, p.x, 1, p.z), z = p.z}
            end
        end
        local blocked = nil
        for i = 2, #points do
            if not self:isStretchClear(points[i - 1], points[i], nil, true, true) then
                blocked = points[i]
                break
            end
        end
        if blocked == nil then
            self.wayPoints = points
            self.partialPath = false
            self.completelyBlocked = false
            self.isFinished = true
            self.smoothDone = true
            self.dubinsDone = true
            ADScheduler:removePathfinderVehicle(self.vehicle)
            return
        end
    end
    self.tourFailX, self.tourFailZ, self.tourFailTime = x, z, g_time or 0
    self:abort()
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
    if self.bandFlood ~= nil and not self.isFinished then
        local done = ADBandRoute.step(self.bandFlood, ADBandRoute.BUDGET)
        if not done and (g_time or 0) < self.bandFloodWaitUntil then
            return
        end
        if not self.bandFloodChecked then
            self.bandFloodChecked = true
            self:checkBandFlood()
        end
    end
    if self.bandTour ~= nil then
        self:updateBandTour()
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
        -- a search to the harvester that
        -- ended blocked (no more cells to explore, or its own limit) gives the same result with more steps: the 728 ran
        -- 9 identical restarts of 8 s (steps 201/400 ... 201/1200) for nothing. Such a search fails at once; the task
        -- then backs off along its own tracks after the next failure from the same place
        if increaseStepsAllowed and keepRestrictions and self.completelyBlocked then
            increaseStepsAllowed = false
        end

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
            self:abortOrTour()
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
    --Allow fruit in the last few grid cells (not behind the harvester: cut ground there)
    if (self:cellDistance(cell) <= 3 and self.goingToPipe and not self.goingBehind) then
        cell.isRestricted = false or wasRestricted
    end
    -- the vehicle already stands there: crop in the start cell (or within START_CROP_RADIUS) does not block the way out,
    -- any further the search has to follow the cut strip
    local nearStart = false
    if hasFruit and not wasRestricted and self.startX ~= nil then
        local center = cell.worldPos or self:gridLocationToWorldLocation(cell)
        nearStart = MathUtil.vector2Length(center.x - self.startX, center.z - self.startZ) < PathFinderModule.START_CROP_RADIUS
        if nearStart and self:isChopperWork() and (self.strictFruit or self.strictField) then
            -- strict: only where the train stands, a turn through the crop ahead of the tractor is no way out
            local _, _, _, hx, hz = self:getStartPose()
            nearStart = (center.x - self.startX) * hx + (center.z - self.startZ) * hz < PathFinderModule.START_CROP_AHEAD
        end
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

    --Allow fruit in the last few grid cells (not behind the harvester: cut ground there)
    if (self:cellDistance(cell) <= 3 and self.goingToPipe and not self.goingBehind) then
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
                    if self:isChopperWork() then
                        -- forage harvester: the shortcut is judged as the final check, at the tyre edges
                        fruitSide = self:getCropHalfWidth()
                    end
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
                elseif (self.strictFruit or self.strictField) and self:tryGuidedDeparture() then
                    self.smoothDone = true
                    return
                elseif self.strictFruit or self.strictField then
                    -- strict: better wait and plan again than drive through the crop, the calling task retries
                    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: no clean path (%s at %.0f,%.0f), strict: waiting and new try", tostring(reason), p.x, p.z)
                    self:abortOrTour()
                    self.smoothStep = 3
                    return
                else
                    PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: path in %.1f m cells kept despite %s at %.0f,%.0f (no better one)", self.minTurnRadius, tostring(reason), p.x, p.z)
                end
            end
            if self.partialPath and not self:trimPartialEnd() then
                PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: partial path without a clean stop point (facing the harvester), waiting and new try")
                if self:tryGuidedDeparture() then
                    self.smoothDone = true
                    return
                end
                self.partialFailCount = (self.partialFailCount or 0) + 1
                if self.partialFailCount >= PathFinderModule.PARTIAL_FACING_LIFT_FAILURES and not self.partialFacingLifted and self:isChopperWork() then
                    self.partialFacingLifted = true
                end
                self:abortOrTour()
                self.smoothStep = 3
                return
            end
            -- a route leaves: the next wait counts from zero again
            self.partialFailCount = 0
            self.partialFacingLifted = false
            self.guidedInRow = 0
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
            elseif sharp > 0 and (self.strictFruit or self.strictField) and self:usePartialBeforeSharp() then
                -- the route is driven up to the corner, planned again there
            elseif sharp > 0 and (self.strictFruit or self.strictField) and self:tryGuidedDeparture() then
                self.smoothDone = true
                return
            elseif sharp > 0 and (self.strictFruit or self.strictField) and self:tryGuidedDeparture(true) then
                self.smoothDone = true
                return
            elseif sharp > 0 and (self.strictFruit or self.strictField) then
                PathFinderModule.debugMsg(self.vehicle, "PathFinderModule: %d corner(s) too sharp for the train even in %.1f m cells, strict: waiting and new try", sharp, self.minTurnRadius)
                self:abortOrTour()
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
        -- a whole route leaves: the further departures count from zero again
        if not self.partialPath then
            self.guidedLossInRow = 0
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

    -- field ground the cut strips do not reach (wheel lane through the crop, pocket): counted as crop
    if not cell.isRestricted and cell.isOnField and self:isOutOfBand(worldPos.x, worldPos.z) then
        cell.isRestricted = true
        cell.hasFruit = true
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
    local band = self:getBandDistanceAtNode(node)
    if band then
        return band * 1.5 + (node.cost + goal_node.cost) * 0.5
    elseif band == false then
        -- strips known but this place cannot reach the target through them
        return self:get_cost(node, goal_node) * 1.5 + PathFinderModule.BAND_UNREACHED_PENALTY + (node.cost + goal_node.cost) * 0.5
    end
    return self:get_cost(node, goal_node) * 1.5 + (node.cost + goal_node.cost) * 0.5
end

-- real driving distance from a grid node to the target through the cut strips: number, false when the strips
-- are known but the node is not in them, nil when there is no flood (any other trip)
function PathFinderModule:getBandDistanceAtNode(node)
    if not self:isBandFloodReady() then
        return nil
    end
    local p = self:gridLocationToWorldLocation(node)
    return ADBandRoute.getDistance(self.bandFlood, p.x, p.z) or false
end

function PathFinderModule:isBandFloodReady()
    return self.bandFlood ~= nil and self.bandFlood.done and not self.bandFlood.failed and self.bandFloodStartDistance ~= nil
end

-- distance gained toward the target: real distance in the strips when both ends are in them
function PathFinderModule:getBandGain(fromX, fromZ, toX, toZ)
    if not self:isBandFloodReady() then
        return nil
    end
    local a = ADBandRoute.getDistance(self.bandFlood, fromX, fromZ, PathFinderModule.BAND_START_LOOKUP)
    if a == nil then
        return nil
    end
    local b = ADBandRoute.getDistance(self.bandFlood, toX, toZ)
    if b == nil then
        -- from the strips to a place they do not reach: a dead end, never a gain
        return -math.huge
    end
    return a - b
end

-- field ground the strips do not reach (a wheel lane through the crop, a pocket): no way for the train.
-- Not judged near the start (the train stands where it stands) and near the target
function PathFinderModule:isOutOfBand(x, z)
    -- only when the strips link the start to the target: else the search keeps its usual checks
    if not self:isBandFloodReady() or not self.bandFloodStartDistance then
        return false
    end
    if MathUtil.vector2Length(x - self.startX, z - self.startZ) < PathFinderModule.VERIFY_SKIP_START
        or MathUtil.vector2Length(x - self.target.x, z - self.target.z) < PathFinderModule.VERIFY_SKIP_END then
        return false
    end
    return ADBandRoute.getDistance(self.bandFlood, x, z, math.max(1, self.minTurnRadius / 2)) == nil
end

-- the grid (cells of the turning radius, 9-10 m) finds no way through
-- a cut strip. The flood of the cut strips (ADBandRoute, 1 m points, whole train width) knows the real way: traced back
-- from the tractor to the target, thinned, each piece checked for obstacles (the flood only knows crop and field).
-- Returns the waypoints (isPathFinderPoint) or nil, reason
PathFinderModule.BAND_PATH_THIN = 3          -- m between kept points on a straight stretch
PathFinderModule.BAND_PATH_CORNER = 12       -- deg: a heading change above this keeps the point
PathFinderModule.BAND_PATH_START_AHEAD = 1.5 -- m: the first point is ahead of the tractor at least
PathFinderModule.BAND_PATH_MAX_HEADING = 100 -- deg between the tractor heading and the first piece
PathFinderModule.BAND_PATH_END_TOLERANCE = 6 -- m between the end of the trace and the target at most
function PathFinderModule:getBandPath()
    local flood = self.bandFlood
    if flood == nil then
        return nil, "no strip flood"
    end
    if not flood.done then
        return nil, "strip flood not done"
    end
    if flood.failed then
        return nil, "no free point near the target"
    end
    local vx, vy, vz, hx, hz = self:getStartPose()
    local raw = ADBandRoute.tracePath(flood, vx, vz, PathFinderModule.BAND_START_LOOKUP)
    if raw == nil or #raw < 2 then
        return nil, "no strip links the start to the target"
    end
    local tail = raw[#raw]
    if MathUtil.vector2Length(tail.x - flood.x, tail.z - flood.z) > ADBandRoute.TARGET_SNAP + PathFinderModule.BAND_PATH_END_TOLERANCE then
        return nil, string.format("strip interrupted %.0f m from the target", MathUtil.vector2Length(tail.x - flood.x, tail.z - flood.z))
    end
    -- the trace starts at the reached point closest to the tractor, maybe beside or behind it: first point ahead
    local first = nil
    for i = 1, math.min(#raw, 20) do
        local dx, dz = raw[i].x - vx, raw[i].z - vz
        if dx * hx + dz * hz >= PathFinderModule.BAND_PATH_START_AHEAD then
            first = i
            break
        end
    end
    if first == nil or first >= #raw then
        return nil, "the strip starts behind the tractor"
    end
    local look = raw[math.min(#raw, first + 3)]
    local angle = math.abs(AutoDrive.angleBetween({x = hx, z = hz}, {x = look.x - vx, z = look.z - vz}))
    if angle > PathFinderModule.BAND_PATH_MAX_HEADING then
        return nil, string.format("the strip leaves %.0f deg off my heading", angle)
    end
    -- thinning: a point every BAND_PATH_THIN m, every corner, the last one
    local kept = {{x = vx, z = vz}}
    local lastKept = kept[1]
    local lastDir = nil
    for i = first, #raw do
        local p = raw[i]
        local dx, dz = p.x - lastKept.x, p.z - lastKept.z
        local d = math.sqrt(dx * dx + dz * dz)
        local corner = false
        if lastDir ~= nil and i < #raw then
            local q = raw[i + 1]
            corner = math.abs(AutoDrive.angleBetween(lastDir, {x = q.x - p.x, z = q.z - p.z})) > PathFinderModule.BAND_PATH_CORNER
        end
        if (i == #raw or corner or d >= PathFinderModule.BAND_PATH_THIN) and d > 0.5 then
            kept[#kept + 1] = {x = p.x, z = p.z}
            lastDir = {x = dx / d, z = dz / d}
            lastKept = kept[#kept]
        end
    end
    if #kept < 2 then
        return nil, "strip too short"
    end
    -- obstacles along each piece (walls, trees, learned blocked zones): the flood only knows crop and field
    local halfWidth = self:getTrainWidth() / 2 + FollowCombineTask.LINE_WIDTH_MARGIN
    local height = PathFinderModule.getTrainHeight(self.vehicle) + PathFinderModule.OBSTACLE_HEIGHT_MARGIN
    for i = 2, #kept do
        local a, b = kept[i - 1], kept[i]
        local dx, dz = b.x - a.x, b.z - a.z
        local len = math.sqrt(dx * dx + dz * dz)
        if len > 0.1 then
            dx, dz = dx / len, dz / len
            local angleRad = AutoDrive.normalizeAngle(math.atan2(-dz, dx))
            local midX, midZ = (a.x + b.x) / 2, (a.z + b.z) / 2
            local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, midX, 1, midZ)
            local probe = {hits = 0, callback = FollowCombineTask.lineCollisionCallback}
            overlapBox(midX, y + height / 2, midZ, 0, angleRad, 0, len / 2, height / 2, halfWidth, "callback", probe, AutoDrive.collisionMaskTerrain, true, true, true, true)
            if probe.hits > 0 then
                return nil, string.format("obstacle on the strip at %.0f,%.0f", midX, midZ)
            end
            if self:boxHitsBlockedZone(midX, midZ, angleRad, len / 2, halfWidth) then
                return nil, string.format("blocked zone on the strip at %.0f,%.0f", midX, midZ)
            end
        end
    end
    local path = {}
    for i, p in ipairs(kept) do
        path[i] = {x = p.x, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, p.x, 1, p.z), z = p.z, isPathFinderPoint = true}
    end
    return path
end

-- once the flood is done: distance of the start through the strips (false: the start is not linked to the target)
function PathFinderModule:checkBandFlood()
    local flood = self.bandFlood
    if not flood.done then
        return
    end
    if flood.failed then
        return
    end
    -- the start stands at the crop edge: a wider look around it
    local d = ADBandRoute.getDistance(flood, self.startX, self.startZ, PathFinderModule.BAND_START_LOOKUP)
    self.bandFloodStartDistance = d
    if d == nil then
        self.bandFloodStartDistance = false
        return
    end
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

-- the direct curve must not loop around to reach a point at a hard angle (the harvester is gone meanwhile),
-- and in strict mode the band swept by the whole train (trailer inside the curves) must be free of crop
PathFinderModule.DIRECT_CURVE_MAX_FACTOR = 2
PathFinderModule.DIRECT_CURVE_MIN_LENGTH = 15
function PathFinderModule:checkDirectCurve(path)
    if path == nil or #path < 2 then
        return ADDubins.EDUBNOPATH
    end
    if not self:isChopperWork() then
        return ADDubins.EDUBOK
    end
    local a, b = path[1], path[#path]
    local distance = MathUtil.vector2Length(b.x - a.x, b.z - a.z)
    local length = #path -- samples are 1 m apart
    if length > math.max(PathFinderModule.DIRECT_CURVE_MAX_FACTOR * distance, PathFinderModule.DIRECT_CURVE_MIN_LENGTH) then
        return ADDubins.EDUBNOPATH
    end
    if (self.strictFruit or self.strictField) then
        local clear, cropAt = self:isGuidedBandClear(path, self.realTurnRadius or self.minTurnRadius, 0)
        if not clear then
            return ADDubins.EDUBNOPATH
        end
    end
    return ADDubins.EDUBOK
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
                if result == ADDubins.EDUBOK then
                    result = self:checkDirectCurve(self.dubins.outPath)
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
