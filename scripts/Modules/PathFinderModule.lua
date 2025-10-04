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

PathFinderModule.PP_MAX_EAGER_LOOKAHEAD_STEPS = 1

PathFinderModule.MIN_FRUIT_VALUE = 50
PathFinderModule.SLOPE_DETECTION_THRESHOLD = math.rad(20)
PathFinderModule.NEW_PF_STEP_FACTOR = 4
--[[
from Giants Engine:
AITurnStrategy.SLOPE_DETECTION_THRESHOLD  = 0.5235987755983
]]

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
    self.chasingVehicle = false
    self.isSecondChasingVehicle = false
    self.max_pathfinder_steps = 0

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
    self:startPathPlanningToWayPoint(closest, destinationId)
    self.goingToNetwork = true
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
    local wayPoints = ADGraphManager:pathFromTo(wayPointId, destinationId)
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
    local _, worldY, _ = getWorldTranslation(combine.components[1].node)
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
    self.chasingVehicle = true
    self.isSecondChasingVehicle = true
    if targetVehicle.ad ~= nil and targetVehicle.ad.pathFinderModule ~= nil and targetVehicle.ad.pathFinderModule.fruitToCheck ~= nil then
        self.fruitToCheck = targetVehicle.ad.pathFinderModule.fruitToCheck
    end
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
    self.startX = vehicleWorldX + self.PATHFINDER_START_DISTANCE * vehicleRx
    self.startZ = vehicleWorldZ + self.PATHFINDER_START_DISTANCE * vehicleRz

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
    self.max_pathfinder_steps = PathFinderModule.MAX_PATHFINDER_STEPS_TOTAL * AutoDrive.getSetting("pathFinderTime")

    self.fruitToCheck = nil

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
    self.chasingVehicle = false
    self.isSecondChasingVehicle = false
    self.goingToNetwork = false
    self.destinationId = nil
    self.completelyBlocked = false
    self.targetBlocked = false --self.targetCell.hasCollision or self.targetCell.isRestricted --> always false TODO: how to use this ?
    self.blockedByOtherVehicle = false
    self.avoidFruitSetting = AutoDrive.getSetting("avoidFruit", self.vehicle)
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

function PathFinderModule:restartAtNextWayPoint()
    self.targetWayPointId = self.appendWayPoints[2].id
    local targetNode = ADGraphManager:getWayPointById(self.targetWayPointId)
    local wayPoints = ADGraphManager:pathFromTo(self.targetWayPointId, self.destinationId)
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
            self.appendWayPoints = ADGraphManager:pathFromTo(self.targetWayPointId, self.destinationId)
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
        local fallBackModeAllowed1 = (not self.chasingVehicle) and (not self.isSecondChasingVehicle) and (self.restrictToField) and (not self.fallBackMode1)    -- disable restrict to field
        local fallBackModeAllowed2 = (not self.chasingVehicle) and (not self.isSecondChasingVehicle) and (self.restrictToField) and (not self.fallBackMode2)    -- disable restrict to field border
        local fallBackModeAllowed3 = (not self.chasingVehicle) and (not self.isSecondChasingVehicle) and (self.avoidFruitSetting) and (not self.fallBackMode3)    -- disable avoid fruit
        local increaseStepsAllowed = (not self.chasingVehicle) and (not self.isSecondChasingVehicle) and (self.max_pathfinder_steps < PathFinderModule.MAX_PATHFINDER_STEPS_TOTAL * AutoDrive.getSetting("pathFinderTime"))    -- increase number of steps if possible

        -- Only allow auto restart when planning path to network and we can adjust target wayPoint
        local retryAllowed = self.destinationId ~= nil and self.retryCounter < self.PATHFINDER_MAX_RETRIES

        if fallBackModeAllowed1 then
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
            self.max_pathfinder_steps = math.min(self.max_pathfinder_steps, PathFinderModule.MAX_PATHFINDER_STEPS_TOTAL * AutoDrive.getSetting("pathFinderTime"))
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
                if self:isDriveableAstar(neighbor) then
                    if not self.closedset[neighbor] then
                        if not cost then cost = self:get_cost(current, neighbor) end
                        local tentative_g_score = self.g_score[current] + cost
                        local openset_idx = self.openset[neighbor]
                        if not openset_idx or tentative_g_score < self.g_score[neighbor] then
                            self.came_from[neighbor] = current
                            self.g_score[neighbor] = tentative_g_score
                            self.h_score[neighbor] = self.h_score[neighbor] or self:estimate_cost(neighbor, self.nodeGoal)
                            self.f_score[neighbor] = tentative_g_score + self.h_score[neighbor]
                            self.openset[neighbor] = true
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

    cell.isOnField = AutoDrive.checkIsOnField(worldPos.x, 0, worldPos.z)
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
        local shapeDefinition = self:getShapeDefByDirectionType(cell)   --> return shape for the cell according to direction, on ground level, 2.65m height
        local ignoreObstaclesUpToHeight = 0.5
        local shapes = overlapBox(shapeDefinition.x, shapeDefinition.y + ignoreObstaclesUpToHeight, shapeDefinition.z, 0, shapeDefinition.angleRad, 0, shapeDefinition.widthX, shapeDefinition.height - ignoreObstaclesUpToHeight, shapeDefinition.widthZ, "collisionTestCallbackIgnore", nil, self.mask, true, true, true, true)
        cell.hasCollision = cell.hasCollision or (shapes > 0)
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

function PathFinderModule:checkForFruitInArea(cell, corners)

    if self.goingToNetwork then
        -- on the way to network, check all fruit types
        self.fruitToCheck = nil
    end
    if self.fruitToCheck == nil then
        for _, fruitType in pairs(g_fruitTypeManager:getFruitTypes()) do
            if not (fruitType == g_fruitTypeManager:getFruitTypeByName("MEADOW")) then
                local fruitTypeIndex = fruitType.index
                self:checkForFruitTypeInArea(cell, fruitTypeIndex, corners)
            end
            --stop if cell is already restricted and/or fruit type is now known
            if cell.isRestricted ~= false or self.fruitToCheck ~= nil then
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
    local wasRestricted = cell.isRestricted
    cell.isRestricted = cell.isRestricted or (fruitValue > PathFinderModule.MIN_FRUIT_VALUE)

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

        local shapeDefinition = self:getShapeDefByDirectionType(cell, true)   --> return shape for the cell according to direction, on ground level, 2.65m height
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
    shapeDefinition.height = self.vehicle.size.height --2.65

    shapeDefinition.x = worldPos.x
    shapeDefinition.z = worldPos.z
    shapeDefinition.widthX = self.minTurnRadius / 2
    shapeDefinition.widthZ = self.minTurnRadius / 2

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
    shapeDefinition.height = self.vehicle.size.height --2.65

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
            local y = worldPos.y
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

                if not hasCollision then
                    if self.isNewPF then
                        self.collisionhits = 0
                        local shapes = overlapBox(worldPos.x + vectorX / 2, y + 3, worldPos.z + vectorZ / 2, 0, angleRad, 0, length / 2 + 2.5, 2.65, sideLength + 1.5, "collisionTestCallback", self, self.mask, true, true, true, true)
                        hasCollision = hasCollision or (self.collisionhits > 0)
                    else
                        local shapes = overlapBox(worldPos.x + vectorX / 2, y + 3, worldPos.z + vectorZ / 2, 0, angleRad, 0, length / 2 + 2.5, 2.65, sideLength + 1.5, "Ignore", nil, self.mask, true, true, true, true)
                        hasCollision = hasCollision or (shapes > 0)
                    end

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

                    if self.goingToNetwork then
                        -- check for all fruit types
                        for _, fruitType in pairs(g_fruitTypeManager:getFruitTypes()) do
                            if not (fruitType == g_fruitTypeManager:getFruitTypeByName("MEADOW")) then
                                local fruitTypeIndex = fruitType.index
                                local fruitValue = 0
                                if self.isSecondChasingVehicle then
                                    fruitValue = AutoDrive.getFruitValue(fruitTypeIndex, cornerWideX, cornerWideZ, cornerWide2X, cornerWide2Z, cornerWide4X, cornerWide4Z)
                                else
                                    fruitValue = AutoDrive.getFruitValue(fruitTypeIndex, cornerX, cornerZ, corner2X, corner2Z, corner4X, corner4Z)
                                end
                                hasCollision = hasCollision or (fruitValue > 50)
                                if hasCollision then

                                    if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                                        PathFinderModule.debugVehicleMsg(self.vehicle,
                                            string.format("PFM smoothResultingPPPath_Refined hasCollision %d",
                                                7
                                            )
                                        )
                                    end
                                    break
                                end
                            end
                        end
                    else
                        -- check only for fruit type detected on field
                        if self.fruitToCheck ~= nil then
                            local fruitValue = 0
                            if self.isSecondChasingVehicle then
                                fruitValue = AutoDrive.getFruitValue(self.fruitToCheck, cornerWideX, cornerWideZ, cornerWide2X, cornerWide2Z, cornerWide4X, cornerWide4Z)
                            else
                                fruitValue = AutoDrive.getFruitValue(self.fruitToCheck, cornerX, cornerZ, corner2X, corner2Z, corner4X, corner4Z)
                            end
                            hasCollision = hasCollision or (fruitValue > 50)

                            if hasCollision then
                                if self.vehicle ~= nil and self.vehicle.ad ~= nil and self.vehicle.ad.debug ~= nil and AutoDrive.debugVehicleMsg ~= nil then
                                    PathFinderModule.debugVehicleMsg(self.vehicle,
                                        string.format("PFM smoothResultingPPPath_Refined hasCollision %d",
                                            8
                                        )
                                    )
                                end
                            end
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

            if foundCollision or ((self.smoothIndex + self.totalEagerSteps) >= (#self.wayPoints - unfilteredEndPointCount)) then
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
                    DebugUtil.drawOverlapBox(shapeDefinition.x, shapeDefinition.y + 3, shapeDefinition.z, 0, shapeDefinition.angleRad, 0, shapeDefinition.widthX, 2.65, shapeDefinition.widthZ, 1, 1, 1)
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
                local text = string.format("%d",i)
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
    cell.shapeDefinition = self:getShapeDefByDirectionType_New(cell)   --> return shape for the cell according to direction, on ground level, 2.65m height

    local worldPos = self:gridLocationToWorldLocation(cell)
    --Try going through the checks in a way that fast checks happen before slower ones which might then be skipped

    cell.isOnField = AutoDrive.checkIsOnField(worldPos.x, 0, worldPos.z)

    -- check the most probable restrictions on field first to prevent unneccessary checks
    if not cell.isRestricted and self.restrictToField and not (self.fallBackMode1 or self.fallBackMode2) then
        -- in fallBackMode1 we ignore the field restriction
        cell.isRestricted = cell.isRestricted or (not cell.isOnField)
        if not cell.isOnField then
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
        self:checkForFruitInArea(cell, corners) -- set cell.isRestricted if fruit found
        table.insert(self.fruitAreas, corners)
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
        local shapes = overlapBox(cell.shapeDefinition.x, cell.shapeDefinition.y + 3, cell.shapeDefinition.z, 0, cell.shapeDefinition.angleRad, 0, cell.shapeDefinition.widthX, 2.65, cell.shapeDefinition.widthZ, "collisionTestCallback", self, self.mask, true, true, true, true)
        cell.hasCollision = cell.hasCollision or (self.collisionhits > 0)
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
    return not(cell.isRestricted)
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
function PathFinderModule:pop_best_node(set, score)
    local best, node = math.huge, nil

    for k, v in pairs(set) do
        local s = score[k]

        if s < best then
            best  = s or math.huge
            node = k
        end
    end
    if not node then return end
    set[node] = nil
    return node
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

    if self.appendWayPoints ~= nil then
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

    cell.isOnField = AutoDrive.checkIsOnField(cell.worldPos.x, 0, cell.worldPos.z)

    -- check the most probable restrictions on field first to prevent unneccessary checks
    if not cell.isRestricted and self.restrictToField and not (self.fallBackMode1 or self.fallBackMode2) then
        cell.isRestricted = cell.isRestricted or (not cell.isOnField)
        if not cell.isOnField then
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
        local shapes = overlapBox(cell.worldPos.x, cell.worldPos.y + 3, cell.worldPos.z, 0, cell.t, 0, sizeMax, 2.65, sizeMax, "collisionTestCallback", self, self.mask, true, true, true, true)
        cell.hasCollision = cell.hasCollision or (self.collisionhits > 0)
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
                local fromCell = nil
                for i, wayPoint in ipairs(self.dubins.outPath) do
                    local cell = get_node(wayPoint.x, wayPoint.z)
                    cell.worldPos = {x = wayPoint.x, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, wayPoint.x, 1, wayPoint.z), z = wayPoint.z}
                    cell.t = wayPoint.t
                    cell.incomming = fromCell
                    fromCell = cell
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
    result = self.dubins:dubins_shortest_path(ADDubins.DubinsPath, self.q0, self.q1, self.minTurnRadius)
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
            self.dubinsPath = self.dubins.outPath
            return self.dubinsPath
        end
    end

    for i = 1, 6, 1 do
        self.dubins.outPath = {}
        result = self.dubins:dubins_path(ADDubins.DubinsPath, self.q0, self.q1, self.minTurnRadius, i)
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
                local currentCollMask = getCollisionFilterMask(transformId)
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
