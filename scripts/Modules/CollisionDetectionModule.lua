ADCollisionDetectionModule = {}

function ADCollisionDetectionModule:new(vehicle)
    local o = {}
    setmetatable(o, self)
    self.__index = self
    o.vehicle = vehicle
    o.detectedObstable = false
    o.reverseSectionClear = AutoDriveTON:new()
    o.reverseSectionClear.elapsedTime = 20000
    o.detectedCollision = false
    o.lastReverseCheck = false
    return o
end

function ADCollisionDetectionModule:hasDetectedObstable(dt)
    local reverseSectionBlocked = self:detectTrafficOnUpcomingReverseSection()

    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
        AutoDrive.debugMsg(self.vehicle, "CDM: hasDetectedObstable reverseSectionBlocked %s", tostring(reverseSectionBlocked))
    end

    self.reverseSectionClear:timer(not reverseSectionBlocked, 10000, dt)
    local detectObstacle = self:detectObstacle()
    local detectAdTrafficOnRoute = self:detectAdTrafficOnRoute()

    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
        AutoDrive.debugMsg(self.vehicle, "CDM: hasDetectedObstable detectObstacle %s detectAdTrafficOnRoute %s not reverseSectionClear:done %s"
        , tostring(detectObstacle)
        , tostring(detectAdTrafficOnRoute)
        , tostring(not self.reverseSectionClear:done())
        )
    end

    self.detectedObstable = detectObstacle or detectAdTrafficOnRoute or not self.reverseSectionClear:done()
    return self.detectedObstable
end

function ADCollisionDetectionModule:update(dt)
end

function ADCollisionDetectionModule:detectObstacle()
    if AutoDrive.getSetting("enableTrafficDetection") >= 1 then
        if self.vehicle.ad.sensors.frontSensorDynamicShort:pollInfo() then
            local frontSensorDynamicInBunkerArea = false
            local sensorLocation = self.vehicle.ad.sensors.frontSensorDynamicShort:getLocationByPosition()
            local vehX, vehY, vehZ = getWorldTranslation(self.vehicle.components[1].node)
            local worldOffsetX, worldOffsetY, worldOffsetZ =  AutoDrive.localDirectionToWorld(self.vehicle, sensorLocation.x, 0, sensorLocation.z)
            for _, trigger in pairs(ADTriggerManager.getBunkerSilos()) do
                if trigger and trigger.bunkerSiloArea ~= nil then
                    local x1, z1 = trigger.bunkerSiloArea.sx, trigger.bunkerSiloArea.sz
                    local x2, z2 = trigger.bunkerSiloArea.wx, trigger.bunkerSiloArea.wz
                    local x3, z3 = trigger.bunkerSiloArea.hx, trigger.bunkerSiloArea.hz
                    if MathUtil.hasRectangleLineIntersection2D(x1, z1, x2 - x1, z2 - z1, x3 - x1, z3 - z1, vehX + worldOffsetX, vehZ + worldOffsetZ, 0, 1) then
                        frontSensorDynamicInBunkerArea = true
                        break
                    end
                end
            end
            if (not frontSensorDynamicInBunkerArea) then
                if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
                    AutoDrive.debugMsg(self.vehicle, "CDM: detectObstacle frontSensorDynamicShort:pollInfo -> return true")
                end
                return true
            end
        end
    end

    if (g_updateLoopIndex % AutoDrive.PERF_FRAMES == 0) then
        local excludedList = self.vehicle.ad.taskModule:getActiveTask():getExcludedVehiclesForCollisionCheck()

        local box = self.vehicle.ad.sensors.frontSensorDynamicLong:getBoxShape()
        local boundingBox = {}
        boundingBox[1] = box.topLeft
        boundingBox[2] = box.topRight
        boundingBox[3] = box.downRight
        boundingBox[4] = box.downLeft
        boundingBox[1].y = box.y

        self.detectedCollision = AutoDrive:checkForVehicleCollision(self.vehicle, boundingBox, excludedList)

        if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
            AutoDrive.debugMsg(self.vehicle, "CDM: detectObstacle self.detectedCollision %s", tostring(self.detectedCollision))
        end
    end
    return self.detectedCollision
end

function ADCollisionDetectionModule:detectAdTrafficOnRoute()
    local wayPoints, currentWayPoint = self.vehicle.ad.drivePathModule:getWayPoints()
    if self.vehicle.ad.stateModule:isActive() and wayPoints ~= nil and self.vehicle.ad.drivePathModule:isOnRoadNetwork() then
        if (g_updateLoopIndex % AutoDrive.PERF_FRAMES == 0) then
            self.trafficVehicle = nil
            local idToCheck = 0
            local alreadyOnDualRoute = false
            if wayPoints[currentWayPoint - 1] ~= nil and wayPoints[currentWayPoint] ~= nil then
                alreadyOnDualRoute = ADGraphManager:isDualRoad(wayPoints[currentWayPoint - 1], wayPoints[currentWayPoint])
            end

            if wayPoints[currentWayPoint + idToCheck] ~= nil and wayPoints[currentWayPoint + idToCheck + 1] ~= nil and not alreadyOnDualRoute then

                local routePoints = {}
                local continueSearch = true
                while (continueSearch == true) do
                    local startNode = wayPoints[currentWayPoint + idToCheck]
                    local targetNode = wayPoints[currentWayPoint + idToCheck + 1]
                    if (startNode ~= nil) and (targetNode ~= nil) then
                        local testDual = ADGraphManager:isDualRoad(startNode, targetNode)
                        if testDual or idToCheck < 5 then
                            -- collect dual and single
                            table.insert(routePoints, startNode.id)
                            continueSearch = true
                        else
                            -- end
                            continueSearch = false
                        end
                    else
                        continueSearch = false
                    end
                    idToCheck = idToCheck + 1
                end

                if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
                    AutoDrive.debugMsg(self.vehicle, "CDM: detectAdTrafficOnRoute #routePoints %d", #routePoints)
                end

                if #routePoints >= 2 then
                    for _, other in pairs(AutoDrive.getAllVehicles()) do
                        if other ~= self.vehicle and other.ad ~= nil and other.ad.stateModule ~= nil and other.ad.stateModule:isActive() and other.ad.drivePathModule:isOnRoadNetwork() then
                            local onSameRoute = false
                            local foundY = false
                            local window = 4
                            local i = -window
                            local otherWayPoints, otherCurrentWayPoint = other.ad.drivePathModule:getWayPoints()
                            while i <= window and not foundY do
                                if otherWayPoints ~= nil and otherWayPoints[otherCurrentWayPoint + i] ~= nil then
                                    for j, point in pairs(routePoints) do
                                        if point == otherWayPoints[otherCurrentWayPoint + i].id then
                                            onSameRoute = true

                                            if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
                                                AutoDrive.debugMsg(self.vehicle, "CDM: detectAdTrafficOnRoute onSameRoute other %s i %d j %d point ID %d"
                                                , tostring(other and other.getName and other:getName() or "unknown")
                                                , i
                                                , j
                                                , point
                                                )
                                            end

                                            if routePoints[j - 1] ~= nil and otherWayPoints[otherCurrentWayPoint + i - 1] ~= nil then
                                                if routePoints[j - 1] ~= otherWayPoints[otherCurrentWayPoint + i - 1].id then
                                                    -- Y right-of-way
                                                    foundY = true

                                                    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
                                                        AutoDrive.debugMsg(self.vehicle, "CDM: detectAdTrafficOnRoute Y right-of-way point ID %d"
                                                        , point
                                                        )
                                                    end
                                                    break
                                                end
                                            end
                                        end
                                    end
                                end
                                i = i + 1
                            end

                            if onSameRoute and other.ad.collisionDetectionModule:getDetectedVehicle() == nil and foundY then

                                if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
                                    AutoDrive.debugMsg(self.vehicle, "CDM: detectAdTrafficOnRoute onSameRoute and foundY other %s"
                                    , tostring(other and other.getName and other:getName() or "unknown")
                                    )
                                end

                                self.trafficVehicle = other
                                return true
                            end
                        end
                    end
                end
            end
        else
            if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
                AutoDrive.debugMsg(self.vehicle, "CDM: detectAdTrafficOnRoute self.trafficVehicle ~= nil -> %s"
                , tostring(self.trafficVehicle and self.trafficVehicle.getName and self.trafficVehicle:getName() or "unknown")
                )
            end
            return self.trafficVehicle ~= nil
        end
    end
    return false
end

function ADCollisionDetectionModule:detectTrafficOnUpcomingReverseSection()
    local wayPoints, currentWayPoint = self.vehicle.ad.drivePathModule:getWayPoints()
    if self.vehicle.ad.stateModule:isActive() and wayPoints ~= nil and self.vehicle.ad.drivePathModule:isOnRoadNetwork() then
        if (g_updateLoopIndex % AutoDrive.PERF_FRAMES == 0) then
            self.lastReverseCheck = false
            local idToCheck = 1

            if wayPoints[currentWayPoint + idToCheck] ~= nil and wayPoints[currentWayPoint + idToCheck + 1] ~= nil then
                local reverseSection = ADGraphManager:isReverseRoad(wayPoints[currentWayPoint + idToCheck], wayPoints[currentWayPoint + idToCheck + 1])

                if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
                    if reverseSection then
                        AutoDrive.debugMsg(self.vehicle, "CDM: detectTrafficOnUpcomingReverseSection reverseSection ID %d"
                        , wayPoints[currentWayPoint + idToCheck].id
                        )
                    else
                        AutoDrive.debugMsg(self.vehicle, "CDM: detectTrafficOnUpcomingReverseSection not reverseSection")
                    end
                end

                local reverseSectionPoints = {}
                idToCheck = 0
                while (reverseSection == true) or (idToCheck < 20) do
                    local startNode = wayPoints[currentWayPoint + idToCheck]
                    local targetNode = wayPoints[currentWayPoint + idToCheck + 1]
                    if (startNode ~= nil) and (targetNode ~= nil) then
                        if ADGraphManager:isReverseRoad(startNode, targetNode) == true then

                            if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
                                AutoDrive.debugMsg(self.vehicle, "CDM: detectTrafficOnUpcomingReverseSection isReverseRoad -> insert(reverseSectionPoints idToCheck %d startNode.id %d targetNode.id %d"
                                , idToCheck
                                , startNode.id
                                , targetNode.id
                                )
                            end

                            table.insert(reverseSectionPoints, startNode.id)
                            reverseSection = true
                        else
                            reverseSection = false
                        end
                    else
                        reverseSection = false
                    end
                    idToCheck = idToCheck + 1
                end

                if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
                    AutoDrive.debugMsg(self.vehicle, "CDM: detectTrafficOnUpcomingReverseSection #reverseSectionPoints %d"
                    , #reverseSectionPoints
                    )
                end

                if #reverseSectionPoints > 0 then
                    --print(self.vehicle.ad.stateModule:getName() .. " - detected reverse section ahead")
                    for _, other in pairs(AutoDrive.getAllVehicles()) do
                        if other ~= self.vehicle and other.ad ~= nil and other.ad.stateModule ~= nil and other.ad.stateModule:isActive() and other.ad.drivePathModule:isOnRoadNetwork() and
                not (other.ad ~= nil and other.ad == self.vehicle.ad)       -- some trailed harvester get assigned AD from the trailing vehicle, see "attachable.ad = self.ad" in Specialisation
                then
                            local onSameRoute = false
                            local i = -10
                            local otherWayPoints, otherCurrentWayPoint = other.ad.drivePathModule:getWayPoints()
                            while i <= 10 do
                                if otherWayPoints ~= nil and otherWayPoints[otherCurrentWayPoint + i] ~= nil then
                                    for _, point in pairs(reverseSectionPoints) do
                                        if point == otherWayPoints[otherCurrentWayPoint + i].id then

                                            if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
                                                AutoDrive.debugMsg(self.vehicle, "CDM: detectTrafficOnUpcomingReverseSection point %d otherCurrentWayPoint ID %d onSameRoute = true"
                                                , point
                                                , otherWayPoints[otherCurrentWayPoint + i].id
                                                )
                                            end

                                            onSameRoute = true
                                            break
                                        end
                                    end
                                end
                                i = i + 1
                            end

                            if onSameRoute == true then

                                if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
                                    AutoDrive.debugMsg(self.vehicle, "CDM: detectTrafficOnUpcomingReverseSection onSameRoute trafficVehicle = other lastReverseCheck = true other %s"
                                    , tostring(other.getName and other:getName() or "unknown")
                                    )
                                end

                                --print(self.vehicle.ad.stateModule:getName() .. " - detected reverse section ahead - another vehicle on it")
                                self.trafficVehicle = other
                                self.lastReverseCheck = true
                            end
                        end
                    end
                end
            end
        else

            if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
                AutoDrive.debugMsg(self.vehicle, "CDM: detectTrafficOnUpcomingReverseSection self.lastReverseCheck %s"
                , tostring(self.lastReverseCheck)
                )
            end

            return self.lastReverseCheck
        end
    end

    return false
end

function ADCollisionDetectionModule:getDetectedVehicle()

    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
        AutoDrive.debugMsg(self.vehicle, "CDM: getDetectedVehicle self.trafficVehicle %s"
        , tostring(self.trafficVehicle and self.trafficVehicle.getName and self.trafficVehicle:getName() or "nil")
        )
    end

    return self.trafficVehicle
end

function ADCollisionDetectionModule:checkReverseCollision()
    local trailers, trailerCount = AutoDrive.getAllUnits(self.vehicle)
    local mostBackImplement = AutoDrive.getMostBackImplementOf(self.vehicle)

    local trailer = nil
    if trailers and trailerCount > 1 and self.vehicle.trailer ~= nil and self.vehicle.trailer ~= self.vehicle then
        -- vehicle.trailer is the controlable reverse attachable
        trailer = trailers[trailerCount]
    elseif mostBackImplement ~= nil then
        trailer = mostBackImplement
    else
        local ret = self.vehicle.ad.sensors.rearSensor:pollInfo()

        if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
            AutoDrive.debugMsg(self.vehicle, "CDM: checkReverseCollision 1 rearSensor:pollInfo %s"
            , tostring(ret)
            )
        end

        return ret
    end
    if trailer ~= nil then
        if trailer.ad == nil then
            trailer.ad = {}
        end
        ADSensor:handleSensors(trailer, 0)
        --trailer.ad.sensors.rearSensor.drawDebug = true
        trailer.ad.sensors.rearSensor.enabled = true
        local ret = trailer.ad.sensors.rearSensor:pollInfo()

        if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
            AutoDrive.debugMsg(self.vehicle, "CDM: checkReverseCollision 2 rearSensor:pollInfo %s"
            , tostring(ret)
            )
        end

        return ret
    end
end
