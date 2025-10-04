-- addConsoleCommand("adParseSplines", "Parse traffic splines into waypoints", "adParseSplines", AutoDrive)
-- addConsoleCommand("adRemoveSplines", "Remove parsed traffic spline nodes", "removeTrafficSplineNodes", AutoDrive)
-- addConsoleCommand("adCreateJunctions", "Create junctions", "createJunctionCommand", AutoDrive)
-- addConsoleCommand("adRemoveJunctions", "Remove junctions", "removeJunctionCommand", AutoDrive)

function AutoDrive:removeTrafficSplineNodes()
	ADGraphManager:removeNodesWithFlag(AutoDrive.FLAG_TRAFFIC_SYSTEM)
end

function AutoDrive:adParseSplines()
	local startNodes = {}
	local endNodes = {}
	local usedSplines = {}
	local splines = table.clone(g_currentMission.aiSystem.roadSplines)

	for _, spline in pairs(splines) do
		self:createWaypointsForSpline(startNodes, endNodes, usedSplines, splines, spline)
		if not table.contains(usedSplines, spline) then
			table.insert(usedSplines, spline)
		end
	end

	AutoDrive.startNodes = startNodes
	AutoDrive.endNodes = endNodes
end

function AutoDrive:createWaypointsForSpline(startNodes, endNodes, usedSplines, splines, spline)
	local lastX, lastY, lastZ = 0,0,0
	local length = getSplineLength(spline)
	local secondLastX, secondLastY, secondLastZ = 0,0,0
	local mapSize = Utils.getNoNil(g_currentMission.terrainSize, 2048) / 2
	local lastId = -1
	if length > 0 then
		local reverseSpline = AutoDrive:checkForSplineInReverseDirection(splines, spline)
		local isDualRoad = reverseSpline ~= nil
		if reverseSpline ~= nil and table.contains(usedSplines, reverseSpline) then
			return
		end
		for i=0, 1, 1.0/length do
			local posX, posY, posZ = getSplinePosition(spline, i)
			if posX ~= nil then
				local maxDistance = 3
				if lastX ~= 0 and secondLastX ~= 0 then
					local angle = math.abs(AutoDrive.angleBetween({x = posX - lastX, z = posZ - lastZ}, {x = lastX - secondLastX, z = lastZ - secondLastZ}))
					maxDistance = 6
					if angle < 0.5 then
						maxDistance = 12
					elseif angle < 1 then
						maxDistance = 6
					elseif angle < 2 then
						maxDistance = 4
					elseif angle < 4 then
						maxDistance = 3
					elseif angle < 7 then
						maxDistance = 2
					elseif angle < 14 then
						maxDistance = 1
					elseif angle < 27 then
						maxDistance = 0.5
					else
						maxDistance = 0.25
					end
				end

				if MathUtil.vector3Length(lastX - posX, lastY - posY, lastZ - posZ) > maxDistance then
					local connectLast = false
					if lastX ~= 0 then
						connectLast = true
					end
					
					if posX < mapSize and posZ < mapSize then
						local previousId = -1
						if i == 0 then
							previousId = AutoDrive:getSplineStartConnection(spline)
						end
							
						if previousId >= 0 then
							local wp = ADGraphManager:getWayPointById(previousId)
							secondLastX, secondLastY, secondLastZ = lastX, lastY, lastZ
							lastX, lastY, lastZ = wp.x, wp.y, wp.z
							lastId = previousId
						else
							local connectId = 0
							if lastId ~= 0 then
								connectId = lastId
								lastId = 0
							end
							ADGraphManager:recordWayPoint(posX, posY, posZ, connectLast, isDualRoad, false, connectId, AutoDrive.FLAG_TRAFFIC_SYSTEM)

							if lastX == 0 then
								local wpId = ADGraphManager:getWayPointsCount()
								local wp = ADGraphManager:getWayPointById(wpId)
								table.insert(startNodes, wp)
							end
	
							secondLastX, secondLastY, secondLastZ = lastX, lastY, lastZ
							lastX, lastY, lastZ = posX, posY, posZ
						end					
					end
				end
			end
		end

		local posX, posY, posZ = getSplinePosition(spline, 1)
		local targetId = AutoDrive:getSplineEndConnection(spline)
		if targetId >= 0 then			
			local wpId = ADGraphManager:getWayPointsCount()
			local wp = ADGraphManager:getWayPointById(wpId)
			ADGraphManager:toggleConnectionBetween(wp, ADGraphManager:getWayPointById(targetId), false, isDualRoad)
		else
			if posX < mapSize and posZ < mapSize then
				ADGraphManager:recordWayPoint(posX, posY, posZ, true, isDualRoad, false, 0, AutoDrive.FLAG_TRAFFIC_SYSTEM)
			end
	
			local wpId = ADGraphManager:getWayPointsCount()
			local wp = ADGraphManager:getWayPointById(wpId)
			table.insert(endNodes, wp)
		end		
	end	
end

function AutoDrive:checkForSplineInReverseDirection(splines, spline)
	local length = getSplineLength(spline)
	if length > 0 then
		local startX, _, startZ = getSplinePosition(spline, 0)
		local endX, _, endZ = getSplinePosition(spline, 1)

		for _, otherSpline in pairs(splines) do
			length = getSplineLength(otherSpline)
			if length > 0 then
				local otherStartX, _, otherStartZ = getSplinePosition(otherSpline, 0)
				local otherEndX, _, otherEndZ = getSplinePosition(otherSpline, 1)

				if AutoDrive.Equals(endX, otherStartX) and AutoDrive.Equals(endZ, otherStartZ) and AutoDrive.Equals(startX, otherEndX) and AutoDrive.Equals(startZ, otherEndZ) then
					return otherSpline
				end					
			end
		end
	end
end

function AutoDrive:getSplineStartConnection(spline)
	local length = getSplineLength(spline)
	if length > 0 then
		local startX, _, startZ = getSplinePosition(spline, 0)

		for wpId, wp in pairs(ADGraphManager:getWayPoints()) do
			if AutoDrive.Equals(startX, wp.x) and AutoDrive.Equals(startZ, wp.z) then
				return wpId
			end
		end
	end

	return -1
end

function AutoDrive:getSplineEndConnection(spline)
	local length = getSplineLength(spline)
	if length > 0 then
		local endX, _, endZ = getSplinePosition(spline, 1)

		for wpId, wp in pairs(ADGraphManager:getWayPoints()) do
			if AutoDrive.Equals(endX, wp.x) and AutoDrive.Equals(endZ, wp.z) then
				return wpId
			end
		end
	end

	return -1
end

function AutoDrive.Equals(a, b, tolerance)
	local tol = tolerance or 0.3

	return math.abs(a - b) <= tol
end

function AutoDrive:createJunctionCommand()
	--self:createJunctions(AutoDrive.startNodes, AutoDrive.endNodes, 150, 60)
	--self:createJunctions(AutoDrive.startNodes, AutoDrive.endNodes, 30, 120)
end

function AutoDrive:removeJunctionCommand()
	ADGraphManager:removeNodesWithFlag(AutoDrive.FLAG_TRAFFIC_SYSTEM_CONNECTION)
end

function AutoDrive:createJunctions(startNodes, endNodes, maxAngle, maxDist)
	--print("AutoDrive:createJunctions")
    AutoDrive.splineInterpolationUserCurvature = 5
	for _, endNode in pairs(endNodes) do
		if endNode.incoming ~= nil and #endNode.incoming > 0 then
			local incomingNode = ADGraphManager:getWayPointById(endNode.incoming[1])

			for __, startNode in pairs(startNodes) do
				if startNode.out ~= nil and #startNode.out > 0 then
					local outNode = ADGraphManager:getWayPointById(startNode.out[1])
					
					local angle = math.abs(AutoDrive.angleBetween({x = outNode.x - startNode.x, z = outNode.z - startNode.z}, {x = endNode.x - incomingNode.x, z = endNode.z - incomingNode.z}))
					local angle2 = math.abs(AutoDrive.angleBetween({x = startNode.x - endNode.x, z = startNode.z - endNode.z}, {x = endNode.x - incomingNode.x, z = endNode.z - incomingNode.z}))
					local angle3 = math.abs(AutoDrive.angleBetween({x = startNode.x - endNode.x, z = startNode.z - endNode.z}, {x = outNode.x - startNode.x, z = outNode.z - startNode.z}))
					local dist = ADGraphManager:getDistanceBetweenNodes(startNode.id, endNode.id)

					if angle < maxAngle and angle2 < 85 and angle3 < 85 and dist < maxDist then

                        local existingPath = ADPathCalculator:GetPath(endNode.id, startNode.id, {})
                        if (existingPath == nil or #existingPath == 0 or #existingPath > 40) then
                            self.splineInterpolation = nil
                            AutoDrive:createSplineInterpolationBetween(endNode, startNode)
                            -- Todo: check validity of path by checking for collision along created path
                            if self.splineInterpolation ~= nil and self.splineInterpolation.waypoints ~= nil and #self.splineInterpolation.waypoints > 2 then
                                if AutoDrive:checkForCollisionOnSpline() then
                                    local lastId = endNode.id
                                    local lastHeight = endNode.y
                                    for wpId, wp in pairs(self.splineInterpolation.waypoints) do
                                        if wpId ~= 1 and wpId < (#self.splineInterpolation.waypoints - 1) then
                                            if math.abs(wp.y - lastHeight) > 1 then -- prevent point dropping into the ground in case of bridges etc
                                                wp.y = lastHeight
                                            end			
                                            ADGraphManager:recordWayPoint(wp.x, wp.y, wp.z, true, false, false, lastId, AutoDrive.FLAG_TRAFFIC_SYSTEM_CONNECTION)
                                            lastId = ADGraphManager:getWayPointsCount()
											lastHeight = wp.y
                                        end
                                    end

                                    local wp = ADGraphManager:getWayPointById(lastId)
                                    ADGraphManager:toggleConnectionBetween(wp, startNode, false, false)
                                end
                            else
                                --print("AutoDrive:createJunctions - Fallback to toggle connections")
                                ADGraphManager:toggleConnectionBetween(endNode, startNode, false, false)
                            end
                            --ADGraphManager:toggleConnectionBetween(endNode, startNode, false, false)
                        end						
					end
				end				
			end
		end
	end
	
    AutoDrive.splineInterpolationUserCurvature = 0.49
end

function AutoDrive:checkForCollisionOnSpline()
	local widthX = 1.8
	local height = 2.3
    local mask = AutoDrive.collisionMaskSplines

	for wpId, wp in pairs(self.splineInterpolation.waypoints) do
		if wpId > 1 then
			local wpLast = self.splineInterpolation.waypoints[wpId - 1]
			local deltaX, deltaY, deltaZ = wp.x - wpLast.x, wp.y - wpLast.y, wp.z - wpLast.z
			local centerX, centerY, centerZ = wpLast.x + deltaX/2,  wpLast.y + deltaY/2,  wpLast.z + deltaZ/2
			local angleRad = math.atan2(deltaX, deltaZ)
    		angleRad = AutoDrive.normalizeAngle(angleRad)
            local length = MathUtil.vector2Length(deltaX, deltaZ) / 2

            local angleX = -MathUtil.getYRotationFromDirection(deltaY, length*2)

			local shapes = overlapBox(centerX, centerY+3, centerZ, angleX, angleRad, 0, widthX, height, length, "collisionTestCallbackIgnore", nil, mask, true, true, true, true)
            local r,g,b = 0,1,0
			if shapes > 0 then
                r = 1
                --DebugUtil.drawOverlapBox(centerX, centerY+3, centerZ, angleX, angleRad, 0, widthX, height, length, r, g, b)
				return false
			end
            --DebugUtil.drawOverlapBox(centerX, centerY+3, centerZ, angleX, angleRad, 0, widthX, height, length, r, g, b)
		end
	end
	return true
end
