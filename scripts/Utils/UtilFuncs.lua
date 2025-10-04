string.randomCharset = {
	"0",
	"1",
	"2",
	"3",
	"4",
	"5",
	"6",
	"7",
	"8",
	"9",
	"A",
	"B",
	"C",
	"D",
	"E",
	"F",
	"G",
	"H",
	"I",
	"J",
	"K",
	"L",
	"M",
	"N",
	"O",
	"P",
	"Q",
	"R",
	"S",
	"T",
	"U",
	"V",
	"W",
	"X",
	"Y",
	"Z",
	"a",
	"b",
	"c",
	"d",
	"e",
	"f",
	"g",
	"h",
	"i",
	"j",
	"k",
	"l",
	"m",
	"n",
	"o",
	"p",
	"q",
	"r",
	"s",
	"t",
	"u",
	"v",
	"w",
	"x",
	"y",
	"z"
}

--- Calculates a much better result of world height by using a raycast.
--- The original function `getTerrainHeightAtWorldPos` returns wrong results if, for example, the terrain underneath a road has gaps.
--- As the raycast uses a callback function, this function must be splitted in two parts.
--- @param x number X Coordinate
--- @param z number Z Coordinate
--- @return number Height of the terrain
function AutoDrive:getTerrainHeightAtWorldPos(x, z, startingHeight)
	self.raycastHeight = nil
	local collisionMask = CollisionFlag.DEFAULT + CollisionFlag.ROAD + CollisionFlag.TERRAIN
	-- get a starting height with the basic function
	local startHeight = startingHeight or getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 1, z)
	-- do a raycast from a bit above y
	raycastClosest(x, startHeight + 3, z, 0, -1, 0, 5, "getTerrainHeightAtWorldPos_Callback", self, collisionMask)
	return self.raycastHeight or startHeight
end

--- Callback function called by AutoDrive:getTerrainHeightAtWorldPos()
function AutoDrive:getTerrainHeightAtWorldPos_Callback(hitObjectId, x, y, z, distance)
	if y ~= nil then
		self.raycastHeight = y
	--else
		--print("Raycast returned nil value!!!")
	end
end

function AutoDrive.streamReadStringOrEmpty(streamId)
	local string = streamReadString(streamId)
	if string == nil or string == "nil" then
		string = ""
	end
	return string
end

function AutoDrive.streamWriteStringOrEmpty(streamId, string)
	if string == nil or string == "" then
		string = "nil"
	end
	streamWriteString(streamId, string)
end

function AutoDrive.streamReadUIntNList(streamId, numberOfBits)
	local list = {}
	local len = streamReadUIntN(streamId, numberOfBits)
	for i = 1, len do
		local v = streamReadUIntN(streamId, numberOfBits)
		table.insert(list, v)
	end
	return list
end

function AutoDrive.streamWriteUIntNList(streamId, list, numberOfBits)
	list = list or {}
	streamWriteUIntN(streamId, #list, numberOfBits)
	for _, v in pairs(list) do
		streamWriteUIntN(streamId, v, numberOfBits)
	end
end

function AutoDrive.boxesIntersect(a, b)
	local polygons = {a, b}
	local minA, maxA, minB, maxB

	for _, polygon in pairs(polygons) do
		-- for each polygon, look at each edge of the polygon, and determine if it separates
		-- the two shapes

		for i1, _ in pairs(polygon) do
			--grab 2 vertices to create an edge
			local i2 = (i1 % 4 + 1)
			local p1 = polygon[i1]
			local p2 = polygon[i2]

			-- find the line perpendicular to this edge
			local normal = {x = p2.z - p1.z, z = p1.x - p2.x}

			minA = nil
			maxA = nil
			-- for each vertex in the first shape, project it onto the line perpendicular to the edge
			-- and keep track of the min and max of these values

			for _, corner in pairs(polygons[1]) do
				local projected = normal.x * corner.x + normal.z * corner.z
				if minA == nil or projected < minA then
					minA = projected
				end
				if maxA == nil or projected > maxA then
					maxA = projected
				end
			end

			--for each vertex in the second shape, project it onto the line perpendicular to the edge
			--and keep track of the min and max of these values
			minB = nil
			maxB = nil
			for _, corner in pairs(polygons[2]) do
				local projected = normal.x * corner.x + normal.z * corner.z
				if minB == nil or projected < minB then
					minB = projected
				end
				if maxB == nil or projected > maxB then
					maxB = projected
				end
			end
			-- if there is no overlap between the projects, the edge we are looking at separates the two
			-- polygons, and we know there is no overlap
			if maxA < minB or maxB < minA then
				--Logging.info("polygons don't intersect!");
				return false
			end
		end
	end

	--Logging.info("polygons intersect!");
	return true
end

-- Pumps N' Hoses DLC provides this function, we reuse their function if defined.
if math.clamp == nil then
	function math.clamp(value, minValue, maxValue)
		if minValue ~= nil and value ~= nil and maxValue ~= nil then
			return math.max(minValue, math.min(maxValue, value))
		end
		return value
	end
end

function table:contains(value)
	for _, v in pairs(self) do
		if v == value then
			return true
		end
	end
	return false
end

function table:f_contains(func)
	for _, v in pairs(self) do
		if func(v) then
			return true
		end
	end
	return false
end

function table:indexOf(value)
	for k, v in pairs(self) do
		if v == value then
			return k
		end
	end
	return nil
end

function table:f_indexOf(func)
	for k, v in pairs(self) do
		if func(v) then
			return k
		end
	end
	return nil
end

function table:f_find(func)
	for _, v in pairs(self) do
		if func(v) then
			return v
		end
	end
	return nil
end

function table:f_filter(func)
	local new = {}
	for _, v in pairs(self) do
		if func(v) then
			table.insert(new, v)
		end
	end
	return new
end

function table:removeValue(value)
	for k, v in pairs(self) do
		if v == value then
			table.remove(self, k)
			return true
		end
	end
	return false
end

function table:f_remove(func)
	for k, v in pairs(self) do
		if func(v) then
			table.remove(self, k)
		end
	end
end

function table:count()
	local c = 0
	if self ~= nil then
		for _ in pairs(self) do
			c = c + 1
		end
	end
	return c
end

function table:f_count(func)
	local c = 0
	if self ~= nil then
		for _, v in pairs(self) do
			if func(v) then
				c = c + 1
			end
		end
	end
	return c
end

function table:concatNil(sep, i, j)
	local res = table.concat(self, sep, i, j)
	if res == "" then
		return nil
	end
	return res
end

function string.random(length)
	if not length or length <= 0 then
		return ""
	end
	return string.random(length - 1) .. string.randomCharset[math.random(1, #string.randomCharset)]
end

function AutoDrive.localize(text)
	for m in text:gmatch("$l10n_.-;") do
		local l10n = m:gsub("$l10n_", ""):gsub(";", "")
		text = text:gsub(m, g_i18n:getText(l10n))
	end
	return text
end

function AutoDrive.angleBetween(vec1, vec2)
	--local scalarproduct_top = vec1.x * vec2.x + vec1.z * vec2.z;
	--local scalarproduct_down = math.sqrt(vec1.x * vec1.x + vec1.z*vec1.z) * math.sqrt(vec2.x * vec2.x + vec2.z*vec2.z)
	--local scalarproduct = scalarproduct_top / scalarproduct_down;
	local angle = math.atan2(vec2.z, vec2.x) - math.atan2(vec1.z, vec1.x)
	angle = AutoDrive.normalizeAngleToPlusMinusPI(angle)
	return math.deg(angle) --math.acos(angle)
end

function AutoDrive.normalizeAngle(inputAngle)
	if inputAngle > (2 * math.pi) then
		inputAngle = inputAngle - (2 * math.pi)
	else
		if inputAngle < -(2 * math.pi) then
			inputAngle = inputAngle + (2 * math.pi)
		end
	end

	return inputAngle
end

function AutoDrive.normalizeAngleToPlusMinusPI(inputAngle)
	if inputAngle > (math.pi) then
		inputAngle = inputAngle - (2 * math.pi)
	else
		if inputAngle < -(math.pi) then
			inputAngle = inputAngle + (2 * math.pi)
		end
	end

	return inputAngle
end

function AutoDrive.round(num)
	local under = math.floor(num)
	local upper = math.ceil(num)
	local underV = -(under - num)
	local upperV = upper - num
	if (upperV > underV) then
		return under
	else
		return upper
	end
end

function AutoDrive.getWorldDirection(fromX, fromY, fromZ, toX, toY, toZ)
	-- NOTE: if only 2D is needed, pass fromY and toY as 0
	local wdx, wdy, wdz = toX - fromX, toY - fromY, toZ - fromZ
	local dist = MathUtil.vector3Length(wdx, wdy, wdz) -- length of vector
	if dist and dist > 0.01 then
		wdx, wdy, wdz = wdx / dist, wdy / dist, wdz / dist -- if not too short: normalize
		return wdx, wdy, wdz, dist
	end
	return 0, 0, 0, 0
end

function AutoDrive.renderTable(posX, posY, textSize, inputTable, maxDepth)
	if inputTable == nil then
		return
	end
	maxDepth = maxDepth or 2
	local function renderTableRecursively(posX, posY, textSize, inputTable, depth, maxDepth, i)
		if depth >= maxDepth then
			return i
		end
		for k, v in pairs(inputTable) do
			local offset = i * textSize * 1.05
			setTextAlignment(RenderText.ALIGN_RIGHT)
			renderText(posX, posY - offset, textSize, tostring(k) .. " :")
			setTextAlignment(RenderText.ALIGN_LEFT)
			if type(v) ~= "table" then
				renderText(posX, posY - offset, textSize, " " .. tostring(v))
			end
			i = i + 1
			if type(v) == "table" then
				i = renderTableRecursively(posX + textSize * 2, posY, textSize, v, depth + 1, maxDepth, i)
			end
		end
		return i
	end
	local i = 0
	setTextColor(1, 1, 1, 1)
	setTextBold(false)
	textSize = getCorrectTextSize(textSize)
	for k, v in pairs(inputTable) do
		local offset = i * textSize * 1.05
		setTextAlignment(RenderText.ALIGN_RIGHT)
		renderText(posX, posY - offset, textSize, tostring(k) .. " :")
		setTextAlignment(RenderText.ALIGN_LEFT)
		if type(v) ~= "table" then
			renderText(posX, posY - offset, textSize, " " .. tostring(v))
		end
		i = i + 1
		if type(v) == "table" then
			i = renderTableRecursively(posX + textSize * 2, posY, textSize, v, 1, maxDepth, i)
		end
	end
end

function AutoDrive.dumpTable(inputTable, name, maxDepth, currentDepth)

	local function iterSorted(t)
		local tkeys = {}
		for k in pairs(t) do
			table.insert(tkeys, k)
		end

		table.sort(tkeys, function (a,b) return tostring(a) < tostring(b) end)

		local i = 0
		local n = #tkeys
		return function ()
					i = i + 1
					if i <= n then
						return tkeys[i], t[tkeys[i]]
				    end
				end
	end
		  
	maxDepth = maxDepth or 5
	currentDepth = currentDepth or 0
	if currentDepth > maxDepth then
		return
	end
	if currentDepth == 0 then
		AutoDrive.seenTables = {}
		print(name .. " = {}")
	end

	table.insert(AutoDrive.seenTables, inputTable)

	for k, v in iterSorted(inputTable) do
		local newName = string.format("%s.%s", name, k)
		if type(k) == "number" or type(k) == "table" then
			newName = string.format("%s[%s]", name, k)
		end

		if type(v) == "table" then
			if not AutoDrive.seenTables[v] then
				print(newName .. " = {}")
				AutoDrive.dumpTable(v, newName, maxDepth, currentDepth+1)
			else
				print(newName .. " <seen>")
			end
		else
			print(string.format("%s = %s", newName, v))
		end
	end

	if getmetatable(inputTable) ~= nil then
		for k, v in iterSorted(getmetatable(inputTable)) do
			local newName = string.format("%s.%s", name, k)
			if type(k) == "number" then
				newName = string.format("%s[%s]", name, k)
			end

			if type(v) == "table" then
				print(newName .. " = {}")
				AutoDrive.dumpTable(v, newName, maxDepth, currentDepth+1)
			else
				print(string.format("%s = %s", newName, v))
			end
		end
	end
end

-- addConsoleCommand("adSetDebugChannel", "Set new debug channel", "setDebugChannel", AutoDrive)

function AutoDrive:setDebugChannel(newDebugChannel)
	if newDebugChannel ~= nil then
		newDebugChannel = tonumber(newDebugChannel)
		if newDebugChannel == 0 then
			AutoDrive.currentDebugChannelMask = 0
		else
			if bit32.band(AutoDrive.currentDebugChannelMask, newDebugChannel) == newDebugChannel then
				AutoDrive.currentDebugChannelMask = AutoDrive.currentDebugChannelMask - newDebugChannel
			else
				AutoDrive.currentDebugChannelMask = bit32.bor(AutoDrive.currentDebugChannelMask, newDebugChannel)
			end
		end
	else
		AutoDrive.currentDebugChannelMask = AutoDrive.DC_ALL
	end
    AutoDriveDebugSettingsEvent.sendEvent(AutoDrive.currentDebugChannelMask)

	AutoDrive.showNetworkEvents()
end

-- addConsoleCommand("adDumpTable", "Dump Table to log", "dumpTableToLog", AutoDrive)

function AutoDrive:dumpTableToLog(input, ...)
	local f = getfenv(0).loadstring('return ' .. input)
	AutoDrive.dumpTable(f(), "Table:" .. input, 1)
end

function AutoDrive:createSplineInterpolationBetween(startNode, endNode)
	if startNode == nil or endNode == nil then
		return
	end

	AutoDrive.splineInterpolation = { 
		startNode = startNode,
		endNode = endNode,
		curvature = AutoDrive.splineInterpolationUserCurvature,
		waypoints = { startNode },
		valid = true 
	}

	if AutoDrive.splineInterpolationUserCurvature == nil then
		AutoDrive.splineInterpolationUserCurvature = 0.49
	end

	if table.contains(startNode.out, endNode.id) or table.contains(endNode.incoming, startNode.id) then
		-- nodes are already connected - do not create preview
		return
	end

	if AutoDrive.splineInterpolationUserCurvature >= 0.5 then
		local p0, p3 = AutoDrive:getSplineControlPoints(startNode, endNode)
		if p0 ~= nil and p3 ~= nil then
			return AutoDrive:createSplineWithControlPoints(startNode, p0, endNode, p3)
		end
	end
    -- fallback to straight line connection behaviour
end

function AutoDrive:getSplineControlPoints(startNode, endNode)
	-- Returns control points for the given startNode and endNode
	if #startNode.incoming + #startNode.out == 0 then
		return nil, nil
	end
	if #endNode.incoming + #endNode.out == 0 then
		return nil, nil
	end

	local function getWaypointDirection(node, waypointId)
		-- returns unit vector from start/end node to a waypoint
		local wp = ADGraphManager:getWayPointById(waypointId)
		local wpVec = ADVectorUtils.subtract2D(node, wp)
		if MathUtil.vector2Length(wpVec.x, wpVec.z) < 1e-8 then
			return nil
		end
		return ADVectorUtils.unitVector2D(wpVec)
	end

	local function getControlPoint(node, otherNode)
		-- returns the node that forms the straightest connection (closest to 180 degrees)
		local bestAngle, bestDirection = nil, nil
		local startEndVec = ADVectorUtils.subtract2D(node, otherNode)

		for _, px in pairs(node.incoming) do
			local dir = getWaypointDirection(node, px)
			if dir ~= nil then
				local angle = math.abs(180 - math.abs(AutoDrive.angleBetween(startEndVec, dir)))
				if bestAngle == nil or bestAngle > angle then
					bestAngle = angle
					bestDirection = dir
				end
			end
		end
		for _, px in pairs(node.out) do
			local dir = getWaypointDirection(node, px)
			if dir ~= nil then
				local angle = math.abs(180 - math.abs(AutoDrive.angleBetween(startEndVec, dir)))
				if bestAngle == nil or bestAngle > angle then
					bestAngle = angle
					bestDirection = dir
				end
			end
		end
		if bestDirection == nil then
			return nil
		end
		return ADVectorUtils.add2D(node, bestDirection)
	end
	return getControlPoint(startNode, endNode), getControlPoint(endNode, startNode)
end

function AutoDrive:createSplineWithControlPoints(startNode, p0, endNode, p3)
	self.splineInterpolation = {
		MIN_CURVATURE = 0.5,
		MAX_CURVATURE = 3.5,
		startNode = startNode,
		p0 = p0,
		endNode = endNode,
		p3 = p3,
		curvature = AutoDrive.splineInterpolationUserCurvature,
		waypoints = { startNode },
		valid = true
	}

	if AutoDrive.splineInterpolationUserCurvature >= self.splineInterpolation.MIN_CURVATURE then
		if AutoDrive.splineInterpolationUserCurvature == 5 then
			-- calculate the angle between start tangent and end tangent
			local dAngle = math.abs(AutoDrive.angleBetween(
				ADVectorUtils.subtract2D(p0, startNode),
				ADVectorUtils.subtract2D(endNode, p3)
			))
			self.splineInterpolation.curvature = ADVectorUtils.linterp(0, 180, dAngle, 1.5, 2.5)
		end

		-- distance from start to end, divided by two to give it more roundness...
		local dStartEnd = ADVectorUtils.distance2D(startNode, endNode) / self.splineInterpolation.curvature

		-- we need to normalize the length of start-p0 and end-p3, otherwise their length will influence the curve
		-- get vector from start->p0
		local vStartp0 = ADVectorUtils.subtract2D(startNode, p0)
		-- calculate unit vector of vStartp0
		vStartp0 = ADVectorUtils.unitVector2D(vStartp0)
		-- scale it like start->end
		vStartp0 = ADVectorUtils.scaleVector2D(vStartp0, dStartEnd)
		-- add it to the start Vector so that we get new p0
		p0 = ADVectorUtils.add2D(startNode, vStartp0)
		-- make sure p0 has a y value
		p0.y = startNode.y

		-- same for end->p3
		local vEndp3 = ADVectorUtils.subtract2D(endNode, p3)
		vEndp3 = ADVectorUtils.unitVector2D(vEndp3)
		vEndp3 = ADVectorUtils.scaleVector2D(vEndp3, dStartEnd)
		p3  = ADVectorUtils.add2D(endNode, vEndp3)
		p3.y = endNode.y

		local prevWP = startNode
		local secondLastWp = nil
		-- we're calculting a VERY smooth curve and whenever the new point on the curve has a good distance to the last one create a new waypoint
		-- but make sure that the last point also has a good distance to the endNode
		for i = 1, 200 do
			local px = AutoDrive:CatmullRomInterpolate(i, p0, startNode, endNode, p3, 200)
			local distPrev = ADVectorUtils.distance2D(prevWP, px)
			local distEnd = ADVectorUtils.distance2D(px, endNode)

			local minDistance = 1
			if secondLastWp ~= nil and prevWP ~= nil then
				minDistance = AutoDrive.getMaxDistanceForNextWp(secondLastWp, prevWP, px)
			end

			if distPrev > minDistance and distEnd >= 0.5 then
				-- get height at terrain
				px.y = AutoDrive:getTerrainHeightAtWorldPos(px.x, px.z, prevWP.y)
				table.insert(self.splineInterpolation.waypoints, px)

				-- Trying out if this slightly delayed call results in a more stable raycastHeight detection
				local dummy = 1
				for i = 1, 1000 do
					dummy = dummy + i
				end
				self.splineInterpolation.waypoints[#self.splineInterpolation.waypoints].y = self.raycastHeight or self.splineInterpolation.waypoints[#self.splineInterpolation.waypoints].y
				if self.splineInterpolation.waypoints[#self.splineInterpolation.waypoints].y == nil then
					self.splineInterpolation.waypoints[#self.splineInterpolation.waypoints].y = prevWP.y
				end
				
				secondLastWp = prevWP
				prevWP = px
			end
		end
		table.insert(self.splineInterpolation.waypoints, endNode)
		return self.splineInterpolation.waypoints
	else
		return
	end
end

function AutoDrive.getMaxDistanceForNextWp(secondLastWp, lastWp, currentWp)
	local angle = math.abs(AutoDrive.angleBetween({x = currentWp.x - lastWp.x, z = currentWp.z - lastWp.z}, {x = lastWp.x - secondLastWp.x, z = lastWp.z - secondLastWp.z}))
	local max_distance = 6
	if angle < 0.5 then
		max_distance = 12
	elseif angle < 1 then
		max_distance = 6
	elseif angle < 2 then
		max_distance = 4
	elseif angle < 4 then
		max_distance = 3
	elseif angle < 7 then
		max_distance = 2
	elseif angle < 14 then
		max_distance = 1
	elseif angle < 27 then
		max_distance = 0.5
	else
		max_distance = 0.25
	end

	return max_distance
end

function AutoDrive:CatmullRomInterpolate(index, p0, p1, p2, p3, segments)
	local px = {x=nil, y=nil, z=nil}
	local x = {p0.x, p1.x, p2.x, p3.x}
	local z = {p0.z, p1.z, p2.z, p3.z}
	local time = {0, 1, 2, 3} -- linear at start... calculate weights over time
	local total = 0.0

	for i = 2, 4 do
		local dx = x[i] - x[i - 1]
		local dz = z[i] - z[i - 1]
		-- the .9 is giving the wideness and roundness of the curve,
		-- lower values (like .25 will be more straight, while high values like .95 will be wider and rounder)
		total = total + math.pow(dx * dx + dz * dz, 0.95)
		time[i] = total
	end
    local tstart = time[2]
	local tend = time[3]
	local t = tstart + (index * (tend - tstart)) / segments

	local L01 = p0.x * (time[2] - t) / (time[2] - time[1]) + p1.x * (t - time[1]) / (time[2] - time[1])
	local L12 = p1.x * (time[3] - t) / (time[3] - time[2]) + p2.x * (t - time[2]) / (time[3] - time[2])
	local L23 = p2.x * (time[4] - t) / (time[4] - time[3]) + p3.x * (t - time[3]) / (time[4] - time[3])
	local L012 = L01 * (time[3] - t) / (time[3] - time[1]) + L12 * (t - time[1]) / (time[3] - time[1])
	local L123 = L12 * (time[4] - t) / (time[4] - time[2]) + L23 * (t - time[2]) / (time[4] - time[2])
	local C12 = L012 * (time[3] - t) / (time[3] - time[2]) + L123 * (t - time[2]) / (time[3] - time[2])
	px.x = C12

	L01 = p0.z * (time[2] - t) / (time[2] - time[1]) + p1.z * (t - time[1]) / (time[2] - time[1])
	L12 = p1.z * (time[3] - t) / (time[3] - time[2]) + p2.z * (t - time[2]) / (time[3] - time[2])
	L23 = p2.z * (time[4] - t) / (time[4] - time[3]) + p3.z * (t - time[3]) / (time[4] - time[3])
	L012 = L01 * (time[3] - t) / (time[3] - time[1]) + L12 * (t - time[1]) / (time[3] - time[1])
	L123 = L12 * (time[4] - t) / (time[4] - time[2]) + L23 * (t - time[2]) / (time[4] - time[2])
	C12 = L012 * (time[3] - t) / (time[3] - time[2]) + L123 * (t - time[2]) / (time[3] - time[2])
	px.z = C12

	return px
end

ADVectorUtils = {}

--- Calculates the unit vector on a given vector.
--- @param vector table Table with x and z properties.
--- @return table Unitvector as table with x and z properties.
function ADVectorUtils.unitVector2D(vector)
	local x, z = vector.x or 0, vector.z or 0
	local q = math.sqrt( (x * x) + ( z * z ) )
	return {x = x / q, z = z / q}
end

--- Scales a vector by a given scalar.
--- @param vector table Table with x and z properties. 
--- @param scale number Scale
--- @return table Vector
function ADVectorUtils.scaleVector2D(vector, scale)
	scale = scale or 1.0
	return { x = ( vector.x * scale ) or 0, z = ( vector.z * scale ) or 0 }
end

--- Returns the distance between zwo vectors using their x and z coordinates.
--- @param vectorA table with x and z property
--- @param vectorB table with x and z property
--- @return number Distance between vectorA and vectorB
function ADVectorUtils.distance2D(vectorA, vectorB)
	return MathUtil.vector2Length(vectorA.x - vectorB.x, vectorA.z - vectorB.z)
end

--- Returns a new vector pointing from vectorA to vectorB by subtracting A from B
--- @param vectorA table with x and z property
--- @param vectorB table with x and z property
--- @return table Vector pointing from A to B
function ADVectorUtils.subtract2D(vectorA, vectorB)
	return {x = vectorB.x - vectorA.x, z = vectorB.z - vectorA.z}
end

--- Inverts a vector with x and z properties. 
--- @param vector table with x and z property
--- @return table Vector inverted
function ADVectorUtils.invert2D(vector)
	return {x = vector.x * -1, z = vector.z * -1}
end

--- Adds x and z values of two given vectors and returns a new vector with x and z properties.
--- @param vectorA table with x and z property
--- @param vectorB table with x and z property
--- @return table Vector
function ADVectorUtils.add2D(vectorA, vectorB)
	return {x = vectorA.x + vectorB.x, z = vectorA.z + vectorB.z}
end

--- Does a linear interpolation based on the in* range and value, and returns a new value
--- fitting in the out range. Second return value is the interpolated value between 0..1.
--- @param inMin number Minimum value for input range
--- @param inMax number Maximum number for input range
--- @param inValue number Current value for input range
--- @param outMin number Minimum value vor output range
--- @param outMax number Maximum value for output range
--- @return number, number Interpolated value in output range, value in range 0..1
function ADVectorUtils.linterp(inMin, inMax, inValue, outMin, outMax)
	-- normalize input, make min range boundary = 0, nval is between 0..1
	local imax = inMax - inMin
	local nval = math.clamp(inValue - inMin, 0, imax) / imax

	-- normalize output
	local omax = outMax - outMin
	local oval = outMin + ( omax * nval )
	return oval, nval
end

function AutoDrive.getDebugChannelIsSet(debugChannel)
	return bit32.band(AutoDrive.currentDebugChannelMask, debugChannel) > 0
end

function AutoDrive.debugPrint(vehicle, debugChannel, debugText, ...)
	if AutoDrive.getDebugChannelIsSet(debugChannel) then
        AutoDrive.debugMsg(vehicle, debugText, ...)
	end
end

function AutoDrive.debugMsg(vehicle, debugText, ...)
    local printText = "[AD] " .. tostring(g_updateLoopIndex) .. " "
    if vehicle ~= nil then
        if vehicle.ad ~= nil and vehicle.ad.stateModule ~= nil and vehicle == vehicle:getRootVehicle() then
            printText = printText .. vehicle.ad.stateModule:getName() .. " (" .. tostring(vehicle.id or 0) .. ") : "
        elseif vehicle.getName ~= nil then
            printText = printText .. vehicle:getName() .. " (" .. tostring(vehicle.id or 0) .. ") : "
        else
            printText = printText .. tostring(vehicle) .. " (" .. tostring(vehicle.id or 0) .. ") : "
        end
    end

    Logging.info(printText .. debugText, ...)
end

function AutoDrive.errorMsg(vehicle, debugText, ...)
    local printText = "[AD] " .. tostring(g_updateLoopIndex) .. " "
    if vehicle ~= nil then
        if vehicle.ad ~= nil and vehicle.ad.stateModule ~= nil and vehicle == vehicle:getRootVehicle() then
            printText = printText .. vehicle.ad.stateModule:getName() .. " (" .. tostring(vehicle.id or 0) .. ") : "
        elseif vehicle.getName ~= nil then
            printText = printText .. vehicle:getName() .. " (" .. tostring(vehicle.id or 0) .. ") : "
        else
            printText = printText .. tostring(vehicle) .. " (" .. tostring(vehicle.id or 0) .. ") : "
        end
    end

    Logging.error(printText .. debugText, ...)
end

AutoDrive.debug = {}
AutoDrive.debug.connectionSendEventBackup = nil
AutoDrive.debug.serverBroadcastEventBackup = nil
AutoDrive.debug.lastSentEvent = nil
AutoDrive.debug.lastSentEventSize = 0

function AutoDrive.showNetworkEvents()
	if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_NETWORKINFO) then
		-- Activating network debug
		if g_server ~= nil then
			if AutoDrive.debug.serverBroadcastEventBackup == nil then
				AutoDrive.debug.serverBroadcastEventBackup = g_server.broadcastEvent
				g_server.broadcastEvent = Utils.overwrittenFunction(g_server.broadcastEvent, AutoDrive.ServerBroadcastEvent)
			end
		else
			local connection = g_client:getServerConnection()
			if AutoDrive.debug.connectionSendEventBackup == nil then
				AutoDrive.debug.connectionSendEventBackup = connection.sendEvent
				connection.sendEvent = Utils.overwrittenFunction(connection.sendEvent, AutoDrive.ConnectionSendEvent)
			end
		end
	else
		-- Deactivating network debug
		if g_server ~= nil then
			if AutoDrive.debug.serverBroadcastEventBackup ~= nil then
				g_server.broadcastEvent = AutoDrive.debug.serverBroadcastEventBackup
				AutoDrive.debug.serverBroadcastEventBackup = nil
			end
		else
			local connection = g_client:getServerConnection()
			if AutoDrive.debug.connectionSendEventBackup ~= nil then
				connection.sendEvent = AutoDrive.debug.connectionSendEventBackup
				AutoDrive.debug.connectionSendEventBackup = nil
			end
		end
	end
end

function AutoDrive:ServerBroadcastEvent(superFunc, event, sendLocal, ignoreConnection, ghostObject, force)
	local eCopy = {}
	eCopy.event = AutoDrive.tableClone(event)
	eCopy.eventName = eCopy.event.className or EventIds.eventIdToName[event.eventId]
	eCopy.sendLocal = sendLocal or false
	eCopy.ignoreConnection = ignoreConnection or "nil"
	eCopy.force = force or false
	eCopy.clients = table.count(self.clientConnections) - 1
	superFunc(self, event, sendLocal, ignoreConnection, ghostObject, force)
	eCopy.size = AutoDrive.debug.lastSentEventSize
	if eCopy.clients > 0 then
		AutoDrive.debugPrint(nil, AutoDrive.DC_NETWORKINFO, "%s size %s (x%s = %s) Bytes", eCopy.eventName, eCopy.size / eCopy.clients, eCopy.clients, eCopy.size)
	else
		AutoDrive.debugPrint(nil, AutoDrive.DC_NETWORKINFO, "%s", eCopy.eventName)
	end
	AutoDrive.debug.lastSentEvent = eCopy
end

function AutoDrive:ConnectionSendEvent(superFunc, event, deleteEvent, force)
	local eCopy = {}
	eCopy.event = AutoDrive.tableClone(event)
	eCopy.eventName = eCopy.event.className or EventIds.eventIdToName[event.eventId]
	eCopy.deleteEvent = deleteEvent or true
	eCopy.force = force or false
	superFunc(self, event, deleteEvent, force)
	eCopy.size = AutoDrive.debug.lastSentEventSize
	AutoDrive.debugPrint(nil, AutoDrive.DC_NETWORKINFO, "%s size %s Bytes", eCopy.eventName, eCopy.size)
	AutoDrive.debug.lastSentEvent = eCopy
end

function NetworkNode:addPacketSize(packetType, packetSizeInBytes)
	if (AutoDrive.debug.connectionSendEventBackup ~= nil or AutoDrive.debug.serverBroadcastEventBackup ~= nil) and packetType == NetworkNode.PACKET_EVENT then
		AutoDrive.debug.lastSentEventSize = packetSizeInBytes
	end
	if self.showNetworkTraffic then
		self.packetBytes[packetType] = self.packetBytes[packetType] + packetSizeInBytes
	end
end

function AutoDrive.tableClone(org)
	local otype = type(org)
	local copy
	if otype == "table" then
		copy = {}
		for org_key, org_value in pairs(org) do
			copy[org_key] = org_value
		end
	else -- number, string, boolean, etc
		copy = org
	end
	return copy
end

function AutoDrive.overwrittenStaticFunction(oldFunc, newFunc)
	return function(...)
		return newFunc(oldFunc, ...)
	end
end

function AutoDrive.renderColoredTextAtWorldPosition(x, y, z, text, textSize, color)
	local sx, sy, sz = project(x, y, z)
	if sx > -1 and sx < 2 and sy > -1 and sy < 2 and sz <= 1 then
		setTextAlignment(RenderText.ALIGN_CENTER)
		setTextBold(false)
		setTextColor(0.0, 0.0, 0.0, 0.75)
		renderText(sx, sy - 0.0015, textSize, text)
		setTextColor(color.r, color.g, color.b, 1.0)
		renderText(sx, sy, textSize, text)
		setTextAlignment(RenderText.ALIGN_LEFT)
	end
end

function AutoDrive.checkIsOnField(worldX, worldY, worldZ)	-- kept only for reference in case the new detection causes issues
	local densityBits = 0

	if worldY == 0 then
		worldY = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, worldX, 1, worldZ)
	end

	local bits = getDensityAtWorldPos(g_currentMission.terrainDetailId, worldX, worldY, worldZ)
	densityBits = bit32.bor(densityBits, bits)
	if densityBits ~= 0 then
		return true
	end

	return false
end

function AutoDrive.checkIsOnField_notFS22(startWorldX, worldY, startWorldZ)
    local data = g_currentMission.densityMapModifiers.getAIDensityHeightArea
    local modifier = data.modifier
    local filter = data.filter
    local widthWorldX = startWorldX - 0.1
    local widthWorldZ = startWorldZ - 0.1
    local heightWorldX = startWorldX + 0.1
    local heightWorldZ = startWorldZ + 0.1
    modifier:setParallelogramWorldCoords(startWorldX, startWorldZ, widthWorldX, widthWorldZ, heightWorldX, heightWorldZ, "ppp")
    filter:setValueCompareParams("greater", 0)
    local _, detailArea, _ = modifier:executeGet(filter)
    if detailArea == 0 then
        return false
    else
        return true
    end
end

--[[
Sprayer.registerOverwrittenFunctions =
	Utils.appendedFunction(
	Sprayer.registerOverwrittenFunctions,
	function(vehicleType)
		-- Work-around/fix for issue #863 ( thanks to DeckerMMIV )
		-- Having a slurry tank with a spreading unit attached, then avoid having the AI automatically turn these on when FollowMe is active.
		SpecializationUtil.registerOverwrittenFunction(
			vehicleType,
			"getIsAIActive",
			function(self, superFunc)
				local rootVehicle = self:getRootVehicle()
				if nil ~= rootVehicle and rootVehicle.ad ~= nil and rootVehicle.ad.stateModule ~= nil and rootVehicle.ad.stateModule:isActive() and self ~= rootVehicle then
					return false -- "Hackish" work-around, in attempt at convincing Sprayer.LUA to NOT turn on
				end
				return superFunc(self)
			end
		)
	end
)
]]

-- TODO check if working without this
-- FillTrigger doesn't allow filling non controlled tools ???
--[[ function AutoDrive:getIsActivatable(superFunc, objectToFill)
	--when the trigger is filling, it uses this function without objectToFill
	if objectToFill ~= nil then
		local vehicle = objectToFill:getRootVehicle()
		if vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.stateModule:isActive() then
			--if i'm in the vehicle, all is good and I can use the normal function, if not, i have to cheat:
			if AutoDrive.getControlledVehicle() ~= vehicle then
				local oldControlledVehicle = AutoDrive.getControlledVehicle()

				AutoDrive.setEnteredVehicle(vehicle)
				local result = true
				if superFunc ~= nil then
					result = superFunc(self, objectToFill)
				end

                AutoDrive.setEnteredVehicle(oldControlledVehicle)
				return result
			end
		end
	end
	local result = true
	if superFunc ~= nil then
		result = superFunc(self, objectToFill)
	end
	return result
end
 ]]

function AutoDrive:handleSplineCurvature(offset)
	-- returns true is the event was handled
	if AutoDrive.splineInterpolation ~= nil and AutoDrive.splineInterpolation.valid then
        AutoDrive.splineInterpolationUserCurvature = math.clamp(AutoDrive.splineInterpolationUserCurvature + offset/12, 0.49, 3.5)
		return true
	end
	return AutoDrive.mouseWheelActive
end

function AutoDrive:zoomSmoothly(superFunc, offset)
	if not AutoDrive:handleSplineCurvature(offset) then
		superFunc(self, offset)
	end
end

function AutoDrive:onZoomTopDownCamera(superFunc, action, offset, ...)
	if not AutoDrive:handleSplineCurvature(-offset) then
		superFunc(self, action, offset, ...)
	end
end

function AutoDrive:onFillTypeSelection(fillType)
    AutoDrive.debugPrint(self.validFillableObject, AutoDrive.DC_VEHICLEINFO, "AutoDrive:onFillTypeSelection start... #self.fillableObjects %s fillType %s self.validFillableObject %s self.currentFillableObject %s self.isLoading %s", tostring(table.count(self.fillableObjects)), tostring(fillType), tostring(self.validFillableObject), tostring(self.currentFillableObject), tostring(self.isLoading))

    -- fillUnit for wrong fillType is active, so disable loading and check again
    if fillType ~= nil and fillType ~= FillType.UNKNOWN then
        if self.validFillableObject and self.validFillableFillUnitIndex then
            if self.validFillableObject.getFillUnitSupportsFillType and not self.validFillableObject:getFillUnitSupportsFillType(self.validFillableFillUnitIndex, fillType) then
                AutoDrive.debugPrint(self.validFillableObject, AutoDrive.DC_VEHICLEINFO, "AutoDrive:onFillTypeSelection not getFillUnitSupportsFillType -> isLoading = false")
                self.isLoading = false
            end
            if self.validFillableObject.getFillUnitAllowsFillType and not self.validFillableObject:getFillUnitAllowsFillType(self.validFillableFillUnitIndex, fillType) then
                AutoDrive.debugPrint(self.validFillableObject, AutoDrive.DC_VEHICLEINFO, "AutoDrive:onFillTypeSelection not getFillUnitAllowsFillType ")
                self.isLoading = false
            end
        end
    end

	if not self.isLoading then
        AutoDrive.debugPrint(self.validFillableObject, AutoDrive.DC_VEHICLEINFO, "AutoDrive:onFillTypeSelection not self.isLoading")
        if fillType ~= nil and fillType ~= FillType.UNKNOWN then
            for _, fillableObject in pairs(self.fillableObjects) do --copied from gdn getIsActivatable to get a valid Fillable Object even without entering vehicle (needed for refuel first time)
                AutoDrive.debugPrint(fillableObject.object, AutoDrive.DC_VEHICLEINFO, "AutoDrive:onFillTypeSelection fillableObject.object fillableObject.fillUnitIndex %s", tostring(fillableObject.fillUnitIndex))

                if fillableObject.object.getFillUnitSupportsToolType and fillableObject.object:getFillUnitSupportsToolType(fillableObject.fillUnitIndex, ToolType.TRIGGER) then
                    AutoDrive.debugPrint(fillableObject.object, AutoDrive.DC_VEHICLEINFO, "AutoDrive:onFillTypeSelection getFillUnitSupportsToolType")
                    if fillableObject.object.getFillUnitSupportsFillType and fillableObject.object:getFillUnitSupportsFillType(fillableObject.fillUnitIndex, fillType) then
                        AutoDrive.debugPrint(fillableObject.object, AutoDrive.DC_VEHICLEINFO, "AutoDrive:onFillTypeSelection getFillUnitSupportsFillType")
                        if fillableObject.object.getFillUnitAllowsFillType and fillableObject.object:getFillUnitAllowsFillType(fillableObject.fillUnitIndex, fillType) then
                            AutoDrive.debugPrint(fillableObject.object, AutoDrive.DC_VEHICLEINFO, "AutoDrive:onFillTypeSelection getFillUnitAllowsFillType")
                            if fillableObject.object.getFillUnitFreeCapacity and fillableObject.object:getFillUnitFreeCapacity(fillableObject.fillUnitIndex) > 0 then
                                AutoDrive.debugPrint(fillableObject.object, AutoDrive.DC_VEHICLEINFO, "AutoDrive:onFillTypeSelection setIsLoading self.selectedFillType %s -> fillType %s", tostring(self.selectedFillType), tostring(fillType))
                                local rootVehicle = fillableObject.object.rootVehicle
                                local onRouteToRefuel = rootVehicle and rootVehicle.ad and rootVehicle.ad.onRouteToRefuel
                                if not onRouteToRefuel then
                                    -- do not load any fuel type in case of refuel active!
                                    self:setIsLoading(true, fillableObject.object, fillableObject.fillUnitIndex, fillType)
                                end
                                break
                            end
                        end
                    end
                end
            end
            if self.currentFillableObject ~= nil and self.fillUnitIndex == nil and AutoDrive.adErrorFillTypeSelection == nil then
                -- log an invalid load trigger
                AutoDrive.adErrorFillTypeSelection = true
                local rootVehicle = self.currentFillableObject.rootVehicle
                local name = rootVehicle and rootVehicle.getName and rootVehicle:getName() or "unknown vehicle"
                Logging.info("[AD] Info: invalid load trigger (fillUnitIndex == nil) for vehicle '%s'", tostring(name))
                if rootVehicle then
                    local x, y, z = getWorldTranslation(rootVehicle)
                    Logging.info("[AD] Info: invalid load trigger (fillUnitIndex == nil) vehicle position: %s , %s", tostring(x), tostring(z))
                end
            end
            AutoDrive.debugPrint(self.validFillableObject, AutoDrive.DC_VEHICLEINFO, "AutoDrive:onFillTypeSelection self.isLoading %s", tostring(self.isLoading))
        end
    end

	if not self.isLoading then
        AutoDrive.debugPrint(self.validFillableObject, AutoDrive.DC_VEHICLEINFO, "AutoDrive:onFillTypeSelection not self.isLoading")
        if fillType ~= nil and fillType ~= FillType.UNKNOWN then
            for _, fillableObject in pairs(self.fillableObjects) do

                local spec = fillableObject.object.spec_motorized
                if spec ~= nil and spec.consumersByFillTypeName ~= nil then

                    for _, consumer in pairs(spec.consumersByFillTypeName) do
                        local fillUnitIndex = consumer.fillUnitIndex
                        AutoDrive.debugPrint(fillableObject.object, AutoDrive.DC_VEHICLEINFO, "AutoDrive:onFillTypeSelection fillUnitIndex %s", tostring(fillUnitIndex))
                        if fillableObject.object.getFillUnitSupportsToolType and fillableObject.object:getFillUnitSupportsToolType(fillUnitIndex, ToolType.TRIGGER) then
                            AutoDrive.debugPrint(fillableObject.object, AutoDrive.DC_VEHICLEINFO, "AutoDrive:onFillTypeSelection getFillUnitSupportsToolType")
                            if fillableObject.object.getFillUnitSupportsFillType and fillableObject.object:getFillUnitSupportsFillType(fillUnitIndex, fillType) then
                                AutoDrive.debugPrint(fillableObject.object, AutoDrive.DC_VEHICLEINFO, "AutoDrive:onFillTypeSelection getFillUnitSupportsFillType")
                                if fillableObject.object.getFillUnitAllowsFillType and fillableObject.object:getFillUnitAllowsFillType(fillUnitIndex, fillType) then
                                    AutoDrive.debugPrint(fillableObject.object, AutoDrive.DC_VEHICLEINFO, "AutoDrive:onFillTypeSelection getFillUnitAllowsFillType")
                                    if fillableObject.object.getFillUnitFreeCapacity and fillableObject.object:getFillUnitFreeCapacity(fillUnitIndex) > 0 then
                                        AutoDrive.debugPrint(fillableObject.object, AutoDrive.DC_VEHICLEINFO, "AutoDrive:onFillTypeSelection setIsLoading self.selectedFillType %s -> fillType %s", tostring(self.selectedFillType), tostring(fillType))
                                        self:setIsLoading(true, fillableObject.object, fillUnitIndex, fillType)
                                        break
                                    end
                                end
                            end
                        end
                    end
                end
            end
            AutoDrive.debugPrint(self.validFillableObject, AutoDrive.DC_VEHICLEINFO, "AutoDrive:onFillTypeSelection self.isLoading %s", tostring(self.isLoading))
        end
    end
    AutoDrive.debugPrint(nil, AutoDrive.DC_VEHICLEINFO, "AutoDrive:onFillTypeSelection end")
end

function AutoDrive.sign(x)
	if x < 0 then
		return -1
	elseif x > 0 then
		return 1
	else
		return 0
	end
end

function AutoDrive.segmentIntersects(x1, y1, x2, y2, x3, y3, x4, y4)
	local d = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
	local Ua_n = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3))
	local Ub_n = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3))

	local Ua = Ua_n / d
	local Ub = Ub_n / d

	if d ~= 0 then
		local x = x1 + Ua * (x2 - x1)
		local y = y1 + Ua * (y2 - y1)
		local insideSector = Ua > 0
		local insideSecondSector = d > 0
		return x, y, insideSector, insideSecondSector
	else
		return 0, 0, false, false
	end
end

-- find WP linked to themselve - fatal error, parm: true for direct data correction in mapWayPoints{}
function AutoDrive.checkWaypointsLinkedtothemselve(correctit)
    local network = ADGraphManager:getWayPoints()
	local overallnumberWP = ADGraphManager:getWayPointsCount()
	local count = 0
	
	if overallnumberWP < 3 then return end
	
	for i, point in pairs(network) do
		if #point.out > 0 then
			for j, linkedNodeId_1 in ipairs(point.out) do
				local wp_2 = network[linkedNodeId_1]
				if wp_2 ~= nil then
					if (i == linkedNodeId_1) then
						if correctit then
							table.remove(network[i].out,j)
							count = count + 1
						end
					end
				end
			end
		end
	end
	if count > 0 then
		AutoDrive.debugPrint(nil, AutoDrive.DC_ROADNETWORKINFO, "removed %s waypoint links to themselve", tostring(count))
	end
end

-- find WP with multiple same out ID - parm: true for direct data correction in mapWayPoints{}
function AutoDrive.checkWaypointsMultipleSameOut(correctit)
    local network = ADGraphManager:getWayPoints()
	local overallnumberWP = ADGraphManager:getWayPointsCount()
	local count = 0

	if overallnumberWP < 3 then return end

	for i, point in pairs(network) do
		if #point.out > 1 then
			for j, linkedNodeId_1 in ipairs(network[i].out) do
				local wp_2 = network[linkedNodeId_1]
				if wp_2 ~= nil then
					for k, linkedNodeId_2 in ipairs(network[i].out) do
						if k>j then
							local wp_3 = network[linkedNodeId_2]
							if wp_3 ~= nil then
								if j < #network[i].out then
									if (network[i].out[j] == network[i].out[k]) then
										if correctit then
											table.remove(network[i].out,k)
											count = count + 1
										end
									end
								end
							end
						end
					end
				end
			end
		end
	end
	if count > 0 then
		AutoDrive.debugPrint(nil, AutoDrive.DC_ROADNETWORKINFO, "removed %s waypoint with multiple same out links", tostring(count))
	end
end

function AutoDrive.playSample(sample, volume, forced)
    if (AutoDrive.getSetting("playSounds") and sample~= nil) or forced then
        playSample(sample, 1, volume, 0, 0, 0)
    end
end

function AutoDrive.stringToNumberList(text, sep)
	sep = sep or ','
	local list = {}
	for _, v in pairs(string.split(text, sep)) do
		local num = tonumber(v)
		if num ~= nil then
			table.insert(list, num)
		end
	end
	return list
end

AutoDrive.debugTransform = false

function AutoDrive.isReverseDriving(vehicle)
	return 	vehicle.spec_reverseDriving ~= nil and vehicle.spec_reverseDriving.hasReverseDriving and vehicle.spec_reverseDriving.isReverseDriving
end


function AutoDrive.localToLocalForReverseDriving(vehicle, x, z)
	if AutoDrive.isReverseDriving(vehicle) then
		return -x, -z
	else
		return x, z
	end
end

function AutoDrive.selectVehicleNode(vehicle, node_or_identifier)
	if node_or_identifier == nil then
		return vehicle.components[1].node
	end
	if type(node_or_identifier) == "number" then
		return node_or_identifier
	end
end

function AutoDrive.localToWorld(vehicle, x, y, z, node_or_identifier)
	AutoDrive.debugTransformMsg(vehicle, "AutoDrive:localToWorld(%f, %f, %f)", x, y, z)
	x, z = AutoDrive.localToLocalForReverseDriving(vehicle, x, z)
	local node = AutoDrive.selectVehicleNode(vehicle, node_or_identifier)
	return localToWorld(node, x, y, z)
end


function AutoDrive.worldToLocal(vehicle, x, y, z, node_or_identifier)
	AutoDrive.debugTransformMsg(vehicle, "AutoDrive:worldToLocal(%f, %f, %f)", x, y, z)
	local node = AutoDrive.selectVehicleNode(vehicle, node_or_identifier)
	local localX, localY, localZ = worldToLocal(node, x, y, z)
	localX, localZ = AutoDrive.localToLocalForReverseDriving(vehicle, localX, localZ)
	return localX, localY, localZ
end

function AutoDrive.localDirectionToWorld(vehicle, x, y, z, node_or_identifier)
	AutoDrive.debugTransformMsg(vehicle, "AutoDrive:localDirectionToWorld(%f, %f, %f)", x, y, z)
	x, z = AutoDrive.localToLocalForReverseDriving(vehicle, x, z)
	local node = AutoDrive.selectVehicleNode(vehicle, node_or_identifier)
	return localDirectionToWorld(node, x, y, z)
end

function AutoDrive.worldDirectionToLocal(vehicle, x, y, z, node_or_identifier)
	AutoDrive.debugTransformMsg(vehicle, "AutoDrive:worldDirectionToLocal(%f, %f, %f)", x, y, z)
	local node = AutoDrive.selectVehicleNode(vehicle, node_or_identifier)
	local localX, localY, localZ = worldDirectionToLocal(node, x, y, z)
	localX, localZ = AutoDrive.localToLocalForReverseDriving(vehicle, localX, localZ)
	return localX, localY, localZ
end

function AutoDrive.getDriveDirection(vehicle, x, y, z, node_or_identifier)
	AutoDrive.debugTransformMsg(vehicle, "AutoDrive:localDirectionToWorld(%f, %f, %f)", x, y, z)
	local node = AutoDrive.selectVehicleNode(vehicle, node_or_identifier)
	local directionX, directionZ = AIVehicleUtil.getDriveDirection(node, x, y, z)
	return AutoDrive.localToLocalForReverseDriving(vehicle, directionX, directionZ)
end

function AutoDrive.driveInDirection(self, dt, steeringAngleLimit, acceleration, slowAcceleration, slowAngleLimit, allowedToDrive, moveForwards, lx, lz, maxSpeed, slowDownFactor)
	-- Having to set this so that the bug in driveInDirection doesn't occur
	self.motor = self:getMotor()
	self.cruiseControl = {}
	self.cruiseControl.state = self:getCruiseControlState()
	lx, lz = AutoDrive.localToLocalForReverseDriving(self, lx, lz)
	AutoDrive.debugTransformMsg(self, "AutoDrive:driveInDirection(%f, %f), limit=%f, slow_limit=%f, accel=%f, slow_accel=%f, allowed=%s, forwards=%s, speed=%f, slowdown=%f", 
	                            lx, lz, steeringAngleLimit, slowAngleLimit, acceleration, slowAcceleration, tostring(allowedToDrive), tostring(moveForwards), maxSpeed, slowDownFactor)
	if AutoDrive.isReverseDriving(self) then
		steeringAngleLimit = -steeringAngleLimit
	end
	AIVehicleUtil.driveInDirection(self, dt, steeringAngleLimit, acceleration, slowAcceleration, slowAngleLimit, allowedToDrive, moveForwards, lx, lz, maxSpeed, slowDownFactor)
end

function AutoDrive.debugTransformMsg(vehicle, debugText, ...)
    if AutoDrive.debugTransform == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    end
end
