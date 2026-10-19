ADPathCalculator = {}

ADPathCalculator.MAX_ITERATIONS = 10000000

-- vehicle (optional): sections where its train does not fit (wall, tree, post, building, bridge railing
-- closer than half its width + margin) get a big extra weight, so another route is taken when one exists
function ADPathCalculator:GetPath(startID, targetID, preferredStartIds, vehicle)
	local count = 0
	local countIterations = 0
    local narrowWeight = self:getNarrowWeightFunc(vehicle)

    if not ADGraphManager:areWayPointsPrepared() then
        ADPathCalculator.widthCache = {} -- network changed: measure again
        ADPathCalculator.narrowSections = {}
		AutoDrive.checkWaypointsLinkedtothemselve(true)		-- find WP linked to themselve, with parameter true issues will be fixed
		AutoDrive.checkWaypointsMultipleSameOut(true)		-- find WP with multiple same out ID, with parameter true issues will be fixed
        ADGraphManager:prepareWayPoints()
    end

    local network = ADGraphManager:getWayPoints()
    local addedWeights = self:getDetourWeights()
    
    if startID == nil or targetID == nil or network[startID] == nil or network[targetID] == nil then
        return {}
    end

    if startID == targetID then
        return {network[startID]}
    end

    local candidates = SortedQueue:new("distance")
    candidates:enqueue({p=network[startID], distance=0, pre=-1})

    local results = {}

    local foundTarget = false
    local sqrt = math.sqrt
    local distanceFunc = function(a, b, c)
        return sqrt((a.x - b.x)* (a.x - b.x) + (a.z - b.z) * (a.z - b.z))
    end
    local distanceFunc2 = function(a, b, c)
        return ADGraphManager:getDriveTimeBetweenNodes(a.id, b.id, c, 30, true)
    end

    local isSubPrio = function(pointToTest) 
        return bit32.band(pointToTest.flags, AutoDrive.FLAG_SUBPRIO) > 0
    end

    local lastPredecessor = nil
    while not candidates:empty() and not foundTarget and (countIterations < ADPathCalculator.MAX_ITERATIONS) do
        countIterations = countIterations + 1
        local next = candidates:dequeue()
        local point, distance, previousPoint = next.p, next.distance, next.pre
        while point ~= nil and (countIterations < ADPathCalculator.MAX_ITERATIONS) do
            countIterations = countIterations + 1
            if results[point.id] == nil then
                results[point.id] = {}
            end
            if point.id == targetID then
                foundTarget = true
                lastPredecessor = previousPoint
                point = nil
                break
            else
                -- Check all out points if
                -- a) is startpoint
                -- b) has multiple incoming routes? (Because we could have followed the wrong path to that intersection after all, I guess)
                -- c) we have multiple outgoing routes (coming from previousPoint)
                if previousPoint == -1 or #point.incoming > 1 or (point.transitMapping[previousPoint] == nil or #point.transitMapping[previousPoint] > 1) then
                    
                    -- use either the default .out variable of each point or preferably, use the transitMapping 
                    local outMap = point.out
                    if previousPoint ~= -1 and point.transitMapping[previousPoint] ~= nil then
                        outMap = point.transitMapping[previousPoint]
                    end

                    for _, outId in pairs(outMap) do
                        countIterations = countIterations + 1
                        if (countIterations > ADPathCalculator.MAX_ITERATIONS) then
                            -- emergency exit
                            return {}
                        end
                        local outPoint = network[outId]
                        
                        -- Add this out point into our results table with the current distance
                        if results[point.id][outId] == nil then
                            results[point.id][outId] = {distance=distance, pre=previousPoint}
                        else
                            if results[point.id][outId].distance > distance then
                                results[point.id][outId] = {distance=distance, pre=previousPoint}
                            end
                        end

                        -- First check if this outgoing point has already been completely analyzed before
                        local toBeAdded = true
                        if results[outId] ~= nil then
                            local allOutsTested = true
                            if outPoint.transitMapping[point.id] ~= nil then
                                for _, nextOutId in pairs(outPoint.transitMapping[point.id]) do
                                    -- Check for shortcut here as well!
                                    allOutsTested = allOutsTested and results[outId][nextOutId] ~= nil and not (results[outId][nextOutId].distance > distance)
                                end
                            end
                            if allOutsTested then
                                toBeAdded = false
                            end
                        end

                        -- Add this point then
                        if toBeAdded then --or (#point.incoming > 1)
                            local factor = 1
                            if isSubPrio(outPoint) then
                                factor = 20
                            end
                            local preventTurnaroundWeight = 0
                            if point.id == startID then
                                if not table.contains(preferredStartIds, outPoint.id) then
                                    preventTurnaroundWeight = 5000000
                                end
                            end
                            --candidates:enqueue({p=outPoint, distance=(distance + (distanceFunc(outPoint.x - point.x, outPoint.z - point.z) + (addedWeights[outPoint.id] or 0)) * factor + preventTurnaroundWeight), pre=point.id})
                            --candidates:enqueue({p=outPoint, distance=(distance + (distanceFunc(point, outPoint, previousPoint) + (addedWeights[outPoint.id] or 0)) * factor + preventTurnaroundWeight), pre=point.id})
                            candidates:enqueue({p=outPoint, distance=(distance + (distanceFunc2(point, outPoint, previousPoint) + (addedWeights[outPoint.id] or 0)) * factor + preventTurnaroundWeight + narrowWeight(point, outPoint)), pre=point.id})
                        end
                    end
                    point = nil
                else
                    -- We can continue analyzing if this is a segment with only one possible direction
                    if #point.transitMapping[previousPoint] == 1 then
                        local outPoint = network[point.transitMapping[previousPoint][1]]
                        if results[point.id][outPoint.id] == nil then
                            results[point.id][outPoint.id] = {distance=distance, pre=previousPoint}
                        else
                            if results[point.id][outPoint.id].distance > distance then
                                results[point.id][outPoint.id] = {distance=distance, pre=previousPoint}
                            end
                        end

                        local factor = 1
                        if isSubPrio(outPoint) then
                            factor = 20
                        end

                        --distance = distance + (distanceFunc(outPoint.x - point.x, outPoint.z - point.z) + (addedWeights[outPoint.id] or 0)) * factor
                        --distance = distance + (distanceFunc(point, outPoint, previousPoint) + (addedWeights[outPoint.id] or 0)) * factor
                        distance = distance + (distanceFunc2(point, outPoint, previousPoint) + (addedWeights[outPoint.id] or 0)) * factor + narrowWeight(point, outPoint)
                        
                        previousPoint = point.id
                        point = outPoint
                    else
                        point = nil
                    end
                end
            end
        end
    end
    
    if not foundTarget then
        return {}
    end

    --Now we just have to reverse engineer the path
    local inversePath = {}
    local point = network[targetID]
    local counter = 0
    while point ~= nil do
        counter = counter + 1
        local previousPoint = network[lastPredecessor]
        lastPredecessor = results[lastPredecessor][point.id].pre

        table.insert(inversePath, point)
        point = previousPoint

        if lastPredecessor == -1 then
            point = nil
        end
        --emergency stop - we have some loop in the graph that went through till here
        if counter >= 500000 then
            return {}
        end
    end

    local path  = {}
    local count = #inversePath
    for i=0, count-1 do
        path[count-i] = inversePath[i+1]
    end

    if vehicle ~= nil then
        local narrow = 0
        for i = 1, #path - 1 do
            if narrowWeight(path[i], path[i + 1]) > 0 then
                narrow = narrow + 1
            end
        end
        if narrow > 0 then
            AutoDrive.debugPrint(vehicle, AutoDrive.DC_PATHINFO, "ADPathCalculator:GetPath no wide enough route, %d narrow section(s) kept (needs %.1f m)", narrow, ADPathCalculator.lastNeededWidth or 0)
        end
    end

    return path
end

function ADPathCalculator:getDetourWeights()
    local addedWeights = {}
    local maxDetour = AutoDrive.getSetting("mapMarkerDetour")
    if maxDetour > 0 then
        for _, marker in pairs(ADGraphManager:getMapMarkers()) do
            addedWeights[marker.id] = maxDetour
        end
    end
    return addedWeights
end

-- route width check ------------------------------------------------------------------------------
ADPathCalculator.NARROW_WEIGHT = 100000        -- added to a too narrow section: used only if there is no other way
ADPathCalculator.WIDTH_MARGIN = 0.05           -- m free on each side of the train
ADPathCalculator.WIDTH_CHECK_LOW = 0.3         -- m above ground: curbs and verges are ignored
ADPathCalculator.WIDTH_CHECK_HIGH = 3.0        -- m above ground
ADPathCalculator.widthMask = CollisionFlag.DEFAULT + CollisionFlag.STATIC_OBJECT + CollisionFlag.TREE + CollisionFlag.BUILDING
ADPathCalculator.widthCache = {}               -- [halfWidth][fromId][toId] = true when too narrow, false when fine

ADPathCalculator.FOLDED_TOOL_WIDTH = 2.4        -- m, transport width counted for a foldable tool

-- width the train needs to pass: tractor measured at its tyres, trailers at their size, foldable tools at their transport
-- width (their declared size is often the unfolded one)
function ADPathCalculator.getPassageWidth(vehicle)
    local width = 0
    local units = AutoDrive.getAllUnits(vehicle)
    if units ~= nil then
        for _, unit in pairs(units) do
            if unit.size ~= nil and unit.size.width ~= nil then
                local unitWidth = unit.size.width
                local foldable = unit.spec_foldable
                if unit == vehicle then
                    unitWidth = ADPathCalculator.getWheelsWidth(vehicle) or unitWidth
                elseif foldable ~= nil and foldable.foldingParts ~= nil and #foldable.foldingParts > 0 then
                    unitWidth = math.min(unitWidth, ADPathCalculator.FOLDED_TOOL_WIDTH)
                end
                width = math.max(width, unitWidth)
            end
        end
    end
    if width <= 0 then
        width = 3
    end
    return width
end

-- width from the outside of the left tyre to the outside of the right tyre, mounted (visible) dual
-- wheels included; nil when it cannot be measured
function ADPathCalculator.getWheelsWidth(vehicle)
    local wheels = vehicle.spec_wheels ~= nil and vehicle.spec_wheels.wheels or nil
    local root = vehicle.rootNode or (vehicle.components ~= nil and vehicle.components[1] ~= nil and vehicle.components[1].node)
    if wheels == nil or root == nil or root == 0 then
        return nil
    end
    local minX, maxX = math.huge, -math.huge
    local function add(node, width)
        if node == nil or node == 0 or type(width) ~= "number" or width <= 0 then
            return
        end
        local ok, visible = pcall(getEffectiveVisibility, node)
        if ok and visible == false then
            return -- dual wheel not mounted
        end
        local okPos, x = pcall(localToLocal, node, root, 0, 0, 0)
        if okPos and type(x) == "number" then
            minX = math.min(minX, x - width / 2)
            maxX = math.max(maxX, x + width / 2)
        end
    end
    for _, wheel in ipairs(wheels) do
        local physicsWidth = wheel.physics ~= nil and wheel.physics.width or nil
        if wheel.visualWheels ~= nil and #wheel.visualWheels > 0 then
            for _, visualWheel in ipairs(wheel.visualWheels) do
                add(visualWheel.node, visualWheel.width or physicsWidth)
            end
        else
            add(wheel.repr or wheel.driveNode or wheel.node, physicsWidth)
        end
    end
    if maxX <= minX or maxX - minX < 1 or maxX - minX > 10 then
        return nil
    end
    local width = maxX - minX
    return width
end

function ADPathCalculator:getNarrowWeightFunc(vehicle)
    if vehicle == nil or g_currentMission == nil or g_currentMission.terrainRootNode == nil then
        return function() return 0 end
    end
    local width = ADPathCalculator.getPassageWidth(vehicle)
    -- rounded to 10 cm so vehicles of about the same width share the measures
    local halfWidth = math.ceil((width / 2 + ADPathCalculator.WIDTH_MARGIN) * 10) / 10
    ADPathCalculator.lastNeededWidth = halfWidth * 2
    ADPathCalculator.widthCache[halfWidth] = ADPathCalculator.widthCache[halfWidth] or {}
    local cache = ADPathCalculator.widthCache[halfWidth]
    return function(a, b)
        local fromCache = cache[a.id]
        if fromCache == nil then
            fromCache = {}
            cache[a.id] = fromCache
        end
        local narrow = fromCache[b.id]
        if narrow == nil then
            narrow = ADPathCalculator.isSectionNarrow(a, b, halfWidth)
            fromCache[b.id] = narrow
        end
        if narrow then
            return ADPathCalculator.NARROW_WEIGHT
        end
        return 0
    end
end

function ADPathCalculator.isSectionNarrow(a, b, halfWidth)
    local dx, dz = b.x - a.x, b.z - a.z
    local length = math.sqrt(dx * dx + dz * dz)
    if length < 0.1 then
        return false
    end
    local cx, cz = (a.x + b.x) / 2, (a.z + b.z) / 2
    -- height of the network line (road or bridge deck), not of the terrain under a bridge
    local y
    if type(a.y) == "number" and type(b.y) == "number" and a.y ~= -1 and b.y ~= -1 then
        y = (a.y + b.y) / 2
    else
        y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, cx, 1, cz)
    end
    local low, high = ADPathCalculator.WIDTH_CHECK_LOW, ADPathCalculator.WIDTH_CHECK_HIGH
    ADPathCalculator.widthHits = 0
    ADPathCalculator.widthHitNode = nil
    overlapBox(cx, y + (low + high) / 2, cz, 0, math.atan2(dx, dz), 0, halfWidth, (high - low) / 2, length / 2, "widthCheckCallback", ADPathCalculator, ADPathCalculator.widthMask, true, true, true, true)
    if ADPathCalculator.widthHits > 0 then
        ADPathCalculator.recordNarrowSection(a, b, halfWidth, ADPathCalculator.widthHitNode)
        return true
    end
    return false
end

-- debug: narrow sections found, drawn in red with the AutoDrive sensor debug channel
ADPathCalculator.narrowSections = {}

function ADPathCalculator.recordNarrowSection(a, b, halfWidth, hitNode)
    local hitName, hx, hy, hz = "?", nil, nil, nil
    if hitNode ~= nil and hitNode ~= 0 then
        local okName, name = pcall(getName, hitNode)
        if okName and name ~= nil then
            hitName = name
        end
        local okParent, parent = pcall(getParent, hitNode)
        if okParent and parent ~= nil and parent ~= 0 then
            local okParentName, parentName = pcall(getName, parent)
            if okParentName and parentName ~= nil then
                hitName = parentName .. "/" .. hitName
            end
        end
        local okPos, x, y, z = pcall(getWorldTranslation, hitNode)
        if okPos then
            hx, hy, hz = x, y, z
        end
    end
    table.insert(ADPathCalculator.narrowSections, {a = a, b = b, halfWidth = halfWidth, hitName = hitName, hx = hx, hy = hy, hz = hz})
end

local function drawRectangle(x, y, z, dirX, dirZ, halfWidth, front, back, r, g, b)
    local sideX, sideZ = dirZ, -dirX
    local corners = {
        {x + dirX * front + sideX * halfWidth, z + dirZ * front + sideZ * halfWidth},
        {x + dirX * front - sideX * halfWidth, z + dirZ * front - sideZ * halfWidth},
        {x - dirX * back - sideX * halfWidth, z - dirZ * back - sideZ * halfWidth},
        {x - dirX * back + sideX * halfWidth, z - dirZ * back + sideZ * halfWidth}
    }
    for i = 1, 4 do
        local c1, c2 = corners[i], corners[i % 4 + 1]
        ADDrawingManager:addLineTask(c1[1], y, c1[2], c2[1], y, c2[2], 1, r, g, b)
    end
end

function ADPathCalculator.debugDraw()
    local vehicle = AutoDrive.getControlledVehicle()
    if vehicle == nil or vehicle.components == nil or g_currentMission == nil or g_currentMission.terrainRootNode == nil then
        return
    end
    local vx, vy, vz = getWorldTranslation(vehicle.components[1].node)
    local dirX, _, dirZ = AutoDrive.localDirectionToWorld(vehicle, 0, 0, 1)
    local length = vehicle.size ~= nil and vehicle.size.length or 5
    -- blue: width taken into account for narrow passages
    local halfWidth = ADPathCalculator.getPassageWidth(vehicle) / 2
    drawRectangle(vx, vy + 0.3, vz, dirX, dirZ, halfWidth, length / 2, length / 2, 0, 0.4, 1)
    -- red: sections refused as too narrow, and the object that was hit
    local maxDistance = AutoDrive.drawDistance or 300
    for _, section in ipairs(ADPathCalculator.narrowSections) do
        local a, b = section.a, section.b
        local cx, cz = (a.x + b.x) / 2, (a.z + b.z) / 2
        if MathUtil.vector2Length(cx - vx, cz - vz) < maxDistance then
            local dx, dz = b.x - a.x, b.z - a.z
            local sectionLength = math.sqrt(dx * dx + dz * dz)
            if sectionLength > 0.1 then
                local y = ((type(a.y) == "number" and type(b.y) == "number" and a.y ~= -1 and b.y ~= -1) and (a.y + b.y) / 2 or getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, cx, 1, cz)) + ADPathCalculator.WIDTH_CHECK_LOW
                drawRectangle(cx, y, cz, dx / sectionLength, dz / sectionLength, section.halfWidth, sectionLength / 2, sectionLength / 2, 1, 0, 0)
            end
            if section.hx ~= nil then
                ADDrawingManager:addLineTask(section.hx, section.hy, section.hz, section.hx, section.hy + 4, section.hz, 1, 1, 0, 0)
            end
        end
    end
end

function ADPathCalculator.isMapBorder(node)
    for _ = 1, 2 do
        if node == nil or node == 0 then
            return false
        end
        local ok, name = pcall(getName, node)
        if ok and type(name) == "string" and string.find(string.lower(name), "border", 1, true) ~= nil then
            return true
        end
        local okParent, parent = pcall(getParent, node)
        node = okParent and parent or nil
    end
    return false
end

function ADPathCalculator:widthCheckCallback(transformId)
    if transformId == 0 or transformId == g_currentMission.terrainRootNode then
        return true
    end
    if ADPathCalculator.isMapBorder(transformId) then
        return true -- invisible wall at the map edge
    end
    local object = g_currentMission:getNodeObject(transformId)
    if object ~= nil and (object.rootVehicle ~= nil or object.isa ~= nil and object:isa(Vehicle)) then
        return true -- vehicles move, they are not part of the road width
    end
    ADPathCalculator.widthHits = ADPathCalculator.widthHits + 1
    ADPathCalculator.widthHitNode = transformId
    return false
end
