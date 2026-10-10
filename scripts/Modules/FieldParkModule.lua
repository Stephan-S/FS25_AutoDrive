--[[
    ADFieldPark
    -----------
    Separate evolution (2nd pull request, after the head-on passing one):
    an unloader waiting for a harvester call (CombineUnloaderMode -> WaitForCallTask) parks inside the field,
    along the field border, on ground without growing crop and away from the roads, instead of blocking the
    road at the field entrance.

    Depends on ADTrafficYieldModule (1st pull request) for the shared helpers: train dimensions
    (measureTrain, getWidth, getTrainExtents) and the exact obstacle test (overlapHit).
]]

ADFieldPark = {}
ADFieldPark.parked = {}                             -- vehicle -> spot where it is parked, waiting for a call

ADFieldPark.RADII = {45, 55, 65, 75, 85}            -- m, rings searched around the vehicle (beyond ENTRANCE_CLEARANCE)
ADFieldPark.ANGLE_STEP = 10                         -- degrees between two candidates on a ring
ADFieldPark.HEADING_STEP = 30                       -- degrees between two headings tried on each candidate
ADFieldPark.EDGE_SEARCH = 10                        -- m looked at beside the train to find the field border
ADFieldPark.ROAD_CLEARANCE = 4                      -- m between the parked train and any AutoDrive waypoint
ADFieldPark.ENTRANCE_CLEARANCE = 40                 -- m between the parked train and the place where the unloader waits (field entrance)
ADFieldPark.HARVESTER_RANGE = 400                   -- m around the unloader where its harvester is looked for
ADFieldPark.PLANNING_TIMEOUT = 20000                -- ms given to the pathfinder for one spot
ADFieldPark.RETRY_DELAY = 30000                     -- ms before looking again when no spot could be reached
ADFieldPark.MIN_TURN_RADIUS = 6                     -- m, tightest turn tried for the direct path
ADFieldPark.CROP_MARGIN = 0.25                      -- m added on each side of the train when checking the crop
ADFieldPark.QUEUE_GAP = 3                           -- m between two unloaders parked one behind the other
ADFieldPark.MAX_TRIES = 3                           -- spots tried in turn when no clean path leads to the previous one


function ADFieldPark.isEnabled()
    local value = AutoDrive.getSetting("fieldParkWhileWaiting")
    return value == true or value == 1
end

-- growing crop at a world position (cut, withered, grass and meadow do not count)
function ADFieldPark.hasGrowingCrop(x, z)
    if FSDensityMapUtil == nil or FSDensityMapUtil.getFruitTypeIndexAtWorldPos == nil then
        return false
    end
    local fruitTypeIndex, growthState = FSDensityMapUtil.getFruitTypeIndexAtWorldPos(x, z)
    if fruitTypeIndex == nil or fruitTypeIndex == 0 or growthState == nil then
        return false
    end
    if fruitTypeIndex == FruitType.MEADOW or fruitTypeIndex == FruitType.GRASS then
        return false
    end
    local fruit = g_fruitTypeManager:getFruitTypeByIndex(fruitTypeIndex)
    if fruit == nil then
        return false
    end
    return not (fruit:getIsCut(growthState) or fruit:getIsWithered(growthState))
end

-- AutoDrive waypoints around a position (computed once per search)
function ADFieldPark.getWayPointsAround(x, z, radius)
    local result = {}
    local wayPoints = ADGraphManager:getWayPoints()
    if wayPoints == nil then
        return result
    end
    for _, wp in pairs(wayPoints) do
        if math.abs(wp.x - x) < radius and math.abs(wp.z - z) < radius then
            table.insert(result, wp)
        end
    end
    return result
end

-- distance from (px, pz) to the footprint rectangle centred at (cx, cz) heading (dx, dz)
function ADFieldPark.distanceToFootprint(px, pz, cx, cz, dx, dz, halfW, halfL)
    local along = (px - cx) * dx + (pz - cz) * dz
    local side = (px - cx) * dz - (pz - cz) * dx
    local outAlong = math.max(math.abs(along) - halfL, 0)
    local outSide = math.max(math.abs(side) - halfW, 0)
    return math.sqrt(outAlong * outAlong + outSide * outSide)
end

-- ground under the footprint of the train centred at (cx, cz) heading (dx, dz): on the field, no crop, flat,
-- away from the roads. Returns the highest terrain height, or nil when the ground does not fit
function ADFieldPark.isGroundFree(cx, cz, dx, dz, halfW, halfL, nearbyWayPoints, py)
    local samples = {{0, 0}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}, {0, -1}, {0, 1}}
    local minH, maxH = math.huge, -math.huge
    for _, s in ipairs(samples) do
        -- GIANTS local +x (left) of the heading is (dz, -dx)
        local x = cx + dz * s[1] * halfW + dx * s[2] * halfL
        local z = cz - dx * s[1] * halfW + dz * s[2] * halfL
        local h = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, py, z)
        minH, maxH = math.min(minH, h), math.max(maxH, h)
        if not AutoDrive.checkIsOnField(x, h, z) or ADFieldPark.hasGrowingCrop(x, z) then
            return nil
        end
    end
    if maxH - minH > ADTrafficYieldModule.MAX_SLOPE_DIFF then
        return nil
    end
    -- the whole train, trailers included, away from the road network
    for _, wp in ipairs(nearbyWayPoints) do
        if ADFieldPark.distanceToFootprint(wp.x, wp.z, cx, cz, dx, dz, halfW, halfL) < ADFieldPark.ROAD_CLEARANCE then
            return nil
        end
    end
    return maxH
end

-- free ground between one side of the footprint and the field border, at a point of the train (along = -1 rear, 1 front)
function ADFieldPark.getEdgeDistance(cx, cz, dx, dz, halfW, halfL, sideSign, along, py)
    local bx = cx + dx * along * halfL * 0.8
    local bz = cz + dz * along * halfL * 0.8
    for d = 1, ADFieldPark.EDGE_SEARCH do
        local x = bx + dz * sideSign * (halfW + d)
        local z = bz - dx * sideSign * (halfW + d)
        if not AutoDrive.checkIsOnField(x, py, z) then
            return d - 1
        end
    end
    return ADFieldPark.EDGE_SEARCH
end

-- how well the footprint lines up along the field border (lower is better): both ends of the train close to
-- the border on the same side, so it stays on the edge of the field, out of the harvester's way
function ADFieldPark.getEdgeCost(cx, cz, dx, dz, halfW, halfL, py)
    local best, bestSide, bestGap = math.huge, 1, 0
    for _, sideSign in ipairs({1, -1}) do
        local front = ADFieldPark.getEdgeDistance(cx, cz, dx, dz, halfW, halfL, sideSign, 1, py)
        local rear = ADFieldPark.getEdgeDistance(cx, cz, dx, dz, halfW, halfL, sideSign, -1, py)
        local cost = front + rear + 2 * math.abs(front - rear)
        if cost < best then
            best, bestSide, bestGap = cost, sideSign, math.min(front, rear)
        end
    end
    -- returns also the side of the border and the free gap left between the train and it
    return best, bestSide, bestGap
end

-- true when the current position is already a good waiting place
function ADFieldPark.isGoodWaitingPlace(vehicle)
    local x, py, z = ADTrafficYieldModule.getPosition(vehicle)
    local dx, dz = ADTrafficYieldModule.getDirection(vehicle)
    local front, rear = ADTrafficYieldModule.getTrainExtents(vehicle)
    local length = front + rear
    -- centre of the whole train
    local cx, cz = x + dx * (front - length / 2), z + dz * (front - length / 2)
    local halfW = ADTrafficYieldModule.getWidth(vehicle) / 2 + 0.5
    local nearby = ADFieldPark.getWayPointsAround(x, z, 60)
    for _, wp in ipairs(nearby) do
        if ADFieldPark.distanceToFootprint(wp.x, wp.z, cx, cz, dx, dz, halfW, length / 2 + 0.5) < ADFieldPark.ROAD_CLEARANCE then
            return false
        end
    end
    return AutoDrive.checkIsOnField(cx, py, cz)
end

-- harvester the unloader works with: same field marker first, otherwise the closest registered harvester
-- (driven by a helper or another mod), within ADFieldPark.HARVESTER_RANGE
function ADFieldPark.findHarvester(vehicle, x, z)
    local best, bestDistance = nil, ADFieldPark.HARVESTER_RANGE
    local marker = vehicle.ad.stateModule ~= nil and vehicle.ad.stateModule:getFirstMarker() or nil
    for _, list in ipairs({ADHarvestManager.harvesters or {}, ADHarvestManager.idleHarvesters or {}}) do
        for _, harvester in pairs(list) do
            if harvester.components ~= nil then
                local hx, _, hz = getWorldTranslation(harvester.components[1].node)
                local distance = MathUtil.vector2Length(hx - x, hz - z)
                local sameMarker = marker ~= nil and harvester.ad ~= nil and harvester.ad.stateModule ~= nil
                    and harvester.ad.stateModule:getFirstMarker() == marker
                if sameMarker then
                    distance = 0
                end
                if distance < bestDistance or (best == nil and distance <= bestDistance) then
                    best, bestDistance = harvester, distance
                end
            end
        end
    end
    return best
end

-- starts a search of waiting places inside the field around the vehicle. The search is spread over several
-- frames (one candidate position per call of ADFieldPark.searchStep) to avoid a hitch
function ADFieldPark.newSearch(vehicle)
    ADTrafficYieldModule.measureTrain(vehicle)
    local search = {vehicle = vehicle}
    search.x, search.py, search.z = ADTrafficYieldModule.getPosition(vehicle)
    search.vdx, search.vdz = ADTrafficYieldModule.getDirection(vehicle)
    search.halfW = ADTrafficYieldModule.getWidth(vehicle) / 2 + 0.5
    search.front, search.rear = ADTrafficYieldModule.getTrainExtents(vehicle)
    search.length = search.front + search.rear
    search.halfL = search.length / 2 + 0.5
    local maxRadius = ADFieldPark.RADII[#ADFieldPark.RADII]
    search.nearby = ADFieldPark.getWayPointsAround(search.x, search.z, maxRadius + search.length + 10)
    search.candidates = {}
    search.radiusIndex = 1
    search.angle = 0
    -- the harvester must already be working in the field: park along the border on its side (left or right of
    -- the entrance), where it has already harvested. Otherwise there is nothing to do, wait at the entrance
    local harvester = ADFieldPark.findHarvester(vehicle, search.x, search.z)
    if harvester == nil then
        search.result = "no harvester"
        return search
    end
    local hx, hy, hz = getWorldTranslation(harvester.components[1].node)
    if not AutoDrive.checkIsOnField(hx, hy, hz) then
        search.result = "harvester not in the field yet"
        return search
    end
    -- GIANTS local +x (left) of the heading is (dz, -dx)
    local lateral = (hx - search.x) * search.vdz - (hz - search.z) * search.vdx
    if math.abs(lateral) > 5 then
        search.side = lateral > 0 and 1 or -1
    end
    return search
end

-- one step of the search; returns nil while searching, then the list of spots best first: along the field
-- border, then close to the vehicle, then nose ahead. Each spot is {x, y, z, dirX, dirZ}, where the tractor stops
-- an unloader reached its spot / left it
function ADFieldPark.setParked(vehicle, spot)
    ADFieldPark.parked[vehicle] = spot
end

-- spots right behind, then right in front of the unloaders already parked nearby, in line with them, so the
-- unloaders queue along the border. They come before any other candidate
function ADFieldPark.addQueueCandidates(search)
    for other, spot in pairs(ADFieldPark.parked) do
        local valid = other ~= search.vehicle and other.isDeleted ~= true and other.components ~= nil and spot.cx ~= nil
        if valid then
            -- still standing there
            local ox, _, oz = ADTrafficYieldModule.getPosition(other)
            valid = MathUtil.vector2Length(ox - spot.x, oz - spot.z) < 5
        end
        if not valid then
            ADFieldPark.parked[other] = nil
        elseif MathUtil.vector2Length(spot.cx - search.x, spot.cz - search.z) < ADFieldPark.RADII[#ADFieldPark.RADII] + 50 then
            local offset = spot.halfL + ADFieldPark.QUEUE_GAP + search.halfL
            for rank, sign in ipairs({-1, 1}) do
                local cx, cz = spot.cx + spot.dirX * sign * offset, spot.cz + spot.dirZ * sign * offset
                local maxH = ADFieldPark.isSpotAllowed(search, cx, cz, spot.dirX, spot.dirZ)
                    and ADFieldPark.isGroundFree(cx, cz, spot.dirX, spot.dirZ, search.halfW, search.halfL, search.nearby, search.py) or nil
                if maxH ~= nil then
                    table.insert(search.candidates, {cx = cx, cz = cz, dx = spot.dirX, dz = spot.dirZ, maxH = maxH, cost = -1000 + rank})
                end
            end
        end
    end
end

function ADFieldPark.searchStep(search)
    if search.result ~= nil then
        return {}
    end
    if not search.queueDone then
        search.queueDone = true
        ADFieldPark.addQueueCandidates(search)
        return nil
    end
    local radius = ADFieldPark.RADII[search.radiusIndex]
    if radius ~= nil then
        ADFieldPark.addCandidates(search, radius, search.angle)
        search.angle = search.angle + ADFieldPark.ANGLE_STEP
        if search.angle >= 360 then
            search.angle = 0
            search.radiusIndex = search.radiusIndex + 1
        end
        return nil
    end
    return ADFieldPark.pickSpots(search)
end

-- on the harvester side of the entrance, and far enough from it
function ADFieldPark.isSpotAllowed(search, cx, cz, dx, dz)
    if search.side ~= nil then
        local lateral = (cx - search.x) * search.vdz - (cz - search.z) * search.vdx
        if lateral * search.side <= 0 then
            return false
        end
    end
    return ADFieldPark.distanceToFootprint(search.x, search.z, cx, cz, dx, dz, search.halfW, search.halfL) >= ADFieldPark.ENTRANCE_CLEARANCE
end

-- all the headings of the train centred on one point of a ring
function ADFieldPark.addCandidates(search, radius, angle)
    local halfW, halfL, py = search.halfW, search.halfL, search.py
    local a = math.rad(angle)
    local cx, cz = search.x + math.sin(a) * radius, search.z + math.cos(a) * radius
    if not AutoDrive.checkIsOnField(cx, py, cz) then
        return
    end
    for heading = 0, 359, ADFieldPark.HEADING_STEP do
        local h = math.rad(heading)
        local dx, dz = math.sin(h), math.cos(h)
        local px, pz = cx, cz
        local maxH = ADFieldPark.isSpotAllowed(search, px, pz, dx, dz) and ADFieldPark.isGroundFree(px, pz, dx, dz, halfW, halfL, search.nearby, py) or nil
        if maxH ~= nil then
            local edgeCost, side, gap = ADFieldPark.getEdgeCost(px, pz, dx, dz, halfW, halfL, py)
            -- the rings are coarse: slide the train sideways to about 1 m from the border when the ground allows it
            if gap > 1 and gap < ADFieldPark.EDGE_SEARCH then
                local sx, sz = px + dz * side * (gap - 1), pz - dx * side * (gap - 1)
                local slidMaxH = ADFieldPark.isSpotAllowed(search, sx, sz, dx, dz) and ADFieldPark.isGroundFree(sx, sz, dx, dz, halfW, halfL, search.nearby, py) or nil
                if slidMaxH ~= nil then
                    px, pz, maxH = sx, sz, slidMaxH
                    edgeCost = ADFieldPark.getEdgeCost(px, pz, dx, dz, halfW, halfL, py)
                end
            end
            -- nose pointing away from the entrance: the train drives straight in along the border
            local ox, oz = px - search.x, pz - search.z
            local away = (dx * ox + dz * oz) / math.max(MathUtil.vector2Length(ox, oz), 0.1)
            local cost = edgeCost + 0.2 * radius - 2 * (dx * search.vdx + dz * search.vdz) - 3 * away
            table.insert(search.candidates, {cx = px, cz = pz, dx = dx, dz = dz, maxH = maxH, cost = cost})
        end
    end
end

-- best candidates first, obstacle test (vehicles, bales, trees...) only on them
function ADFieldPark.pickSpots(search)
    local candidates = search.candidates
    if #candidates == 0 then
        search.result = "no harvested ground along the border"
    end
    table.sort(candidates, function(c1, c2) return c1.cost < c2.cost end)
    local spots = {}
    for _, c in ipairs(candidates) do
        if not search.vehicle.ad.trafficYieldModule:overlapHit(c.cx, c.maxH + 1.9, c.cz, math.atan2(c.dx, c.dz), search.halfW, 1.3, search.halfL) then
            -- the footprint is centred on (cx, cz): the tractor stops ahead of the centre so that the whole train,
            -- trailers included, stands inside it
            local shift = search.length / 2 - search.front
            local tx, tz = c.cx + c.dx * shift, c.cz + c.dz * shift
            table.insert(spots, {
                x = tx, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, tx, search.py, tz), z = tz,
                dirX = c.dx, dirZ = c.dz,
                -- footprint, for the next unloader that queues behind this one
                cx = c.cx, cz = c.cz, halfL = search.halfL,
                queued = c.cost < -900
            })
            if #spots >= ADFieldPark.MAX_TRIES then
                break
            end
        end
    end
    return spots
end

-- direct path to a spot: a Dubins curve from the unloader to the spot, checked every meter with the width of the
-- train against growing crop and every few meters against obstacles. The AutoDrive pathfinder works on a grid of
-- the turn radius size, too coarse for a harvested strip along the border, so it is only used when this fails
function ADFieldPark.findDirectPath(vehicle, spot)
    if ADDubins == nil then
        return nil
    end
    local x, _, z = ADTrafficYieldModule.getPosition(vehicle)
    local rx, rz = ADTrafficYieldModule.getDirection(vehicle)
    local q0 = {x, -z, AutoDrive.normalizeAngle(math.atan2(rx, rz) + math.pi * 1.5)}
    local q1 = {spot.x, -spot.z, AutoDrive.normalizeAngle(math.atan2(spot.dirX, spot.dirZ) + math.pi * 1.5)}
    -- the crop is checked with the real width of the train plus a small margin
    local halfW = ADTrafficYieldModule.getWidth(vehicle) / 2 + ADFieldPark.CROP_MARGIN
    -- turn radius: the AutoDrive one first (often a default of 9 m with a trailer), then the tighter one of the
    -- tractor itself, as a driver would do to enter a harvested strip along the border
    local radii = {}
    for _, radius in ipairs({AutoDrive.getDriverRadius(vehicle), AutoDrive.getDriverRadius(vehicle, true), ADFieldPark.MIN_TURN_RADIUS}) do
        if radius ~= nil and radius >= ADFieldPark.MIN_TURN_RADIUS and (#radii == 0 or radius < radii[#radii] - 0.5) then
            table.insert(radii, radius)
        end
    end
    local dubins = ADDubins:new()
    local reasons = nil
    for _, radius in ipairs(radii) do
        -- 0: shortest path, then each path type in turn
        for pathType = 0, 6 do
            dubins.outPath = {}
            local result
            if pathType == 0 then
                result = dubins:dubins_shortest_path(ADDubins.DubinsPath, q0, q1, radius)
            else
                result = dubins:dubins_path(ADDubins.DubinsPath, q0, q1, radius, pathType)
            end
            if result == ADDubins.EDUBOK
                and dubins:dubins_path_sample_many(ADDubins.DubinsPath, 1, dubins.createWayPoints) == ADDubins.EDUBOK
                and #dubins.outPath > 1 then
                local blocked, bx, bz, index = ADFieldPark.isPathClear(vehicle, dubins.outPath, halfW)
                if blocked == nil then
                    vehicle.ad.trafficYieldModule:log("field park: direct path with a turn radius of %.1f m", radius)
                    return dubins.outPath
                end
                if pathType == 0 then
                    reasons = (reasons or "") .. string.format(" [radius %.1f: %s at x=%.0f z=%.0f, point %d/%d]", radius, blocked, bx, bz, index, #dubins.outPath)
                end
            end
        end
    end
    vehicle.ad.trafficYieldModule:log("field park: no direct path from x=%.0f z=%.0f heading %.0f°:%s",
        x, z, math.deg(math.atan2(rx, rz)), reasons or " no curve")
    return nil
end

function ADFieldPark.isPathClear(vehicle, path, halfW)
    for i = 1, #path - 1 do
        local p, n = path[i], path[i + 1]
        local dx, dz = n.x - p.x, n.z - p.z
        local length = MathUtil.vector2Length(dx, dz)
        if length > 0.01 then
            dx, dz = dx / length, dz / length
            -- centre and both sides of the train (GIANTS local +x (left) of the heading is (dz, -dx))
            for _, side in ipairs({0, -1, 1}) do
                if ADFieldPark.hasGrowingCrop(p.x + dz * side * halfW, p.z - dx * side * halfW) then
                    return "crop", p.x, p.z, i
                end
            end
            -- obstacles (walls, poles, trees...) under the tractor, every 4 m, away from the start
            if i > 6 and i % 4 == 0 and vehicle.ad.trafficYieldModule:overlapHit(p.x, p.y + 1.9, p.z, math.atan2(dx, dz), halfW, 1.3, 2.5) then
                return "obstacle", p.x, p.z, i
            end
        end
    end
    -- nil when the path is clear, otherwise what blocks it and where
    return nil
end
