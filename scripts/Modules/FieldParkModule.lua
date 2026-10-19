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
ADFieldPark.ROAD_CLEARANCE = 4                      -- m between the parked train and any AutoDrive waypoint
ADFieldPark.ENTRANCE_CLEARANCE = 40                 -- m between the parked train and the place where the unloader waits (field entrance)
ADFieldPark.HARVESTER_RANGE = 400                   -- m around the unloader where its harvester is looked for
ADFieldPark.PLANNING_TIMEOUT = 20000                -- ms given to the pathfinder for one spot
ADFieldPark.RETRY_DELAY = 30000                     -- ms before looking again when no spot could be reached
ADFieldPark.MIN_TURN_RADIUS = 6                     -- m, tightest turn tried for the direct path
ADFieldPark.CROP_MARGIN = 0.7                       -- m added on each side of the train when checking the crop
ADFieldPark.FIELD_MARGIN = 0.5                      -- m kept between the footprint (already 0.5 m wider than the train) and the field border
ADFieldPark.BORDER_SEARCH = 25                      -- m looked at around a ring position to find the field border
ADFieldPark.BORDER_RAY_STEP = 15                    -- degrees between two rays cast to find the border
ADFieldPark.BORDER_BASE = 12                        -- m between the two border points that give its direction
ADFieldPark.NOSE_SEARCH = 30                        -- m looked at ahead of the tractor to find the field it waits at
ADFieldPark.HARVESTER_CLEARANCE = 20                -- m kept between the parked train and the standing crop: room for the harvester to turn on the headland
ADFieldPark.CLEARANCE_STEP = 4                      -- m between two crop samples around the train
ADFieldPark.PARALLEL_TOLERANCE = 5                  -- degrees, a parked train within this angle of the border counts as parallel
ADFieldPark.BORDER_GAPS = {0.5, 1, 1.5, 2, 3, 4}    -- m tried between the footprint (0.5 m wider than the train) and the border
ADFieldPark.SAMPLE_STEP = 2                         -- m between two ground samples
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
    return not (fruit:getIsCut(growthState) or fruit:getIsWithered(growthState)), fruitTypeIndex
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
-- points every ADFieldPark.SAMPLE_STEP on the outline of a footprint, plus its centre line
function ADFieldPark.getOutline(cx, cz, dx, dz, halfW, halfL)
    local points = {}
    local function add(across, along)
        -- GIANTS local +x (left) of the heading is (dz, -dx)
        table.insert(points, {cx + dz * across + dx * along, cz - dx * across + dz * along})
    end
    local n = math.max(1, math.ceil(2 * halfL / ADFieldPark.SAMPLE_STEP))
    for i = 0, n do
        local along = -halfL + 2 * halfL * i / n
        add(-halfW, along)
        add(0, along)
        add(halfW, along)
    end
    local m = math.max(1, math.ceil(2 * halfW / ADFieldPark.SAMPLE_STEP))
    for i = 1, m - 1 do
        local across = -halfW + 2 * halfW * i / m
        add(across, -halfL)
        add(across, halfL)
    end
    return points
end

-- field ground of the harvester: the ground connected to it, as the AutoDrive pathfinder sees it, so that a
-- neighbouring field (another number, behind a hedge or a grass strip) never counts as this field
function ADFieldPark.getHarvesterArea(hx, hz)
    if PathFinderModule ~= nil and PathFinderModule.getConnectedFieldArea ~= nil and PathFinderModule.isInFieldArea ~= nil then
        return PathFinderModule.getConnectedFieldArea(hx, hz)
    end
    return nil
end

-- on the field of the harvester being looked at (ADFieldPark.area), or on any field when it is not known
function ADFieldPark.isOnField(x, y, z)
    if not AutoDrive.checkIsOnField(x, y, z) then
        return false
    end
    local area = ADFieldPark.area
    return area == nil or area.capped or PathFinderModule.isInFieldArea(area, x, z)
end

-- standing crop within ADFieldPark.HARVESTER_CLEARANCE around the footprint: the harvester still works there and
-- turns on the harvested ground next to it. Small forgotten patches do not count
function ADFieldPark.hasCropNearby(cx, cz, dx, dz, halfW, halfL)
    local clearance, step = ADFieldPark.HARVESTER_CLEARANCE, ADFieldPark.CLEARANCE_STEP
    local cache = {}
    for along = -(halfL + clearance), halfL + clearance, step do
        for across = -(halfW + clearance), halfW + clearance, step do
            -- GIANTS local +x (left) of the heading is (dz, -dx)
            local x, z = cx + dz * across + dx * along, cz - dx * across + dz * along
            local crop, fruitTypeIndex = ADFieldPark.hasGrowingCrop(x, z)
            if crop and not ADFieldPark.isCropIsland(fruitTypeIndex, x, z, cache) then
                return true
            end
        end
    end
    return false
end

function ADFieldPark.isGroundFree(cx, cz, dx, dz, halfW, halfL, nearbyWayPoints, py)
    local minH, maxH = math.huge, -math.huge
    -- the whole outline on the field with a margin, on harvested ground (no crop of any kind)
    local margin = ADFieldPark.FIELD_MARGIN
    for _, p in ipairs(ADFieldPark.getOutline(cx, cz, dx, dz, halfW + margin, halfL + margin)) do
        local h = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, p[1], py, p[2])
        minH, maxH = math.min(minH, h), math.max(maxH, h)
        if not ADFieldPark.isOnField(p[1], h, p[2]) or ADFieldPark.hasGrowingCrop(p[1], p[2]) then
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
    if ADFieldPark.hasCropNearby(cx, cz, dx, dz, halfW, halfL) then
        return nil
    end
    return maxH
end

-- true when the current position is already a good waiting place
function ADFieldPark.isGoodWaitingPlace(vehicle)
    local x, py, z = ADTrafficYieldModule.getPosition(vehicle)
    ADFieldPark.area = nil
    local harvester = ADFieldPark.findHarvester(vehicle, x, z)
    if harvester ~= nil then
        local hx, _, hz = getWorldTranslation(harvester.components[1].node)
        ADFieldPark.area = ADFieldPark.getHarvesterArea(hx, hz)
    end
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
    for _, p in ipairs(ADFieldPark.getOutline(cx, cz, dx, dz, halfW, length / 2 + 0.5)) do
        if not ADFieldPark.isOnField(p[1], py, p[2]) then
            return false
        end
    end
    if ADFieldPark.hasCropNearby(cx, cz, dx, dz, halfW, length / 2 + 0.5) then
        return false
    end
    -- and parked parallel to the border, close to it
    local bx, bz, tx, tz, nx, nz = ADFieldPark.findBorder(cx, cz, py)
    if bx == nil or math.abs(dx * tx + dz * tz) < math.cos(math.rad(ADFieldPark.PARALLEL_TOLERANCE)) then
        return false
    end
    return (bx - cx) * nx + (bz - cz) * nz - halfW < ADFieldPark.BORDER_GAPS[#ADFieldPark.BORDER_GAPS] + 0.5
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
    -- the field to park in is the one the nose of the tractor points to (it waits at its entrance); the
    -- harvester must work in that one, otherwise the unloader waits for it at the entrance
    local noseArea = nil
    for d = 2, ADFieldPark.NOSE_SEARCH, 2 do
        local fx, fz = search.x + search.vdx * d, search.z + search.vdz * d
        if AutoDrive.checkIsOnField(fx, search.py, fz) then
            noseArea = ADFieldPark.getHarvesterArea(fx, fz)
            break
        end
    end
    search.area = noseArea or ADFieldPark.getHarvesterArea(hx, hz)
    if noseArea ~= nil and not noseArea.capped and not PathFinderModule.isInFieldArea(noseArea, hx, hz) then
        search.result = "harvester not in the field ahead"
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
    ADFieldPark.area = search.area
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
-- distance from (x, z) to the field border along (nx, nz), nil when the border is farther than maxDistance
function ADFieldPark.castToBorder(x, z, nx, nz, maxDistance, py)
    for d = 0.5, maxDistance, 0.5 do
        if not ADFieldPark.isOnField(x + nx * d, py, z + nz * d) then
            -- narrowed down to a few centimeters, the direction of the border depends on it
            local inside, outside = d - 0.5, d
            for _ = 1, 4 do
                local mid = (inside + outside) / 2
                if ADFieldPark.isOnField(x + nx * mid, py, z + nz * mid) then
                    inside = mid
                else
                    outside = mid
                end
            end
            return (inside + outside) / 2
        end
    end
    return nil
end

-- border of the field near (cx, cz): a point on it, its direction (tx, tz) and the normal (nx, nz) pointing out
-- of the field, measured on the ground so that the train can be parked exactly parallel to it
function ADFieldPark.findBorder(cx, cz, py)
    local best, nx, nz = nil, 0, 0
    for a = 0, 345, ADFieldPark.BORDER_RAY_STEP do
        local rx, rz = math.sin(math.rad(a)), math.cos(math.rad(a))
        local d = ADFieldPark.castToBorder(cx, cz, rx, rz, ADFieldPark.BORDER_SEARCH, py)
        if d ~= nil and (best == nil or d < best) then
            best, nx, nz = d, rx, rz
        end
    end
    if best == nil then
        return nil
    end
    -- two points of the border, on each side, give its direction; done twice to settle on the real normal
    local half = ADFieldPark.BORDER_BASE / 2
    local px, pz, tx, tz
    for _ = 1, 2 do
        tx, tz = nz, -nx
        local d1 = ADFieldPark.castToBorder(cx + tx * half, cz + tz * half, nx, nz, ADFieldPark.BORDER_SEARCH * 2, py)
        local d2 = ADFieldPark.castToBorder(cx - tx * half, cz - tz * half, nx, nz, ADFieldPark.BORDER_SEARCH * 2, py)
        if d1 == nil or d2 == nil then
            return nil
        end
        local x1, z1 = cx + tx * half + nx * d1, cz + tz * half + nz * d1
        local x2, z2 = cx - tx * half + nx * d2, cz - tz * half + nz * d2
        local length = MathUtil.vector2Length(x1 - x2, z1 - z2)
        if length < 0.1 then
            return nil
        end
        tx, tz = (x1 - x2) / length, (z1 - z2) / length
        -- normal pointing out of the field (same side as the first ray)
        local mx, mz = -tz, tx
        if mx * nx + mz * nz < 0 then
            mx, mz = -mx, -mz
        end
        nx, nz = mx, mz
        px, pz = (x1 + x2) / 2, (z1 + z2) / 2
    end
    return px, pz, tx, tz, nx, nz
end

function ADFieldPark.addCandidates(search, radius, angle)
    local halfW, halfL, py = search.halfW, search.halfL, search.py
    local a = math.rad(angle)
    local cx, cz = search.x + math.sin(a) * radius, search.z + math.cos(a) * radius
    if not ADFieldPark.isOnField(cx, py, cz) then
        return
    end
    -- always parallel to the border: the heading is the direction of the border itself, the train
    -- stands about 1 m away from it
    local bx, bz, tx, tz, nx, nz = ADFieldPark.findBorder(cx, cz, py)
    if bx == nil then
        return
    end
    for _, sign in ipairs({1, -1}) do
        local dx, dz = tx * sign, tz * sign
        for _, gap in ipairs(ADFieldPark.BORDER_GAPS) do
            local px, pz = bx - nx * (halfW + gap), bz - nz * (halfW + gap)
            local maxH = ADFieldPark.isSpotAllowed(search, px, pz, dx, dz) and ADFieldPark.isGroundFree(px, pz, dx, dz, halfW, halfL, search.nearby, py) or nil
            if maxH ~= nil then
                -- nose pointing away from the entrance: the train drives straight in along the border
                local ox, oz = px - search.x, pz - search.z
                local away = (dx * ox + dz * oz) / math.max(MathUtil.vector2Length(ox, oz), 0.1)
                local cost = 3 * gap + 0.2 * radius - 2 * (dx * search.vdx + dz * search.vdz) - 3 * away
                table.insert(search.candidates, {cx = px, cz = pz, dx = dx, dz = dz, maxH = maxH, cost = cost})
                break
            end
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
    -- fields the path may cross: the one of the spot and the one the unloader stands on, identified like the
    -- AutoDrive pathfinder does in strict mode (connected field ground, then map field outline, then farmland)
    local fieldRule = nil
    if g_farmlandManager ~= nil and PathFinderModule ~= nil and PathFinderModule.isFieldPositionAllowed ~= nil and PathFinderModule.getFieldAtPosition ~= nil then
        fieldRule = {strictField = true, allowedFields = {}, allowedFarmlands = {}, allowedAreas = {}}
        for _, pos in ipairs({{x = spot.x, z = spot.z}, {x = x, z = z}}) do
            if AutoDrive.checkIsOnField(pos.x, 0, pos.z) then
                local area = ADFieldPark.getHarvesterArea(pos.x, pos.z)
                if area ~= nil then
                    fieldRule.allowedAreas[area] = true
                end
                local field = PathFinderModule.getFieldAtPosition(pos.x, pos.z)
                if field ~= nil then
                    fieldRule.allowedFields[field] = true
                end
                fieldRule.allowedFarmlands[g_farmlandManager:getFarmlandIdAtWorldPosition(pos.x, pos.z)] = true
            end
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
                local blocked, bx, bz, index = ADFieldPark.isPathClear(vehicle, dubins.outPath, halfW, fieldRule)
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

function ADFieldPark.isCropIsland(fruitTypeIndex, x, z, cache)
    if PathFinderModule == nil or PathFinderModule.isFruitIsland == nil or AutoDrive.getFruitValue == nil then
        return false
    end
    return PathFinderModule.isFruitIsland({islandCache = cache}, fruitTypeIndex, x, z)
end

function ADFieldPark.isPathClear(vehicle, path, halfW, fieldRule)
    local islandCache = {}
    for i = 1, #path - 1 do
        local p, n = path[i], path[i + 1]
        local dx, dz = n.x - p.x, n.z - p.z
        local length = MathUtil.vector2Length(dx, dz)
        if length > 0.01 then
            dx, dz = dx / length, dz / length
            -- across the whole width of the train (GIANTS local +x (left) of the heading is (dz, -dx))
            for _, side in ipairs({0, -0.5, 0.5, -1, 1}) do
                local x, z = p.x + dz * side * halfW, p.z - dx * side * halfW
                local crop, fruitTypeIndex = ADFieldPark.hasGrowingCrop(x, z)
                -- small patches forgotten by the harvester may be driven over, like the AutoDrive pathfinder does
                if crop and not ADFieldPark.isCropIsland(fruitTypeIndex, x, z, islandCache) then
                    return "crop", x, z, i
                end
                -- grass and verges are fine, another field is not
                if fieldRule ~= nil and not PathFinderModule.isFieldPositionAllowed(fieldRule, x, z) then
                    return "other field", x, z, i
                end
            end
            -- obstacles (hedges, walls, poles, trees...) under the tractor, every 2 m
            if i > 2 and i % 2 == 0 and vehicle.ad.trafficYieldModule:overlapHit(p.x, (p.y or 0) + 1.9, p.z, math.atan2(dx, dz), halfW, 1.3, 2.5) then
                return "obstacle", p.x, p.z, i
            end
        end
    end
    -- nil when the path is clear, otherwise what blocks it and where
    return nil
end
