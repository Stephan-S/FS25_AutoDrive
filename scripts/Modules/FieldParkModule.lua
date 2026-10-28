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
-- nothing found on the harvester's side while harvested ground is free elsewhere along the border (near the entrance,
-- along the far headland): last pass, both sides, farther, closer to the entrance
ADFieldPark.WIDE_RADII = {30, 40, 55, 70, 85, 100, 115, 130, 145}
ADFieldPark.WIDE_ENTRANCE_CLEARANCE = 25
ADFieldPark.ENTRANCE_CLEARANCE = 40                 -- m between the parked train and the place where the unloader waits (field entrance)
ADFieldPark.HARVESTER_RANGE = 400                   -- m around the unloader where its harvester is looked for
ADFieldPark.PLANNING_TIMEOUT = 20000                -- ms given to the pathfinder search for one spot
ADFieldPark.SMOOTHING_TIMEOUT = 20000               -- ms more for the smoothing once the path is found
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
ADFieldPark.PARALLEL_TOLERANCE_PARKED = 10       -- degrees, same for a train found parked (loaded savegame): the border curves a little along it
ADFieldPark.PARALLEL_TOLERANCE = 5                  -- degrees, a parked train within this angle of the border counts as parallel
ADFieldPark.BORDER_GAPS = {-0.5, 0, 0.5, 1, 1.5, 2, 3, 4} -- m tried between the footprint (0.5 m wider than the train) and the border: as close to the grass as possible
ADFieldPark.GRASS_MARGIN = 1.1                      -- m: the margin around the parked train may lie on the grass this close to the field
ADFieldPark.CENTRE_OFFSETS = {0, 0.5, 1, 1.5, 2}    -- m from the border to the centre line of the train, into the field: astride the border first, half on the grass
ADFieldPark.ISLAND_MAX_AREA = 10                    -- m2: a crop patch this small, forgotten by the harvester, may be driven over
ADFieldPark.ISLAND_STEP = 0.5                       -- m between two samples of a crop patch
ADFieldPark.START_FREE = 15                         -- m from the unloader where the path may leave the road over the grass
ADFieldPark.START_ANY_FIELD = 12                    -- m from the unloader never refused for another field (entrance)
ADFieldPark.FRINGE = 3                              -- m: crop left by the harvester along the field edge, may be driven over
ADFieldPark.STRADDLE_GAPS = {0.5, 1, 1.5, 2, 3, 4, 0, -1, -2, -3} -- second pass: astride the border, half on the grass
ADFieldPark.SAMPLE_STEP = 2                         -- m between two ground samples
ADFieldPark.QUEUE_GAP = 3                           -- m between two unloaders parked one behind the other
ADFieldPark.MAX_TRIES = 3                           -- spots tried in turn when no clean path leads to the previous one
-- forage harvester (opening a field, narrow cut strip): a spot must leave room for another trailer to pass,
-- and stay out of the harvester's way
ADFieldPark.PASS_MIN_WIDTH = 4                      -- m of free cut ground beside the parked train
ADFieldPark.PASS_SEARCH = 12                        -- m looked at beside the train
ADFieldPark.LANE_LENGTH = 60                        -- m ahead of the harvester kept free
ADFieldPark.LANE_MARGIN = 2                         -- m added to the harvester half width and the train half width
ADFieldPark.HARVESTER_MIN_DISTANCE = 15             -- m between the parked train and the harvester
ADFieldPark.FIELD_CAPACITY = 2                      -- unloaders parked in the field for one harvester; the next ones wait beside the road
ADFieldPark.ROADSIDE_RANGE = 100                    -- m from the entrance where a roadside place is looked for
ADFieldPark.ROADSIDE_MIN = 15                       -- m kept free in front of the entrance
ADFieldPark.ROAD_HALF_WIDTH = 2.5                   -- m between an AutoDrive road line and a train parked beside it
ADFieldPark.ROADSIDE_STEP = 5                       -- m between two places tried along a road
ADFieldPark.MOTION_MIN = 2                          -- m the harvester must have moved to give its direction
ADFieldPark.MOTION_TTL = 120000                     -- ms a measured direction of the harvester stays valid
ADFieldPark.harvesterMotion = {}                    -- harvester -> {x, z, t, dx, dz, tDir, sense...}
ADFieldPark.TRACK_INTERVAL = 1000                   -- ms between two looks at a working harvester
ADFieldPark.SENSE_RUN = 30                          -- m driven along the border the same way: a headland, not a turn at a row end
ADFieldPark.SENSE_BORDER = 15                       -- m at most between the harvester on its headland and the border
ADFieldPark.memos = {}                              -- place remembered for each field (first unloader parked there)
ADFieldPark.MEMO_START_DISTANCE = 3                 -- m: an unloader starting this close to the remembered start takes the same path
ADFieldPark.MEMO_START_ANGLE = 20                   -- degrees, same for its heading
ADFieldPark.LEAD_EXTRA = 4                          -- m added to the train length for the end of the path along the border
ADFieldPark.LEAD_MIN = 8                            -- m, shortest end along the border (near a field corner)
ADFieldPark.LEAD_STEP = 2                           -- m between two points of the end along the border
ADFieldPark.LEAD_ACROSS = 8                         -- m looked at on each side to find the border at each point
ADFieldPark.QUEUE_ENTRANCE_CLEARANCE = 15           -- forage harvester: m kept from the entrance by an unloader queued behind another


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

-- crop strip the harvester left along the field edge (its header does not reach the edge): off field within
-- ADFieldPark.FRINGE in some direction
function ADFieldPark.isBorderFringe(x, z)
    local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 0, z)
    for _, d in ipairs({1, 2, ADFieldPark.FRINGE}) do
        for i = 0, 7 do
            local a = i * math.pi / 4
            if not AutoDrive.checkIsOnField(x + math.sin(a) * d, y, z + math.cos(a) * d) then
                return true
            end
        end
    end
    return false
end

-- field ground within distance of (x, z)
function ADFieldPark.isNearField(x, z, distance)
    -- 16 directions: a border seen at an angle is found within 2 % of the distance
    for _, d in ipairs({distance / 2, distance}) do
        for i = 0, 15 do
            local a = i * math.pi / 8
            if AutoDrive.checkIsOnField(x + math.sin(a) * d, 0, z + math.cos(a) * d) then
                return true
            end
        end
    end
    return false
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
    if ADFieldPark.relaxed then
        -- second pass: no place away from the crop, the harvested headland strip is better than the road
        return false
    end
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
        local onField = ADFieldPark.isOnField(p[1], h, p[2])
        -- second pass: the grass along the border is fine too, never another field
        -- and the margin of a train parked right against the grass
        local onGrass = not onField and not AutoDrive.checkIsOnField(p[1], h, p[2])
            and (ADFieldPark.relaxed or ADFieldPark.isNearField(p[1], p[2], halfW + margin + 0.2))
        if not onField and not onGrass then
            ADFieldPark.reject = AutoDrive.checkIsOnField(p[1], h, p[2]) and "other field" or "off field"
            return nil
        end
        local crop, fruitTypeIndex = ADFieldPark.hasGrowingCrop(p[1], p[2])
        if crop and not ADFieldPark.isCropIsland(fruitTypeIndex, p[1], p[2], {}) and not (ADFieldPark.relaxed and ADFieldPark.isBorderFringe(p[1], p[2])) then
            ADFieldPark.reject = "crop"
            return nil
        end
    end
    if maxH - minH > ADTrafficYieldModule.MAX_SLOPE_DIFF then
        ADFieldPark.reject = "slope"
        return nil
    end
    -- the whole train, trailers included, away from the road network
    for _, wp in ipairs(nearbyWayPoints) do
        if ADFieldPark.distanceToFootprint(wp.x, wp.z, cx, cz, dx, dz, halfW, halfL) < ADFieldPark.ROAD_CLEARANCE then
            ADFieldPark.reject = "road"
            return nil
        end
    end
    if ADFieldPark.hasCropNearby(cx, cz, dx, dz, halfW, halfL) then
        ADFieldPark.reject = "crop within 20 m"
        return nil
    end
    if not ADFieldPark.hasRunOut(cx, cz, dx, dz, halfW, halfL, py) then
        ADFieldPark.reject = "no run-out"
        return nil
    end
    return maxH
end

ADFieldPark.RUN_OUT = 15                            -- m of free ground needed ahead of the nose to drive away again
ADFieldPark.RUN_OUT_STEP = 2.5

-- the train must be able to leave its place straight ahead (not parked with its nose in a corner of the field,
-- against the border): the ground ahead of the nose, over the width of the train, is field or the grass
-- along the border, with no standing crop
function ADFieldPark.hasRunOut(cx, cz, dx, dz, halfW, halfL, py)
    local nx, nz = cx + dx * halfL, cz + dz * halfL
    local d = ADFieldPark.RUN_OUT_STEP
    while d <= ADFieldPark.RUN_OUT do
        for _, side in ipairs({0, -1, 1}) do
            -- GIANTS local +x (left) of the heading is (dz, -dx)
            local x, z = nx + dx * d + dz * side * halfW, nz + dz * d - dx * side * halfW
            local h = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, py, z)
            local onField = ADFieldPark.isOnField(x, h, z)
            local onGrass = not onField and not AutoDrive.checkIsOnField(x, h, z) and ADFieldPark.isNearField(x, z, halfW + 1)
            if not onField and not onGrass then
                return false
            end
            local crop, fruitTypeIndex = ADFieldPark.hasGrowingCrop(x, z)
            if crop and not ADFieldPark.isCropIsland(fruitTypeIndex, x, z, {}) and not ADFieldPark.isBorderFringe(x, z) then
                return false
            end
        end
        d = d + ADFieldPark.RUN_OUT_STEP
    end
    return true
end

-- true when the current position is already a good waiting place: on harvested ground along the border, away from
-- the road network. Same rules as a spot of the second pass (crop beyond the train allowed), so that a train parked
-- there by the field park, then saved and loaded again, stays where it is. Returns true and the spot (footprint), or
-- false and the reason
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
    local halfL = length / 2 + 0.5
    local nearby = ADFieldPark.getWayPointsAround(x, z, 60)
    for _, wp in ipairs(nearby) do
        if ADFieldPark.distanceToFootprint(wp.x, wp.z, cx, cz, dx, dz, halfW, halfL) < ADFieldPark.ROAD_CLEARANCE then
            return false, "road"
        end
    end
    for _, p in ipairs(ADFieldPark.getOutline(cx, cz, dx, dz, halfW, halfL)) do
        if not ADFieldPark.isOnField(p[1], py, p[2]) and AutoDrive.checkIsOnField(p[1], py, p[2]) then
            return false, "other field"
        end
        if not ADFieldPark.isOnField(p[1], py, p[2]) and not ADFieldPark.isNearField(p[1], p[2], halfW + 0.7) then
            return false, "off field"
        end
        local crop, fruitTypeIndex = ADFieldPark.hasGrowingCrop(p[1], p[2])
        if crop and not ADFieldPark.isCropIsland(fruitTypeIndex, p[1], p[2], {}) and not ADFieldPark.isBorderFringe(p[1], p[2]) then
            return false, "crop"
        end
    end
    if not ADFieldPark.hasRunOut(cx, cz, dx, dz, halfW, halfL, py) then
        return false, "no run-out"
    end
    -- and parked parallel to the border, close to it
    local bx, bz, tx, tz, nx, nz = ADFieldPark.findBorder(cx, cz, py)
    if bx == nil then
        return false, "no border"
    end
    if math.abs(dx * tx + dz * tz) < math.cos(math.rad(ADFieldPark.PARALLEL_TOLERANCE_PARKED)) then
        return false, string.format("not parallel to the border (%.0f°)", math.deg(math.acos(math.min(1, math.abs(dx * tx + dz * tz)))))
    end
    if (bx - cx) * nx + (bz - cz) * nz - halfW >= ADFieldPark.BORDER_GAPS[#ADFieldPark.BORDER_GAPS] + 0.5 then
        return false, "too far from the border"
    end
    return true, {x = x, y = py, z = z, dirX = dx, dirZ = dz, cx = cx, cz = cz, halfL = halfL, harvester = harvester}
end

-- direction the harvester drives in, measured on its positions (the node axis of some harvesters points backwards)
function ADFieldPark.updateHarvesterMotion(harvester)
    if harvester == nil or harvester.components == nil then
        return nil
    end
    local hx, _, hz = getWorldTranslation(harvester.components[1].node)
    local m = ADFieldPark.harvesterMotion[harvester]
    if m == nil then
        m = {x = hx, z = hz, t = g_time}
        ADFieldPark.harvesterMotion[harvester] = m
    end
    local d = MathUtil.vector2Length(hx - m.x, hz - m.z)
    if d >= ADFieldPark.MOTION_MIN then
        m.dx, m.dz, m.tDir = (hx - m.x) / d, (hz - m.z) / d, g_time
        m.x, m.z, m.t = hx, hz, g_time
    elseif g_time - m.t > 15000 then
        -- standing still: start a new measure from here
        m.x, m.z, m.t = hx, hz, g_time
    end
    if m.dx ~= nil and g_time - m.tDir < ADFieldPark.MOTION_TTL then
        return m.dx, m.dz
    end
    return nil
end

-- the way the harvester goes round the field along its border (headlands): +1 / -1 seen from the border outward
-- normal, nil while unknown. A parked train along any border faces the same way as the harvester passes there.
-- Measured once on its first headland (ADFieldPark.trackHarvester), kept for the whole field
function ADFieldPark.getHarvesterSense(search)
    local m = search.harvester ~= nil and ADFieldPark.harvesterMotion[search.harvester] or nil
    if m ~= nil and m.sense ~= nil then
        return m.sense
    end
    if search.memo ~= nil then
        return search.memo.sense
    end
    return nil
end

-- called for every working harvester (ADHarvestManager:update): follows it all the time and keeps the sense of its
-- first headland, ADFieldPark.SENSE_RUN m driven along the border the same way (not a turn at the end of a row)
function ADFieldPark.trackHarvester(harvester)
    if harvester == nil or harvester.components == nil then
        return
    end
    local m = ADFieldPark.harvesterMotion[harvester]
    if m ~= nil and m.trackTime ~= nil and g_time - m.trackTime < ADFieldPark.TRACK_INTERVAL and g_time >= m.trackTime then
        return
    end
    ADFieldPark.updateHarvesterMotion(harvester)
    m = ADFieldPark.harvesterMotion[harvester]
    m.trackTime = g_time
    local hx, hy, hz = getWorldTranslation(harvester.components[1].node)
    -- Courseplay (even standing still): the sense of the first headland is in the course of the harvester, known at once
    local courseSense = ADFieldPark.getCourseSense(harvester, m)
    if courseSense ~= nil and courseSense ~= m.sense then
        m.sense, m.senseArea, m.senseX, m.senseZ = courseSense, nil, hx, hz
        local memo = ADFieldPark.findMemo(harvester)
        if memo ~= nil and memo.sense ~= courseSense then
            ADFieldPark.forgetMemo(memo)
        end
    end
    if m.lastX == nil then
        m.lastX, m.lastZ = hx, hz
        return
    end
    local d = MathUtil.vector2Length(hx - m.lastX, hz - m.lastZ)
    if d < 1 then
        return
    end
    local mdx, mdz = (hx - m.lastX) / d, (hz - m.lastZ) / d
    m.lastX, m.lastZ = hx, hz
    if d > 50 or not AutoDrive.checkIsOnField(hx, hy, hz) then
        -- moved on the road, or teleported
        m.run, m.runSense = 0, nil
        return
    end
    if m.area == nil or (not m.area.capped and not PathFinderModule.isInFieldArea(m.area, hx, hz)) then
        m.area = ADFieldPark.getHarvesterArea(hx, hz)
    end
    -- another field: its first headland gives a new sense
    if m.sense ~= nil and m.senseArea ~= nil and m.area ~= m.senseArea
        and (m.senseArea.capped or not PathFinderModule.isInFieldArea(m.senseArea, hx, hz))
        and (not m.senseArea.capped or MathUtil.vector2Length(hx - m.senseX, hz - m.senseZ) > ADFieldPark.HARVESTER_RANGE) then
        m.sense, m.senseArea = nil, nil
    end
    if m.sense ~= nil then
        return
    end
    local area, relaxed = ADFieldPark.area, ADFieldPark.relaxed
    ADFieldPark.area, ADFieldPark.relaxed = m.area, false
    local bx, bz, tx, tz, nx, nz = ADFieldPark.findBorder(hx, hz, hy)
    ADFieldPark.area, ADFieldPark.relaxed = area, relaxed
    local s = nil
    if bx ~= nil and math.abs((bx - hx) * nx + (bz - hz) * nz) < ADFieldPark.SENSE_BORDER and math.abs(mdx * tx + mdz * tz) > 0.85 then
        s = (nx * mdz - nz * mdx) > 0 and 1 or -1
    end
    if s ~= nil and s == m.runSense then
        m.run = m.run + d
    else
        m.run, m.runSense = s ~= nil and d or 0, s
    end
    if s ~= nil and m.run >= ADFieldPark.SENSE_RUN then
        m.sense, m.senseArea, m.senseX, m.senseZ = s, m.area, hx, hz
        -- a remembered place of this field that faces the other way is from another harvest: forgotten
        local memo = ADFieldPark.findMemo(harvester)
        if memo ~= nil and memo.sense ~= s then
            ADFieldPark.forgetMemo(memo)
        end
    end
end

-- sense of the outermost headland of the Courseplay course of the harvester (or of the tractor pulling it): +1 / -1
-- like getHarvesterSense, from the signed area of the headland loop; nil without Courseplay course or headland
function ADFieldPark.getCourseSense(harvester, m)
    local ok, sense = pcall(function()
        for _, v in ipairs({harvester, harvester.getRootVehicle ~= nil and harvester:getRootVehicle() or nil, harvester.trailingVehicle}) do
            local course = v ~= nil and v.getFieldWorkCourse ~= nil and v:getFieldWorkCourse() or nil
            if course ~= nil and course.waypoints ~= nil then
                if m.course == course then
                    return m.courseSense
                end
                local first, last, prev = nil, nil, nil
                local area, n = 0, 0
                for _, wp in ipairs(course.waypoints) do
                    local a = wp.attributes
                    if a ~= nil and a:getHeadlandPassNumber() == 1 and not a:isIslandHeadland() and wp.x ~= nil and wp.z ~= nil then
                        if prev ~= nil then
                            area = area + prev.x * wp.z - wp.x * prev.z
                        end
                        first = first or wp
                        prev, last, n = wp, wp, n + 1
                    end
                end
                local s = nil
                if n >= 10 then
                    area = area + last.x * first.z - first.x * last.z
                    s = area > 0 and 1 or -1
                end
                m.course, m.courseSense = course, s
                return s
            end
        end
        return nil
    end)
    return ok and sense or nil
end

-- the unloaders may go into the field: the sense of the harvester is known, a place is remembered for its field,
-- or another unloader already stands there (the next ones queue behind it)
function ADFieldPark.canParkFor(vehicle, harvester)
    if harvester == nil then
        return true
    end
    local m = ADFieldPark.harvesterMotion[harvester]
    return (m ~= nil and m.sense ~= nil) or ADFieldPark.findMemo(harvester) ~= nil
        or ADFieldPark.countInField(vehicle, harvester) > 0
end

-- place found the first time in a field, used by all the next unloaders (no new search, same path). Kept in the
-- savegame (AutoDrive_fieldPark.xml) for the next game session
function ADFieldPark.getMemoFile()
    local mission = g_currentMission
    local dir = mission ~= nil and mission.missionInfo ~= nil and mission.missionInfo.savegameDirectory or nil
    if dir == nil then
        return nil
    end
    return dir .. "/AutoDrive_fieldPark.xml"
end

function ADFieldPark.loadMemos()
    if ADFieldPark.memosLoaded then
        return
    end
    ADFieldPark.memosLoaded = true
    local file = ADFieldPark.getMemoFile()
    if file == nil or not fileExists(file) then
        return
    end
    local xml = loadXMLFile("AutoDrive_fieldPark", file)
    if xml == nil or xml == 0 then
        return
    end
    local i = 0
    while true do
        local key = string.format("fieldPark.spot(%d)", i)
        local fx = getXMLFloat(xml, key .. "#fx")
        if fx == nil then
            break
        end
        local memo = {
            fx = fx, fz = getXMLFloat(xml, key .. "#fz"),
            cx = getXMLFloat(xml, key .. "#cx"), cz = getXMLFloat(xml, key .. "#cz"),
            dirX = getXMLFloat(xml, key .. "#dirX"), dirZ = getXMLFloat(xml, key .. "#dirZ"),
            sense = getXMLInt(xml, key .. "#sense")
        }
        local sx = getXMLFloat(xml, key .. "#sx")
        -- paths remembered before the straight lead-in (23/10 12h55) ended askew: not kept
        if sx ~= nil and getXMLBool(xml, key .. "#straight") == true then
            memo.start = {x = sx, z = getXMLFloat(xml, key .. "#sz"), dx = getXMLFloat(xml, key .. "#sdx"), dz = getXMLFloat(xml, key .. "#sdz")}
            memo.target = {x = getXMLFloat(xml, key .. "#tx"), z = getXMLFloat(xml, key .. "#tz")}
            memo.path = {}
            local j = 0
            while true do
                local pk = string.format("%s.p(%d)", key, j)
                local px = getXMLFloat(xml, pk .. "#x")
                if px == nil then
                    break
                end
                table.insert(memo.path, {x = px, y = getXMLFloat(xml, pk .. "#y"), z = getXMLFloat(xml, pk .. "#z")})
                j = j + 1
            end
            if #memo.path < 2 then
                memo.path, memo.start, memo.target = nil, nil, nil
            end
        end
        if memo.fz ~= nil and memo.cx ~= nil and memo.cz ~= nil and memo.dirX ~= nil and memo.dirZ ~= nil and memo.sense ~= nil then
            table.insert(ADFieldPark.memos, memo)
        end
        i = i + 1
    end
    delete(xml)
end

function ADFieldPark.saveMemos()
    local file = ADFieldPark.getMemoFile()
    if file == nil then
        return
    end
    local xml = createXMLFile("AutoDrive_fieldPark", file, "fieldPark")
    if xml == nil or xml == 0 then
        return
    end
    for i, memo in ipairs(ADFieldPark.memos) do
        local key = string.format("fieldPark.spot(%d)", i - 1)
        setXMLFloat(xml, key .. "#fx", memo.fx)
        setXMLFloat(xml, key .. "#fz", memo.fz)
        setXMLFloat(xml, key .. "#cx", memo.cx)
        setXMLFloat(xml, key .. "#cz", memo.cz)
        setXMLFloat(xml, key .. "#dirX", memo.dirX)
        setXMLFloat(xml, key .. "#dirZ", memo.dirZ)
        setXMLInt(xml, key .. "#sense", memo.sense)
        if memo.path ~= nil then
            setXMLBool(xml, key .. "#straight", true)
            setXMLFloat(xml, key .. "#sx", memo.start.x)
            setXMLFloat(xml, key .. "#sz", memo.start.z)
            setXMLFloat(xml, key .. "#sdx", memo.start.dx)
            setXMLFloat(xml, key .. "#sdz", memo.start.dz)
            setXMLFloat(xml, key .. "#tx", memo.target.x)
            setXMLFloat(xml, key .. "#tz", memo.target.z)
            for j, p in ipairs(memo.path) do
                local pk = string.format("%s.p(%d)", key, j - 1)
                setXMLFloat(xml, pk .. "#x", p.x)
                setXMLFloat(xml, pk .. "#y", p.y)
                setXMLFloat(xml, pk .. "#z", p.z)
            end
        end
    end
    saveXMLFile(xml)
    delete(xml)
end

-- remembered place of the field the harvester works in
function ADFieldPark.findMemo(harvester)
    if harvester == nil or harvester.components == nil then
        return nil
    end
    ADFieldPark.loadMemos()
    if #ADFieldPark.memos == 0 then
        return nil
    end
    local hx, _, hz = getWorldTranslation(harvester.components[1].node)
    local m = ADFieldPark.harvesterMotion[harvester]
    local area = m ~= nil and m.area or nil
    if area == nil or (not area.capped and not PathFinderModule.isInFieldArea(area, hx, hz)) then
        area = ADFieldPark.getHarvesterArea(hx, hz)
    end
    if area == nil then
        return nil
    end
    for _, memo in ipairs(ADFieldPark.memos) do
        if area.capped then
            if MathUtil.vector2Length(memo.cx - hx, memo.cz - hz) < ADFieldPark.HARVESTER_RANGE and AutoDrive.checkIsOnField(memo.fx, 0, memo.fz) then
                return memo
            end
        elseif PathFinderModule.isInFieldArea(area, memo.fx, memo.fz) then
            return memo
        end
    end
    return nil
end

function ADFieldPark.forgetMemo(memo)
    table.removeValue(ADFieldPark.memos, memo)
    ADFieldPark.saveMemos()
end

-- the first unloader reached its place in a field with no remembered place yet: remember it, with the path that led
-- there. Not for a place in line behind another unloader, nor beside the road
function ADFieldPark.rememberSpot(vehicle, spot, path, start)
    if spot == nil or spot.queued or spot.roadside or spot.memo ~= nil or spot.sense == nil or spot.harvester == nil then
        return
    end
    if ADFieldPark.findMemo(spot.harvester) ~= nil then
        return
    end
    -- a point surely inside the field, a few meters from the border, identifies the field
    local fx, fz = nil, nil
    for _, d in ipairs({4, 6, 8}) do
        for a = 0, 315, 45 do
            local x, z = spot.cx + math.sin(math.rad(a)) * d, spot.cz + math.cos(math.rad(a)) * d
            if fx == nil and AutoDrive.checkIsOnField(x, 0, z) and AutoDrive.checkIsOnField(spot.cx + math.sin(math.rad(a)) * (d + 2), 0, spot.cz + math.cos(math.rad(a)) * (d + 2)) then
                fx, fz = x, z
            end
        end
    end
    if fx == nil then
        return
    end
    local memo = {fx = fx, fz = fz, cx = spot.cx, cz = spot.cz, dirX = spot.dirX, dirZ = spot.dirZ, sense = spot.sense}
    if path ~= nil and #path >= 2 and start ~= nil then
        memo.path = {}
        for _, p in ipairs(path) do
            table.insert(memo.path, {x = p.x, y = p.y, z = p.z})
        end
        memo.start = {x = start.x, z = start.z, dx = start.dx, dz = start.dz}
        memo.target = {x = spot.x, z = spot.z}
    end
    table.insert(ADFieldPark.memos, memo)
    ADFieldPark.saveMemos()
end

-- the end of the path follows the field border: its points, a train length (+ ADFieldPark.LEAD_EXTRA) back
-- from the spot, at the same distance from the border as the spot, every ADFieldPark.LEAD_STEP m. A curved border
-- gives a curve, a straight one a straight line. The point where it starts and the points to the spot; the spot
-- itself and nil when no part of it is clear (crop, obstacle, another field, a road)
function ADFieldPark.getLeadIn(vehicle, spot)
    local front, rear = ADTrafficYieldModule.getTrainExtents(vehicle)
    local full = front + rear + ADFieldPark.LEAD_EXTRA
    local halfW = ADTrafficYieldModule.getWidth(vehicle) / 2 + 0.3
    -- the field of the harvester: the border is looked for on it, the trace never on another field nor a road
    local area = nil
    local m = spot.harvester ~= nil and ADFieldPark.harvesterMotion[spot.harvester] or nil
    if m ~= nil and m.area ~= nil then
        area = m.area
    elseif spot.harvester ~= nil and spot.harvester.components ~= nil then
        local hx, _, hz = getWorldTranslation(spot.harvester.components[1].node)
        area = ADFieldPark.getHarvesterArea(hx, hz)
    end
    local savedArea, savedRelaxed = ADFieldPark.area, ADFieldPark.relaxed
    ADFieldPark.area, ADFieldPark.relaxed = area, false
    local py = spot.y or 0
    -- distance of the spot from the border (along the normal pointing out of the field)
    local trace = {{x = spot.x, z = spot.z}}
    -- normal of the train pointing out of the field, and the distance of the spot from the border along it
    local nx, nz = spot.dirZ, -spot.dirX
    local offset = ADFieldPark.borderAcross(spot.x, spot.z, nx, nz, py)
    local back = ADFieldPark.borderAcross(spot.x, spot.z, -nx, -nz, py)
    if offset == nil or (back ~= nil and math.abs(back) < math.abs(offset)) then
        nx, nz, offset = -nx, -nz, back
    end
    if offset ~= nil then
        local x, z, hx, hz = spot.x, spot.z, -spot.dirX, -spot.dirZ
        local travelled = 0
        while travelled < full do
            -- one step back, then across to the border (same side), at the same distance from it
            local cx, cz = x + hx * ADFieldPark.LEAD_STEP, z + hz * ADFieldPark.LEAD_STEP
            local d = ADFieldPark.borderAcross(cx, cz, nx, nz, py)
            if d == nil then
                break
            end
            local px, pz = cx + nx * (d - offset), cz + nz * (d - offset)
            local step = MathUtil.vector2Length(px - x, pz - z)
            if step < 0.5 or ((px - x) * hx + (pz - z) * hz) / step < 0.7 then
                -- the border turns too much here (field corner)
                break
            end
            local ohx, ohz = hx, hz
            hx, hz = (px - x) / step, (pz - z) / step
            -- the normal turns with the border, same side
            local mx, mz = hz, -hx
            if mx * nx + mz * nz < 0 then
                mx, mz = -mx, -mz
            end
            nx, nz = mx, mz
            x, z = px, pz
            travelled = travelled + step
            table.insert(trace, 1, {x = x, z = z})
        end
    end
    ADFieldPark.area, ADFieldPark.relaxed = savedArea, savedRelaxed
    local roads = ADFieldPark.getWayPointsAround(spot.x, spot.z, full + 20)
    local reason = "no border"
    -- shortened from its far end until it is clear
    while #trace >= 2 and (#trace - 1) * ADFieldPark.LEAD_STEP >= ADFieldPark.LEAD_MIN do
        local points = {}
        for _, t in ipairs(trace) do
            table.insert(points, {x = t.x, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, t.x, py, t.z), z = t.z})
        end
        local blocked, ox, oz = ADFieldPark.isPathClear(vehicle, points, halfW, nil, spot.roadside == true)
        for k = 1, #points - 1 do
            local p, n = points[k], points[k + 1]
            local l = math.max(MathUtil.vector2Length(n.x - p.x, n.z - p.z), 0.01)
            local dx, dz = (n.x - p.x) / l, (n.z - p.z) / l
            for _, side in ipairs({0, -1, 1}) do
                local x, z = p.x + dz * side * halfW, p.z - dx * side * halfW
                if blocked == nil and area ~= nil and not area.capped and AutoDrive.checkIsOnField(x, 0, z) and not PathFinderModule.isInFieldArea(area, x, z) then
                    blocked, ox, oz = "other field", x, z
                end
            end
            for _, wp in ipairs(roads) do
                if blocked == nil and MathUtil.vector2Length(wp.x - p.x, wp.z - p.z) < halfW + 1 then
                    blocked, ox, oz = "road", p.x, p.z
                end
            end
        end
        if blocked == nil then
            local l = math.max(MathUtil.vector2Length(points[2].x - points[1].x, points[2].z - points[1].z), 0.01)
            local target = {x = points[1].x, y = points[1].y, z = points[1].z,
                dirX = (points[2].x - points[1].x) / l, dirZ = (points[2].z - points[1].z) / l, roadside = spot.roadside}
            table.remove(points, 1)
            return target, points
        end
        reason = string.format("%s at x=%.0f z=%.0f", blocked, ox, oz)
        table.remove(trace, 1)
    end
    vehicle.ad.trafficYieldModule:log("field park: no lead-in along the border (%s), straight to the spot", reason)
    return spot, nil
end

-- distance from (x, z) to the field border along (nx, nz) (pointing out of the field), negative when (x, z) is off the
-- field; nil when no border within ADFieldPark.LEAD_ACROSS m
function ADFieldPark.borderAcross(x, z, nx, nz, py)
    local step = 0.25
    local wasOn = nil
    for t = -ADFieldPark.LEAD_ACROSS, ADFieldPark.LEAD_ACROSS, step do
        local on = ADFieldPark.isOnField(x + nx * t, py, z + nz * t)
        if wasOn == true and not on then
            return t - step / 2
        end
        wasOn = on
    end
    return nil
end

-- the path to the lead-in point, then the part along the border
function ADFieldPark.appendLeadIn(path, leadIn)
    if leadIn == nil then
        return path
    end
    local result = {}
    for _, p in ipairs(path) do
        table.insert(result, p)
    end
    for _, p in ipairs(leadIn) do
        table.insert(result, {x = p.x, y = p.y, z = p.z})
    end
    return result
end

-- the remembered path, when this unloader starts from the same place toward the same stop
function ADFieldPark.getMemoPath(vehicle, spot)
    local memo = spot.memo
    if memo == nil or memo.path == nil then
        return nil
    end
    local x, _, z = ADTrafficYieldModule.getPosition(vehicle)
    local dx, dz = ADTrafficYieldModule.getDirection(vehicle)
    if MathUtil.vector2Length(x - memo.start.x, z - memo.start.z) > ADFieldPark.MEMO_START_DISTANCE
        or dx * memo.start.dx + dz * memo.start.dz < math.cos(math.rad(ADFieldPark.MEMO_START_ANGLE))
        or MathUtil.vector2Length(spot.x - memo.target.x, spot.z - memo.target.z) > 1 then
        return nil
    end
    local path = {}
    for _, p in ipairs(memo.path) do
        table.insert(path, {x = p.x, y = p.y, z = p.z})
    end
    return path
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
-- origin (optional): {x, z, dx, dz} the search starts from instead of where the vehicle stands (an unloader waiting
-- beside the road looks again from the entrance it came to, without driving there)
function ADFieldPark.newSearch(vehicle, origin)
    ADTrafficYieldModule.measureTrain(vehicle)
    local search = {vehicle = vehicle}
    search.x, search.py, search.z = ADTrafficYieldModule.getPosition(vehicle)
    search.vdx, search.vdz = ADTrafficYieldModule.getDirection(vehicle)
    if origin ~= nil then
        search.x, search.z, search.vdx, search.vdz = origin.x, origin.z, origin.dx, origin.dz
        search.py = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, origin.x, 0, origin.z)
    end
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
    search.harvester = harvester
    search.chopper = harvester.ad ~= nil and harvester.ad.isChopper == true
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
            search.noseX, search.noseZ = fx, fz
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
    -- the place remembered for this field, and the sense of the first headland: unknown, nobody in the field yet
    -- and nothing remembered, wait beside the road (no place chosen at random)
    search.memo = ADFieldPark.findMemo(harvester)
    -- the side is the one the harvester set off to along the border at the entrance: the unloaders turn
    -- the same way and park in the cutting direction. Where it is now only when its sense is not known
    local sense = ADFieldPark.getHarvesterSense(search)
    if sense ~= nil and search.noseX ~= nil then
        local area, relaxed = ADFieldPark.area, ADFieldPark.relaxed
        ADFieldPark.area, ADFieldPark.relaxed = search.area, false
        local bx, _, tx, tz, nx, nz = ADFieldPark.findBorder(search.noseX, search.noseZ, search.py)
        ADFieldPark.area, ADFieldPark.relaxed = area, relaxed
        if bx ~= nil then
            -- the harvester goes along this border with (n x d) of the sign of its sense
            local dx, dz = tx, tz
            if ((nx * dz - nz * dx) > 0 and 1 or -1) ~= sense then
                dx, dz = -dx, -dz
            end
            local across = dx * search.vdz - dz * search.vdx
            if math.abs(across) > 0.3 then
                search.side = across > 0 and 1 or -1
                vehicle.ad.trafficYieldModule:log("field park: side %d, the way the harvester set off along the border at the entrance", search.side)
                -- a place remembered on the other side (before this rule) is forgotten
                local memo = search.memo
                if memo ~= nil and ((memo.cx - search.x) * search.vdz - (memo.cz - search.z) * search.vdx) * search.side < 0 then
                    vehicle.ad.trafficYieldModule:log("field park: remembered place on the other side, forgotten")
                    ADFieldPark.forgetMemo(memo)
                    search.memo = nil
                end
            end
        end
    end
    if sense == nil and search.memo == nil and ADFieldPark.countInField(vehicle, harvester) == 0 then
        search.result = "harvester direction not known yet (first headland not done), waiting beside the road"
        search.waitSense = true
    end
    return search
end

-- one step of the search; returns nil while searching, then the list of spots best first: along the field
-- border, then close to the vehicle, then nose ahead. Each spot is {x, y, z, dirX, dirZ}, where the tractor stops
-- an unloader reached its spot / left it. reserved: on its way there, the spot is already taken for the others
function ADFieldPark.setParked(vehicle, spot, reserved)
    if spot ~= nil then
        spot.reserved = reserved == true
    end
    ADFieldPark.parked[vehicle] = spot
end

-- free cut field ground beside a train footprint (centre cx, cz, direction dx, dz), widest side, in m
function ADFieldPark.getPassWidth(cx, cz, dx, dz, halfW, halfL)
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
            while o <= halfW + ADFieldPark.PASS_SEARCH do
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

-- forage harvester: the footprint stands in its lane ahead, or too close to it
function ADFieldPark.isInHarvesterWay(harvester, cx, cz, halfW, halfL)
    if harvester == nil or harvester.components == nil then
        return false
    end
    local hx, _, hz = getWorldTranslation(harvester.components[1].node)
    local rx, _, rz = AutoDrive.localDirectionToWorld(harvester, 0, 0, 1)
    local dx, dz = cx - hx, cz - hz
    if math.sqrt(dx * dx + dz * dz) - halfL < ADFieldPark.HARVESTER_MIN_DISTANCE then
        return true
    end
    local ahead = dx * rx + dz * rz
    local side = math.abs(dx * rz - dz * rx)
    local width = (harvester.size ~= nil and harvester.size.width or 4) / 2 + halfW + ADFieldPark.LANE_MARGIN
    return ahead > -halfL and ahead < ADFieldPark.LANE_LENGTH + halfL and side < width
end

-- the footprint overlaps the spot of another unloader, parked there or on its way to it
function ADFieldPark.overlapsTakenSpot(search, cx, cz, dx, dz)
    for other, spot in pairs(ADFieldPark.parked) do
        if other ~= search.vehicle and spot ~= nil and spot.cx ~= nil then
            local ox, oz = cx - spot.cx, cz - spot.cz
            local along = math.abs(ox * dx + oz * dz)
            local across = math.abs(ox * dz - oz * dx)
            if along < search.halfL + (spot.halfL or search.halfL) + ADFieldPark.QUEUE_GAP - 0.5 and across < search.halfW * 2 + 0.5 then
                return true
            end
        end
    end
    return false
end

-- spots right behind, then right in front of the unloaders already parked nearby, in line with them, so the
-- unloaders queue along the border. They come before any other candidate
function ADFieldPark.addQueueCandidates(search)
    for other, spot in pairs(ADFieldPark.parked) do
        local valid = other ~= search.vehicle and other.isDeleted ~= true and other.components ~= nil and spot.cx ~= nil
        if valid and spot.roadside then
            -- waiting beside the road, not a place in the field to queue behind
        elseif valid and not spot.reserved then
            -- still standing there
            local ox, _, oz = ADTrafficYieldModule.getPosition(other)
            valid = MathUtil.vector2Length(ox - spot.x, oz - spot.z) < 5
        end
        if not valid then
            ADFieldPark.parked[other] = nil
        elseif MathUtil.vector2Length(spot.cx - search.x, spot.cz - search.z) < ADFieldPark.RADII[#ADFieldPark.RADII] + 50 then
            local offset = spot.halfL + ADFieldPark.QUEUE_GAP + search.halfL
            -- right behind it only (order of arrival along the border)
            for rank, sign in ipairs({-1}) do
                local cx, cz = spot.cx + spot.dirX * sign * offset, spot.cz + spot.dirZ * sign * offset
                -- forage harvester: in line behind the last one, in order of arrival (queue rules, see isSpotAllowed)
                local queued = true
                -- the line behind a parked unloader is on the same harvested strip as its place: the crop
                -- beyond the train does not matter (relaxed, as for the remembered place), and the refusal is logged
                ADFieldPark.reject = "taken, in the harvester's way or too close to the entrance"
                local relaxed = ADFieldPark.relaxed
                ADFieldPark.relaxed = true
                local maxH = ADFieldPark.isSpotAllowed(search, cx, cz, spot.dirX, spot.dirZ, queued)
                    and ADFieldPark.isGroundFree(cx, cz, spot.dirX, spot.dirZ, search.halfW, search.halfL, search.nearby, search.py) or nil
                ADFieldPark.relaxed = relaxed
                if maxH == nil then
                    search.vehicle.ad.trafficYieldModule:log("field park: no place in line behind %s at x=%.0f z=%.0f (%s)", ADTrafficYieldModule.getName(other), cx, cz, tostring(ADFieldPark.reject))
                end
                if maxH ~= nil then
                    local c = {cx = cx, cz = cz, dx = spot.dirX, dz = spot.dirZ, maxH = maxH, cost = -1000 + rank}
                    if sign < 0 then
                        -- behind it: moves up to its place when it leaves
                        c.ahead, c.aheadVehicle = spot, other
                    end
                    table.insert(search.candidates, c)
                end
            end
        end
    end
end

function ADFieldPark.searchStep(search)
    ADFieldPark.relaxed = false
    if search.result ~= nil then
        return {}
    end
    ADFieldPark.area = search.area
    ADFieldPark.relaxed = search.relaxed == true
    ADFieldPark.updateHarvesterMotion(search.harvester)
    if not search.queueDone then
        search.queueDone = true
        ADFieldPark.addQueueCandidates(search)
        ADFieldPark.addMemoCandidate(search)
        if search.memoFree then
            -- the remembered place (or the line behind it): no search along the border
            return ADFieldPark.pickSpots(search)
        end
        return nil
    end
    local radius = (search.wide and ADFieldPark.WIDE_RADII or ADFieldPark.RADII)[search.radiusIndex]
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

-- the place remembered for this field, free and still on clean ground: first choice
function ADFieldPark.addMemoCandidate(search)
    local memo = search.memo
    if memo == nil or search.relaxed then
        return
    end
    for _, c in ipairs(search.candidates) do
        if c.ahead ~= nil and c.ahead.memo == memo then
            -- the remembered place is taken, the line behind it is offered
            search.memoFree = true
        end
    end
    -- queued: no side rule (the unloader may come from another road), never in the harvester's way. The place was
    -- good once: the crop farther than the train itself does not matter (relaxed)
    ADFieldPark.reject = "taken"
    local relaxed = ADFieldPark.relaxed
    ADFieldPark.relaxed = true
    local maxH = ADFieldPark.isSpotAllowed(search, memo.cx, memo.cz, memo.dirX, memo.dirZ, true)
        and ADFieldPark.isGroundFree(memo.cx, memo.cz, memo.dirX, memo.dirZ, search.halfW, search.halfL, search.nearby, search.py) or nil
    ADFieldPark.relaxed = relaxed
    if maxH ~= nil and search.vehicle.ad.trafficYieldModule:overlapHit(memo.cx, maxH + 1.9, memo.cz, math.atan2(memo.dirX, memo.dirZ), search.halfW, 1.3, search.halfL) then
        maxH = nil
    end
    if maxH ~= nil then
        table.insert(search.candidates, {cx = memo.cx, cz = memo.cz, dx = memo.dirX, dz = memo.dirZ, maxH = maxH, cost = -2000, memo = memo})
        search.memoFree = true
        search.vehicle.ad.trafficYieldModule:log("field park: remembered place of this field is free")
    elseif not search.memoFree then
        search.vehicle.ad.trafficYieldModule:log("field park: remembered place of this field not usable (%s), looking along the border", tostring(ADFieldPark.reject))
    end
end

-- on the harvester side of the entrance, and far enough from it
-- queued (forage harvester, in line behind a parked unloader): no side rule, no room-to-pass rule (the line is
-- wanted), QUEUE_ENTRANCE_CLEARANCE from the entrance; never in the harvester's way
function ADFieldPark.isSpotAllowed(search, cx, cz, dx, dz, queued)
    if ADFieldPark.overlapsTakenSpot(search, cx, cz, dx, dz) then
        return false
    end
    if queued then
        if ADFieldPark.isInHarvesterWay(search.harvester, cx, cz, search.halfW, search.halfL) then
            return false
        end
        return ADFieldPark.distanceToFootprint(search.x, search.z, cx, cz, dx, dz, search.halfW, search.halfL) >= ADFieldPark.QUEUE_ENTRANCE_CLEARANCE
    end
    if search.chopper then
        if ADFieldPark.isInHarvesterWay(search.harvester, cx, cz, search.halfW, search.halfL) then
            search.stats = search.stats or {}
            search.stats["harvester way"] = (search.stats["harvester way"] or 0) + 1
            return false
        end
        if not search.narrow and ADFieldPark.getPassWidth(cx, cz, dx, dz, search.halfW, search.halfL) < ADFieldPark.PASS_MIN_WIDTH then
            search.stats = search.stats or {}
            search.stats["no room to pass"] = (search.stats["no room to pass"] or 0) + 1
            return false
        end
    end
    if search.side ~= nil then
        local lateral = (cx - search.x) * search.vdz - (cz - search.z) * search.vdx
        if lateral * search.side <= 0 then
            return false
        end
    end
    if ADFieldPark.isParkedBetween(search, cx, cz) then
        -- a place beyond an unloader parked along the border: the way there goes through it.
        -- The line behind it is the place for this one
        ADFieldPark.reject = "beyond a parked unloader"
        return false
    end
    return ADFieldPark.distanceToFootprint(search.x, search.z, cx, cz, dx, dz, search.halfW, search.halfL) >= (search.wide and ADFieldPark.WIDE_ENTRANCE_CLEARANCE or ADFieldPark.ENTRANCE_CLEARANCE)
end

ADFieldPark.BETWEEN_MARGIN = 2 -- m added to the half length of the parked train across the straight line to the place

-- an unloader parked in the field (not beside the road) stands between the searching unloader and the place (cx, cz):
-- close to the straight line from one to the other, with the parked train's half length as width (it stands along
-- the border, the place farther along the same border)
function ADFieldPark.isParkedBetween(search, cx, cz)
    local vx, vz = cx - search.x, cz - search.z
    local length = MathUtil.vector2Length(vx, vz)
    if length < 1 then
        return false
    end
    vx, vz = vx / length, vz / length
    for other, spot in pairs(ADFieldPark.parked) do
        if other ~= search.vehicle and spot ~= nil and spot.cx ~= nil and not spot.roadside and other.isDeleted ~= true then
            local ox, oz = spot.cx - search.x, spot.cz - search.z
            local along = ox * vx + oz * vz
            local across = math.abs(ox * vz - oz * vx)
            local halfL = spot.halfL or search.halfL
            if along > halfL and along < length - 1 and across < halfL + search.halfW + ADFieldPark.BETWEEN_MARGIN then
                return true
            end
        end
    end
    return false
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
        ADFieldPark.count(search, "no border")
        return
    end
    if search.chopper then
        -- straight along the border over the whole train: heading from the border points at both ends
        local b1x, b1z = ADFieldPark.findBorder(cx - tx * halfL, cz - tz * halfL, py)
        local b2x, b2z = ADFieldPark.findBorder(cx + tx * halfL, cz + tz * halfL, py)
        if b1x ~= nil and b2x ~= nil then
            local cl = MathUtil.vector2Length(b2x - b1x, b2z - b1z)
            if cl > halfL then
                local ntx, ntz = (b2x - b1x) / cl, (b2z - b1z) / cl
                if ntx * tx + ntz * tz < 0 then
                    ntx, ntz = -ntx, -ntz
                end
                -- normal keeps its side (toward the border)
                local nnx, nnz = -ntz, ntx
                if nnx * nx + nnz * nz < 0 then
                    nnx, nnz = -nnx, -nnz
                end
                tx, tz, nx, nz = ntx, ntz, nnx, nnz
            end
        end
    end
    for _, sign in ipairs({1, -1}) do
        local dx, dz = tx * sign, tz * sign
        -- the centre line of the train on the border (half on the grass), a little into the field when
        -- something stands there
        for _, offset in ipairs(ADFieldPark.CENTRE_OFFSETS) do
            local px, pz = bx - nx * offset, bz - nz * offset
            local maxH = nil
            ADFieldPark.reject = "side or entrance"
            if ADFieldPark.isSpotAllowed(search, px, pz, dx, dz) then
                maxH = ADFieldPark.isGroundFree(px, pz, dx, dz, halfW, halfL, search.nearby, py)
                if maxH ~= nil and search.vehicle.ad.trafficYieldModule:overlapHit(px, maxH + 1.9, pz, math.atan2(dx, dz), halfW, 1.3, halfL) then
                    ADFieldPark.reject = "obstacle"
                    maxH = nil
                end
            end
            if maxH == nil then
                ADFieldPark.count(search, ADFieldPark.reject)
            else
                -- nose pointing away from the entrance: the train drives straight in along the border
                local ox, oz = px - search.x, pz - search.z
                local away = (dx * ox + dz * oz) / math.max(MathUtil.vector2Length(ox, oz), 0.1)
                -- the spot farthest from the entrance first, the next one queues behind it
                local cost = 3 * offset - 0.5 * radius - 3 * away
                table.insert(search.candidates, {cx = px, cz = pz, dx = dx, dz = dz, nx = nx, nz = nz, maxH = maxH, cost = cost})
                break
            end
        end
    end
end

-- diagnostic: why the places along the border were refused
function ADFieldPark.count(search, reason)
    search.stats = search.stats or {}
    search.stats[reason] = (search.stats[reason] or 0) + 1
end

-- best candidates first, obstacle test (vehicles, bales, trees...) only on them
function ADFieldPark.pickSpots(search)
    local candidates = search.candidates
    local parts = {}
    for reason, n in pairs(search.stats or {}) do
        table.insert(parts, string.format("%s %d", reason, n))
    end
    search.vehicle.ad.trafficYieldModule:log("field park: %d places, refused: %s (side %s, at x=%.0f z=%.0f)", #candidates,
        table.concat(parts, ", "), tostring(search.side), search.x, search.z)
    search.lastStats = search.stats or {}
    search.stats = {}
    -- the places facing the wrong way do not count: the next passes run before giving up (otherwise one place facing
    -- the wrong way sends the unloader to wait beside the road while the harvested strip is wide)
    local sense = ADFieldPark.getHarvesterSense(search)
    local facing = #candidates
    if sense ~= nil then
        facing = 0
        for _, c in ipairs(candidates) do
            if c.nx == nil or ((c.nx * c.dz - c.nz * c.dx) > 0 and 1 or -1) == sense then
                facing = facing + 1
            end
        end
        if facing == 0 and #candidates > 0 and not search.wide then
            search.vehicle.ad.trafficYieldModule:log("field park: %d places but none faces the harvester's way, next pass", #candidates)
            search.candidates = {}
            candidates = search.candidates
        end
    end
    if #candidates == 0 and not search.relaxed and not search.memoFree then
        -- nothing away from the standing crop (the harvester is still on its headlands): look again without the
        -- harvester clearance, the unloader must not stay on the road
        search.relaxed = true
        search.queueDone = false
        search.radiusIndex = 1
        search.angle = 0
        search.vehicle.ad.trafficYieldModule:log("field park: no spot away from the crop, trying the harvested strip")
        return nil
    end
    if #candidates == 0 and search.relaxed and search.chopper and not search.narrow and not search.memoFree and (search.lastStats["no room to pass"] or 0) > 0 then
        -- the forage harvester has only cut its first strip along the border, too narrow for another trailer to pass
        -- beside a parked one: the unloaders queue in that strip anyway (the called one at the front drives on to the
        -- harvester, the next moves up), rather than places on the far side reached through the crop or a wait at the
        -- entrance
        search.narrow = true
        search.queueDone = false
        search.radiusIndex = 1
        search.angle = 0
        search.vehicle.ad.trafficYieldModule:log("field park: only the narrow first strip is cut, queuing in it without room to pass")
        return nil
    end
    if #candidates == 0 and search.relaxed and not search.wide and not search.memoFree then
        -- still nothing on the harvester's side within 85 m: both sides of the entrance, up to 145 m, 25 m from it
        search.wide = true
        search.wideSide = search.side
        search.side = nil
        search.queueDone = false
        search.radiusIndex = 1
        search.angle = 0
        local maxRadius = ADFieldPark.WIDE_RADII[#ADFieldPark.WIDE_RADII]
        search.nearby = ADFieldPark.getWayPointsAround(search.x, search.z, maxRadius + search.length + 10)
        search.vehicle.ad.trafficYieldModule:log("field park: nothing on side %s, looking on both sides up to %d m, %d m from the entrance", tostring(search.wideSide), maxRadius, ADFieldPark.WIDE_ENTRANCE_CLEARANCE)
        return nil
    end
    ADFieldPark.relaxed = false
    if #candidates == 0 then
        search.result = "no harvested ground along the border"
    end
    -- nose the way the harvester sets off along the border (its first headland)
    if sense ~= nil then
        local kept = {}
        for _, c in ipairs(candidates) do
            if c.nx == nil or ((c.nx * c.dz - c.nz * c.dx) > 0 and 1 or -1) == sense then
                table.insert(kept, c)
            end
        end
        search.vehicle.ad.trafficYieldModule:log("field park: harvester sense %d, %d of %d places face its way", sense, #kept, #candidates)
        if #kept == 0 and search.wide and #candidates > 0 then
            -- last pass: a place facing the other way is better than blocking the entrance
            search.vehicle.ad.trafficYieldModule:log("field park: none faces its way, the others are kept (last pass)")
            kept = candidates
        end
        candidates = kept
    else
        -- only the places in line with an unloader already parked: they face its way
        local kept = {}
        for _, c in ipairs(candidates) do
            if c.nx == nil then
                table.insert(kept, c)
            end
        end
        search.vehicle.ad.trafficYieldModule:log("field park: harvester direction not known yet, %d of %d places in line with a parked unloader", #kept, #candidates)
        candidates = kept
        if #kept == 0 then
            search.waitSense = true
        end
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
                queued = c.cost < -900 and c.memo == nil,
                ahead = c.ahead, aheadVehicle = c.aheadVehicle,
                harvester = search.harvester,
                -- the remembered place, or the sense this place was chosen for (remembered for the next unloaders)
                memo = c.memo or (c.ahead ~= nil and c.ahead.memo or nil), sense = sense
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
                local blocked, bx, bz, index = ADFieldPark.isPathClear(vehicle, dubins.outPath, halfW, fieldRule, spot.roadside == true)
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

-- crop patch of at most ADFieldPark.ISLAND_MAX_AREA, forgotten by the harvester: its connected crop ground is
-- filled on a grid of ADFieldPark.ISLAND_STEP, stopped as soon as it is bigger
function ADFieldPark.isCropIsland(fruitTypeIndex, x, z, cache)
    local step = ADFieldPark.ISLAND_STEP
    local maxCells = ADFieldPark.ISLAND_MAX_AREA / (step * step)
    local i0, j0 = math.floor(x / step + 0.5), math.floor(z / step + 0.5)
    local key0 = i0 .. "_" .. j0
    if cache[key0] ~= nil then
        return cache[key0]
    end
    local seen, queue, head = {[key0] = true}, {{i0, j0}}, 1
    local count = 0
    while head <= #queue do
        local c = queue[head]
        head = head + 1
        count = count + 1
        if count > maxCells then
            break
        end
        for _, d in ipairs({{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) do
            local i, j = c[1] + d[1], c[2] + d[2]
            local key = i .. "_" .. j
            if not seen[key] then
                seen[key] = true
                if ADFieldPark.hasGrowingCrop(i * step, j * step) then
                    table.insert(queue, {i, j})
                end
            end
        end
    end
    local island = count <= maxCells
    -- every crop sample reached belongs to the same patch
    for k = 1, #queue do
        cache[queue[k][1] .. "_" .. queue[k][2]] = island
    end
    return island
end

function ADFieldPark.isPathClear(vehicle, path, halfW, fieldRule, anyGrass)
    local islandCache = {}
    -- the roads around (from beside the road into the field, the path follows the road to the entrance)
    local roads = ADFieldPark.getWayPointsAround(path[1].x, path[1].z, ADFieldPark.ROADSIDE_RANGE + 60)
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
                if crop and not ADFieldPark.isCropIsland(fruitTypeIndex, x, z, islandCache) and not ADFieldPark.isBorderFringe(x, z) then
                    return "crop", x, z, i
                end
                if AutoDrive.checkIsOnField(x, 0, z) then
                    -- to a place beside the road: no field at all, except at the entrance (no U-turn in the field)
                    if anyGrass and MathUtil.vector2Length(x - path[1].x, z - path[1].z) >= ADFieldPark.START_ANY_FIELD then
                        return "field", x, z, i
                    end
                    -- another field is not allowed
                    if fieldRule ~= nil and MathUtil.vector2Length(x - path[1].x, z - path[1].z) >= ADFieldPark.START_ANY_FIELD
                        and not PathFinderModule.isFieldPositionAllowed(fieldRule, x, z) then
                        return "other field", x, z, i
                    end
                elseif not anyGrass and MathUtil.vector2Length(x - path[1].x, z - path[1].z) > ADFieldPark.START_FREE
                    and not ADFieldPark.isNearField(x, z, halfW + 0.1) and not ADFieldPark.isOnRoad(roads, x, z) then
                    -- the grass only along the border, the train at most half on it (trees, ditches beyond),
                    -- except when leaving the road at the entrance
                    return "grass", x, z, i
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


-- on an AutoDrive road (within ADFieldPark.ROAD_HALF_WIDTH + 1 m of its line)
function ADFieldPark.isOnRoad(roads, x, z)
    local limit = ADFieldPark.ROAD_HALF_WIDTH + 1
    for _, wp in ipairs(roads) do
        if math.abs(wp.x - x) < 30 and math.abs(wp.z - z) < 30 then
            for _, id in pairs(wp.out or {}) do
                local nb = ADGraphManager:getWayPointById(id)
                if nb ~= nil then
                    local sx, sz = nb.x - wp.x, nb.z - wp.z
                    local l2 = sx * sx + sz * sz
                    local t = l2 > 0 and math.max(0, math.min(1, ((x - wp.x) * sx + (z - wp.z) * sz) / l2)) or 0
                    if MathUtil.vector2Length(wp.x + sx * t - x, wp.z + sz * t - z) < limit then
                        return true
                    end
                end
            end
        end
    end
    return false
end

-- unloaders parked in the field (or driving to their spot there) for this harvester, the vehicle itself excluded
function ADFieldPark.countInField(vehicle, harvester)
    local count = 0
    for other, spot in pairs(ADFieldPark.parked) do
        -- a full unloader stuck in the field (no clean way out, ExitFieldTask) does not take a place: the next one may
        -- come in so that the harvester goes on and opens the way
        if other ~= vehicle and spot ~= nil and not spot.roadside and other.isDeleted ~= true
            and (spot.harvester == nil or spot.harvester == harvester)
            and not (other.ad ~= nil and other.ad.exitStuckFull ~= nil) then
            count = count + 1
        end
    end
    return count
end

-- first one waiting beside the road for this harvester (order of arrival)
function ADFieldPark.isFirstOnRoadside(vehicle, harvester)
    local mine = vehicle.ad ~= nil and vehicle.ad.unloaderQueueTime or math.huge
    for other, spot in pairs(ADFieldPark.parked) do
        if other ~= vehicle and spot ~= nil and spot.roadside and spot.harvester == harvester and other.isDeleted ~= true then
            local t = other.ad ~= nil and other.ad.unloaderQueueTime or math.huge
            if t < mine then
                return false
            end
        end
    end
    return true
end

-- places beside the AutoDrive roads within ADFieldPark.ROADSIDE_RANGE of the entrance (x, z), parallel to the road,
-- the whole train off every road line, off the fields, on flat free ground, nearest to the entrance first
function ADFieldPark.findRoadsideSpots(vehicle, harvester, x, z)
    ADTrafficYieldModule.measureTrain(vehicle)
    local front, rear = ADTrafficYieldModule.getTrainExtents(vehicle)
    local length = front + rear
    local halfW = ADTrafficYieldModule.getWidth(vehicle) / 2 + 0.5
    local halfL = length / 2 + 0.5
    local range = ADFieldPark.ROADSIDE_RANGE
    local nearby = ADFieldPark.getWayPointsAround(x, z, range + length + 20)
    local probe = {vehicle = vehicle, halfW = halfW, halfL = halfL}
    local candidates = {}
    for _, wp in ipairs(nearby) do
        for _, id in pairs(wp.out or {}) do
            local nb = ADGraphManager:getWayPointById(id)
            local segment = nb ~= nil and MathUtil.vector2Length(nb.x - wp.x, nb.z - wp.z) or 0
            if segment > 0.5 then
                local tx, tz = (nb.x - wp.x) / segment, (nb.z - wp.z) / segment
                for along = 0, segment, ADFieldPark.ROADSIDE_STEP do
                    local rx, rz = wp.x + tx * along, wp.z + tz * along
                    for _, side in ipairs({1, -1}) do
                        local nx, nz = tz * side, -tx * side
                        local cx, cz = rx + nx * (ADFieldPark.ROAD_HALF_WIDTH + halfW), rz + nz * (ADFieldPark.ROAD_HALF_WIDTH + halfW)
                        local distance = MathUtil.vector2Length(cx - x, cz - z)
                        if distance >= ADFieldPark.ROADSIDE_MIN + halfL and distance <= range then
                            -- nose toward the entrance, to drive in when a place in the field is free
                            local dx, dz = tx, tz
                            if dx * (x - cx) + dz * (z - cz) < 0 then
                                dx, dz = -dx, -dz
                            end
                            table.insert(candidates, {cx = cx, cz = cz, dx = dx, dz = dz, distance = distance})
                        end
                    end
                end
            end
        end
    end
    table.sort(candidates, function(c1, c2) return c1.distance < c2.distance end)
    local spots = {}
    for _, c in ipairs(candidates) do
        if ADFieldPark.isRoadsideFree(vehicle, probe, c, nearby, halfW, halfL) then
            local shift = length / 2 - front
            local tx, tz = c.cx + c.dx * shift, c.cz + c.dz * shift
            table.insert(spots, {
                x = tx, y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, tx, 0, tz), z = tz,
                dirX = c.dx, dirZ = c.dz, cx = c.cx, cz = c.cz, halfL = halfL,
                roadside = true, harvester = harvester, entrance = {x = x, z = z}, distance = c.distance
            })
            if #spots >= ADFieldPark.MAX_TRIES then
                break
            end
        end
    end
    return spots
end

function ADFieldPark.isRoadsideFree(vehicle, probe, c, nearby, halfW, halfL)
    if ADFieldPark.overlapsTakenSpot(probe, c.cx, c.cz, c.dx, c.dz) then
        return false
    end
    for _, wp in ipairs(nearby) do
        if ADFieldPark.distanceToFootprint(wp.x, wp.z, c.cx, c.cz, c.dx, c.dz, halfW, halfL) < ADFieldPark.ROAD_HALF_WIDTH then
            return false
        end
    end
    local minH, maxH = math.huge, -math.huge
    for _, p in ipairs(ADFieldPark.getOutline(c.cx, c.cz, c.dx, c.dz, halfW, halfL)) do
        local h = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, p[1], 0, p[2])
        minH, maxH = math.min(minH, h), math.max(maxH, h)
        if AutoDrive.checkIsOnField(p[1], h, p[2]) then
            return false
        end
    end
    if maxH - minH > ADTrafficYieldModule.MAX_SLOPE_DIFF then
        return false
    end
    if vehicle.ad.trafficYieldModule:overlapHit(c.cx, maxH + 1.9, c.cz, math.atan2(c.dx, c.dz), halfW, 1.3, halfL) then
        return false
    end
    c.maxH = maxH
    return true
end

-- field entrance shared by a full unloader driving out and an empty one coming in (nose to nose at the entrance):
-- the one coming in gives way, it stops on the road before the entrance until the full one is out
ADFieldPark.EXIT_YIELD_RADIUS = 45                  -- m around the network point a full unloader drives out to
ADFieldPark.EXIT_YIELD_DONE = 10                    -- m: the full unloader this close to its point is out of the field
ADFieldPark.EXIT_YIELD_RANGE = 250                  -- m: a full unloader further from its point is not waited for
ADFieldPark.EXIT_YIELD_STALE = 3000                 -- ms: exit point not refreshed for this long (task ended, AutoDrive stopped)
ADFieldPark.EXIT_YIELD_TIMEOUT = 120000             -- ms waited at most for one full unloader
ADFieldPark.EXIT_POINT_TAKEN = 12                   -- m: a vehicle standing this close to the exit point blocks it
ADFieldPark.EXIT_ON_POINT_TIMEOUT = 15000           -- ms waited at most by a vehicle standing on the exit point itself (the full one looks for another point meanwhile)

-- full unloader driving out of the field to the network point (x, z), refreshed each frame by ExitFieldTask
function ADFieldPark.setFieldExit(vehicle, x, z)
    if x == nil then
        vehicle.ad.fieldExit = nil
    else
        vehicle.ad.fieldExit = {x = x, z = z, t = g_time or 0}
    end
end

-- another unloader driving out to a network point near this vehicle and not out yet: the vehicle, its distance to
-- the point and this vehicle's distance to it
function ADFieldPark.getExitInWay(vehicle)
    local all = g_currentMission ~= nil and g_currentMission.vehicleSystem ~= nil and g_currentMission.vehicleSystem.vehicles or {}
    local x, _, z = getWorldTranslation(vehicle.components[1].node)
    for _, other in pairs(all) do
        local exit = other ~= vehicle and other.ad ~= nil and other.isDeleted ~= true and other.ad.fieldExit or nil
        if exit ~= nil and (g_time or 0) - exit.t < ADFieldPark.EXIT_YIELD_STALE then
            local mine = MathUtil.vector2Length(x - exit.x, z - exit.z)
            if mine < ADFieldPark.EXIT_YIELD_RADIUS then
                local ox, _, oz = getWorldTranslation(other.components[1].node)
                local left = MathUtil.vector2Length(ox - exit.x, oz - exit.z)
                if left > ADFieldPark.EXIT_YIELD_DONE and left < ADFieldPark.EXIT_YIELD_RANGE then
                    return other, left, mine
                end
            end
        end
    end
    return nil
end

-- true while the vehicle must hold (stopped) for a full unloader coming out; logs once per full unloader
function ADFieldPark.holdForExit(vehicle, dt, where)
    local other, left, mine = ADFieldPark.getExitInWay(vehicle)
    local hold = vehicle.ad.exitYield
    local module = vehicle.ad.trafficYieldModule
    if other == nil then
        if hold ~= nil then
            if module ~= nil then
                module:log("field entrance: %s is out, going on (waited %d s)", tostring(hold.name), math.floor(hold.waited / 1000))
            end
            vehicle.ad.exitYield = nil
        end
        return false
    end
    if hold == nil or hold.other ~= other then
        hold = {other = other, name = other:getName(), waited = 0}
        vehicle.ad.exitYield = hold
        if module ~= nil then
            module:log("field entrance: %s drives out full (%.0f m left), waiting %s %.0f m from the entrance", tostring(hold.name), left, tostring(where), mine)
        end
    end
    hold.waited = hold.waited + dt
    local limit = mine < ADFieldPark.EXIT_POINT_TAKEN and ADFieldPark.EXIT_ON_POINT_TIMEOUT or ADFieldPark.EXIT_YIELD_TIMEOUT
    if hold.waited > limit then
        if not hold.gaveUp then
            hold.gaveUp = true
            if module ~= nil then
                module:log("field entrance: %s still not out after %d s, not waiting any longer", tostring(hold.name), limit / 1000)
            end
        end
        return false
    end
    vehicle.ad.specialDrivingModule:stopVehicle()
    vehicle.ad.specialDrivingModule:update(dt)
    return true
end

-- a vehicle other than this one standing on the network point (x, z): an unloader waiting there for its call
function ADFieldPark.isExitPointTaken(vehicle, x, z)
    local all = g_currentMission ~= nil and g_currentMission.vehicleSystem ~= nil and g_currentMission.vehicleSystem.vehicles or {}
    for _, other in pairs(all) do
        if other ~= vehicle and other.ad ~= nil and other.isDeleted ~= true and other.components ~= nil and other.getRootVehicle ~= nil
            and other:getRootVehicle() == other and (other.lastSpeedReal or 0) < 0.0005 and other.ad.stateModule ~= nil
            and other.ad.stateModule:isActive() then
            local ox, _, oz = getWorldTranslation(other.components[1].node)
            if MathUtil.vector2Length(ox - x, oz - z) < ADFieldPark.EXIT_POINT_TAKEN then
                return other
            end
        end
    end
    return nil
end
