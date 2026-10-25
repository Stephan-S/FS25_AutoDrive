-- forage harvester, strict, no route found: a forward-only route through the cut strips to the point behind the
-- harvester, arriving in its direction. A search over
-- poses (position + 16 headings): straight steps and arcs at the turn radius only, never reversing, every meter checked
-- free of crop over the train half width with its centre on the field. When the U-turn has no room the search finds
-- the loop around through the strips by itself. Runs over several frames (BUDGET ms each).
ADBandTour = {}
ADBandTour.HEADINGS = 16
ADBandTour.STEP = 2 -- m of a straight step
ADBandTour.SAMPLE = 0.5 -- m between map samples of the disc
ADBandTour.CELL = 1 -- m, closed set
ADBandTour.BUDGET = 4 -- ms per frame
ADBandTour.MAX_EXPANSIONS = 150000
ADBandTour.MAX_TIME = 10000 -- ms from the start, the train waits meanwhile
ADBandTour.MAX_COST = 3000 -- m
ADBandTour.GOAL_DISTANCE = 3 -- m around the target
ADBandTour.GOAL_HEADING = math.rad(25) -- around the harvester direction
ADBandTour.START_FREE = 3 -- m from the start not checked (the train stands there)
ADBandTour.TURN_COST = 1.1 -- an arc costs a little more than a straight step
ADBandTour.NO_BAND_PENALTY = 30 -- m added to the estimate where the strips do not reach

local function clock()
    if getTimeSec ~= nil then
        return getTimeSec() * 1000
    end
    return os.clock() * 1000
end

-- map readers (replaced in the tests)
function ADBandTour.cropAt(x, z)
    return PathFinderModule.getMapCropAt(x, z) ~= 0
end

function ADBandTour.onFieldAt(x, z)
    return PathFinderModule.getMapOnFieldAt(x, z) == true
end

-- estimate: real distance through the strips (flood toward the target) when known
function ADBandTour.defaultEstimate(tour, x, z)
    local straight = math.sqrt((x - tour.tx) ^ 2 + (z - tour.tz) ^ 2)
    if tour.flood ~= nil then
        local d = ADBandRoute.getDistance(tour.flood, x, z)
        if d ~= nil then
            return math.max(d, straight)
        end
        return straight + ADBandTour.NO_BAND_PENALTY
    end
    return straight
end

local function heapPush(heap, item)
    heap[#heap + 1] = item
    local i = #heap
    while i > 1 do
        local p = math.floor(i / 2)
        if heap[p].f <= heap[i].f then
            break
        end
        heap[p], heap[i] = heap[i], heap[p]
        i = p
    end
end

local function heapPop(heap)
    local top = heap[1]
    local last = heap[#heap]
    heap[#heap] = nil
    if #heap > 0 then
        heap[1] = last
        local i = 1
        while true do
            local l, r, s = i * 2, i * 2 + 1, i
            if l <= #heap and heap[l].f < heap[s].f then
                s = l
            end
            if r <= #heap and heap[r].f < heap[s].f then
                s = r
            end
            if s == i then
                break
            end
            heap[s], heap[i] = heap[i], heap[s]
            i = s
        end
    end
    return top
end

-- start pose x, z, heading (rad, direction sin/cos), target tx, tz and arrival heading th, turn radius, train half width
function ADBandTour.start(x, z, heading, tx, tz, th, radius, halfWidth, flood)
    local tour = {sx = x, sz = z, tx = tx, tz = tz, th = th, radius = radius, flood = flood,
        samples = {}, free = {}, closed = {}, heap = {}, expansions = 0, done = false, startClock = clock()}
    local r = math.floor(halfWidth * 2 + 0.5) / 2
    tour.offsets = {}
    local n = math.ceil(r / ADBandTour.SAMPLE)
    for j = -n, n do
        for i = -n, n do
            local dx, dz = i * ADBandTour.SAMPLE, j * ADBandTour.SAMPLE
            if dx * dx + dz * dz <= r * r + 0.001 then
                tour.offsets[#tour.offsets + 1] = {i, j}
            end
        end
    end
    -- motions: straight, and arcs turning one heading step at the radius
    local dh = 2 * math.pi / ADBandTour.HEADINGS
    tour.arcLength = radius * dh
    local h0 = math.floor(heading / dh + 0.5) % ADBandTour.HEADINGS
    local node = {x = x, z = z, h = h0, g = 0, parent = nil}
    node.f = ADBandTour.defaultEstimate(tour, x, z)
    heapPush(tour.heap, node)
    return tour
end

local function sampleBlocked(tour, si, sj)
    local key = si * 1000003 + sj
    local v = tour.samples[key]
    if v == nil then
        v = ADBandTour.cropAt(si * ADBandTour.SAMPLE, sj * ADBandTour.SAMPLE)
        tour.samples[key] = v
    end
    return v
end

-- the train fits at x, z: centre on the field, no crop in the disc (read on the 0.5 m grid)
function ADBandTour.isFree(tour, x, z)
    local si, sj = math.floor(x / ADBandTour.SAMPLE + 0.5), math.floor(z / ADBandTour.SAMPLE + 0.5)
    local key = si * 1000003 + sj
    local v = tour.free[key]
    if v ~= nil then
        return v
    end
    v = ADBandTour.onFieldAt(x, z)
    if v then
        for _, o in ipairs(tour.offsets) do
            if sampleBlocked(tour, si + o[1], sj + o[2]) then
                v = false
                break
            end
        end
    end
    tour.free[key] = v
    return v
end

-- end pose and points every meter of one motion (turn -1, 0, 1) from x, z, heading index h
local function motion(tour, x, z, h, turn)
    local dh = 2 * math.pi / ADBandTour.HEADINGS
    local a = h * dh
    local points = {}
    if turn == 0 then
        local steps = ADBandTour.STEP
        for k = 1, steps do
            points[k] = {x = x + math.sin(a) * k, z = z + math.cos(a) * k}
        end
        return points[#points].x, points[#points].z, h, points, ADBandTour.STEP
    end
    -- centre of the turn beside the train, turn > 0: heading angle grows
    local r = tour.radius
    local cx, cz = x + math.cos(a) * r * turn, z - math.sin(a) * r * turn
    local n = math.max(2, math.ceil(tour.arcLength))
    for k = 1, n do
        local ak = a + turn * dh * k / n
        points[k] = {x = cx - math.cos(ak) * r * turn, z = cz + math.sin(ak) * r * turn}
    end
    return points[n].x, points[n].z, (h + turn) % ADBandTour.HEADINGS, points, tour.arcLength * ADBandTour.TURN_COST
end

local function angleDiff(a, b)
    local d = (a - b) % (2 * math.pi)
    if d > math.pi then
        d = d - 2 * math.pi
    end
    return math.abs(d)
end

-- runs the search for budget ms (nil: to the end). Returns true when done; tour.path holds the points if found
function ADBandTour.step(tour, budget)
    if tour.done then
        return true
    end
    local stopAt = budget ~= nil and (clock() + budget) or math.huge
    local dh = 2 * math.pi / ADBandTour.HEADINGS
    local count = 0
    if clock() - tour.startClock > ADBandTour.MAX_TIME then
        tour.timeout = true
        tour.heap = {}
    end
    while #tour.heap > 0 do
        local node = heapPop(tour.heap)
        local key = (math.floor(node.x / ADBandTour.CELL + 0.5) * 100003 + math.floor(node.z / ADBandTour.CELL + 0.5)) * 32 + node.h
        if not tour.closed[key] then
            tour.closed[key] = true
            tour.expansions = tour.expansions + 1
            local goalDistance = math.sqrt((node.x - tour.tx) ^ 2 + (node.z - tour.tz) ^ 2)
            if tour.closest == nil or goalDistance < tour.closest then
                tour.closest = goalDistance
            end
            if goalDistance <= ADBandTour.GOAL_DISTANCE
                and angleDiff(node.h * dh, tour.th) <= ADBandTour.GOAL_HEADING then
                tour.done, tour.found = true, true
                local path = {}
                local n = node
                while n ~= nil do
                    if n.points ~= nil then
                        for k = #n.points, 1, -1 do
                            table.insert(path, 1, n.points[k])
                        end
                    end
                    n = n.parent
                end
                table.insert(path, 1, {x = tour.sx, z = tour.sz})
                tour.path = path
                tour.length = node.g
                break
            end
            if tour.expansions >= ADBandTour.MAX_EXPANSIONS or node.g > ADBandTour.MAX_COST then
                tour.done = true
                break
            end
            for _, turn in ipairs({0, -1, 1}) do
                local ex, ez, eh, points, cost = motion(tour, node.x, node.z, node.h, turn)
                local ok = true
                for _, p in ipairs(points) do
                    if math.sqrt((p.x - tour.sx) ^ 2 + (p.z - tour.sz) ^ 2) > ADBandTour.START_FREE and not ADBandTour.isFree(tour, p.x, p.z) then
                        ok = false
                        break
                    end
                end
                if ok then
                    local child = {x = ex, z = ez, h = eh, g = node.g + cost, parent = node, points = points}
                    child.f = child.g + ADBandTour.defaultEstimate(tour, ex, ez)
                    heapPush(tour.heap, child)
                end
            end
            count = count + 1
            if count % 50 == 0 and clock() >= stopAt then
                return false
            end
        end
    end
    tour.done = true
    if not tour.found then
        tour.goalFree = ADBandTour.isFree(tour, tour.tx, tour.tz)
    end
    tour.heap, tour.closed, tour.samples, tour.free = {}, nil, nil, nil
    tour.seconds = (clock() - tour.startClock) / 1000
    return true
end
