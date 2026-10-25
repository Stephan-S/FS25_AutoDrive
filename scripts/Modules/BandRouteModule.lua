-- route inside the cut strips of a field (forage harvester work, strict checks).
-- A flood from the target over the field map (0.5 m samples): a point counts only when the whole disc of the
-- train half width around it is cut and its centre on the field, so a 2 m wheel lane through the crop is no way.
-- Every point reached gets its real driving distance to the target (around the crop). The search uses it as
-- estimate instead of the straight line, the partial routes and guided departures measure their gain with it.
-- The flood runs over several frames (BUDGET ms each), the floods are kept TTL ms and shared by all vehicles.
ADBandRoute = {}
ADBandRoute.STEP = 1 -- m between flood points
ADBandRoute.SAMPLE = 0.5 -- m between map samples of the disc
ADBandRoute.BUDGET = 4 -- ms per frame
ADBandRoute.MAX_COST = 30000 -- 3000 m (cost in decimeters)
ADBandRoute.TTL = 30000
ADBandRoute.MAX_FLOODS = 4
ADBandRoute.TARGET_SNAP = 20 -- m: a target off the field or near the crop starts from the closest free point
ADBandRoute.LOOKUP_RADIUS = 3 -- m around a position to find a reached point
ADBandRoute.floods = {}

local function clock()
    if getTimeSec ~= nil then
        return getTimeSec() * 1000
    end
    return os.clock() * 1000
end

local function keyOf(ix, iz)
    return (ix + 50000) * 100000 + (iz + 50000)
end

-- map readers (replaced in the tests)
function ADBandRoute.cropAt(x, z)
    return PathFinderModule.getMapCropAt(x, z) ~= 0
end

function ADBandRoute.onFieldAt(x, z)
    return PathFinderModule.getMapOnFieldAt(x, z) == true
end

-- the samples of the disc, as offsets in SAMPLE steps
local function discOffsets(radius)
    local offsets = {}
    local n = math.ceil(radius / ADBandRoute.SAMPLE)
    for j = -n, n do
        for i = -n, n do
            local dx, dz = i * ADBandRoute.SAMPLE, j * ADBandRoute.SAMPLE
            if dx * dx + dz * dz <= radius * radius + 0.001 then
                offsets[#offsets + 1] = {i, j}
            end
        end
    end
    return offsets
end

-- crop at a sample, read once per flood (grass beside the field counts as free, the centre must be on the field)
local function sampleBlocked(flood, si, sj)
    local key = keyOf(si, sj)
    local v = flood.samples[key]
    if v == nil then
        v = ADBandRoute.cropAt(si * ADBandRoute.SAMPLE, sj * ADBandRoute.SAMPLE)
        flood.samples[key] = v
    end
    return v
end

function ADBandRoute.isFreeNode(flood, ix, iz)
    local key = keyOf(ix, iz)
    local v = flood.free[key]
    if v ~= nil then
        return v
    end
    local x, z = ix * ADBandRoute.STEP, iz * ADBandRoute.STEP
    v = ADBandRoute.onFieldAt(x, z)
    if v then
        local per = ADBandRoute.STEP / ADBandRoute.SAMPLE
        local si, sj = ix * per, iz * per
        for _, o in ipairs(flood.offsets) do
            if sampleBlocked(flood, si + o[1], sj + o[2]) then
                v = false
                break
            end
        end
    end
    flood.free[key] = v
    return v
end

-- flood toward (x, z) for a train of halfWidth: an existing one is shared
function ADBandRoute.request(x, z, halfWidth)
    local radius = math.floor(halfWidth * 2 + 0.5) / 2
    local key = string.format("%d_%d_%.1f", math.floor(x / 2 + 0.5), math.floor(z / 2 + 0.5), radius)
    local now = g_time or 0
    local flood = ADBandRoute.floods[key]
    if flood ~= nil and now >= flood.time and now - flood.time < ADBandRoute.TTL then
        return flood
    end
    local count, oldest, oldestKey = 0, nil, nil
    for k, f in pairs(ADBandRoute.floods) do
        count = count + 1
        if oldest == nil or f.time < oldest.time then
            oldest, oldestKey = f, k
        end
    end
    if count >= ADBandRoute.MAX_FLOODS and oldestKey ~= nil then
        ADBandRoute.floods[oldestKey] = nil
    end
    flood = {key = key, x = x, z = z, radius = radius, offsets = discOffsets(radius), time = now, startClock = clock(),
        samples = {}, free = {}, dist = {}, buckets = {}, current = 0, reached = 0, done = false, seeded = false}
    ADBandRoute.floods[key] = flood
    return flood
end

local function push(flood, key, cost)
    local bucket = flood.buckets[cost]
    if bucket == nil then
        bucket = {}
        flood.buckets[cost] = bucket
    end
    bucket[#bucket + 1] = key
end

-- seeds: the free points closest to the target (it may stand on the road or next to the crop)
local function seed(flood)
    local step = ADBandRoute.STEP
    local cx, cz = math.floor(flood.x / step + 0.5), math.floor(flood.z / step + 0.5)
    local best, bestD = nil, math.huge
    local r = math.floor(ADBandRoute.TARGET_SNAP / step)
    for ring = 0, r do
        -- rings from the target outward: the first ring with a free point holds the closest ones (within a ring step)
        for iz = cz - ring, cz + ring do
            for ix = cx - ring, cx + ring do
                if math.max(math.abs(ix - cx), math.abs(iz - cz)) == ring and ADBandRoute.isFreeNode(flood, ix, iz) then
                    local d = math.sqrt((ix * step - flood.x) ^ 2 + (iz * step - flood.z) ^ 2)
                    if d < bestD then
                        best, bestD = {ix, iz}, d
                    end
                end
            end
        end
        if best ~= nil and ring * step > bestD then
            break
        end
    end
    flood.seeded = true
    if best == nil then
        flood.done = true
        flood.failed = true
        return
    end
    local cost = math.floor(bestD * 10 + 0.5)
    local key = keyOf(best[1], best[2])
    flood.dist[key] = cost
    flood.current = cost
    push(flood, key, cost)
end

local NEIGHBOURS = {{1, 0, 10}, {-1, 0, 10}, {0, 1, 10}, {0, -1, 10}, {1, 1, 14}, {1, -1, 14}, {-1, 1, 14}, {-1, -1, 14}}

-- runs the flood for budget ms (nil: to the end). Returns true when done
function ADBandRoute.step(flood, budget)
    if flood.done then
        return true
    end
    if not flood.seeded then
        seed(flood)
        if flood.done then
            return true
        end
    end
    local stopAt = budget ~= nil and (clock() + budget) or math.huge
    local count = 0
    while flood.current <= ADBandRoute.MAX_COST do
        local bucket = flood.buckets[flood.current]
        if bucket ~= nil and #bucket > 0 then
            local key = bucket[#bucket]
            bucket[#bucket] = nil
            if flood.dist[key] == flood.current then
                flood.reached = flood.reached + 1
                local ix, iz = math.floor(key / 100000) - 50000, key % 100000 - 50000
                for _, n in ipairs(NEIGHBOURS) do
                    local nx, nz = ix + n[1], iz + n[2]
                    local nkey = keyOf(nx, nz)
                    local cost = flood.current + n[3]
                    local old = flood.dist[nkey]
                    if (old == nil or cost < old) and ADBandRoute.isFreeNode(flood, nx, nz) then
                        -- diagonal: both sides free too (no corner cut through the crop)
                        if n[3] == 10 or (ADBandRoute.isFreeNode(flood, ix + n[1], iz) and ADBandRoute.isFreeNode(flood, ix, iz + n[2])) then
                            flood.dist[nkey] = cost
                            push(flood, nkey, cost)
                        end
                    end
                end
                count = count + 1
                if count % 50 == 0 and clock() >= stopAt then
                    return false
                end
            end
        else
            flood.buckets[flood.current] = nil
            flood.current = flood.current + 1
            -- nothing left: every bucket ahead is empty
            if next(flood.buckets) == nil then
                break
            end
        end
    end
    flood.done = true
    -- only the distances are kept (memory)
    flood.buckets, flood.samples, flood.free = {}, nil, nil
    flood.seconds = (clock() - flood.startClock) / 1000
    return true
end

-- real driving distance (m) from a position to the target, nil when no reached point is close
function ADBandRoute.getDistance(flood, x, z, radius)
    if flood == nil or not flood.done or flood.failed then
        return nil
    end
    local step = ADBandRoute.STEP
    local cx, cz = math.floor(x / step + 0.5), math.floor(z / step + 0.5)
    local r = math.floor((radius or ADBandRoute.LOOKUP_RADIUS) / step + 0.5)
    local best = nil
    for iz = cz - r, cz + r do
        for ix = cx - r, cx + r do
            local d = flood.dist[keyOf(ix, iz)]
            if d ~= nil then
                local total = d / 10 + math.sqrt((ix * step - x) ^ 2 + (iz * step - z) ^ 2)
                if best == nil or total < best then
                    best = total
                end
            end
        end
    end
    return best
end

-- the way itself, from a position back to the target (the seed),
-- following the distances downhill. Returns points {x, z} STEP m apart (the first near the position, the last at the
-- target), nil when the position is not reached by the flood
function ADBandRoute.tracePath(flood, x, z, radius)
    if flood == nil or not flood.done or flood.failed then
        return nil
    end
    local step = ADBandRoute.STEP
    local cx, cz = math.floor(x / step + 0.5), math.floor(z / step + 0.5)
    local r = math.floor((radius or ADBandRoute.LOOKUP_RADIUS) / step + 0.5)
    local best, bix, biz = nil, nil, nil
    for iz = cz - r, cz + r do
        for ix = cx - r, cx + r do
            local d = flood.dist[keyOf(ix, iz)]
            if d ~= nil then
                local total = d / 10 + math.sqrt((ix * step - x) ^ 2 + (iz * step - z) ^ 2)
                if best == nil or total < best then
                    best, bix, biz = total, ix, iz
                end
            end
        end
    end
    if best == nil then
        return nil
    end
    local path = {}
    local ix, iz = bix, biz
    for _ = 1, 20000 do
        path[#path + 1] = {x = ix * step, z = iz * step}
        local nix, niz, nd = nil, nil, flood.dist[keyOf(ix, iz)]
        for _, n in ipairs(NEIGHBOURS) do
            local jx, jz = ix + n[1], iz + n[2]
            local v = flood.dist[keyOf(jx, jz)]
            -- diagonal only when both sides are reached too (no corner cut through the crop, as the flood itself)
            if v ~= nil and v < nd and (n[3] == 10 or (flood.dist[keyOf(jx, iz)] ~= nil and flood.dist[keyOf(ix, jz)] ~= nil)) then
                nd, nix, niz = v, jx, jz
            end
        end
        if nix == nil then
            break
        end
        ix, iz = nix, niz
    end
    return path
end
