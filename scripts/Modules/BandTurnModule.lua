-- U-turn of a tractor and trailer in a narrow strip (forage harvester unloaders only).
-- The manoeuvre (forward and reverse legs, the trailer may be backed into the standing crop, idea) is built
-- on a local map of the strip read once (field ground, crop, obstacles such as trees and vehicles), with the real
-- train geometry (tractor axle, hitch, trailer axle) and the tractor turning radius. Used when the unloader faces the
-- wrong way: arriving head-on at the harvester, or leaving full with the field exit behind.
ADBandTurn = {}

ADBandTurn.MAP_RADIUS = 32            -- m around the tractor read into the local map
ADBandTurn.END_ROW_MAP_RADIUS = 40    -- m, end-of-row U-turn (reverse then U-turn)
ADBandTurn.MAP_STEP = 0.5             -- m, local map resolution (crop, field)
ADBandTurn.OBSTACLE_STEP = 1          -- m between two obstacle probes
ADBandTurn.MAP_READS_PER_FRAME = 1500
ADBandTurn.OBSTACLE_PROBES_PER_FRAME = 250
ADBandTurn.SUB_STEP = 0.25            -- m, kinematic integration step
ADBandTurn.GOAL_HEADING = 20          -- deg, final heading tolerance
ADBandTurn.GOAL_ARTICULATION = 15     -- deg, trailer nearly straight at the end
ADBandTurn.GOAL_TRAILER_CROP = 4      -- crop samples still tolerated under the trailer at the end
ADBandTurn.MAX_ARTICULATION = 65      -- deg, never jackknife
ADBandTurn.FIELD_TOLERANCE = 1.0      -- m of the body allowed over the allowed ground
ADBandTurn.GRASS_MAX = 4              -- m of grass beyond the field border allowed when nothing stands there ;
                                      -- limited: a stream is not seen as an obstacle
ADBandTurn.REVERSE_COST = 1.5
ADBandTurn.SWITCH_COST = 6            -- m added per change of direction
ADBandTurn.TRACTOR_CROP_COST = 2      -- per crop sample under the tractor
ADBandTurn.TRAILER_CROP_COST = 0.3    -- per crop sample under the trailer (allowed)
ADBandTurn.TRACTOR_FRONT_FROM = 1.0   -- m ahead of the rear axle: from there the tractor (front wheels) never enters the crop
ADBandTurn.RADIUS_MARGIN = 1.1        -- planned turning radius = tractor radius * margin
ADBandTurn.RETRY_DISTANCE = 15        -- m: no new try near a failed one ...
ADBandTurn.RETRY_TIME = 120000        -- ms ... for this long
-- driving the manoeuvre
ADBandTurn.SPEED = 5                  -- km/h
ADBandTurn.LOOK_AHEAD = 2             -- m, forward legs (tractor axle): short, less corner cutting
ADBandTurn.LOOK_BEHIND = 4            -- m, reverse legs (trailer axle)
ADBandTurn.LEG_PAUSE = 800            -- ms standing still between two legs
ADBandTurn.FRONT_CROP_TOLERANCE = 2  -- front samples over crop accepted while driving (map cells of 0.5 m)
ADBandTurn.MAX_DEVIATION = 2.5        -- m off the planned line: stop
ADBandTurn.STUCK_TIME = 6000          -- ms without moving: stop
ADBandTurn.MAX_REPLANS = 3            -- new plans from the current pose when off the line
ADBandTurn.STUCK_MOVE = 0.3           -- m: less than this counts as not moving
ADBandTurn.TIMEOUT = 150000           -- ms for the whole manoeuvre
-- joining the harvester after the turn (head-on arrival)
ADBandTurn.JOIN_WAIT_TIME = 60000     -- ms waiting for the harvester to come alongside
ADBandTurn.JOIN_STOPPED_TIME = 5000   -- ms harvester standing still: back up to its pipe
ADBandTurn.JOIN_MAX_REVERSE = 40      -- m
ADBandTurn.JOIN_REVERSE = false       -- back up to the pipe of a standing harvester (off)
ADBandTurn.HOLD_HARVESTER_DISTANCE = 35 -- m: Courseplay harvester held while the train turns

local atan2 = math.atan2 or math.atan
local PI, TWO_PI = math.pi, 2 * math.pi

local function wrap(a)
    a = (a + PI) % TWO_PI
    if a < 0 then
        a = a + TWO_PI
    end
    return a - PI
end

ADBandTurn.wrap = wrap

-- ---------------------------------------------------------------------------------------------------------------
-- search (pure: map and geometry given, no game call, tested outside the game)
-- ---------------------------------------------------------------------------------------------------------------

-- map: {ox, oz, step, n, cells} cells[iz * n + ix + 1] bit flags: 1 off field, 2 crop, 4 obstacle; outside = all
ADBandTurn.OFF_FIELD, ADBandTurn.CROP, ADBandTurn.OBSTACLE = 1, 2, 4

function ADBandTurn.mapFlags(map, x, z)
    local ix = math.floor((x - map.ox) / map.step)
    local iz = math.floor((z - map.oz) / map.step)
    if ix < 0 or iz < 0 or ix >= map.n or iz >= map.n then
        return 7
    end
    return map.cells[iz * map.n + ix + 1] or 7
end

local function hasFlag(flags, flag)
    return flags % (flag * 2) >= flag
end

-- one body (box along heading a around the reference point x, z): ok, crop samples
local function checkBody(map, x, z, a, box)
    local dx, dz = math.sin(a), math.cos(a)
    local nx, nz = dz, -dx
    local crop, frontCrop = 0, 0
    local half = box.half
    local inner = math.max(0, half - ADBandTurn.FIELD_TOLERANCE)
    local nl = math.max(1, math.ceil(box.front - box.rear))
    for i = 0, nl do
        local l = box.rear + (box.front - box.rear) * i / nl
        for k = 1, 4 do
            local w = box.widths[k]
            local px, pz = x + dx * l + nx * w, z + dz * l + nz * w
            local flags = ADBandTurn.mapFlags(map, px, pz)
            if hasFlag(flags, ADBandTurn.OBSTACLE) then
                return false, 0, "obstacle"
            end
            if hasFlag(flags, ADBandTurn.CROP) then
                crop = crop + 1
                if box.frontFrom ~= nil and l > box.frontFrom then
                    frontCrop = frontCrop + 1
                end
            end
            -- field: the body may reach FIELD_TOLERANCE over the border
            local ws = (w >= 0 and 1 or -1) * math.min(math.abs(w), inner)
            if hasFlag(ADBandTurn.mapFlags(map, x + dx * l + nx * ws, z + dz * l + nz * ws), ADBandTurn.OFF_FIELD) then
                return false, 0, "field"
            end
        end
    end
    return true, crop, nil, frontCrop
end

-- grass next to the field becomes allowed ground up to GRASS_MAX from the field (obstacles are probed separately)
function ADBandTurn.allowGrass(map)
    local n = map.n
    -- the field-only map is kept: the manoeuvre is built on both, the cheaper one wins
    local strict = {ox = map.ox, oz = map.oz, step = map.step, n = n, cells = {}}
    for k = 1, n * n do
        strict.cells[k] = map.cells[k]
    end
    map.strict = strict
    local depth = math.floor(ADBandTurn.GRASS_MAX / map.step + 0.5)
    local dist = {}
    local queue, head = {}, 1
    for k = 1, n * n do
        if not hasFlag(map.cells[k], ADBandTurn.OFF_FIELD) then
            dist[k] = 0
            queue[#queue + 1] = k
        end
    end
    while head <= #queue do
        local k = queue[head]
        head = head + 1
        local d = dist[k]
        if d < depth then
            local ix, iz = (k - 1) % n, math.floor((k - 1) / n)
            for _, o in ipairs({{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}) do
                local jx, jz = ix + o[1], iz + o[2]
                if jx >= 0 and jz >= 0 and jx < n and jz < n then
                    local j = jz * n + jx + 1
                    if dist[j] == nil then
                        dist[j] = d + 1
                        queue[#queue + 1] = j
                        map.cells[j] = map.cells[j] - ADBandTurn.OFF_FIELD
                    end
                end
            end
        end
    end
end

function ADBandTurn.prepareBox(box)
    box.widths = {-box.half, -box.half / 3, box.half / 3, box.half}
    return box
end

-- pose: tractor axle x, z, heading a; trailer axle tx, tz, heading b
function ADBandTurn.checkPose(map, geo, pose)
    local ok, tractorCrop, reason, frontCrop = checkBody(map, pose.x, pose.z, pose.a, geo.tractorBox)
    if not ok then
        return false, reason
    end
    pose.tractorFrontCrop = frontCrop
    local ok2, trailerCrop, reason2 = checkBody(map, pose.tx, pose.tz, pose.b, geo.trailerBox)
    if not ok2 then
        return false, reason2
    end
    return true, nil, tractorCrop, trailerCrop
end

-- drives ds (signed) with curvature k from a pose: exact hitch geometry, the trailer axle follows the hitch
function ADBandTurn.move(geo, pose, ds, k, out)
    local x, z, a = pose.x, pose.z, pose.a
    local tx, tz = pose.tx, pose.tz
    local steps = math.max(1, math.ceil(math.abs(ds) / ADBandTurn.SUB_STEP))
    local s = ds / steps
    for _ = 1, steps do
        if math.abs(k) < 1e-6 then
            x, z = x + math.sin(a) * s, z + math.cos(a) * s
        else
            local na = a + k * s
            -- exact arc
            x = x + (math.cos(a) - math.cos(na)) / k
            z = z + (math.sin(na) - math.sin(a)) / k
            a = na
        end
        local hx, hz = x - math.sin(a) * geo.hitch, z - math.cos(a) * geo.hitch
        local vx, vz = hx - tx, hz - tz
        local l = math.sqrt(vx * vx + vz * vz)
        if l > 1e-6 then
            tx, tz = hx - vx / l * geo.trailer, hz - vz / l * geo.trailer
        end
        if out ~= nil then
            out[#out + 1] = {x = x, z = z, tx = tx, tz = tz}
        end
    end
    local hx, hz = x - math.sin(a) * geo.hitch, z - math.cos(a) * geo.hitch
    local b = atan2(hx - tx, hz - tz)
    return {x = x, z = z, a = wrap(a), tx = tx, tz = tz, b = b}
end

-- trailer axle from the tractor pose and trailer heading
function ADBandTurn.trailerFrom(geo, x, z, a, b)
    local hx, hz = x - math.sin(a) * geo.hitch, z - math.cos(a) * geo.hitch
    return hx - math.sin(b) * geo.trailer, hz - math.cos(b) * geo.trailer
end

-- The manoeuvre, built step by step like a driver does it (the trailer goes back into the crop):
-- 1. forward: if needed first sideways to the crop edge, then an S: away from the crop, then the nose toward the
-- crop (the front wheels never enter it)
-- 2. reverse: the trailer swings back into the crop until it stands across the strip, tail in the crop
-- 3. forward at full lock toward the new heading; if the strip is too short: reverse a little, forward again
-- 4. forward until the trailer is straight behind and out of the crop
-- Every 0.5 m the whole train is checked on the local map (field, crop, obstacles). Variants of step 1 (lengths of
-- the S) are tried, the cheapest complete one is kept.
ADBandTurn.STEP = 0.5                   -- m per simulated step
ADBandTurn.SHIFT = {0, 3, 5, 7}         -- m of each arc of a first S moving the train sideways to the crop edge
ADBandTurn.BACK_OUT_EXTRA = {0, 3, 6}     -- m reversed beyond the point where the front wheels leave the crop (nose in it)
ADBandTurn.S_OUT = {0, 2, 4, 6, 8, 10}  -- m turning away from the crop
ADBandTurn.S_IN = {0, 2, 4, 6, 8, 10, 12} -- m turning toward the crop
ADBandTurn.REVERSE_MAX = 25             -- m per reverse leg
ADBandTurn.FORWARD_MAX = 30             -- m per forward leg
ADBandTurn.REVERSE_ARTICULATION = 55    -- deg kept at most while reversing
ADBandTurn.MAX_CYCLES = 3               -- reverse / forward cycles after the first: 3, 5 or 7 legs
ADBandTurn.EDGE_MARGIN = 0.5              -- m kept free beyond the end of each reverse leg (crop for the tractor, edge, obstacle)
ADBandTurn.STALL_LENGTH = 3             -- m reversed without gaining anything on the turn: the leg ends
ADBandTurn.STRAIGHTEN_LENGTH = 25       -- m driven at most to straighten the trailer at the end
ADBandTurn.VARIANTS_PER_FRAME = 6
ADBandTurn.CROP_SCAN = 12               -- m beside the train to find the crop side
-- end of row (drawing): 1) one forward U-turn (blue), 2) straight back first, then the forward
-- U-turn (red), 3) only then the 3-point manoeuvres (green). Never in the crop, never out of the field
ADBandTurn.UTURN_LEAD = {0, 3, 6}                  -- m straight on before the forward U-turn
ADBandTurn.UTURN_BACK = {4, 8, 12, 16, 20, 25}     -- m straight back before it (red)
ADBandTurn.UTURN_SWING = {0, 3, 5, 8, 11}         -- m first turned the other way (left to go right, wider circle)
ADBandTurn.UTURN_OTHER_SIDE_COST = 15              -- m added when turning away from the preferred side (harvester side)
ADBandTurn.UTURN_BONUS = 20                        -- m taken off U-turn variants: preferred over the 3-point ones
-- Y manoeuvre (his drawing: red forward on a diagonal, blue long straight reverse along it, green
-- forward turn into the row)
ADBandTurn.Y_SWING = {0, 4, 8, 12}                 -- m of arc turned the other way first (the diagonal)
ADBandTurn.Y_LEAD = {0, 6, 12}                     -- m straight on along the diagonal
ADBandTurn.Y_BACK = {15, 25, 35, 50}               -- m straight back along the diagonal
ADBandTurn.Y_MAP_RADIUS = 50                       -- m: the long reverse needs the room on the local map
ADBandTurn.Y_GAIN = 2                              -- articulation wanted per rad of trailer heading error while backing
ADBandTurn.Y_MAX_ARTICULATION = math.rad(30)       -- rad, at most

local function try(search, pose, ds, k)
    local samples = {}
    local p = ADBandTurn.move(search.geo, pose, ds, k, samples)
    if math.abs(wrap(p.a - p.b)) > math.rad(ADBandTurn.MAX_ARTICULATION) then
        return nil
    end
    local ok, _, tractorCrop, trailerCrop = ADBandTurn.checkPose(search.map, search.geo, p)
    -- the front wheels never go into the crop, the rear wheels may when the trailer is backed into it. Started with
    -- the nose in the crop (blocked): only reversing out of it, never deeper
    if not ok or (p.tractorFrontCrop > 0 and (ds > 0 or p.tractorFrontCrop > (pose.tractorFrontCrop or 0))) then
        return nil
    end
    p.tractorCrop, p.trailerCrop = tractorCrop, trailerCrop
    return p, samples
end

-- angle turned in the turning sense since the start (rad, -pi/2 .. 3/2 pi)
local function turned(search, a)
    return wrap((a - search.start.a) * search.sign + PI / 2) - PI / 2
end

local function newLeg(dir, pose)
    return {dir = dir, points = {}, length = 0, endHeading = pose.a}
end

local function addStep(run, leg, samples, p)
    for _, s in ipairs(samples) do
        leg.points[#leg.points + 1] = s
    end
    leg.length = leg.length + ADBandTurn.STEP
    leg.endHeading = p.a
    run.crop = run.crop + p.trailerCrop
    run.tractorCrop = (run.tractorCrop or 0) + p.tractorCrop
    run.pose = p
end

-- forward with a fixed curvature for a length; false if blocked before
local function forwardArc(search, run, leg, k, length)
    local driven = 0
    while driven < length - 1e-6 do
        local p, samples = try(search, run.pose, ADBandTurn.STEP, k)
        if p == nil then
            return false
        end
        addStep(run, leg, samples, p)
        driven = driven + ADBandTurn.STEP
    end
    return true
end

-- reverse as far as the room allows (each leg to the edge, EDGE_MARGIN kept): each step the steering
-- that swings the trailer most into the turning sense (tail into the crop), articulation limited. Ends at the edge,
-- or when it gains nothing on the turn for STALL_LENGTH. untilAcross: first reverse (trailer across the strip)
local function reverseLeg(search, run, maxLength, untilAcross)
    local leg = newLeg(-1, run.pose)
    local K = 1 / search.geo.radius
    local stall = 0
    while leg.length < maxLength do
        local best, bestSamples, bestScore = nil, nil, -math.huge
        for _, k in ipairs({-K, -K / 2, 0, K / 2, K}) do
            local p, samples = try(search, run.pose, -ADBandTurn.STEP, k)
            if p ~= nil and try(search, p, -ADBandTurn.EDGE_MARGIN, k) == nil then
                p = nil
            end
            if p ~= nil then
                local art = math.abs(wrap(p.a - p.b))
                local prevArt = math.abs(wrap(run.pose.a - run.pose.b))
                if art <= math.rad(ADBandTurn.REVERSE_ARTICULATION) or art < prevArt then
                    local score = turned(search, p.b) - turned(search, run.pose.b)
                    if not untilAcross then
                        -- second reverse legs: gain room, keep the heading
                        score = -math.abs(turned(search, p.a) - turned(search, run.pose.a)) * 0.1 + score
                    end
                    score = score - p.trailerCrop * 0.0005 - p.tractorCrop * 0.003
                    if score > bestScore then
                        best, bestSamples, bestScore = p, samples, score
                    end
                end
            end
        end
        if best == nil then
            break
        end
        stall = bestScore <= 0 and stall + ADBandTurn.STEP or 0
        addStep(run, leg, bestSamples, best)
        if math.abs(wrap(run.pose.a - search.goal)) <= math.rad(ADBandTurn.GOAL_HEADING / 2) then
            break
        end
        if stall >= ADBandTurn.STALL_LENGTH then
            break
        end
    end
    return leg
end

-- forward at full lock in the turning sense (half lock when full lock is blocked) until the goal heading or blocked
local function forwardTurnLeg(search, run)
    local leg = newLeg(1, run.pose)
    local K = search.sign / search.geo.radius
    while leg.length < ADBandTurn.FORWARD_MAX do
        if math.abs(wrap(run.pose.a - search.goal)) <= math.rad(ADBandTurn.GOAL_HEADING / 2) then
            break
        end
        local p, samples = try(search, run.pose, ADBandTurn.STEP, K)
        if p == nil then
            p, samples = try(search, run.pose, ADBandTurn.STEP, K / 2)
        end
        if p == nil then
            break
        end
        addStep(run, leg, samples, p)
    end
    return leg
end

-- nose in the crop: reverse with curvature k until the front wheels are out, then extra meters. false if blocked
local function backOut(search, run, leg, k, extra)
    local after = 0
    while after < extra - 1e-6 or (run.pose.tractorFrontCrop or 0) > 0 do
        if leg.length >= ADBandTurn.REVERSE_MAX then
            return false
        end
        local p, samples = try(search, run.pose, -ADBandTurn.STEP, k)
        if p == nil then
            return false
        end
        addStep(run, leg, samples, p)
        if p.tractorFrontCrop == 0 then
            after = after + ADBandTurn.STEP
        end
    end
    return true
end

-- forward U-turn at full lock to the goal heading, after a straight lead (> 0) or a straight reverse (< 0) along
-- the own axis. No crop at all under the train. Returns {legs, cost} or nil, reason
function ADBandTurn.buildUTurn(search, straight, swing)
    local K = 1 / search.geo.radius
    local run = {pose = search.start, crop = 0}
    run.pose.trailerCrop = 0
    local legs = {}
    if straight < 0 then
        local leg = newLeg(-1, run.pose)
        -- straight back: each step the steering that keeps the trailer in line and on its heading
        local b0 = run.pose.b
        while leg.length < -straight - 1e-6 do
            local best, bestSamples, bestScore = nil, nil, math.huge
            for _, k in ipairs({-K, -K / 2, -K / 4, 0, K / 4, K / 2, K}) do
                local p, samples = try(search, run.pose, -ADBandTurn.STEP, k)
                if p ~= nil then
                    local score = math.abs(wrap(p.a - p.b)) + math.abs(wrap(p.b - b0))
                    if score < bestScore then
                        best, bestSamples, bestScore = p, samples, score
                    end
                end
            end
            if best == nil then
                return nil, "reverse blocked"
            end
            addStep(run, leg, bestSamples, best)
        end
        legs[#legs + 1] = leg
    end
    local leg = newLeg(1, run.pose)
    if straight > 0 and not forwardArc(search, run, leg, 0, straight) then
        return nil, "straight forward blocked"
    end
    -- swing out the other way first, at full lock: the circle of the U-turn gets wider
    if (swing or 0) > 0 and not forwardArc(search, run, leg, -search.sign * K, swing) then
        return nil, "counter-steer blocked"
    end
    while math.abs(wrap(run.pose.a - search.goal)) > math.rad(ADBandTurn.GOAL_HEADING / 2) do
        if leg.length >= ADBandTurn.FORWARD_MAX * 2 then
            return nil, "U-turn too long"
        end
        local p, samples = try(search, run.pose, ADBandTurn.STEP, search.sign * K)
        if p == nil then
            return nil, string.format("U-turn blocked at %.0f deg", math.deg(turned(search, run.pose.a)))
        end
        addStep(run, leg, samples, p)
    end
    -- straighten the trailer behind
    local driven = 0
    while math.abs(wrap(run.pose.a - run.pose.b)) > math.rad(ADBandTurn.GOAL_ARTICULATION) do
        if driven >= ADBandTurn.STRAIGHTEN_LENGTH then
            return nil, "trailer not straight"
        end
        local err = wrap(search.goal - run.pose.a)
        local p, samples = try(search, run.pose, ADBandTurn.STEP, math.max(-K, math.min(K, err / 4)))
        if p == nil then
            return nil, "straightening blocked"
        end
        addStep(run, leg, samples, p)
        driven = driven + ADBandTurn.STEP
    end
    legs[#legs + 1] = leg
    -- crop under the train: AutoDrive setting avoidFruit: strict refused, on costed, off ignored
    local cropMode = search.rules ~= nil and search.rules.crop or "strict"
    if cropMode == "strict" and (run.crop > 0 or (run.tractorCrop or 0) > 0) then
        return nil, "crop under the train"
    end
    local cost = -ADBandTurn.UTURN_BONUS
    if cropMode ~= "off" then
        cost = cost + run.crop * ADBandTurn.TRAILER_CROP_COST + (run.tractorCrop or 0) * ADBandTurn.TRACTOR_CROP_COST
    end
    for _, l in ipairs(legs) do
        cost = cost + l.length * (l.dir < 0 and ADBandTurn.REVERSE_COST or 1) + ADBandTurn.SWITCH_COST
    end
    if search.preferSign ~= nil and search.sign ~= search.preferSign then
        cost = cost + ADBandTurn.UTURN_OTHER_SIDE_COST
    end
    return {legs = legs, cost = cost, crop = 0, uturn = true}
end

-- Y manoeuvre: forward at full lock the other way over swing m then straight over lead m (red, a
-- diagonal), straight back along it over back m with the trailer kept in line (blue), forward at full lock to the goal
-- heading and straighten (green). No crop at all under the train (strict). Returns {legs, cost} or nil, reason
function ADBandTurn.buildYTurn(search, lead, swing, back)
    local K = 1 / search.geo.radius
    local run = {pose = search.start, crop = 0}
    run.pose.trailerCrop = 0
    local legs = {}
    -- red
    local leg = newLeg(1, run.pose)
    if (swing or 0) > 0 and not forwardArc(search, run, leg, -search.sign * K, swing) then
        return nil, "Y: diagonal forward blocked"
    end
    if (lead or 0) > 0 and not forwardArc(search, run, leg, 0, lead) then
        return nil, "Y: straight forward blocked"
    end
    if leg.length > 0 then
        legs[#legs + 1] = leg
    end
    -- blue: straight back along the diagonal (the tractor heading after the red leg). After the arc the trailer still
    -- points the old way and reversing swings it further out: like a driver, the tractor is counter-steered past the
    -- trailer heading (articulation against the error) until the trailer lies on the diagonal, then kept in line
    leg = newLeg(-1, run.pose)
    local a0 = run.pose.a
    while leg.length < back - 1e-6 do
        local pose = run.pose
        local e = wrap(a0 - pose.b)
        local wanted = math.max(-ADBandTurn.Y_MAX_ARTICULATION, math.min(ADBandTurn.Y_MAX_ARTICULATION, -ADBandTurn.Y_GAIN * e))
        local aWanted = pose.b + wanted
        local best, bestSamples, bestErr = nil, nil, math.huge
        for _, k in ipairs({-K, -K / 2, -K / 4, 0, K / 4, K / 2, K}) do
            local p, samples = try(search, pose, -ADBandTurn.STEP, k)
            if p ~= nil then
                local err = math.abs(wrap(p.a - aWanted))
                if err < bestErr then
                    best, bestSamples, bestErr = p, samples, err
                end
            end
        end
        if best == nil then
            return nil, string.format("Y: reverse blocked at %.0f m", leg.length)
        end
        addStep(run, leg, bestSamples, best)
    end
    legs[#legs + 1] = leg
    -- green: full lock to the goal heading, then straighten the trailer
    leg = newLeg(1, run.pose)
    while math.abs(wrap(run.pose.a - search.goal)) > math.rad(ADBandTurn.GOAL_HEADING / 2) do
        if leg.length >= ADBandTurn.FORWARD_MAX * 2 then
            return nil, "Y: U-turn too long"
        end
        local p, samples = try(search, run.pose, ADBandTurn.STEP, search.sign * K)
        if p == nil then
            return nil, string.format("Y: U-turn blocked at %.0f deg", math.deg(turned(search, run.pose.a)))
        end
        addStep(run, leg, samples, p)
    end
    local driven = 0
    while math.abs(wrap(run.pose.a - run.pose.b)) > math.rad(ADBandTurn.GOAL_ARTICULATION) do
        if driven >= ADBandTurn.STRAIGHTEN_LENGTH then
            return nil, "Y: trailer not straight"
        end
        local err = wrap(search.goal - run.pose.a)
        local p, samples = try(search, run.pose, ADBandTurn.STEP, math.max(-K, math.min(K, err / 4)))
        if p == nil then
            return nil, "Y: straightening blocked"
        end
        addStep(run, leg, samples, p)
        driven = driven + ADBandTurn.STEP
    end
    legs[#legs + 1] = leg
    local cropMode = search.rules ~= nil and search.rules.crop or "strict"
    if cropMode == "strict" and (run.crop > 0 or (run.tractorCrop or 0) > 0) then
        return nil, "Y: crop under the train"
    end
    -- half the U-turn bonus: a plain U-turn stays cheaper, the Y beats the 3-point manoeuvres
    local cost = -ADBandTurn.UTURN_BONUS / 2
    if cropMode ~= "off" then
        cost = cost + run.crop * ADBandTurn.TRAILER_CROP_COST + (run.tractorCrop or 0) * ADBandTurn.TRACTOR_CROP_COST
    end
    for _, l in ipairs(legs) do
        cost = cost + l.length * (l.dir < 0 and ADBandTurn.REVERSE_COST or 1) + ADBandTurn.SWITCH_COST
    end
    if search.preferSign ~= nil and search.sign ~= search.preferSign then
        cost = cost + ADBandTurn.UTURN_OTHER_SIDE_COST
    end
    return {legs = legs, cost = cost, crop = 0, uturn = true, yturn = true}
end

-- one variant: sideways shift, S out / in lengths, back-out (nose in the crop: {curvature sign, extra m}).
-- Returns {legs, cost} or nil, reason
function ADBandTurn.buildVariant(search, shift, sOut, sIn, back)
    local K = 1 / search.geo.radius
    local run = {pose = search.start, crop = 0}
    run.pose.trailerCrop = 0
    local legs = {}
    -- 0. nose in the crop: back out first
    if back ~= nil then
        local out = newLeg(-1, run.pose)
        if not backOut(search, run, out, back[1] * search.sign * K, back[2]) then
            return nil, "reverse out of the crop blocked"
        end
        if #out.points > 0 then
            legs[#legs + 1] = out
        end
    end
    -- 1. the S
    local leg = newLeg(1, run.pose)
    if not forwardArc(search, run, leg, -search.sign * K, shift) or not forwardArc(search, run, leg, search.sign * K, shift)
        or not forwardArc(search, run, leg, search.sign * K, sOut) or not forwardArc(search, run, leg, -search.sign * K, sIn) then
        return nil, "S blocked"
    end
    if leg.length > 0 then
        legs[#legs + 1] = leg
    end
    -- 2. trailer back into the crop, across the strip
    leg = reverseLeg(search, run, ADBandTurn.REVERSE_MAX, true)
    if leg.length < 1 then
        return nil, "reverse impossible"
    end
    local previous = legs[#legs]
    if previous ~= nil and previous.dir < 0 then
        -- straight after the back-out: one reverse leg
        for _, p in ipairs(leg.points) do
            previous.points[#previous.points + 1] = p
        end
        previous.length = previous.length + leg.length
        previous.endHeading = leg.endHeading
    else
        legs[#legs + 1] = leg
    end
    -- 3. forward turn, more cycles if needed
    local cycles = 0
    while true do
        leg = forwardTurnLeg(search, run)
        if leg.length >= ADBandTurn.STEP then
            legs[#legs + 1] = leg
        end
        if math.abs(wrap(run.pose.a - search.goal)) <= math.rad(ADBandTurn.GOAL_HEADING) then
            break
        end
        cycles = cycles + 1
        if cycles > ADBandTurn.MAX_CYCLES or leg.length < 1 and cycles > 1 then
            return nil, string.format("heading turned only %.0f deg", math.deg(turned(search, run.pose.a)))
        end
        leg = reverseLeg(search, run, ADBandTurn.REVERSE_MAX, false)
        if leg.length < 1 then
            return nil, string.format("blocked at %.0f deg", math.deg(turned(search, run.pose.a)))
        end
        legs[#legs + 1] = leg
    end
    -- 4. straighten
    leg = legs[#legs]
    if leg.dir < 0 then
        leg = newLeg(1, run.pose)
        legs[#legs + 1] = leg
    end
    local driven = 0
    while math.abs(wrap(run.pose.a - run.pose.b)) > math.rad(ADBandTurn.GOAL_ARTICULATION) or run.pose.trailerCrop > ADBandTurn.GOAL_TRAILER_CROP
        or (run.pose.tractorCrop or 0) > 0 do
        if driven >= ADBandTurn.STRAIGHTEN_LENGTH then
            return nil, "trailer not straight"
        end
        local err = wrap(search.goal - run.pose.a)
        local k = math.max(-K, math.min(K, err / 4))
        local p, samples = try(search, run.pose, ADBandTurn.STEP, k)
        if p == nil then
            p, samples = try(search, run.pose, ADBandTurn.STEP, 0)
        end
        if p == nil then
            return nil, "straightening blocked"
        end
        addStep(run, leg, samples, p)
        driven = driven + ADBandTurn.STEP
    end
    local cost = run.crop * ADBandTurn.TRAILER_CROP_COST + (run.tractorCrop or 0) * ADBandTurn.TRACTOR_CROP_COST
    for _, l in ipairs(legs) do
        cost = cost + l.length * (l.dir < 0 and ADBandTurn.REVERSE_COST or 1) + ADBandTurn.SWITCH_COST
    end
    return {legs = legs, cost = cost, crop = run.crop}
end

-- crop side: +1 crop on the right of the heading, -1 on the left, nil if none near
function ADBandTurn.findCropSide(map, pose)
    local rx, rz = math.cos(pose.a), -math.sin(pose.a)
    local best, bestD = nil, math.huge
    for _, side in ipairs({1, -1}) do
        for d = 0, ADBandTurn.CROP_SCAN, 0.5 do
            local flags = ADBandTurn.mapFlags(map, pose.x + rx * d * side, pose.z + rz * d * side)
            if flags % 4 >= 2 and flags % 2 == 0 then
                if d < bestD then
                    best, bestD = side, d
                end
                break
            end
        end
    end
    return best, bestD
end

-- search object: start pose, goal heading (rad); step(n) returns nil while running, then true (self.legs) or false
function ADBandTurn.newSearch(map, geo, start, goalHeading, preferSign, rules)
    local search = {map = map, fieldMap = map.strict or map, geo = geo, start = start, goal = goalHeading, variants = {}, index = 0, expansions = 0, preferSign = preferSign, rules = rules}
    -- U-turn ground: AutoDrive setting restrictToField: strict = field only, otherwise grass allowed too
    local uturnMap = (rules == nil or rules.field == "strict") and (map.strict or map) or map
    -- end of row: forward U-turns first (field only, both senses, preferred side cheaper)
    if preferSign ~= nil and (start.tractorFrontCrop or 0) == 0 then
        for _, sign in ipairs({preferSign, -preferSign}) do
            for _, swing in ipairs(ADBandTurn.UTURN_SWING) do
                for _, lead in ipairs(ADBandTurn.UTURN_LEAD) do
                    search.variants[#search.variants + 1] = {"uturn", lead, swing, uturnMap, sign}
                end
                for _, back in ipairs(ADBandTurn.UTURN_BACK) do
                    search.variants[#search.variants + 1] = {"uturn", -back, swing, uturnMap, sign}
                end
            end
        end
    end
    -- Y manoeuvre: whenever the train is not nose in the crop and the map is wide enough, both senses
    if (start.tractorFrontCrop or 0) == 0 and map.yturn then
        local ySigns = preferSign ~= nil and {preferSign, -preferSign} or {1, -1}
        for _, sign in ipairs(ySigns) do
            for _, swing in ipairs(ADBandTurn.Y_SWING) do
                for _, lead in ipairs(ADBandTurn.Y_LEAD) do
                    for _, back in ipairs(ADBandTurn.Y_BACK) do
                        if swing + lead > 0 or preferSign == nil then
                            search.variants[#search.variants + 1] = {"yturn", lead, swing, uturnMap, sign, back}
                        end
                    end
                end
            end
        end
    end
    ADBandTurn.checkPose(map.strict or map, geo, start)
    local cropSide, distance = ADBandTurn.findCropSide(map, start)
    search.cropSide, search.cropDistance = cropSide, distance
    -- the heading turns away from the crop first (positive angle = turning right), then the other way: no crop
    -- close by (or a blocked side), both senses are tried and the cheapest manoeuvre is kept
    local signs = cropSide ~= nil and {-cropSide, cropSide} or {1, -1}
    -- nose in the crop (blocked against it): back out first, no sideways shift
    local backs, shifts = {false}, ADBandTurn.SHIFT
    if (start.tractorFrontCrop or 0) > 0 then
        backs, shifts = {}, {0}
        for _, kSign in ipairs({-1, 0, 1}) do
            for _, extra in ipairs(ADBandTurn.BACK_OUT_EXTRA) do
                backs[#backs + 1] = {kSign, extra}
            end
        end
    end
    -- end of row: the field setting applies to these manoeuvres too (narrow strip fallback keeps the grass)
    local maps = {map, map.strict}
    if rules ~= nil and rules.field == "strict" then
        maps = {map.strict or map}
    end
    for _, sign in ipairs(signs) do
        for _, m in ipairs(maps) do
            for _, back in ipairs(backs) do
                for _, shift in ipairs(shifts) do
                    for _, sOut in ipairs(ADBandTurn.S_OUT) do
                        for _, sIn in ipairs(ADBandTurn.S_IN) do
                            search.variants[#search.variants + 1] = {shift, sOut, sIn, m, sign, back or nil}
                        end
                    end
                end
            end
        end
    end
    return search
end

function ADBandTurn.stepSearch(search)
    for _ = 1, ADBandTurn.VARIANTS_PER_FRAME do
        search.index = search.index + 1
        local variant = search.variants[search.index]
        if variant == nil then
            if search.best ~= nil then
                search.legs = search.best.legs
                search.grass = search.best.grass
                search.sign = search.best.sign
                search.uturn = search.best.uturn
                search.yturn = search.best.yturn
                return true
            end
            search.failReason = "no manoeuvre possible (" .. tostring(search.lastReason) .. ")"
            return false
        end
        search.expansions = search.expansions + 1
        search.map = variant[4]
        search.sign = variant[5]
        local result, reason
        if variant[1] == "uturn" then
            result, reason = ADBandTurn.buildUTurn(search, variant[2], variant[3])
        elseif variant[1] == "yturn" then
            result, reason = ADBandTurn.buildYTurn(search, variant[2], variant[3], variant[6])
        else
            result, reason = ADBandTurn.buildVariant(search, variant[1], variant[2], variant[3], variant[6])
        end
        if result ~= nil then
            result.grass = variant[4] ~= search.fieldMap
            result.sign = variant[5]
        end
        if result ~= nil and (search.best == nil or result.cost < search.best.cost) then
            search.best = result
        elseif result == nil then
            if (variant[1] == "uturn" or variant[1] == "yturn") and variant[5] == search.preferSign then
                search.uturnReasons = search.uturnReasons or {}
                search.uturnReasons[reason] = (search.uturnReasons[reason] or 0) + 1
            end
            search.lastReason = reason
            search.reasons = search.reasons or {}
            search.reasons[reason] = (search.reasons[reason] or 0) + 1
        end
    end
    return nil
end

-- ---------------------------------------------------------------------------------------------------------------
-- game side
-- ---------------------------------------------------------------------------------------------------------------

local function logName(vehicle)
    return tostring(vehicle ~= nil and vehicle.getName ~= nil and vehicle:getName() or "?")
end

local function headingOfNode(node)
    local dx, _, dz = localDirectionToWorld(node, 0, 0, 1)
    return atan2(dx, dz)
end
ADBandTurn.headingOfNode = headingOfNode

-- box of a unit (size.length, size.width, size.lengthOffset around its root node) along heading a, around x, z
local function unitBox(unit, x, z, a, frontFrom)
    local root = unit.components[1].node
    local length = unit.size ~= nil and unit.size.length or 6
    local width = unit.size ~= nil and unit.size.width or 3
    local offset = unit.size ~= nil and unit.size.lengthOffset or 0
    local cx, _, cz = localToWorld(root, 0, 0, offset)
    local c = (cx - x) * math.sin(a) + (cz - z) * math.cos(a)
    return ADBandTurn.prepareBox({front = c + length / 2, rear = c - length / 2, half = width / 2, frontFrom = frontFrom})
end

-- tractor and one trailer: axle positions, hitch distance, trailer length, boxes, radius. nil, reason if unusable
function ADBandTurn.getGeometry(vehicle)
    local sdm = vehicle.ad.specialDrivingModule
    local reverseNode = sdm:getReverseNode()
    local trailer = vehicle.trailer
    if reverseNode == nil or trailer == nil or sdm.reverseSolo then
        return nil, "no wheeled trailer"
    end
    local units = 0
    for _, implement in pairs(AutoDrive.getAllImplements(vehicle, true)) do
        if implement ~= vehicle and implement.spec_wheels ~= nil and implement.spec_wheels.wheels ~= nil and #implement.spec_wheels.wheels > 0 then
            units = units + 1
        end
    end
    if units > 1 then
        return nil, "plusieurs remorques"
    end
    local axleNode = vehicle.getAIDirectionNode ~= nil and vehicle:getAIDirectionNode() or vehicle.components[1].node
    local x, _, z = getWorldTranslation(axleNode)
    local a = headingOfNode(axleNode)
    local tx, _, tz = getWorldTranslation(reverseNode)
    local b = headingOfNode(trailer.components[1].node)
    -- hitch: the input attacher joint of the trailer, else where the two axes meet
    local hx, hz = nil, nil
    if trailer.getActiveInputAttacherJoint ~= nil then
        local joint = trailer:getActiveInputAttacherJoint()
        if joint ~= nil and joint.node ~= nil then
            local jx, _, jz = getWorldTranslation(joint.node)
            hx, hz = jx, jz
        end
    end
    if hx == nil then
        local l = math.max(4, AutoDrive.getTractorTrainLength(vehicle, false, true) * 0.6)
        hx, hz = tx + math.sin(b) * l, tz + math.cos(b) * l
    end
    local hitch = (x - hx) * math.sin(a) + (z - hz) * math.cos(a)
    local trailerLength = MathUtil.vector2Length(hx - tx, hz - tz)
    if trailerLength < 2 then
        return nil, "trailer geometry unknown"
    end
    local radius = AutoDrive.getDriverRadius(vehicle, true)
    if radius == nil or radius < 3 or radius > 20 then
        radius = 7
    end
    local geo = {
        hitch = hitch,
        trailer = trailerLength,
        radius = radius * ADBandTurn.RADIUS_MARGIN,
        tractorBox = unitBox(vehicle, x, z, a, ADBandTurn.TRACTOR_FRONT_FROM),
        trailerBox = unitBox(trailer, tx, tz, b)
    }
    -- the trailer heading used by the search is the hitch direction seen from its axle
    local pose = {x = x, z = z, a = a, tx = tx, tz = tz, b = atan2(hx - tx, hz - tz)}
    return geo, pose
end

-- the local map, read over several frames: returns true when complete
function ADBandTurn.updateMapReading(bt)
    local map = bt.map
    local pf = PathFinderModule
    local total = map.n * map.n
    local stored = pf.ignoreIsolatedCrop
    pf.ignoreIsolatedCrop = true
    local budget = ADBandTurn.MAP_READS_PER_FRAME
    while bt.readIndex < total and budget > 0 do
        local i = bt.readIndex
        local ix, iz = i % map.n, math.floor(i / map.n)
        local x, z = map.ox + (ix + 0.5) * map.step, map.oz + (iz + 0.5) * map.step
        local flags = 0
        if not pf.getMapOnFieldAt(x, z) then
            flags = flags + ADBandTurn.OFF_FIELD
        end
        local crop = pf.getMapCropAt(x, z)
        if crop ~= nil and crop ~= 0 then
            flags = flags + ADBandTurn.CROP
        end
        map.cells[i + 1] = flags
        bt.readIndex = i + 1
        budget = budget - 1
    end
    pf.ignoreIsolatedCrop = stored
    if bt.readIndex < total then
        return false
    end
    if not bt.grassDone then
        bt.grassDone = true
        ADBandTurn.allowGrass(map)
    end
    -- obstacles: small boxes 0.5 to 2.5 m above the ground (trees, posts, walls, vehicles; the own train ignored)
    local module = bt.vehicle.ad.trafficYieldModule
    if module == nil or module.overlapHit == nil then
        return true
    end
    local step = ADBandTurn.OBSTACLE_STEP
    local np = math.floor(2 * (bt.mapRadius or ADBandTurn.MAP_RADIUS) / step)
    local probes = ADBandTurn.OBSTACLE_PROBES_PER_FRAME
    -- end of row: the harvester is left out of the calculation, it drives away during the U-turn
    module.overlapIgnoreVehicle = bt.preferSign ~= nil and bt.harvester or nil
    while bt.probeIndex < np * np and probes > 0 do
        local i = bt.probeIndex
        local px, pz = map.ox + (i % np + 0.5) * step, map.oz + (math.floor(i / np) + 0.5) * step
        local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, px, 0, pz)
        if module:overlapHit(px, y + 1.5, pz, 0, step / 2, 1.0, step / 2) then
            bt.obstacles = bt.obstacles + 1
            local per = math.floor(step / map.step + 0.5)
            local bx, bz = math.floor((px - step / 2 - map.ox) / map.step + 0.5), math.floor((pz - step / 2 - map.oz) / map.step + 0.5)
            for jz = bz, bz + per - 1 do
                for jx = bx, bx + per - 1 do
                    if jx >= 0 and jz >= 0 and jx < map.n and jz < map.n then
                        local k = jz * map.n + jx + 1
                        for _, m in ipairs({map, map.strict}) do
                            if not hasFlag(m.cells[k], ADBandTurn.OBSTACLE) then
                                m.cells[k] = m.cells[k] + ADBandTurn.OBSTACLE
                            end
                        end
                    end
                end
            end
        end
        bt.probeIndex = i + 1
        probes = probes - 1
    end
    module.overlapIgnoreVehicle = nil
    return bt.probeIndex >= np * np
end

function ADBandTurn.canTry(vehicle)
    local last = vehicle.ad.bandTurnFailed
    if last == nil then
        return true
    end
    local x, _, z = getWorldTranslation(vehicle.components[1].node)
    return (g_time or 0) - last.t > ADBandTurn.RETRY_TIME or MathUtil.vector2Length(x - last.x, z - last.z) > ADBandTurn.RETRY_DISTANCE
end

-- heading of the train reversed against the harvester (head-on or driving away from it), harvester not turning.
-- minAngle (deg, default 120): angle to its heading from which it counts as behind
function ADBandTurn.isFacingHarvester(vehicle, harvester, maxDistance, minAngle)
    if harvester == nil or harvester.components == nil then
        return false
    end
    if AutoDrive.getDistanceBetween(vehicle, harvester) > (maxDistance or 100) then
        return false
    end
    if AutoDrive.getIsCPTurning ~= nil and AutoDrive:getIsCPTurning(harvester) then
        return false
    end
    -- same angle as the hand-over checks of the unloader mode (degrees)
    local mode = vehicle.ad.modes ~= nil and vehicle.ad.modes[AutoDrive.MODE_UNLOAD] or nil
    if mode == nil or mode.getAngleToCombineHeading == nil or mode.combine ~= harvester then
        return false
    end
    local angle = mode:getAngleToCombineHeading()
    return angle ~= nil and angle > (minAngle or 120) and angle <= 180, angle
end

-- starts the U-turn (heading reversed). harvester: after the turn, wait for it or back up to its pipe
-- goalHeading: heading to reach instead of the reversed one (the heading of a standing harvester)
function ADBandTurn.start(vehicle, reason, harvester, preferSign, goalHeading)
    local name = logName(vehicle)
    local geo, pose = ADBandTurn.getGeometry(vehicle)
    if geo == nil then
        ADBandTurn.markFailed(vehicle)
        return false
    end
    -- end of row: wider map, a reverse then a U-turn needs the room
    -- with a harvester: wider still, the Y manoeuvre backs up to 50 m
    local r = harvester ~= nil and ADBandTurn.Y_MAP_RADIUS or (preferSign ~= nil and ADBandTurn.END_ROW_MAP_RADIUS or ADBandTurn.MAP_RADIUS)
    local map = {ox = pose.x - r, oz = pose.z - r, step = ADBandTurn.MAP_STEP, n = math.floor(2 * r / ADBandTurn.MAP_STEP), cells = {}, yturn = harvester ~= nil}
    vehicle.ad.bandTurn = {
        vehicle = vehicle, geo = geo, start = pose, map = map, readIndex = 0, probeIndex = 0, obstacles = 0,
        phase = "map", reason = reason, harvester = harvester, time = 0, goal = goalHeading ~= nil and wrap(goalHeading) or wrap(pose.a + PI), preferSign = preferSign, mapRadius = r
    }
    -- the AutoDrive settings apply to every turn near a harvester (the
    -- fallback turn of the catch task allowed the grass)
    if preferSign ~= nil or harvester ~= nil then
        local field, fruit = AutoDrive.getSetting("restrictToField", vehicle), AutoDrive.getSetting("avoidFruit", vehicle)
        vehicle.ad.bandTurn.rules = {
            field = field == "strict" and "strict" or (field and "on" or "off"),
            crop = fruit == "strict" and "strict" or (fruit and "on" or "off")
        }
    end
    return true
end

-- new map and search from the current pose, the goal heading kept
function ADBandTurn.replan(vehicle, bt, why)
    local geo, pose = ADBandTurn.getGeometry(vehicle)
    if geo == nil then
        return false
    end
    bt.replans = (bt.replans or 0) + 1
    local r = bt.mapRadius or ADBandTurn.MAP_RADIUS
    bt.geo, bt.start = geo, pose
    bt.map = {ox = pose.x - r, oz = pose.z - r, step = ADBandTurn.MAP_STEP, n = math.floor(2 * r / ADBandTurn.MAP_STEP), cells = {}, yturn = bt.harvester ~= nil}
    bt.readIndex, bt.probeIndex, bt.obstacles, bt.grassDone = 0, 0, 0, nil
    bt.legs, bt.search, bt.stillPos, bt.holdReason = nil, nil, nil, nil
    bt.phase, bt.time = "map", 0
    return true
end

function ADBandTurn.markFailed(vehicle)
    local x, _, z = getWorldTranslation(vehicle.components[1].node)
    vehicle.ad.bandTurnFailed = {x = x, z = z, t = g_time or 0}
end

function ADBandTurn.stop(vehicle, dt)
    local sdm = vehicle.ad.specialDrivingModule
    sdm:stopVehicle()
    sdm:update(dt)
end

function ADBandTurn.fail(vehicle, dt, why)
    ADBandTurn.markFailed(vehicle)
    vehicle.ad.bandTurn = nil
    ADBandTurn.stop(vehicle, dt)
    return "failed"
end

function ADBandTurn.abort(vehicle)
    if vehicle ~= nil and vehicle.ad ~= nil then
        local bt = vehicle.ad.bandTurn
        if bt ~= nil then
            -- the task was stopped from outside (driver stopped, task replaced): say who did it
            if printCallstack ~= nil then
                printCallstack()
            end
        end
        vehicle.ad.bandTurn = nil
    end
end

-- "running", "done" (turned), "joined" (turned and alongside the harvester) or "failed"
function ADBandTurn.update(vehicle, dt)
    local bt = vehicle.ad.bandTurn
    if bt == nil then
        return "failed"
    end
    bt.time = bt.time + dt
    local name = logName(vehicle)
    if bt.phase == "map" then
        ADBandTurn.stop(vehicle, dt)
        if ADBandTurn.updateMapReading(bt) then
            -- nose in the crop (blocked against it): the goal is the way the trailer came, reversed
            ADBandTurn.checkPose(bt.map.strict or bt.map, bt.geo, bt.start)
            if (bt.start.tractorFrontCrop or 0) > 0 and bt.replans == nil then
                bt.goal = wrap(bt.start.b + PI)
            end
            bt.search = ADBandTurn.newSearch(bt.map, bt.geo, bt.start, bt.goal, bt.preferSign, bt.rules)
            bt.phase = "search"
        end
        return "running"
    elseif bt.phase == "search" then
        ADBandTurn.stop(vehicle, dt)
        local result = ADBandTurn.stepSearch(bt.search)
        if result == nil then
            return "running"
        elseif result == false then
            return ADBandTurn.fail(vehicle, dt, string.format("%s, %d tries, %d obstacles seen",
                tostring(bt.search.failReason), bt.search.expansions, bt.obstacles))
        end
        bt.legs = bt.search.legs
        local parts = {}
        for _, leg in ipairs(bt.legs) do
            parts[#parts + 1] = string.format("%s %.0f m", leg.dir > 0 and "forward" or "reverse", leg.length)
        end
        if bt.preferSign ~= nil and bt.search.uturnReasons ~= nil then
            local why = {}
            for r, n in pairs(bt.search.uturnReasons) do
                why[#why + 1] = string.format("%s x%d", r, n)
            end
        end
        bt.search = nil
        -- the map is kept: the front of the tractor is checked against the crop while driving
        bt.legIndex, bt.pointIndex, bt.pause, bt.stillTime = 1, 1, ADBandTurn.LEG_PAUSE, 0
        bt.phase = "drive"
        return "running"
    elseif bt.phase == "drive" then
        if bt.time > ADBandTurn.TIMEOUT then
            return ADBandTurn.fail(vehicle, dt, "too long")
        end
        ADBandTurn.holdHarvester(vehicle, bt)
        local state = ADBandTurn.driveLeg(vehicle, bt, dt)
        if state == "running" then
            return "running"
        elseif state ~= "done" then
            -- off the planned line (the reverse is not exact): plan the rest again from here, same goal
            if (string.find(state, "off line", 1, true) ~= nil or string.find(state, "crop", 1, true) ~= nil or string.find(state, "blocked", 1, true) ~= nil) and (bt.replans or 0) < ADBandTurn.MAX_REPLANS and ADBandTurn.replan(vehicle, bt, state) then
                ADBandTurn.stop(vehicle, dt)
                return "running"
            end
            return ADBandTurn.fail(vehicle, dt, state)
        end
        bt.legIndex = bt.legIndex + 1
        bt.pointIndex, bt.pause, bt.stillTime = 1, ADBandTurn.LEG_PAUSE, 0
        if bt.legIndex <= #bt.legs then
            return "running"
        end
        -- once turned, hand back to AutoDrive at once, no waiting for the harvester here
        vehicle.ad.bandTurn = nil
        ADBandTurn.stop(vehicle, dt)
        return "done"
    elseif bt.phase == "join" then
        return ADBandTurn.updateJoin(vehicle, bt, dt)
    end
    return ADBandTurn.fail(vehicle, dt, "etat inconnu")
end

function ADBandTurn.holdHarvester(vehicle, bt)
    local harvester = bt.harvester or vehicle.ad.lastHarvester
    if harvester ~= nil and harvester.components ~= nil and AutoDrive.getDistanceBetween(vehicle, harvester) < ADBandTurn.HOLD_HARVESTER_DISTANCE then
        AutoDrive:holdCPCombine(harvester)
    end
end

local function nearestIndex(points, from, x, z, useTrailer)
    local best, bestD = from, math.huge
    for i = from, math.min(#points, from + 40) do
        local p = points[i]
        local px, pz = p.x, p.z
        if useTrailer then
            px, pz = p.tx, p.tz
        end
        local d = MathUtil.vector2Length(px - x, pz - z)
        if d < bestD then
            best, bestD = i, d
        end
    end
    return best, bestD
end

-- one leg: "running", "done" or the reason of a stop
function ADBandTurn.driveLeg(vehicle, bt, dt)
    local leg = bt.legs[bt.legIndex]
    local sdm = vehicle.ad.specialDrivingModule
    if bt.pause > 0 then
        bt.pause = bt.pause - dt
        sdm:stopVehicle()
        sdm:update(dt)
        if bt.pause <= 0 then
            sdm:reset()
        end
        return "running"
    end
    local axleNode = vehicle.getAIDirectionNode ~= nil and vehicle:getAIDirectionNode() or vehicle.components[1].node
    local x, y, z = getWorldTranslation(axleNode)
    local points = leg.points
    if #points == 0 then
        return "done"
    end
    local last = points[#points]
    -- stuck: less than STUCK_MOVE in STUCK_TIME (the speed alone can jitter above zero)
    if bt.stillPos == nil or MathUtil.vector2Length(x - bt.stillPos.x, z - bt.stillPos.z) > ADBandTurn.STUCK_MOVE then
        bt.stillPos = {x = x, z = z}
        bt.stillTime = 0
    else
        bt.stillTime = bt.stillTime + dt
        if bt.stillTime > ADBandTurn.STUCK_TIME then
            local trailerNode = vehicle.trailer ~= nil and vehicle.trailer.components ~= nil and vehicle.trailer.components[1].node or nil
            local articulation = trailerNode ~= nil and math.deg(math.abs(wrap(headingOfNode(axleNode) - headingOfNode(trailerNode)))) or -1
            return string.format("blocked %.0f s in place (%s; speed %.1f km/h, motor %s, articulation %.0f deg, time %s)", bt.stillTime / 1000,
                tostring(bt.holdReason or "no progress"), math.abs(vehicle.lastSpeedReal or 0) * 3600,
                tostring(vehicle.getIsMotorStarted ~= nil and vehicle:getIsMotorStarted()), articulation, leg.dir > 0 and "forward" or "reverse")
        end
    end
    -- the engine may have been stopped while waiting (stopped timer of the special driving module)
    sdm:releaseVehicle()
    if vehicle.startMotor and not vehicle:getIsMotorStarted() and vehicle:getCanMotorRun() then
        vehicle:startMotor()
    end
    if leg.dir > 0 then
        local i, d = nearestIndex(points, bt.pointIndex, x, z, false)
        bt.pointIndex = i
        if d > ADBandTurn.MAX_DEVIATION then
            return string.format("off line by %.1f m forward", d)
        end
        -- the front wheels never go into the crop: the line was cut inside, stop and plan again from here
        if bt.map ~= nil then
            local _, _, _, frontCrop = checkBody(bt.map, x, z, headingOfNode(axleNode), bt.geo.tractorBox)
            if (frontCrop or 0) > ADBandTurn.FRONT_CROP_TOLERANCE then
                return string.format("tractor front in the crop (off line %.1f m)", d)
            end
        end
        -- end of the leg reached or passed (along its last direction)
        local prev = points[math.max(1, #points - 2)]
        local ex, ez = last.x - prev.x, last.z - prev.z
        if i >= #points - 1 or (ex * (last.x - x) + ez * (last.z - z)) <= 0 then
            return "done"
        end
        local j = i
        while j < #points and MathUtil.vector2Length(points[j].x - x, points[j].z - z) < ADBandTurn.LOOK_AHEAD do
            j = j + 1
        end
        local t = points[j]
        local lx, lz = AutoDrive.getDriveDirection(vehicle, t.x, y, t.z, axleNode)
        local maxAngle = 60
        if vehicle.maxRotation then
            maxAngle = vehicle.maxRotation > TWO_PI and vehicle.maxRotation or math.deg(vehicle.maxRotation)
        end
        sdm:releaseVehicle()
        if vehicle.startMotor and not vehicle:getIsMotorStarted() and vehicle:getCanMotorRun() then
            vehicle:startMotor()
        end
        vehicle.ad.trailerModule:handleTrailerReversing(false)
        AutoDrive.driveInDirection(vehicle, dt, maxAngle, 0.5, 0.2, 20, true, true, lx, lz, ADBandTurn.SPEED, 1)
        return "running"
    end
    -- reverse: the trailer axle follows its planned line (special driving module, as the track reverse)
    local reverseNode = sdm:getReverseNode()
    if reverseNode == nil then
        return "no reverse point left"
    end
    local rx, _, rz = getWorldTranslation(reverseNode)
    local i, d = nearestIndex(points, bt.pointIndex, rx, rz, true)
    bt.pointIndex = i
    if d > ADBandTurn.MAX_DEVIATION + 0.5 then
        return string.format("off line by %.1f m reversing", d)
    end
    local prev = points[math.max(1, #points - 2)]
    local ex, ez = last.x - prev.x, last.z - prev.z
    if i >= #points - 1 or (ex * (last.x - x) + ez * (last.z - z)) <= 0 then
        return "done"
    end
    local j = i
    while j < #points and MathUtil.vector2Length(points[j].tx - rx, points[j].tz - rz) < ADBandTurn.LOOK_BEHIND do
        j = j + 1
    end
    local t = points[j]
    if vehicle.ad.collisionDetectionModule ~= nil and vehicle.ad.collisionDetectionModule:checkReverseCollision() then
        -- the reverse module holds the train: something behind the trailer
        bt.holdReason = "obstacle behind the trailer"
    else
        bt.holdReason = nil
    end
    sdm:reverseToTargetLocation(dt, {x = t.tx, y = y, z = t.tz}, ADBandTurn.SPEED)
    return "running"
end

-- after a head-on arrival: the harvester is behind, wait for it to come alongside, or back up to its pipe if it waits
function ADBandTurn.updateJoin(vehicle, bt, dt)
    local name = logName(vehicle)
    local harvester = bt.harvester
    local mode = vehicle.ad.modes ~= nil and vehicle.ad.modes[AutoDrive.MODE_UNLOAD] or nil
    if harvester == nil or harvester.components == nil or mode == nil or mode.getPipeChasePosition == nil then
        vehicle.ad.bandTurn = nil
        return "done"
    end
    local chasePos = mode:getPipeChasePosition()
    if chasePos == nil then
        vehicle.ad.bandTurn = nil
        return "done"
    end
    local _, _, cz = AutoDrive.worldToLocal(vehicle, chasePos.x, chasePos.y or 0, chasePos.z)
    if cz > -2 then
        vehicle.ad.bandTurn = nil
        return "joined"
    end
    bt.joinTime = bt.joinTime + dt
    local stopped = math.abs(harvester.lastSpeedReal or 0) < 0.0003
    bt.joinStill = stopped and bt.joinStill + dt or 0
    local sdm = vehicle.ad.specialDrivingModule
    local x, _, z = getWorldTranslation(vehicle.components[1].node)
    -- no long blind reverse to the pipe any more, the unloader waits for the harvester where it is
    if ADBandTurn.JOIN_REVERSE and bt.joinStart == nil and bt.joinStill > ADBandTurn.JOIN_STOPPED_TIME then
        bt.joinStart = {x = x, z = z}
        sdm:reset()
    end
    if bt.joinStart ~= nil then
        local backed = MathUtil.vector2Length(x - bt.joinStart.x, z - bt.joinStart.z)
        bt.joinBlocked = math.abs(vehicle.lastSpeedReal or 0) < 0.0002 and (bt.joinBlocked or 0) + dt or 0
        if backed > ADBandTurn.JOIN_MAX_REVERSE or bt.joinBlocked > ADBandTurn.JOIN_STOPPED_TIME then
            vehicle.ad.bandTurn = nil
            ADBandTurn.stop(vehicle, dt)
            return "done"
        end
        -- straight back on the own axis, the trailer leads
        local reverseNode = sdm:getReverseNode()
        if reverseNode == nil then
            vehicle.ad.bandTurn = nil
            return "done"
        end
        local tx, ty, tz = AutoDrive.localToWorld(vehicle, 0, 0, cz - 6)
        sdm:reverseToTargetLocation(dt, {x = tx, y = ty, z = tz}, ADBandTurn.SPEED)
        return "running"
    end
    if bt.joinTime > ADBandTurn.JOIN_WAIT_TIME then
        vehicle.ad.bandTurn = nil
        ADBandTurn.stop(vehicle, dt)
        return "done"
    end
    ADBandTurn.stop(vehicle, dt)
    return "running"
end
