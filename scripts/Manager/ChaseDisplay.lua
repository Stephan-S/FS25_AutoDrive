-- debug channel, kept in
-- the AutoDrive settings. Drawn every frame without the editor mode, for every AutoDrive unloader (unload mode) that
-- drives: its route (blue), the chase line and place (green when clear, red when blocked), the crumbs of the 1st
-- unloader it follows (yellow dots) and the place aimed at (magenta), the step-aside place (orange), the target of a
-- route being planned (white), and a text above the tractor with its task
ADChaseDisplay = {}
ADChaseDisplay.TEXT_SIZE = 0.012
ADChaseDisplay.ROUTE_MAX_POINTS = 150
ADChaseDisplay.COLORS = {
    route = {0.2, 0.5, 1},
    clear = {0.1, 1, 0.1},
    blocked = {1, 0.1, 0.1},
    crumbs = {1, 0.9, 0.1},
    aim = {1, 0.2, 1},
    aside = {1, 0.55, 0.1},
    planning = {1, 1, 1},
}

local function terrainY(x, z)
    return getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 0, z)
end

local function line(ax, ay, az, bx, by, bz, c)
    ADDrawingManager:addLineTask(ax, ay or terrainY(ax, az), az, bx, by or terrainY(bx, bz), bz, 1, c[1], c[2], c[3])
end

local function sphere(x, y, z, c, scale)
    ADDrawingManager:addSphereTask(x, (y or terrainY(x, z)) + 0.6, z, scale or 1, c[1], c[2], c[3], 0.6)
end

local function text(x, y, z, str, c)
    if AutoDrive.renderColoredTextAtWorldPosition ~= nil then
        AutoDrive.renderColoredTextAtWorldPosition(x, y, z, str, getCorrectTextSize(ADChaseDisplay.TEXT_SIZE), {r = c[1], g = c[2], b = c[3]})
    end
end

local function sideName(side)
    if side == AutoDrive.CHASEPOS_LEFT then
        return "left"
    elseif side == AutoDrive.CHASEPOS_RIGHT then
        return "right"
    elseif side == AutoDrive.CHASEPOS_REAR then
        return "behind"
    end
    return "?"
end

function ADChaseDisplay.isEnabled()
    return AutoDrive.DC_CHASEDISPLAY ~= nil and AutoDrive.getDebugChannelIsSet(AutoDrive.DC_CHASEDISPLAY)
end

function ADChaseDisplay.draw()
    if g_currentMission == nil or ADDrawingManager == nil or AutoDrive.getAllVehicles == nil then
        return
    end
    for _, vehicle in pairs(AutoDrive.getAllVehicles()) do
        if vehicle ~= nil and vehicle.ad ~= nil and vehicle.components ~= nil and vehicle.ad.stateModule ~= nil
            and vehicle.ad.stateModule.isActive ~= nil and vehicle.ad.stateModule:isActive()
            and vehicle.ad.stateModule.getMode ~= nil and vehicle.ad.stateModule:getMode() == AutoDrive.MODE_UNLOAD then
            local ok, err = pcall(ADChaseDisplay.drawVehicle, vehicle)
        end
    end
end

function ADChaseDisplay.drawVehicle(vehicle)
    local vx, vy, vz = getWorldTranslation(vehicle.components[1].node)
    local C = ADChaseDisplay.COLORS
    local lines = {}
    local task = vehicle.ad.taskModule ~= nil and vehicle.ad.taskModule.getActiveTask ~= nil and vehicle.ad.taskModule:getActiveTask() or nil
    local mode = vehicle.ad.modes ~= nil and vehicle.ad.modes[AutoDrive.MODE_UNLOAD] or nil

    -- the route being driven
    local dpm = vehicle.ad.drivePathModule
    local wps = dpm ~= nil and dpm.getWayPoints ~= nil and dpm:getWayPoints() or nil
    if wps ~= nil and #wps > 1 then
        local from = 1
        if dpm.getCurrentWayPointIndex ~= nil then
            from = math.max(1, (dpm:getCurrentWayPointIndex() or 2) - 1)
        end
        local last = nil
        for i = from, math.min(#wps, from + ADChaseDisplay.ROUTE_MAX_POINTS) do
            local p = wps[i]
            if last ~= nil and p ~= nil then
                line(last.x, last.y, last.z, p.x, p.y, p.z, C.route)
            end
            last = p
        end
    end

    local pfm = vehicle.ad.pathFinderModule
    if pfm ~= nil and pfm.target ~= nil and pfm.isFinished == false then
        sphere(pfm.target.x, nil, pfm.target.z, C.planning)
        line(vx, vy, vz, pfm.target.x, nil, pfm.target.z, C.planning)
        lines[#lines + 1] = "planning a route"
    end

    if task ~= nil then
        local taskName = task.taskType or "tache"
        -- the chase (FollowCombineTask)
        if task.chasePos ~= nil and task.combine ~= nil then
            local clear = task.lineClear ~= false
            local c = clear and C.clear or C.blocked
            line(vx, vy, vz, task.chasePos.x, task.chasePos.y, task.chasePos.z, c)
            sphere(task.chasePos.x, task.chasePos.y, task.chasePos.z, c)
            local t = "place " .. sideName(task.chaseSide)
            if not clear then
                t = t .. " (line blocked: " .. tostring(task.lineBlockReason) .. ")"
            end
            text(task.chasePos.x, (task.chasePos.y or terrainY(task.chasePos.x, task.chasePos.z)) + 1.5, task.chasePos.z, t, c)
            lines[#lines + 1] = "poursuite : " .. t
        end
        -- the follow of the 1st unloader (DriveToVehicleTask, FollowVehicleTask, StepAsideTask)
        local first = task.targetVehicle or task.targetUnloader
        if first ~= nil and first.ad ~= nil and first.ad.modes ~= nil and first ~= task.combine then
            local firstMode = first.ad.modes[AutoDrive.MODE_UNLOAD]
            local crumbs = firstMode ~= nil and firstMode.getBreadCrumbs ~= nil and firstMode:getBreadCrumbs() or nil
            local items = crumbs ~= nil and crumbs.items or nil
            if items ~= nil then
                for _, cr in ipairs(items) do
                    ADDrawingManager:addSmallSphereTask(cr.x, cr.y or terrainY(cr.x, cr.z), cr.z, C.crumbs[1], C.crumbs[2], C.crumbs[3])
                end
            end
            local what = string.format("follows %s (%d crumbs)", tostring(first.getName ~= nil and first:getName() or "?"), items ~= nil and #items or 0)
            if task.target ~= nil and task.target.x ~= nil then
                local c = task.taskType == "StepAsideTask" and C.aside or C.aim
                line(vx, vy, vz, task.target.x, task.target.y, task.target.z, c)
                sphere(task.target.x, task.target.y, task.target.z, c)
                local label = task.taskType == "StepAsideTask" and "place aside" or ("target: " .. tostring(task.targetKind or "crumb"))
                text(task.target.x, (task.target.y or terrainY(task.target.x, task.target.z)) + 1.5, task.target.z, label, c)
                what = what .. ", " .. label
            end
            lines[#lines + 1] = what
        end
        local ok, info = pcall(task.getInfoText, task)
        lines[#lines + 1] = taskName .. ((ok and info ~= nil) and (" : " .. tostring(info)) or "")
    end
    if mode ~= nil and mode.getStateName ~= nil then
        lines[#lines + 1] = "etat : " .. tostring(mode:getStateName())
    end

    -- the text above the tractor
    local name = tostring(vehicle.getName ~= nil and vehicle:getName() or "?")
    text(vx, vy + 4.5, vz, name, C.planning)
    for i, l in ipairs(lines) do
        text(vx, vy + 4.5 - i * 0.55, vz, l, C.planning)
    end
end
