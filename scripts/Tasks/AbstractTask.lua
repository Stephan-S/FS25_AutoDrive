AbstractTask = {}
AbstractTask_mt = {__index = AbstractTask}

function AbstractTask:setUp()
end

function AbstractTask:update(dt)
end

function AbstractTask:abort()
end

function AbstractTask:finished()
end

function AbstractTask:getInfoText()
    return nil
end

function AbstractTask:getI18nInfo()
    return ""
end

function AbstractTask:getExcludedVehiclesForCollisionCheck()
    return {}
end

-- status text: seconds spent in the current state (key: the state, or any value that changes with the phase)
function AbstractTask:getStateSeconds(key)
    if key == nil then
        key = self.state
    end
    local now = g_time or 0
    if self.infoStateKey ~= key or self.infoStateStart == nil then
        self.infoStateKey = key
        self.infoStateStart = now
    end
    return math.floor((now - self.infoStateStart) / 1000)
end

-- status text while the pathfinder runs: what it does and for how long
function AbstractTask:getPlanningI18n()
    return "$l10n_AD_task_planning_way;" .. string.format(" (%d s)", self:getStateSeconds())
end

-- status text: the harvester named by its kind (forage harvester or combine)
function AbstractTask.getHarvesterI18n(combine)
    if combine ~= nil and combine.ad ~= nil and combine.ad.isChopper then
        return "$l10n_AD_task_hv_chopper;"
    end
    return "$l10n_AD_task_hv_combine;"
end
