ADHarvestManager = {}
ADHarvestManager.debug = false

ADHarvestManager.MAX_PREDRIVE_LEVEL = 0.85
ADHarvestManager.MAX_SEARCH_RANGE = 300

function ADHarvestManager:load()
    self.harvesters = {}
    self.idleHarvesters = {}
    self.activeUnloaders = {}
    self.idleUnloaders = {}
    self.assignmentDelayTimer = AutoDriveTON:new()
end

function ADHarvestManager:registerHarvester(harvester)
    ADHarvestManager.debugMsg(harvester, "ADHarvestManager:registerHarvester")
    if not table.contains(self.idleHarvesters, harvester) and not table.contains(self.harvesters, harvester) then
        ADHarvestManager.debugMsg(harvester, "ADHarvestManager:registerHarvester - inserted")
        if harvester ~= nil and harvester.ad ~= nil then
            local rootVehicle = harvester:getRootVehicle()
            rootVehicle.ad.isRegisterdHarvester = true
        end
        if g_server ~= nil then
            table.insert(self.idleHarvesters, harvester)
        end
    end
end

function ADHarvestManager:unregisterHarvester(harvester)
    ADHarvestManager.debugMsg(harvester, "ADHarvestManager:unregisterHarvester")
    if harvester ~= nil and harvester.ad ~= nil then
        local rootVehicle = harvester:getRootVehicle()
        rootVehicle.ad.isRegisterdHarvester = false
    end
    if g_server ~= nil then
        if table.contains(self.idleHarvesters, harvester) then
            local index = table.indexOf(self.idleHarvesters, harvester)
            local harvester = table.remove(self.idleHarvesters, index)
            ADHarvestManager.debugMsg(harvester, "ADHarvestManager:unregisterHarvester - removed - idleHarvesters")
        end
        if table.contains(self.harvesters, harvester) then
            local index = table.indexOf(self.harvesters, harvester)
            local harvester = table.remove(self.harvesters, index)
            ADHarvestManager.debugMsg(harvester, "ADHarvestManager:unregisterHarvester - removed - harvesters")
        end
    end
end

function ADHarvestManager:registerAsUnloader(vehicle)
    ADHarvestManager.debugMsg(vehicle, "ADHarvestManager:registerAsUnloader")
    --remove from active and idle list
    -- an unloader that waits for a call again while keeping its harvester
    -- (U-turn given up, new route: setToWaitForCall(true)) is not leaving it, its follower stays its follower. The
    -- promotion here sent an unloader to the harvester and back behind an unloader every 2 s
    local mode = vehicle.ad ~= nil and vehicle.ad.modes ~= nil and vehicle.ad.modes[AutoDrive.MODE_UNLOAD] or nil
    local keepFollower = mode ~= nil and mode.combine ~= nil and mode:getFollowingUnloader() ~= nil
    self:unregisterAsUnloader(vehicle, keepFollower)
    if not table.contains(self.idleUnloaders, vehicle) then
        ADHarvestManager.debugMsg(vehicle, "ADHarvestManager:registerAsUnloader - inserted")
        table.insert(self.idleUnloaders, vehicle)
        -- queue by order of arrival: the first unloader waiting for this harvester is called first.
        -- A failed call keeps the place in the queue, it is only cleared when the unloader leaves to unload
        if vehicle.ad ~= nil and vehicle.ad.unloaderQueueTime == nil then
            vehicle.ad.unloaderQueueTime = g_time
        end
    end
end

function ADHarvestManager:unregisterAsUnloader(vehicle, keepFollower)
    if vehicle.ad and vehicle.ad.modes ~= nil and vehicle.ad.modes[AutoDrive.MODE_UNLOAD] ~= nil and not keepFollower then
        local followingUnloder = vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getFollowingUnloader()
        if followingUnloder ~= nil then
            --promote following unloader to current unloader
            followingUnloder.ad.modes[AutoDrive.MODE_UNLOAD]:promoteFollowingUnloader(vehicle.ad.modes[AutoDrive.MODE_UNLOAD].combine)
        end
    end
    if table.contains(self.idleUnloaders, vehicle) then
        table.removeValue(self.idleUnloaders, vehicle)
    end
    if table.contains(self.activeUnloaders, vehicle) then
        table.removeValue(self.activeUnloaders, vehicle)
        local controlledVehicle = AutoDrive.getControlledVehicle()
        if controlledVehicle ~= nil and vehicle == controlledVehicle then
            --Give the player some time to reset/reposition the fired unloader
            self.assignmentDelayTimer:timer(false)
        else
            --Only short delay for AI controlled unloader being removed
            self.assignmentDelayTimer:timer(false)
            self.assignmentDelayTimer.elapsedTime = 10000
        end
    end
end

function ADHarvestManager:fireUnloader(unloader, harvester)
    if unloader.ad.stateModule:isActive() then
        local follower = unloader.ad.modes[AutoDrive.MODE_UNLOAD]:getFollowingUnloader()
        if follower ~= nil then
            follower.ad.taskModule:abortAllTasks()
            follower.ad.taskModule:addTask(StopAndDisableADTask:new(follower, ADTaskModule.DONT_PROPAGATE, true))
        end
        unloader.ad.taskModule:abortAllTasks()
        unloader.ad.taskModule:addTask(StopAndDisableADTask:new(unloader, ADTaskModule.DONT_PROPAGATE, true))
    end
    self:unregisterAsUnloader(unloader)
end

function ADHarvestManager:update(dt)
    self.assignmentDelayTimer:timer(true, 10000, dt)
    for _, idleHarvester in pairs(self.idleHarvesters) do
        local vehicle = idleHarvester
        if vehicle.isTrailedHarvester then
            vehicle = vehicle.trailingVehicle
        end
        if (vehicle.getIsAIActive ~= nil and vehicle:getIsAIActive()) or (AutoDrive:getIsEntered(vehicle:getRootVehicle()) and AutoDrive.isPipeOut(vehicle)) then
            -- ADHarvestManager.debugMsg(idleHarvester, "ADHarvestManager:update add to harvesters")
            table.insert(self.harvesters, idleHarvester)
            table.removeValue(self.idleHarvesters, idleHarvester)
        end
    end
    for _, harvester in pairs(self.harvesters) do
        local vehicle = harvester
        if vehicle.isTrailedHarvester then
            vehicle = vehicle.trailingVehicle
        end
        local unloader = self:getAssignedUnloader(harvester)
        local driverStopped = not ((vehicle.getIsAIActive ~= nil and vehicle:getIsAIActive()) or (AutoDrive:getIsEntered(vehicle:getRootVehicle()) and AutoDrive.isPipeOut(vehicle)))
        if driverStopped and harvester.ad ~= nil and harvester.ad.isChopper and not (vehicle.ad.onRouteToRefuel or vehicle.ad.onRouteToRepair) then
            -- forage harvester: its driver (Courseplay) stops for a moment now and then. The unloaders keep their place,
            -- the queue and the attached unloader, and wait; only a long stop releases them as before
            harvester.ad.driverStoppedSince = harvester.ad.driverStoppedSince or g_time
            if g_time - harvester.ad.driverStoppedSince < ADHarvestManager.CHOPPER_DRIVER_STOP_TIME then
                driverStopped = false
            end
        elseif harvester.ad ~= nil then
            harvester.ad.driverStoppedSince = nil
        end
        if driverStopped
            or (vehicle.ad.onRouteToRefuel or vehicle.ad.onRouteToRepair) -- harvester is going to refuel or park
            or (unloader ~= nil and unloader.ad.stateModule:getFirstMarker() ~= vehicle.ad.stateModule:getFirstMarker()) -- harvester - unloader targets do not match
        then
            -- ADHarvestManager.debugMsg(harvester, "ADHarvestManager:update change to idleHarvesters")
            table.insert(self.idleHarvesters, harvester)
            table.removeValue(self.harvesters, harvester)

            if unloader ~= nil then
                -- ADHarvestManager.debugMsg(harvester, "ADHarvestManager:update fireUnloader %s"
                -- , tostring(unloader and unloader:getName())
                -- )
                self:fireUnloader(unloader, harvester)
            end
        end
    end

    for _, harvester in pairs(self.harvesters) do
        if harvester ~= nil and g_currentMission.nodeToObject[harvester.components[1].node] ~= nil and entityExists(harvester.components[1].node) then
            --if self.assignmentDelayTimer:done() then
                if not self:alreadyAssignedUnloader(harvester) then
                    -- ADHarvestManager.debugMsg(harvester, "ADHarvestManager:update doesHarvesterNeedUnloading %s isHarvesterActive %s"
                    -- , tostring(ADHarvestManager.doesHarvesterNeedUnloading(harvester))
                    -- , tostring(ADHarvestManager.isHarvesterActive(harvester))
                    -- )
                    if ADHarvestManager.doesHarvesterNeedUnloading(harvester) or (ADHarvestManager.isHarvesterActive(harvester)) then
                        -- ADHarvestManager.debugMsg(harvester, "ADHarvestManager:update assignUnloaderToHarvester")
                        self:assignUnloaderToHarvester(harvester)
                    end
                else
                    if AutoDrive.getSetting("callSecondUnloader", harvester) then
                        local unloader = self:getAssignedUnloader(harvester)
                        if unloader and unloader.ad and unloader.ad.modes and unloader.ad.modes[AutoDrive.MODE_UNLOAD]:getFollowingUnloader() == nil then
                            local trailers, _ = AutoDrive.getAllUnits(unloader)

                            local fillLevel, _, _, fillFreeCapacity = AutoDrive.getAllFillLevels(trailers)
                            local maxCapacity = fillLevel + fillFreeCapacity

                            if fillLevel >= (maxCapacity * AutoDrive.getSetting("preCallLevel", harvester)) then
                                local closestUnloader = self:getClosestIdleUnloader(harvester)
                                if closestUnloader ~= nil then
                                    closestUnloader.ad.modes[AutoDrive.MODE_UNLOAD]:driveToUnloader(unloader)
                                end
                            end
                        end
                    end
                end
            --end
        else
            -- ADHarvestManager.debugMsg(harvester, "ADHarvestManager:update remove from harvesters")
            table.removeValue(self.harvesters, harvester)
        end
    end

    local function updateMovementTimers(harvester)
        if harvester == nil or harvester.ad == nil or harvester.ad.noMovementTimer == nil or harvester.lastSpeedReal == nil then
            return
        end

        local movingDirection = harvester.movingDirection
        if harvester.ad.isReverseAttached then
            movingDirection = -harvester.movingDirection
        end
        harvester.ad.noMovementTimer:timer((harvester.lastSpeedReal <= 0.0004), 3000, dt)
        if ((harvester.lastSpeedReal * movingDirection) >= 0.0004) then
            harvester.ad.driveForwardTimer:timer(true, 4000, dt)
        else
            harvester.ad.driveForwardTimer:timer(false)
        end
    end
    
    for _, harvester in pairs(self.harvesters) do
        updateMovementTimers(harvester)
    end
    for _, harvester in pairs(self.idleHarvesters) do
        updateMovementTimers(harvester)
    end



    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_COMBINEINFO) then
        local debug = {}
        debug.harvesters = {}
        for _, harvester in pairs(self.harvesters) do
            local infoTable = {}
            infoTable.name = harvester:getName()
            if self:getAssignedUnloader(harvester) ~= nil then
                infoTable.unloader = self:getAssignedUnloader(harvester):getName()
            end
            if self:getAssignedUnloader(harvester) ~= nil and self:getAssignedUnloader(harvester).ad.modes[AutoDrive.MODE_UNLOAD]:getFollowingUnloader() ~= nil then
                infoTable.follower = self:getAssignedUnloader(harvester).ad.modes[AutoDrive.MODE_UNLOAD]:getFollowingUnloader():getName()
            end
            table.insert(debug.harvesters, infoTable)
        end
        debug.idleUnloaders = {}
        for _, idleUnloader in pairs(self.idleUnloaders) do
            local infoTable = {}
            infoTable.name = idleUnloader:getName()
            if idleUnloader.ad.modes[AutoDrive.MODE_UNLOAD].combine ~= nil then
                infoTable.unloader = idleUnloader.ad.modes[AutoDrive.MODE_UNLOAD].combine:getName()
            end
            if idleUnloader.ad.modes[AutoDrive.MODE_UNLOAD]:getFollowingUnloader() ~= nil then
                infoTable.follower = idleUnloader.ad.modes[AutoDrive.MODE_UNLOAD]:getFollowingUnloader():getName()
            end
            table.insert(debug.idleUnloaders, infoTable)
        end
        debug.activeUnloaders = {}
        for _, activeUnloader in pairs(self.activeUnloaders) do
            local infoTable = {}
            infoTable.name = activeUnloader:getName()
            if activeUnloader.ad.modes[AutoDrive.MODE_UNLOAD].combine ~= nil then
                infoTable.unloader = activeUnloader.ad.modes[AutoDrive.MODE_UNLOAD].combine:getName()
            end
            if activeUnloader.ad.modes[AutoDrive.MODE_UNLOAD]:getFollowingUnloader() ~= nil then
                infoTable.follower = activeUnloader.ad.modes[AutoDrive.MODE_UNLOAD]:getFollowingUnloader():getName()
            end
            table.insert(debug.activeUnloaders, infoTable)
        end
        debug.delayTimer = self.assignmentDelayTimer.elapsedTime
        AutoDrive.renderTable(0.65, 0.6, 0.014, debug, 3)
    end
end

function ADHarvestManager.doesHarvesterNeedUnloading(harvester, ignorePipe)
    local ret = false

    local pipeOut = AutoDrive.isPipeOut(harvester)
    ret = (
            (
                (pipeOut)
            )
            and 
            harvester.ad.noMovementTimer.elapsedTime > 5000
        )
    -- ADHarvestManager.debugMsg(harvester, "ADHarvestManager.doesHarvesterNeedUnloading cpIsCalling %s pipeOut %s noMovementTimer %s"
    -- , tostring(cpIsCalling)
    -- , tostring(pipeOut)
    -- , tostring(harvester.ad.noMovementTimer.elapsedTime > 5000)
    -- )
    return ret
end

function ADHarvestManager.isHarvesterActive(harvester)
    if harvester.ad.isChopper then
        return true
    else
        local fillLevel, fillCapacity, _, fillFreeCapacity = AutoDrive.getObjectFillLevels(harvester)
        local fillPercent = fillCapacity > 0 and (fillLevel / fillCapacity) or 0
        local reachedPreCallLevel = fillPercent >= AutoDrive.getSetting("preCallLevel", harvester)
        local isAlmostFull = fillPercent >= ADHarvestManager.MAX_PREDRIVE_LEVEL or fillFreeCapacity < 0.1

        -- Only chase the rear on low fill levels of the combine. This should prevent getting into unneccessarily tight spots for the final approach to the pipe.
        -- Also for small fields, there is often no purpose in chasing so far behind the combine as it will already start a turn soon
        local allowedToChase = true
        if not harvester.ad.isSugarcaneHarvester then
            local chasingRear = false
            local pipeSide = AutoDrive.getPipeSide(harvester)
            if pipeSide == AutoDrive.CHASEPOS_LEFT then
                local leftBlocked = harvester.ad.sensors.leftSensor:pollInfo()
                local leftFrontBlocked = harvester.ad.sensors.leftFrontSensor:pollInfo()
                chasingRear = leftBlocked or leftFrontBlocked
            else
                local rightBlocked = harvester.ad.sensors.rightSensor:pollInfo()
                local rightBlockedBlocked = harvester.ad.sensors.rightFrontSensor:pollInfo()
                chasingRear = rightBlocked or rightBlockedBlocked
            end

            if fillPercent > 0.9 or (fillPercent > 0.7 and chasingRear) then
                allowedToChase = false
            end
        end

        local manuallyControlled = AutoDrive:getIsEntered(harvester:getRootVehicle()) and (not (harvester.getIsAIActive ~= nil and harvester:getIsAIActive()))

        if manuallyControlled then
            return  AutoDrive.isPipeOut(harvester)
        end
        -- ADHarvestManager.debugMsg(harvester, "ADHarvestManager.isHarvesterActive reachedPreCallLevel %s isAlmostFull %s allowedToChase %s", reachedPreCallLevel, isAlmostFull, allowedToChase)

        return reachedPreCallLevel and (not isAlmostFull) and allowedToChase
    end

    return false
end

function ADHarvestManager:assignUnloaderToHarvester(harvester)
    local closestUnloader = self:getClosestIdleUnloader(harvester)
    if closestUnloader ~= nil then
        if harvester.ad ~= nil and harvester.ad.isChopper then
            -- forage harvester: this unloader stays its own until it is full (chopper only)
            harvester.ad.stickyUnloader = closestUnloader
        end
        ADHarvestManager.debugMsg(closestUnloader, "ADHarvestManager:assignUnloaderToHarvester ")
        closestUnloader.ad.modes[AutoDrive.MODE_UNLOAD]:assignToHarvester(harvester)
        table.insert(self.activeUnloaders, closestUnloader)
        table.removeValue(self.idleUnloaders, closestUnloader)
    end
end

function ADHarvestManager:alreadyAssignedUnloader(harvester)
    for _, unloader in pairs(self.activeUnloaders) do
        if unloader.ad.modes[AutoDrive.MODE_UNLOAD].combine == harvester then
            return true
        end
    end
    return false
end

function ADHarvestManager:getAssignedUnloader(harvester)
    for _, unloader in pairs(self.activeUnloaders) do
        if unloader.ad.modes[AutoDrive.MODE_UNLOAD].combine == harvester then
            return unloader
        end
    end
    return nil
end

-- forage harvester only: the unloader it worked with last, until that one is full and leaves to unload
ADHarvestManager.CHOPPER_DRIVER_STOP_TIME = 300000 -- ms, forage harvester driver stopped longer: unloaders released

function ADHarvestManager.releaseStickyUnloader(vehicle, reason)
    for _, list in ipairs({ADHarvestManager.harvesters or {}, ADHarvestManager.idleHarvesters or {}}) do
        for _, harvester in pairs(list) do
            if harvester ~= nil and harvester.ad ~= nil and harvester.ad.stickyUnloader == vehicle then
                harvester.ad.stickyUnloader = nil
            end
        end
    end
end

ADHarvestManager.QUEUE_SAME_TIME = 10000 -- ms: unloaders registered this close together are called by their place in the line
ADHarvestManager.QUEUE_LINE_ANGLE = 30 -- deg at most between the headings of two unloaders in the same line
ADHarvestManager.QUEUE_LINE_SIDE = 6 -- m at most aside from the other one's axis
ADHarvestManager.QUEUE_LINE_AHEAD = 2 -- m ahead at least

-- unloader a stands in front of unloader b in the same line: same heading, on b's axis, ahead of b
function ADHarvestManager.isAheadInLine(a, b)
    if a == nil or b == nil or a.components == nil or b.components == nil then
        return false
    end
    local ax, _, az = getWorldTranslation(a.components[1].node)
    local bx, _, bz = getWorldTranslation(b.components[1].node)
    local ahx, _, ahz = localDirectionToWorld(a.components[1].node, 0, 0, 1)
    local bhx, _, bhz = localDirectionToWorld(b.components[1].node, 0, 0, 1)
    local la, lb = math.sqrt(ahx * ahx + ahz * ahz), math.sqrt(bhx * bhx + bhz * bhz)
    if la < 0.01 or lb < 0.01 then
        return false
    end
    ahx, ahz, bhx, bhz = ahx / la, ahz / la, bhx / lb, bhz / lb
    if ahx * bhx + ahz * bhz < math.cos(math.rad(ADHarvestManager.QUEUE_LINE_ANGLE)) then
        return false
    end
    local dx, dz = ax - bx, az - bz
    local along = dx * bhx + dz * bhz
    local aside = math.abs(dx * bhz - dz * bhx)
    return along >= ADHarvestManager.QUEUE_LINE_AHEAD and aside <= ADHarvestManager.QUEUE_LINE_SIDE
end

-- combine harvester: the closest idle unloader (former behaviour).
-- Forage harvester: its own unloader while not full, else the first in the queue (order of arrival, then distance):
-- in a narrow cut strip the trailers cannot pass each other, the one that came first must be called first
function ADHarvestManager:getClosestIdleUnloader(harvester)
    local chopper = harvester ~= nil and harvester.ad ~= nil and harvester.ad.isChopper == true
    local sticky = chopper and harvester.ad.stickyUnloader or nil
    if sticky ~= nil and table.contains(self.idleUnloaders, sticky) then
        local handedOver = sticky.ad.skipHarvester == harvester and sticky.ad.skipHarvesterUntil ~= nil and g_time < sticky.ad.skipHarvesterUntil
        if not handedOver then
            return sticky
        end
    end
    local closestUnloader = nil
    local closestDistance = math.huge
    local closestTime = math.huge
    local matching = 0
    for _, unloader in pairs(self.idleUnloaders) do
        -- sort by distance to combine first
        local distance = AutoDrive.getDistanceBetween(unloader, harvester)
        --local distanceMatch = distance <= ADHarvestManager.MAX_SEARCH_RANGE and AutoDrive.getSetting("findDriver")
        local targetsMatch = unloader.ad.stateModule:getFirstMarker() == harvester.ad.stateModule:getFirstMarker()
        -- an unloader that gave this harvester up after reversing twice is not called back by it for a while
        local handedOver = unloader.ad.skipHarvester == harvester and unloader.ad.skipHarvesterUntil ~= nil and g_time < unloader.ad.skipHarvesterUntil
        if targetsMatch and not handedOver then --if distanceMatch or targetsMatch then
            matching = matching + 1
            -- order of arrival for both harvester types (unloaders queue in the field and beside the road)
            local queueTime = unloader.ad.unloaderQueueTime or math.huge
            if closestUnloader == nil or queueTime < closestTime or (queueTime == closestTime and distance < closestDistance) then
                closestUnloader = unloader
                closestDistance = distance
                closestTime = queueTime
            end
        end
    end
    -- unloaders registered within QUEUE_SAME_TIME of each other (game load: an unloader was
    -- called, the 728 stood in front of it in the line) go by their place in the line, the one in front first
    if chopper and closestUnloader ~= nil and matching > 1 then
        local first, firstTime = closestUnloader, closestTime
        local moved = 0
        local changed = true
        while changed and moved < matching do
            changed = false
            for _, unloader in pairs(self.idleUnloaders) do
                local queueTime = unloader.ad.unloaderQueueTime or math.huge
                local handedOver = unloader.ad.skipHarvester == harvester and unloader.ad.skipHarvesterUntil ~= nil and g_time < unloader.ad.skipHarvesterUntil
                if unloader ~= closestUnloader and not handedOver and unloader.ad.stateModule:getFirstMarker() == harvester.ad.stateModule:getFirstMarker()
                    and math.abs(queueTime - closestTime) <= ADHarvestManager.QUEUE_SAME_TIME and ADHarvestManager.isAheadInLine(unloader, closestUnloader) then
                    closestUnloader = unloader
                    closestDistance = AutoDrive.getDistanceBetween(unloader, harvester)
                    closestTime = queueTime
                    moved = moved + 1
                    changed = true
                    break
                end
            end
        end
    end
    return closestUnloader
end

function ADHarvestManager:hasHarvesterAvailableUnloader(harvester)
    local rootHarvester = harvester and harvester.getRootVehicle and harvester:getRootVehicle()
    if rootHarvester and rootHarvester.ad and rootHarvester.ad.stateModule then
        for _, unloader in pairs(self.idleUnloaders) do
            local targetsMatch = unloader.ad.stateModule:getFirstMarker() == rootHarvester.ad.stateModule:getFirstMarker()
            if targetsMatch then
                return true
            end
        end
        for _, unloader in pairs(self.activeUnloaders) do
            if unloader.ad.modes[AutoDrive.MODE_UNLOAD].combine == rootHarvester then
                return true
            end
        end
    end
    return false
end

function ADHarvestManager:adHasHarvesterAvailableUnloader()
    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "ADHarvestManager:adHasHarvesterAvailableUnloader hasHarvesterAvailableUnloader %s "
    , tostring(ADHarvestManager:hasHarvesterAvailableUnloader(self))
    )
    return ADHarvestManager:hasHarvesterAvailableUnloader(self)
end

function ADHarvestManager:hasHarvesterPotentialUnloaders(harvester)
    for _, unloader in pairs(self.idleUnloaders) do
        local targetsMatch = unloader.ad.stateModule:getFirstMarker() == harvester.ad.stateModule:getFirstMarker()
        if targetsMatch then
            return true
        end
    end
    for _, unloader in pairs(self.activeUnloaders) do
        local targetsMatch = unloader.ad.stateModule:getFirstMarker() == harvester.ad.stateModule:getFirstMarker()
        if targetsMatch then
            return true
        end
    end
    for _, other in pairs(AutoDrive.getAllVehicles()) do
        if other ~= self.vehicle and other.ad ~= nil and other.ad.stateModule ~= nil and other.ad.stateModule:isActive() and other.ad.stateModule:getFirstMarker() == harvester.ad.stateModule:getFirstMarker() and other.ad.stateModule:getMode() == AutoDrive.MODE_UNLOAD then
            return true
        end
    end
    
    return false
end

function ADHarvestManager:hasVehiclePotentialHarvesters(vehicle)
    for _, harvester in pairs(self.idleHarvesters) do
        local targetsMatch = vehicle.ad.stateModule:getFirstMarker() == harvester.ad.stateModule:getFirstMarker()
        if targetsMatch then
            return true
        end
    end
    for _, harvester in pairs(self.harvesters) do
        local targetsMatch = vehicle.ad.stateModule:getFirstMarker() == harvester.ad.stateModule:getFirstMarker()
        if targetsMatch then
            return true
        end
    end
    return false
end

function ADHarvestManager.getOpenPipePercent(harvester)
	local pipePercent = 1
	local openPipe = false
	local fillLevel = 0
	local capacity = 0
	if harvester ~= nil and harvester.getCurrentDischargeNode ~= nil then
		local dischargeNode = harvester:getCurrentDischargeNode()
		if dischargeNode ~= nil then
			fillLevel = harvester:getFillUnitFillLevel(dischargeNode.fillUnitIndex)
			capacity = harvester:getFillUnitCapacity(dischargeNode.fillUnitIndex)
		end
		if capacity ~= nil and capacity > 0 and AutoDrive.getSetting("preCallLevel", harvester) ~= nil and ADHarvestManager:getAssignedUnloader(harvester) ~= nil and AutoDrive.dynamicChaseDistance then
			pipePercent = AutoDrive.getSetting("preCallLevel", harvester)
			if fillLevel > (pipePercent * capacity) then
				openPipe = true
			end
		end
	end
	return openPipe, pipePercent
end

function ADHarvestManager.debugMsg(vehicle, debugText, ...)
    if ADHarvestManager.debug == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    else
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_COMBINEINFO, debugText, ...)
    end
end
