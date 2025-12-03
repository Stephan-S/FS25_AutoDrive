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
    self:unregisterAsUnloader(vehicle)
    if not table.contains(self.idleUnloaders, vehicle) then
        ADHarvestManager.debugMsg(vehicle, "ADHarvestManager:registerAsUnloader - inserted")
        table.insert(self.idleUnloaders, vehicle)
    end
end

function ADHarvestManager:unregisterAsUnloader(vehicle)
    if vehicle.ad and vehicle.ad.modes ~= nil and vehicle.ad.modes[AutoDrive.MODE_UNLOAD] ~= nil then
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
        if not ((vehicle.getIsAIActive ~= nil and vehicle:getIsAIActive()) or (AutoDrive:getIsEntered(vehicle:getRootVehicle()) and AutoDrive.isPipeOut(vehicle)))
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

function ADHarvestManager:getClosestIdleUnloader(harvester)
    local closestUnloader = nil
    local closestDistance = math.huge
    for _, unloader in pairs(self.idleUnloaders) do
        -- sort by distance to combine first
        local distance = AutoDrive.getDistanceBetween(unloader, harvester)
        --local distanceMatch = distance <= ADHarvestManager.MAX_SEARCH_RANGE and AutoDrive.getSetting("findDriver")
        local targetsMatch = unloader.ad.stateModule:getFirstMarker() == harvester.ad.stateModule:getFirstMarker()
        if targetsMatch then --if distanceMatch or targetsMatch then
            if closestUnloader == nil or distance < closestDistance then
                closestUnloader = unloader
                closestDistance = distance
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
