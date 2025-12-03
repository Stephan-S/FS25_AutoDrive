CombineUnloaderMode = ADInheritsFrom(AbstractMode)
CombineUnloaderMode.debug = false

CombineUnloaderMode.STATE_INIT = {}
CombineUnloaderMode.STATE_WAIT_TO_BE_CALLED = {}
CombineUnloaderMode.STATE_DRIVE_TO_COMBINE = {}
CombineUnloaderMode.STATE_DRIVE_TO_PIPE = {}
CombineUnloaderMode.STATE_LEAVE_CROP = {}
CombineUnloaderMode.STATE_DRIVE_TO_START = {}
CombineUnloaderMode.STATE_DRIVE_TO_UNLOAD = {}
CombineUnloaderMode.STATE_FOLLOW_COMBINE = {}
CombineUnloaderMode.STATE_ACTIVE_UNLOAD_COMBINE = {}
CombineUnloaderMode.STATE_FOLLOW_CURRENT_UNLOADER = {}
CombineUnloaderMode.STATE_EXIT_FIELD = {}
CombineUnloaderMode.STATE_REVERSE_FROM_BAD_LOCATION = {}

CombineUnloaderMode.MAX_COMBINE_FILLLEVEL_CHASING = 101
CombineUnloaderMode.STATIC_X_OFFSET_FROM_HEADER = 0

function CombineUnloaderMode:new(vehicle)
    local o = CombineUnloaderMode:create()
    o.vehicle = vehicle
    o.trailers = nil
    o.trailerCount = 0
    o.tractorTrainLength = 0
    CombineUnloaderMode.reset(o)
    CombineUnloaderMode.setStateNames(o)
    return o
end

function CombineUnloaderMode:reset()
    self.state = self.STATE_INIT
    self.activeTask = nil
    ADHarvestManager:unregisterAsUnloader(self.vehicle)
    self.combine = nil
    self.followingUnloader = nil
    self.breadCrumbs = Queue:new()
    self.lastBreadCrumb = nil
    self.failedPathFinder = 0
    self.trailers, self.trailerCount = AutoDrive.getAllUnits(self.vehicle)
    self.tractorTrainLength = AutoDrive.getTractorTrainLength(self.vehicle, true, false)
    self.vehicle.ad.trailerModule:reset()
    AutoDrive.getAllDischargeableUnits(self.vehicle, true) -- force initialisation
end

function CombineUnloaderMode:start(user)
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:start start self.state %s", tostring(self:getStateName()))
    if not self.vehicle.ad.stateModule:isActive() then
        self.vehicle:startAutoDrive()
    end

    if self.vehicle.ad.stateModule:getFirstMarker() == nil or self.vehicle.ad.stateModule:getSecondMarker() == nil then
        return
    end

    self:reset()

    self.activeTask = self:getNextTask()
    if self.activeTask ~= nil then
        self.vehicle.ad.taskModule:addTask(self.activeTask)
    end
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:start end self.state %s", tostring(self:getStateName()))
end

function CombineUnloaderMode:monitorTasks(dt)
    if self.combine ~= nil and (self.state == self.STATE_DRIVE_TO_START or self.state == self.STATE_DRIVE_TO_UNLOAD or self.state == self.STATE_EXIT_FIELD) then
        if AutoDrive.getDistanceBetween(self.vehicle, self.combine) > 25 then
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:monitorTasks -> unregisterAsUnloader")
            ADHarvestManager:unregisterAsUnloader(self.vehicle)
            self.followingUnloader = nil
            self.combine = nil
        end
    end
    if self.combine ~= nil and self.state == self.STATE_ACTIVE_UNLOAD_COMBINE then
        self:leaveBreadCrumbs()
    end
    --We are stuck
    if self.failedPathFinder >= 5 or ((self.vehicle.ad.specialDrivingModule:shouldStopMotor() or self.vehicle.ad.specialDrivingModule.stoppedTimer:done()) and self.vehicle.ad.specialDrivingModule.isBlocked) then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:monitorTasks() - detected stuck vehicle - try reversing out of it now")
        self.vehicle.ad.specialDrivingModule:releaseVehicle()
        self.vehicle.ad.taskModule:abortAllTasks()
        self.activeTask = ReverseFromBadLocationTask:new(self.vehicle)
        self.state = self.STATE_REVERSE_FROM_BAD_LOCATION
        self.vehicle.ad.taskModule:addTask(self.activeTask)
        self.failedPathFinder = 0
    end

    if self.vehicle.lastSpeedReal > 0.0013 then
        self.failedPathFinder = 0
    end
end

function CombineUnloaderMode:notifyAboutFailedPathfinder()
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:notifyAboutFailedPathfinder self.failedPathFinder %s ADGraphManager:getDistanceFromNetwork(self.vehicle) > 20 %s", tostring(self.failedPathFinder), tostring(ADGraphManager:getDistanceFromNetwork(self.vehicle) > 20))
    --print("CombineUnloaderMode:notifyAboutFailedPathfinder() - blocked: " .. tostring(self.vehicle.ad.pathFinderModule.completelyBlocked) .. " distance: " .. ADGraphManager:getDistanceFromNetwork(self.vehicle))
    if self.vehicle.ad.pathFinderModule.completelyBlocked and ADGraphManager:getDistanceFromNetwork(self.vehicle) > 20 then
        self.failedPathFinder = self.failedPathFinder + 1
    --print("Increased Failed pathfinder count to: " .. self.failedPathFinder)
    end
end

function CombineUnloaderMode:leaveBreadCrumbs()
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local rx, _, rz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)

    if self.lastBreadCrumb == nil then
        self.lastBreadCrumb = {x = x, y = y, z = z, dirX = rx, dirZ = rz}
        self.breadCrumbs:Enqueue(self.lastBreadCrumb)
    else
        if (self.vehicle.lastSpeedReal * self.vehicle.movingDirection) > 0 then
            local _, _, diffZ = AutoDrive.worldToLocal(self.vehicle, self.lastBreadCrumb.x, self.lastBreadCrumb.y, self.lastBreadCrumb.z)
            local vec1 = {x = x - self.lastBreadCrumb.x, z = z - self.lastBreadCrumb.z}
            local angleToNewPoint = AutoDrive.angleBetween({x = self.lastBreadCrumb.dirX, z = self.lastBreadCrumb.dirZ}, vec1)
            local minDistance = 2.5
            if math.abs(angleToNewPoint) > 40 then
                minDistance = 15
            end
            if diffZ < -1 and MathUtil.vector2Length(x - self.lastBreadCrumb.x, z - self.lastBreadCrumb.z) > minDistance and math.abs(angleToNewPoint) < 90 then
                self.lastBreadCrumb = {x = x, y = y, z = z, dirX = vec1.x, dirZ = vec1.z}
                self.breadCrumbs:Enqueue(self.lastBreadCrumb)
            end
        end
    end
end

function CombineUnloaderMode:getBreadCrumbs()
    return self.breadCrumbs
end

function CombineUnloaderMode:promoteFollowingUnloader(combine)
    self.combine = combine
    if self.vehicle.ad.taskModule.activeTask ~= nil and self.vehicle.ad.taskModule.activeTask.signalPromotion ~= nil then
        self.vehicle.ad.taskModule.activeTask:signalPromotion()
    end
end

function CombineUnloaderMode:handleFinishedTask()
    -- CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:handleFinishedTask")
    self.vehicle.ad.trailerModule:reset()
    self.lastTask = self.activeTask
    self.activeTask = self:getNextTask()
    if self.activeTask ~= nil then
        self.vehicle.ad.taskModule:addTask(self.activeTask)
    end
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:handleFinishedTask self.state %s", tostring(self:getStateName()))
end

function CombineUnloaderMode:stop()
end

function CombineUnloaderMode:continue()
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local point = nil
    local distanceToStart = 0
    if
        self.vehicle.ad ~= nil and ADGraphManager.getWayPointById ~= nil and self.vehicle.ad.stateModule ~= nil and self.vehicle.ad.stateModule.getFirstMarker ~= nil and self.vehicle.ad.stateModule:getFirstMarker() ~= nil and
            self.vehicle.ad.stateModule:getFirstMarker() ~= 0 and
            self.vehicle.ad.stateModule:getFirstMarker().id ~= nil
     then
        point = ADGraphManager:getWayPointById(self.vehicle.ad.stateModule:getFirstMarker().id)
        if point ~= nil then
            distanceToStart = MathUtil.vector2Length(x - point.x, z - point.z)
        end
    end

    if self.state == self.STATE_DRIVE_TO_UNLOAD then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:continue self.STATE_DRIVE_TO_UNLOAD")
        self.activeTask:continue()
    else
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:continue self.state" .. tostring(self:getStateName()))
        self.vehicle.ad.taskModule:abortCurrentTask()

        if AutoDrive.checkIsOnField(x, y, z) and distanceToStart > 30 then
            -- is activated on a field - use ExitFieldTask to leave field according to setting
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:continue ExitFieldTask...")
            self.activeTask = ExitFieldTask:new(self.vehicle)
            self.state = self.STATE_EXIT_FIELD
        else
            if (AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYDELIVER or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders") then
                local nextTarget = ADMultipleTargetsManager:getNextTarget(self.vehicle, false)
                if nextTarget ~= nil then
                    self.vehicle.ad.stateModule:setSecondMarker(nextTarget)
                end
            end
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:continue UnloadAtDestinationTask...")
            self.activeTask = UnloadAtDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getSecondMarker().id)
            self.state = self.STATE_DRIVE_TO_UNLOAD
        end

        ADHarvestManager:unregisterAsUnloader(self.vehicle)
        self.followingUnloader = nil
        self.combine = nil
        self.vehicle.ad.taskModule:addTask(self.activeTask)
    end
end

function CombineUnloaderMode:getNextTask()
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask start self.state %s", tostring(self:getStateName()))
    local nextTask

    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local point = nil
    local _, _, filledToUnload, _ = AutoDrive.getAllFillLevels(self.trailers)

    if self.state == self.STATE_INIT then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask STATE_INIT filledToUnload %s", tostring(filledToUnload))
        if filledToUnload then -- fill level above setting unload level
            if (AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYDELIVER or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders") then
                local nextTarget = ADMultipleTargetsManager:getNextTarget(self.vehicle, false)
                if nextTarget ~= nil then
                    self.vehicle.ad.stateModule:setSecondMarker(nextTarget)
                end
            end

            nextTask = self:getTaskAfterUnload(filledToUnload)

            ADHarvestManager:unregisterAsUnloader(self.vehicle)
            self.followingUnloader = nil
            self.combine = nil
        else
            if not AutoDrive.checkIsOnField(x, y, z) then
                nextTask = DriveToDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getFirstMarker().id)
                self.state = self.STATE_DRIVE_TO_START
            else
                self:setToWaitForCall()
            end
        end
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_INIT end self.state %s", tostring(self:getStateName()))
    elseif self.state == self.STATE_DRIVE_TO_COMBINE then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_DRIVE_TO_COMBINE")
        -- we finished the precall to combine route
        -- check if we should wait / pull up to combines pipe
        nextTask = FollowCombineTask:new(self.vehicle, self.combine)
        self.state = self.STATE_ACTIVE_UNLOAD_COMBINE
        self.breadCrumbs = Queue:new()
        self.lastBreadCrumb = nil
    elseif self.state == self.STATE_DRIVE_TO_PIPE or self.state == self.STATE_REVERSE_FROM_BAD_LOCATION then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_DRIVE_TO_PIPE | STATE_REVERSE_FROM_BAD_LOCATION")
        --Drive to pipe can be finished when combine is emptied or when vehicle has reached 'old' pipe position and should switch to active mode
        nextTask = self:getTaskAfterUnload(filledToUnload)
    elseif self.state == self.STATE_LEAVE_CROP then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_LEAVE_CROP")
        self:setToWaitForCall()
    elseif self.state == self.STATE_DRIVE_TO_UNLOAD then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_DRIVE_TO_UNLOAD")
        nextTask = DriveToDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getFirstMarker().id)
        if (AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYDELIVER or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders") then
            local nextTarget = ADMultipleTargetsManager:getNextTarget(self.vehicle, true)
            if nextTarget ~= nil then
                self.vehicle.ad.stateModule:setSecondMarker(nextTarget)
            end
        end
        self.state = self.STATE_DRIVE_TO_START
    elseif self.state == self.STATE_DRIVE_TO_START then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_DRIVE_TO_START")
        self:setToWaitForCall()
    elseif self.state == self.STATE_ACTIVE_UNLOAD_COMBINE then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_ACTIVE_UNLOAD_COMBINE")
        if filledToUnload then
            nextTask = self:getTaskAfterUnload(filledToUnload)
        else
            nextTask = HandleHarvesterTurnTask:new(self.vehicle, self.combine)
            self.state = self.STATE_DRIVE_TO_COMBINE
        end
    elseif self.state == self.STATE_FOLLOW_CURRENT_UNLOADER then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_FOLLOW_CURRENT_UNLOADER")
        if self.targetUnloader ~= nil then
            self.targetUnloader.ad.modes[AutoDrive.MODE_UNLOAD]:unregisterFollowingUnloader()
        end
        nextTask = self:getTaskAfterUnload(filledToUnload)
    elseif self.state == self.STATE_EXIT_FIELD then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_EXIT_FIELD")
        if (AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYDELIVER or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders") then
            local nextTarget = ADMultipleTargetsManager:getNextTarget(self.vehicle, false)
            if nextTarget ~= nil then
                self.vehicle.ad.stateModule:setSecondMarker(nextTarget)
            end
        end
        nextTask = UnloadAtDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getSecondMarker().id)
        self.state = self.STATE_DRIVE_TO_UNLOAD
    end

    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask end self.state %s", tostring(self:getStateName()))
    return nextTask
end

function CombineUnloaderMode:setToWaitForCall(keepCombine)
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:setToWaitForCall start self.state %s keepCombine %s", tostring(self:getStateName()), tostring(keepCombine))
    -- We just have to wait to be wait to be called (again)
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local point = nil
    local distanceToStart = 0
    if
        self.vehicle.ad ~= nil and ADGraphManager.getWayPointById ~= nil and self.vehicle.ad.stateModule ~= nil and self.vehicle.ad.stateModule.getFirstMarker ~= nil
        and self.vehicle.ad.stateModule:getFirstMarker() ~= nil and self.vehicle.ad.stateModule:getFirstMarker() ~= 0 and self.vehicle.ad.stateModule:getFirstMarker().id ~= nil
    then
        point = ADGraphManager:getWayPointById(self.vehicle.ad.stateModule:getFirstMarker().id)
        if point ~= nil then
            distanceToStart = MathUtil.vector2Length(x - point.x, z - point.z)
        end
    end

    local _, _, filledToUnload, _ = AutoDrive.getAllFillLevels(self.trailers)
    if filledToUnload then
        if AutoDrive.checkIsOnField(x, y, z) and distanceToStart > 30 then
            -- is activated on a field - use ExitFieldTask to leave field according to setting
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:setToWaitForCall ExitFieldTask...")
            self.vehicle.ad.taskModule:addTask(ExitFieldTask:new(self.vehicle))
            self.state = self.STATE_EXIT_FIELD
        else
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:setToWaitForCall UnloadAtDestinationTask...")
            self.vehicle.ad.taskModule:addTask(UnloadAtDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getSecondMarker().id))
            self.state = self.STATE_DRIVE_TO_UNLOAD
        end
    else
	    self.state = self.STATE_WAIT_TO_BE_CALLED
	    self.vehicle.ad.taskModule:addTask(WaitForCallTask:new(self.vehicle))
	    if self.combine ~= nil and self.combine.ad ~= nil and (keepCombine == nil or keepCombine ~= true) then
	        self.combine = nil
	    end
    end
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:setToWaitForCall end self.state %s self.combine %s", tostring(self:getStateName()), tostring(self.combine))
end

function CombineUnloaderMode:assignToHarvester(harvester)
    if self.state == self.STATE_WAIT_TO_BE_CALLED then

        self.vehicle.ad.taskModule:abortCurrentTask()
        self.combine = harvester
        self.combineRootVehicle = self.combine:getRootVehicle()
        AutoDrive.validateCachedPipeData(harvester)

        -- if combine has extended pipe, aim for that. Otherwise DriveToVehicle and choose from there
        if AutoDrive.isPipeOut(self.combine) then

            local cfillLevel, cfillCapacity, _, cleftCapacity = AutoDrive.getObjectFillLevels(self.combine)
            local cFillRatio = cfillCapacity > 0 and cfillLevel / cfillCapacity or 0

            if (self.combine.ad.isHarvester) 
                and (self.combine.ad.noMovementTimer.elapsedTime > 500 or cleftCapacity < 0.1 or cFillRatio > 0.945)
            then
                -- default unloading - no movement
                self.state = self.STATE_DRIVE_TO_PIPE
                self.vehicle.ad.taskModule:addTask(EmptyHarvesterTask:new(self.vehicle, self.combine))
            else
                -- Probably active unloading for choppers and moving combines
                self.state = self.STATE_DRIVE_TO_COMBINE
                self.vehicle.ad.taskModule:addTask(CatchCombinePipeTask:new(self.vehicle, self.combine))
            end
        else
            self.state = self.STATE_DRIVE_TO_COMBINE
            self.vehicle.ad.taskModule:addTask(CatchCombinePipeTask:new(self.vehicle, self.combine))
        end
    end
end

function CombineUnloaderMode:driveToUnloader(unloader)
    if self.state == self.STATE_WAIT_TO_BE_CALLED then
        self.vehicle.ad.taskModule:abortCurrentTask()
        self.vehicle.ad.taskModule:addTask(DriveToVehicleTask:new(self.vehicle, unloader))
        unloader.ad.modes[AutoDrive.MODE_UNLOAD]:registerFollowingUnloader(self.vehicle)
        self.targetUnloader = unloader
        self.state = self.STATE_FOLLOW_CURRENT_UNLOADER
    end
end

function CombineUnloaderMode:getTaskAfterUnload(filledToUnload)
    local nextTask
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getTaskAfterUnload start filledToUnload %s", tostring(filledToUnload))

    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local point = nil
    local distanceToStart = 0
    if
        self.vehicle.ad ~= nil and ADGraphManager.getWayPointById ~= nil and self.vehicle.ad.stateModule ~= nil and self.vehicle.ad.stateModule.getFirstMarker ~= nil and self.vehicle.ad.stateModule:getFirstMarker() ~= nil and
            self.vehicle.ad.stateModule:getFirstMarker() ~= 0 and
            self.vehicle.ad.stateModule:getFirstMarker().id ~= nil
     then
        point = ADGraphManager:getWayPointById(self.vehicle.ad.stateModule:getFirstMarker().id)
        if point ~= nil then
            distanceToStart = MathUtil.vector2Length(x - point.x, z - point.z)
        end
    end

    if filledToUnload then
        --ADHarvestManager:unregisterAsUnloader(self.vehicle)
        --self.followingUnloader = nil
        --self.combine = nil
        if AutoDrive.checkIsOnField(x, y, z) and distanceToStart > 30 then
            -- is activated on a field - use ExitFieldTask to leave field according to setting
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getTaskAfterUnload ExitFieldTask...")
            nextTask = ExitFieldTask:new(self.vehicle)
            self.state = self.STATE_EXIT_FIELD
        else
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getTaskAfterUnload UnloadAtDestinationTask...")
            nextTask = UnloadAtDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getSecondMarker().id)
            self.state = self.STATE_DRIVE_TO_UNLOAD
        end
    else
        -- Should we park in the field?
        if (self.combine and self.combine.ad.isChopper) or (AutoDrive.getSetting("parkInField", self.vehicle) or (self.lastTask ~= nil and self.lastTask.stayOnField)) then
            -- If we are in fruit, we should clear it
            if AutoDrive.isVehicleOrTrailerInCrop(self.vehicle, true) then
                CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getTaskAfterUnload ClearCropTask...")
                nextTask = ClearCropTask:new(self.vehicle, self.combine)
                self.state = self.STATE_LEAVE_CROP
            else
                CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getTaskAfterUnload setToWaitForCall")
                self:setToWaitForCall()
            end
        else
            ADHarvestManager:unregisterAsUnloader(self.vehicle)
            nextTask = DriveToDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getFirstMarker().id)
            self.state = self.STATE_DRIVE_TO_START
        end
    end
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getTaskAfterUnload end")
    return nextTask
end

function CombineUnloaderMode:shouldLoadOnTrigger()
    return false
end

function CombineUnloaderMode:shouldUnloadAtTrigger()
    return self.state == self.STATE_DRIVE_TO_UNLOAD
end

function CombineUnloaderMode:getUnloaderOnSide()
    local vehicleX, vehicleY, vehicleZ = getWorldTranslation(self.vehicle.components[1].node)
    local diffX, _, diffZ = AutoDrive.worldToLocal(self.combine, vehicleX, vehicleY, vehicleZ, self.combine.ad.ADRootNode)
    local leftright = AutoDrive.CHASEPOS_UNKNOWN
    local frontback = AutoDrive.CHASEPOS_FRONT
    -- If we're not clearly on one side or the other of the combine, we don't
    -- give a clear answer
    if math.abs(diffX) > self.combine.size.width / 2 then
        leftright = AutoDrive.sign(diffX)
    end

    local maxZ = self.tractorTrainLength
    if diffZ < maxZ then
        frontback = AutoDrive.CHASEPOS_REAR
    end

    return leftright, frontback
end

function CombineUnloaderMode:isUnloaderOnCorrectSide(chaseSide)
    local sideIndex = chaseSide

    if sideIndex == nil and self.chasePosIndex == nil then
        return false
    elseif sideIndex == nil then
        sideIndex = self.chasePosIndex
    end

    local leftRight, frontBack = self:getUnloaderOnSide()

    if (leftRight == sideIndex and frontBack == AutoDrive.CHASEPOS_REAR) or (leftRight == AutoDrive.CHASEPOS_UNKNOWN and frontBack == AutoDrive.CHASEPOS_REAR) or frontBack == sideIndex then
        return true
    else
        return false
    end
end

function CombineUnloaderMode:getPipeSlopeCorrection2()
    local dischargeX, dichargeY, dischargeZ = getWorldTranslation(AutoDrive.getDischargeNode(self.combine))
    local diffX, diffY, _ = AutoDrive.worldToLocal(self.combine, dischargeX, dichargeY, dischargeZ, self.combine.ad.ADRootNode)
    if math.abs(diffX) < self.combine.size.width / 2 then
        -- Some pipes curl up so tight they cause a collisions.
        -- We just don't try to correct in this case.
        return 0
    end

    local heightUnderPipe = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, dischargeX, dichargeY, dischargeZ)
    -- It would be nice if we could use the discharge node direction and the terrain under the harveser,
    -- but discharge node rotations are untrustworthy.
    local wux, wuy, wuz = getTerrainNormalAtWorldPos(g_currentMission.terrainRootNode, dischargeX, dichargeY - heightUnderPipe, dischargeZ)
    local ux, uy, uz = AutoDrive.worldDirectionToLocal(self.combine, wux, wuy, wuz, self.combine.ad.ADRootNode)

    -- This is backwards from the usual order of these variables, but I need deviation from 0 and pi, not
    -- pi/2 and 3pi/4, so we adjust the coordinate system
    local theta = math.atan(ux / uy)

    local pipePosition = diffX * math.cos(theta) + diffY * math.sin(theta)
    local currentElevationCorrection = pipePosition - diffX

    if math.abs(currentElevationCorrection) > 1 then
        -- Assume something has gone very wrong if the correction gets too large.
        return 0
    end

    return currentElevationCorrection
end

function CombineUnloaderMode:getPipeSlopeCorrection1()
    self.combineX, self.combineY, self.combineZ = getWorldTranslation(self.combineNode)
    local nodeX, nodeY, nodeZ = getWorldTranslation(AutoDrive.getDischargeNode(self.combine))
    local heightUnderCombine = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, self.combineX, self.combineY, self.combineZ)
    local heightUnderPipe = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, nodeX, nodeY, nodeZ)
    -- want this to be negative if the ground is lower under the pipe
    local dh = heightUnderPipe - heightUnderCombine
    local hyp = MathUtil.vector3Length(self.combineX - nodeX, heightUnderCombine - heightUnderPipe, self.combineZ - nodeZ)
    local run = math.sqrt(hyp * hyp - dh * dh)
    local elevationCorrection = (hyp + (nodeY - heightUnderPipe) * (dh / hyp)) - run
    return elevationCorrection * self.pipeSide
end

function CombineUnloaderMode:getPipeSlopeCorrection()
    return self:getPipeSlopeCorrection2()
end

function CombineUnloaderMode:getSideChaseOffsetX()
    -- NB: We cannot apply slope correction until after we have chosen which side
    -- we are chasing on! This function only finds the base X offset "to the left".
    -- Slope and side correction MUST be applied in CombineUnloaderMode:getPipeChasePosition
    -- AFTER determining the chase side. Or this function needs to be rewritten.
    local pipeOffset = AutoDrive.getSetting("pipeOffset", self.vehicle)
    local unloaderWidest = self.vehicle.size.width
    local headerExtra = math.max((AutoDrive.getFrontToolWidth(self.combine) - self.combine.size.width) / 2, 0)

    local sideChaseTermPipeIn = self.combine.size.width / 2 + unloaderWidest / 2 + headerExtra
    local sideChaseTermPipeOut = AutoDrive.getPipeLength(self.combine)
    -- Some combines fold up their pipe so tight that targeting it could cause a collision.
    -- So, choose the max between the two to avoid a collison
    local sideChaseTermX = math.max(sideChaseTermPipeIn, sideChaseTermPipeOut)

    if self.combine.ad.isSugarcaneHarvester then
        -- check for SugarcaneHarvester has to be first, as it also is IsBufferCombine!
        sideChaseTermX = AutoDrive.getPipeLength(self.combine)
    elseif self.combine.ad.isAutoAimingChopper then
        sideChaseTermX = sideChaseTermPipeIn + CombineUnloaderMode.STATIC_X_OFFSET_FROM_HEADER
    elseif (self.combine.ad ~= nil and self.combine.ad.storedPipeLength ~= nil) or AutoDrive.isPipeOut(self.combine) then
        -- If the pipe is extended, though, target it regardless
        sideChaseTermX = sideChaseTermPipeOut
    end

    return sideChaseTermX + pipeOffset
end

function CombineUnloaderMode:getSideChaseOffsetX_new()
    -- NB: We cannot apply slope correction until after we have chosen which side
    -- we are chasing on! This function only finds the base X offset "to the left".
    -- Slope and side correction MUST be applied in CombineUnloaderMode:getPipeChasePosition
    -- AFTER determining the chase side. Or this function needs to be rewritten.
    local pipeOffset = AutoDrive.getSetting("pipeOffset", self.vehicle)
    local unloaderWidest = self.vehicle.size.width
    local headerExtra = math.max((AutoDrive.getFrontToolWidth(self.combine) - self.combine.size.width) / 2, 0)

    local sideChaseTermPipeIn = self.combine.size.width / 2 + unloaderWidest / 2 + headerExtra
    -- Some combines fold up their pipe so tight that targeting it could cause a collision.
    -- So, choose the max between the two to avoid a collison
    local sideChaseTermX = 0

    if self.combine.ad.isSugarcaneHarvester then
        -- check for SugarcaneHarvester has to be first, as it also is IsBufferCombine!
        sideChaseTermX = AutoDrive.getPipeLength(self.combine)
    elseif self.combine.ad.isAutoAimingChopper then
        sideChaseTermX = sideChaseTermPipeIn + CombineUnloaderMode.STATIC_X_OFFSET_FROM_HEADER
    elseif (self.combine.ad ~= nil and self.combine.ad.storedPipeLength ~= nil) or AutoDrive.isPipeOut(self.combine) then
        -- If the pipe is extended, though, target it regardless
        sideChaseTermX = 0
    end

    return sideChaseTermX + pipeOffset
end

function CombineUnloaderMode:getPipeChaseWayPoint(offsetX, offsetZ)
    local wayPoint = {x = 0, y = 0, z = 0}
    local trailer = g_currentMission.nodeToObject[self.targetFillNode]
    if trailer == nil then
        CombineUnloaderMode.debugMsg(self.vehicle, "ERROR: CombineUnloaderMode:getPipeChaseWayPoint trailer == nil")
        trailer = self.vehicle
    end
    local _, trailerDistY, _ = localToLocal(self.targetFillNode, trailer.components[1].node, 0, 0, 0)
    local dischargeNode = AutoDrive.getDischargeNode(self.combine)
    if dischargeNode == nil then
        CombineUnloaderMode.debugMsg(self.vehicle, "ERROR: CombineUnloaderMode:getPipeChaseWayPoint dischargeNode == nil")
        dischargeNode = self.combine.components[1].node
    end
    local combinePipeRootNode = AutoDrive.getPipeRoot(self.combine)
    if combinePipeRootNode == nil then
        CombineUnloaderMode.debugMsg(self.vehicle, "ERROR: CombineUnloaderMode:getPipeChaseWayPoint combinePipeRootNode == nil")
        combinePipeRootNode = self.combine.components[1].node
    end
    local targetX, targetY, targetZ = getWorldTranslation(dischargeNode)
    local combineX, combineY, combineZ = getWorldTranslation(combinePipeRootNode)
    local _, combineDistY, _ = localToLocal(dischargeNode, combinePipeRootNode, 0, 0, 0)
    local diffY = combineDistY - trailerDistY -- distance discharge to fill point
    local pipeSide = self.pipeSide
    if pipeSide == 0 then
        pipeSide = 1
    end
    local pipeOffset = AutoDrive.getSetting("pipeOffset", self.vehicle)
    local worldOffsetX1, worldOffsetY1, worldOffsetZ1 = AutoDrive.localDirectionToWorld(self.combine, 0, -diffY, 0, dischargeNode)
    local targetDistX, _, targetDistZ = AutoDrive.worldToLocal(self.combine, targetX + worldOffsetX1, targetY + worldOffsetY1, targetZ + worldOffsetZ1, combinePipeRootNode)
    local worldOffsetX2, worldOffsetY2, worldOffsetZ2 = AutoDrive.localDirectionToWorld(self.combine, targetDistX + (offsetX or 0) + (pipeSide * pipeOffset), 0, targetDistZ + (offsetZ or 0), combinePipeRootNode)
    wayPoint.x = combineX + worldOffsetX2
    wayPoint.y = combineY + worldOffsetY2
    wayPoint.z = combineZ + worldOffsetZ2
    return wayPoint
end

function CombineUnloaderMode:getDynamicSideChaseOffsetZ()
    local nodeX, nodeY, nodeZ = getWorldTranslation(AutoDrive.getDischargeNode(self.combine))
    local _, _, pipeZOffsetToCombine = AutoDrive.worldToLocal(self.combine, nodeX, nodeY, nodeZ, self.combine.ad.ADRootNode)
    local targetX, targetY, targetZ = getWorldTranslation(self.targetFillNode)

    local _, _, vehicleZOffsetToTarget = AutoDrive.worldToLocal(self.vehicle, targetX, targetY, targetZ)

    local sideChaseTermZ = pipeZOffsetToCombine - vehicleZOffsetToTarget

    return sideChaseTermZ
end

function CombineUnloaderMode:getDynamicSideChaseOffsetZ_fromDischargeNode(planningPhase)
    local targetX, targetY, targetZ = getWorldTranslation(self.targetFillNode)
    
    local _, _, vehicleZOffsetToTarget = AutoDrive.worldToLocal(self.vehicle, targetX, targetY, targetZ)
    local offset = -vehicleZOffsetToTarget
    if planningPhase then
        offset = offset + (self.vehicle.size.length/2)
    end
    return offset
end

function CombineUnloaderMode:getDynamicSideChaseOffsetZ_FS19()
    -- The default maximum will place the front of the unloader at the back of the header
    local pipeZOffsetToCombine = 0
    if AutoDrive.isPipeOut(self.combine) then
        local nodeX, nodeY, nodeZ = getWorldTranslation(AutoDrive.getDischargeNode(self.combine))
        local _, _, diffZ = AutoDrive.worldToLocal(self.combine, nodeX, nodeY, nodeZ, self.combine.ad.ADRootNode)
        pipeZOffsetToCombine = diffZ
    end
    local vehicleX, vehicleY, vehicleZ = getWorldTranslation(self.vehicle.components[1].node)

--[[
    -- AimTargetNode - not used as it not always fit to the fill area of trailers
    if self.targetTrailer.getFillUnitAutoAimTargetNode ~= nil then
        local node = self.targetTrailer:getFillUnitAutoAimTargetNode(1)
        if node ~= nil then
            -- use dynamic fill in trailer if available, to better follow unloading the harvester
            targetNode = node
        end
    end
]]
    local _, _, diffZ = AutoDrive.worldToLocal(self.targetTrailer, vehicleX, vehicleY, vehicleZ)

    -- We gradually move the chase node forward as a function of fill level because it's pretty and
    -- helps the sugarcane harvester. We start at at the front of the trailer. We use an exponential
    -- to increase dwell time towards the front of the trailer, since loads migrate towards the back.

    -- The constant additions should put at precisely at the joint of the vehicle and trailer, then correct for
    -- only moving the midpoint of the tractor
    local constantAdditionsZ = 1 + self.vehicle.size.length / 2 - self.targetTrailer.size.length / 2
    -- We then gradually move back, but don't use the last part of trailer for cosmetic reasons
    local dynamicAdditionsZ = diffZ + pipeZOffsetToCombine
    dynamicAdditionsZ = dynamicAdditionsZ + math.max((self.targetTrailer.size.length * 0.5 - 2) ^ self.targetTrailerFillRatio, 0)
    local sideChaseTermZ = constantAdditionsZ + dynamicAdditionsZ
    return sideChaseTermZ
end

function CombineUnloaderMode:getSideChaseOffsetZ(dynamic)
    if dynamic then
        return self:getDynamicSideChaseOffsetZ()
    else
        return (self.combine.size.length - self.vehicle.size.length + AutoDrive.getFrontToolLength(self.combine)) / 2
    end
end

function CombineUnloaderMode:getRearChaseOffsetX(leftBlocked, rightBlocked)
    local rearChaseOffset = (self.combine.size.width / 2 + self.vehicle.size.width / 2) + 1

    if self.combine.ad.isAutoAimingChopper and not self.combine.ad.isSugarcaneHarvester then
        return 0
    elseif rightBlocked and leftBlocked then
        return 0
    elseif leftBlocked then
        return -rearChaseOffset
    else
        return rearChaseOffset
    end
end

function CombineUnloaderMode:getRearChaseOffsetZ(planningPhase)
    local followDistance = AutoDrive.getSetting("followDistance", self.vehicle)
    local rearChaseOffset = -followDistance - (self.combine.size.length / 2)
    if self.combine.ad.isAutoAimingChopper and not self.combine.ad.isSugarcaneHarvester then
        if planningPhase then
            rearChaseOffset = - (self.combine.size.length / 2)
        end
    else
        -- math.sqrt(2) ensures the trailer could straighten if it was turned 90 degrees, and it makes this point further
        -- back than the pathfinder (straightening) target in PathFinderModule:startPathPlanningToPipe
        -- math.sqrt(2) gives the hypotenuse of an isosceles right trangle with side length equal to the length
        -- of the trailer
        if self.combine.ad.isSugarcaneHarvester then
            rearChaseOffset = -self.combine.size.length / 2 - self.tractorTrainLength * math.sqrt(2)
        else
            local combineSensors = self.combine.ad.sensors or self.combineRootVehicle.ad.sensors
            if self.combine.lastSpeedReal > 0.002 and combineSensors.frontSensorFruit:pollInfo() then
                rearChaseOffset = -10
            else
                --there is no need to be close to the rear of the harvester here. We can make it hard on the pathfinder since we have no strong desire to chase there anyway for normal harvesters
                --Especially when they are CP driven, we have to be prepared for that massive reverse maneuver when the combine is filled and wants to avoid the crop.
                rearChaseOffset = -45
            end
        end
    end

    return rearChaseOffset
end

function CombineUnloaderMode:getPipeChasePosition(planningPhase)
    local chaseNode
    local sideIndex = AutoDrive.CHASEPOS_REAR

    -- in case of rotated attached combine use the trailing vehicle sensors
    local combineSensors = self.combine.ad.sensors or self.combineRootVehicle.ad.sensors

    local leftBlocked = (AutoDrive.getSetting("avoidFruit", self.vehicle) and combineSensors.leftSensorFruit:pollInfo())
    or combineSensors.leftSensor:pollInfo()
    or (AutoDrive.getSetting("followOnlyOnField", self.vehicle) and (not combineSensors.leftSensorField:pollInfo()))

    local leftFrontBlocked = (AutoDrive.getSetting("avoidFruit", self.vehicle) and combineSensors.leftFrontSensorFruit:pollInfo())
    or combineSensors.leftFrontSensor:pollInfo()

    local rightBlocked = (AutoDrive.getSetting("avoidFruit", self.vehicle) and combineSensors.rightSensorFruit:pollInfo())
    or combineSensors.rightSensor:pollInfo()
    or (AutoDrive.getSetting("followOnlyOnField", self.vehicle) and (not combineSensors.rightSensorField:pollInfo()))

    local rightFrontBlocked = (AutoDrive.getSetting("avoidFruit", self.vehicle) and combineSensors.rightFrontSensorFruit:pollInfo())
    or combineSensors.rightFrontSensor:pollInfo()

    rightBlocked = rightBlocked or rightFrontBlocked
    leftBlocked = leftBlocked or leftFrontBlocked

-- TODO: does this make sense, i.e. leftBlocked = leftBlocked or leftFrontBlocked <-> elseif leftFrontBlocked and (not rightFrontBlocked) then
    -- prefer side where front is also free
    if (not leftBlocked) and (not rightBlocked) then
        if (not leftFrontBlocked) and rightFrontBlocked then
            rightBlocked = true
        elseif leftFrontBlocked and (not rightFrontBlocked) then
            leftBlocked = true
        end
    end

    self.pipeSide = AutoDrive.getPipeSide(self.combine)
    self.targetFillUnit, self.targetFillNode = AutoDrive.getNextFreeDischargeableUnit(self.vehicle)

    local sideChaseTermX = self:getSideChaseOffsetX()
    local sideChaseTermZ = self:getSideChaseOffsetZ(AutoDrive.dynamicChaseDistance or self.combine.ad.isHarvester)
    local rearChaseTermZ = self:getRearChaseOffsetZ(planningPhase)

    if self.combine.ad.isChopper then
        -- any chopper
        --CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getPipeChasePosition=IsBufferCombine")
        local leftChasePos = AutoDrive.createWayPointRelativeToVehicle(self.combine, sideChaseTermX + self:getPipeSlopeCorrection(), sideChaseTermZ - 2)
        local rightChasePos = AutoDrive.createWayPointRelativeToVehicle(self.combine, -(sideChaseTermX + self:getPipeSlopeCorrection()), sideChaseTermZ - 2)
        local rearChasePos = AutoDrive.createWayPointRelativeToVehicle(self.combine, 0, rearChaseTermZ)
        local angleToLeftChaseSide = self:getAngleToChasePos(leftChasePos)
        local angleToRearChaseSide = self:getAngleToChasePos(rearChasePos)
        local chaseSideSetting = AutoDrive.getSetting("chaseSide", self.combine)

        -- Default to the side of the harvester the unloader is already on
        -- then check if there is a better side
        chaseNode = leftChasePos
        sideIndex = AutoDrive.CHASEPOS_LEFT
        local unloaderPos, _ = self:getUnloaderOnSide()
        if unloaderPos == AutoDrive.CHASEPOS_RIGHT then
            chaseNode = rightChasePos
            sideIndex = AutoDrive.CHASEPOS_RIGHT
        end
        -- following order is important!
        if self.combine.ad.isFixedPipeChopper then
            -- chopper with fixed unloading
            local sideOffsetZ = self:getDynamicSideChaseOffsetZ_fromDischargeNode(planningPhase)
            chaseNode = self:getPipeChaseWayPoint(0, sideOffsetZ)
            sideIndex = self.pipeSide
        elseif AutoDrive:getIsCPActive(self.combine) and AutoDrive.combineIsTurning(self.combine) then
            chaseNode = rearChasePos
            sideIndex = AutoDrive.CHASEPOS_REAR
        elseif chaseSideSetting ~= AutoDrive.CHASEPOS_AUTO then
            -- consider chaseSide setting
            sideIndex = chaseSideSetting
            if chaseSideSetting == AutoDrive.CHASEPOS_LEFT then
                chaseNode = leftChasePos
            elseif chaseSideSetting == AutoDrive.CHASEPOS_RIGHT then
                chaseNode = rightChasePos
            else
                chaseNode = rearChasePos
            end
        elseif (not leftBlocked) and ((self:isUnloaderOnCorrectSide(AutoDrive.CHASEPOS_LEFT) and angleToLeftChaseSide < angleToRearChaseSide) or planningPhase) then
            chaseNode = leftChasePos
            sideIndex = AutoDrive.CHASEPOS_LEFT
        elseif (not rightBlocked) and (self:isUnloaderOnCorrectSide(AutoDrive.CHASEPOS_RIGHT) or planningPhase) then
            chaseNode = rightChasePos
            sideIndex = AutoDrive.CHASEPOS_RIGHT
        elseif not self.combine.ad.isSugarcaneHarvester then
            chaseNode = rearChasePos
            sideIndex = AutoDrive.CHASEPOS_REAR
        end
    else
        -- harveser with bunker
        --CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getPipeChasePosition:IsNormalCombine")
        local rearChaseTermX = self:getRearChaseOffsetX(leftBlocked, rightBlocked)

        local sideChasePos = AutoDrive.createWayPointRelativeToVehicle(self.combine, self.pipeSide * (sideChaseTermX + self:getPipeSlopeCorrection()), sideChaseTermZ)
        local rearChasePos = AutoDrive.createWayPointRelativeToVehicle(self.combine, rearChaseTermX, rearChaseTermZ)
        local angleToSideChaseSide = self:getAngleToChasePos(sideChasePos)
        local angleToRearChaseSide = self:getAngleToChasePos(rearChasePos)

        AutoDrive.useNewPipeOffsets = true

        if AutoDrive.useNewPipeOffsets and AutoDrive.isPipeOut(self.combine) then
            local sideOffsetZ = self:getDynamicSideChaseOffsetZ_fromDischargeNode(planningPhase)
            -- local sideOffsetX = self.pipeSide * self:getSideChaseOffsetX_new()
            -- sideChasePos = AutoDrive.createWayPointRelativeToDischargeNode(self.combine, sideOffsetX, sideOffsetZ)
            sideChasePos = self:getPipeChaseWayPoint(0, sideOffsetZ)
            angleToSideChaseSide = self:getAngleToChasePos(sideChasePos)
        end

        if
            (
                (
                    (self.pipeSide == AutoDrive.CHASEPOS_LEFT and not leftBlocked) 
                    or (self.pipeSide == AutoDrive.CHASEPOS_RIGHT and not rightBlocked)
                ) 
                and
                (
                    (
                        self:isUnloaderOnCorrectSide(self.pipeSide) 
                        -- and math.abs(angleToSideChaseSide) < math.abs(angleToRearChaseSide)
                    )
                    or (planningPhase == true)
                )
            )
            or
            (
                (planningPhase == true) and (self.combine.ad.noMovementTimer.elapsedTime > 1000)
            )
         then
            -- Take into account a right sided harvester, e.g. potato harvester.
            chaseNode = sideChasePos
            sideIndex = self.pipeSide
         else
            sideIndex = AutoDrive.CHASEPOS_REAR
            chaseNode = rearChasePos
         end
    end

    self.chasePosIndex = sideIndex
    return chaseNode, self.chasePosIndex
end

function CombineUnloaderMode:getAngleToCombineHeading()
    if self.vehicle == nil or self.combine == nil then
        return math.huge
    end

    local combineRx, _, combineRz = AutoDrive.localDirectionToWorld(self.combine, 0, 0, 1, self.combine.ad.ADRootNode)
    local rx, _, rz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)

    return math.abs(AutoDrive.angleBetween({x = rx, z = rz}, {x = combineRx, z = combineRz}))
end

function CombineUnloaderMode:getAngleToCombine()
    if self.vehicle == nil or self.combine == nil then
        return math.huge
    end

    local vehicleX, _, vehicleZ = getWorldTranslation(self.vehicle.components[1].node)
    local combineX, _, combineZ = getWorldTranslation(self.combine.components[1].node)
    local rx, _, rz =  AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1, self.combine.ad.ADRootNode)

    return math.abs(AutoDrive.angleBetween({x = rx, z = rz}, {x = combineX - vehicleX, z = combineZ - vehicleZ}))
end

function CombineUnloaderMode:getAngleToChasePos(chasePos)
    local worldX, _, worldZ = getWorldTranslation(self.vehicle.components[1].node)
    local rx, _, rz =  AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local angle = AutoDrive.angleBetween({x = rx, z = rz}, {x = chasePos.x - worldX, z = chasePos.z - worldZ})

    return angle
end

function CombineUnloaderMode:getFollowingUnloader()
    return self.followingUnloader
end

function CombineUnloaderMode:registerFollowingUnloader(followingUnloader)
    self.followingUnloader = followingUnloader
end

function CombineUnloaderMode:unregisterFollowingUnloader()
    self.followingUnloader = nil
end

function CombineUnloaderMode:setStateNames()
    if self.statesToNames == nil then
        self.statesToNames = {}
        for name, id in pairs(CombineUnloaderMode) do
            if string.sub(name, 1, 6) == "STATE_" then
                self.statesToNames[id] = name
            end
        end
    end
end

function CombineUnloaderMode:getStateName(state)
    local requestedState = state
    if requestedState == nil then
        requestedState = self.state
    end
    if requestedState == nil then
        Logging.error("[AD] CombineUnloaderMode: Could not find name for state ->%s<- !", tostring(requestedState))
    end
    return self.statesToNames[requestedState] or ""
end

function CombineUnloaderMode.debugMsg(vehicle, debugText, ...)
    if CombineUnloaderMode.debug == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    else
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_COMBINEINFO, debugText, ...)
    end
end
