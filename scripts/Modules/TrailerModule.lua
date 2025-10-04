ADTrailerModule = {}

ADTrailerModule.LOAD_RETRY_TIME = 3000
ADTrailerModule.LOAD_DELAY_TIME = 500
ADTrailerModule.UNLOAD_RETRY_TIME = 30000

function ADTrailerModule:new(vehicle)
    local o = {}
    setmetatable(o, self)
    self.__index = self
    o.vehicle = vehicle
    ADTrailerModule.reset(o)
    o.trailers = nil
    o.trailerCount = 0
    return o
end

function ADTrailerModule:reset()
    if self.isUnloadingWithTrailer ~= nil and self.isUnloadingWithTrailer.setDischargeState then
        self.isUnloadingWithTrailer:setDischargeState(Dischargeable.DISCHARGE_STATE_OFF)
    end
    self.isLoading = false
    self.isUnloading = false
    self.isUnloadingWithTrailer = nil
    self.isUnloadingWithFillUnit = nil
    self.unloadingToBunkerSilo = false
    self.siloTrigger = nil
    self.bunkerTrigger = nil
    self.bunkerTrailer = nil
    self.trigger = nil
    self.isLoadingToFillUnitIndex = nil
    self.isLoadingToTrailer = nil
    self.foundSuitableTrigger = false
    self.filledToUnload = false
    if self.loadRetryTimer == nil then
        self.loadRetryTimer = AutoDriveTON:new()
    else
        self.loadRetryTimer:timer(false)      -- clear timer
    end
    if self.loadDelayTimer == nil then
        self.loadDelayTimer = AutoDriveTON:new()
    else
        self.loadDelayTimer:timer(false)      -- clear timer
    end
    if self.unloadDelayTimer == nil then
        self.unloadDelayTimer = AutoDriveTON:new()
    else
        self.unloadDelayTimer:timer(false)      -- clear timer
    end
    if self.unloadRetryTimer == nil then
        self.unloadRetryTimer = AutoDriveTON:new()
    else
        self.unloadRetryTimer:timer(false)      -- clear timer
    end
    if self.stuckInBunkerTimer == nil then
        self.stuckInBunkerTimer = AutoDriveTON:new()
    else
        self.stuckInBunkerTimer:timer(false)      -- clear timer
    end
    self:clearTrailerUnloadTimers()
    self.trailers, self.trailerCount = AutoDrive.getAllUnits(self.vehicle)
    AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, false)
    AutoDrive.setAugerPipeOpen(self.trailers, false)
    self:handleTrailerReversing(false)
    self.count = 0
    self.hasAL = false
    for i=1, self.trailerCount do
        self.hasAL = self.hasAL or AutoDrive:hasAL(self.trailers[i])
    end
    self.activeAL = false
    self.actualDistanceToUnloadTrigger = math.huge
    self.oldDistanceToUnloadTrigger = math.huge
    self.lastUnloadRotateTrigger = nil
    self.unloadRotate = false
end

function ADTrailerModule:isActiveAtTrigger()
    --AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:isActiveAtTrigger self.isLoading %s self.isUnloading %s", tostring(self.isLoading), tostring(self.isUnloading))
    return self.isLoading or self.isUnloading
end

function ADTrailerModule:isUnloadingToBunkerSilo()
    return self.unloadingToBunkerSilo
end

function ADTrailerModule:getBunkerTrigger()
    return self.bunkerTrigger
end

function ADTrailerModule:getSiloTrigger()
    return self.siloTrigger
end

function ADTrailerModule:getBunkerSiloSpeed()
    local trailer = self.bunkerTrailer
    local trigger = self.bunkerTrigger
    local fillLevel, _, _ =  AutoDrive.getObjectFillLevels(trailer)

    if trailer ~= nil and trailer.getCurrentDischargeNode ~= nil and fillLevel ~= nil and fillLevel > 0 then
        local dischargeNode = trailer:getCurrentDischargeNode()
        if dischargeNode ~= nil and trigger ~= nil and trigger.bunkerSiloArea ~= nil then
            local dischargeSpeed = dischargeNode.emptySpeed
            local unloadTimeInMS = fillLevel / dischargeSpeed

            local dischargeNodeX, dischargeNodeY, dischargeNodeZ = getWorldTranslation(dischargeNode.node)
            local rx, _, rz =  AutoDrive.localDirectionToWorld(trailer, 0, 0, 1)
            local normalVector = {x = -rz, z = rx}

            --                                                                                  vecW
            local x1, z1 = trigger.bunkerSiloArea.sx, trigger.bunkerSiloArea.sz --              1 ---- 2
            --local x2, z2 = trigger.bunkerSiloArea.wx, trigger.bunkerSiloArea.wz--     vecH    | ---- |
            local x3, z3 = trigger.bunkerSiloArea.hx, trigger.bunkerSiloArea.hz --              | ---- |
            --local x4, z4 = x2 + (x3 - x1), z2 + (z3 - z1) --                                  3 ---- 4    4 = 2 + vecH

            local vecH = {x = (x3 - x1), z = (z3 - z1)}
            local vecHLength = MathUtil.vector2Length(vecH.x, vecH.z)

            local hitX, hitZ, insideBunker, positive = AutoDrive.segmentIntersects(x1, z1, x3, z3, dischargeNodeX, dischargeNodeZ, dischargeNodeX + 50 * normalVector.x, dischargeNodeZ + 50 * normalVector.z)

            --ADDrawingManager:addLineTask(dischargeNodeX, dischargeNodeY + 3, dischargeNodeZ , dischargeNodeX + 50 * normalVector.x,dischargeNodeY + 3, dischargeNodeZ + 50 * normalVector.z, 1, 1, 0, 0)
            --ADDrawingManager:addLineTask(x1, dischargeNodeY + 3, z1 , x3, dischargeNodeY + 3, z3, 1, 1, 0, 0)

            if hitX ~= 0 and hitZ ~= 0 then
                --ADDrawingManager:addLineTask(x1, dischargeNodeY + 5, z1 , hitX, dischargeNodeY + 5, hitZ, 1, 0, 0, 1)
                local remainingDistance = vecHLength
                if insideBunker then
                    local drivenDistance = MathUtil.vector2Length(hitX - x1, hitZ - z1)
                    if not positive then
                        drivenDistance = MathUtil.vector2Length(hitX - x3, hitZ - z3)
                    end

                    remainingDistance = vecHLength - drivenDistance - 2  -- finish 2m before the end of the silo
                end

                local speed = ((math.max(1, remainingDistance) / unloadTimeInMS) * 1000) * 3.6 * 1

                return speed
            end
        end
    end
    return 12
end

function ADTrailerModule:update(dt)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:update start")
    local updateStatesDone = false
    if self.trailerCount == 0 then
        return
    end
    local distanceToUnload = AutoDrive.getDistanceToUnloadPosition(self.vehicle)

    if self.vehicle.ad.stateModule:getCurrentMode():shouldUnloadAtTrigger() and (AutoDrive.isInRangeToLoadUnloadTarget(self.vehicle) or distanceToUnload < (ADTriggerManager.getMaxBunkerSiloLength()  + AutoDrive.getMaxTriggerDistance(self.vehicle))) then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:update updateUnload")
        if not updateStatesDone then
            self:updateStates()
            updateStatesDone = true
        end
        self:updateUnload(dt)
    end
    if self.vehicle.ad.stateModule:getCurrentMode():shouldLoadOnTrigger() and AutoDrive.isInRangeToLoadUnloadTarget(self.vehicle) then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:update updateLoad")
        if not updateStatesDone then
            self:updateStates()
            updateStatesDone = true
        end
        if self.hasAL == true then
            -- activate AutoLoad only once when in destination range
            if self.activeAL == false then
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad trailer with AL found -> activate AL in all trailers")
                self.activeAL = true
                -- AutoDrive.activateALTrailers(self.vehicle, self.trailers)
                -- do not activate AL here, vehicle might still move, loading not possible during drive!
                -- no further actions required, monitoring via fill level - see load from source without trigger
            end
        else
            self:updateLoad(dt)
        end
    end
    self:handleTrailerCovers()

    -- self:handleTrailerReversing()
    self.lastFillLevel = self.fillLevel
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:update end lastFillLevel %s", tostring(self.lastFillLevel))
end

function ADTrailerModule:handleTrailerCovers()
    -- open trailer cover if trigger is reachable
    local isInRangeToLoadUnloadTarget = AutoDrive.isInRangeToLoadUnloadTarget(self.vehicle)
    AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, isInRangeToLoadUnloadTarget)
    if isInRangeToLoadUnloadTarget and self.hasAL then
        -- open curtains for UAL
        AutoDrive.openAllCurtains(self.trailers, true) -- open curtain at UAL trailers
    end
end

function ADTrailerModule:updateStates()
    self.fillLevel, _, self.filledToUnload, _ = AutoDrive.getAllFillLevels(self.trailers)

    self.fillUnits = 0
    if self.lastFillLevel == nil then
        self.lastFillLevel = self.fillLevel
    end
    self.blocked = self.lastFillLevel <= self.fillLevel
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateStates start self.isLoading %s self.isUnloading %s self.lastFillLevel %.1f self.fillLevel %.1f self.blocked %s", tostring(self.isLoading), tostring(self.isUnloading), self.lastFillLevel, self.fillLevel, tostring(self.blocked))
    for _, trailer in pairs(self.trailers) do
        if trailer.getFillUnits ~= nil then
            self.fillUnits = self.fillUnits + #trailer:getFillUnits()
        end
        local tipState = Trailer.TIPSTATE_CLOSED
        if trailer.getTipState ~= nil then
            tipState = trailer:getTipState()
            self.blocked = self.blocked and (not (tipState == Trailer.TIPSTATE_OPENING or tipState == Trailer.TIPSTATE_CLOSING))
        end
    end
    self.unloadRotate = (AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYDELIVER or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders")
    if not self.unloadRotate then
        self.lastUnloadRotateTrigger = nil
    end
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateStates end self.isLoading %s self.isUnloading %s self.fillUnits %s self.blocked %s", tostring(self.isLoading), tostring(self.isUnloading), tostring(self.fillUnits), tostring(self.blocked))
end


function ADTrailerModule:canBeHandledInReverse()
    if self.trailers == nil then
        self.trailers, self.trailerCount = AutoDrive.getAllUnits(self.vehicle)
    end

    return #self.trailers <= 2
end

-- Code snippets used from mod: FS19_TrailerJointBlock - credits to Northern_Strike
function ADTrailerModule:handleTrailerReversing(blockTrailers)
    if self.trailers == nil then
        self:updateStates()
        return
    end
    for i, trailer in ipairs(self.trailers) do
        local specAttachable = trailer.spec_attachable
        if specAttachable and blockTrailers and specAttachable.steeringAxleAngle and specAttachable.steeringAxleAngle ~= 0 then
            specAttachable.steeringAxleAngle = 0
        end

        if i > 1 and #trailer.components > 1 then
            -- ignore trailing vehicle
            if #trailer.componentJoints >= 1 then
                if trailer.ad == nil then
                    trailer.ad = {}
                    trailer.ad.lastBlockedState = false
                    trailer.ad.targetBlockedState = false
                end

                trailer.ad.targetBlockedState = blockTrailers

                if trailer.ad.rotLimitBackup == nil then
                    for index, joint in pairs(trailer.componentJoints) do
                        if joint.rotLimit[1] == 0 and joint.rotLimit[2] ~= 0 and joint.rotLimit[3] == 0 then
                            trailer.ad.rotIndex = index
                            trailer.ad.rotLimitBackup = joint.rotLimit[2]
                            break
                        end
                    end
                end
                if trailer.ad.rotIndex then
                    if trailer.ad.lastBlockedState ~= trailer.ad.targetBlockedState then
                        if trailer.ad.targetBlockedState then
                            trailer:setComponentJointRotLimit(trailer.componentJoints[trailer.ad.rotIndex], 2, 0, 0)
                        else
                            trailer:setComponentJointRotLimit(trailer.componentJoints[trailer.ad.rotIndex], 2, -trailer.ad.rotLimitBackup, trailer.ad.rotLimitBackup)
                        end
                        trailer.ad.lastBlockedState = trailer.ad.targetBlockedState;
                    end
                end
            end
        end
    end
end

function ADTrailerModule:updateLoad(dt)
    self.count =  self.count + dt

    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad start self.trigger %s", tostring(self.trigger))
    local fillUnitFull = AutoDrive.getIsFillUnitFull(self.isLoadingToTrailer, self.isLoadingToFillUnitIndex)
    local fillFound = false
    local checkForContinue = false
    local checkFillUnitFull = false

    -- update trigger timer
    if self.trigger ~= nil and self.trigger.stoppedTimer ~= nil then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad update trigger timer self.trigger.isLoading %s", tostring(self.trigger.isLoading))
        self.trigger.stoppedTimer:timer(not self.trigger.isLoading,300,dt)
        if self.trigger.isLoading then
            -- if still loading reset retry timer
            self.loadRetryTimer:timer(false, ADTrailerModule.LOAD_RETRY_TIME)     -- reset the timer to try load again
        end
    end

    -- update retry timer
    self.loadRetryTimer:timer(true, ADTrailerModule.LOAD_RETRY_TIME, dt)
    -- update load delay timer
    self.loadDelayTimer:timer(self.lastFillLevel >= self.fillLevel and self.trigger == self, ADTrailerModule.LOAD_DELAY_TIME, dt)

    if self.trigger == nil and not fillUnitFull then

        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad search for load self.trigger %s", tostring(self.trigger))

        -- look for triggers with requested fill type
        local loadPairs = AutoDrive.getTriggerAndTrailerPairs(self.vehicle, dt)
        local index = 0
        for _, pair in pairs(loadPairs) do
            index = index + 1
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad pair.hasFill %s", tostring(pair.hasFill))

            if pair.hasFill then
                -- initiate load only if fill is available
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad Try loading at trigger now pair.fillUnitIndex %s", tostring(pair.fillUnitIndex))
                fillFound = true
                -- start loading

                self.vehicle.ad.stateModule:selectPreferredFillTypeFromFillLevels(pair.fillLevels)
                self:tryLoadingAtTrigger(pair.trailer, pair.trigger, pair.fillUnitIndex)
                self.foundSuitableTrigger = true    -- loading trigger was found
                return
            end
        end

        -- check for load water from ground
        local waterTrailer = AutoDrive.getWaterTrailerInWater(self.vehicle, self.trailers)
        if waterTrailer ~= nil and waterTrailer.setIsWaterTrailerFilling ~= nil then
            waterTrailer:setIsWaterTrailerFilling(true)
            fillFound = true
            self.isLoading = true
            self.foundSuitableTrigger = true    -- loading trigger was found
            self.trigger = waterTrailer         -- need a trigger to not search again
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad WaterTrailer found water -> start load")
            return
        end

        if not self.isLoading then
            -- try overload from liquid trailers, containers etc.
            if AutoDrive.startFillTrigger(self.trailers) ~= nil then
                -- no further actions required, monitoring via fill level - see load from source without trigger
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad overload fillTrigger found -> load already started")
            elseif AutoDrive.startLoadTreePlanter(self.trailers) ~= nil then
                -- no further actions required, monitoring via fill level - see load from source without trigger
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad overload treePlanter found -> load already started")
            end           
        end

        -- check for load from source without trigger
        if self.lastFillLevel < self.fillLevel then
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad load from source without trigger...")
            fillFound = true
            self.isLoading = true
            self.trigger = self                 -- need a trigger to not search again
            -- update load delay timer
            self.loadDelayTimer:timer(false, ADTrailerModule.LOAD_DELAY_TIME)
        end

        if #loadPairs == 0 and waterTrailer == nil and self.trigger ~= self then
            self.isLoading = false
            self.trigger = nil
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad loadPairs == 0, Nothing found to load - continue drive -> return")
            return
        else
            if fillFound then
                -- load already initiated - see above
            else
                checkForContinue = true
            end
        end
    end
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad check for load done self.trigger %s", tostring(self.trigger))
    if self.trigger ~= nil and self.trigger.stoppedTimer ~= nil and self.trigger.stoppedTimer:done() then
        -- load from trigger
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad loading finished - check fill level...")
        checkFillUnitFull = true
    elseif self.trigger ~= nil and self.trigger.stoppedTimer == nil and self.trigger.spec_waterTrailer ~= nil and self.trigger.spec_waterTrailer.isFilling ~= nil and not self.trigger.spec_waterTrailer.isFilling then
        -- load water
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad WaterTrailer full")
        checkFillUnitFull = true
    elseif self.trigger ~= nil and self.trigger.stoppedTimer == nil and self.trigger == self and self.loadDelayTimer:done() and self.lastFillLevel >= self.fillLevel then
        -- load from source without trigger
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad trailers full")
        checkFillUnitFull = true
    else
        -- still loading from trigger
        -- AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad still loading from trigger -> return ")
        -- return
    end

    if checkFillUnitFull then
        if fillUnitFull or (self.trigger ~= nil and self.trigger.stoppedTimer == nil and self.trigger.spec_waterTrailer ~= nil and self.trigger.spec_waterTrailer.isFilling ~= nil and not self.trigger.spec_waterTrailer.isFilling) or (self.trigger ~= nil and self.trigger.stoppedTimer == nil and self.trigger == self and self.loadDelayTimer:done() and self.lastFillLevel >= self.fillLevel) then
            self.isLoading = false
            self.trigger = nil
            self.isLoadingToFillUnitIndex = 0
            self.isLoadingToTrailer = nil
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad fillUnitFull %s", tostring(fillUnitFull))
            return
        else
            checkForContinue = true
        end
    end

    if checkForContinue then
        if AutoDrive.checkForContinueOnEmptyLoadTrigger(self.vehicle) or self.filledToUnload then
            -- continue or unload fill level reached
            self.isLoading = false
            self.trigger = nil
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad continue -> return")
            return
        else
            -- not continue - retry cycle
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad retry cycle")
            self.isLoading = true
            if self.loadRetryTimer:done() then
                -- initiate load animation
                local loadPairs = AutoDrive.getTriggerAndTrailerPairs(self.vehicle, dt)
                for _, pair in pairs(loadPairs) do
                    self:tryLoadingAtTrigger(pair.trailer, pair.trigger, pair.fillUnitIndex)
                end
                -- self.trigger = nil
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad try loading again after some time")
                self.loadRetryTimer:timer(false, ADTrailerModule.LOAD_RETRY_TIME)     -- reset the timer to try load again
                return
            else
                -- wait for fill
            end
        end
    end
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateLoad end")
end

function ADTrailerModule:stopLoading()
    self.isLoading = false
end

function ADTrailerModule:stopUnloading()
    self.isUnloading = false
    self.unloadingToBunkerSilo = false
    for _, trailer in pairs(self.trailers) do
        if trailer.setDischargeState ~= nil then
            trailer:setDischargeState(Dischargeable.DISCHARGE_STATE_OFF)
        end
    end
    self:clearTrailerUnloadTimers()
    if self.unloadDelayTimer ~= nil then
        self.unloadDelayTimer:timer(false)      -- clear timer
    end
    self.lastUnloadRotateTrigger = nil
end

function ADTrailerModule:updateUnload(dt)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateUnload ")
    AutoDrive.setAugerPipeOpen(self.trailers,  AutoDrive.getDistanceToUnloadPosition(self.vehicle) <= AutoDrive.getMaxTriggerDistance(self.vehicle))

    if not self.isUnloading and not (self.lastUnloadRotateTrigger) then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateUnload not self.isUnloading")
        -- Check if we can unload at some trigger
            for _, trailer in pairs(self.trailers) do
                local unloadTrigger = self:lookForPossibleUnloadTrigger(trailer)
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateUnload not self.isUnloading unloadTrigger %s", tostring(unloadTrigger))
                if unloadTrigger ~= nil then
                    if trailer.unloadDelayTimer == nil then
                        trailer.unloadDelayTimer = AutoDriveTON:new()
                    end
                    trailer.unloadDelayTimer:timer(unloadTrigger ~= nil, 250, dt)
                    if unloadTrigger ~= nil and (trailer.unloadDelayTimer:done() or unloadTrigger.bunkerSiloArea ~= nil) then
                        trailer.unloadDelayTimer:timer(false)       -- clear timer
                        self:startUnloadingIntoTrigger(trailer, unloadTrigger)
                    end
                end
                -- overload to another trailer
                if (trailer.spec_pipe ~= nil) then
                    if trailer:getDischargeState() ~= Dischargeable.DISCHARGE_STATE_OFF then
                        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateUnload unload via pipe")
                        self.isUnloading = true
                        self.isUnloadingWithTrailer = trailer
                        self.isUnloadingWithFillUnit = trailer:getCurrentDischargeNode().fillUnitIndex
                    end
                end
            end
    else
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateUnload Monitor unloading")
        local fillUnitEmpty = AutoDrive.getIsFillUnitEmpty(self.isUnloadingWithTrailer, self.isUnloadingWithFillUnit)
        local allTrailersClosed = self:areAllTrailersClosed(dt)
        self.unloadDelayTimer:timer(self.isUnloading, 250, dt)
        self.stuckInBunkerTimer:timer((self.vehicle.lastSpeedReal * 3600 < 1), 500, dt)
        if self.unloadDelayTimer:done() then
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:updateUnload Monitor unloading unloadDelayTimer:done areAllTrailersClosed %s", tostring(allTrailersClosed))
            self.unloadRetryTimer:timer(self.isUnloading, ADTrailerModule.UNLOAD_RETRY_TIME, dt)

            if allTrailersClosed and (fillUnitEmpty or self.unloadRotate) then
            -- all trailers closed and empty or rotate
                self.unloadDelayTimer:timer(false)      -- clear timer
                self.isUnloading = false
                self.unloadingToBunkerSilo = false
                if self.isUnloadingWithTrailer ~= nil and self.isUnloadingWithTrailer.setDischargeState then
                    self.isUnloadingWithTrailer:setDischargeState(Dischargeable.DISCHARGE_STATE_OFF)
                end
            elseif fillUnitEmpty and self.unloadingToBunkerSilo then
                self.unloadDelayTimer:timer(false)      -- clear timer
                self.unloadingToBunkerSilo = false
            elseif allTrailersClosed and self.isUnloadingWithTrailer ~= nil and self.isUnloadingWithTrailer.spec_pipe ~= nil then
                -- unload auger wagon to another trailer
                self.unloadDelayTimer:timer(false)      -- clear timer
                self.isUnloading = false
            elseif self.unloadRetryTimer:done() and self.isUnloadingWithTrailer ~= nil and self.unloadingToBunkerSilo == false then
                if self.isUnloadingWithTrailer ~= nil and self.isUnloadingWithTrailer.setDischargeState then
                    self.isUnloadingWithTrailer:setDischargeState(Dischargeable.DISCHARGE_STATE_OBJECT)
                end
                self.unloadRetryTimer:timer(false)      -- clear timer
            elseif not (self.vehicle.ad.drivePathModule:getIsReversing()) and self.unloadingToBunkerSilo and self.stuckInBunkerTimer:done() then
                -- stuck in silo bunker
                if self.isUnloadingWithTrailer ~= nil and self.isUnloadingWithTrailer.setDischargeState then
                    self.isUnloadingWithTrailer:setDischargeState(Dischargeable.DISCHARGE_STATE_OFF)
                end
                self.unloadDelayTimer:timer(false)      -- clear timer
                self.isUnloading = false
                self.unloadingToBunkerSilo = false
            end
        end
    end
end

function ADTrailerModule:tryLoadingAtTrigger(trailer, trigger, fillUnitIndex)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:tryLoadingAtTrigger start self.count %s", tostring(self.count))
    self.count = 0

    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:tryLoadingAtTrigger start")
    local fillUnits = trailer:getFillUnits()

    local isFillUnitFull = trailer:getFillUnitFreeCapacity(fillUnitIndex) <= 0.1
    if not isFillUnitFull and (not trigger.isLoading) then
        -- activate load trigger
        local trailerIsInRange = AutoDrive.trailerIsInTriggerList(trailer, trigger, fillUnitIndex)
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:tryLoadingAtTrigger trailerIsInRange %s", tostring(trailerIsInRange))
        if trailerIsInRange then --and not self.isLoading then
            if #fillUnits > 1 then
                --print("startLoadingCorrectFillTypeAtTrigger now - " .. fillUnitIndex)
                self:startLoadingCorrectFillTypeAtTrigger(trailer, trigger, fillUnitIndex)
            else
                --print("startLoadingAtTrigger now - " .. fillUnitIndex .. " fillType: " .. self.vehicle.ad.stateModule:getFillType())
                self:startLoadingAtTrigger(trigger, self.vehicle.ad.stateModule:getFillType(), fillUnitIndex, trailer)
            end
            self.isLoading = self.isLoading or trigger.isLoading
        end
    end
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:tryLoadingAtTrigger end")
end

function ADTrailerModule:startLoadingCorrectFillTypeAtTrigger(trailer, trigger, fillUnitIndex)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:startLoadingCorrectFillTypeAtTrigger start")
    if not AutoDrive.fillTypesMatch(self.vehicle, trigger, trailer, nil, fillUnitIndex) then
        -- local storedFillType = self.vehicle.ad.stateModule:getFillType()
        local toCheck = {'SEEDS','FERTILIZER','LIQUIDFERTILIZER'}

        for _, fillTypeName in pairs(toCheck) do
            local fillTypeIndex = g_fillTypeManager:getFillTypeIndexByName(fillTypeName)
            -- self.vehicle.ad.stateModule:setFillType(fillTypeIndex)
            if AutoDrive.fillTypesMatch(self.vehicle, trigger, trailer, nil, fillUnitIndex) then
                self:startLoadingAtTrigger(trigger, fillTypeIndex, fillUnitIndex, trailer)
                -- self.vehicle.ad.stateModule:setFillType(storedFillType)
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:startLoadingCorrectFillTypeAtTrigger found fillType 'SEEDS','FERTILIZER','LIQUIDFERTILIZER' - return")
                return
            end
        end

        -- self.vehicle.ad.stateModule:setFillType(storedFillType)
    else
        self:startLoadingAtTrigger(trigger, self.vehicle.ad.stateModule:getFillType(), fillUnitIndex, trailer)
    end
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:startLoadingCorrectFillTypeAtTrigger end")
end

function ADTrailerModule:startLoadingAtTrigger(trigger, fillType, fillUnitIndex, trailer)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:startLoadingAtTrigger start trigger %s fillUnitIndex %s", tostring(trigger), tostring(fillUnitIndex))
    trigger.autoStart = true
    trigger.selectedFillType = fillType
    trigger:onFillTypeSelection(fillType)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:startLoadingAtTrigger trigger.isLoading %s", tostring(trigger.isLoading))
    if trigger.isLoading then
        trigger.selectedFillType = fillType
        g_effectManager:setEffectTypeInfo(trigger.effects, trigger.selectedFillType)
        trigger.autoStart = false
        -- reset trigger load timer
        trigger.stoppedTimer:timer(false, 300)

        self.isLoading = true
        self.trigger = trigger
        self.isLoadingToFillUnitIndex = fillUnitIndex
        self.isLoadingToTrailer = trailer
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:startLoadingAtTrigger self.trigger %s", tostring(self.trigger))
    end

    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:startLoadingAtTrigger end")
end

function ADTrailerModule:lookForPossibleUnloadTrigger(trailer)
    self.siloTrigger = nil
    self.bunkerTrigger = nil
    AutoDrive.findAndSetBestTipPoint(self.vehicle, trailer)

    if (trailer.getCurrentDischargeNode == nil or self.fillLevel == 0) then
        return nil
    end

    -- local distanceToTarget = AutoDrive.getDistanceToUnloadPosition(self.vehicle)
    local distanceToTarget = AutoDrive.getDistanceToUnloadPosition(trailer)

    if distanceToTarget < AutoDrive.getMaxTriggerDistance(self.vehicle) then
        -- silo trigger - found by CanDischargeToObject, no need to loop through all triggers
        if trailer.getCanDischargeToObject and trailer.getCurrentDischargeNode and trailer.getDischargeState then
            if trailer:getCanDischargeToObject(trailer:getCurrentDischargeNode()) then
                if (trailer:getDischargeState() == Dischargeable.DISCHARGE_STATE_OFF and trailer.spec_pipe == nil)
                    or (trailer.spec_pipe ~= nil and trailer.spec_pipe.currentState >= 2)
                then
                    local trigger = {}
                    self.siloTrigger = trigger
                    return trigger
                end
            end
        end
    end

    local trailerX, trailerY, trailerZ = getWorldTranslation(trailer.components[1].node)
    local isInBunkerSiloRange = distanceToTarget < (ADTriggerManager.getMaxBunkerSiloLength() + AutoDrive.getMaxTriggerDistance(self.vehicle))
    if isInBunkerSiloRange then
        for _, trigger in pairs(ADTriggerManager.getBunkerSilos()) do
            if trigger and trigger.bunkerSiloArea ~= nil then
                local triggerX, _, triggerZ = ADTriggerManager.getTriggerPos(trigger)
                if triggerX ~= nil then
                    -- bunker silo
                    if AutoDrive.isTrailerInBunkerSiloArea(trailer, trigger) then
                        self.bunkerTrigger = trigger
                        return trigger
                    end
                end
            end
        end
    end
    return nil
end

function ADTrailerModule:startUnloadingIntoTrigger(trailer, trigger)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "ADTrailerModule:startUnloadingIntoTrigger start")

    if trigger.bunkerSiloArea == nil then
        -- tip trigger
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "Start unloading - fillUnitIndex: %s", tostring(trailer:getCurrentDischargeNode().fillUnitIndex))
        trailer:setDischargeState(Dischargeable.DISCHARGE_STATE_OBJECT)

        if self.unloadRotate then
            -- keep the already served unload trigger
            self.lastUnloadRotateTrigger = trigger
        end

        self.isUnloading = true
        self.isUnloadingWithTrailer = trailer
        self.isUnloadingWithFillUnit = trailer:getCurrentDischargeNode().fillUnitIndex
    elseif trigger.bunkerSiloArea ~= nil then
        -- bunker silo
        if (not self.vehicle.ad.drivePathModule:getIsReversing() and not (self.vehicle.lastSpeedReal * 3600 < 1)) -- forward through bunker silo
            or (self.vehicle.ad.drivePathModule:getIsReversing() and self.vehicle:getLastSpeed() < 1) then -- reverse into bunker silo
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_TRAILERINFO, "Start unloading into bunkersilo - fillUnitIndex: %s", tostring(trailer:getCurrentDischargeNode().fillUnitIndex))
            trailer:setDischargeState(Dischargeable.DISCHARGE_STATE_GROUND)
            self.isUnloading = true
            self.unloadingToBunkerSilo = true
            self.bunkerTrailer = trailer
            self.isUnloadingWithTrailer = trailer
            self.isUnloadingWithFillUnit = trailer:getCurrentDischargeNode().fillUnitIndex
            trailer:getCurrentDischargeNode().lastEffect = nil -- disable effect to start unloading before effect is complete visible
        end
    end
end

function ADTrailerModule:getIsBlocked(dt)
    if self.isUnloadingWithTrailer ~= nil and self.unloadingToBunkerSilo then
        if self.blockedTimer == nil then
            self.blockedTimer = AutoDriveTON:new()
        end
        return self.blockedTimer:timer(self.blocked, 1000, dt)
    end
    return false
end

function ADTrailerModule:areAllTrailersClosed(dt)
    local allClosed = true
    for _, trailer in pairs(self.trailers) do
        local tipState = Trailer.TIPSTATE_CLOSED
        if trailer.getTipState ~= nil then
            tipState = trailer:getTipState()
        end
        local dischargeState = Dischargeable.DISCHARGE_STATE_OFF
        if trailer.getDischargeState ~= nil then
            dischargeState = trailer:getDischargeState()
        end
        if trailer.noDischargeTimer == nil then
            trailer.noDischargeTimer = AutoDriveTON:new()
        end
        --print("Tipstate: " .. tipState .. " dischargeState: " .. dischargeState)
        local senseUnloading = false
        if self.fillLevel ~= nil then
            if self.lastFillLevel == nil or math.abs(self.lastFillLevel - self.fillLevel) > 0.1 or (self.unloadingToBunkerSilo and self.fillLevel > 0) then
                senseUnloading = true
            end
        end
        --print("Tipstate: " .. tipState .. " dischargeState: " .. dischargeState .. " senseUnloading: " .. tostring(senseUnloading) .. " lastFillLevel: " .. self.lastFillLevel .. " current: " .. self.fillLevel)
        senseUnloading = senseUnloading or tipState == Trailer.TIPSTATE_OPENING or tipState == Trailer.TIPSTATE_CLOSING
        if not trailer.noDischargeTimer:timer((not senseUnloading) or (tipState == Trailer.TIPSTATE_CLOSED and dischargeState == Dischargeable.DISCHARGE_STATE_OFF), 500, dt) then
            allClosed = false
        end
    end

    return allClosed
end

function ADTrailerModule:wasAtSuitableTrigger()
    return self.foundSuitableTrigger
end

function ADTrailerModule:clearTrailerUnloadTimers()
    if self.trailers ~= nil then
        for _, trailer in pairs(self.trailers) do
            if trailer ~= nil and trailer.unloadDelayTimer ~= nil then
                trailer.unloadDelayTimer:timer(false)       -- clear timer
            end
        end
    end
end

function ADTrailerModule:getCanStopMotor()
    local ret=true
    if self.isUnloading and self.isUnloadingWithTrailer ~= nil then
        local tipState = Trailer.TIPSTATE_CLOSED
        if self.isUnloadingWithTrailer.getTipState ~= nil then
            tipState = self.isUnloadingWithTrailer:getTipState()
        end
        local dischargeState = Dischargeable.DISCHARGE_STATE_OFF
        if self.isUnloadingWithTrailer.getDischargeState ~= nil then
            dischargeState = self.isUnloadingWithTrailer:getDischargeState()
        end
        if (tipState ~= Trailer.TIPSTATE_CLOSED or dischargeState ~= Dischargeable.DISCHARGE_STATE_OFF) then
            ret = false
        end
    end
    return ret
end

function ADTrailerModule:getHasAL()
    return self.hasAL
end

function ADTrailerModule.debugMsg(vehicle, debugText, ...)
    if ADTrailerModule.debug == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    end
end
