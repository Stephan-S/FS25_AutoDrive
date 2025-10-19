ADStateModule = {}

ADStateModule.CREATE_OFF = 1
ADStateModule.CREATE_NORMAL = 2
ADStateModule.CREATE_DUAL = 3
ADStateModule.CREATE_SUB_PRIO = 4
ADStateModule.CREATE_SUB_PRIO_DUAL = 5
ADStateModule.CREATE_NORMAL_TWOWAY = 6
ADStateModule.CREATE_DUAL_TWOWAY = 7
ADStateModule.CREATE_SUB_PRIO_TWOWAY = 8
ADStateModule.CREATE_SUB_PRIO_DUAL_TWOWAY = 9

ADStateModule.CALCULATE_REMAINING_DRIVETIME_INTERVAL = 1000

ADStateModule.HIGHEST_MODE = 5

ADStateModule.BUNKER_UNLOAD_TRIGGER = 1
ADStateModule.BUNKER_UNLOAD_TRAILER = 2

ADStateModule.HELPER_NONE = 10/2 -- must not be 0
ADStateModule.HELPER_CP = 1
ADStateModule.HELPER_AIVE = 2
ADStateModule.HELPER_AI = 3
ADStateModule.NUM_HELPER_TYPES = 3  -- number of valid helper types above

function ADStateModule:new(vehicle)
    local o = {}
    setmetatable(o, self)
    self.__index = self
    o.vehicle = vehicle
    ADStateModule.reset(o)
    return o
end

function ADStateModule:reset()
    self.active = false
    self.mode = AutoDrive.MODE_DRIVETO
    self.firstMarker = ADGraphManager:getMapMarkerById(1)
    self.secondMarker = ADGraphManager:getMapMarkerById(1)
    self.creationMode = ADStateModule.CREATE_OFF

    self.fillType = FillType.UNKNOWN
    self.selectedFillTypes = {}
    self.loadByFillLevel = true
    self.loopCounter = 0
    self.loopsDone = 0

    self.speedLimit = AutoDrive.getVehicleMaxSpeed(self.vehicle)
    self.fieldSpeedLimit = AutoDrive.getVehicleMaxSpeed(self.vehicle)

    self.parkDestination = -1
    self.parkDestinationAtJobFinished = -1

    self.currentDestination = nil

    self.currentTaskInfo = ""
    self.currentLocalizedTaskInfo = ""

    self.currentWayPointId = -1
    self.nextWayPointId = -1

    self.pointToNeighbour = false
    self.currentNeighbourToPointAt = -1
    self.neighbourPoints = {}

    self.startHelper = false

    if self.vehicle.cpStartStopDriver then
        self.usedHelper = ADStateModule.HELPER_CP
    elseif self.vehicle.acParameters then
        self.usedHelper = ADStateModule.HELPER_AIVE
    elseif self.vehicle.getLastJob then
        self.usedHelper = ADStateModule.HELPER_AI
    else
        self.usedHelper = ADStateModule.HELPER_NONE
    end

    self.driverName = g_i18n:getText("UNKNOWN")
    if self.vehicle.getName ~= nil then
        self.driverName = self.vehicle:getName()
    end
    self.remainingDriveTime = 0
    self.calculateRemainingDriveTimeInterval = 0
    self.activeBeforeSave = false
    self.bunkerUnloadType = ADStateModule.BUNKER_UNLOAD_TRIGGER
    self.automaticUnloadTarget = false
    self.automaticPickupTarget = false
    self.harversterPairingOk = false
    self.currentHelperIndex = 0
    self.playerFarmId = FarmManager.SPECTATOR_FARM_ID
    self.actualFarmId = FarmManager.SPECTATOR_FARM_ID
end

function ADStateModule:readFromXMLFile(xmlFile, key)
    if not xmlFile:hasProperty(key) then
        return
    end

    local mode = xmlFile:getValue(key .. "#mode")
    --local mode = getXMLInt(xmlFile, key .. "#mode")
    if mode ~= nil then
        --if mode == AutoDrive.MODE_BGA then
            --mode = AutoDrive.MODE_DRIVETO
        --end
        self.mode = mode
    end

    local firstMarker = xmlFile:getValue(key .. "#firstMarker")
    if firstMarker ~= nil then
        self.firstMarker = ADGraphManager:getMapMarkerById(firstMarker)
    else
        self.firstMarker = ADGraphManager:getMapMarkerById(1)
    end

    local secondMarker = xmlFile:getValue(key .. "#secondMarker")
    if secondMarker ~= nil then
        self.secondMarker = ADGraphManager:getMapMarkerById(secondMarker)
    else
        self.secondMarker = ADGraphManager:getMapMarkerById(1)
    end

    local fillType = xmlFile:getValue(key .. "#fillType")
    if fillType ~= nil then
        self.fillType = fillType
    end

    local selectedFillTypes = xmlFile:getValue(key .. "#selectedFillTypes")
    if selectedFillTypes ~= nil then
        self.selectedFillTypes = AutoDrive.stringToNumberList(selectedFillTypes)
    elseif self.fillType ~= AutoDrive.UAL_FILLTYPE_ALL then
        self.selectedFillTypes = {self.fillType}
    end

    local loadByFillLevel = xmlFile:getValue(key .. "#loadByFillLevel")
    if loadByFillLevel ~= nil then
        self.loadByFillLevel = loadByFillLevel
    end

    local loopCounter = xmlFile:getValue(key .. "#loopCounter")
    if loopCounter ~= nil then
        self.loopCounter = loopCounter
    end

    local loopsDone = xmlFile:getValue(key .. "#loopsDone")
    if loopsDone ~= nil then
        self.loopsDone = loopsDone
    end

    local speedLimit = xmlFile:getValue(key .. "#speedLimit")
    if speedLimit ~= nil then
        self.speedLimit = math.min(speedLimit, AutoDrive.getVehicleMaxSpeed(self.vehicle))
    end

    local fieldSpeedLimit = xmlFile:getValue(key .. "#fieldSpeedLimit")
    if fieldSpeedLimit ~= nil then
        self.fieldSpeedLimit = math.min(fieldSpeedLimit, AutoDrive.getVehicleMaxSpeed(self.vehicle))
    end

    local parkDestination = xmlFile:getValue(key .. "#parkDestination")
    if parkDestination ~= nil then
        self.parkDestination = parkDestination
    end

    local driverName = xmlFile:getValue(key .. "#driverName")
    if driverName ~= nil then
        self.driverName = driverName
    end

    local active = xmlFile:getValue(key .. "#active")
    if active ~= nil then
        self.activeBeforeSave = active
    end

    local startHelper = xmlFile:getValue(key .. "#startHelper")
    if startHelper ~= nil then
        self.startHelper = startHelper
    end

    local bunkerUnloadType = xmlFile:getValue(key .. "#bunkerUnloadType")
    if bunkerUnloadType ~= nil then
        self.bunkerUnloadType = bunkerUnloadType
    end

    local actualFarmId = xmlFile:getValue(key .. "#actualFarmId")
    if actualFarmId ~= nil then
        self.actualFarmId = actualFarmId
    end

    -- local automaticUnloadTarget = xmlFile:getValue(key .. "#automaticUnloadTarget")
    -- if automaticUnloadTarget ~= nil then
        -- self.automaticUnloadTarget = automaticUnloadTarget
    -- end

    -- local automaticPickupTarget = xmlFile:getValue(key .. "#automaticPickupTarget")
    -- if automaticPickupTarget ~= nil then
        -- self.automaticPickupTarget = automaticPickupTarget
    -- end
end

function ADStateModule:saveToXMLFile(xmlFile, key)
    xmlFile:setValue(key .. "#mode", self.mode)
    if self.firstMarker ~= nil then
        xmlFile:setValue(key .. "#firstMarker", self.firstMarker.markerIndex)
    end
    if self.secondMarker ~= nil then
        xmlFile:setValue(key .. "#secondMarker", self.secondMarker.markerIndex)
    end
    xmlFile:setValue(key .. "#fillType", self.fillType)
    xmlFile:setValue(key .. "#selectedFillTypes", table.concat(self.selectedFillTypes, ','))
    xmlFile:setValue(key .. "#loadByFillLevel", self.loadByFillLevel)
    xmlFile:setValue(key .. "#loopCounter", self.loopCounter)
    xmlFile:setValue(key .. "#loopsDone", self.loopsDone)
    xmlFile:setValue(key .. "#speedLimit", self.speedLimit)
    xmlFile:setValue(key .. "#fieldSpeedLimit", self.fieldSpeedLimit)
    xmlFile:setValue(key .. "#driverName", self.driverName)
    xmlFile:setValue(key .. "#active", self.active)
    xmlFile:setValue(key .. "#startHelper", self.startHelper)
    xmlFile:setValue(key .. "#bunkerUnloadType", self.bunkerUnloadType)
    xmlFile:setValue(key .. "#actualFarmId", self.actualFarmId)
    -- xmlFile:setValue(key .. "#automaticUnloadTarget", self.automaticUnloadTarget)
    -- xmlFile:setValue(key .. "#automaticPickupTarget", self.automaticPickupTarget)
end

function ADStateModule:doWriteStream(streamId)
    streamWriteBool(streamId, self.active)
    streamWriteUIntN(streamId, self.mode, 4)
    streamWriteUIntN(streamId, self:getFirstMarkerId() + 1, 20)
    streamWriteUIntN(streamId, self:getSecondMarkerId() + 1, 20)
    streamWriteUIntN(streamId, self.creationMode, 3)
    streamWriteUIntN(streamId, self.fillType, 10)
    AutoDrive.streamWriteUIntNList(streamId, self.selectedFillTypes, 10)
    streamWriteBool(streamId, self.loadByFillLevel)
    streamWriteUIntN(streamId, self.loopCounter, 7)
    streamWriteUIntN(streamId, self.loopsDone, 7)
    streamWriteUIntN(streamId, self.speedLimit, 8)
    streamWriteUIntN(streamId, self.fieldSpeedLimit, 8)
    streamWriteUIntN(streamId, self.parkDestination + 1, 20)
    streamWriteUIntN(streamId, self.parkDestinationAtJobFinished + 1, 20)
    streamWriteUIntN(streamId, self:getCurrentDestinationId() + 1, 20)
    streamWriteString(streamId, self.currentTaskInfo)
    streamWriteUIntN(streamId, self.currentWayPointId + 1, 20)
    streamWriteUIntN(streamId, self.nextWayPointId + 1, 20)
    streamWriteBool(streamId, self.startHelper)
    streamWriteUIntN(streamId, self.usedHelper, 3)
    streamWriteString(streamId, self.driverName)
    streamWriteUInt16(streamId, self.remainingDriveTime)
    streamWriteUIntN(streamId, self.bunkerUnloadType, 3)
    streamWriteBool(streamId, self.automaticUnloadTarget)
    streamWriteBool(streamId, self.automaticPickupTarget)
    streamWriteBool(streamId, self.harversterPairingOk)
    streamWriteUInt8(streamId, self.currentHelperIndex)
    streamWriteUInt8(streamId, self.playerFarmId)
    streamWriteUInt8(streamId, self.actualFarmId)
end

function ADStateModule:doReadStream(streamId)
    self.active = streamReadBool(streamId)
    self.mode = streamReadUIntN(streamId, 4)
    self.firstMarker = ADGraphManager:getMapMarkerById(streamReadUIntN(streamId, 20) - 1)
    self.secondMarker = ADGraphManager:getMapMarkerById(streamReadUIntN(streamId, 20) - 1)
    self.creationMode = streamReadUIntN(streamId, 3)
    self.fillType = streamReadUIntN(streamId, 10)
    self.selectedFillTypes = AutoDrive.streamReadUIntNList(streamId, 10)
    self.loadByFillLevel = streamReadBool(streamId)
    self.loopCounter = streamReadUIntN(streamId, 7)
    self.loopsDone = streamReadUIntN(streamId, 7)
    self.speedLimit = streamReadUIntN(streamId, 8)
    self.fieldSpeedLimit = streamReadUIntN(streamId, 8)
    self.parkDestination = streamReadUIntN(streamId, 20) - 1
    self.parkDestinationAtJobFinished = streamReadUIntN(streamId, 20) - 1
    self.currentDestination = ADGraphManager:getMapMarkerById(streamReadUIntN(streamId, 20) - 1)
    self.currentTaskInfo = streamReadString(streamId)
    self.currentWayPointId = streamReadUIntN(streamId, 20) - 1
    self.nextWayPointId = streamReadUIntN(streamId, 20) - 1
    self.startHelper = streamReadBool(streamId)
    self.usedHelper = streamReadUIntN(streamId, 3)
    self.driverName = streamReadString(streamId)
    self.remainingDriveTime = streamReadUInt16(streamId)
    self.bunkerUnloadType = streamReadUIntN(streamId, 3)
    self.automaticUnloadTarget = streamReadBool(streamId)
    self.automaticPickupTarget = streamReadBool(streamId)
    self.harversterPairingOk = streamReadBool(streamId)
    self.currentHelperIndex = streamReadUInt8(streamId)
    self.playerFarmId = streamReadUInt8(streamId)
    self.actualFarmId = streamReadUInt8(streamId)

    self.currentLocalizedTaskInfo = AutoDrive.localize(self.currentTaskInfo)
end

function ADStateModule:writeStream(streamId)
    self:doWriteStream(streamId)
end

function ADStateModule:readStream(streamId)
    self:doReadStream(streamId)
end

function ADStateModule:writeUpdateStream(streamId)
    self:doWriteStream(streamId)
end

function ADStateModule:readUpdateStream(streamId)
    self:doReadStream(streamId)
end

function ADStateModule:update(dt)
    if self.active == true and g_server ~= nil then
        -- remaining drive time shall be calculated only if AD driving and only on server
        self.calculateRemainingDriveTimeInterval = self.calculateRemainingDriveTimeInterval + dt
        if AutoDrive:getIsEntered(self.vehicle) and (self.calculateRemainingDriveTimeInterval > ADStateModule.CALCULATE_REMAINING_DRIVETIME_INTERVAL) then
            -- performance: calculation if vehicle is entered by any user
            self.calculateRemainingDriveTimeInterval = 0
            self:calculateRemainingDriveTime()
        end
        local remainingDriveTimeInterval = AutoDrive.getSetting("remainingDriveTimeInterval")
        if remainingDriveTimeInterval > 0 and self.calculateRemainingDriveTimeInterval > (remainingDriveTimeInterval * 1000) then
            -- performance: calculation for external mods
            self.calculateRemainingDriveTimeInterval = 0
            self:calculateRemainingDriveTime()
        end
    end

    if self.parkDestination ~= -1 then
        -- transfer park destination to vehicle data as all park destinations are in vehicle data now
        if self.vehicle.advd ~= nil then
            self.vehicle.advd:setParkDestination(self.vehicle, self.parkDestination, false)
            self.parkDestination = -1
        end
    end

    if g_server ~= nil then
        if self.vehicle.ad.isRegisterdHarvester or (self:getMode() == AutoDrive.MODE_UNLOAD and self.active) then
            if self.vehicle.ad.isRegisterdHarvester then
                if ADHarvestManager:hasHarvesterPotentialUnloaders(self.vehicle) ~= self.harversterPairingOk then
                    self:setHarvesterPairingOk(not self.harversterPairingOk)
                end
            else
                if ADHarvestManager:hasVehiclePotentialHarvesters(self.vehicle) ~= self.harversterPairingOk then
                    self:setHarvesterPairingOk(not self.harversterPairingOk)
                end
            end
        end
    end

    if g_client ~= nil and AutoDrive:getIsEntered(self.vehicle) and AutoDrive.getDebugChannelIsSet(AutoDrive.DC_VEHICLEINFO) then
		-- debug output only displayed on client with entered vehicle
        local debug = {}
        debug.active = self.active
        debug.mode = self.mode
        if self.firstMarker ~= nil then
            debug.firstMarker = self.firstMarker.name
        end
        if self.secondMarker ~= nil then
            debug.secondMarker = self.secondMarker.name
        end
        debug.creationMode = self.creationMode
        debug.fillType = self.fillType
        debug.loopCounter = self.loopCounter
        debug.loopsDone = self.loopsDone
        debug.speedLimit = self.speedLimit
        debug.fieldSpeedLimit = self.fieldSpeedLimit
        debug.parkDestination = self.parkDestination
        debug.parkDestinationAtJobFinished = self.parkDestinationAtJobFinished
        if self.currentDestination ~= nil then
            debug.currentDestination = self.currentDestination.name
        end
        debug.currentTaskInfo = self.currentTaskInfo
        debug.currentLocalizedTaskInfo = self.currentLocalizedTaskInfo
        debug.currentWayPointId = self.currentWayPointId
        debug.nextWayPointId = self.nextWayPointId
        debug.startHelper = self.startHelper
        debug.usedHelper = self.usedHelper
        debug.driverName = self.driverName
        debug.remainingDriveTime = self.remainingDriveTime
        if self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD].combine ~= nil then
            debug.combine = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD].combine:getName()
        else
            debug.combine = "-"
        end
        if ADHarvestManager:getAssignedUnloader(self.vehicle) ~= nil then
            debug.unloader = ADHarvestManager:getAssignedUnloader(self.vehicle):getName()
        else
            debug.unloader = "-"
        end
        if self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getFollowingUnloader() ~= nil then
            debug.follower = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getFollowingUnloader():getName()
        else
            debug.follower = "-"
        end
        AutoDrive.renderTable(0.4, 0.4, 0.014, debug)
    end
end

function ADStateModule:toggleStartHelper()
    self.startHelper = not self.startHelper
    self:raiseDirtyFlag()
end

function ADStateModule:setStartHelper(enabled)
    if enabled ~= self.startHelper then
        self.startHelper = enabled
        self:raiseDirtyFlag()
    end
end

function ADStateModule:getStartHelper()
    return self.startHelper
end

function ADStateModule.getNextHelperType(helper)
    -- AI -> CP -> AIVE -> AI ...
    if helper == ADStateModule.HELPER_AI then
        return ADStateModule.HELPER_CP
    elseif helper == ADStateModule.HELPER_CP then
        return ADStateModule.HELPER_AIVE
    elseif helper == ADStateModule.HELPER_AIVE then
        return ADStateModule.HELPER_AI
    else
        return ADStateModule.HELPER_NONE
    end
end

function ADStateModule:isHelperTypeValid(helper)
    if helper == ADStateModule.HELPER_NONE then
        return true
    end
    if helper == ADStateModule.HELPER_AI and self.vehicle.getLastJob then
        return true
    end
    if helper == ADStateModule.HELPER_CP and self.vehicle.cpStartStopDriver then
        return true
    end
    if helper == ADStateModule.HELPER_AIVE and self.vehicle.acParameters then
        return true
    end
    return false
end

function ADStateModule:toggleUsedHelper()
    local helper = self.usedHelper
    for _ = 1, ADStateModule.NUM_HELPER_TYPES do
        helper = self.getNextHelperType(helper)
        if self:isHelperTypeValid(helper) then
            break
        end
    end
    if not self:isHelperTypeValid(helper) then
        helper = ADStateModule.HELPER_NONE
    end
    if helper ~= self.usedHelper then
        self.usedHelper = helper
        self:raiseDirtyFlag()
    end
end

function ADStateModule:getUsedHelper()
    return self.usedHelper
end

-- define helpers which may be restarted
function ADStateModule:getCanRestartHelper()
    return self:getStartHelper() and (self:getUsedHelper() == ADStateModule.HELPER_AI
        or self:getUsedHelper() == ADStateModule.HELPER_CP)
end

function ADStateModule:toggleAutomaticUnloadTarget()
    if self.vehicle.spec_locomotive then
        return
    end
    if AutoDrive.automaticUnloadTarget then
        self.automaticUnloadTarget = not self.automaticUnloadTarget
        self:raiseDirtyFlag()
    end
end

function ADStateModule:setAutomaticUnloadTarget(enabled)
    if self.vehicle.spec_locomotive then
        return
    end
    if AutoDrive.automaticUnloadTarget then
        if enabled ~= self.automaticUnloadTarget then
            self.automaticUnloadTarget = enabled
            self:raiseDirtyFlag()
        end
    end
end

function ADStateModule:getAutomaticUnloadTarget()
    if AutoDrive.automaticUnloadTarget then
        return self.automaticUnloadTarget and not self.vehicle.spec_locomotive
    else
        return false
    end
end

function ADStateModule:toggleAutomaticPickupTarget()
    if self.vehicle.spec_locomotive then
        return
    end
    if AutoDrive.automaticPickupTarget then
        self.automaticPickupTarget = not self.automaticPickupTarget
        self:raiseDirtyFlag()
    end
end

function ADStateModule:setAutomaticPickupTarget(enabled)
    if self.vehicle.spec_locomotive then
        return
    end
    if AutoDrive.automaticPickupTarget then
        if enabled ~= self.automaticPickupTarget then
            self.automaticPickupTarget = enabled
            self:raiseDirtyFlag()
        end
    end
end

function ADStateModule:getAutomaticPickupTarget()
    if AutoDrive.automaticPickupTarget then
        return self.automaticPickupTarget and not self.vehicle.spec_locomotive
    else
        return false
    end
end

function ADStateModule:setHarvesterPairingOk(ok)
    if ok ~= self.harversterPairingOk then
        self.harversterPairingOk = ok
        self:raiseDirtyFlag()
    end
end

function ADStateModule:getHarvesterPairingOk()
    return self.harversterPairingOk
end

function ADStateModule:getCurrentWayPointId()
    return self.currentWayPointId
end

function ADStateModule:setCurrentWayPointId(wayPointId)
    if wayPointId ~= self.currentWayPointId then
        self.currentWayPointId = wayPointId
        self:raiseDirtyFlag()
    end
end

function ADStateModule:getCurrentWayPoint()
    return ADGraphManager:getWayPointById(self.currentWayPointId)
end

function ADStateModule:getNextWayPointId()
    return self.nextWayPointId
end

function ADStateModule:setNextWayPointId(wayPointId)
    if wayPointId ~= self.nextWayPointId then
        self.nextWayPointId = wayPointId
        self:raiseDirtyFlag()
    end
end

function ADStateModule:getNextWayPoint()
    if self.nextWayPointId > 1 then
        return ADGraphManager:getWayPointById(self.nextWayPointId)
    end
    return nil
end

function ADStateModule:getCurrentTaskInfo()
    return self.currentTaskInfo
end

function ADStateModule:getCurrentLocalizedTaskInfo()
    return self.currentLocalizedTaskInfo
end

function ADStateModule:setCurrentTaskInfo(text)
    if text ~= nil and text ~= self.currentTaskInfo then
        self.currentTaskInfo = text
        self.currentLocalizedTaskInfo = AutoDrive.localize(text)
        self:raiseDirtyFlag()
    end
end

function ADStateModule:getCurrentDestination()
    return self.currentDestination
end

function ADStateModule:getCurrentDestinationId()
    if self.currentDestination ~= nil then
        return self.currentDestination.markerIndex
    end
    return -1
end

function ADStateModule:setCurrentDestination(marker)
    self.currentDestination = marker
    self:raiseDirtyFlag()
end

function ADStateModule:getMode()
    return self.mode
end

function ADStateModule:getCurrentMode()
    return self.vehicle.ad.modes[self.mode]
end

function ADStateModule:nextMode()
    if self.mode < ADStateModule.HIGHEST_MODE then
        self.mode = self.mode + 1
        if self.vehicle.spec_locomotive and self.mode == AutoDrive.MODE_UNLOAD then
            -- skip harvester mode for train
            self.mode = self.mode + 1
            if self.mode >= ADStateModule.HIGHEST_MODE then
                self.mode = AutoDrive.MODE_DRIVETO
            end
        end
    else
        self.mode = AutoDrive.MODE_DRIVETO
    end
    self:setAutomaticPickupTarget(false) -- disable automatic target on mode change
    self:setAutomaticUnloadTarget(false) -- disable automatic target on mode change
    AutoDrive.Hud.lastUIScale = 0
    self:raiseDirtyFlag()
end

function ADStateModule:previousMode()
    if self.mode > AutoDrive.MODE_DRIVETO then
        self.mode = self.mode - 1
        if self.vehicle.spec_locomotive and self.mode == AutoDrive.MODE_UNLOAD then
            -- skip harvester mode for train
            self.mode = self.mode - 1
            if self.mode <= ADStateModule.MODE_DRIVETO then
                self.mode = AutoDrive.HIGHEST_MODE
            end
        end
    else
        self.mode = ADStateModule.HIGHEST_MODE
    end
    self:setAutomaticPickupTarget(false) -- disable automatic target on mode change
    self:setAutomaticUnloadTarget(false) -- disable automatic target on mode change
    AutoDrive.Hud.lastUIScale = 0
    self:raiseDirtyFlag()
end

function ADStateModule:setMode(newMode)
    if self.vehicle.spec_locomotive and newMode == AutoDrive.MODE_UNLOAD then
        -- skip harvester mode for train
        return
    elseif newMode >= AutoDrive.MODE_DRIVETO and newMode <= ADStateModule.HIGHEST_MODE and newMode ~= self.mode then
        self.mode = newMode
        self:setAutomaticPickupTarget(false) -- disable automatic target on mode change
        self:setAutomaticUnloadTarget(false) -- disable automatic target on mode change
        AutoDrive.Hud.lastUIScale = 0
        self:raiseDirtyFlag()
    end
end

function ADStateModule:isActive()
    return self.active
end

function ADStateModule:setActive(active)
    self.remainingDriveTime = 0
    if active ~= self.active then
        self.active = active
        self:raiseDirtyFlag()
    end

    if self.active then
        self.creationMode = ADStateModule.CREATE_OFF
        self:raiseDirtyFlag()
    end
end

function ADStateModule:isInCreationMode()
    return (self.creationMode ~= ADStateModule.CREATE_OFF)
end

function ADStateModule:isInNormalCreationMode()
    return self.creationMode == ADStateModule.CREATE_NORMAL
end

function ADStateModule:isInDualCreationMode()
    return self.creationMode == ADStateModule.CREATE_DUAL
end

function ADStateModule:isInSubPrioCreationMode()
    return self.creationMode == ADStateModule.CREATE_SUB_PRIO
end

function ADStateModule:isInSubPrioDualCreationMode()
    return self.creationMode == ADStateModule.CREATE_SUB_PRIO_DUAL
end

function ADStateModule:isInNormalTwoWayCreationMode()
    return self.creationMode == ADStateModule.CREATE_NORMAL_TWOWAY
end

function ADStateModule:isInDualTwoWayCreationMode()
    return self.creationMode == ADStateModule.CREATE_DUAL_TWOWAY
end

function ADStateModule:isInSubPrioTwoWayCreationMode()
    return self.creationMode == ADStateModule.CREATE_SUB_PRIO_TWOWAY
end

function ADStateModule:isInSubPrioDualTwoWayCreationMode()
    return self.creationMode == ADStateModule.CREATE_SUB_PRIO_DUAL_TWOWAY
end

function ADStateModule:disableCreationMode()
    self.creationMode = ADStateModule.CREATE_OFF
    self:raiseDirtyFlag()
    if self.vehicle.ad.recordingModule ~= nil then
        self.vehicle.ad.recordingModule:stop()
    end
end

function ADStateModule:startNormalCreationMode()
    self.creationMode = ADStateModule.CREATE_NORMAL
    self:raiseDirtyFlag()
    if self.vehicle.ad.recordingModule ~= nil then
        self.vehicle.ad.recordingModule:start(false, false, false)
    end
end

function ADStateModule:startDualCreationMode()
    self.creationMode = ADStateModule.CREATE_DUAL
    self:raiseDirtyFlag()
    if self.vehicle.ad.recordingModule ~= nil then
        self.vehicle.ad.recordingModule:start(true, false, false)
    end
end

function ADStateModule:startSubPrioCreationMode()
    self.creationMode = ADStateModule.CREATE_SUB_PRIO
    self:raiseDirtyFlag()
    if self.vehicle.ad.recordingModule ~= nil then
        self.vehicle.ad.recordingModule:start(false, true, false)
    end
end

function ADStateModule:startSubPrioDualCreationMode()
    self.creationMode = ADStateModule.CREATE_SUB_PRIO_DUAL
    self:raiseDirtyFlag()
    if self.vehicle.ad.recordingModule ~= nil then
        self.vehicle.ad.recordingModule:start(true, true, false)
    end
end

function ADStateModule:startNormalTwoWayCreationMode()
    self.creationMode = ADStateModule.CREATE_NORMAL_TWOWAY
    self:raiseDirtyFlag()
    if self.vehicle.ad.recordingModule ~= nil then
        self.vehicle.ad.recordingModule:start(false, false, true)
    end
end

function ADStateModule:startDualTwoWayCreationMode()
    self.creationMode = ADStateModule.CREATE_DUAL_TWOWAY
    self:raiseDirtyFlag()
    if self.vehicle.ad.recordingModule ~= nil then
        self.vehicle.ad.recordingModule:start(true, false, true)
    end
end

function ADStateModule:startSubPrioTwoWayCreationMode()
    self.creationMode = ADStateModule.CREATE_SUB_PRIO_TWOWAY
    self:raiseDirtyFlag()
    if self.vehicle.ad.recordingModule ~= nil then
        self.vehicle.ad.recordingModule:start(false, true, true)
    end
end

function ADStateModule:startSubPrioDualTwoWayCreationMode()
    self.creationMode = ADStateModule.CREATE_SUB_PRIO_DUAL_TWOWAY
    self:raiseDirtyFlag()
    if self.vehicle.ad.recordingModule ~= nil then
        self.vehicle.ad.recordingModule:start(true, true, true)
    end
end

function ADStateModule:getLoopCounter()
    return self.loopCounter
end

function ADStateModule:changeLoopCounter(increment, fast, wheel)
    local newCounter = self.loopCounter
    local delta = fast and 10 or 1
    if increment then
        newCounter = newCounter + delta
        if newCounter >= 100 then
            if fast or wheel then
                newCounter = 99
            else
                newCounter = newCounter % 100
            end
        end
    else
        newCounter = newCounter - delta
        if newCounter < 0 then
            if fast or wheel then
                newCounter = 0
            else
                newCounter = (newCounter + 100) % 100
            end
        end
    end
    self.loopCounter = newCounter
    self:raiseDirtyFlag()
end

function ADStateModule:getLoopsDone()
    return self.loopsDone
end

function ADStateModule:setLoopsDone(loopsDone)
    self.loopsDone = loopsDone
    self:raiseDirtyFlag()
end

function ADStateModule:setName(newName)
    self.driverName = newName
end

function ADStateModule:getName()
    return self.driverName
end

function ADStateModule:getFirstMarker()
    return self.firstMarker
end

function ADStateModule:getFirstMarkerId()
    if self.firstMarker ~= nil then
        return self.firstMarker.markerIndex
    else
        return -1
    end
end

function ADStateModule:getFirstWayPoint()
    if self.firstMarker ~= nil then
        return self.firstMarker.id
    else
        return -1
    end
end

function ADStateModule:getFirstMarkerName()
    if self.firstMarker ~= nil then
        return self.firstMarker.name
    else
        return nil
    end
end

function ADStateModule:setFirstMarker(markerId)
    self.firstMarker = ADGraphManager:getMapMarkerById(markerId)
    self:raiseDirtyFlag()
end

function ADStateModule:setFirstMarkerByWayPointId(wayPointId)
    for markerId, mapMarker in pairs(ADGraphManager:getMapMarkers()) do
        if mapMarker.id == wayPointId then
            self:setFirstMarker(markerId)
            break
        end
    end
end

function ADStateModule:setFirstMarkerByName(markerName)
    for markerId, mapMarker in pairs(ADGraphManager:getMapMarkers()) do
        if mapMarker.name == markerName then
            self:setFirstMarker(markerId)
            break
        end
    end
end

function ADStateModule:getSecondMarker()
    return self.secondMarker
end

function ADStateModule:getSecondMarkerId()
    if self.secondMarker ~= nil then
        return self.secondMarker.markerIndex
    else
        return -1
    end
end

function ADStateModule:getSecondWayPoint()
    if self.secondMarker ~= nil then
        return self.secondMarker.id
    else
        return -1
    end
end

function ADStateModule:getSecondMarkerName()
    if self.secondMarker ~= nil then
        return self.secondMarker.name
    else
        return nil
    end
end

function ADStateModule:setSecondMarker(markerId)
    self.secondMarker = ADGraphManager:getMapMarkerById(markerId)
    self:raiseDirtyFlag()
end

function ADStateModule:setSecondMarkerByWayPointId(wayPointId)
    for markerId, mapMarker in pairs(ADGraphManager:getMapMarkers()) do
        if mapMarker.id == wayPointId then
            self:setSecondMarker(markerId)
            break
        end
    end
end

function ADStateModule:setSecondMarkerByName(markerName)
    for markerId, mapMarker in pairs(ADGraphManager:getMapMarkers()) do
        if mapMarker.name == markerName then
            self:setSecondMarker(markerId)
            break
        end
    end
end

function ADStateModule:getFillType()
    return self.fillType
end

function ADStateModule:getSelectedFillTypes()
    return self.selectedFillTypes
end

function ADStateModule:setFillType(fillType)
    if fillType > 0 and self.fillType ~= fillType then
        self.fillType = fillType
        AutoDrive:setALFillType(self.vehicle, fillType)
        if not table.contains(self.selectedFillTypes, fillType) then
            self.selectedFillTypes = {fillType}
        end
        self:raiseDirtyFlag()
    end
end

function ADStateModule:toggleFillTypeSelection(fillType)
    if fillType > 0 and self.fillType ~= AutoDrive.UAL_FILLTYPE_ALL and fillType ~= AutoDrive.UAL_FILLTYPE_ALL then 
        -- AutoDrive.UAL_FILLTYPE_ALL is for all fillTypes in UAL
        -- do not toggle UAL_FILLTYPE_ALL or additional fillTypes to UAL_FILLTYPE_ALL
        if table.contains(self.selectedFillTypes, fillType) then
            table.removeValue(self.selectedFillTypes, fillType)
            if self.fillType == fillType and #self.selectedFillTypes > 0 then
                -- the deselected filltype was the active filltype -> select the first remaining item
                self.fillType = self.selectedFillTypes[1]
            end
        else
            table.insert(self.selectedFillTypes, fillType)
            if not table.contains(self.selectedFillTypes, self.fillType) then
                -- selectedFillTypes was empty, select the new fillType
                self.fillType = fillType
            end
        end
        self:raiseDirtyFlag()
    end
end

function ADStateModule:toggleAllFillTypeSelections(fillType)
    if fillType > 0 then
        local supportedFillTypes = AutoDrive.getSupportedFillTypesOfAllUnitsAlphabetically(self.vehicle, AutoDrive.UAL_FILLTYPE_ALL)
        -- do not toggle fillType AutoDrive.UAL_FILLTYPE_ALL!
        if supportedFillTypes and #supportedFillTypes > 0 then
            for _, selected in pairs(supportedFillTypes) do
                if not table.contains(self.selectedFillTypes, selected) then
                    -- at least one supported fillType not yet selected. Select all
                    self.fillType = selected
                    self.selectedFillTypes = supportedFillTypes
                    self:raiseDirtyFlag()
                    return
                end
            end
            -- all fillTypes selected - clear selection and only select the given item
            if fillType == AutoDrive.UAL_FILLTYPE_ALL then
                self.selectedFillTypes = {}
            else
                self.selectedFillTypes = {fillType}
            end
            self.fillType = fillType
            self:raiseDirtyFlag()
        end
    end
end


function ADStateModule:toggleLoadByFillLevel()
    self.loadByFillLevel = not self.loadByFillLevel
    self:raiseDirtyFlag()
end

function ADStateModule:setLoadByFillLevel(enabled)
    self.loadByFillLevel = enabled
    self:raiseDirtyFlag()
end

function ADStateModule:getLoadByFillLevel()
    return self.loadByFillLevel
end


function ADStateModule:selectPreferredFillTypeFromFillLevels(fillLevels)
    if #self.selectedFillTypes == 0 then
        return
    end

    local fillLevelList = {}  -- get a list of fill levels
    for _, fillLevel in pairs(fillLevels) do
        table.insert(fillLevelList, fillLevel)
    end
    table.sort(fillLevelList)  -- sort it
    local requiredFillLevel = fillLevelList[#fillLevelList]
    local idx = table.indexOf(self.selectedFillTypes, self.fillType) or 0 -- starting point
    local loopsLeft = #self.selectedFillTypes
    local pickNextNonEmpty = requiredFillLevel == -1 or not self.loadByFillLevel
    if idx == nil or requiredFillLevel == nil then
        return
    end
    if pickNextNonEmpty  then
        -- infinite trigger (all fill levels are -1) or load-by-fill-level diabled: pick the next available filltype
        idx = (idx % #self.selectedFillTypes) + 1
    end
    while true do
        local fillType = self.selectedFillTypes[idx]
        if fillLevels[fillType] ~= nil and fillLevels[fillType] ~= 0 and (fillLevels[fillType] == requiredFillLevel or pickNextNonEmpty) then
            -- found suitable filltype
            self.fillType = fillType
            break
        end

        idx = (idx % #self.selectedFillTypes) + 1
        loopsLeft = loopsLeft - 1
        if loopsLeft <= 0 then
            break
        end
    end
end

function ADStateModule:nextSelectedFillType()
    if self.selectedFillTypes and #self.selectedFillTypes > 0 then
        for index, fillType in ipairs(self.selectedFillTypes) do
            if self.fillType == fillType then
                if self.selectedFillTypes[index + 1] ~= nil and g_fillTypeManager:getFillTypeByIndex(self.selectedFillTypes[index + 1]) ~= nil then
                    -- found valid next selectedFillType
                    self.fillType = self.selectedFillTypes[index + 1]
                    AutoDrive:setALFillType(self.vehicle, self.selectedFillTypes[index + 1])
                else
                    if self.selectedFillTypes[1] ~= nil and g_fillTypeManager:getFillTypeByIndex(self.selectedFillTypes[1]) ~= nil then
                        -- select the first selectedFillType
                        self.fillType = self.selectedFillTypes[1]
                        AutoDrive:setALFillType(self.vehicle, self.selectedFillTypes[1])
                    end
                end
                break
            end
        end
        self:raiseDirtyFlag()
    end
end

function ADStateModule:nextFillType()
    local supportedFillTypes = AutoDrive.getSupportedFillTypesOfAllUnitsAlphabetically(self.vehicle)
    if supportedFillTypes and #supportedFillTypes > 0 then
        for index, fillType in ipairs(supportedFillTypes) do
            if self.fillType == fillType then
                if supportedFillTypes[index + 1] ~= nil and g_fillTypeManager:getFillTypeByIndex(supportedFillTypes[index + 1]) ~= nil then
                    -- found valid next supported fillType
                    self.fillType = supportedFillTypes[index + 1]
                else
                    if supportedFillTypes[1] ~= nil and g_fillTypeManager:getFillTypeByIndex(supportedFillTypes[1]) ~= nil then
                        -- select the first supported fillType
                        self.fillType = supportedFillTypes[1]
                    end
                end
                break
            end
        end
        self:raiseDirtyFlag()
    end
end

function ADStateModule:previousFillType()
    local supportedFillTypes = AutoDrive.getSupportedFillTypesOfAllUnitsAlphabetically(self.vehicle)
    if supportedFillTypes and #supportedFillTypes > 0 then
        for index, fillType in ipairs(supportedFillTypes) do
            if self.fillType == fillType then
                if index > 1 and supportedFillTypes[index - 1] ~= nil and g_fillTypeManager:getFillTypeByIndex(supportedFillTypes[index - 1]) ~= nil then
                    -- found valid previous supported fillType
                    self.fillType = supportedFillTypes[index - 1]
                else
                    if supportedFillTypes[#supportedFillTypes] ~= nil and g_fillTypeManager:getFillTypeByIndex(supportedFillTypes[#supportedFillTypes]) ~= nil then
                        -- select the last supported fillType
                        self.fillType = supportedFillTypes[#supportedFillTypes]
                    end
                end
                break
            end
        end
        self:raiseDirtyFlag()
    end
end

function ADStateModule:getSpeedLimit()
    return self.speedLimit
end

function ADStateModule:increaseSpeedLimit()
    if self.speedLimit < AutoDrive.getVehicleMaxSpeed(self.vehicle) then
        self.speedLimit = self.speedLimit + 1
    end
    self:raiseDirtyFlag()
end

function ADStateModule:decreaseSpeedLimit()
    if self.speedLimit > 2 then
        self.speedLimit = self.speedLimit - 1
    end
    self:raiseDirtyFlag()
end

function ADStateModule:getFieldSpeedLimit()
    return self.fieldSpeedLimit
end

function ADStateModule:increaseFieldSpeedLimit()
    if self.fieldSpeedLimit < AutoDrive.getVehicleMaxSpeed(self.vehicle) then
        self.fieldSpeedLimit = self.fieldSpeedLimit + 1
    end
    self:raiseDirtyFlag()
end

function ADStateModule:decreaseFieldSpeedLimit()
    if self.fieldSpeedLimit > 2 then
        self.fieldSpeedLimit = self.fieldSpeedLimit - 1
    end
    self:raiseDirtyFlag()
end

function ADStateModule:getParkDestinationAtJobFinished()
    return self.parkDestinationAtJobFinished
end

function ADStateModule:setParkDestinationAtJobFinished(parkDestination)
    self.parkDestinationAtJobFinished = parkDestination
    self:raiseDirtyFlag()
end

function ADStateModule:getSelectedNeighbourPoint()
    if not self.pointToNeighbour then
        return nil
    end
    return self.neighbourPoints[self.currentNeighbourToPointAt]
end

function ADStateModule:togglePointToNeighbor()
    self.pointToNeighbour = not self.pointToNeighbour
    if self.pointToNeighbour then
        self:updateNeighborPoint()
    end
end

function ADStateModule:changeNeighborPoint(increase)
    self.currentNeighbourToPointAt = self.currentNeighbourToPointAt + increase
    if self.currentNeighbourToPointAt < 1 then
        self.currentNeighbourToPointAt = #self.neighbourPoints
    end
    if self.neighbourPoints[self.currentNeighbourToPointAt] == nil then
        self.currentNeighbourToPointAt = 1
    end
end

function ADStateModule:updateNeighborPoint()
    -- Find all candidate points, no further away than 15 units from vehicle
    local candidateNeighborPoints =
        table.f_filter(
        self.vehicle:getWayPointsDistance(),
        function(elem)
            return elem.distance <= 15
        end
    )

    -- If more than one point found, then arrange them from inner closest to further out
    if #candidateNeighborPoints > 1 then
        -- Sort by distance
        table.sort(
            candidateNeighborPoints,
            function(left, right)
                return left.distance < right.distance
            end
        )
        -- Clear the array for any previous 'points'
        self.neighbourPoints = {}
        -- Only need 'point' in the neighbourPoints-array
        for _, elem in pairs(candidateNeighborPoints) do
            table.insert(self.neighbourPoints, elem.wayPoint)
        end
        -- Begin at the 2nd closest one (assuming 1st is 'ourself / the closest')
        self.currentNeighbourToPointAt = 2

        -- But try to find a node with no IncomingRoads, and use that as starting from
        for idx, point in pairs(self.neighbourPoints) do
            if #point.incoming < 1 then
                self.currentNeighbourToPointAt = idx
                break -- Since array was already sorted by distance, we dont need to search for another one
            end
        end
    end
end

function ADStateModule:raiseDirtyFlag()
    self.vehicle:raiseDirtyFlags(self.vehicle.ad.dirtyFlag)
end

function ADStateModule:setNextTargetInFolder()
    local group = self.secondMarker.group
    if group ~= "All" then
        local nextMarkerInGroup = nil
        local markerSeen = false
        local firstMarkerInGroup = nil
        for _, marker in ipairs(ADGraphManager:getMapMarkersInGroup(group)) do
            if marker.group == group then
                if firstMarkerInGroup == nil then
                    firstMarkerInGroup = marker.markerIndex
                end

                if markerSeen and nextMarkerInGroup == nil then
                    nextMarkerInGroup = marker.markerIndex
                end

                if marker.markerIndex == self.secondMarker.markerIndex then
                    markerSeen = true
                end
            end
        end

        local markerToSet = self.secondMarker
        if nextMarkerInGroup ~= nil then
            markerToSet = nextMarkerInGroup
        elseif firstMarkerInGroup ~= nil then
            markerToSet = firstMarkerInGroup
        end

        self:setSecondMarker(markerToSet)
        AutoDrive.Hud.lastUIScale = 0
    end
end

function ADStateModule:resetMarkersOnReload()
    local newFirstMarker = nil
    if self.firstMarker ~= nil and self.firstMarker.id ~= nil then
        newFirstMarker = ADGraphManager:getMapMarkerByWayPointId(self.firstMarker.id)
    end
    if newFirstMarker ~= nil then
        self.firstMarker = newFirstMarker
    else
        self.firstMarker = ADGraphManager:getMapMarkerById(1)
    end

    local newSecondMarker = nil
    if self.secondMarker ~= nil and self.secondMarker.id ~= nil then
        newSecondMarker = ADGraphManager:getMapMarkerByWayPointId(self.secondMarker.id)
    end
    if newSecondMarker ~= nil then
        self.secondMarker = newSecondMarker
    else
        self.secondMarker = ADGraphManager:getMapMarkerById(1)
    end
    self:raiseDirtyFlag()
end

function ADStateModule:calculateRemainingDriveTime()
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    if not AutoDrive.checkIsOnField(x, y, z) then
		local wp, currentWayPoint = self.vehicle.ad.drivePathModule:getWayPoints()
		if wp ~= nil and currentWayPoint > 0 then
			self.remainingDriveTime = ADGraphManager:getDriveTimeForWaypoints(wp, currentWayPoint, math.min((self.vehicle.spec_motorized.motor.maxForwardSpeed * 3.6), self:getSpeedLimit()))
		else
			self.remainingDriveTime = 0
		end
	else
		self.remainingDriveTime = 0
	end
	self:raiseDirtyFlag()
end

function ADStateModule:getRemainingDriveTime()
    -- AD internal use
	return self.remainingDriveTime
end

function ADStateModule:adGetRemainingDriveTime()
    -- for external mods
    if AutoDrive.getSetting("remainingDriveTimeInterval") > 0 then
        return self.remainingDriveTime
    else
        return 0
    end
end


function ADStateModule:nextBunkerUnloadType()
    if self.bunkerUnloadType < ADStateModule.BUNKER_UNLOAD_TRAILER then
        self.bunkerUnloadType = self.bunkerUnloadType + 1
    else
        self.bunkerUnloadType = ADStateModule.BUNKER_UNLOAD_TRIGGER
    end
    self:raiseDirtyFlag()
end

function ADStateModule:getBunkerUnloadType()
    return self.bunkerUnloadType
end

function ADStateModule:getBunkerUnloadTypeIsTrigger()
    return self.bunkerUnloadType == ADStateModule.BUNKER_UNLOAD_TRIGGER
end

function ADStateModule:getCurrentHelperIndex()
    return self.currentHelperIndex
end

function ADStateModule:setCurrentHelperIndex(currentHelperIndex)
    if self.currentHelperIndex ~= currentHelperIndex then
        self.currentHelperIndex = currentHelperIndex
        self:raiseDirtyFlag()
    end
end

function ADStateModule:getPlayerFarmId(farmId)
    return self.playerFarmId
end

function ADStateModule:setPlayerFarmId(farmId, sendEvent)
    if farmId and self.playerFarmId ~= farmId then
        self.playerFarmId = farmId
        if sendEvent == nil or sendEvent == true then
            self:raiseDirtyFlag()
        end
    end
end

function ADStateModule:getActualFarmId(farmId)
    return self.actualFarmId
end

function ADStateModule:setActualFarmId(farmId, sendEvent)
    if farmId and self.actualFarmId ~= farmId then
        self.actualFarmId = farmId
        if sendEvent == nil or sendEvent == true then
            self:raiseDirtyFlag()
        end
    end
end
