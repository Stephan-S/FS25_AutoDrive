function AutoDrive.prerequisitesPresent(specializations)
    return (SpecializationUtil.hasSpecialization(AIVehicle, specializations)
    and SpecializationUtil.hasSpecialization(Motorized, specializations)
    and SpecializationUtil.hasSpecialization(Drivable, specializations)
    and SpecializationUtil.hasSpecialization(Enterable, specializations)
    )
    or
-- locomotive
    (
    SpecializationUtil.hasSpecialization(SplineVehicle, specializations)
    and SpecializationUtil.hasSpecialization(Drivable, specializations)
    and SpecializationUtil.hasSpecialization(Locomotive, specializations)
    )
end

function AutoDrive.registerEventListeners(vehicleType)
    for _, n in pairs(
        {
            "onUpdate",
            "onRegisterActionEvents",
            "onDelete",
            "onDrawUIInfo",
            "onPreLoad",
            "onPostLoad",
            "onLoad",
            "saveToXMLFile",
            "onReadStream",
            "onWriteStream",
            "onReadUpdateStream",
            "onWriteUpdateStream",
            "onUpdateTick",
            "onStartAutoDrive",
            "onStopAutoDrive",
            "onPostAttachImplement",
            "onPreDetachImplement",
            "onPostDetachImplement",
            "onEnterVehicle",
            "onLeaveVehicle",
            "onSetBroken",
            -- CP events, see ExternalInterface.lua
            "onCpFinished",
            "onCpEmpty",
            "onCpFull",
            "onCpFuelEmpty",
            "onCpBroken",
            -- Giants helper events
            "onAIJobFinished",
        }
    ) do
        SpecializationUtil.registerEventListener(vehicleType, n, AutoDrive)
    end
end

function AutoDrive.registerOverwrittenFunctions(vehicleType)
    SpecializationUtil.registerOverwrittenFunction(vehicleType, "getCanMotorRun",                       AutoDrive.getCanMotorRun)
    SpecializationUtil.registerOverwrittenFunction(vehicleType, "getIsAIActive",                        AutoDrive.getIsAIActive)
    SpecializationUtil.registerOverwrittenFunction(vehicleType, "getIsVehicleControlledByPlayer",       AutoDrive.getIsVehicleControlledByPlayer)
    SpecializationUtil.registerOverwrittenFunction(vehicleType, "getActiveFarm",                        AutoDrive.getActiveFarm)

    -- Disables click to switch, if the user clicks on the hud or the editor mode is active.
    -- see ExternalInterface.lua
    if vehicleType.functions["enterVehicleRaycastClickToSwitch"] ~= nil then
        SpecializationUtil.registerOverwrittenFunction(vehicleType, "enterVehicleRaycastClickToSwitch", AutoDrive.enterVehicleRaycastClickToSwitch)
    end
end

function AutoDrive.registerFunctions(vehicleType)
    SpecializationUtil.registerFunction(vehicleType, "startAutoDrive", AutoDrive.startAutoDrive)
    SpecializationUtil.registerFunction(vehicleType, "stopAutoDrive", AutoDrive.stopAutoDrive)
    SpecializationUtil.registerFunction(vehicleType, "toggleMouse", AutoDrive.toggleMouse)
    SpecializationUtil.registerFunction(vehicleType, "updateWayPointsDistance", AutoDrive.updateWayPointsDistance)
    SpecializationUtil.registerFunction(vehicleType, "resetClosestWayPoint", AutoDrive.resetClosestWayPoint)
    SpecializationUtil.registerFunction(vehicleType, "resetWayPointsDistance", AutoDrive.resetWayPointsDistance)
    SpecializationUtil.registerFunction(vehicleType, "getWayPointsDistance", AutoDrive.getWayPointsDistance)
    SpecializationUtil.registerFunction(vehicleType, "getClosestWayPoint", AutoDrive.getClosestWayPoint)
    SpecializationUtil.registerFunction(vehicleType, "getClosestNotReversedWayPoint", AutoDrive.getClosestNotReversedWayPoint)
    SpecializationUtil.registerFunction(vehicleType, "getWayPointsInRange", AutoDrive.getWayPointsInRange)
    SpecializationUtil.registerFunction(vehicleType, "getWayPointIdsInRange", AutoDrive.getWayPointIdsInRange)
    SpecializationUtil.registerFunction(vehicleType, "onDrawEditorMode", AutoDrive.onDrawEditorMode)
    SpecializationUtil.registerFunction(vehicleType, "onDrawPreviews", AutoDrive.onDrawPreviews)
    SpecializationUtil.registerFunction(vehicleType, "updateClosestWayPoint", AutoDrive.updateClosestWayPoint)
    SpecializationUtil.registerFunction(vehicleType, "collisionTestCallback", AutoDrive.collisionTestCallback)
    SpecializationUtil.registerFunction(vehicleType, "generateUTurn", AutoDrive.generateUTurn)
    SpecializationUtil.registerFunction(vehicleType, "getCanAdTakeControl", AutoDrive.getCanAdTakeControl) -- see ExternalInterface.lua
    SpecializationUtil.registerFunction(vehicleType, "adGetRemainingDriveTime", AutoDrive.adGetRemainingDriveTime)
    SpecializationUtil.registerFunction(vehicleType, "adHasHarvesterAvailableUnloader", ADHarvestManager.adHasHarvesterAvailableUnloader)
end

function AutoDrive.registerEvents(vehicleType)
    SpecializationUtil.registerEvent(vehicleType, "onStartAutoDrive")
    SpecializationUtil.registerEvent(vehicleType, "onStopAutoDrive")
    SpecializationUtil.registerEvent(vehicleType, "onAutoDriveParked")
end

function AutoDrive:onRegisterActionEvents(_, isOnActiveVehicle)
    local registerEvents = isOnActiveVehicle
    if self.ad ~= nil then
        registerEvents = registerEvents or self == AutoDrive.getControlledVehicle()
    end

    -- only in active vehicle
    if registerEvents then
        -- attach our actions
        local _, eventName
        local showF1Help = AutoDrive.getSetting("showHelp")
        for _, action in pairs(ADInputManager.actionsToInputs) do
            _, eventName = InputBinding.registerActionEvent(g_inputBinding, action[1], self, ADInputManager.onActionCall, false, true, false, true)
            if action[5] then
                g_inputBinding:setActionEventTextVisibility(eventName, action[5] and showF1Help)
                if showF1Help then
                    if action[6] then
                        g_inputBinding:setActionEventTextPriority(eventName, action[6])
                    end
                end
            else
                g_inputBinding:setActionEventTextVisibility(eventName, false)
            end
        end
    end
end

function AutoDrive.initSpecialization()
    -- print("Calling AutoDrive initSpecialization")
    local schema = Vehicle.xmlSchema
    schema:setXMLSpecializationType("AutoDrive")

    schema:register(XMLValueType.FLOAT, "vehicle.AutoDrive#followDistance", "Follow distance for harveste unloading", 1)
    schema:setXMLSpecializationType()

    local schemaSavegame = Vehicle.xmlSchemaSavegame

    for settingName, setting in pairs(AutoDrive.settings) do
        if setting.isVehicleSpecific then
            schemaSavegame:register(XMLValueType.INT, "vehicles.vehicle(?).AutoDrive#" .. settingName, setting.text, setting.default)
        end
    end

    schemaSavegame:register(XMLValueType.STRING, "vehicles.vehicle(?).AutoDrive#groups", "groups")
    schemaSavegame:register(XMLValueType.INT, "vehicles.vehicle(?).AutoDrive#mode", "mode")
    schemaSavegame:register(XMLValueType.INT, "vehicles.vehicle(?).AutoDrive#firstMarker", "firstMarker")
    schemaSavegame:register(XMLValueType.INT, "vehicles.vehicle(?).AutoDrive#secondMarker", "secondMarker")
    schemaSavegame:register(XMLValueType.INT, "vehicles.vehicle(?).AutoDrive#fillType", "fillType")
    schemaSavegame:register(XMLValueType.STRING, "vehicles.vehicle(?).AutoDrive#selectedFillTypes", "selectedFillTypes")
    schemaSavegame:register(XMLValueType.BOOL, "vehicles.vehicle(?).AutoDrive#loadByFillLevel", "loadByFillLevel")
    schemaSavegame:register(XMLValueType.INT, "vehicles.vehicle(?).AutoDrive#loopCounter", "loopCounter")
    schemaSavegame:register(XMLValueType.INT, "vehicles.vehicle(?).AutoDrive#loopsDone", "loopsDone")
    schemaSavegame:register(XMLValueType.INT, "vehicles.vehicle(?).AutoDrive#speedLimit", "speedLimit")
    schemaSavegame:register(XMLValueType.INT, "vehicles.vehicle(?).AutoDrive#fieldSpeedLimit", "fieldSpeedLimit")
    schemaSavegame:register(XMLValueType.STRING, "vehicles.vehicle(?).AutoDrive#driverName", "driverName")
    schemaSavegame:register(XMLValueType.BOOL, "vehicles.vehicle(?).AutoDrive#active", "active")
    schemaSavegame:register(XMLValueType.BOOL, "vehicles.vehicle(?).AutoDrive#startHelper", "startHelper")
    schemaSavegame:register(XMLValueType.INT, "vehicles.vehicle(?).AutoDrive#parkDestination", "parkDestination")
    schemaSavegame:register(XMLValueType.INT, "vehicles.vehicle(?).AutoDrive#bunkerUnloadType", "bunkerUnloadType")
    if AutoDrive.automaticUnloadTarget then
        schemaSavegame:register(XMLValueType.BOOL, "vehicles.vehicle(?).AutoDrive#automaticUnloadTarget", "automaticUnloadTarget")
    end
    if AutoDrive.automaticPickupTarget then
        schemaSavegame:register(XMLValueType.BOOL, "vehicles.vehicle(?).AutoDrive#automaticPickupTarget", "automaticPickupTarget")
    end
end

function AutoDrive:onPreLoad(savegame)
    if self.spec_autodrive == nil then
        self.spec_autodrive = AutoDrive
        -- AutoDrive.debugMsg(self, "AutoDrive:onPreLoad")
        -- AutoDriveRegister.addModTranslations(g_i18n)
    end
end

function AutoDrive:onLoad(savegame)
    -- This will run before initial MP sync
    if self.ad == nil then
        self.ad = {}
    end
    self.ad.dirtyFlag = self:getNextDirtyFlag()
    self.ad.smootherDriving = {}
    self.ad.smootherDriving.lastMaxSpeed = 0
    self.ad.groups = {}
    self.ad.currentHelper = nil

    self.ad.distances = {}
    self.ad.distances.wayPoints = nil
    self.ad.distances.closest = {}
    self.ad.distances.closest.wayPoint = -1
    self.ad.distances.closest.distance = 0
    self.ad.distances.closestNotReverse = {}
    self.ad.distances.closestNotReverse.wayPoint = -1
    self.ad.distances.closestNotReverse.distance = 0

    self.ad.lastDrawPosition = {x = 0, z = 0}

    self.ad.stateModule = ADStateModule:new(self)
    self.ad.recordingModule = ADRecordingModule:new(self)
    self.ad.trailerModule = ADTrailerModule:new(self)
    self.ad.taskModule = ADTaskModule:new(self)
    self.ad.drivePathModule = ADDrivePathModule:new(self)
    self.ad.specialDrivingModule = ADSpecialDrivingModule:new(self)
    self.ad.collisionDetectionModule = ADCollisionDetectionModule:new(self)
    self.ad.pathFinderModule = PathFinderModule:new(self)
    if self.spec_locomotive then
        self.ad.trainModule = ADTrainModule:new(self)
    end

    self.ad.modes = {}
    self.ad.modes[AutoDrive.MODE_DRIVETO] = DriveToMode:new(self)
    self.ad.modes[AutoDrive.MODE_DELIVERTO] = UnloadAtMode:new(self)
    self.ad.modes[AutoDrive.MODE_PICKUPANDDELIVER] = PickupAndDeliverMode:new(self)
    self.ad.modes[AutoDrive.MODE_LOAD] = LoadMode:new(self)
    self.ad.modes[AutoDrive.MODE_BGA] = BGAMode:new(self)
    self.ad.modes[AutoDrive.MODE_UNLOAD] = CombineUnloaderMode:new(self)

    self.ad.onRouteToPark = false
    self.ad.onRouteToRefuel = false
    self.ad.onRouteToRepair = false
    self.ad.isStoppingWithError = false
    self.ad.isCpEmpty = false
    self.ad.isCpFull = false
    self.ad.isAIJobFinished = false

    AutoDrive.resetMouseSelections(self)
end

function AutoDrive:onPostLoad(savegame)
    -- This will run before initial MP sync
    --print("Running post load for vehicle: " .. self:getName())
    if self.ad == nil then
        return
    end

    for groupName, _ in pairs(ADGraphManager:getGroups()) do
        self.ad.groups[groupName] = false
    end

    if self.isServer then
        if savegame ~= nil then

-- Logging.info("[AD] AutoDrive:onPostLoad savegame.xmlFile ->%s<-", tostring(savegame.xmlFile))
            local xmlFile = savegame.xmlFile
            -- local xmlFile = loadXMLFile("vehicleXML", savegame.xmlFile)
            local key = savegame.key .. ".AutoDrive"
-- Logging.info("[AD] AutoDrive:onPostLoad key ->%s<-", tostring(key))
            -- print("Trying to load xml keys from: " .. key)

            self.ad.stateModule:readFromXMLFile(xmlFile, key)
            AutoDrive.readVehicleSettingsFromXML(self, xmlFile, key)

            if xmlFile:hasProperty(key) then
                local groupString = xmlFile:getValue(key .. "#groups")
                if groupString ~= nil then
                    local groupTable = string.split(groupString, ";")
                    for _, groupCombined in pairs(groupTable) do
                        local groupNameAndBool = string.split(groupCombined, ",")
                        if tonumber(groupNameAndBool[2]) >= 1 then
                            self.ad.groups[groupNameAndBool[1]] = true
                        else
                            self.ad.groups[groupNameAndBool[1]] = false
                        end
                    end
                end
            end
        end

        self.ad.noMovementTimer = AutoDriveTON:new()
        self.ad.driveForwardTimer = AutoDriveTON:new()

        if not (AIJobFieldWork and AIJobFieldWork.getPricePerMs) then
            Logging.error("[AD] AutoDrive:onPostLoad AIJobFieldWork not found!")
        end
    end

    if self.ad.typeIsConveyorBelt == nil then
        if self.type and self.type.name and self.type.name == "conveyorBelt" then
            self.ad.typeIsConveyorBelt = true
        else
            self.ad.typeIsConveyorBelt = false
        end
    end

    -- sugarcane harvester need special consideration
    if self.spec_pipe ~= nil and self.spec_enterable ~= nil and self.spec_combine ~= nil then
        if self.typeName == "combineCutterFruitPreparer" then
            local _, vehicleFillCapacity, _, _ = AutoDrive.getObjectFillLevels(self)
            self.ad.isSugarcaneHarvester = vehicleFillCapacity == math.huge
        end
    end

    -- harvester types
    local isValidHarvester = AutoDrive.setCombineType(self)
    if isValidHarvester then
        ADHarvestManager:registerHarvester(self)
    end

    if self.ad.settings == nil then
        AutoDrive.copySettingsToVehicle(self)
    end

    self.ad.foldStartTime = 0
    -- Pure client side state
    self.ad.nToolTipWait = 300
    self.ad.sToolTip = ""
    self.ad.destinationFilterText = ""

    self.ad.showingMouse = false

    self.ad.lastMouseState = false
    -- Creating a new transform on front of the vehicle
    self.ad.frontNode = createTransformGroup(self:getName() .. "_frontNode")
    link(self.components[1].node, self.ad.frontNode)
    setTranslation(self.ad.frontNode, 0, 0, self.size.length / 2 + self.size.lengthOffset + 0.75)

    local x, y, z = getWorldTranslation(self.components[1].node)
    self.ad.lastDrawPosition = {x = x, z = z}

    if self.spec_enterable ~= nil and self.spec_enterable.cameras ~= nil then
        for _, camera in pairs(self.spec_enterable.cameras) do
            camera.storedIsRotatable = camera.isRotatable
        end
    end
end

function AutoDrive:onWriteStream(streamId, connection)
    if self.ad == nil then
        return
    end

    local count = 0
    for _, setting in pairs(AutoDrive.settings) do
        if setting ~= nil and setting.isVehicleSpecific and not setting.isUserSpecific then
            count = count + 1
        end
    end

    streamWriteUInt16(streamId, count)

    for settingName, setting in pairs(AutoDrive.settings) do
        if setting ~= nil and setting.isVehicleSpecific and not setting.isUserSpecific then
            streamWriteString(streamId, settingName)
            streamWriteUInt16(streamId, AutoDrive.getSettingState(settingName, self))
        end
    end

    self.ad.stateModule:writeStream(streamId)
    if self.ad.stateModule:isActive() and self.ad.currentHelper ~= nil then
        streamWriteUInt8(streamId, self.ad.currentHelper.index)
    else
        streamWriteUInt8(streamId, 0)
    end
end

function AutoDrive:onReadStream(streamId, connection)
    if self.ad == nil then
        return
    end

    local count = streamReadUInt16(streamId)
    for i = 1, count do
        local settingName = streamReadString(streamId)
        local value = streamReadUInt16(streamId)
        if self.ad.settings[settingName] == nil then
            Logging.warning("[AutoDrive] AutoDrive:onReadStream() encountered unknown setting " .. tostring(settingName))
        else
            self.ad.settings[settingName].current = value
            self.ad.settings[settingName].new = value
        end
    end

    self.ad.stateModule:readStream(streamId)
    local helperIndex = streamReadUInt8(streamId)
    if helperIndex > 0 then
        -- in case we receive a helper index greater then actual number of helpers, we need to increase the number of helpers
        AutoDrive.checkAddHelper(self, helperIndex) -- use helper with index helperIndex
    end
end

function AutoDrive:onUpdateTick(dt, isActiveForInput, isActiveForInputIgnoreSelection, isSelected)
    -- waypoints distances are updated once every ~2 frames
    -- self:resetClosestWayPoint()
    -- if we want to update distances every frame, when lines drawing is enabled, we can move this at the end of onDraw function

    if self.isServer then
        self.ad.recordingModule:updateTick(dt, isActiveForInput, isActiveForInputIgnoreSelection, isSelected)

        if AutoDrive.getSetting("FoldImplements", self) and self.ad.stateModule:isActive() then
            -- fold combine ladder if user exit the vehicle #679
            if not AutoDrive.isLadderFolded(self) then
                AutoDrive.foldLadder(self)
            end
        end

        local farmID = 0
        if self.getOwnerFarmId then
            farmID = self:getOwnerFarmId()
        end
        if farmID ~= nil and farmID > 0 and self.ad.stateModule:isActive() then
            local driverWages = AutoDrive.getSetting("driverWages")
            local difficultyMultiplier = AutoDrive.getPriceMultiplier()
            local pricePerMs = AIJobFieldWork and AIJobFieldWork.getPricePerMs and AIJobFieldWork:getPricePerMs() or 0.0005
            local price = -dt * difficultyMultiplier * (driverWages) * pricePerMs
            --price = price + (dt * difficultyMultiplier * 0.001)   -- add the price which AI internal already substracted - no longer required for FS22
            g_currentMission:addMoney(price, farmID, MoneyType.AI, true)
        end
    end

    if self.ad.lastMouseState ~= g_inputBinding:getShowMouseCursor() then
        self:toggleMouse()
    end
end

function AutoDrive:onReadUpdateStream(streamId, timestamp, connection)
    if self.ad == nil then
        return
    end
    if connection:getIsServer() then
        if streamReadBool(streamId) then
            self.ad.stateModule:readUpdateStream(streamId)

            local currentHelperIndex = self.ad.stateModule:getCurrentHelperIndex()
            if self.ad.currentHelper == nil or currentHelperIndex <= 0 then
                -- in case we receive a helper index greater then actual number of helpers, we need to increase the number of helpers
                AutoDrive.checkAddHelper(self, currentHelperIndex) -- use helper with index helperIndex
            end

        end
    end
end

function AutoDrive:onWriteUpdateStream(streamId, connection, dirtyMask)
    if self.ad == nil then
        return
    end
    if not connection:getIsServer() then
        if streamWriteBool(streamId, bit32.band(dirtyMask, self.ad.dirtyFlag) ~= 0) then
            self.ad.stateModule:writeUpdateStream(streamId)
        end
    end
end

function AutoDrive:onUpdate(dt)
    if self.isServer and self.ad.stateModule:isActive() then
        self.ad.recordingModule:update(dt)

        if not AutoDrive.getSetting("FoldImplements", self) or (self.ad.foldStartTime + AutoDrive.foldTimeout < g_time) then
            self.ad.taskModule:update(dt)
        else
            -- should fold implements
            if not AutoDrive.getAllImplementsFolded(self) then
                if (g_updateLoopIndex % (AutoDrive.PERF_FRAMES) == 0) then
                    -- fold animations take some time, so no need to check and initiate each frame
                    if self.startMotor then
                        if not self:getIsMotorStarted() then
                            self:startMotor()
                        end
                    end
                    AutoDrive.foldAllImplements(self)
                end
                if self.ad ~= nil and self.ad.specialDrivingModule ~= nil then
                    self.ad.specialDrivingModule.motorShouldNotBeStopped = true
                    self.ad.specialDrivingModule:stopVehicle()
                    self.ad.specialDrivingModule:update(dt)
                    self.ad.specialDrivingModule.motorShouldNotBeStopped = false
                end
            else
                -- all folded - no further tries necessary
                self.ad.foldStartTime = 0
                AutoDrive.getAllVehicleDimensions(self, true)
                self:raiseActive()
            end
        end
    end

    self.ad.stateModule:update(dt)

    ADSensor:handleSensors(self, dt)

    if not self.ad.stateModule:isActive() and self.ad.taskModule:getNumberOfTasks() > 0 then
        self.ad.taskModule:abortAllTasks()
    end

    if self.isServer then
        AutoDrive.updateAutoDriveLights(self)
    end

    --For 'legacy' purposes, this value should be kept since other mods already test for this:
    self.ad.mapMarkerSelected = self.ad.stateModule:getFirstMarkerId()
    self.ad.mapMarkerSelected_Unload = self.ad.stateModule:getSecondMarkerId()
end

function AutoDrive:saveToXMLFile(xmlFile, key, usedModNames)
    if self.ad == nil or self.ad.stateModule == nil then
        return
    end
    local adKey = string.gsub(key, "FS25_AutoDrive.AutoDrive", "AutoDrive")

    --if not xmlFile:hasProperty(key) then
        --xmlFile:setValue(adKey, {})
        --return
    --end

    self.ad.stateModule:saveToXMLFile(xmlFile, adKey)
    AutoDrive.saveVehicleSettingsToXMLFile(self, xmlFile, adKey)

    if self.ad.groups ~= nil then
        local combinedString = ""
        for groupName, _ in pairs(ADGraphManager:getGroups()) do
            for myGroupName, value in pairs(self.ad.groups) do
                if groupName == myGroupName then
                    if string.len(combinedString) > 0 then
                        combinedString = combinedString .. ";"
                    end
                    if value == true then
                        combinedString = combinedString .. myGroupName .. ",1"
                    else
                        combinedString = combinedString .. myGroupName .. ",0"
                    end
                end
            end
        end
        xmlFile:setValue(adKey .. "#groups", combinedString)
    end
end

function AutoDrive:onDrawUIInfo()
    local controlledVehicle = AutoDrive.getControlledVehicle()
	if controlledVehicle == nil or self ~= controlledVehicle then
        -- user not in control of this vehicle
        return
    end

    if AutoDrive.getSetting("showHUD") then
        AutoDrive.Hud:drawHud(self)
    end

    if AutoDrive.getSetting("showNextPath") == true then
        local sWP = self.ad.stateModule:getCurrentWayPoint()
        local eWP = self.ad.stateModule:getNextWayPoint()
        if sWP ~= nil and eWP ~= nil then
            --draw line with direction markers (arrow)
            ADDrawingManager:addLineTask(sWP.x, sWP.y, sWP.z, eWP.x, eWP.y, eWP.z, 1.2, unpack(AutoDrive.currentColors.ad_color_currentConnection))
            ADDrawingManager:addArrowTask(sWP.x, sWP.y, sWP.z, eWP.x, eWP.y, eWP.z, 1.2, ADDrawingManager.arrows.position.start, unpack(AutoDrive.currentColors.ad_color_currentConnection))
        end
    end

    if (AutoDrive.isEditorModeEnabled() or AutoDrive.isEditorShowEnabled()) then
        self:onDrawEditorMode()
        self:onDrawPreviews()
    end

    local x, y, z = getWorldTranslation(self.components[1].node)
    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
        for _, otherVehicle in pairs(AutoDrive.getAllVehicles()) do
            if otherVehicle ~= nil and otherVehicle.ad ~= nil and otherVehicle.ad.drivePathModule ~= nil and otherVehicle.ad.drivePathModule:getWayPoints() ~= nil and not otherVehicle.ad.drivePathModule:isTargetReached() then
                local currentIndex = otherVehicle.ad.drivePathModule:getCurrentWayPointIndex()

                local lastPoint = nil
                for index, point in ipairs(otherVehicle.ad.drivePathModule:getWayPoints()) do
                    if point.isPathFinderPoint and index >= currentIndex and lastPoint ~= nil and MathUtil.vector2Length(x - point.x, z - point.z) < 160 then
                        ADDrawingManager:addLineTask(lastPoint.x, lastPoint.y, lastPoint.z, point.x, point.y, point.z, 1, 1, 0.09, 0.09)
                        ADDrawingManager:addArrowTask(lastPoint.x, lastPoint.y, lastPoint.z, point.x, point.y, point.z, 1, ADDrawingManager.arrows.position.start, 1, 0.09, 0.09)

                        if AutoDrive.getSettingState("lineHeight") == 1 then
                            local gy = point.y - AutoDrive.drawHeight + 4
                            local ty = lastPoint.y - AutoDrive.drawHeight + 4
                            ADDrawingManager:addLineTask(point.x, gy, point.z, point.x, point.y, point.z, 1, 1, 0.09, 0.09)
                            ADDrawingManager:addSphereTask(point.x, gy, point.z, 3, 1, 0.09, 0.09, 0.15)
                            ADDrawingManager:addLineTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, 1, 0.09, 0.09)
                            ADDrawingManager:addArrowTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, ADDrawingManager.arrows.position.start, 1, 0.09, 0.09)
                        else
                            local gy = point.y - AutoDrive.drawHeight - 4
                            local ty = lastPoint.y - AutoDrive.drawHeight - 4
                            ADDrawingManager:addLineTask(point.x, gy, point.z, point.x, point.y, point.z, 1, 1, 0.09, 0.09)
                            ADDrawingManager:addSphereTask(point.x, gy, point.z, 3, 1, 0.09, 0.09, 0.15)
                            ADDrawingManager:addLineTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, 1, 0.09, 0.09)
                            ADDrawingManager:addArrowTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, ADDrawingManager.arrows.position.start, 1, 0.09, 0.09)
                        end
                    end
                    lastPoint = point
                end
            end
        end

        for _, otherVehicle in pairs(AutoDrive.getAllVehicles()) do
            if otherVehicle ~= nil and otherVehicle.ad ~= nil and otherVehicle.ad.drivePathModule ~= nil and otherVehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getBreadCrumbs() ~= nil then
                local lastPoint = nil
                for index, point in ipairs(otherVehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getBreadCrumbs().items) do
                    if lastPoint ~= nil and MathUtil.vector2Length(x - point.x, z - point.z) < 80 then
                        ADDrawingManager:addLineTask(lastPoint.x, lastPoint.y, lastPoint.z, point.x, point.y, point.z, 1, 1.0, 0.769, 0.051)
                        ADDrawingManager:addArrowTask(lastPoint.x, lastPoint.y, lastPoint.z, point.x, point.y, point.z, 1, ADDrawingManager.arrows.position.start, 1.0, 0.769, 0.051)

                        if AutoDrive.getSettingState("lineHeight") == 1 then
                            local gy = point.y - AutoDrive.drawHeight + 4
                            local ty = lastPoint.y - AutoDrive.drawHeight + 4
                            ADDrawingManager:addLineTask(point.x, gy, point.z, point.x, point.y, point.z, 1, 1.0, 0.769, 0.051)
                            ADDrawingManager:addSphereTask(point.x, gy, point.z, 3, 1.0, 0.769, 0.051, 0.15)
                            ADDrawingManager:addLineTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, 1.0, 0.769, 0.051)
                            ADDrawingManager:addArrowTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, ADDrawingManager.arrows.position.start, 1.0, 0.769, 0.051)
                        else
                            local gy = point.y - AutoDrive.drawHeight - 4
                            local ty = lastPoint.y - AutoDrive.drawHeight - 4
                            ADDrawingManager:addLineTask(point.x, gy, point.z, point.x, point.y, point.z, 1, 1.0, 0.769, 0.051)
                            ADDrawingManager:addSphereTask(point.x, gy, point.z, 3, 1.0, 0.769, 0.051, 0.15)
                            ADDrawingManager:addLineTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, 1.0, 0.769, 0.051)
                            ADDrawingManager:addArrowTask(lastPoint.x, ty, lastPoint.z, point.x, gy, point.z, 1, ADDrawingManager.arrows.position.start, 1.0, 0.769, 0.051)
                        end
                    end
                    lastPoint = point
                end
            end
        end
    end
    if AutoDrive.devOnDraw ~= nil then
        AutoDrive.devOnDraw(self)
    end

    --[[
    if self.ad.isCombine then
        AutoDrive.drawTripod(AutoDrive.getDischargeNode(self))
        AutoDrive.drawTripod(self.components[1].node, {x=0,y=3,z=0})
        if self.components[2] ~= nil then
            AutoDrive.drawTripod(self.components[2].node, {x=0,y=3,z=0})
        end
    end
    --]]
end

function AutoDrive:onDrawPreviews()
    if AutoDrive.splineInterpolation == nil or not AutoDrive.splineInterpolation.valid then
        return
    end

    local lastHeight = AutoDrive.splineInterpolation.startNode.y
    local lastWp = AutoDrive.splineInterpolation.startNode
    local targetWp = AutoDrive.splineInterpolation.endNode
    local arrowPosition = ADDrawingManager.arrows.position.middle
    local collisionFree = AutoDrive:checkForCollisionOnSpline()
    local color
    local isSubPrio = ADGraphManager:getIsPointSubPrio(lastWp.id) or ADGraphManager:getIsPointSubPrio(targetWp.id)
    local isDual = AutoDrive.leftCTRLmodifierKeyPressed and AutoDrive.leftALTmodifierKeyPressed
    if not collisionFree then
        color = AutoDrive.currentColors.ad_color_previewNotOk
    elseif isDual and isSubPrio then
        color = AutoDrive.currentColors.ad_color_previewSubPrioDualConnection
    elseif isDual then
        color = AutoDrive.currentColors.ad_color_previewDualConnection
    elseif isSubPrio then
        color = AutoDrive.currentColors.ad_color_previewSubPrioSingleConnection
    else
        color = AutoDrive.currentColors.ad_color_previewSingleConnection
    end

    for wpId, wp in pairs(AutoDrive.splineInterpolation.waypoints) do
        if wpId ~= 1 and wpId < (#AutoDrive.splineInterpolation.waypoints - 1) then
            if math.abs(wp.y - lastHeight) > 1 then -- prevent point dropping into the ground in case of bridges etc
                wp.y = lastHeight
            end

            ADDrawingManager:addLineTask(lastWp.x, lastWp.y, lastWp.z, wp.x, wp.y, wp.z, 1, unpack(color))
            if not isDual then
                ADDrawingManager:addArrowTask(lastWp.x, lastWp.y, lastWp.z, wp.x, wp.y, wp.z, 1, arrowPosition, unpack(color))
            end
            lastWp = {x = wp.x, y = wp.y, z = wp.z}
            lastHeight = wp.y
        end
    end

    
    ADDrawingManager:addLineTask(lastWp.x, lastWp.y, lastWp.z, targetWp.x, targetWp.y, targetWp.z, 1, unpack(color))
    if not isDual then
        ADDrawingManager:addArrowTask(lastWp.x, lastWp.y, lastWp.z, targetWp.x, targetWp.y, targetWp.z, 1, arrowPosition, unpack(color))
    end
end

function AutoDrive:onPostAttachImplement(attachable, inputJointDescIndex, jointDescIndex, loadFromSavegame)
    if attachable["spec_FS19_addon_strawHarvest.strawHarvestPelletizer"] ~= nil then
        attachable.isPremos = true
    end
    if (attachable.spec_pipe ~= nil and attachable.spec_combine ~= nil) or attachable.isPremos then
        attachable.ad = self.ad -- takeover i.e. sensors from trailing vehicle

        local vx, _, vz =  AutoDrive.localDirectionToWorld(self, 0, 0, 1)
        local cx, _, cz =  AutoDrive.localDirectionToWorld(attachable, 0, 0, 1)
        local angle = math.abs(AutoDrive.angleBetween({x = vx, z = vz}, {x = cx, z = cz}))

        if angle > 100 then
            -- attachable is rotated attached - use sensors from vehicle
            attachable.ad.sensors = nil
            attachable.ad.isReverseAttached = true
            attachable.ad.ADRootNode = createTransformGroup("ADRootNode")
            link(attachable.rootNode, attachable.ad.ADRootNode)
            setRotation(attachable.ad.ADRootNode, 0, math.pi, 0)
            setTranslation(attachable.ad.ADRootNode, 0, 0, 0)
        end
        attachable.isTrailedHarvester = true
        attachable.trailingVehicle = self
        -- harvester types
        local isValidHarvester = AutoDrive.setCombineType(attachable)
        if isValidHarvester then
            self.ad.attachableCombine = attachable
            ADHarvestManager:registerHarvester(attachable)
        end
    end
    AutoDrive.setValidSupportedFillType(self)

    local controlledVehicle = AutoDrive.getControlledVehicle()
    if controlledVehicle and controlledVehicle.ad and controlledVehicle == self then
        AutoDrive.Hud.lastUIScale = 0
    end
    AutoDrive.getFrontToolWidth(self, true)
end

function AutoDrive:onPreDetachImplement(implement)
    local attachable = implement.object
    if attachable.isTrailedHarvester and attachable.trailingVehicle == self then
        attachable.ad = nil
        self.ad.attachableCombine = nil
        ADHarvestManager:unregisterHarvester(attachable)
        attachable.isTrailedHarvester = false
        attachable.trailingVehicle = nil
        self.ad.isRegisterdHarvester = nil
    end
    if self.ad ~= nil then
        self.ad.frontToolWidth = nil
        self.ad.frontToolLength = nil
    end
end

-- Giants special behaviour: at time of the event the implement and all implements attached to it are still attached!
-- thats why the attached and all following have to be taken to special consideration!
function AutoDrive:onPostDetachImplement(implementIndex)
    local controlledVehicle = AutoDrive.getControlledVehicle()
    AutoDrive.setValidSupportedFillType(self, implementIndex)
    if controlledVehicle and controlledVehicle.ad and controlledVehicle == self then
        AutoDrive.Hud.lastUIScale = 0
    end
end

function AutoDrive:onEnterVehicle(isControlling)
    local controlledVehicle = AutoDrive.getControlledVehicle()
    if AutoDrive:hasAL(self) then
        -- AutoLoad
        local currentFillType = AutoDrive:getALCurrentFillType(self)
        if currentFillType ~= nil then
            self.ad.stateModule:setFillType(currentFillType)
        else
            self.ad.stateModule:setFillType(FillType.UNKNOWN)
        end
    end
    if controlledVehicle and controlledVehicle.ad and controlledVehicle == self then
        AutoDrive.Hud.lastUIScale = 0
    end
    if self.isServer and self.ad and self.ad.stateModule and not self.ad.stateModule:isActive() then
        -- do not force dimension update while tabbing through active AD vehicles, which might be unfolded
        if AutoDrive.getAllImplementsFolded(self) then
            AutoDrive.getAllVehicleDimensions(self, true)
        end
    end

    local spec = self.spec_enterable
    if spec and self:getIsControlled() then
        if self.ad and self.ad.stateModule then
            self.ad.stateModule:setPlayerFarmId(spec.controllerFarmId)
        end
        if not self.ad.stateModule:isActive() and not AutoDrive:getIsCPActive(self) then
            self.ad.stateModule:setActualFarmId(self.ad.stateModule:getPlayerFarmId()) -- onEnterVehicle
        end
    end
    self:resetWayPointsDistance()
end

function AutoDrive:onLeaveVehicle(wasEntered)
    AutoDrive.debugPrint(self, AutoDrive.DC_VEHICLEINFO, "AutoDrive:onLeaveVehicle wasEntered: %s", tostring(wasEntered))
    if not wasEntered then
        return
    end
    if not AutoDrive.getSetting("RecordWhileNotInVehicle") then
        if self.ad ~= nil and self.ad.stateModule ~= nil then
            self.ad.stateModule:disableCreationMode()
        end
    end
    local spec = self.spec_enterable
    if spec then
        if self.ad and self.ad.stateModule then
            self.ad.stateModule:setPlayerFarmId(0)
        end
        if not self.ad.stateModule:isActive() and not AutoDrive:getIsCPActive(self) and not self:getIsAIActive() then
            self.ad.stateModule:setActualFarmId(self.ad.stateModule:getPlayerFarmId()) -- onLeaveVehicle
        end
    end
    if g_inputBinding:getShowMouseCursor() then
        -- there are several ways to trigger this function while in the main menu - this delays hiding the mouse until the menu was closed
        AutoDrive.hideMouseCursorOnNextTick = true
    end
    AutoDrive.Hud:closeAllPullDownLists(self)
end

function AutoDrive:onSetBroken()
    if self.ad ~= nil and self.ad.stateModule ~= nil then
		if self.ad.stateModule:isActive() then
			self:stopAutoDrive()
		end
	end
end

function AutoDrive:onDelete()
    ADHarvestManager:unregisterHarvester(self)
    ADHarvestManager:unregisterAsUnloader(self)
    AutoDriveHud:deleteMapHotspot(self)
end

function AutoDrive:onDrawEditorMode()
    local isActive = self.ad.stateModule:isActive()
    local DrawingManager = ADDrawingManager

    local startNode = self.components[1].node
    local x, y, z = getWorldTranslation(startNode)

    local isEditorShowEnabled = AutoDrive.isEditorShowEnabled()
    local isInExtendedEditorMode = AutoDrive.isInExtendedEditorMode()
    local isInConstructionMode = AutoDrive.isInConstructionMode()
    local mouseActiveForAutoDrive = (AutoDrive.isMouseActiveForHud() or AutoDrive.isMouseActiveForEditor()) and g_inputBinding:getShowMouseCursor()

    if isInConstructionMode then
        x = g_constructionScreen.camera.cameraX
        z = g_constructionScreen.camera.cameraZ
    end

    local dy = y + 3.5 - AutoDrive.getSetting("lineHeight")
    local maxDistance = AutoDrive.drawDistance
    local arrowPosition = DrawingManager.arrows.position.start

    local previewDirection =
                    not AutoDrive.leftLSHIFTmodifierKeyPressed
                    and AutoDrive.leftCTRLmodifierKeyPressed
                    and not AutoDrive.leftALTmodifierKeyPressed
                    and not AutoDrive.rightSHIFTmodifierKeyPressed

    local previewSubPrio =
                    AutoDrive.leftLSHIFTmodifierKeyPressed
                    and not AutoDrive.leftCTRLmodifierKeyPressed
                    and not AutoDrive.leftALTmodifierKeyPressed
                    and not AutoDrive.rightSHIFTmodifierKeyPressed

    if isEditorShowEnabled or isInExtendedEditorMode then
        local distance = MathUtil.vector2Length(x - self.ad.lastDrawPosition.x, z - self.ad.lastDrawPosition.z)
        if distance > AutoDrive.drawDistance / 2 then
            self.ad.lastDrawPosition = {x = x, z = z}
            self:resetWayPointsDistance()
        end
    end

    if AutoDrive:getIsEntered(self) and ADGraphManager:hasChanges() then
        self:resetWayPointsDistance()
        ADGraphManager:resetChanges()
    end
    
    if self.ad.distances.wayPoints == nil then
        self:updateWayPointsDistance(x, z)
    end

    --Draw close destinations
    if isEditorShowEnabled or isInExtendedEditorMode then
        for _, marker in pairs(ADGraphManager:getMapMarkers()) do
            local wp = ADGraphManager:getWayPointById(marker.id)
            if wp then
                if MathUtil.vector2Length(wp.x - x, wp.z - z) < maxDistance then
                    local scale = AutoDrive.getSetting("scaleMarkerText") or 1
                    Utils.renderTextAtWorldPosition(wp.x, wp.y + 4, wp.z, marker.name, getCorrectTextSize(0.013) * scale, 0)
                    DrawingManager:addMarkerTask(wp.x, wp.y + 0.45, wp.z)
                end
            end
        end
    end

    if ADGraphManager:getWayPointById(1) ~= nil
        and isInExtendedEditorMode
        and not isInConstructionMode
        then
        --Draw line to selected neighbor point
        local neighbour = self.ad.stateModule:getSelectedNeighbourPoint()
        if neighbour ~= nil then
            DrawingManager:addLineTask(x, dy, z, neighbour.x, neighbour.y, neighbour.z, 1, 1, 1, 0)
        end

        --Draw line to closest point
        local closest, _ = self:getClosestWayPoint(true)
        local wp = ADGraphManager:getWayPointById(closest)
        if wp ~= nil then
            DrawingManager:addLineTask(x, dy, z, wp.x, wp.y, wp.z, 1, unpack(AutoDrive.currentColors.ad_color_closestLine))
            DrawingManager:addSmallSphereTask(x, dy, z, unpack(AutoDrive.currentColors.ad_color_closestLine))
        end
    end

    local outPointsSeen = {}
    for _, point in pairs(self:getWayPointsInRange(0, maxDistance)) do
        local x = point.x
        local y = point.y
        local z = point.z
        local isSubPrio = ADGraphManager:getIsPointSubPrio(point.id)

        if isInExtendedEditorMode then
            arrowPosition = DrawingManager.arrows.position.middle
            if AutoDrive.enableSphere == true then
                if (AutoDrive.mouseIsAtPos(point, 0.01) or point.isSelected) and mouseActiveForAutoDrive then
                    DrawingManager:addSphereTask(x, y, z, 3, unpack(AutoDrive.currentColors.ad_color_hoveredNode))
                else
                    if point.id == self.ad.selectedNodeId then
                        DrawingManager:addSphereTask(x, y, z, 3, unpack(AutoDrive.currentColors.ad_color_selectedNode))
                    else
                        if isSubPrio then
                            DrawingManager:addSphereTask(x, y, z, 3, unpack(AutoDrive.currentColors.ad_color_subPrioNode))
                        else
                            if point.colors ~= nil then
                                DrawingManager:addSphereTask(x, y, z, 3, unpack(point.colors))
                            else
                                DrawingManager:addSphereTask(x, y, z, 3, unpack(AutoDrive.currentColors.ad_color_default))
                            end
                        end
                    end
                end

                -- If the lines are drawn above the vehicle, we have to draw a line to the reference point on the ground and a second cube there for moving the node position
                if AutoDrive.getSettingState("lineHeight") > 1 then
                    local gy = y - AutoDrive.drawHeight - AutoDrive.getSetting("lineHeight")
                    DrawingManager:addLineTask(x, y, z, x, gy, z, 1, unpack(AutoDrive.currentColors.ad_color_editorHeightLine))

                    if (AutoDrive.mouseIsAtPos(point, 0.01) or AutoDrive.mouseIsAtPos({x = x, y = gy, z = z}, 0.01)) and mouseActiveForAutoDrive then
                        DrawingManager:addSphereTask(x, gy, z, 3, unpack(AutoDrive.currentColors.ad_color_hoveredNode))
                    else
                        if point.id == self.ad.selectedNodeId then
                            DrawingManager:addSphereTask(x, gy, z, 3, unpack(AutoDrive.currentColors.ad_color_selectedNode))
                        else
                            if isSubPrio then
                                DrawingManager:addSphereTask(x, gy, z, 3, unpack(AutoDrive.currentColors.ad_color_subPrioNode))
                            else
                                DrawingManager:addSphereTask(x, gy, z, 3, unpack(AutoDrive.currentColors.ad_color_default))
                            end
                        end
                    end
                end

                -- draw previous and next points in different colors - note: sequence is important
                if point.out ~= nil and not isActive and mouseActiveForAutoDrive then
                    for _, neighbor in pairs(point.out) do
                        local nWp = ADGraphManager:getWayPointById(neighbor)
                        if nWp ~= nil then
                            if AutoDrive.mouseIsAtPos(nWp, 0.01) then
                                -- draw previous point in GOLDHOFER_PINK1
                                DrawingManager:addSphereTask(point.x, point.y, point.z, 3.4, unpack(AutoDrive.currentColors.ad_color_previousNode))
                            end
                            if AutoDrive.mouseIsAtPos(point, 0.01) then
                                -- draw next point
                                DrawingManager:addSphereTask(nWp.x, nWp.y, nWp.z, 3.2, unpack(AutoDrive.currentColors.ad_color_nextNode))
                            end
                        end
                    end
                end
            end
        end

-- draw connection lines
        if point.out ~= nil and (isInExtendedEditorMode or isEditorShowEnabled) then

            for _, neighbor in pairs(point.out) do
                -- if a section is active, skip these connections, they are drawn below
                local skipSectionDraw = false
                if self.ad.sectionWayPoints ~= nil and #self.ad.sectionWayPoints > 2 then
                    if
                        table.contains(self.ad.sectionWayPoints, point.id)
                        and table.contains(self.ad.sectionWayPoints, neighbor)
                        and (previewDirection or previewSubPrio)
                        then
                        skipSectionDraw = true
                    end
                end

                table.insert(outPointsSeen, neighbor)
                local target = ADGraphManager:getWayPointById(neighbor)
                local targetIsSubPrio = ADGraphManager:getIsPointSubPrio(neighbor)
                if target ~= nil and not skipSectionDraw then
                    --check if outgoing connection is a dual way connection
                    local nWp = ADGraphManager:getWayPointById(neighbor)
                    if point.incoming == nil or table.contains(point.incoming, neighbor) then
                        --draw dual way line
                        if point.id > nWp.id then
                            if isSubPrio or targetIsSubPrio then
                                DrawingManager:addLineTask(x, y, z, nWp.x, nWp.y, nWp.z, 1, unpack(AutoDrive.currentColors.ad_color_subPrioDualConnection))
                            else
                                DrawingManager:addLineTask(x, y, z, nWp.x, nWp.y, nWp.z, 1, unpack(AutoDrive.currentColors.ad_color_dualConnection))
                            end
                        end
                    else
                        --draw line with direction markers (arrow)
                        if (nWp.incoming == nil or table.contains(nWp.incoming, point.id)) then
                            -- one way line
                            if isSubPrio or targetIsSubPrio then
                                DrawingManager:addLineTask(x, y, z, nWp.x, nWp.y, nWp.z, 1, unpack(AutoDrive.currentColors.ad_color_subPrioSingleConnection))
                                DrawingManager:addArrowTask(x, y, z, nWp.x, nWp.y, nWp.z, 1, arrowPosition, unpack(AutoDrive.currentColors.ad_color_subPrioSingleConnection))
                            else
                                DrawingManager:addLineTask(x, y, z, nWp.x, nWp.y, nWp.z, 1, unpack(AutoDrive.currentColors.ad_color_singleConnection))
                                DrawingManager:addArrowTask(x, y, z, nWp.x, nWp.y, nWp.z, 1, arrowPosition, unpack(AutoDrive.currentColors.ad_color_singleConnection))
                            end
                        else
                            -- reverse way line
                            DrawingManager:addLineTask(x, y, z, nWp.x, nWp.y, nWp.z, 1, unpack(AutoDrive.currentColors.ad_color_reverseConnection))
                            DrawingManager:addArrowTask(x, y, z, nWp.x, nWp.y, nWp.z, 1, arrowPosition, unpack(AutoDrive.currentColors.ad_color_reverseConnection))
                        end
                    end
                end
            end
        end

        --just a quick way to highlight single (forgotten) points with no connections
        if ((#point.out == 0) and (#point.incoming == 0) and not table.contains(outPointsSeen, point.id) and point.colors == nil)
        and (isInExtendedEditorMode or isEditorShowEnabled) then
            y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 1, z) + 0.5
            DrawingManager:addCrossTask(x, y, z)
        end
    end

-- draw the section, with respect to preview types, active or next connections and SubPrio
    if (previewDirection or previewSubPrio) and self.ad.sectionWayPoints ~= nil and #self.ad.sectionWayPoints > 2 then
        local sectionPrio = ADGraphManager:getIsPointSubPrio(self.ad.sectionWayPoints[2])   -- 2nd WayPoint is the 1st in section and has the actual Prio
        local wayPointsDirection = ADGraphManager:getIsWayPointJunction(self.ad.sectionWayPoints[1], self.ad.sectionWayPoints[2])
        for i = 1, #self.ad.sectionWayPoints - 1 do
            local start = ADGraphManager:getWayPointById(self.ad.sectionWayPoints[i])
            local target = ADGraphManager:getWayPointById(self.ad.sectionWayPoints[i+1])

            if wayPointsDirection == 1 then
                if previewSubPrio then
                    if not sectionPrio then
                        DrawingManager:addLineTask(start.x, start.y, start.z, target.x, target.y, target.z, 1, unpack(AutoDrive.currentColors.ad_color_subPrioSingleConnection))
                        DrawingManager:addArrowTask(start.x, start.y, start.z, target.x, target.y, target.z, 1, arrowPosition, unpack(AutoDrive.currentColors.ad_color_subPrioSingleConnection))
                    else
                        DrawingManager:addLineTask(start.x, start.y, start.z, target.x, target.y, target.z, 1, unpack(AutoDrive.currentColors.ad_color_singleConnection))
                        DrawingManager:addArrowTask(start.x, start.y, start.z, target.x, target.y, target.z, 1, arrowPosition, unpack(AutoDrive.currentColors.ad_color_singleConnection))
                    end
                elseif previewDirection then
                    -- new direction backward
                    if not sectionPrio then
                        DrawingManager:addLineTask(target.x, target.y, target.z, start.x, start.y, start.z, 1,  unpack(AutoDrive.currentColors.ad_color_singleConnection))       -- green
                        DrawingManager:addArrowTask(target.x, target.y, target.z, start.x, start.y, start.z, 1, arrowPosition, unpack(AutoDrive.currentColors.ad_color_singleConnection))
                    else
                        DrawingManager:addLineTask(target.x, target.y, target.z, start.x, start.y, start.z, 1,  unpack(AutoDrive.currentColors.ad_color_subPrioSingleConnection))    -- orange
                        DrawingManager:addArrowTask(target.x, target.y, target.z, start.x, start.y, start.z, 1, arrowPosition, unpack(AutoDrive.currentColors.ad_color_subPrioSingleConnection))
                    end
                else
                end
            elseif wayPointsDirection == 2 then
                if previewSubPrio then
                    if not sectionPrio then
                        DrawingManager:addLineTask(target.x, target.y, target.z, start.x, start.y, start.z, 1,  unpack(AutoDrive.currentColors.ad_color_subPrioSingleConnection))    -- orange
                        DrawingManager:addArrowTask(target.x, target.y, target.z, start.x, start.y, start.z, 1, arrowPosition, unpack(AutoDrive.currentColors.ad_color_subPrioSingleConnection))
                    else
                        DrawingManager:addLineTask(target.x, target.y, target.z, start.x, start.y, start.z, 1,  unpack(AutoDrive.currentColors.ad_color_singleConnection))       -- green
                        DrawingManager:addArrowTask(target.x, target.y, target.z, start.x, start.y, start.z, 1, arrowPosition, unpack(AutoDrive.currentColors.ad_color_singleConnection))
                    end
                elseif previewDirection then
                    -- new direction dual
                    if not sectionPrio then
                        DrawingManager:addLineTask(start.x, start.y, start.z, target.x, target.y, target.z, 1, unpack(AutoDrive.currentColors.ad_color_dualConnection))  -- blue
                    else
                        DrawingManager:addLineTask(start.x, start.y, start.z, target.x, target.y, target.z, 1, unpack(AutoDrive.currentColors.ad_color_subPrioDualConnection))    -- dark orange
                    end
                else
                end
            elseif wayPointsDirection == 3 then
                if previewSubPrio then
                    if not sectionPrio then
                        DrawingManager:addLineTask(start.x, start.y, start.z, target.x, target.y, target.z, 1, unpack(AutoDrive.currentColors.ad_color_subPrioDualConnection))    -- dark orange
                    else
                        DrawingManager:addLineTask(start.x, start.y, start.z, target.x, target.y, target.z, 1, unpack(AutoDrive.currentColors.ad_color_dualConnection))    -- blue
                    end
                elseif previewDirection then
                    -- new direction forward
                    if not sectionPrio then
                        DrawingManager:addLineTask(start.x, start.y, start.z, target.x, target.y, target.z, 1, unpack(AutoDrive.currentColors.ad_color_singleConnection))        -- green
                        DrawingManager:addArrowTask(start.x, start.y, start.z, target.x, target.y, target.z, 1, arrowPosition, unpack(AutoDrive.currentColors.ad_color_singleConnection))
                    else
                        DrawingManager:addLineTask(start.x, start.y, start.z, target.x, target.y, target.z, 1, unpack(AutoDrive.currentColors.ad_color_subPrioSingleConnection)) -- orange
                        DrawingManager:addArrowTask(start.x, start.y, start.z, target.x, target.y, target.z, 1, arrowPosition, unpack(AutoDrive.currentColors.ad_color_subPrioSingleConnection))
                    end
                else
                end
            end
        end
    end

    if isInExtendedEditorMode and AutoDrive.enableSphere == true then
        if self.ad.selectionWayPoints and #self.ad.selectionWayPoints > 0 then
            -- draw range circle for selection wayPoints
            local selectedWayPoint = ADGraphManager:getWayPointById(self.ad.selectedNodeId)
            DebugUtil.drawDebugCircle(selectedWayPoint.x, selectedWayPoint.y, selectedWayPoint.z, self.ad.selectionRange, 30, AutoDrive.currentColors.ad_color_hoveredNode, true)
        end
    end
end

function AutoDrive:startAutoDrive()
    if self.isServer then

        if self.spec_locomotive then
            -- fix FS25 #262
            if self.maxRotTime == 0 then
                self.maxRotTime = 1
            end
            if self.minRotTime == 0 then
                self.minRotTime = -1
            end
        end

        if g_currentMission.aiSystem and g_currentMission.aiSystem.activeJobVehicles then
            for index, jobVehicle in pairs(g_currentMission.aiSystem.activeJobVehicles) do
                if self == jobVehicle then
                    AutoDriveMessageEvent.sendMessageOrNotification(self, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s: $l10n_AD_AIJob_active;", 5000, self.ad.stateModule:getName())
                    return
                end
            end
        end

        if not self.ad.stateModule:isActive() then
            self.ad.stateModule:setActive(true)

            self.ad.isStoppingWithError = false
            self.ad.onRouteToPark = false
            AutoDrive.resetFoldState(self)
            AutoDrive.getAllVehicleDimensions(self, true)
            if self.spec_aiVehicle ~= nil then
                if self.getAINeedsTrafficCollisionBox ~= nil then
                    if self:getAINeedsTrafficCollisionBox() then
                        if AIFieldWorker.TRAFFIC_COLLISION ~= nil and AIFieldWorker.TRAFFIC_COLLISION ~= 0 then
                            self.spec_aiVehicle.aiTrafficCollision = AIFieldWorker.TRAFFIC_COLLISION
                        end
                    end
                end
                if self.spec_aiVehicle.aiTrafficCollisionTranslation ~= nil then
                    self.spec_aiVehicle.aiTrafficCollisionTranslation[2] = -1000
                end
            end

            if self.ad.currentHelper == nil or self.ad.stateModule:getCurrentHelperIndex() <= 0 then
                -- no helper assigned
                if #g_helperManager.availableHelpers == 0 then
                    -- assume helper limit over
                    AutoDrive.checkAddHelper(self, nil, 1) -- add 1 helper
                end
                local currentHelper = g_helperManager:getRandomHelper()
                if currentHelper ~= nil then
                    g_helperManager:useHelper(currentHelper)
                end
                if currentHelper == nil then
                    local name = self.getName and self:getName() or ""
                    Logging.error("[AD] AutoDrive:startAutoDrive ERROR: unable to get helper for vehicle %s", tostring(name))
                end
                if currentHelper and currentHelper.index then
                    self.ad.currentHelper = currentHelper
                    self.ad.stateModule:setCurrentHelperIndex(currentHelper.index)
                end
            end

            AutoDriveStartStopEvent:sendStartEvent(self)

        end
    else
        Logging.devError("AutoDrive:startAutoDrive() must be called only on the server.")
    end
end

function AutoDrive:stopAutoDrive()

    if self.isServer then
        ADScheduler:removePathfinderVehicle(self)

        if self.ad.stateModule:isActive() then
            self.ad.drivePathModule:reset()
            self.ad.specialDrivingModule:reset()
            self.ad.trailerModule:reset()

            if self.spec_locomotive and self.ad and self.ad.trainModule then
                self.ad.trainModule:reset()
            end
            for _, mode in pairs(self.ad.modes) do
                mode:reset()
            end

            if self.setBeaconLightsVisibility ~= nil and AutoDrive.getSetting("useBeaconLights", self) then
                self:setBeaconLightsVisibility(false)
            end
            if self.setTurnLightState ~= nil then
                self:setTurnLightState(Lights.TURNLIGHT_OFF)
            end

            if not self.spec_locomotive then
                self.ad.trailerModule:handleTrailerReversing(false)
                AutoDrive.driveInDirection(self, 16, 30, 0, 0.2, 20, false, self.ad.drivingForward, 0, 0, 0, 1)
                self:setCruiseControlState(Drivable.CRUISECONTROL_STATE_OFF)
            end

            if self.ad.onRouteToPark then
                SpecializationUtil.raiseEvent(self, "onAutoDriveParked")
            end

            if not AutoDrive:getIsEntered(self) and not self.ad.isStoppingWithError then --self.ad.onRouteToPark and
                self.ad.onRouteToPark = false
            end

            if not self.spec_locomotive then
                if self.ad.sensors ~= nil then
                    for _, sensor in pairs(self.ad.sensors) do
                        sensor:setEnabled(false)
                    end
                end
            end
            if self.spec_aiVehicle and self.spec_aiVehicle.aiTrafficCollisionTranslation ~= nil then
                self.spec_aiVehicle.aiTrafficCollisionTranslation[2] = 0
            end

            self.ad.stateModule:setActive(false)

            self.ad.taskModule:abortAllTasks()
            self.ad.taskModule:reset()

            self.ad.isCpEmpty = false
            self.ad.isCpFull = false
            self.ad.isAIJobFinished = false

            if self.ad.isStoppingWithError == true then
                self.ad.onRouteToRefuel = false
                self.ad.onRouteToRepair = false
                AutoDrive.debugPrint(self, AutoDrive.DC_VEHICLEINFO, "AutoDrive:startAutoDrive self.ad.onRouteToRefuel %s", tostring(self.ad.onRouteToRefuel))
            end
            AutoDrive.updateAutoDriveLights(self, true)

            local isStartingAIVE = not self.ad.isStoppingWithError and self.ad.stateModule:getStartHelper() and self.ad.stateModule:getUsedHelper() == ADStateModule.HELPER_AIVE
            local isPassingToCP = not self.ad.isStoppingWithError and self.ad.stateModule:getStartHelper() and self.ad.stateModule:getUsedHelper() == ADStateModule.HELPER_CP
            local isPassingToAI = not self.ad.isStoppingWithError and self.ad.stateModule:getStartHelper() and self.ad.stateModule:getUsedHelper() == ADStateModule.HELPER_AI

            if not isStartingAIVE and not isPassingToCP and not isPassingToAI then
                if not AutoDrive:getIsEntered(self) then
                    if self.deactivateLights ~= nil then
                        self:deactivateLights()
                    end

                    if self.spec_locomotive then
                        if self.setCruiseControlState then
                            self:setCruiseControlState(Drivable.CRUISECONTROL_STATE_OFF)
                            self:updateVehiclePhysics(0, 0, 0, 16)
                            self:raiseActive()
                        end
                    end

                    if self.stopMotor ~= nil then
                        self:stopMotor()
                    end
                end
            end

            AutoDriveStartStopEvent:sendStopEvent(self, isPassingToCP or isPassingToAI, isStartingAIVE)

			-- currently the pass to CP is only working correct from this call
            if self.ad.stateModule:getStartHelper() then
                if  self.ad.stateModule:getUsedHelper() == ADStateModule.HELPER_CP then
                    AutoDrive.passToExternalMod_CP(self)
                elseif  self.ad.stateModule:getUsedHelper() == ADStateModule.HELPER_AIVE then
                    AutoDrive.passToExternalMod_AIVE(self)
                elseif  self.ad.stateModule:getUsedHelper() == ADStateModule.HELPER_AI then
                    AutoDrive.passToExternalMod_AI(self)
                end
            end
        end
    else
        Logging.devError("AutoDrive:stopAutoDrive() must be called only on the server.")
    end
end

function AutoDrive:onStartAutoDrive()
    self.forceIsActive = true
    self.spec_motorized.stopMotorOnLeave = false
    self.spec_enterable.disableCharacterOnLeave = false

    if self.spec_motorized.motor ~= nil then
        self.spec_motorized.motor:setGearShiftMode(VehicleMotor.SHIFT_MODE_AUTOMATIC)
    end

    if self.setRandomVehicleCharacter ~= nil then
        local helperIndex = self.ad.stateModule:getCurrentHelperIndex()
        if helperIndex > 0 then
            local helper = g_helperManager:getHelperByIndex(helperIndex)
            self:setRandomVehicleCharacter(helper)
        else
-- TODO: event is received before stateModule update, so use a random character as fallback
            self:setRandomVehicleCharacter()
        end
    end

    AutoDriveHud:createMapHotspot(self)

    if g_server ~= nil then
        if AutoDrive.getSetting("enableParkAtJobFinished", self) and ((self.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER) or (self.ad.stateModule:getMode() == AutoDrive.MODE_DELIVERTO)) then
            local actualParkDestination = self.ad.stateModule:getParkDestinationAtJobFinished()
            if actualParkDestination >= 1 then
            else
                AutoDriveMessageEvent.sendMessage(self, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_parkVehicle_noPosSet;", 5000, self.ad.stateModule:getName())
            end
        end
    end
end

function AutoDrive.checkAddHelper(vehicle, helperIndex, numHelpersToAdd)
    local source = g_helperManager.indexToHelper[1]
    local numToAdd = numHelpersToAdd or 0
    if helperIndex and helperIndex > 0 and g_helperManager.numHelpers < helperIndex then
        numToAdd = helperIndex - g_helperManager.numHelpers
    end
    if numToAdd > 0 then
        for i = 1, numToAdd do
            g_helperManager.numHelpers = g_helperManager.numHelpers + 1
            local helper = {}
            -- helper.name = source.name .. "_" .. math.random(100, 1000)
            helper.name = source.name .. "_" .. g_helperManager.numHelpers
            helper.index = g_helperManager.numHelpers
            helper.color = source.color
            helper.title = helper.name
            helper.playerStyle = source.playerStyle
            g_helperManager.helpers[helper.name] = helper
            g_helperManager.nameToIndex[helper.name] = g_helperManager.numHelpers
            g_helperManager.indexToHelper[g_helperManager.numHelpers] = helper
            table.insert(g_helperManager.availableHelpers, helper)
            g_currentMission.maxNumHirables = g_currentMission.maxNumHirables + 1;
        end
    end
    local helper = nil
    if helperIndex and helperIndex > 0 then
        helper = g_helperManager:getHelperByIndex(helperIndex)
    end
    if vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.stateModule ~= nil then
        if vehicle.ad.stateModule:isActive() then
            if vehicle.ad.currentHelper == nil or vehicle.ad.stateModule:getCurrentHelperIndex() <= 0 then
                -- no proper helper set
                vehicle.ad.currentHelper = helper
                if helper ~= nil then
                    g_helperManager:useHelper(helper)
                    vehicle.ad.stateModule:setCurrentHelperIndex(helper.index)
                end
            end
        else
            vehicle.ad.currentHelper = nil
            vehicle.ad.stateModule:setCurrentHelperIndex(0)
        end
    end
end

function AutoDrive:onStopAutoDrive(isPassingToCP, isStartingAIVE)

	if not isPassingToCP then
        self.forceIsActive = false
        self.spec_motorized.stopMotorOnLeave = true
        self.spec_enterable.disableCharacterOnLeave = true
        if self.restoreVehicleCharacter ~= nil then
            self:restoreVehicleCharacter()
        end
    end

        if self.ad.currentHelper ~= nil then
            g_helperManager:releaseHelper(self.ad.currentHelper)
        end
        self.ad.currentHelper = nil
        self.ad.stateModule:setCurrentHelperIndex(0)

        if self.spec_motorized.motor ~= nil then
            self.spec_motorized.motor:setGearShiftMode(self.spec_motorized.gearShiftMode)
        end

    -- In case we get this event before the status has been updated with the readStream
    if self.ad.stateModule:isActive() then
        -- Set this to false without raising flags. The update should already be on the wire
        -- Otherwise this following requestActionEventUpdate will not allow user input since the AI is still active
        self.ad.stateModule.active = false
    end

    self:requestActionEventUpdate()

    AutoDriveHud:deleteMapHotspot(self)

    if self.isServer then
    	-- currently not working for CP!
        -- AutoDrive.passToExternalMod(self)
    end
end

function AutoDrive.passToExternalMod_CP(vehicle)
    if vehicle == nil or vehicle.ad == nil or vehicle.ad.stateModule == nil then
        return
    end
    local x, y, z = getWorldTranslation(vehicle.components[1].node)

    local point = nil
    local distanceToStart = math.huge
    if
        vehicle.ad ~= nil and ADGraphManager.getWayPointById ~= nil and vehicle.ad.stateModule ~= nil and vehicle.ad.stateModule.getFirstMarker ~= nil and vehicle.ad.stateModule:getFirstMarker() ~= nil and vehicle.ad.stateModule:getFirstMarker() ~= 0 and
            vehicle.ad.stateModule:getFirstMarker().id ~= nil
     then
        point = ADGraphManager:getWayPointById(vehicle.ad.stateModule:getFirstMarker().id)
        if point ~= nil then
            distanceToStart = MathUtil.vector2Length(x - point.x, z - point.z)
        end
    end
    -- TODO: check if this dirty hack works in future!
    local isControlled = vehicle:getIsControlled()

    if not vehicle.ad.isStoppingWithError and distanceToStart < 30 then
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive.passToExternalMod_CP")
        -- CP button enabled
        if (vehicle.cpStartStopDriver ~= nil and vehicle.ad.stateModule:getUsedHelper() == ADStateModule.HELPER_CP) then
            -- CP button active
            vehicle.spec_enterable.isControlled = false
            if vehicle.ad.restartCP == true then
                -- restart CP to continue
                vehicle.ad.restartCP = false
                AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive.passToExternalMod_CP pass control to CP with restart")
                AutoDrive:RestartCP(vehicle)
            else
                -- start CP from beginning
                AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive.passToExternalMod_CP pass control to CP with start")
                AutoDrive:StartCP(vehicle)
            end
            vehicle.spec_enterable.isControlled = isControlled
        end
    end
end

function AutoDrive.passToExternalMod_AIVE(vehicle)
    if vehicle == nil or vehicle.ad == nil or vehicle.ad.stateModule == nil then
        return
    end
    local x, _, z = getWorldTranslation(vehicle.components[1].node)

    local point = nil
    local distanceToStart = math.huge
    if
        vehicle.ad ~= nil and ADGraphManager.getWayPointById ~= nil and vehicle.ad.stateModule ~= nil and vehicle.ad.stateModule.getFirstMarker ~= nil and vehicle.ad.stateModule:getFirstMarker() ~= nil and vehicle.ad.stateModule:getFirstMarker() ~= 0 and
            vehicle.ad.stateModule:getFirstMarker().id ~= nil
     then
        point = ADGraphManager:getWayPointById(vehicle.ad.stateModule:getFirstMarker().id)
        if point ~= nil then
            distanceToStart = MathUtil.vector2Length(x - point.x, z - point.z)
        end
    end

    if not vehicle.ad.isStoppingWithError and distanceToStart < 30 then
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive.passToExternalMod_AIVE AIVE button active")
        -- AIVE button active
        if vehicle.acParameters ~= nil then
            vehicle.ad.stateModule:setStartHelper(false)  -- disable helper button
            vehicle.acParameters.enabled = true
            AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive.passToExternalMod_AIVE pass control to AIVE with startAIVehicle")
            vehicle:toggleAIVehicle()
        end
    end
end

function AutoDrive.passToExternalMod_AI(vehicle)
    if vehicle == nil or vehicle.ad == nil or vehicle.ad.stateModule == nil then
        return
    end

    local x, y, z = getWorldTranslation(vehicle.components[1].node)

    local point = nil
    local distanceToStart = math.huge
    if
        vehicle.ad ~= nil and ADGraphManager.getWayPointById ~= nil and vehicle.ad.stateModule ~= nil and vehicle.ad.stateModule.getFirstMarker ~= nil 
        and vehicle.ad.stateModule:getFirstMarker() ~= nil and vehicle.ad.stateModule:getFirstMarker() ~= 0 and vehicle.ad.stateModule:getFirstMarker().id ~= nil
     then
        point = ADGraphManager:getWayPointById(vehicle.ad.stateModule:getFirstMarker().id)
        if point ~= nil then
            distanceToStart = MathUtil.vector2Length(x - point.x, z - point.z)
        end
    end

    if (not vehicle.ad.isStoppingWithError and distanceToStart < 30) then
        local success, errorMessage
        if vehicle.getLastJob then
            if (vehicle.getIsOnField and vehicle:getIsOnField()) then
                AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive.passToExternalMod pass to other mod...")
                local fieldJob = vehicle:getLastJob()
                if fieldJob == nil then
                    -- no job present - generate fielwork job new
                    fieldJob = g_currentMission.aiJobTypeManager:createJob(AIJobType.FIELDWORK)
                end
                if fieldJob then
                        -- job present - continue fielwork job
                    fieldJob:applyCurrentState(vehicle, g_currentMission, vehicle:getOwnerFarmId(), false)
                    fieldJob:setValues()
                    success, errorMessage = fieldJob:validate(vehicle:getOwnerFarmId())
                    if success then
                        AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive.passToExternalMod lastJob aiSystem:startJob startJob")
                        g_currentMission.aiSystem:startJob(fieldJob, vehicle:getOwnerFarmId())
                        return
                    end
                end
            end
        else
            return
        end
        AutoDriveMessageEvent.sendMessageOrNotification(vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s: $l10n_AD_Unable_to_start_AIJob;", 5000, vehicle.ad.stateModule:getName())
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDrive.passToExternalMod lastJob aiSystem:startJob errorMessage: %s"
        , tostring(errorMessage)
        )
    end
end

function AutoDrive:adGetRemainingDriveTime()
    if not self.spec_locomotive and self.ad and self.ad.stateModule and self.ad.stateModule:isActive() then
        return self.ad.stateModule:adGetRemainingDriveTime()
    end
    return 0
end

function AutoDrive:updateWayPointsDistance(x, z)
    self.ad.distances.wayPoints = {}
    self.ad.distances.closest.wayPoint = nil
    self.ad.distances.closest.distance = math.huge
    self.ad.distances.closestNotReverse.wayPoint = nil
    self.ad.distances.closestNotReverse.distance = math.huge

    if x == nil or z == nil then
        x, _, z = getWorldTranslation(self.components[1].node)
    end

    --We should see some perfomance increase by localizing the sqrt/pow functions right here
    local sqrt = math.sqrt
    local distanceFunc = function(a, b)
        return sqrt(a * a + b * b)
    end
    for _, wp in pairs(ADGraphManager:getWayPoints()) do
        local distance = distanceFunc(wp.x - x, wp.z - z)
        if distance < self.ad.distances.closest.distance then
            self.ad.distances.closest.distance = distance
            self.ad.distances.closest.wayPoint = wp
        end
        if distance <= AutoDrive.drawDistance then
            table.insert(self.ad.distances.wayPoints, {distance = distance, wayPoint = wp})
        end
        if distance < self.ad.distances.closestNotReverse.distance and (wp.incoming == nil or #wp.incoming > 0) then
            self.ad.distances.closestNotReverse.distance = distance
            self.ad.distances.closestNotReverse.wayPoint = wp
        end
    end
end

function AutoDrive:resetClosestWayPoint()
    self.ad.distances.closest.wayPoint = -1
end

function AutoDrive:resetWayPointsDistance()
    self.ad.distances.wayPoints = nil
end

function AutoDrive:getWayPointsDistance()
    return self.ad.distances.wayPoints
end

function AutoDrive:updateClosestWayPoint()
    if self.ad.distances.wayPoints == nil then
        self:updateWayPointsDistance()
    end
    self.ad.distances.closest.wayPoint = nil
    self.ad.distances.closest.distance = math.huge
    self.ad.distances.closestNotReverse.wayPoint = nil
    self.ad.distances.closestNotReverse.distance = math.huge

    if self.ad.distances.wayPoints == nil then
        -- something went wrong, so exit
        return
    end
    local x, _, z = getWorldTranslation(self.components[1].node)

    --We should see some perfomance increase by localizing the sqrt/pow functions right here
    local sqrt = math.sqrt
    local distanceFunc = function(a, b)
        return sqrt(a * a + b * b)
    end
    for _, elem in pairs(self.ad.distances.wayPoints) do
        local wp = elem.wayPoint
        local distance = distanceFunc(wp.x - x, wp.z - z)
        if distance < self.ad.distances.closest.distance then
            self.ad.distances.closest.distance = distance
            self.ad.distances.closest.wayPoint = wp
        end
        if distance < self.ad.distances.closestNotReverse.distance and (wp.incoming == nil or #wp.incoming > 0) then
            self.ad.distances.closestNotReverse.distance = distance
            self.ad.distances.closestNotReverse.wayPoint = wp
        end
    end
end

-- update distances only if not called in (frame) update functions
function AutoDrive:getClosestWayPoint(noUpdate)
    if noUpdate == nil or noUpdate == false then
        -- update on request function calls - force all update
        self:updateWayPointsDistance()
    end
    if self.ad.distances.closest.wayPoint == nil or noUpdate == true then
        -- get closest wayPoint in view distance -> perfomance improvement
        self:updateClosestWayPoint()
    end
    if self.ad.distances.closest.wayPoint ~= nil then
        return self.ad.distances.closest.wayPoint.id, self.ad.distances.closest.distance
    end
    return -1, math.huge
end

function AutoDrive:getClosestNotReversedWayPoint()
    self:updateWayPointsDistance()
    if self.ad.distances.closestNotReverse.wayPoint ~= nil then
        return self.ad.distances.closestNotReverse.wayPoint.id, self.ad.distances.closestNotReverse.distance
    end
    return -1, math.huge
end

function AutoDrive:getWayPointsInRange(minDistance, maxDistance)
    if self.ad.distances.wayPoints == nil then
        self:updateWayPointsDistance()
    end
    local inRange = {}
    for _, elem in pairs(self.ad.distances.wayPoints) do
        if elem.distance >= minDistance and elem.distance <= maxDistance and elem.wayPoint.id > 0 then
            table.insert(inRange, elem.wayPoint)
        end
    end
    return inRange
end

function AutoDrive:getWayPointIdsInRange(minDistance, maxDistance)
    self:updateWayPointsDistance()
    local inRange = {}
    for _, elem in pairs(self.ad.distances.wayPoints) do
        if elem.distance >= minDistance and elem.distance <= maxDistance and elem.wayPoint.id > 0 then
            table.insert(inRange, elem.wayPoint.id)
        end
    end
    return inRange
end

function AutoDrive:toggleMouse()
    if g_inputBinding:getShowMouseCursor() then
        if self.spec_enterable ~= nil and self.spec_enterable.cameras ~= nil then
            for _, camera in pairs(self.spec_enterable.cameras) do
                camera.storedAllowTranslation = camera.allowTranslation
                --camera.storedIsRotatable = camera.isRotatable
                camera.allowTranslation = false
                camera.isRotatable = false
            end
        end
    else
        if self.spec_enterable ~= nil and self.spec_enterable.cameras ~= nil then
            for _, camera in pairs(self.spec_enterable.cameras) do
                if camera.storedAllowTranslation ~= nil then
                    camera.allowTranslation = camera.storedAllowTranslation
                else
                    camera.allowTranslation = true
                end
                if camera.storedIsRotatable ~= nil then
                    camera.isRotatable = camera.storedIsRotatable
                else
                    camera.isRotatable = true
                end
            end
        end

        AutoDrive.resetMouseSelections(self)
    end
    self.ad.lastMouseState = g_inputBinding:getShowMouseCursor()
end

function AutoDrive:updateAutoDriveLights(switchOff)
    if not self.setTurnLightState then
        AutoDrive.errorMsg(self, "AutoDrive:updateAutoDriveLights self.setTurnLightState %s", tostring(self.setTurnLightState))
    end
    if switchOff then
        if AutoDrive.getSetting("useHazardLightReverse", self) and self.setTurnLightState then
            self:setTurnLightState(Lights.TURNLIGHT_OFF)
        end
    elseif self.ad ~= nil and self.ad.stateModule:isActive() then
        local isInRangeToLoadUnloadTarget = false
        local isInBunkerSilo              = false
        local isOnField                   = ( self.getIsOnField ~= nil and self:getIsOnField() )

        if AutoDrive.getSetting("useWorkLightsLoading", self) then
            isInRangeToLoadUnloadTarget = AutoDrive.isInRangeToLoadUnloadTarget(self)
        end

        if AutoDrive.getSetting("useWorkLightsSilo", self) then
            isInBunkerSilo = AutoDrive.isVehicleInBunkerSiloArea(self)
        end

        if self.updateAutomaticLights ~= nil then
            self:updateAutomaticLights(not g_currentMission.environment.isSunOn, isOnField or isInRangeToLoadUnloadTarget or isInBunkerSilo)
        end

        if self.setTurnLightState then
            if AutoDrive.getSetting("useHazardLightReverse", self) then
                local drivingReverse = (self.lastSpeedReal * self.movingDirection) < 0
                local isReverseDrivingMode = AutoDrive.isReverseDriving(self)
                if drivingReverse ~= isReverseDrivingMode then
                    self:setTurnLightState(Lights.TURNLIGHT_HAZARD, true)
                elseif self.lastSpeedReal * 3600 < 0.1 then
                    self:setTurnLightState(Lights.TURNLIGHT_OFF)
                end
            end
        end
    end
end

function AutoDrive:getCanMotorRun(superFunc)
    if self.ad ~= nil and self.ad.stateModule:isActive() and self.ad.specialDrivingModule:shouldStopMotor() then
        return false
    else
        return superFunc(self)
    end
end

function AutoDrive:getIsAIActive(superFunc)
    if self.ad and self.ad.typeIsConveyorBelt and self.getAttacherVehicle and self:getAttacherVehicle() then
        -- conveyor belt attached to vehicle - report as not active
        return false
    end

    return superFunc(self) or (self.ad and self.ad.stateModule and self.ad.stateModule:isActive())
end

function AutoDrive:getIsVehicleControlledByPlayer(superFunc)
    return superFunc(self) and not self.ad.stateModule:isActive()
end

function AutoDrive:getActiveFarm(superFunc)
    if self.ad and self.ad.stateModule and self.ad.stateModule:isActive() then
        local actualFarmID = self.ad.stateModule:getActualFarmId()
        if actualFarmID > FarmManager.SPECTATOR_FARM_ID then
            -- return farmID only for valid farms, not spectator farm
            return actualFarmID
        end
    end
    return superFunc(self)
end

function AutoDrive:generateUTurn(left)
    if self.ad.uTurn == nil then
        self.ad.uTurn = {}
        self.ad.uTurn.expectedColliCallbacks = 0
        self.ad.uTurn.inProgress = false
        self.ad.uTurn.doneChecking = true
    end
    if not self.ad.uTurn.inProgress then
        self.ad.uTurn.doneChecking = false
        self.ad.uTurn.inProgress = true

        local radius = AutoDrive.getDriverRadius(self, true)
        local vehX, vehY, vehZ = getWorldTranslation(self.components[1].node)
        local resolution = 20

        -- Determine area to check
        local points = {}
        if left then
            for i = 1, (resolution + 1) do
                local circlePoint = {   x = -math.cos((i-1) * math.pi / resolution) * radius + radius,
                                        y = math.sin((i-1) * math.pi / resolution) * radius }
                local worldX, _, worldZ = AutoDrive.localToWorld(self, circlePoint.x, 0, circlePoint.y)
                local point = { x = worldX, y = vehY, z = worldZ }
                local rayCastResult = AutoDrive:getTerrainHeightAtWorldPos(worldX, worldZ)
                point.y = rayCastResult or point.y
                local dummy = 1
                for i = 1, 1000 do
                    dummy = dummy + i
                end
                point.y = AutoDrive.raycastHeight or point.y

                table.insert(points, point)
            end
            local worldX, _, worldZ = AutoDrive.localToWorld(self, 2*radius, 0, -1)
            table.insert(points, {x=worldX, y=vehY, z=worldZ})
            worldX, _, worldZ = AutoDrive.localToWorld(self, 2*radius, 0, -4)
            table.insert(points, {x=worldX, y=vehY, z=worldZ})
            worldX, _, worldZ = AutoDrive.localToWorld(self, 2*radius, 0, -8)
            table.insert(points, {x=worldX, y=vehY, z=worldZ})
        else
            for i = 1, (resolution + 1) do
                local circlePoint = {   x = math.cos((i-1) * math.pi / resolution) * radius - radius,
                                        y = math.sin((i-1) * math.pi / resolution) * radius }
                local worldX, _, worldZ = AutoDrive.localToWorld(self, circlePoint.x, 0, circlePoint.y)
                local point = { x = worldX, y = vehY, z = worldZ }
                local rayCastResult = AutoDrive:getTerrainHeightAtWorldPos(worldX, worldZ)
                point.y = rayCastResult or point.y
                local dummy = 1
                for i = 1, 1000 do
                    dummy = dummy + i
                end
                point.y = AutoDrive.raycastHeight or point.y

                table.insert(points, point)
            end
            local worldX, _, worldZ = AutoDrive.localToWorld(self, -2*radius, 0, -1)
            table.insert(points, {x=worldX, y=vehY, z=worldZ})
            worldX, _, worldZ = AutoDrive.localToWorld(self, -2*radius, 0, -4)
            table.insert(points, {x=worldX, y=vehY, z=worldZ})
            worldX, _, worldZ = AutoDrive.localToWorld(self, -2*radius, 0, -8)
            table.insert(points, {x=worldX, y=vehY, z=worldZ})
        end

        --- Coll check:
        local widthX = self.size.width / 1.75
        local height = 2.3

        local mask = AutoDrive.collisionMaskTerrain

        self.ad.uTurn.expectedColliCallbacks = 0
        self.ad.uTurn.colliFound = false
        self.ad.uTurn.points = points

        for i, wp in pairs(points) do
            if i > 1 and i < (#points - 1) then
                local wpLast = points[i - 1]
                local deltaX, deltaY, deltaZ = wp.x - wpLast.x, wp.y - wpLast.y, wp.z - wpLast.z
                local centerX, centerY, centerZ = wpLast.x + deltaX/2,  wpLast.y + deltaY/2,  wpLast.z + deltaZ/2
                local angleRad = math.atan2(deltaX, deltaZ)
                angleRad = AutoDrive.normalizeAngle(angleRad)
                local length = MathUtil.vector2Length(deltaX, deltaZ) / 2

                local angleX = -MathUtil.getYRotationFromDirection(deltaY, length*2)

                local shapes = overlapBox(centerX, centerY+3, centerZ, angleX, angleRad, 0, widthX, height, length, "collisionTestCallback", self, mask, true, true, true, true)
                if shapes > 0 then
                    self.ad.uTurn.expectedColliCallbacks = self.ad.uTurn.expectedColliCallbacks + 1
                end
                --[[
                local r,g,b = 0,1,0
                if shapes > 0 then
                    r = 1
                    g = 0
                    DebugUtil.drawOverlapBox(centerX, centerY+3, centerZ, angleX, angleRad, 0, widthX, height, length, r, g, b)
                end
                DebugUtil.drawOverlapBox(centerX, centerY+3, centerZ, angleX, angleRad, 0, widthX, height, length, r, g, b)
                --]]
            end
        end
    elseif self.ad.uTurn.inProgress then
        local r,g,b = 0,1,0
        if self.ad.uTurn.colliFound then
            r = 1
            g = 0
        end

        for i, p in ipairs(self.ad.uTurn.points) do
            if i > 1 then
                ADDrawingManager:addLineTask(self.ad.uTurn.points[i-1].x, self.ad.uTurn.points[i-1].y, self.ad.uTurn.points[i-1].z, p.x, p.y, p.z, 1, r, g, b)
                ADDrawingManager:addArrowTask(self.ad.uTurn.points[i-1].x, self.ad.uTurn.points[i-1].y, self.ad.uTurn.points[i-1].z, p.x, p.y, p.z, 1, ADDrawingManager.arrows.position.middle, unpack(AutoDrive.currentColors.ad_color_subPrioSingleConnection))
            end
        end

        if self.ad.uTurn.doneChecking then
            self.ad.uTurn.inProgress = false
        end
    end

    -- Coll check with large box
    --local centerX, centerY, centerZ = AutoDrive.localToWorld(vehicle, radius, 0, radius/2)
    --local shapes = overlapBox(centerX, centerY+3, centerZ, angleX, angleRad, 0, widthX, height, length, "collisionTestCallbackIgnore", nil, mask, true, true, true)
end

function AutoDrive:collisionTestCallback(transformId, x, y, z, distance)
    self.ad.uTurn.expectedColliCallbacks = math.max(0, self.ad.uTurn.expectedColliCallbacks - 1)
    if transformId ~= 0 and transformId ~= g_currentMission.terrainRootNode then
        if g_currentMission.nodeToObject[transformId] ~= nil then
            if g_currentMission.nodeToObject[transformId] ~= self and not AutoDrive:checkIsConnected(self, g_currentMission.nodeToObject[transformId]) then
                self.ad.uTurn.colliFound = true
            end
        else
            self.ad.uTurn.colliFound = true
        end

        if self.ad.uTurn.inProgress and (self.ad.uTurn.expectedColliCallbacks == 0 or self.ad.uTurn.colliFound) then
            self.ad.uTurn.doneChecking = true
        end
    end
end

