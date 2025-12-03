AutoDrive = {}
AutoDrive.version = "3.0.0.8"

AutoDrive.directory = g_currentModDirectory

g_autoDriveUIConfigPath = AutoDrive.directory .. "textures/ad_gui.xml"


AutoDrive.experimentalFeatures = {}

AutoDrive.automaticUnloadTarget = false
AutoDrive.automaticPickupTarget = false

AutoDrive.dynamicChaseDistance = true
AutoDrive.smootherDriving = true
AutoDrive.developmentControls = false

AutoDrive.mapHotspotsBuffer = {}

AutoDrive.drawHeight = 0.3
AutoDrive.drawDistance = getViewDistanceCoeff() * 100

AutoDrive.MODE_DRIVETO = 1
AutoDrive.MODE_PICKUPANDDELIVER = 2
AutoDrive.MODE_DELIVERTO = 3
AutoDrive.MODE_LOAD = 4
AutoDrive.MODE_UNLOAD = 5
AutoDrive.MODE_BGA = 6

AutoDrive.DC_NONE = 0
AutoDrive.DC_VEHICLEINFO = 1
AutoDrive.DC_COMBINEINFO = 2
AutoDrive.DC_TRAILERINFO = 4
AutoDrive.DC_DEVINFO = 8
AutoDrive.DC_PATHINFO = 16
AutoDrive.DC_SENSORINFO = 32
AutoDrive.DC_NETWORKINFO = 64
AutoDrive.DC_EXTERNALINTERFACEINFO = 128
AutoDrive.DC_RENDERINFO = 256
AutoDrive.DC_ROADNETWORKINFO = 512
AutoDrive.DC_BGA_MODE = 1024
AutoDrive.DC_TRAINS = 2048
AutoDrive.DC_ALL = 65535

AutoDrive.currentDebugChannelMask = AutoDrive.DC_NONE

-- rotate target modes
AutoDrive.RT_NONE = 1
AutoDrive.RT_ONLYPICKUP = 2
AutoDrive.RT_ONLYDELIVER = 3
AutoDrive.RT_PICKUPANDDELIVER = 4

AutoDrive.EDITOR_OFF = 1
AutoDrive.EDITOR_ON = 2
AutoDrive.EDITOR_EXTENDED = 3
AutoDrive.EDITOR_SHOW = 4

AutoDrive.SCAN_DIALOG_NONE = 0
AutoDrive.SCAN_DIALOG_OPEN = 1
AutoDrive.SCAN_DIALOG_RESULT_YES = 2
AutoDrive.SCAN_DIALOG_RESULT_NO = 3
AutoDrive.SCAN_DIALOG_RESULT_DONE = 4
AutoDrive.scanDialogState = AutoDrive.SCAN_DIALOG_NONE

AutoDrive.foldTimeout = 30000         -- 30 s time to fold all implements
AutoDrive.MAX_BUNKERSILO_LENGTH = 100 -- length of bunker silo where speed should be lowered

-- number of frames for performance modulo operation
AutoDrive.PERF_FRAMES = 20
AutoDrive.PERF_FRAMES_HIGH = 4

AutoDrive.toggleSphere = true
AutoDrive.enableSphere = true

AutoDrive.FLAG_NONE = 0
AutoDrive.FLAG_SUBPRIO = 1
AutoDrive.FLAG_TRAFFIC_SYSTEM = 2
AutoDrive.FLAG_TRAFFIC_SYSTEM_CONNECTION = 4

-- add this to measured size of vehicles
AutoDrive.DIMENSION_ADDITION = 0.2

-- AD invoked by which type of user
AutoDrive.USER_PLAYER = 1
AutoDrive.USER_GIANTS = 2
AutoDrive.USER_CP = 3

AutoDrive.CHASEPOS_UNKNOWN = 0
AutoDrive.CHASEPOS_AUTO = 5
AutoDrive.CHASEPOS_LEFT = 1
AutoDrive.CHASEPOS_RIGHT = -1
AutoDrive.CHASEPOS_REAR = 3
AutoDrive.CHASEPOS_FRONT = 4

AutoDrive.UAL_FILLTYPE_ALL = 1 -- value for set all materials in UniversalAutoload!

AutoDrive.MAX_REFUEL_TRIGGER_DISTANCE = 15
AutoDrive.REFUEL_LEVEL = 0.15
AutoDrive.BUNKERSILO_CONNECTED_DISTANCE = 10

AutoDrive.colors = {
	ad_color_singleConnection = { 0, 1, 0, 1 },
	ad_color_dualConnection = { 0, 0, 1, 1 },
	ad_color_reverseConnection = { 0, 0.569, 0.835, 1 },
	ad_color_default = { 1, 0, 0, 0.3 },
	ad_color_subPrioSingleConnection = { 1, 0.531, 0.14, 1 },
	ad_color_subPrioDualConnection = { 0.389, 0.177, 0, 1 },
	ad_color_subPrioNode = { 1, 0.531, 0.14, 0.3 },
	ad_color_hoveredNode = { 0, 0, 1, 0.15 },
	ad_color_previousNode = { 1, 0.2195, 0.6524, 0.5 }, --GOLDHOFER_PINK1
	ad_color_nextNode = { 1, 0.7, 0, 0.5 },
	ad_color_selectedNode = { 0, 1, 0, 0.15 },
	ad_color_currentConnection = { 1, 1, 1, 1 },
	ad_color_closestLine = { 1, 0, 0, 1 },
	ad_color_editorHeightLine = { 1, 1, 1, 1 },
	ad_color_previewSingleConnection = { 0.3, 0.9, 0, 1 },
	ad_color_previewDualConnection = { 0, 0, 0.9, 1 },
	ad_color_previewSubPrioSingleConnection = { 0.9, 0.4, 0.1, 1 },
	ad_color_previewSubPrioDualConnection = { 0.3, 0.15, 0, 1 },
	ad_color_previewOk = { 0.3, 0.9, 0, 1 },
	ad_color_previewNotOk = { 1, 0.1, 0, 1 },
	ad_color_textInputBackground = { 0.0227, 0.5346, 0.8519, 1 }, -- Giants original
	ad_color_hudTextDefault = { 1, 1, 1, 1 },
	ad_color_hudTextHover = { 0.51, 0.67, 0.05, 1 },
	ad_color_hudTextSpecial = { 0.66, 0.83, 0.34, 1 },
	ad_color_hudTextHoverSpecial = { 0.45, 0.73, 0.05, 1 },

}

AutoDrive.currentColors = {} -- this will hold the current colors, derived from default colors above, overwritten by local settings

AutoDrive.fuelFillTypes = {
	"DIESEL",
	"METHANE",
	"ELECTRICCHARGE",
	"DEF",
	"AIR"
}

AutoDrive.nonFillableFillTypes = { -- these fillTypes should not be transported
	"AIR"
}

AutoDrive.seedFillTypes = {
	'SEEDS',
	'FERTILIZER',
	'LIQUIDFERTILIZER'
}

AutoDrive.modesToStartFromCP = {
	-- AutoDrive.MODE_DRIVETO, not allowed
	AutoDrive.MODE_PICKUPANDDELIVER,
	-- AutoDrive.MODE_DELIVERTO, not allowed
	AutoDrive.MODE_LOAD,
	AutoDrive.MODE_UNLOAD
	-- AutoDrive.MODE_BGA not allowed
}

function AutoDrive:restartMySavegame()
	if g_server then
		restartApplication(true, " -autoStartSavegameId 1")
	end
end

function AutoDrive:loadMap(name)
	Logging.info("[AD] Start register later loaded mods...")
	-- second iteration to register AD to vehicle types which where loaded after AD
	AutoDriveRegister.registerAutoDrive()
	AutoDriveRegister.registerVehicleData()
	AutoDriveRegister.registerPlaceableData()
	Logging.info("[AD] Start register later loaded mods end")

	addConsoleCommand('restartMySavegame', 'Restart my savegame', 'restartMySavegame', self)

	if g_server ~= nil then
		AutoDrive.AutoDriveSync = AutoDriveSync.new(g_server ~= nil, g_client ~= nil)
		AutoDrive.AutoDriveSync:register(false)
	end

	AutoDrive:loadGUI()

	Logging.info("[AD] Map title: %s", g_currentMission.missionInfo.map.title)

	AutoDrive.loadedMap = g_currentMission.missionInfo.map.title
	AutoDrive.loadedMap = string.gsub(AutoDrive.loadedMap, " ", "_")
	AutoDrive.loadedMap = string.gsub(AutoDrive.loadedMap, "%.", "_")
	AutoDrive.loadedMap = string.gsub(AutoDrive.loadedMap, ",", "_")
	AutoDrive.loadedMap = string.gsub(AutoDrive.loadedMap, ":", "_")
	AutoDrive.loadedMap = string.gsub(AutoDrive.loadedMap, ";", "_")
	AutoDrive.loadedMap = string.gsub(AutoDrive.loadedMap, "'", "_")

	Logging.info("[AD] Parsed map title: %s", AutoDrive.loadedMap)

	-- That's probably bad, but for the moment I can't find another way to know if development controls are enabled
	local gameXmlFilePath = getUserProfileAppPath() .. "game.xml"
	if fileExists(gameXmlFilePath) then
		local gameXmlFile = loadXMLFile("game_XML", gameXmlFilePath)
		if gameXmlFile ~= nil then
			if hasXMLProperty(gameXmlFile, "game.development.controls") then
				AutoDrive.developmentControls = Utils.getNoNil(getXMLBool(gameXmlFile, "game.development.controls"), AutoDrive.developmentControls)
			else
				AutoDrive.errorMsg(nil, "AutoDrive:loadMap game.development.controls not found!")
			end
		else
			AutoDrive.errorMsg(nil, "AutoDrive:loadMap could not load ->%s<- !", tostring(gameXmlFilePath))
		end
	else
		AutoDrive.errorMsg(nil, "AutoDrive:loadMap file not exist ->%s<- !", tostring(gameXmlFilePath))
	end

	-- calculate the collision masks only once
	AutoDrive.collisionMaskTerrain = ADCollSensor.getMask()
	AutoDrive.collisionMaskVehicleDimesions = CollisionFlag.VEHICLE

	ADGraphManager:load()

	AutoDrive.loadStoredXML()

	AutoDrive:resetColorAssignment(0, true) -- set default colors

	AutoDrive.readLocalSettingsFromXML()

	ADUserDataManager:load()

	AutoDrive.Hud = AutoDriveHud:new()

	if g_server ~= nil then
		ADUserDataManager:loadFromXml()
	end

	AutoDrive.Hud:loadHud()

	-- Save Configuration when saving savegame
	-- Fix for 1.5 - FSBaseMission.saveSavegame = Utils.appendedFunction(FSBaseMission.saveSavegame, AutoDrive.saveSavegame)
	ItemSystem.save = Utils.prependedFunction(ItemSystem.save, AutoDrive.saveSavegame)

	LoadTrigger.onFillTypeSelection = Utils.appendedFunction(LoadTrigger.onFillTypeSelection, AutoDrive.onFillTypeSelection)

	VehicleCamera.zoomSmoothly = Utils.overwrittenFunction(VehicleCamera.zoomSmoothly, AutoDrive.zoomSmoothly)
	GuiTopDownCamera.onZoom = Utils.overwrittenFunction(GuiTopDownCamera.onZoom, AutoDrive.onZoomTopDownCamera)

	LoadTrigger.load = Utils.overwrittenFunction(LoadTrigger.load, ADTriggerManager.loadTriggerLoad)

	LoadTrigger.delete = Utils.overwrittenFunction(LoadTrigger.delete, ADTriggerManager.loadTriggerDelete)

	Placeable.onBuy = Utils.appendedFunction(Placeable.onBuy, ADTriggerManager.onPlaceableBuy)

	Placeable.onSell = Utils.appendedFunction(Placeable.onSell, ADTriggerManager.onPlaceableSell)

	IngameMapElement.mouseEvent = Utils.overwrittenFunction(IngameMapElement.mouseEvent, AutoDrive.ingameMapElementMouseEvent)

	FSBaseMission.removeVehicle = Utils.prependedFunction(FSBaseMission.removeVehicle, AutoDrive.preRemoveVehicle)

	ConstructionScreen.draw = Utils.appendedFunction(ConstructionScreen.draw, AutoDrive.constructionScreenDraw)

	ADRoutesManager:load()

	ADDrawingManager:load()

	ADMessagesManager:load()

	ADHarvestManager:load()

	ADScheduler:load()

	ADInputManager:load()

	ADMultipleTargetsManager:load()

	ADUnloadManager:load()

	AutoDrivePlaceableData:load()

	InGameMenuMapFrame.refreshContextInput = Utils.appendedFunction(InGameMenuMapFrame.refreshContextInput, AutoDrive.refreshContextInputMapFrame)
	BaseMission.draw = Utils.appendedFunction(BaseMission.draw, AutoDrive.drawBaseMission)
	InGameMenuMapFrame.setMapSelectionItem = Utils.overwrittenFunction(InGameMenuMapFrame.setMapSelectionItem, AutoDrive.InGameMenuMapFrameSetMapSelectionItem)
end

function AutoDrive:refreshContextInputMapFrame()
	if AutoDrive.aiFrameOpen then
		local hotspot = self.currentHotspot
		if hotspot ~= nil then
			local vehicle = InGameMenuMapUtil.getHotspotVehicle(hotspot)
			local allowed = g_currentMission.accessHandler:canPlayerAccess(vehicle, g_currentMission.playerSystem.getLocalPlayer())
			if vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.stateModule ~= nil and allowed then
				AutoDrive.aiFrameVehicle = vehicle
				AutoDrive.Hud.lastUIScale = 0
			end
		end
	end
end

function AutoDrive:drawBaseMission()
	local menuOpen = g_inGameMenu.isOpen
	local correctPage = g_inGameMenu.pageMapOverview ~= nil and g_inGameMenu.pageMapOverview == g_inGameMenu.currentPage
	local isNewJobTab =  g_inGameMenu.mapOverviewSelector ~= nil and g_inGameMenu.mapOverviewSelector.state == g_inGameMenu.pageMapOverview.AI_CREATE_JOB
	local isWorkerListTab =  g_inGameMenu.mapOverviewSelector ~= nil and g_inGameMenu.mapOverviewSelector.state == g_inGameMenu.pageMapOverview.AI_WORKER_LIST

	if menuOpen and correctPage and isNewJobTab then
		if not AutoDrive.aiFrameOpen then
			AutoDrive.aiFrameOpen = true
			AutoDrive.aiFrameVehicle = AutoDrive.getControlledVehicle()
        end
    else
        AutoDrive.aiFrameOpen = false
        AutoDrive.aiFrameVehicle = nil
    end
    if menuOpen and correctPage then
        if not AutoDrive.hasMapCache then
            AutoDrive.hasMapCache = true
            AutoDrive.aiNetworkOnMapCache = nil
		end
		if isWorkerListTab then
			AutoDrive:drawRouteOnMap()
		end
		if isNewJobTab then
			AutoDrive.drawNetworkOnMap()
		end
		if (isNewJobTab or isWorkerListTab) and AutoDrive.aiFrameVehicle ~= nil then
		    if AutoDrive.aiFrameVehicle.ad and AutoDrive.aiFrameVehicle.ad.stateModule then
		        if AutoDrive.Hud ~= nil then
		            if AutoDrive.getSetting("showHUD") then
		                AutoDrive.Hud:drawHud(AutoDrive.aiFrameVehicle)
		            end
		        end
		    end
		end
	else
        AutoDrive.hasMapCache = false
		AutoDrive.aiNetworkOnMapCache = nil
	end
end

function AutoDrive:InGameMenuMapFrameSetMapSelectionItem(superFunc, hotspot)
	if hotspot ~= nil and hotspot.isADMarker and AutoDrive.aiFrameOpen then
		if AutoDrive.showMarkersOnMainMenuMap() and AutoDrive.getSetting("switchToMarkersOnMap") then
			local vehicle = AutoDrive.getADFocusVehicle()
			if vehicle ~= nil then
				AutoDriveHudInputEventEvent:sendFirstMarkerEvent(vehicle, hotspot.markerID)
				return
			end
		end
	end
	return superFunc(self, hotspot)
end

function AutoDrive.drawRouteOnMap()
	local vehicle = AutoDrive.getADFocusVehicle()
	if vehicle == nil then
		return
	end

	if AutoDrive.courseOverlayId == nil then
		AutoDrive.courseOverlayId = createImageOverlay('data/shared/default_normal.dds')
	end

	local dx, dz, dx2D, dy2D, width, rotation, r, g, b

	local WPs, currentWp = vehicle.ad.drivePathModule:getWayPoints()
	if WPs ~= nil then
		local lastWp = nil
		local skips = 0
		for index, wp in pairs(WPs) do
			if skips == 0 then
				if lastWp ~= nil and index >= currentWp then
					local startX, startY, startVisible = AutoDrive.getScreenPosFromWorldPos(lastWp.x, lastWp.z)
					local endX, endY, endVisible = AutoDrive.getScreenPosFromWorldPos(wp.x, wp.z)

					if startX and startY and startVisible and endX and endY and endVisible then
						dx2D = endX - startX;
						dy2D = (endY - startY) / g_screenAspectRatio;
						width = MathUtil.vector2Length(dx2D, dy2D);

						dx = wp.x - lastWp.x;
						dz = wp.z - lastWp.z;
						rotation = MathUtil.getYRotationFromDirection(dx, dz) - math.pi * 0.5;

						local lineThickness = 2 / g_screenHeight
						setOverlayColor(AutoDrive.courseOverlayId, 0.3, 0.5, 0.56, 1)
						setOverlayRotation(AutoDrive.courseOverlayId, rotation, 0, 0)

						renderOverlay(AutoDrive.courseOverlayId, startX, startY, width, lineThickness)
					end
					setOverlayRotation(AutoDrive.courseOverlayId, 0, 0, 0) -- reset overlay rotation
				end
				lastWp = wp
			end
			skips = (skips + 1) % 1
		end
	end
end

function AutoDrive.createNetworkOnMapCache()
	if AutoDrive.aiNetworkOnMapCache ~= nil then
		return
	end
	local cache = {}

	local dx, dz, dx2D, dy2D, width, rotation, r, g, b

	local isSubPrio = function(pointToTest)
		return bit32.band(pointToTest.flags, AutoDrive.FLAG_SUBPRIO) > 0
	end

	local network = ADGraphManager:getWayPoints()
	if network ~= nil then
		for _, node in pairs(network) do
			if node.out ~= nil then
				for _, outNodeId in pairs(node.out) do
					local outNode = network[outNodeId]
					local startX, startY = AutoDrive.getScaledWorldPos(node.x, node.z)
					local endX, endY = AutoDrive.getScaledWorldPos(outNode.x, outNode.z)

					if startX and startY and endX and endY then
						dx = outNode.x - node.x;
						dz = outNode.z - node.z;
						rotation = MathUtil.getYRotationFromDirection(dx, dz) - math.pi * 0.5;

						local r, g, b, a = unpack(AutoDrive.currentColors.ad_color_singleConnection)

						if isSubPrio(outNode) then
							r, g, b, a = unpack(AutoDrive.currentColors.ad_color_subPrioSingleConnection)
						end

						if ADGraphManager:isDualRoad(node, outNode) then
							r, g, b, a = unpack(AutoDrive.currentColors.ad_color_dualConnection)
							if isSubPrio(outNode) then
								r, g, b, a = unpack(AutoDrive.currentColors.ad_color_subPrioDualConnection)
							end
						elseif ADGraphManager:isReverseRoad(node, outNode) then
							r, g, b, a = unpack(AutoDrive.currentColors.ad_color_reverseConnection)
						end

						table.insert(cache, { 
							startX = startX, startY = startY, endX = endX, endY = endY, rotation = rotation,
							r = r, g = g, b = b, a = a
						})
					end
				end
			end
		end
	end
	AutoDrive.aiNetworkOnMapCache = cache
end

function AutoDrive.drawNetworkOnMap()
	if not AutoDrive.isEditorModeEnabled() then
		return
	end

	if AutoDrive.courseOverlayId == nil then
		AutoDrive.courseOverlayId = createImageOverlay('data/shared/default_normal.dds')
	end

	AutoDrive.createNetworkOnMapCache()
	
	local dx2D, dy2D, length
	local lineThickness = 2 / g_screenHeight


	for _, item in pairs(AutoDrive.aiNetworkOnMapCache) do
		local startX, startY, startVisible = AutoDrive.getScreenPosFromScaledWorldPos(item.startX, item.startY)
		local endX, endY, endVisible = AutoDrive.getScreenPosFromScaledWorldPos(item.endX, item.endY)
		if startVisible and endVisible then
			dx2D = endX - startX;
			dy2D = (endY - startY) / g_screenAspectRatio;
			length = MathUtil.vector2Length(dx2D, dy2D);

			setOverlayColor(AutoDrive.courseOverlayId, item.r, item.g, item.b, item.a)
			setOverlayRotation(AutoDrive.courseOverlayId, item.rotation, 0, 0)
			renderOverlay(AutoDrive.courseOverlayId, startX, startY, length, lineThickness)
		end
	end
end

function AutoDrive.getScaledWorldPos(worldX, worldZ)
	local objectX = (worldX + g_inGameMenu.baseIngameMap.worldCenterOffsetX) / g_inGameMenu.baseIngameMap.worldSizeX * 0.5 + 0.25
	local objectZ = (worldZ + g_inGameMenu.baseIngameMap.worldCenterOffsetZ) / g_inGameMenu.baseIngameMap.worldSizeZ * 0.5 + 0.25
	return objectX, objectZ
end

function AutoDrive.getScreenPosFromScaledWorldPos(scaledWorldX, scaledWorldZ)
	local x, y, _, _ = g_inGameMenu.baseIngameMap.layout:getMapObjectPosition(scaledWorldX, scaledWorldZ, 0, 0, 0, true)
	local clipMinX = g_inGameMenu.pageMapOverview.ingameMap.absoluteSizeOffset[1]
	local clipMinY = g_inGameMenu.pageMapOverview.ingameMap.absoluteSizeOffset[2]
	local clipMaxX = g_inGameMenu.pageMapOverview.ingameMap.absSize[1] + clipMinX
	local clipMaxY = g_inGameMenu.pageMapOverview.ingameMap.absSize[2] + clipMinY
	local visible = x >= clipMinX and x <= clipMaxX and y >= clipMinY and y <= clipMaxY
	return x, y, visible
end


function AutoDrive.getScreenPosFromWorldPos(worldX, worldZ)
	local scaledX, scaledZ = AutoDrive.getScaledWorldPos(worldX, worldZ)
	local x, y, visible = AutoDrive.getScreenPosFromScaledWorldPos(scaledX, scaledZ)
	return x, y, visible
end

function AutoDrive:init()

	-- AutoDrive.debugMsg(nil, "[AD] AutoDrive:init start...")

	if g_server == nil then
		-- Here we could ask to server the initial sync
		AutoDriveUserConnectedEvent.sendEvent()
	else
		ADGraphManager:checkYPositionIntegrity()
	end
	AutoDrive.updateDestinationsMapHotspots()
	AutoDrive:registerDestinationListener(AutoDrive, AutoDrive.updateDestinationsMapHotspots)

	if AutoDrive.notificationSample == nil then
		local fileName = Utils.getFilename("sounds/notification_ok.ogg", AutoDrive.directory)
		AutoDrive.notificationSample = createSample("AutoDrive_Notification_ok")
		loadSample(AutoDrive.notificationSample, fileName, false)

		fileName = Utils.getFilename("sounds/notification_warning.ogg", AutoDrive.directory)
		AutoDrive.notificationWarningSample = createSample("AutoDrive_Notification_warning")
		loadSample(AutoDrive.notificationWarningSample, fileName, false)

		fileName = Utils.getFilename("sounds/click_up.ogg", AutoDrive.directory)
		AutoDrive.mouseClickSample = createSample("AutoDrive_mouseClick")
		loadSample(AutoDrive.mouseClickSample, fileName, false)

		fileName = Utils.getFilename("sounds/recordWaypoint.ogg", AutoDrive.directory)
		AutoDrive.recordWaypointSample = createSample("AutoDrive_recordWaypoint")
		loadSample(AutoDrive.recordWaypointSample, fileName, false)

		fileName = Utils.getFilename("sounds/selectedWayPoint.ogg", AutoDrive.directory)
		AutoDrive.selectedWayPointSample = createSample("AutoDrive_selectedWayPoint")
		loadSample(AutoDrive.selectedWayPointSample, fileName, false)
	end
	AutoDrivePlaceableData:setActive(true)
	AutoDrive:setValidSupportedFillTypesForAllVehicles()
	AutoDrive:autostartHelpers()
	AutoDrive.shownErrors = {}
end

function AutoDrive:saveSavegame()
	--    Logging.info("[AD] AutoDrive:saveSavegame start")
	if g_server ~= nil then
		--        Logging.info("[AD] AutoDrive:saveSavegame g_server ~= nil start")
		AutoDrive.saveToXML()
		ADUserDataManager:saveToXml()
		--        Logging.info("[AD] AutoDrive:saveSavegame g_server ~= nil end")
	end
	--    Logging.info("[AD] AutoDrive:saveSavegame end")
end

function AutoDrive:deleteMap()
	-- this function is called even befor the game is compeltely started in case you insert a wrong password for mp game, so we need to check that "mapHotspotsBuffer" and "unRegisterDestinationListener" are not nil
	if AutoDrive.mapHotspotsBuffer ~= nil then
		-- Removing and deleting all map hotspots
		for _, mh in pairs(AutoDrive.mapHotspotsBuffer) do
			-- g_currentMission:removeMapHotspot(mh)
			-- mh:delete()
		end
	end
	AutoDrive.mapHotspotsBuffer = {}
	AutoDrive.mapHotspotsBuffer = nil

	if (AutoDrive.unRegisterDestinationListener ~= nil) then
		AutoDrive:unRegisterDestinationListener(AutoDrive)
	end
	ADRoutesManager:delete()
end

function AutoDrive:keyEvent(unicode, sym, modifier, isDown)
	AutoDrive.leftCTRLmodifierKeyPressed = bit32.band(modifier, Input.MOD_LCTRL) > 0
	AutoDrive.leftALTmodifierKeyPressed = bit32.band(modifier, Input.MOD_LALT) > 0
	AutoDrive.leftLSHIFTmodifierKeyPressed = bit32.band(modifier, Input.MOD_LSHIFT) > 0
	AutoDrive.isCAPSKeyActive = bit32.band(modifier, Input.MOD_CAPS) > 0
	AutoDrive.rightCTRLmodifierKeyPressed = bit32.band(modifier, Input.MOD_RCTRL) > 0
	AutoDrive.rightSHIFTmodifierKeyPressed = bit32.band(modifier, Input.MOD_RSHIFT) > 0

	if AutoDrive.isInExtendedEditorMode() then
		if (AutoDrive.rightCTRLmodifierKeyPressed and AutoDrive.toggleSphere == true) then
			AutoDrive.toggleSphere = false
		elseif (AutoDrive.rightCTRLmodifierKeyPressed and AutoDrive.toggleSphere == false) then
			AutoDrive.toggleSphere = true
		end

		if (AutoDrive.leftCTRLmodifierKeyPressed or AutoDrive.leftALTmodifierKeyPressed) then
			AutoDrive.enableSphere = true
		else
			AutoDrive.enableSphere = AutoDrive.toggleSphere
		end
	end
end

function AutoDrive:mouseEvent(posX, posY, isDown, isUp, button)
	local vehicle = AutoDrive.getADFocusVehicle()
	local mouseActiveForAutoDrive = (AutoDrive.isMouseActiveForHud() or AutoDrive.isMouseActiveForEditor()) and g_inputBinding:getShowMouseCursor()

	if not mouseActiveForAutoDrive then
		AutoDrive.lastButtonDown = nil
		return
	end

	if vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.nToolTipWait ~= nil then
		if vehicle.ad.sToolTip ~= "" then
			if vehicle.ad.nToolTipWait <= 0 then
				vehicle.ad.sToolTip = ""
			else
				vehicle.ad.nToolTipWait = vehicle.ad.nToolTipWait - 1
			end
		end
	end

	if (isDown or AutoDrive.lastButtonDown == button) or button == 0 or button > 3 then
		if vehicle and vehicle.ad and vehicle.ad.stateModule then
			if AutoDrive.getSetting("showHUD") then
				-- pass event to vehicle with active HUD
				AutoDrive.Hud:mouseEvent(vehicle, posX, posY, isDown, isUp, button)
			end
		end

		ADMessagesManager:mouseEvent(posX, posY, isDown, isUp, button)
	end

	if button > 0 and button <= 3 and isDown then
		AutoDrive.lastButtonDown = button
	elseif button > 0 and isUp and AutoDrive.lastButtonDown == button then
		AutoDrive.lastButtonDown = nil
	end
end

function AutoDrive:handleScanDialog()
	if AutoDrive.scanDialogState == AutoDrive.SCAN_DIALOG_RESULT_DONE then
		return true
	end

	if AutoDrive.scanDialogState == AutoDrive.SCAN_DIALOG_NONE then
		if ADGraphManager:getWayPointsCount() > 0 then
			-- AutoDrive.debugMsg(nil, "[AD] AutoDrive:update not-new -> SCAN_DIALOG_RESULT_NO")
			AutoDrive.scanDialogState = AutoDrive.SCAN_DIALOG_RESULT_DONE
			return true
		elseif g_server ~= nil and g_dedicatedServer == nil then
			-- open dialog
			if not g_gui:getIsGuiVisible() then
				--AutoDrive.debugMsg(nil, "[AD] AutoDrive:update SCAN_DIALOG_OPEN")
				AutoDrive.onOpenScanConfirmation()
				AutoDrive.scanDialogState = AutoDrive.SCAN_DIALOG_OPEN
			end
			return false
		else
			-- AutoDrive.debugMsg(nil, "[AD] AutoDrive:update dedi -> SCAN_DIALOG_RESULT_NO")
			AutoDrive.scanDialogState = AutoDrive.SCAN_DIALOG_RESULT_NO
			return true
		end
	end

	if AutoDrive.scanDialogState == AutoDrive.SCAN_DIALOG_OPEN then
		-- dialog still open
		return false
	end

	if AutoDrive.scanDialogState == AutoDrive.SCAN_DIALOG_RESULT_YES then
		-- AutoDrive.debugMsg(nil, "[AD] AutoDrive:update SCAN_DIALOG_RESULT_YES")
		AutoDrive.scanDialogState = AutoDrive.SCAN_DIALOG_RESULT_DONE
		AutoDrive:adParseSplines()
		AutoDrive:createJunctionCommand()
		return true
	end

	if AutoDrive.scanDialogState == AutoDrive.SCAN_DIALOG_RESULT_NO then
		-- AutoDrive.debugMsg(nil, "[AD] AutoDrive:update SCAN_DIALOG_RESULT_NO")
		AutoDrive.scanDialogState = AutoDrive.SCAN_DIALOG_RESULT_DONE
		if ADGraphManager:getWayPointsCount() == 0 then
			AutoDrive.loadStoredXML(true)
		end
		return true
	end
end

function AutoDrive:update(dt)
	if not AutoDrive:handleScanDialog() then
		return
	end

	if AutoDrive.isFirstRun == nil then
		AutoDrive.isFirstRun = false
		self:init()
		if AutoDrive.devAutoDriveInit ~= nil then
			AutoDrive.devAutoDriveInit()
		end
	end

	if AutoDrive.hideMouseCursorOnNextTick == true and not g_inGameMenu.isOpen then
		AutoDrive.hideMouseCursorOnNextTick = nil
		g_inputBinding:setShowMouseCursor(false)
	end

	if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_NETWORKINFO) then
		if AutoDrive.debug.lastSentEvent ~= nil then
			AutoDrive.renderTable(0.3, 0.9, 0.009, AutoDrive.debug.lastSentEvent)
		end
	end
	if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) and AutoDrive.getDebugChannelIsSet(AutoDrive.DC_VEHICLEINFO) then
		AutoDrive.debugDrawBoundingBoxForVehicles()
	end

	if AutoDrive.getSetting("showHUD") then
		AutoDrive.Hud:update(dt)
	end

	if g_server ~= nil then
		ADHarvestManager:update(dt)
		ADScheduler:update(dt)
		ADUnloadManager:update(dt)
	end

	ADMessagesManager:update(dt)
	ADTriggerManager:update(dt)
	ADRoutesManager:update(dt)

    if AutoDrive.devOnUpdate then
        AutoDrive.devOnUpdate(dt)
    end
end

function AutoDrive:draw()
	ADDrawingManager:draw()
	ADMessagesManager:draw()
end

function AutoDrive:constructionScreenDraw()
	local vehicle = AutoDrive.getControlledVehicle()
	if vehicle ~= nil and vehicle.ad ~= nil then
		AutoDrive.onDrawEditorMode(vehicle)
		AutoDrive.onDrawPreviews(vehicle)
		ADDrawingManager:draw()
	end
end

function AutoDrive:preRemoveVehicle(vehicle)
	if vehicle.ad ~= nil and vehicle.ad.stateModule ~= nil then
		if vehicle.ad.stateModule:isActive() then
			vehicle:stopAutoDrive()
		end
		vehicle.ad.stateModule:disableCreationMode()
	end
end


