AutoDriveHud = {}
AutoDrive.FONT_SCALE = 0.0115
AutoDrive.PULLDOWN_ITEM_COUNT = 20

AutoDrive.ItemFilterList = {}

AutoDrive.pullDownListExpanded = 0
AutoDrive.pullDownListDirection = 0
AutoDrive.mouseWheelActive = false
AutoDrive.mouseOverHud = false

AutoDriveHud.debug = false

AutoDriveHud.defaultHeaderHeight = 0.016
AutoDriveHud.headerHeightWithTips = 0.185
AutoDriveHud.headerHeightWithEditor = 0.115
AutoDriveHud.elementSize = 32
AutoDriveHud.elementGap = 3
AutoDriveHud.listItemSize = 20

AutoDriveHud.ELEMENTS = {
	-- table of all possible hud elements, x/y are used for the editor
	["pulldownTarget"] = { w=7, h=1, x=0, y=1 },
	["pulldownUnload"] = { w=7, h=1, x=0, y=2 },
	["pulldownFilltype"] = { w=7, h=1, x=0, y=3 },
	["record"] = { w=1, h=1, x=0, y=4, button={[1]="input_record", [2]="input_record_dual", [3]="input_record_subPrio", [4]="input_record_subPrioDual", [5]="input_record_twoWay", [6]="input_record_dualTwoWay", [7]="input_record_subPrioTwoWay", [8]="input_record_subPrioDualTwoWay", tip="input_ADRecord"}},
	["routesManager"] = { w=1, h=1, x=1, y=4, button={[1]="input_routesManager", tip="input_AD_routes_manager"}},
	["createMarker"] = { w=1, h=1, x=2, y=4, button={[1]="input_createMapMarker", tip="input_ADDebugCreateMapMarker"}},
	["removeWaypoint"] = { w=1, h=1, x=3, y=4, button={[1]="input_removeWaypoint", [2]="input_removeMapMarker", tip="input_ADDebugDeleteWayPoint"}},
	["editMarker"] = { w=1, h=1, x=4, y=4, button={[1]="input_editMapMarker", tip="input_ADRenameMapMarker"}},
	["removeMarker"] = { w=1, h=1, x=5, y=4, button={[1]="input_removeMapMarker", tip="input_ADDebugDeleteDestination"}},
	["startHelper"] = { w=1, h=1, x=6, y=4, button={[1]="input_startHelper", [2]="input_toggleUsedHelper", tip="hud_startHelper"}},
	["fieldSpeed"] = { w=1, h=1, x=10, y=3, speed={field=true} },
	["settings"] = { w=1, h=1, x=7, y=3, button={[1]="input_openGUI", tip="input_ADOpenGUI"}},
	["startStop"] = { w=1, h=1, x=7, y=2, button={[1]="input_start_stop", tip="input_ADEnDisable"}},
	["mode"] = { w=1, h=1, x=8, y=2, button={[1]="input_silomode", [2]="input_previousMode", tip="input_ADSilomode"}},
	["continue"] = { w=1, h=1, x=9, y=2, button={[1]="input_continue", tip="input_AD_continue"}},
	["park"] = { w=1, h=1, x=10, y=2, button={[1]="input_parkVehicle", [2]="input_setParkDestination", [6]="input_setParkDestination", tip="input_ADParkVehicle"}},
	["loopCounter"] = { w=1, h=1, x=11, y=2 },
	["speed"] = { w=1, h=1, x=9, y=3, speed={field=false} },
	["editor"] = { w=1, h=1, x=8, y=3, button={[1]="input_debug", [2]="input_displayMapPoints",  tip="input_ADActivateDebug"}},
	["trafficDetection"] = { w=1, h=1, x=7, y=1, settings={[1]="enableTrafficDetection", tip="gui_ad_enableTrafficDetection"}},
	["rotateTargets"] = { w=1, h=1, x=8, y=1, settings={[1]="rotateTargets", tip="gui_ad_rotateTargets"}},
	["exitField"] = { w=1, h=1, x=9, y=1, settings={[1]="exitField", tip="gui_ad_exitField"}},
	["restrictToField"] = { w=1, h=1, x=10, y=1, settings={[1]="restrictToField", tip="gui_ad_restrictToField"}},
	["avoidFruit"] = { w=1, h=1, x=11, y=1, settings={[1]="avoidFruit", tip="gui_ad_avoidFruit"}},
	["decHudWidth"] = { w=1, h=1, editor={[1]="input_decHudWidth", tip="gui_ad_decrementHudWidth"}},
	["incHudWidth"] = { w=1, h=1, editor={[1]="input_incHudWidth", tip="gui_ad_incrementHudWidth"}},
	["decHudHeight"] = { w=1, h=1, editor={[1]="input_decHudHeight", tip="gui_ad_decrementHudHeight"}},
	["incHudHeight"] = { w=1, h=1, editor={[1]="input_incHudHeight", tip="gui_ad_incrementHudHeight"}},
	["rotateHudPresets"] = { w=1, h=1, editor={[1]="input_rotateHudPresets", tip="gui_ad_rotateHudPresets"}},
	["saveHud"] = { w=1, h=1, editor={[1]="input_saveHud", tip="gui_ad_saveHud"}},
	["parkAtJobFinished"] = { w=1, h=1, x=7, y=4, settings={[1]="enableParkAtJobFinished", tip="gui_ad_enableParkAtJobFinished"}},
	["foldImplements"] = { w=1, h=1, x=8, y=4, settings={[1]="FoldImplements", tip="gui_ad_FoldImplements"}},
	["alUnload"] = { w=1, h=1, x=9, y=4, settings={[1]="ALUnload", tip="gui_ad_ALUnload"}},

	--[[
	NEW:
	  Unload fill level: AutoDrive.settings.unloadFillLevel
	  Restrict unloader to field: AutoDrive.settings.followOnlyOnField
	--]]
}

function AutoDriveHud:new()
	local o = {}
	setmetatable(o, self)
	self.__index = self
	-- not allowed fillTypes in HUD
	AutoDrive.ItemFilterList = {
		g_fillTypeManager:getFillTypeIndexByName("AIR"),
		g_fillTypeManager:getFillTypeIndexByName("CHICKEN_TYPE_BLACK"),
		g_fillTypeManager:getFillTypeIndexByName("CHICKEN_TYPE_WHITE"),
		g_fillTypeManager:getFillTypeIndexByName("CHICKEN_TYPE_BROWN"),
		g_fillTypeManager:getFillTypeIndexByName("CHICKEN_TYPE_ROOSTER"),
		g_fillTypeManager:getFillTypeIndexByName("COW_TYPE_BROWN"),
		g_fillTypeManager:getFillTypeIndexByName("COW_TYPE_BROWN_WHITE"),
		g_fillTypeManager:getFillTypeIndexByName("COW_TYPE_BLACK"),
		g_fillTypeManager:getFillTypeIndexByName("COW_TYPE_BLACK_WHITE"),
		g_fillTypeManager:getFillTypeIndexByName("COW_TYPE_BRAHMAN_BROWN"),
		g_fillTypeManager:getFillTypeIndexByName("COW_TYPE_BRAHMAN_WHITE"),
		g_fillTypeManager:getFillTypeIndexByName("COW_TYPE_BRAHMAN_LIGHT_BROWN"),
		g_fillTypeManager:getFillTypeIndexByName("COW_TYPE_BRAHMAN_GREY"),
		g_fillTypeManager:getFillTypeIndexByName("EGG"),
		g_fillTypeManager:getFillTypeIndexByName("HORSE_TYPE_BEIGE"),
		g_fillTypeManager:getFillTypeIndexByName("HORSE_TYPE_BLACK"),
		g_fillTypeManager:getFillTypeIndexByName("HORSE_TYPE_BROWN"),
		g_fillTypeManager:getFillTypeIndexByName("HORSE_TYPE_BROWN_WHITE"),
		g_fillTypeManager:getFillTypeIndexByName("HORSE_TYPE_DARK_BROWN"),
		g_fillTypeManager:getFillTypeIndexByName("HORSE_TYPE_GREY"),
		g_fillTypeManager:getFillTypeIndexByName("HORSE_TYPE_LIGHT_BROWN"),
		g_fillTypeManager:getFillTypeIndexByName("HORSE_TYPE_RED_BROWN"),
		g_fillTypeManager:getFillTypeIndexByName("OILSEEDRADISH"),
		g_fillTypeManager:getFillTypeIndexByName("PIG_TYPE_RED"),
		g_fillTypeManager:getFillTypeIndexByName("PIG_TYPE_WHITE"),
		g_fillTypeManager:getFillTypeIndexByName("PIG_TYPE_BLACK_WHITE"),
		g_fillTypeManager:getFillTypeIndexByName("PIG_TYPE_BLACK"),
		g_fillTypeManager:getFillTypeIndexByName("ROUNDBALE"),
		g_fillTypeManager:getFillTypeIndexByName("ROUNDBALE_GRASS"),
		g_fillTypeManager:getFillTypeIndexByName("ROUNDBALE_DRYGRASS"),
		g_fillTypeManager:getFillTypeIndexByName("ROUNDBALE_WHEAT"),
		g_fillTypeManager:getFillTypeIndexByName("ROUNDBALE_BARLEY"),
		g_fillTypeManager:getFillTypeIndexByName("SHEEP_TYPE_WHITE"),
		g_fillTypeManager:getFillTypeIndexByName("SHEEP_TYPE_BROWN"),
		g_fillTypeManager:getFillTypeIndexByName("SHEEP_TYPE_BLACK_WHITE"),
		g_fillTypeManager:getFillTypeIndexByName("SHEEP_TYPE_BLACK"),
		g_fillTypeManager:getFillTypeIndexByName("SQUAREBALE"),
		g_fillTypeManager:getFillTypeIndexByName("SQUAREBALE_WHEAT"),
		g_fillTypeManager:getFillTypeIndexByName("SQUAREBALE_BARLEY"),
		g_fillTypeManager:getFillTypeIndexByName("TARP"),
		g_fillTypeManager:getFillTypeIndexByName("TREESAPLINGS"),
		g_fillTypeManager:getFillTypeIndexByName("UNKNOWN"),
		g_fillTypeManager:getFillTypeIndexByName("WEED"),
		g_fillTypeManager:getFillTypeIndexByName("WOOL")
	}
	o.isMoving = false
	o.isShowingTips = false
	o.isEditingHud = false
	o.movingElement = nil
	o.hudEditorElements = {}
	return o
end

function AutoDriveHud:createHudAt(hudX, hudY)
	if self.elements == nil then
		self:loadHud()
	end
	local uiScale = g_gameSettings:getValue("uiScale")
	if AutoDrive.getSetting("guiScale") ~= 0 then
		uiScale = AutoDrive.getSetting("guiScale")
	end

	if self.isShowingTips then
		self.headerHeight = AutoDriveHud.headerHeightWithTips * uiScale
	elseif self.isEditingHud then
		self.headerHeight = AutoDriveHud.headerHeightWithEditor * uiScale
	else
		self.headerHeight = AutoDriveHud.defaultHeaderHeight * uiScale
	end

	self.elementWidth, self.elementHeight = getNormalizedScreenValues(uiScale * self.elementSize, uiScale * self.elementSize)
	self.gapWidth, self.gapHeight = getNormalizedScreenValues(uiScale * self.elementGap, uiScale * self.elementGap)
	self.width, self.height = getNormalizedScreenValues(
		(self.numElementsH * (self.elementSize + self.elementGap) + self.elementGap) * uiScale,
		(self.numElementsV * (self.elementSize + self.elementGap) + self.elementGap) * uiScale + self.headerHeight)
	_, self.listItemHeight = getNormalizedScreenValues(self.listItemSize * uiScale, self.listItemSize * uiScale)

	self.headerIconHeight = AutoDriveHud.defaultHeaderHeight * uiScale
	self.headerIconWidth = self.headerIconHeight * (g_screenHeight / g_screenWidth)
	self.headerLabelWidth = self.width - 4 * self.gapWidth - 3 * self.headerIconWidth

	self.posX = math.clamp(hudX, 0, 1 - self.width)
	self.posY = math.clamp(hudY, 0, 1 - self.height)
	AutoDrive.HudX = self.posX
	AutoDrive.HudY = self.posY

	self.hudEditorElementsH = 12
	self.hudEditorElementsV = 5
	self.hudEditorWidth = self.hudEditorElementsH * (self.elementWidth + self.gapWidth) + self.gapWidth
	self.hudEditorHeight = self.hudEditorElementsV * (self.elementHeight + self.gapHeight) + self.gapHeight
	self.hudEditorPosX = self.posX - self.hudEditorWidth - self.gapWidth
	self.hudEditorPosY = self.posY + self.height - self.hudEditorHeight

	self.hudElements = {}
	self.hudEditorElements = {}

	-- background
	local headerY = self.posY + (self.elementHeight + self.gapHeight) * self.numElementsV + self.gapHeight
	table.insert(self.hudElements, ADHudIcon:new(self.posX, self.posY, self.width, self.height, "ad_gui.Background", 0, "background"))
	table.insert(self.hudElements, ADHudIcon:new(self.posX, headerY, self.width, self.headerHeight, "ad_gui.Header", 1, "header"))

	-- HUD editor
	if self.isEditingHud then
		self:createHudEditor()
	end

	-- header icons
	table.insert(self.hudElements, ADHudButton:new(self.posX + self.width - self.headerIconWidth, headerY, self.headerIconWidth, self.headerIconHeight, "input_toggleHud"))
	table.insert(self.hudElements, ADHudButton:new(self.posX + self.width - 2 * self.headerIconWidth, headerY, self.headerIconWidth, self.headerIconHeight, "input_toggleHudExtension"))
	table.insert(self.hudElements, ADHudEditorButton:new(self.posX + self.width - 3 * self.headerIconWidth, headerY, self.headerIconWidth, self.headerIconHeight, "toggleEditHud", "gui_ad_toggleHudEditor"))

	-- hud elements
	for _, element in ipairs(self.elements) do
		self:addElement(element, self.hudElements)
	end

	-- Refreshing layer sequence must be called, after all elements have been added
	self:refreshHudElementsLayerSequence()
end

function AutoDriveHud:createHudEditor()
	table.insert(self.hudElements, ADHudIcon:new(self.hudEditorPosX, self.hudEditorPosY, self.hudEditorWidth, self.hudEditorHeight, "ad_gui.Background", 0, "editorBackground"))
	self:addElement({name="decHudWidth", x=0, y=0, hudEditor=true }, self.hudElements)
	self:addElement({name="incHudWidth", x=1, y=0, hudEditor=true }, self.hudElements)
	self:addElement({name="decHudHeight", x=2, y=0, hudEditor=true }, self.hudElements)
	self:addElement({name="incHudHeight", x=3, y=0, hudEditor=true }, self.hudElements)
	self:addElement({name="rotateHudPresets", x=4, y=0, hudEditor=true }, self.hudElements)
	self:addElement({name="saveHud", x=5, y=0, hudEditor=true }, self.hudElements)
	self:addElement({name="editor", x=11, y=0, hudEditor=true }, self.hudElements)
	for name, config in pairs(AutoDriveHud.ELEMENTS) do
		if config.editor == nil and config.x ~= nil and config.y ~= nil then
			local elements = {}
			self:addElement({ name = name, x = config.x, y = config.y, hudEditor = true }, elements)
			self.hudEditorElements[name] = elements
		end
	end
end

function AutoDriveHud:addElement(element, elementList)
	local config = AutoDriveHud.ELEMENTS[element.name]
	if config == nil then
		Logging.error("Unknown HUD element: %s", element.name)
		return
	end

	local vehicle = AutoDrive.getADFocusVehicle()
	local edit = element.edit
	local h = config.h * (self.elementHeight + self.gapHeight) - self.gapHeight

	local posX, posY, numV = self.posX, self.posY, self.numElementsV
	if element.hudEditor then
		posX, posY, numV = self.hudEditorPosX, self.hudEditorPosY, self.hudEditorElementsV
	elseif element.absolutePos ~= nil then
		posX, posY, numV = element.absolutePos.x, element.absolutePos.y, 1
	elseif element.x + config.w > self.numElementsH then
		return
	elseif element.y + config.h > self.numElementsV then
		return
	end

	local function X(offset)
		offset = offset or 0
		return posX + (element.x + offset) * (self.elementWidth + self.gapWidth) + self.gapWidth
	end
	local function Y(offset)
		offset = offset or 0
		return posY + (numV - element.y - offset - 1) * (self.elementHeight + self.gapHeight) + self.gapHeight
	end
	local function W(w)
		w = w or config.w
		return w * (self.elementWidth + self.gapWidth) - self.gapWidth
	end

	if config.button ~= nil then
		local btn = config.button
		table.insert(elementList, ADHudButton:new(X(), Y(), W(), h, btn[1], btn[2], btn[3], btn[4], btn[5], btn[6], btn[7], btn[8], btn.tip, 1, edit))
	elseif config.speed ~= nil then
		table.insert(elementList, ADHudSpeedmeter:new(X(), Y(), W(), h, config.speed.field, edit))
	elseif config.settings ~= nil then
		table.insert(elementList, ADHudSettingsButton:new(X(), Y(), W(), h, config.settings[1], config.settings.tip, 1, edit))
	elseif config.editor ~= nil then
		table.insert(elementList, ADHudEditorButton:new(X(), Y(), W(), h, config.editor[1], config.editor.tip))

	elseif element.name == "pulldownTarget" then
		table.insert(elementList, ADHudButton:new(X(), Y(), W(1), h, "input_toggleAutomaticPickupTarget", nil, nil, nil, nil, nil, nil, nil, "input_ADToggleAutomaticPickupTarget", 1, edit))
		self.targetPullDownList = ADPullDownList:new(X(1), Y(), W(6), self.listItemHeight, ADPullDownList.TYPE_TARGET, 1, edit)
		table.insert(elementList, self.targetPullDownList)
	elseif element.name == "pulldownUnload" then
		table.insert(elementList, ADHudButton:new(X(), Y(), W(1), h, "input_toggleAutomaticUnloadTarget", nil, nil, nil, nil, nil, nil, nil, "input_ADToggleAutomaticUnloadTarget", 1, edit))
		table.insert(elementList, ADPullDownList:new(X(1), Y(), W(6), self.listItemHeight, ADPullDownList.TYPE_UNLOAD, 1, edit))
	elseif element.name == "pulldownFilltype" then
		table.insert(elementList, ADHudButton:new(X(), Y(), W(1), h, "input_toggleLoadByFillLevel", nil, nil, nil, nil, nil, nil, nil, "input_ADToggleLoadByFillLevel", 1, edit))
		table.insert(elementList, ADPullDownList:new(X(1), Y(), W(6), self.listItemHeight, ADPullDownList.TYPE_FILLTYPE, 1, edit))
		table.insert(elementList, HudHarvesterInfo:new(X(1), Y(), W(6), self.listItemHeight))
	elseif element.name == "loopCounter" then
		if vehicle == nil or vehicle.ad.stateModule:getMode() ~= AutoDrive.MODE_BGA then
	 		table.insert(elementList, ADHudCounterButton:new(X(), Y(), W(), h, "loop_counter", edit))
		else
			table.insert(elementList, ADHudButton:new(X(), Y(), W(), h, "input_bunkerUnloadType", nil, nil, nil, nil, nil, nil, nil, "input_ADbunkerUnloadType", 1, edit))
		end
	else
		Logging.error("Unknown HUD element: %s", element.name)
	end
end

function AutoDriveHud:refreshHudElementsLayerSequence()
	-- Sort the elements by their layer index, for optimizing drawHud and mouseEvent methods
	if self.hudElements ~= nil then
		table.sort(
			self.hudElements,
			function(a, b)
				return a.layer < b.layer
			end
		)
	end
end

function AutoDriveHud:drawHud(vehicle)
	local controlledVehicle = AutoDrive.getControlledVehicle()
	if (vehicle ~= nil and vehicle == controlledVehicle) or AutoDrive.aiFrameOpen then
		local uiScale = g_gameSettings:getValue("uiScale")
		if AutoDrive.getSetting("guiScale") ~= 0 then
			uiScale = AutoDrive.getSetting("guiScale")
		end

		if self.lastUIScale == nil then
			self.lastUIScale = uiScale
		end

		if self.lastUIScale ~= uiScale then
			self:createHudAt(self.posX, self.posY)
		end
		self.lastUIScale = uiScale

        if self.hudElements ~= nil then
			new2DLayer()
            for _, element in ipairs(self.hudElements) do
                element:onDraw(vehicle, uiScale)
            end
			for _, elements in pairs(self.hudEditorElements) do
				for _, element in ipairs(elements) do
					element:onDraw(vehicle, uiScale)
				end
			end
        end
	end
end

function AutoDriveHud:update(dt)
    if self.hudElements ~= nil then
        for _, element in ipairs(self.hudElements) do
            element:update(dt)
        end
    end
	for _, elements in pairs(self.hudEditorElements) do
		for _, element in ipairs(elements) do
			element:update(dt)
		end
	end
end

function AutoDriveHud:toggleHudExtension(vehicle)
	self.isShowingTips = not self.isShowingTips
	self:createHudAt(self.posX, self.posY)
end

function AutoDriveHud:toggleHud(vehicle)
    if not AutoDrive.getSetting("showHUD") then
        AutoDrive.setSettingState("showHUD", 2)
    else
        AutoDrive.setSettingState("showHUD", 1)
    end
end

function AutoDriveHud:isMouseOverHud( x, y)
	--- Checks if a hud element was hit.
    local focusVehicle = AutoDrive.getADFocusVehicle()
	if focusVehicle ~= nil then
        if focusVehicle.ad and focusVehicle.ad.stateModule then
            if AutoDrive.getSetting("showHUD") then
                if AutoDrive.Hud and AutoDrive.Hud.hudElements then
                    for i= 1,#AutoDrive.Hud.hudElements do 
                        if AutoDrive.Hud.hudElements[i]:hit(x, y, 0) then 
                            return true
                        end
                    end
                end
            end
        end
    end
end

function AutoDriveHud:mouseEventOnHudElements(vehicle, posX, posY, isDown, isUp, button)
	-- returns "handled"
	if self.isEditingHud then
		if self.movingElement ~= nil then
			if (button == 1 and isUp) or not AutoDrive.isMouseActiveForHud() then
				self:stopMovingHudElement(button == 1 and isUp)
			else
				self:moveHudElement(posX, posY)
			end
			return true -- handled
		end

		if self:mouseEventOnHudEditorElements(vehicle, posX, posY, isDown, isUp, button) then
			return true -- handled
		end
	end
	if self.hudElements ~= nil then
		-- Start with highest layer value (last in array), and then iterate backwards.
		for i = #self.hudElements, 1, -1 do
			local element = self.hudElements[i]
			local layer = element.layer
			local mouseEventHandled, silent = element:mouseEvent(vehicle, posX, posY, isDown, isUp, button, layer)
			if mouseEventHandled then
				-- Maybe a PullDownList have been expanded/collapsed, so need to refresh layer sequence
				self:refreshHudElementsLayerSequence()
				if silent == nil or silent == false then
					AutoDrive.playSample(AutoDrive.mouseClickSample, 0.45)
				end
				return true -- handled
			end
		end
	end
	if AutoDrive.pullDownListExpanded > 0 and button >= 1 and button <= 3 and isUp then
		AutoDrive.Hud:closeAllPullDownLists(vehicle)
	end
	if self.isMoving then
		if (button == 1 and isUp) or not AutoDrive.isMouseActiveForHud() then
			self:stopMovingHud()
		else
			self:moveHud(posX, posY)
		end
		return true -- handled
	end
	return false -- not handled
end

function AutoDriveHud:mouseEventHandleSelection(vehicle, isUp, button)
	if
		not AutoDrive.leftLSHIFTmodifierKeyPressed
		and not AutoDrive.leftCTRLmodifierKeyPressed
		and AutoDrive.leftALTmodifierKeyPressed 
		and not AutoDrive.rightSHIFTmodifierKeyPressed
		and vehicle.ad.newcreated == nil
		and vehicle.ad.selectedNodeId ~= nil
		then
		-- selected node and LALT pressed       
		AutoDriveHud.debugMsg(vehicle, "AutoDriveHud:mouseEvent selection start")
        vehicle.ad.selectionRange = vehicle.ad.selectionRange or 1	-- start with 1m range
		AutoDrive.mouseWheelActive = true
		if button == 4 and isUp and vehicle.ad.selectionRange > 1 then
			AutoDriveHud.debugMsg(vehicle, "AutoDriveHud:mouseEvent decrement")
			-- decrement range
			vehicle.ad.selectionActive = true
			vehicle.ad.selectionRange = vehicle.ad.selectionRange - 0.5
			vehicle.ad.selectionWayPoints = self:getSelectionWayPoints(vehicle)
		elseif button == 5 and isUp and vehicle.ad.selectionRange < AutoDrive.drawDistance / 2 then
			AutoDriveHud.debugMsg(vehicle, "AutoDriveHud:mouseEvent increment")
			-- increment range
			vehicle.ad.selectionActive = true
			vehicle.ad.selectionRange = vehicle.ad.selectionRange + 0.5
			vehicle.ad.selectionWayPoints = self:getSelectionWayPoints(vehicle)
		elseif vehicle.ad.selectionActive then
			-- activated again
			vehicle.ad.selectionWayPoints = self:getSelectionWayPoints(vehicle)
		end
		if button == 1 and isUp and vehicle.ad.selectionActive then
			-- delete selected wayPoints
			AutoDriveHud.debugMsg(vehicle, "AutoDriveHud:mouseEvent delete selection")
			ADGraphManager:deleteWayPointsInSelection(vehicle)
			vehicle:resetWayPointsDistance()
			vehicle.ad.selectedNodeId = nil
		end
	else
		-- clear selection wayPoints
		if vehicle.ad.selectionWayPoints and #vehicle.ad.selectionWayPoints > 0 then
			for _, wayPointId in pairs(vehicle.ad.selectionWayPoints) do
				local wayPoint = ADGraphManager:getWayPointById(wayPointId)
				wayPoint.isSelected = false
			end
			vehicle.ad.selectionWayPoints = {}
		end
	end
end

function AutoDriveHud:mouseEventResetSelectedNode(vehicle)
    if
        not AutoDrive.leftLSHIFTmodifierKeyPressed
        and not AutoDrive.leftCTRLmodifierKeyPressed
        and not AutoDrive.leftALTmodifierKeyPressed
        and not AutoDrive.rightSHIFTmodifierKeyPressed
        and vehicle.ad.newcreated ~= nil 
        and vehicle.ad.selectedNodeId == vehicle.ad.newcreated
        then
        -- if LCTRL is not pressed - no auto-connect to previous created new point, disable selected point
        AutoDriveHud.debugMsg(vehicle, "AutoDriveHud:mouseEvent selectedNodeId = nil")
        vehicle.ad.selectedNodeId = nil
        vehicle.ad.newcreated = nil
    end
end

function AutoDriveHud:mouseEventFindHoveredNode(vehicle)
    -- try to get a waypoint in mouse range
    for _, point in pairs(vehicle:getWayPointsInRange(0, AutoDrive.drawDistance)) do
        if AutoDrive.mouseIsAtPos(point, 0.01) then
            vehicle.ad.hoveredNodeId = point.id
            return
        end
    end
end

function AutoDriveHud:mouseEventCreateSplineInterpolation(vehicle)
    if vehicle.ad.selectedNodeId ~= nil and vehicle.ad.selectedNodeId ~= vehicle.ad.hoveredNodeId then
        AutoDrive:createSplineInterpolationBetween(ADGraphManager:getWayPointById(vehicle.ad.selectedNodeId), ADGraphManager:getWayPointById(vehicle.ad.hoveredNodeId))
    end
end

function AutoDriveHud:mouseEventConnectWaypoints(vehicle, connectDual)
    if vehicle.ad.selectedNodeId ~= vehicle.ad.hoveredNodeId then
        local reverseDirection = AutoDrive.rightSHIFTmodifierKeyPressed

        if not table.contains(ADGraphManager:getWayPointById(vehicle.ad.selectedNodeId).out, vehicle.ad.hoveredNodeId) then
            -- connect selected point with hovered point

            if AutoDrive.splineInterpolation ~= nil and AutoDrive.splineInterpolation.valid and AutoDrive.splineInterpolation.waypoints ~= nil and #AutoDrive.splineInterpolation.waypoints > 2 then
                local waypoints = {}
                local lastHeight = ADGraphManager:getWayPointById(vehicle.ad.selectedNodeId).y
                for wpId, wp in pairs(AutoDrive.splineInterpolation.waypoints) do
                    if wpId ~= 1 and wpId < (#AutoDrive.splineInterpolation.waypoints - 1) then
                        if math.abs(wp.y - lastHeight) > 1 then -- prevent point dropping into the ground in case of bridges etc
                            wp.y = lastHeight
                        end
                        table.insert(waypoints, {x=wp.x, y=wp.y, z=wp.z})
                        lastHeight = wp.y
                    end
                end

                ADGraphManager:createSplineConnection(vehicle.ad.selectedNodeId, waypoints, vehicle.ad.hoveredNodeId, connectDual)
            else
                AutoDriveHud.debugMsg(vehicle, "AutoDriveHud:mouseEvent toggleConnectionBetween 1 vehicle.ad.selectedNodeId %d vehicle.ad.hoveredNodeId %d", vehicle.ad.selectedNodeId, vehicle.ad.hoveredNodeId)
                ADGraphManager:toggleConnectionBetween(ADGraphManager:getWayPointById(vehicle.ad.selectedNodeId), ADGraphManager:getWayPointById(vehicle.ad.hoveredNodeId), reverseDirection, connectDual)
            end

            AutoDrive.splineInterpolationUserCurvature = nil
        else
            AutoDriveHud.debugMsg(vehicle, "AutoDriveHud:mouseEvent toggleConnectionBetween 1 vehicle.ad.selectedNodeId %d vehicle.ad.hoveredNodeId %d", vehicle.ad.selectedNodeId, vehicle.ad.hoveredNodeId)
            ADGraphManager:toggleConnectionBetween(ADGraphManager:getWayPointById(vehicle.ad.selectedNodeId), ADGraphManager:getWayPointById(vehicle.ad.hoveredNodeId), reverseDirection, connectDual)
        end
    end    
    AutoDrive.playSample(AutoDrive.selectedWayPointSample, 0.75)
    
    -- unselect point
    AutoDriveHud.debugMsg(vehicle, "AutoDriveHud:mouseEvent unselect point selectedNodeId = nil")
    vehicle.ad.selectedNodeId = nil
    return true
end

function AutoDriveHud:mouseEventSelectWaypoint(vehicle)
    -- select point
    -- no selectedNodeId: hoveredNodeId is now selectedNodeId
    vehicle.ad.selectedNodeId = vehicle.ad.hoveredNodeId
    AutoDrive.splineInterpolationUserCurvature = nil
    AutoDriveHud.debugMsg(vehicle, "AutoDriveHud:mouseEvent select point selectedNodeId %d", vehicle.ad.selectedNodeId)
    
    AutoDrive.playSample(AutoDrive.selectedWayPointSample, 0.75)

    -- color assignment goes in here
    if AutoDrive.getSetting("colorAssignmentMode") and g_server ~= nil and g_client ~= nil and g_dedicatedServer == nil then
        local colorPoint = ADGraphManager:getWayPointById(vehicle.ad.selectedNodeId)
        if colorPoint ~= nil and colorPoint.colors ~= nil then
            AutoDriveHud.debugMsg(vehicle, "AutoDriveHud:mouseEvent point.colors %.3f %.3f %.3f", colorPoint.colors[1], colorPoint.colors[2], colorPoint.colors[3])
            vehicle.ad.selectedColorNodeId = vehicle.ad.selectedNodeId
            vehicle.ad.selectedNodeId = nil
            -- only allowed in single player game
            ADInputManager:input_openColorSettings()
        end
    end
end

function AutoDriveHud:mouseEventSetupAutoConnection(vehicle, isUp, button)
    -- if LCTRL is pressed, you can select a waypoint so that when you will create a new one they will be connected (auto connection to existing waypoint)
    if 
        button == 1
        and isUp
        and not AutoDrive.leftLSHIFTmodifierKeyPressed
        and AutoDrive.leftCTRLmodifierKeyPressed
        and not AutoDrive.leftALTmodifierKeyPressed
        and not AutoDrive.rightSHIFTmodifierKeyPressed
        and vehicle.ad.selectedNodeId == nil
        and vehicle.ad.hoveredNodeId ~= nil
        then
        vehicle.ad.newcreated = vehicle.ad.hoveredNodeId
        vehicle.ad.selectedNodeId = vehicle.ad.newcreated
        AutoDriveHud.debugMsg(vehicle, "AutoDriveHud:mouseEvent auto connection 1 selectedNodeId %d", vehicle.ad.selectedNodeId)
    end
end

function AutoDriveHud:mouseEventSelectOrConnectWaypoint(vehicle, isUp, button)
    -- returns adjustedPaths

    local connectOneWay = not AutoDrive.leftLSHIFTmodifierKeyPressed and not AutoDrive.leftCTRLmodifierKeyPressed and not AutoDrive.leftALTmodifierKeyPressed
    local connectDual = not AutoDrive.leftLSHIFTmodifierKeyPressed and AutoDrive.leftCTRLmodifierKeyPressed and AutoDrive.leftALTmodifierKeyPressed
    if button == 1 and isUp and (connectOneWay or connectDual)
        then
        -- left mouse button to select point / connect to already selected point
        if vehicle.ad.selectedNodeId ~= nil then
            -- point selected - connect
            return self:mouseEventConnectWaypoints(vehicle, connectDual)
        else
            -- select point
            self:mouseEventSelectWaypoint(vehicle)
        end
    end
    return false
end

function AutoDriveHud:mouseEventStartMovingNode(vehicle, isDown, button)
    if
        (button == 2 or button == 3)
        and isDown
        and not AutoDrive.leftLSHIFTmodifierKeyPressed
        and not AutoDrive.leftCTRLmodifierKeyPressed
        and not AutoDrive.leftALTmodifierKeyPressed
        and not AutoDrive.rightSHIFTmodifierKeyPressed
        then
        -- middle or right mouse button to move points - waypoint at mouse position selected to move
        if vehicle.ad.nodeToMoveId == nil then
            vehicle.ad.nodeToMoveId = vehicle.ad.hoveredNodeId
        end
    end
end

function AutoDriveHud:mouseEventMoveNode(vehicle)
    if vehicle.ad.nodeToMoveId ~= nil then
        -- move point at mouse position
        AutoDriveHud.debugMsg(vehicle, "AutoDriveHud:mouseEvent moveNodeToMousePos nodeToMoveId %d", vehicle.ad.nodeToMoveId)
        AutoDrive.moveNodeToMousePos(vehicle.ad.nodeToMoveId)
    end
end

function AutoDriveHud:mouseEventFinishMovingNode(vehicle, isUp, button)
    if
        (button == 2 or button == 3)
        and isUp
        -- leftLSHIFT needed to be checked in changeWayPointPosition
        and not AutoDrive.leftCTRLmodifierKeyPressed
        and not AutoDrive.leftALTmodifierKeyPressed 
        and not AutoDrive.rightSHIFTmodifierKeyPressed
        then
        if vehicle.ad.nodeToMoveId ~= nil then
            -- middle or right mouse button to move points - end of move -> change waypoint coordinates now
            ADGraphManager:changeWayPointPosition(vehicle.ad.nodeToMoveId)
            vehicle.ad.nodeToMoveId = nil
        end
    end
end

function AutoDriveHud:mouseEventToggleWaypointPriority(vehicle, isUp, button, adjustedPaths)
    -- if LSHIFT is pressed, selecting a waypoint will toggle its priority
    if
        button == 1
        and isUp
        and AutoDrive.leftLSHIFTmodifierKeyPressed
        and not AutoDrive.leftCTRLmodifierKeyPressed
        and not AutoDrive.leftALTmodifierKeyPressed
        and not AutoDrive.rightSHIFTmodifierKeyPressed
        and vehicle.ad.hoveredNodeId ~= nil
        and vehicle.ad.selectedNodeId == nil
        and not adjustedPaths
        then
        ADGraphManager:toggleWayPointAsSubPrio(vehicle.ad.hoveredNodeId)
        AutoDriveHud.debugMsg(vehicle, "AutoDriveHud:mouseEvent toggleWayPointAsSubPrio 2 hoveredNodeId %d", vehicle.ad.hoveredNodeId)
    end
end

function AutoDriveHud:mouseEventCreateWaypoint(vehicle, isUp, button)
    if
        vehicle.ad.nodeToMoveId == nil
        and vehicle.ad.hoveredNodeId == nil
        and button == 1
        and isUp
        -- and not AutoDrive.leftLSHIFTmodifierKeyPressed -- sub-priority
        and AutoDrive.leftCTRLmodifierKeyPressed
        -- and not AutoDrive.leftALTmodifierKeyPressed  -- dual connection
        -- and not AutoDrive.rightSHIFTmodifierKeyPressed -- reverse
        then
        local reverseDirection = AutoDrive.rightSHIFTmodifierKeyPressed
        local subPrio = AutoDrive.leftLSHIFTmodifierKeyPressed and not reverseDirection
        local dualConnection = AutoDrive.leftALTmodifierKeyPressed and not reverseDirection
            
        --For rough depth assertion, we use the closest nodes location as this is roughly in the screen's center
        local closest = vehicle:getClosestWayPoint()
        closest = ADGraphManager:getWayPointById(closest)
        if closest ~= nil then
            local _, _, depth = project(closest.x, closest.y, closest.z)

            local x, y, z = unProject(g_lastMousePosX, g_lastMousePosY, depth)
            -- And just to correct for slope changes, we now set the height to the terrain height
            y = AutoDrive:getTerrainHeightAtWorldPos(x, z)

            local screenX, screenY, depthNew = project(x, y + AutoDrive.drawHeight + AutoDrive.getSetting("lineHeight"), z)

            local maxLoops = 1000
            local minDistance = MathUtil.vector2Length(g_lastMousePosX - screenX, g_lastMousePosY - screenY)
            local minX, minY, minZ = x, y, z
            while minDistance > 0.002 and maxLoops > 0 do
                maxLoops = maxLoops - 1
                if screenY > g_lastMousePosY then
                    depth = depth - 0.0001
                else
                    depth = depth + 0.0001
                end

                x, y, z = unProject(g_lastMousePosX, g_lastMousePosY, depth)
                y = AutoDrive:getTerrainHeightAtWorldPos(x, z)

                screenX, screenY, depthNew = project(x, y + AutoDrive.drawHeight + AutoDrive.getSetting("lineHeight"), z)

                local distance = MathUtil.vector2Length(g_lastMousePosX - screenX, g_lastMousePosY - screenY)
                if distance < minDistance then
                    minX = x
                    minY = y
                    minZ = z
                    minDistance = distance
                end
            end

            if AutoDrive.getSetting("colorAssignmentMode") and g_server ~= nil and g_client ~= nil and g_dedicatedServer == nil then
                -- only allowed in single player game to create the color selection
                AutoDrive.createColorSelectionWayPoints(vehicle)
            else
                ADGraphManager:createWayPoint(minX, minY, minZ)
            end
            -- auto connect to previous created point not working proper in MP, so deactivated at all
            if g_server ~= nil and g_client ~= nil then -- this will be true on dedi servers !!!
                -- auto connect only working in single player properly !
                local createdId = ADGraphManager:getWayPointsCount()
                
                if subPrio then
                    ADGraphManager:toggleWayPointAsSubPrio(createdId)
                    AutoDriveHud.debugMsg(vehicle, "AutoDriveHud:mouseEvent toggleWayPointAsSubPrio 3 createdId %d", createdId)
                end

                if vehicle.ad.newcreated ~= nil and vehicle.ad.selectedNodeId == vehicle.ad.newcreated then
                    -- connect only if previous created point is selected and newcreated ~= nil
                    AutoDriveHud.debugMsg(vehicle, "AutoDriveHud:mouseEvent toggleConnectionBetween 2 vehicle.ad.selectedNodeId %d to %d", vehicle.ad.selectedNodeId, createdId)
                    ADGraphManager:toggleConnectionBetween(ADGraphManager:getWayPointById(vehicle.ad.selectedNodeId), ADGraphManager:getWayPointById(createdId), reverseDirection, dualConnection)
                end
                vehicle.ad.newcreated = createdId
                vehicle.ad.selectedNodeId = vehicle.ad.newcreated
                AutoDriveHud.debugMsg(vehicle, "AutoDriveHud:mouseEvent auto connection 2 selectedNodeId %d", vehicle.ad.selectedNodeId)
            end
        end
    end
end

function AutoDriveHud:mouseEventDeleteWaypoint(vehicle, isUp, button, adjustedPaths)
    if
        vehicle.ad.hoveredNodeId ~= nil
        and vehicle.ad.nodeToMoveId == nil
        and vehicle.ad.selectedNodeId == nil
        and not adjustedPaths
        and button == 1
        and isUp
        and not AutoDrive.leftLSHIFTmodifierKeyPressed
        and not AutoDrive.leftCTRLmodifierKeyPressed
        and AutoDrive.leftALTmodifierKeyPressed
        and not AutoDrive.rightSHIFTmodifierKeyPressed
        then
        -- Left alt for deleting the currently hovered node
        AutoDriveHud.debugMsg(vehicle, "AutoDriveHud:mouseEvent removeWayPoint hoveredNodeId %d", vehicle.ad.hoveredNodeId)
        ADGraphManager:removeWayPoint(vehicle.ad.hoveredNodeId)
    end
end

function AutoDriveHud:mouseEvent(vehicle, posX, posY, isDown, isUp, button)
	local mouseActiveForAutoDrive = (AutoDrive.isMouseActiveForHud() or AutoDrive.isMouseActiveForEditor()) and g_inputBinding:getShowMouseCursor()
	if mouseActiveForAutoDrive then
		local mouseEventHandled = false
		if AutoDrive.splineInterpolation ~= nil then
			AutoDrive.splineInterpolation.valid = false
		end
		AutoDrive.mouseWheelActive = false
		AutoDrive.mouseOverHud = AutoDriveHud:isMouseOverHud(posX, posY)

        if AutoDrive.isMouseActiveForHud() then
    		mouseEventHandled = self:mouseEventOnHudElements(vehicle, posX, posY, isDown, isUp, button)
        end

        if not AutoDrive.isMouseActiveForEditor() then
            -- disable waypoint manipulation
            AutoDrive.resetMouseSelections(vehicle)
        end

        vehicle.ad.hoveredNodeId = nil
        vehicle.ad.sectionWayPoints = {}
        
        local adjustedPaths = false
        if not mouseEventHandled and AutoDrive.isInExtendedEditorMode() then
            self:mouseEventHandleSelection(vehicle, isUp, button)
            self:mouseEventResetSelectedNode(vehicle)
            self:mouseEventFindHoveredNode(vehicle)
            self:mouseEventMoveNode(vehicle)

			if vehicle.ad.hoveredNodeId ~= nil then
                -- mouse is hovering over a waypoint
                self:mouseEventCreateSplineInterpolation(vehicle)
                adjustedPaths = self:mouseEventSelectOrConnectWaypoint(vehicle, isUp, button)
                self:mouseEventStartMovingNode(vehicle, isDown, button)
			end
            self:mouseEventFinishMovingNode(vehicle, isUp, button)
            self:mouseEventSetupAutoConnection(vehicle, isUp, button)
            self:mouseEventToggleWaypointPriority(vehicle, isUp, button, adjustedPaths)
            self:mouseEventCreateWaypoint(vehicle, isUp, button)
            self:mouseEventDeleteWaypoint(vehicle, isUp, button, adjustedPaths)
            AutoDrive.handleWayPointSection(vehicle, button, isUp)
		else
			AutoDrive.resetMouseSelections(vehicle)
		end
	else
		AutoDrive.resetMouseSelections(vehicle)
	end

	AutoDrive.mouseWheelActive = AutoDrive.mouseWheelActive or (AutoDrive.pullDownListExpanded ~= 0)
end

function AutoDrive.resetMouseSelections(vehicle)
	if vehicle ~= nil and vehicle.ad ~= nil then
		vehicle.ad.selectedNodeId = nil
		vehicle.ad.nodeToMoveId = nil
		vehicle.ad.hoveredNodeId = nil
		vehicle.ad.newcreated = nil
		vehicle.ad.sectionWayPoints = {}
		vehicle.ad.selectionWayPoints = {}
	end
end

function AutoDrive.handleWayPointSection(vehicle, button, isUp)
	-- AutoDriveHud.debugMsg(vehicle, "AutoDrive.handleWayPointSection vehicle.ad.selectedNodeId %s vehicle.ad.hoveredNodeId %s", tostring(vehicle.ad.selectedNodeId), tostring(vehicle.ad.hoveredNodeId))
    if vehicle.ad.selectedNodeId ~= nil and vehicle.ad.hoveredNodeId ~= nil and vehicle.ad.selectedNodeId ~= vehicle.ad.hoveredNodeId then
        local wayPointsDirection = ADGraphManager:getIsWayPointJunction(vehicle.ad.selectedNodeId, vehicle.ad.hoveredNodeId)
        if wayPointsDirection > 0 and  wayPointsDirection < 4 then
            vehicle.ad.sectionWayPoints = ADGraphManager:getWayPointsInSection(vehicle.ad.selectedNodeId, vehicle.ad.hoveredNodeId, wayPointsDirection)
			-- AutoDriveHud.debugMsg(vehicle, "AutoDrive.handleWayPointSection button %d isUp %s AutoDrive.leftCTRLmodifierKeyPressed %s", button, tostring(isUp), tostring(AutoDrive.leftCTRLmodifierKeyPressed))
            if button == 1 and isUp
                and not AutoDrive.leftLSHIFTmodifierKeyPressed
                and AutoDrive.leftCTRLmodifierKeyPressed
                and not AutoDrive.leftALTmodifierKeyPressed
                and not AutoDrive.rightSHIFTmodifierKeyPressed
                then
                wayPointsDirection = wayPointsDirection + 1
                if wayPointsDirection > 3 then
                    wayPointsDirection = 1
                end
                ADGraphManager:setConnectionBetweenWayPointsInSection(vehicle.ad.selectedNodeId, vehicle.ad.hoveredNodeId, wayPointsDirection)
                vehicle.ad.selectedNodeId = nil -- unselect the current node after action done
            end

            if button == 1 and isUp
                and AutoDrive.leftLSHIFTmodifierKeyPressed
                and not AutoDrive.leftCTRLmodifierKeyPressed
                and not AutoDrive.leftALTmodifierKeyPressed
                and not AutoDrive.rightSHIFTmodifierKeyPressed
                then
                if vehicle.ad.sectionWayPoints ~= nil and #vehicle.ad.sectionWayPoints > 2 then
                    local sectionPrio = ADGraphManager:getIsPointSubPrio(vehicle.ad.sectionWayPoints[2])   -- 2nd WayPoint is the 1st in section and has the actual Prio
                    local flags = 0
                    if sectionPrio then
                        flags = AutoDrive.FLAG_NONE
                    else
                        flags = AutoDrive.FLAG_SUBPRIO
                    end
                    ADGraphManager:setWayPointsFlagsInSection(vehicle.ad.selectedNodeId, vehicle.ad.hoveredNodeId, flags)
                    vehicle.ad.selectedNodeId = nil -- unselect the current node after action done
                end
            end

            if button == 1 and isUp
                and AutoDrive.leftLSHIFTmodifierKeyPressed
                and AutoDrive.leftCTRLmodifierKeyPressed
                and AutoDrive.leftALTmodifierKeyPressed
                and not AutoDrive.rightSHIFTmodifierKeyPressed
                then
                ADGraphManager:deleteWayPointsInSection(vehicle.ad.selectedNodeId, vehicle.ad.hoveredNodeId)
                vehicle.ad.selectedNodeId = nil -- unselect the current node to prevent further deletions nearby by mouse clicks
            end
        end
    end
end

function AutoDrive.moveNodeToMousePos(nodeID)
	local node = ADGraphManager:getWayPointById(nodeID)

	-- First I use project to get a proper depth value for the unproject funtion
	local _, _, depth = project(node.x, node.y + AutoDrive.drawHeight + AutoDrive.getSetting("lineHeight"), node.z)

	if node ~= nil and g_lastMousePosX ~= nil and g_lastMousePosY ~= nil then
		node.x, _, node.z = unProject(g_lastMousePosX, g_lastMousePosY, depth)
		if not AutoDrive.leftLSHIFTmodifierKeyPressed then
			node.y = AutoDrive:getTerrainHeightAtWorldPos(node.x, node.z)
		end
		ADGraphManager:markChanges()
	end
end

function AutoDriveHud:startMovingHud(mouseX, mouseY)
	self.isMoving = true
	self.lastMousePosX = mouseX
	self.lastMousePosY = mouseY
end

function AutoDriveHud:moveHud(posX, posY)
	if self.isMoving then
		local diffX = posX - self.lastMousePosX
		local diffY = posY - self.lastMousePosY
		self:createHudAt(self.posX + diffX, self.posY + diffY)
		self.lastMousePosX = posX
		self.lastMousePosY = posY
	end
end

function AutoDriveHud:stopMovingHud()
	self.isMoving = false
	ADUserDataManager:sendToServer()
end

function AutoDriveHud:getModeName(vehicle)
	if vehicle.ad.stateModule:getMode() == AutoDrive.MODE_DRIVETO then
		return g_i18n:getText("AD_MODE_DRIVETO")
	elseif vehicle.ad.stateModule:getMode() == AutoDrive.MODE_DELIVERTO then
		return g_i18n:getText("AD_MODE_DELIVERTO")
	elseif vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER then
		return g_i18n:getText("AD_MODE_PICKUPANDDELIVER")
	elseif vehicle.ad.stateModule:getMode() == AutoDrive.MODE_UNLOAD then
		return g_i18n:getText("AD_MODE_UNLOAD")
	elseif vehicle.ad.stateModule:getMode() == AutoDrive.MODE_LOAD then
		return g_i18n:getText("AD_MODE_LOAD")
	elseif vehicle.ad.stateModule:getMode() == AutoDrive.MODE_BGA then
		return g_i18n:getText("AD_MODE_BGA")
	end

	return ""
end

function AutoDriveHud:has_value(tab, val)
	for _, value in ipairs(tab) do
		if value == val then
			return true
		end
	end

	return false
end

function AutoDriveHud:closeAllPullDownLists(vehicle)
	if self.hudElements ~= nil then
		for _, hudElement in pairs(self.hudElements) do
			if hudElement.collapse ~= nil and hudElement.state ~= nil and hudElement.state == ADPullDownList.STATE_EXPANDED then
				hudElement:collapse(vehicle, false)
			end
		end
	end
	-- PullDownList(s) have been collapsed, so need to refresh layer sequence
	self:refreshHudElementsLayerSequence()
end

function AutoDriveHud:createMapHotspot(vehicle)
	local _, textOffsetY = getNormalizedScreenValues(0, -5)
	vehicle.ad.mapHotspot = AIHotspot.new()
	vehicle.ad.mapHotspot:setAIHelperName("AD: " .. vehicle.ad.stateModule:getName())
	vehicle.ad.mapHotspot:setVehicle(vehicle)
	if vehicle.getOwnerFarmId ~= nil then
		vehicle.ad.mapHotspot:setOwnerFarmId(vehicle:getOwnerFarmId())
	end
	vehicle.ad.mapHotspot.textOffsetY = textOffsetY
	g_currentMission.hud:addMapHotspot(vehicle.ad.mapHotspot)
end

function AutoDriveHud:deleteMapHotspot(vehicle)
	if vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.mapHotspot ~= nil then
		vehicle.ad.mapHotspot:setVehicle(nil)
		g_currentMission.hud:removeMapHotspot(vehicle.ad.mapHotspot)
		vehicle.ad.mapHotspot:delete()
		vehicle.ad.mapHotspot = nil
	end
end

function AutoDriveHud:getSelectionWayPoints(vehicle)
    local selectionWayPoints = {}
    if vehicle.ad.selectionActive and vehicle.ad.selectedNodeId then
        local selectedWayPoint = ADGraphManager:getWayPointById(vehicle.ad.selectedNodeId)
        for _, elem in pairs(AutoDrive.getWayPointsDistance(vehicle)) do
            local distance = MathUtil.vector2Length(elem.wayPoint.x - selectedWayPoint.x , elem.wayPoint.z - selectedWayPoint.z)
            if distance <= vehicle.ad.selectionRange then
                elem.wayPoint.isSelected = true
                table.insert(selectionWayPoints, elem.wayPoint.id)
            else
                elem.wayPoint.isSelected = false
            end
        end
    end
    return selectionWayPoints
end

function AutoDrive:ingameMapElementMouseEvent(superFunc, posX, posY, isDown, isUp, button, eventUsed)
	eventUsed = superFunc(self, posX, posY, isDown, isUp, button, eventUsed)

    if isUp and button == Input.MOUSE_BUTTON_LEFT then
        local hotspot = g_currentMission.hud.ingameMap.selectedHotspot
        if hotspot ~= nil and hotspot.isADMarker then
            local targetVehicle = AutoDrive.getADFocusVehicle()
            if targetVehicle ~= nil and AutoDrive.showMarkersOnMainMenuMap() and AutoDrive.getSetting("switchToMarkersOnMap") then
                AutoDriveHudInputEventEvent:sendFirstMarkerEvent(targetVehicle, hotspot.markerID)
                return
            end
        end
    end

    if isUp and button == Input.MOUSE_BUTTON_RIGHT then
        for _, hotspot in pairs(self.ingameMap.hotspots) do
            if hotspot.isADMarker then
                local hotspotPosX, hotspotPosY =  hotspot.lastScreenPositionX, hotspot.lastScreenPositionY
                if hotspotPosX and GuiUtils.checkOverlayOverlap(posX, posY, hotspotPosX, hotspotPosY, hotspot:getWidth(), hotspot:getHeight(), nil) then
                    local targetVehicle = AutoDrive.getADFocusVehicle()
                    if targetVehicle ~= nil and AutoDrive.showMarkersOnMainMenuMap() and AutoDrive.getSetting("switchToMarkersOnMap") then
                        AutoDriveHudInputEventEvent:sendSecondMarkerEvent(targetVehicle, hotspot.markerID)
                    end
                    break
                end
            end
        end
    end

	return eventUsed
end

function AutoDrive.getPlayerHotspot()
--[[
    spec.playerHotspot = PlayerHotspot.new()
    spec.playerHotspot:setVehicle(self)

    g_currentMission:addInteractiveVehicle(self)
    g_currentMission:addEnterableVehicle(self)

    if spec.playerHotspot ~= nil then
        spec.playerHotspot:setOwnerFarmId(self:getActiveFarm())
        g_currentMission:addMapHotspot(spec.playerHotspot)
    end
]]
        local mapHotspot = PlayerHotspot.new()

        -- mapHotspot:setOwnerFarmId(self:getActiveFarm())
        -- mapHotspot:setOwnerFarmId(0) -- all, visitor etc. ???

--[[
        mapHotspot.ownerFarmId = AutoDrive.getPlayer().farmId
        mapHotspot.clickArea.area[1] = 0.13
        mapHotspot.clickArea.area[2] = 0.13
        mapHotspot.clickArea.area[3] = 0.74
        mapHotspot.clickArea.area[4] = 0.74
]]

        -- mapHotspot.isHotspotSelectionActive = true ???
    return mapHotspot
end

function AutoDrive.getTourHotspot()
--[[
    if self.mapHotspot == nil then
        self.mapHotspot = TourHotspot.new()
        g_currentMission:addMapHotspot(self.mapHotspot)
    end

    self.mapHotspot:setWorldPosition(x, z)

    -- Find 'hidden' icon used internally only
    local h = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x,y,z)
    if y > h then
        g_currentMission:setMapTargetHotspot(self.mapHotspot)
        g_currentMission.disableMapTargetHotspotHiding = true
    else
        g_currentMission:setMapTargetHotspot(nil)
        g_currentMission.disableMapTargetHotspotHiding = false
    end
]]
        local mapHotspot = TourHotspot.new()

--[[
        mapHotspot.ownerFarmId = AutoDrive.getPlayer().farmId
        mapHotspot.clickArea.area[1] = 0.13
        mapHotspot.clickArea.area[2] = 0.13
        mapHotspot.clickArea.area[3] = 0.74
        mapHotspot.clickArea.area[4] = 0.74
]]
        -- mapHotspot.isHotspotSelectionActive = true ???
    return mapHotspot
end

function AutoDrive.getPlaceableHotspot()
	--[[
			local hotspot = PlaceableHotspot.new()
			hotspot:setPlaceable(self)
	
			local hotspotTypeName = self.xmlFile:getValue(key .. "#type", "UNLOADING")
			local hotspotType = PlaceableHotspot.getTypeByName(hotspotTypeName)
			if hotspotType == nil then
				Logging.xmlWarning(self.xmlFile, "Unknown placeable hotspot type '%s'. Falling back to type 'UNLOADING'\nAvailable types: %s", hotspotTypeName, table.concatKeys(PlaceableHotspot.TYPE, " "))
				hotspotType = PlaceableHotspot.TYPE.UNLOADING
			end
			hotspot:setPlaceableType(hotspotType)
	
			local linkNode = self.xmlFile:getValue(key .. "#linkNode", nil, self.components, self.i3dMappings) or self.rootNode
			if linkNode ~= nil then
				local x, _, z = getWorldTranslation(linkNode)
				hotspot:setWorldPosition(x, z)
			end
	
			local teleportNode = self.xmlFile:getValue(key .. "#teleportNode", nil, self.components, self.i3dMappings)
			if teleportNode ~= nil then
				local x, y, z = getWorldTranslation(teleportNode)
				hotspot:setTeleportWorldPosition(x, y, z)
			end
	
			local worldPositionX, worldPositionZ = self.xmlFile:getValue(key .. "#worldPosition", nil)
			if worldPositionX ~= nil then
				hotspot:setWorldPosition(worldPositionX, worldPositionZ)
			end
	
			local teleportX, teleportY, teleportZ = self.xmlFile:getValue(key .. "#teleportWorldPosition", nil)
			if teleportX ~= nil then
				if g_currentMission ~= nil then
					teleportY = math.max(teleportY, getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, teleportX, 0, teleportZ))
				end
				hotspot:setTeleportWorldPosition(teleportX, teleportY, teleportZ)
			end
	
			local text = self.xmlFile:getValue(key.."#text", nil)
			if text ~= nil then
				text = g_i18n:convertText(text, self.customEnvironment)
				hotspot:setName(text)
			end
	]]
			local mapHotspot = PlaceableHotspot.new()
	
	--[[
			mapHotspot.ownerFarmId = AutoDrive.getPlayer().farmId
			mapHotspot.clickArea.area[1] = 0.13
			mapHotspot.clickArea.area[2] = 0.13
			mapHotspot.clickArea.area[3] = 0.74
			mapHotspot.clickArea.area[4] = 0.74
	]]
			-- mapHotspot.isHotspotSelectionActive = true ???
	
			mapHotspot:setPlaceableType(PlaceableHotspot.TYPE.UNLOADING)
			-- mapHotspot:setTeleportWorldPosition(x, y, z)
	
	
		return mapHotspot
	end

function AutoDrive.showMarkersOnMainMenuMap()
	return bit32.band(AutoDrive.getSetting("showMarkersOnMap"), 0x01) > 0
end

function AutoDrive.showMarkersOnIngameMap()
	return bit32.band(AutoDrive.getSetting("showMarkersOnMap"), 0x02) > 0
end

function AutoDrive.updateDestinationsMapHotspots()
    AutoDrive.debugPrint(nil, AutoDrive.DC_DEVINFO, "AutoDrive.updateDestinationsMapHotspots()")

    local width, height = getNormalizedScreenValues(9, 9)

	if AutoDrive.mapHotspotsBuffer ~= nil then
        -- Removing all old map hotspots
        for _, mapHotspot in pairs(AutoDrive.mapHotspotsBuffer) do
            g_currentMission.hud:removeMapHotspot(mapHotspot)
            mapHotspot:delete()
        end
    end
    AutoDrive.mapHotspotsBuffer = {}

	-- Updating and adding hotspots
    for index, marker in ipairs(ADGraphManager:getMapMarkers()) do
		local wp = ADGraphManager:getWayPointById(marker.id)
        if wp ~= nil then
			local mapHotspot = AutoDriveHotspot.new(index, marker)
			g_currentMission.hud:addMapHotspot(mapHotspot)
            table.insert(AutoDrive.mapHotspotsBuffer, mapHotspot)

            mapHotspot:setWorldPosition(wp.x, wp.z)

            mapHotspot:setTeleportWorldPosition(wp.x, wp.y + 2, wp.z)

            mapHotspot:setName(marker.name)
        end
    end
end

function AutoDrive.createColorSelectionWayPoints(vehicle)
    if vehicle ~= nil and vehicle.ad ~= nil  then
        local startNode = vehicle.ad.frontNode
        local x1, _, z1 = getWorldTranslation(startNode)
        local y1 = AutoDrive:getTerrainHeightAtWorldPos(x1, z1)

		local function hsv_to_rgb(h, s, v)
			local p = v * (1 - s)
			local q = v - p
			local r = math.clamp(3 * math.abs((h / 180) % 2 - 1) - 1, 0, 1)
			local g = math.clamp(3 * math.abs(((h - 120) / 180) % 2 - 1) - 1, 0, 1)
			local b = math.clamp(3 * math.abs(((h + 120) / 180) % 2 - 1) - 1, 0, 1)
			return { p + q * r, p + q * g, p + q * b }
		end

		for sv = 0, 100, 5 do
			local s = math.clamp(sv / 50, 0, 1)
			local v = math.clamp((100 - sv) / 50, 0, 1)
			for h = 0, 359, 600/math.max(sv, 1) do
				local h_rad = h * math.pi / 180
				local colors = hsv_to_rgb(h, s, v)
				local x = math.cos(h_rad) * sv / 15
                local y = math.sin(h_rad) * sv / 15
                local rx, _, rz = AutoDrive.localDirectionToWorld(vehicle, x, 0, y, startNode)
                ADGraphManager:createWayPointColored(x1 + rx, y1 + 1, z1 + rz, colors)
			end
		end
    end
end

function AutoDriveHud.debugMsg(vehicle, debugText, ...)
    if AutoDriveHud.debug == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    end
end
