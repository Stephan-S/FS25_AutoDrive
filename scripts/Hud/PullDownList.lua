ADPullDownList = ADInheritsFrom(ADGenericHudElement)

ADPullDownList.STATE_COLLAPSED = 0
ADPullDownList.STATE_EXPANDED = 1

ADPullDownList.TYPE_TARGET = 1
ADPullDownList.TYPE_UNLOAD = 2
ADPullDownList.TYPE_FILLTYPE = 3

ADPullDownList.EXPANDED_DOWN = -1
ADPullDownList.EXPANDED_UP = 1

ADPullDownList.MAX_SHOWN = 24
ADPullDownList.MIN_SHOWN = 2

-- top level icons
ADPullDownList.ICON_COLLAPSALL = 1
ADPullDownList.ICON_FILTER = 2
ADPullDownList.ICON_CREATE_FOLDER = 3

-- folder icons
-- ADPullDownList.ICON_EXPAND_COLLAPS = 1
ADPullDownList.ICON_DELETE_FOLDER = 1


function ADPullDownList:initReusableOverlaysOnlyOnce()
    local function overlayReuseOrNew(overlay, imageSlice)
        if overlay == nil then
            overlay = g_overlayManager:createOverlay(imageSlice, 0, 0, self.iconSize.width, self.iconSize.height)
        end
        return overlay
    end

    -- For avoiding creating/destroying multiple Overlay objects in onDraw, so here we only create
    -- one instance of each icon (overlay). Then in onDraw we use the game engine's renderOverlay()
    -- API method directly. Should reduce excessive memory allocation/deallocation tremendously.
    ADPullDownList.ovPlus = overlayReuseOrNew(ADPullDownList.ovPlus, self.imagePlus)
    ADPullDownList.ovMinus = overlayReuseOrNew(ADPullDownList.ovMinus, self.imageMinus)
    ADPullDownList.ovFilter = overlayReuseOrNew(ADPullDownList.ovFilter, self.imageFilter)
    ADPullDownList.ovCollapseAll = overlayReuseOrNew(ADPullDownList.ovCollapseAll, self.imageCollapseAll)
end

function ADPullDownList:new(posX, posY, width, height, type, selected)
    local o = ADPullDownList:create()
    o:init(posX, posY, width, height)
    o.selected = selected
    o.hovered = selected
    o.type = type
    o.size.height = AutoDrive.Hud.buttonHeight  -- uiScale * 32
    self.autoLoadFillTypes = nil

    AutoDrive.pullDownListExpanded = 0
-- icons in list 1/2 size of HUD icons?
-- position from right to left
-- right icon:
    o.rightIconPos  = {x = o.position.x + o.size.width -  AutoDrive.Hud.buttonWidth / 2      - AutoDrive.Hud.gapWidth,     y = o.position.y + (o.size.height - AutoDrive.Hud.buttonHeight / 2) / 2}
    o.rightIconPos2 = {x = o.position.x + o.size.width - (AutoDrive.Hud.buttonWidth / 2) * 2 - AutoDrive.Hud.gapWidth * 3, y = o.position.y + (o.size.height - AutoDrive.Hud.buttonHeight / 2) / 2}
    o.rightIconPos3 = {x = o.position.x + o.size.width - (AutoDrive.Hud.buttonWidth / 2) * 3 - AutoDrive.Hud.gapWidth * 5, y = o.position.y + (o.size.height - AutoDrive.Hud.buttonHeight / 2) / 2}
-- most left icon:
    o.rightIconPos4 = {x = o.position.x + o.size.width - (AutoDrive.Hud.buttonWidth / 2) * 4 - AutoDrive.Hud.gapWidth * 6, y = o.position.y + (o.size.height - AutoDrive.Hud.buttonHeight / 2) / 2}

-- icons in list 1/2 size of HUD icons
    o.iconSize = {width = AutoDrive.Hud.buttonWidth / 2, height = AutoDrive.Hud.buttonHeight / 2}
    o.rowSize = {width = AutoDrive.Hud.buttonWidth / 2, height = AutoDrive.Hud.listItemHeight / 2}

    o.layer = 6

    o.imageBG = "ad_gui.4xlongBorderFilled"
    o.imageBGTop = "ad_gui.4xlongBorderTopFilled"
    o.imageBGBottom = "ad_gui.4xlongBorderBottomFilled"
    o.imageBGStretch = "ad_gui.4xlongBorderStretchFilled"
    o.imageExpand = "ad_gui.arrowExpand"
    o.imageCollapseAll = "ad_gui.arrowCollapseAll"
    o.imagePlus = "ad_gui.plusSign"
    o.imageMinus = "ad_gui.minusSign"
    o.imageFilter = "ad_gui.zoom"

    o.ovBG = g_overlayManager:createOverlay(o.imageBG, o.position.x, o.position.y, o.size.width, o.size.height)
    o.ovExpand = g_overlayManager:createOverlay(o.imageExpand, o.rightIconPos.x, o.rightIconPos.y, o.iconSize.width, o.iconSize.height)

    o.state = ADPullDownList.STATE_COLLAPSED
    o.isVisible = true

    o:createSelection()

    o.expandedSize = {width = o.size.width, height = o.rowSize.height * (ADPullDownList.MAX_SHOWN + 1) + o.size.height / 2}
    if o.position.y >= 0.5 then
        o.direction = ADPullDownList.EXPANDED_DOWN
    else
        o.direction = ADPullDownList.EXPANDED_UP
    end

    o:initReusableOverlaysOnlyOnce()

    return o
end

function ADPullDownList:update(dt)
    if self.startedDraggingTimer ~= nil then
        self.startedDraggingTimer = self.startedDraggingTimer + dt
    end
end

function ADPullDownList:onDraw(vehicle, uiScale)
    if not (self.type ~= ADPullDownList.TYPE_FILLTYPE or vehicle.ad.stateModule:getMode() == AutoDrive.MODE_LOAD or vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER) then
        -- do not show fruit pulldownlist in modes where it makes no sense
        return
    end
    self:updateState(vehicle)
    if self.isVisible == false then
        return
    end

    local adFontSize = AutoDrive.FONT_SCALE * uiScale
    setTextAlignment(RenderText.ALIGN_LEFT)

    if self.state == ADPullDownList.STATE_COLLAPSED then
        self.ovBG:render()
        self.ovExpand:render()

        if (AutoDrive.pullDownListExpanded ~= 0) and ((AutoDrive.pullDownListExpanded > self.type and AutoDrive.pullDownListDirection == ADPullDownList.EXPANDED_UP) or (AutoDrive.pullDownListExpanded < self.type and AutoDrive.pullDownListDirection == ADPullDownList.EXPANDED_DOWN)) then
            -- Do not draw text, as other pull-down-list is expanded and "blocks" the visiblity of this text
            return
        end

        local text = self:shortenTextToWidth(self.text, (self.size.width - 3 * AutoDrive.Hud.gapWidth - self.iconSize.width), adFontSize)

        local textHeight = getTextHeight(adFontSize, text)

        local posX = self.position.x + AutoDrive.Hud.gapWidth
        local posY = self.position.y + (self.size.height - textHeight) / 2

        setTextBold(false)
        setTextColor(unpack(AutoDrive.currentColors.ad_color_hudTextDefault))

        -- TODO: Move this out of onDraw, as it SHOULD NOT be checked at-every-frame-update whether text needs to be rendered in green or not.
        if vehicle.ad.stateModule:isActive() then
            local targetToCheck = "nil"
            if self.type == ADPullDownList.TYPE_TARGET and vehicle.ad.stateModule:getFirstMarker() ~= nil then
                targetToCheck = vehicle.ad.stateModule:getFirstMarker().name
            elseif self.type == ADPullDownList.TYPE_UNLOAD and vehicle.ad.stateModule:getSecondMarker() ~= nil then
                targetToCheck = vehicle.ad.stateModule:getSecondMarker().name
            end
            local actualTarget = ""

            local destination = vehicle.ad.stateModule:getCurrentDestination()
            if destination ~= nil then
                actualTarget = destination.name
            end

            if actualTarget == targetToCheck then
                setTextColor(unpack(AutoDrive.currentColors.ad_color_hudTextHover))
            end
        end

        renderText(posX, posY, adFontSize, text)
    else
        drawFilledRect(self.position.x, self.expandedBottom, self.size.width, self.expandedSize.height, 0, 0, 0, 0.9)
        self.ovTop:render()
        self.ovStretch:render()
        self.ovBottom:render()

        local useFolders = AutoDrive.getSetting("useFolders")

        -- fixed position for top icons + actions
        if self.type ~= ADPullDownList.TYPE_FILLTYPE then
            -- handle top icons + actions -> in line 1
            local boxPos = self:getTextPositionByDisplayIndex(1, uiScale)
            local posY = boxPos.y

            -- left: collapse all
            renderOverlay(ADPullDownList.ovCollapseAll.overlayId,   self.rightIconPos.x,  posY, self.iconSize.width, self.iconSize.height)
            -- 2nd: filter
            renderOverlay(ADPullDownList.ovFilter.overlayId,        self.rightIconPos2.x, posY, self.iconSize.width, self.iconSize.height)

            if AutoDrive.isEditorModeEnabled() then
                -- 3rd: add folder
                renderOverlay(ADPullDownList.ovPlus.overlayId,          self.rightIconPos3.x, posY, self.iconSize.width, self.iconSize.height)
            end
        end

        for i = 1, ADPullDownList.MAX_SHOWN - 1, 1 do
            local listEntry = self:getListElementByDisplayIndex(vehicle, i)
            if listEntry ~= nil then
                local text = listEntry.displayName
                if text == "All" and listEntry.isFolder then
                    text = g_i18n:getText("gui_ad_default")
                    if vehicle.ad.destinationFilterText ~= "" then
                        text = text .. " " .. g_i18n:getText("gui_ad_filter") .. ": " .. vehicle.ad.destinationFilterText
                    end
                end
                if listEntry.isFolder == false and self.type ~= ADPullDownList.TYPE_FILLTYPE and useFolders then
                    text = "   " .. text
                end

                local textTargetWidth

                if listEntry.isFolder then
                    textTargetWidth = math.abs(self.rightIconPos3.x - self.position.x) - AutoDrive.Hud.gapWidth
                else
                    textTargetWidth = math.abs(self.rightIconPos.x - self.position.x) + AutoDrive.Hud.gapWidth
                end
                text = self:shortenTextToWidth(text, textTargetWidth, adFontSize)

                local textPosition = self:getTextPositionByDisplayIndex(i + 1, uiScale)     -- start 1 line later

                if listEntry.isFolder then
--[[
                    -- expand / collaps icons for folder in list
                    if vehicle.ad.groups[listEntry.displayName] then
                        renderOverlay(ADPullDownList.ovCollapse.overlayId, self.rightIconPos.x, textPosition.y, self.iconSize.width, self.iconSize.height)
                    else
                        renderOverlay(ADPullDownList.ovExpand.overlayId, self.rightIconPos.x, textPosition.y, self.iconSize.width, self.iconSize.height)
                    end
]]

                    if AutoDrive.isEditorModeEnabled() then
                        if (listEntry.displayName ~= "All") and self:getItemCountForGroup(listEntry.displayName) <= 0 then
                            -- icon minus for delete folder only for empty folders and not standard folder
                            renderOverlay(ADPullDownList.ovMinus.overlayId, self.rightIconPos.x, textPosition.y, self.iconSize.width, self.iconSize.height)
                        end
                    end
                end

                local makeBold = listEntry.isFolder
                local makeSpecial = listEntry.isFolder

                if self.type == ADPullDownList.TYPE_FILLTYPE then
                    makeBold = table.contains(vehicle.ad.stateModule:getSelectedFillTypes(), listEntry.returnValue)
                    makeSpecial = makeBold or vehicle.ad.stateModule:getFillType() == listEntry.returnValue
                end

                setTextBold(makeBold)

                if self.hovered == self.selected + (i - 1) then
                    -- mouse hovering over selected item
                    if makeSpecial then
                        setTextColor(unpack(AutoDrive.currentColors.ad_color_hudTextHoverSpecial))
                    else
                        setTextColor(unpack(AutoDrive.currentColors.ad_color_hudTextHover))
                    end
                else
                    -- other element
                    if makeSpecial then
                        setTextColor(unpack(AutoDrive.currentColors.ad_color_hudTextSpecial))
                    else
                        setTextColor(unpack(AutoDrive.currentColors.ad_color_hudTextDefault))
                    end
                end

                renderText(textPosition.x, textPosition.y, adFontSize, text)
            else
                if i == 1 then
                    i = i - 1
                    self.selected = 1
                    self.hovered = 1
                end
            end
        end

        -- moving item to draw in yellow
        if AutoDrive.isEditorModeEnabled() and self.dragged ~= nil and self.startedDraggingTimer > 200 then
            if g_lastMousePosX ~= nil and g_lastMousePosY ~= nil then
                setTextBold(true)
                setTextColor(unpack(AutoDrive.currentColors.ad_color_hudTextHover))

                renderText(g_lastMousePosX, g_lastMousePosY, adFontSize, self.draggedElement.displayName)
            end
        end
    end
end

function ADPullDownList:shortenTextToWidth(textInput, maxWidth, fontSize)
    if textInput == nil then
        return ""
    end
    local maxNumChars = getTextLineLength(fontSize, textInput, maxWidth)
    return utf8Substr(textInput, 0, maxNumChars)
end

function ADPullDownList:getListElementByDisplayIndex(vehicle, index)
    return self:getListElementByIndex(vehicle, (index - 1) + self.selected)
end

function ADPullDownList:getListElementByIndex(vehicle, index)
    local counter = 1
    if self.type ~= ADPullDownList.TYPE_FILLTYPE then
        local useFolders = AutoDrive.getSetting("useFolders")
        for groupID, entries in pairs(self.options) do
            if useFolders then
                if counter == index then
                    return {displayName = self:groupIDToGroupName(self.fakeGroupIDs[groupID]), returnValue = self:groupIDToGroupName(self.fakeGroupIDs[groupID]), isFolder = true}
                end
                counter = counter + 1
            end
            if vehicle.ad.groups[self:groupIDToGroupName(self.fakeGroupIDs[groupID])] == true or (not useFolders) then
                for _, entry in pairs(entries) do
                    if vehicle.ad.destinationFilterText == "" or string.match(string.lower(entry.displayName), string.lower(vehicle.ad.destinationFilterText)) then
                        if counter == index then
                            return {displayName = entry.displayName, returnValue = entry.returnValue, isFolder = false}
                        end
                        counter = counter + 1
                    end
                end
            end
        end
    else
        local entry = self.options[1][index]
        if entry ~= nil then
            return {displayName = entry.displayName, returnValue = entry.returnValue, isFolder = false}
        end
    end
    return nil
end

function ADPullDownList:getBoxPositionByDisplayIndex(index)
    local posX = self.position.x + AutoDrive.Hud.gapWidth
    local posY = self.position.y + AutoDrive.Hud.listItemHeight * (index - 1) * -1
    if self.direction == ADPullDownList.EXPANDED_UP then
        posY = posY + self.expandedSize.height - self.size.height
    end
    return {x = posX, y = posY, width = self.expandedSize.width, height = self.size.height}
end

function ADPullDownList:getTextPositionByDisplayIndex(index, uiScale)
    local boxPos = self:getBoxPositionByDisplayIndex(index)

    local adFontSize = AutoDrive.FONT_SCALE * uiScale
    local textHeight = getTextHeight(adFontSize, "text")

    boxPos.y = boxPos.y + (self.size.height - textHeight) / 2
    return boxPos
end

function ADPullDownList:getElementAt(vehicle, posX, posY)
    local uiScale = g_gameSettings:getValue("uiScale")
    if AutoDrive.getSetting("guiScale") ~= 0 then
        uiScale = AutoDrive.getSetting("guiScale")
    end

    -- handle top icons + actions -> in line 1
    local boxPos = self:getTextPositionByDisplayIndex(1, uiScale)
    if posX >= boxPos.x and posX <= (boxPos.x + boxPos.width) and posY >= boxPos.y and posY <= (boxPos.y + boxPos.height) then
        local hitIcon = 0
        if posX >= self.rightIconPos.x then
            hitIcon = 1
        elseif posX >= self.rightIconPos2.x then
            hitIcon = 2
        elseif posX >= self.rightIconPos3.x then
            hitIcon = 3
        elseif posX >= self.rightIconPos4.x then
            hitIcon = 4
        end

        return nil, 0, hitIcon
    end

    for i = 1, ADPullDownList.MAX_SHOWN - 1, 1 do
        local listEntry = self:getListElementByDisplayIndex(vehicle, i)
        if listEntry ~= nil then
            local boxPos = self:getTextPositionByDisplayIndex(i + 1, uiScale)

            if posX >= boxPos.x and posX <= (boxPos.x + boxPos.width) and posY >= boxPos.y and posY <= (boxPos.y + boxPos.height) then
                local hitIcon = 0
                if posX >= self.rightIconPos.x then
                    hitIcon = 1
                elseif posX >= self.rightIconPos2.x then
                    hitIcon = 2
                elseif posX >= self.rightIconPos3.x and listEntry.isFolder then
                    hitIcon = 3
                elseif posX >= self.rightIconPos4.x and listEntry.isFolder then
                    hitIcon = 4
                end

                return listEntry, i, hitIcon
            end
        end
    end
    return nil
end

function ADPullDownList:updateState(vehicle)
    local newState, newSelection = self:getNewState(vehicle)
    self.state = newState
    self.selected = newSelection
    self:updateVisibility(vehicle)
end

function ADPullDownList:updateVisibility(vehicle)
    local newVisibility = self.isVisible
    if self.type == ADPullDownList.TYPE_UNLOAD then
        if (vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER or vehicle.ad.stateModule:getMode() == AutoDrive.MODE_UNLOAD or vehicle.ad.stateModule:getMode() == AutoDrive.MODE_LOAD) then
            newVisibility = true
        else
            newVisibility = false
        end
    end
    if self.type == ADPullDownList.TYPE_TARGET then
        if vehicle.ad.stateModule:getMode() == AutoDrive.MODE_BGA then
            newVisibility = false
        else
            newVisibility = true
        end
    end

    self.isVisible = newVisibility
end

function ADPullDownList:createSelection()
    self.fakeGroupIDs = {}
    self.fakeGroupIDs[1] = 1
    if self.type == ADPullDownList.TYPE_TARGET then
        self:createSelection_Target()
        self:sortCurrentItems()
    elseif self.type == ADPullDownList.TYPE_UNLOAD then
        self:createSelection_Target()
        self:sortCurrentItems()
    elseif self.type == ADPullDownList.TYPE_FILLTYPE then
        self:createSelection_FillType()
        self:sortFillTypes()
    end

end

function ADPullDownList:createSelection_Target()
    local useFolders = AutoDrive.getSetting("useFolders")

    self.options = {}
    if useFolders then
        self:sortGroups()
    else
        self.options[1] = {}
    end

    if #self.options == 0 then
        self.options[1] = {}
    end

    for markerID, marker in pairs(ADGraphManager:getMapMarkers()) do
        if useFolders then
            table.insert(self.options[self.groups[marker.group]], {displayName = marker.name, returnValue = markerID})
        else
            table.insert(self.options[1], {displayName = marker.name, returnValue = markerID})
        end
    end
end

function ADPullDownList:sortGroups()
    self.options = {}

    local sort_func = function(a, b)
        a = string.lower(tostring(a))
        b = string.lower(tostring(b))
        local patt = "^(.-)%s*(%d+)$"
        local _, _, col1, num1 = a:find(patt)
        local _, _, col2, num2 = b:find(patt)
        if (col1 and col2) and col1 == col2 then
            return tonumber(num1) < tonumber(num2)
        end
        return a < b
    end

    local inverseTable = {}
    for groupName, groupID in pairs(ADGraphManager:getGroups()) do
        inverseTable[groupID] = groupName
    end

    table.sort(inverseTable, sort_func)

    self.options[1] = {}
    self.groups = {}
    self.groups["All"] = 1
    self.fakeGroupIDs = {}
    self.fakeGroupIDs[1] = 1

    local i = 2
    for _, groupName in pairs(inverseTable) do
        if groupName ~= "All" then
            self.options[i] = {}
            self.groups[groupName] = i
            self.fakeGroupIDs[i] = ADGraphManager:getGroupByName(groupName)
            i = i + 1
        end
    end
end

function ADPullDownList:createSelection_FillType()
    local supportedFillTypes = nil
    self.autoLoadFillTypes = nil
	local vehicle = AutoDrive.getADFocusVehicle()

    if vehicle ~= nil then
        local trailers, _ = AutoDrive.getAllUnits(vehicle)
        if trailers then
            for _, trailer in ipairs(trailers) do
                if AutoDrive:hasAL(trailer) then
                    local alFillTypes = AutoDrive:getALFillTypes(trailer)
                    if alFillTypes ~= nil and #alFillTypes > 0 then
                        -- self.autoLoadFillTypes is either nil or it contains items. It is never empty.
                        self.autoLoadFillTypes = alFillTypes
                        break
                    end
                end
            end
        end
        if self.autoLoadFillTypes == nil then
            supportedFillTypes = AutoDrive.getValidSupportedFillTypes(vehicle)
        end
    end

    self.options = {}
    self.options[1] = {}
    local fillTypeIndex = 1
    local itemListIndex = 1
    local lastIndexReached = false
    self.options[1][itemListIndex] = {displayName = "", returnValue = 0} -- default empty list
    if self.autoLoadFillTypes ~= nil then
        -- AutoLoad
        self.options[1][itemListIndex] = {displayName = g_i18n:getText("gui_ad_Text_all"), returnValue = AutoDrive.UAL_FILLTYPE_ALL}
        -- AutoDrive.UAL_FILLTYPE_ALL is for all fillTypes in UAL
        -- 1st entry in list is key for all filltypes
        itemListIndex = itemListIndex + 1
        for _, fillType in ipairs(self.autoLoadFillTypes) do
            self.options[1][itemListIndex] = {displayName = fillType.fillTypeName, returnValue = fillType.fillTypeID}
            itemListIndex = itemListIndex + 1
        end
    else
        while not lastIndexReached do
            if g_fillTypeManager:getFillTypeByIndex(fillTypeIndex) ~= nil then
                if (not AutoDriveHud:has_value(AutoDrive.ItemFilterList, fillTypeIndex)) and (supportedFillTypes == nil or table.contains(supportedFillTypes, fillTypeIndex)) then
                    self.options[1][itemListIndex] = {displayName = g_fillTypeManager:getFillTypeByIndex(fillTypeIndex).title, returnValue = fillTypeIndex}
                    itemListIndex = itemListIndex + 1
                end
            else
                lastIndexReached = true
            end
            fillTypeIndex = fillTypeIndex + 1
        end
    end
end

function ADPullDownList:getNewState(vehicle)
    local newState = self.state
    local newSelection = self.selected
    if self.type == ADPullDownList.TYPE_TARGET then
        self:getNewState_Target(vehicle)
    elseif self.type == ADPullDownList.TYPE_UNLOAD then
        self:getNewState_Unload(vehicle)
    elseif self.type == ADPullDownList.TYPE_FILLTYPE then
        self:getNewState_FillType(vehicle)
    end

    return newState, newSelection
end

function ADPullDownList:getNewState_Target(vehicle)
    local newState = self.state
    local newSelection = self.selected
    if self.state == ADPullDownList.STATE_COLLAPSED then
        local markerSelected = vehicle.ad.stateModule:getFirstMarkerId()
        if markerSelected ~= nil and markerSelected >= 1 and ADGraphManager:getMapMarkerById(markerSelected) ~= nil then
            self.text = ADGraphManager:getMapMarkerById(markerSelected).name
        else
            self.text = ""
        end
    end
    return newState, newSelection
end

function ADPullDownList:getNewState_Unload(vehicle)
    local newState = self.state
    local newSelection = self.selected
    if self.state == ADPullDownList.STATE_COLLAPSED then
        local markerSelected = vehicle.ad.stateModule:getSecondMarker()
        if markerSelected ~= nil then
            self.text = markerSelected.name
        else
            self.text = ""
        end
    end
    return newState, newSelection
end

function ADPullDownList:getNewState_FillType(vehicle)
    local newState = self.state
    local newSelection = self.selected
    if self.state == ADPullDownList.STATE_COLLAPSED then
        if self.autoLoadFillTypes ~= nil then
            local currentFillTypeID = vehicle.ad.stateModule:getFillType()
            if currentFillTypeID == AutoDrive.UAL_FILLTYPE_ALL then
                self.text = self.options[1][1].displayName -- first item in list is for all fillTypes in UAL
            else
                self.text = g_fillTypeManager:getFillTypeTitleByIndex(currentFillTypeID) or ""
            end
        else
            local fillType = g_fillTypeManager:getFillTypeByIndex(vehicle.ad.stateModule:getFillType())
            if fillType and vehicle.ad.stateModule:getFillType() ~= g_fillTypeManager:getFillTypeIndexByName('UNKNOWN') then
                self.text = fillType.title
            else
                self.text = ""
            end
        end
    end
    return newState, newSelection
end

function ADPullDownList:hit(posX, posY, layer)
    if self.state == ADPullDownList.STATE_EXPANDED then
        if self.direction == ADPullDownList.EXPANDED_DOWN then
            return layer <= self.layer and posX >= self.position.x and posX <= (self.position.x + self.expandedSize.width) and posY >= (self.position.y - self.expandedSize.height + self.size.height) and posY <= (self.position.y + self.size.height)
        else
            return layer <= self.layer and posX >= self.position.x and posX <= (self.position.x + self.expandedSize.width) and posY >= (self.position.y) and posY <= (self.position.y + self.expandedSize.height)
        end
    end
    return layer <= self.layer and posX >= self.position.x and posX <= (self.position.x + self.size.width) and posY >= self.position.y and posY <= (self.position.y + self.size.height)
end

function ADPullDownList:act(vehicle, posX, posY, isDown, isUp, button)
    if (self.type ~= ADPullDownList.TYPE_FILLTYPE or vehicle.ad.stateModule:getMode() == AutoDrive.MODE_LOAD or vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER) and (self.type ~= ADPullDownList.TYPE_UNLOAD or vehicle.ad.stateModule:getMode() == AutoDrive.MODE_LOAD or vehicle.ad.stateModule:getMode() == AutoDrive.MODE_PICKUPANDDELIVER or vehicle.ad.stateModule:getMode() == AutoDrive.MODE_UNLOAD) then
        local hitElement, hitIndex, hitIcon = self:getElementAt(vehicle, posX, posY)
        if button == 1 and isUp then
            -- left mouse button
            if self.state == ADPullDownList.STATE_EXPANDED and self.type == ADPullDownList.TYPE_FILLTYPE and AutoDrive.leftCTRLmodifierKeyPressed then
                -- left Ctrl click on open filltype list. Toggle item, including auto-load
                local value = self:getHoverEntryReturnValue(vehicle)
                if value ~= nil then
                    AutoDriveHudInputEventEvent:sendToggleFillTypeSelectionEvent(vehicle, value)
                end
            elseif self.state == ADPullDownList.STATE_EXPANDED and self.type == ADPullDownList.TYPE_FILLTYPE and AutoDrive.leftALTmodifierKeyPressed then
                -- left Alt click on open filltype list. Toggle item, including auto-load
                local value = self:getHoverEntryReturnValue(vehicle)
                if value ~= nil then
                    AutoDriveHudInputEventEvent:sendToggleAllFillTypeSelectionsEvent(vehicle, value)
                end
            elseif self.state == ADPullDownList.STATE_EXPANDED and self.type ~= ADPullDownList.TYPE_FILLTYPE and AutoDrive.isEditorModeEnabled() and AutoDrive.getSetting("useFolders") and self.dragged ~= nil and self.startedDraggingTimer > 200 then
                -- drag element to hitElement
                if hitElement ~= nil then
                    self:sortDraggedInGroup(self.draggedElement, hitElement)
                end
            elseif self.state == ADPullDownList.STATE_COLLAPSED and AutoDrive.pullDownListExpanded <= 0 then
                -- expand the collapsed list
                self:expand(vehicle)
            elseif self.state == ADPullDownList.STATE_EXPANDED and (hitElement ~=nil and not hitElement.isFolder) then
                -- clicked outside -> collaps complete list
                self:collapse(vehicle, true)
            elseif self.state == ADPullDownList.STATE_EXPANDED and self.type ~= ADPullDownList.TYPE_FILLTYPE and hitIcon ~= nil and hitIcon == ADPullDownList.ICON_COLLAPSALL and hitIndex ~= nil and hitIndex == 0 then
                -- top icon -> collaps all folders
                for groupId,_ in pairs(vehicle.ad.groups) do
                    vehicle.ad.groups[groupId] = false
                end
                self.selected = 1
                self.hovered = 1
            elseif self.state == ADPullDownList.STATE_EXPANDED and self.type ~= ADPullDownList.TYPE_FILLTYPE and hitIcon ~= nil and hitIcon == ADPullDownList.ICON_FILTER and hitIndex ~= nil and hitIndex == 0 then
                -- top icon -> Filter
                ADInputManager:onInputCall(vehicle, "input_setDestinationFilter")
                -- TODO: show only results
                for groupId,_ in pairs(vehicle.ad.groups) do
                    vehicle.ad.groups[groupId] = true
                end
                self.selected = 1
                self.hovered = 1
            elseif self.state == ADPullDownList.STATE_EXPANDED and self.type ~= ADPullDownList.TYPE_FILLTYPE and hitIcon ~= nil and hitIcon == ADPullDownList.ICON_CREATE_FOLDER and hitIndex ~= nil and hitIndex == 0 and AutoDrive.isEditorModeEnabled() then
                -- create new folder
                -- self:collapse(vehicle, true)
                AutoDrive.onOpenEnterGroupName()
            elseif self.state == ADPullDownList.STATE_EXPANDED and hitElement ~= nil then
                -- clicked in element list, not top icon
                if (hitIcon ~= nil and hitIcon ~= ADPullDownList.ICON_DELETE_FOLDER) and hitElement.isFolder then
                    -- clicked folder, but not icon delete
                    if hitElement.isFolder then
                        vehicle.ad.groups[hitElement.returnValue] = not vehicle.ad.groups[hitElement.returnValue]
                    end
                elseif self.state == ADPullDownList.STATE_EXPANDED and hitIcon ~= nil and hitIcon == ADPullDownList.ICON_DELETE_FOLDER and AutoDrive.isEditorModeEnabled() then
                    -- remove empty folder
                    if hitElement.isFolder and (hitElement.displayName ~= "All") then
                        if self:getItemCountForGroup(hitElement.displayName) <= 0 then
                            -- AutoDrive.pullDownListExpanded = 0
                            ADGraphManager:removeGroup(hitElement.returnValue)
                        end
                    end
                end
            end
            self.dragged = nil
            self.startedDraggingTimer = nil
            return true
        elseif (button == 3) and isUp then
            -- right mouse button
            if self.state == ADPullDownList.STATE_EXPANDED and self.type ~= ADPullDownList.TYPE_FILLTYPE and hitIcon ~= nil and hitIcon == ADPullDownList.ICON_FILTER and hitIndex ~= nil and hitIndex == 0 then
                -- clear filter
                vehicle.ad.destinationFilterText = ""
                for groupId,_ in pairs(vehicle.ad.groups) do
                    vehicle.ad.groups[groupId] = false
                end
            end
            return true
        elseif button == 4 and isUp then
            -- mouse wheel
            local oldSelected = self.selected
            local decrement = 1
            if AutoDrive.leftCTRLmodifierKeyPressed then
                decrement = 10
            end
            self.selected = math.max(1, self.selected - decrement)
            if oldSelected ~= self.selected then
                self.hovered = math.max(1, self.hovered - decrement)
            end
            if self.hovered > (self.selected + ADPullDownList.MAX_SHOWN - decrement) then
                self.hovered = self.selected + ADPullDownList.MAX_SHOWN - decrement
            end
            AutoDrive.mouseWheelActive = true
            return true
        elseif button == 5 and isUp then
            -- mouse wheel
            local increment = 1
            if AutoDrive.leftCTRLmodifierKeyPressed then
                increment = 10
            end
            if self:getListElementByIndex(vehicle, self.selected + increment + ADPullDownList.MAX_SHOWN - 3) ~= nil then
                self.selected = self.selected + increment
                if self:getListElementByIndex(vehicle, self.hovered + increment + ADPullDownList.MAX_SHOWN - 3) ~= nil then
                    self.hovered = self.hovered + increment
                end
            end
            AutoDrive.mouseWheelActive = true
            return true
        elseif self.state == ADPullDownList.STATE_EXPANDED and button == 1 and isDown and AutoDrive.getSetting("useFolders") and AutoDrive.isEditorModeEnabled() and self.type ~= ADPullDownList.TYPE_FILLTYPE then
            -- drag an element
            if hitIndex ~= nil and self.dragged == nil and hitElement ~= nil and not hitElement.isFolder then
                self.dragged = self.selected + (hitIndex - 1)
                self.startedDraggingTimer = 0
                self.draggedElement = hitElement
            end
        elseif isDown == false and isUp == false then
            if hitIndex ~= nil then
                self.hovered = self.selected + (hitIndex - 1)
            end
        end
    end
    return false
end

function ADPullDownList:expand(vehicle)
    local itemCount = self:getItemCount()
    if itemCount > 0 then
        if self.state == ADPullDownList.STATE_COLLAPSED then
            self.layer = self.layer + 1
        end
        self.state = ADPullDownList.STATE_EXPANDED

        AutoDrive.pullDownListExpanded = self.type
        AutoDrive.pullDownListDirection = self.direction

        --possibly adjust height to number of elements (visible)
        self.expandedSize.height = math.min(itemCount + ADPullDownList.MIN_SHOWN, ADPullDownList.MAX_SHOWN) * AutoDrive.Hud.listItemHeight + self.size.height / 2

        if self.direction == ADPullDownList.EXPANDED_UP then
            self.expandedBottom = self.position.y
        else
            self.expandedBottom = self.position.y - self.expandedSize.height + self.size.height
        end
        self.ovTop = g_overlayManager:createOverlay(self.imageBGTop, self.position.x, self.expandedBottom + self.expandedSize.height - self.size.height / 2, self.size.width, self.size.height / 2)
        self.ovStretch = g_overlayManager:createOverlay(self.imageBGStretch, self.position.x, self.expandedBottom + self.size.height / 2, self.size.width, self.expandedSize.height - self.size.height)
        self.ovBottom = g_overlayManager:createOverlay(self.imageBGBottom, self.position.x, self.expandedBottom, self.size.width, self.size.height / 2)
        self:setSelected(vehicle)
    end
end

function ADPullDownList:getHoverEntryReturnValue(vehicle)
    if self.hovered ~= nil then
        local selectedEntry = self:getListElementByIndex(vehicle, self.hovered)
        if selectedEntry ~= nil and selectedEntry.returnValue ~= nil and selectedEntry.isFolder == false then
            return selectedEntry.returnValue
        end
    end
    return nil
end

function ADPullDownList:collapse(vehicle, setItem)
    if self.state == ADPullDownList.STATE_EXPANDED then
        self.layer = self.layer - 1
    end
    self.state = ADPullDownList.STATE_COLLAPSED
    AutoDrive.pullDownListExpanded = 0

    if self.hovered ~= nil and setItem ~= nil and setItem == true then
        local selectedEntry = self:getListElementByIndex(vehicle, self.hovered)
        if selectedEntry ~= nil and selectedEntry.returnValue ~= nil and selectedEntry.isFolder == false then
            if self.type == ADPullDownList.TYPE_TARGET then
                if ADGraphManager:getMapMarkerById(selectedEntry.returnValue) ~= nil then
                    AutoDriveHudInputEventEvent:sendFirstMarkerEvent(vehicle, selectedEntry.returnValue)
                end
            elseif self.type == ADPullDownList.TYPE_UNLOAD then
                if ADGraphManager:getMapMarkerById(selectedEntry.returnValue) ~= nil then
                    AutoDriveHudInputEventEvent:sendSecondMarkerEvent(vehicle, selectedEntry.returnValue)
                end
            elseif self.type == ADPullDownList.TYPE_FILLTYPE then
                -- AutoDrive.debugMsg(vehicle, "ADPullDownList:collapse self.hovered %s selectedEntry.returnValue %s", tostring(self.hovered), tostring(selectedEntry.returnValue))
                AutoDriveHudInputEventEvent:sendFillTypeEvent(vehicle, selectedEntry.returnValue)
            end
        end
    end
    AutoDrive.Hud.lastUIScale = 0
end

function ADPullDownList:setSelected(vehicle)
    self.selected = 1
    self.hovered = 1
    if self.type == ADPullDownList.TYPE_TARGET then
        local index = 1
        local useFolders = AutoDrive.getSetting("useFolders")
        for groupID, entries in pairs(self.options) do
            if useFolders then
                index = index + 1
            end

            if vehicle.ad.groups[self:groupIDToGroupName(self.fakeGroupIDs[groupID])] == true or (not useFolders) then

                for _, entry in pairs(entries) do
                    if entry.returnValue == vehicle.ad.stateModule:getFirstMarkerId() then
                        self.selected = index
                        self.hovered = self.selected
                        if not vehicle.ad.groups[self:groupIDToGroupName(self.fakeGroupIDs[groupID])] then
                            vehicle.ad.groups[self:groupIDToGroupName(self.fakeGroupIDs[groupID])] = true
                        end
                        break
                    end
                    index = index + 1
                end
            end
            if self.selected ~= 1 then
                break
            end
        end
    elseif self.type == ADPullDownList.TYPE_UNLOAD then
        local index = 1
        local useFolders = AutoDrive.getSetting("useFolders")
        for groupID, entries in pairs(self.options) do
            if useFolders then
                index = index + 1
            end
            for _, entry in pairs(entries) do
                if entry.returnValue == vehicle.ad.stateModule:getSecondMarkerId() then
                    self.selected = index
                    self.hovered = self.selected
                    if not vehicle.ad.groups[self:groupIDToGroupName(self.fakeGroupIDs[groupID])] then
                        vehicle.ad.groups[self:groupIDToGroupName(self.fakeGroupIDs[groupID])] = true
                    end
                    break
                end
                index = index + 1
            end

            if self.selected ~= 1 then
                break
            end
        end
    elseif self.type == ADPullDownList.TYPE_FILLTYPE then
        local index = 1
        for _, entries in pairs(self.options) do
            --index = index + 1;
            for _, entry in pairs(entries) do
                if entry.returnValue == vehicle.ad.stateModule:getFillType() then
                    self.selected = index
                    self.hovered = self.selected
                    break
                end
                index = index + 1
            end
            if self.selected ~= 1 then
                break
            end
        end
    end

    local reachedTop = false
    local numberOfElementsVisible = math.min(self:getItemCount(), ADPullDownList.MAX_SHOWN - 2)
    while (not reachedTop) do
        if self:getListElementByIndex(vehicle, self.selected - 1) ~= nil and (self.hovered < (self.selected + numberOfElementsVisible - 1)) then
            self.selected = self.selected - 1
        else
            reachedTop = true
        end
    end
end

function ADPullDownList:groupIDToGroupName(id)
    for groupName, groupId in pairs(ADGraphManager:getGroups()) do
        if groupId == id then
            return groupName
        end
    end
    return nil
end

function ADPullDownList:sortCurrentItems()
    local sort_func = function(a, b)
        a = string.lower(tostring(a.displayName))
        b = string.lower(tostring(b.displayName))
        local patt = "^(.-)%s*(%d+)$"
        local _, _, col1, num1 = a:find(patt)
        local _, _, col2, num2 = b:find(patt)
        if (col1 and col2) and col1 == col2 then
            return tonumber(num1) < tonumber(num2)
        end
        return a < b
    end

    for id, _ in pairs(self.options) do
        table.sort(self.options[id], sort_func)
    end
end

function ADPullDownList:sortFillTypes()
    local sort_func = function(a, b)
        if a.returnValue == AutoDrive.UAL_FILLTYPE_ALL then
            return true
        elseif b.returnValue == AutoDrive.UAL_FILLTYPE_ALL then
            return false
        end
        a = string.lower(tostring(a.displayName))
        b = string.lower(tostring(b.displayName))
        local patt = "^(.-)%s*(%d+)$"
        local _, _, col1, num1 = a:find(patt)
        local _, _, col2, num2 = b:find(patt)
        if (col1 and col2) and col1 == col2 then
            return tonumber(num1) < tonumber(num2)
        end
        return a < b
    end
    for id, _ in pairs(self.options) do
        table.sort(self.options[id], sort_func)
    end
end

function ADPullDownList:getItemCount()
    local count = #self.options
    if AutoDrive.getSetting("useFolders") == false or self.type == ADPullDownList.TYPE_FILLTYPE then
        count = 0
    end
    for _, list in pairs(self.options) do
        count = count + #list
    end
    return count
end

function ADPullDownList:getItemCountForGroup(groupName)
    local groupID = self.groups[groupName]
    if groupID ~= nil and self.options[groupID] ~= nil then
        return #self.options[groupID]
    end
    return 0
end

function ADPullDownList:moveCurrentElementToFolder(vehicle, hitElement)
    local mapMarker = vehicle.ad.stateModule:getFirstMarker()
    local targetGroupName = hitElement.returnValue

    if targetGroupName == ADGraphManager.debugGroupName then
        -- do not drag elements in debug group
        return
    end
    if self.type == ADPullDownList.TYPE_UNLOAD then
        mapMarker = vehicle.ad.stateModule:getSecondMarker()
    end

    if mapMarker ~= nil then
        for _, entries in pairs(self.options) do
            for i, entry in pairs(entries) do
                if entry.returnValue == mapMarker.markerIndex then
                    table.remove(entries, i)
                end
            end
        end

        table.insert(self.options[self.groups[targetGroupName]], {displayName = mapMarker.name, returnValue = mapMarker.markerIndex})

        ADGraphManager:changeMapMarkerGroup(targetGroupName, mapMarker.markerIndex)

        self:sortCurrentItems()
    end
end

function ADPullDownList:sortDraggedInGroup(draggedElement, hitElement)
    local targetGroupName
    if hitElement.isFolder then
        targetGroupName = hitElement.returnValue
    else
        targetGroupName = ADGraphManager:getMapMarkerById(hitElement.returnValue).group
    end
    if targetGroupName == ADGraphManager.debugGroupName then
        -- do not drag elements in debug group
        return
    end
    for _, entries in pairs(self.options) do
        for i, entry in pairs(entries) do
            if entry.returnValue == draggedElement.returnValue then
                table.remove(entries, i)
            end
        end
    end

    table.insert(self.options[self.groups[targetGroupName]], {displayName = draggedElement.displayName, returnValue = draggedElement.returnValue})

    ADGraphManager:changeMapMarkerGroup(targetGroupName, draggedElement.returnValue)

    self:sortCurrentItems()
end
