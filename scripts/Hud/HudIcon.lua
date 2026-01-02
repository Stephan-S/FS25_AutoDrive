ADHudIcon = ADInheritsFrom(ADGenericHudElement)

function ADHudIcon:new(posX, posY, width, height, image, layer, name)
    local o = ADHudIcon:create()
    o:init(posX, posY, width, height)
    o.layer = layer
    o.name = name
    o.image = image
    o.isVisible = true
    o.lastLineCount = 1
    o.ov = Overlay.new(g_baseUIFilename, o.position.x, o.position.y, width, height)
    o.ov:setUVs(g_colorBgUVs)
    return o
end

function ADHudIcon:onDraw(vehicle, uiScale)
    if self.name == "header" then
        self:onDrawHeader(vehicle, uiScale)
    end

    if self.isVisible and self.ov ~= nil then
        if self.name == "header" then
            self.ov:setColor(unpack(AutoDrive.currentColors.ad_color_header))
        elseif self.name == "background" then
            self.ov:setColor(unpack(AutoDrive.currentColors.ad_color_background))
        end
        self.ov:render()
    end
end

function ADHudIcon:onDrawHeader(vehicle, uiScale)
    local adFontSize = 0.011 * uiScale
    local textHeight = getTextHeight(adFontSize, "text")
    local adPosX = self.position.x + AutoDrive.Hud.gapWidth
    local adPosY = self.position.y + (self.size.height - textHeight) / 2

    if AutoDrive.Hud.isShowingTips or AutoDrive.Hud.isEditingHud then
        adPosY = self.position.y + (AutoDrive.Hud.gapHeight)
    end

    setTextBold(true)
    setTextColor(unpack(AutoDrive.currentColors.ad_color_hudTextDefault))
    setTextAlignment(RenderText.ALIGN_LEFT)
    self:renderDefaultText(vehicle, adFontSize, adPosX, adPosY)
    if AutoDrive.Hud.isShowingTips then
        adPosY = adPosY + (textHeight + AutoDrive.Hud.gapHeight) * self.lastLineCount
        self:renderEditorTips(textHeight, adFontSize, adPosX, adPosY)
    end
    if AutoDrive.Hud.isEditingHud then
        adPosY = adPosY + (textHeight + AutoDrive.Hud.gapHeight) * self.lastLineCount
        self:renderHudEditorTips(textHeight, adFontSize, adPosX, adPosY)
    end

end

function ADHudIcon:updateTooltip(vehicle)
end

function ADHudIcon:renderDefaultText(vehicle, fontSize, posX, posY)
    local textHeight = getTextHeight(fontSize, "text")
    local textToShow = "AutoDrive"
    textToShow = textToShow .. " - " .. AutoDrive.version
    textToShow = textToShow .. " - " .. AutoDriveHud:getModeName(vehicle)
    textToShow = self:addVehicleDriveTimeString(vehicle, textToShow)
    if vehicle.ad.toolTipOnNewLine ~= true then
        textToShow = self:addTooltipString(vehicle, textToShow)
    end

    local taskInfo = vehicle.ad.stateModule:getCurrentLocalizedTaskInfo()
    if taskInfo ~= "" then
        textToShow = textToShow .. " - " .. taskInfo
    end

    if AutoDrive.isEditorModeEnabled() and AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
        if vehicle.ad.pathFinderModule.steps > 0 then
            textToShow = textToShow .. " - " .. "Fallback: " .. tostring(vehicle.ad.pathFinderModule.fallBackMode)
        end
    end
    local lines = self:splitTextByLength(textToShow, fontSize, AutoDrive.Hud.headerLabelWidth)
    if vehicle.ad.toolTipOnNewLine == true then
        local toolTipText = self:addTooltipString(vehicle, "")
        if toolTipText ~= "" then
            table.insert(lines, string.sub(toolTipText, 4))
        end
    end

    if #lines ~= self.lastLineCount and self.ov ~= nil then
        self.ov:setDimension(nil, self.size.height + (textHeight + AutoDrive.Hud.gapHeight) * (#lines - 1))        
    end

    for lineNumber, lineText in pairs(lines) do
        if AutoDrive.pullDownListExpanded == 0 then
            renderText(posX, posY, fontSize, lineText)
            posY = posY + textHeight + AutoDrive.Hud.gapHeight
        end
    end
    self.lastLineCount = #lines
end

function ADHudIcon:renderEditorTips(textHeight, fontSize, posX, posY)
    local editorTips = {
        g_i18n:getText("gui_ad_editorTip_11"),
        g_i18n:getText("gui_ad_editorTip_10"),
        g_i18n:getText("gui_ad_editorTip_9"),
        g_i18n:getText("gui_ad_editorTip_8"),
        g_i18n:getText("gui_ad_editorTip_7"),
        g_i18n:getText("gui_ad_editorTip_6"),
        g_i18n:getText("gui_ad_editorTip_5"),
        g_i18n:getText("gui_ad_editorTip_4"),
        g_i18n:getText("gui_ad_editorTip_3"),
        g_i18n:getText("gui_ad_editorTip_2"),
        g_i18n:getText("gui_ad_editorTip_1")
    }

    local gap = textHeight / 2
    posY = posY + gap  -- gap below tips

    for tipId, tip in ipairs(editorTips) do
        if AutoDrive.pullDownListExpanded == 0 then
            renderText(posX, posY, fontSize, tip)
            posY = posY + textHeight + AutoDrive.Hud.gapHeight
            if tipId == 3 or tipId == 6 then
                posY = posY + gap
            end
        end
    end
end

function ADHudIcon:renderHudEditorTips(textHeight, fontSize, posX, posY)
    local editorTips = {
        g_i18n:getText("gui_ad_hudEditorTip_6"),
        g_i18n:getText("gui_ad_hudEditorTip_5"),
        g_i18n:getText("gui_ad_hudEditorTip_4"),
        g_i18n:getText("gui_ad_hudEditorTip_3"),
        g_i18n:getText("gui_ad_hudEditorTip_2"),
        g_i18n:getText("gui_ad_hudEditorTip_1"),
    }

    local gap = textHeight / 2
    posY = posY + gap  -- gap below tips

    for tipId, tip in ipairs(editorTips) do
        if AutoDrive.pullDownListExpanded == 0 then
            renderText(posX, posY, fontSize, tip)
            posY = posY + textHeight + AutoDrive.Hud.gapHeight
            if tipId == 1 or tipId == 4 then
                posY = posY + gap
            end
        end
    end
end

function ADHudIcon:addVehicleDriveTimeString(vehicle, currentText)
    local remainingTime = vehicle.ad.stateModule:getRemainingDriveTime()
    if remainingTime ~= 0 then
        local remainingMinutes = math.floor(remainingTime / 60)
        local remainingSeconds = remainingTime % 60
        if remainingMinutes > 0 then
            currentText = currentText .. " - " .. string.format("%.0f", remainingMinutes) .. ":" .. string.format("%02d", math.floor(remainingSeconds))
        elseif remainingSeconds ~= 0 then
            currentText = currentText .. " - " .. string.format("%2.0f", remainingSeconds) .. "s"
        end
    end
    return currentText
end

function ADHudIcon:addTooltipString(vehicle, currentText)
    if vehicle.ad.sToolTip ~= "" and AutoDrive.getSetting("showTooltips") then
        if vehicle.ad.toolTipIsSetting then
            currentText = currentText .. " - " .. g_i18n:getText(vehicle.ad.sToolTip)
        else
            currentText = currentText .. " - " .. string.sub(g_i18n:getText(vehicle.ad.sToolTip), 5)
        end

        if vehicle.ad.sToolTipInfo ~= nil then
            currentText = currentText .. " - " .. vehicle.ad.sToolTipInfo
        end
    end
    return currentText
end

function ADHudIcon:splitTextByLength(text, fontSize, maxLength)
    local lines = {}
    local textParts = string.split(text, "-")
    local line = textParts[1]
    local index = 2
    while index <= #textParts do
        if getTextWidth(fontSize, line .. "-" .. textParts[index]) > maxLength then
            table.insert(lines, line)
            line = textParts[index]:sub(2)
        else
            line = line .. "-" .. textParts[index]
        end
        index = index + 1
    end
    table.insert(lines, line)
    return lines
end

    
function ADHudIcon:act(vehicle, posX, posY, isDown, isUp, button)
    if self.name == "header" then
        if button == 1 and isDown and AutoDrive.pullDownListExpanded == 0 then
            if (not g_inGameMenu.isOpen or AutoDrive.aiFrameOpen) then
                AutoDrive.Hud:startMovingHud(posX, posY)
                return true
            end
        end
    end
    return false
end
