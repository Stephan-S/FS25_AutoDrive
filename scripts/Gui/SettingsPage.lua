--
-- AutoDrive GUI
-- V1.0.0.0
--
-- @author Stephan Schlosser
-- @date 08/04/2019

ADSettingsPage = {}
ADSettingsPage.debug = false

local ADSettingsPage_mt = Class(ADSettingsPage, TabbedMenuFrameElement)

function ADSettingsPage.new(target)
    local self = TabbedMenuFrameElement.new(target, ADSettingsPage_mt)
    self:include(ADGuiDebugMixin)
    self.pageElements = {}
    self.settingElements = {}
    self.isEvenRow = false
    return self
end

function ADSettingsPage:onFrameOpen()
    self:debugMsg("ADSettingsPage[%s]:onFrameOpen", self.name)
    ADSettingsPage:superClass().onFrameOpen(self)
    if not self:hasChanges() then
        self:loadGUISettings()
    end
    FocusManager:setFocus(self.boxLayout)
end

--[[
function ADSettingsPage:onGuiSetupFinished() -- texts will be set in loadGUISetting
    self:debugMsg("ADSettingsPage[%s]:onGuiSetupFinished", self.name)
    ADSettingsPage:superClass().onGuiSetupFinished(self)

    -- set up element text options, BinaryOption does not allow doing this in onCreate.
    for _, element in pairs(self.settingElements) do
        local setting = AutoDrive.settings[element.name]
        local labels = {}
        for i = 1, #setting.texts, 1 do
            if setting.translate == true then
                local text = g_i18n:getText(setting.texts[i])
                local missingText = "Missing"
                if text:sub(1, string.len(missingText)) == missingText then
                    labels[i] = setting.texts[i]
                else
                    labels[i] = text
                end
            else
                labels[i] = setting.texts[i]
            end
        end
        element:setTexts(labels)
    end
end
]]

function ADSettingsPage:onCreateAutoDriveSettingRow(element)
    self:debugMsg("ADSettingsPage[%s]:onCreateAutoDriveSettingRow, isEvenRow=%s", self.name, tostring(self.isEvenRow))
    element:setImageColor(nil, table.unpack(InGameMenuSettingsFrame.COLOR_ALTERNATING[self.isEvenRow]))
    self.isEvenRow = not self.isEvenRow
end

function ADSettingsPage:onCreateAutoDriveSetting(element)
    self:debugMsg("ADSettingsPage[%s]:onCreateAutoDriveSetting name=%s", self.name, element.name)
    self.pageElements[element.name] = element
end

function ADSettingsPage:onOptionChange(state, element)
    self:debugMsg("ADSettingsPage[%s]:onOptionChange, name=%s, state=%s", self.name, element.name, tostring(state))

    local controlledVehicle = AutoDrive.getControlledVehicle()
    local settingName = element.name
    local tempSettingName = settingName .. "AL"
    if controlledVehicle and controlledVehicle.ad and controlledVehicle.ad.hasAL and AutoDrive.settings[tempSettingName] then
        -- change to AL
        settingName = tempSettingName
    end

    local setting = AutoDrive.settings[settingName]
    if setting.isVehicleSpecific and controlledVehicle ~= nil and controlledVehicle.ad ~= nil and controlledVehicle.ad.settings[settingName] ~= nil then
        setting = controlledVehicle.ad.settings[settingName]
    end
    self:debugMsg("ADSettingsPage[%s]:onOptionChange hasAL %s settingName %s", self.name, tostring(controlledVehicle and controlledVehicle.ad and controlledVehicle.ad.hasAL), tostring(settingName))
    setting.new = state
end

function ADSettingsPage:hasChanges()
    self:debugMsg("ADSettingsPage[%s]:hasChanges start", self.name)
    local controlledVehicle = AutoDrive.getControlledVehicle()
    for settingName, _ in pairs(self.settingElements) do
        if AutoDrive.settings[settingName] ~= nil then
            local setting = AutoDrive.settings[settingName]
            if setting.isVehicleSpecific and controlledVehicle ~= nil and controlledVehicle.ad ~= nil and controlledVehicle.ad.settings[settingName] ~= nil then
                setting = controlledVehicle.ad.settings[settingName]
            end
            if setting.new ~= nil and setting.new ~= setting.current then
                self:debugMsg("ADSettingsPage[%s]:hasChanges = true (%s)", self.name, settingName)
                return true
            end
        end
    end
    self:debugMsg("ADSettingsPage[%s]:hasChanges = false", self.name)
    return false
end

function ADSettingsPage:loadGUISettings()
    local controlledVehicle = AutoDrive.getControlledVehicle()
    self:debugMsg("ADSettingsPage[%s]:loadGUISettings hasAL %s", self.name, tostring(controlledVehicle and controlledVehicle.ad and controlledVehicle.ad.hasAL))
    self.settingElements = {}
    for settingName, element in pairs(self.pageElements) do
        local tempSettingName = settingName .. "AL"
        if controlledVehicle and controlledVehicle.ad and controlledVehicle.ad.hasAL and AutoDrive.settings[tempSettingName] then
            -- change to AL
            self.settingElements[tempSettingName] = element
        else
            self.settingElements[settingName] = element
        end
    end

    for settingName, _ in pairs(self.settingElements) do
        if AutoDrive.settings[settingName] ~= nil then
            local setting = AutoDrive.settings[settingName]
            if setting.isVehicleSpecific and controlledVehicle ~= nil and controlledVehicle.ad ~= nil and controlledVehicle.ad.settings[settingName] ~= nil then
                setting = controlledVehicle.ad.settings[settingName]
            end
            self:debugMsg("ADSettingsPage[%s]:loadGUISettings settingName %s current %s", self.name, tostring(settingName), tostring(settingName.current))
            self:loadGUISetting(settingName, setting.current)
        end
    end
end

function ADSettingsPage:loadGUISetting(settingName, state)
    self:debugMsg("ADSettingsPage[%s]:loadGUISetting start", self.name)
    local element = self.settingElements[settingName]
    local setting = AutoDrive.settings[settingName]
    local labels = {}
    if setting.texts == nil or (setting.texts and #setting.texts == 0) then
        for i = 1, #setting.values, 1 do
            if setting.values[i] ~= nil then
                labels[i] = string.format("%s", setting.values[i])
            end
        end
    else
        for i = 1, #setting.texts, 1 do
            if setting.translate == true then
                local text = g_i18n:getText(setting.texts[i])
                local missingText = "Missing"
                if text:sub(1, string.len(missingText)) == missingText then
                    labels[i] = setting.texts[i]
                else
                    labels[i] = text
                end
            else
                labels[i] = setting.texts[i]
            end
        end
    end
    element:setTexts(labels)
    element:setState(state, false)
    self:onOptionChange(state, element)
end

