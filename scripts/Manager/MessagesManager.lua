ADMessagesManager = {}
ADMessagesManager.messageTypes = {}
ADMessagesManager.messageTypes.INFO = 1
ADMessagesManager.messageTypes.WARN = 2
ADMessagesManager.messageTypes.ERROR = 3

ADMessagesManager.messages = {}
ADMessagesManager.currentMessage = nil
ADMessagesManager.currentMessageTimer = 0

ADMessagesManager.notifications = {}
ADMessagesManager.currentNotification = nil
ADMessagesManager.currentNotificationTimer = 0

ADMessagesManager.history = {}

ADMessagesManager.lastNotificationVehicle = nil

ADMessagesManager.huds = {}
ADMessagesManager.huds.message = {}
ADMessagesManager.huds.message.text = ""
ADMessagesManager.huds.message.textSize = 0.0135
ADMessagesManager.huds.message.posX = 0.5
ADMessagesManager.huds.message.posY = 0.92
ADMessagesManager.huds.message.headerOverlay = 0
ADMessagesManager.huds.message.headerOverlayHeight = 0.01
ADMessagesManager.huds.message.backgroundOverlay = 0
ADMessagesManager.huds.message.backgroundOverlayHeight = 0.06
ADMessagesManager.huds.message.dismissOverlay = 0
ADMessagesManager.huds.message.infoIconOverlay = 0
ADMessagesManager.huds.message.errorIconOverlay = 0
ADMessagesManager.huds.message.warnIconOverlay = 0

ADMessagesManager.huds.notification = {}
ADMessagesManager.huds.notification.text = ""
ADMessagesManager.huds.notification.textSize = 0.0135
ADMessagesManager.huds.notification.posX = 0.5
ADMessagesManager.huds.notification.posY = 0.92
ADMessagesManager.huds.notification.headerOverlay = 0
ADMessagesManager.huds.notification.headerOverlayHeight = 0.01
ADMessagesManager.huds.notification.backgroundOverlay = 0
ADMessagesManager.huds.notification.backgroundOverlayHeight = 0.06
ADMessagesManager.huds.notification.dismissOverlay = 0
ADMessagesManager.huds.notification.goToOverlay = 0
ADMessagesManager.huds.message.infoIconOverlay = 0
ADMessagesManager.huds.message.errorIconOverlay = 0
ADMessagesManager.huds.message.warnIconOverlay = 0

function ADMessagesManager:load()
    self.messages = Queue:new()
    self.notifications = Queue:new()
    self:loadHud(self.huds.message)
    self:loadHud(self.huds.notification)
end

function ADMessagesManager:loadHud(hud)
    local textSize = getCorrectTextSize(hud.textSize)
    hud.headerOverlay = g_overlayManager:createOverlay("ad_gui.Header_message", hud.posX, hud.posY + (textSize * 1.6), 0, hud.headerOverlayHeight)
    hud.headerOverlay:setAlignment(Overlay.ALIGN_VERTICAL_BOTTOM, Overlay.ALIGN_HORIZONTAL_CENTER)
    hud.backgroundOverlay = g_overlayManager:createOverlay("ad_gui.messageBackground", hud.posX, hud.posY, 0, hud.backgroundOverlayHeight)
    hud.backgroundOverlay:setAlignment(Overlay.ALIGN_VERTICAL_MIDDLE, Overlay.ALIGN_HORIZONTAL_CENTER)
    hud.dismissOverlay = g_overlayManager:createOverlay("ad_gui.input_toggleHud", 0, hud.posY - (textSize), hud.headerOverlayHeight * 2 / g_screenAspectRatio, hud.headerOverlayHeight * 2)
    hud.dismissOverlay:setAlignment(Overlay.ALIGN_VERTICAL_MIDDLE, Overlay.ALIGN_HORIZONTAL_CENTER)
    if hud.goToOverlay ~= nil then
        hud.goToOverlay = g_overlayManager:createOverlay("ad_gui.input_goTo_1", 0, hud.posY - (textSize), hud.headerOverlayHeight * 2 / g_screenAspectRatio, hud.headerOverlayHeight * 2)
        hud.goToOverlay:setAlignment(Overlay.ALIGN_VERTICAL_MIDDLE, Overlay.ALIGN_HORIZONTAL_CENTER)
    end
    hud.infoIconOverlay = g_overlayManager:createOverlay("ad_gui.info_icon", 0, hud.posY + (textSize * 1.5), hud.headerOverlayHeight * 1.2 / g_screenAspectRatio, hud.headerOverlayHeight * 1.2)
    hud.infoIconOverlay:setAlignment(Overlay.ALIGN_VERTICAL_BOTTOM, Overlay.ALIGN_HORIZONTAL_LEFT)
    hud.errorIconOverlay = g_overlayManager:createOverlay("ad_gui.error_icon", 0, hud.posY + (textSize * 1.5), hud.headerOverlayHeight * 1.2 / g_screenAspectRatio, hud.headerOverlayHeight * 1.2)
    hud.errorIconOverlay:setAlignment(Overlay.ALIGN_VERTICAL_BOTTOM, Overlay.ALIGN_HORIZONTAL_LEFT)
    hud.warnIconOverlay = g_overlayManager:createOverlay("ad_gui.warn_icon", 0, hud.posY + (textSize * 1.5), hud.headerOverlayHeight * 1.2 / g_screenAspectRatio, hud.headerOverlayHeight * 1.2)
    hud.warnIconOverlay:setAlignment(Overlay.ALIGN_VERTICAL_BOTTOM, Overlay.ALIGN_HORIZONTAL_LEFT)
end

function ADMessagesManager:addInfoMessage(vehicle, text, duration) -- not used
    self:addMessage(vehicle, self.messageTypes.INFO, text, duration)
end

function ADMessagesManager:addWarnMessage(vehicle, text, duration) -- not used
    self:addMessage(vehicle, self.messageTypes.WARN, text, duration)
end

function ADMessagesManager:addErrorMessage(vehicle, text, duration) -- not used
    self:addMessage(vehicle, self.messageTypes.ERROR, text, duration)
end

function ADMessagesManager:addMessage(vehicle, messageType, text, duration)
    local exists = false
    if self.currentMessage ~= nil and self.currentMessage.messageType == messageType and self.currentMessage.text == text then
        exists = true
    end
    exists =
        exists or
        table.f_contains(
            self.messages:GetItems(),
            function(i)
                return i.messageType == messageType and i.text == text
            end
        )
    if not exists then
        self.messages:Enqueue({vehicle = AutoDrive.getControlledVehicle(), messageType = messageType, text = text, duration = duration})
    end
end

function ADMessagesManager:addNotification(vehicle, messageType, text, duration)
    if vehicle and vehicle == AutoDrive.getControlledVehicle() then
        self:addMessage(vehicle, messageType, text, duration)
    elseif vehicle and vehicle.ad and vehicle.ad.stateModule and vehicle.ad.stateModule:getActualFarmId() == g_localPlayer:getFarmId() and not AutoDrive:getIsEntered(vehicle) then
        local exists = false
        if self.currentNotification ~= nil and self.currentNotification.messageType == messageType and self.currentNotification.text == text and self.currentNotification.vehicle == vehicle then
            exists = true
        end
        exists =
            exists or
            table.f_contains(
                self.notifications:GetItems(),
                function(i)
                    return i.messageType == messageType and i.text == text and i.vehicle == vehicle
                end
            )
        if not exists then
            self.notifications:Enqueue({vehicle = vehicle, messageType = messageType, text = text, duration = duration})
        end
    end
end

function ADMessagesManager:removeCurrentMessage()
    -- self:addToHistory(self.currentMessage)
    self.currentMessage = nil
    self.currentMessageTimer = 0
end

function ADMessagesManager:removeCurrentNotification()
    self:addToHistory(self.currentNotification)
    self.currentNotification = nil
    self.currentNotificationTimer = 0
end

function ADMessagesManager:addToHistory(item)
    table.insert(self.history, 1, item)
    if AutoDrive.gui and AutoDrive.gui.ADNotificationsHistoryGui  and AutoDrive.gui.ADNotificationsHistoryGui.isOpen then
        AutoDrive.gui.ADNotificationsHistoryGui:refreshItems()
    end
end

function ADMessagesManager:removeFromHistory(index)
    table.remove(self.history, index)
    if AutoDrive.gui and AutoDrive.gui.ADNotificationsHistoryGui  and AutoDrive.gui.ADNotificationsHistoryGui.isOpen then
        AutoDrive.gui.ADNotificationsHistoryGui:refreshItems()
    end
end

function ADMessagesManager:clearHistory()
    self.history = {}
    if AutoDrive.gui and AutoDrive.gui.ADNotificationsHistoryGui  and AutoDrive.gui.ADNotificationsHistoryGui.isOpen then
        AutoDrive.gui.ADNotificationsHistoryGui:refreshItems()
    end
end

function ADMessagesManager:getHistory()
    return self.history
end

function ADMessagesManager:update(dt)
    -- messages handling
    if self.currentMessage == nil then
        self.currentMessage = self.messages:Dequeue()
        if self.currentMessage ~= nil then
            local nd = AutoDrive.getSetting("notifications")
            if nd ~= 0 then   
                if self.currentMessage.messageType == ADMessagesManager.messageTypes.ERROR then    
                    AutoDrive.playSample(AutoDrive.notificationWarningSample, 0.4)
                else            
                    AutoDrive.playSample(AutoDrive.notificationSample, 0.4)
                end
                self.currentMessage.duration = self.currentMessage.duration * nd
                self:updateHud(self.huds.message, self.currentMessage.text, self.currentMessage.messageType)
            else
                self:removeCurrentMessage()
            end
        end
    else
        self.currentMessageTimer = self.currentMessageTimer + dt
        -- if we have more messages in queue we decrease their lifespan
        local lifeSpan = self.currentMessage.duration
        if self.messages:Count() > 0 then
            lifeSpan = lifeSpan / 2
        end
        if self.currentMessageTimer >= lifeSpan then
            self:removeCurrentMessage()
        end
    end

    -- notifications handling
    if self.currentNotification == nil then
        self.currentNotification = self.notifications:Dequeue()
        if self.currentNotification ~= nil then
            self.lastNotificationVehicle = self.currentNotification.vehicle
            local nd = AutoDrive.getSetting("notifications")
            if nd ~= 0 then
                if self.currentNotification.messageType == ADMessagesManager.messageTypes.ERROR then
                    AutoDrive.playSample(AutoDrive.notificationWarningSample, 0.9)
                else         
                    AutoDrive.playSample(AutoDrive.notificationSample, 0.9)
                end
                self.currentNotification.duration = self.currentNotification.duration * nd
                self:updateHud(self.huds.notification, self.currentNotification.text, self.currentNotification.messageType)
            else
                self:removeCurrentNotification()
            end
        end
    else
        self.currentNotificationTimer = self.currentNotificationTimer + dt
        if self.currentNotificationTimer >= self.currentNotification.duration then
            self:removeCurrentNotification()
        end
    end
end

function ADMessagesManager:mouseEvent(posX, posY, isDown, isUp, button)
    if isUp and button == 1 then
        if self.currentNotification ~= nil then
            -- notification prefered as it allow to change to vehicle
            local ov = self.huds.notification.dismissOverlay
            local x, y = ov:getPosition()
            if posX >= x - ov.width / 2 and posY >= y - ov.height / 2 and posX <= x + ov.width / 2 and posY <= y + ov.height / 2 then
                self:removeCurrentNotification()
            end
            ov = self.huds.notification.goToOverlay
            x, y = ov:getPosition()
            if posX >= x - ov.width / 2 and posY >= y - ov.height / 2 and posX <= x + ov.width / 2 and posY <= y + ov.height / 2 then
                self:goToVehicle()
                self:removeCurrentNotification()
            end
        elseif self.currentMessage ~= nil then
            local ov = self.huds.message.dismissOverlay
            local x, y = ov:getPosition()
            if posX >= x - ov.width / 2 and posY >= y - ov.height / 2 and posX <= x + ov.width / 2 and posY <= y + ov.height / 2 then
                self:removeCurrentMessage()
            end
        end
    end
end

function ADMessagesManager:draw()
    if self.currentNotification ~= nil then
        -- notification prefered as it allow to change to vehicle
        self:drawHud(self.huds.notification)
    elseif self.currentMessage ~= nil then
        self:drawHud(self.huds.message)
    end
end

function ADMessagesManager:drawHud(hud)
    setTextAlignment(RenderText.ALIGN_CENTER)
    setTextBold(false)
    setTextColor(unpack(AutoDrive.currentColors.ad_color_hudTextDefault))
    renderText(hud.posX, hud.posY, getCorrectTextSize(hud.textSize), hud.text)
    hud.backgroundOverlay:render()
    hud.headerOverlay:render()
    hud.dismissOverlay:render()
    if hud.goToOverlay ~= nil then
        hud.goToOverlay:render()
    end
    hud.infoIconOverlay:render()
    hud.warnIconOverlay:render()
    hud.errorIconOverlay:render()
end

function ADMessagesManager:updateHud(hud, text, mType)
    hud.text = text
    setTextBold(false)
    local textWidth = getTextWidth(getCorrectTextSize(hud.textSize), hud.text)
    hud.backgroundOverlay:setDimension(textWidth + 0.03, nil)
    hud.headerOverlay:setDimension(textWidth + 0.03, nil)
    hud.dismissOverlay:setPosition(hud.posX, nil)
    if hud.goToOverlay ~= nil then
        hud.goToOverlay:setPosition(hud.posX - (hud.goToOverlay.width * 2), nil)
        hud.dismissOverlay:setPosition(hud.posX + (hud.dismissOverlay.width * 2), nil)
    end
    hud.infoIconOverlay:setPosition(hud.posX - ((textWidth + 0.03) / 2), nil)
    hud.infoIconOverlay:setIsVisible(mType == self.messageTypes.INFO)
    hud.warnIconOverlay:setPosition(hud.posX - ((textWidth + 0.03) / 2), nil)
    hud.warnIconOverlay:setIsVisible(mType == self.messageTypes.WARN)
    hud.errorIconOverlay:setPosition(hud.posX - ((textWidth + 0.03) / 2), nil)
    hud.errorIconOverlay:setIsVisible(mType == self.messageTypes.ERROR)
end

function ADMessagesManager:goToVehicle()
    if self.lastNotificationVehicle ~= nil then
        AutoDrive.requestToEnterVehicle(self.lastNotificationVehicle)
    end
end