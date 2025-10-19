AutoDriveMessageEvent = {}
AutoDriveMessageEvent_mt = Class(AutoDriveMessageEvent, Event)

InitEventClass(AutoDriveMessageEvent, "AutoDriveMessageEvent")

function AutoDriveMessageEvent.emptyNew()
    local self = Event.new(AutoDriveMessageEvent_mt)
    return self
end

function AutoDriveMessageEvent.new(vehicle, isNotification, messageType, text, duration, args)
    local self = AutoDriveMessageEvent.emptyNew()
    self.vehicle = vehicle
    self.isNotification = isNotification
    self.messageType = messageType
    self.text = text
    self.duration = duration
    self.args = args
    return self
end

function AutoDriveMessageEvent:writeStream(streamId, connection)
    streamWriteInt32(streamId, NetworkUtil.getObjectId(self.vehicle))
    streamWriteBool(streamId, self.isNotification)
    streamWriteUIntN(streamId, self.messageType, 4)
    streamWriteString(streamId, self.text or "")
    streamWriteUIntN(streamId, self.duration, 16)
    local argsCount = #self.args
    streamWriteUIntN(streamId, argsCount, 5)
    for i = 1, argsCount do
        streamWriteString(streamId, tostring(self.args[i]))
    end
end

function AutoDriveMessageEvent:readStream(streamId, connection)
    self.vehicle = NetworkUtil.getObject(streamReadInt32(streamId))
    self.isNotification = streamReadBool(streamId)
    self.messageType = streamReadUIntN(streamId, 4)
    self.text = streamReadString(streamId)
    self.duration = streamReadUIntN(streamId, 16)
    self.args = {}
    local argsCount = streamReadUIntN(streamId, 5)
    for i = 1, argsCount do
        self.args[i] = streamReadString(streamId)
    end
    self:run(connection)
end

function AutoDriveMessageEvent:run(connection)
    if g_server ~= nil and connection:getIsServer() == false then
        -- If the event is coming from a client, server have only to broadcast
        if self.isNotification then
            AutoDriveMessageEvent.sendNotification(self.messageType, self.text, self.duration, unpack(self.args))
        end
    else
        -- If the event is coming from the server
        if g_dedicatedServer == nil then
            -- localization
            self.text = AutoDrive.localize(self.text)

            -- formatting
            if #self.args > 0 then
                self.text = string.format(self.text, unpack(self.args))
            end

            if not self.isNotification then
                ADMessagesManager:addMessage(self.vehicle, self.messageType, self.text, self.duration)
            else
                ADMessagesManager:addNotification(self.vehicle, self.messageType, self.text, self.duration)
            end
        end
    end
end

-- this will send a message only to the player who's driving the vehicle (if there is one)
function AutoDriveMessageEvent.sendMessage(vehicle, messageType, text, duration, ...)
    if g_server ~= nil then
        -- Server have to send only to owner
        if vehicle.ownerConnection ~= nil then
            vehicle.ownerConnection:sendEvent(AutoDriveMessageEvent.new(vehicle, false, messageType, text, duration, {...}))
        end
    else
        Logging.error("A client is trying to send a message event.")
        printCallstack()
    end
end

-- this will send a notification to all players
function AutoDriveMessageEvent.sendNotification(vehicle, messageType, text, duration, ...)
    if g_server ~= nil then
        -- Server have to broadcast to all clients and himself
        g_server:broadcastEvent(AutoDriveMessageEvent.new(vehicle, true, messageType, text, duration, {...}), true)
    else
        -- Client have to send to server
        --g_client:getServerConnection():sendEvent(event)
        Logging.error("A client is trying to send a notification event.")
        printCallstack()
    end
end

-- this will send a message only to the player who's driving the vehicle if there is one, otherwise it will send a notification to everyone
function AutoDriveMessageEvent.sendMessageOrNotification(vehicle, messageType, text, duration, ...)
    if g_server ~= nil then
        -- Server have to send notification
        AutoDriveMessageEvent.sendNotification(vehicle, messageType, text, duration, ...)
    else
        Logging.error("A client is trying to send a message or notification event.")
        printCallstack()
    end
end
