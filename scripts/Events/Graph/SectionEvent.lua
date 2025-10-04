AutoDriveSectionEvent = {}
AutoDriveSectionEvent_mt = Class(AutoDriveSectionEvent, Event)

AutoDriveSectionEvent.OPERATION_NONE = 0
AutoDriveSectionEvent.OPERATION_CONNECTION_DIRECTION = 1
AutoDriveSectionEvent.OPERATION_CONNECTION_FLAGS = 2
AutoDriveSectionEvent.OPERATION_CONNECTION_DELETE = 3

InitEventClass(AutoDriveSectionEvent, "AutoDriveSectionEvent")

function AutoDriveSectionEvent.emptyNew()
    local self = Event.new(AutoDriveSectionEvent_mt)
    return self
end

function AutoDriveSectionEvent.new(operation, startNodeId, endNodeId, arg1)
    local self = AutoDriveSectionEvent.emptyNew()
    self.operation = operation or AutoDriveSectionEvent.OPERATION_NONE
    self.startNodeId = startNodeId or 0
    self.endNodeId = endNodeId or 0
    self.arg1 = arg1 or 0
    return self
end

function AutoDriveSectionEvent:writeStream(streamId, connection)
    streamWriteUInt8(streamId, self.operation)
    streamWriteUIntN(streamId, self.startNodeId, 20)
    streamWriteUIntN(streamId, self.endNodeId, 20)
    streamWriteInt32(streamId, self.arg1)
end

function AutoDriveSectionEvent:readStream(streamId, connection)
    self.operation = streamReadUInt8(streamId)
    self.startNodeId = streamReadUIntN(streamId, 20)
    self.endNodeId = streamReadUIntN(streamId, 20)
    self.arg1 = streamReadInt32(streamId)
    self:run(connection)
end

function AutoDriveSectionEvent:run(connection)
    if g_server ~= nil and connection:getIsServer() == false then
        -- If the event is coming from a client, server have only to broadcast
        AutoDriveSectionEvent.sendEvent(self.operation, self.startNodeId, self.endNodeId, self.arg1)
    else
        -- ADGraphManager:setConnectionBetween(self.operation, self.startNode, self.endNode, false)
        if self.operation == AutoDriveSectionEvent.OPERATION_CONNECTION_DIRECTION then
            ADGraphManager:setConnectionBetweenWayPointsInSection(self.startNodeId, self.endNodeId, self.arg1, false)
        elseif self.operation == AutoDriveSectionEvent.OPERATION_CONNECTION_FLAGS then
            ADGraphManager:setWayPointsFlagsInSection(self.startNodeId, self.endNodeId, self.arg1, false)
        elseif self.operation == AutoDriveSectionEvent.OPERATION_CONNECTION_DELETE then
            ADGraphManager:deleteWayPointsInSection(self.startNodeId, self.endNodeId, false)
        end
    end
end

function AutoDriveSectionEvent.sendEvent(operation, startNodeId, endNodeId, arg1)
    local event = AutoDriveSectionEvent.new(operation, startNodeId, endNodeId, arg1)
    if g_server ~= nil then
        -- Server have to broadcast to all clients and himself
        g_server:broadcastEvent(event, true)
    else
        -- Client have to send to server
        g_client:getServerConnection():sendEvent(event)
    end
end
