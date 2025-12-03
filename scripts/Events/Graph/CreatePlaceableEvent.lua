CreatePlaceableEvent = {}
CreatePlaceableEvent_mt = Class(CreatePlaceableEvent, Event)

InitEventClass(CreatePlaceableEvent, "CreatePlaceableEvent")

function CreatePlaceableEvent.emptyNew()
    local self = Event.new(CreatePlaceableEvent_mt)
    return self
end

function CreatePlaceableEvent.new(wayPoints, mapMarkers)
    local self = CreatePlaceableEvent.emptyNew()
    self.wayPoints = wayPoints
    self.mapMarkers = mapMarkers
    return self
end

function CreatePlaceableEvent:writeStream(streamId, connection)
    local paramsXZ = g_currentMission.vehicleXZPosCompressionParams
    local paramsY = g_currentMission.vehicleYPosCompressionParams

    streamWriteUInt32(streamId, #self.wayPoints)
    for _, wp in pairs(self.wayPoints) do
        NetworkUtil.writeCompressedWorldPosition(streamId, wp.x, paramsXZ)
        NetworkUtil.writeCompressedWorldPosition(streamId, wp.y, paramsY)
        NetworkUtil.writeCompressedWorldPosition(streamId, wp.z, paramsXZ)

        -- writing the amount of out nodes we are going to send
        streamWriteUInt8(streamId, #wp.out)
        -- writing out nodes
        for _, out in pairs(wp.out) do
            streamWriteUInt32(streamId, out)
        end

        -- writing the amount of incoming nodes we are going to send
        streamWriteUInt8(streamId, #wp.incoming)
        -- writing incoming nodes
        for _, incoming in pairs(wp.incoming) do
            streamWriteUInt32(streamId, incoming)
        end

        streamWriteUInt8(streamId, wp.flags)
    end

    -- writing the amount of markers we are going to send
    local markersCount = #self.mapMarkers
    streamWriteUInt8(streamId, markersCount)
    -- writing markers
    for _, marker in pairs(self.mapMarkers) do
        streamWriteUInt32(streamId, marker.id)
        AutoDrive.streamWriteStringOrEmpty(streamId, marker.name)
    end
end

function CreatePlaceableEvent:readStream(streamId, connection)
    local paramsXZ = g_currentMission.vehicleXZPosCompressionParams
    local paramsY = g_currentMission.vehicleYPosCompressionParams

    self.wayPoints = {}
    local wpCount = streamReadUInt32(streamId)
    for i=1, wpCount do
        local x = NetworkUtil.readCompressedWorldPosition(streamId, paramsXZ)
        local y = NetworkUtil.readCompressedWorldPosition(streamId, paramsY)
        local z = NetworkUtil.readCompressedWorldPosition(streamId, paramsXZ)
        local wp = {id = i, x = x, y = y, z = z, out = {}, incoming = {}}

        -- reading amount of out nodes we are going to read
        local outCount = streamReadUInt8(streamId)
        -- reading out nodes
        for ii = 1, outCount do
            wp.out[ii] = streamReadUInt32(streamId)
        end

        -- reading amount of incoming nodes we are going to read
        local incomingCount = streamReadUInt8(streamId)
        -- reading incoming nodes
        for ii = 1, incomingCount do
            wp.incoming[ii] = streamReadUInt32(streamId)
        end

        local flags = streamReadUInt8(streamId)

        wp.flags = flags

        self.wayPoints[i] = wp
    end

    self.mapMarkers = {}
    -- reading amount of markers we are going to read
    local markersCount = streamReadUInt8(streamId)
    -- reading markers
    for i = 1, markersCount do
        local marker = {}
        marker.id = streamReadUInt32(streamId)
        marker.name = AutoDrive.streamReadStringOrEmpty(streamId)
        self.mapMarkers[i] = marker
    end

    self:run(connection)
end

function CreatePlaceableEvent:run(connection)
    if g_server ~= nil and connection:getIsServer() == false then
        -- If the event is coming from a client, server have only to broadcast
        CreatePlaceableEvent.sendEvent(self.wayPoints, self.mapMarkers)
    else
        -- If the event is coming from the server, both clients and server have to create the way point
        AutoDrivePlaceableData:createPlaceable(self.wayPoints, self.mapMarkers, false)
    end
end

function CreatePlaceableEvent.sendEvent(wayPoints, mapMarkers)
    local event = CreatePlaceableEvent.new(wayPoints, mapMarkers)
    if g_server ~= nil then
        -- Server have to broadcast to all clients and himself
        g_server:broadcastEvent(event, true)
    else
        -- Client have to send to server
        g_client:getServerConnection():sendEvent(event)
    end
end
