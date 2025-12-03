AutoDriveDeleteWayPointsEvent = {}
AutoDriveDeleteWayPointsEvent_mt = Class(AutoDriveDeleteWayPointsEvent, Event)

InitEventClass(AutoDriveDeleteWayPointsEvent, "AutoDriveDeleteWayPointsEvent")

function AutoDriveDeleteWayPointsEvent.emptyNew()
	local self = Event.new(AutoDriveDeleteWayPointsEvent_mt)
	return self
end

function AutoDriveDeleteWayPointsEvent.new(wayPointIDs)
    local self = AutoDriveDeleteWayPointsEvent.emptyNew()
	self.wayPointIDs = wayPointIDs or {}
	return self
end

function AutoDriveDeleteWayPointsEvent:writeStream(streamId, connection)
    local wayPointsCount = #self.wayPointIDs
    streamWriteUInt32(streamId, wayPointsCount)
    if wayPointsCount > 0 then
        for _, wayPointIDs in pairs(self.wayPointIDs) do
            streamWriteUIntN(streamId, wayPointIDs, 20)
        end
    end
end

function AutoDriveDeleteWayPointsEvent:readStream(streamId, connection)
    self.wayPointIDs = {}
    local wayPointsCount = streamReadUInt32(streamId)
    if wayPointsCount > 0 then
        for i = 1, wayPointsCount do
    	    self.wayPointIDs[i] = streamReadUIntN(streamId, 20)
        end
    end
	self:run(connection)
end

function AutoDriveDeleteWayPointsEvent:run(connection)
	if g_server ~= nil and connection:getIsServer() == false then
		-- If the event is coming from a client, server have only to broadcast
		AutoDriveDeleteWayPointsEvent.sendEvent(self.wayPointIDs)
	else
		-- If the event is coming from the server, both clients and server have to delete the way point
        local wayPointsCount = #self.wayPointIDs
        if wayPointsCount > 0 then
            for i = 1, wayPointsCount do
                ADGraphManager:removeWayPoint(self.wayPointIDs[i], false)
            end
        end
	end
end

function AutoDriveDeleteWayPointsEvent.sendEvent(wayPointIDs)
	local event = AutoDriveDeleteWayPointsEvent.new(wayPointIDs)
	if g_server ~= nil then
		-- Server have to broadcast to all clients and himself
		g_server:broadcastEvent(event, true)
	else
		-- Client have to send to server
		g_client:getServerConnection():sendEvent(event)
	end
end
