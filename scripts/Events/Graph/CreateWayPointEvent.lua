AutoDriveCreateWayPointEvent = {}
AutoDriveCreateWayPointEvent_mt = Class(AutoDriveCreateWayPointEvent, Event)

InitEventClass(AutoDriveCreateWayPointEvent, "AutoDriveCreateWayPointEvent")

function AutoDriveCreateWayPointEvent.emptyNew()
	local self = Event.new(AutoDriveCreateWayPointEvent_mt)
	return self
end

function AutoDriveCreateWayPointEvent.new(x, y, z, out, incoming, flags, connect, reverseDirection, dualConnection)
	local self = AutoDriveCreateWayPointEvent.emptyNew()
	self.x = x
	self.y = y
	self.z = z
	self.out = out
	self.incoming = incoming
	self.flags = flags
	self.connect = connect
	self.reverseDirection = reverseDirection
	self.dualConnection = dualConnection
	return self
end

function AutoDriveCreateWayPointEvent:writeStream(streamId, connection)
	streamWriteFloat32(streamId, self.x)
	streamWriteFloat32(streamId, self.y)
	streamWriteFloat32(streamId, self.z)

	local count = self.out and #self.out or 0
	streamWriteUInt16(streamId, count)
	if self.out and #self.out > 0 then
		for _, out in pairs(self.out) do
			streamWriteUIntN(streamId, out + 1, 24)
		end
	end

	count = self.incoming and #self.incoming or 0
	streamWriteUInt16(streamId, count)
	if self.incoming and #self.incoming > 0 then
		for _, incoming in pairs(self.incoming) do
			streamWriteUIntN(streamId, incoming + 1, 24)
		end
	end

	streamWriteUInt16(streamId, self.flags)
	streamWriteBool(streamId, self.connect)
	streamWriteBool(streamId, self.reverseDirection)
	streamWriteBool(streamId, self.dualConnection)
end

function AutoDriveCreateWayPointEvent:readStream(streamId, connection)
	self.x = streamReadFloat32(streamId)
	self.y = streamReadFloat32(streamId)
	self.z = streamReadFloat32(streamId)
	self.out = {}
	self.incoming = {}

	local count = streamReadUInt16(streamId)
	if count > 0 then
		for ii = 1, count do
			self.out[ii] = streamReadUIntN(streamId, 24) - 1
		end
	end

	count = streamReadUInt16(streamId)
	if count > 0 then
		for ii = 1, count do
			self.incoming[ii] = streamReadUIntN(streamId, 24) - 1
		end
	end

	self.flags = streamReadUInt16(streamId)
	self.connect = streamReadBool(streamId)
	self.reverseDirection = streamReadBool(streamId)
	self.dualConnection = streamReadBool(streamId)
	self:run(connection)
end

function AutoDriveCreateWayPointEvent:run(connection)
	if g_server ~= nil and connection:getIsServer() == false then
		-- If the event is coming from a client, server have only to broadcast
		AutoDriveCreateWayPointEvent.sendEvent(self.x, self.y, self.z, self.out, self.incoming, self.flags, self.connect, self.reverseDirection, self.dualConnection)
	else
		-- If the event is coming from the server, both clients and server have to create the way point
		ADGraphManager:createWayPointWithConnections(self.x, self.y, self.z, self.out, self.incoming, self.flags, self.connect, self.reverseDirection, self.dualConnection, false)
	end
end

function AutoDriveCreateWayPointEvent.sendEvent(x, y, z, out, incoming, flags, connect, reverseDirection, dualConnection)
	local event = AutoDriveCreateWayPointEvent.new(x, y, z, out, incoming, flags, connect, reverseDirection, dualConnection)
	if g_server ~= nil then
		-- Server have to broadcast to all clients and himself
		g_server:broadcastEvent(event, true)
	else
		-- Client have to send to server
		g_client:getServerConnection():sendEvent(event)
	end
end
