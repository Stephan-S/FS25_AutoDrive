AutoDriveSync = {}
AutoDriveSync.MWPC_SEND_NUM_BITS = 0 -- that's dynamic
AutoDriveSync.MWPC_SNB_SEND_NUM_BITS = 5 -- 0 -> 31
AutoDriveSync.OIWPC_SEND_NUM_BITS = 6 -- 0 -> 63
AutoDriveSync.MC_SEND_NUM_BITS = 12 -- 0 -> 4095
AutoDriveSync.GC_SEND_NUM_BITS = 10 -- 0 -> 1023
AutoDriveSync.FLAG_SEND_NUM_BITS = 5 -- 0 -> 31

AutoDriveSync_mt = Class(AutoDriveSync, Object)

InitObjectClass(AutoDriveSync, "AutoDriveSync")

function AutoDriveSync.new(isServer, isClient, customMt)
    local self = Object.new(isServer, isClient, customMt or AutoDriveSync_mt)
    self.dirtyFlag = self:getNextDirtyFlag()
    registerObjectClassName(self, "AutoDriveSync")
    return self
end

function AutoDriveSync:delete()
    unregisterObjectClassName(self)
    AutoDriveSync:superClass().delete(self)
end

function AutoDriveSync:writeStream(streamId, connection)
    AutoDriveSync.streamWriteGraph(streamId, ADGraphManager:getWayPoints(), ADGraphManager:getMapMarkers(), ADGraphManager:getGroups())
    AutoDriveSync:superClass().writeStream(self, streamId, connection)
end

function AutoDriveSync:readStream(streamId, connection) 
    if connection:getIsServer() then
        local wayPoints, mapMarkers, groups = AutoDriveSync.streamReadGraph(streamId)
        ADGraphManager:setWayPoints(wayPoints)
        ADGraphManager:setMapMarkers(mapMarkers)
        AutoDrive:notifyDestinationListeners()
        ADGraphManager:setGroups(groups)
        AutoDrive.Hud.lastUIScale = 0
        AutoDriveSync:superClass().readStream(self, streamId, connection)
    end
end

function AutoDriveSync:readUpdateStream(streamId, timestamp, connection)
    --print(string.format("AutoDriveSync:readUpdateStream(%s, %s, %s)", streamId, timestamp, connection))
    AutoDriveSync:superClass().readUpdateStream(self, streamId, timestamp, connection)
end

function AutoDriveSync:writeUpdateStream(streamId, connection, dirtyMask)
    --print(string.format("AutoDriveSync:writeUpdateStream(%s, %s, %s)", streamId, connection, dirtyMask))
    AutoDriveSync:superClass().writeUpdateStream(self, streamId, connection, dirtyMask)
end

function AutoDriveSync:update(dt)
    AutoDriveSync:superClass().update(self, dt)
end

function AutoDriveSync:updateTick(dt)
    -- we need "update" both on server and client
    --self:raiseActive()

    -- we need "writeUpdateStream" only on server
    --if self.isServer then
    --    self:raiseDirtyFlags(self.dirtyFlag)
    --end

    AutoDriveSync:superClass().updateTick(self, dt)
end

function AutoDriveSync:draw()
    -- currently not called
    AutoDriveSync:superClass().draw(self)
end

function AutoDriveSync:mouseEvent(posX, posY, isDown, isUp, button)
    -- currently not called
    AutoDriveSync:superClass().mouseEvent(self, posX, posY, isDown, isUp, button)
end

function AutoDriveSync.streamWriteGraph(streamId, wayPoints, mapMarkers, groups)
    local self = AutoDriveSync
    local paramsXZ = g_currentMission.vehicleXZPosCompressionParams
    local paramsY = g_currentMission.vehicleYPosCompressionParams

    local offset = streamGetWriteOffset(streamId)
    local time = netGetTime()

    local wayPointsCount = #wayPoints
    -- writing the amount of bits we are going to use as "MWPC_SEND_NUM_BITS"
    self.MWPC_SEND_NUM_BITS = math.ceil(math.log(wayPointsCount + 1, 2))
    streamWriteUIntN(streamId, self.MWPC_SEND_NUM_BITS, self.MWPC_SNB_SEND_NUM_BITS)

    -- writing the amount of waypoints we are going to send
    streamWriteUIntN(streamId, wayPointsCount, self.MWPC_SEND_NUM_BITS)
    self.print("[AutoDriveSync] Writing %s waypoints", wayPointsCount)

    -- writing waypoints
    for i, wp in pairs(wayPoints) do
        if wp.id ~= i then
            Logging.error(string.format("[AutoDriveSync] Waypoint number %s have a wrong id %s", i, wp.id))
        end
        NetworkUtil.writeCompressedWorldPosition(streamId, wp.x, paramsXZ)
        NetworkUtil.writeCompressedWorldPosition(streamId, wp.y, paramsY)
        NetworkUtil.writeCompressedWorldPosition(streamId, wp.z, paramsXZ)

        -- writing the amount of out nodes we are going to send
        streamWriteUIntN(streamId, #wp.out, self.OIWPC_SEND_NUM_BITS)
        -- writing out nodes
        for _, out in pairs(wp.out) do
            streamWriteUIntN(streamId, out, self.MWPC_SEND_NUM_BITS)
        end

        -- writing the amount of incoming nodes we are going to send
        streamWriteUIntN(streamId, #wp.incoming, self.OIWPC_SEND_NUM_BITS)
        -- writing incoming nodes
        for _, incoming in pairs(wp.incoming) do
            streamWriteUIntN(streamId, incoming, self.MWPC_SEND_NUM_BITS)
        end
        
        streamWriteUIntN(streamId, wp.flags, self.FLAG_SEND_NUM_BITS)
    end

    -- writing the amount of markers we are going to send
    local markersCount = #mapMarkers
    self.print("[AutoDriveSync] Writing %s markers", markersCount)
    streamWriteUIntN(streamId, markersCount, self.MC_SEND_NUM_BITS)
    -- writing markers
    for _, marker in pairs(mapMarkers) do
        streamWriteUIntN(streamId, marker.id, self.MWPC_SEND_NUM_BITS)
        AutoDrive.streamWriteStringOrEmpty(streamId, marker.name)
        AutoDrive.streamWriteStringOrEmpty(streamId, marker.group)
        streamWriteBool(streamId, (marker.isADDebug == true))
    end

    -- writing the amount of groups we are going to send
    local groupsCount = table.count(groups)
    streamWriteUIntN(streamId, groupsCount, self.GC_SEND_NUM_BITS)
    self.print("[AutoDriveSync] Writing %s groups", groupsCount)
    -- writing groups
    for gName, gId in pairs(groups) do
        streamWriteUIntN(streamId, gId, self.GC_SEND_NUM_BITS)
        AutoDrive.streamWriteStringOrEmpty(streamId, gName)
    end

    offset = streamGetWriteOffset(streamId) - offset
    self.print("[AutoDriveSync] Written %s bits (%s bytes) in %s ms", offset, math.floor(offset / 8), netGetTime() - time)
end

function AutoDriveSync.streamReadGraph(streamId)
    local self = AutoDriveSync

    local wayPoints = {}
    local mapMarkers = {}
    local groups = {}

    local paramsXZ = g_currentMission.vehicleXZPosCompressionParams
    local paramsY = g_currentMission.vehicleYPosCompressionParams

    local offset = streamGetReadOffset(streamId)
    local time = netGetTime()

    -- reading the amount of bits we are going to use as "MWPC_SEND_NUM_BITS"
    self.MWPC_SEND_NUM_BITS = streamReadUIntN(streamId, self.MWPC_SNB_SEND_NUM_BITS)

    -- reading amount of waypoints we are going to read
    local wpsToRead = streamReadUIntN(streamId, self.MWPC_SEND_NUM_BITS)
    self.print("[AutoDriveSync] Reading %s way points", wpsToRead)

    -- reading waypoints
    for i = 1, wpsToRead do
        local x = NetworkUtil.readCompressedWorldPosition(streamId, paramsXZ)
        local y = NetworkUtil.readCompressedWorldPosition(streamId, paramsY)
        local z = NetworkUtil.readCompressedWorldPosition(streamId, paramsXZ)

        local wp = {id = i, x = x, y = y, z = z, out = {}, incoming = {}}

        -- reading amount of out nodes we are going to read
        local outCount = streamReadUIntN(streamId, self.OIWPC_SEND_NUM_BITS)
        -- reading out nodes
        for ii = 1, outCount do
            wp.out[ii] = streamReadUIntN(streamId, self.MWPC_SEND_NUM_BITS)
        end

        -- reading amount of incoming nodes we are going to read
        local incomingCount = streamReadUIntN(streamId, self.OIWPC_SEND_NUM_BITS)
        -- reading incoming nodes
        for ii = 1, incomingCount do
            wp.incoming[ii] = streamReadUIntN(streamId, self.MWPC_SEND_NUM_BITS)
        end
        
        local flags = streamReadUIntN(streamId, self.FLAG_SEND_NUM_BITS)

        wp.flags = flags

        wayPoints[i] = wp
    end

    -- reading amount of markers we are going to read
    local mapMarkerCounter = streamReadUIntN(streamId, self.MC_SEND_NUM_BITS)
    self.print("[AutoDriveSync] Reading %s markers", mapMarkerCounter)
    -- reading markers
    for i = 1, mapMarkerCounter do
        local markerId = streamReadUIntN(streamId, self.MWPC_SEND_NUM_BITS)
        if wayPoints[markerId] ~= nil then
            local marker = {}
            marker.id = markerId
            marker.markerIndex = i
            marker.name = AutoDrive.streamReadStringOrEmpty(streamId)
            marker.group = AutoDrive.streamReadStringOrEmpty(streamId)
            marker.isADDebug = streamReadBool(streamId)
            mapMarkers[i] = marker
        else
            Logging.error("[AutoDriveSync] Error receiving marker %s (%s)", AutoDrive.streamReadStringOrEmpty(streamId), markerId)
            -- we have to read everything to keep the right reading order
            _ = AutoDrive.streamReadStringOrEmpty(streamId)
            _ = streamReadBool(streamId)
        end
    end

    -- reading amount of groups we are going to read
    local groupsCount = streamReadUIntN(streamId, self.GC_SEND_NUM_BITS)
    self.print("[AutoDriveSync] Reading %s groups", groupsCount)
    -- reading groups
    for i = 1, groupsCount do
        local gId = streamReadUIntN(streamId, self.GC_SEND_NUM_BITS)
        local gName = AutoDrive.streamReadStringOrEmpty(streamId)
        if gName ~= nil and gName ~= "" then
            groups[gName] = gId
        end
    end

    offset = streamGetReadOffset(streamId) - offset
    self.print("[AutoDriveSync] Read %s bits (%s bytes) in %s ms", offset, offset / 8, netGetTime() - time)
    return wayPoints, mapMarkers, groups
end

function AutoDriveSync.print(text, ...)
    if g_dedicatedServer ~= nil then
        Logging.info(text, ...)
    else
        Logging.info(text, ...)
    end
end
