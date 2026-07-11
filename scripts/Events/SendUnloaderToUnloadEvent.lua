AutoDriveSendUnloaderToUnloadEvent = {}
local AutoDriveSendUnloaderToUnloadEvent_mt = Class(AutoDriveSendUnloaderToUnloadEvent, Event)

InitEventClass(AutoDriveSendUnloaderToUnloadEvent, "AutoDriveSendUnloaderToUnloadEvent")

function AutoDriveSendUnloaderToUnloadEvent.emptyNew()
    return Event.new(AutoDriveSendUnloaderToUnloadEvent_mt)
end

function AutoDriveSendUnloaderToUnloadEvent.new(harvester, unloader)
    local self = AutoDriveSendUnloaderToUnloadEvent.emptyNew()
    self.harvester = harvester
    self.unloader = unloader
    return self
end

function AutoDriveSendUnloaderToUnloadEvent:writeStream(streamId, connection)
    NetworkUtil.writeNodeObject(streamId, self.harvester)
    NetworkUtil.writeNodeObject(streamId, self.unloader)
end

function AutoDriveSendUnloaderToUnloadEvent:readStream(streamId, connection)
    self.harvester = NetworkUtil.readNodeObject(streamId)
    self.unloader = NetworkUtil.readNodeObject(streamId)
    self:run(connection)
end

function AutoDriveSendUnloaderToUnloadEvent:run(connection)
    if g_server ~= nil then
        ADHarvestManager:sendUnloaderToUnload(self.harvester, self.unloader)
    end
end

function AutoDriveSendUnloaderToUnloadEvent.sendEvent(harvester, unloader)
    if g_server ~= nil then
        ADHarvestManager:sendUnloaderToUnload(harvester, unloader)
    elseif g_client ~= nil then
        g_client:getServerConnection():sendEvent(AutoDriveSendUnloaderToUnloadEvent.new(harvester, unloader))
    end
end

