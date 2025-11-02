AutoDriveVehicleData = {}
function AutoDriveVehicleData:new(vehicle)
    local o = {}
    setmetatable(o, self)
    self.__index = self
    o.vehicle = vehicle
    AutoDriveVehicleData.reset(o)
    return o
end

function AutoDriveVehicleData:reset()
    self.parkDestination = -1
    self.driverName = g_i18n:getText("UNKNOWN")
    if self.vehicle.getName ~= nil then
        self.driverName = self.vehicle:getName()
    end
end

function AutoDriveVehicleData.prerequisitesPresent(specializations)
    return true
end

function AutoDriveVehicleData.registerEventListeners(vehicleType)
    AutoDrive.debugPrint(nil, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDriveVehicleData.registerEventListeners ")
    for _, n in pairs(
        {
            "onPreLoad",
            "onLoad",
            "onPostLoad",
            "onSelect",
            "onEnterVehicle",
            "onPreDetach",
            "saveToXMLFile",
            "onReadStream",
            "onWriteStream"
            -- "onReadUpdateStream",
            -- "onWriteUpdateStream"
        }
    ) do
        SpecializationUtil.registerEventListener(vehicleType, n, AutoDriveVehicleData)
    end
end

function AutoDriveVehicleData.registerFunctions(vehicleType)
    SpecializationUtil.registerFunction(vehicleType, "getParkDestination", AutoDriveVehicleData.getParkDestination)
    SpecializationUtil.registerFunction(vehicleType, "setParkDestination", AutoDriveVehicleData.setParkDestination)
end

function AutoDriveVehicleData:onPreLoad(savegame)
    -- if self.spec_advd == nil then
    -- self.spec_advd = AutoDriveVehicleData
    -- end
end

function AutoDriveVehicleData:onLoad(savegame)
    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDriveVehicleData.onLoad vehicle %s savegame %s", tostring(self:getName()), tostring(savegame))
    if self.advd == nil then
        self.advd = {}
    end
    self.advd = AutoDriveVehicleData:new(self)
    self.advd.dirtyFlag = self:getNextDirtyFlag()
    self.advd.parkDestination = -1
end

function AutoDriveVehicleData:onPostLoad(savegame)
    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDriveVehicleData.onPostLoad vehicle %s savegame %s self %s", tostring(self:getName()), tostring(savegame), tostring(self))
    if self.advd == nil then
        return
    end
    if self.isServer then
        if savegame ~= nil then
            AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDriveVehicleData.onPostLoad self.isServer")
            local xmlFile = savegame.xmlFile
            local adkey = savegame.key .. ".AutoDrive"
            if xmlFile:hasProperty(adkey) then
                self.advd.parkDestination = Utils.getNoNil(xmlFile:getValue(adkey .. "#parkDestination"), -1)
            end            
        end
    end
end

--[[
tool selection seems strange on dedi servers as known!
That's why the following event is only taken on clients and send as event in the network
onEnterVehicle is used here to detect an already selected worktool and send it's park destination over network as job end position and display in HUD
]]
function AutoDriveVehicleData:onEnterVehicle()
    local actualparkDestination = -1
    if g_dedicatedServer == nil then
        -- only send the client event to server
        local selectedWorkTool = AutoDrive.getSelectedWorkTool(self)
        if selectedWorkTool ~= nil and selectedWorkTool.advd ~= nil then
            actualparkDestination = selectedWorkTool.advd.parkDestination
        else
            actualparkDestination = self.advd.parkDestination
        end
        local rootVehicle = self:getRootVehicle()
        if rootVehicle ~= nil and rootVehicle.ad ~= nil and AutoDrive:getIsEntered(rootVehicle) and self == rootVehicle then
            if actualparkDestination == nil then
                actualparkDestination = -1
            end
            -- propagate park destination only if vehicle is entered
            AutoDriveVehicleData:assignRootVehicleParkDestination(rootVehicle, actualparkDestination)
        end
    end
end

--[[
tool selection seems strange on dedi servers as known!
That's why the following event is only taken on clients and send as event in the network
onSelect is used here to detect an actualy selected worktool and send it's park destination over network as job end position and display in HUD
]]
function AutoDriveVehicleData:onSelect()
    local actualparkDestination = -1
    if g_dedicatedServer == nil then
        -- only send the client event to server
        if self.advd ~= nil then
            actualparkDestination = self.advd.parkDestination

            if actualparkDestination == nil then
                actualparkDestination = -1
            end

            local rootVehicle = self:getRootVehicle()
            if rootVehicle ~= nil and rootVehicle.ad ~= nil and AutoDrive:getIsEntered(rootVehicle) then
                -- propagate park destination only if vehicle is entered, as Giants engine also select vehicle, tools on startup
                AutoDriveVehicleData:assignRootVehicleParkDestination(rootVehicle, actualparkDestination)
            end
        end
    end
end

--[[
tool selection seems strange on dedi servers as known!
That's why the following event is only taken on clients and send as event in the network
onPreDetach is used here, as with automatic engine start the vehicle is not selectable, to detect an actualy selected worktool and send it's park destination over network as job end position and display in HUD
]]
function AutoDriveVehicleData:onPreDetach(attacherVehicle, implement)
    local actualparkDestination = -1
    if g_dedicatedServer == nil then
        -- only send the client event to server
        if attacherVehicle.advd ~= nil then
            actualparkDestination = attacherVehicle.advd.parkDestination

            if actualparkDestination == nil then
                actualparkDestination = -1
            end

            local rootVehicle = attacherVehicle:getRootVehicle()
            if rootVehicle ~= nil and rootVehicle.ad ~= nil and AutoDrive:getIsEntered(rootVehicle) then
                -- propagate park destination only if vehicle is entered, as Giants engine also select vehicle, tools on startup
                AutoDriveVehicleData:assignRootVehicleParkDestination(rootVehicle, actualparkDestination)
            end
        end
    end
end

function AutoDriveVehicleData:saveToXMLFile(xmlFile, key)
    AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDriveVehicleData.saveToXMLFile vehicle %s", tostring(self:getName()))

    if self.advd == nil then
        return
    end
    local actualparkDestination = self.advd.parkDestination
    if actualparkDestination == nil then
        actualparkDestination = -1
    end
    if actualparkDestination ~= nil and actualparkDestination > 0 then
        AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDriveVehicleData.saveToXMLFile actualparkDestination %s", tostring(actualparkDestination))
        local adKey = string.gsub(key, "FS25_AutoDrive.AutoDriveVehicleData", "AutoDrive")
        xmlFile:setValue(adKey .. "#parkDestination", actualparkDestination)
    end
end

-- this is important to sync the park destinations from server to clients, later only clients will send the park destination to server as event!
function AutoDriveVehicleData:onReadStream(streamId, connection) -- Called on client side on join
    if self ~= nil and self.getName ~= nil then
        AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDriveVehicleData.onReadStream vehicle %s", tostring(self:getName()))
    end
    self.advd.parkDestination = streamReadUIntN(streamId, 20) - 1
end

-- this is important to sync the park destinations from server to clients, later only clients will send the park destination to server as event!
function AutoDriveVehicleData:onWriteStream(streamId, connection) -- Called on server side on join
    if self ~= nil and self.getName ~= nil then
        AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDriveVehicleData.onWriteStream vehicle %s", tostring(self:getName()))
    end
    streamWriteUIntN(streamId, self.advd.parkDestination + 1, 20)
end

function AutoDriveVehicleData:onReadUpdateStream(streamId, timestamp, connection) -- Called on on update
    if self ~= nil and self.getName ~= nil then
        AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDriveVehicleData.onReadUpdateStream vehicle %s", tostring(self:getName()))
    end
    if connection:getIsServer() then
        if streamReadBool(streamId) then
            AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDriveVehicleData.onReadUpdateStream streamReadBool ")
            self.advd.parkDestination = streamReadUIntN(streamId, 20) - 1
        end
    end
end

function AutoDriveVehicleData:onWriteUpdateStream(streamId, connection, dirtyMask) -- Called on on update
    if self ~= nil and self.getName ~= nil then
        AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDriveVehicleData.onWriteUpdateStream vehicle %s", tostring(self:getName()))
    end
    if not connection:getIsServer() then
        if streamWriteBool(streamId, bit32.band(dirtyMask, self.advd.dirtyFlag) ~= 0) then
            AutoDrive.debugPrint(self, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDriveVehicleData.onWriteUpdateStream streamReadBool ")
            streamWriteUIntN(streamId, self.advd.parkDestination + 1, 20)
        end
    end
end

function AutoDriveVehicleData:raiseDirtyFlag()
    self.vehicle:raiseDirtyFlags(self.vehicle.advd.dirtyFlag)
end

function AutoDriveVehicleData:getParkDestination(vehicle)
    if vehicle ~= nil then
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDriveVehicleData:getParkDestination vehicle %s", tostring(vehicle:getName()))
    end
    if vehicle == nil or vehicle.advd == nil then
        return -1
    end

    return vehicle.advd.parkDestination
end

-- set the park destination for vehicle, which could be a worktool, attachment or vehicle itself
function AutoDriveVehicleData:setParkDestination(vehicle, parkDestination, sendEvent)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDriveVehicleData:setParkDestination vehicle %s parkDestination %s", tostring(vehicle), tostring(parkDestination))
    if vehicle == nil or vehicle.advd == nil then
        return
    end

    if sendEvent == nil or sendEvent == true then
        -- Propagating way point deletion all over the network
        AutoDriveVehicleDataEventSetToolParkDestination.sendEvent(vehicle, parkDestination)
    else
        vehicle.advd.parkDestination = parkDestination

        local rootVehicle = vehicle:getRootVehicle()
            -- set park destination in stateModule as well to keep it up to date
        if rootVehicle ~= nil and rootVehicle.ad ~= nil and rootVehicle.ad.stateModule ~= nil then
            rootVehicle.ad.stateModule:setParkDestinationAtJobFinished(parkDestination)
        end
    end
end

-- assign park destination to root vehicle from park destination of worktool or vehicle itself
function AutoDriveVehicleData:assignRootVehicleParkDestination(vehicle, parkDestination, sendEvent)
    if vehicle == nil or vehicle.advd == nil or parkDestination == nil then
        return
    end

    if sendEvent == nil or sendEvent == true then
        -- Propagating way point deletion all over the network
        AutoDriveVehicleDataEventAssignParkDestination.sendEvent(vehicle, parkDestination)
    else
        if vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.stateModule ~= nil then
            vehicle.ad.stateModule:setParkDestinationAtJobFinished(parkDestination)
        end
    end
end

-- event to set the actual park destination for vehicle job end, dependend on selected worktool
AutoDriveVehicleDataEventAssignParkDestination = {}
AutoDriveVehicleDataEventAssignParkDestination_mt = Class(AutoDriveVehicleDataEventAssignParkDestination, Event)

InitEventClass(AutoDriveVehicleDataEventAssignParkDestination, "AutoDriveVehicleDataEventAssignParkDestination")

function AutoDriveVehicleDataEventAssignParkDestination.emptyNew()
    local self = Event.new(AutoDriveVehicleDataEventAssignParkDestination_mt)
    return self
end

function AutoDriveVehicleDataEventAssignParkDestination.new(vehicle, parkDestination)
    local self = AutoDriveVehicleDataEventAssignParkDestination.emptyNew()
    self.vehicle = vehicle
    self.parkDestination = parkDestination
    return self
end

function AutoDriveVehicleDataEventAssignParkDestination:writeStream(streamId, connection)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDriveVehicleDataEventAssignParkDestination:writeStream connection %s", tostring(connection))
    NetworkUtil.writeNodeObjectId(streamId, NetworkUtil.getObjectId(self.vehicle))
    streamWriteUIntN(streamId, self.parkDestination + 1, 20)
end

function AutoDriveVehicleDataEventAssignParkDestination:readStream(streamId, connection)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDriveVehicleDataEventAssignParkDestination:readStream connection %s", tostring(connection))
    self.vehicle = NetworkUtil.getObject(NetworkUtil.readNodeObjectId(streamId))
    self.parkDestination = streamReadUIntN(streamId, 20) - 1
    self:run(connection)
end

function AutoDriveVehicleDataEventAssignParkDestination:run(connection)
	if g_server ~= nil and connection:getIsServer() == false then
		-- If the event is coming from a client, server have only to broadcast
        AutoDriveVehicleDataEventAssignParkDestination.sendEvent(self.vehicle, self.parkDestination)
	else
		-- If the event is coming from the server, both clients and server have to react
        AutoDriveVehicleData:assignRootVehicleParkDestination(self.vehicle, self.parkDestination, false)
	end
end

function AutoDriveVehicleDataEventAssignParkDestination.sendEvent(vehicle, parkDestination)
	local event = AutoDriveVehicleDataEventAssignParkDestination.new(vehicle, parkDestination)

	if g_server ~= nil then
		-- Server have to broadcast to all clients and himself
		g_server:broadcastEvent(event, true)
	else
		-- Client have to send to server
		g_client:getServerConnection():sendEvent(event)
	end
end

-- event to set a new park destination for worktool, vehicle itself
AutoDriveVehicleDataEventSetToolParkDestination = {}
AutoDriveVehicleDataEventSetToolParkDestination_mt = Class(AutoDriveVehicleDataEventSetToolParkDestination, Event)

InitEventClass(AutoDriveVehicleDataEventSetToolParkDestination, "AutoDriveVehicleDataEventSetToolParkDestination")

function AutoDriveVehicleDataEventSetToolParkDestination.emptyNew()
    local self = Event.new(AutoDriveVehicleDataEventSetToolParkDestination_mt)
    return self
end

function AutoDriveVehicleDataEventSetToolParkDestination.new(vehicle, parkDestination)
    local self = AutoDriveVehicleDataEventSetToolParkDestination.emptyNew()
    self.vehicle = vehicle
    self.parkDestination = parkDestination
    return self
end

function AutoDriveVehicleDataEventSetToolParkDestination:writeStream(streamId, connection)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDriveVehicleDataEventSetToolParkDestination:writeStream connection %s", tostring(connection))
    NetworkUtil.writeNodeObjectId(streamId, NetworkUtil.getObjectId(self.vehicle))
    streamWriteUIntN(streamId, self.parkDestination + 1, 20)
end

function AutoDriveVehicleDataEventSetToolParkDestination:readStream(streamId, connection)
    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_EXTERNALINTERFACEINFO, "AutoDriveVehicleDataEventSetToolParkDestination:readStream connection %s", tostring(connection))
    self.vehicle = NetworkUtil.getObject(NetworkUtil.readNodeObjectId(streamId))
    self.parkDestination = streamReadUIntN(streamId, 20) - 1
    self:run(connection)
end

function AutoDriveVehicleDataEventSetToolParkDestination:run(connection)
	if g_server ~= nil and connection:getIsServer() == false then
		-- If the event is coming from a client, server have only to broadcast
        AutoDriveVehicleDataEventSetToolParkDestination.sendEvent(self.vehicle, self.parkDestination)
	else
		-- If the event is coming from the server, both clients and server have to react
        self.vehicle.advd:setParkDestination(self.vehicle, self.parkDestination, false)
	end
end

function AutoDriveVehicleDataEventSetToolParkDestination.sendEvent(vehicle, parkDestination)
	local event = AutoDriveVehicleDataEventSetToolParkDestination.new(vehicle, parkDestination)

	if g_server ~= nil then
		-- Server have to broadcast to all clients and himself
		g_server:broadcastEvent(event, true)
	else
		-- Client have to send to server
		g_client:getServerConnection():sendEvent(event)
	end
end

