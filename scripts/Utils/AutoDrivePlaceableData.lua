AutoDrivePlaceableData = {}

function AutoDrivePlaceableData:load()
    AutoDrivePlaceableData.active = false
    AutoDrivePlaceableData.reset()
end

function AutoDrivePlaceableData.reset()
    if AutoDrivePlaceableData.xmlFile then
        delete(AutoDrivePlaceableData.xmlFile)
    end
    AutoDrivePlaceableData.placeable = nil
    AutoDrivePlaceableData.xmlFile = nil
    AutoDrivePlaceableData.wayPoints = {}
    AutoDrivePlaceableData.mapMarkers = {}
    AutoDrivePlaceableData.showErrorDialog = nil
    AutoDrivePlaceableData.showConfirmationDialog = nil
end

function AutoDrivePlaceableData.prerequisitesPresent(specializations)
    return true
end

function AutoDrivePlaceableData.registerEventListeners(placeableType)
    for _, n in pairs(
        {
            "onFinalizePlacement"
        }
    ) do
        SpecializationUtil.registerEventListener(placeableType, n, AutoDrivePlaceableData)
    end
end

function AutoDrivePlaceableData:setActive(active)
    if active ~= nil then
        AutoDrivePlaceableData.active = active
    end
end

function AutoDrivePlaceableData.callBack(result)
    if AutoDrivePlaceableData.showConfirmationDialog then
        AutoDrivePlaceableData.showConfirmationDialog = nil
        if result == true then
            if AutoDrivePlaceableData.xmlFile then
                local ret = AutoDrivePlaceableData.readGraphFromXml(AutoDrivePlaceableData.xmlFile, AutoDrivePlaceableData.placeable)
                if ret < 0 then
                    AutoDrivePlaceableData.showError(ret)
                else
                    AutoDrivePlaceableData.reset() -- all done
                end
            end
        end
    end
    if AutoDrivePlaceableData.showErrorDialog then
        AutoDrivePlaceableData.showErrorDialog = nil
        AutoDrivePlaceableData.reset() -- all done
    end

end

function AutoDrivePlaceableData.showError(error)
    local args = {text = g_i18n:getText("gui_ad_adpd_showError") .. " " .. error}
    local dialog = g_gui:showDialog("InfoDialog")
    if dialog then
        dialog.target:setDialogType(Utils.getNoNil(args.dialogType, DialogElement.TYPE_WARNING))
        dialog.target:setText(args.text)
        dialog.target:setCallback(AutoDrivePlaceableData.callBack)
        -- dialog.target:setButtonTexts(args.okText)
        -- dialog.target:setButtonAction(args.buttonAction)
        AutoDrivePlaceableData.showErrorDialog = dialog
    end
end

function AutoDrivePlaceableData.showConfirmation()
    local args = {text = g_i18n:getText("gui_ad_adpd_showConfirmation")}
    local dialog = g_gui:showDialog("YesNoDialog")
    if dialog then
        dialog.target:setDialogType(Utils.getNoNil(args.dialogType, DialogElement.TYPE_QUESTION))
        dialog.target:setText(args.text)
        dialog.target:setCallback(AutoDrivePlaceableData.callBack)
        -- dialog.target:setButtonTexts(args.okText)
        -- dialog.target:setButtonAction(args.buttonAction)
        AutoDrivePlaceableData.showConfirmationDialog = dialog
    end
end

-- TODO: find a solution to only process the event for the user which is placing the object and not other user hold the same object in the construction screen
function AutoDrivePlaceableData:onFinalizePlacement_TODO()
    if AutoDrivePlaceableData.active then
        if self.isClient and g_gui and g_gui.currentGui and g_gui.currentGuiName and g_gui.currentGuiName == "ConstructionScreen" then
            if g_currentMission.placeableSystem and g_currentMission.placeableSystem.placeables and #g_currentMission.placeableSystem.placeables > 0 then
                AutoDrivePlaceableData.placeable = g_currentMission.placeableSystem.placeables[#g_currentMission.placeableSystem.placeables]
                if g_gui.currentGui.target and g_gui.currentGui.target.brush and g_gui.currentGui.target.brush.placeable then
                    if g_gui.currentGui.target.brush.placeable and AutoDrivePlaceableData.placeable then

                        local currentUserId = nil
                        local brushCurrentUserId = g_gui.currentGui.target.brush.currentUserId
                        AutoDrive.debugMsg(nil, "AutoDrivePlaceableData:onFinalizePlacement brushCurrentUserId %s "
                        , tostring(brushCurrentUserId)
                        )
--[[
                        -- if connection:getIsServer() then
                        local connection = g_client:getServerConnection()
                        AutoDrive.debugMsg(nil, "AutoDrivePlaceableData:onFinalizePlacement connection %s "
                        , tostring(connection)
                        )
                        AutoDrive.debugMsg(nil, "AutoDrivePlaceableData:onFinalizePlacement connection:getIsServer() %s "
                        , tostring(connection:getIsServer())
                        )

                        -- local connection = self:getOwnerConnection()
                        if connection ~= nil then
                            -- MP
                            currentUserId = g_currentMission.userManager:getUserIdByConnection(connection)
                            AutoDrive.debugMsg(nil, "AutoDrivePlaceableData:onFinalizePlacement currentUserId %s "
                            , tostring(currentUserId)
                            )
                        end
                        if currentUserId == nil or currentUserId == brushCurrentUserId then
]]
                        AutoDrive.debugMsg(nil, "AutoDrivePlaceableData:onFinalizePlacement g_localPlayer.userId %s "
                        , tostring(g_localPlayer.userId)
                        )
                        if g_localPlayer.userId == brushCurrentUserId then
                            local xmlFileName = g_currentMission.placeableSystem.placeables[#g_currentMission.placeableSystem.placeables].configFileName
                            if xmlFileName then
                                AutoDrivePlaceableData.xmlFile = loadXMLFile("placeable_xml", xmlFileName)
                                if AutoDrivePlaceableData.xmlFile then
                                    if hasXMLProperty(AutoDrivePlaceableData.xmlFile, "placeable")
                                    and hasXMLProperty(AutoDrivePlaceableData.xmlFile, "placeable.AutoDrive")
                                    and hasXMLProperty(AutoDrivePlaceableData.xmlFile, "placeable.AutoDrive.wayPoints")
                                    then
                                        AutoDrivePlaceableData.showConfirmation()
                                    else
                                        -- reset all
                                        AutoDrivePlaceableData.reset()
                                    end
                                else
                                    -- reset all
                                    AutoDrivePlaceableData.reset()
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end

function AutoDrivePlaceableData:onFinalizePlacement()
    if AutoDrivePlaceableData.active then
        if self.isServer then
            if g_currentMission.placeableSystem and g_currentMission.placeableSystem.placeables and #g_currentMission.placeableSystem.placeables > 0 then
                AutoDrivePlaceableData.placeable = g_currentMission.placeableSystem.placeables[#g_currentMission.placeableSystem.placeables]
                if AutoDrivePlaceableData.placeable then
                    local xmlFileName = g_currentMission.placeableSystem.placeables[#g_currentMission.placeableSystem.placeables].configFileName
                    if xmlFileName then
                        AutoDrivePlaceableData.xmlFile = loadXMLFile("placeable_xml", xmlFileName)
                        if AutoDrivePlaceableData.xmlFile then
                            if hasXMLProperty(AutoDrivePlaceableData.xmlFile, "placeable")
                            and hasXMLProperty(AutoDrivePlaceableData.xmlFile, "placeable.AutoDrive")
                            and hasXMLProperty(AutoDrivePlaceableData.xmlFile, "placeable.AutoDrive.wayPoints")
                            then
                                -- AutoDrivePlaceableData.showConfirmation()
                                local ret = AutoDrivePlaceableData.readGraphFromXml(AutoDrivePlaceableData.xmlFile, AutoDrivePlaceableData.placeable)
                                if ret < 0 then
                                    if self.isClient then
                                        AutoDrivePlaceableData.showError(ret)
                                    else
                                        Logging.error(g_i18n:getText("gui_ad_adpd_showError") .. " " .. ret)
                                    end
                                    return
                                end
                            end
                        end
                    end
                end
            end
        end
    end
    -- reset all
    AutoDrivePlaceableData.reset()
end

function AutoDrivePlaceableData.readGraphFromXml(xmlFile, placeable)
    AutoDrivePlaceableData.wayPoints = {}
    AutoDrivePlaceableData.mapMarkers = {}

    do
        local function checkProperty(key)
            if not hasXMLProperty(xmlFile, key) then
                AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData.readGraphFromXml no property for key %s ", tostring(key))
                return -1
            end
        end

        local function checkString(key)
            local tempString = getXMLString(xmlFile, key)
            if tempString == nil then
                AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData.readGraphFromXml no value for key %s ", tostring(key))
                return -2
            end
            local xTable = string.split(tempString, ",")
            if #xTable == 0 then
                AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData.readGraphFromXml no values for key %s ", tostring(key))
                return -3
            end
        end

        checkProperty("placeable")
        checkProperty("placeable.AutoDrive")
        checkProperty("placeable.AutoDrive.wayPoints")

        local key
        local tempString

        key = "placeable.AutoDrive.waypoints.x"
        checkProperty(key)
        checkString(key)
        local xt = string.split(getXMLString(xmlFile, key), ",")
        key = "placeable.AutoDrive.waypoints.y" -- not required, only for consistency check
        checkProperty(key)
        checkString(key)
        local yt = string.split(getXMLString(xmlFile, key), ",")
        key = "placeable.AutoDrive.waypoints.z"
        checkProperty(key)
        checkString(key)
        local zt = string.split(getXMLString(xmlFile, key), ",")
        key = "placeable.AutoDrive.waypoints.out"
        checkProperty(key)
        -- checkString(key)
        local ot = string.split(getXMLString(xmlFile, key), ";")
        key = "placeable.AutoDrive.waypoints.incoming"
        checkProperty(key)
        -- checkString(key)
        local it = string.split(getXMLString(xmlFile, key), ";")
        key = "placeable.AutoDrive.waypoints.flags"
        checkProperty(key)
        checkString(key)
        local ft = string.split(getXMLString(xmlFile, key), ",")

        if #xt == 0 or #yt == 0 or #zt == 0 or #ot == 0 or #it == 0 or #ft == 0 or #xt ~= #yt or #xt ~= #zt or #xt ~= #ot or #xt ~= #it or #xt ~= #ft then
            if #xt == 0 then
                AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #xt == 0")
            end
            if #yt == 0 then
                AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #yt == 0")
            end
            if #zt == 0 then
                AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #zt == 0")
            end
            if #ot == 0 then
                AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #ot == 0")
            end
            if #it == 0 then
                AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #it == 0")
            end
            if #ft == 0 then
                AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #ft == 0")
            end

            if #xt ~= #yt then
                AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #xt ~= #yt")
            end
            if #xt ~= #zt then
                AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #xt ~= #zt")
            end
            if #xt ~= #ot then
                AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #xt ~= #ot")
            end
            if #xt ~= #it then
                AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #xt ~= #it")
            end
            if #xt ~= #ft then
                AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData:readGraphFromXml invalid consitency #xt ~= #ft %s %s"
                , tostring(#xt)
                , tostring(#ft)
                )
            end

            return -4
        end

        local waypointsCount = #xt

        local mapMarker = {}
        local mapMarkerCounter = 1

        while mapMarker ~= nil do
            mapMarker.id = getXMLFloat(xmlFile, "placeable.AutoDrive.mapmarker.mm" .. mapMarkerCounter .. ".id")
            -- if id is still nil, we are at the end of the list and stop here

            if mapMarker.id == nil then
                mapMarker = nil
                break
            end
            if mapMarker.id > waypointsCount or mapMarker.id < 1 then -- invalid marker id
                AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData.readGraphFromXml invalid marker id %s ", tostring(mapMarker.id))
                return -5
            end

            mapMarker.id = mapMarker.id + ADGraphManager:getWayPointsCount()

            mapMarker.name = getXMLString(xmlFile, "placeable.AutoDrive.mapmarker.mm" .. mapMarkerCounter .. ".name")
            if mapMarker.name then
                mapMarker.name = (#ADGraphManager:getMapMarkers() + 1) .. "_" .. mapMarker.name -- add the # in front to avoid multiple same names
            else
                AutoDrive.debugMsg(nil, "ERROR AutoDrivePlaceableData.readGraphFromXml missing marker name")
                return -6
            end
--[[
            mapMarker.group = "All"
            -- make sure group existst
            if ADGraphManager:getGroupByName(mapMarker.group) == nil then
                ADGraphManager:addGroup(mapMarker.group)
            end
]]
            table.insert(AutoDrivePlaceableData.mapMarkers, mapMarker) -- collect all mapMarkers to be created, but do it only at end if everthing is fine
            mapMarker = {}
            mapMarkerCounter = mapMarkerCounter + 1
        end
        -- done loading Map Markers

        -- localization for better performances
        local tnum = tonumber
        local tbin = table.insert
        local stsp = string.split

        local tempNode = createTransformGroup("tempNode")
        link(placeable.rootNode, tempNode)
        local rx, ry, rz = getWorldRotation(placeable.rootNode)
        setRotation(tempNode, 0, ry, 0)

        for i = 1, waypointsCount do
            setTranslation(tempNode, tnum(xt[i]), 0, tnum(zt[i]))
            local pointx, pointy, pointz = getWorldTranslation(tempNode)
            local wp = {
                id = i,
                x = pointx,
                y = AutoDrive:getTerrainHeightAtWorldPos(pointx, pointz),
                z = pointz,
                out = {}, 
                incoming = {}
            }
            if ot[i] and ot[i] ~= "-1" then
                for _, out in pairs(stsp(ot[i], ",")) do
                    local num = tnum(out) and tnum(out) or 0
                    if num > 0 and num <= waypointsCount then
                        -- avoid inconsistent links
                        tbin(wp.out, num + ADGraphManager:getWayPointsCount())
                    end
                end
            end
            if it[i] and it[i] ~= "-1" then
                for _, incoming in pairs(stsp(it[i], ",")) do
                    local num = tnum(incoming) and tnum(incoming) or 0
                    if num > 0 and num <= waypointsCount then
                        -- avoid inconsistent links
                        tbin(wp.incoming, num + ADGraphManager:getWayPointsCount())
                    end
                end
            end

            local num = ft[i] and tnum(ft[i]) or 0
            if num > 0 then
                wp.flags = num
            else
                wp.flags = 0
            end

            wp.id = ADGraphManager:getWayPointsCount() + i
            table.insert(AutoDrivePlaceableData.wayPoints, wp) -- collect all waypoints to be created, but do it only at end if everthing is fine
            i = i + 1
        end
        if tempNode then
            delete(tempNode)
        end
    end

    -- user confirmed import
    if AutoDrivePlaceableData.mapMarkers and #AutoDrivePlaceableData.mapMarkers > 0
        and AutoDrivePlaceableData.wayPoints and #AutoDrivePlaceableData.wayPoints > 0 then
            AutoDrivePlaceableData:createPlaceable(AutoDrivePlaceableData.wayPoints, AutoDrivePlaceableData.mapMarkers)
    end
    return 0 -- OK
end

function AutoDrivePlaceableData:createPlaceable(wayPoints, mapMarkers, sendEvent)
    if sendEvent == nil or sendEvent == true then
        -- Propagating way point deletion all over the network
        CreatePlaceableEvent.sendEvent(wayPoints, mapMarkers)
    else
        for _, wp in pairs(wayPoints) do
            ADGraphManager:createWayPointWithConnections(wp.x, wp.y, wp.z, wp.out, wp.incoming, wp.flags, false)
        end
        for _, mapMarker in pairs(mapMarkers) do
            ADGraphManager:createMapMarker(mapMarker.id, mapMarker.name, false)
        end

        AutoDrive:notifyDestinationListeners()
    end
end
