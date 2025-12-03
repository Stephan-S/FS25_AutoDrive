--
-- Mod: AutoDrive
--
-- Author: Stephan
-- Email: Stephan910@web.de
-- Date: 17.12.2021
-- Version: 2.0.0.2

-- #############################################################################

source(Utils.getFilename("scripts/AutoDrive.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Utils/AutoDriveVehicleData.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Utils/AutoDrivePlaceableData.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Specialization.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Sync.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/XML.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Settings.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Gui.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Hud.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/ExternalInterface.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/PathCalculation.lua", g_currentModDirectory))

source(Utils.getFilename("scripts/Hud/GenericHudElement.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Hud/Hotspot.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Hud/HudButton.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Hud/HudCounterButton.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Hud/HudSettingsButton.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Hud/HudIcon.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Hud/HudSpeedmeter.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Hud/PullDownList.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Hud/HudHarvesterInfo.lua", g_currentModDirectory))

source(Utils.getFilename("scripts/Events/GroupsEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/UserDataEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/UpdateSettingsEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/RenameDriverEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/UserConnectedEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/ExperimentalFeaturesEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/MessageEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/InputEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/HudInputEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/DebugSettingsEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/StartStopEvent.lua", g_currentModDirectory))

source(Utils.getFilename("scripts/Events/Graph/CreateMapMarkerEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/Graph/DeleteMapMarkerEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/Graph/RenameMapMarkerEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/Graph/ChangeMapMarkerGroupEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/Graph/ToggleConnectionEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/Graph/DeleteWayPointsEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/Graph/CreateWayPointEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/Graph/RecordWayPointEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/Graph/MoveWayPointEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/Graph/RoutesUploadEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/Graph/SectionEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/Graph/SetConnectionEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/Graph/CreateSplineConnectionEvent.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Events/Graph/CreatePlaceableEvent.lua", g_currentModDirectory))

source(Utils.getFilename("scripts/Utils/AutoDriveTON.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Utils/TrailerUtil.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Utils/CombineUtil.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Utils/UtilFuncs.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Utils/Queue.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Utils/RingQueue.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Utils/Buffer.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Utils/FlaggedTable.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Utils/CollisionDetectionUtils.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Utils/PathFinderUtils.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Utils/Dubins.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Utils/AutoDriveUtilFuncs.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Utils/SortedQueue.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Utils/DevFuncs.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Utils/TrafficSplineUtils.lua", g_currentModDirectory))

source(Utils.getFilename("scripts/Manager/RoutesManager.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Manager/DrawingManager.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Manager/MessagesManager.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Manager/GraphManager.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Manager/TriggerManager.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Manager/HarvestManager.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Manager/InputManager.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Manager/UserDataManager.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Manager/MultipleTargetsManager.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Manager/Scheduler.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Manager/UnloadManager.lua", g_currentModDirectory))

source(Utils.getFilename("scripts/Tasks/AbstractTask.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/DriveToDestinationTask.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/StopAndDisableADTask.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/UnloadAtDestinationTask.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/LoadAtDestinationTask.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/RestartADTask.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/UnloadBGATask.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/DriveToVehicleTask.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/EmptyHarvesterTask.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/WaitForCallTask.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/ClearCropTask.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/CatchCombinePipeTask.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/FollowCombineTask.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/RefuelTask.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/ExitFieldTask.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/FollowVehicleTask.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/ReverseFromBadLocationTask.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/ParkTask.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/RepairTask.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/HandleHarvesterTurnTask.lua", g_currentModDirectory))

source(Utils.getFilename("scripts/Modules/DrivePathModule.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Modules/CollisionDetectionModule.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Modules/SpecialDrivingModule.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Modules/TaskModule.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Modules/TrailerModule.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Modules/PathFinderModule.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Modules/StateModule.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Modules/RecordingModule.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Modules/TrainModule.lua", g_currentModDirectory))

source(Utils.getFilename("scripts/Modes/AbstractMode.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Modes/DriveToMode.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Modes/UnloadAtMode.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Modes/PickupAndDeliverMode.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Modes/LoadMode.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Modes/BGAMode.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Modes/CombineUnloaderMode.lua", g_currentModDirectory))

source(Utils.getFilename("scripts/Sensors/VirtualSensors.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Sensors/CollSensor.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Sensors/CollSensorSplit.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Sensors/FruitSensor.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Sensors/FieldSensor.lua", g_currentModDirectory))

source(Utils.getFilename("scripts/Gui/RoutesManagerGUI.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Gui/NotificationsHistoryGUI.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Gui/ColorSettingsGUI.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Gui/EnterDriverNameGUI.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Gui/EnterGroupNameGUI.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Gui/EnterTargetNameGUI.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Gui/ScanConfirmationGUI.lua", g_currentModDirectory))

source(Utils.getFilename("scripts/Gui/EnterDestinationFilterGUI.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Gui/SettingsPage.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Gui/DebugSettingsPage.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Gui/Settings.lua", g_currentModDirectory))

source(Utils.getFilename("scripts/Tasks/Turns/AbstractTurn.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/Turns/ReverseOffsetTurn.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/Turns/OffsetTurn.lua", g_currentModDirectory))
source(Utils.getFilename("scripts/Tasks/Turns/ForwardOffsetTurn.lua", g_currentModDirectory))

AutoDriveRegister = {}
AutoDriveRegister.version = g_modManager:getModByName(g_currentModName).version

if AutoDrive.ADSpecName == nil then
    AutoDrive.ADSpecName = g_currentModName .. ".AutoDrive"
end

if g_specializationManager:getSpecializationByName("AutoDrive") == nil then
    g_specializationManager:addSpecialization("AutoDrive", "AutoDrive", Utils.getFilename("scripts/AutoDrive.lua", g_currentModDirectory), nil)
end

function AutoDriveRegister.registerAutoDrive()

    if AutoDrive == nil then
        Logging.error("[AD] Unable to add specialization 'AutoDrive'")
        return
    end

    for vehicleType, typeDef in pairs(g_vehicleTypeManager.types) do
        if typeDef ~= nil and vehicleType ~= "horse" and (not typeDef.hasADSpec == true) then
            if AutoDrive.prerequisitesPresent(typeDef.specializations) then
                --Logging.info('[AD] Attached to vehicleType "%s"', vehicleType)
                if typeDef.specializationsByName[AutoDrive.ADSpecName] == nil then
                    g_vehicleTypeManager:addSpecialization(vehicleType, AutoDrive.ADSpecName)
                    typeDef.hasADSpec = true
                end
            end
        end
    end
end

if AutoDrive.ADVDSpecName == nil then
    AutoDrive.ADVDSpecName = g_currentModName .. ".AutoDriveVehicleData"
end

if g_specializationManager:getSpecializationByName("AutoDriveVehicleData") == nil then
	g_specializationManager:addSpecialization("AutoDriveVehicleData", "AutoDriveVehicleData", Utils.getFilename("scripts/Utils/AutoDriveVehicleData.lua", g_currentModDirectory), nil)
end

function AutoDriveRegister.registerVehicleData()

	if AutoDriveVehicleData == nil then
		Logging.error("[AutoDriveVehicleData] Unable to add specialization 'AutoDriveVehicleData'")
		return
	end

	for vehicleType, typeDef in pairs(g_vehicleTypeManager.types) do
		if typeDef ~= nil and vehicleType ~= "horse" and (not typeDef.hasADVDSpec == true) then
			if AutoDriveVehicleData.prerequisitesPresent(typeDef.specializations) then
				if typeDef.specializationsByName[AutoDrive.ADVDSpecName] == nil then
					g_vehicleTypeManager:addSpecialization(vehicleType, AutoDrive.ADVDSpecName)
					typeDef.hasADVDSpec = true
				end
			end
		end
	end
end

if AutoDrive.ADPDSpecName == nil then
    AutoDrive.ADPDSpecName = g_currentModName .. ".AutoDrivePlaceableData"
end

if g_placeableSpecializationManager:getSpecializationByName("AutoDrivePlaceableData") == nil then
	g_placeableSpecializationManager:addSpecialization("AutoDrivePlaceableData", "AutoDrivePlaceableData", Utils.getFilename("scripts/Utils/AutoDrivePlaceableData.lua", g_currentModDirectory), nil)
end

function AutoDriveRegister.registerPlaceableData()

	if AutoDriveVehicleData == nil then
		Logging.error("[AutoDriveVehicleData] Unable to add specialization 'AutoDriveVehicleData'")
		return
	end

    local validPlaceableTypes =
    {
        "baseHusbandry",
        "beehive",
        "beehivePalletSpawner",
        "bunkerSilo",
        "buyingStation",
        "buyingStationManure",
        "chargingStation",
        "chickenHusbandry",
        "constructible",
        "constructibleFarmhouse",
        "cowHusbandry",
        "cowHusbandryBarn",
        "cowHusbandryBarnMilk",
        "cowHusbandryBarnMilkFeedingRobot",
        "cowHusbandryPasture",
        "cowHusbandryPastureStraw",
        "decoObject",
        "destructible",
        "factory",
        "farmhouse",
        "garageSolarPanels",
        "garageWorkshop",
        "garageWorkshopSolarPanels",
        "greenhouse",
        "heapSpawner",
        "horseHusbandry",
        "horseHusbandryPasture",
        "manureHeap",
        "objectStorage",
        "pigHusbandry",
        "pigHusbandryPasture",
        "productionPoint",
        "sellingStation",
        "sheepHusbandry",
        "silo",
        "siloExtension",
        "simplePlaceable",
        "toolShed",
        "warehouse",
        "weighingStation",
        "workshop"
    }

    local count = 0
	for placeableType, typeDef in pairs(g_placeableTypeManager.types) do
        for _, validPlaceableType in pairs(validPlaceableTypes) do
            if placeableType == validPlaceableType and (not typeDef.hasADPDpec == true) then
                if AutoDrivePlaceableData.prerequisitesPresent(typeDef.specializations) then
                    if typeDef.specializationsByName[AutoDrive.ADPDSpecName] == nil then
                        g_placeableTypeManager:addSpecialization(placeableType, AutoDrive.ADPDSpecName)
                        typeDef.hasADPDpec = true
                        count = count + 1
                        break
                    end
                end
            end
        end
    end
    Logging.info("AutoDrive registered %d placeable Types.", count)
end

-- We need this for network debug functions
EventIds.eventIdToName = {}

for eName, eId in pairs(EventIds) do
	if string.sub(eName, 1, 6) == "EVENT_" then
		EventIds.eventIdToName[eId] = eName
	end
end

function AutoDriveRegister:loadMap(name)
	Logging.info("[AutoDrive] Loaded mod version %s (by Stephan). Full version number: %s", self.version, AutoDrive.version)
end

function AutoDriveRegister:deleteMap()
end

function AutoDriveRegister:keyEvent(unicode, sym, modifier, isDown)
end

function AutoDriveRegister:mouseEvent(posX, posY, isDown, isUp, button)
end

function AutoDriveRegister:update(dt)
end

function AutoDriveRegister:draw()
end

--Knowledge to register translations in l10n space and to use the helpLineManager taken from the Seasons mod (Thank you!)
function AutoDriveRegister.onMissionWillLoad(i18n)
	AutoDriveRegister.addModTranslations(i18n)
end

function AutoDriveValidateVehicleTypes(TypeManager)
	AutoDriveRegister.onMissionWillLoad(g_i18n)
end

---Copy our translations to global space.
function AutoDriveRegister.addModTranslations(i18n)
	-- We can copy all our translations to the global table because we prefix everything with ad_ or have unique names with 'AD' in it.
	-- The mod-based l10n lookup only really works for vehicles, not UI and script mods.
	local global = getfenv(0).g_i18n.texts

	for key, text in pairs(i18n.texts) do
		global[key] = text
	end
end

function AutoDriveLoadedMission(mission, superFunc, node)
	superFunc(mission, node)

	if mission.cancelLoading then
		return
	end
end

local function check()
    if AutoDrive == nil then
        Logging.error("[AD] AutoDriveRegister.check Unable to add specialization 'AutoDrive'")
        return
    end

	local mods = g_modManager:getActiveMods()
	local foundMod = false

    if mods then
        for _, activemod in pairs(mods) do
            if activemod.title == "AutoDrive" then
                foundMod = true
                break
            end
        end
    end

    if g_currentModName then
        if g_currentModName == "FS25_AutoDrive" and foundMod then
            if AutoDrive.currentModName == nil then
                AutoDrive.currentModName = g_currentModName

                -- Mission00.loadMission00Finished = Utils.overwrittenFunction(Mission00.loadMission00Finished, AutoDriveLoadedMission)
                TypeManager.validateTypes = Utils.prependedFunction(TypeManager.validateTypes, AutoDriveValidateVehicleTypes)
                addModEventListener(AutoDriveRegister)
                addModEventListener(AutoDrive)
                -- first iteration to register AD to vehicle types
                AutoDriveRegister.registerAutoDrive()
                AutoDriveRegister.registerVehicleData()
                AutoDriveRegister.registerPlaceableData()
            end
        else
            Logging.error("[AD] ERROR: AutoDriveRegister.check g_currentModName %s", tostring(g_currentModName))
            local args = {text = "AutoDrive: Wrong mod file name / Falscher Dateiname " .. g_currentModName .. " <--> FS25_AutoDrive or mod corrupt / oder Mod defekt"}
            local dialog = g_gui:showDialog("InfoDialog")
            if dialog then
                dialog.target:setDialogType(Utils.getNoNil(args.dialogType, DialogElement.TYPE_WARNING))
                dialog.target:setText(args.text)
                dialog.target:setCallback(args.callback, args.target, args.args)
                dialog.target:setButtonTexts(args.okText)
                dialog.target:setButtonAction(args.buttonAction)
            end
            return
        end
    end
end

check()