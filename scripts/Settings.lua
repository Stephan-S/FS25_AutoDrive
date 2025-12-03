--[[
values - values which are used as return by getSetting with index current
current - current index of values which is in use
default - index used by first use of AD or invoked by settings screen default button
texts - showed in the settings screen selections, translated to current language
translate - enable translation of the texts, if not set true no translation will be applied
tooltip - help text shown in the settings screen
isVehicleSpecific - this setting is specific for the current vehicle only
isUserSpecific - this setting is specific for the current user only
if isVehicleSpecific is false or nil and isUserSpecific is false or nil the setting is global
shallNotBeSaved - this setting is only valid during runtime and will not be saved
]]

AutoDrive.settings = {}

AutoDrive.settings.blinkValue = {
    values = {
        0,
        10,
        20,
        30,
        40,
        50,
        60,
        70,
        80,
        90,
        100,
        110,
        120,
        130,
        140,
        150,
        160
    },
    texts = {
        "0",
        "10",
        "20",
        "30",
        "40",
        "50",
        "60",
        "70",
        "80",
        "90",
        "100",
        "110",
        "120",
        "130",
        "140",
        "150",
        "160"
    },
    default = 1,
    current = 1,
    text = "gui_ad_blinkValue",
    tooltip = "gui_ad_blinkValue_tooltip",
    translate = false,
    isVehicleSpecific = false
}

AutoDrive.settings.collisionHeigth = {
    values = {
        0.25,
        0.5,
        0.75,
        1.0,
        1.25,
        1.5,
        1.75,
        2.0,
        2.25,
        2.5,
        2.75,
        3.0,
        3.25,
        3.5,
        3.75,
        4.0
    },
    texts = {
        "0.25 m",
        "0.5 m",
        "0.75 m",
        "1.0 m",
        "1.25 m",
        "1.5 m",
        "1.75 m",
        "2.0 m",
        "2.25 m",
        "2.5 m",
        "2.75 m",
        "3.0 m",
        "3.25 m",
        "3.5 m",
        "3.75 m",
        "4.0 m"
    },
    default = 2,
    current = 2,
    text = "gui_ad_collisionHeigth",
    tooltip = "gui_ad_collisionHeigth_tooltip",
    translate = false,
    isVehicleSpecific = false
}


AutoDrive.settings.pipeOffset = {
    values = {
        -5.0,
        -4.75,
        -4.5,
        -4.25,
        -4.0,
        -3.75,
        -3.5,
        -3.25,
        -3.0,
        -2.75,
        -2.5,
        -2.25,
        -2.0,
        -1.75,
        -1.5,
        -1.25,
        -1.0,
        -0.95,
        -0.9,
        -0.85,
        -0.8,
        -0.75,
        -0.7,
        -0.65,
        -0.6,
        -0.55,
        -0.5,
        -0.45,
        -0.4,
        -0.35,
        -0.3,
        -0.25,
        -0.2,
        -0.15,
        -0.1,
        -0.05,
        0,
        0.05,
        0.1,
        0.15,
        0.2,
        0.25,
        0.3,
        0.35,
        0.4,
        0.45,
        0.5,
        0.55,
        0.6,
        0.65,
        0.7,
        0.75,
        0.8,
        0.85,
        0.9,
        0.95,
        1.0,
        1.05,
        1.10,
        1.15,
        1.20,
        1.25,
        1.30,
        1.35,
        1.40,
        1.45,
        1.5,
        1.55,
        1.60,
        1.65,
        1.70,
        1.75,
        2.0,
        2.25,
        2.5,
        2.75,
        3.0,
        3.25,
        3.5,
        3.75,
        4.0,
        4.25,
        4.5,
        4.75,
        5.0
    },
    texts = {
        "-5.0m",
        "-4.75m",
        "-4.5m",
        "-4.25m",
        "-4.0m",
        "-3.75m",
        "-3.5m",
        "-3.25m",
        "-3.0m",
        "-2.75m",
        "-2.5m",
        "-2.25m",
        "-2.0m",
        "-1.75m",
        "-1.5m",
        "-1.25m",
        "-1.0m",
        "-0.95m",
        "-0.9m",
        "-0.85m",
        "-0.8m",
        "-0.75m",
        "-0.7m",
        "-0.65m",
        "-0.6m",
        "-0.55m",
        "-0.5m",
        "-0.45m",
        "-0.4m",
        "-0.35m",
        "-0.3m",
        "-0.25m",
        "-0.2m",
        "-0.15m",
        "-0.1m",
        "-0.05m",
        "0 m",
        "0.05 m",
        "0.1 m",
        "0.15 m",
        "0.2 m",
        "0.25 m",
        "0.3 m",
        "0.35 m",
        "0.4 m",
        "0.45 m",
        "0.5 m",
        "0.55 m",
        "0.6 m",
        "0.65 m",
        "0.7 m",
        "0.75 m",
        "0.8 m",
        "0.85 m",
        "0.9 m",
        "0.95 m",
        "1.0 m",
        "1.05 m",
        "1.10 m",
        "1.15 m",
        "1.20 m",
        "1.25 m",
        "1.30 m",
        "1.35 m",
        "1.40 m",
        "1.45 m",
        "1.5 m",
        "1.55 m",
        "1.60 m",
        "1.65 m",
        "1.70 m",
        "1.75 m",
        "2.0 m",
        "2.25 m",
        "2.5 m",
        "2.75 m",
        "3.0 m",
        "3.25 m",
        "3.5 m",
        "3.75 m",
        "4.0 m",
        "4.25 m",
        "4.5 m",
        "4.75 m",
        "5.0 m"
    },
    default = 41,
    current = 41,
    text = "gui_ad_pipe_offset",
    tooltip = "gui_ad_pipe_offset_tooltip",
    translate = false,
    isVehicleSpecific = true
}

AutoDrive.settings.followDistance = {
    values = {
        0,
        0.25,
        0.5,
        0.75,
        1.0,
        1.25,
        1.5,
        1.75,
        2.0,
        2.25,
        2.5,
        2.75,
        3.0,
        3.25,
        3.5,
        3.75,
        4.0,
        4.25,
        4.5,
        4.75,
        5.0,
        5.25,
        5.5,
        5.75,
        6.0,
        6.25,
        6.5,
        6.75,
        7.0,
        7.25,
        7.5,
        7.75,
        8.0
    },
    texts = {
        "0 m",
        "0.25 m",
        "0.5 m",
        "0.75 m",
        "1.0 m",
        "1.25 m",
        "1.5 m",
        "1.75 m",
        "2.0 m",
        "2.25 m",
        "2.5 m",
        "2.75 m",
        "3.0 m",
        "3.25 m",
        "3.5 m",
        "3.75 m",
        "4.0 m",
        "4.25 m",
        "4.5 m",
        "4.75 m",
        "5.0 m",
        "5.25 m",
        "5.5 m",
        "5.75 m",
        "6.0 m",
        "6.25 m",
        "6.5 m",
        "6.75 m",
        "7.0 m",
        "7.25 m",
        "7.5 m",
        "7.75 m",
        "8.0 m"
    },
    default = 8,
    current = 8,
    text = "gui_ad_followDistance",
    tooltip = "gui_ad_followDistance_tooltip",
    translate = false,
    isVehicleSpecific = true
}

AutoDrive.settings.lookAheadTurning = {
    values = {4, 5, 6, 7, 8, 10, 12},
    texts = {"4 m", "5 m", "6 m", "7 m", "8 m", "10 m", "12 m"},
    default = 5,
    current = 5,
    text = "gui_ad_lookahead_turning",
    tooltip = "gui_ad_lookahead_turning_tooltip",
    translate = false,
    isVehicleSpecific = false
}

AutoDrive.settings.mapMarkerDetour = {
    values = {0, 10, 50, 100, 200, 300, 500, 1000, 10000},
    texts = {"0m", "10m", "50m", "100m", "200m", "500m", "1000m", "10000m"},
    default = 1,
    current = 1,
    text = "gui_ad_mapMarkerDetour",
    tooltip = "gui_ad_mapMarkerDetour_tooltip",
    translate = false,
    isVehicleSpecific = false
}

AutoDrive.settings.continueOnEmptySilo = {
    values = {false, true},
    texts = {"gui_ad_wait", "gui_ad_drive"},
    default = 1,
    current = 1,
    text = "gui_ad_siloEmpty",
    tooltip = "gui_ad_siloEmpty_tooltip",
    translate = true,
    isVehicleSpecific = false
}

AutoDrive.settings.autoConnectStart = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 2,
    current = 2,
    text = "gui_ad_autoConnect_start",
    tooltip = "gui_ad_autoConnect_start_tooltip",
    translate = true,
    isVehicleSpecific = false
}

AutoDrive.settings.autoConnectEnd = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 1,
    current = 1,
    text = "gui_ad_autoConnect_end",
    tooltip = "gui_ad_autoConnect_end_tooltip",
    translate = true,
    isVehicleSpecific = false
}

AutoDrive.settings.parkInField = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 2,
    current = 2,
    text = "gui_ad_parkInField",
    tooltip = "gui_ad_parkInField_tooltip",
    translate = true,
    isVehicleSpecific = true
}

AutoDrive.settings.unloadFillLevel = {
    values = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.85, 0.90, 0.95, 0.99, 1},
    texts = {"0%", "10%", "20%", "30%", "40%", "50%", "60%", "70%", "80%", "85%", "90%", "95%", "99%", "100%"},
    default = 10,
    current = 10,
    text = "gui_ad_unloadFillLevel",
    tooltip = "gui_ad_unloadFillLevel_tooltip",
    translate = false,
    isVehicleSpecific = true
}

AutoDrive.settings.findDriver = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 1,
    current = 1,
    text = "gui_ad_findDriver",
    tooltip = "gui_ad_findDriver_tooltip",
    translate = true,
    isVehicleSpecific = false
}

AutoDrive.settings.guiScale = {
    values = {0, 0.4,0.45,0.5,0.55,0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95,1,1.05,1.1,1.15,1.2,1.25,1.3,1.35,1.4,1.45,1.5,1.55,1.6,1.65,1.7,1.75,1.8,1.85,1.9,1.95,2},
    texts = {
        "Default",
        "40%",
        "45%",
        "50%",
        "55%",
        "60%",
        "65%",
        "70%",
        "75%",
        "80%",
        "85%",
        "90%",
        "95%",
        "100%",
        "105%",
        "110%",
        "115%",
        "120%",
        "125%",
        "130%",
        "135%",
        "140%",
        "145%",
        "150%",
        "155%",
        "160%",
        "165%",
        "170%",
        "175%",
        "180%",
        "185%",
        "190%",
        "195%",
        "200%"
    },
    default = 1,
    current = 1,
    text = "gui_ad_gui_scale",
    tooltip = "gui_ad_gui_scale_tooltip",
    translate = false,
    isVehicleSpecific = false,
    isUserSpecific = true
}

AutoDrive.settings.notifications = {
    values = {0, 0.5, 1, 2, 5, math.huge},
    texts = {"gui_ad_notifications_text_1", "gui_ad_notifications_text_2", "gui_ad_notifications_text_3", "gui_ad_notifications_text_4", "gui_ad_notifications_text_5", "gui_ad_notifications_text_6"},
    default = 3,
    current = 3,
    text = "gui_ad_notifications",
    tooltip = "gui_ad_notifications_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = true
}

AutoDrive.settings.exitField = {
    values = {0, 1, 2},
    texts = {"gui_ad_default", "gui_ad_after_start", "gui_ad_closest"},
    default = 1,
    current = 1,
    text = "gui_ad_exitField",
    tooltip = "gui_ad_exitField_tooltip",
    translate = true,
    isVehicleSpecific = true
}

AutoDrive.settings.showHelp = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 2,
    current = 2,
    text = "gui_ad_showHelp",
    tooltip = "gui_ad_showHelp_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = true
}

AutoDrive.settings.driverWages = {
    values = {0, 0.5, 1, 2.5, 5.0, 10.0},
    texts = {"0%", "50%", "100%", "250%", "500%", "1000%"},
    default = 3,
    current = 3,
    text = "gui_ad_driverWages",
    tooltip = "gui_ad_driverWages_tooltip",
    translate = false,
    isVehicleSpecific = false
}

AutoDrive.settings.showNextPath = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 2,
    current = 2,
    text = "gui_ad_showNextPath",
    tooltip = "gui_ad_showNextPath_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = true
}

AutoDrive.settings.avoidFruit = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 2,
    current = 2,
    text = "gui_ad_avoidFruit",
    tooltip = "gui_ad_avoidFruit_tooltip",
    translate = true,
    isVehicleSpecific = true
}

AutoDrive.settings.pathFinderTime = {
    values = {0.25, 0.5, 1.0, 1.5, 2, 3},
    texts = {"x0.25", "x0.5", "x1.0", "x1.5", "x2", "x3"},
    default = 2,
    current = 2,
    text = "gui_ad_pathFinderTime",
    tooltip = "gui_ad_pathFinderTime_tooltip",
    translate = false,
    isVehicleSpecific = false
}

AutoDrive.settings.lineHeight = {
    values = {0, 4},
    texts = {"gui_ad_ground", "gui_ad_aboveDriver"},
    default = 1,
    current = 1,
    text = "gui_ad_lineHeight",
    tooltip = "gui_ad_lineHeight_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = true
}

AutoDrive.settings.enableTrafficDetection = {
    values = {0, 1},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 1,
    current = 1,
    text = "gui_ad_enableTrafficDetection",
    tooltip = "gui_ad_enableTrafficDetection_tooltip",
    translate = true,
    isVehicleSpecific = false
}

AutoDrive.settings.shovelWidth = {
    values = {0, 0.2, 0.4, 0.6, 0.8, 1, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2, 3.4, 3.6, 3.8, 4.0},
    texts = {"0m", "0.2m", "0.4m", "0.6m", "0.8m", "1.0m", "1.2m", "1.4m", "1.6m", "1.8m", "2.0m", "2.2m", "2.4m", "2.6m", "2.8m", "3.0m", "3.2m", "3.4m", "3.6m", "3.8m", "4.0m"},
    default = 1,
    current = 1,
    text = "gui_ad_shovelWidth",
    tooltip = "gui_ad_shovelWidth_tooltip",
    translate = false,
    isVehicleSpecific = true
}


AutoDrive.settings.shovelHeight = {
    values = {
        -0.5,
        -0.48,
        -0.46,
        -0.44,
        -0.42,
        -0.40,
        -0.38,
        -0.36,
        -0.34,
        -0.32,
        -0.3,
        -0.28,
        -0.26,
        -0.24,
        -0.22,
        -0.20,
        -0.18,
        -0.16,
        -0.14,
        -0.12,
        -0.10,
        -0.08,
        -0.06,
        -0.04,
        -0.02,
        0,
        0.02,
        0.04,
        0.06,
        0.08,
        0.1,
        0.12,
        0.14,
        0.16,
        0.18,
        0.20,
        0.22,
        0.24,
        0.26,
        0.28,
        0.3,
        0.32,
        0.34,
        0.36,
        0.38,
        0.40,
        0.42,
        0.44,
        0.46,
        0.48,
        0.5
    },
    texts = {
        "-50cm",
        "-48cm",
        "-46cm",
        "-44cm",
        "-42cm",
        "-40cm",
        "-38cm",
        "-36cm",
        "-34cm",
        "-32cm",
        "-30cm",
        "-28cm",
        "-26cm",
        "-24cm",
        "-22cm",
        "-20cm",
        "-18cm",
        "-16cm",
        "-14cm",
        "-12cm",
        "-10cm",
        "-8cm",
        "-6cm",
        "-4cm",
        "-2cm",
        "0cm",
        "2cm",
        "4cm",
        "6cm",
        "8cm",
        "10cm",
        "12cm",
        "14cm",
        "16cm",
        "18cm",
        "20cm",
        "22cm",
        "24cm",
        "26cm",
        "28cm",
        "30cm",
        "32cm",
        "34cm",
        "36cm",
        "38cm",
        "40cm",
        "42cm",
        "44cm",
        "46cm",
        "48cm",
        "50cm"
    },
    default = 26,
    current = 26,
    text = "gui_ad_shovelHeight",
    tooltip = "gui_ad_shovelHeight_tooltip",
    translate = false,
    isVehicleSpecific = true
}

AutoDrive.settings.useFolders = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 1,
    current = 1,
    text = "gui_ad_useFolders",
    tooltip = "gui_ad_useFolders_tooltip",
    translate = true,
    isVehicleSpecific = false
}

AutoDrive.settings.preCallLevel = {
    values = {0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.85, 0.90, 1},
    texts = {"0%", "10%", "20%", "30%", "40%", "50%", "60%", "70%", "80%", "85%", "90%", "100%"},
    default = 7,
    current = 7,
    text = "gui_ad_preCallLevel",
    tooltip = "gui_ad_preCallLevel_tooltip",
    translate = false,
    isVehicleSpecific = true
}

AutoDrive.settings.chaseSide = {
    values = {AutoDrive.CHASEPOS_AUTO, AutoDrive.CHASEPOS_LEFT, AutoDrive.CHASEPOS_REAR, AutoDrive.CHASEPOS_RIGHT},
    texts = {"Auto", "AD_task_chase_side_left", "AD_task_chase_side_rear", "AD_task_chase_side_right"},
    default = 1,
    current = 1,
    text = "gui_ad_chaseSide",
    tooltip = "gui_ad_chaseSide_tooltip",
    translate = true,
    isVehicleSpecific = true
}

AutoDrive.settings.rotateTargets = {
    values = {AutoDrive.RT_NONE, AutoDrive.RT_ONLYPICKUP, AutoDrive.RT_ONLYDELIVER, AutoDrive.RT_PICKUPANDDELIVER},
    texts = {"gui_ad_none", "gui_ad_onlyPickup", "gui_ad_onlyDeliver", "gui_ad_PickupandDeliver"},
    default = 1,
    current = 1,
    text = "gui_ad_rotateTargets",
    tooltip = "gui_ad_rotateTargets_tooltip",
    translate = true,
    isVehicleSpecific = true
}

AutoDrive.settings.maxTriggerDistance = {
    values = {10, 25, 50, 100, 200},
    texts = {"10 m", "25 m", "50 m", "100 m", "200 m"},
    default = 2,
    current = 2,
    text = "gui_ad_maxTriggerDistance",
    tooltip = "gui_ad_maxTriggerDistance_tooltip",
    translate = false,
    isVehicleSpecific = false
}

AutoDrive.settings.maxTriggerDistanceVehicle = {
    values = {0, 10, 25, 50, 100, 200},
    texts = {"gui_ad_useGlobalSetting", "10 m", "25 m", "50 m", "100 m", "200 m"},
    default = 0,
    current = 0,
    text = "gui_ad_maxTriggerDistance",
    tooltip = "gui_ad_maxTriggerDistance_tooltip",
    translate = true,
    isVehicleSpecific = true,
}


AutoDrive.settings.useBeaconLights = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 1,
    current = 1,
    text = "gui_ad_useBeaconLights",
    tooltip = "gui_ad_useBeaconLights_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = false
}

AutoDrive.settings.activeUnloading = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 2,
    current = 2,
    text = "gui_ad_activeUnloading",
    tooltip = "gui_ad_activeUnloading_tooltip",
    translate = true,
    isVehicleSpecific = true
}

AutoDrive.settings.restrictToField = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 1,
    current = 1,
    text = "gui_ad_restrictToField",
    tooltip = "gui_ad_restrictToField_tooltip",
    translate = true,
    isVehicleSpecific = true
}

AutoDrive.settings.showTooltips = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 2,
    current = 2,
    text = "gui_ad_showTooltips",
    tooltip = "gui_ad_showTooltips_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = true
}

AutoDrive.settings.autoRefuel = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 2,
    current = 2,
    text = "gui_ad_autoRefuel",
    tooltip = "gui_ad_autoRefuel_tooltip",
    translate = true,
    isVehicleSpecific = true
}

AutoDrive.settings.autoRepair = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 2,
    current = 2,
    text = "gui_ad_autoRepair",
    tooltip = "gui_ad_autoRepair_tooltip",
    translate = true,
    isVehicleSpecific = true
}

AutoDrive.settings.showMarkersOnMap = {
    values = {0x00, 0x01, 0x02, 0x03},
    texts = {"gui_ad_no", "gui_ad_mainMenuMap", "gui_ad_ingameMap", "gui_ad_both"},
    default = 2,
    current = 2,
    text = "gui_ad_showMarkersOnMap",
    tooltip = "gui_ad_showMarkersOnMap_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = true
}

AutoDrive.settings.switchToMarkersOnMap = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 1,
    current = 1,
    text = "gui_ad_switchToMarkersOnMap",
    tooltip = "gui_ad_switchToMarkersOnMap_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = true
}

AutoDrive.settings.cornerSpeed = {
    values = {0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0, 1.05, 1.1, 1.15, 1.2, 1.25, 1.3, 1.35, 1.4, 1.45, 1.5, 1.55, 1.6, 1.65, 1.7, 1.75, 1.8, 1.85, 1.9, 2.0},
    texts = {"50%", "55%", "60%", "65%", "70%", "75%", "80%", "85%", "90%", "95%", "100%", "105%", "110%", "115%", "120%", "125%", "130%", "135%", "140%", "145%", "150%", "155%", "160%", "165%", "170%", "175%", "180%", "185%", "190%", "195%", "200%"},
    default = 11,
    current = 11,
    text = "gui_ad_cornerSpeed",
    tooltip = "gui_ad_cornerSpeed_tooltip",
    translate = false,
    isVehicleSpecific = true
}

AutoDrive.settings.callSecondUnloader = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 1,
    current = 1,
    text = "gui_ad_callSecondUnloader",
    tooltip = "gui_ad_callSecondUnloader_tooltip",
    translate = true,
    isVehicleSpecific = true
}

AutoDrive.settings.followOnlyOnField = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 1,
    current = 1,
    text = "gui_ad_followOnlyOnField",
    tooltip = "gui_ad_followOnlyOnField_tooltip",
    translate = true,
    isVehicleSpecific = true
}

AutoDrive.settings.addSettingsToHUD = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 1,
    current = 1,
    text = "gui_ad_addSettingsToHUD",
    tooltip = "gui_ad_addSettingsToHUD_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = true
}

AutoDrive.settings.iconSetToUse = {
    values = {1, 2, 3},
    texts = {"AutoDrive", "Hirschfeld", "Holger"},
    default = 1,
    current = 1,
    text = "gui_ad_iconSetToUse",
    tooltip = "gui_ad_iconSetToUse_tooltip",
    translate = false,
    isVehicleSpecific = false,
    isUserSpecific = true
}

AutoDrive.settings.wideHUD = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 1,
    current = 1,
    text = "gui_ad_wideHUD",
    tooltip = "gui_ad_wideHUD_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = true
}

AutoDrive.settings.showHUD = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 1,
    current = 1,
    text = "",
    tooltip = "",
    translate = false,
    isVehicleSpecific = false,
    isUserSpecific = true
}

AutoDrive.settings.EditorMode = {
    values = {1, 2, 3, 4},
    texts = {"EDITOR_OFF", "EDITOR_ON", "EDITOR_EXTENDED", "EDITOR_SHOW"},
    default = 1,
    current = 1,
    text = "",
    tooltip = "",
    translate = false,
    isVehicleSpecific = false,
    isUserSpecific = true
}

AutoDrive.settings.enableParkAtJobFinished = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 1,
    current = 1,
    text = "gui_ad_enableParkAtJobFinished",
    tooltip = "gui_ad_enableParkAtJobFinished_tooltip",
    translate = true,
    isVehicleSpecific = true
}

AutoDrive.settings.autoTipSide = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 2,
    current = 2,
    text = "gui_ad_autoTipSide",
    tooltip = "gui_ad_autoTipSide_tooltip",
    translate = true,
    isVehicleSpecific = true
}

AutoDrive.settings.autoTrailerCover = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 2,
    current = 2,
    text = "gui_ad_autoTrailerCover",
    tooltip = "gui_ad_autoTrailerCover_tooltip",
    translate = true,
    isVehicleSpecific = true
}

AutoDrive.settings.ALUnload = {
    values = {0, 1, 2, 3, 4},
    texts = {"gui_ad_AL_off", "gui_ad_AL_center", "gui_ad_AL_left", "gui_ad_AL_behind", "gui_ad_AL_right"},
    default = 1,
    current = 1,
    text = "gui_ad_ALUnload",
    tooltip = "gui_ad_ALUnload_tooltip",
    translate = true,
    isVehicleSpecific = true
}
AutoDrive.settings.ALUnloadWaitTime = {
    values = {0, 1000, 3000, 5000, 10000, 15000, 20000, 25000, 30000, 60000, 120000, 300000, 600000},
    texts = {"0", "1s", "3s", "5s", "10s", "15s", "20s", "25s", "30s", "1min", "2min", "5min", "10min"},
    default = 1,
    current = 1,
    text = "gui_ad_ALUnloadWaitTime",
    tooltip = "gui_ad_ALUnloadWaitTime_tooltip",
    translate = false,
    isVehicleSpecific = true
}

AutoDrive.settings.playSounds = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 2,
    current = 2,
    text = "gui_ad_playSounds",
    tooltip = "gui_ad_playSounds_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = true
}

AutoDrive.settings.useWorkLightsLoading = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 2,
    current = 2,
    text = "gui_ad_worklightsWhenLoading",
    tooltip = "gui_ad_worklightsWhenLoading_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = false
}

AutoDrive.settings.useWorkLightsSilo = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 2,
    current = 2,
    text = "gui_ad_worklightsWhenSilo",
    tooltip = "gui_ad_worklightsWhenSilo_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = false
}

AutoDrive.settings.useHazardLightReverse = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 2,
    current = 2,
    text = "gui_ad_hazardLightReverse",
    tooltip = "gui_ad_hazardLightReverse_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = false
}

AutoDrive.settings.scaleLines = {
    values = {0.5, 1, 2, 3, 4, 5, 6, 10, 20, 30, 50, 100},
    texts = {"50%", "100%", "200%", "300%", "400%", "500%", "600%", "1000%", "2000%", "3000%", "5000%", "10000%"},
    default = 2,
    current = 2,
    text = "gui_ad_scaleLines",
    tooltip = "gui_ad_scaleLines_tooltip",
    translate = false,
    isVehicleSpecific = false,
    isUserSpecific = true
}

AutoDrive.settings.scaleMarkerText = {
    values = {0.5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10},
    texts = {"50%", "100%", "200%", "300%", "400%", "500%", "600%", "700%", "800%", "900%", "1000%"},
    default = 2,
    current = 2,
    text = "gui_ad_scaleMarkerText",
    tooltip = "gui_ad_scaleMarkerText_tooltip",
    translate = false,
    isVehicleSpecific = false,
    isUserSpecific = true
}

AutoDrive.settings.remainingDriveTimeInterval = {
    values = {0, 1, 3, 5, 10, 20, 30, 60},
    texts = {"gui_ad_off", "1s", "3s", "5s", "10s", "20s", "30s", "60s"},
    default = 5,
    current = 5,
    text = "gui_ad_remainingDriveTimeInterval",
    tooltip = "gui_ad_remainingDriveTimeInterval_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = false
}

AutoDrive.settings.UMRange = {
    values = { 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 120, 140, 160, 180, 200, 250, 300, 400, 500},
    texts = { "gui_ad_off", "10", "20", "30", "40", "50", "60", "70", "80", "90", "100", "120", "140", "160", "180", "200", "250", "300", "400", "500"},
    default = 1,
    current = 1,
    text = "gui_ad_UMRange",
    tooltip = "gui_ad_UMRange_tooltip",
    translate = true,
    isVehicleSpecific = false
}

AutoDrive.settings.Pathfinder = {
    values = {0, 1},
    texts = {"gui_ad_pathfinder_custom", "gui_ad_pathfinder_astar"},
    default = 2,
    current = 2,
    text = "gui_ad_pathfinder",
    tooltip = "gui_ad_pathfinder_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = false
}

AutoDrive.settings.enableRoutesManagerOnDediServer = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 1,
    current = 1,
    text = "gui_ad_enableRoutesManagerOnDediServer",
    tooltip = "gui_ad_enableRoutesManagerOnDediServer_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = false
}

AutoDrive.settings.autostartHelpers = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 1,
    current = 1,
    text = "gui_ad_autostartHelpers",
    tooltip = "gui_ad_autostartHelpers_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = false
}

AutoDrive.settings.detectSwath = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 2,
    current = 2,
    text = "gui_ad_detectSwath",
    tooltip = "gui_ad_detectSwath_tooltip",
    translate = true,
    isVehicleSpecific = true,
    isUserSpecific = false
}

AutoDrive.settings.colorAssignmentMode = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 1,
    current = 1,
    text = "gui_ad_colorAssignmentMode",
    tooltip = "gui_ad_colorAssignmentMode_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = true
}

AutoDrive.settings.FoldImplements = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 1,
    current = 1,
    text = "gui_ad_FoldImplements",
    tooltip = "gui_ad_FoldImplements_tooltip",
    translate = true,
    isVehicleSpecific = true,
    isUserSpecific = false
}

AutoDrive.settings.RecordWhileNotInVehicle = {
    values = {false, true},
    texts = {"gui_ad_no", "gui_ad_yes"},
    default = 1,
    current = 1,
    text = "gui_ad_RecordWhileNotInVehicle",
    tooltip = "gui_ad_RecordWhileNotInVehicle_tooltip",
    translate = true,
    isVehicleSpecific = false,
    isUserSpecific = true,
    shallNotBeSaved = true
}

AutoDrive.settings.RecordDriveDirectionOffset = {
    values = {
        -6.0,
        -5.9,
        -5.8,
        -5.7,
        -5.6,
        -5.5,
        -5.4,
        -5.3,
        -5.2,
        -5.1,
        -5.0,
        -4.9,
        -4.8,
        -4.7,
        -4.6,
        -4.5,
        -4.4,
        -4.3,
        -4.2,
        -4.1,
        -4.0,
        -3.9,
        -3.8,
        -3.7,
        -3.6,
        -3.5,
        -3.4,
        -3.3,
        -3.2,
        -3.1,
        -3.0,
        -2.9,
        -2.8,
        -2.7,
        -2.6,
        -2.5,
        -2.4,
        -2.3,
        -2.2,
        -2.1,
        -2.0,
        -1.9,
        -1.8,
        -1.7,
        -1.6,
        -1.5,
        -1.4,
        -1.3,
        -1.2,
        -1.1,
        -1.0,
        -0.9,
        -0.8,
        -0.7,
        -0.6,
        -0.5,
        -0.4,
        -0.3,
        -0.2,
        -0.1,
        0,
        0.1,
        0.2,
        0.3,
        0.4,
        0.5,
        0.6,
        0.7,
        0.8,
        0.9,
        1.0,
        1.1,
        1.2,
        1.3,
        1.4,
        1.5,
        1.6,
        1.7,
        1.8,
        1.9,
        2.0,
        2.1,
        2.2,
        2.3,
        2.4,
        2.5,
        2.6,
        2.7,
        2.8,
        2.9,
        3.0,
        3.1,
        3.2,
        3.3,
        3.4,
        3.5,
        3.6,
        3.7,
        3.8,
        3.9,
        4.0,
        4.1,
        4.2,
        4.3,
        4.4,
        4.5,
        4.6,
        4.7,
        4.8,
        4.9,
        5.0,
		5.1,
        5.2,
        5.3,
        5.4,
        5.5,
        5.6,
        5.7,
        5.8,
        5.9,
        6.0
    },
    texts = {
		"-6.0 m",
        "-5.9 m",
        "-5.8 m",
        "-5.7 m",
        "-5.6 m",
        "-5.5 m",
        "-5.4 m",
        "-5.3 m",
        "-5.2 m",
        "-5.1 m",
        "-5.0 m",
        "-4.9 m",
        "-4.8 m",
        "-4.7 m",
        "-4.6 m",
        "-4.5 m",
        "-4.4 m",
        "-4.3 m",
        "-4.2 m",
        "-4.1 m",
        "-4.0 m",
        "-3.9 m",
        "-3.8 m",
        "-3.7 m",
        "-3.6 m",
        "-3.5 m",
        "-3.4 m",
        "-3.3 m",
        "-3.2 m",
        "-3.1 m",
        "-3.0 m",
        "-2.9 m",
        "-2.8 m",
        "-2.7 m",
        "-2.6 m",
        "-2.5 m",
        "-2.4 m",
        "-2.3 m",
        "-2.2 m",
        "-2.1 m",
        "-2.0 m",
        "-1.9 m",
        "-1.8 m",
        "-1.7 m",
        "-1.6 m",
        "-1.5 m",
        "-1.4 m",
        "-1.3 m",
        "-1.2 m",
        "-1.1 m",
        "-1.0 m",
        "-0.9 m",
        "-0.8 m",
        "-0.7 m",
        "-0.6 m",
        "-0.5 m",
        "-0.4 m",
        "-0.3 m",
        "-0.2 m",
        "-0.1 m",
        "0 m",
        "0.1 m",
        "0.2 m",
        "0.3 m",
        "0.4 m",
        "0.5 m",
        "0.6 m",
        "0.7 m",
        "0.8 m",
        "0.9 m",
        "1.0 m",
        "1.1 m",
        "1.2 m",
        "1.3 m",
        "1.4 m",
        "1.5 m",
        "1.6 m",
        "1.7 m",
        "1.8 m",
        "1.9 m",
        "2.0 m",
        "2.1 m",
        "2.2 m",
        "2.3 m",
        "2.4 m",
        "2.5 m",
        "2.6 m",
        "2.7 m",
        "2.8 m",
        "2.9 m",
        "3.0 m",
        "3.1 m",
        "3.2 m",
        "3.3 m",
        "3.4 m",
        "3.5 m",
        "3.6 m",
        "3.7 m",
        "3.8 m",
        "3.9 m",
        "4.0 m",
        "4.1 m",
        "4.2 m",
        "4.3 m",
        "4.4 m",
        "4.5 m",
        "4.6 m",
        "4.7 m",
        "4.8 m",
        "4.9 m",
        "5.0 m",
		"5.1 m",
        "5.2 m",
        "5.3 m",
        "5.4 m",
        "5.5 m",
        "5.6 m",
        "5.7 m",
        "5.8 m",
        "5.9 m",
        "6.0 m"
    },
    default = 61,
    current = 61,
    text = "gui_ad_RecordTwoRoads",
    tooltip = "gui_ad_RecordTwoRoads_tooltip",
    translate = true,
    isVehicleSpecific = false
}

AutoDrive.settings.RecordOppositeDriveDirectionOffset = {
    values = {
        -6.0,
        -5.9,
        -5.8,
        -5.7,
        -5.6,
        -5.5,
        -5.4,
        -5.3,
        -5.2,
        -5.1,
        -5.0,
        -4.9,
        -4.8,
        -4.7,
        -4.6,
        -4.5,
        -4.4,
        -4.3,
        -4.2,
        -4.1,
        -4.0,
        -3.9,
        -3.8,
        -3.7,
        -3.6,
        -3.5,
        -3.4,
        -3.3,
        -3.2,
        -3.1,
        -3.0,
        -2.9,
        -2.8,
        -2.7,
        -2.6,
        -2.5,
        -2.4,
        -2.3,
        -2.2,
        -2.1,
        -2.0,
        -1.9,
        -1.8,
        -1.7,
        -1.6,
        -1.5,
        -1.4,
        -1.3,
        -1.2,
        -1.1,
        -1.0,
        -0.9,
        -0.8,
        -0.7,
        -0.6,
        -0.5,
        -0.4,
        -0.3,
        -0.2,
        -0.1,
        0,
        0.1,
        0.2,
        0.3,
        0.4,
        0.5,
        0.6,
        0.7,
        0.8,
        0.9,
        1.0,
        1.1,
        1.2,
        1.3,
        1.4,
        1.5,
        1.6,
        1.7,
        1.8,
        1.9,
        2.0,
        2.1,
        2.2,
        2.3,
        2.4,
        2.5,
        2.6,
        2.7,
        2.8,
        2.9,
        3.0,
        3.1,
        3.2,
        3.3,
        3.4,
        3.5,
        3.6,
        3.7,
        3.8,
        3.9,
        4.0,
        4.1,
        4.2,
        4.3,
        4.4,
        4.5,
        4.6,
        4.7,
        4.8,
        4.9,
        5.0,
		5.1,
        5.2,
        5.3,
        5.4,
        5.5,
        5.6,
        5.7,
        5.8,
        5.9,
        6.0
    },
    texts = {
		"-6.0 m",
        "-5.9 m",
        "-5.8 m",
        "-5.7 m",
        "-5.6 m",
        "-5.5 m",
        "-5.4 m",
        "-5.3 m",
        "-5.2 m",
        "-5.1 m",
        "-5.0 m",
        "-4.9 m",
        "-4.8 m",
        "-4.7 m",
        "-4.6 m",
        "-4.5 m",
        "-4.4 m",
        "-4.3 m",
        "-4.2 m",
        "-4.1 m",
        "-4.0 m",
        "-3.9 m",
        "-3.8 m",
        "-3.7 m",
        "-3.6 m",
        "-3.5 m",
        "-3.4 m",
        "-3.3 m",
        "-3.2 m",
        "-3.1 m",
        "-3.0 m",
        "-2.9 m",
        "-2.8 m",
        "-2.7 m",
        "-2.6 m",
        "-2.5 m",
        "-2.4 m",
        "-2.3 m",
        "-2.2 m",
        "-2.1 m",
        "-2.0 m",
        "-1.9 m",
        "-1.8 m",
        "-1.7 m",
        "-1.6 m",
        "-1.5 m",
        "-1.4 m",
        "-1.3 m",
        "-1.2 m",
        "-1.1 m",
        "-1.0 m",
        "-0.9 m",
        "-0.8 m",
        "-0.7 m",
        "-0.6 m",
        "-0.5 m",
        "-0.4 m",
        "-0.3 m",
        "-0.2 m",
        "-0.1 m",
        "0 m",
        "0.1 m",
        "0.2 m",
        "0.3 m",
        "0.4 m",
        "0.5 m",
        "0.6 m",
        "0.7 m",
        "0.8 m",
        "0.9 m",
        "1.0 m",
        "1.1 m",
        "1.2 m",
        "1.3 m",
        "1.4 m",
        "1.5 m",
        "1.6 m",
        "1.7 m",
        "1.8 m",
        "1.9 m",
        "2.0 m",
        "2.1 m",
        "2.2 m",
        "2.3 m",
        "2.4 m",
        "2.5 m",
        "2.6 m",
        "2.7 m",
        "2.8 m",
        "2.9 m",
        "3.0 m",
        "3.1 m",
        "3.2 m",
        "3.3 m",
        "3.4 m",
        "3.5 m",
        "3.6 m",
        "3.7 m",
        "3.8 m",
        "3.9 m",
        "4.0 m",
        "4.1 m",
        "4.2 m",
        "4.3 m",
        "4.4 m",
        "4.5 m",
        "4.6 m",
        "4.7 m",
        "4.8 m",
        "4.9 m",
        "5.0 m",
		"5.1 m",
        "5.2 m",
        "5.3 m",
        "5.4 m",
        "5.5 m",
        "5.6 m",
        "5.7 m",
        "5.8 m",
        "5.9 m",
        "6.0 m"
    },
    default = 61,
    current = 61,
    text = "gui_ad_RecordTwoRoads",
    tooltip = "gui_ad_RecordTwoRoads_tooltip",
    translate = true,
    isVehicleSpecific = false
}

function AutoDrive.getSetting(settingName, vehicle)
    if AutoDrive.settings[settingName] ~= nil then
        local setting = AutoDrive.settings[settingName]
        if setting.isVehicleSpecific and vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.settings ~= nil then
            --try loading vehicle specific setting first, if available
            if vehicle.ad.settings[settingName] ~= nil then
                setting = vehicle.ad.settings[settingName]
            end
        end
        if setting.values[setting.current] == nil then
            setting.current = setting.default
        end
        if setting.values[setting.current] == nil then
            setting.current = 1
        end
        return setting.values[setting.current]
    end
end

function AutoDrive.getSettingState(settingName, vehicle)
    if AutoDrive.settings[settingName] ~= nil then
        local setting = AutoDrive.settings[settingName]
        if setting.isVehicleSpecific and vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.settings ~= nil then
            --try loading vehicle specific setting first, if available
            if vehicle.ad.settings[settingName] ~= nil then
                setting = vehicle.ad.settings[settingName]
            end
        end
        if setting.values[setting.current] == nil then
            setting.current = setting.default
        end
        if setting.values[setting.current] == nil then
            setting.current = 1
        end
        return setting.current
    end
end

function AutoDrive.setSettingState(settingName, value, vehicle)
    if AutoDrive.settings[settingName] ~= nil then
        local setting = AutoDrive.settings[settingName]
        if setting.isVehicleSpecific and vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.settings ~= nil then --try loading vehicle specific setting first, if available
            if vehicle.ad.settings[settingName] ~= nil then
                setting = vehicle.ad.settings[settingName]
            end
        end
        setting.current = value
        setting.new = value
    end
end

function AutoDrive.copySettingsToVehicle(vehicle)
    if vehicle == nil then
        Logging.error("[AD] AutoDrive.copySettingsToVehicle vehicle == nil")
        return
    end
    if vehicle.ad == nil then
        vehicle.ad = {}
    end
    if vehicle.ad.settings == nil then
        vehicle.ad.settings = {}
    end
    for settingName, setting in pairs(AutoDrive.settings) do
        if setting.isVehicleSpecific then
            local settingVehicle = {}
            settingVehicle.values = setting.values
            settingVehicle.texts = setting.texts
            settingVehicle.default = setting.default
            settingVehicle.userDefault = setting.userDefault
            if setting.userDefault ~= nil then
                settingVehicle.current = setting.userDefault
            else
                settingVehicle.current = setting.default
            end
            settingVehicle.text = setting.text
            settingVehicle.tooltip = setting.tooltip
            settingVehicle.translate = setting.translate
            vehicle.ad.settings[settingName] = settingVehicle
        end
    end
end

function AutoDrive.readVehicleSettingsFromXML(vehicle, xmlFile, key)
    if vehicle == nil then
        Logging.error("[AD] AutoDrive.readVehicleSettingsFromXML vehicle == nil")
        return
    end
    if vehicle.ad == nil then
        vehicle.ad = {}
    end
    if vehicle.ad.settings == nil then
        vehicle.ad.settings = {}
    end

    vehicle.ad.settings = {}
    for settingName, setting in pairs(AutoDrive.settings) do
        if setting.isVehicleSpecific and not setting.shallNotBeSaved then
            local settingVehicle = {}
            settingVehicle.values = setting.values
            settingVehicle.default = setting.default
            settingVehicle.userDefault = setting.userDefault
            if setting.userDefault ~= nil then
                settingVehicle.current = setting.userDefault
            else
                settingVehicle.current = setting.default
            end
            vehicle.ad.settings[settingName] = settingVehicle

            if xmlFile:hasProperty(key) then
                local storedSetting = xmlFile:getValue(key .. "#" .. settingName)
                if storedSetting ~= nil then
                    vehicle.ad.settings[settingName].current = storedSetting
                end
            end
        end
    end
end

function AutoDrive.saveVehicleSettingsToXMLFile(vehicle, xmlFile, key)
    if vehicle then
        for settingName, setting in pairs(AutoDrive.settings) do
            if setting.isVehicleSpecific and vehicle.ad.settings ~= nil and vehicle.ad.settings[settingName] ~= nil and not setting.shallNotBeSaved then
                xmlFile:setValue(key .. "#" .. settingName, vehicle.ad.settings[settingName].current)
            end
        end
    end
end

function AutoDrive.getMaxTriggerDistance(vehicle)
    -- the max-trigger-distance can be set globally and per-vehicle.
    -- a per-vehicle setting of 0 means "use global value"
    -- NB: this might not be the best place for this function
    local distance = AutoDrive.getSetting("maxTriggerDistanceVehicle", vehicle)
    if distance == 0 then
        distance = AutoDrive.getSetting("maxTriggerDistance")
    end
    return distance
end
