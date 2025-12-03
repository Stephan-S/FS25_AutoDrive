AutoDriveHotspot = {}
AutoDriveHotspot_mt = Class(AutoDriveHotspot, PlaceableHotspot)

function AutoDriveHotspot.new(index, marker)
	local self = PlaceableHotspot.new(AutoDriveHotspot_mt)
	self.isADMarker = true
	self.isADDebug = marker.isADDebug == true
	self.markerID = index
	self.width, self.height = getNormalizedScreenValues(40, 40)

	local sliceId
	if marker.isADDebug == true then
		sliceId = "ad_gui.debug_marker"
	else
		sliceId = "ad_gui.marker"
	end
	self.icon = g_overlayManager:createOverlay(sliceId, 0, 0, self.width, self.height)
	self.iconSmall = g_overlayManager:createOverlay(sliceId, 0, 0, self.width, self.height)
	return self
end

function AutoDriveHotspot.getCategory(self)
	return MapHotspot.CATEGORY_STEERABLE
end

function AutoDriveHotspot.getIsPersistent(self)
	return false
end

function AutoDriveHotspot.getRenderLast(self)
	return true
end

function AutoDriveHotspot.getIsVisible(self)
	if g_inGameMenu.isOpen then
		-- main menu is open
		return AutoDrive.showMarkersOnMainMenuMap()
	elseif self.isADDebug == true then
		-- debug marker on ingame map, always hide
		return false
	else
		-- marker in ingame map
		return AutoDrive.showMarkersOnIngameMap()
	end
end
