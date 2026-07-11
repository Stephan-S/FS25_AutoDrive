ADSelectUnloaderGui = {}

local ADSelectUnloaderGui_mt = Class(ADSelectUnloaderGui, DialogElement)

function ADSelectUnloaderGui.new(target)
    local self = DialogElement.new(target, ADSelectUnloaderGui_mt)
    self.harvester = nil
    self.listItems = {}
    return self
end

function ADSelectUnloaderGui:setHarvester(harvester)
    self.harvester = harvester
end

function ADSelectUnloaderGui:onOpen()
    ADSelectUnloaderGui:superClass().onOpen(self)
    self.unloaderList:setDataSource(self)
    self:refreshItems()
end

function ADSelectUnloaderGui:getNumberOfItemsInSection(list, section)
    if list == self.unloaderList then
        return #self.listItems
    end
    return 0
end

function ADSelectUnloaderGui:populateCellForItemInSection(list, section, index, cell)
    if list == self.unloaderList then
        cell.attributes.listItemText:setText(self.listItems[index].text)
    end
end

function ADSelectUnloaderGui:refreshItems()
    self.listItems = {}
    for _, unloader in pairs(ADHarvestManager:getIdleUnloadersForHarvester(self.harvester)) do
        local trailers = AutoDrive.getAllUnits(unloader)
        local fillLevel, _, _, fillFreeCapacity = AutoDrive.getAllFillLevels(trailers)
        local capacity = fillLevel + fillFreeCapacity
        local fillPercent = capacity > 0 and fillLevel / capacity * 100 or 0
        local name = unloader.ad.stateModule:getName()
        table.insert(self.listItems, {
            unloader = unloader,
            name = name,
            text = string.format("%s    %.0f l    %.1f%%", name, fillLevel, fillPercent)
        })
    end
    table.sort(self.listItems, function(a, b) return a.name < b.name end)
    self.unloaderList:reloadData()
end

function ADSelectUnloaderGui:selectUnloader(index)
    local item = self.listItems[index]
    if item ~= nil then
        AutoDriveSendUnloaderToUnloadEvent.sendEvent(self.harvester, item.unloader)
        self:onClickBack()
    end
end

function ADSelectUnloaderGui:onDoubleClick(list, section, index, cell)
    self:selectUnloader(index)
end

function ADSelectUnloaderGui:onClickSelect()
    self:selectUnloader(self.unloaderList:getSelectedIndexInSection())
end

