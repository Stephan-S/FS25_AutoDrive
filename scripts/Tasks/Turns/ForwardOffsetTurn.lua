ForwardOffsetTurn = ADInheritsFrom(AbstractTurn)

ForwardOffsetTurn.MAX_STRAIGHTENING_DISTANCE = 18

function ForwardOffsetTurn:new(turnParameters)
    local o = ForwardOffsetTurn:create()
    o.generationStep = 0
    o.turnGenerated = false
    o.turnParameters = turnParameters
    o.path = {}
    o.task = o.turnParameters.handleHarvesterTask
    o.failed = false
    return o
end

function ForwardOffsetTurn.checkValidity(turnParameters)
    return turnParameters.angle > 165 and turnParameters.angle < 195
end

function ForwardOffsetTurn:update(dt)
    if self.failed then
        return false
    end

    if not self.turnGenerated then
        self:generateTurn()
    end

    if self.turnGenerated then
        return true
    end

    return false
end

function ForwardOffsetTurn:getWayPoints()
    return self.path
end

function ForwardOffsetTurn:generateTurn()
    local vehicle = self.turnParameters.vehicle
    if self.generationStep == 0 then
        self.path = {}
        self.generationStep = self.generationStep + 1
    elseif self.generationStep == 1 then
        local offsetX, offsetZ = self.task:getUturnOffsetValues(self.turnParameters.targetLeft)
        local targetPoints = self.task:generateReverseToCombineSection(true)
        local diffZ = 0
        if targetPoints[1] ~= nil then
            _, _, diffZ = AutoDrive.worldToLocal(vehicle, targetPoints[1].x, targetPoints[1].y, targetPoints[1].z)
        end
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_COMBINEINFO, "ForwardOffsetTurn:generateTurn - offsetZ: " .. offsetZ .. " diffZ: " .. diffZ)
        offsetZ = math.max(offsetZ, diffZ)

        local uturnNodes = self.task:generateUTurn(self.turnParameters.startPosition, self.turnParameters.startDir.x, self.turnParameters.startDir.z, self.turnParameters.targetLeft, {offsetX, offsetZ})
        for i, node in pairs(uturnNodes) do
            table.insert(self.path, node)
        end
        self.generationStep = self.generationStep + 1
        if uturnNodes == nil or #uturnNodes == 0 then
            self.failed = true
        else            
            self.turnGenerated = true
        end
    end    
end