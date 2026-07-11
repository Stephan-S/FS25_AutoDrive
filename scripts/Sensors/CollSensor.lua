ADCollSensor = ADInheritsFrom(ADSensor)

--[[    
    from collisionMaskFlags.xml
    <flag bit="0" name="DEFAULT" desc="The default bit"/>
    <flag bit="1" name="STATIC_OBJECT" desc="Static object"/>
    <flag bit="2" name="CAMERA_BLOCKING" desc="Blocks the player camera from being inside"/>
    <flag bit="3" name="GROUND_TIP_BLOCKING" desc="Blocks tipping on the ground beneath/above"/>
    <flag bit="4" name="PLACEMENT_BLOCKING" desc="Blocks placing objects via construction"/>
    <flag bit="5" name="AI_BLOCKING" desc="Blocks vehicle navigation map beneath/above"/>
    <flag bit="6" name="PRECIPITATION_BLOCKING" desc="Masks all precipitation inside and below the collision"/>
    <flag bit="8" name="TERRAIN" desc="Terrain without tip any or displacement"/>
    <flag bit="9" name="TERRAIN_DELTA" desc="Tip anything"/>
    <flag bit="10" name="TERRAIN_DISPLACEMENT" desc="Terrain displacement (tiretrack deformation)"/>
    <flag bit="11" name="TREE" desc="A tree"/>
    <flag bit="12" name="BUILDING" desc="A building"/>
    <flag bit="13" name="ROAD" desc="A road"/>
    <flag bit="14" name="AI_DRIVABLE" desc="Blocks vehicle navigation map at the vertical faces of the mesh if they are above the terrain"/>
    <flag bit="16" name="VEHICLE" desc="A vehicle"/>
    <flag bit="17" name="VEHICLE_FORK" desc="A vehicle fork tip for pallets or bales"/>
    <flag bit="18" name="DYNAMIC_OBJECT" desc="A dynamic object"/>
    <flag bit="19" name="TRAFFIC_VEHICLE" desc="A AI traffic vehicle"/>
    <flag bit="20" name="PLAYER" desc="A player"/>
    <flag bit="21" name="ANIMAL" desc="An Animal"/>
    <flag bit="22" name="ANIMAL_POSITIONING" desc="For animal to walk on (position is raycast from above)"/>
    <flag bit="23" name="ANIMAL_NAV_MESH_BLOCKING" desc="Area of the collision is excluded from generated nav meshes"/>
    <flag bit="24" name="TRAFFIC_VEHICLE_BLOCKING" desc="Blocks AI traffic vehicles"/>
    <flag bit="28" name="INTERACTABLE_TARGET" desc="An interactable trigger that the player can target"/>
    <flag bit="29" name="TRIGGER" desc="A trigger"/>
    <flag bit="30" name="FILLABLE" desc="A fillable node. For trailer fillNodes and unload triggers"/>
    <flag bit="31" name="WATER" desc="A water plane"/>
]]


function ADCollSensor:new(vehicle, sensorParameters)
    local o = ADCollSensor:create()
    o:init(vehicle, ADSensor.TYPE_COLLISION, sensorParameters)
    o.hit = false
    o.newHit = false
    o.vehicle = vehicle --test collbox and coll bits mode
    o.mask = 0

    return o
end

function ADCollSensor.getMask()
    return CollisionFlag.DEFAULT + CollisionFlag.STATIC_OBJECT + CollisionFlag.DYNAMIC_OBJECT + CollisionFlag.VEHICLE + CollisionFlag.TERRAIN_DELTA
    + CollisionFlag.TREE + CollisionFlag.BUILDING + CollisionFlag.TRAFFIC_VEHICLE + CollisionFlag.TRAFFIC_VEHICLE_BLOCKING
end

function ADCollSensor:onUpdate(dt)
    self.mask = self:getMask()
    local box = self:getBoxShape()
    self.newHit = false

    local offsetCompensation = math.max(-math.tan(box.rx) * box.size[3], 0)
    box.y = math.max(getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, box.x, 300, box.z), box.y) + offsetCompensation
    overlapBox(box.x, box.y, box.z, box.rx, box.ry, 0, box.size[1], box.size[2], box.size[3], "collisionTestCallback", self, self.mask, true, true, true, true)
    self.hit = self.newHit
    self:setTriggered(self.hit)
    self:onDrawDebug(box)
end

function ADCollSensor:collisionTestCallback(transformId)
    local unloadDriver = ADHarvestManager:getAssignedUnloader(self.vehicle.ad.attachableCombine or self.vehicle)
    local collisionObject = self:getCollisionObject(transformId)
    local isBlocking = self:isElementBlockingVehicle(transformId)

    if collisionObject ~= nil then
        if isBlocking and not self:isCollisionObjectExcluded(collisionObject) and collisionObject ~= self and collisionObject ~= self.vehicle and not AutoDrive:checkIsConnected(self.vehicle:getRootVehicle(), collisionObject) then
            if unloadDriver == nil or (collisionObject ~= unloadDriver and (not AutoDrive:checkIsConnected(unloadDriver:getRootVehicle(), collisionObject))) then
                self.newHit = true
                self:debugCollisionNode(transformId)
            end
        end
    elseif isBlocking then
        self.newHit = true
        self:debugCollisionNode(transformId)
    end
end
