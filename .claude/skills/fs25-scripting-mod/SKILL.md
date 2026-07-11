---
name: fs25-scripting-mod
description: Use when developing FS25 Lua scripting mods, writing specializations, overriding base game functions, debugging mod scripts, or understanding the FS25 scripting architecture.
---

# FS25 Scripting Mod Development

## Overview

FS25 mods extend the game via Lua scripts. Always reference the base game scripts to understand existing patterns before writing new code.

## When to Use

- Writing or debugging Lua scripts for an FS25 mod
- Creating vehicle specializations or placeable extensions
- Overriding or extending base game behavior
- Understanding how a base game system works

## Base Game Scripts

**IMPORTANT**: Before writing any mod script, search the base game scripts for how the game implements similar functionality. The decompiled scripts are the primary reference.

**Base game scripts**: `dataS/scripts/` (relative to game data root)

You will find a symlink to the decompiled scripts in the project root named 'baseGameScripts' for easy reference.

### Key Directories

| Directory | Contains |
|---|---|
| `vehicles/specializations/` | Vehicle specializations (Motorized, Drivable, Fillable, etc.) |
| `placeables/specializations/` | Placeable specializations |
| `specialization/` | SpecializationManager, TypeManager, SpecializationUtil |
| `gui/` | GUI elements, dialogs, screens |
| `events/` | Network event classes |
| `economy/` | Economy, prices, selling points |
| `environment/` | Weather, seasons, time |
| `animals/` | Animal husbandry system |
| `field/` | Field state, ownership, missions |
| `fruits/` | Fruit types and growth |
| `internalMods/` | DLC/internal mods (e.g. FS25_precisionFarming) |

### Important Base Files

| File | Purpose |
|---|---|
| `main.lua` | Entry point, class system (`source()`, `Class()`) |
| `BaseMission.lua` | Mission lifecycle, loading |
| `FSBaseMission.lua` | FS-specific mission logic |
| `events.lua` | Event registration |
| `MessageCenter.lua` | Pub/sub messaging system |
| `MessageType.lua` | Message type constants |

## Mod Structure

```
FS25_ModName/
  modDesc.xml          # Mod metadata, script entry points
  scripts/
    MySpecialization.lua
    events/
      MyEvent.lua
```

### modDesc.xml Script Registration

```xml
<modDesc descVersion="106">
    <!-- Specialization type -->
    <specializations>
        <specialization name="mySpec" className="MySpecialization" filename="scripts/MySpecialization.lua" />
    </specializations>

    <!-- Attach to vehicle types -->
    <vehicleTypes>
        <type name="myVehicle" parent="baseDrivable">
            <specialization name="mySpec" />
        </type>
    </vehicleTypes>

    <!-- Or: standalone script -->
    <extraSourceFiles>
        <sourceFile filename="scripts/main.lua" />
    </extraSourceFiles>
</modDesc>
```

## Specialization Pattern

The standard pattern for vehicle/placeable specializations:

```lua
MySpecialization = {}

function MySpecialization.prerequisitesPresent(specializations)
    return SpecializationUtil.hasSpecialization(Drivable, specializations)
end

function MySpecialization.registerEventListeners(vehicleType)
    SpecializationUtil.registerEventListener(vehicleType, "onLoad", MySpecialization)
    SpecializationUtil.registerEventListener(vehicleType, "onUpdate", MySpecialization)
    SpecializationUtil.registerEventListener(vehicleType, "onDelete", MySpecialization)
end

function MySpecialization.registerFunctions(vehicleType)
    SpecializationUtil.registerFunction(vehicleType, "myCustomFunction", MySpecialization.myCustomFunction)
end

function MySpecialization.registerOverwrittenFunctions(vehicleType)
    SpecializationUtil.registerOverwrittenFunction(vehicleType, "someBaseFunction", MySpecialization.someBaseFunction)
end

function MySpecialization:onLoad(savegame)
    self.spec_mySpecialization = {}
    -- init
end

function MySpecialization:onUpdate(dt, isActiveForInput, isActiveForInputIgnoreSelection, isSelected)
    -- per-frame logic
end
```

### Critical FS25 Registration Rules

- If you read/write custom savegame XML values (e.g. in `onPostLoad` or `saveToXMLFile`), register all corresponding paths in `initSpecialization()` on `Vehicle.xmlSchemaSavegame` first.
- If you call a custom method via `self:myMethod()` from specialization callbacks, register that method in `registerFunctions()` so it is available on the vehicle instance.
- Missing XML schema registration can cause runtime XML access errors; missing function registration can cause nil-method runtime errors.

## Overwriting Base Functions

Use `registerOverwrittenFunction` — calls your function with the original as first arg:

```lua
function MySpecialization.registerOverwrittenFunctions(vehicleType)
    SpecializationUtil.registerOverwrittenFunction(vehicleType, "getCanBeSelected", MySpecialization.getCanBeSelected)
end

function MySpecialization:getCanBeSelected(superFunc)
    if someCondition then
        return false
    end
    return superFunc(self)
end
```

## Common Event Listeners

| Event | When |
|---|---|
| `onLoad` | Vehicle/placeable loaded from XML |
| `onPostLoad` | After all specializations loaded |
| `onDelete` | Object being deleted |
| `onUpdate` | Every frame (active vehicles) |
| `onUpdateTick` | Every network tick |
| `onDraw` | Render pass |
| `onReadStream` / `onWriteStream` | Network sync (initial) |
| `onReadUpdateStream` / `onWriteUpdateStream` | Network sync (updates) |
| `saveToXMLFile` | Savegame write |

## Network Events

```lua
MyEvent = {}
local MyEvent_mt = Class(MyEvent, Event)

InitEventClass(MyEvent, "MyEvent")

function MyEvent.emptyNew()
    return Event.new(MyEvent_mt)
end

function MyEvent.new(vehicle, value)
    local self = MyEvent.emptyNew()
    self.vehicle = vehicle
    self.value = value
    return self
end

function MyEvent:readStream(streamId, connection)
    self.vehicle = NetworkUtil.readNodeObject(streamId)
    self.value = streamReadFloat32(streamId)
    self:run(connection)
end

function MyEvent:writeStream(streamId, connection)
    NetworkUtil.writeNodeObject(streamId, self.vehicle)
    streamWriteFloat32(streamId, self.value)
end

function MyEvent:run(connection)
    if self.vehicle ~= nil then
        self.vehicle:myFunction(self.value, true)
    end
end
```

## Debugging

- **Game log**: Check `log.txt` in the game data folder for errors
- **print()**: Outputs to log and in-game console
- **In-game console**: `~` key, use `gsToggleStats` for performance overlay
- Common error: `attempt to index nil` usually means a specialization dependency is missing or `spec_` table not initialized

## Workflow

1. Identify what base game system to extend — search `dataS/scripts/` for similar functionality
2. Read the relevant base specialization to understand the API
3. Write your specialization following the same patterns
4. Register in modDesc.xml
5. Test in-game, check log.txt for errors
6. Verify both registrations: schema paths for XML access and `registerFunctions()` for all methods invoked via `self:...()`