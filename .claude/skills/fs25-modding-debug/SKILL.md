---
name: fs25-modding-debug
description: Use when debugging FS25 mods, reading log.txt errors, troubleshooting crashes, fixing common mod issues, or using console commands.
---

# FS25 Modding Debug Guide

## Overview

Systematic approach to debugging FS25 mods — reading logs, common errors, console commands, and troubleshooting techniques.

## When to Use

- Mod crashes or doesn't load
- Errors in log.txt
- In-game behavior doesn't match expectations
- Performance issues

## Log File Location

| Platform | Path |
|---|---|
| **Windows** | `%USERPROFILE%\Documents\My Games\FarmingSimulator2025\log.txt` |
| **macOS** | `~/Library/Application Support/FarmingSimulator2025/log.txt` |

The log is overwritten each game launch. Save it before restarting if you need to preserve it.

## Reading the Log

### Error Severity

| Prefix | Meaning |
|---|---|
| `Error:` | Critical — mod won't work correctly |
| `Warning:` | Non-fatal — may cause issues |
| `LUA call stack:` | Lua error with traceback |
| `data/` or mod path | Context for where the error occurred |

### Common Error Patterns

**Missing node / nil index:**
```
Error: Can't find node 'wheelRepr' in '...'
```
Fix: Check i3dMappings or node path. Node was renamed or removed from I3D.

**Lua nil error:**
```
Error: attempt to index a nil value
LUA call stack: ...mySpecialization.lua:42
```
Fix: A variable is nil — check if prerequisite specialization is loaded, or if `spec_` table was initialized.

**Missing file:**
```
Error: Can't load resource '...textures/diffuse.dds'
```
Fix: File doesn't exist at that path. Check case sensitivity (Linux/macOS).

**XML parse error:**
```
Error: XML parse error in '...modDesc.xml' at line 15
```
Fix: Invalid XML — unclosed tag, wrong encoding, or special characters in attributes.

**descVersion mismatch:**
```
Warning: ModDesc version ... is not supported
```
Fix: Update `descVersion` in modDesc.xml to current version.

**Specialization error:**
```
Error: Unable to add specialization 'mySpec' ... prerequisites not met
```
Fix: `prerequisitesPresent()` returned false. Check required specializations.

## Console Commands

Open console with **`~`** (tilde) key in-game.

| Command | Purpose |
|---|---|
| `gsToggleStats` | Performance overlay (FPS, draw calls) |
| `gsVehicleDebug` | Vehicle debug rendering |
| `gsFillUnitDebug` | Fill unit debug info |
| `gsAIDebug` | AI worker debug visualization |
| `gsPhysicsDebug` | Physics collision shapes |
| `gsRenderCollisions` | Show collision meshes |
| `csReloadMods` | Reload script-only mods without restart |
| `gsFieldDebug` | Field ownership/state overlay |

## Dev Mode

Launch with `-cheats` flag for extra commands:
- Free camera, teleport, money cheats
- Additional debug overlays

## Debugging Checklist

1. **Check log.txt** — always start here, errors are listed in order
2. **Validate XML** — malformed XML is the #1 cause of "mod doesn't load"
3. **Check file paths** — case-sensitive on macOS, verify all referenced files exist
4. **Check modDesc.xml** — correct `descVersion`, valid `<specializations>` and `<vehicleTypes>`
5. **Test with only your mod** — disable other mods to rule out conflicts
6. **Compare with base game** — find a similar base game asset and diff against yours
7. **Add print() statements** — output to log.txt for Lua debugging
8. **Check i3dMappings** — wrong node paths are a common source of nil errors

## Performance Issues

| Symptom | Likely Cause | Fix |
|---|---|---|
| Low FPS near mod | Too many draw calls | Merge meshes, reduce materials |
| Stutter on load | Large textures | Reduce resolution, add mipmaps |
| Physics lag | Too many collision shapes | Simplify collision meshes |
| Memory spike | Uncompressed textures | Use DDS compression |

## Network / Multiplayer Issues

- All custom events must implement `readStream`/`writeStream`
- State changes must be synced via events, not just local assignment
- Test with dedicated server + client to catch sync bugs
- `dirtyFlag` system controls what gets synced each tick