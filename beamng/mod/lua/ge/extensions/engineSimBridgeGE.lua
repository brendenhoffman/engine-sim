-- Game-engine side of the bridge:
--  * vehicle Lua stops while the game is paused, so keep EngineSim linked from here ("BESH")
--  * describe the player's car and engine to engine-sim ("BESV" + JSON), so it can show it and,
--    later, build a matching engine

local M = {}

local jbeamIO = require('jbeam/io')

local enginePort = 4570
local udp
local heartbeatTimer = 0
local vehicleInfoTimer = 0
local vehicleInfoDue = true

local function socketReady()
  if not udp then
    udp = socket.udp()
    udp:settimeout(0)
    udp:setpeername("127.0.0.1", enginePort)
  end
  return udp
end

-- Part names are either plain text or a translation object like
-- { txt = "ui.vehicleconfig.information.name.BRAND", ctx = { brand1 = "Civetta Scintilla" } };
-- the context values are the readable part
-- Translation keys like "ui.vehicleconfig.brand.Civetta Scintilla" end in the readable text
local function stripKey(text)
  return (text:gsub("^ui%.vehicleconfig%.%w+%.", ""))
end

local function readableName(name)
  if type(name) == "string" then return stripKey(name) end
  if type(name) ~= "table" then return nil end
  if type(name.ctx) == "table" then
    local keys, words = {}, {}
    for key, value in pairs(name.ctx) do
      if type(value) == "string" then table.insert(keys, key) end
    end
    table.sort(keys)
    for _, key in ipairs(keys) do table.insert(words, stripKey(name.ctx[key])) end
    if #words > 0 then return table.concat(words, " ") end
  end
  return type(name.txt) == "string" and stripKey(name.txt) or nil
end

local function numberOr(value)
  return type(value) == "number" and value or nil
end

-- Display name and file of a part, from the vehicle's part context
local function partInfo(bundle, partName)
  if not partName or not bundle.ioCtx then return nil end
  local ok, part = pcall(jbeamIO.getPart, bundle.ioCtx, partName)
  if not ok or type(part) ~= "table" then return { part = partName } end
  local info = part.information or {}
  return { part = partName, name = readableName(info.name), value = numberOr(info.value) }
end

-- The installed part that defines a section: several parts can add to one (an ECU part sets the
-- rev limit or boost, an intake part tweaks torque), so prefer the one with the most of `keys`
-- (the fields only the real hardware defines), then the one adding the most fields
local function definingPart(vdata, sectionName, keys)
  if not sectionName then return nil end
  local bestName, bestPart, bestScore = nil, nil, -1
  for partName, part in pairs(vdata.activePartsData or {}) do
    local section = type(part) == "table" and part[sectionName]
    if type(section) == "table" then
      local score = 0
      for _ in pairs(section) do score = score + 1 end
      for _, key in ipairs(keys or {}) do
        if section[key] ~= nil then score = score + 1000 end
      end
      if score > bestScore then bestName, bestPart, bestScore = partName, part, score end
    end
  end
  if not bestName then return nil end
  local info = bestPart.information or {}
  return { part = bestName, name = readableName(info.name), value = numberOr(info.value) }
end

-- jbeam torque tables start with a header row: [["rpm", "torque"], [0, 0], [500, 120], ...]
local function torqueTable(rows)
  if type(rows) ~= "table" then return nil end
  local out = {}
  for _, row in ipairs(rows) do
    if type(row) == "table" and type(row[1]) == "number" and type(row[2]) == "number" then
      table.insert(out, { row[1], row[2] })
    end
  end
  return #out > 0 and out or nil
end

local function cylinderCount(vdata, soundSection)
  local sound = soundSection and vdata[soundSection]
  return sound and numberOr(sound.fundamentalFrequencyCylinderCount)
end

local function soundSample(vdata, soundSection)
  local sound = soundSection and vdata[soundSection]
  return sound and sound.sampleName
end

local function describePlayerVehicle()
  local bundle = core_vehicle_manager and core_vehicle_manager.getPlayerVehicleData()
  if not bundle or not bundle.vdata then return nil end
  local vdata = bundle.vdata

  -- The combustion engine in the powertrain (entries carry type and name)
  local engineName
  for _, device in pairs(vdata.powertrain or {}) do
    if type(device) == "table" and device.type == "combustionEngine" and device.name then
      engineName = device.name
      break
    end
  end

  -- Brand and model as the vehicle selector shows them; sub-models (Cherrier Ardente is a
  -- Vivace) and the configuration ("Rally") come from the config's info
  local model, configuration
  local ok, details = pcall(core_vehicles.getCurrentVehicleDetails)
  if ok and type(details) == "table" then
    local config = type(details.configs) == "table" and details.configs or {}
    local base = type(details.model) == "table" and details.model or {}
    local brand = readableName(config.Brand or base.Brand)
    local name = readableName(config.Name or base.Name)
    if name then model = brand and (brand .. " " .. name) or name end
    configuration = readableName(config.Configuration)
  end

  local info = {
    v = 1,
    vehicle = {
      id = bundle.id,
      directory = vdata.vehicleDirectory,
      model = model or (partInfo(bundle, bundle.mainPartName) or {}).name,
      configuration = configuration,
    },
  }

  local engine = engineName and vdata[engineName]
  if type(engine) ~= "table" then
    return info -- electric, or no engine fitted
  end

  local part = definingPart(vdata, engineName, { "torque", "inertia", "idleRPM" }) or {}
  info.engine = {
    part = part.part,
    name = part.name,
    cylinders = cylinderCount(vdata, engine.soundConfigExhaust) or cylinderCount(vdata, engine.soundConfig),
    fuel = engine.requiredEnergyType,
    idle_rpm = numberOr(engine.idleRPM),
    max_rpm = numberOr(engine.maxRPM),
    rev_limiter_rpm = numberOr(engine.revLimiterRPM),
    max_physical_rpm = numberOr(engine.maxPhysicalRPM),
    inertia = numberOr(engine.inertia),
    friction = numberOr(engine.friction),
    dynamic_friction = numberOr(engine.dynamicFriction),
    engine_brake_torque = numberOr(engine.engineBrakeTorque),
    torque = torqueTable(engine.torque),
    placement = engine.enginePlacement,
    engine_sample = soundSample(vdata, engine.soundConfig),
    exhaust_sample = soundSample(vdata, engine.soundConfigExhaust),
  }

  local turbo = engine.turbocharger and vdata[engine.turbocharger]
  if type(turbo) == "table" and next(turbo) ~= nil then
    local turboPart = definingPart(vdata, engine.turbocharger, { "turboExhaustCurve", "turboPressureCurve", "maxExhaustPower", "inertia" }) or {}
    info.turbo = {
      part = turboPart.part,
      name = turboPart.name,
      wastegate_psi = numberOr(turbo.wastegateStart),
      max_exhaust_power = numberOr(turbo.maxExhaustPower),
    }
  end

  local supercharger = engine.supercharger and vdata[engine.supercharger]
  if type(supercharger) == "table" and next(supercharger) ~= nil then
    local scPart = definingPart(vdata, engine.supercharger, { "type", "lobes", "gearRatio" }) or {}
    info.supercharger = {
      part = scPart.part,
      name = scPart.name,
      type = supercharger.type,
      psi_per_1k_rpm = numberOr(supercharger.pressurePSIPer1kRPM),
      gear_ratio = numberOr(supercharger.gearRatio),
    }
  end

  return info
end

local function sendVehicleInfo()
  local ok, info = pcall(describePlayerVehicle)
  if not ok then
    log("W", "engineSimBridge", "couldn't describe the vehicle: " .. tostring(info))
    return
  end
  if info then socketReady():send("BESV" .. jsonEncode(info)) end
end

local function onUpdate(dtReal)
  -- The car's description: right after spawning or switching, then every few seconds in case
  -- engine-sim started later
  vehicleInfoTimer = vehicleInfoTimer + dtReal
  if vehicleInfoDue or vehicleInfoTimer > 3 then
    vehicleInfoDue = false
    vehicleInfoTimer = 0
    sendVehicleInfo()
  end

  if not simTimeAuthority.getPause() then return end
  heartbeatTimer = heartbeatTimer + dtReal
  if heartbeatTimer < 0.1 then return end
  heartbeatTimer = 0
  socketReady():send("BESH")
end

local function onVehicleSwitched()
  vehicleInfoDue = true
end

local function onVehicleSpawned()
  vehicleInfoDue = true
end

local function onExtensionUnloaded()
  if udp then udp:close() end
end

M.onUpdate = onUpdate
M.onVehicleSwitched = onVehicleSwitched
M.onVehicleSpawned = onVehicleSpawned
M.onExtensionUnloaded = onExtensionUnloaded

return M
