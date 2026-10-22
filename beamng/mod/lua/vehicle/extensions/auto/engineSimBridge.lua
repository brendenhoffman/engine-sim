-- Vehicle side of the bridge: sends rpm, throttle and ignition to engine-sim, applies its torque,
-- and swaps the player car's engine/exhaust blends for silent markers the DSP plugin replaces.

local M = {}

local ffi = require("ffi")
pcall(ffi.cdef, [[
typedef struct { char magic[4]; float rpm, throttle, ignition, volume, pan; } engineSimOut_t;
typedef struct { char magic[4]; float torque, rpm, boost; } engineSimIn_t;
]])

local enginePort = 4570
local useEngineSimTorque = true

local outPacket = ffi.new("engineSimOut_t")
local inPacket = ffi.new("engineSimIn_t")
local profileValues = ffi.new("float[256]")
ffi.copy(outPacket.magic, "BES1", 4)

local udp
local engine
local origTorqueUpdate
local esTorque, esAge = 0, math.huge
-- engine-sim's manifold pressure above ambient, kPa (negative: vacuum), for the boost gauge
local esBoost
local active = false
local originalBlends = {}
local appliedProfileId
local stockEngine

-- Sound settings from engine-sim ("BESC"): flatten the car's EQ while linked (it's tuned for
-- the car's own samples), and choose which events carry engine-sim's sound
local soundConfig = { neutralEq = true, intake = 0.35, exhaust = 1, tuning = false, burble = true }
-- The game's sustained afterfire ("popcorn" burble, a recorded sample) can be switched off
-- while engine-sim plays; its single bangs and shift bangs stay. The flames go with the burble.
local stockBurble
local function applyBurble(on)
  if not engine then return end
  if stockBurble == nil then stockBurble = engine.sustainedAfterFireCoef or 0 end
  engine.sustainedAfterFireCoef = on and stockBurble or 0
end
local markersOn = false
local stockParams = {}
local soundRefs = { engine = "engineSoundID", exhaust = "engineSoundIDExhaust" }

local markerBlends = {
  engine = "art/sound/engineSim/engineSimEngine.sfxBlend2D.json",
  exhaust = "art/sound/engineSim/engineSimExhaust.sfxBlend2D.json",
}

local function exhaustNodePairs()
  local nodePairs = {}
  for _, v in pairs(engine.thermals and engine.thermals.exhaustEndNodes or {}) do
    table.insert(nodePairs, {v.start, v.finish})
  end
  if #nodePairs == 0 then nodePairs = {{engine.engineNodeID, engine.engineNodeID}} end
  return nodePairs
end

-- shortcut: restoring re-inits with gains 1/1 instead of the jbeam's on/offLoadGain; fine as a
-- toggle, read soundConfig from v.data if the restored sound level matters
local function useMarkerBlends(on)
  local cfg = engine.soundConfiguration
  log("I", "engineSimBridge", string.format("%s: sound config %s, engine sound %s, exhaust sound %s",
    on and "linking" or "unlinking", cfg and "found" or "MISSING",
    tostring(engine.engineSoundID), tostring(engine.engineSoundIDExhaust)))
  if not cfg then return end
  if engine.engineSoundID and cfg.engine then
    originalBlends.engine = originalBlends.engine or cfg.engine.blendFile
    engine:initEngineSound(engine.engineSoundID, on and markerBlends.engine or originalBlends.engine, {engine.engineNodeID}, 1, 1, "engine")
  end
  if engine.engineSoundIDExhaust and cfg.exhaust then
    originalBlends.exhaust = originalBlends.exhaust or cfg.exhaust.blendFile
    engine:initExhaustSound(engine.engineSoundIDExhaust, on and markerBlends.exhaust or originalBlends.exhaust, exhaustNodePairs(), 1, 1, "exhaust")
  end
end

-- Re-sends each event's parameters: the stock ones, or while linked a copy with the EQ flattened
-- and/or the event muted. setEngineSoundParameterList needs the full table (gain/muffling offsets).
local function applySoundParams(linked)
  local cfg = engine.soundConfiguration
  if not cfg then return end
  for ref, idKey in pairs(soundRefs) do
    local id = engine[idKey]
    if id and cfg[ref] and cfg[ref].params then
      stockParams[ref] = stockParams[ref] or deepcopy(cfg[ref].params)
      local p = deepcopy(stockParams[ref])
      if linked then
        if soundConfig.neutralEq then
          p.eq_a_freq, p.eq_a_gain = sounds.hzToFMODHz(20), 0
          p.eq_b_freq, p.eq_b_gain = sounds.hzToFMODHz(20000), 0
          p.eq_c_gain, p.eq_d_gain, p.eq_e_gain = 0, 0, 0
        end
        -- Each source's level as an absolute gain. The car's own base gains balance its
        -- recordings (often the exhaust a few dB down) and mean nothing for engine-sim's audio;
        -- kept, they made the engine-bay copy the louder one, so the exhaust sound seemed to
        -- come from the engine.
        local level = ref == "engine" and soundConfig.intake or soundConfig.exhaust
        p.base_gain = level <= 0.001 and -80 or 20 * math.log10(math.min(level, 1))
        p.gainOffset = 0 -- the game adds this to base_gain
      end
      engine:setEngineSoundParameterList(id, p, ref)
      log("I", "engineSimBridge", string.format("%s %s params: base_gain %s main_gain %s muffled %s",
        linked and "linked" or "stock", ref, tostring(p.base_gain), tostring(p.main_gain), tostring(p.muffled)))
    end
  end
end

local tunedFields = {"torqueCurve", "idleRPM", "idleAV", "maxRPM", "maxAV", "revLimiterRPM", "revLimiterAV",
  "revLimiterMaxAV", "invRevLimiterRange", "inertia", "initialInertia", "invEngInertia", "halfInvEngInertia",
  "friction", "initialFriction", "dynamicFriction", "engineBrakeTorque", "torqueData", "maxPower", "maxTorque", "maxPowerThrottleMap",
  "maxTorqueRating", "maxPhysicalAV"}

local function refreshTorqueData()
  engine.torqueData = engine:getTorqueData()
  engine.maxPower = engine.torqueData.maxPower
  engine.maxTorque = engine.torqueData.maxTorque
  engine.maxPowerThrottleMap = engine.torqueData.maxPower * 735.499
end

-- The gearbox logic (vehicleController/shiftLogic/*) copies the engine's idle and max rpm when the
-- car spawns; the tach's scale and redline, and automatic shift points, come from that copy
local function syncGearboxRPM()
  for name, module in pairs(package.loaded) do
    if type(name) == "string" and name:find("vehicleController/shiftLogic/", 1, true)
      and type(module) == "table" and module.maxRPM ~= nil then
      module.maxRPM = engine.maxRPM
      module.idleRPM = engine.idleRPM
    end
  end
end

local function restoreStockEngine()
  if stockEngine then
    for _, k in ipairs(tunedFields) do engine[k] = stockEngine[k] end
    syncGearboxRPM()
  end
  appliedProfileId = nil
end

-- p (0-based): id, idle rpm, limit rpm, inertia, friction, dynamic friction, n, rpm[n], gross torque[n]
local function applyProfile(p, count)
  local n = p[6]
  if n < 2 or count < 7 + 2 * n then return end
  if not stockEngine then
    stockEngine = {}
    for _, k in ipairs(tunedFields) do stockEngine[k] = engine[k] end
  end
  local idleRPM, limitRPM, rpmToAV = p[1], p[2], 0.104719755

  local curve, j, peakTorque = {}, 0, 0
  for rpm = 0, math.floor(limitRPM + 1000) do
    while j < n - 2 and p[7 + j + 1] < rpm do j = j + 1 end
    local r0, r1 = p[7 + j], p[7 + j + 1]
    local t = clamp((rpm - r0) / (r1 - r0), 0, 1)
    local torque = p[7 + n + j] + (p[7 + n + j + 1] - p[7 + n + j]) * t
    if rpm < idleRPM * 0.3 then torque = 0 end
    if rpm > limitRPM then torque = torque * math.max(1 - (rpm - limitRPM) / 1000, 0) end
    curve[rpm] = torque
    peakTorque = math.max(peakTorque, torque)
  end
  engine.torqueCurve = curve

  -- BeamNG damages (and eventually blows) an engine making more than its rated torque; the
  -- profiled engine can make far more than the stock one, so rate it for what it now makes
  -- The profiled engine is fictional as far as the car's parts go: its torque times the car's
  -- boost (a turbo multiplies the whole curve) can be far past the stock rating, so switch the
  -- over-torque damage off while tuned (a rating <= 0 disables it)
  if (stockEngine.maxTorqueRating or -1) > 0 then
    engine.maxTorqueRating = -1
  end

  -- Past maxPhysicalAV BeamNG bends valves and seizes the engine; it's the stock redline plus a
  -- margin, so revving to the profiled engine's limiter would count as over-revving
  if stockEngine.maxPhysicalAV then
    engine.maxPhysicalAV = math.max(stockEngine.maxPhysicalAV, limitRPM * 1.25 * rpmToAV)
  end

  engine.friction, engine.initialFriction = p[4], p[4]
  engine.dynamicFriction = p[5]
  engine.engineBrakeTorque = 0
  engine.idleRPM, engine.idleAV = idleRPM, idleRPM * rpmToAV
  engine.maxRPM, engine.maxAV = limitRPM, limitRPM * rpmToAV
  if engine.hasRevLimiter then
    engine.revLimiterRPM, engine.revLimiterAV = limitRPM, limitRPM * rpmToAV
    if engine.revLimiterMaxAVOvershoot then
      engine.revLimiterMaxAV = engine.maxAV + engine.revLimiterMaxAVOvershoot
      engine.invRevLimiterRange = 1 / engine.revLimiterMaxAVOvershoot
    end
  end
  local inertia = clamp(p[3], 0.02, 2)
  engine.inertia, engine.initialInertia = inertia, inertia
  engine.invEngInertia = 1 / inertia
  engine.halfInvEngInertia = 0.5 / inertia
  refreshTorqueData()
  syncGearboxRPM()
  appliedProfileId = p[0]
end

local function torqueUpdate(device, dt)
  origTorqueUpdate(device, dt)
  if not (active and useEngineSimTorque and esTorque and esAge < 0.5) then return end
  -- Starting, stalling and engine-off stay pure BeamNG: engine-sim torque near 0 rpm is
  -- mostly compression spikes, and feeding it back spun the crank backwards.
  if device.ignitionCoef <= 0 or device.isStalled or device.outputAV1 < device.idleAV * 0.6 then return end

  local beamTorque = device.lastOutputTorque - device.frictionTorque
  local dAV = dt * (esTorque - beamTorque) * device.invEngInertia * device.outputAVState
  for i = 1, device.activeOutputPortCount do
    local name = device.outputAVNames[device.activeOutputPorts[i]]
    device[name] = math.max(device[name] + dAV, 0)
  end
end

local function updateGFX(dt)
  if not engine then return end

  -- reset re-selects the engine's update functions, so re-wrap when that happens
  if engine.torqueUpdate ~= torqueUpdate then
    origTorqueUpdate = engine.torqueUpdate
    engine.torqueUpdate = torqueUpdate
    appliedProfileId = nil
  end

  esAge = esAge + dt
  while true do
    local data = udp:receive()
    if not data then break end
    -- 12 bytes from older engine-sim builds, 16 with the boost
    local tag = (#data == 12 or #data == ffi.sizeof(inPacket)) and data:sub(1, 4)
    if tag == "BES1" or tag == "BESN" then
      inPacket.boost = 0
      ffi.copy(inPacket, data, #data)
      esBoost = #data >= 16 and inPacket.boost or nil
      esAge = 0
    end
    if tag == "BES1" then
      esTorque = clamp(inPacket.torque, -0.5 * engine.maxTorque, 1.5 * engine.maxTorque)
    elseif tag == "BESN" then
      esTorque = nil
    elseif data:sub(1, 4) == "BESC" and #data >= 4 + 4 * 4 then
      -- version, neutral EQ, intake level, exhaust level[, profiling[, game burble]]
      ffi.copy(profileValues, data:sub(5), math.min(#data - 4, 24))
      local burble = true
      if #data >= 4 + 4 * 6 then burble = profileValues[5] ~= 0 end
      if burble ~= soundConfig.burble then
        soundConfig.burble = burble
        if markersOn then applyBurble(burble) end
      end
      local changed = (profileValues[1] ~= 0) ~= soundConfig.neutralEq
        or math.abs(profileValues[2] - soundConfig.intake) > 0.005
        or math.abs(profileValues[3] - soundConfig.exhaust) > 0.005
      soundConfig.neutralEq = profileValues[1] ~= 0
      soundConfig.intake = math.max(0, math.min(profileValues[2], 1))
      soundConfig.exhaust = math.max(0, math.min(profileValues[3], 1))
      soundConfig.tuning = #data >= 4 + 4 * 5 and profileValues[4] ~= 0
      if changed and markersOn then applySoundParams(true) end
    elseif data:sub(1, 4) == "BESP" and #data <= 4 + 4 * 256 then
      local count = math.floor((#data - 4) / 4)
      ffi.copy(profileValues, data:sub(5), count * 4)
      if active and profileValues[0] == 1 then
        if profileValues[1] ~= appliedProfileId then
          local p = {} -- 0-based copy without the enabled flag (no ffi pointer math in BeamNG)
          for i = 1, count - 1 do p[i - 1] = profileValues[i] end
          applyProfile(p, count - 1)
        end
      elseif appliedProfileId then
        restoreStockEngine()
      end
    end
  end

  -- Stock sound and engine come back while EngineSim is closed or disconnected
  local seated = playerInfo.firstPlayerSeated == true
  local linked = seated and esAge < 1
  -- engine-sim's sound while linked, except during a profiling sweep (which doesn't sound like
  -- driving): the car's own sound plays then
  local wantMarkers = linked and not soundConfig.tuning
  if wantMarkers ~= markersOn then
    useMarkerBlends(wantMarkers)
    applySoundParams(wantMarkers)
    applyBurble(not wantMarkers or soundConfig.burble)
    markersOn = wantMarkers
  end
  if not linked and appliedProfileId then restoreStockEngine() end
  active = linked
  if not seated then return end

  -- The car's boost gauge shows engine-sim's manifold pressure (psi; negative is vacuum)
  -- rather than the game's own turbo, which doesn't know what engine-sim is doing
  if linked and esBoost then
    electrics.values.turboBoost = esBoost * 0.145038
  end

  outPacket.rpm = math.abs(engine.outputAV1) * 9.549296596
  -- Pedal only: BeamNG's randomized idle-controller pulses made engine-sim surge at idle
  outPacket.throttle = math.min(engine.requestedThrottle or 0, engine.throttle or 0)
  outPacket.ignition = (engine.ignitionCoef or 0) > 0 and 1 or 0
  outPacket.volume = 1
  outPacket.pan = 0
  udp:send(ffi.string(outPacket, ffi.sizeof(outPacket)))
end

local function onExtensionLoaded()
  engine = powertrain.getDevicesByType("combustionEngine")[1]
  if not engine then return false end

  udp = socket.udp()
  udp:settimeout(0)
  udp:setpeername("127.0.0.1", enginePort)
end

-- Only runs when the vehicle reloads or despawns, which rebuilds its sounds and engine anyway.
-- Restoring blends here queued initEngineSound for sound IDs the reloaded vehicle no longer
-- drives, leaving a stray looping engine sound.
local function onExtensionUnloaded()
  if udp then udp:close() end
end

M.onExtensionLoaded = onExtensionLoaded
M.onExtensionUnloaded = onExtensionUnloaded
M.updateGFX = updateGFX

return M
