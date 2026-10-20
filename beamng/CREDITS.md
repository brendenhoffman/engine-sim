# Credits

The BeamNG side of this project is ported from
[BeamEngineSim](https://github.com/spidermodders/BeamEngineSim) (MIT, see
`LICENSE.BeamEngineSim`), imported at commit `23d0ec12bb8d73a2785b8f887aee60d6e342c835`.

- `mod/` is BeamEngineSim's `EngineSimulatorMod`. The vehicle Lua adds sound settings sent by
  engine-sim (`BESC`): flattening the car's EQ while linked, and muting the intake or exhaust event.
- `fmod-plugin/blend_layer_dsp.cpp` is BeamEngineSim's `AudioPlugin/blend_layer_dsp.cpp`, ported
  to Linux: FMOD functions are found with `dlsym` instead of by scanning the executable, and
  MinHook is replaced by a small Zydis-based inline hook (`inline_hook.cpp`).
- The UDP bridge and engine profiling in `host/` follow BeamEngineSim's engine-sim changes.

BeamNG.drive is a trademark of BeamNG GmbH. This project is not affiliated with BeamNG GmbH,
spidermodders or AngeTheGreat.
