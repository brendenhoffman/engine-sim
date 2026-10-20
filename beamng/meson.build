# BeamNG integration: the FMOD plugin, the mod zip, and helpers to install them into the game
if get_option('beamng_plugin')
  subdir('fmod-plugin')
endif

python = find_program('python3')

mod_zip = custom_target(
  'beamng-mod',
  input: files(
    'mod/art/sound/engineSim/engineSimEngine.sfxBlend2D.json',
    'mod/art/sound/engineSim/engineSimExhaust.sfxBlend2D.json',
    'mod/art/sound/engineSim/engineSimMarkerEngine.wav',
    'mod/art/sound/engineSim/engineSimMarkerExhaust.wav',
    'mod/lua/ge/extensions/engineSimBridgeGE.lua',
    'mod/lua/vehicle/extensions/auto/engineSimBridge.lua',
    'mod/mod_info/ENGINESIMBRIDGE/icon.jpg',
    'mod/mod_info/ENGINESIMBRIDGE/info.json',
    'mod/scripts/engineSimBridge/modScript.lua',
  ),
  output: 'engineSimBridge.zip',
  command: [python, files('../tools/make-mod-zip.py'), meson.current_source_dir() / 'mod', '@OUTPUT@'],
  build_by_default: true,
  install: true,
  install_dir: get_option('datadir') / 'engine-sim' / 'beamng',
)

install_script = files('../tools/beamng-install.py')

if get_option('beamng_plugin')
  run_target('beamng-install',
    command: [python, install_script, 'install', '--plugin', beamng_plugin, '--mod', mod_zip],
  )
endif
run_target('beamng-uninstall', command: [python, install_script, 'uninstall'])
run_target('beamng-status', command: [python, install_script, 'status'])
