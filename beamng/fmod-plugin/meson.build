# Drop-in replacement for BeamNG's BinLinux/libBeamNGBlendLayerDSP.so
zydis_dep = dependency('zydis', version: '>=4.0', default_options: ['default_library=static'])

beamng_plugin = shared_library(
  'BeamNGBlendLayerDSP',
  files(
    'blend_layer_dsp.cpp',
    'inline_hook.cpp',
  ),
  dependencies: [zydis_dep, threads_dep, cpp.find_library('dl', required: false)],
  # Only FMODGetDSPDescription is exported, and the C++ runtime is linked in so the plugin
  # doesn't depend on the libstdc++ inside Steam's runtime
  gnu_symbol_visibility: 'hidden',
  link_args: [
    '-static-libstdc++',
    '-static-libgcc',
    '-Wl,--no-undefined',
    '-Wl,--version-script=' + (meson.current_source_dir() / 'exports.map'),
  ],
  link_depends: files('exports.map'),
  install: true,
  install_dir: get_option('libdir') / 'engine-sim' / 'beamng',
)

# Loads the plugin into a fake game that exports the same FMOD functions (x86-64 only)
if host_machine.cpu_family() == 'x86_64'
  fake_fmod_host = executable(
    'fake-fmod-host',
    files('test/fake_fmod_host.cpp'),
    dependencies: [threads_dep, cpp.find_library('dl', required: false)],
    export_dynamic: true,
    override_options: ['optimization=0'],
  )

  test('beamng-plugin', fake_fmod_host,
    args: [beamng_plugin],
    # Its own audio port, so it works while the game (and the real plugin) is running
    env: {'XDG_STATE_HOME': meson.current_build_dir(), 'ENGINE_SIM_AUDIO_PORT': '45710'},
    is_parallel: false,
  )
endif
