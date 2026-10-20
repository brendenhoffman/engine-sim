# Engine host: script loading, engine lifetime and the simulation thread. No graphics.
build_info_h = configure_file(
  input: 'include/build_info.h.in',
  output: 'build_info.h',
  configuration: build_info,
)

host_inc = include_directories('include', '.')

host_lib = static_library(
  'engine-sim-host',
  files(
    'src/engine_host.cpp',
    'src/wav_reader.cpp',
  ),
  build_info_h,
  include_directories: host_inc,
  dependencies: [sim_dep, scripting_dep],
)

host_dep = declare_dependency(
  link_with: host_lib,
  sources: build_info_h,
  include_directories: host_inc,
  dependencies: [sim_dep, scripting_dep],
)
