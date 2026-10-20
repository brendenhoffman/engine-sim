# Compiler for .mr engine scripts, built on piranha
scripting_lib = static_library(
  'engine-sim-scripting',
  files(
    'src/channel_types.cpp',
    'src/compiler.cpp',
    'src/engine_context.cpp',
    'src/language_rules.cpp',
  ),
  include_directories: include_directories('include'),
  dependencies: [sim_dep, piranha_dep],
)

scripting_dep = declare_dependency(
  link_with: scripting_lib,
  include_directories: include_directories('include'),
  dependencies: [sim_dep, piranha_dep],
)
