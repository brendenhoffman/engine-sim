project(
  'engine-sim',
  'cpp',
  version: '0.1.12',
  license: 'MIT',
  meson_version: '>=1.1',
  default_options: [
    'cpp_std=c++17',
    'buildtype=release',
    'b_ndebug=if-release',
    'warning_level=1',
  ],
)

cpp = meson.get_compiler('cpp')
threads_dep = dependency('threads')

build_gui = get_option('gui').allowed()

# Where installed binaries look for assets/, es/ and delta/ (fonts + shaders)
data_root = get_option('data_root')
if data_root == ''
  data_root = get_option('prefix') / get_option('datadir') / 'engine-sim'
endif

build_info = configuration_data()
build_info.set_quoted('ENGINE_SIM_PROJECT_VERSION', meson.project_version())
build_info.set_quoted('ENGINE_SIM_SYSTEM_NAME', host_machine.system())
build_info.set_quoted('ENGINE_SIM_DATA_ROOT', data_root)
# Fallback so binaries run straight from the build dir without installing
build_info.set_quoted('ENGINE_SIM_SOURCE_DATA_ROOT', meson.project_source_root() / 'data')

subdir('third_party')
subdir('sim')
subdir('scripting')
subdir('host')

if build_gui
  subdir('frontend/gui')
endif
if get_option('headless')
  subdir('frontend/headless')
endif
build_control_panel = get_option('control_panel').allowed() and dependency('sdl3', version: '>=3.2', required: get_option('control_panel')).found()
if build_control_panel
  subdir('frontend/control')
endif
subdir('beamng')

install_subdir('data', install_dir: data_root, strip_directory: true)

gtest_dep = dependency('gtest', main: true, required: get_option('tests'))
if gtest_dep.found()
  subdir('tests')
endif

summary(
  {
    'GUI': build_gui,
    'Headless': get_option('headless'),
    'Control panel': build_control_panel,
    'BeamNG plugin': get_option('beamng_plugin'),
    'Tests': gtest_dep.found(),
    'Data root': data_root,
  },
  section: 'engine-sim',
)
