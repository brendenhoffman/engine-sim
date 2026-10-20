engine_sim_tests = executable(
  'engine-sim-tests',
  files(
    'function_test.cpp',
    'gas_system_tests.cpp',
    'host_settings_tests.cpp',
    'synthesizer_tests.cpp',
    'wav_reader_tests.cpp',
  ),
  cpp_args: ['-DENGINE_SIM_TEST_DATA_ROOT="' + (meson.project_source_root() / 'data') + '"'],
  dependencies: [host_dep, gtest_dep],
)

test('engine-sim-tests', engine_sim_tests, protocol: 'gtest', timeout: 300)
