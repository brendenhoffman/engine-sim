engine_sim_tests = executable(
  'engine-sim-tests',
  files(
    'function_test.cpp',
    'gas_system_tests.cpp',
    'synthesizer_tests.cpp',
  ),
  dependencies: [sim_dep, gtest_dep],
)

test('engine-sim-tests', engine_sim_tests, protocol: 'gtest', timeout: 300)
