# piranha (vendored, see ../VENDORED.md)
flex = find_program('flex')
bison = find_program('bison')

piranha_parser = custom_target(
  'piranha-parser',
  input: 'flex-bison/specification.y',
  output: ['parser.auto.cpp', 'parser.auto.h'],
  command: [bison, '--no-lines', '-o', '@OUTPUT0@', '--defines=@OUTPUT1@', '@INPUT@'],
)

piranha_scanner = custom_target(
  'piranha-scanner',
  input: 'flex-bison/scanner.l',
  output: ['scanner.auto.cpp', 'scanner.auto.h'],
  command: [flex, '--noline', '-o', '@OUTPUT0@', '--header-file=@OUTPUT1@', '@INPUT@'],
)

piranha_lib = static_library(
  'piranha',
  files(
    'src/assembly.cpp',
    'src/channel_node.cpp',
    'src/channel_type.cpp',
    'src/compilation_error.cpp',
    'src/compiler.cpp',
    'src/error_list.cpp',
    'src/float_conversions.cpp',
    'src/fundamental_types.cpp',
    'src/int_conversions.cpp',
    'src/ir_attribute.cpp',
    'src/ir_attribute_definition.cpp',
    'src/ir_attribute_definition_list.cpp',
    'src/ir_attribute_list.cpp',
    'src/ir_binary_operator.cpp',
    'src/ir_compilation_unit.cpp',
    'src/ir_context_tree.cpp',
    'src/ir_import_statement.cpp',
    'src/ir_node.cpp',
    'src/ir_node_definition.cpp',
    'src/ir_parser_structure.cpp',
    'src/ir_unary_operator.cpp',
    'src/ir_value.cpp',
    'src/language_rules.cpp',
    'src/memory_tracker.cpp',
    'src/no_op_node.cpp',
    'src/node.cpp',
    'src/node_allocator.cpp',
    'src/node_container.cpp',
    'src/node_graph.cpp',
    'src/node_output.cpp',
    'src/node_program.cpp',
    'src/path.cpp',
    'src/standard_allocator.cpp',
    'src/string_conversions.cpp',
    'src/vector_constructor.cpp',
  ),
  piranha_parser,
  piranha_scanner,
  include_directories: include_directories('include'),
)

piranha_dep = declare_dependency(
  link_with: piranha_lib,
  # scanner.h includes the generated <parser.auto.h> (build dir of '.'), which in turn includes
  # "../include/ir_*.h"; 'src' makes that relative path land on piranha/include. piranha/include
  # itself stays off the path: its compiler.h/node.h would shadow the scripting headers.
  sources: [piranha_parser[1]],
  include_directories: [third_party_inc, include_directories('.', 'src')],
)
