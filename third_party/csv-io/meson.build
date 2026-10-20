# csv-io (vendored, see ../VENDORED.md)
csv_io_lib = static_library(
  'csv-io',
  files(
    'src/csv_data.cpp',
    'src/information.cpp',
  ),
  include_directories: include_directories('include'),
)

csv_io_dep = declare_dependency(
  link_with: csv_io_lib,
  include_directories: third_party_inc,
)
