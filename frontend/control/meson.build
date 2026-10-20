# engine-sim-control: Dear ImGui on SDL3's GPU API (Vulkan on Linux)
imgui_dep = dependency('imgui', version: '>=1.92', default_options: [
  'default_library=static',
  'sdl3=enabled',
  'sdl3_gpu=enabled',
  'opengl=disabled',
  'vulkan=disabled',
  'sdl2=disabled',
  'sdl2_renderer=disabled',
  'sdl3_renderer=disabled',
  'glfw=disabled',
  'webgpu=disabled',
  'allegro5=disabled',
  'freetype=disabled',
])

executable(
  'engine-sim-control',
  files(
    'control_client.cpp',
    'launcher.cpp',
    'main.cpp',
  ),
  dependencies: [
    imgui_dep,
    dependency('sdl3', version: '>=3.2'),
    dependency('nlohmann_json', version: '>=3.10'),
    threads_dep,
  ],
  install: true,
)
