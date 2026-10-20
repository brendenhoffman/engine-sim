# Vendored libraries

These were git submodules of bobsayshilol/engine-sim (sdl-build). They are now plain
directories because three of them pinned commits that only exist on bobsayshilol's forks,
and delta-studio is being modified heavily (Windows backends removed, SDL3 port).

All are MIT licensed by Ange Yaghi; see each LICENSE.

| Library | Source | Commit imported |
| --- | --- | --- |
| delta-studio | github.com/bobsayshilol/delta-studio (fork of ange-yaghi/delta-studio) | 137566fe51ebf47cc1980e747167e331b1edc5dc |
| piranha | github.com/bobsayshilol/piranha (fork of ange-yaghi/piranha) | 252140a067bac0f843b3ee2faaa307fc776633bb |
| simple-2d-constraint-solver | github.com/bobsayshilol/simple-2d-constraint-solver (fork of ange-yaghi/simple-2d-constraint-solver) | 7e7e8682489ea47838ebe45fa6bf4cfbccade260 |
| csv-io | github.com/ange-yaghi/csv-io | 2112c55e1e831c8f7a1b91de23722b7926ad3d00 |
| stb_image.h | github.com/nothings/stb (added to delta-studio for texture loading) | a7d4c2175630e1ea5b6f8dd00ab96fed0b97b96d |

## Local changes

- All CMake files, demos, tests, docs and prebuilt Windows libraries/DLLs removed.
- delta-studio: Direct3D 10/11, Vulkan, DirectSound and Win32 backends removed, along with
  their enum values. Fonts and shaders moved to data/delta/. SDL_image replaced by stb_image.
  Missing <cstdint> includes added. ysSubclassRegistry array sized correctly (was one short).
- piranha: bundled FlexLexer.h dropped in favour of the system one.
- Windows/MSVC preprocessor branches stripped everywhere with unifdef.
