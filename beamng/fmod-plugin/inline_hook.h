#ifndef ENGINE_SIM_BEAMNG_INLINE_HOOK_H
#define ENGINE_SIM_BEAMNG_INLINE_HOOK_H

#include <string>

// Redirects an x86-64 function to `detour` by overwriting its first instructions with an
// absolute jump. `*original` receives a trampoline that runs the overwritten instructions and
// jumps back, so the detour can call through to the real function.
//
// Refuses (returns false, sets `error`) rather than guessing when the first instructions can't
// be moved safely: RIP-relative operands, branches or returns inside the overwritten bytes.
bool installInlineHook(void *target, void *detour, void **original, std::string *error);

#endif /* ENGINE_SIM_BEAMNG_INLINE_HOOK_H */
