#include "inline_hook.h"

#include <Zydis/Zydis.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sys/mman.h>
#include <unistd.h>

namespace {
    // jmp qword [rip+0] ; .quad destination
    constexpr size_t JumpSize = 14;

    void writeAbsoluteJump(uint8_t *at, const void *destination) {
        at[0] = 0xFF;
        at[1] = 0x25;
        std::memset(at + 2, 0, 4);
        const uint64_t address = reinterpret_cast<uint64_t>(destination);
        std::memcpy(at + 6, &address, sizeof(address));
    }

    std::string hex(const void *p) {
        char buffer[32];
        std::snprintf(buffer, sizeof(buffer), "%p", p);
        return buffer;
    }
}

bool installInlineHook(void *target, void *detour, void **original, std::string *error) {
    ZydisDecoder decoder;
    ZydisDecoderInit(&decoder, ZYDIS_MACHINE_MODE_LONG_64, ZYDIS_STACK_WIDTH_64);

    auto *code = static_cast<uint8_t *>(target);

    // Whole instructions covering the jump we're about to write
    size_t stolen = 0;
    while (stolen < JumpSize) {
        ZydisDecodedInstruction instruction;
        if (ZYAN_FAILED(ZydisDecoderDecodeInstruction(&decoder, nullptr, code + stolen, ZYDIS_MAX_INSTRUCTION_LENGTH, &instruction))) {
            *error = "could not decode instruction at " + hex(code + stolen);
            return false;
        }

        if (instruction.attributes & ZYDIS_ATTRIB_IS_RELATIVE) {
            *error = "position-dependent instruction in function prologue at " + hex(code + stolen);
            return false;
        }

        switch (instruction.meta.category) {
            case ZYDIS_CATEGORY_RET:
            case ZYDIS_CATEGORY_CALL:
            case ZYDIS_CATEGORY_UNCOND_BR:
            case ZYDIS_CATEGORY_COND_BR:
                *error = "function too short to hook at " + hex(code + stolen);
                return false;
            default:
                break;
        }

        stolen += instruction.length;
    }

    // Trampoline: the stolen instructions, then a jump back to the rest of the function
    const long pageSize = sysconf(_SC_PAGESIZE);
    void *trampolineMemory = mmap(nullptr, pageSize, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (trampolineMemory == MAP_FAILED) {
        *error = "could not allocate trampoline";
        return false;
    }

    auto *trampoline = static_cast<uint8_t *>(trampolineMemory);
    std::memcpy(trampoline, code, stolen);
    writeAbsoluteJump(trampoline + stolen, code + stolen);
    if (mprotect(trampolineMemory, pageSize, PROT_READ | PROT_EXEC) != 0) {
        munmap(trampolineMemory, pageSize);
        *error = "could not make trampoline executable";
        return false;
    }

    // Make the target's page(s) writable, keeping them executable: other code shares them
    const uintptr_t first = reinterpret_cast<uintptr_t>(code) & ~static_cast<uintptr_t>(pageSize - 1);
    const uintptr_t last = (reinterpret_cast<uintptr_t>(code) + stolen - 1) & ~static_cast<uintptr_t>(pageSize - 1);
    const size_t length = last - first + pageSize;
    if (mprotect(reinterpret_cast<void *>(first), length, PROT_READ | PROT_WRITE | PROT_EXEC) != 0) {
        munmap(trampolineMemory, pageSize);
        *error = "could not unprotect target code";
        return false;
    }

    // The trampoline must be published before any thread can reach the detour
    *original = trampoline;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);

    // A thread calling the function while we patch must never run half-written code: park the
    // entry on a 2-byte jump-to-self, write the rest, then swap in the head of the real jump.
    // Bytes left over from partially overwritten instructions become int3.
    uint8_t patch[JumpSize + ZYDIS_MAX_INSTRUCTION_LENGTH];
    writeAbsoluteJump(patch, detour);
    std::memset(patch + JumpSize, 0xCC, stolen - JumpSize);

    const uint16_t spin = 0xFEEB; // jmp $ (EB FE)
    __atomic_store_n(reinterpret_cast<uint16_t *>(code), spin, __ATOMIC_SEQ_CST);
    std::memcpy(code + 2, patch + 2, stolen - 2);
    __atomic_thread_fence(__ATOMIC_SEQ_CST);

    uint16_t head;
    std::memcpy(&head, patch, 2);
    __atomic_store_n(reinterpret_cast<uint16_t *>(code), head, __ATOMIC_SEQ_CST);

    mprotect(reinterpret_cast<void *>(first), length, PROT_READ | PROT_EXEC);
    __builtin___clear_cache(reinterpret_cast<char *>(code), reinterpret_cast<char *>(code + stolen));

    return true;
}
