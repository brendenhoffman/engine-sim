#include "../include/yds_sdl_audio_system.h"

#include "../include/yds_sdl_audio_device.h"

#include <SDL3/SDL.h>

ysSdlAudioSystem::ysSdlAudioSystem() : ysAudioSystem(API::Sdl) {
    // Make sure that the audio system is up
    if (!SDL_InitSubSystem(SDL_INIT_AUDIO)) {
        SDL_LogError(SDL_LOG_CATEGORY_AUDIO, "SDL audio init failed: %s", SDL_GetError());
    }
}

ysSdlAudioSystem::~ysSdlAudioSystem() {
    // Devices own SDL audio streams, so they must go before the subsystem shuts down
    m_devices.Clear();
    SDL_QuitSubSystem(SDL_INIT_AUDIO);
}

void ysSdlAudioSystem::EnumerateDevices() {
    ysAudioSystem::EnumerateDevices();

    // Only the default output is used. SDL follows it if the user switches outputs, and the
    // stream converts our format to whatever the hardware runs at.
    auto *newDevice = m_devices.NewGeneric<ysSdlAudioDevice>();
    if (!newDevice->Open()) {
        SDL_LogError(SDL_LOG_CATEGORY_AUDIO, "Could not open audio output: %s", SDL_GetError());
    }
}

void ysSdlAudioSystem::ConnectDevice(ysAudioDevice *device, ysWindow *windowAssociation) {
    ysAudioSystem::ConnectDevice(device, windowAssociation);
}

void ysSdlAudioSystem::ConnectDeviceConsole(ysAudioDevice *device) {
    ysAudioSystem::ConnectDeviceConsole(device);
}

void ysSdlAudioSystem::DisconnectDevice(ysAudioDevice *device) {
    ysAudioSystem::DisconnectDevice(device);
}
