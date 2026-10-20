#include "../include/yds_sdl_window_system.h"

#include "../include/yds_sdl_input_system.h"
#include "../include/yds_sdl_monitor.h"
#include "../include/yds_sdl_window.h"

#include <SDL3/SDL.h>

ysSdlWindowSystem::ysSdlWindowSystem() : ysWindowSystem(Platform::Sdl) {
    if (!SDL_InitSubSystem(SDL_INIT_VIDEO)) {
        SDL_LogError(SDL_LOG_CATEGORY_VIDEO, "SDL video init failed: %s", SDL_GetError());
    }
}

ysSdlWindowSystem::~ysSdlWindowSystem() {
    SDL_QuitSubSystem(SDL_INIT_VIDEO);
}

ysError ysSdlWindowSystem::NewWindow(ysWindow **newWindow) {
    YDS_ERROR_DECLARE("NewWindow");

    if (newWindow == nullptr) return YDS_ERROR_RETURN(ysError::InvalidParameter);


    ysSdlWindow *sdlWindow = m_windowArray.NewGeneric<ysSdlWindow>();

    *newWindow = static_cast<ysWindow *>(sdlWindow);
    return YDS_ERROR_RETURN(ysError::None);
}

ysMonitor *ysSdlWindowSystem::NewMonitor() {
    return m_monitorArray.NewGeneric<ysSdlMonitor>();
}

void ysSdlWindowSystem::SurveyMonitors() {
    ysWindowSystem::SurveyMonitors();

    int numMonitors = 0;
    SDL_DisplayID *displays = SDL_GetDisplays(&numMonitors);
    if (displays == nullptr) return;

    for (int i = 0; i < numMonitors; i++) {
        SDL_Rect rect;
        if (!SDL_GetDisplayBounds(displays[i], &rect)) continue;

        ysMonitor *monitor = NewMonitor();
        monitor->SetOrigin(rect.x, rect.y);

        // Windows aren't created with SDL_WINDOW_HIGH_PIXEL_DENSITY, so window coordinates and
        // drawable pixels are the same units (the compositor scales on HiDPI displays)
        monitor->SetLogicalSize(rect.w, rect.h);
        monitor->SetPhysicalSize(rect.w, rect.h);
        monitor->CalculateScaling();
    }

    SDL_free(displays);
}

void ysSdlWindowSystem::ProcessMessages() {
    auto *inputSystem = static_cast<ysSdlInputSystem *>(GetInputSystem());
    auto *window = ysWindowSystem::GetWindow(0);

    // Grab all the queued events
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_EVENT_QUIT:
            case SDL_EVENT_WINDOW_CLOSE_REQUESTED:
                ysWindowSystem::Get()->CloseWindow(window);
                break;

            case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
                window->SetWindowSize(event.window.data1, event.window.data2);
                break;

            // Input events
            case SDL_EVENT_KEY_DOWN:
            case SDL_EVENT_KEY_UP:
            case SDL_EVENT_MOUSE_MOTION:
            case SDL_EVENT_MOUSE_BUTTON_DOWN:
            case SDL_EVENT_MOUSE_BUTTON_UP:
            case SDL_EVENT_MOUSE_WHEEL:
                inputSystem->ProcessEvent(event);
                break;

            default:
                // Ignore it
                break;
        }
    }
}
