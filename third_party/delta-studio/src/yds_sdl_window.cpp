#include "../include/yds_sdl_window.h"

#include <SDL3/SDL.h>

ysSdlWindow::ysSdlWindow() : ysWindow(Platform::Sdl) {
    /* void */
}

ysSdlWindow::~ysSdlWindow() {
    /* void */
}

ysError ysSdlWindow::InitializeWindow(ysWindow *parent, std::string title, WindowStyle style, int x, int y, int width, int height, ysMonitor *monitor, ysContextObject::DeviceAPI api) {
    YDS_ERROR_DECLARE("InitializeWindow");

    if (!CheckCompatibility(parent)) return YDS_ERROR_RETURN(ysError::IncompatiblePlatforms);

    YDS_NESTED_ERROR_CALL(ysWindow::InitializeWindow(parent, title, style, x, y, width, height, monitor, api));

    SDL_WindowFlags flags = 0;

    if (m_windowState == WindowState::Hidden)
        flags |= SDL_WINDOW_HIDDEN;

    switch (style) {
    case WindowStyle::Windowed:
        flags |= SDL_WINDOW_RESIZABLE;
        break;
    case WindowStyle::Fullscreen:
        flags |= SDL_WINDOW_FULLSCREEN;
        break;
    case WindowStyle::Popup:
        flags |= SDL_WINDOW_BORDERLESS;
        break;
    case WindowStyle::Unknown:
        // do nothing
        break;
    }

    switch (api) {
    case ysContextObject::DeviceAPI::OpenGL4_0:
        flags |= SDL_WINDOW_OPENGL;
        break;
    default:
        break;
    }

    // Position is left to the compositor (Wayland doesn't allow clients to place windows)
    m_window = SDL_CreateWindow(title.c_str(), width, height, flags);
    if (m_window == nullptr) {
        SDL_LogError(SDL_LOG_CATEGORY_VIDEO, "SDL_CreateWindow failed: %s", SDL_GetError());
        return YDS_ERROR_RETURN(ysError::CouldNotCreateWindow);
    }

    // The compositor may not honour the requested size, so start from what we actually got
    int pixelWidth = width, pixelHeight = height;
    SDL_GetWindowSizeInPixels(m_window, &pixelWidth, &pixelHeight);
    SetWindowSize(pixelWidth, pixelHeight);

    return YDS_ERROR_RETURN(ysError::None);
}

void ysSdlWindow::SetState(WindowState state) {
    if (state == m_windowState)
        return;

    ysWindow::SetState(state);

    switch (state) {
    case WindowState::Visible:
        SDL_ShowWindow(m_window);
        break;
    case WindowState::Hidden:
        SDL_HideWindow(m_window);
        break;
    case WindowState::Maximized:
        SDL_MaximizeWindow(m_window);
        break;
    case WindowState::Minimized:
        SDL_MinimizeWindow(m_window);
        break;
    case WindowState::Closed:
        SDL_DestroyWindow(m_window);
        break;
    case WindowState::Unknown:
        // do nothing
        break;
    }
}

bool ysSdlWindow::SetWindowStyle(WindowStyle style) {
    if (!ysWindow::SetWindowStyle(style)) return false;

    // SDL3 fullscreen defaults to borderless at desktop resolution and restores the previous
    // windowed size on exit. The resulting SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED updates our size.
    if (style == WindowStyle::Windowed) {
        SDL_SetWindowFullscreen(m_window, false);
        SDL_RaiseWindow(m_window);
    } else if (style == WindowStyle::Fullscreen) {
        SDL_SetWindowFullscreen(m_window, true);
    }

    return true;
}

void ysSdlWindow::SetTitle(std::string title) {
    SDL_SetWindowTitle(m_window, title.c_str());
}
