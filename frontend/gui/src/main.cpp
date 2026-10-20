#include "engine_sim_application.h"

static void runApp(void *handle, ysContextObject::DeviceAPI api) {
    EngineSimApplication application;
    application.initialize(handle, api);
    application.run();
    application.destroy();
}

int main() {
    runApp(nullptr, ysContextObject::DeviceAPI::OpenGL4_0);
    return 0;
}

