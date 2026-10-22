#include <gtest/gtest.h>

#include "engine_host.h"

#include "engine.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <thread>

namespace {
    HostPaths testPaths(const std::filesystem::path &dir) {
        HostPaths paths;
        paths.dataRoot = ENGINE_SIM_TEST_DATA_ROOT;
        paths.userData = (dir / "user").string();
        paths.outputPath = (dir / "out").string();
        paths.configPath = (dir / "config").string();
        std::filesystem::create_directories(paths.userData);
        std::filesystem::create_directories(paths.outputPath);
        return paths;
    }
}

// If the physics diverges, the host reloads the engine instead of producing garbage forever
TEST(EngineHostTests, ReloadsAnEngineWhosePhysicsBlewUp) {
    const auto dir = std::filesystem::temp_directory_path() / "engine-sim-host-test";
    std::filesystem::remove_all(dir);

    EngineHost host(testPaths(dir));
    const std::string script = std::string(ENGINE_SIM_TEST_DATA_ROOT) + "/assets/engines/atg-video-2/07_gm_ls.mr";
    ASSERT_TRUE(host.loadScript(script).engineLoaded);
    host.start();

    {
        auto lock = host.lock();
        host.engine()->getOutputCrankshaft()->m_body.v_theta = std::nan("");
    }

    // The simulation thread notices within a frame and posts a reload for the owner thread
    bool recovered = false;
    for (int i = 0; i < 200 && !recovered; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        host.runPending();
        auto lock = host.lock();
        recovered = std::isfinite(host.engine()->getSpeed()) && host.loadedScript() == script;
        if (recovered) {
            // And it keeps running afterwards
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            lock.lock();
            recovered = std::isfinite(host.engine()->getSpeed());
        }
    }
    host.stop();
    EXPECT_TRUE(recovered);

    std::filesystem::remove_all(dir);
}
