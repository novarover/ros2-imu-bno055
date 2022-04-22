#pragma once

#include <thread>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <iostream>
#include <functional>
#include <unistd.h>
#include <csignal>
#include <sys/types.h>

namespace watchdog {

class Watchdog {
  public:
    Watchdog();
    Watchdog(std::function<void()> _callback);
    ~Watchdog();
    void start(std::chrono::milliseconds _interval);
    void stop();
    void refresh();

  private:
    std::chrono::milliseconds interval;
    std::atomic<bool> isRunning;
    std::thread thread;
    std::function<void()> callback;
    std::mutex mutex;
    std::chrono::steady_clock::time_point lastRefreshTime;
    std::condition_variable stopCondition;
    void loop();
};

}

