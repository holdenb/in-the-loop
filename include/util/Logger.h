#pragma once

#include <chrono>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <string_view>

namespace util
{

class ILogger
{
public:
  virtual ~ILogger() = default;
  virtual void debug(std::string_view message) = 0;
  virtual void info(std::string_view message) = 0;
  virtual void warning(std::string_view message) = 0;
  virtual void error(std::string_view message) = 0;

protected:
  enum class LogLevel
  {
    DEBUG,
    INFO,
    WARNING,
    ERROR
  };

  std::string toString(LogLevel log_level)
  {
    std::string ret;

    switch (log_level)
    {
      case LogLevel::DEBUG: return "DEBUG";
      case LogLevel::INFO: return "INFO";
      case LogLevel::WARNING: return "WARNING";
      case LogLevel::ERROR: return "ERROR"; break;
    }

    return ret;
  }
};

//====================================================================//
class NoOpLogger : public ILogger
{
public:
  NoOpLogger() = default;
  void debug(std::string_view _) override { return; }
  void info(std::string_view _) override { return; }
  void warning(std::string_view _) override { return; }
  void error(std::string_view _) override { return; }
};

//====================================================================//
class Logger : public ILogger
{
public:
  virtual void debug(std::string_view message) override
  {
    _log(message, LogLevel::DEBUG);
  }

  virtual void info(std::string_view message) override
  {
    _log(message, LogLevel::INFO);
  }

  virtual void warning(std::string_view message) override
  {
    _log(message, LogLevel::WARNING);
  }

  virtual void error(std::string_view message) override
  {
    _log(message, LogLevel::ERROR);
  }

private:
  void _log(std::string_view message, LogLevel log_level)
  {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);

    std::tm time_info;

    // Use local time (thread-safe on >= c++20)
#ifdef _WIN32
    localtime_s(&time_info, &now_c);
#else
    localtime_r(&now_c, &time_info);
#endif

    std::cout << "[" << std::put_time(&time_info, "%Y-%m-%d %H:%M:%S") << "]["
              << toString(log_level) << "] " << message << std::endl;
  };
};

}  // namespace util
