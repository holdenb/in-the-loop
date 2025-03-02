#pragma once

#include <iostream>
#include <mutex>
#include <string_view>

namespace util
{

class ILogger
{
public:
  virtual ~ILogger() = default;
  virtual void log(std::string_view message) = 0;
};

//====================================================================//
class Logger : public ILogger
{
public:
  virtual void log(std::string_view message) override
  {
    std::cout << message << std::endl;
  }
};

//====================================================================//
class NoOpLogger : public ILogger
{
public:
  NoOpLogger() = default;
  void log(std::string_view _) override { return; }
};

//====================================================================//
class TSLogger : public ILogger
{
public:
  TSLogger() : m_mutex(){};

  void log(std::string_view message) override
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::cout << message << std::endl;
  }

private:
  std::mutex m_mutex;
};

}  // namespace util
