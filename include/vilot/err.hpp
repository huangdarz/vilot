#pragma once
#include "pros/rtos.hpp"

namespace vilot {

class ErrorHandler {
 public:
  ErrorHandler(const ErrorHandler&) = delete;
  ErrorHandler& operator=(const ErrorHandler&) = delete;

  static ErrorHandler& get_instance();

  void register_handler(std::function<void()> on_error);

  void report();

 private:
  explicit ErrorHandler();

  [[noreturn]] void update() const;

  std::function<void()> on_error;
  pros::Task err_task;
};

}  // namespace vilot