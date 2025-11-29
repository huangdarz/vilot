#include <functional>
#include <utility>
#include "pros/rtos.hpp"
#include "vilot/err.hpp"

using namespace vilot;

ErrorHandler::ErrorHandler()
    : on_error(nullptr),
      err_task([this] { update(); }, TASK_PRIORITY_MAX,
               TASK_STACK_DEPTH_DEFAULT, "Error Handler") {}

ErrorHandler& ErrorHandler::get_instance() {
  static ErrorHandler instance;
  return instance;
}

void ErrorHandler::register_handler(std::function<void()> on_error) {
  this->on_error = std::move(on_error);
}

void ErrorHandler::report() {
  this->err_task.notify();
}

[[noreturn]] void ErrorHandler::update() const {
  for (;;) {
    pros::Task::notify_take(true, TIMEOUT_MAX);
    if (this->on_error) {
      this->on_error();
    }
  }
}
