/*
Copyright (c) 2025 Patryk Dudziński

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
 * Authors: Patryk Dudziński
 */

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <fcntl.h>

#include <unistd.h>

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "can_device/can_device.hpp"
#include "shared_types_nomad/status.hpp"

using namespace nomad_core;
using namespace nomad_hardware;

std::vector<std::shared_ptr<CanDriver>> CanDriver::global_can_driver;


CanDriver::CanDriver(const std::string &can_interface, bool _threaded, uint32_t _timeout_us, size_t _queue_size, size_t _buffer_size)
: can_interface_name(can_interface), threaded(_threaded), queue_size(_queue_size), socket_fd(-1),
  buffer_size(_buffer_size), is_open(false), timeout_us(_timeout_us), addr({}), ifr({}) {
}

CanDriver::~CanDriver() {
  // close_can();
  internall_close_can();
}

Result<std::shared_ptr<CanDriver>>
CanDriver::Make(const std::string &can_interface, bool threaded, uint32_t timeout_us, size_t _queue_size, size_t _buffer_size) {
  if(can_interface.empty()) {
    return Status::Invalid("CAN interface is empty");
  }
  if(timeout_us <= 0) {
    return Status::Invalid("Invalid timeout value");
  }

  if(_queue_size <= 0) {
    return Status::Invalid("Invalid queue size");
  }

  if(_buffer_size <= CANFD_MAX_DLEN) {
    return Status::Invalid("Invalid buffer size");
  }

  // auto mayby_can = std::find_if(global_can_driver.begin(), global_can_driver.end(),
  //                               [can_interface](const std::shared_ptr<CanDriver> &driver) {
  //                                 return driver->can_interface_name == can_interface;
  //                               });
  // // if we have already created a driver for this interface, return it
  // if(mayby_can != global_can_driver.end()) {
  //   return Result<std::shared_ptr<CanDriver>>::OK(*mayby_can);
  // }
  // if the driver is not found, create a new one
  auto driver =
  std::shared_ptr<CanDriver>(new CanDriver(can_interface, threaded, timeout_us, _queue_size, _buffer_size));
  // global_can_driver.push_back(driver);
  return Result<std::shared_ptr<CanDriver>>::OK(driver);
}

Status CanDriver::open_can() {
  if(is_open) {
    return Status::OK();
    // return Status::Invalid("CAN socket is already open");
  }

  // check if we have registered callbacks
  if(callbacks_.empty() && threaded) {
    return Status::Invalid("No can callbacks registered");
  }

  // Open the CAN socket
  socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if(socket_fd < 0) {
    return Status::ExecutionError("CanDriver: Failed to open CAN socket");
  }

  // Set the socket to non-blocking mode
  strncpy(ifr.ifr_name, can_interface_name.c_str(), can_interface_name.size());
  if(ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
    return Status::ExecutionError("CanDriver: Failed to get CAN interface index name:" + can_interface_name);
  }

  // creat filter for all the registered callbacks
  std::vector<can_filter> filters;

  for(auto &callback : callbacks_) {
    can_filter filter;
    filter.can_id   = callback.first;
    filter.can_mask = callback.first | CAN_RTR_FLAG;
    filters.push_back(filter);
  }

  if(setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), filters.size() * sizeof(can_filter)) < 0) {
    return Status::ExecutionError("CanDriver: Failed to set CAN filter/mask");
  }

  // set buffer
  if(setsockopt(socket_fd, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size)) < 0) {
    return Status::ExecutionError("CanDriver: Failed to set CAN buffer size  setsockopt failed");
  }

  // set Timeout
  timeval timeout;
  timeout.tv_sec  = 0;
  timeout.tv_usec = timeout_us;
  if(setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout))) {
    internall_close_can();
    return Status::ExecutionError("CanDriver: Set CAN socket option 'SO_RCVTIMEO' failed !");
  }

  // Bind the socket to the CAN interface
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if(bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    return Status::ExecutionError("CanDriver: Socket Bind Error");
  }

  if(threaded) {
    // start the threads if threads are enabled
    run_thread               = true;
    can_read_thread          = std::thread(&CanDriver::handle_can_transmission, this);
    can_perform_tasks_thread = std::thread(&CanDriver::handle_can_tasks, this);

    if(!can_read_thread.joinable()) {
      internall_close_can();
      return Status::ExecutionError("CanDriver: Failed to create CAN thread");
    }

    if(!can_perform_tasks_thread.joinable()) {
      internall_close_can();
      return Status::ExecutionError("CanDriver: Failed to create CAN thread");
    }
  }

  is_open = true;
  return Status::OK();
}

Status CanDriver::close_can(std::shared_ptr<CanDriver> &can_driver) {
  if(!can_driver) {
    return Status::Invalid("CAN driver is null");
  }

  // auto maybe_can = std::find_if(global_can_driver.begin(), global_can_driver.end(),
  //                               [can_driver](const std::shared_ptr<CanDriver> &driver) {
  //                                 return driver->can_interface_name == can_driver->can_interface_name;
  //                               });

  // RCLCPP_INFO(rclcpp::get_logger("CanDriver"), "Current global_can_driver elements:");
  // for(const auto &driver : global_can_driver) {
  //   RCLCPP_INFO(rclcpp::get_logger("CanDriver"), "  Interface: %s, use_count: %zu",
  //               driver->can_interface_name.c_str(), driver.use_count());
  // }

  // RCLCPP_INFO(rclcpp::get_logger("CanDriver"), "Closing CAN driver for interface: %s",
  //             can_driver->can_interface_name.c_str());

  // if(maybe_can == global_can_driver.end()) {
  //   return Status::Invalid("CAN driver not found");
  // }


  // RCLCPP_INFO(rclcpp::get_logger("CanDriver"), "before maybe_can->use_count() : %zu", maybe_can->use_count());

  // we reset the pointer to the driver, so it will remove it references.
  if(can_driver) {
    can_driver.reset();
  }

  // RCLCPP_INFO(rclcpp::get_logger("CanDriver"), "after maybe_can->use_count() : %zu", maybe_can->use_count());

  // we check if the driver is used by other instances
  // if(maybe_can->use_count() > 1) {
  //   return Status::OK();
  // }


  // if the driver is not used by other instances, we close it
  // mayby_can->get()->internall_close_can();

  // remove the driver from the global list
  // global_can_driver.erase(maybe_can);
  // this will drop the last reference to the driver,
  // so it will trigger the destructor
  // and close the socket

  return Status::OK();
}

Status CanDriver::internall_close_can() {
  run_thread = false;

  if(can_read_thread.joinable()) {
    can_read_thread.join();
  }

  if(can_perform_tasks_thread.joinable()) {
    can_perform_tasks_thread.join();
  }

  if(socket_fd >= 0) {
    close(socket_fd);
    socket_fd = -1;
  }

  is_open = false;
  return Status::OK();
}

Status CanDriver::send_to_queue(const CanFrame &frame) {
  std::lock_guard<std::mutex> lock(queue_mutex);
  if(!is_open) {
    return Status::Invalid("CAN socket is not open");
  }

  if(frame.size > CANFD_MAX_DLEN) {
    return Status::Invalid("CAN frame size exceeds maximum data size");
  }

  if(frame.id == 0) {
    return Status::Invalid("CAN frame ID is invalid");
  }

  send_queue.push(frame);

  RCLCPP_DEBUG(rclcpp::get_logger("CanDriver"), "Sending CAN frame with ID %u", frame.id);
  return Status::OK();
}

Status CanDriver::add_callback(uint32_t id, can_callback_type callback, void *args) {
  bool can_was_open = is_open;
  if(is_open) {
    internall_close_can();
    // return Status::Invalid("Cannot add callback after opening CAN socket");
  }

  callbacks_.find(id);
  if(callbacks_.find(id) != callbacks_.end()) {
    return Status::KeyError("Callback for ID already exists");
  }
  if(callback == nullptr) {
    return Status::Invalid("Callback function is null");
  }
  // Add the callback for the specified ID
  callbacks_[id] = std::make_pair(callback, args);

  if(can_was_open) {
    return open_can();
  }
  return Status::OK();
}

Status CanDriver::remove_callback(uint32_t id) {
  bool can_was_open = is_open;
  if(is_open) {
    internall_close_can();
    return Status::Invalid("Cannot remove callback after opening CAN socket");
  }

  auto it = callbacks_.find(id);
  if(it == callbacks_.end())
    return Status::KeyError("Callback for ID not found");
  callbacks_.erase(it);

  if(can_was_open) {
    return open_can();
  }
  return Status::OK();
}

Status CanDriver::write_can_frame(const CanFrame &canframe) {
  if(!is_open) {
    return Status::Invalid("CAN socket is not open");
  }
  auto frame = canframe.to_can_frame<can_frame>();

  // std::lock_guard<std::mutex> lock(can_mutex);
  ssize_t bytes_written = write(socket_fd, &frame, sizeof(frame));
  if(bytes_written < 0) {
    return Status::IOError("CanDriver: Failed to write CAN frame");
  }
  return Status::OK();
}

Result<CanFrame> CanDriver::read_can_frame() {
  if(!is_open) {
    return Status::Invalid("CAN socket is not open");
  }
  // std::lock_guard<std::mutex> lock(can_mutex);
  can_frame frame;
  ssize_t bytes_read = read(socket_fd, &frame, sizeof(frame));
  if(bytes_read < 0) {
    return Status::IOError("CanDriver: Failed to read CAN frame");
  }
  return Result<CanFrame>::OK(CanFrame::from_can_frame<decltype(frame)>(frame));
}

Result<CanFrame> CanDriver::read_and_handle_frame() {
  // std::lock_guard<std::mutex> lock(queue_mutex);
  NOMAD_ASSING_OR_RETURN(frame, read_can_frame());
  uint32_t id = frame.id | (frame.is_remote_request ? CAN_RTR_FLAG : 0);
  auto it     = callbacks_.find(id);
  if(it != callbacks_.end()) {
    auto callback = it->second.first;
    auto args     = it->second.second;
    callback(*this, frame, args);
  }
  return Result<CanFrame>::OK(frame);
}

Status CanDriver::send(const CanFrame &frame) {
  if(!is_open) {
    return Status::Invalid("CAN socket is not open");
  }

  if(frame.size > CANFD_MAX_DLEN) {
    return Status::Invalid("CAN frame size exceeds maximum data size");
  }

  if(frame.id == 0) {
    return Status::Invalid("CAN frame ID is invalid");
  }

  if(threaded) {
    return send_to_queue(frame);
  } else {
    // If not threaded, send the frame directly
    return write_can_frame(frame);
  }
}

Result<CanFrame> CanDriver::receive() {
  if(!is_open) {
    return Status::Invalid("CAN socket is not open");
  }

  if(threaded) {
    return Status::Invalid("Cannot receive in threaded mode");
  }
  return read_and_handle_frame();
}

void CanDriver::handle_can_transmission() {
  while(run_thread) {

    // Check if there are any frames to send
    queue_mutex.lock();
    auto queue_empty = send_queue.empty();
    queue_mutex.unlock();

    if(queue_empty) {
      std::unique_lock<std::mutex> lock(queue_mutex);
      queue_condition.wait_for(lock, std::chrono::microseconds(100), [this]() { return !send_queue.empty(); });
      continue;
    }
    auto frame = send_queue.front();
    send_queue.pop();
    write_can_frame(frame);
  }
}

void CanDriver::handle_can_tasks() {
  while(run_thread) {
    (void)read_and_handle_frame();
  }
}