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

#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <fcntl.h>

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "shared_types_nomad/status.hpp"
#include <unordered_map>
#include <functional>
#include <queue>
#include <mutex>


using namespace nomad_core;

namespace nomad_hardware {

template <typename T>
concept CanFrameType = std::same_as<T, can_frame> || std::same_as<T, canfd_frame>;


/// @brief CAN data frame.
struct CanFrame {

  /// @brief CAN ID of the frame. Either standard (11 bits) or extended (29 bits).
  uint32_t id;

  /// @brief Length of the data in the frame. Maximum length is 64 bytes.
  uint8_t size;

  /// @brief Data of the frame. Maximum length is 64 bytes.
  uint8_t data[CANFD_MAX_DLEN];

  /// @brief Flag to indicate if the frame is a remote request.
  bool is_remote_request;

  /// @brief Flag to indicate if the frame is an extended frame.
  bool is_extended;


  /// @brief Convert the CanFrame to a SocketCAN can_frame structure.
  template <CanFrameType T> T to_can_frame() const {
    T frame;
    frame.can_id = id | (is_extended ? CAN_EFF_FLAG : 0) | (is_remote_request ? CAN_RTR_FLAG : 0);
    frame.len    = size;
    memcpy(frame.data, data, size);
    return frame;
  }

  /// @brief Convert a SocketCAN can_frame structure to a CanFrame.
  template <CanFrameType T> static CanFrame from_can_frame(const T &frame) {
    CanFrame cf;
    cf.is_extended       = (frame.can_id & CAN_EFF_FLAG) != 0;
    cf.is_remote_request = (frame.can_id & CAN_RTR_FLAG) != 0;
    cf.size              = frame.len;
    cf.id                = frame.can_id & CAN_ERR_MASK;
    memcpy(cf.data, frame.data, frame.len);
    return cf;
  }
};

class CanDriver {
public:
  using can_callback_type = std::function<void(CanDriver &, const CanFrame &, void *)>;
  ~CanDriver();

  /// @brief Create a new CanDriver instance.
  /// @param can_interface Name of the CAN interface to use (e.g. "can0").
  /// @param threaded If true, the driver creates two thread to run callback and handel can communication.
  /// @param timeout_us Timeout for the CAN socket in microseconds should be big if thread is used.
  /// @param queue_size Size of the queue for sending CAN frames.
  /// @return A shared pointer to the CanDriver instance.
  /// @note The queue size should be big enough to handle the number of frames that will be sent.
  static Result<std::shared_ptr<CanDriver>> Make(const std::string &can_interface,
                                                 bool threaded       = true,
                                                 uint32_t timeout_us = 100000,
                                                 size_t queue_size   = 32,
                                                 size_t buffer_size  = 512);

  /// @brief Send a CAN frame to the CAN bus.
  /// @param frame The CAN frame to send.
  /// @note The frame will be sent to the CAN bus immediately if the driver is not threaded.
  /// @return Status of the operation.
  Status send(const CanFrame &frame);

  /// @brief Receive a CAN frame from the CAN bus.
  /// @return The received CAN frame.
  /// @note The frame will be received from the CAN bus immediately if the driver is not threaded.
  /// @note If callbacks are registered, the frame will be passed to the callback function and handled there.
  /// @note If callbacks for the received frame are not registered, the frame will be simply returned.
  /// @note If the driver is threaded, this function will always return an error and not do anything.
  Result<CanFrame> receive();

  /// @brief Add a callback for a specific CAN ID.
  /// @param id The CAN ID to listen for, if you want to receive callback for a remote request you have to set "id = SOME_CAN_ID | CAN_RTR_FLAG".
  /// @param callback The callback function to call when a frame with the specified ID is received.
  /// @param args Optional arguments to pass to the callback function.
  /// @return Status of the operation.
  /// @note Callback can only be added before opening the CAN socket
  /// @note When no callback are registered, and the thread is used, the driver will not receive any frames.
  /// @note When normal mode is used (instead of threaded), the driver don't have to have any callback registered.
  /// but then handling incoming data will have to be implemented by the user after receiving the frame.
  Status add_callback(uint32_t id, can_callback_type callback, void *args = nullptr);

  /// @brief Remove a callback for a specific CAN ID.
  /// @param id The CAN ID to remove the callback for.
  /// @return Status of the operation.
  /// @note Callback can only be removed before opening the CAN socket
  Status remove_callback(uint32_t id);

  /// @brief Open the CAN socket.
  /// @note This function will create a new thread to handle the CAN socket.
  /// @return Status of the operation.
  Status open_can();

  /// @brief Close the CAN socket if is the last one.
  /// @param can_driver The CanDriver instance to close.
  /// @note This function will reset the pointer to the CanDriver instance, when its the last one then CAN will be closed.
  /// @note This function will not close the CAN socket if there are other instances using it.
  /// @return Status of the operation.
  static Status close_can(std::shared_ptr<CanDriver> &can_driver);


private:
  CanDriver(const std::string &can_interface, bool threaded, uint32_t timeout_us, size_t queue_size = 32, size_t buffer_size = 512);

  /// @brief Handle CAN transmission in a separate thread.
  void handle_can_transmission();

  /// @brief Handle CAN tasks in a separate thread.
  void handle_can_tasks();

  /// @brief Send a CAN frame to the queue. that will be sent to the CAN bus.
  Status send_to_queue(const CanFrame &frame);

  /// @brief Write a CAN frame to the socket.
  Status write_can_frame(const CanFrame &frame);

  /// @brief Read a CAN frame from the socket.
  Result<CanFrame> read_can_frame();

  Result<CanFrame> read_and_handle_frame();

  Status internall_close_can();

  std::queue<CanFrame> send_queue;
  std::queue<CanFrame> receive_queue;
  std::condition_variable queue_condition;
  std::mutex queue_mutex;
  std::mutex can_mutex;
  std::unordered_map<uint32_t, std::pair<can_callback_type, void *>> callbacks_;

  std::thread can_read_thread;
  std::thread can_perform_tasks_thread;
  std::atomic<bool> run_thread{ false };


  std::string can_interface_name;
  bool threaded;
  size_t queue_size;
  int socket_fd;
  size_t buffer_size;
  bool is_open = false;

  const uint32_t timeout_us;
  sockaddr_can addr;
  ifreq ifr;

  int close_calls = 0;

  static std::vector<std::shared_ptr<CanDriver>> global_can_driver;
};


} // namespace nomad_hardware
