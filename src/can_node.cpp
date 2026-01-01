#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "can_device/can_device.hpp"
#include "can_messages.h"



/**
 * test node for the CAN driver.
 * test speed benchmark and data transfer.
 *
 * Results 5.05.2025
 * Request: 200Hz + 65bit answer  Error:0%
 */


class CanNode : public rclcpp::Node {
public:
  CanNode() : Node("can_node") {
    RCLCPP_INFO(this->get_logger(), "CAN Node has been started.");
    can_driver = CanDriver::Make("can0", true, 10000).valueOrDie();
    can_driver->add_callback(CAN_BAROMETER_STATUS_FRAME_ID,
                             std::bind(&CanNode::can_callback_status, this, std::placeholders::_1,
                                       std::placeholders::_2, std::placeholders::_3),
                             this);
    can_driver->add_callback(CAN_BAROMETER_DATA_FRAME_ID,
                             std::bind(&CanNode::can_callback_data, this, std::placeholders::_1,
                                       std::placeholders::_2, std::placeholders::_3),
                             this);
    can_driver->open_can();

    start_time = std::chrono::steady_clock::now();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&CanNode::timer_callback, this));
  }

  ~CanNode() {
    // can_driver->close_can();
    CanDriver::close_can(can_driver);
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();

    float err  = (float)(sent - recived) * 100.0f / sent;
    float freq = (float)recived * 1000.0f / elapsed_time;
    RCLCPP_INFO(this->get_logger(), "CAN node stoped Rec: %d Sent: %d  Error: %d ", recived, sent, error);
    RCLCPP_INFO(this->get_logger(), "CAN node stoped Prec: %.6f%%    Freq: %.3f%%", err, freq);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::_V2::steady_clock::time_point start_time;

  void timer_callback() {
    CanFrame frame_stat          = {};
    frame_stat.id                = CAN_BAROMETER_STATUS_FRAME_ID;
    frame_stat.is_remote_request = true;
    frame_stat.is_extended       = CAN_BAROMETER_STATUS_IS_EXTENDED;
    can_driver->send(frame_stat);
    sent++;

    CanFrame frame_data          = {};
    frame_data.id                = CAN_BAROMETER_DATA_FRAME_ID;
    frame_data.is_remote_request = true;
    frame_data.is_extended       = CAN_BAROMETER_DATA_IS_EXTENDED;
    can_driver->send(frame_data);
    // RCLCPP_INFO(this->get_logger(), "Barometer status: %d", barometer_status);
    // RCLCPP_INFO(this->get_logger(), "Barometer temperature: %f", barometer_temperature);
    sent++;
  }

  size_t recived = 0;
  size_t sent    = 0;
  size_t error   = 0;

  std::shared_ptr<CanDriver> can_driver;


  float barometer_pressure;
  float barometer_temperature;
  int barometer_status;

  void can_callback_status(CanDriver &can, const CanFrame &frame, void *args) {
    (void)can;
    (void)args;

    can_barometer_status_t status;
    if(can_barometer_status_unpack(&status, frame.data, frame.size)) {
      RCLCPP_ERROR(rclcpp::get_logger("CanNode"), "Failed to unpack barometer status");
      error++;
      return;
    }

    barometer_status = status.status;
    RCLCPP_INFO(rclcpp::get_logger("CanNode"), "Barometer status: %d  temp: %f", barometer_status, barometer_temperature);
    recived++;
  }

  void can_callback_data(CanDriver &can, const CanFrame &frame, void *args) {
    (void)can;
    (void)args;

    can_barometer_data_t data;
    if(can_barometer_data_unpack(&data, frame.data, frame.size)) {
      RCLCPP_ERROR(rclcpp::get_logger("CanNode"), "Failed to unpack barometer data");
      error++;
      return;
    }
    barometer_temperature = data.temperature;
    barometer_pressure    = data.pressure;
    RCLCPP_INFO(rclcpp::get_logger("CanNode"), "Barometer status: %d  temp: %f  pres: %f", barometer_status,
                barometer_temperature, barometer_pressure);
    recived++;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanNode>());
  rclcpp::shutdown();
  return 0;
}