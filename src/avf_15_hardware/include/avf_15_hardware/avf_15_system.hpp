#ifndef AVF_15_HARDWARE__AVF_15_SYSTEM_HPP_
#define AVF_15_HARDWARE__AVF_15_SYSTEM_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace avf_15_hardware
{

class AVF15SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AVF15SystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  enum JointIndex : std::size_t
  {
    MAIN_WHEEL = 0,
    STEERING = 1,
    MAIN_FORK = 2,
    SECOND_FORK = 3,
    JOINT_COUNT = 4
  };

  bool open_transport();
  void close_transport();

  bool read_hardware();

  bool send_wheel_velocity(double velocity_rad_s);
  bool send_steering_position(double position_rad);
  bool send_main_fork_position(double position_m);
  bool send_second_fork_position(double position_m);

  bool stop_all_actuators();

  double convert_main_sensor_to_meters(double raw) const;
  double convert_second_sensor_to_meters(double raw) const;

  double clamp_value(
    double value,
    double minimum,
    double maximum) const;

  rclcpp::Logger logger_{
    rclcpp::get_logger("AVF15SystemHardware")
  };

  // Порядок соответствует JointIndex.
  std::array<std::string, JOINT_COUNT> joint_names_{};

  // Команды от ros2_controllers.
  std::array<double, JOINT_COUNT> command_position_{};
  std::array<double, JOINT_COUNT> command_velocity_{};

  // Фактические состояния оборудования.
  std::array<double, JOINT_COUNT> state_position_{};
  std::array<double, JOINT_COUNT> state_velocity_{};

  std::array<double, JOINT_COUNT> previous_position_{};

  // Настройки транспорта.
  std::string transport_type_{"mock"};
  std::string device_{"can0"};

  int command_timeout_ms_{200};

  // Калибровка магнитных датчиков.
  double main_sensor_raw_min_{0.0};
  double main_sensor_raw_max_{4095.0};

  double second_sensor_raw_min_{0.0};
  double second_sensor_raw_max_{4095.0};

  double main_fork_position_min_{-0.10};
  double main_fork_position_max_{1.00};

  double second_fork_position_min_{-0.10};
  double second_fork_position_max_{1.00};

  // Ограничения исполнительных механизмов.
  double wheel_velocity_limit_{10.0};
  double steering_position_limit_{0.70};

  // Сырые значения датчиков.
  double main_sensor_raw_{0.0};
  double second_sensor_raw_{0.0};

  bool configured_{false};
  bool active_{false};
  bool communication_ok_{false};

  // Временный тестовый режим без реального оборудования.
  bool mock_mode_{true};
};

}  // namespace avf_15_hardware

#endif  // AVF_15_HARDWARE__AVF_15_SYSTEM_HPP_