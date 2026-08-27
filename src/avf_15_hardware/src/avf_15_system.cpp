#include "avf_15_hardware/avf_15_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"


namespace
{

std::string get_string_parameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  const std::string & default_value)
{
  const auto iterator = info.hardware_parameters.find(name);

  if (iterator == info.hardware_parameters.end())
  {
    return default_value;
  }

  return iterator->second;
}


double get_double_parameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  double default_value,
  const rclcpp::Logger & logger)
{
  const auto iterator = info.hardware_parameters.find(name);

  if (iterator == info.hardware_parameters.end())
  {
    return default_value;
  }

  try
  {
    return std::stod(iterator->second);
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(
      logger,
      "Invalid double parameter '%s': '%s', error: %s",
      name.c_str(),
      iterator->second.c_str(),
      exception.what());

    return default_value;
  }
}


int get_int_parameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  int default_value,
  const rclcpp::Logger & logger)
{
  const auto iterator = info.hardware_parameters.find(name);

  if (iterator == info.hardware_parameters.end())
  {
    return default_value;
  }

  try
  {
    return std::stoi(iterator->second);
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(
      logger,
      "Invalid integer parameter '%s': '%s', error: %s",
      name.c_str(),
      iterator->second.c_str(),
      exception.what());

    return default_value;
  }
}

}  // namespace


namespace avf_15_hardware
{

hardware_interface::CallbackReturn
AVF15SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  const auto parent_result =
    hardware_interface::SystemInterface::on_init(info);

  if (parent_result != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != JOINT_COUNT)
  {
    RCLCPP_ERROR(
      logger_,
      "Expected %zu joints, but received %zu",
      static_cast<std::size_t>(JOINT_COUNT),
      info_.joints.size());

    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_[MAIN_WHEEL] = "J_main_wheel";
  joint_names_[STEERING] = "J_main_wheel_rotate";
  joint_names_[MAIN_FORK] = "J_main_villas";
  joint_names_[SECOND_FORK] = "J_second_villas";

  /*
   * Проверка наличия всех необходимых joint.
   * Сделано без lambda-выражений, чтобы исключить ошибки синтаксиса.
   */
  for (const auto & expected_name : joint_names_)
  {
    bool joint_found = false;

    for (const auto & joint : info_.joints)
    {
      if (joint.name == expected_name)
      {
        joint_found = true;
        break;
      }
    }

    if (!joint_found)
    {
      RCLCPP_ERROR(
        logger_,
        "Required joint '%s' is missing",
        expected_name.c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  transport_type_ = get_string_parameter(
    info_,
    "transport_type",
    "mock");

  device_ = get_string_parameter(
    info_,
    "device",
    "can0");

  command_timeout_ms_ = get_int_parameter(
    info_,
    "command_timeout_ms",
    200,
    logger_);

  main_sensor_raw_min_ = get_double_parameter(
    info_,
    "main_sensor_raw_min",
    0.0,
    logger_);

  main_sensor_raw_max_ = get_double_parameter(
    info_,
    "main_sensor_raw_max",
    4095.0,
    logger_);

  second_sensor_raw_min_ = get_double_parameter(
    info_,
    "second_sensor_raw_min",
    0.0,
    logger_);

  second_sensor_raw_max_ = get_double_parameter(
    info_,
    "second_sensor_raw_max",
    4095.0,
    logger_);

  main_fork_position_min_ = get_double_parameter(
    info_,
    "main_fork_position_min",
    -0.10,
    logger_);

  main_fork_position_max_ = get_double_parameter(
    info_,
    "main_fork_position_max",
    1.00,
    logger_);

  second_fork_position_min_ = get_double_parameter(
    info_,
    "second_fork_position_min",
    -0.10,
    logger_);

  second_fork_position_max_ = get_double_parameter(
    info_,
    "second_fork_position_max",
    1.00,
    logger_);

  wheel_velocity_limit_ = get_double_parameter(
    info_,
    "wheel_velocity_limit",
    10.0,
    logger_);

  steering_position_limit_ = get_double_parameter(
    info_,
    "steering_position_limit",
    0.70,
    logger_);

  if (command_timeout_ms_ <= 0)
  {
    RCLCPP_ERROR(
      logger_,
      "command_timeout_ms must be greater than zero");

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (main_sensor_raw_max_ <= main_sensor_raw_min_)
  {
    RCLCPP_ERROR(
      logger_,
      "Invalid main sensor calibration: min=%f, max=%f",
      main_sensor_raw_min_,
      main_sensor_raw_max_);

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (second_sensor_raw_max_ <= second_sensor_raw_min_)
  {
    RCLCPP_ERROR(
      logger_,
      "Invalid second sensor calibration: min=%f, max=%f",
      second_sensor_raw_min_,
      second_sensor_raw_max_);

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (main_fork_position_max_ <= main_fork_position_min_)
  {
    RCLCPP_ERROR(
      logger_,
      "Invalid main fork limits: min=%f, max=%f",
      main_fork_position_min_,
      main_fork_position_max_);

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (second_fork_position_max_ <= second_fork_position_min_)
  {
    RCLCPP_ERROR(
      logger_,
      "Invalid second fork limits: min=%f, max=%f",
      second_fork_position_min_,
      second_fork_position_max_);

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (wheel_velocity_limit_ <= 0.0)
  {
    RCLCPP_ERROR(
      logger_,
      "wheel_velocity_limit must be greater than zero");

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (steering_position_limit_ <= 0.0)
  {
    RCLCPP_ERROR(
      logger_,
      "steering_position_limit must be greater than zero");

    return hardware_interface::CallbackReturn::ERROR;
  }

  mock_mode_ = transport_type_ == "mock";

  command_position_.fill(0.0);
  command_velocity_.fill(0.0);

  state_position_.fill(0.0);
  state_velocity_.fill(0.0);
  previous_position_.fill(0.0);

  main_sensor_raw_ = main_sensor_raw_min_;
  second_sensor_raw_ = second_sensor_raw_min_;

  configured_ = false;
  active_ = false;
  communication_ok_ = false;

  RCLCPP_INFO(
    logger_,
    "Initialized AVF-15 hardware: transport=%s, device=%s",
    transport_type_.c_str(),
    device_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
AVF15SystemHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!open_transport())
  {
    RCLCPP_ERROR(
      logger_,
      "Could not open hardware transport '%s' on '%s'",
      transport_type_.c_str(),
      device_.c_str());

    return hardware_interface::CallbackReturn::ERROR;
  }

  configured_ = true;
  communication_ok_ = true;

  RCLCPP_INFO(logger_, "AVF-15 hardware configured");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
AVF15SystemHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (active_)
  {
    stop_all_actuators();
  }

  close_transport();

  configured_ = false;
  active_ = false;
  communication_ok_ = false;

  RCLCPP_INFO(logger_, "AVF-15 hardware cleaned up");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
AVF15SystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!configured_ || !communication_ok_)
  {
    RCLCPP_ERROR(
      logger_,
      "Cannot activate: hardware is not configured");

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!read_hardware())
  {
    RCLCPP_ERROR(
      logger_,
      "Cannot read initial hardware state");

    return hardware_interface::CallbackReturn::ERROR;
  }

  previous_position_ = state_position_;

  /*
   * При активации position commands устанавливаются
   * равными текущим состояниям, чтобы не было рывка.
   */
  command_position_[STEERING] =
    state_position_[STEERING];

  command_position_[MAIN_FORK] =
    state_position_[MAIN_FORK];

  command_position_[SECOND_FORK] =
    state_position_[SECOND_FORK];

  command_velocity_[MAIN_WHEEL] = 0.0;

  active_ = true;

  RCLCPP_INFO(logger_, "AVF-15 hardware activated");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
AVF15SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  /*
   * Сначала отправляем команду остановки, затем снимаем active_.
   */
  const bool stopped = stop_all_actuators();

  active_ = false;

  if (!stopped)
  {
    RCLCPP_ERROR(
      logger_,
      "Failed to stop one or more actuators");

    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "AVF-15 hardware deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface>
AVF15SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(8);

  interfaces.emplace_back(
    joint_names_[MAIN_WHEEL],
    hardware_interface::HW_IF_POSITION,
    &state_position_[MAIN_WHEEL]);

  interfaces.emplace_back(
    joint_names_[MAIN_WHEEL],
    hardware_interface::HW_IF_VELOCITY,
    &state_velocity_[MAIN_WHEEL]);

  interfaces.emplace_back(
    joint_names_[STEERING],
    hardware_interface::HW_IF_POSITION,
    &state_position_[STEERING]);

  interfaces.emplace_back(
    joint_names_[STEERING],
    hardware_interface::HW_IF_VELOCITY,
    &state_velocity_[STEERING]);

  interfaces.emplace_back(
    joint_names_[MAIN_FORK],
    hardware_interface::HW_IF_POSITION,
    &state_position_[MAIN_FORK]);

  interfaces.emplace_back(
    joint_names_[MAIN_FORK],
    hardware_interface::HW_IF_VELOCITY,
    &state_velocity_[MAIN_FORK]);

  interfaces.emplace_back(
    joint_names_[SECOND_FORK],
    hardware_interface::HW_IF_POSITION,
    &state_position_[SECOND_FORK]);

  interfaces.emplace_back(
    joint_names_[SECOND_FORK],
    hardware_interface::HW_IF_VELOCITY,
    &state_velocity_[SECOND_FORK]);

  return interfaces;
}


std::vector<hardware_interface::CommandInterface>
AVF15SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(4);

  /*
   * Ведущее колесо получает команду угловой скорости.
   */
  interfaces.emplace_back(
    joint_names_[MAIN_WHEEL],
    hardware_interface::HW_IF_VELOCITY,
    &command_velocity_[MAIN_WHEEL]);

  /*
   * Рулевой механизм получает целевой угол.
   */
  interfaces.emplace_back(
    joint_names_[STEERING],
    hardware_interface::HW_IF_POSITION,
    &command_position_[STEERING]);

  /*
   * Секции вил получают целевые позиции.
   */
  interfaces.emplace_back(
    joint_names_[MAIN_FORK],
    hardware_interface::HW_IF_POSITION,
    &command_position_[MAIN_FORK]);

  interfaces.emplace_back(
    joint_names_[SECOND_FORK],
    hardware_interface::HW_IF_POSITION,
    &command_position_[SECOND_FORK]);

  return interfaces;
}


hardware_interface::return_type
AVF15SystemHardware::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  if (!active_)
  {
    return hardware_interface::return_type::OK;
  }

  if (!read_hardware())
  {
    communication_ok_ = false;
    stop_all_actuators();

    RCLCPP_ERROR(
      logger_,
      "Hardware read failed. Actuators were stopped.");

    return hardware_interface::return_type::ERROR;
  }

  communication_ok_ = true;

  const double dt = period.seconds();

  if (dt > 0.0)
  {
    /*
     * В mock-режиме интегрируем положение ведущего колеса:
     *
     * position_new = position_old + velocity * dt
     */
    if (mock_mode_)
    {
      state_position_[MAIN_WHEEL] +=
        state_velocity_[MAIN_WHEEL] * dt;
    }

    state_velocity_[STEERING] =
      (state_position_[STEERING] -
      previous_position_[STEERING]) / dt;

    state_velocity_[MAIN_FORK] =
      (state_position_[MAIN_FORK] -
      previous_position_[MAIN_FORK]) / dt;

    state_velocity_[SECOND_FORK] =
      (state_position_[SECOND_FORK] -
      previous_position_[SECOND_FORK]) / dt;
  }
  else
  {
    state_velocity_[STEERING] = 0.0;
    state_velocity_[MAIN_FORK] = 0.0;
    state_velocity_[SECOND_FORK] = 0.0;
  }

  previous_position_ = state_position_;

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type
AVF15SystemHardware::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!active_ || !communication_ok_)
  {
    stop_all_actuators();
    return hardware_interface::return_type::ERROR;
  }

  if (!std::isfinite(command_velocity_[MAIN_WHEEL]) ||
      !std::isfinite(command_position_[STEERING]) ||
      !std::isfinite(command_position_[MAIN_FORK]) ||
      !std::isfinite(command_position_[SECOND_FORK]))
  {
    RCLCPP_ERROR(
      logger_,
      "Received non-finite command. Actuators were stopped.");

    stop_all_actuators();

    return hardware_interface::return_type::ERROR;
  }

  const double wheel_velocity = clamp_value(
    command_velocity_[MAIN_WHEEL],
    -wheel_velocity_limit_,
    wheel_velocity_limit_);

  const double steering_position = clamp_value(
    command_position_[STEERING],
    -steering_position_limit_,
    steering_position_limit_);

  const double main_fork_position = clamp_value(
    command_position_[MAIN_FORK],
    main_fork_position_min_,
    main_fork_position_max_);

  const double second_fork_position = clamp_value(
    command_position_[SECOND_FORK],
    second_fork_position_min_,
    second_fork_position_max_);

  bool success = true;

  success =
    send_wheel_velocity(wheel_velocity) &&
    success;

  success =
    send_steering_position(steering_position) &&
    success;

  success =
    send_main_fork_position(main_fork_position) &&
    success;

  success =
    send_second_fork_position(second_fork_position) &&
    success;

  if (!success)
  {
    communication_ok_ = false;
    stop_all_actuators();

    RCLCPP_ERROR(
      logger_,
      "Hardware write failed. Actuators were stopped.");

    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}


bool AVF15SystemHardware::open_transport()
{
  if (mock_mode_)
  {
    RCLCPP_WARN(
      logger_,
      "Running in mock transport mode. "
      "No physical hardware will be controlled.");

    return true;
  }

  /*
   * TODO: открыть интерфейс реального оборудования.
   *
   * Примеры:
   *   CAN / SocketCAN
   *   CANopen
   *   Modbus RTU
   *   Modbus TCP
   *   Serial
   *   EtherCAT
   *
   * После открытия необходимо:
   *   1. Проверить доступность каждого привода.
   *   2. Проверить аварийные входы.
   *   3. Проверить концевики.
   *   4. Перевести приводы в безопасное состояние.
   *   5. Только затем вернуть true.
   */

  RCLCPP_ERROR(
    logger_,
    "Transport '%s' is not implemented",
    transport_type_.c_str());

  return false;
}


void AVF15SystemHardware::close_transport()
{
  if (mock_mode_)
  {
    return;
  }

  /*
   * TODO:
   *   1. Остановить двигатели.
   *   2. Выключить разрешение приводов.
   *   3. Закрыть CAN/Serial/Modbus соединение.
   */
}


bool AVF15SystemHardware::read_hardware()
{
  if (mock_mode_)
  {
    /*
     * Ведущее колесо:
     * команда скорости имитирует измеренную скорость.
     */
    state_velocity_[MAIN_WHEEL] =
      clamp_value(
        command_velocity_[MAIN_WHEEL],
        -wheel_velocity_limit_,
        wheel_velocity_limit_);

    /*
     * Рулевой механизм и вилы:
     * измеренная позиция сразу следует за командой.
     */
    state_position_[STEERING] =
      clamp_value(
        command_position_[STEERING],
        -steering_position_limit_,
        steering_position_limit_);

    state_position_[MAIN_FORK] =
      clamp_value(
        command_position_[MAIN_FORK],
        main_fork_position_min_,
        main_fork_position_max_);

    state_position_[SECOND_FORK] =
      clamp_value(
        command_position_[SECOND_FORK],
        second_fork_position_min_,
        second_fork_position_max_);

    return true;
  }

  /*
   * TODO: читать реальное оборудование.
   *
   * Примерная структура:
   *
   * const auto wheel_position_raw =
   *   drive.read_encoder_position();
   *
   * const auto wheel_velocity_raw =
   *   drive.read_encoder_velocity();
   *
   * const auto steering_raw =
   *   steering.read_position();
   *
   * main_sensor_raw_ =
   *   fork_sensor.read_main_position();
   *
   * second_sensor_raw_ =
   *   fork_sensor.read_second_position();
   *
   * state_position_[MAIN_WHEEL] =
   *   convert_encoder_to_radians(wheel_position_raw);
   *
   * state_velocity_[MAIN_WHEEL] =
   *   convert_encoder_speed_to_rad_s(wheel_velocity_raw);
   *
   * state_position_[STEERING] =
   *   convert_steering_position_to_radians(steering_raw);
   *
   * state_position_[MAIN_FORK] =
   *   convert_main_sensor_to_meters(main_sensor_raw_);
   *
   * state_position_[SECOND_FORK] =
   *   convert_second_sensor_to_meters(second_sensor_raw_);
   */

  return false;
}


bool AVF15SystemHardware::send_wheel_velocity(
  double velocity_rad_s)
{
  if (mock_mode_)
  {
    command_velocity_[MAIN_WHEEL] =
      clamp_value(
        velocity_rad_s,
        -wheel_velocity_limit_,
        wheel_velocity_limit_);

    return true;
  }

  /*
   * TODO: отправить команду ведущему приводу.
   *
   * Если привод принимает RPM:
   *
   * constexpr double two_pi =
   *   6.28318530717958647692;
   *
   * const double rpm =
   *   velocity_rad_s * 60.0 / two_pi;
   *
   * motor_driver.send_velocity_rpm(rpm);
   */

  return false;
}


bool AVF15SystemHardware::send_steering_position(
  double position_rad)
{
  if (mock_mode_)
  {
    command_position_[STEERING] =
      clamp_value(
        position_rad,
        -steering_position_limit_,
        steering_position_limit_);

    return true;
  }

  /*
   * TODO:
   *   1. Преобразовать радианы в единицы рулевого привода.
   *   2. Ограничить физический угол.
   *   3. Проверить состояние концевиков.
   *   4. Отправить команду.
   */

  return false;
}


bool AVF15SystemHardware::send_main_fork_position(
  double position_m)
{
  if (mock_mode_)
  {
    command_position_[MAIN_FORK] =
      clamp_value(
        position_m,
        main_fork_position_min_,
        main_fork_position_max_);

    return true;
  }

  /*
   * TODO:
   * Если привод принимает целевое положение:
   *   преобразовать метры в единицы привода;
   *   проверить концевики;
   *   отправить target position.
   *
   * Если привод принимает скорость или PWM:
   *   position command interface использовать нельзя напрямую.
   *   Потребуется velocity/effort interface и регулятор положения.
   */

  return false;
}


bool AVF15SystemHardware::send_second_fork_position(
  double position_m)
{
  if (mock_mode_)
  {
    command_position_[SECOND_FORK] =
      clamp_value(
        position_m,
        second_fork_position_min_,
        second_fork_position_max_);

    return true;
  }

  /*
   * TODO:
   * Отправить команду второй секции вил.
   */

  return false;
}


bool AVF15SystemHardware::stop_all_actuators()
{
  /*
   * Всегда очищаем команду движения ведущего колеса.
   */
  command_velocity_[MAIN_WHEEL] = 0.0;

  if (mock_mode_)
  {
    state_velocity_[MAIN_WHEEL] = 0.0;

    return true;
  }

  /*
   * TODO:
   *   1. Передать ведущему двигателю скорость 0.
   *   2. Остановить двигатель вил.
   *   3. Перевести рулевой привод в безопасное удержание.
   *   4. При необходимости отключить enable приводов.
   *
   * Аппаратный E-stop должен работать независимо от ROS.
   */

  return false;
}


double AVF15SystemHardware::convert_main_sensor_to_meters(
  double raw) const
{
  const double denominator =
    main_sensor_raw_max_ - main_sensor_raw_min_;

  if (std::abs(denominator) <
    std::numeric_limits<double>::epsilon())
  {
    return main_fork_position_min_;
  }

  const double normalized =
    (raw - main_sensor_raw_min_) / denominator;

  const double position =
    main_fork_position_min_ +
    normalized *
    (main_fork_position_max_ -
    main_fork_position_min_);

  return clamp_value(
    position,
    main_fork_position_min_,
    main_fork_position_max_);
}


double AVF15SystemHardware::convert_second_sensor_to_meters(
  double raw) const
{
  const double denominator =
    second_sensor_raw_max_ -
    second_sensor_raw_min_;

  if (std::abs(denominator) <
    std::numeric_limits<double>::epsilon())
  {
    return second_fork_position_min_;
  }

  const double normalized =
    (raw - second_sensor_raw_min_) /
    denominator;

  const double position =
    second_fork_position_min_ +
    normalized *
    (second_fork_position_max_ -
    second_fork_position_min_);

  return clamp_value(
    position,
    second_fork_position_min_,
    second_fork_position_max_);
}


double AVF15SystemHardware::clamp_value(
  double value,
  double minimum,
  double maximum) const
{
  return std::clamp(
    value,
    minimum,
    maximum);
}

}  // namespace avf_15_hardware


PLUGINLIB_EXPORT_CLASS(
  avf_15_hardware::AVF15SystemHardware,
  hardware_interface::SystemInterface)