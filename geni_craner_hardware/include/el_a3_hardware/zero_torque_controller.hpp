/**
 * @file zero_torque_controller.hpp
 * @brief Zero-torque (gravity compensated) controller for EL-A3
 *
 * Claims effort command interfaces. Computes gravity compensation via
 * Pinocchio RNEA and writes torques to the hardware interface, which
 * in turn sends Kp=0, Kd=damping, torque=gravity to the motors.
 */

#ifndef EL_A3_HARDWARE__ZERO_TORQUE_CONTROLLER_HPP_
#define EL_A3_HARDWARE__ZERO_TORQUE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace el_a3_hardware
{

class ZeroTorqueController : public controller_interface::ControllerInterface
{
public:
  ZeroTorqueController() = default;
  ~ZeroTorqueController() override = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<std::string> joint_names_;
  double kd_{1.0};

  // Pinocchio
  pinocchio::Model model_;
  pinocchio::Data data_;
  bool pinocchio_ok_{false};

  // Calibrated inertia
  std::string inertia_config_path_;
  bool loadAndApplyCalibratedInertia(const std::string & path);

  // Debug publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gravity_pub_;
};

}  // namespace el_a3_hardware

#endif  // EL_A3_HARDWARE__ZERO_TORQUE_CONTROLLER_HPP_
