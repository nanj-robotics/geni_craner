/**
 * @file zero_torque_controller.cpp
 * @brief Zero-torque (gravity compensated) controller for EL-A3
 */

#include "el_a3_hardware/zero_torque_controller.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace el_a3_hardware
{

controller_interface::CallbackReturn ZeroTorqueController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
  auto_declare<double>("kd", 1.0);
  auto_declare<std::string>("urdf_path", "");
  auto_declare<std::string>("inertia_config_path", "");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ZeroTorqueController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints configured");
    return controller_interface::CallbackReturn::ERROR;
  }

  kd_ = get_node()->get_parameter("kd").as_double();
  kd_ = std::clamp(kd_, 0.0, 5.0);

  std::string urdf_path = get_node()->get_parameter("urdf_path").as_string();
  if (!urdf_path.empty()) {
    try {
      pinocchio::urdf::buildModel(urdf_path, model_);
      data_ = pinocchio::Data(model_);
      pinocchio_ok_ = true;
      RCLCPP_INFO(get_node()->get_logger(),
                  "Pinocchio model loaded: nq=%d, nv=%d", model_.nq, model_.nv);

      inertia_config_path_ = get_node()->get_parameter("inertia_config_path").as_string();
      if (!inertia_config_path_.empty() && pinocchio_ok_) {
        if (loadAndApplyCalibratedInertia(inertia_config_path_)) {
          RCLCPP_INFO(get_node()->get_logger(),
                      "Calibrated inertia applied from: %s", inertia_config_path_.c_str());
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Pinocchio init failed: %s (gravity comp disabled)", e.what());
      pinocchio_ok_ = false;
    }
  }

  gravity_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "~/gravity_torque", 10);

  RCLCPP_INFO(get_node()->get_logger(),
              "ZeroTorqueController configured: %zu joints, kd=%.2f, pinocchio=%s, calibrated=%s",
              joint_names_.size(), kd_, pinocchio_ok_ ? "yes" : "no",
              (!inertia_config_path_.empty()) ? "yes" : "no");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ZeroTorqueController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    conf.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return conf;
}

controller_interface::InterfaceConfiguration
ZeroTorqueController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    conf.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    conf.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return conf;
}

controller_interface::CallbackReturn ZeroTorqueController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_WARN(get_node()->get_logger(),
              "Zero-torque mode ACTIVATED (kd=%.2f, gravity=%s)",
              kd_, pinocchio_ok_ ? "on" : "off");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ZeroTorqueController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Zero-torque mode DEACTIVATED");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ZeroTorqueController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  size_t n = joint_names_.size();
  std::vector<double> gravity_torques(n, 0.0);

  if (pinocchio_ok_) {
    try {
      Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
      Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
      Eigen::VectorXd a = Eigen::VectorXd::Zero(model_.nv);

      size_t nq = std::min(n, static_cast<size_t>(model_.nq));
      for (size_t i = 0; i < nq; ++i) {
        q[i] = state_interfaces_[i * 2].get_value();  // position
      }

      Eigen::VectorXd tau = pinocchio::rnea(model_, data_, q, v, a);

      for (size_t i = 0; i < std::min(n, static_cast<size_t>(tau.size())); ++i) {
        gravity_torques[i] = tau[i];
      }
    } catch (const std::exception & e) {
      (void)e;
    }
  }

  for (size_t i = 0; i < n; ++i) {
    command_interfaces_[i].set_value(gravity_torques[i]);
  }

  // Publish debug (every ~10th cycle)
  static int pub_counter = 0;
  if (++pub_counter >= 10 && gravity_pub_) {
    pub_counter = 0;
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_node()->now();
    msg.name = joint_names_;
    msg.effort = gravity_torques;
    gravity_pub_->publish(msg);
  }

  return controller_interface::return_type::OK;
}

bool ZeroTorqueController::loadAndApplyCalibratedInertia(const std::string & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Cannot open inertia config: %s", path.c_str());
    return false;
  }

  struct InertiaEntry { double mass; double com_x, com_y, com_z; bool loaded; };
  // L1..L6 indexed 0..5
  std::vector<InertiaEntry> entries(6);
  for (auto & e : entries) { e.loaded = false; }

  // Pre-fill with current URDF values
  for (int i = 0; i < 6 && i + 1 < model_.nbodies; ++i) {
    entries[i].mass  = model_.inertias[i + 1].mass();
    entries[i].com_x = model_.inertias[i + 1].lever()[0];
    entries[i].com_y = model_.inertias[i + 1].lever()[1];
    entries[i].com_z = model_.inertias[i + 1].lever()[2];
  }

  std::string line, current_joint;
  bool in_inertia_params = false;
  bool use_calibrated = false;

  while (std::getline(file, line)) {
    size_t pos = line.find_first_not_of(" \t");
    if (pos == std::string::npos || line[pos] == '#') continue;

    if (line.find("use_calibrated_params:") != std::string::npos) {
      use_calibrated = (line.find("true") != std::string::npos);
      continue;
    }
    if (line.find("inertia_params:") != std::string::npos) {
      in_inertia_params = true;
      continue;
    }
    if (!in_inertia_params) continue;

    for (int j = 2; j <= 6; ++j) {
      std::string key = "L" + std::to_string(j) + ":";
      if (line.find(key) != std::string::npos &&
          line.find("mass") == std::string::npos &&
          line.find("com") == std::string::npos) {
        current_joint = "L" + std::to_string(j);
        break;
      }
    }

    if (!current_joint.empty() && line.find("mass:") != std::string::npos) {
      size_t cp = line.find("mass:");
      std::string val = line.substr(cp + 5);
      size_t hp = val.find('#');
      if (hp != std::string::npos) val = val.substr(0, hp);
      val.erase(std::remove_if(val.begin(), val.end(), ::isspace), val.end());
      int idx = std::stoi(current_joint.substr(1)) - 1;
      if (idx >= 0 && idx < 6) {
        entries[idx].mass = std::stod(val);
        entries[idx].loaded = true;
      }
    }

    if (!current_joint.empty() && line.find("com:") != std::string::npos) {
      size_t bs = line.find('['), be = line.find(']');
      if (bs != std::string::npos && be != std::string::npos) {
        std::stringstream ss(line.substr(bs + 1, be - bs - 1));
        std::vector<double> cv;
        std::string tok;
        while (std::getline(ss, tok, ',')) {
          tok.erase(std::remove_if(tok.begin(), tok.end(), ::isspace), tok.end());
          if (!tok.empty()) cv.push_back(std::stod(tok));
        }
        int idx = std::stoi(current_joint.substr(1)) - 1;
        if (idx >= 0 && idx < 6 && cv.size() >= 3) {
          entries[idx].com_x = cv[0];
          entries[idx].com_y = cv[1];
          entries[idx].com_z = cv[2];
          entries[idx].loaded = true;
        }
      }
    }
  }

  if (!use_calibrated) {
    RCLCPP_INFO(get_node()->get_logger(),
                "inertia config has use_calibrated_params=false, skipping");
    return false;
  }

  // Apply to Pinocchio model (L2-L6, model index i+1 skips universe)
  int applied = 0;
  for (int i = 1; i < 6 && i + 1 < model_.nbodies; ++i) {
    if (!entries[i].loaded) continue;
    model_.inertias[i + 1].mass()     = entries[i].mass;
    model_.inertias[i + 1].lever()[0] = entries[i].com_x;
    model_.inertias[i + 1].lever()[1] = entries[i].com_y;
    model_.inertias[i + 1].lever()[2] = entries[i].com_z;
    RCLCPP_INFO(get_node()->get_logger(),
                "  L%d: mass=%.4f, com=[%.6f, %.6f, %.6f]",
                i + 1, entries[i].mass,
                entries[i].com_x, entries[i].com_y, entries[i].com_z);
    ++applied;
  }

  data_ = pinocchio::Data(model_);
  RCLCPP_INFO(get_node()->get_logger(),
              "Applied calibrated inertia to %d joints", applied);
  return applied > 0;
}

}  // namespace el_a3_hardware

PLUGINLIB_EXPORT_CLASS(
  el_a3_hardware::ZeroTorqueController,
  controller_interface::ControllerInterface)
