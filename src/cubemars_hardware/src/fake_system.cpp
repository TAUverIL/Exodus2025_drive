#include "cubemars_hardware/fake_system.hpp"

#include <cmath>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cubemars_hardware
{

hardware_interface::CallbackReturn FakeCubeMarsHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("FakeCubeMarsHardware"),
    "Initializing fake hardware interface for %zu joints", info_.joints.size());

  // Initialize state and command vectors
  hw_states_positions_.resize(info_.joints.size(), 0.0);
  hw_states_velocities_.resize(info_.joints.size(), 0.0);
  hw_states_efforts_.resize(info_.joints.size(), 0.0);
  hw_states_temperatures_.resize(info_.joints.size(), 25.0);  // Room temperature
  
  hw_commands_positions_.resize(info_.joints.size(), 0.0);
  hw_commands_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_efforts_.resize(info_.joints.size(), 0.0);

  // Simple damping for simulation
  hw_position_damping_.resize(info_.joints.size(), 0.95);
  hw_velocity_damping_.resize(info_.joints.size(), 0.90);

  // Initialize states to zero position
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    hw_states_positions_[i] = 0.0;
    hw_commands_positions_[i] = 0.0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FakeCubeMarsHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("FakeCubeMarsHardware"), "Configuring fake hardware");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
FakeCubeMarsHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_efforts_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "temperature", &hw_states_temperatures_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FakeCubeMarsHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn FakeCubeMarsHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("FakeCubeMarsHardware"), "Activating fake hardware");
  
  // Set initial command to current state
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (std::isnan(hw_commands_positions_[i]))
    {
      hw_commands_positions_[i] = hw_states_positions_[i];
    }
    if (std::isnan(hw_commands_velocities_[i]))
    {
      hw_commands_velocities_[i] = 0.0;
    }
    if (std::isnan(hw_commands_efforts_[i]))
    {
      hw_commands_efforts_[i] = 0.0;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FakeCubeMarsHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("FakeCubeMarsHardware"), "Deactivating fake hardware");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FakeCubeMarsHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // In fake hardware, states are already updated in write()
  // This mimics sensor reading, but we're just using the internally tracked state
  
  // Simulate slight temperature variation based on effort
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    // Temperature increases slightly with effort, decreases towards ambient
    double ambient_temp = 25.0;
    double effort_heating = std::abs(hw_states_efforts_[i]) * 2.0;
    hw_states_temperatures_[i] = hw_states_temperatures_[i] * 0.95 + 
                                  (ambient_temp + effort_heating) * 0.05;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FakeCubeMarsHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Simple simulation: mirror commanded values to state with some damping
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    // Position control (most common mode)
    if (!std::isnan(hw_commands_positions_[i]))
    {
      // Simple proportional control simulation
      double position_error = hw_commands_positions_[i] - hw_states_positions_[i];
      double commanded_velocity = position_error * 5.0;  // Simple P controller
      
      // Update velocity with damping
      hw_states_velocities_[i] = commanded_velocity * hw_velocity_damping_[i];
      
      // Update position
      hw_states_positions_[i] += hw_states_velocities_[i] * period.seconds();
      
      // Estimate effort from position error (spring model)
      hw_states_efforts_[i] = position_error * 1.0;
    }
    // Velocity control
    else if (!std::isnan(hw_commands_velocities_[i]))
    {
      hw_states_velocities_[i] = hw_commands_velocities_[i] * hw_velocity_damping_[i];
      hw_states_positions_[i] += hw_states_velocities_[i] * period.seconds();
      hw_states_efforts_[i] = hw_commands_velocities_[i] * 0.1;  // Damping force
    }
    // Effort control
    else if (!std::isnan(hw_commands_efforts_[i]))
    {
      // Simple mass-spring-damper model
      hw_states_efforts_[i] = hw_commands_efforts_[i];
      double acceleration = hw_commands_efforts_[i] * 0.1;  // Assume unit mass
      hw_states_velocities_[i] += acceleration * period.seconds();
      hw_states_velocities_[i] *= hw_velocity_damping_[i];
      hw_states_positions_[i] += hw_states_velocities_[i] * period.seconds();
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace cubemars_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cubemars_hardware::FakeCubeMarsHardware, hardware_interface::SystemInterface)

