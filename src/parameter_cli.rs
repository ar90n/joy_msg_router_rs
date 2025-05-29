use clap::{Arg, Command, ArgMatches};
use serde_json;
use crate::parameter_manager::{ParameterManager, ParameterType};
use crate::error::{JoyRouterError, JoyRouterResult};

/// Command-line interface for parameter management
pub struct ParameterCli {
    /// Description of all available commands
    commands: Vec<CliCommand>,
}

/// A CLI command definition
struct CliCommand {
    /// Command name
    name: String,
    /// Command description
    description: String,
    /// Command arguments
    args: Vec<CliArgument>,
}

/// A CLI argument definition
struct CliArgument {
    /// Argument name
    name: String,
    /// Argument description
    description: String,
    /// Whether argument is required
    required: bool,
    /// Possible values (for validation)
    possible_values: Option<Vec<String>>,
}

impl ParameterCli {
    /// Create a new parameter CLI
    pub fn new() -> Self {
        Self {
            commands: vec![
                CliCommand {
                    name: "list".to_string(),
                    description: "List all available parameters".to_string(),
                    args: vec![],
                },
                CliCommand {
                    name: "get".to_string(),
                    description: "Get parameter value".to_string(),
                    args: vec![
                        CliArgument {
                            name: "name".to_string(),
                            description: "Parameter name".to_string(),
                            required: true,
                            possible_values: None,
                        },
                    ],
                },
                CliCommand {
                    name: "set".to_string(),
                    description: "Set parameter value".to_string(),
                    args: vec![
                        CliArgument {
                            name: "name".to_string(),
                            description: "Parameter name".to_string(),
                            required: true,
                            possible_values: None,
                        },
                        CliArgument {
                            name: "value".to_string(),
                            description: "Parameter value".to_string(),
                            required: true,
                            possible_values: None,
                        },
                    ],
                },
                CliCommand {
                    name: "describe".to_string(),
                    description: "Describe parameter with constraints".to_string(),
                    args: vec![
                        CliArgument {
                            name: "name".to_string(),
                            description: "Parameter name".to_string(),
                            required: true,
                            possible_values: None,
                        },
                    ],
                },
                CliCommand {
                    name: "reload".to_string(),
                    description: "Reload configuration from file".to_string(),
                    args: vec![],
                },
                CliCommand {
                    name: "switch-profile".to_string(),
                    description: "Switch to a different profile".to_string(),
                    args: vec![
                        CliArgument {
                            name: "profile".to_string(),
                            description: "Profile name".to_string(),
                            required: true,
                            possible_values: None,
                        },
                    ],
                },
                CliCommand {
                    name: "export".to_string(),
                    description: "Export all parameters as JSON".to_string(),
                    args: vec![],
                },
            ],
        }
    }

    /// Build the CLI application
    pub fn build_app(&self) -> Command {
        Command::new("joy_param")
            .about("ROS2 Joy Message Router Parameter Management")
            .version("1.0")
            .author("Joy Router Team")
            .subcommand(
                Command::new("list")
                    .about("List all available parameters")
            )
            .subcommand(
                Command::new("get")
                    .about("Get parameter value")
                    .arg(Arg::new("name")
                        .help("Parameter name")
                        .required(true))
            )
            .subcommand(
                Command::new("set")
                    .about("Set parameter value")
                    .arg(Arg::new("name")
                        .help("Parameter name")
                        .required(true))
                    .arg(Arg::new("value")
                        .help("Parameter value")
                        .required(true))
            )
            .subcommand(
                Command::new("describe")
                    .about("Describe parameter with constraints")
                    .arg(Arg::new("name")
                        .help("Parameter name")
                        .required(true))
            )
            .subcommand(
                Command::new("reload")
                    .about("Reload configuration from file")
            )
            .subcommand(
                Command::new("switch-profile")
                    .about("Switch to a different profile")
                    .arg(Arg::new("profile")
                        .help("Profile name")
                        .required(true))
            )
            .subcommand(
                Command::new("export")
                    .about("Export all parameters as JSON")
            )
    }

    /// Process CLI commands
    pub fn process_command(
        &self,
        matches: &ArgMatches,
        parameter_manager: &ParameterManager,
    ) -> JoyRouterResult<String> {
        match matches.subcommand() {
            Some(("list", _)) => self.list_parameters(parameter_manager),
            Some(("get", sub_matches)) => {
                let name = sub_matches.get_one::<String>("name")
                    .ok_or_else(|| JoyRouterError::ConfigError("Parameter name is required".to_string()))?;
                self.get_parameter(parameter_manager, name)
            }
            Some(("set", sub_matches)) => {
                let name = sub_matches.get_one::<String>("name")
                    .ok_or_else(|| JoyRouterError::ConfigError("Parameter name is required".to_string()))?;
                let value = sub_matches.get_one::<String>("value")
                    .ok_or_else(|| JoyRouterError::ConfigError("Parameter value is required".to_string()))?;
                self.set_parameter(parameter_manager, name, value)
            }
            Some(("describe", sub_matches)) => {
                let name = sub_matches.get_one::<String>("name")
                    .ok_or_else(|| JoyRouterError::ConfigError("Parameter name is required".to_string()))?;
                self.describe_parameter(parameter_manager, name)
            }
            Some(("reload", _)) => self.reload_configuration(parameter_manager),
            Some(("switch-profile", sub_matches)) => {
                let profile = sub_matches.get_one::<String>("profile")
                    .ok_or_else(|| JoyRouterError::ConfigError("Profile name is required".to_string()))?;
                self.switch_profile(parameter_manager, profile)
            }
            Some(("export", _)) => self.export_parameters(parameter_manager),
            _ => Ok("No command specified. Use --help for available commands.".to_string()),
        }
    }

    /// List all parameters
    fn list_parameters(&self, parameter_manager: &ParameterManager) -> JoyRouterResult<String> {
        let mut output = String::new();
        output.push_str("Available Parameters:\n");
        output.push_str("====================\n\n");

        let descriptors = parameter_manager.get_descriptors();
        let current_values = parameter_manager.get_all_parameters()?;

        for (name, descriptor) in descriptors {
            let default_value = "N/A".to_string();
            let current_value = current_values.get(name).unwrap_or(&default_value);
            let readonly_marker = if descriptor.read_only { " (read-only)" } else { "" };
            
            output.push_str(&format!(
                "  {}{}: {} ({})\n    Current value: {}\n\n",
                name,
                readonly_marker,
                descriptor.description,
                Self::type_to_string(&descriptor.param_type),
                current_value
            ));
        }

        Ok(output)
    }

    /// Get a parameter value
    fn get_parameter(&self, parameter_manager: &ParameterManager, name: &str) -> JoyRouterResult<String> {
        if let Some(value) = parameter_manager.get_parameter(name) {
            Ok(format!("{}: {}", name, value))
        } else {
            Err(JoyRouterError::ConfigError(format!("Parameter '{}' not found", name)))
        }
    }

    /// Set a parameter value
    fn set_parameter(&self, parameter_manager: &ParameterManager, name: &str, value: &str) -> JoyRouterResult<String> {
        parameter_manager.set_parameter(name, value)?;
        Ok(format!("Parameter '{}' set to '{}'", name, value))
    }

    /// Describe a parameter with its constraints
    fn describe_parameter(&self, parameter_manager: &ParameterManager, name: &str) -> JoyRouterResult<String> {
        let descriptors = parameter_manager.get_descriptors();
        let descriptor = descriptors.get(name)
            .ok_or_else(|| JoyRouterError::ConfigError(format!("Parameter '{}' not found", name)))?;

        let mut output = String::new();
        output.push_str(&format!("Parameter: {}\n", name));
        output.push_str(&format!("Description: {}\n", descriptor.description));
        output.push_str(&format!("Type: {}\n", Self::type_to_string(&descriptor.param_type)));
        output.push_str(&format!("Read-only: {}\n", descriptor.read_only));

        if let Some(constraints) = &descriptor.constraints {
            output.push_str("Constraints:\n");
            if let Some(min) = constraints.min_value {
                output.push_str(&format!("  Minimum value: {}\n", min));
            }
            if let Some(max) = constraints.max_value {
                output.push_str(&format!("  Maximum value: {}\n", max));
            }
            if let Some(values) = &constraints.valid_values {
                output.push_str(&format!("  Valid values: {:?}\n", values));
            }
            if let Some(max_len) = constraints.max_length {
                output.push_str(&format!("  Maximum length: {}\n", max_len));
            }
        }

        if let Some(current_value) = parameter_manager.get_parameter(name) {
            output.push_str(&format!("Current value: {}\n", current_value));
        }

        Ok(output)
    }

    /// Reload configuration
    fn reload_configuration(&self, parameter_manager: &ParameterManager) -> JoyRouterResult<String> {
        parameter_manager.reload_configuration()?;
        Ok("Configuration reloaded successfully".to_string())
    }

    /// Switch profile
    fn switch_profile(&self, parameter_manager: &ParameterManager, profile: &str) -> JoyRouterResult<String> {
        parameter_manager.switch_profile(profile)?;
        Ok(format!("Switched to profile: {}", profile))
    }

    /// Export parameters as JSON
    fn export_parameters(&self, parameter_manager: &ParameterManager) -> JoyRouterResult<String> {
        let parameters = parameter_manager.get_all_parameters()?;
        let json = serde_json::to_string_pretty(&parameters)
            .map_err(|e| JoyRouterError::ConfigError(format!("Failed to serialize parameters: {}", e)))?;
        Ok(json)
    }

    /// Convert parameter type to string
    fn type_to_string(param_type: &ParameterType) -> &'static str {
        match param_type {
            ParameterType::String => "string",
            ParameterType::Integer => "integer",
            ParameterType::Double => "double",
            ParameterType::Boolean => "boolean",
            ParameterType::StringArray => "string[]",
            ParameterType::IntegerArray => "integer[]",
            ParameterType::DoubleArray => "double[]",
        }
    }
}

/// Standalone parameter management utility
pub fn run_parameter_cli() -> JoyRouterResult<()> {
    // This would typically connect to a running node via ROS2 services
    // For now, we'll return an informational message
    println!("Parameter CLI would connect to running joy_msg_router node");
    println!("Use 'ros2 param list /joy_msg_router' to see available parameters");
    println!("Use 'ros2 param get /joy_msg_router <param_name>' to get values");
    println!("Use 'ros2 param set /joy_msg_router <param_name> <value>' to set values");
    
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cli_creation() {
        let cli = ParameterCli::new();
        assert!(!cli.commands.is_empty());
        
        let app = cli.build_app();
        assert_eq!(app.get_name(), "joy_param");
    }

    #[test]
    fn test_type_conversion() {
        assert_eq!(ParameterCli::type_to_string(&ParameterType::String), "string");
        assert_eq!(ParameterCli::type_to_string(&ParameterType::Integer), "integer");
        assert_eq!(ParameterCli::type_to_string(&ParameterType::Double), "double");
        assert_eq!(ParameterCli::type_to_string(&ParameterType::Boolean), "boolean");
        assert_eq!(ParameterCli::type_to_string(&ParameterType::StringArray), "string[]");
        assert_eq!(ParameterCli::type_to_string(&ParameterType::IntegerArray), "integer[]");
        assert_eq!(ParameterCli::type_to_string(&ParameterType::DoubleArray), "double[]");
    }
}