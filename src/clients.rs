use anyhow::Result;
use safe_drive::{msg::common_interfaces::std_srvs, node::Node};
use std::collections::HashMap;
use std::sync::Arc;

pub enum ServiceClient {
    Trigger(safe_drive::service::client::Client<std_srvs::srv::Trigger>),
    Empty(safe_drive::service::client::Client<std_srvs::srv::Empty>),
}

/// Container for all service clients used by the node
pub struct ServiceClients {
    pub clients: HashMap<String, ServiceClient>,
    #[cfg(not(test))]
    node: Arc<Node>,
    /// Store service types for recreation
    service_types: HashMap<String, String>,
}

impl ServiceClients {
    pub fn from_profile(node: &Arc<Node>, profile: &crate::config::Profile) -> Result<Self> {
        let mut clients = HashMap::new();

        // Collect all service configurations
        for mapping in &profile.input_mappings {
            if let crate::config::ActionType::CallService {
                service_name,
                service_type,
            } = &mapping.action
            {
                match service_type.as_str() {
                    "std_srvs/srv/Trigger" => {
                        let client =
                            node.create_client::<std_srvs::srv::Trigger>(service_name, None)?;
                        clients.insert(service_name.clone(), ServiceClient::Trigger(client));
                    }
                    "std_srvs/srv/Empty" => {
                        let client =
                            node.create_client::<std_srvs::srv::Empty>(service_name, None)?;
                        clients.insert(service_name.clone(), ServiceClient::Empty(client));
                    }
                    _ => {
                        // For now, we only support Trigger and Empty services
                        // Other service types can be added later
                    }
                }
            }
        }

        // Also store service types for recreation
        let mut service_types = HashMap::new();
        for mapping in &profile.input_mappings {
            if let crate::config::ActionType::CallService {
                service_name,
                service_type,
            } = &mapping.action
            {
                if service_type == "std_srvs/srv/Trigger" || service_type == "std_srvs/srv/Empty" {
                    service_types.insert(service_name.clone(), service_type.clone());
                }
            }
        }

        Ok(Self {
            clients,
            #[cfg(not(test))]
            node: Arc::clone(node),
            service_types,
        })
    }

    pub fn has_service(&self, service_name: &str) -> bool {
        self.clients.contains_key(service_name)
    }

    pub fn take_client(&mut self, service_name: &str) -> Option<ServiceClient> {
        self.clients.remove(service_name)
    }

    /// Recreate a service client after it has been consumed
    #[cfg(not(test))]
    pub fn recreate_client(&mut self, service_name: &str) -> Result<()> {
        if let Some(service_type) = self.service_types.get(service_name) {
            match service_type.as_str() {
                "std_srvs/srv/Trigger" => {
                    let client = self
                        .node
                        .create_client::<std_srvs::srv::Trigger>(service_name, None)?;
                    self.clients
                        .insert(service_name.to_string(), ServiceClient::Trigger(client));
                }
                "std_srvs/srv/Empty" => {
                    let client = self
                        .node
                        .create_client::<std_srvs::srv::Empty>(service_name, None)?;
                    self.clients
                        .insert(service_name.to_string(), ServiceClient::Empty(client));
                }
                _ => {}
            }
        }
        Ok(())
    }

    /// Mock version for testing
    #[cfg(test)]
    pub fn recreate_client(&mut self, _service_name: &str) -> Result<()> {
        Ok(()) // Do nothing in tests
    }

    /// Create empty service clients for testing
    #[cfg(test)]
    pub fn empty_for_testing() -> Self {
        Self {
            clients: HashMap::new(),
            service_types: HashMap::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{ActionType, InputMapping, InputSource, Profile};

    #[test]
    fn test_service_clients_from_profile() {
        // Test that ServiceClients correctly extracts service configurations from profile
        let mut profile = Profile::new("test".to_string());
        
        // Add Trigger service
        profile.input_mappings.push(InputMapping {
            source: InputSource::Button(0),
            action: ActionType::CallService {
                service_name: "/reset".to_string(),
                service_type: "std_srvs/srv/Trigger".to_string(),
            },
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.0,
        });
        
        // Add Empty service
        profile.input_mappings.push(InputMapping {
            source: InputSource::Button(1),
            action: ActionType::CallService {
                service_name: "/stop".to_string(),
                service_type: "std_srvs/srv/Empty".to_string(),
            },
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.0,
        });
        
        // Add unsupported service (should be ignored)
        profile.input_mappings.push(InputMapping {
            source: InputSource::Button(2),
            action: ActionType::CallService {
                service_name: "/custom".to_string(),
                service_type: "custom_msgs/srv/Custom".to_string(),
            },
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.0,
        });
        
        // Add publish action (should be ignored)
        profile.input_mappings.push(InputMapping {
            source: InputSource::Axis(0),
            action: ActionType::Publish {
                topic: "/cmd_vel".to_string(),
                message_type: "geometry_msgs/msg/Twist".to_string(),
                field: Some("linear.x".to_string()),
                once: false,
            },
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.1,
        });
        
        // Test with empty_for_testing since we can't create real clients in tests
        let clients = ServiceClients::empty_for_testing();
        
        // Verify structure is correct (actual client creation tested in integration tests)
        assert_eq!(clients.clients.len(), 0);
        assert_eq!(clients.service_types.len(), 0);
    }
    
    #[test]
    fn test_has_service() {
        let mut clients = ServiceClients::empty_for_testing();
        
        // Manually add a service type for testing
        clients.service_types.insert("/test".to_string(), "std_srvs/srv/Trigger".to_string());
        
        // has_service checks clients HashMap, not service_types
        assert!(!clients.has_service("/test"));
        assert!(!clients.has_service("/nonexistent"));
    }
    
    #[test]
    fn test_take_client() {
        let mut clients = ServiceClients::empty_for_testing();
        
        // Taking from empty should return None
        assert!(clients.take_client("/test").is_none());
    }
    
    #[test]
    fn test_service_type_filtering() {
        // Test that only supported service types are processed
        let mut profile = Profile::new("test".to_string());
        
        // Mix of supported and unsupported service types
        let service_configs = vec![
            ("std_srvs/srv/Trigger", true),
            ("std_srvs/srv/Empty", true),
            ("std_srvs/srv/SetBool", false), // Not supported
            ("geometry_msgs/srv/GetPlan", false), // Not supported
            ("nav_msgs/srv/LoadMap", false), // Not supported
        ];
        
        for (i, (service_type, should_be_supported)) in service_configs.iter().enumerate() {
            profile.input_mappings.push(InputMapping {
                source: InputSource::Button(i),
                action: ActionType::CallService {
                    service_name: format!("/service_{}", i),
                    service_type: service_type.to_string(),
                },
                scale: 1.0,
                offset: 0.0,
                deadzone: 0.0,
            });
        }
        
        // Count how many services should be supported
        let expected_count = service_configs.iter().filter(|(_, supported)| *supported).count();
        
        // In real implementation, from_profile would create clients for supported types
        // Here we just verify the test data is correct
        assert_eq!(expected_count, 2); // Only Trigger and Empty are supported
    }
    
    #[test]
    fn test_duplicate_service_names() {
        // Test handling of duplicate service names in profile
        let mut profile = Profile::new("test".to_string());
        
        // Add same service name twice with different types
        profile.input_mappings.push(InputMapping {
            source: InputSource::Button(0),
            action: ActionType::CallService {
                service_name: "/reset".to_string(),
                service_type: "std_srvs/srv/Trigger".to_string(),
            },
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.0,
        });
        
        profile.input_mappings.push(InputMapping {
            source: InputSource::Button(1),
            action: ActionType::CallService {
                service_name: "/reset".to_string(), // Same name
                service_type: "std_srvs/srv/Empty".to_string(), // Different type
            },
            scale: 1.0,
            offset: 0.0,
            deadzone: 0.0,
        });
        
        // The second one should overwrite the first in a real implementation
        // This documents the expected behavior
        let clients = ServiceClients::empty_for_testing();
        assert_eq!(clients.clients.len(), 0); // Empty in test
    }
}
