use anyhow::Result;
use safe_drive::node::Node;
use std::collections::HashMap;
use std::sync::Arc;

/// Container for all service clients used by the node
/// For now, we store the service names and handle the actual service calls
/// in the main loop since safe_drive's service support requires proper ROS message types
pub struct ServiceClients {
    pub trigger_services: HashMap<String, String>, // service_name -> service_type
}

impl ServiceClients {
    pub fn from_profile(_node: &Arc<Node>, profile: &crate::config::Profile) -> Result<Self> {
        let mut trigger_services = HashMap::new();

        // Collect all service configurations
        for mapping in &profile.input_mappings {
            if let crate::config::ActionType::CallService {
                service_name,
                service_type,
            } = &mapping.action
            {
                match service_type.as_str() {
                    "std_srvs/srv/Trigger" => {
                        trigger_services.insert(service_name.clone(), service_type.clone());
                    }
                    _ => {
                        // For now, we only support Trigger services
                        // Other service types can be added later
                    }
                }
            }
        }

        Ok(Self { trigger_services })
    }
    
    pub fn has_service(&self, service_name: &str) -> bool {
        self.trigger_services.contains_key(service_name)
    }
    
    #[allow(dead_code)]
    pub fn get_service_type(&self, service_name: &str) -> Option<&String> {
        self.trigger_services.get(service_name)
    }
    
    /// Create service clients from mappings (for testing)
    #[cfg(test)]
    pub fn from_mappings(mappings: &[crate::config::InputMapping]) -> Self {
        let mut trigger_services = HashMap::new();
        
        for mapping in mappings {
            if let crate::config::ActionType::CallService { service_name, service_type } = &mapping.action {
                if service_type == "std_srvs/srv/Trigger" {
                    trigger_services.insert(service_name.clone(), service_type.clone());
                }
            }
        }
        
        Self {
            trigger_services,
        }
    }
}