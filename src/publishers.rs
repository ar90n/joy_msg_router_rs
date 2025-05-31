use anyhow::{anyhow, Result};
use geometry_msgs::msg::{Twist, Vector3};
use safe_drive::node::Node;
use std::collections::HashMap;
use std::sync::Arc;
use std_msgs::msg::{Bool, Float32, Float64, Int16, Int32, Int64, String as StringMsg};

/// Enum to hold different publisher types
pub enum Publisher {
    Bool(safe_drive::topic::publisher::Publisher<Bool>),
    Int16(safe_drive::topic::publisher::Publisher<Int16>),
    Int32(safe_drive::topic::publisher::Publisher<Int32>),
    Int64(safe_drive::topic::publisher::Publisher<Int64>),
    Float32(safe_drive::topic::publisher::Publisher<Float32>),
    Float64(safe_drive::topic::publisher::Publisher<Float64>),
    String(safe_drive::topic::publisher::Publisher<StringMsg>),
    Twist(safe_drive::topic::publisher::Publisher<Twist>),
    Vector3(safe_drive::topic::publisher::Publisher<Vector3>),
}

/// Container for publishers
pub struct Publishers {
    /// Map from topic to publisher
    publishers: HashMap<String, (Publisher, String)>, // (publisher, message_type)
}

impl Publishers {
    pub fn new() -> Self {
        Self {
            publishers: HashMap::new(),
        }
    }

    /// Create a publisher for a generic message type
    pub fn create_publisher(
        &mut self,
        node: &Arc<Node>,
        topic: &str,
        message_type: &str,
    ) -> Result<()> {
        // Check if publisher already exists
        if self.publishers.contains_key(topic) {
            return Ok(());
        }

        let publisher = match message_type {
            "std_msgs/msg/Bool" => Publisher::Bool(node.create_publisher::<Bool>(topic, None)?),
            "std_msgs/msg/Int16" => Publisher::Int16(node.create_publisher::<Int16>(topic, None)?),
            "std_msgs/msg/Int32" => Publisher::Int32(node.create_publisher::<Int32>(topic, None)?),
            "std_msgs/msg/Int64" => Publisher::Int64(node.create_publisher::<Int64>(topic, None)?),
            "std_msgs/msg/Float32" => {
                Publisher::Float32(node.create_publisher::<Float32>(topic, None)?)
            }
            "std_msgs/msg/Float64" => {
                Publisher::Float64(node.create_publisher::<Float64>(topic, None)?)
            }
            "std_msgs/msg/String" => {
                Publisher::String(node.create_publisher::<StringMsg>(topic, None)?)
            }
            "geometry_msgs/msg/Twist" => {
                Publisher::Twist(node.create_publisher::<Twist>(topic, None)?)
            }
            "geometry_msgs/msg/Vector3" => {
                Publisher::Vector3(node.create_publisher::<Vector3>(topic, None)?)
            }
            _ => return Err(anyhow!("Unsupported message type: {}", message_type)),
        };

        self.publishers
            .insert(topic.to_string(), (publisher, message_type.to_string()));
        Ok(())
    }

    /// Publish a value to a generic topic
    pub fn publish_value(&self, topic: &str, value: f64, field: Option<&str>) -> Result<()> {
        let (publisher, message_type) = self
            .publishers
            .get(topic)
            .ok_or_else(|| anyhow!("No publisher for topic: {}", topic))?;

        match (publisher, message_type.as_str()) {
            (Publisher::Bool(pub_ref), "std_msgs/msg/Bool") => {
                let mut msg = Bool::new().unwrap();
                msg.data = value != 0.0;
                pub_ref
                    .send(&msg)
                    .map_err(|e| anyhow!("Failed to send Bool: {:?}", e))?;
            }
            (Publisher::Int16(pub_ref), "std_msgs/msg/Int16") => {
                let mut msg = Int16::new().unwrap();
                msg.data = value as i16;
                pub_ref
                    .send(&msg)
                    .map_err(|e| anyhow!("Failed to send Bool: {:?}", e))?;
            }
            (Publisher::Int32(pub_ref), "std_msgs/msg/Int32") => {
                let mut msg = Int32::new().unwrap();
                msg.data = value as i32;
                pub_ref
                    .send(&msg)
                    .map_err(|e| anyhow!("Failed to send Bool: {:?}", e))?;
            }
            (Publisher::Int64(pub_ref), "std_msgs/msg/Int64") => {
                let mut msg = Int64::new().unwrap();
                msg.data = value as i64;
                pub_ref
                    .send(&msg)
                    .map_err(|e| anyhow!("Failed to send Bool: {:?}", e))?;
            }
            (Publisher::Float32(pub_ref), "std_msgs/msg/Float32") => {
                let mut msg = Float32::new().unwrap();
                msg.data = value as f32;
                pub_ref
                    .send(&msg)
                    .map_err(|e| anyhow!("Failed to send Bool: {:?}", e))?;
            }
            (Publisher::Float64(pub_ref), "std_msgs/msg/Float64") => {
                let mut msg = Float64::new().unwrap();
                msg.data = value;
                pub_ref
                    .send(&msg)
                    .map_err(|e| anyhow!("Failed to send Bool: {:?}", e))?;
            }
            (Publisher::String(pub_ref), "std_msgs/msg/String") => {
                let mut msg = StringMsg::new().unwrap();
                msg.data.assign(&value.to_string());
                pub_ref
                    .send(&msg)
                    .map_err(|e| anyhow!("Failed to send String: {:?}", e))?;
            }
            (Publisher::Twist(pub_ref), "geometry_msgs/msg/Twist") => {
                let mut msg = Twist::new().unwrap();
                if let Some(field_name) = field {
                    match field_name {
                        "linear_x" | "linear.x" => msg.linear.x = value,
                        "linear_y" | "linear.y" => msg.linear.y = value,
                        "linear_z" | "linear.z" => msg.linear.z = value,
                        "angular_x" | "angular.x" => msg.angular.x = value,
                        "angular_y" | "angular.y" => msg.angular.y = value,
                        "angular_z" | "angular.z" => msg.angular.z = value,
                        _ => return Err(anyhow!("Unknown Twist field: {}", field_name)),
                    }
                }
                pub_ref
                    .send(&msg)
                    .map_err(|e| anyhow!("Failed to send Bool: {:?}", e))?;
            }
            (Publisher::Vector3(pub_ref), "geometry_msgs/msg/Vector3") => {
                let mut msg = Vector3::new().unwrap();
                if let Some(field_name) = field {
                    match field_name {
                        "x" => msg.x = value,
                        "y" => msg.y = value,
                        "z" => msg.z = value,
                        _ => return Err(anyhow!("Unknown Vector3 field: {}", field_name)),
                    }
                }
                pub_ref
                    .send(&msg)
                    .map_err(|e| anyhow!("Failed to send Bool: {:?}", e))?;
            }
            _ => return Err(anyhow!("Type mismatch for topic: {}", topic)),
        }

        Ok(())
    }

    /// Create publishers from profile
    pub fn from_profile(node: &Arc<Node>, profile: &crate::config::Profile) -> Result<Self> {
        let mut publishers = Self::new();

        // Scan through all mappings to find generic publishers
        for mapping in &profile.input_mappings {
            if let crate::config::ActionType::Publish {
                topic,
                message_type,
                ..
            } = &mapping.action
            {
                publishers.create_publisher(node, topic, message_type)?;
            }
        }

        Ok(publishers)
    }

    /// Check if a topic publishes Twist messages
    #[allow(dead_code)]
    pub fn is_twist_topic(&self, topic: &str) -> bool {
        self.publishers
            .get(topic)
            .map(|(_, msg_type)| msg_type == "geometry_msgs/msg/Twist")
            .unwrap_or(false)
    }


    /// Get supported message types
    #[allow(dead_code)]
    pub fn supported_types() -> Vec<&'static str> {
        vec![
            "std_msgs/msg/Bool",
            "std_msgs/msg/Int16",
            "std_msgs/msg/Int32",
            "std_msgs/msg/Int64",
            "std_msgs/msg/Float32",
            "std_msgs/msg/Float64",
            "std_msgs/msg/String",
            "geometry_msgs/msg/Twist",
            "geometry_msgs/msg/Vector3",
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_supported_types() {
        let types = Publishers::supported_types();
        assert!(types.contains(&"std_msgs/msg/Float64"));
        assert!(types.contains(&"geometry_msgs/msg/Twist"));
        assert!(types.contains(&"geometry_msgs/msg/Vector3"));
    }
}
