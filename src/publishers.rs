use std::collections::HashMap;
use std::sync::Arc;
use safe_drive::node::Node;
use safe_drive::topic::publisher::Publisher;
use safe_drive::error::DynError;
use geometry_msgs::msg::{Twist, TwistStamped};
use std_msgs::msg::{Bool, Int32, Float64, String as StringMsg};

/// Container for all publishers used by the node
pub struct Publishers {
    pub twist_publisher: Publisher<Twist>,
    pub twist_stamped_publisher: Option<Publisher<TwistStamped>>,
    pub bool_publishers: HashMap<String, Publisher<Bool>>,
    pub int32_publishers: HashMap<String, Publisher<Int32>>,
    pub float64_publishers: HashMap<String, Publisher<Float64>>,
    pub string_publishers: HashMap<String, Publisher<StringMsg>>,
}

impl Publishers {
    /// Create publishers based on the profile configuration
    pub fn from_profile(node: &Arc<Node>, profile: &crate::config::Profile) -> Result<Self, DynError> {
        // Always create the default twist publisher
        let twist_publisher = node.create_publisher::<Twist>("cmd_vel", None)?;
        
        // Check if we need a TwistStamped publisher
        let mut need_twist_stamped = false;
        let mut bool_topics = std::collections::HashSet::new();
        let mut int32_topics = std::collections::HashSet::new();
        let mut float64_topics = std::collections::HashSet::new();
        let mut string_topics = std::collections::HashSet::new();
        
        // Scan button mappings to determine what publishers we need
        for mapping in &profile.button_mappings {
            match &mapping.action {
                crate::config::ActionType::PublishTwistStamped { .. } => {
                    need_twist_stamped = true;
                }
                crate::config::ActionType::PublishBool { topic, .. } => {
                    bool_topics.insert(topic.clone());
                }
                crate::config::ActionType::PublishInt32 { topic, .. } => {
                    int32_topics.insert(topic.clone());
                }
                crate::config::ActionType::PublishFloat64 { topic, .. } => {
                    float64_topics.insert(topic.clone());
                }
                crate::config::ActionType::PublishString { topic, .. } => {
                    string_topics.insert(topic.clone());
                }
                _ => {}
            }
        }
        
        // Create TwistStamped publisher if needed
        let twist_stamped_publisher = if need_twist_stamped {
            Some(node.create_publisher::<TwistStamped>("cmd_vel_stamped", None)?)
        } else {
            None
        };
        
        // Create Bool publishers
        let mut bool_publishers = HashMap::new();
        for topic in bool_topics {
            bool_publishers.insert(topic.clone(), node.create_publisher::<Bool>(&topic, None)?);
        }
        
        // Create Int32 publishers
        let mut int32_publishers = HashMap::new();
        for topic in int32_topics {
            int32_publishers.insert(topic.clone(), node.create_publisher::<Int32>(&topic, None)?);
        }
        
        // Create Float64 publishers
        let mut float64_publishers = HashMap::new();
        for topic in float64_topics {
            float64_publishers.insert(topic.clone(), node.create_publisher::<Float64>(&topic, None)?);
        }
        
        // Create String publishers
        let mut string_publishers = HashMap::new();
        for topic in string_topics {
            string_publishers.insert(topic.clone(), node.create_publisher::<StringMsg>(&topic, None)?);
        }
        
        Ok(Self {
            twist_publisher,
            twist_stamped_publisher,
            bool_publishers,
            int32_publishers,
            float64_publishers,
            string_publishers,
        })
    }
}