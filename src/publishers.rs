use geometry_msgs::msg::{Twist, TwistStamped};
use safe_drive::node::Node;
use safe_drive::topic::publisher::Publisher;
use std::collections::HashMap;
use std::sync::Arc;
use anyhow::Result;
use std_msgs::msg::{Bool, Float64, Int32, String as StringMsg};

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
    pub fn from_profile(
        node: &Arc<Node>,
        profile: &crate::config::Profile,
    ) -> Result<Self> {
        let twist_publisher = node
            .create_publisher::<Twist>("cmd_vel", None)
            .map_err(|e| anyhow::anyhow!("Failed to create twist publisher on /cmd_vel: {:?}", e))?;

        let mut need_twist_stamped = false;
        let mut bool_topics = std::collections::HashSet::new();
        let mut int32_topics = std::collections::HashSet::new();
        let mut float64_topics = std::collections::HashSet::new();
        let mut string_topics = std::collections::HashSet::new();

        for mapping in &profile.input_mappings {
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

        let twist_stamped_publisher = if need_twist_stamped {
            Some(
                node.create_publisher::<TwistStamped>("cmd_vel_stamped", None)
                    .map_err(|e| anyhow::anyhow!("Failed to create twist stamped publisher on /cmd_vel_stamped: {:?}", e))?,
            )
        } else {
            None
        };

        let mut bool_publishers = HashMap::new();
        for topic in bool_topics {
            bool_publishers.insert(
                topic.clone(),
                node.create_publisher::<Bool>(&topic, None)
                    .map_err(|e| anyhow::anyhow!("Failed to create Bool publisher on {}: {:?}", topic, e))?,
            );
        }

        let mut int32_publishers = HashMap::new();
        for topic in int32_topics {
            int32_publishers.insert(
                topic.clone(),
                node.create_publisher::<Int32>(&topic, None)
                    .map_err(|e| anyhow::anyhow!("Failed to create Int32 publisher on {}: {:?}", topic, e))?,
            );
        }

        let mut float64_publishers = HashMap::new();
        for topic in float64_topics {
            float64_publishers.insert(
                topic.clone(),
                node.create_publisher::<Float64>(&topic, None)
                    .map_err(|e| anyhow::anyhow!("Failed to create Float64 publisher on {}: {:?}", topic, e))?,
            );
        }

        let mut string_publishers = HashMap::new();
        for topic in string_topics {
            string_publishers.insert(
                topic.clone(),
                node.create_publisher::<StringMsg>(&topic, None)
                    .map_err(|e| anyhow::anyhow!("Failed to create String publisher on {}: {:?}", topic, e))?,
            );
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
