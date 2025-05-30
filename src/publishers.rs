use geometry_msgs::msg::Twist;
use safe_drive::node::Node;
use safe_drive::topic::publisher::Publisher;
use std::collections::HashMap;
use std::sync::Arc;
use anyhow::Result;
use std_msgs::msg::Bool;

/// Container for all publishers used by the node
pub struct Publishers {
    pub twist_publisher: Publisher<Twist>,
    pub bool_publishers: HashMap<String, Publisher<Bool>>,
}

impl Publishers {
    pub fn from_profile(
        node: &Arc<Node>,
        profile: &crate::config::Profile,
    ) -> Result<Self> {
        let twist_publisher = node
            .create_publisher::<Twist>("cmd_vel", None)
            .map_err(|e| anyhow::anyhow!("Failed to create twist publisher on /cmd_vel: {:?}", e))?;

        let mut bool_topics = std::collections::HashSet::new();

        for mapping in &profile.input_mappings {
            if let crate::config::ActionType::PublishBool { topic, .. } = &mapping.action {
                bool_topics.insert(topic.clone());
            }
        }

        let mut bool_publishers = HashMap::new();
        for topic in bool_topics {
            bool_publishers.insert(
                topic.clone(),
                node.create_publisher::<Bool>(&topic, None)
                    .map_err(|e| anyhow::anyhow!("Failed to create Bool publisher on {}: {:?}", topic, e))?,
            );
        }

        Ok(Self {
            twist_publisher,
            bool_publishers,
        })
    }
}
