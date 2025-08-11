#include <ros/ros.h>
#include <gpbf_msgs/Obstacle.h>
#include <gpbf_msgs/ObstacleArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ObstaclePublisher {
public:
    ObstaclePublisher(ros::NodeHandle& nh) : nh_(nh), tfBuffer_(), tfListener_(tfBuffer_) {
        // 订阅Gazebo模型状态
        gazebo_sub_ = nh_.subscribe("/gazebo/model_states", 10, &ObstaclePublisher::gazeboCallback, this);
        
        // 创建障碍物发布器
        obstacle_pub_ = nh_.advertise<gpbf_msgs::ObstacleArray>("/obstacles", 10);
        
        // 设置障碍物名称前缀
        obstacle_prefix_ = "obstacle";
        
        ROS_INFO("Obstacle publisher initialized");
    }
    
    void gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        // 创建障碍物数组消息
        gpbf_msgs::ObstacleArray obstacles_msg;
        obstacles_msg.header.stamp = ros::Time::now();
        obstacles_msg.header.frame_id = "map";
        
        // 遍历所有模型
        for (size_t i = 0; i < msg->name.size(); i++) {
            const std::string& name = msg->name[i];
            
            // 检查是否是障碍物
            if (name.find(obstacle_prefix_) == 0) {
                // 创建障碍物消息
                gpbf_msgs::Obstacle obstacle;
                
                // 设置障碍物位置
                obstacle.position = msg->pose[i].position;
                
                // 设置障碍物半径 (从Gazebo世界文件中硬编码获取)
                obstacle.radius = 0.5;  // 在实际应用中，您可能想从参数服务器获取或动态计算
                
                // 设置障碍物ID
                try {
                    // 提取ID，假设格式为"obstacle1", "obstacle2"等
                    std::string id_str = name.substr(obstacle_prefix_.length());
                    obstacle.id = std::stoi(id_str);
                } catch (const std::exception& e) {
                    obstacle.id = i;  // 如果无法提取ID，使用索引
                }
                
                // 添加到数组
                obstacles_msg.obstacles.push_back(obstacle);
            }
        }
        
        // 发布障碍物数组
        if (!obstacles_msg.obstacles.empty()) {
            obstacle_pub_.publish(obstacles_msg);
        }
    }
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber gazebo_sub_;
    ros::Publisher obstacle_pub_;
    std::string obstacle_prefix_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_publisher");
    ros::NodeHandle nh;
    
    ObstaclePublisher obstacle_publisher(nh);
    
    ros::spin();
    
    return 0;
}
