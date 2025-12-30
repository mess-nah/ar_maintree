#ifndef forestnavServer_hpp
#define forestnavServer_hpp


#include <functional>
#include <memory>
#include <thread>
#include <atomic>
#include <cmath>
#include <mutex>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "robotlib/controllers/pid.hpp"
// #include "sensor_msgs/msg/imu.hpp"



namespace FN
{
    enum  Rotate
    {
        zero,
        pos_hpi,
        neg_hpi, 
    };
    
    enum  Climb
    {
        up,
        down,
    };
    
    enum SensorMode
    {
        SICK,
        ODOM,
    };
    
    enum  State{
        DECISION,
        ROTATE,
        MOVE,
        CLIMB,
        ROLL,
        RETRACT,
        GO,
        DONE
    };

    
    
}     


class ForestNav : public rclcpp::Node
{
public:
        
        FN::State state_;
        FN::Rotate rotate_;
        FN::Climb climb_;
        FN::SensorMode sensor_mode_;

        
        std::vector<int> planned_path_;
        size_t path_idx_=0;

        
        float sick_dist_=0.0;   
        float odom_dist_=0.0;   
        float current_yaw_=0.0; 
        float imu_yaw_ = 0.0;

        int curr_node = 2;
        int next_node = 2;
        int curr_h = 20;
        int next_h =20; 

        bool rear_edge_detected= false;
        bool front_edge_detected= false;

        float move_target_ =0;    
        float rotate_target_= 0;  
        double climb_target_ = 720;
       
        PID sick_pid_;
        PID odom_pid_;
        PID yaw_pid_;

        unsigned int sample_time_ms_=50;
        double move_tolerance_m_= 0.02;
        double yaw_tolerance_rad_=0.02;
        double roll_timeout_s_=5.0;
        double rev_tolerance_rad_ = 0.02;
        double dist_tol_sick = 0.02;
        double dist_tol_odom =0.02;
        double current_climb_ = 500;
      

        std_msgs::msg::Float32MultiArray  control_signal;

        explicit ForestNav(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        ~ForestNav();

        
        void nodeAnalysis();
        void decideSensor();
        void processState();
        void doMovePID();
        void doRotatePID();
        void doClimb();
        void doRoll();
        void doRetract();
        void doGo();
        void stopCmd();
        
       
                
        void sickCallback(const std_msgs::msg::Float32::SharedPtr msg);
        void odomCallback(const std_msgs::msg::Float32::SharedPtr msg);
        void proxRearCallback(const std_msgs::msg::Bool::SharedPtr msg);
        void proxFrontCallback(const std_msgs::msg::Bool::SharedPtr msg);
        void imuCallback(const std_msgs::msg::Float32::SharedPtr msg);
        void velCallback(const std_msgs::msg::Float32::SharedPtr msg);
        void publish_cmd();

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub_;
        
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sick_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr odom_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr prox_rear_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr prox_front_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr imu_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_sub_;
        
        rclcpp::TimerBase::SharedPtr timer_;

       
        
    };

    #endif
