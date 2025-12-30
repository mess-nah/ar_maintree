#include "forest_nav_cpp/action/forestnavServer.hpp"

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

ForestNav::ForestNav(const rclcpp::NodeOptions & options)
:Node("forest_nav", options)
    
      
{
        state_ =FN::State::DECISION;
        sensor_mode_=FN::SensorMode::ODOM;
        sick_dist_=-1.0f;
        odom_dist_=0.0f;
        current_yaw_=0.0f;
        imu_yaw_ = 0.0f;
        move_target_=0.0f;
        rotate_target_=0.0f;
        sample_time_ms_=50;
        move_tolerance_m_=0.02f;
        yaw_tolerance_rad_=0.02f;
        roll_timeout_s_=5.0;
        rev_tolerance_rad_ = 0.02f;
        dist_tol_sick = 0.02f;
        dist_tol_odom =0.02f;
        current_climb_ = 500.0f;

        cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/control_signals", 10);
        

        sick_sub_ = this->create_subscription<std_msgs::msg::Float32>("/sick_data_received", 10,
        std::bind(&ForestNav::sickCallback, this, std::placeholders::_1)
        );
        odom_sub_ = this->create_subscription<std_msgs::msg::Float32>("/odom_data_received", 10,
        std::bind(&ForestNav::odomCallback, this, std::placeholders::_1)
        );

        prox_rear_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/prox_rear_received", 10, std::bind(&ForestNav::proxRearCallback, this, std::placeholders::_1));
        prox_front_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/prox_front_received", 10, std::bind(&ForestNav::proxFrontCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<std_msgs::msg::Float32>("/imu_data_received", 10,
        std::bind(&ForestNav::imuCallback, this, std::placeholders::_1)
        );    

        vel_sub_ = this->create_subscription<std_msgs::msg::Float32>("/vel_data_received", 10,
        std::bind(&ForestNav::velCallback, this, std::placeholders::_1
        ));


        control_signal.data.resize(6, 0.0f);
        planned_path_ = {2, 3, 6, 9, 12}; 
        path_idx_ = 0;
        


        sick_pid_.setTunings(0.08f, 0.001f, 0.004f);
        sick_pid_.setOutputLimits(-0.6f, 0.6f);
        sick_pid_.setSampleTime(50);
        sick_pid_.setMode(AUTOMATIC);
        sick_pid_.init();

        odom_pid_.setTunings(0.08f, 0.001f, 0.004f);
        odom_pid_.setOutputLimits(-0.6f, 0.6f);
        odom_pid_.setSampleTime(50);
        odom_pid_.setMode(AUTOMATIC);
        odom_pid_.init();

        yaw_pid_.setTunings(1.2f, 0.0f, 0.02f);
        yaw_pid_.setOutputLimits(-1.2f, 1.2f);
        yaw_pid_.setSampleTime(50);
        yaw_pid_.setMode(AUTOMATIC);
        yaw_pid_.init();


        this->declare_parameter<int>("sample_time_ms", static_cast<int>(sample_time_ms_));
        this->declare_parameter<double>("move_tolerance_m", move_tolerance_m_);
        this->declare_parameter<double>("yaw_tolerance_rad", yaw_tolerance_rad_);
        this->declare_parameter<double>("roll_timeout_s", roll_timeout_s_);
        this->declare_parameter<double>("rev_tolerance_rad", rev_tolerance_rad_);
        this->declare_parameter<double>("climb_target", climb_target_);
        this->declare_parameter<double>("dist_tol_sick", dist_tol_sick);
        this->declare_parameter<double>("dist_tol_odom", dist_tol_odom);
        this->declare_parameter<double>("current_climb", current_climb_);
        





        sample_time_ms_ = static_cast<unsigned int>(this->get_parameter("sample_time_ms").as_int());
        move_tolerance_m_ = this->get_parameter("move_tolerance_m").as_double();
        yaw_tolerance_rad_ = this->get_parameter("yaw_tolerance_rad").as_double();
        roll_timeout_s_ = this->get_parameter("roll_timeout_s").as_double();
        rev_tolerance_rad_ = this->get_parameter("rev_tolerance_rad").as_double();
        climb_target_= this->get_parameter("climb_target").as_double();
        dist_tol_sick= this->get_parameter("dist_tol_sick").as_double();
        dist_tol_odom= this->get_parameter("dist_tol_odom").as_double();
        current_climb_= this->get_parameter("current_climb").as_double();







        auto sample_time = std::chrono::milliseconds(sample_time_ms_);
        timer_ = this->create_wall_timer(sample_time, std::bind(&ForestNav::processState, this));

        RCLCPP_INFO(this->get_logger(), "ForestNav node started");
 }
ForestNav::~ForestNav() = default;

void ForestNav::sickCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    sick_dist_ = msg->data;
}
void ForestNav::imuCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    imu_yaw_ = msg->data;
}
void ForestNav::odomCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    
    odom_dist_ = msg->data;
}
void ForestNav::velCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    
    float vel = msg->data;
    // printf(vel)
}
void ForestNav::proxFrontCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    front_edge_detected = msg->data;
}

void ForestNav::proxRearCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    rear_edge_detected = msg->data;
}

void ForestNav::decideSensor()
{
    if (next_h > curr_h)
        sensor_mode_ = FN::SICK;
    else
        sensor_mode_ = FN::ODOM;
}

void ForestNav::nodeAnalysis()
{
    std::unordered_map<int,int> node_height = {
        {1,40},{2,20},{3,40},{4,60},
        {5,40},{6,20},{7,40},{8,60},
        {9,40},{10,20},{11,40},{12,20}
    };
    curr_h = node_height[curr_node];
    next_h = node_height[next_node];

    int diff = next_h - curr_h;
    if (diff == 20) 
        climb_ = FN::Climb::up;
    if (diff == -20) 
        climb_ = FN::Climb::down;
    
    if (next_node - curr_node == -1)
        rotate_ = FN::Rotate::neg_hpi;
    if (next_node - curr_node == 1)
        rotate_ = FN::Rotate::pos_hpi;
    else
        rotate_ = FN::Rotate::zero;

    
    
    
    
}

void ForestNav::processState()
{   

    for (int i=0; i < 6; i++){
        control_signal.data[i] = 0.0;
    }
    while (path_idx_ >= planned_path_.size() - 1)
    {
        
    
     
        switch (state_)
        {
            case FN::State::DECISION:
                curr_node = planned_path_[path_idx_];
                next_node = planned_path_[path_idx_ + 1];
                nodeAnalysis();
                state_ = FN::State::ROTATE;
                break;

            case FN::State::ROTATE:
                doRotatePID();
                break;

            case FN::State::MOVE:

                doMovePID();
                break; 

            case FN::State::CLIMB:
                doClimb();
                break;

            case FN::State::ROLL:
                doRoll();
                break;

            case FN::State::RETRACT:
                doRetract();
                break;

            case FN::State::GO:
                doGo();
                break;


            case FN::State::DONE:
    
                path_idx_++;
                state_ = FN::State::DECISION;
                break;

            default:
                stopCmd();
                break;
        }
    }
    publish_cmd();
}
void ForestNav::publish_cmd()
{
    cmd_pub_->publish(control_signal);
}

void ForestNav::stopCmd()
{
    for (int i = 0; i < 6; i ++){
        control_signal.data[i]=0.0;
    }
    printf("Robot in default state");
   
}

 
void ForestNav::doMovePID()
{
    
    decideSensor();
    float dist_err = 0.0; 
    float dist_tolerance = 0.0;
    if (sensor_mode_ == FN::SICK)
    {
        sick_pid_.setpoint = 50;
        sick_pid_.input = sick_dist_;
        sick_pid_.compute();
        control_signal.data[0]= sick_pid_.output;
        dist_err = 50 - sick_dist_;
        dist_tolerance = dist_tol_sick;
    }
    else 
    {
        odom_pid_.setpoint = 600;
        odom_pid_.input = odom_dist_;
        odom_pid_.compute();
        control_signal.data[0]= sick_pid_.output;
        dist_err = 600 - odom_dist_;
        dist_tolerance = dist_tol_odom;
    }

     if (std::fabs(dist_err) < dist_tolerance)
    {
        control_signal.data[0]= 0.0;
        state_ = FN::State::CLIMB; 
    }
}

void ForestNav::doRotatePID()
{
    yaw_pid_.input = current_yaw_;
    yaw_pid_.setpoint = rotate_target_;

    yaw_pid_.compute();
    control_signal.data[2] = yaw_pid_.output;

    float yaw_err = rotate_target_ - current_yaw_ ; 

    if (std::fabs(yaw_err) < yaw_tolerance_rad_)
    {
        control_signal.data[2] = 0.0;
        state_ = FN::State::MOVE;
    } 
}

void ForestNav::doClimb()
{
    
    if (climb_ = FN::Climb::up)
        control_signal.data[3] = 1.0;
    else if (climb_ = FN::Climb::down)
    {
        control_signal.data[3] = -1.0;
    }
    float rev_err = climb_target_ - current_climb_;

    if (std::fabs(rev_err) < rev_tolerance_rad_)
    {
        control_signal.data[3] = 0.0;
        state_ = FN::State::ROLL;
    } 
}


void ForestNav::doRoll()
{
    
    control_signal.data[4]= 1.0;
    if ((climb_ == FN::Climb::up && rear_edge_detected) || (climb_ == FN::Climb::down && front_edge_detected))
    {
        control_signal.data[4] = 0.0;
        state_ = FN::State::RETRACT;
    }
    
    
}
void ForestNav::doRetract()
{
    if (climb_ = FN::Climb::up)
        control_signal.data[3] = -1.0;
    else if (climb_ = FN::Climb::down)
    {
        control_signal.data[3] = 1.0;
    }
    float rev_err = climb_target_ - current_climb_;
    if (std::fabs(rev_err) < rev_tolerance_rad_)
    {
        control_signal.data[3] = 0.0;
        state_ = FN::State::GO;
    } 
    
}

void ForestNav::doGo()
{
    float dist_err = 0.0; 
    float dist_tolerance = 0.0;
    if (sensor_mode_ == FN::SICK)
    {
        odom_pid_.setpoint = 600;
        odom_pid_.input = odom_dist_;
        odom_pid_.compute();
        control_signal.data[0]= sick_pid_.output;
        dist_err = 600 - odom_dist_;
        dist_tolerance = dist_tol_odom;
        
    }
    else 
    {
        sick_pid_.setpoint = 50;
        sick_pid_.input = sick_dist_;
        sick_pid_.compute();
        control_signal.data[0]= sick_pid_.output;
        dist_err = 50 - sick_dist_;
        dist_tolerance = dist_tol_sick;
    }

     if (std::fabs(dist_err) < dist_tolerance)
    {
        control_signal.data[0]= 0.0;
        state_ = FN::State::DONE; 
    }
    
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForestNav>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
















    