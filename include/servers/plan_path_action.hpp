#ifndef PLAN_PATH_ACTION_HPP
#define PLAN_PATH_ACTION_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ar_maintree/action/plan_path.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>

#include <unordered_map>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>

class DijkstraPlannerAction : public rclcpp::Node
{
public:
    DijkstraPlannerAction();

private:
    
    const int W_AR    = 20;
    const int W_MR    = 50;
    const int W_FAKE  = 999;
    const int W_NORMAL = 1;

    
    std::unordered_map<int, std::vector<std::pair<int,int>>> graph_;
    std::unordered_map<int, std::vector<std::pair<int,int>>> base_graph_;

  
    std::vector<int> ar_nodes_;
    std::vector<int> mr_nodes_;
    std::vector<int> fake_nodes_;

    bool ar_received_{false};
    bool mr_received_{false};
    bool fake_received_{false};

    int start_node_;
    std::vector<int> goal_nodes_;

    rclcpp_action::Server<ar_maintree::action::PlanPath>::SharedPtr action_server_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_ar_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_mr_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_fake_;

   
    void arCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void mrCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void fakeCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

   
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const ar_maintree::action::PlanPath::Goal> goal
    );

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ar_maintree::action::PlanPath>> goal_handle
    );

    void handleAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ar_maintree::action::PlanPath>> goal_handle
    );

    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ar_maintree::action::PlanPath>> goal_handle
    );

    void applyWeights();

    void dijkstra(
        int start,
        std::unordered_map<int, int>& dist,
        std::unordered_map<int, int>& prev
    );

    std::vector<int> buildPath(
        const std::unordered_map<int, int>& prev,
        int goal
    );
};

#endif  
