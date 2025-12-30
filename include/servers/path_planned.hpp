#ifndef PATH_PLANNED_HPP
#define PATH_PLANNED_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <unordered_map>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>

class DijkstraPlanner : public rclcpp::Node
{
public:
    DijkstraPlanner();

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

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_ar_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_mr_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_fake_;

   
    void arCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void mrCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void fakeCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

   
    void compute();
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
