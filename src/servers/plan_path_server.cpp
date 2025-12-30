
#include <algorithm>
#include <limits>
#include <queue>
#include <functional>
#include <thread>
#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>
#include "ar_maintree/action/plan_path.hpp"
#include "ar_maintree/include/servers/plan_path_action.hpp"
#include "rclcpp_actions/rclcpp_actions.hpp"

using PlanPath = ar_maintree::action::PlanPath;
using GoalHandlePlanPath =
    rclcpp_actions::ServerGoalHandle<PlanPath>;

using std::placeholders::_1;
using std::placeholders::_2;


class DijkstraPlannerAction : public rclcpp::Node
{
public:
    DijkstraPlannerAction() : Node("dijkstra_planner_action")
    {
    base_graph_ = {
        {2,  {{5,1},{1,1},{3,1}}},
        {1,  {{4,1},{2,1}}},
        {3,  {{6,1},{2,1}}},
        {4,  {{5,1},{7,1}}},
        {6,  {{5,1},{9,1}}},
        {9,  {{12,1},{8,1}}},
        {8,  {{11,1},{9,1},{7,1}}},
        {11, {{12,1},{10,1}}},
        {7,  {{10,1},{8,1}}},
        {5,  {{6,1},{4,1},{8,1}}},
        {10, {{11,1}}},
        {12, {{11,1}}}
    };

    graph_ = base_graph_;

    start_node_ = 2;
    goal_nodes_ = {12, 10};

    

    sub_ar_ = create_subscription<std_msgs::msg::Int32MultiArray>(
        "/ar_nodes", 10,
        std::bind(&DijkstraPlannerAction::arCallback, this, _1));

    sub_mr_ = create_subscription<std_msgs::msg::Int32MultiArray>(
        "/mr_nodes", 10,
        std::bind(&DijkstraPlannerAction::mrCallback, this, _1));

    sub_fake_ = create_subscription<std_msgs::msg::Int32MultiArray>(
        "/fake_nodes", 10,
        std::bind(&DijkstraPlannerAction::fakeCallback, this, _1));

    action_server_ =
        rclcpp_actions::create_server<PlanPath>(
            this,
            "plan_path",
            std::bind(&DijkstraPlannerAction::handleGoal, this, _1, _2),
            std::bind(&DijkstraPlannerAction::handleCancel, this, _1),
            std::bind(&DijkstraPlannerAction::handleAccepted, this, _1));

    RCLCPP_INFO(get_logger(), "Dijkstra Planner Action Server started");
}
}
private:

    void arCallback(
        const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        ar_nodes_ = msg->data;
        ar_received_ = true;
        
    }

    void mrCallback(
        const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        mr_nodes_ = msg->data;
        mr_received_ = true;
        
    }

    void fakeCallback(
        const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        fake_nodes_ = msg->data;
        fake_received_ = true;
        
    }

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const PlanPath::Goal> goal)
    {
         if (!ar_received_ || !mr_received_ || !fake_received_) {
            RCLCPP_WARN(get_logger(), "CV data not ready, rejecting goal");
            return rclcpp_action::GoalResponse::REJECT;
    }
        start_node_ = goal->start_node;
        goal_nodes_ = goal->goal_nodes;

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }


    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandlePlanPath> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(
        const std::shared_ptr<GoalHandlePlanPath> goal_handle)
    {
        std::thread(
            std::bind(&DijkstraPlannerAction::execute, this, goal_handle))
            .detach();
    }

    void execute(
        const std::shared_ptr<GoalHandlePlanPath> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Executing goal");

        auto feedback = std::make_shared<PlanPath::Feedback>();
        auto result = std::make_shared<PlanPath::Result>();

        feedback->status = "Applying weights based on CV data";
        goal_handle->publish_feedback(feedback);

        applyWeights();

        feedback->status = "Planning path using Dijkstra's algorithm";
        goal_handle->publish_feedback(feedback);

        std::unordered_map<int, int> dist, prev;
        dijkstra(start_node_, dist, prev);  

        int best_goal = goal_nodes_[0];
        for (int g : goal_nodes_) 
            if (dist[g] < dist[best_goal]) 
                best_goal = g;

        result->path = buildPath(best_goal, prev);
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Path found");


            
        }
       

    void applyWeights()
    {
        graph_ = base_graph_;
        for(auto& [node, edges] : graph_)
        {
            for(auto& edge : edges)
            {
                int nbr = edge.first;

                if (isIn(nbr, ar_nodes_))      edge.second = 50;
                else if (isIn(nbr, mr_nodes_)) edge.second = 20;
                else if (isIn(nbr, fake_nodes_)) edge.second = 100;
                else                            edge.second = 1;
            }
        }
    }

    void dijkstra(int start,
                  std::unordered_map<int, int>& dist,
                  std::unordered_map<int, int>& prev)
    {
        const int INF = std::numeric_limits<int>::max();

        for (const auto& [node, _] : graph_)
        {
            dist[node] = INF;
            prev[node] = -1;
        }

        dist[start] = 0;
        using QNode = std::pair<int, int>;
        std::priority_queue<QNode, std::vector<QNode>, std::greater<QNode>> pq;

        pq.push({start, 0});

        while (!pq.empty())
        {
            auto [d, u]  = pq.top();
            pq.pop();

            if (d > dist[u]) continue;
            for (auto[v, w] : graph_[u])
            {
               if (dist[u] + w < dist[v]) {
                    dist[v] = dist[u] + w;
                    prev[v] = u;
                    pq.push({dist[v], v});
            }
        }
    }
}

    std::vector<int> buildPath(int goal,
                               const std::unordered_map<int, int>& prev)
    {
        std::vector<int> path;
        for (int at = goal; at != -1; at = prev.at(at))
            path.insert(path.begin(), at);
        return path;
    }

    rclcpp_action::Server<PlanPath>::SharedPtr action_server_;
    std::unordered_map<int, std::vector<std::pair<int, int>>> base_graph

    std::vector<int> ar_nodes_;
    std::vector<int> mr_nodes_;
    std::vector<int> fake_nodes_;
    std::vector<int> goal_nodes_;
    std::unordered_map<int, std::vector<std::pair<int, int>>> graph_;
    bool ar_received_ = false;
    bool mr_received_ = false;
    bool fake_received_ = false;
    int start_node_;

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_ar_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_mr_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_fake_;

    int main(int argc, char** argv)
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<DijkstraPlannerAction>());
        rclcpp::shutdown();
        return 0;
    }