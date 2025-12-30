#include "dijkstra_planner.hpp"

using std::placeholders::_1;

DijkstraPlanner::DijkstraPlanner()
: Node("dijkstra_planner")
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

    pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
        "/planned_path", 10);

    sub_ar_ = create_subscription<std_msgs::msg::Int32MultiArray>(
        "/ar_nodes", 10,
        std::bind(&DijkstraPlanner::arCallback, this, _1));

    sub_mr_ = create_subscription<std_msgs::msg::Int32MultiArray>(
        "/mr_nodes", 10,
        std::bind(&DijkstraPlanner::mrCallback, this, _1));

    sub_fake_ = create_subscription<std_msgs::msg::Int32MultiArray>(
        "/fake_nodes", 10,
        std::bind(&DijkstraPlanner::fakeCallback, this, _1));
}



void DijkstraPlanner::arCallback(
    const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    ar_nodes_ = msg->data;
    ar_received_ = true;
    compute();
}

void DijkstraPlanner::mrCallback(
    const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    mr_nodes_ = msg->data;
    mr_received_ = true;
    compute();
}

void DijkstraPlanner::fakeCallback(
    const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    fake_nodes_ = msg->data;
    fake_received_ = true;
    compute();
}



void DijkstraPlanner::compute()
{
    if (!(ar_received_ && mr_received_ && fake_received_))
        return;

    RCLCPP_INFO(get_logger(), "Received full CV data");

    applyWeights();

    std::unordered_map<int, int> dist;
    std::unordered_map<int, int> prev;

    dijkstra(start_node_, dist, prev);

    int best_goal = goal_nodes_[0];
    for (int g : goal_nodes_) {
        if (dist[g] < dist[best_goal])
            best_goal = g;
    }

    auto path = buildPath(prev, best_goal);

    std_msgs::msg::Int32MultiArray msg;
    msg.data = path;
    pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "Published final path");
}

void DijkstraPlanner::applyWeights()
{
    graph_ = base_graph_;

    auto isIn = [](int x, const std::vector<int>& v) {
        return std::find(v.begin(), v.end(), x) != v.end();
    };

    for (auto& [node, edges] : graph_) {
        for (auto& edge : edges) {
            int nbr = edge.first;

            if (isIn(nbr, ar_nodes_))
                edge.second = W_AR;
            else if (isIn(nbr, mr_nodes_))
                edge.second = W_MR;
            else if (isIn(nbr, fake_nodes_))
                edge.second = W_FAKE;
            else
                edge.second = W_NORMAL;
        }
    }
}



void DijkstraPlanner::dijkstra(
    int start,
    std::unordered_map<int, int>& dist,
    std::unordered_map<int, int>& prev)
{
    const int INF = std::numeric_limits<int>::max();

    for (const auto& [node, _] : graph_) {
        dist[node] = INF;
        prev[node] = -1;
    }

    dist[start] = 0;

    using QNode = std::pair<int,int>; // (dist, node)
    std::priority_queue<QNode, std::vector<QNode>, std::greater<>> pq;
    pq.push({0, start});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();

        if (d > dist[u]) continue;

        for (auto [v, w] : graph_[u]) {
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                prev[v] = u;
                pq.push({dist[v], v});
            }
        }
    }
}

std::vector<int> DijkstraPlanner::buildPath(
    const std::unordered_map<int, int>& prev,
    int goal)
{
    std::vector<int> path;
    for (int at = goal; at != -1; at = prev.at(at))
        path.insert(path.begin(), at);
    return path;
}
