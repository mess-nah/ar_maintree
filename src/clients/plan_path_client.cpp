#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ar_maintree/action/plan_path.hpp"
#include "ar_maintree/include/servers/plan_path_action.hpp"

using PlanPath = ar_maintree::action::PlanPath;
using GoalHandlePlanPath =
    rclcpp_actions::ClientGoalHandle<PlanPath>;

class PlanPathClient : public rclcpp::Node
{
public:
    PlanPathClient() : Node("plan_path_client")
    {
        client_= rclcpp_action::create_client<PlanPath> (
            this,
            "plan_path");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&PlanPathClient::sendGoal, this));
    }
private:
    rclcpp_action::Client<PlanPath>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void sendGoal()
    {
        timer_->cancel();

        if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Action server not available after waiting");
            
            return;
        }

        auto goal_msg = PlanPath::Goal();
        goal_msg.start_node = 2;
        goal_msg.goal_nodes = {10, 12};

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options =
            rclcpp_action::Client<PlanPath>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            std::bind(&PlanPathClient::goalResponseCallback, this, std::placeholders::_1);

        send_goal_options.feedback_callback =
            std::bind(&PlanPathClient::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

        send_goal_options.result_callback =
            std::bind(&PlanPathClient::resultCallback, this, std::placeholders::_1);

        client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goalResponseCallback(
        std::shared_future<GoalHandlePlanPath::SharedPtr> future)
    {
        
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedbackCallback(
        GoalHandlePlanPath::SharedPtr,
        const std::shared_ptr<const PlanPath::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback: %s", feedback->status.c_str());
    }

    void resultCallback(const GoalHandlePlanPath::WrappedResult& result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }

        RCLCPP_INFO(get_logger(), "Resulting path:");
        for (int n: result.result->path) {
            RCLCPP_INFO(get_logger(), "%d", n);
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlanPathClient>());
    rclcpp::shutdown();
    return 0;
}