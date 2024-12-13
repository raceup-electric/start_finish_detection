#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "eufs_msgs/msg/cone_array_with_covariance.hpp"
#include "kvaser_reader_driver/msg/can_data.hpp"

class StartFinishDetectionNode : public rclcpp::Node {
public:
    StartFinishDetectionNode()
        : Node("start_finish_detection_node"),
          tracking_started_(false),
          exited_radius_(false) {
        // Initialize publisher for CanData messages
        finish_publisher_ = this->create_publisher<kvaser_reader_driver::msg::CanData>("/can_data_in", 10);

        // Initialize subscriber
        cone_subscriber_ = this->create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
            "/cone_pose", 10,
            std::bind(&StartFinishDetectionNode::processCones, this, std::placeholders::_1));
    }

private:
    void processCones(const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg) {
        // Check if there are any big orange cones
        if (msg->big_orange_cones.size() < 2) {
            return; // Not enough cones to calculate midpoint
        }

        // Calculate the middle point of the closest two big orange cones
        auto closest_middle_point = calculateMiddlePoint(msg->big_orange_cones[0].point, msg->big_orange_cones[1].point);
        double current_distance = calculateDistanceToPoint(closest_middle_point);

        // If we are not tracking, check if the center point is within the start radius
        if (!tracking_started_ && current_distance < START_RADIUS) {
            tracking_started_ = true;
            initial_middle_point_ = closest_middle_point;
            exited_radius_ = false; // Reset exit flag
        }

        // If we are tracking, check if we've exited the radius or if there are no big orange cones
        if (tracking_started_) {
            double exit_distance = calculateDistanceBetweenPoints(closest_middle_point, initial_middle_point_);
            if (exit_distance > EXIT_RADIUS || msg->big_orange_cones.empty()) {
                exited_radius_ = true;
            }

            // Trigger finish if we have exited the radius or no cones are detected
            if (exited_radius_ && current_distance < START_RADIUS && !msg->big_orange_cones.empty()) {
                publishFinishMessage();
                tracking_started_ = false; // Reset tracking
                exited_radius_ = false;
            }
        }
    }

    geometry_msgs::msg::Point calculateMiddlePoint(const geometry_msgs::msg::Point &point1, 
                                                   const geometry_msgs::msg::Point &point2) {
        geometry_msgs::msg::Point midpoint;
        midpoint.x = (point1.x + point2.x) / 2.0;
        midpoint.y = (point1.y + point2.y) / 2.0;
        midpoint.z = (point1.z + point2.z) / 2.0;
        return midpoint;
    }

    double calculateDistanceToPoint(const geometry_msgs::msg::Point &point) {
        return std::sqrt(point.x * point.x + point.y * point.y); // Assuming z = 0 (2D distance)
    }

    double calculateDistanceBetweenPoints(const geometry_msgs::msg::Point &point1, const geometry_msgs::msg::Point &point2) {
        return std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2));
    }

    void publishFinishMessage() {
        kvaser_reader_driver::msg::CanData can_message;

        // Set header
        can_message.header.stamp = this->get_clock()->now();

        // Set CAN message details
        can_message.id = 0x02; // BRAKE_RELE_ID
        can_message.length = 8; // Default length, can adjust based on msg_body
        can_message.flags = 0; // Flags as per your need (default to 0)
        
        // Set msg_body (could be a specific pattern or empty)
        std::fill(can_message.msg_body.begin(), can_message.msg_body.end(), 0); // Example empty message

        can_message.timestamp = static_cast<uint32_t>(rclcpp::Time(this->get_clock()->now()).seconds());

        // Publish the CAN data message
        finish_publisher_->publish(can_message);
    }

    // Publishers and subscribers
    rclcpp::Publisher<kvaser_reader_driver::msg::CanData>::SharedPtr finish_publisher_;
    rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr cone_subscriber_;

    // Tracking state
    bool tracking_started_ = false;
    bool exited_radius_;
    geometry_msgs::msg::Point initial_middle_point_;
    static constexpr double START_RADIUS = 4.0; // Threshold to trigger start
    static constexpr double EXIT_RADIUS = 5.0; // Threshold to confirm exit
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StartFinishDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
