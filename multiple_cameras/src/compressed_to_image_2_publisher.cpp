#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CompressedImageSubscriber : public rclcpp::Node {
public:
    CompressedImageSubscriber() : Node("camera_2_compressed_image_subscriber") {
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("camera2/image/compressed", 10, std::bind(&CompressedImageSubscriber::imageCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera2/image/raw", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        try {
            // Convert the decompressed image to ROS Image message
            cv::Mat image = cv::imdecode(cv::Mat(msg->data), 1); 
    
            sensor_msgs::msg::Image img_msg;
            cv_bridge::CvImage img_bridge = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, image);
            
            img_bridge.toImageMsg(img_msg); 

            // Publish the decompressed image
            publisher_->publish(img_msg);

            RCLCPP_INFO(this->get_logger(), "image: %i %i %i", image.size().height, image.size().width, image.channels());

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CompressedImageSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}