/*
Author:
    Lachlan Mares, lachlan.mares@gmail.com

License:
    GPL-3.0

Description:

*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CompressedImageSubscriber : public rclcpp::Node {
public:
    CompressedImageSubscriber() : Node("multiple_camera_compressed_image_subscriber") {
        camera0_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(declareAndReadStringParameter("camera0_compressed_topic", "camera0/image/compressed"), 10, std::bind(&CompressedImageSubscriber::imageCallback0, this, std::placeholders::_1));
        camera0_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(declareAndReadStringParameter("camera0_topic", "camera0/image/decompressed"), 10);
        
        camera1_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(declareAndReadStringParameter("camera1_compressed_topic", "camera1/image/compressed"), 10, std::bind(&CompressedImageSubscriber::imageCallback1, this, std::placeholders::_1));
        camera1_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(declareAndReadStringParameter("camera1_topic", "camera1/image/decompressed"), 10);
        
        camera2_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(declareAndReadStringParameter("camera2_compressed_topic", "camera2/image/compressed"), 10, std::bind(&CompressedImageSubscriber::imageCallback2, this, std::placeholders::_1));
        camera2_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(declareAndReadStringParameter("camera2_topic", "camera2/image/decompressed"), 10);
        
        camera3_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(declareAndReadStringParameter("camera3_compressed_topic", "camera3/image/compressed"), 10, std::bind(&CompressedImageSubscriber::imageCallback3, this, std::placeholders::_1));
        camera3_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(declareAndReadStringParameter("camera3_topic", "camera3/image/decompressed"), 10);
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera0_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera1_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera2_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera3_subscriber_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera0_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera1_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera2_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera3_publisher_;

    sensor_msgs::msg::Image compressedImageToImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        // Convert the decompressed image to ROS Image message
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), 1); 

        sensor_msgs::msg::Image img_msg;
        cv_bridge::CvImage img_bridge = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, image);
        
        img_bridge.toImageMsg(img_msg); 

        return img_msg;
    }

    void imageCallback0(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        try {
            camera0_publisher_->publish(compressedImageToImage(msg));

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Camera 0 CV Bridge exception: %s", e.what());
        }
    }

    void imageCallback1(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        try {
            camera1_publisher_->publish(compressedImageToImage(msg));
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Camera 1 CV Bridge exception: %s", e.what());
        }
    }
    
    void imageCallback2(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        try {
            camera2_publisher_->publish(compressedImageToImage(msg));
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Camera 2 CV Bridge exception: %s", e.what());
        }
    }
    
    void imageCallback3(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        try {
            camera3_publisher_->publish(compressedImageToImage(msg));
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Camera 3 CV Bridge exception: %s", e.what());
        }
    }

    std::string declareAndReadStringParameter(std::string param_name, std::string default_value) {
        this->declare_parameter(param_name, default_value);
        try {
            return this->get_parameter(param_name).get_parameter_value().get<std::string>();
        } catch (...) {
            return default_value;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CompressedImageSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}