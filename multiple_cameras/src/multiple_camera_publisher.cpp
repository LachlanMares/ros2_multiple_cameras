
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "multiple_cameras/srv/modify_camera_settings.hpp" 
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef struct css {
    bool opened {true};
    bool enable_publishing {true};
    int height {640};
    int width {480};
    int fps {30};
    int retries {3};
} camera_settings_struct;

class MultipleCameraNode : public rclcpp::Node {
public:
    MultipleCameraNode() : Node("multiple_camera_node") {

        service_ = this->create_service<multiple_cameras::srv::ModifyCameraSettings>("modify_camera_settings", std::bind(&MultipleCameraNode::modifyCameraSettings, this, std::placeholders::_1, std::placeholders::_2));
        pub0_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera0/image/compressed", 10);
        pub1_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera1/image/compressed", 10);
        pub2_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera2/image/compressed", 10);
        pub3_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera3/image/compressed", 10);

        // Initialize cameras
        cap0_.open(0);
        if (!cap0_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera 0.");
            cap0_settings.opened = false;
        }
               
        cap1_.open(1);
        if (!cap1_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera 1.");
            cap1_settings.opened = false;
        }

        cap2_.open(2);
        if (!cap2_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera 2.");
            cap2_settings.opened = false;
        }

        cap3_.open(3);
        if (!cap3_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera 3.");
            cap3_settings.opened = false;
        }

        // Set up timer to capture and publish images
        if (cap0_settings.opened) {
            cap0_settings.height = cap0_.get(cv::CAP_PROP_FRAME_HEIGHT);
            cap0_settings.width = cap0_.get(cv::CAP_PROP_FRAME_WIDTH);
            cap0_.set(cv::CAP_PROP_FPS, float(cap0_settings.fps));
            cap0_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / cap0_settings.fps), std::bind(&MultipleCameraNode::captureAndPublishImages0, this));
        }

        if (cap1_settings.opened) {
            cap1_settings.height = cap1_.get(cv::CAP_PROP_FRAME_HEIGHT);
            cap1_settings.width = cap1_.get(cv::CAP_PROP_FRAME_WIDTH);
            cap1_.set(cv::CAP_PROP_FPS, float(cap1_settings.fps));
            cap1_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / cap1_settings.fps), std::bind(&MultipleCameraNode::captureAndPublishImages1, this));
        }

        if (cap2_settings.opened) {
            cap2_settings.height = cap2_.get(cv::CAP_PROP_FRAME_HEIGHT);
            cap2_settings.width = cap2_.get(cv::CAP_PROP_FRAME_WIDTH);
            cap2_.set(cv::CAP_PROP_FPS, float(cap2_settings.fps));            
            cap2_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / cap2_settings.fps), std::bind(&MultipleCameraNode::captureAndPublishImages2, this));
        }

        if (cap3_settings.opened) {
            cap3_settings.height = cap3_.get(cv::CAP_PROP_FRAME_HEIGHT);
            cap3_settings.width = cap3_.get(cv::CAP_PROP_FRAME_WIDTH);
            cap3_.set(cv::CAP_PROP_FPS, float(cap3_settings.fps));
            cap3_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / cap3_settings.fps), std::bind(&MultipleCameraNode::captureAndPublishImages3, this));
        }

        reconnection_timer_ = this->create_wall_timer(std::chrono::milliseconds(10000), std::bind(&MultipleCameraNode::checkCameraConnections, this));
    }

private:
    rclcpp::Service<multiple_cameras::srv::ModifyCameraSettings>::SharedPtr service_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub0_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub1_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub2_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub3_;
    
    rclcpp::TimerBase::SharedPtr cap0_timer_;
    rclcpp::TimerBase::SharedPtr cap1_timer_;
    rclcpp::TimerBase::SharedPtr cap2_timer_;
    rclcpp::TimerBase::SharedPtr cap3_timer_;
    rclcpp::TimerBase::SharedPtr reconnection_timer_;

    cv::VideoCapture cap0_;
    cv::VideoCapture cap1_;
    cv::VideoCapture cap2_;
    cv::VideoCapture cap3_;

    camera_settings_struct cap0_settings;
    camera_settings_struct cap1_settings;
    camera_settings_struct cap2_settings;
    camera_settings_struct cap3_settings;

    void checkCameraConnections() {
        if (!cap0_settings.opened && cap0_settings.retries > 0) {
            cap0_.open(0);

            if (!cap0_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera 0, retries left %i", cap0_settings.retries);
                cap0_settings.retries--;
            } else {
                cap0_settings.opened = true;
                cap0_settings.retries = 3;
            }
        }

        if (!cap1_settings.opened && cap1_settings.retries > 0) {
            cap1_.open(1);

            if (!cap1_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera 1, retries left %i", cap1_settings.retries);
                cap1_settings.retries--;
            } else {
                cap1_settings.opened = true;
                cap1_settings.retries = 3;
            }
        }

        if (!cap2_settings.opened && cap2_settings.retries > 0) {
            cap2_.open(2);

            if (!cap2_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera 2, retries left %i", cap2_settings.retries);
                cap2_settings.retries--;
            } else {
                cap2_settings.opened = true;
                cap2_settings.retries = 3;
            }
        }        

        if (!cap3_settings.opened && cap3_settings.retries > 0) {
            cap3_.open(3);

            if (!cap3_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera 3, retries left %i", cap3_settings.retries);
                cap3_settings.retries--;
            } else {
                cap3_settings.opened = true;
                cap3_settings.retries = 3;
            }
        }
    }

    bool updateSettingsStruct(camera_settings_struct* cap_settings, const std::shared_ptr<multiple_cameras::srv::ModifyCameraSettings::Request> request, int cap_num) {
        bool update_fps = false;

        if (request->enable_publishing != cap_settings->enable_publishing) {
            cap_settings->enable_publishing = request->enable_publishing;
            RCLCPP_INFO(this->get_logger(), "Publishing camera %i, set to %s", cap_num, cap_settings->enable_publishing ? "true" : "false");
        }         

        if (request->height != cap_settings->height && request->height > 0) {
            cap_settings->height = request->height;
            RCLCPP_INFO(this->get_logger(), "Changed camera %i, height to %li", cap_num, request->height);
        }

        if (request->width != cap_settings->width && request->width > 0) {
            cap_settings->width = request->width;
            RCLCPP_INFO(this->get_logger(), "Changed camera %i, width to %li", cap_num, request->width);
        }

        if (request->fps != cap_settings->fps && request->fps > 0) {
            cap_settings->fps = request->fps;
            RCLCPP_INFO(this->get_logger(), "Changed camera %i, fps to %li", cap_num, request->fps);
            update_fps = true;
        }
        
        return update_fps;
    }

    void modifyCameraSettings(const std::shared_ptr<multiple_cameras::srv::ModifyCameraSettings::Request> request, std::shared_ptr<multiple_cameras::srv::ModifyCameraSettings::Response> response) {
        switch (request->camera_id) {
            case 0:
                if (updateSettingsStruct(&cap0_settings, request, 0)) {
                    cap0_timer_.reset();
                    cap0_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / cap0_settings.fps), std::bind(&MultipleCameraNode::captureAndPublishImages0, this));
                }
                response->success = true;
                break;

            case 1:
                if (updateSettingsStruct(&cap1_settings, request, 1)) {
                    cap1_timer_.reset();
                    cap1_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / cap1_settings.fps), std::bind(&MultipleCameraNode::captureAndPublishImages1, this));
                }
                response->success = true;
                break;

            case 2:
                if (updateSettingsStruct(&cap2_settings, request, 2)) {
                    cap2_timer_.reset();
                    cap2_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / cap2_settings.fps), std::bind(&MultipleCameraNode::captureAndPublishImages2, this));
                }
                response->success = true;
                break;

            case 3:
                if (updateSettingsStruct(&cap3_settings, request, 3)) {
                    cap3_timer_.reset();
                    cap3_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / cap3_settings.fps), std::bind(&MultipleCameraNode::captureAndPublishImages3, this));
                }
                response->success = true;
                break;

            default:
                response->success = false;
                break;   
       }   
    }

    // Capture and publish images
    void captureAndPublishImages0() {
        publishImageFromCamera(cap0_, pub0_, 0, cap0_settings);
    }

    void captureAndPublishImages1() {
        publishImageFromCamera(cap1_, pub1_, 1, cap1_settings);
    }

    void captureAndPublishImages2() {
        publishImageFromCamera(cap2_, pub2_, 2, cap2_settings);
    }

    void captureAndPublishImages3() {
        publishImageFromCamera(cap3_, pub3_, 3, cap3_settings);
    }

    void publishImageFromCamera(cv::VideoCapture& cap, const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr& pub, int cap_num, camera_settings_struct& cam_settings) {
        if (cam_settings.enable_publishing && cam_settings.opened) {
            sensor_msgs::msg::CompressedImage msg;
            cv::Mat frame;

            cap >> frame;

            if (frame.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to capture frame from camera %i.", cap_num);
                return;
            }

            if (frame.size().height != cam_settings.height || frame.size().width != cam_settings.width) {
                cv::Mat resized_frame;
                cv::resize(frame, resized_frame, cv::Size(cam_settings.width, cam_settings.height), 0, 0, cv::INTER_CUBIC);
                cv::imencode(".jpg", resized_frame, msg.data);

            } else {
                cv::imencode(".jpg", frame, msg.data);
            }

            pub->publish(msg);
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MultipleCameraNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}