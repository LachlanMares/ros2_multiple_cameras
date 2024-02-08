
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "opencv2/opencv.hpp"

typedef struct css {
    bool opened {true};
    bool enable_publishing {false};
    int height {640};
    int width {480};
    int fps {20};
} camera_settings_struct;

class MultipleCameraNode : public rclcpp::Node {
public:
    MultipleCameraNode() : Node("multiple_camera_node") {

        // Set up service to modify parameters
        service_ = this->create_service<ModifySettings>("modify_camera_settings", std::bind(&CameraNode::ModifySettings, this, std::placeholders::_1, std::placeholders::_2));

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
            pub0_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera0/image/compressed", 10);
            cap0_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / cap0_settings.fps), std::bind(&CameraNode::captureAndPublishImages0, this));
        }

        if (cap1_settings.opened) {
            pub1_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera1/image/compressed", 10);
            cap1_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / cap1_settings.fps), std::bind(&CameraNode::captureAndPublishImages1, this));
        }

        if (cap2_settings.opened) {
            pub2_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera2/image/compressed", 10);
            cap2_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / cap2_settings.fps), std::bind(&CameraNode::captureAndPublishImages2, this));
        }

        if (cap3_settings.opened) {
            pub3_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera3/image/compressed", 10);
            cap3_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / cap3_settings.fps), std::bind(&CameraNode::captureAndPublishImages3, this));
        }
    }

private:
    rclcpp::Service<ModifySettings>::SharedPtr service_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub0_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub1_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub2_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub3_;
    
    rclcpp::TimerBase::SharedPtr cap0_timer_;
    rclcpp::TimerBase::SharedPtr cap1_timer_;
    rclcpp::TimerBase::SharedPtr cap2_timer_;
    rclcpp::TimerBase::SharedPtr cap3_timer_;

    cv::VideoCapture cap0_;
    cv::VideoCapture cap1_;
    cv::VideoCapture cap2_;
    cv::VideoCapture cap3_;

    camera_settings_struct cap0_settings;
    camera_settings_struct cap1_settings;
    camera_settings_struct cap2_settings;
    camera_settings_struct cap3_settings;

    // Modify settings service callback
    void ModifySettings(const std::shared_ptr<ModifySettings::Request> request, std::shared_ptr<ModifySettings::Response> response) {
        switch (request->camera_id) {
            case 0:
                cap0_settings.enable_publishing = request->enable_publishing;
                cap0_settings.height = request->height;
                cap0_settings.width = request->width;
                cap0_settings.fps = request->fps;
                cap0_timer_.cancel();
                cap0_timer_.reset();
                cap0_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / cap0_settings.fps), std::bind(&CameraNode::captureAndPublishImages0, this));
                response->success = true;
                break;

            case 1:
                cap1_settings.enable_publishing = request->enable_publishing;
                cap1_settings.height = request->height;
                cap1_settings.width = request->width;
                cap1_settings.fps = request->fps;
                cap1_timer_.cancel();
                cap1_timer_.reset();
                cap1_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / cap1_settings.fps), std::bind(&CameraNode::captureAndPublishImages1, this));
                response->success = true;
                break;

            case 2:
                cap2_settings.enable_publishing = request->enable_publishing;
                cap2_settings.height = request->height;
                cap2_settings.width = request->width;
                cap2_settings.fps = request->fps;
                cap2_timer_.cancel();
                cap2_timer_.reset();
                cap2_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / cap2_settings.fps), std::bind(&CameraNode::captureAndPublishImages2, this));
                response->success = true;
                break;

            case 3:
                cap3_settings.enable_publishing = request->enable_publishing;
                cap3_settings.height = request->height;
                cap3_settings.width = request->width;
                cap3_settings.fps = request->fps;
                cap3_timer_.cancel();
                cap3_timer_.reset();
                cap3_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / cap3_settings.fps), std::bind(&CameraNode::captureAndPublishImages3, this));
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
        if (cam_settings->enable_publishing && camera_settings->opened) {
            sensor_msgs::msg::CompressedImage msg;
            cv::Mat frame;

            cap >> frame;

            if (frame.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to capture frame from camera %i.", cap_num);
                return;
            }

            if (frame.size().height != cam_settings->height || frame.size().width != cam_settings->width) {
                cv::Mat resized_frame;
                cv::resize(frame, resized_frame, cv::Size(_cam_settings->width, cam_settings->height), 0, 0, cv::INTER_CUBIC);
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