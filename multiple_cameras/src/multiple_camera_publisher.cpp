
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "multiple_cameras/srv/modify_camera_settings.hpp" 
#include "multiple_cameras/msg/multiple_camera_status.hpp" 
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iterator>

typedef struct css {
    bool opened {true};
    bool enable_publishing {true};
    int height {720};
    int width {1280};
    int raw_height {720};
    int raw_width {1280};
    int fps {10};
    int retries {3};
    int dev_id {0};
    std::vector<long int> supported_fps;

} camera_settings_struct;

class MultipleCameraNode : public rclcpp::Node {
public:
    MultipleCameraNode() : Node("multiple_camera_node") {
        // Publishers
        _modify_settings_service = this->create_service<multiple_cameras::srv::ModifyCameraSettings>(declareAndReadStringParameter("modify_camera_settings_service", "multiple_camera/modify_camera_settings"), std::bind(&MultipleCameraNode::modifyCameraSettings, this, std::placeholders::_1, std::placeholders::_2));
        _camera_0_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(declareAndReadStringParameter("camera0_compressed_topic", "camera0/image/compressed"), 10);
        _camera_1_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(declareAndReadStringParameter("camera1_compressed_topic", "camera1/image/compressed"), 10);
        _camera_2_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(declareAndReadStringParameter("camera2_compressed_topic", "camera2/image/compressed"), 10);
        _camera_3_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(declareAndReadStringParameter("camera3_compressed_topic", "camera3/image/compressed"), 10);
        _status_msg_publisher = this->create_publisher<multiple_cameras::msg::MultipleCameraStatus>(declareAndReadStringParameter("multiple_camera_status_topic", "multiple_camera/status"), 10);

        _camera_0_settings.dev_id = declareAndReadIntParameter("camera0_dev_id", 0);
        _camera_1_settings.dev_id = declareAndReadIntParameter("camera1_dev_id", 1);
        _camera_2_settings.dev_id = declareAndReadIntParameter("camera2_dev_id", 2);
        _camera_3_settings.dev_id = declareAndReadIntParameter("camera3_dev_id", 3);

        _camera_0_settings.supported_fps = declareAndReadVectorIntParameter("camera0_supported_fps", {5, 10, 20, 30});
        _camera_1_settings.supported_fps = declareAndReadVectorIntParameter("camera1_supported_fps", {5, 10, 20, 30});
        _camera_2_settings.supported_fps = declareAndReadVectorIntParameter("camera2_supported_fps", {5, 10, 20, 30});
        _camera_3_settings.supported_fps = declareAndReadVectorIntParameter("camera3_supported_fps", {5, 10, 20, 30});

        // Initialize cameras
        _camera_0.open(_camera_0_settings.dev_id);
        if (!_camera_0.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera 0.");
            _camera_0_settings.opened = false;

        } else {
            _camera_0_settings.opened = true;
            updateCameraSettings(_camera_0, _camera_0_settings, 0);
            updateTimerSettings(0, false);
        }
               
        _camera_1.open(_camera_1_settings.dev_id);
        if (!_camera_1.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera 1.");
            _camera_1_settings.opened = false;

        } else {
            _camera_1_settings.opened = true;
            updateCameraSettings(_camera_1, _camera_1_settings, 1);
            updateTimerSettings(1, false);   
        }

        _camera_2.open(_camera_2_settings.dev_id);
        if (!_camera_2.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera 2.");
            _camera_2_settings.opened = false;

        } else {
            _camera_2_settings.opened = true;
            updateCameraSettings(_camera_2, _camera_2_settings, 2);
            updateTimerSettings(2, false); 
        }

        _camera_3.open(_camera_3_settings.dev_id);
        if (!_camera_3.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera 3.");
            _camera_3_settings.opened = false;
        } else {
            _camera_3_settings.opened = true;
            updateCameraSettings(_camera_3, _camera_3_settings, 3);    
            updateTimerSettings(3, false);  
        }

        _reconnection_timer = this->create_wall_timer(std::chrono::milliseconds(10000), std::bind(&MultipleCameraNode::checkCameraConnections, this));
        _status_message_timer = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&MultipleCameraNode::sendStatusMessage, this));
    }

private:
    rclcpp::Service<multiple_cameras::srv::ModifyCameraSettings>::SharedPtr _modify_settings_service;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr _camera_0_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr _camera_1_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr _camera_2_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr _camera_3_publisher;
    rclcpp::Publisher<multiple_cameras::msg::MultipleCameraStatus>::SharedPtr _status_msg_publisher;
    
    rclcpp::TimerBase::SharedPtr _camera_0_timer;
    rclcpp::TimerBase::SharedPtr _camera_1_timer;
    rclcpp::TimerBase::SharedPtr _camera_2_timer;
    rclcpp::TimerBase::SharedPtr _camera_3_timer;
    rclcpp::TimerBase::SharedPtr _reconnection_timer;
    rclcpp::TimerBase::SharedPtr _status_message_timer;

    cv::VideoCapture _camera_0;
    cv::VideoCapture _camera_1;
    cv::VideoCapture _camera_2;
    cv::VideoCapture _camera_3;

    camera_settings_struct _camera_0_settings;
    camera_settings_struct _camera_1_settings;
    camera_settings_struct _camera_2_settings;
    camera_settings_struct _camera_3_settings;

    std::string declareAndReadStringParameter(std::string param_name, std::string default_value) {
        this->declare_parameter(param_name, default_value);
        try {
            return this->get_parameter(param_name).get_parameter_value().get<std::string>();
        } catch (...) {
            return default_value;
        }
    }

    std::vector<long int> declareAndReadVectorIntParameter(std::string param_name, std::vector<long int> default_value) {
        rclcpp::Parameter param;

        this->declare_parameter(param_name, default_value);
        try {
            this->get_parameter(param_name, param);          
            return param.as_integer_array();

        } catch (...) {
            return default_value;
        }
    }

    int declareAndReadIntParameter(std::string param_name, int default_value) {
        this->declare_parameter(param_name, default_value);
        try {
            return this->get_parameter(param_name).get_parameter_value().get<int>();
        } catch (...) {
            return default_value;
        }
    }

    void sendStatusMessage() {
        multiple_cameras::msg::MultipleCameraStatus sts_msg;

        // Camera 0
        sts_msg.camera_0.opened = _camera_0_settings.opened;
        sts_msg.camera_0.enable_publishing = _camera_0_settings.enable_publishing;
        sts_msg.camera_0.height = _camera_0_settings.height;
        sts_msg.camera_0.width = _camera_0_settings.width;
        sts_msg.camera_0.fps = _camera_0_settings.fps;
        
        // Camera 1
        sts_msg.camera_1.opened = _camera_1_settings.opened;
        sts_msg.camera_1.enable_publishing = _camera_1_settings.enable_publishing;
        sts_msg.camera_1.height = _camera_1_settings.height;
        sts_msg.camera_1.width = _camera_1_settings.width;
        sts_msg.camera_1.fps = _camera_1_settings.fps;

        // Camera 2
        sts_msg.camera_2.opened = _camera_2_settings.opened;
        sts_msg.camera_2.enable_publishing = _camera_2_settings.enable_publishing;
        sts_msg.camera_2.height = _camera_2_settings.height;
        sts_msg.camera_2.width = _camera_2_settings.width;
        sts_msg.camera_2.fps = _camera_2_settings.fps;

        // Camera 3
        sts_msg.camera_3.opened = _camera_3_settings.opened;
        sts_msg.camera_3.enable_publishing = _camera_3_settings.enable_publishing;
        sts_msg.camera_3.height = _camera_3_settings.height;        
        sts_msg.camera_3.width = _camera_3_settings.width;        
        sts_msg.camera_3.fps = _camera_3_settings.fps;

        _status_msg_publisher->publish(sts_msg);
    }

    void updateTimerSettings(int timer_number, bool reset_timer) {
        switch (timer_number) {
            case 0:
                if (reset_timer) {
                    _camera_0_timer.reset();
                }
                _camera_0_timer = this->create_wall_timer(std::chrono::milliseconds(1000 / _camera_0_settings.fps), std::bind(&MultipleCameraNode::captureAndPublishImages0, this));        
                break;
            case 1:
                if (reset_timer) {
                    _camera_1_timer.reset();
                }
                _camera_1_timer = this->create_wall_timer(std::chrono::milliseconds(1000 / _camera_1_settings.fps), std::bind(&MultipleCameraNode::captureAndPublishImages1, this));        
                break;
            case 2:
                if (reset_timer) {
                    _camera_2_timer.reset();
                }
                _camera_2_timer = this->create_wall_timer(std::chrono::milliseconds(1000 / _camera_2_settings.fps), std::bind(&MultipleCameraNode::captureAndPublishImages2, this));        
                break;
            case 3:
                if (reset_timer) {
                    _camera_3_timer.reset();
                }
                _camera_3_timer = this->create_wall_timer(std::chrono::milliseconds(1000 / _camera_3_settings.fps), std::bind(&MultipleCameraNode::captureAndPublishImages3, this));        
                break;      
        }                          
    }

    void updateHardwareFps(cv::VideoCapture& cap, camera_settings_struct& cam_settings, int cam_number) {
        bool hardware_fps_set = false;
        int len_supported_fps = int(std::size(cam_settings.supported_fps));

        for (int k=0; k<len_supported_fps; k++) {
            if (cam_settings.supported_fps[k] >= cam_settings.fps) {
                cap.set(cv::CAP_PROP_FPS, float(cam_settings.supported_fps[k]));
                RCLCPP_INFO(this->get_logger(), "Camera %i hardware set to %li fps", cam_number, cam_settings.supported_fps[k]);
                hardware_fps_set = true;
                break;
            }
        }

        if (!hardware_fps_set) {
            cam_settings.fps = int(cam_settings.supported_fps[len_supported_fps-1]);
            cap.set(cv::CAP_PROP_FPS, float(cam_settings.supported_fps[len_supported_fps-1]));
            RCLCPP_INFO(this->get_logger(), "Camera %i reqested fps exceeds supported, set to %li fps", cam_number, cam_settings.supported_fps[len_supported_fps-1]);
        }
    }

    void updateCameraSettings(cv::VideoCapture& cap, camera_settings_struct& cam_settings, int cam_number) {
        cv::Mat frame;
        
        cam_settings.height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        cam_settings.width = cap.get(cv::CAP_PROP_FRAME_WIDTH);

        updateHardwareFps(cap, cam_settings, cam_number);
        
        cap >> frame;
        
        if (!frame.empty()) {
            cam_settings.raw_height = frame.size().height;
            cam_settings.raw_width = frame.size().width;
            cam_settings.height = (cam_settings.height != cam_settings.raw_height) ? cam_settings.raw_height : cam_settings.height;
            cam_settings.width = (cam_settings.width != cam_settings.raw_width) ? cam_settings.raw_width : cam_settings.width;
        }
    }

    void checkCameraConnections() {
        if (!_camera_0_settings.opened && _camera_0_settings.retries > 0) {
            _camera_0.open(_camera_0_settings.dev_id);

            if (!_camera_0.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera 0, retries left %i", _camera_0_settings.retries);
                _camera_0_settings.retries--;

            } else {
                _camera_0_settings.opened = true;
                updateCameraSettings(_camera_0, _camera_0_settings, 0);
                updateTimerSettings(0, true);
                _camera_0_settings.retries = 3;
            }
        }

        if (!_camera_1_settings.opened && _camera_1_settings.retries > 0) {
            _camera_1.open(_camera_1_settings.dev_id);

            if (!_camera_1.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera 1, retries left %i", _camera_1_settings.retries);
                _camera_1_settings.retries--;

            } else {
                _camera_1_settings.opened = true;
                updateCameraSettings(_camera_1, _camera_1_settings, 1);
                updateTimerSettings(1, true);
                _camera_1_settings.retries = 3;
            }
        }

        if (!_camera_2_settings.opened && _camera_2_settings.retries > 0) {
            _camera_2.open(_camera_2_settings.dev_id);

            if (!_camera_2.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera 2, retries left %i", _camera_2_settings.retries);
                _camera_2_settings.retries--;

            } else {
                _camera_2_settings.opened = true;
                updateCameraSettings(_camera_2, _camera_2_settings, 2);
                updateTimerSettings(2, true);
                _camera_2_settings.retries = 3;
            }
        }        

        if (!_camera_3_settings.opened && _camera_3_settings.retries > 0) {
            _camera_3.open(_camera_3_settings.dev_id);

            if (!_camera_3.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera 3, retries left %i", _camera_3_settings.retries);
                _camera_3_settings.retries--;

            } else {
                _camera_3_settings.opened = true;
                updateCameraSettings(_camera_3, _camera_3_settings, 3);
                updateTimerSettings(3, true);
                _camera_3_settings.retries = 3;
            }
        }
    }

    bool updateSettingsStruct(camera_settings_struct* camera_settings, const std::shared_ptr<multiple_cameras::srv::ModifyCameraSettings::Request> request, int camera_number) {
        bool update_fps = false;

        if (request->enable_publishing != camera_settings->enable_publishing) {
            camera_settings->enable_publishing = request->enable_publishing;
            RCLCPP_INFO(this->get_logger(), "Publishing camera %i, set to %s", camera_number, camera_settings->enable_publishing ? "true" : "false");
        }         

        if (request->height != camera_settings->height) {
            if (request->height <= 0) {
                RCLCPP_ERROR(this->get_logger(), "Requested height of %li for camera %i is dumb think harder next time", request->height, camera_number);
            
            } else { 
                if (request->height != camera_settings->raw_height) { 
                    RCLCPP_INFO(this->get_logger(), "Requested height of %li for camera %i differs from default cv::resize will be used", request->height, camera_number);
                }

                camera_settings->height = request->height;
                RCLCPP_INFO(this->get_logger(), "Changed camera %i, height to %li", camera_number, request->height);
            }
        }

        if (request->width != camera_settings->width) {
            if (request->width <= 0) {
                RCLCPP_ERROR(this->get_logger(), "Requested width of %li for camera %i is dumb think harder next time", request->width, camera_number);
            
            } else {
                if (request->width != camera_settings->raw_width) { 
                    RCLCPP_INFO(this->get_logger(), "Requested width of %li for camera %i differs from default cv::resize will be used", request->width, camera_number);
                }
            
                camera_settings->width = request->width;            
                RCLCPP_INFO(this->get_logger(), "Changed camera %i, width to %li", camera_number, request->width);
            }
        }

        if (request->fps != camera_settings->fps && request->fps > 0) {
            camera_settings->fps = request->fps;
            RCLCPP_INFO(this->get_logger(), "Changed camera %i, fps to %li", camera_number, request->fps);
            update_fps = true;
        }
        
        return update_fps;
    }

    void modifyCameraSettings(const std::shared_ptr<multiple_cameras::srv::ModifyCameraSettings::Request> request, std::shared_ptr<multiple_cameras::srv::ModifyCameraSettings::Response> response) {
        switch (request->camera_id) {
            case 0:
                if (updateSettingsStruct(&_camera_0_settings, request, 0)) {
                    updateHardwareFps(_camera_0, _camera_0_settings, 0);
                    updateTimerSettings(0, true);
                }
                response->success = true;
                break;

            case 1:
                if (updateSettingsStruct(&_camera_1_settings, request, 1)) {  
                    updateHardwareFps(_camera_1, _camera_1_settings, 1);             
                    updateTimerSettings(1, true);                
                }
                response->success = true;
                break;

            case 2:
                if (updateSettingsStruct(&_camera_2_settings, request, 2)) {
                    updateHardwareFps(_camera_2, _camera_2_settings, 2);
                    updateTimerSettings(2, true);                
                }
                response->success = true;
                break;

            case 3:
                if (updateSettingsStruct(&_camera_3_settings, request, 3)) {
                    updateHardwareFps(_camera_3, _camera_3_settings, 3);
                    updateTimerSettings(3, true);
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
        publishImageFromCamera(_camera_0, _camera_0_publisher, 0, _camera_0_settings);
    }

    void captureAndPublishImages1() {
        publishImageFromCamera(_camera_1, _camera_1_publisher, 1, _camera_1_settings);
    }

    void captureAndPublishImages2() {
        publishImageFromCamera(_camera_2, _camera_2_publisher, 2, _camera_2_settings);
    }

    void captureAndPublishImages3() {
        publishImageFromCamera(_camera_3, _camera_3_publisher, 3, _camera_3_settings);
    }

    void publishImageFromCamera(cv::VideoCapture& cap, const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr& pub, int camera_number, camera_settings_struct& cam_settings) {
        if (cam_settings.enable_publishing && cam_settings.opened) {
            sensor_msgs::msg::CompressedImage msg;
            cv::Mat frame;

            cap >> frame;

            if (frame.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to capture frame from camera %i, resetting", camera_number);
                cap.release();
                cam_settings.opened = false;
                cam_settings.retries = 3;
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