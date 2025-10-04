#include "rclcpp/init_options.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include <chrono>
#include <memory>
#include "std_msgs/msg/string.hpp"
#include "MvCameraControl.h"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

class ImgPublisher: public rclcpp::Node
{
public: 
    ImgPublisher(std::string name):Node(name){
        RCLCPP_INFO(this->get_logger(),"我是%s",name.c_str());
        declare_camera_parameters();
        register_parameter_callback();
        if (!init_camera()){
            RCLCPP_ERROR(this->get_logger(), "相机初始化失败，节点启动失败");
            return;
        }
        if (!sync_param_to_camera()){
            RCLCPP_WARN(this->get_logger(), "启动参数同步失败，使用相机默认值");
        }
        is_running_ = true;
        capture_thread_ = std::thread(&ImgPublisher::capture_loop, this);
        RCLCPP_INFO(this->get_logger(), "节点启动完成");
    }
    ~ImgPublisher(){
        is_running_ = false;
        if (capture_thread_.joinable()){
            capture_thread_.join();
        }
        stop_camera();
        RCLCPP_INFO(this->get_logger(), "节点已关闭");
    }
private:
    std::atomic<bool> need_reconnect_{false};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    void* m_handle_ =  nullptr;
    int n_buf_size_ = 0 ;
    unsigned char* p_frame_buf_ = nullptr;
    std::thread capture_thread_;
    std::atomic<bool> is_running_{false};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr 
        on_parameters_set_callback_handle_;
    
    bool init_camera()
    {
        unsigned int nTLayerType = MV_GIGE_DEVICE | MV_USB_DEVICE;
        MV_CC_DEVICE_INFO_LIST m_stDevList;
        int nRet = MV_CC_EnumDevices(nTLayerType, &m_stDevList);
        if (MV_OK != nRet){
            RCLCPP_ERROR(this->get_logger(), "枚举设备失败，错误码: 0x%x", nRet);
            return false;
        }

        if (m_stDevList.nDeviceNum == 0){
            RCLCPP_ERROR(this->get_logger(), "未找到相机设备");
            return false;
        }
        int nDeviceIndex = 0;
        MV_CC_DEVICE_INFO m_stDevInfo;
        memcpy(&m_stDevInfo, m_stDevList.pDeviceInfo[nDeviceIndex], sizeof(MV_CC_DEVICE_INFO));
        
        nRet = MV_CC_CreateHandle(&m_handle_, &m_stDevInfo);
        if (MV_OK != nRet){
            RCLCPP_ERROR(this->get_logger(), "创建句柄失败，错误码: 0x%x", nRet);
            return false;
        }
        nRet = MV_CC_OpenDevice(m_handle_);
        if (MV_OK != nRet){
            RCLCPP_ERROR(this->get_logger(), "打开设备失败，错误码: 0x%x", nRet);
            MV_CC_DestroyHandle(m_handle_);
            return false;
        }
        nRet = MV_CC_StartGrabbing(m_handle_);
        if (MV_OK != nRet){
            RCLCPP_ERROR(this->get_logger(), "开始取流失败，错误码: 0x%x", nRet);
            MV_CC_CloseDevice(m_handle_);
            MV_CC_DestroyHandle(m_handle_);
            return false;
        }

        MVCC_INTVALUE stIntvalue,stWidth,stHeight;
        nRet = MV_CC_GetIntValue(m_handle_, "PayloadSize", &stIntvalue);
        if (MV_OK != nRet){
            RCLCPP_ERROR(this->get_logger(), "获取PayloadSize失败，错误码: 0x%x", nRet);
            stop_camera();
            return false;
        }
        nRet = MV_CC_GetIntValue(m_handle_, "Width", &stWidth);
        if (MV_OK != nRet) {
            RCLCPP_ERROR(this->get_logger(), "获取宽度失败，错误码: 0x%x", nRet);
            stop_camera();
            return false;
        }
        nRet = MV_CC_GetIntValue(m_handle_, "Height", &stHeight);
        if (MV_OK != nRet) {
            RCLCPP_ERROR(this->get_logger(), "获取高度失败，错误码: 0x%x", nRet);
            stop_camera();
            return false;
        }
        RCLCPP_INFO(this->get_logger(),"PayloadSize:%d stWidth: %d  stHeight:%d",stIntvalue.nCurValue,stWidth.nCurValue,stHeight.nCurValue);
        n_buf_size_ = (stIntvalue.nCurValue>(stWidth.nCurValue*stHeight.nCurValue*3+4096))?stIntvalue.nCurValue:(stWidth.nCurValue*stHeight.nCurValue*3+4096);
        //n_buf_size_ = 4665600;
        p_frame_buf_ = (unsigned char*)malloc(n_buf_size_);
        if (!p_frame_buf_){
            RCLCPP_ERROR(this->get_logger(), "内存分配失败");
            stop_camera();
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "相机初始化成功");
        return true;
    }

    void stop_camera()
    {
        if (m_handle_){
            MV_CC_StopGrabbing(m_handle_);
            MV_CC_CloseDevice(m_handle_);
            MV_CC_DestroyHandle(m_handle_);
            m_handle_ = nullptr;
        }
        
        if (p_frame_buf_){
            free(p_frame_buf_);
            p_frame_buf_ = nullptr;
        }
    }

    void capture_loop()
    {
        MV_FRAME_OUT_INFO_EX stInfo;
        memset(&stInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
        unsigned int frame_count = 0;
        int nRet;
        while (is_running_ && rclcpp::ok()){
            nRet = MV_CC_GetOneFrameTimeout(m_handle_, p_frame_buf_, n_buf_size_, &stInfo, 300);
            if (MV_OK == nRet)
            {
                frame_count++;
                if (frame_count % 20 == 0)  // 每20帧打印一次日志
                {
                    RCLCPP_INFO(this->get_logger(), "已采集 %d 帧图像", frame_count);
                }
                publish_image(p_frame_buf_, stInfo);
                RCLCPP_INFO(this->get_logger(),"此张图片大小为:%d",stInfo.nFrameLen);
            }
            else
            {
    
                
                RCLCPP_WARN(this->get_logger(), "获取图像失败：错误码=0x%x", 
                       nRet);
            }
        }
    }

    void publish_image(unsigned char* data, MV_FRAME_OUT_INFO_EX& info)
    {
        cv::Mat img;
        switch (info.enPixelType)
        {
            case PixelType_Gvsp_RGB8_Packed:
                img = cv::Mat(info.nHeight, info.nWidth, CV_8UC3, data);
                cv::cvtColor(img, img, cv::COLOR_RGB2BGR);  // 转换为OpenCV的BGR格式
                break;
            case PixelType_Gvsp_Mono8:
                img = cv::Mat(info.nHeight, info.nWidth, CV_8UC1, data);
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "不支持的像素格式: %ld", info.enPixelType);
                return;
        }
        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(),
            (info.enPixelType == PixelType_Gvsp_RGB8_Packed) ? "bgr8" : "mono8",
            img
        ).toImageMsg();
        
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "camera_link";
        image_publisher_->publish(*msg);
    }
    void declare_camera_parameters()
    {
        this->declare_parameter("camera_ip", "");  // 默认空（自动枚举），可传192.168.1.100
        this->declare_parameter("camera_serial", "");  // 默认空，可传相机序列号
        this->declare_parameter("image_topic", "/image_raw");  // 默认话题名
        this->declare_parameter("exposure_time", 1000.0);  // 默认1000μs
        this->declare_parameter("gain", 1.0);              // 默认1.0（无增益）
        this->declare_parameter("frame_rate", 20.0);       // 默认20fps
        this->declare_parameter("pixel_format", "rgb8");  // 默认灰度，可选rgb8
    }
    bool sync_param_to_camera()
    {
        if (m_handle_ == nullptr){
            RCLCPP_ERROR(this->get_logger(), "相机句柄为空，无法同步参数");
            return false;
        }
        double exposure_time = this->get_parameter("exposure_time").as_double();
        double gain = this->get_parameter("gain").as_double();
        double frame_rate = this->get_parameter("frame_rate").as_double();
        std::string pixel_format = this->get_parameter("pixel_format").as_string();
        std::string image_topic = this->get_parameter("image_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "初始参数：曝光=%.1fμs，增益=%.1f，帧率=%.1ffps，格式=%s，话题=%s",
                   exposure_time, gain, frame_rate, pixel_format.c_str(), image_topic.c_str());
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic, 10);
        if (!set_exposure_time(exposure_time)) return false;
        if (!set_gain(gain)) return false;
        if (!set_frame_rate(frame_rate)) return false;
        if (!set_pixel_format(pixel_format)) return false; 
        return true;
    }
    void register_parameter_callback(){
        on_parameters_set_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ImgPublisher::on_parameter_changed, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "参数动态回调注册完成");
    }

    rcl_interfaces::msg::SetParametersResult on_parameter_changed(
        const std::vector<rclcpp::Parameter> &parameters){
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "参数修改成功";

        for (const auto &param : parameters) {
            if (param.get_name() == "exposure_time") {
                if (!set_exposure_time(param.as_double())) {
                    result.successful = false;
                    result.reason = "曝光时间修改失败";
                    break;
                }
            }
            else if (param.get_name() == "gain") {
                if (!set_gain(param.as_double())) {
                    result.successful = false;
                    result.reason = "增益修改失败";
                    break;
                }
            }
            else if (param.get_name() == "frame_rate") {
                if (!set_frame_rate(param.as_double())) {
                    result.successful = false;
                    result.reason = "帧率修改失败";
                    break;
                }
            }
            else if (param.get_name() == "pixel_format") {
                if (!set_pixel_format(param.as_string())) {
                    result.successful = false;
                    result.reason = "像素格式修改失败";
                    break;
                }
            }
            else if (param.get_name() == "image_topic") {
                std::string new_topic = param.as_string();
                if (image_publisher_ && image_publisher_->get_topic_name() == new_topic) {
                    RCLCPP_INFO(this->get_logger(), "图像话题未变更：%s", new_topic.c_str());
                    continue;
                }
                image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(new_topic, 10);
                RCLCPP_INFO(this->get_logger(), "图像话题已更新为：%s", new_topic.c_str());
            }
            else {
                result.successful = false;
                result.reason = "不支持的参数：" + param.get_name();
                break;
            }
        }

        return result;


    }
    bool set_gain(double gain) {
        if (gain < 1.0 || gain > 10.0) {
            RCLCPP_ERROR(this->get_logger(), "增益超出范围（1.0~10.0）");
            return false;
        }
        if (m_handle_ == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "相机未初始化，无法设置增益");
            return false;
        }
        int nRet = MV_CC_SetFloatValue(m_handle_, "Gain", gain);
        if (MV_OK != nRet) {
            RCLCPP_ERROR(this->get_logger(), "设置增益失败，错误码：0x%x", nRet);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "增益已更新为：%.1f", gain);
        return true;
    }

    
    bool set_frame_rate(double fps) {
        if (fps < 1.0 || fps > 60.0) {
            RCLCPP_ERROR(this->get_logger(), "帧率超出范围（1.0~60.0fps）");
            return false;
        }
        if (m_handle_ == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "相机未初始化，无法设置帧率");
            return false;
        }

      
        MV_CC_StopGrabbing(m_handle_);
        int nRet = MV_CC_SetFloatValue(m_handle_, "AcquisitionFrameRate", fps);
        if (MV_OK != nRet) {
            MV_CC_StartGrabbing(m_handle_);  
            RCLCPP_ERROR(this->get_logger(), "设置帧率失败，错误码：0x%x", nRet);
            return false;
        }
        MV_CC_StartGrabbing(m_handle_);  
        RCLCPP_INFO(this->get_logger(), "帧率已更新为：%.1ffps", fps);
        return true;
    }

    // 私有方法：设置像素格式（需停止取流）
    bool set_pixel_format(const std::string& format) {
        unsigned int sdk_type;
        if (format == "mono8") {
            sdk_type = PixelType_Gvsp_Mono8;
        } else if (format == "rgb8") {
            sdk_type = PixelType_Gvsp_RGB8_Packed;
        } else {
            RCLCPP_ERROR(this->get_logger(), "不支持的格式：%s（仅支持mono8/rgb8）", format.c_str());
            return false;
        }

        if (m_handle_ == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "相机未初始化，无法设置像素格式");
            return false;
        }

        
        MV_CC_StopGrabbing(m_handle_);
        int nRet = MV_CC_SetEnumValue(m_handle_, "PixelFormat", sdk_type);
        if (MV_OK != nRet) {
            MV_CC_StartGrabbing(m_handle_); 
            RCLCPP_ERROR(this->get_logger(), "设置像素格式失败，错误码：0x%x", nRet);
            return false;
        }
        MV_CC_StartGrabbing(m_handle_);  
        RCLCPP_INFO(this->get_logger(), "像素格式已更新为：%s", format.c_str());
        return true;
    }

    bool set_exposure_time(double exposure) {
        if (exposure < 10.0 || exposure > 1000000.0) {
            RCLCPP_ERROR(this->get_logger(), "曝光时间超出范围（10~1000000μs）");
            return false;
        }
        if (m_handle_ == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "相机未初始化，无法设置曝光时间");
            return false;
        }
        int nRet = MV_CC_SetFloatValue(m_handle_, "ExposureTime", exposure);
        if (MV_OK != nRet) {
            RCLCPP_ERROR(this->get_logger(), "设置曝光时间失败，错误码：0x%x", nRet);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "曝光时间已更新为：%.1fμs", exposure);
        return true;
    }
    

    
};

int main(int argc,char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImgPublisher>("img_publisher_");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}