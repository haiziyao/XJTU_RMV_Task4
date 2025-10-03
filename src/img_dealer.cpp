#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rmv_task04/msg/object_array.hpp"  // 修改为当前包的消息路径
#include "rmv_task04/msg/object.hpp"       // 添加单个物体消息的引用
#include <vector>

class ImgDealer : public rclcpp::Node
{
public:
    ImgDealer() : Node("deal_img_node")
    {
        // 订阅图像话题（与相机节点发布的话题一致）
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&ImgDealer::image_callback, this, std::placeholders::_1)
        );

        // 发布物体识别结果（自定义话题）
        objects_publisher_ = this->create_publisher<rmv_task04::msg::ObjectArray>(  // 修改消息命名空间
            "/detected_objects", 10
        );

        RCLCPP_INFO(this->get_logger(), "deal_img_node 启动成功，开始处理图像...");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<rmv_task04::msg::ObjectArray>::SharedPtr objects_publisher_;  // 修改消息命名空间

    // 图像回调函数：处理图像并识别物体
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {   
        RCLCPP_INFO(this->get_logger(), 
            "宽: %d, 高: %d, 编码: %s, 步长: %d, 数据长度: %lu",
            msg->width, msg->height, msg->encoding.c_str(),
            msg->step, msg->data.size());

        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            cv::Mat frame = cv_ptr->image;

            // ✅ 显示原始图像（只用于测试通信）
            cv::imshow("Received Image", frame);
            cv::waitKey(1);

            // ✅ 构建一个空的 ObjectArray 消息，只是验证能发布
            rmv_task04::msg::ObjectArray objects_msg;
            objects_msg.header = msg->header;
            objects_publisher_->publish(objects_msg);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换错误: %s", e.what());
        }
        catch (cv::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV 错误: %s", e.what());
        }
    }


    
    std::vector<rmv_task04::msg::Object> detect_objects(cv::Mat& frame)  // 修改消息命名空间
    {
        std::vector<rmv_task04::msg::Object> objects;  // 修改消息命名空间

        // 1. 颜色阈值（检测红色）
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask);  // 红色低阈值
        cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask);  // 红色高阈值

        // 2. 形态学操作去除噪声
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

        // 3. 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 4. 筛选轮廓并生成物体信息
        for (auto& contour : contours)
        {
            // 过滤小面积轮廓（去除噪声）
            double area = cv::contourArea(contour);
            if (area < 500)  // 面积小于500像素的忽略
                continue;

            // 计算最小外接矩形
            cv::Rect bbox = cv::boundingRect(contour);

            // 构建物体消息
            rmv_task04::msg::Object obj;  // 修改消息命名空间
            obj.class_name = "red_object";  // 物体类别
            obj.x = bbox.x + bbox.width / 2.0;  // 中心x坐标
            obj.y = bbox.y + bbox.height / 2.0; // 中心y坐标
            obj.width = bbox.width;
            obj.height = bbox.height;
            obj.score = 0.9;  // 置信度（模拟）

            objects.push_back(obj);

            // 可视化：在图像上画框（可选，用于调试）
            cv::rectangle(frame, bbox, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, "red_object", cv::Point(bbox.x, bbox.y-10), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }

        // 显示处理后的图像（调试用）
        cv::imshow("Processed Image", frame);
        cv::waitKey(1);  // 必须添加，否则窗口无响应

        return objects;
    }
};

int main(int argc, char**argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImgDealer>();  // 修复类名错误（原DealImgNode改为ImgDealer）
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();  
    return 0;
}
