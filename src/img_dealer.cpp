#include "rclcpp/rclcpp.hpp"
#include "rmv_task04/msg/detail/object__struct.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rmv_task04/msg/object_array.hpp"  
#include "rmv_task04/msg/object.hpp"       
#include <vector>

class ImgDealer : public rclcpp::Node
{
public:
    ImgDealer() : Node("deal_img_node")
    {
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&ImgDealer::image_callback, this, std::placeholders::_1)
        );

        objects_publisher_ = this->create_publisher<rmv_task04::msg::ObjectArray>( 
            "/detected_objects", 10
        );
        cv::namedWindow("Trackbars", 400);
        cv::createTrackbar("Hue Min", "Trackbars", &hmin, 179);
        cv::createTrackbar("Hue Max", "Trackbars", &hmax, 179);
        cv::createTrackbar("Sat Min", "Trackbars", &smin, 255);
        cv::createTrackbar("Sat Max", "Trackbars", &smax, 255);
        cv::createTrackbar("Val Min", "Trackbars", &vmin, 255);
        cv::createTrackbar("Val Max", "Trackbars", &vmax, 255);
        RCLCPP_INFO(this->get_logger(), "deal_img_node 启动成功，开始处理图像...");
    }

private:
    int hmin = 62,smin=0,vmin=255;
    int hmax =105,smax=59,vmax=255;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<rmv_task04::msg::ObjectArray>::SharedPtr objects_publisher_;  

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
            std::vector<rmv_task04::msg::Object> objects = detect_objects(frame);
            rmv_task04::msg::ObjectArray objects_msg ;
            objects_msg.header = msg->header;
            objects_msg.objects = objects ;
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


    
    std::vector<rmv_task04::msg::Object> detect_objects(cv::Mat& img)  
    {
        std::vector<rmv_task04::msg::Object> objects; 

        cv::Scalar lower(hmin,smin,vmin);
        cv::Scalar upper(hmax,smax,vmax);

        cv::Mat img_gaussblur,img_hsv,img_mask,img_morph;
        cv::GaussianBlur(img, img_gaussblur, cv::Size(13,13), 7);
        cv::cvtColor(img_gaussblur, img_hsv, cv::COLOR_BGR2HSV);
        cv::inRange(img_hsv, lower, upper, img_mask);
        cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(7,7));
        morphologyEx(img_mask, img_morph, cv::MORPH_CLOSE, kernel);
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        std::vector<cv::Rect> rects;
        findContours(img_morph,contours,hierarchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
        for (auto& contour : contours){
            double area = cv::contourArea(contour);
            if (area < 500)  
                continue;
            cv::Rect bbox = cv::boundingRect(contour);

            rmv_task04::msg::Object obj;  // 修改消息命名空间
            obj.class_name = "red_object";  // 物体类别
            obj.x = bbox.x + bbox.width / 2.0;  // 中心x坐标
            obj.y = bbox.y + bbox.height / 2.0; // 中心y坐标
            obj.width = bbox.width;
            obj.height = bbox.height;
            obj.score = 0.9;  // 置信度（模拟）
            objects.push_back(obj);
            cv::rectangle(img, bbox, cv::Scalar(0, 255, 0), 2);
            cv::putText(img, "red_object", cv::Point(bbox.x, bbox.y-10), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }

        cv::imshow("img_raw Image", img);
        cv::imshow("img_morph",img_morph);
        cv::waitKey(1);  

        return objects;
    }
};

int main(int argc, char**argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImgDealer>();  
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();  
    return 0;
}
