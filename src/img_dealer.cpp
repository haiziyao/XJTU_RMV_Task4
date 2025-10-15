#include "rclcpp/rclcpp.hpp"
#include "rmv_task04/msg/detail/object__struct.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rmv_task04/msg/object_array.hpp"  
#include "rmv_task04/msg/object.hpp"       
#include <vector>
#include "arm_detector.h"

cv::Mat input_image, show_image;

class ImgDealer : public rclcpp::Node
{
public:
    ImgDealer() : Node("deal_img_node"),
                  detector(binary_thres, detect_color, light_params, armor_params),
                  pnp_solver(camera_matrix, dist_coeffs)
    {
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&ImgDealer::image_callback, this, std::placeholders::_1)
        );

        objects_publisher_ = this->create_publisher<rmv_task04::msg::ObjectArray>( 
            "/detected_objects", 10
        );
        
        light_params.min_ratio = 0.1;        // 光源宽高比最小值
        light_params.max_ratio = 0.5;        // 光源宽高比最大值
        light_params.max_angle = 30.0;       // 光源最大倾斜角度（度）
        light_params.min_fill_ratio = 0.6;   // 光源最小填充率

        armor_params.min_light_ratio = 0.8;  // 两光源长度比最小值
        armor_params.min_small_center_distance = 1.0;  // 小型装甲板中心距下限
        armor_params.max_small_center_distance = 3.0;  // 小型装甲板中心距上限
        armor_params.min_large_center_distance = 3.0;  // 大型装甲板中心距下限
        armor_params.max_large_center_distance = 6.0;  // 大型装甲板中心距上限
        armor_params.max_angle = 10.0;       // 光源连线最大水平角度
        
        // 检查模型文件是否存在
        std::ifstream model_check(model_path);
        if (!model_check.good()) {  
            RCLCPP_INFO(this->get_logger(), "错误：模型文件不存在！路径：%s",model_path.c_str());
        }

        // 检查标签文件是否存在
        std::ifstream label_check(label_path);
        if (!label_check.good()) {
            RCLCPP_INFO(this->get_logger(), "错误：标签文件不存在！路径：%s",label_path.c_str());
        }

        // 初始化数字分类器
        detector.classifier = std::make_unique<NumberClassifier>(
            model_path, label_path, conf_threshold, ignore_classes
        );

        RCLCPP_INFO(this->get_logger(), "deal_img_node 启动成功，开始处理图像...");
    }

private:
    
    Detector detector;
    PnPSolver pnp_solver;
    const int binary_thres = 120;        // 二值化阈值
    const int detect_color = BLUE;        // 检测颜色（RED=0 或 BLUE=1）
    const std::string model_path = "model/mlp.onnx";  // 数字识别模型路径
    const std::string label_path = "model/label.txt"; // 标签文件路径
    const double conf_threshold = 0.5;   // 识别置信度阈值
    const std::vector<std::string> ignore_classes = {};  // 忽略类别

    std::array<double, 9> camera_matrix = {
        800.0, 0.0, 320.0,    // fx, 0, cx
        0.0, 800.0, 240.0,    // 0, fy, cy
        0.0, 0.0, 1.0         // 0, 0, 1
    };
    std::vector<double> dist_coeffs = {0.0, 0.0, 0.0, 0.0, 0.0};  // 畸变系数

    LightParams light_params;
    ArmorParams armor_params;
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
            
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
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


    
    std::vector<rmv_task04::msg::Object> detect_objects(cv::Mat& input_image)  
    {
        std::vector<rmv_task04::msg::Object> objects; 
        if (input_image.empty()) {
                RCLCPP_ERROR(this->get_logger(), "无法获取图像帧！");
            }
        cv::cvtColor(input_image, input_image, cv::COLOR_BGR2RGB);

        // 执行装甲板检测
        std::vector<Armor> detected_armors = detector.detect(input_image);

        // 对每个检测到的装甲板求解PnP
        for (auto& armor : detected_armors) {
            cv::Mat rvec, tvec;
            if (pnp_solver.solvePnP(armor, rvec, tvec)) {
                // 计算距离（平移向量的x分量，单位：米）
                float distance = tvec.at<double>(0);
                
                // 格式化显示信息（距离和坐标）
                std::stringstream ss;
                ss << armor.number << " " 
                    << std::fixed << std::setprecision(2) 
                    << distance << "m";
                armor.classfication_result = ss.str();
            }
        }

        // 准备显示图像（转回BGR格式）
        cv::cvtColor(input_image, show_image, cv::COLOR_RGB2BGR);
        
        // 绘制检测结果
        detector.drawResults(show_image);

        // 显示结果窗口
        cv::imshow("装甲板实时检测", show_image);
        char key = cv::waitKey(1);


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
