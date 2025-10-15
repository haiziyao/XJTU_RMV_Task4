#ifndef AA7A170C_04C1_44F1_8BA5_B2453B4D8FC2
#define AA7A170C_04C1_44F1_8BA5_B2453B4D8FC2
#ifndef ARMOR_DETECTOR_H
#define ARMOR_DETECTOR_H

#include <cstddef>
#include <fstream>  
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <ostream>
#include <vector>
#include <tesseract/baseapi.h>
#include <onnxruntime_cxx_api.h>
#include <algorithm>

// 颜色定义
const int RED = 0;
const int BLUE = 1;

// 装甲板类型枚举
enum class ArmorType { SMALL, LARGE, INVALID };
const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

// 光源检测参数结构体
struct LightParams {
    double min_ratio;        // 光源宽高比最小值（width/height）
    double max_ratio;        // 光源宽高比最大值
    double max_angle;        // 光源最大倾斜角度（度）
    double min_fill_ratio;   // 光源最小填充率（内部像素占外接矩形比例）
};



// 装甲板匹配参数结构体
struct ArmorParams {
    double min_light_ratio;  // 两个光源长度比最小值（短/长）
    // 小型装甲板的光源中心距离范围（归一化到光源平均长度）
    double min_small_center_distance;
    double max_small_center_distance;
    // 大型装甲板的光源中心距离范围（归一化到光源平均长度）
    double min_large_center_distance;
    double max_large_center_distance;
    double max_angle;        // 光源中心连线的最大水平角度（度）
};

// 光源结构体（继承自cv::Rect以方便获取边界框）
struct Light : public cv::Rect {
    Light() = default;
    explicit Light(cv::Rect box, cv::Point2f top, cv::Point2f bottom, int area, float tilt_angle);

    int color;                // 光源颜色（RED或BLUE）
    cv::Point2f top, bottom;  // 光源的上下端点
    cv::Point2f center;       // 光源中心点
    double length;            // 光源长度（上下端点距离）
    double width;             // 光源宽度（面积/长度）
    float tilt_angle;         // 光源倾斜角度（度）
};

// 装甲板结构体
struct Armor {
    Armor() = default;
    Armor(const Light &l1, const Light &l2);

    Light left_light, right_light;  // 左右光源
    cv::Point2f center;             // 装甲板中心点
    ArmorType type;                 // 装甲板类型（小型/大型/无效）

    // 数字识别相关
    cv::Mat number_img;             // 提取的数字图像
    std::string number;             // 识别出的数字
    float confidence;               // 识别置信度
    std::string classfication_result; // 格式化的识别结果（含置信度）
};

// 数字分类器类（负责提取数字和识别）
class NumberClassifier {
public:
    NumberClassifier(
        const std::string &model_path, 
        const std::string &label_path, 
        const double threshold,
        const std::vector<std::string> &ignore_classes = {}
    );

    // 从装甲板中提取数字图像
    void extractNumbers(const cv::Mat &src, std::vector<Armor> &armors);

    // 识别数字并过滤无效结果
    void classify(std::vector<Armor> &armors);

    double threshold;  // 识别置信度阈值

private:
    cv::dnn::Net net_;                 // 神经网络模型（ONNX）
    std::vector<std::string> class_names_;  // 类别标签
    std::vector<std::string> ignore_classes_;  // 需忽略的类别
};

// 装甲板检测器类（核心检测逻辑）
class Detector {
public:
    // 构造函数：初始化检测参数
    Detector(
        const int &bin_thres, 
        const int &color, 
        const LightParams &l, 
        const ArmorParams &a
    );

    // 预处理图像（转为灰度图并二值化）
    cv::Mat preprocessImage(const cv::Mat &rgb_img);

    // 核心检测流程：返回检测到的装甲板
    std::vector<Armor> detect(const cv::Mat &input);

    // 从二值图中检测光源
    std::vector<Light> findLights(const cv::Mat &rgb_img, const cv::Mat &binary_img);

    // 匹配光源组成装甲板
    std::vector<Armor> matchLights(const std::vector<Light> &lights);

    // 绘制检测结果（光源、装甲板、数字）
    void drawResults(cv::Mat &img);

    // 获取所有提取的数字图像（垂直拼接）
    cv::Mat getAllNumbersImage();

    // 数字分类器（智能指针，需外部初始化）
    std::unique_ptr<NumberClassifier> classifier;

private:
    // 判断一个区域是否为有效光源
    bool isLight(const Light &light);

    // 判断两个光源是否能组成有效装甲板
    ArmorType isArmor(const Light &light_1, const Light &light_2);

    // 检查两个光源组成的区域是否包含其他光源（避免错误匹配）
    bool containLight(
        const Light &light_1, 
        const Light &light_2, 
        const std::vector<Light> &lights
    );

    const int binary_thres;    // 二值化阈值
    const int detect_color;    // 目标检测颜色（RED或BLUE）
    const LightParams l;       // 光源检测参数
    const ArmorParams a;       // 装甲板匹配参数
    cv::Mat binary_img;        // 二值化图像（用于调试）
    std::vector<Armor> armors_; // 检测到的装甲板
    std::vector<Light> lights_; // 检测到的光源
};

class PnPSolver
{
public:
  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients);

  // Get 3d position
  bool solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec);

  // Calculate the distance between armor center and image center
  float calculateDistanceToCenter(const cv::Point2f & image_point);

private:
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // Unit: mm
  static constexpr float SMALL_ARMOR_WIDTH = 132;
  static constexpr float SMALL_ARMOR_HEIGHT = 57;
  static constexpr float LARGE_ARMOR_WIDTH = 223;
  static constexpr float LARGE_ARMOR_HEIGHT = 57;

  // Four vertices of armor in 3d
  std::vector<cv::Point3f> small_armor_points_;
  std::vector<cv::Point3f> large_armor_points_;
};


int start();


#endif // ARMOR_DETECTOR_H


#endif /* AA7A170C_04C1_44F1_8BA5_B2453B4D8FC2 */
