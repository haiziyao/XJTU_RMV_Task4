
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
#include "arm_detector.h"


cv::Mat Detector::preprocessImage(const cv::Mat & rgb_img)
{
  cv::Mat gray_img;
  cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);

  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, binary_thres, 255, cv::THRESH_BINARY);

  return binary_img;
}


std::vector<Armor> Detector::detect(const cv::Mat & input)
{
  binary_img = preprocessImage(input);
  lights_ = findLights(input, binary_img);
  armors_ = matchLights(lights_);

  if (!armors_.empty()) {
    classifier->extractNumbers(input, armors_);
    classifier->classify(armors_);
  }

  return armors_;
}

std::vector<Light> Detector::findLights(const cv::Mat & rgb_img, const cv::Mat & binary_img)
{
  using std::vector;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  vector<Light> lights;
  //this->debug_lights.data.clear();

  for (const auto & contour : contours) {
    if (contour.size() < 5) continue;

    auto b_rect = cv::boundingRect(contour);
    auto r_rect = cv::minAreaRect(contour);
    cv::Mat mask = cv::Mat::zeros(b_rect.size(), CV_8UC1);
    std::vector<cv::Point> mask_contour;
    for (const auto & p : contour) {
      mask_contour.emplace_back(p - cv::Point(b_rect.x, b_rect.y));
    }
    cv::fillPoly(mask, {mask_contour}, 255);
    std::vector<cv::Point> points;
    cv::findNonZero(mask, points);
    // points / rotated rect area
    bool is_fill_rotated_rect =
      points.size() / (r_rect.size.width * r_rect.size.height) > l.min_fill_ratio;
    cv::Vec4f return_param;
    cv::fitLine(points, return_param, cv::DIST_L2, 0, 0.01, 0.01);
    cv::Point2f top, bottom;
    double angle_k;
    if (int(return_param[0] * 100) == 100 || int(return_param[1] * 100) == 0) {
      top = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y);
      bottom = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y + b_rect.height);
      angle_k = 0;
    } else {
      auto k = return_param[1] / return_param[0];
      auto b = (return_param[3] + b_rect.y) - k * (return_param[2] + b_rect.x);
      top = cv::Point2f((b_rect.y - b) / k, b_rect.y);
      bottom = cv::Point2f((b_rect.y + b_rect.height - b) / k, b_rect.y + b_rect.height);
      angle_k = std::atan(k) / CV_PI * 180 - 90;
      if (angle_k > 90) {
        angle_k = 180 - angle_k;
      }
    }
    auto light = Light(b_rect, top, bottom, points.size(), angle_k);

    if (isLight(light) && is_fill_rotated_rect) {
      auto rect = light;
      if (  // Avoid assertion failed
        0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rgb_img.cols && 0 <= rect.y &&
        0 <= rect.height && rect.y + rect.height <= rgb_img.rows) {
        int sum_r = 0, sum_b = 0;
        auto roi = rgb_img(rect);
        // Iterate through the ROI
        for (int i = 0; i < roi.rows; i++) {
          for (int j = 0; j < roi.cols; j++) {
            if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
              // if point is inside contour
              sum_r += roi.at<cv::Vec3b>(i, j)[0];
              sum_b += roi.at<cv::Vec3b>(i, j)[2];
            }
          }
        }
        // Sum of red pixels > sum of blue pixels ?
        light.color = sum_r > sum_b ? RED : BLUE;
        lights.emplace_back(light);
      }
    }
  }

  return lights;
}




bool Detector::isLight(const Light & light)
{
  // The ratio of light (short side / long side)
  float ratio = light.width / light.length;
  bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

  bool angle_ok = light.tilt_angle < l.max_angle;

  bool is_light = ratio_ok && angle_ok;

  // Fill in debug information
  // auto_aim_interfaces::msg::DebugLight light_data;
  // light_data.center_x = light.center.x;
  // light_data.ratio = ratio;
  // light_data.angle = light.tilt_angle;
  // light_data.is_light = is_light;
  // this->debug_lights.data.emplace_back(light_data);

  return is_light;
}



std::vector<Armor> Detector::matchLights(const std::vector<Light> & lights)
{
  std::vector<Armor> armors;
  //this->debug_armors.data.clear();

  // Loop all the pairing of lights\

  //其实我觉得这里得优化一下，颜色是友方的灯条就应该在扔进lights之前判断一下，一直在这里会浪费很多资源的(hzy)
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      if (light_1->color != detect_color || light_2->color != detect_color) continue;

      if (containLight(*light_1, *light_2, lights)) {
        continue;
      }

      auto type = isArmor(*light_1, *light_2);
      if (type != ArmorType::INVALID) {
        auto armor = Armor(*light_1, *light_2);
        armor.type = type;
        armors.emplace_back(armor);
      }
    }
  }

  return armors;
}


// Check if there is another light in the boundingRect formed by the 2 lights
bool Detector::containLight(
  const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
{
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);

  for (const auto & test_light : lights) {
    if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

    if (
      bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
      bounding_rect.contains(test_light.center)) {
      return true;
    }
  }

  return false;
}

ArmorType Detector::isArmor(const Light & light_1, const Light & light_2)
{
  // Ratio of the length of 2 lights (short side / long side)
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                             : light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

  // Distance between the center of 2 lights (unit : light length)
  float avg_light_length = (light_1.length + light_2.length) / 2;
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok = (a.min_small_center_distance <= center_distance &&
                             center_distance < a.max_small_center_distance) ||
                            (a.min_large_center_distance <= center_distance &&
                             center_distance < a.max_large_center_distance);

  // Angle of light center connection
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  bool angle_ok = angle < a.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // Judge armor type
  ArmorType type;
  if (is_armor) {
    type = center_distance > a.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
  } else {
    type = ArmorType::INVALID;
  }

  // Fill in debug information
  // auto_aim_interfaces::msg::DebugArmor armor_data;
  // armor_data.type = ARMOR_TYPE_STR[static_cast<int>(type)];
  // armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
  // armor_data.light_ratio = light_length_ratio;
  // armor_data.center_distance = center_distance;
  // armor_data.angle = angle;
  // this->debug_armors.data.emplace_back(armor_data);

  return type;
}

cv::Mat Detector::getAllNumbersImage()
{
  if (armors_.empty()) {
    return cv::Mat(cv::Size(20, 28), CV_8UC1);
  } else {
    std::vector<cv::Mat> number_imgs;
    number_imgs.reserve(armors_.size());
    for (auto & armor : armors_) {
      number_imgs.emplace_back(armor.number_img);
    }
    cv::Mat all_num_img;
    cv::vconcat(number_imgs, all_num_img);
    return all_num_img;
  }
}

void Detector::drawResults(cv::Mat & img)
{
  // Draw Lights
  for (const auto & light : lights_) {
    cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
    cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
    auto line_color = light.color == RED ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 0, 255);
    cv::line(img, light.top, light.bottom, line_color, 1);
  }

  // Draw armors
  for (const auto & armor : armors_) {
    cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
    cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
  }

  // Show numbers and confidence
  for (const auto & armor : armors_) {
    cv::putText(
      img, armor.classfication_result, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 255, 255), 2);
  }
}



NumberClassifier::NumberClassifier(
  const std::string & model_path, const std::string & label_path, const double thre,
  const std::vector<std::string> & ignore_classes)
: threshold(thre), ignore_classes_(ignore_classes)
{
  net_ = cv::dnn::readNetFromONNX(model_path);

  std::ifstream label_file(label_path);
  std::string line;
  while (std::getline(label_file, line)) {
    class_names_.push_back(line);
  }
}

void NumberClassifier::extractNumbers(const cv::Mat & src, std::vector<Armor> & armors)
{
  // Light length in image
  const int light_length = 12;
  // Image size after warp
  const int warp_height = 28;
  const int small_armor_width = 32;
  const int large_armor_width = 54;
  // Number ROI size
  const cv::Size roi_size(20, 28);

  for (auto & armor : armors) {
    // Warp perspective transform
    cv::Point2f lights_vertices[4] = {
      armor.left_light.bottom, armor.left_light.top, armor.right_light.top,
      armor.right_light.bottom};

    const int top_light_y = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    const int warp_width = armor.type == ArmorType::SMALL ? small_armor_width : large_armor_width;
    cv::Point2f target_vertices[4] = {
      cv::Point(0, bottom_light_y),
      cv::Point(0, top_light_y),
      cv::Point(warp_width - 1, top_light_y),
      cv::Point(warp_width - 1, bottom_light_y),
    };
    cv::Mat number_image;
    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
    cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

    // Get ROI
    number_image =
      number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // Binarize
    cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    armor.number_img = number_image;
  }
}

void NumberClassifier::classify(std::vector<Armor> & armors)
{
  for (auto & armor : armors) {
    cv::Mat image = armor.number_img.clone();

    // Normalize
    image = image / 255.0;

    // Create blob from image
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob);

    // Set the input blob for the neural network
    net_.setInput(blob);
    // Forward pass the image blob through the model
    cv::Mat outputs = net_.forward();

    // Do softmax
    float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
    cv::Mat softmax_prob;
    cv::exp(outputs - max_prob, softmax_prob);
    float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    softmax_prob /= sum;

    double confidence;
    cv::Point class_id_point;
    minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;

    armor.confidence = confidence;
    armor.number = class_names_[label_id];

    std::stringstream result_ss;
    result_ss << armor.number << ": " << std::fixed << std::setprecision(1)
              << armor.confidence * 100.0 << "%";
    armor.classfication_result = result_ss.str();
  }

  armors.erase(
    std::remove_if(
      armors.begin(), armors.end(),
      [this](const Armor & armor) {
        if (armor.confidence < threshold) {
          return true;
        }

        for (const auto & ignore_class : ignore_classes_) {
          if (armor.number == ignore_class) {
            return true;
          }
        }

        bool mismatch_armor_type = false;
        if (armor.type == ArmorType::LARGE) {
          mismatch_armor_type =
            armor.number == "outpost" || armor.number == "2" || armor.number == "guard";
        } else if (armor.type == ArmorType::SMALL) {
          mismatch_armor_type = armor.number == "1" || armor.number == "base";
        }
        return mismatch_armor_type;
      }),
    armors.end());
}



//--------------------------------------
// ======================================
// 补充 Light 结构体构造函数实现
// ======================================
Light::Light(cv::Rect box, cv::Point2f top, cv::Point2f bottom, int area, float tilt_angle)
    : cv::Rect(box),  // 初始化父类 cv::Rect
      top(top), 
      bottom(bottom), 
      tilt_angle(tilt_angle) {
    // 计算光源长度（上下端点欧氏距离）
    this->length = cv::norm(top - bottom);
    // 计算光源宽度（面积 / 长度，因光源近似矩形）
    this->width = static_cast<double>(area) / this->length;
    // 计算光源中心点（上下端点中点）
    this->center = (top + bottom) / 2.0f;
    // 颜色默认初始化（后续在 findLights 中赋值）
    this->color = -1;
}

// ======================================
// 补充 Armor 结构体构造函数实现
// ======================================
Armor::Armor(const Light &l1, const Light &l2) {
    // 区分左右光源：x坐标小的为左光源
    if (l1.center.x < l2.center.x) {
        this->left_light = l1;
        this->right_light = l2;
    } else {
        this->left_light = l2;
        this->right_light = l1;
    }
    // 计算装甲板中心点（左右光源中心的中点）
    this->center = (this->left_light.center + this->right_light.center) / 2.0f;
    // 类型默认初始化（后续在 matchLights 中赋值）
    this->type = ArmorType::INVALID;
    // 置信度默认初始化（后续在 classify 中赋值）
    this->confidence = 0.0f;
}

// ======================================
// 补充 Detector 类构造函数实现
// ======================================
Detector::Detector(const int &bin_thres, const int &color, const LightParams &l, const ArmorParams &a)
    : binary_thres(bin_thres),
      detect_color(color),
      l(l),
      a(a) {
    // 初始化数字分类器为 nullptr（后续在 main 中手动创建）
    this->classifier = nullptr;
}




PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
  // Unit: m
  constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
  constexpr double large_half_y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;

  // Start from bottom left in clockwise order
  // Model coordinate: x forward, y left, z up
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));

  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, -large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, -large_half_z));
}

bool PnPSolver::solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec)
{
  std::vector<cv::Point2f> image_armor_points;

  // Fill in image points
  image_armor_points.emplace_back(armor.left_light.bottom);
  image_armor_points.emplace_back(armor.left_light.top);
  image_armor_points.emplace_back(armor.right_light.top);
  image_armor_points.emplace_back(armor.right_light.bottom);

  // Solve pnp
  auto object_points = armor.type == ArmorType::SMALL ? small_armor_points_ : large_armor_points_;
  return cv::solvePnP(
    object_points, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}





int start(){
// ======================================
    // 1. 配置检测参数（根据实际场景调整）
    // ======================================
    LightParams light_params;
    light_params.min_ratio = 0.1;        // 光源宽高比最小值
    light_params.max_ratio = 0.5;        // 光源宽高比最大值
    light_params.max_angle = 30.0;       // 光源最大倾斜角度（度）
    light_params.min_fill_ratio = 0.6;   // 光源最小填充率

    ArmorParams armor_params;
    armor_params.min_light_ratio = 0.8;  // 两光源长度比最小值
    armor_params.min_small_center_distance = 1.0;  // 小型装甲板中心距下限
    armor_params.max_small_center_distance = 3.0;  // 小型装甲板中心距上限
    armor_params.min_large_center_distance = 3.0;  // 大型装甲板中心距下限
    armor_params.max_large_center_distance = 6.0;  // 大型装甲板中心距上限
    armor_params.max_angle = 10.0;       // 光源连线最大水平角度

    const int binary_thres = 120;        // 二值化阈值
    const int detect_color = BLUE;        // 检测颜色（RED=0 或 BLUE=1）
    const std::string model_path = "model/mlp.onnx";  // 数字识别模型路径
    const std::string label_path = "model/label.txt"; // 标签文件路径
    const double conf_threshold = 0.5;   // 识别置信度阈值
    const std::vector<std::string> ignore_classes = {};  // 忽略类别

    // ======================================
    // 2. 相机内参和畸变系数（需要根据实际标定结果修改）
    // ======================================
    std::array<double, 9> camera_matrix = {
        800.0, 0.0, 320.0,    // fx, 0, cx
        0.0, 800.0, 240.0,    // 0, fy, cy
        0.0, 0.0, 1.0         // 0, 0, 1
    };
    std::vector<double> dist_coeffs = {0.0, 0.0, 0.0, 0.0, 0.0};  // 畸变系数

    // ======================================
    // 3. 初始化检测器、分类器和PnP求解器
    // ======================================
    try {
        Detector detector(binary_thres, detect_color, light_params, armor_params);
        PnPSolver pnp_solver(camera_matrix, dist_coeffs);
        
        // 检查模型文件是否存在
        std::ifstream model_check(model_path);
        if (!model_check.good()) {
            std::cerr << "错误：模型文件不存在！路径：" << model_path << std::endl;
            return -1;
        }

        // 检查标签文件是否存在
        std::ifstream label_check(label_path);
        if (!label_check.good()) {
            std::cerr << "错误：标签文件不存在！路径：" << label_path << std::endl;
            return -1;
        }

        // 初始化数字分类器
        detector.classifier = std::make_unique<NumberClassifier>(
            model_path, label_path, conf_threshold, ignore_classes
        );

        // ======================================
        // 4. 打开摄像头
        // ======================================
        cv::VideoCapture cap(0);  // 0表示默认摄像头，多摄像头可尝试1、2等
        if (!cap.isOpened()) {
            std::cerr << "摄像头打开失败！检查设备连接或权限" << std::endl;
            return -1;
        }

        

        // ======================================
        // 5. 实时检测循环
        // ======================================
        cv::Mat input_image, show_image;
        std::cout << "开始实时检测，按ESC键退出..." << std::endl;

        while (true) {
            // 读取一帧图像
            cap >> input_image;
            if (input_image.empty()) {
                std::cerr << "无法获取图像帧！" << std::endl;
                break;
            }

            // 转换颜色空间（OpenCV默认BGR，代码中使用RGB）
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

            // 按ESC键退出（ASCII码27）
            char key = cv::waitKey(1);
            if (key == 27) {
                std::cout << "用户退出检测" << std::endl;
                break;
            }
        }

        // 释放资源
        cap.release();
        cv::destroyAllWindows();

    } catch (const std::exception& e) {
        std::cerr << "程序异常：" << e.what() << std::endl;
        return -1;
    }

    return 0;
}
