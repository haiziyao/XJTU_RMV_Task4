#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <example_interfaces/srv/detail/add_two_ints__struct.hpp>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/utilities.hpp>
#include "example_interfaces/srv/add_two_ints.hpp"

class ServiceClient01: public rclcpp::Node{
public:
    ServiceClient01(std::string name):rclcpp::Node(name){
        RCLCPP_INFO(this->get_logger(),"%s节点已经启动",name.c_str());
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_");
    }
    void send_request(int a,int b){
        RCLCPP_INFO(this->get_logger(),"计算%d+%d",a,b);
        while(!client_->wait_for_service(std::chrono::seconds(1))){
            if(!rclcpp::ok()){
                RCLCPP_ERROR(this->get_logger(),"等待服务的过程被打断");
                return;
            }
            RCLCPP_INFO(this->get_logger(),"等待服务上线中");
        }
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts_Request>();
        request->a = a;
        request->b = b;

        client_->async_send_request(request,std::bind(&ServiceClient01::result_callback_,this,std::placeholders::_1));
    }
    
private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>:: SharedPtr client_;
    void result_callback_(
        rclcpp::Client<example_interfaces::srv::AddTwoInts> ::SharedFuture result_future
    ){
        auto response = result_future.get();
        RCLCPP_INFO(this->get_logger(),"计算结果: %ld",response->sum);
    }
};

int main(int argc,char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServiceClient01>("Service_client_01");
    node->send_request(5, 6);
    rclcpp::spin(node);
    rclcpp::shutdown();
}