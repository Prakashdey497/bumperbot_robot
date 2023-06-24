#include <rclcpp/rclcpp.hpp>

class MyNode:public rclcpp::Node
{
public:
    MyNode(): Node("simple_node"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(),"Node Started");

        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                        std::bind(&MyNode::timer_callback,this));
        
    }

private:
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(),"Hello world %d",counter_);
        counter_ ++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};




int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    // auto node = std::make_shared<rclcpp::Node>("simple_node");
    // RCLCPP_INFO(node->get_logger(),"Hello world");

    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}