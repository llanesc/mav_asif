#include "mav_control_router.hpp"

using namespace asif;

int main(int argc, char *argv[]) {
    std::cout << "Starting MAV2 Control example..." << std::endl;
    Eigen::initParallel();
    Eigen::setNbThreads(16);
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MavControlRouter>(1));

    rclcpp::shutdown();
    return 0;
}