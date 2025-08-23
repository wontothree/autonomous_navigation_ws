#include "mc_localizer/mc_localizer_ros.hpp"

namespace mc_localizer {

MCLocalizerROS::MCLocalizerROS() : Node("mc_localizer_node")
{
    // hyper parameter
    scan_topic_name_ = "/diffbot/scan";
    odom_topic_name_ = "/diffbot/odom";
    initial_pose_topic_name_ = "/initialpose";
    // subscribers
    initial_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initial_pose_topic_name_,
        10,
        [this](const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped> msg) {
            this->callback_initial_pose(msg);
        }
    ); // initial pose subscriber
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_name_,
        10,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            this->callback_scan(msg);
        }
    ); // laser scan subscriber
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_name_,
        10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->callback_odom(msg);
        }
    ); // odometry subscriber
    
    // publisher
    particle_set_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("particle_set", 10);


    // timer callback (main execute)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_),
        [this]() { this->callback_timer(); }
    );

}

void MCLocalizerROS::callback_timer()
{
    //
}

void MCLocalizerROS::callback_initial_pose(
    const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped> initial_pose
)
{
    // declare quaternion
    tf2::Quaternion orientation_quaternion(
        initial_pose->pose.pose.orientation.x,
        initial_pose->pose.pose.orientation.y,
        initial_pose->pose.pose.orientation.z,
        initial_pose->pose.pose.orientation.w
    );

    // calculate roll, pitch, yaw and we only take yaw
    double _, __, yaw;
    tf2::Matrix3x3 rotation_matrix(orientation_quaternion);
    rotation_matrix.getRPY(_, __, yaw);

    // set robot pose
    mclocalizer_object_.mcl_estimated_pose_.set_pose(
        initial_pose->pose.pose.position.x,
        initial_pose->pose.pose.position.y,
        yaw
    );

    // random sampling particles to generate particle set
    mclocalizer_object_.sample_particles(mclocalizer_object_.mcl_estimated_pose_, mclocalizer_object_.initial_pose_noise_);

    // // 필요 시 신뢰도 초기화
    // if (estimateReliability_)
    //     resetReliabilities();

    // // 초기화 완료 flag
    // isInitialized_ = true;
}

void MCLocalizerROS::callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
{
    if (1) // canUpdateScan_
        scan_ = *scan;
    // if (!gotScan_)
    //     gotScan_ = true;
}

void MCLocalizerROS::callback_odom(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    std::cout << "odom callback" << std::endl;
}

void MCLocalizerROS::publish_particle_set(const std::vector<Particle>& particle_set)
{
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < particle_set.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        // marker.header.frame_id = map_frame_id_;  // RViz에서 사용하는 fixed frame
        marker.header.stamp = this->now();       
        marker.ns = "particles";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;  // particle를 sphere로 표시
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 위치
        marker.pose.position.x = particle_set[i].get_x();
        marker.pose.position.y = particle_set[i].get_y();
        marker.pose.position.z = 0.05;  // 바닥 위 약간 띄움

        // 방향 (Sphere는 방향 필요 없음)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // 크기
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        // 색상: weight 기반으로 alpha 설정 가능
        double w = particle_set[i].get_weight();
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = std::min(std::max(w * 10.0, 0.1), 1.0);

        marker_array.markers.push_back(marker);
    }

    // 퍼블리시
    particle_set_publisher_->publish(marker_array);
}

} // namespace mc_localizer