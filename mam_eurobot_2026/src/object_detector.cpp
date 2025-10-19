#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>  
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ObjectDetector : public rclcpp::Node {
public:
    ObjectDetector() : Node("object_detector") {
        this->declare_parameter("leaf_size", 0.01);
        this->declare_parameter("floor_offset", -0.119);

        leaf_size_ = this->get_parameter("leaf_size").as_double();
        floor_offset_ = this->get_parameter("floor_offset").as_double();

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar_3d/points", 10,
            std::bind(&ObjectDetector::pointCloudCallback, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera", 10,
            std::bind(&ObjectDetector::image_callback, this, std::placeholders::_1));


        debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug_cloud", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/supposed_objects", 10);

        R_  << 0, 1, 0,   // x_cam = y_gazebo
            0, 0, -1,  // y_cam = -z_gazebo
            1, 0, 0;        
        t_ = Eigen::Vector3f(0.0f, 0.0f, -0.03f);

        K_ = (cv::Mat_<double>(3,3) << 
            554.3827, 0.0,     320.0,
            0.0,      415.787, 240.0,
            0.0,      0.0,     1.0);
        
        RCLCPP_INFO(this->get_logger(), "ObjectDetector initialized.");
    }

private:
    struct ClusterParams {
        std::string name;
        double cluster_tolerance;
        int min_cluster_size;
        int max_cluster_size;
    };

    std::vector<ClusterParams> param_sets = {
        {"nah",  0.009,  45, 2000},
        {"mittel", 0.018, 45,  2000},
        {"weit", 0.027, 20,  2000},
        {"sweit", 0.036, 5,  2000},
        {"ssweit", 0.045, 2,  2000}
    };

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame_raw;
         try {
           frame_raw = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }  

        cv::Mat hsv;
        cv::cvtColor(frame_raw, hsv, cv::COLOR_BGR2HSV);

        //Blau (RAL 5017)
        cv::Scalar lower_blue(100, 100, 50);
        cv::Scalar upper_blue(130, 255, 255);
        cv::Mat mask_blue;
        cv::inRange(hsv, lower_blue, upper_blue, mask_blue);

        //Gelb (RAL 1023)
        cv::Scalar lower_yellow(20, 100, 100);
        cv::Scalar upper_yellow(35, 255, 255);
        cv::Mat mask_yellow;
        cv::inRange(hsv, lower_yellow, upper_yellow, mask_yellow);

        // Schwarz (RAL 9017)
        cv::Scalar lower_black(0, 0, 0);
        cv::Scalar upper_black(180, 255, 50);
        cv::Mat mask_black;
        cv::inRange(hsv, lower_black, upper_black, mask_black);

        // Grau (RAL 7032)
        cv::Scalar lower_gray(0, 0, 80);
        cv::Scalar upper_gray(180, 50, 200);
        cv::Mat mask_gray;
        cv::inRange(hsv, lower_gray, upper_gray, mask_gray);

        cv::Mat combined_mask = mask_blue | mask_yellow | mask_black | mask_gray;

        frame_raw.copyTo(frame_, combined_mask); // Nur relevante Farben bleiben
    }


    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clean(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& pt : cloud->points) {
            if (pcl::isFinite(pt) && pt.z > floor_offset_) cloud_clean->points.push_back(pt);
        }

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud(cloud_clean);
        vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        vg.filter(*filtered);

        pcl::PointCloud<pcl::PointXYZ>::Ptr clustered(new pcl::PointCloud<pcl::PointXYZ>);
        visualization_msgs::msg::MarkerArray marker_array;
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header = msg->header;

        // Clear all markers once

        int marker_id = 0;

        for (const auto& params : param_sets) {
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(filtered);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(params.cluster_tolerance);
            ec.setMinClusterSize(params.min_cluster_size);
            ec.setMaxClusterSize(params.max_cluster_size);
            ec.setSearchMethod(tree);
            ec.setInputCloud(filtered);
            ec.extract(cluster_indices);

            visualization_msgs::msg::Marker clear_marker;
            clear_marker.header = msg->header;
            clear_marker.ns = "box_" + params.name;
            clear_marker.id = marker_id++;
            clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array.markers.push_back(clear_marker);


            for (const auto& indices : cluster_indices) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
                std::vector<cv::Vec3b> cluster_colors;

                for (int idx : indices.indices) {
                    const auto& pt = filtered->points[idx];
                    clustered->points.push_back(pt);
                    cluster->points.push_back(pt);

                    Eigen::Vector3f p_lidar(pt.x, pt.y, pt.z);
                    Eigen::Vector3f p_cam = R_ * p_lidar + t_;
                    if (p_cam.z() <= 0) continue; // hinter der Kamera

                    float u = (K_.at<double>(0,0) * p_cam.x() / p_cam.z()) + K_.at<double>(0,2);
                    float v = (K_.at<double>(1,1) * p_cam.y() / p_cam.z()) + K_.at<double>(1,2);
                
                    if (u >= 0 && u < frame_.cols && v >= 0 && v < frame_.rows) {
                        cv::Vec3b color = frame_.at<cv::Vec3b>(cv::Point(u,v));
                        cluster_colors.push_back(color); 
                    }
                }

                // OBB berechnen
                pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
                feature_extractor.setInputCloud(cluster);
                feature_extractor.compute();

                pcl::PointXYZ min_pt_OBB, max_pt_OBB, position;
                Eigen::Matrix3f rotational_matrix;
                feature_extractor.getOBB(min_pt_OBB, max_pt_OBB, position, rotational_matrix);

                // Höhe berechnen (wie bisher)
                auto height = std::abs(max_pt_OBB.z - floor_offset_);
                if (height < 0.01) continue;

                // Länge und Breite aus OBB
                float length = std::abs(max_pt_OBB.x - min_pt_OBB.x);
                float width  = std::abs(max_pt_OBB.y - min_pt_OBB.y);
                if (length > 1.5 || width > 1.5) continue; // recognize nut, robots

                // Quaternion aus Rotationsmatrix
                Eigen::Quaternionf quat(rotational_matrix);
                geometry_msgs::msg::Quaternion orientation;
                orientation.x = quat.x();
                orientation.y = quat.y();
                orientation.z = quat.z();
                orientation.w = quat.w();

                // Marker setzen
                visualization_msgs::msg::Marker marker;
                marker.header = msg->header;
                marker.ns = "box_" + params.name;
                marker.id = marker_id++;
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.action = visualization_msgs::msg::Marker::ADD;

                marker.pose.position.x = position.x;
                marker.pose.position.y = position.y;
                marker.pose.position.z = position.z;
                marker.pose.orientation = orientation;

                marker.scale.x = length;
                marker.scale.y = width;
                marker.scale.z = height;

                if (cluster_colors.empty()) {
                    marker.color.r = (params.name == "weit") ? 0.3 : 0.4 ;
                    marker.color.g = (params.name == "nah") ? 0.1 : 0.2;
                    marker.color.b = 0.5;
                } else {
                    cv::Scalar avg_bgr = cv::mean(cluster_colors);
                    cv::Mat bgr_pixel(1, 1, CV_8UC3, cv::Vec3b(avg_bgr[0], avg_bgr[1], avg_bgr[2]));
                    cv::Mat hsv_pixel;
                    cv::cvtColor(bgr_pixel, hsv_pixel, cv::COLOR_BGR2HSV);
                    cv::Vec3b avg_hsv = hsv_pixel.at<cv::Vec3b>(0, 0);

                    if (is_blue(avg_hsv)) {
                        marker.color.r = 0.0;
                        marker.color.g = 0.0;
                        marker.color.b = 1.0;
                    }
                    else if (is_yellow(avg_hsv)) {
                        marker.color.r = 1.0;
                        marker.color.g = 1.0;
                        marker.color.b = 0.0;
                    }
                    else if (is_black(avg_hsv)) {
                        marker.color.r = 1.0;
                        marker.color.g = 1.0;
                        marker.color.b = 1.0;
                    } 
                    else {
                        marker.color.r = 0.5;
                        marker.color.g = 0.5;
                        marker.color.b = 0.5;
                    } 
                }

                marker.color.a = 0.6;
                marker_array.markers.push_back(marker);
            }
        }

        marker_pub_->publish(marker_array);

        sensor_msgs::msg::PointCloud2 clustered_msg;
        pcl::toROSMsg(*clustered, clustered_msg);
        clustered_msg.header = msg->header;
        debug_pub_->publish(clustered_msg);
    }

    bool is_blue(const cv::Vec3b& hsv_color) const
    {
        return (hsv_color[0] >= 90 && hsv_color[0] <= 140 &&
                hsv_color[1] >= 50 &&
                hsv_color[2] >= 40);
    }

    bool is_yellow(const cv::Vec3b& hsv_color) const
    {
        return (hsv_color[0] >= 15 && hsv_color[0] <= 45 &&
                hsv_color[1] >= 50 &&
                hsv_color[2] >= 80);
    }

    bool is_black(const cv::Vec3b& hsv_color) const
    {
        return (hsv_color[2] <= 30 && hsv_color[1] <= 60);
    }

    bool is_gray(const cv::Vec3b& hsv_color) const
    {
        return (hsv_color[1] <= 50 && hsv_color[2] >= 80 && hsv_color[2] <= 200);
    }


    bool isRelevantColor(const cv::Vec3b& hsv_color) const
    {
        return is_blue(hsv_color) || is_yellow(hsv_color) || is_black(hsv_color) || is_gray(hsv_color);
    }

    float leaf_size_;
    float floor_offset_;

    cv::Mat frame_;
    Eigen::Matrix3f R_; // Rotation Lidar → Kamera
    Eigen::Vector3f t_; // Translation Lidar → Kamera
    cv::Mat K_; // kamera Matrix 
    /* | f_x  0   c_x   |
       | 0   f_y  c_y   |
       | 0    0    1    |
    */

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetector>());
    rclcpp::shutdown();
    return 0;
}
