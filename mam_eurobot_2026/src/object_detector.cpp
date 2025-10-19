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

        pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detected_objects", 10);
        debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug_cloud", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/supposed_objects", 10);

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
        {"weit", 0.027, 20,  2000}
    };

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
                for (int idx : indices.indices) {
                    const auto& pt = filtered->points[idx];
                    clustered->points.push_back(pt);
                    cluster->points.push_back(pt);
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

                marker.color.r = (params.name == "weit") ? 0.5 : 1 ;
                marker.color.g = (params.name == "nah") ? 0.5 : 1.0;
                marker.color.b = 0.0;
                marker.color.a = 0.6;

                marker_array.markers.push_back(marker);
            }
        }

        marker_pub_->publish(marker_array);
        // pub_->publish(pose_array);

        sensor_msgs::msg::PointCloud2 clustered_msg;
        pcl::toROSMsg(*clustered, clustered_msg);
        clustered_msg.header = msg->header;
        debug_pub_->publish(clustered_msg);
    }

    float leaf_size_;
    float floor_offset_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_;
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
