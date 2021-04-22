#include "utility.h"
#include <ceres/ceres.h>
#include <pcl/visualization/range_image_visualizer.h>

void calcuate_curv(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer viewer("output");
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::RangeImage range_image;

    pcl::io::loadPCDFile("../data/pcd6666.pcd", *input_cloud);
    pcl::io::loadPCDFile("../data/pcd6666.pcd", *target_cloud);

    // float angle_resulation = (1 / 180 * M_PI);
    // float max_angle_Width = (360 / 180 * M_PI);
    // float max_angle_height = (180 / 180 * M_PI);
    // Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    Eigen::Affine3f sensor_pose;
    range_image.createFromPointCloud(
                                    *input_cloud, 
                                    pcl::deg2rad(float(0.2)),
                                    pcl::deg2rad(float(2)), 
                                    M_PI * 2, 
                                    M_PI, 
                                    (Eigen::Affine3f)Eigen::Translation3f(0, 0, 0), 
                                    pcl::RangeImage::LASER_FRAME, 
                                    0, 
                                    0, 
                                    0
    );
    std::cout << range_image << endl;
    std::cout << input_cloud.get()->size() << endl;
    
    viewer.addPointCloud(input_cloud, "cloud");
    while(!viewer.wasStopped()) {
        viewer.spinOnce();
    }
    return 0;
}