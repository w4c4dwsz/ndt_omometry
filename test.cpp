#include "utility.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/StdVector>
#include <algorithm>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/model_outlier_removal.h>

#include "GPF.h"

int main()
{
    GPF gpf;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_cloud_not_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_cloud_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer viewer("hello world");

    pcl::io::loadPCDFile("../data/pcd6090.pcd", *cloud);

    gpf.set_input(cloud);
    
    gpf.filter(*all_cloud_not_ground, *all_cloud_ground);
    
    cout << all_cloud_ground->points.size() <<endl;

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> all_cloud_not_ground_color_h(all_cloud_not_ground, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> all_cloud_ground_color_h(all_cloud_ground, 255, 105, 180);
    viewer.addPointCloud(all_cloud_not_ground, all_cloud_not_ground_color_h, "not-ground");
    viewer.addPointCloud(all_cloud_ground, all_cloud_ground_color_h, "ground");

    while(!viewer.wasStopped())
    {
        viewer.spin();
        std::this_thread::sleep_for(1s);
    }        
} 