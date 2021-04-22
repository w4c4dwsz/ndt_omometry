#include "utility.h"

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer viewer;
    cloud->width = 50;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        cloud->points[i].x = 1024 * rand()/(RAND_MAX + 1.0f);
        cloud->points[i].x = 1024 * rand()/(RAND_MAX + 1.0f);
        cloud->points[i].x = 1024 * rand()/(RAND_MAX + 1.0f);
    }
    pcl::io::savePCDFileASCII("test_pcd", *cloud);
    viewer.addPointCloud(cloud, "cloud");
    while (!viewer.wasStopped())
    {
        viewer.spin();
        std::this_thread::sleep_for(10ms);
    }
    
    return 0;
}