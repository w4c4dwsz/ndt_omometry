#include "utility.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
int main(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile("../data/pcd6110.pcd", *cloud);
    pcl::visualization::PCLVisualizer viewer;
    int v1 = 0;
    viewer.createViewPort(0.0,0.0,0.5,0.5,v1);
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> input_color(cloud);
    viewer.addPointCloud(cloud, input_color, "input_color", v1);
    viewer.addText("oridinary_cloud", 10,10,"oridinary_cloud", v1);

    //voxel start
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.1,0.1,0.1);
    voxel.filter(*filter_cloud);
    int v2 = 0;
    viewer.createViewPort(0.5,0.0,1.0,0.5,v2);
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> filter_color(filter_cloud);
    viewer.addPointCloud(filter_cloud, filter_color,"filter_cloud", v2);
    viewer.addText("filter_cloud", 10,10,"filter_cloud", v2);

    //cluster 
    // pcl::SACSegmentation<pcl::PointXYZ> seg;
    // pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // seg.setInputCloud(filter_cloud);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.segment(*indices, *coefficients);

    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // extract.setInputCloud(filter_cloud);
    // extract.setIndices(indices);
    // extract.setNegative(false);
    // extract.filter(*output_cloud);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0,1);
    pass.setInputCloud(filter_cloud);
    //pass.setNegative(true);
    pass.filter(*output_cloud);
    
    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    // extract.setInputCloud(filter_cloud);
    // extract.setIndices(indices);
    // extract.setNegative(false);
    // extract.filter(*output_cloud);
    int v3 = 0;
    viewer.createViewPort(0.0,0.5,0.5,1.0, v3);
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> output_color(output_cloud);
    viewer.addText("output_cloud",10,10,"output_txt", v3);
    viewer.addPointCloud(output_cloud, output_color, "output_color", v3);
    while(!viewer.wasStopped()) {
        viewer.spin();
        std::this_thread::sleep_for(1ms);
    }

    return 1;
}