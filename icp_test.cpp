#include "utility.h"
#include "RGF.h"

struct SmoothData{
    SmoothData(double curv, int ind) : curvature(curv), index(ind){}
    double curvature;
    int index;
};

int main() {
    // 初始化一些变量
    pcl::visualization::PCLVisualizer viewer("hello world");
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../data/pcd6669.pcd", *input_cloud);
    vector<int> index;
    pcl::removeNaNFromPointCloud<pcl::PointXYZ>(*input_cloud, *input_cloud, index);
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_i(new pcl::PointCloud<pcl::PointXYZI>); 
    RGF IP;
    

    // XYZ->XYZI
    for(auto p :input_cloud->points){
    pcl::PointXYZI p_i;
    p_i.x = p.x;
    p_i.y = p.y;
    p_i.z = p.z;
    p_i.intensity = 0;
    //cout << p_i << endl;
    input_cloud_i->push_back(p_i);
    }

    // 利用RGF类处理输入的点云图像
    cv::Mat labelMat;
    vector<int> segmentLineIndex;
    IP.setInputCloud(input_cloud_i);
    IP.getSegmentationCloud(*input_cloud, labelMat, segmentLineIndex);


}