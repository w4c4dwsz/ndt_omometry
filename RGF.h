#include "utility.h"    

#define PI 3.14159265

typedef pcl::PointXYZI  PointType;



// VLP-16
extern const int N_SCAN;
extern const int Horizon_SCAN;
extern const float ang_res_x;
extern const float ang_res_y;
extern const float ang_bottom;
extern const int groundScanInd;
extern const float sensorMinimumRange;
extern const float sensorMountAngle;

extern const bool loopClosureEnableFlag;
extern const double mappingProcessInterval;

extern const float scanPeriod;
extern const int systemDelay;
extern const int imuQueLength;

// LEGO-LOAM的点云滤波算法
class RGF{
    private:

    
    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    

    pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fullCLoudRGB;
    pcl::PointCloud<PointType>::Ptr groundCloud;
    pcl::PointCloud<PointType>::Ptr noGroundCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
    pcl::PointCloud<PointType>::Ptr outlierCloud;

    PointType nanPoint; // fill in fullCloud at each iteration
    pcl::PointXYZRGB nanPointRGB;

    cv::Mat rangeMat; // range matrix for range image
    cv::Mat labelMat; // label matrix for segmentaiton marking
    cv::Mat groundMat; // ground matrix for ground cloud marking
    int labelCount;

    float startOrientation;
    float endOrientation;

    

    std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process

    uint16_t *allPushedIndX; // array for tracking points of a segmented object
    uint16_t *allPushedIndY;

    uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
    uint16_t *queueIndY;

    

    public:
    RGF();

    void allocateMemory();

    void resetParameters();

    ~RGF(){}

    /**
     * 将点云投影为深度图
     * **/
    void projectPointCloud();

    // 取出地面
    void groundRemoval();

    // 设置输入的滤波点云数据
    void setInputCloud(pcl::PointCloud<PointType>::Ptr input_cloud){
        laserCloudIn = input_cloud;
    }

    void filter(pcl::PointCloud<PointType> &ground_cloud, pcl::PointCloud<PointType> &no_ground_cloud);

    void cloudSegmentation();

    void labelComponents(int row, int col);

    void visualzation(pcl::PointCloud<pcl::PointXYZRGB> &cloud);

    void getSegmentationCloud(pcl::PointCloud<pcl::PointXYZ> & segmentationCloud, cv::Mat &tagMat, vector<int> & segmentLineIndex, 
                                pcl::PointCloud<pcl::PointXYZ> & groundCloud, vector<int> &groundIndex);
};    



