#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <queue>
#include <thread>
#include <chrono>
#include <string>
#include <algorithm>
#include <cmath>

using namespace std;
// 基于RANSC的地面滤波算法
class GPF
{
    private:
        // 算法参数变量设置
        int N_iter; // number of iterations
        int div_segment_num; // number of the segment used to dividing pointcloud
        double N_LPR; // number of points used to estimate the LPR(lowest point representative)
        double Th_seeds; // threshold for points to be considered initial seeds
        double Th_dist; // threshold distance frome plane 

        //存储点云用数据
        // pcl::PointCloud<pcl::PointXYZ>::Ptr all_cloud_ground_;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr all_cloud_not_ground_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;

    public:
        GPF():
                N_iter(3),
                div_segment_num(6),
                N_LPR(0.3),
                Th_seeds(0.1),
                Th_dist(0.2){};

        ~GPF(){};

        int filter(pcl::PointCloud<pcl::PointXYZ>& all_cloud_not_ground, pcl::PointCloud<pcl::PointXYZ>& all_cloud_ground);

        void set_input(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
        {
            input_cloud_ = input_cloud;
        }

        void set_N_iter(int m_N_iter)
        {
            N_iter = m_N_iter;
        }

        void set_div_segment_num(int m_div_segment_num)
        {
            div_segment_num = m_div_segment_num;
        }

        void set_N_LPR(double m_N_LPR)
        {
            N_LPR = m_N_LPR;
        }

        void set_Th_seeds(double m_Th_seeds)
        {
            Th_seeds = m_Th_seeds;
        }

        void set_Th_dist(double m_Th_dist)
        {
            Th_dist = m_Th_dist;
        }
};

