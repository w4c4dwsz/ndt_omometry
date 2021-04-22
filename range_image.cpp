#include "utility.h"
#include "RGF.h"
#include "ceres/ceres.h"
#include "eigen3/Eigen/Core"
#define DEBUG

DEFINE_bool(real_data, true, "if use the real_data");
DEFINE_double(t_x, 0, "the translate move on x axis");
DEFINE_double(t_y, 0, "the translate move on y axis");
DEFINE_double(t_z, 0, "the translate move on z axis");
DEFINE_double(pitch, 0, "pitch-x");
DEFINE_double(yaw, 0, "yaw-y");
DEFINE_double(roll, 0, "roll-z");
DEFINE_double(TransformationEpsilon, 0.1, "setTransformationEpsilon");
DEFINE_double(Resolution, 1, "setResolution");
DEFINE_double(targetFileID, 0, "the pcd file id for the target cloud");
DEFINE_double(sourceFileID, 1, "the pcd file id for the source cloud");
DEFINE_double(MaximumIterations, 35, "the MaximumIterations");

// ceres costfunction struct
class KeyPointCostFunction {
    public:
    KeyPointCostFunction(Eigen::Vector3f curr, Eigen::Vector3f lastA, Eigen::Vector3f lastB)
                        :currPoint(curr),
                        lastPointA(lastA),
                        lastPointB(lastB) {}
    
    template <typename T>
    bool operator()(const T * const q, const T * const t, T * residual) const 
    {

        Eigen::Matrix<T, 3, 1> currPos(T(currPoint.x()), T(currPoint.y()), T(currPoint.z()));
        Eigen::Matrix<T, 3, 1> lastPosA(T(lastPointA.x()), T(lastPointA.y()), T(lastPointA.z()));
        Eigen::Matrix<T, 3, 1> lastPosB( T(lastPointB.x()), T(lastPointB.y()), T(lastPointB.z()));
        Eigen::Quaternion<T> quaterionMatrix(q[3], q[0], q[1], q[2]);
        Eigen::Matrix<T, 3, 1> transformationMatrix(t[0], t[1], t[2]);
        Eigen::Matrix<T, 3, 1> currPosAsLast;
        currPosAsLast = quaterionMatrix * currPos + transformationMatrix;

        Eigen::Matrix<T, 3, 1> nu = (currPosAsLast - lastPosA).cross(currPosAsLast - lastPosB);
        Eigen::Matrix<T, 3, 1> de = lastPosA - lastPosB;

        residual[0] = nu.x() / de.norm();
		residual[1] = nu.y() / de.norm();
		residual[2] = nu.z() / de.norm();

        return true;
    }

    Eigen::Vector3f currPoint, lastPointA, lastPointB;
};


struct SmoothData{
    SmoothData(double curv, int ind) : curvature(curv), index(ind){}
    double curvature;
    int index;
};
class RangeImage {
    public:
        RangeImage(int N_SCAN = 16,  int Horizon_SCAN = 1800, float ang_res_x = 0.2, float ang_res_y = 2.0, float ang_bottom = 15.0+0.1):
                    _cloud(nullptr),
                    _N_SCAN(N_SCAN),
                    _Horizon_SCAN(Horizon_SCAN),
                    _ang_res_x(ang_res_x),
                    _ang_res_y(ang_res_y),
                    _ang_bottom(ang_bottom) {
                        for(int i = 0; i < N_SCAN; i++)
                            _data.push_back(vector<double>(Horizon_SCAN, -1));
                        _segmentLineIndex.resize(_N_SCAN * _Horizon_SCAN);
                    }

        void setInput(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, vector<int> &segment_index) {
            _cloud.reset();
            _cloud = input_cloud;
            _segmentLineIndex = segment_index;
        }
    
        void calculateCurvature() {
            // calcuate range
            _range.clear();
            _smooth.clear();
            for(auto point : *_cloud) {
                _range.push_back(
                    sqrt(point.x*point.x + point.y*point.y + point.z*point.z)
                );
            }

            for(int i = 0; i < _N_SCAN; i++)
                for(int j = _segmentLineIndex[i]; j < _segmentLineIndex[i + 1]; j++) {
                    int sum = 0;
                    int segmentLineSize = _segmentLineIndex[i + 1] - _segmentLineIndex[i];
                    if (segmentLineSize == 0)
                        continue;

                    for(int k = -10; k < 11; k++) {
                        int ind = ( (j - _segmentLineIndex[i]) + k + segmentLineSize) % segmentLineSize;
                        sum += _range[_segmentLineIndex[i] + ind] - _range[j];
                    }
                    SmoothData smooth(sum, j);
                    _smooth.push_back(smooth);
                }

            // sort(_smooth.begin(), _smooth.end(), [](SmoothData a, SmoothData b)-> bool {return a.curvature > b.curvature;});
            for (auto data : _smooth){
                if(isnan(data.curvature)) {
                    printf("[%f, %f, %f]\n", _cloud.get()->points[data.index].x, _cloud.get()->points[data.index].y, _cloud.get()->points[data.index].z);
                }

            }
                
            cout << _smooth.size() << endl;
        }

        void printData() {
            for(auto dat : _data)
                for(auto da : dat)
                    printf("%f\n", da);
        }
        
        int getValidPointQuantity(){
            int count = 0;
            for(auto dat : _data)
                for(auto da : dat){
                    if(da != -1) {
                        count++;
                    }
                        
                }
            return count;
        }

        void getSmooth(vector<SmoothData> * v) {
            for (int i = 0; i < _smooth.size(); i++)
            {
                v->push_back(_smooth[i]);
            }
            
        }
    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
        vector<vector<double>> _data;
        vector<int> _segmentLineIndex;
        vector<float> _range;
        vector<SmoothData> _smooth;
        int _N_SCAN;
        int _Horizon_SCAN;
        float _ang_res_x;
        float _ang_res_y;
        float _ang_bottom;
};



int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    std::vector<google::CommandLineFlagInfo> flags;
    google::GetAllFlags(&flags);

    class RangeImage ri;
    pcl::visualization::PCLVisualizer viewer("hello world");
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_new(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../data/pcd6533.pcd", *input_cloud);
    pcl::io::loadPCDFile("../data/pcd6538.pcd", *input_cloud_new);
    
    // 制作虚拟点云数据
    
    // Eigen::Matrix4f T_real = Eigen::Matrix4f::Identity();
    // T_real.block<3,1>(0, 3) = Eigen::Vector3f(FLAGS_t_x, FLAGS_t_y, FLAGS_t_z);
    // Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    // Eigen::AngleAxisf t_V1(M_PI * (FLAGS_pitch /180), Eigen::Vector3f(1, 0, 0));
    // Eigen::AngleAxisf t_V2(M_PI * (FLAGS_yaw /180), Eigen::Vector3f(0, 1, 0));
    // Eigen::AngleAxisf t_V3(M_PI * (FLAGS_roll /180), Eigen::Vector3f(0, 0, 1));
    // R = R * t_V1 * t_V2 * t_V3;
    // T_real.block<3,3>(0,0) = R;
    // pcl::transformPointCloud(*input_cloud_new, *input_cloud, T_real);

    vector<int> index;
    pcl::removeNaNFromPointCloud<pcl::PointXYZ>(*input_cloud, *input_cloud, index);
    pcl::removeNaNFromPointCloud<pcl::PointXYZ>(*input_cloud_new, *input_cloud_new, index);
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_i(new pcl::PointCloud<pcl::PointXYZI>); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_new_i(new pcl::PointCloud<pcl::PointXYZI>);
    RGF IP;
    /**
     * 声明存储特征点的变量，包括点特征点和面特征点
     */
    cv::Mat labelMat;
    cv::Mat labelMatNew;
    vector<int> segmentLineIndex;
    vector<int> segmentLineIndexNew;
    vector<int> groundIndexLast;
    vector<int> groundIndexNew;
    pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloudLast(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloudNew(new pcl::PointCloud<pcl::PointXYZ>);

    /**
     * 将输入点云从pcl::PointXYZ转为pcl::PointXYZI
     */
    for(auto p :input_cloud->points){
        pcl::PointXYZI p_i;
        p_i.x = p.x;
        p_i.y = p.y;
        p_i.z = p.z;
        p_i.intensity = 0;
        //cout << p_i << endl;
        input_cloud_i->push_back(p_i);
    }

    for(auto p :input_cloud_new->points){
        pcl::PointXYZI p_i;
        p_i.x = p.x;
        p_i.y = p.y;
        p_i.z = p.z;
        p_i.intensity = 0;
        //cout << p_i << endl;
        input_cloud_new_i->push_back(p_i);
    }




    /**
     * @brief: 对输入的数据进行处理
     */
    // 处理上一帧点云
    IP.setInputCloud(input_cloud_i);
    IP.getSegmentationCloud(*input_cloud, labelMat, segmentLineIndex, *groundCloudLast, groundIndexLast);

    // 处理上一帧点云中点特征点的曲度
    ri.setInput(input_cloud, segmentLineIndex);
    ri.calculateCurvature();
    vector<SmoothData> smooth_vector;
    ri.getSmooth(&smooth_vector);
    // 处理上一帧点云中面特征点的曲度
    ri.setInput(groundCloudLast, groundIndexLast);
    ri.calculateCurvature();
    vector<SmoothData> smoothVectorLast;
    ri.getSmooth(&smoothVectorLast);

    // 处理当前帧点云
    IP.setInputCloud(input_cloud_new_i);
    IP.getSegmentationCloud(*input_cloud_new, labelMatNew, segmentLineIndexNew, *groundCloudNew, groundIndexNew);

    ri.setInput(input_cloud_new, segmentLineIndexNew);
    ri.calculateCurvature();
    vector<SmoothData> smooth_vector_new;
    ri.getSmooth(&smooth_vector_new);

    /// TODO: 添加处理新一帧特征点的曲度


    cout << "=====================" << endl;
    cout << "smooth_vector size = " << endl;
    cout << smooth_vector.size() << endl;
    cout << "smooth_vector_new size = " << endl;
    cout << smooth_vector_new.size() << endl;
    cout << " input_cloud size = " << input_cloud->size() << endl;
    cout << " input_cloud size_new = " << input_cloud_new->size() << endl;
    cout << "=====================" << endl;
    //    for(int i = 0; i < smooth_vector.size(); i++) {
    //        if(i > smooth_vector_new.size())
    //            continue;
    //        if(smooth_vector[i].curvature != smooth_vector_new[i].curvature) {
    //            cout << "success" << endl;
    //            break;
    //        }
    //    }
    //    cout << " ======================" << endl;
    //    printf("segmentLineIndexNew.size() = %d \n segmentLineIndexNew[15] = %d \n segmentLineIndexNew[16] = %d \n ", segmentLineIndexNew.size(), segmentLineIndexNew[15], segmentLineIndexNew[16]);
    //  对所有特征点分段提取特征点
    // 附近的特征点不选取，使得选取特征点离散
    // sort(_smooth.begin(), _smooth.end(), [](SmoothData a, SmoothData b)-> bool {return a.curvature > b.curvature;});
    // 提取特征点，对每一段上一帧提取200个，这一帧提取20个，存储在cloud中

    pcl::PointCloud<pcl::PointXYZ>::Ptr keyPointCloudLast(new pcl::PointCloud<pcl::PointXYZ>);
    vector<SmoothData> keyPointCloudLastSmooth;
    pcl::PointCloud<pcl::PointXYZ>::Ptr keyPointCloudNew(new pcl::PointCloud<pcl::PointXYZ>);
    vector<SmoothData> keyPointCloudNewSmooth;
    vector<bool> cloudNeighborPicked(smooth_vector.size(), false);

    // DEBUG
    // cout << "DEBUG1 :  start get keypoint for new" << endl;

    /// 提取NEW点云中的点特征点
    for(int lineIndex = 0; lineIndex < 6; lineIndex++)  // 对每一行，每一次只搜索其中一段（共分为六个段）
    {
        // 记录当前部分已经找到的最大值
        // printf("DEBUG2 : lineIndex = %d\n ====================\n", lineIndex);
        for(int rowIndex = 0; rowIndex < N_SCAN; rowIndex++) {

//            printf("DEBUG2 : lineIndex = %d, rowIndex = %d\n", lineIndex, rowIndex);
            // 当前行开始位置
            int columnStart = segmentLineIndexNew[rowIndex];
            // 当前行结束位置
            int columnEnd = segmentLineIndexNew[rowIndex + 1];
            if(columnStart == columnEnd)
                continue;
            // 当前段开始位置
            int lineStart = columnStart + (columnEnd - columnStart) * lineIndex / 6;
            // 当前段结束位置
            int lineEnd = columnStart + (columnEnd - columnStart) * (lineIndex + 1) / 6 - 1;
            
//            printf("DEBUG3 : columnStart = %d, columnEnd = %d \n", columnStart, columnEnd);
//            printf("DEBUG3 : lineStart = %d, lineEnd = %d\n", lineStart, lineEnd);
            // 当前段进行排序
            sort(smooth_vector_new.begin() + lineStart, smooth_vector_new.begin() + lineEnd, [](SmoothData a, SmoothData b)-> bool {return a.curvature > b.curvature;});
            printf("===========================\n");
            
            // 找出其中最大的几个值
            int countKeyPointNumber = 0;
            for (int i = lineStart; i < lineEnd; i++)
            {
                int index = smooth_vector_new[i].index;
                // 如果特征点数量足够，则跳出循环
                if(countKeyPointNumber > 2) 
                    break;
                
                // 如果曲率较小或者周围被选取过，则跳过该特征点
                if(smooth_vector_new[i].curvature < 0.1 || cloudNeighborPicked[i] == true)
                    continue;
                
                // 如果上述条件都不满足，选择该特征点，存入点云中，并更新其周围点云标记
                countKeyPointNumber++;
                keyPointCloudNew->points.push_back(input_cloud_new->points.at(smooth_vector_new[i].index));
                keyPointCloudNewSmooth.push_back(smooth_vector_new[i]);
                for(int j = -10; j < 10; j++) {
                    int neighborIndex = (index + j + Horizon_SCAN) % Horizon_SCAN;
                    double diffX = input_cloud_new->points.at(neighborIndex).x - input_cloud_new->points.at(index).x;
                    double diffY = input_cloud_new->points.at(neighborIndex).y - input_cloud_new->points.at(index).y;
                    double diffZ = input_cloud_new->points.at(neighborIndex).z - input_cloud_new->points.at(index).z;

                    if(diffX * diffX + diffY * diffY + diffZ * diffZ > 0.1)
                        break;

                    cloudNeighborPicked[neighborIndex] = true;
                }
            }
            
        }
    }

    // DEBUG
    cout << "============================" << endl;
    cout << " finish get keypoint for new" << endl;
    cout << " total keypoint numer is " << keyPointCloudNew->size() << endl;
    cout << "============================" << endl;

    // 提取LAST点云中的特征点
    cloudNeighborPicked.clear();
    for(int lineIndex = 0; lineIndex < 6; lineIndex++)  // 对每一行，每一次只搜索其中一段（共分为六个段）
    {
        // 记录当前部分已经找到的最大值

        for(int rowIndex = 0; rowIndex < N_SCAN; rowIndex++) {
            // 当前行开始位置
            int columnStart = segmentLineIndex[rowIndex];
            // 当前行结束位置
            int columnEnd = segmentLineIndex[rowIndex + 1];

            if(columnEnd == columnStart) 
                continue;

            // 当前段开始位置
            int lineStart = columnStart + (columnEnd - columnStart) * lineIndex / 6;
            // 当前段结束位置
            int lineEnd = columnStart + (columnEnd - columnStart) * (lineIndex + 1) / 6 - 1;
            // 当前段进行排序
            sort(smooth_vector.begin() + lineStart, smooth_vector.begin() + lineEnd, [](SmoothData a, SmoothData b)-> bool {return a.curvature > b.curvature;});
            // 找出其中最大的几个值
            int countKeyPointNumber = 0;
            for (int i = lineStart; i < lineEnd; i++)
            {
                int index = smooth_vector[i].index;
                // 如果特征点数量足够，则跳出循环
                if(countKeyPointNumber > 20) 
                    break;
                
                // 如果曲率较小或者周围被选取过，则跳过该特征点
                if(smooth_vector[i].curvature < 0.1 || cloudNeighborPicked[i] == true)
                    continue;
                
                // 如果上述条件都不满足，选择该特征点，存入点云中，并更新其周围点云标记
                countKeyPointNumber++;
                keyPointCloudLast->points.push_back(input_cloud->points.at(smooth_vector[i].index));
                keyPointCloudLastSmooth.push_back(smooth_vector[i]);

                for(int j = -10; j < 10; j++) {
                    int neighborIndex = (index + j + Horizon_SCAN) % Horizon_SCAN;
                    double diffX = input_cloud->points.at(neighborIndex).x - input_cloud->points.at(index).x;
                    double diffY = input_cloud->points.at(neighborIndex).y - input_cloud->points.at(index).y;
                    double diffZ = input_cloud->points.at(neighborIndex).z - input_cloud->points.at(index).z;

                    if(diffX * diffX + diffY * diffY + diffZ * diffZ > 0.1)
                        break;

                    cloudNeighborPicked[neighborIndex] = true;
                }
            }
            
        }
    }

    // DEBUG
    cout << "============================" << endl;
    cout << " finish get keypoint for last" << endl;
    cout << " total keypoint numer is " << keyPointCloudLast->size() << endl;
    cout << "============================" << endl;

    // for (int i = 0; i < 200; i++)
    // {
    //     keyPointCloudLast->points.push_back(input_cloud->points.at(smooth_vector[i].index));    
    // }
    // for (int i = 0; i < 20; i++)
    // {
    //     keyPointCloudNew->points.push_back(input_cloud_new->points.at(smooth_vector_new[i].index));    
    // }
    
    cout << "=========================" << endl;
    cout << " key point get finish" << endl;
    cout << "key point new number = " << keyPointCloudNew->size() << endl;
    cout << "key point last number = " << keyPointCloudLast->size() << endl;
    cout << "+++++++++++++++++++++++++" << endl;

    // 记录最终取出的匹配用的特征点
    pcl::PointCloud<pcl::PointXYZ>::Ptr keyPointFindOutCloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 使用kdtree来寻找最近邻居
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
    kdTree.setInputCloud(keyPointCloudLast);
    ceres::Problem problem;
    double  partQ[] = {0.0, 0.0, 0.0, 1};
    double  partT[] = {0.0, 0.0, 0.0};
    problem.AddParameterBlock(partQ, 4, new ceres::EigenQuaternionParameterization);
    problem.AddParameterBlock(partT, 3);

    for(auto point : keyPointCloudNew->points) {
        pcl::Indices indices;
        vector<float> sqrDistance;
        int K = 1;
        indices.resize(K);
        sqrDistance.resize(K);
        kdTree.nearestKSearch(point, K, indices, sqrDistance);
        
        // TODO:优化这里！
        // 不是简单的选择最近的点作为对应的特征点进行优化
        // 加入额外的约束--最近的处在不同段上的点作为对应的特征点！（让其成为一条直线）
        // 记录最近点和次近点在input_cloud点云中INDEX
        int closestPointInd = -1;
        int minPointInd = -1; 

        // 如果最近点与当前搜索点距离足够近，则选择该点并计算次近点
        if(sqrDistance[0] < 25) {
            closestPointInd = keyPointCloudLastSmooth[indices[0]].index;
            int closestPointLineInd;
            double minPointDis = 25;

            for(int i = 0; i < segmentLineIndex.size(); i++) {
                if(segmentLineIndex[i] > closestPointInd) {
                    closestPointLineInd = i - 1;
                    break;
                }
            }
            
            // 向正向搜索
            for(int i = indices[0] + 1; i < keyPointCloudLast->size(); i++) {
                int searchPointIndex = keyPointCloudLastSmooth[i].index;
                int searchPointLineIndex = -1;
                // 计算当前所处行数
                for(int line = 0; line < segmentLineIndex.size(); line++) {
                    if(segmentLineIndex[line] > searchPointIndex) {
                        searchPointLineIndex = line - 1;
                        break;
                    }
                }

                // 超出范围则跳过该点
                if(searchPointLineIndex < 0)
                    continue;

                // 如果扫描位置超出附近行的范围，则结束搜索
                if(searchPointLineIndex > closestPointLineInd + 2.5)
                    break;

                // 如果在同一行则跳过
                if(searchPointLineIndex == closestPointLineInd)
                    continue;
                
                printf("DEBUG 458 : searchPointIndex = %d, closestPointInd = %d \n", searchPointIndex, closestPointInd);

                // 计算当前搜索点与最近点的距离
                float pointSqrDis = (input_cloud->at(searchPointIndex).x - input_cloud->at(closestPointInd).x) * (input_cloud->at(searchPointIndex).x - input_cloud->at(closestPointInd).x) +
                        (input_cloud->at(searchPointIndex).y - input_cloud->at(closestPointInd).y) * (input_cloud->at(searchPointIndex).y - input_cloud->at(closestPointInd).y) +
                        (input_cloud->at(searchPointIndex).z - input_cloud->at(closestPointInd).z) * (input_cloud->at(searchPointIndex).z - input_cloud->at(closestPointInd).z);
                
                if(pointSqrDis < minPointDis)
                {
                    // 找到更近的点，记录该点的位置,和距离
                    minPointDis = pointSqrDis;
                    minPointInd = searchPointIndex;
                }
            }
            // 向反向搜索
            for(int i = indices[0] - 1; i > 0 ; i--) {
                int searchPointIndex = keyPointCloudLastSmooth[i].index;
                int searchPointLineIndex = -1;
                // 计算当前所处行数
                for(int line = 0; line < segmentLineIndex.size(); line++) {
                    if(segmentLineIndex[line] > searchPointIndex) {
                        searchPointLineIndex = line - 1;
                        break;
                    }
                }

                // 超出范围则跳过该点
                if(searchPointLineIndex < 0)
                    continue;

                // 如果扫描位置超出附近行的范围，则结束搜索
                if(searchPointLineIndex > closestPointLineInd + 2.5)
                    break;

                // 如果在同一行则跳过
                if(searchPointLineIndex == closestPointLineInd)
                    continue;
                
                // 计算当前搜索点与最近点的距离
                float pointSqrDis = (input_cloud->at(searchPointIndex).x - input_cloud->at(closestPointInd).x) * (input_cloud->at(searchPointIndex).x - input_cloud->at(closestPointInd).x) +
                        (input_cloud->at(searchPointIndex).y - input_cloud->at(closestPointInd).y) * (input_cloud->at(searchPointIndex).y - input_cloud->at(closestPointInd).y) +
                        (input_cloud->at(searchPointIndex).z - input_cloud->at(closestPointInd).z) * (input_cloud->at(searchPointIndex).z - input_cloud->at(closestPointInd).z);

                if(pointSqrDis < minPointDis)
                {
                    // 找到更近的点，记录该点的位置,和距离
                    minPointDis = pointSqrDis;
                    minPointInd = searchPointIndex;
                }
            }
        }

        // DEBUG 
        cout << "++++++++++++++++++++++++++++" << endl;
        cout << "closestPointInd = " << closestPointInd << endl;
        cout << "minPointInd = " << minPointInd << endl;
        cout << "++++++++++++++++++++++++++++" << endl;

        if(closestPointInd >= 0 && minPointInd >= 0) {
            // 把最近点和次近点仍入点云中
            keyPointFindOutCloud->points.push_back(input_cloud->points[closestPointInd]);
            keyPointFindOutCloud->points.push_back(input_cloud->points[minPointInd]);
            Eigen::Vector3f currPoint(point.x, point.y, point.z);
        
            Eigen::Vector3f lastPointA(
                input_cloud->points[closestPointInd].x,
                input_cloud->points[closestPointInd].y,
                input_cloud->points[closestPointInd].z
            );
            Eigen::Vector3f lastPointB(
                input_cloud->points[minPointInd].x,
                input_cloud->points[minPointInd].y,
                input_cloud->points[minPointInd].z
            );

            ceres::CostFunction * keyPointCost = new ceres::AutoDiffCostFunction<KeyPointCostFunction, 3, 4, 3> 
                                                    (new KeyPointCostFunction(currPoint, lastPointA, lastPointB));
            // 利用Ceres构建最小二乘问题，优化位姿
            problem.AddResidualBlock(keyPointCost, NULL, partQ, partT);


            // for(int i = 0 ; i < indices.size(); i++){
            //     cout << indices[i]<<"-" << sqrDistance[i] << endl;
            // }
        }
        
    }
    
    //配置problem的solver
    cout << "problem.NumResiduals = "<< problem.NumResiduals() << endl;
    cout << "problem.NumResidualBlocks = " << problem.NumResidualBlocks() << endl;
    cout << "problem.NumParameters = " << problem.NumParameters() << endl;
    cout << "problem.NumParameterBlocks = " << problem.NumParameterBlocks() << endl;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 25;
    options.minimizer_progress_to_stdout = true;
    
    // 求解最小二乘问题
    ceres::Solve(options, &problem, &summary);
    // 打印结果
    cout << "=========================" << endl;
    cout << "ceres ouput " << endl;
    cout << "=========================" << endl;
    cout << summary.BriefReport() << endl;
    cout << "=========================" << endl;
    cout << endl;
    cout << "[q,t]" << endl;
    cout << "=========================" << endl;
    printf("q = [%f, %f, %f, %f]\n ", partQ[0], partQ[1], partQ[2], partQ[3]);
    printf("t = [%f, %f, %f\n", partT[0], partT[1], partT[2]);
    cout << "=========================" << endl;

    // 生成旋转矩阵
    Eigen::Quaternion<float> rotationQuaternion(partQ[3], partQ[0], partQ[1], partQ[2]);
    Eigen::Matrix<float, 4, 4> transformMatrix = Eigen::Matrix<float, 4, 4>::Identity();
    Eigen::Vector3f ea = rotationQuaternion.toRotationMatrix().eulerAngles(2, 1, 0);
    transformMatrix.block<3, 3>(0, 0) = rotationQuaternion.matrix();
    transformMatrix(0, 3) = partT[0];
    transformMatrix(1, 3) = partT[1];
    transformMatrix(2, 3) = partT[2];
    cout << transformMatrix << endl;
    cout << "===================" << endl;
    cout << ea << endl;
    cout << "===================" << endl;
    // 利用PCL transformationpointcloud 旋转
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputKeyPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*input_cloud_new, *outputCloud, transformMatrix); 
    pcl::transformPointCloud(*keyPointCloudNew, *outputKeyPointCloud, transformMatrix);

    // visualization config 
    int v1 = 0, v2 = 1, v3 = 2, v4 = 3;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  keyPointCloudNewColor(keyPointCloudNew, 115, 115, 32);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  keyPointCloudLastColor(keyPointCloudLast, 185, 115, 115);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  outputKeyPointCloudColor(keyPointCloudLast, 185, 85, 225);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  findoutKeyPointCloudColor(keyPointFindOutCloud, 185, 85, 34);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  lastCLoudColor(outputCloud, 231, 23, 123);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  inputCLoudColor(input_cloud, 200, 223, 43);
    viewer.createViewPort(0, 0.5, 0.5, 1, v1);
    viewer.createViewPort(0.5, 0.5, 1, 1, v2);
    viewer.createViewPort( 0, 0, 0.5, 0.5, v3);
    viewer.createViewPort(0.5, 0, 1, 0.5, v4);

    // v1中为上一帧与匹配后一帧
    viewer.addPointCloud(input_cloud,"cloud",v1);
    viewer.addPointCloud(outputCloud, lastCLoudColor, "cloud_o_in", v1);
    viewer.addText("lastCloud and outCloud", 10, 10, "v1Text", v1);

    // v2中为上一帧与当前帧点云
    viewer.addText("lastCloud and nowCloud", 10, 10, "v2Text", v2);
    viewer.addPointCloud(input_cloud, inputCLoudColor, "cloud_output",v2);
    viewer.addPointCloud(input_cloud_new, "cloud_new",v2);

    // v3中为上一帧特征点与当前帧特征点
    viewer.addPointCloud(keyPointCloudLast, keyPointCloudLastColor, "v3_cloud_keypoint_last", v3);
    viewer.addPointCloud(keyPointCloudNew, keyPointCloudNewColor, "v3_cloud_keypoint_new", v3);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "v3_cloud_keypoint_new");
    viewer.addPointCloud(keyPointFindOutCloud, findoutKeyPointCloudColor, "v3_cloud_keypoint_findout", v3);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "v3_cloud_keypoint_findout");
    viewer.addText("lastKeyPoint and curKeyPoint", 10, 10, "v3Text", v3);

    // v4中为上一帧特征点与变换后特征点
    viewer.addPointCloud(keyPointCloudLast, keyPointCloudLastColor, "v4LastCLoud", v4);
    viewer.addPointCloud(outputKeyPointCloud, outputKeyPointCloudColor, "v4output", v4);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "v4output");
    viewer.addText("lastKeyPoint and outputKeyPoint", 10, 10, "v4Text", v4);
    // viewer.addPointCloud(keyPointCloudNew,keyPointCloudNewColor,"keynew", v1);
    // viewer.addPointCloud(keyPointCloudLast,keyPointCloudLastColor,"keylast", v2);


    // for(int i = 0; i < 100; i++) {
    //     string str = "point" + to_string(i);
    //     // cout << str << endl;
    //     // cout << smooth_vector[i].index << endl;
    //     viewer.addSphere(input_cloud.get()->points.at(smooth_vector[i].index),0.25, str, v1);
    //     viewer.addSphere(input_cloud.get()->points.at(smooth_vector[smooth_vector.size() - 1 - i].index),0.25, str + "smo", v1);
    //     viewer.addSphere(input_cloud_new.get()->points.at(smooth_vector_new[i].index),0.25, str + "new", v2);
    // }
    
    // printf("ouput size = %d\n", ri.getValidPointQuantity());
    // printf("ouput - input = %d\n", int(input_cloud.get()->size() - ri.getValidPointQuantity()));
    while(!viewer.wasStopped()) {
        viewer.spinOnce();
        std::this_thread::sleep_for(1ms);
    }
    return 0;
}