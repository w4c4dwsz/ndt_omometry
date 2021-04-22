#ifndef _RGF_
#define _RGF_
#include "RGF.h"
#endif
extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 2000;
extern const float ang_res_x = 0.18;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0+0.1;
extern const int groundScanInd = 7;

extern const float sensorMinimumRange = 1.0;
extern const float sensorMountAngle = 0.0;
extern const float segmentTheta = 15.0/180.0*M_PI; // decrese this value may improve accuracy
extern const int segmentValidPointNum = 5;
extern const int segmentValidLineNum = 3;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


extern const bool loopClosureEnableFlag = false;
extern const double mappingProcessInterval = 0.3;

extern const float scanPeriod = 0.1;
extern const int systemDelay = 0;
extern const int imuQueLength = 200;

    
RGF::RGF()
{
    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;

    nanPointRGB.x = std::numeric_limits<float>::quiet_NaN();
    nanPointRGB.y = std::numeric_limits<float>::quiet_NaN();
    nanPointRGB.z = std::numeric_limits<float>::quiet_NaN();
    nanPointRGB.r = 0;
    nanPointRGB.g = 0;
    nanPointRGB.b = 0;

    allocateMemory();
    resetParameters();
}

void RGF::allocateMemory(){

    laserCloudIn.reset(new pcl::PointCloud<PointType>());
    

    fullCloud.reset(new pcl::PointCloud<PointType>());
    fullInfoCloud.reset(new pcl::PointCloud<PointType>());
    fullCLoudRGB.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    groundCloud.reset(new pcl::PointCloud<PointType>());
    noGroundCloud.reset(new pcl::PointCloud<PointType>());
    segmentedCloud.reset(new pcl::PointCloud<PointType>());
    segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
    outlierCloud.reset(new pcl::PointCloud<PointType>());

    fullCloud->points.resize(N_SCAN*Horizon_SCAN);
    fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);
    fullCLoudRGB->points.resize(N_SCAN*Horizon_SCAN);

    std::pair<int8_t, int8_t> neighbor;
    neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
    neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
    neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
    neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

    allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
    allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

    queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
    queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];
}

void RGF::resetParameters(){
    laserCloudIn->clear();
    groundCloud->clear();
    noGroundCloud->clear();
    segmentedCloud->clear();
    segmentedCloudPure->clear();
    outlierCloud->clear();

    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
    labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
    labelCount = 1;

    std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
    std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    std::fill(fullCLoudRGB->points.begin(), fullCLoudRGB->points.end(), nanPointRGB);
}

void RGF::projectPointCloud(){
    // range image projection
    float verticalAngle, horizonAngle, range;
    size_t rowIdn, columnIdn, index, cloudSize; 
    PointType thisPoint;

    cloudSize = laserCloudIn->points.size();

    for (size_t i = 0; i < cloudSize; ++i){

        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;
        // find the row and column index in the iamge for this point
        
        // 计算垂直角度
        verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
        //ang_res_y = 2.0; ang_bottom = 15.0+0.1;
        //其中ang_res_y是作者使用的激光雷达的垂直解析度，ang_bottom是激光雷达的负扫描范围，将垂直角度从-15到15转换为0到30度后再除以垂直解析度
        rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
        // cout << "rowIdn" << rowIdn << endl;
        // N_SCAN 为垂直高度最大值，对异常点进行处理
        if (rowIdn < 0 || rowIdn >= N_SCAN)
            continue;

        //计算水平角度
        horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
        /**
         *    Horizon_SCAN = 1800
         *    https://gutsgwh1997.github.io/2020/07/31/LeGo-LOAM%E4%B8%AD%E7%9A%84ImageProjection%E8%8A%82%E7%82%B9/
         **/
        columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
        
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;
        
        // cout << " columnIdn = " << columnIdn << endl;

        if (columnIdn < 0 || columnIdn >= Horizon_SCAN){
            cout << " columnIdn = " << columnIdn << endl;
            continue;
        }
            

        // 计算点到激光雷达的距离R（x^2+y^2+z^2)
        range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
        // cout << "range = " << range << endl;
        if (range < sensorMinimumRange)
            continue;
        // 装填rangeMat
        rangeMat.at<float>(rowIdn, columnIdn) = range;


        thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;
        
        // 计算该点在所有点云中的索引index, 并且填充到fullCloud和fullInfoCloud中
        index = columnIdn  + rowIdn * Horizon_SCAN;
        fullCloud->points[index] = thisPoint;
        fullInfoCloud->points[index] = thisPoint;
        fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
    }
    //cout << "RGF range project raw point size = " << cloudSize << endl;
    //cout << "RGF range project new point size = " << fullInfoCloud->size() << endl;
    //cout << rangeMat.size() << endl;
}

void RGF::groundRemoval(){          
    size_t lowerInd, upperInd;
    float diffX, diffY, diffZ, angle;
    float lowerZ;
    
    // groundMat
    // -1, no valid info to check if ground of not
    //  0, initial value, after validation, means not ground
    //  1, ground
    for (size_t j = 0; j < Horizon_SCAN; ++j){
        for (size_t i = 0; i < (groundScanInd); ++i){

            lowerInd = j + ( i )*Horizon_SCAN;
            upperInd = j + (i+1)*Horizon_SCAN;

            if (fullCloud->points[lowerInd].intensity == -1 ||
                fullCloud->points[upperInd].intensity == -1){
                // no info to check, invalid points
                groundMat.at<int8_t>(i,j) = -1;
                continue;
            }
                
            diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
            diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
            diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;
            lowerZ = fullCloud->points[lowerInd].z;
            angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;
            
            if (abs(angle - sensorMountAngle) <= 7.5 ){
                groundMat.at<int8_t>(i,j) = 1;
                groundMat.at<int8_t>(i+1,j) = 1;
                
            }
        }
    }
    // extract ground cloud (groundMat == 1)
    // mark entry that doesn't need to label (ground and invalid point) for segmentation
    // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
    for (size_t i = 0; i < N_SCAN; ++i){
        for (size_t j = 0; j < Horizon_SCAN; ++j){
            if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX){
                labelMat.at<int>(i,j) = -1;
            }
        }
    }
    for (size_t i = 0; i < N_SCAN; ++i){
        for (size_t j = 0; j < Horizon_SCAN; ++j){
            if (groundMat.at<int8_t>(i,j) == 1)
                groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
            else
                noGroundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
        }
    }

    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // // Create the segmentation object
    // pcl::SACSegmentation<pcl::PointXYZI> seg;
    // // Optional
    // seg.setOptimizeCoefficients (true);
    // // Mandatory
    // seg.setModelType (pcl::SACMODEL_PLANE);
    // seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setDistanceThreshold (0.01);

    // seg.setInputCloud (groundCloud);
    // seg.segment (*inliers, *coefficients);

    // cout<< coefficients-> values[0] << endl;
    // cout<< coefficients-> values[1] << endl;
    // cout<< coefficients-> values[2] << endl;
    // cout<< coefficients-> values[3] << endl;


    // for (size_t i = 0; i < N_SCAN; ++i){
    //     for (size_t j = 0; j < Horizon_SCAN; ++j){
    //         double distance_from_point_to_plane;
    //             distance_from_point_to_plane = abs( (coefficients->values[0]) * ( fullCloud->points[j + i*Horizon_SCAN].x ) + 
    //                                                 (coefficients->values[1]) * ( fullCloud->points[j + i*Horizon_SCAN].y ) +
    //                                                 (coefficients->values[2]) * ( fullCloud->points[j + i*Horizon_SCAN].z ) +
    //                                                 (coefficients->values[3])
    //                                             );

    //         // cout << "distance_from_point_to_plane = " << distance_from_point_to_plane << endl;
    //         if (distance_from_point_to_plane <= 0.01 && groundMat.at<int8_t>(i,j) == 1)
    //             groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
    //         else
    //             noGroundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
    //     }
    // }

    
}

void RGF::filter(pcl::PointCloud<PointType> &ground_cloud, pcl::PointCloud<PointType> &no_ground_cloud){
    groundCloud->clear();
    noGroundCloud->clear();
    segmentedCloud->clear();
    segmentedCloudPure->clear();
    outlierCloud->clear();
    fullCLoudRGB->clear();

    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
    labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
    labelCount = 1;

    std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
    std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);

    projectPointCloud();
    groundRemoval();
    //cout << "groundCloud" << groundCloud->size() << endl;
    //cout << "nogroundCloud" << noGroundCloud->size() << endl;
    //cout << "total cloud number = " << groundCloud->size() + noGroundCloud->size() << endl;
    ground_cloud = *groundCloud;
    no_ground_cloud = *noGroundCloud;
}

void RGF::labelComponents(int row, int col){
    // use std::queue std::vector std::deque will slow the program down greatly
    float d1, d2, alpha, angle;
    int fromIndX, fromIndY, thisIndX, thisIndY; 
    bool lineCountFlag[N_SCAN] = {false};

    queueIndX[0] = row;
    queueIndY[0] = col;
    int queueSize = 1;
    int queueStartInd = 0;
    int queueEndInd = 1;

    allPushedIndX[0] = row;
    allPushedIndY[0] = col;
    int allPushedIndSize = 1;
    
    while(queueSize > 0){
        // Pop point
        fromIndX = queueIndX[queueStartInd];
        fromIndY = queueIndY[queueStartInd];
        --queueSize;
        ++queueStartInd;
        // Mark popped point
        labelMat.at<int>(fromIndX, fromIndY) = labelCount;
        // Loop through all the neighboring grids of popped grid
        for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){
            // new index
            thisIndX = fromIndX + (*iter).first;
            thisIndY = fromIndY + (*iter).second;
            // index should be within the boundary
            if (thisIndX < 0 || thisIndX >= N_SCAN)
                continue;
            // at range image margin (left or right side)
            if (thisIndY < 0)
                thisIndY = Horizon_SCAN - 1;
            if (thisIndY >= Horizon_SCAN)
                thisIndY = 0;
            // prevent infinite loop (caused by put already examined point back)
            if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                continue;

            d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                            rangeMat.at<float>(thisIndX, thisIndY));
            d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                            rangeMat.at<float>(thisIndX, thisIndY));

            if ((*iter).first == 0)
                alpha = segmentAlphaX;
            else
                alpha = segmentAlphaY;

            angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

            if (angle > segmentTheta){

                queueIndX[queueEndInd] = thisIndX;
                queueIndY[queueEndInd] = thisIndY;
                ++queueSize;
                ++queueEndInd;

                labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                lineCountFlag[thisIndX] = true;

                allPushedIndX[allPushedIndSize] = thisIndX;
                allPushedIndY[allPushedIndSize] = thisIndY;
                ++allPushedIndSize;
            }
        }
    }

    // check if this segment is valid
    bool feasibleSegment = false;
    if (allPushedIndSize >= 30)
        feasibleSegment = true;
    else if (allPushedIndSize >= segmentValidPointNum){
        int lineCount = 0;
        for (size_t i = 0; i < N_SCAN; ++i)
            if (lineCountFlag[i] == true)
                ++lineCount;
        if (lineCount >= segmentValidLineNum)
            feasibleSegment = true;            
    }
    // segment is valid, mark these points
    if (feasibleSegment == true){
        ++labelCount;
    }else{ // segment is invalid, mark these points
        for (size_t i = 0; i < allPushedIndSize; ++i){
            labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
        }
    }
}

void RGF::cloudSegmentation(){
    // segmentation process
    for (size_t i = 0; i < N_SCAN; ++i)
        for (size_t j = 0; j < Horizon_SCAN; ++j)
            if (labelMat.at<int>(i,j) == 0)
                labelComponents(i, j);
    
}

/**
 * @brief use to visualization
 * @param cloud [in] : input point cloud
 */
void RGF::visualzation(pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
    cout <<"visualzation test" << endl;
    std::default_random_engine engine;
    std::uniform_int_distribution<std::uint8_t> rgbGenerate(100,255);
    cout << "labelCount"<<labelCount << endl;

    cloud.clear();
    cloud.resize(N_SCAN*Horizon_SCAN);

    size_t RGBNum = 10;
    std::uint8_t a[RGBNum][3];
    for (size_t i = 0; i < RGBNum; i++)
    {   int R = rgbGenerate(engine);
        int G = rgbGenerate(engine);
        int B = rgbGenerate(engine);
        a[i][0] = R;
        a[i][1] = G;
        a[i][2] = B;
    }
    
    int groundPointCount = 0;
    for(size_t i = 0; i < N_SCAN; i++) {
        for( size_t j = 0; j < Horizon_SCAN; j++) {
            
            int label = labelMat.at<int>(i,j);
            int colorLabel = label % RGBNum;
            if(label != 999999 && label != -1) {
                
                pcl::PointXYZRGB point;
                point.x = fullCloud->points.at(j + i * Horizon_SCAN).x;
                point.y = fullCloud->points[j + i * Horizon_SCAN].y;
                point.z = fullCloud->points[j + i * Horizon_SCAN].z;
                point.r = a[colorLabel][0];
                point.g = a[colorLabel][1];
                point.b = a[colorLabel][2];
                cloud.points.push_back(point);
                continue;
            }

            // if(groundMat.at<int8_t>(i, j) == 1)
            // {   
            //     pcl::PointXYZRGB point;
            //     point.x = fullCloud->points.at(j + i * Horizon_SCAN).x;
            //     point.y = fullCloud->points[j + i * Horizon_SCAN].y;
            //     point.z = fullCloud->points[j + i * Horizon_SCAN].z;
            //     point.r = static_cast<uint8_t>(0);
            //     point.g = static_cast<uint8_t>(0);
            //     point.b = static_cast<uint8_t>(255);
            //     cloud.points.push_back(point);
            //     groundPointCount ++;
            //       continue;
            // }
            
        }
    }
    cout << "this is " << endl;
    cout << "ground point number = " << groundPointCount << endl;
    
    //viewer.spinOnce();
    cout << "finish" << endl;
    return;
}

void RGF::getSegmentationCloud(pcl::PointCloud<pcl::PointXYZ> & segmentationCloud, cv::Mat &tagMat, vector<int> &segmentLineIndex,
                                pcl::PointCloud<pcl::PointXYZ> & groundCloudPoint, vector<int> &groundIndex) {
    groundCloud->clear();
    noGroundCloud->clear();
    segmentedCloud->clear();
    segmentedCloudPure->clear();
    outlierCloud->clear();
    fullCLoudRGB->clear();
    segmentationCloud.clear();

    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
    labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
    labelCount = 1;

    std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
    std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);

    projectPointCloud();
    groundRemoval();
    cloudSegmentation();
    // 将点云图像封装为向量

    int segmentCount = 0;
    
    for (int i = 0; i < N_SCAN; i++) {
        // printf("i = %d\n", i);
        segmentLineIndex.push_back(segmentCount);
        // segmentLineIndex.push_back(segmentCount);
        for (int j = 0; j < Horizon_SCAN; j++) {
            int label = labelMat.at<int>(i,j);
            // if(i == 15)
            //         printf("enter 15 \n");
            //         printf("label = %d", label);
            if(label != 999999 && label != -1) {
                if(i == 15)
                    printf("enter 15 \n");
                
                pcl::PointXYZ point;
                point.x = fullCloud ->at(j + i * Horizon_SCAN).x;
                point.y = fullCloud ->at(j + i * Horizon_SCAN).y;
                point.z = fullCloud ->at(j + i * Horizon_SCAN).z;
                segmentationCloud.points.push_back(point);
                segmentCount++;
                
            }
        }
        printf("i = %d, \n", i);
        printf("segmentCount = %d\n", segmentCount);
    }
    segmentLineIndex.push_back(segmentCount);
    labelMat.copyTo(tagMat);

    int groundCount = 0;
    for (int i = 0; i < N_SCAN; i++) {
        // printf("i = %d\n", i);
        groundIndex.push_back(groundCount);

        for (int j = 0; j < Horizon_SCAN; j++) {
            int label = labelMat.at<int>(i,j);
            // if(i == 15)
            //         printf("enter 15 \n");
            //         printf("label = %d", label);
            if(label == -1) {
                if (i == 15)
                    printf("enter 15 \n");

                pcl::PointXYZ point;
                point.x = fullCloud->at(j + i * Horizon_SCAN).x;
                point.y = fullCloud->at(j + i * Horizon_SCAN).y;
                point.z = fullCloud->at(j + i * Horizon_SCAN).z;
                groundCloudPoint.points.push_back(point);
                groundCount++;
            }
        }
    }

    groundIndex.push_back(groundCount);  
    
}