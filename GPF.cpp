#include "GPF.h"

int GPF::filter(pcl::PointCloud<pcl::PointXYZ>& all_cloud_not_ground, pcl::PointCloud<pcl::PointXYZ>& all_cloud_ground)
{
    using vector_point = vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>;
    using point_cloud_type = pcl::PointCloud<pcl::PointXYZ>;
    using point_cloud_ptr = point_cloud_type::Ptr;

    auto t1 = chrono::system_clock::now();
        // 将输入点云按照x方向划分为不同组数据
        double div_threat = 2 * M_PI / static_cast<double>(div_segment_num);
        cout << "div_threat = " << div_threat << endl;

        // 初始化div_cloud_v
        // vector<point_cloud_ptr> cloud_v;
        vector<unique_ptr<vector_point>> div_cloud_v;
        for (size_t i = 0; i < div_segment_num; i++)
        {
            div_cloud_v.push_back(unique_ptr<vector_point>(new vector_point));
            cout << div_cloud_v[i].get() << endl;
        }

        // 将输入点云分割为不同组并存入到向量div_cloud_v中
        for(auto point : input_cloud_->points)
        {
            double threat;
            int temp;
            threat = atan2(point.y, point.x);
            if(threat < 0)
                threat += 2 * M_PI;
            // cout << threat << endl;
            temp = static_cast<int>(threat / div_threat);
            //cout << temp << endl;
            div_cloud_v[temp].get()->push_back(point);
        }
        
        
        
        // visual test
        // for (size_t i = 0; i < div_segment_num; i++)
        // {
        //     point_cloud_ptr cloud (new point_cloud_type);
        //     cloud->points = *div_cloud_v[i].get();
        //     cout << cloud->points.size() << endl;
        //     pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> signle_color(cloud);
        //     cout << "cloud" + to_string(i) <<endl;
        //     viewer.addPointCloud(cloud, signle_color, ("cloud" + to_string(i)) );
        //     viewer.spinOnce();
        //     std::this_thread::sleep_for(1ms);
        // }

        // pcl::PointCloud<pcl::PointXYZ>::Ptr all_cloud_ground(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr all_cloud_not_ground(new pcl::PointCloud<pcl::PointXYZ>);

        //对每一个segment中的点云数据进行地面滤波
        auto iter = div_cloud_v.begin();
        while (iter != div_cloud_v.end())
        {
            auto point_p = iter->get();
            vector_point seed_v;
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_not_ground(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            // 对点云按照高度进行排序
            sort(point_p->begin(), point_p->end(), [](pcl::PointXYZ a, pcl::PointXYZ b){return a.z < b.z;});

            // 计算点云的平均高度
            double mean_z = 0;
            int temp = static_cast<size_t>(point_p->size() * N_LPR);
            for (size_t i = 0; i < temp ; i++)
                mean_z += (*point_p)[i].z;

            mean_z /= temp;
            
            //计算初始种子
            for (size_t i = 0; i< point_p->size(); i++) {
                if((*point_p)[i].z < (mean_z + Th_seeds)) {
                    // seed_v.push_back((*point_p)[i]);
                    cloud_ground->points.push_back((*point_p)[i]);
                }else{
                    cloud_not_ground->points.push_back((*point_p)[i]);
                }
            }

            //main loop
            for (size_t i = 0; i < N_iter; i++)
            {
                // Estimateplane(P_g),利用RANSAC估计地面模型

                pcl::SACSegmentation<pcl::PointXYZ> seg;
                seg.setModelType (pcl::SACMODEL_PLANE);
                seg.setMethodType (pcl::SAC_RANSAC);
                seg.setDistanceThreshold (0.01);
                seg.setInputCloud(cloud_ground);
                seg.segment(*inliers, *model);

                if(inliers->indices.size() == 0){
                    PCL_ERROR("Could not estimate a planar model for the given dataset.");
                    return -1;
                }

                // cout << "main loop" << i << endl;
                // cout<< "model" << model->values[0] << endl
                //                 << model->values[1] << endl
                //                 << model -> values[2] << endl
                //                 << model -> values[3] << endl;

                

                //清空cloud_ground和cloud_not_ground
                cloud_ground->points.erase(cloud_ground->points.begin(),cloud_ground->points.end());
                cloud_not_ground->points.erase(cloud_not_ground->points.begin(),cloud_not_ground->points.end());

                //重新计算地面点云以及非地面点云
                
                for (size_t i = 0; i < point_p->size(); i++)
                {
                    //计算点到平面的距离
                    double distance_from_point_to_plane;
                    distance_from_point_to_plane = abs( (model->values[0]) * ((*point_p)[i].x ) + 
                                                        (model->values[1]) * ((*point_p)[i].y ) +
                                                        (model->values[2]) * ((*point_p)[i].z ) +
                                                        (model->values[3])
                                                        );
                    //根据距离判断是否是地面点
                    if(distance_from_point_to_plane < Th_dist) {
                        cloud_ground->points.push_back((*point_p)[i]);
                    }else {
                        cloud_not_ground->points.push_back((*point_p)[i]);
                    }
                }
                
                //判断异常点
                Eigen::Vector3d axis(0, 0, 1);
                Eigen::Vector3d n(model->values[0], model->values[1], model->values[2]);
                double v_cos = axis.dot(n)/(axis.norm() * n.norm());
                double v_threat = acos(v_cos) * 180 /M_PI;
                 cout <<"iter = "<< i << " | v_cos = " << v_cos << " | v_threat = " << v_threat <<endl;
                // 如果检测到底地面法向量与z轴夹角过大，该组数据停止查找地面。
                if(v_threat > 60) {
                    *cloud_not_ground += *cloud_ground;
                    cloud_ground->points.erase(cloud_ground->points.begin(),cloud_ground->points.end());
                    break;
                }
            }
            
            //更新地面点云和非地面点云
            all_cloud_ground += *cloud_ground;
            all_cloud_not_ground += *cloud_not_ground;
            iter++;
        }
        cout << all_cloud_ground.points.size() <<endl;
        auto t2 = chrono::system_clock::now();
        auto time = chrono::duration_cast<chrono::milliseconds>(t2 - t1);
        return 0;
}

