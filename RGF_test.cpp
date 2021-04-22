#ifndef _RGF_
#define _RGF_
#include "RGF.h"
#endif

bool visualIsStoped = false;
void visualCallback(const pcl::visualization::KeyboardEvent &event, void * );

int main(int argc, char** argv){
    
    RGF IP;
    pcl::visualization::PCLVisualizer viewer("hello world");

    // 获取文件列表
    std::ifstream fout;
    std::queue<string> pcd_filename_v;
    fout.open("../data/list");

    if(!fout.is_open())
    {
        cout << "can't open file list!" <<endl;
        return -1;
    }

    cout << " get the file name start" << endl;
    while(!fout.eof())
    {
        string file_name;
        fout >> file_name;
        // cout << file_name <<endl;
        if(!file_name.empty())
            pcd_filename_v.push(file_name);        
    }

    while(!pcd_filename_v.empty() && !viewer.wasStopped())
    {   
        if(visualIsStoped)
        {
            viewer.spinOnce();
            std::this_thread::sleep_for(1ms);
            continue;
        }
        viewer.removeAllPointClouds();
            

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud_not_ground(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud_ground(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>());
        string pcd_filename = pcd_filename_v.front();
        pcd_filename_v.pop();
        // cout << "the pcd file process now is " << pcd_filename <<endl;

        //读取PCD
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if(pcl::io::loadPCDFile("../data/" + pcd_filename, *cloud))
        {
            cout<<"loadPCDFile "<< pcd_filename <<"falut"<< endl;
            continue;
        }

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    
        cout << "cloud num" << cloud->size()<<endl;

        for(auto p :cloud->points){
            pcl::PointXYZI p_i;
            p_i.x = p.x;
            p_i.y = p.y;
            p_i.z = p.z;
            p_i.intensity = 0;
            //cout << p_i << endl;
            cloud_i->push_back(p_i);
        }
        auto t1 = chrono::system_clock::now();
        cout << "cloud num" << cloud_i->size()<<endl;
        IP.setInputCloud(cloud_i);
        IP.filter(*all_cloud_ground, *all_cloud_not_ground);
        auto t2 = chrono::system_clock::now();
        

        IP.cloudSegmentation();
        auto t3 = chrono::system_clock::now();

        auto filter_time = chrono::duration_cast<chrono::milliseconds>(t2 - t1);
        auto segment_time = chrono::duration_cast<chrono::milliseconds>(t3 - t2);
        cout << "filter_time = " << filter_time.count() << endl;
        cout << "segment_time  = " << segment_time.count() << endl;

        //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> all_cloud_not_ground_color_h(all_cloud_not_ground, 0, 255, 0);
        //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> all_cloud_ground_color_h(all_cloud_ground, 255, 105, 180);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_rgb);
        //viewer.addPointCloud(all_cloud_not_ground, all_cloud_not_ground_color_h, "not-ground");
        //viewer.addPointCloud(all_cloud_ground, all_cloud_ground_color_h, "ground");      
        IP.visualzation(*cloud_rgb);
        cout << cloud_rgb->size() << endl;
        
        viewer.registerKeyboardCallback(&visualCallback,(void*)NULL);
        
        
        viewer.addPointCloud(cloud_rgb, rgb, "segment");

        
        viewer.spinOnce();
        
        std::this_thread::sleep_for(10ms);

        
        

    }
    
    cout << "the pcd file process finish" << endl;
    viewer.close();
    return 0;
}

void visualCallback(const pcl::visualization::KeyboardEvent &event, void * cookies){
    if(event.keyDown() && event.getKeySym() == "space")
        visualIsStoped = !visualIsStoped;  
}