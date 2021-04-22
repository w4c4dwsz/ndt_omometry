#include "utility.h"

DEFINE_bool(real_data, true, "if use the real_data");
DEFINE_double(t_x, 0, "the translate move on x axis");
DEFINE_double(t_y, 0, "the translate move on y axis");
DEFINE_double(t_z, 0, "the translate move on z axis");
DEFINE_double(pitch, 0, "pitch-x");
DEFINE_double(yaw, 0, "yaw-y");
DEFINE_double(roll, 0, "roll-z");
DEFINE_double(TransformationEpsilon, 0.01, "setTransformationEpsilon");
DEFINE_double(Resolution, 1, "setResolution");
DEFINE_double(targetFileID, 0, "the pcd file id for the target cloud");
DEFINE_double(sourceFileID, 1, "the pcd file id for the source cloud");
DEFINE_double(MaximumIterations, 35, "the MaximumIterations");

int main(int argc, char** argv)
{
    
    // 处理输入
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    std::vector<google::CommandLineFlagInfo> flags;
    google::GetAllFlags(&flags);
    
    for (const auto& flag : flags) {
        // cout << "#" << flag.type << ", default = " << flag.default_value << "\n"
        //      << "#" << flag.description << "\n"
        cout << "--" << flag.name << "=" << flag.current_value << endl;
    }
    // 获取文件列表
    std::ifstream fout;
    std::vector<string> pcd_filename_v;
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
        pcd_filename_v.push_back(file_name);
    }
    
    cout << " get the file name finish" << endl; 
    
    
    pcl::visualization::PCLVisualizer viewer("viewer");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
    viewer.initCameraParameters();
    viewer.addPointCloud(cloud_p, "cloud");
    for (auto file_name : pcd_filename_v)
    {   
        cout << file_name << endl;
        pcl::io::loadPCDFile("../data/" + file_name, *cloud_p);
        viewer.updatePointCloud(cloud_p);
        viewer.spinOnce(0.0000000000001);
        std::this_thread::sleep_for(chrono::duration<int, std::ratio<1,1>>(1));
    }
    

    
    while (viewer.wasStopped())
    {   
        viewer.spin();

    }
    return 0;
}