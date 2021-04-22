#include "utility.h"

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
    
    //TODO: 增加文件载入时候的判断
    string target_file_name = "../data/" + pcd_filename_v[FLAGS_targetFileID];
    string source_file_name = "../data/" + pcd_filename_v[FLAGS_sourceFileID];

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_p(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_p(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_p(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_p(new pcl::PointCloud<pcl::PointXYZ>);

    // 准备模拟数据
    pcl::io::loadPCDFile(target_file_name, *target_p);
    // pcl::io::loadPCDFile("../data/iray2.pcd", *target_p);

    Eigen::Matrix4f T_real = Eigen::Matrix4f::Identity();
    T_real.block<3,1>(0, 3) = Eigen::Vector3f(FLAGS_t_x, FLAGS_t_y, FLAGS_t_z);
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    Eigen::AngleAxisf t_V1(M_PI * (FLAGS_pitch /180), Eigen::Vector3f(1, 0, 0));
    Eigen::AngleAxisf t_V2(M_PI * (FLAGS_yaw /180), Eigen::Vector3f(0, 1, 0));
    Eigen::AngleAxisf t_V3(M_PI * (FLAGS_roll /180), Eigen::Vector3f(0, 0, 1));
    R = R * t_V1 * t_V2 * t_V3;
    T_real.block<3,3>(0,0) = R;
    if(FLAGS_real_data)
    {

    //处理输入数据
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(0.2, 0.2, 0.2);
    filter.setInputCloud(filter_p);
    filter.filter(*source_p);
    }
    // timepoint
    // 测试更多chrono
    
    //ndt进入
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    auto t1 = chrono::system_clock::now();
    ndt.setInputTarget(target_p);
    ndt.setMaximumIterations(FLAGS_MaximumIterations);
    ndt.setTransformationEpsilon(FLAGS_TransformationEpsilon);
    ndt.setStepSize(0.1);      
    ndt.setResolution(FLAGS_Resolution);
    
    ndt.setInputSource(source_p);

    
    cout<<"ndt calcuate begin"<<endl;
    auto t2 = chrono::system_clock::now();
    ndt.align(*output_p);
    auto t3 = chrono::system_clock::now();
    cout<<"ndt calcuate end"<<endl;

    
    auto duration_load = chrono::duration_cast<chrono::milliseconds>(t2 - t1);
    auto duration_reco = chrono::duration_cast<chrono::milliseconds>(t3 - t2);
    auto duration_all = chrono::duration_cast<chrono::milliseconds>(t3 - t1);
    //输出运行信息
    int iter_step = ndt.getFinalNumIteration();
    double score = ndt.getFitnessScore();
    Eigen::Matrix4f T = ndt.getFinalTransformation();
    // pcl::transformPointCloud(*filter_p, *output_p, T);

    cout << "=========================" <<endl;
    if(!FLAGS_real_data)
    {
        cout << "the real translation matrix :" << endl;
        cout << T_real.matrix() <<endl;
        cout << "-------------------------" <<endl;
        cout << "setTransformationEpsilon = " << FLAGS_TransformationEpsilon << endl;
        cout << "setResolution = " << FLAGS_Resolution <<endl;
        cout << "-------------------------" <<endl;
    }
    cout << "ndt iter_step =" << iter_step << endl;
    cout << "ndt score =" << score << endl;
    cout << "-------------------------" <<endl;
    cout << T << endl;
    cout << "-------------------------" <<endl;
    cout << "all time = " << duration_all.count() << " " << chrono::milliseconds::period::den << " " <<chrono::milliseconds::period::num << "seconds" << endl;
    cout << "load time = " << duration_load.count() << "seconds" << endl;
    cout << "reco time = " << duration_reco.count() << "seconds" << endl;
    cout << "=========================" <<endl;
    pcl::transformPointCloud(*filter_p, *output_p, T);

    // Eigen::AngleAxisd t_V(M_PI/4,Eigen::Vector3d(0,0,1));
    // Eigen::Matrix4d R_T = Eigen::Matrix4d::Identity();
    // R_T.block<3,3>(0,0) = t_V.matrix();
    // cout << "========================" <<endl;
    // cout << R_T << endl; 
    // pcl::transformPointCloud(*source_p, *filter_p, T);

    // 可视化
    int v1, v2, v3, v4 = 0;
    pcl::visualization::PCLVisualizer viewer("ndt test");
    viewer.initCameraParameters();

    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> color_output(output_p);
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> color_filter(filter_p);
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> color_target(target_p);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    viewer.addText("source-target", 10, 10, "v1_text", v1);
    viewer.addPointCloud(filter_p, color_filter, "v1_filter_cloud", v1);
    viewer.addPointCloud(target_p, color_target, "v1_target_cloud", v1);

    viewer.addText("output-target", 10, 10, "v2_text", v2);
    viewer.addPointCloud(output_p, color_output, "v2_output_cloud", v2);
    viewer.addPointCloud(target_p, color_target, "v2_target_cloud", v2);
    viewer.addCoordinateSystem(1.0);

    // pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> color_output(output_p);
    // pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> color_target(target_p);
    // pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> color_source(source_p);
    // pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> color_endput(filter_p);
    // viewer.createViewPort(0.0, 0.5, 0.5, 1.0, v1);
    // viewer.createViewPort(0.5, 0.5, 1.0, 1.0, v2);
    // viewer.createViewPort(0.0, 0.0, 0.5, 0.5, v3);
    // viewer.createViewPort(0.5, 0.0, 1.0, 0.5, v4);

    
    // viewer.addText("source-target", 10, 10, "v1_text", v1);
    // viewer.addPointCloud(source_p, color_source, "v1_source_cloud", v1);
    // viewer.addPointCloud(target_p, color_target, "v1_target_cloud", v1);

    // viewer.addText("source-output", 10, 10, "v2_text", v2);
    // viewer.addPointCloud(source_p, color_source, "v2_source_cloud", v2);
    // viewer.addPointCloud(output_p, color_output, "v2_output_cloud", v2);

    // viewer.addText("target-output", 10, 10, "v3_text", v3);
    // viewer.addPointCloud(target_p, color_target, "v3_target_cloud", v3);
    // viewer.addPointCloud(output_p, color_output, "v3_output_cloud", v3);

    // viewer.addText("endput-source", 10, 10, "v4_text", v4);
    // viewer.addPointCloud(source_p, color_source, "v4_source_cloud_clone", v4);
    // viewer.addPointCloud(source_p, color_source, "v4_source_cloud", v4);

    // viewer.addCoordinateSystem(1.0);

    while (!viewer.wasStopped())
    {
        viewer.spin();
        std::this_thread::sleep_for(100ms);
    }
    
    return 0;
}