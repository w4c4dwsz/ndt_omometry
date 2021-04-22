#include "utility.h"
#include "ndt_set.h"

bool GetFileName(vector<string> &pcd_filename_v)
{
    std::ifstream fout;
    fout.open("../data/list");

    if(!fout.is_open())
    {
        cout << "can't open file list!" <<endl;
        fout.close();
        return false;
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
    fout.close();
    return true;
}

int main()
{
    //读取文件名称列表
    vector<string> pcd_filename_v;
    if(!GetFileName(pcd_filename_v))
    {
        return -1;
    }
    
    //声明点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_p(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_p(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_p(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_p(new pcl::PointCloud<pcl::PointXYZ>);

    //加载一个数据
    pcl::io::loadPCDFile("../data/" + pcd_filename_v[0], *target_p);
    
    //制造假数据
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    //生成NDT以及VOXEL
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(0.2,0.2,0.2);
    voxel.setInputCloud(source_p);
    voxel.filter(*source_p);
    ndt_set ndt_setting(&ndt);
    ndt_setting.setTransformationEpsilon(0.0001);
    cout << "ndtsetTransformationEpsilon = 0.0001" << endl;

    //
    for (int i = 0; i < 10; i++)
    {   
        cout << "iter = " << i << endl;
        float t = 0.5;
        cout << t * i << endl;
        T.block<3,1>(0,3) = Eigen::Vector3d( t * i , 0, 0 );
        pcl::transformPointCloud(*target_p, *source_p, T);
        


        ndt.setInputTarget(target_p);
        ndt.setInputSource(source_p);
        ndt_setting.init();
        ndt.align(*output_p);

        // 输出NDT相关信息
        int iter_step = ndt.getFinalNumIteration();
        double score = ndt.getFitnessScore();
        Eigen::Matrix4f T_cacl = ndt.getFinalTransformation();

        cout << "=========================" << endl;
        cout << "T | t = "<< t * i << endl;
        // cout << "=========================" << endl;
        // cout << T << endl;
        cout << "=========================" << endl;
        cout << "ndt iter_step =" << iter_step << endl;
        cout << "ndt score =" << score << endl;
        cout << "T_cacl" << endl;
        cout << "=========================" << endl;
        cout << T_cacl << endl;
        cout << "=================================================" << endl;
    }


    //可视化
        int v1, v2, v3, v4 = 0;
    pcl::visualization::PCLVisualizer viewer("ndt test");
    viewer.initCameraParameters();

    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> color_output(output_p);
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> color_target(target_p);
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> color_source(source_p);
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> color_endput(filter_p);
    viewer.createViewPort(0.0, 0.5, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.5, 1.0, 1.0, v2);
    viewer.createViewPort(0.0, 0.0, 0.5, 0.5, v3);
    viewer.createViewPort(0.5, 0.0, 1.0, 0.5, v4);

    
    viewer.addText("source-target", 10, 10, "v1_text", v1);
    viewer.addPointCloud(source_p, color_source, "v1_source_cloud", v1);
    viewer.addPointCloud(target_p, color_target, "v1_target_cloud", v1);

    viewer.addText("source-output", 10, 10, "v2_text", v2);
    viewer.addPointCloud(source_p, color_source, "v2_source_cloud", v2);
    viewer.addPointCloud(output_p, color_output, "v2_output_cloud", v2);

    viewer.addText("target-output", 10, 10, "v3_text", v3);
    viewer.addPointCloud(target_p, color_target, "v3_target_cloud", v3);
    viewer.addPointCloud(output_p, color_output, "v3_output_cloud", v3);

    viewer.addText("endput-source", 10, 10, "v4_text", v4);
    viewer.addPointCloud(source_p, color_source, "v4_source_cloud_clone", v4);
    viewer.addPointCloud(source_p, color_source, "v4_source_cloud", v4);

    viewer.addCoordinateSystem(1.0);

    while (!viewer.wasStopped())
    {
        viewer.spin();
        std::this_thread::sleep_for(100ms);
    }
}
