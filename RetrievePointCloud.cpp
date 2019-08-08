#include "RetrievePointCloud.h"

#ifdef PCL_PROCESS
///typedef pcl::PointXYZ point;
///typedef pcl::PointXYZRGBA pointcolor;

//typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointCloud<PointT> PointCloud;


RetrievePointCloud::RetrievePointCloud()
: viewer ("Wheel Viewer"),
  Lasercloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr Lasercloud(new pcl::PointCloud<pcl::PointXYZRGB>);
}

RetrievePointCloud::~RetrievePointCloud()
{

}

int RetrievePointCloud::RetriveInit(cv::Point3d *point_coordinate)
{
    for(int i=0; i<960*1280; i++) //height
    {
        //去除(0，0，0)点
        if(!(point_coordinate[i].x && point_coordinate[i].y && point_coordinate[i].z))
        {
            continue;
        }

        tempPts.x = point_coordinate[i].x;
        tempPts.y = point_coordinate[i].y;
        tempPts.z = point_coordinate[i].z;

        //cout << "p:" <<p.x<<","<<p.y<<","<<p.z<<endl;

        tempPts.r = 255; //55
        tempPts.g = 255;//250
        tempPts.b = 255; //55
        Lasercloud->push_back(tempPts);
    }
    cout << "Lasercloud num=" << Lasercloud->width << endl;
    pcl::io::savePCDFile("./dump/input.pcd",*Lasercloud);

    viewer.showCloud(Lasercloud);

    //Lasercloud->clear();

    //while (!viewer.wasStopped ());


#if 0
    for(int i=0; i<960; i++) //height
    {
        tempPts.x = point_coordinate[i].x;
        tempPts.y = point_coordinate[i].y;
        tempPts.z = point_coordinate[i].z;

        //cout << "p:" <<p.x<<","<<p.y<<","<<p.z<<endl;

        tempPts.r=255; //55
        tempPts.g=255;//250
        tempPts.b=0; //55
        Lasercloud->push_back(tempPts);
    }
    cout << "Lasercloud num=" << Lasercloud->width << endl;
    pcl::io::savePCDFile("./dump/input.pcd",*Lasercloud);
    //pcl::io::savePCDFileASCII("TEST.pcd", *Lasercloud);

    //显示点云
    pcl::PCLHeader header;
    header.frame_id="map";
    Lasercloud->header=header;
    Lasercloud->height = 1;
    Lasercloud->width = Lasercloud->points.size();
    Lasercloud->is_dense = false;
    Lasercloud->points.resize (Lasercloud->width * Lasercloud->height);

    //创建一个模型参数对象，用于记录结果
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    //inliers表示误差能容忍的点 记录的是点云的序号
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object,// 创建一个分割器
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandator 设置目标几何形状
    seg.setModelType (pcl::SACMODEL_PLANE);   //SACMODEL_PLANE:平面拟合
    //分割方法：随机采样法
    seg.setMethodType (pcl::SAC_RANSAC);
    //设置误差容忍范围
    seg.setDistanceThreshold (0.01);

    //输入点云
    seg.setInputCloud (Lasercloud);
    //分割点云
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return (-1);
    }

    //save PointCloud
    pcl::io::savePCDFileASCII("./dump/savePCDFileASCII.pcd", *Lasercloud); //cloud.makeShared()

    viewer.showCloud(Lasercloud);

    //清空当前帧的点云
    //https://segmentfault.com/a/1190000007125502
    Lasercloud->clear();

    while (!viewer.wasStopped ());
    //显示点云end
#endif
    return 0;
}
#endif
