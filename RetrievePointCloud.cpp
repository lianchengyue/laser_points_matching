#include "RetrievePointCloud.h"
#include <iostream>

#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/visualization/cloud_viewer.h>//点云查看窗口头文件
//#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;

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
    for(int i=0; i</*960*1280*//*malloc_usable_size(point_coordinate)/sizeof(*point_coordinate)*/3167; i++) //height
    {
        //去除(0，0，0)点
        if(!(point_coordinate[i].x && point_coordinate[i].y && point_coordinate[i].z))
        {
            continue;
        }

        tempPts.x = (int)point_coordinate[i].x;
        tempPts.y = (int)point_coordinate[i].y;//-100;
        tempPts.z = (int)point_coordinate[i].z;///10;

        //cout << "p:" <<p.x<<","<<p.y<<","<<p.z<<endl;

        tempPts.r = 255; //55
        tempPts.g = 255;//250
        tempPts.b = 255; //55
        Lasercloud->push_back(tempPts);
    }
    cout << "Lasercloud num=" << Lasercloud->width << endl;
    pcl::io::savePCDFile("./dump/input.pcd",*Lasercloud);

    //added by flq
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    //
    //pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    //sor.setInputCloud(Lasercloud);
    //sor.setMeanK(50);
    //sor.setStddevMulThresh(1.0);
    //sor.filter(*cloud_filtered);

    //viewer.showCloud(cloud_filtered);
    //added end

    //Display
    viewer.showCloud(Lasercloud);

    //Lasercloud->clear();

    //while (!viewer.wasStopped ());

    return 0;
}

//filtered 3d points input
int RetrievePointCloud::GetP3dInFrame(FilteredP3d *f_p3d, int pts_cnt)
{
    for(int i=0; i<pts_cnt; i++) //height
    {
        //去除(0，0，0)点
        if(!(f_p3d[i].filterd_p3d.x && f_p3d[i].filterd_p3d.y && f_p3d[i].filterd_p3d.z))
        {
            continue;
        }

        tempPts.x = (int)f_p3d[i].filterd_p3d.x/16;
        tempPts.y = (int)f_p3d[i].filterd_p3d.y/16;//-100;
        tempPts.z = (int)f_p3d[i].filterd_p3d.z/160;///10;

        //上色
        if(0 == f_p3d[i].fP3dBelonging % 4)
        {
            tempPts.r = 255;
            tempPts.g = 0;
            tempPts.b = 0;
        }
        else if(1 == f_p3d[i].fP3dBelonging % 4)
        {
            tempPts.r = 0;
            tempPts.g = 255;
            tempPts.b = 0;
        }
        else if(2 == f_p3d[i].fP3dBelonging % 4)
        {
            tempPts.r = 0;
            tempPts.g = 0;
            tempPts.b = 255;
        }
        else if(3 == f_p3d[i].fP3dBelonging % 4)
        {
            tempPts.r = 255;
            tempPts.g = 255;
            tempPts.b = 0;
        }

        Lasercloud->push_back(tempPts);
    }

    cout << "Lasercloud num=" << Lasercloud->width << endl;
    pcl::io::savePCDFile("./dump/input.pcd",*Lasercloud);

    //added by flq
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    //
    //pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    //sor.setInputCloud(Lasercloud);
    //sor.setMeanK(50);
    //sor.setStddevMulThresh(1.0);
    //sor.filter(*cloud_filtered);

    //viewer.showCloud(cloud_filtered);
    //added end

    //Display
    viewer.showCloud(Lasercloud);

    //Lasercloud->clear();

    //while (!viewer.wasStopped ());

    return 0;
}
#endif
