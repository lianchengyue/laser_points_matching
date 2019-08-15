#ifndef RETRIEVEPOINTCLOUD
#define RETRIEVEPOINTCLOUD

#include "Utils.h"

#ifdef PCL_PROCESS
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

/*
#include "extract.hpp"
#include "pcd_file.hpp"
#include "scaner.hpp"
#include "read_config.hpp"
*/

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#if 0
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#endif
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


class RetrievePointCloud
{
public:
    RetrievePointCloud();
    ~RetrievePointCloud();

    int RetriveInit(/*cv::Point3_<float>*/cv::Point3d *point_coordinate);
    int GetP3dInFrame(FilteredP3d *f_p3d, int pts_cnt);

//    pcl::visualization::CloudViewer viewer;

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Lasercloud;
    pcl::PointXYZRGB tempPts;
    pcl::visualization::CloudViewer viewer;
};
#endif //#ifdef PCL_PROCESS

#endif //RETRIEVEPOINTCLOUD
