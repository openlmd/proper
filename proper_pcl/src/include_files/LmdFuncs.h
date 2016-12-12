#ifndef LMD_FUNCS
#define LMD_FUNCS

#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

//declaracions
namespace OpenLmd
{

class PointCloudProcess {

public:
    //Constructor
    PointCloudProcess();
    //Load a point cloud in 'cloud_xyz' member from a '.pcd' file
    void LoadPointFile (std::string fileName);
    //Save 'cloud_xyz' in a file
    void SavePointFile (std::string fileName);
    //Get 'cloud_xyz'
    void getPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    //Get 'new_cloud_xyz'
    void getModPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    //Get selected points
    void getSelPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    //Clear cloud of selected points
    void resetSelPoints();
    //Reset 'cloud_xyz' to 'new_cloud_xyz'
    void ModifyPointCloud();
    void VoxelFilter(double leafX, double leafY, double leafZ);
    void StatisticalFilter(int kNearest, double multiplier);
    void RadiusFilter(int kNearest, double radius);
    void PassthroughFilter(double min, double max, int axis);
    void ResamplingPoints(double radius);
    //Add a point to a cloud
    void AddPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr &add_cloud, pcl::PointXYZ sel_point);
    //Add a point to 'selected_points'
    void AddPoint(pcl::PointXYZ sel_point);
    //Fit a plane to a cloud, set 'ground_coeffs', and get inliers in 'new_cloud_xyz'
    void FitPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_cloud_xyz, double distThreshold = 0.01);
    //Fit to 'selected_points' if n>3, else to 'cloud_xyz'
    void FitPlane(double distThreshold = 0.01);
    void getPlaneCoeffs(Eigen::VectorXf &coeffs);
    void TransformatioMatrix(Eigen::VectorXf coeffs, Eigen::Affine3f &tMatrix);
    void TransformatioMatrix(Eigen::VectorXf coeffs);//TODO: Default coeffs? para que pille ground_coeffs
    void PlaceFrameZPlane(Eigen::VectorXf a, pcl::PointXYZ orig, pcl::PointXYZ dirX, Eigen::Affine3f &frame);
    void CutPlane(bool updown, double offset);
    void getTransformationMatrix(Eigen::Affine3f &tMatrix);
    void SaveMatrix(const std::string fileName, Eigen::Affine3f matrix);
    void SaveMatrix(const std::string fileName = "plane.json");
    void ReorientPointCloud(Eigen::Affine3f tMatrix);
    void ReorientPointCloud();
    void setProjection(float res, int border);
    void ProjectPointCloud(const std::string fileName = "ocvImage.png");
    void PlaceFrame2D(int xVal, int yVal, Eigen::Affine3f &foundPointT);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz;
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud_xyz;
    pcl::PointCloud<pcl::PointXYZ>::Ptr selected_points;
    Eigen::Affine3f t_matrix, paralel_mat;
    //double param[3];
    pcl::PointXYZ min_pt, max_pt;
    Eigen::VectorXf ground_coeffs;
    int border_pixels;
    float resolution;
};
}


#endif
