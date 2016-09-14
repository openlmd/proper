#include "LmdFuncs.h"

namespace OpenLmd
{

PointCloudProcess::PointCloudProcess()
{
    cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
    new_cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
    selected_points.reset(new pcl::PointCloud<pcl::PointXYZ>);
    t_matrix = Eigen::Affine3f::Identity();
    paralel_mat = Eigen::Affine3f::Identity();
    border_pixels = 40;
    resolution = 0.0001;
    ground_coeffs.resize(4);
    ground_coeffs(0) = 0;
    ground_coeffs(1) = 0;
    ground_coeffs(2) = 1;
    ground_coeffs(3) = 0;
}

void PointCloudProcess::LoadPointFile(std::string fileName)
{
    if (fileName != "")
    {
        cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud_xyz) == -1)
        {
            PCL_ERROR ("Couldn't read file *.pcd \n");
            throw "Couldn't read file *.pcd";
        }
    }
    else
    {
        PCL_ERROR ("Invalid file name \n");
        throw "Invalid file name";
    }
    pcl::getMinMax3D(*cloud_xyz, min_pt, max_pt);
    return;
}

void PointCloudProcess::SavePointFile(std::string fileName)
{
    if (fileName != "")
    {
        if ((cloud_xyz->width * cloud_xyz->height) > 0)
        {
            if (fileName.compare(fileName.size()-4,4,".pcd") != 0)
            {
                fileName += ".pcd";
            }
            pcl::io::savePCDFileASCII (fileName, *cloud_xyz);
        }
        else
        {
            PCL_ERROR ("Point cloud empty \n");
            throw "Point cloud empty";
        }
    }
    else
    {
        PCL_ERROR ("Invalid file name \n");
        throw "Invalid file name";
    }
    return;
}

void PointCloudProcess::getPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    int nPoints = cloud_xyz->points.size();
    if (nPoints <= 0)
    {
        throw "Point cloud empty";
    }
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = *cloud_xyz;
    return;
}

void PointCloudProcess::getModPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    int nPoints = new_cloud_xyz->points.size();
    if (nPoints <= 0)
    {
        throw "New point cloud empty";
    }
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = *new_cloud_xyz;
    return;
}

void PointCloudProcess::getSelPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    int nPoints = selected_points->points.size();
    if (nPoints <= 0)
    {
        throw "No selected points";
    }
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = *selected_points;
    return;
}

void PointCloudProcess::resetSelPoints()
{
    selected_points.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void PointCloudProcess::ModifyPointCloud()
{
    if (new_cloud_xyz->width * new_cloud_xyz->height >= 1)
    {
        cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
        *cloud_xyz = *new_cloud_xyz;
        pcl::getMinMax3D(*cloud_xyz, min_pt, max_pt);
    }
    else
    {
        PCL_ERROR ("Empty mod cloud \n");
        throw "Empty mod point cloud";
    }
    return;
}

void PointCloudProcess::VoxelFilter(double leafX, double leafY, double leafZ)
{
    if (leafX == -1)
    {
        leafX = (max_pt.x - min_pt.x) / (cloud_xyz->width * cloud_xyz->height);
    }
    if (leafY == -1)
    {
        leafY = (max_pt.y - min_pt.y) / (cloud_xyz->width * cloud_xyz->height);
    }
    if (leafZ == -1)
    {
        leafZ = (max_pt.z - min_pt.z) / (cloud_xyz->width * cloud_xyz->height);
    }
    if (leafX>0 && leafY>0 && leafZ>0)
    {
        new_cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud_xyz);
        sor.setLeafSize (leafX, leafY, leafZ);
        sor.filter (*new_cloud_xyz);
    }
    else
    {
        PCL_ERROR ("Invalid voxel dimensions \n");
        throw "Invalid voxel dimensions";
    }
    return;
}

void PointCloudProcess::StatisticalFilter(int kNearest, double multiplier)
{
    if (kNearest == -1)
    {
        kNearest = 10;
    }
    if (multiplier == -1)
    {
        multiplier = 1;
    }
    if (kNearest>0 && multiplier>=0)
    {
        new_cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud_xyz);
        sor.setMeanK(kNearest);
        sor.setStddevMulThresh(multiplier);
        sor.filter (*new_cloud_xyz);
    }
    else
    {
        PCL_ERROR ("Invalid statistical parameters \n");
        throw "Invalid statistical parameters";
    }
    return;
}

void PointCloudProcess::RadiusFilter(int kNearest, double radius)
{
    if (kNearest == -1)
    {
        kNearest = 10;
    }
    if (radius == -1)
    {
        radius = (max_pt.x - min_pt.x + max_pt.y - min_pt.y + max_pt.z - min_pt.z) / (3 * sqrt(cloud_xyz->width * cloud_xyz->height)) * kNearest / 2;
    }
    if (kNearest>=1 && radius>0)
    {
        new_cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud_xyz);
        sor.setMinNeighborsInRadius (kNearest);
        sor.setRadiusSearch (radius);
        sor.filter(*new_cloud_xyz);
    }
    else
    {
        PCL_ERROR ("Invalid radius search parameters \n");
        throw "Invalid radius search parameters";
    }
    return;
}

void PointCloudProcess::PassthroughFilter(double min, double max, int axis)
{
    if (axis>=0 && axis<=2)
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud_xyz);
        if (axis == 0)
            pass.setFilterFieldName ("x");
        if (axis == 1)
            pass.setFilterFieldName ("y");
        if (axis == 2)
            pass.setFilterFieldName ("z");
        pass.setFilterLimits (min, max);
        pass.setFilterLimitsNegative (false);
        pass.filter (*new_cloud_xyz);
    }
    else
    {
        PCL_ERROR ("Invalid passtrough filter parameters \n");
        throw "Invalid passtrough filter parameters";
    }
    return;
}

void PointCloudProcess::ResamplingPoints(double radius)
{
    if (radius == -1)
    {
        radius = (max_pt.x - min_pt.x + max_pt.y - min_pt.y + max_pt.z - min_pt.z) / (3 * sqrt(cloud_xyz->width * cloud_xyz->height)) * radius / 2;
    }
    if (radius>0)
    {
        new_cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        mls.setInputCloud (cloud_xyz);
        mls.setPolynomialFit (true);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (radius);
        mls.process(*new_cloud_xyz);
    }
    else
    {
        PCL_ERROR ("Invalid resampling parameters \n");
        throw "Invalid resampling parameters";
    }
    return;
}

void PointCloudProcess::AddPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr &add_cloud, pcl::PointXYZ sel_point)
{
    int nPoints = add_cloud->points.size();
    if (nPoints > 0)
    {
        if (add_cloud->points[nPoints -1].x == sel_point.x && add_cloud->points[nPoints -1].y == sel_point.y && add_cloud->points[nPoints -1].z == sel_point.z)
        {
            //add_cloud->points.pop_back();
            throw "Repeated point";
        }
    }
    add_cloud->points.push_back(sel_point);
    return;
}

void PointCloudProcess::AddPoint(pcl::PointXYZ sel_point)
{
    AddPoint(selected_points, sel_point);
}

void PointCloudProcess::FitPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_cloud_xyz, double distThreshold)
{
    if (distThreshold == 0)
    {
        throw "Threslhold can't be 0.0";
    }
    if (plane_cloud_xyz->points.size() < 3)
    {
        throw "Empty point cloud";
    }
    ground_coeffs.resize(4);
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane_ptr (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (plane_cloud_xyz));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_manual (model_plane_ptr);
    ransac_manual.setDistanceThreshold (distThreshold);
    ransac_manual.computeModel();
    ransac_manual.getModelCoefficients(ground_coeffs);
    std::vector<int> inliers;
    ransac_manual.getInliers(inliers);
    new_cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud<pcl::PointXYZ>(*plane_cloud_xyz, inliers, *new_cloud_xyz);
}

void PointCloudProcess::FitPlane(double distThreshold)
{
    if (selected_points->points.size() > 3)
    {
        FitPlane(selected_points, distThreshold);
    }
    else if (cloud_xyz->points.size() > 3)
    {
        FitPlane(cloud_xyz, distThreshold);
    }
    else
    {
        throw "No valid point cloud";
    }
}

void PointCloudProcess::getPlaneCoeffs(Eigen::VectorXf &coeffs)
{
    coeffs = ground_coeffs;
}

void PointCloudProcess::TransformatioMatrix(Eigen::VectorXf coeffs, Eigen::Affine3f &tMatrix)
{
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    Eigen::Vector3f w(coeffs.x (),coeffs.y (),coeffs.z ());    //Toma w como vector fijo
    if (w[2] < 0)
        w *= -1;
    Eigen::Vector3f v(0,1,0);
    Eigen::Vector3f u(v.cross(w));  //Establece u el plano normal a k
    v = w.cross(u);
    u = v.cross(w);
    u.normalize();
    v.normalize();
    w.normalize();
    for (int i=0;i<3;i++)
    {
        transform_1 (i,0) = u[i];
    }
    for (int i=0;i<3;i++)
    {
        transform_1 (i,1) = v[i];
    }
    for (int i=0;i<3;i++)
    {
        transform_1 (i,2) = w[i];
    }
    pcl::getMinMax3D(*cloud_xyz, min_pt, max_pt);
    transform_1 (0,3) = min_pt.x;
    transform_1 (1,3) = min_pt.y;
    //transform_1 (2,3) = (-1* (coefficients_plano_manual->values[0] * min_pt.x) -1* (coefficients_plano_manual->values[1] * min_pt.y) -1 * coefficients_plano_manual->values[2]) / coefficients_plano_manual->values[3];
    transform_1 (2,3) = min_pt.z;
    tMatrix = Eigen::Affine3f::Identity();
    tMatrix = transform_1;
}

void PointCloudProcess::TransformatioMatrix(Eigen::VectorXf coeffs)
{
    TransformatioMatrix(coeffs, t_matrix);
}

void PointCloudProcess::PlaceFrameZPlane(Eigen::VectorXf a, pcl::PointXYZ orig, pcl::PointXYZ dirX, Eigen::Affine3f &frame)
{
    Eigen::Vector3f w(a(0), a(1), a(2));
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    w.normalize();
    if (w[2] < 0)
        w *= -1;
    Eigen::Vector3f b(dirX.x-orig.x, dirX.y-orig.y, dirX.z-orig.z);
    Eigen::Vector3f v(w.cross(b));
    v.normalize();
    Eigen::Vector3f u(v.cross(w));
    u.normalize();
    for (int i=0;i<3;i++)
    {
        transform_1 (i,0) = u[i];
    }
    for (int i=0;i<3;i++)
    {
        transform_1 (i,1) = v[i];
    }
    for (int i=0;i<3;i++)
    {
        transform_1 (i,2) = w[i];
    }
    frame = Eigen::Affine3f::Identity();
    frame = transform_1;
    frame (0,3) = orig.x;
    frame (1,3) = orig.y;
    frame (2,3) = orig.z;

    Eigen::Quaternionf cuat;
    cuat = Eigen::Quaternionf(frame.linear());

    SaveMatrix("base_frame.json", frame);
}

void PointCloudProcess::CutPlane(bool updown, double offset)
{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_cloud_xyz;
//    plane_cloud_xyz = cloud_xyz;
    float normal;
    std::vector<int> inliers;
    for (int i = 0; i < cloud_xyz->size(); i++ )
    {
        normal = ground_coeffs.x () * cloud_xyz->points[i].x + ground_coeffs.y () * cloud_xyz->points[i].y + ground_coeffs.z () * cloud_xyz->points[i].z + ground_coeffs.w ();
        if (updown){
            if (normal >= (0-offset))
            {
                inliers.push_back(i);
            }
        }
        else
        {
            if (normal <= (0+offset))
            {
                inliers.push_back(i);
            }
        }
    }
    new_cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud_xyz, inliers, *new_cloud_xyz);
}

void PointCloudProcess::getTransformationMatrix(Eigen::Affine3f &tMatrix)
{
    tMatrix = t_matrix;
}

void PointCloudProcess::SaveMatrix(const std::string fileName, Eigen::Affine3f matrix)
{
    Eigen::Quaternionf cuat;
    cuat = Eigen::Quaternionf(matrix.linear());
    ofstream myfile;
    myfile.open (fileName.c_str(), ios::out | ios::trunc);
    myfile << "{\"matrix\": {" << std::endl;
    myfile << "\t" << "\"u\": [" << matrix(0,0) << ", " << matrix(1,0) << ", " << matrix(2,0) << "]," << std::endl;
    myfile << "\t" << "\"v\": [" << matrix(0,1) << ", " << matrix(1,1) << ", " << matrix(2,1) << "]," << std::endl;
    myfile << "\t" << "\"w\": [" << matrix(0,2) << ", " << matrix(1,2) << ", " << matrix(2,2) << "]," << std::endl;
    myfile << "\t" << "\"t\": [" << matrix(0,3) << ", " << matrix(1,3) << ", " << matrix(2,3) << "]" << std::endl;
    myfile << "\t" << "\"quat\": [" << cuat.x() << ", " << cuat.y() << ", " << cuat.z() << ", " << cuat.w() << "]," << std::endl;
    myfile << "}}";
    myfile.close();
}

void PointCloudProcess::SaveMatrix(const std::string fileName)
{
  SaveMatrix(fileName, t_matrix);
}

void PointCloudProcess::ReorientPointCloud(Eigen::Affine3f tMatrix)
{
    paralel_mat = tMatrix;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud;
    temp_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3f transformada = Eigen::Affine3f::Identity();
    transformada.translation() = -1 * tMatrix.translation();
    pcl::transformPointCloud (*cloud_xyz, *new_cloud_xyz, transformada);

    transformada = Eigen::Affine3f::Identity();
    transformada.linear() = tMatrix.linear().transpose();
    pcl::transformPointCloud (*new_cloud_xyz, *temp_cloud, transformada);

    transformada = Eigen::Affine3f::Identity();
    transformada.translation() = tMatrix.translation();
    pcl::transformPointCloud (*temp_cloud, *new_cloud_xyz, transformada);
}

void PointCloudProcess::ReorientPointCloud()
{
    ReorientPointCloud(t_matrix);
}

void PointCloudProcess::setProjection(float res, int border)
{
    resolution = res;
    border_pixels = border;
}

void PointCloudProcess::ProjectPointCloud(const std::string fileName)
{
    float res = resolution;
    int over_pixel = border_pixels;
    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cloud (new pcl::PointCloud<pcl::PointXYZ> (*cloud_xyz) );
    //projection = cv::Scalar(0);,cv::Scalar(0)
    //pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud_xyz, min_pt, max_pt);
    float x_size, y_size, z_size;
    x_size = max_pt.x - min_pt.x;
    y_size = max_pt.y - min_pt.y;
    z_size = max_pt.z - min_pt.z;
    int imageX, imageY;
    imageX = x_size/res+over_pixel;
    imageY = y_size/res+over_pixel;
    cv::Mat oProjection(imageY, imageX,  CV_16UC1);
    oProjection = cv::Scalar(0);
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > data_points;
    data_points =  proj_cloud->points; //boost::get<pcl::PointXYZ>(proj_cloud->points);
    while (!data_points.empty())
     {
        float x, y , z;
        pcl::PointXYZ point = data_points.back();
        x = (point.x - min_pt.x) / res;
        y = (point.y - min_pt.y) / res;
        z = (point.z - min_pt.z) / z_size * 65535;
        x = x+over_pixel/2;
        y = y+over_pixel/2;
        oProjection.at<ushort>((int)y,(int)x)= cv::saturate_cast<ushort>(z);
        data_points.pop_back();
    }
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(3);
    cv::imwrite(fileName, oProjection, compression_params);
}

void PointCloudProcess::PlaceFrame2D(int xVal, int yVal, Eigen::Affine3f &foundPointT)
{
    int over_pixel = border_pixels;
    float res = resolution;
    pcl::PointXYZ min_pt, max_pt, f_pt;
    pcl::getMinMax3D(*cloud_xyz, min_pt, max_pt);
    float x_size, y_size;
    x_size = max_pt.x - min_pt.x;
    y_size = max_pt.y - min_pt.y;
    f_pt.x = (xVal - over_pixel/2) * res + min_pt.x;
    f_pt.y = (yVal - over_pixel/2) * res + min_pt.y;
    f_pt.z = 0;

    //Transformación
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud;
    temp_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3f transformada = Eigen::Affine3f::Identity();
    transformada.translation() = -1 * paralel_mat.translation();
    cloud_xyz->points.push_back(f_pt);
    pcl::transformPointCloud (*cloud_xyz, *new_cloud_xyz, transformada);

    f_pt = new_cloud_xyz->points.back();
    new_cloud_xyz->points.pop_back();
    f_pt.z = 0;
    new_cloud_xyz->points.push_back(f_pt);

    transformada = Eigen::Affine3f::Identity();
    transformada.linear() = paralel_mat.linear();
    pcl::transformPointCloud (*new_cloud_xyz, *temp_cloud, transformada);

    transformada = Eigen::Affine3f::Identity();
    transformada.translation() = paralel_mat.translation();
    pcl::transformPointCloud (*temp_cloud, *new_cloud_xyz, transformada);
    f_pt = new_cloud_xyz->points.back();
    new_cloud_xyz->points.pop_back();
    cloud_xyz->points.pop_back();

    foundPointT = Eigen::Affine3f::Identity();
    foundPointT.linear() = t_matrix.linear();
    foundPointT (0, 3) = f_pt.x;
    foundPointT (1, 3) = f_pt.y;
    foundPointT (2, 3) = f_pt.z;
    foundPointT.rotate(Eigen::AngleAxisf(0,Eigen::Vector3f::UnitZ()));

    SaveMatrix("found_point.json", foundPointT);
}

}
