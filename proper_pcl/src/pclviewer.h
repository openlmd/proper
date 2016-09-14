#ifndef PCLVIEWER_H
#define PCLVIEWER_H

// basic file operations
#include <fstream>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Qt
#include <QMainWindow>
#include <QFileDialog>

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

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

//LMD projects functions
#include "include_files/LmdFuncs.h"

using namespace cv;

struct callback_args{
  // structure used to pass arguments to the callback function
  pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d;
  pcl::PointCloud<pcl::PointXYZ>::Ptr distance_points;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
  bool plane;
};

struct callback_args_class{
  OpenLmd::PointCloudProcess *pointProcessPtr;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void  pp_callback_plane (const pcl::visualization::PointPickingEvent& event, void* args);
void  pp_callback_points (const pcl::visualization::PointPickingEvent& event, void* args);

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:
    explicit PCLViewer (QWidget *parent = 0);
    ~PCLViewer ();

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PointCloudT::Ptr cloud;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz ;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_modificada ;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ref ;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d;

  unsigned int red;
  unsigned int green;
  unsigned int blue;

  OpenLmd::PointCloudProcess pointProcess;

  Eigen::VectorXf ground_coeffs;
  Eigen::Affine3f t_matrix;

  //QImage qProjection;

  //struct callback_args cb_args;
  struct callback_args_class cb_args_class;

private slots:
  void on_pushButton_random_2_clicked();

  void on_pushButton_gardar_clicked();

  void on_pushButton_funcion_1_clicked();

  void on_pushButton_deshacer_filtro_clicked();

  void on_pushButton_clicked();

  void on_pushButton_funcion_2_clicked();

  void on_pushButton_deshacer_plano_clicked();

  void on_pushButton_distances_clicked();

  void on_doubleSpinBox_cloud_size_valueChanged(double arg1);

  void on_doubleSpinBox_mod_size_valueChanged(double arg1);

  void on_doubleSpinBox_sel_size_valueChanged(double arg1);

  void on_pushButton_aceptar_filtro_clicked();

  void on_pushButton_reorienta_clicked();

  void on_pushButton_recorta_plano_clicked();

  void on_pushButton_aceptar_fitting_clicked();

  void on_pushButton_centra_clicked();

  void on_pushButton_proj_points_clicked();

  void on_pushButton_2d_place_clicked();

  void on_pushButton_filter_clicked();

private:
  Ui::PCLViewer *ui;



};

#endif // PCLVIEWER_H
