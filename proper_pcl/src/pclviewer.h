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
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/common/time.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

//Boost Library
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

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

typedef pcl::PointXYZRGBA PointTA;
typedef pcl::PointCloud<PointTA> PointCloudTA;

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
  PointCloudTA::Ptr cloud;

  unsigned int red;
  unsigned int green;
  unsigned int blue;

  OpenLmd::PointCloudProcess pointProcess;

  Eigen::VectorXf ground_coeffs;
  Eigen::Affine3f t_matrix;

  //struct callback_args cb_args;
  struct callback_args_class cb_args_class;

private slots:
  void on_pushButton_load_clicked();

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

  void on_spinBox_cluster_valueChanged(int argv);

  void on_pushButton_aceptar_filtro_clicked();

  void on_pushButton_reorienta_clicked();

  void on_pushButton_recorta_plano_clicked();

  void on_pushButton_aceptar_fitting_clicked();

  void on_pushButton_centra_clicked();

  void on_pushButton_proj_points_clicked();

  void on_pushButton_2d_place_clicked();

  void on_pushButton_filter_clicked();

  void on_pushButton_load_model_clicked();

  void on_pushButton_delete_model_clicked();

  void on_pushButton_init_align_clicked();

  void on_pushButton_fine_alignment_clicked();

  void on_pushButton_test_clicked();

  void on_actionLoad_triggered();

  void on_actionSave_triggered();

private:
  Ui::PCLViewer *ui;

};

#endif // PCLVIEWER_H
