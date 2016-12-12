#include "pclviewer.h"
//#include "include_files/LmdFuncs.h"
#include <QApplication>
#include <QMainWindow>
#include <pcl/visualization/pcl_visualizer.h>

int main (int argc, char *argv[])
{
  //OpenLmd::PointCloudProcess pointProcess;
  QApplication a (argc, argv);
  PCLViewer w;

  w.show ();

  return a.exec ();
}
