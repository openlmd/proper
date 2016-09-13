cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (proper_pcl)
find_package (Qt4 REQUIRED)
find_package (VTK REQUIRED)
find_package (PCL 1.7.1 REQUIRED)
find_package( OpenCV REQUIRED )

include_directories (${PCL_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES src/main.cpp src/pclviewer.cpp src/include_files/LmdFuncs.cpp)
set  (project_HEADERS src/pclviewer.h src/include_files/LmdFuncs.h)
set  (project_FORMS   src/pclviewer.ui)
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (pcl_visualizer ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (pcl_visualizer ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})
target_link_libraries( pcl_visualizer ${OpenCV_LIBS} )
