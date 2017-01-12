#ifndef PCLVISUALIZER_H_INCLUDED
#define PCLVISUALIZER_H_INCLUDED
#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <Eigen/Dense>
#include <Eigen/Core>

using namespace Eigen;

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                    Vector3d rbg);
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void);
void mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void);

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis() ;

void pointCloudVis(const Ref<const MatrixXd>& cloud1, const Ref<const MatrixXd>& cloud2,
                const Ref<const Matrix4d>& transmormationM, Vector3d rgb1, Vector3d rgb2);









#endif // PCLVISUALIZER_H_INCLUDED
