#include "pclVisualizer.h"

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (0.5);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                    Vector3d rgb)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, rgb(0), rgb(1), rgb(2));
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());
  viewer->registerMouseCallback (mouseEventOccurred, (void*)viewer.get ());

  return (viewer);
}


void pointCloudVis(const Ref<const MatrixXd>& cloud1, const Ref<const MatrixXd>& cloud2,
                const Ref<const Matrix4d>& transformationM, Vector3d rgb1, Vector3d rgb2){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointClouds (new pcl::PointCloud<pcl::PointXYZRGB>);

    MatrixXd transformed_cloud_hom(cloud2.rows()+1, cloud2.cols()), transformed_cloud(cloud2.rows(), cloud2.cols());
    transformed_cloud_hom.topRows(3) = cloud2;
    transformed_cloud_hom.bottomRows(1).setOnes();

    transformed_cloud_hom = transformationM * transformed_cloud_hom;

    transformed_cloud = transformed_cloud_hom.topRows(3);


    //filling pcl Point cloud

    for(int i=0; i<cloud1.cols(); ++i){
        pcl::PointXYZRGB point;
        point.x = cloud1.col(i)(0);
        point.y = cloud1.col(i)(1);
        point.z = cloud1.col(i)(2);
        uint8_t r = rgb1(0), g = rgb1(1), b = rgb1(2);    // Example: Red color
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
        point.rgb = *reinterpret_cast<float*>(&rgb);
        pointClouds->points.push_back (point);
        }

    for(int i=0; i<transformed_cloud.cols(); ++i){
        pcl::PointXYZRGB point;
        point.x = transformed_cloud.col(i)(0);
        point.y = transformed_cloud.col(i)(1);
        point.z = transformed_cloud.col(i)(2);
        uint8_t r = rgb2(0), g = rgb2(1), b = rgb2(2);    // Example: Red color
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
        point.rgb = *reinterpret_cast<float*>(&rgb);
        pointClouds->points.push_back (point);
    }

    pointClouds->width = (int) pointClouds->points.size ();
    pointClouds->height = 1;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    viewer = rgbVis(pointClouds);
    viewer->setCameraPosition(-0.84,-0.84,-0.4,
                              0.0,0.0,1.0,
                                0.13,-0.84,-0.51);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }


}
