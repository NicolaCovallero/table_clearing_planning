/**
\example table_clearing_planning_test

\b Description: This examples show how to compute the blocks and onTop predicates
  for the objects standing on the table. The objects have to be previously segmented
  and given to the class as a vector of point clouds. To do the segmentation it is suggested
  to use the tos_supervoxel library. Also this example present an example of use of the 
  EdgeProcessing class that will be used to enhance the perception of the segmented objects
  by estimating in a deterministic way the occluded sides. 

\b Usage:
\code
$ ./table_clearing_planning_test cluttered_#.pcd 
\endcode
*/
#include "table_clearing_planning.h"
#include "tos_supervoxels.h"
#include <pcl/console/parse.h>


uint obj_idx = 0;
uint dir_idx = 1;
bool exit_ = false;
CTableClearingPlanning tcp;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                        void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym() == "n" && event.keyDown ())
  {
    if(obj_idx < tcp.getNumObjects())
    {
      viewer->removeAllShapes();
      tcp.viewerAddPrincipalDirections(viewer,obj_idx);
      tcp.cleanPolygonalMesh(viewer);
      tcp.visualComputeBlockPredicates(viewer,obj_idx,dir_idx);

      if(dir_idx == 4)
      { 
        dir_idx = 1;
        obj_idx++;
      }
      else
      {
        dir_idx++;       
      }
    }
    else
    {
      std::cout << "All objects visited - Exiting\n";
      exit_ = true;
    }
  }
}

int main(int argc, char *argv[])
{
  if(argc < 2)
  {
    std::cout << "Give as argument a point cloud" << std::endl;
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::console::print_highlight ("Loading point cloud...\n");
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud))
  {
    pcl::console::print_error ("Error loading cloud file!\n");
    return (1);
  }

  tcp.setOriginalPointCloud(*cloud);

  // For the segmentation: with our setup in the laboratory a 
  // simple euclidean clustering should work properly
  // segment point cloud
  tos_supervoxels seg;
  seg.init(*cloud);
  seg.set_zmin(0.03f);
  seg.print_parameters();
  seg.segment();
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > segmented_objs;
  segmented_objs = seg.get_segmented_objects_simple();
  pcl::ModelCoefficients plane_coeff = seg.get_plane_coefficients();
  
  EdgeProcessing ep;
  ep.setOriginalPointCloud(*cloud);
  ep.setObjects(segmented_objs);
  ep.setPlaneCoefficients(plane_coeff);  
  tic();
  ep.compute2DEdges();
  ep.computeOccludedSides2D(0.01);
  toc();

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_edge (new pcl::visualization::PCLVisualizer ("3D Viewer Edge"));
  ep.viewerInit(viewer_edge);
  ep.viewerAddAllEdges();
  ep.viewerAddOccludedSides();
  viewer_edge->addPointCloud(cloud,"original_cloud");
  ep.viewerSpin();
  viewer_edge->close();



  // scene perception - planning part
  //
  //  TODO:
  //  4) filters -> check this "Radius Outlier Removal"
  //
  tcp.setObjectsPointCloud(segmented_objs);
  //tcp.voxelizeObjects();
  tcp.setPlaneCoefficients(plane_coeff);
  tcp.setGripperSimpleModel(0.08, 0.1, 0.12, 0.025);
  double opening_width = 0.08;
  double closing_width = 0.025;
  double finger_width = 0.03;
  double deep = 0.03;
  double height = 0.115;
  double closing_height = 0.045;
  tcp.setFingersModel(opening_width,closing_width,finger_width,deep,height,closing_height);

  tcp.computeProjectionsOnTable();
  tcp.computeRichConvexHulls();
  tcp.computePrincipalDirections();
  tcp.computeAABBObjects(true);

  tcp.computeBlockPredicates(true, ORTHOGONAL_PUSHING);
  tcp.computeOnTopPredicates(true);

  //----------- VISUALIZATIONS ----------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  
  tcp.viewerShowClosedFingersModel(viewer);
  tcp.computeSimpleHeuristicGraspingPoses();
  tcp.computeBlockGraspPredicates(true);
  tcp.viewerAddGraspingPoses(viewer);
  tcp.viewerAddApproachingPoses(viewer);
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);  
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (0.3);
  //tcp.viewerAddRichObjectsClouds(viewer); 
  tcp.viewerAddPrincipalDirections(viewer,obj_idx);
  //tcp.viewerAddPrincipalDirections(viewer);
  
  // std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > occluded_sides;
  // occluded_sides = ep.getObjectsOccludedSides(); 
  // tcp.buildFullObjectsCloud(occluded_sides);
  // tcp.voxelizeFullObjects();
  // tcp.viewerAddFullObjectsClouds(viewer);

  tcp.viewerAddObjectsClouds(viewer);

  while (!viewer->wasStopped() && !exit_)
    viewer->spinOnce (100);
  viewer->close();
  
  return 0;
}