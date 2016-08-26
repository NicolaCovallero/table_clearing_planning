/**
\example table_clearing_planning_test

\b Description: This examples show how to compute the blocks and onTop predicates
  for the objects standing on the table. The objects have to be previously segmented
  and given to the class as a vector of point clouds. To do the segmentation it is suggested
  to use the tos_supervoxel library. Also this example presents an example of the use of the 
  EdgeProcessing class that will be used to enhance the perception of the segmented objects
  by estimating the occluded sides. 

  The EdgeProcessing class could be used to improved the library,
  for example instead of using the convex hull you can use the concave hull, more precise,
  but you need to fill the occluded sides with points. This final part is still not well integrated.

\b Usage:
\code
$ ./table_clearing_planning_test cluttered_#.pcd 
\endcode
*/
#include "table_clearing_planning.h"
#include "tos_supervoxels.h"
#include <pcl/console/parse.h>
#include <pcl/console/parse.h>

// all the measures are given in meters
double pushing_limit = 0.3;
double minimum_distance = 0.02;
double pushing_resolution = 0.03;
bool print = true;

// gripper model dimensions
double opening_width = 0.08;
double closing_width = 0.03;
double finger_width = 0.03;
double deep = 0.06;  // it hsould be 0.03 but in order to consider the width of the gripper's base
double height = 0.115;
double closing_height = 0.045;


uint obj_idx = 0;
uint dir_idx = 1;
bool exit_ = false;
CTableClearingPlanning tcp;

void help()
{
  pcl::console::print_error ("The command is: ./table_clearing_planning_test <pcd-file> \n"
  "The possible options are:\n"
  "-pl <value> : set the pushing limit value\n"
  "-pr <value> : set the reolution for the pushing\n"
  "-md <value> : set the minimum distance to the closest object to consider a grasping pose as feasible\n"
  "-print : to print into the terminal all the info regarding the predicates \n"
  "-gow <value> : set the opening width of the gripper\n"
  "-gcw <value> : set the closing width of the gripper\n"
  "-gfw <value> : set the fingers width of the gripper\n"
  "-gd <value> : set the deep of the gripper\n"
  "-gh <value> : set the height of the gripper\n"
  "-gch <value> : set the closing height of the gripper\n"
  "\nParameters for the segmentation:\n"
  "--NT Dsables the single cloud transform \n"
  "-v <voxel resolution>\n-s <seed resolution>\n"
  "-c <color weight> \n-z <spatial weight> \n"
  "-n <normal_weight>\n"
  "---LCCP params----\n"
  "-sc disable sanity criterion\n"
  "-ct concavity tolerance\n"
  "-st smoothness threshold\n"                                 
  "-ec enable extended criterion\n"
  "-smooth min segment size\n"
  "-- Others parameters ---\n"
  "-zmin minimum distance orthogonal to the table plane to consider a point as valid\n"
  "-zmax maximum distance orthogonal to the table plane to consider a point as valid\n"
  "-th_points minimum amount of points to consider a cluster as an object\n"
  "\n");
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                        void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym() == "n" && event.keyDown ())
  {
    if(obj_idx < tcp.getNumObjects())
    {
      viewer->removeAllShapes();
      //tcp.viewerAddOriginalPrincipalDirections(viewer,obj_idx);
      tcp.cleanPolygonalMesh(viewer);
      tcp.visualComputeBlockPredicates(viewer,obj_idx,dir_idx,true,true,ORTHOGONAL_PUSHING,pushing_resolution,pushing_limit,minimum_distance,true);
      tcp.viewerAddPushingGraspingPose(viewer,obj_idx,dir_idx);

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
  if(argc < 2 || pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
  {
    help();
    return -1;
  }

  // parameters for the LCCP segmentation
  tos_supervoxels_parameters opt;
  opt.voxel_resolution = 0.005;
  opt.seed_resolution = 0.02;
  opt.concavity_tolerance_threshold = 15;

  // Set the parameters given as aguments: ---------------------------------------------
  pcl::console::parse (argc, argv, "-pl", pushing_limit);
  pcl::console::parse (argc, argv, "-pr", pushing_resolution);
  pcl::console::parse (argc, argv, "-md", minimum_distance);

  pcl::console::parse (argc, argv, "-gow", opening_width);
  pcl::console::parse (argc, argv, "-gcw", closing_width);
  pcl::console::parse (argc, argv, "-gfw", finger_width);
  pcl::console::parse (argc, argv, "-gd", deep);
  pcl::console::parse (argc, argv, "-gh", height);
  pcl::console::parse (argc, argv, "-gch", closing_height);

  opt.disable_transform = pcl::console::find_switch (argc, argv, "--NT");
  pcl::console::parse (argc, argv, "-v", opt.voxel_resolution);
  pcl::console::parse (argc, argv, "-s", opt.seed_resolution);
  pcl::console::parse (argc, argv, "-c", opt.color_importance);
  pcl::console::parse (argc, argv, "-z", opt.spatial_importance);
  pcl::console::parse (argc, argv, "-n", opt.normal_importance);

  pcl::console::parse (argc, argv, "-ct", opt.concavity_tolerance_threshold);
  pcl::console::parse (argc, argv, "-st", opt.smoothness_threshold);
  opt.use_extended_convexity = pcl::console::find_switch (argc, argv, "-ec");
  opt.use_sanity_criterion = !pcl::console::find_switch (argc, argv, "-sc");
  pcl::console::parse (argc, argv, "-smooth", opt.min_segment_size);

  // table plane estimation - parameters
  pcl::console::parse (argc, argv, "-zmin", opt.zmin);

  // minimum amount of points to consider a cluster as an object
  pcl::console::parse (argc, argv, "-th_points", opt.th_points);
  // -------------------------------------------------------------------------------------------
  // print paramters (neglect the segmentation):

  std::cout << "pushing_limit set to: " << pushing_limit << std::endl;
  std::cout << "pushing_resolution set to: " << pushing_resolution << std::endl;
  std::cout << "minimum_distance set to: " << minimum_distance << std::endl;

  std::cout << "opening_width set to: " << opening_width << std::endl;
  std::cout << "closing_width set to: " << closing_width << std::endl;
  std::cout << "finger_width set to: " << finger_width << std::endl;

  std::cout << "deep set to: " << deep << std::endl;
  std::cout << "height set to: " << height << std::endl;
  std::cout << "closing_height set to: " << closing_height << std::endl;
  // ---------------------------------------------------------------------------------------------
  
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::console::print_highlight ("Loading point cloud...\n");
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud))
  {
    pcl::console::print_error ("Error loading cloud file!\n");
    return (1);
  }

  tcp.setOriginalPointCloud(*cloud);


  // Here the LCCP segmentation is used but you can use whatever kind of segmentation you prefer
  // tos_supervoxels class is a segmentation based on the LCCP algorithm to segment the objects above a table
  tos_supervoxels seg;
  seg.init(*cloud,opt);
  seg.set_zmin(0.03f);
  seg.print_parameters();
  seg.segment();
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > segmented_objs;
  segmented_objs = seg.get_segmented_objects_simple();
  pcl::ModelCoefficients plane_coeff = seg.get_plane_coefficients();
  


  // EdgeProcessing ep;
  // ep.setOriginalPointCloud(*cloud);
  // ep.setObjects(segmented_objs);
  // ep.setPlaneCoefficients(plane_coeff);  
  // tic();
  // ep.compute2DEdges();
  // ep.computeOccludedSides2D(0.01);
  // toc();

  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_edge (new pcl::visualization::PCLVisualizer ("3D Viewer Edge"));
  // ep.viewerInit(viewer_edge);
  // ep.viewerAddAllEdges();
  // ep.viewerAddOccludedSides();
  // viewer_edge->addPointCloud(cloud,"original_cloud");
  // ep.viewerSpin();
  // viewer_edge->close();

  std::cout << "Number of objects: " << segmented_objs.size() << std::endl;
  util::uint64 t_init = util::GetTimeMs64();
  tcp.setObjectsPointCloud(segmented_objs);
  tcp.setPlanePointCloud(*(seg.get_plane_cloud()));
  //tcp.setPushingStep(1.5);
  //tcp.voxelizeObjects();
  tcp.setPlaneCoefficients(plane_coeff);
  tcp.refineSegmentationByBiggestPlane();
  std::cout << "---------------- Refined segmented objects ------------------\n";

  tcp.setGripperModel(opening_width,closing_width,finger_width,deep,height,closing_height);
  tcp.setOpenGripperColor(255,255,255);
  tcp.setClosedGripperColor(100,100,100);
  tcp.computeProjectionsOnTable();
  tcp.computeRichConvexHulls();
  tcp.computePrincipalDirections();
  tcp.computeOBBObjects(true);

  tcp.computeSimpleHeuristicGraspingPoses(PCA_GRASPING);
  tcp.computeBlockPredicates(false, ORTHOGONAL_PUSHING, pushing_resolution,pushing_limit,minimum_distance,true);
  tcp.computeOnTopPredicates(true);
  tcp.computeBlockGraspPredicates(true);
  tcp.printExecutionTimes();

  std::cout << "Total time: " << (double)(util::GetTimeMs64() - t_init) << std::endl;

  //----------- VISUALIZATIONS ----------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // set the camera parameters in order to have the same view always. You could need to change it, depends on the experiment
  viewer->loadCameraParameters("camera_parameters.cam"); 
  viewer->setSize(500,400); // set size of the window

  //tcp.viewerShowClosedFingersModel(viewer);
  //tcp.viewerShowFingersModel(viewer);
  tcp.viewerAddGraspingPoses(viewer);
  //tcp.viewerAddGraspingPose(viewer,0);
  //tcp.viewerAddApproachingPoses(viewer);
  tcp.viewerAddPlaneConvexHull(viewer,255,255,255);
  //tcp.viewerAddOriginalPrincipalDirections(viewer,1);
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);  
  viewer->setBackgroundColor (255, 255, 255);
  //viewer->addCoordinateSystem (0.3);


  //tcp.viewerAddObjectsClouds(viewer);
  //tcp.viewerAddProjection(viewer,0,0,255,0);
  //tcp.viewerAddProjections(viewer);
  //tcp.viewerAddProjectionConvexHull(viewer,1,255,0,0);
  //tcp.viewerAddRichObjectsClouds(viewer); 
  //tcp.viewerAddPrincipalDirections(viewer,obj_idx);
  //tcp.viewerAddPrincipalDirections(viewer);
  //tcp.viewerAddGraspingPose(viewer,0);
  // std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > occluded_sides;
  // occluded_sides = ep.getObjectsOccludedSides(); 
  // tcp.buildFullObjectsCloud(occluded_sides);
  // tcp.voxelizeFullObjects();
  // tcp.viewerAddFullObjectsClouds(viewer);

  //tcp.viewerAddObjectsClouds(viewer);
  
  for (uint i = 0; i < segmented_objs.size(); ++i)
  {
   tcp.viewerAddConvexHulls(viewer,i);  
  }
  // tcp.viewerAddConvexHulls(viewer,0);  
  // tcp.viewerAddConvexHulls(viewer,1);  

  //tcp.viewerAddConvexHulls(viewer,1);  
  //tcp.viewerAddConvexHulls(viewer,5);  
  //tcp.viewerAddConvexHulls(viewer,4);  
  
  
  // tcp.viewerAddPushingGraspingPose(viewer,2,1);
  // tcp.viewerAddPushingGraspingPose(viewer,2,2);
  // tcp.viewerAddPushingGraspingPose(viewer,2,3);
  // tcp.viewerAddPushingGraspingPose(viewer,2,4);


  tcp.printPushingLengths();
  tcp.printPushingEEDistances();

  // tcp.testFclDistance();

  while (!viewer->wasStopped() && !exit_)
    viewer->spinOnce (100);
  viewer->close();

  tcp.printPushingLengths();
  tcp.printPushingEEDistances();
  
  return 0;
}
