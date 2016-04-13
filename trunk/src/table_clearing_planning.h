#ifndef _TABLE_CLEARING_PLANNING_H
#define _TABLE_CLEARING_PLANNING_H

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/common/pca.h>
#include <Eigen/Dense>
#include <string>
#include <sstream>

// translations and projections
#include <pcl/common/geometry.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

// filters
#include <pcl/filters/crop_hull.h>
#include <pcl/2d/morphology.h>
#include <pcl/filters/voxel_grid.h>

// FCL library
#include "fcl/data_types.h"
#include "fcl/math/vec_3f.h"
#include "fcl/math/matrix_3f.h"
#include "fcl/BVH/BVH_model.h"
#include "fcl/BV/BV.h"
#include "fcl/collision_object.h"
#include "fcl/collision.h"
#include "fcl/collision_data.h"
#include <ctime>

#include "utilities.h"
#include "edge_processing.h"


struct ObjectFull
{
  pcl::PointCloud<pcl::PointXYZRGBA> cloud; ///< Point cloud of the object composed of [object + occluded_sides + projection]. It is an unorganized one
  uint index_sides; ///< Index of the point cloud that correspond to the index of the first point of the occluded sides
  uint index_projection; ///< Index of the point cloud that correspond to the index of the first point of the projection
};

/**
* @brief     structure to represent the first 2 pricipal directions as a 3D vector and the centroid of 
*       the object. The principal directions are projected on the plane.
*/
struct PrincipalDirectionsProjected{
  Eigen::Vector3f dir1,dir2,dir3,dir4; ///< direction of the principal direction (dir1 & dir2) and the orthogonal (dir3 & dir4)
  Eigen::Vector4f centroid; ///< centroid of the object

  double dir1_limit;
  double dir3_limit;
};

struct OriginalPrincipalDirections
{
  Eigen::Vector3f p1,p2,p3;
  Eigen::Vector4f centroid;
};

/**
 * @brief Block predicates structure 
 * 
 */
struct BlocksPredicate{
std::vector<uint> block_dir1,
                  block_dir2,
                  block_dir3,
                  block_dir4;
};
// In this way the object is stored in the index of blocks_predicates
// and for each block_dir# there is the index of the object it is colliding
// in that principal direction. So to now all the object that block 
// the object 1 in direction 1 :
// for (int i = 0; i < this->blocks_predicates[i].block_dir1.size(); ++i)
// {
//   /* code */
// }

struct GraspingPose{
  Eigen::Matrix3f rotation;
  Eigen::Vector3f translation;
};


// check this
// http://hamelot.io/programming/using-bullet-only-for-collision-detection/
// for collision: http://stackoverflow.com/questions/5559482/how-would-i-set-up-collision-using-the-bullet-physics-library

// collision tutorial:
// https://www.toptal.com/game/video-game-physics-part-ii-collision-detection-for-solid-objects 
// http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Collision_Callbacks_and_Triggers

// reduce convex hull for optimized collision detection - BULLET:
//http://www.bulletphysics.org/mediawiki-1.5.8/index.php?title=BtShapeHull_vertex_reduction_utility


/**
 * @brief       This class computes the predicates of the table top objects for the planning part
 * 				The class needs as input the segmented objects and the table plane coefficients.
 * 				Be sure to given the objects in input with respect the desired reference frame. 
 */	
class CTableClearingPlanning
{
  typedef pcl::PointXYZRGBA PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> Visualizer;

  struct FclMesh
  {
      // set mesh triangles and vertice indices
      std::vector<fcl::Vec3f> vertices;
      std::vector<fcl::Triangle> triangles;
  };

  static const double PUSHING_LIMIT = 0.1; // default value 0.1 meters = 10 cm
  double pushing_limit;


  double n_objects; ///< number of objects

  // ----------- PREDICATES ---------------
  std::vector<BlocksPredicate> blocks_predicates;
  std::vector<std::vector<uint> > on_top_predicates;  
  // --------------------------------------

  // ------------- Gripper Model ----------
  // We here define the gripper model, we just use its AABB
  struct EndEffector{
    double width;
    double deep;
    double height;

    double distance_plane; ///< distance from the plane 

    // convex hull
    pcl::PointCloud<pcl::PointXYZ > cloud;
    std::vector<pcl::Vertices> vertices;

  }ee_simple_model;///< end effector simple model

  struct FingersModel{
    double opening_width;
    double finger_width;
    double deep;
    double height;
    double closing_height;

    // convex hull
    pcl::PointCloud<pcl::PointXYZ > cloud;
    std::vector<pcl::Vertices> vertices;

  }fingers_model;



  PointCloudT original_cloud;
  std::vector<PointCloudT > objects;///< vector of objects point cloud
  std::vector<PointCloudT > rich_objects;
  std::vector<ObjectFull> objects_full;
  std::vector<PointCloudT > projections; 
  std::vector<PointCloudT > convex_hull_objects;///< vector of the convex hull of each object
  std::vector<PointCloudT > concave_hull_objects;///< vector of the concave hull of each object
  std::vector<PrincipalDirectionsProjected> principal_directions_objects;///< principal directions of 
  std::vector<OriginalPrincipalDirections> original_principal_directions_objects;
  std::vector<std::vector<pcl::Vertices> > convex_hull_vertices,concave_hull_vertices;

  std::vector<AABB> aabb_objects;

  std::vector<GraspingPose> grasping_poses;

  // projections on plane
  std::vector<PointCloudT > concave_hull_projections; 
  std::vector<std::vector<pcl::Vertices> > concave_hull_vertices_projections;
  std::vector<PointCloudT > convex_hull_projections; 
  std::vector<std::vector<pcl::Vertices> > convex_hull_vertices_projections;


  pcl::ModelCoefficients plane_coefficients; ///< coefficients of the plane model 
                                             ///< ax+by+cz+d=0
  Eigen::Vector3f plane_normal; ///< normals of the plane 
  Eigen::Vector3f plane_origin; ///< an arbitrary point of the plane to act like origin


  /**
   * @brief Check with the fcl library yf the transformed object is colliding with another one
   * @details [long description]
   * 
   * @param idx_1 index of the object to transform
   * @param tf transformation of the object
   * @param idx_2 index of the object to check if collides with transformed object 1
   * @return True if the transformed object 1 and the object 2 collides
   */
  bool areObjectCollidingFcl(uint idx_1, fcl::Transform3f tf, uint idx_2);

  /**
  * @brief Check if the gripper is colliding with object indexed by "idx"
  * @details [long description]
  * @return true if there is a collision with object indexed by "idx"
  */
  bool isEEColliding(uint idx, fcl::Transform3f tf);

  /**
   * @brief Get the mesh of the gripper model
   * @details [long description]
   * @return FclMesh containing the vertices and the triangles
   */
  FclMesh getGripperMesh();



  // --------- MY CONVERSIONS --------------------

  /**
   * @brief      Convert from PointT to Eigen::Vector3f
   *
   * @param[in]  point  point to convert
   *
   * @return     point in Eigen::Vector3f format
   */
  Eigen::Vector3f  pointT2vector3f(PointT point);


  /**
   * @brief   Convert the convex hull of the object with index "idx" from pcl to fcl format
   * @details [long description]
   * 
   * @param idx index of the convex hull to convert
   * @return struct which contains the vertices and triangle for the fcl format
   */
  FclMesh pcl2FclConvexHull(uint idx);

  
  void fcl2EigenTransform( Eigen::Vector4f& translate, Eigen::Quaternionf& q_rot,
                           fcl::Transform3f& tf);
  void eigen2FclTransform(Eigen::Vector4f& translate, Eigen::Quaternionf& q_rot,
                                                    fcl::Transform3f& tf);
  void eigen2FclRotation(Eigen::Quaternionf& q_rot, fcl::Matrix3f& rot);
  void eigen2FclRotation(Eigen::Matrix3f eig_rot, fcl::Matrix3f& rot);
  // -----------------
  
  /**
   * @brief      { function_description }
   *
   * @param      cloud1  { parameter_description }
   * @param      cloud2  { parameter_description }
   */
  void translate(PointCloudT& cloud1, PointCloudT& cloud2, Eigen::Vector4f& translation);

  /**
   * @brief Get the centroid of the grasping pose as the surface point of the object which projection on the table plane is similar to the centroid.
   * @details Get the centroid of the grasping pose as the surface point of the object which projection on the table plane is similar to the centroid.
   * 
   * @param centroid [description]
   * @param idx [description]
   */
  void computeSurfaceGraspPoint(Eigen::Vector3f& centroid, uint idx);

  public:

    CTableClearingPlanning();

    /**
     * @brief Initialize the class with the objects
     * 
     * @param objects 
     */
    CTableClearingPlanning(std::vector<PointCloudT > &objects);
    ~CTableClearingPlanning();

    /**
     * @brief Set the gripper model
     * @details Set the dimension of the bounding box of the gripper. This model will be used
     *          during the computation of the block predicates
     * 
     * @param[in] height Dimension of the gripper orthogonal to the plane during the pushing action
     * @param[in] deep Dimension of the gripper parallel to pushing direction during the pushing action
     * @param[in] width Dimension of the gripper orthogonal to pushing direction during the pushing action
     * @param[in] distance_plane Distance in meters from the table plane during the pushing action
     */
    void setGripperSimpleModel(double height, double deep, double width, double distance_plane);


    /**
     * @brief Set the fingers model
     * @details Set the fingers model. The origin is located at height/2, deep/2 and (opening_width + finger_width*2)/2
     * 
     * @image html fingers_model.png
     * 
     * @param opening_width 
     * @param finger_width [description]
     * @param deep [description]
     * @param height [description]
     * @param closing_height [description]
     */
    void setFingersModel(double opening_width, double finger_width,
               double deep, double height, double closing_height);

    /**
     * @brief Set the original point cloud, the one used for the segmentation. 
     * 
     * @param[in] original_cloud 
     */
    void setOriginalPointCloud(PointCloudT original_cloud);


    /**
     * @brief      set point clouds of the objects, reference to project the principal direction to the   plane:
     *             http://www.maplesoft.com/support/help/Maple/view.aspx?path=MathApps%2FProjectionOfVectorOntoPlane
     *
     * @param[in]  objects  vector of point clouds, one per object
     */
    void setObjectsPointCloud(std::vector<PointCloudT > &objects);

    /**
     * @brief      Set the coefficients of the table plane
     *
     * @param[in]  plane_coefficients  coefficients of the table plane
     */
    void setPlaneCoefficients(pcl::ModelCoefficients &plane_coefficients);

    void testTranslation(uint idx);

    /**
     * @brief Compute the Axis Aligned Bounding Box of each object
     * 
     * @param refine_centroids True if you want to refine the centroid by computing the mean of the bounding box
     */
    void computeAABBObjects(bool refine_centroids = true);  

    /**
     * @brief Compute the grasping pose with a simple heuristic.
     * @details Compute the grasping pose with a simple heuristic. 
     * Gripper centered at the centroid with rotation aligned to the principal axes.
     * 
     * @param vertical_poses If it is set to true consider only vertical poses, with respect the table.
     */
    void computeSimpleHeuristicGraspingPoses(bool vertical_poses = false);


    /**
     * @brief      Compute the convex hull of each object
     */
    void computeConvexHulls();

    /**
    * @brief      Compute the concave hull of each object
    */
    void computeConcaveHulls();


    /**
     * @brief      Compute the projections of the objects onto the table plane
     */
    void computeProjectionsOnTable();

    /**
     * @brief      Compute the convex hulls of the projections onto the table plane
     */
    void computeProjectionsConvexHull();

    /**
     * @brief      Compute the convex hull of each object considering also the projections of the 
     *             convex hulls on the plane. These points are then added to the original point cloud 
     *             and the convex hull is recomputed.
     *             IMPORTANT: we can project all the coud and do the convex hull, so it is computed only once
     */
    void computeRichConvexHulls();


    /**
     * @brief      Compute the concave hull of each object considering also the projections of the 
     *             convex hulls on the plane. These points are then added to the original point cloud 
     *             and the convex hull is recomputed.
     *             IMPORTANT: THE CONCAVE HULL IS NOT WELL IMPLEMENTED
     *             SEE: http://www.pointclouds.org/blog/gsoc/goleary/tutorials/concave_hull.php
     */
    void computeRichConcaveHulls();

    /**
     * @brief      Compute the on predicates
     * 
     *             To decide if an oject is on top ot the other one, we work with the convex hulls
     *              of the projections and the projected points. 
     *              Since the pow of the kinect make that the on top objects hide part of the bottom objects
     *              the strategy is the following:
     *                1) for objet 1 check if its points (choose randomly) lies inside the convex hull of the other one
     *                2) if a point lieas inside the convex hull of the other one, tdo the same with the object 2.
     *                3) if object 1 lies inside the convex hull of the object 2, but object 2 does'nt, the object 1 is on
     *                   top of object 2. MAYBE A EROSION TECHNIQUE WILL BE REQUIRED IN ORDER TO AVOID SITUATIONS WHERE 
     *                   THE PROJECTED POINTS ARE TOO MUCH CLOSE
     *                
     */
    void computeOnTopPredicates();

    /**
     * @brief Set the pushing limit
     * @details If it is not setted a default value will be used. The pushing limit is here 
     *          defined as the maximum length (in meters) you want to push the object and therefore 
     *          it acts on the block predicates computation.
     * 
     * @param[in] pushing_limit Desired value for the pushing limit.
     */
    void setPushingLimit(double pushing_limit);

    /**
     * @brief Get block predicates
     * @details This method computes the block predicates for a each object by detecting
     *          collision with the other objects, determining also what object collides 
     *          with the current one, along each principal direction. The length of the push
     *          considered in saved in the private memeber 
     */
    void computeBlockPredicates(bool print=false);


    void testFcl();
    void testFcl2();


    /**
     * @brief    compute the principal directions of the objects
     *           using the point cloud of the convex hull.
     *           (The results should be not as good as computePrincipalDirections())
     */
    void computePrincipalDirectionsConvexHull();
    void computePrincipalDirectionsConcaveHull();

    /**
     * @brief    compute the principal directions of the objects.
     * @details  This is better
     *           because the others that use the hulls the amount of points of the hulls 
     *           depends on the complexity of the shape, so the principal directions would
     *           be affected by the complexity of the shape and not on the quanrity of points. 
     *           This method also include a verse detection in order to have alway dir3 to
     *           the right of dir1 with respect the top of the table. 
     * @image html directions.jpg  
     */
    void computePrincipalDirections();

    /**
     * @brief Voxelize objects
     * @details Remember that this will affect also the on top predicates
     * @param leaf_size Leaf size
     */
    void voxelizeObjects(double leaf_size = 0.01f);

    /**
     * @brief Voxelize Full Objects
     * @details Voxelize Full Objects
     * 
     * @param leaf_size Leaf size
     */
    void voxelizeFullObjects(double leaf_size = 0.01f);


    void buildFullObjectsCloud(std::vector<PointCloudT>& occluded_sides);

    // ------------------- VISUALIZATIONS -----------------------------

    /**
     * @brief Compute the block predicates
     * @details This method computes the block predicates for a certain object and
     *          a certain direction by detecting
     *          collision with the other objects, determining also what object collides 
     *          with the current one.
     *          It also add the viewer all the relative information in order to have a visual
     *          feedback of the process of computing the block predicates. 
     *          The block predicates are then saved in the private member "blocks_predicates".
     * 
     * @param[in] obj_idx Index of the object
     * @param[in] dir_idx Index of the direction [1,2,3,4]
     * @param[in] visualization True to enable the visualization
     * @param[in] print True to print in the terminal each block predicate
     */
    void visualComputeBlockPredicates(Visualizer viewer, uint obj_idx, uint dir_idx,bool visualization = true,
                                      bool print = true);


    /**
     * @brief      Show the convex hulls
     *
     * @param[in]      idx     index of the object to show 
     */
    void viewerAddConvexHulls(Visualizer viewer, uint idx);

    /**
     * @brief      Show the concave hulls
     *
     * @param[in]      idx     index of the object to show 
     */
    void viewerAddConcaveHulls(Visualizer viewer, uint idx);

    /**
     * @brief      Shows the projections of the principal directions of a specific object.
     *
     * @param[in]  i       index of the object 
     */
    void viewerAddPrincipalDirections(Visualizer viewer, uint i);

    /**
     * @brief      Add to the viewer associated to the class the projections 
     *              of the principal directions of each object.
     *
     */
    void viewerAddPrincipalDirections(Visualizer viewer);

    /**
     * @brief Remove from the the viewer associated to the class the principal
     *        directions of a specific object
     * 
     * @param[in] i Index of the object
     */
    void cleanPrincipalDirections(Visualizer viewer, uint i);

    /**
     * @brief Remove the viewer associated to the class all the polygonal mesh
     */
    void cleanPolygonalMesh(Visualizer viewer);


    void viewerAddPlaneNormal(Visualizer viewer);
    void viewerAddObjectsClouds(Visualizer viewer);
    void viewerAddRichObjectsClouds(Visualizer viewer);
    void viewerAddFullObjectsClouds(Visualizer viewer);

    /**
     * @brief      Show the labels in the viewer as 3D text
     *
     *              It is not working, I don't know why. 
     *
     * @param[in]      viewer  { parameter_description }
     */
    void viewerAddObjectsLabel(Visualizer viewer);

    void viewerShowFingersModel(Visualizer viewer);

    /**
     * @brief Show in the viewer associated to the class a transformed object. 
     *        ONLY FOR DEBUGGING PURPOSE - USELESS
     */
    void viewerAddObjectsTransfomed(Visualizer viewer);

    void viewerAddGraspingPose(Visualizer viewer,uint idx);
    void viewerAddGraspingPoses(Visualizer viewer);

    uint getNumObjects();
    std::vector<ObjectFull> getFullObjects();

    std::vector<AABB> getAABBObjects();

    std::vector<BlocksPredicate> getBlockPredicates();
    std::vector<std::vector<uint> > getOnTopPrediates();  

};



#endif

