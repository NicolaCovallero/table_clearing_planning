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
#include <pcl/filters/crop_hull.h>
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
#include "fcl/collision_node.h"
#include "fcl/collision.h"
#include "fcl/collision_data.h"
#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/distance.h"

//#include "fcl_resources/config.h"


#include <ctime>

#include "utilities.h"
#include "edge_processing.h"

#define ORTHOGONAL_PUSHING 0
#define PARALLEL_PUSHING 1
#define VERTICAL_GRASPING 1
#define PCA_GRASPING 0

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
  Eigen::Vector3f p1,p2,p3;//p1 orthogonal to p2 and to p3, they have no the same meaning of dir1,dir2,dir3
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
  Eigen::Quaternionf quaternion;
};

/**
 * @details Grasping poses after pushing
 * 
 */
struct PushingGraspingPose{
  GraspingPose gp_dir1,gp_dir2,gp_dir3,gp_dir4;
  GraspingPose app_dir1,app_dir2,app_dir3,app_dir4;
};

struct Pose{
  Eigen::Matrix3f rotation;
  Eigen::Vector3f translation;
  Eigen::Quaternionf quaternion;
};

/**
 * @brief Structure for the pushing pose
 * @details It includes the pushing poses for the several pushing directions
 *          and the distacne between the gripper and the closest point of the other objects
 * 
 */
struct PushingPose{
  Pose pose_dir1, pose_dir2, pose_dir3, pose_dir4;
  double dist_dir1,dist_dir2,dist_dir3,dist_dir4;
};

/**
 * @brief Structure which includes all the relevan elapsed times of the algorithm
 * @details Structure which includes all the relevan elapsed times of the algorithm
 * 
 */
struct ExecutionTimes{
double on_predicates;
double block_predicates;
double block_grasp_predicates;
double objects_collisions; ///< total time to compute the collision between all the objects O(n^2)
double ee_collisions; ///< total time to compute the collision between the gripper and all the objects O(n^2)
double average_objects_collision; ///< average time to compute a collision between two objects
double average_ee_collision; ///< average time to compute a collision between two objects
};

/**
 * @details Struct which defines the pushing length for each direction
 * 
 */
struct PushingLength
{
  double dir1,dir2,dir3,dir4;
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

  // constant values
  
  static const double PUSHING_STEP = 1.5; // default value 1.5 the OBB dimension
  static const double PUSHING_OBJECT_DISTANCE = 0.05; //5 cm
  static const double APPROACHING_DISTANCE = 0.10; // 10 cm
  static const double DISTANCE_FROM_PLANE_GRASPING_POSES = 0.01; // 1cm



  double pushing_limit; //used only from the block predicates computation functions
  double pushing_step; // k factor, push the object for a distance equal to pushing_step * dimension(pushing_direction)
  double pushing_object_distance; ///< Distance between the tcp and the object for the first point of the pushing action

  uint n_objects; ///< number of objects

  // ----------- PREDICATES ---------------
  std::vector<BlocksPredicate> blocks_predicates;
  std::vector<std::vector<uint> > on_top_predicates;  
  std::vector<std::vector<uint> > block_grasp_predicates;  
  // --------------------------------------

  // ------------- Gripper Model ----------
  /**
   * @brief Structure to define the end effector model
   * @details This structure is used only when you use the PARALLEL pushing pose method (not recommended).
   * 
   */
  struct EndEffector{
    double width;
    double deep;
    double height;

    double distance_plane; ///< distance from the plane 

    // convex hull
    pcl::PointCloud<pcl::PointXYZRGB > cloud;
    std::vector<pcl::Vertices> vertices;

  }ee_simple_model;///< end effector simple model

  /**
   * @brief Structure to define the gripper model
   * @details Structure to define the gripper model for a two finger parallel gripper with a fixed
   *          opening width. It includes all the relevant measures to built a simple model. 
   * 
   */
  struct GripperModel{
    double opening_width;
    double closing_width;
    double finger_width;
    double deep;
    double height;
    double closing_height;

    // convex hull
    pcl::PointCloud<pcl::PointXYZRGB> open_cloud; ///< point cloud of the gripper open
    pcl::PointCloud<pcl::PointXYZRGB> closed_cloud; ///< point cloud of the gripper closed
    std::vector<pcl::Vertices> open_vertices,closed_vertices; ///< vertices of the convex hull

  }gripper_model;

  ExecutionTimes executionTimes; ///< Data regarding the elapsed times of the several phases of the algorithm

  PointCloudT original_cloud; ///< Original point cloud (The oen seen by the Kinect)
  
  PointCloudT plane_cloud;
  PointCloudT plane_convex_hull_2d;
  std::vector<pcl::Vertices> plane_convex_hull_indices;

  std::vector<PointCloudT > objects;///< vector of objects point cloud
  pcl::PointCloud<pcl::PointXYZL> objects_labeled_cloud; ///< A single point cloud with the objects are labelled
  std::vector<PointCloudT > rich_objects;///< Add also the projections onto the table plane to the segmented objects
  std::vector<ObjectFull> objects_full; 
  std::vector<PointCloudT > projections; ///< projections of the segmented objects onto the table plane 
  std::vector<PointCloudT > convex_hull_objects;///< vector of the convex hull of each object
  std::vector<PointCloudT > concave_hull_objects;///< vector of the concave hull of each object
  std::vector<PrincipalDirectionsProjected> principal_directions_objects;///< principal directions projected onto the table plane 
  std::vector<OriginalPrincipalDirections> original_principal_directions_objects; ///< Original principal directions = principal components
  std::vector<std::vector<pcl::Vertices> > convex_hull_vertices,concave_hull_vertices;///< vertices of the hulls


  std::vector<PushingLength> pushing_lengths; ///< The pushing length for each object and pushing direction
  std::vector<PushingGraspingPose> pushing_grasping_poses; // grasping poses estimated for the object once pushed


  std::vector<OBB> obb_objects; ///< Oriented Bounding Box of each object

  std::vector<GraspingPose> grasping_poses,approaching_poses; ///< Grasping and approaching poses
  double approaching_distance; ///< Distance between the approaching distance and the grasping pose
  double distance_from_plane_grasping_poses; ///< Parameter which specifies how much the gripper should be far from the table in the graspin pose (This is a safety measure in roder to avoid collision between the gripper and the table)

  std::vector<PushingPose> pushing_poses;///< Pose of the end effector when push 

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
  * @details Check if the gripper is colliding with object indexed by "idx".
  *           NOTE: this function is used only when you choose a pushing pose
  *           parallel to the table plane. The usggested one is the vertical/ortoghonal one. 
  * @return true if there is a collision with object indexed by "idx"
  */
  bool isEEColliding(uint idx, fcl::Transform3f tf, double& distance);

  /**
   * @details Collission checking for the case we push in orhtoganl mode
   * 
   * @param[in] idx Index of the object 
   * @param[in] tf tf of the gripper
   * @param[out] distance distance to the closest point 
   * 
   * @return true if there is a collision with object indexed by "idx"
   */
  bool isClosedGripperModelColliding  (uint idx, fcl::Transform3f tf, double& distance);

  /**
   * @brief Check if the open gripper model is colliding with other objects
   * @details Check if the open gripper model is colliding with other objects
   * 
   * @param[in] idx Index of the object 
   * @param[in] tf tf of the gripper
   * @param[out] distance distance to the closest point 
   * 
   * @return true if there is collision, false otherwise
   */
  bool isOpenGripperModelColliding(uint idx, fcl::Transform3f tf, double& distance);

  /**
   * @brief Get the mesh of the gripper model
   * @details [long description]
   * @return FclMesh containing the vertices and the triangles
   */
  FclMesh getGripperMesh();

  /**
   * @brief Get the mesh of the closed gripper model
   * @details [long description]
   * @return FclMesh containing the vertices and the triangles
   */
  FclMesh getClosedFingersMesh();

  FclMesh getGripperModelMesh();

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
   * @details   Convert the convex hull of the object with index "idx" from pcl to fcl format
   * 
   * @param idx index of the convex hull to convert
   * @return struct which contains the vertices and triangle for the fcl format
   */
  FclMesh pcl2FclConvexHull(uint idx);

  /**
   * @details Convert the convex hull from pcl to fcl format
   * 
   * @param convex_hull_cloud 
   * @param convex_hull_indices 
   * 
   * @return Fcl mesh
   */
  FclMesh pcl2FclConvexHull(PointCloudT convex_hull_cloud, std::vector<pcl::Vertices> convex_hull_indices);


  /** 
   * @brief Refine the pushing in pose in order to avoid it to be too close to the table
   * 
   * @param x
   * @param y
   * @param z
   * @param safe_margin minimum distace from the table plane to consider the pushing pose not colliding with the table
   */
  void refinePushingPose(Eigen::Vector3f &point,  double safe_margin = 0.015);
  
  void fcl2EigenTransform( Eigen::Vector4f& translate, Eigen::Quaternionf& q_rot,
                           fcl::Transform3f& tf);
  void eigen2FclTransform(Eigen::Vector4f& translate, Eigen::Quaternionf& q_rot,
                                                    fcl::Transform3f& tf);
  void eigen2FclTransform(Eigen::Vector3f& translate, Eigen::Quaternionf& q_rot,
                                                    fcl::Transform3f& tf);
  void eigen2FclRotation(Eigen::Quaternionf& q_rot, fcl::Matrix3f& rot);
  void eigen2FclRotation(Eigen::Matrix3f eig_rot, fcl::Matrix3f& rot);
  // -----------------
  
  /**
   * @brief      Test function
   */
  void translate(PointCloudT& cloud1, PointCloudT& cloud2, Eigen::Vector4f& translation);

  /**
   * @brief Get the centroid of the grasping pose as the surface point of the object which projection on the table plane is similar to the centroid.
   * @details Get the centroid of the grasping pose as the surface point of the object which projection on the table plane is similar to the centroid.
   * 
   * @param[out] surface_point surface point
   * @param[in] idx Index of the object
   */
  void computeSurfaceGraspPoint(Eigen::Vector3f& surface_point, uint idx);

  /**
   * @details Refines the input grasping pose, that is if the gripper
   * central point is further than the plane. It satisfy the following equation
   * \code 
   * a*x + b*y + c*(z+offset) * d > 0 
   * \endcode
   * If such an inequality is true the grasping pose has to be refine, it is translated along
   * its approaching direction by translation_step until that inequalities is no more satisfied.
   * Note the this method doesn't check if after the refining the grasp is still feasible, bu the height of
   * the object should be coherent with the one of the distance from the plane. 
   * 
   * @param gp Input grasping pose
   * @param translation_step 
   * @param distance_from_plane minimum distance from the plane, bigger it is further the plane it is. 
   * 
   * @return True if there was initially in collision. 
   */
  bool refineSimpleGraspingPose(GraspingPose& gp, double translation_step = 0.005, double distance_from_plane = 0.01);

  //void getClosestObjectAndDistance( ,uint object_label, double distance);

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
     *          during the computation of the block predicates if you choose PARALLEL_PUSHING(not recommended).
     *          You don't need to set this model if you do not use the PARALLEL_PUSHING. Here the gripper
     *          is considered as a cube. 
     * 
     * @param[in] height Dimension of the gripper orthogonal to the plane during the pushing action
     * @param[in] deep Dimension of the gripper parallel to pushing direction during the pushing action
     * @param[in] width Dimension of the gripper orthogonal to pushing direction during the pushing action
     * @param[in] distance_plane Distance in meters from the table plane during the pushing action
     */
    void setGripperSimpleModel(double height, double deep, double width, double distance_plane, uint r=1, uint g=255, uint b=255);

    
    

    /**
     * @brief Set the fingers model
     * @details Set the fingers model. The origin is located at height/2, deep/2 and (opening_width + finger_width*2)/2
     * 
     * @image html gripper_model.png
     * 
     * @param opening_width 
     * @param closing_width
     * @param finger_width [description]
     * @param deep [description]
     * @param height [description]
     * @param closing_height [description]
     */
    void setGripperModel(double opening_width, double closing_width, double finger_width,
               double deep, double height, double closing_height, uint r=255, uint g=255, uint b=255);

    void setOpenGripperColor (uint r, uint g, uint b);
    void setClosedGripperColor (uint r, uint g, uint b);

    /**
     * @brief Set the plane point cloud
     * 
     * @param plane_cloud 
     */
    void setPlanePointCloud(PointCloudT plane_cloud);

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
     * @param[in]  convert_to_label true if you want to create a unique point cloud labeled,
     */
    void setObjectsPointCloud(std::vector<PointCloudT > &objects, bool convert_to_label = false);

    /**
     * @brief      Set the coefficients of the table plane
     *
     * @param[in]  plane_coefficients  coefficients of the table plane
     */
    void setPlaneCoefficients(pcl::ModelCoefficients &plane_coefficients);

    void testTranslation(uint idx);

    /**
     * @brief Function to test the distance of FCL for two simple mesh models
     * @details You should see the code to understand how this work. There is a model of a simple
     * pyramid, and a transformation defined in order to check that the distance is correct. 
     */
    void testFclDistance();

    /**
     * @brief Compute the Axis Aligned Bounding Box of each object
     * 
     * @param refine_centroids True if you want to refine the centroid by computing the mean of the bounding box
     */
    void computeOBBObjects(bool refine_centroids = true);  

    /**
     * @brief Compute the grasping pose with a simple heuristic.
     * @details Compute the grasping pose with a simple heuristic. 
     * Gripper centered at the centroid with rotation aligned to the principal axes.
     * The contact point is choosen to be the nearest point which projection on the table plane is 
     * the enarest on the projection of the centroid.
     * 
     * @param vertical_poses If it is set to true consider only vertical poses, with respect the table.
     */
    void computeSimpleHeuristicGraspingPoses(bool vertical_poses = PCA_GRASPING);

    
    /** 
     * @brief Refine the segmentation of the objects by considering only the biggest plane parallel, up to a certain threshold, to the table plane. If the biggest plane is a side of the object this plane is not considered.  
     */
    void refineSegmentationByBiggestPlane();

    /**
     * @brief      Compute the convex hull of each object
     */
    void computeConvexHulls(bool rand_=true);

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
     *             
     * @param rand_ If it is true a random color is assigned to each convex hull          
     */
    void computeRichConvexHulls(bool rand_=true);


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
     * @param th1 threshold: how many points of the objects which is on the top can lie inside the
     * convex hull of the other
     * @param th2 The inverse of th1, a correct balance between them we can achieve a nice result               
     */
    void computeOnTopPredicates(double th1 =100, double th2 = 100, bool print = false);

    /**
     * @brief Set the pushing limit
     * @details If it is not setted a default value will be used. The pushing step
     * is here defined as the maximum times of the OBB dimension to be pushed. 
     * (e.g. : If the pushing_step is 1.5 considering to push ht eojbect 1 in direction 1 
     *  it is pushed for 1.5 the OBB dimension relative to direction 1 (which is the OBB.deep))
     * 
     * @param[in] pushing_step Desired value for the pushing limit.
     */
    void setPushingStep(double pushing_step);

    /**
     * @details Minimum distance of the gripper from the plane, this is in case there are poses too close the plane.
     * The default values is 1 cm.
     * 
     * @param distance_from_plane_grasping_poses 
     */
    void setDistanceFromPlaneForGripper(double distance_from_plane_grasping_poses);

    /**
     * @details distance in meter between the grasping pose and the approaching pose. 
     * The approaching pose is the pose of the gripper where it will be opened.
     * 
     * @param approaching_distance Distance in meters
     */
    void setApproachingDistance(double approaching_distance);

    /**
     * 
     * @details This distance is the distance of the end effector from the object, it is like an offset.
     * 
     * @param pushing_object_distance distance from the objects, in meters
     */
    void setPushingObjectDistance(double pushing_object_distance);

    /**
     * @brief Get block predicates
     * @details This method computes the block predicates for a each object by detecting
     *          collision with the other objects, determining also what object collides 
     *          with the current one, along each principal direction. The length of the push
     *          considered in saved in the private memeber. THis functions also computes
     *          the new grasping poses for a pose in which the object, when it has been pushed, 
     *          can be grasped (If it is not possible grasping it in any pose until having reached 
     *          the maxmimum pushing distance [pushing_limit] the returned grasping pose will be the one at the 
     *          final checked pose).  
     *          
     * @param[in] print True if you want to print in the terminal the predicates
     * @param[in] pushing_method There exist two pushing method: 1) ORTHOGONAL_PUSHING is the one recommended
     *            and it referes to put the end effector orthogonal to the plane 2) PARALLEL_PUSHING refers to
     *            the end effector approaching direction is parallel to the plane (Not recommended)
     * @param[in] resolution It specifies the resolution of checking for the pushing action
     * @param[in] pushing_limit It specifies the maximum limit (in meters) the method checks for collision and grasping. 
     * N          Note that as soon as it finds a pushing length for which the next pose corresponds to a feasible grasp it checks 
     *            no more for further distances         
     * @param[in] minimum_distance Minimum distance from the closest object to consider to future grasp a feasible one.         
     * @param[in] pushing_until_graspable pushing_until_graspable True if the algorithm push until the object can be grasped, otherwise it pushes for
     *            for a length equal to the dimension relative to the pushing direction. The distanceof the end effector during pushing are anyway computed.  
     *                   
     */
    void computeBlockPredicates(  bool print=false, uint pushing_method = ORTHOGONAL_PUSHING, 
                                  double resolution = 0.05, double pushing_limit = 0.2, double minimum_distance = 0.02,
                                  bool pushing_until_graspable = true, double pushing_fixed_distance = 0.0);

    /**
     * @brief Get block grasp predicates
     * @details This method computes the block grasp predicates for each object by detecting 
     *          collision of the open gripper model with the other objects.
     * 
     * @param print True if you want to print in the terminal the predicates.
     * @param minimum_grasping_distance Minimum distance to the closest object to consider the grasp as collision free
     */
    void computeBlockGraspPredicates(bool print=false, double minimum_grasping_distance = 0.02 );


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
     * @param[in] pushing_until_graspable True if the algorithm push until the object can be grasped, otherwise it pushes for
     *            for a length equal to the dimension relative to the pushing direction. The distanceof the end effector during pushing are anyway computed.  
     * @param[in] show_ee True if you want to see the ee for each pushed length
     * @param[in] pushing_fixed_distance Maximum distance to push. This is used only for the experiment comparison. 
     */
    void visualComputeBlockPredicates(Visualizer viewer, uint obj_idx, uint dir_idx,bool visualization = true,
                                      bool print = true, uint pushing_method = ORTHOGONAL_PUSHING,
                                      double resolution = 0.05, double pushing_limit = 0.2, double minimum_distance = 0.02,
                                      bool pushing_until_graspable = true, bool show_ee = false, double pushing_fixed_distance = 0.0);

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
     * @brief      Add to the viewer associated to the class the projections 
     *              of the original principal directions of each object.
     *
     */
    void viewerAddOriginalPrincipalDirections(Visualizer viewer, uint i);

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
    void viewerAddPlaneCloud(Visualizer viewer);

    /**
     * @details Add to the viewer the projections. Moreover each projection  point cloud 
     * is colored with a random color. 
     * 
     * @param viewer viewer
     */
    void viewerAddProjections(Visualizer viewer);

    /**
     * @details Add to the viewer a projection colored with a random color.
     * 
     * @param viewer viewer
     * @param idx index of the object
     */
    void viewerAddProjection(Visualizer viewer,uint idx);

    /**
     * @details Add to the viewer a projection colored with a specifed color.
     * 
     * @param viewer viewer
     * @param idx index of the object
     * @param r  red channel [0-255]
     * @param g  green channel [0-255]
     * @param b  blue channel [0-255]
     */
    void viewerAddProjection(Visualizer viewer,uint idx,float r, float g, float b);


     /**
     * @details Add to the viewer the convex hull of the projection of the idx object. 
     * 
     * @param viewer viewer
     * @param idx index of the object
     */
    void viewerAddProjectionConvexHull(Visualizer viewer,uint idx);


     /**
     * @details Add to the viewer the convex hull of the projection of the idx object with the
     * color of the convex hull given as input. 
     * 
     * @param viewer viewer
     * @param idx index of the object
     * @param r  red channel [0-255]
     * @param g  green channel [0-255]
     * @param b  blue channel [0-255]
     */
    void viewerAddProjectionConvexHull(Visualizer viewer,uint idx,float r, float g, float b);

    /**
     * @details
     * 
     * @param viewer [description]
     * @param r [description]
     * @param g [description]
     * @param b [description]
     */
    void viewerAddPlaneConvexHull(Visualizer viewer, double r=1,double g=1,double b=1);

    /**
     * @brief      Show the labels in the viewer as 3D text
     *
     *              It is not working, I don't know why. 
     *
     * @param[in]      viewer  { parameter_description }
     */
    void viewerAddObjectsLabel(Visualizer viewer);

    /**
     * @details Add in the viewer the open gripper model with colors specified as input [0-1].
     * The gripper is not transformed, so you will see it at the frame of the point cloud.
     * 
     * @param viewer [description]
     * @param r [description]
     * @param g [description]
     * @param b [description]
     */
    void viewerShowGripperModel(Visualizer viewer,double r=1,double g=1,double b=1);

    /**
     * @details Add in the viewer the closed gripper model with colors specified as input [0-1].
     * The gripper is not transformed, so you will see it at the frame of the point cloud.
     * 
     * @param viewer 
     * @param r [0-255]
     * @param g [0-255]
     * @param b [0-255]
     */
    void viewerShowClosedGripperModel(Visualizer viewer,double r=1,double g=1,double b=1);

    /**
     * @brief Show in the viewer associated to the class a transformed object. 
     *        ONLY FOR DEBUGGING PURPOSE - USELESS
     */
    void viewerAddObjectsTransfomed(Visualizer viewer);

    void viewerAddGraspingPose(Visualizer viewer,uint idx);
    void viewerAddGraspingPoses(Visualizer viewer);

    /**
     * @brief Add to the viewer the grasping pose estimated after having pushed the object
     * @details Add to the viewer the grasping pose estimated after having pushed the object
     * 
     * @param viewer 
     * @param obj_idx index of the obejct 
     * @param dir_idx index of the direction
     */
    void viewerAddPushingGraspingPose(Visualizer viewer, uint obj_idx, uint dir_idx);

    void viewerAddApproachingPose(Visualizer viewer, uint idx);
    void viewerAddApproachingPoses(Visualizer viewer);

    uint getNumObjects();
    std::vector<ObjectFull> getFullObjects();

    std::vector<OBB> getOBBObjects();

    std::vector<BlocksPredicate> getBlockPredicates();
    std::vector<std::vector<uint> > getOnTopPredicates();
    std::vector<std::vector<uint> > getBlockGraspPredicates();

    /**
     * @details Get the projected principal directions
     * @return projected principal directions
     */
    std::vector<PrincipalDirectionsProjected> getProjectedPrincipalDirections();

    std::vector<OriginalPrincipalDirections> getOriginalPrincipalDirections();

    std::vector<GraspingPose> getGraspingPoses();

    std::vector<GraspingPose> getApproachingPoses();

    std::vector<PushingPose> getPushingPoses();

    std::vector<PushingGraspingPose> getPushingGraspingPoses(); 



    ExecutionTimes getExecutionTimes();

    double getPushingObjectDistance();

    /**
     * @brief Return the pushing lengths per each object and per each direction
     * @details Return the pushing lengths per each object and per each direction
     */    
     std::vector<PushingLength> getPushingLengths();

    /**
     * @brief Delete all the data ot the class
     * @details Delete all the data ot the class
     */
    void reset();

    /**
     * @details Print in the terminal the several execution times.
     */
    void printExecutionTimes();

    void printPushingLengths();

    void printPushingEEDistances();

    void resetPushingLengths();
    void resetPushingDistance();

};



#endif

