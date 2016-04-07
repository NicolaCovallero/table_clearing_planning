#ifndef _EDGE_PROCESSING_H
#define _EDGE_PROCESSING_H

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


#include <pcl/features/normal_3d.h>

// translations and projections
#include <pcl/common/geometry.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

// 3D edge dectection
#include <pcl/features/organized_edge_detection.h>

// KdTree
#include <pcl/kdtree/kdtree_flann.h>

// #include <pcl/2d/convolution.h>
// #include <pcl/2d/edge.h>
// #include <pcl/2d/kernel.h>
// // to use the kernel you have to add in the /usr/local/include/pcl-1.8/pcl/2d/imp 
// // the file kernel.hpp -> https://github.com/PointCloudLibrary/pcl/blob/master/2d/include/pcl/2d/impl/kernel.hpp
// #include <pcl/2d/morphology.h>
// #include <pcl/pcl_base.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

#include "conversions.h"
#include "utilities.h"

/**
 * @brief EdgeProcessing class detects 3D edges (only depth one - rgb ones are ommited) of the objects and fills the occluded sides until the table.
 * @details This class provides some methods to process the 3D edges, usefull to refill occluded sides or 
 * detected on top predicates. For on top predicates the occluded edges on an object are an index that an object is on top of another one.
 * 
 */
class EdgeProcessing
{
	typedef pcl::PointXYZRGBA PointT;
  	typedef pcl::PointCloud<PointT> PointCloudT;
  	typedef pcl::PointCloud<PointT>::Ptr PointCloudTptr;

  	pcl::PointCloud<pcl::PointXYZRGBA> occluding_edges, ///<  edges on top of the occluded ones
										    occluded_edges, ///< edges that stands on the table
										    boundary_edges, ///< don't know, but they are still useful
										    high_curvature_edges,
										    rgb_edges;

	PointCloudT original_cloud; ///< original point cloud, is the one used also for the segmentation.
								///< It has to be organized, so the same of the kinect.										    
	std::vector<PointCloudT > objects;	///< vector of objects									    
	std::vector<PointCloudT > occluded_sides; ///< vector of occluded sides									    

	std::vector<AABB> aabb_objects;

	struct Edges3D
	{
		std::vector<uint> 	occluding_edges, ///<  edges on top of the occluded ones
						    occluded_edges, ///< edges that stands on the table
						    boundary_edges; ///< don't know, but they are still useful								
	};
	std::vector<Edges3D> objects_edges; ///< vector of edges for each object

	typedef std::vector<uint> Edges2D;
	std::vector<Edges2D> objects_edges_2d; ///< vector of edges for each object


	pcl::ModelCoefficients plane_coefficients; 
	Eigen::Vector3f plane_normal; ///< normals of the plane 
  	Eigen::Vector3f plane_origin; ///< an arbitrary point of the plane to act like origin

  	
	public:

	// VISUALIZATION
  	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer; ///< visualize of the class


	EdgeProcessing();
	~EdgeProcessing();

	void setOriginalPointCloud(PointCloudT& cloud);

	void setObjects(std::vector<PointCloudT > &objects);

    void setPlaneCoefficients(pcl::ModelCoefficients &plane_coefficients);

    void setAABBObjects(std::vector<AABB>& aabb_objects);

	/**
	 * @brief Assign the precomputed 3D edges to each object
	 * @details Assign the precomputed 3D edges to each object
	 */
	void assignObjects3DEdges();

	/**
	 * @brief Compute the occluded sides
	 * @details  Compute the occluded sides by projecting the boundary
	 *  			and occluding edges of the objects up to the table
	 *  			filling the sides with points with a certain density.
	 *  			The edges used are the one computed by EdgeProcessing::compute3DEdges
	 * 
	 * * @param density density, in meters, of the occluded sides point cloud
	 */	
	void computeOccludedSides3D(double density);

	/**
	* @brief Compute the occluded sides
	* 
	* @details  Compute the occluded sides by projecting the boundary
	*  			and occluding edges of the objects up to the table
	*  			filling the sides with points with a certain density.
	*  			The edges used are the one computed by EdgeProcessing::compute2DEdges
	*  			
	*  			There is not a real need to do for each object, we could directly do for the whole objects image
	* 
	* @param density density, in meters, of the occluded sides point cloud
	*/	
	void computeOccludedSides2D(double density);	 

	/**
	 * @brief Compute 3D edges for the input point cloud
	 * @details Only depth edges are considered. This method is slow since it has to compute the edges 
	 * for the whole original point cloud. It is based on an algorithm that works
	 * with organized point clouds. In average it takes 1.45seconds. EdgeProcessing::compute2DEdges is 
	 * based on another method and it takes about 0.1 seconds per object. So in case the objects are 14
	 * the two methods are the same, in case they are too many is better to use this method. This method
	 * should used in order to use the EdgeProcessing::computeOccludedSides2D method toc ompute the occluded sides.
	 * Before to compute the is necessary assigning each edge to each object with the method EdgeProcessing::assignObjects3DEdges. 
	 * \n Usage:
	 * \code
	 * EdgeProcessing ep;
	 * ep.setOriginalPointCloud(*cloud);
	 * ep.setObjects(segmented_objs);
	 * ep.setPlaneCoefficients(plane_coeff);  
	 * ep.compute3DEdges();
	 * ep.assignObjects3DEdges();
	 * ep.computeOccludedSides3D(0.01);
	 * \endcode
	 * 
	 * 
	 * @param cloud Input point cloud - It has to be organized
	 */
	void compute3DEdges(PointCloudTptr cloud);

	/**
	 * @brief Compute 2D edges for the input point cloud
	 * @details For each object creates an image and detect the edges with Sobel filter. 
	 * 			It takes about 100 ms per object. 
	 * \n Usage:
	 * \code
	 * EdgeProcessing ep;
	 * ep.setOriginalPointCloud(*cloud);
	 * ep.setObjects(segmented_objs);
	 * ep.setPlaneCoefficients(plane_coeff);  
	 * ep.compute2DEdges();
	 * ep.computeOccludedSides2D(0.01);
	 * \endcode
	 * @param cloud Input point cloud - It has to be organized
	 */
	void compute2DEdges();


	std::vector<PointCloudT> getObjectsOccludedSides(); 

	/**
	 * @brief Compute 3D edges for the input point cloud and add to the viewer
	 * 
	 * @param cloud Point cloud where look for the edges
	 */
	void test(PointCloudTptr cloud);

	/**
	 * @brief Initialize the viewer
	 */
	void viewerInit(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer);

	/**
	 * @brief Add to the viewer of the class the edges 
	 * 
	 * @param idx Index of the object to show its edges
	 */
	void viewerAddEdgesObject(uint idx);

	/**
	 * @brief Add to the class viewer all the edges associated to objects
	 */
	void viewerAddEdgesObjects();

	/**
	 * @brief Add to the class viewer all the edges 
	 */
	void viewerAddAllEdges();

	/**
	 * @brief Add to the class viewer  occluded sides
	 * @details Add to the class viewer. They will be chaarcterized by the blue color
	 */
	void viewerAddOccludedSides();

	/**
	 * @brief Show the class viewer
	 */
	void viewerSpin();
};

#endif

