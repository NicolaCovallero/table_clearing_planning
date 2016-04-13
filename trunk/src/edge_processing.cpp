#include "edge_processing.h"

EdgeProcessing::EdgeProcessing()
//: viewer(new pcl::visualization::PCLVisualizer ("3D Viewer")) //initialize the boost::shared_ptr viewer
{
//	this->viewer->setBackgroundColor (0, 0, 0);
	//this->viewer->addCoordinateSystem (0.5);

}

EdgeProcessing::~EdgeProcessing(){}

void EdgeProcessing::setOriginalPointCloud(PointCloudT& cloud)
{
	this->original_cloud = cloud;
}

void EdgeProcessing::setObjects(std::vector<PointCloudT > &objects)
{
	this->objects = objects;
}

void EdgeProcessing::setPlaneCoefficients(pcl::ModelCoefficients &plane_coefficients)
{
  this->plane_coefficients = plane_coefficients;
  if(! this->plane_coefficients.values.size() == 4)
  {
    PCL_ERROR("Plane model coefficients not set or the coefficients are not of a plane- Exiting from setPlaneCoefficients()");
    return; 
  }
  
  this->plane_normal[0] = this->plane_coefficients.values[0];
  this->plane_normal[1] = this->plane_coefficients.values[1];
  this->plane_normal[2] = this->plane_coefficients.values[2];
  this->plane_normal.normalize();

  // set plane origin
  this->plane_origin[0] = 1.0;
  this->plane_origin[1] = 1.0;
  // ax+by+cz+d=0 => z = (ax+by+d)/(-c)
  this->plane_origin[2] = (this->plane_coefficients.values[0] * this->plane_origin[0] +
                          this->plane_coefficients.values[1] * this->plane_origin[1] +
                          this->plane_coefficients.values[3]) / (- this->plane_coefficients.values[2]);
}

void EdgeProcessing::setAABBObjects(std::vector<AABB>& aabb_objects)
{
	this->aabb_objects = aabb_objects;
}

void EdgeProcessing::assignObjects3DEdges()
{
	if(this->objects.size() == 0)
	{
		PCL_ERROR("EdgeProcessing::computeObjects3DEdges -> Objects are not set up. Exiting\n");
		return;
	}
	//std::cout << "Assigning edges to objects\n";
	//now we have to fit each edge point to the point of each object 
	// by using KdTree -> Very computational expensive :( 
	this->objects_edges.resize(this->objects.size());
	for (uint i = 0; i < this->objects.size(); ++i) //for each object
	{
		pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
    	kdtree.setInputCloud (this->objects[i].makeShared());
    	uint K = 1;
		for (uint h = 0; h < this->occluding_edges.points.size(); ++h)
		{
			PointT searchPoint = this->occluding_edges.points[h];
			// K nearest neighbor search
		
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
			{
				if(pointNKNSquaredDistance[0] < 0.0001)
					this->objects_edges[i].occluding_edges.push_back((uint)pointIdxNKNSearch[0]);
					
			}
		}
		for (uint h = 0; h < this->occluded_edges.points.size(); ++h)
		{
			PointT searchPoint = this->occluded_edges.points[h];
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
			{
				if(pointNKNSquaredDistance[0] < 0.0001)
					this->objects_edges[i].occluded_edges.push_back((uint)pointIdxNKNSearch[0]);
			}
		}
		for (uint h = 0; h < this->boundary_edges.points.size(); ++h)
		{
			PointT searchPoint = this->boundary_edges.points[h];
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
			{
				if(pointNKNSquaredDistance[0] < 0.0001)
					this->objects_edges[i].boundary_edges.push_back((uint)pointIdxNKNSearch[0]);
			}
		}
	}
	//std::cout << "All the edges have been assigned\n";

}

void EdgeProcessing::computeOccludedSides3D(double density)
{
	this->occluded_sides.resize(this->objects.size());

	for (uint i = 0; i < this->objects.size(); ++i)
	{
		for (uint p = 0; p < this->objects_edges[i].occluding_edges.size(); ++p)
		{
			Eigen::Vector3f eigen_point = pointT2vector3f<PointT>(this->objects[i].points[this->objects_edges[i].occluding_edges[p]]);
			Eigen::Vector3f proj_eigen_point;
			pcl::geometry::project(eigen_point,this->plane_origin,this->plane_normal,proj_eigen_point);
			double distance = (eigen_point - proj_eigen_point).norm();// distance from the point and the table
			
			if(((eigen_point - proj_eigen_point).dot(this->plane_normal) < 0 ) && (distance > 0.005))
			{
				uint n = 1;
				while( (n * density) < distance) //compute until the next point is not above the table
												 // That is it exceeds the distance
				{
					//std::cout << "while\n";
					Eigen::Vector3f eigen_side_point = eigen_point + this->plane_normal*n*density;
					PointT side_point = vector3f2pointT<PointT>(eigen_side_point);
					side_point.r = 255;
					side_point.g = 255;
					side_point.b = 255;
					this->occluded_sides[i].points.push_back(side_point);

					n++;
					if(n > 1000)//only for checking purpose - If we have too many points this means the normal plane is bad defined
					{
						PCL_ERROR("Too many points! Don't set a density smaller than 0.001 meters.\nThe problem is more probably due to the wrong direction of the normal plane\n");
						return;
					}
				}
			}
		}
		for (uint p = 0; p < this->objects_edges[i].boundary_edges.size(); ++p)
		{
			Eigen::Vector3f eigen_point = pointT2vector3f<PointT>(this->objects[i].points[this->objects_edges[i].boundary_edges[p]]);
			Eigen::Vector3f proj_eigen_point;
			pcl::geometry::project(eigen_point,this->plane_origin,this->plane_normal,proj_eigen_point);
			double distance = (eigen_point - proj_eigen_point).norm();// distance from the point and the table
			
			if(((eigen_point - proj_eigen_point).dot(this->plane_normal) < 0 ) && (distance > 0.005))
			{
				uint n = 1;
				while( (n * density) < distance)
				{
					//std::cout << "while\n";
					Eigen::Vector3f eigen_side_point = eigen_point + this->plane_normal*n*density;
					PointT side_point = vector3f2pointT<PointT>(eigen_side_point);
					side_point.r = 255;
					side_point.g = 255;
					side_point.b = 255;
					this->occluded_sides[i].points.push_back(side_point);

					n++;
					if(n > 1000) //only for checking purpose
					{
						PCL_ERROR("Too many points! Don't set a density smaller than 0.001 meters.\nThe problem is more probably due to the wrong direction of the normal plane\n");
						return;
					}
				}
			}
		}
		for (uint p = 0; p < this->objects_edges[i].occluded_edges.size(); ++p)
		{
			Eigen::Vector3f eigen_point = pointT2vector3f<PointT>(this->objects[i].points[this->objects_edges[i].occluded_edges[p]]);
			Eigen::Vector3f proj_eigen_point;
			pcl::geometry::project(eigen_point,this->plane_origin,this->plane_normal,proj_eigen_point);
			double distance = (eigen_point - proj_eigen_point).norm();// distance from the point and the table

			if(((eigen_point - proj_eigen_point).dot(this->plane_normal) < 0 ) && (distance > 0.005))
			{
				uint n = 1;
				while( (n * density) < distance)
				{
					//std::cout << "while\n";
					Eigen::Vector3f eigen_side_point = eigen_point + this->plane_normal*n*density;
					PointT side_point = vector3f2pointT<PointT>(eigen_side_point);
					side_point.r = 255;
					side_point.g = 255;
					side_point.b = 255;
					this->occluded_sides[i].points.push_back(side_point);

					n++;
					if(n > 1000) //only for checking purpose
					{
						PCL_ERROR("Too many points! Don't set a density smaller than 0.001 meters.\nThe problem is more probably due to the wrong direction of the normal plane\n");
						return;
					}
				}
			}
		}
	}
}

void EdgeProcessing::computeOccludedSides2D(double density)
{
	this->occluded_sides.resize(0); //the occluded sides might be previously computed, erase them
	this->occluded_sides.resize(this->objects.size());

	if(!this->original_cloud.isOrganized())
	{
		PCL_ERROR("EdgeProcessing::computeOccludedSides2D the cloud is not organized.\n");
		return;
	}

	for (uint i = 0; i < this->objects.size(); ++i)
	{
		for (uint p = 0; p < this->objects_edges_2d[i].size(); ++p)
		{
			Eigen::Vector3f eigen_point = pointT2vector3f<PointT>(this->original_cloud.points[this->objects_edges_2d[i][p]]);
			Eigen::Vector3f proj_eigen_point;
			pcl::geometry::project(eigen_point,this->plane_origin,this->plane_normal,proj_eigen_point);
			double distance = (eigen_point - proj_eigen_point).norm();// distance from the point and the table
			
			//check the point is not below the table and that it is not too near
			if(((eigen_point - proj_eigen_point).dot(this->plane_normal) < 0 ) && (distance > 0.02))
			{
				uint n = 1;
				while( (n * density) < distance) //compute until the next point is not above the table
												 // That is it exceeds the distance
				{
					//std::cout << "while\n";
					Eigen::Vector3f eigen_side_point = eigen_point + this->plane_normal*n*density;
					PointT side_point = vector3f2pointT<PointT>(eigen_side_point);
					side_point.r = 255;
					side_point.g = 255;
					side_point.b = 255;
					this->occluded_sides[i].points.push_back(side_point);

					n++;
					if(n > 1000)//only for checking purpose - If we have too many points this means the normal plane is bad defined
					{
						PCL_ERROR("Too many points! Don't set a density smaller than 0.001 meters.\nThe problem is more probably due to the wrong direction of the normal plane\n");
						return;
					}
				}
			}
		}
	}
}


void EdgeProcessing::compute3DEdges()
{
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
	ne.setInputCloud (this->original_cloud.makeShared());

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.003);

	// Compute the features
	ne.compute (*normals);

	pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> oed;
  	oed.setInputNormals (normals);
	oed.setInputCloud (this->original_cloud.makeShared());
	oed.setDepthDisconThreshold (0.02); // 2cm
	oed.setMaxSearchNeighbors (10); // it was 50
	pcl::PointCloud<pcl::Label> labels;
	std::vector<pcl::PointIndices> label_indices;
	oed.compute (labels, label_indices);

	if(!this->original_cloud.isOrganized())
		std::cout << "The cloud is NOT organized\n";


	// pcl::PointCloud<pcl::PointXYZRGBA>::Ptr occluding_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
	// 									    occluded_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
	// 									    boundary_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
	// 									    high_curvature_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
	// 									    rgb_edges (new pcl::PointCloud<pcl::PointXYZRGBA>)


	pcl::copyPointCloud (this->original_cloud, label_indices[0].indices, this->boundary_edges);
	pcl::copyPointCloud (this->original_cloud, label_indices[1].indices, this->occluding_edges);
	pcl::copyPointCloud (this->original_cloud, label_indices[2].indices, this->occluded_edges);
	pcl::copyPointCloud (this->original_cloud, label_indices[3].indices, this->high_curvature_edges);
	pcl::copyPointCloud (this->original_cloud, label_indices[4].indices, this->rgb_edges);

}

void EdgeProcessing::compute2DEdges()
{
	this->objects_edges_2d.resize(this->objects.size());
	tic();
	for (uint idx = 0; idx < this->objects.size(); ++idx)
	{
		//create cv::mat same dimension of the original point of the kinect
	    cv::Mat img(480, 640, CV_8U,cv::Scalar(0)); //create an grey image

	    //for the idx object we have to retrieve its location in the original point cloud
	    std::vector<uint> indices;
	    pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
		kdtree.setInputCloud (this->original_cloud.makeShared());
		int K = 1;
	    for (uint i = 0; i < this->objects[idx].points.size(); ++i)
	    {
			PointT searchPoint = this->objects[idx].points[i];
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
			{
				if(pointNKNSquaredDistance[0] < 0.0001)
				{
		            float x,y;
		            y = (int)(pointIdxNKNSearch[0]/this->original_cloud.width);
		            x = pointIdxNKNSearch[0] - y*this->original_cloud.width;
		            img.at<uchar>(y,x) = 255; //transformation coordinates
	 			}
			}
	    }

	    //remove sides pixels with erosion.
	    // This is done in order to be sure to keep pixels of 
	    cv::erode(img,img,cv::Mat());
	    /// Generate grad_x and grad_y
	    cv::Mat edges_mat;
	  	cv::Mat grad_x, grad_y;
	  	cv::Mat abs_grad_x, abs_grad_y;

		int scale = 1;
		int delta = 0;
		int ddepth = CV_16S;

		/// Gradient X
		//cv::Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, cv::BORDER_DEFAULT );
		cv::Sobel( img, grad_x, ddepth, 1, 0, 1, scale, delta, cv::BORDER_DEFAULT );//cv::BORDER_DEFAULT
		cv::convertScaleAbs( grad_x, abs_grad_x );

		/// Gradient Y
		//cv::Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, cv::BORDER_DEFAULT );
		cv::Sobel( img, grad_y, ddepth, 0, 1, 1, scale, delta, cv::BORDER_DEFAULT );
		cv::convertScaleAbs( grad_y, abs_grad_y );

		/// Total Gradient (approximate)
		cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, edges_mat );

	    // find non zero element in the image
	    // http://stackoverflow.com/questions/19242662/opencv-find-all-non-zero-coordinates-of-a-binary-mat-image
	    cv::Mat nonZeroCoordinates;
	    cv::findNonZero(edges_mat, nonZeroCoordinates);
	    for (uint i = 0; i < nonZeroCoordinates.total(); i++ ) {
	        this->objects_edges_2d[idx].push_back((uint)(nonZeroCoordinates.at<cv::Point>(i).x + (int)nonZeroCoordinates.at<cv::Point>(i).y * this->original_cloud.width));

	    }


	    // Uncomment these lines if you want to see how works the edg	es detection
		// cv::namedWindow("Object", CV_WINDOW_AUTOSIZE); //create a window with the name "MyWindow"
		// cv::imshow("Object", edges_mat); //display the image which is stored in the 'img' in the "MyWindow" window
		// cv::waitKey(0);

	}
    
	toc();

}

std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > 
EdgeProcessing::getObjectsOccludedSides()
{
	return this->occluded_sides;
}

std::vector<std::vector<uint> > EdgeProcessing::getObjectsEdges2DIndices()
{
	return this->objects_edges_2d;
}

void EdgeProcessing::viewerInit(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer)
{	
	this->viewer = viewer;
	this->viewer->setBackgroundColor (0, 0, 0);
	this->viewer->addCoordinateSystem (0.5);
}

void EdgeProcessing::viewerAddEdgesObject(uint idx)
{

	//get point clouds
	PointCloudT occluding_edges, boundary_edges, occluded_edges;
	for (uint i = 0; i < this->objects_edges[idx].occluding_edges.size(); ++i)
		occluding_edges.points.push_back(this->objects[idx].points[this->objects_edges[idx].occluding_edges[i]]);
	for (uint i = 0; i < this->objects_edges[idx].boundary_edges.size(); ++i)
		boundary_edges.points.push_back(this->objects[idx].points[this->objects_edges[idx].boundary_edges[i]]);
	for (uint i = 0; i < this->objects_edges[idx].occluded_edges.size(); ++i)
		occluded_edges.points.push_back(this->objects[idx].points[this->objects_edges[idx].occluded_edges[i]]);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb_oe1(occluding_edges.makeShared());
	this->viewer->addPointCloud<PointT> (occluding_edges.makeShared(), rgb_oe1, "occluding_edges");
	this->viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "occluding_edges"); 
  	this->viewer->addText("Purple: occluding_edges", 10, 10);


	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb_be(boundary_edges.makeShared());
	this->viewer->addPointCloud<PointT> (boundary_edges.makeShared(), rgb_be, "boundary_edges");
	this->viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "boundary_edges"); 
	this->viewer->addText("Red: boundary edges", 10, 20);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb_oe(occluded_edges.makeShared());
	this->viewer->addPointCloud<PointT> (occluded_edges.makeShared(), rgb_oe, "occluded_edges");
	this->viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "occluded_edges"); 
	this->viewer->addText("Green: occluded edges", 10, 30);


}

void EdgeProcessing::viewerAddEdgesObjects()
{

	//get point clouds
	PointCloudT occluding_edges, boundary_edges, occluded_edges;
	for (uint idx = 0; idx < this->objects.size(); ++idx)
	{
		for (uint i = 0; i < this->objects_edges[idx].occluding_edges.size(); ++i)
			occluding_edges.points.push_back(this->objects[idx].points[this->objects_edges[idx].occluding_edges[i]]);
		for (uint i = 0; i < this->objects_edges[idx].boundary_edges.size(); ++i)
			boundary_edges.points.push_back(this->objects[idx].points[this->objects_edges[idx].boundary_edges[i]]);
		for (uint i = 0; i < this->objects_edges[idx].occluded_edges.size(); ++i)
			occluded_edges.points.push_back(this->objects[idx].points[this->objects_edges[idx].occluded_edges[i]]);
	}
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb_oe1(occluding_edges.makeShared());
	this->viewer->addPointCloud<PointT> (occluding_edges.makeShared(), rgb_oe1, "occluding_edges");
	this->viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "occluding_edges"); 
  	this->viewer->addText("Purple: occluding_edges", 10, 10);


	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb_be(boundary_edges.makeShared());
	this->viewer->addPointCloud<PointT> (boundary_edges.makeShared(), rgb_be, "boundary_edges");
	this->viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "boundary_edges"); 
	this->viewer->addText("Red: boundary edges", 10, 20);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb_oe(occluded_edges.makeShared());
	this->viewer->addPointCloud<PointT> (occluded_edges.makeShared(), rgb_oe, "occluded_edges");
	this->viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "occluded_edges"); 
	this->viewer->addText("Green: occluded edges", 10, 30);

}

void EdgeProcessing::viewerAddAllEdges()
{

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb_oe1(this->occluding_edges.makeShared());
	this->viewer->addPointCloud<PointT> (this->occluding_edges.makeShared(), rgb_oe1, "occluding_edges");
	this->viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "occluding_edges"); 
  	this->viewer->addText("Purple: occluding_edges", 10, 10);


	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb_be(this->boundary_edges.makeShared());
	this->viewer->addPointCloud<PointT> (this->boundary_edges.makeShared(), rgb_be, "boundary_edges");
	this->viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "boundary_edges"); 
	this->viewer->addText("Red: boundary edges", 10, 20);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb_oe(this->occluded_edges.makeShared());
	this->viewer->addPointCloud<PointT> (this->occluded_edges.makeShared(), rgb_oe, "occluded_edges");
	this->viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "occluded_edges"); 
	this->viewer->addText("Green: occluded edges", 10, 30);

  	return;
}

void EdgeProcessing::viewerAddOccludedSides()
{
	for (uint i = 0; i < this->occluded_sides.size(); ++i)
  	{ 
	    std::string cloud_name = "sides_object";
	    std::ostringstream convert;   // stream used for the conversion
	    convert << i;
	    cloud_name += convert.str();
		pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_cloud(this->occluded_sides[i].makeShared());
		this->viewer->addPointCloud<PointT> (this->occluded_sides[i].makeShared(), rgb_cloud, cloud_name);
	   	this->viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, cloud_name); 
  	}
  	return;

}

void EdgeProcessing::viewerSpin()
{
	while (!this->viewer->wasStopped ())
    	this->viewer->spinOnce (100);
}