#include "table_clearing_planning.h"

CTableClearingPlanning::CTableClearingPlanning()
{
  this->n_objects = 0;
  this->pushing_limit = this->PUSHING_LIMIT;
  return;
}

CTableClearingPlanning::CTableClearingPlanning(std::vector<PointCloudT> &objects)
{
  this->objects = objects;
  this->n_objects = objects.size();
  this->pushing_limit = this->PUSHING_LIMIT;
  return;
} 

CTableClearingPlanning::~CTableClearingPlanning()
{
}


void CTableClearingPlanning::setOriginalPointCloud(PointCloudT original_cloud)
{
  this->original_cloud = original_cloud;
}

bool CTableClearingPlanning::areObjectCollidingFcl(uint idx_1, fcl::Transform3f tf, uint idx_2)
{
  FclMesh mesh1,mesh2;

  mesh1 = this->pcl2FclConvexHull(idx_1);
  mesh2 = this->pcl2FclConvexHull(idx_2);

  // BVHModel is a template class for mesh geometry, for default OBBRSS template is used
  typedef fcl::BVHModel<fcl::OBBRSS> Model;
  Model* model1 = new Model();
  model1->beginModel();
  model1->addSubModel(mesh1.vertices, mesh1.triangles);
  model1->endModel();

  // get the bounding box 
  //fcl::BVNode<fcl::OBBRSS> p = model1->getBV(0);
  //double a = p.bv.width();

  Model* model2 = new Model();
  model2->beginModel();
  model2->addSubModel(mesh2.vertices, mesh2.triangles);
  model2->endModel();


  // no rotation
  fcl::Matrix3f R(1,0,0,
                  0,1,0,
                  0,0,1);
  fcl::Vec3f T(0,0,0); // no translation


  fcl::Transform3f pose(R, T);

  bool enable_contact = true;
  int num_max_contacts = std::numeric_limits<int>::max();
  // set the collision request structure, here we just use the default setting
  fcl::CollisionRequest request(num_max_contacts, enable_contact);
  // result will be returned via the collision result structure
  fcl::CollisionResult result;

  int num_contacts = fcl::collide(model1, tf, model2, pose, 
                             request, result);
  if(num_contacts > 0)
    return true;

  return false;
}

bool CTableClearingPlanning::isEEColliding(uint idx, fcl::Transform3f tf)
{
  FclMesh mesh1,mesh2;

  mesh1 = this->getGripperMesh();
  mesh2 = this->pcl2FclConvexHull(idx);

  // BVHModel is a template class for mesh geometry, for default OBBRSS template is used
  typedef fcl::BVHModel<fcl::OBBRSS> Model;
  Model* ee_model1 = new Model();
  ee_model1->beginModel();
  ee_model1->addSubModel(mesh1.vertices, mesh1.triangles);
  ee_model1->endModel();

  // get the bounding box 
  //fcl::BVNode<fcl::OBBRSS> p = ee_model1->getBV(0);
  //double a = p.bv.width();

  Model* model2 = new Model();
  model2->beginModel();
  model2->addSubModel(mesh2.vertices, mesh2.triangles);
  model2->endModel();


  // no rotation
  fcl::Matrix3f R(1,0,0,
                  0,1,0,
                  0,0,1);
  fcl::Vec3f T(0,0,0); // no translation


  fcl::Transform3f pose(R, T);

  bool enable_contact = true;
  int num_max_contacts = std::numeric_limits<int>::max();
  // set the collision request structure, here we just use the default setting
  fcl::CollisionRequest request(num_max_contacts, enable_contact);
  // result will be returned via the collision result structure
  fcl::CollisionResult result;

  int num_contacts = fcl::collide(ee_model1, tf, model2, pose, 
                             request, result);
  if(num_contacts > 0)
    return true;

  return false;
}

bool CTableClearingPlanning::isFingersModelColliding(uint idx, fcl::Transform3f tf)
{
  FclMesh mesh1,mesh2;

  mesh1 = this->getFingersModelMesh();
  mesh2 = this->pcl2FclConvexHull(idx);

  // BVHModel is a template class for mesh geometry, for default OBBRSS template is used
  typedef fcl::BVHModel<fcl::OBBRSS> Model;
  Model* finger_model = new Model();
  finger_model->beginModel();
  finger_model->addSubModel(mesh1.vertices, mesh1.triangles);
  finger_model->endModel();

  // get the bounding box 
  //fcl::BVNode<fcl::OBBRSS> p = ee_model1->getBV(0);
  //double a = p.bv.width();

  Model* model2 = new Model();
  model2->beginModel();
  model2->addSubModel(mesh2.vertices, mesh2.triangles);
  model2->endModel();


  // no rotation
  fcl::Matrix3f R(1,0,0,
                  0,1,0,
                  0,0,1);
  fcl::Vec3f T(0,0,0); // no translation


  fcl::Transform3f pose(R, T);

  bool enable_contact = true;
  int num_max_contacts = std::numeric_limits<int>::max();
  // set the collision request structure, here we just use the default setting
  fcl::CollisionRequest request(num_max_contacts, enable_contact);
  // result will be returned via the collision result structure
  fcl::CollisionResult result;

  int num_contacts = fcl::collide(finger_model, tf, model2, pose, 
                             request, result);
  if(num_contacts > 0)
    return true;

  return false;
}

CTableClearingPlanning::FclMesh
CTableClearingPlanning::getGripperMesh()
{
  FclMesh fcl_mesh;
  // initialize the struct
  fcl_mesh.vertices.resize(0);
  fcl_mesh.triangles.resize(0);

  // check for correct input
  if(this->ee_simple_model.cloud.points.size() == 0 )
  {
    PCL_ERROR("Gripper Model not set\n");
    return fcl_mesh;
  }

  // ----------------- fill vertices ----------------------------
  // for each vertex of the convex hull
  for (uint i = 0; i < this->ee_simple_model.cloud.points.size(); ++i)
  {
    fcl::Vec3f vec_tmp(this->ee_simple_model.cloud.points[i].x,
                       this->ee_simple_model.cloud.points[i].y,
                       this->ee_simple_model.cloud.points[i].z);
    fcl_mesh.vertices.push_back(vec_tmp);
  }

  // ----------------- fill triangles ---------------------------
  for (uint i = 0; i < this->ee_simple_model.vertices.size(); ++i)
  {

    fcl::Triangle triangle_tmp(this->ee_simple_model.vertices[i].vertices[0],
                               this->ee_simple_model.vertices[i].vertices[1],
                               this->ee_simple_model.vertices[i].vertices[2]);
    fcl_mesh.triangles.push_back(triangle_tmp);
  }

  return fcl_mesh; 
}

CTableClearingPlanning::FclMesh
CTableClearingPlanning::getFingersModelMesh()
{
  FclMesh fcl_mesh;
  // initialize the struct
  fcl_mesh.vertices.resize(0);
  fcl_mesh.triangles.resize(0);

  // check for correct input
  if(this->fingers_model.cloud.points.size() == 0 )
  {
    PCL_ERROR("Fingers Model not set\n");
    return fcl_mesh;
  }

  // ----------------- fill vertices ----------------------------
  // for each vertex of the convex hull
  for (uint i = 0; i < this->fingers_model.cloud.points.size(); ++i)
  {
    fcl::Vec3f vec_tmp(this->fingers_model.cloud.points[i].x,
                       this->fingers_model.cloud.points[i].y,
                       this->fingers_model.cloud.points[i].z);
    fcl_mesh.vertices.push_back(vec_tmp);
  }

  // ----------------- fill triangles ---------------------------
  for (uint i = 0; i < this->fingers_model.vertices.size(); ++i)
  {

    fcl::Triangle triangle_tmp(this->fingers_model.vertices[i].vertices[0],
                               this->fingers_model.vertices[i].vertices[1],
                               this->fingers_model.vertices[i].vertices[2]);
    fcl_mesh.triangles.push_back(triangle_tmp);
  }

  return fcl_mesh; 
}

Eigen::Vector3f  CTableClearingPlanning::pointT2vector3f(PointT point)
{
  Eigen::Vector3f vec;
  vec[0] = point.x;
  vec[1] = point.y;
  vec[2] = point.z;
  return vec;
}

CTableClearingPlanning::FclMesh
CTableClearingPlanning::pcl2FclConvexHull(uint idx)
{

  FclMesh fcl_mesh;
  // initialize the struct
  fcl_mesh.vertices.resize(0);
  fcl_mesh.triangles.resize(0);

  // check for correct input
  if(this->convex_hull_objects.size() == 0)
  {
    if (!((this->convex_hull_objects[idx].points.size() > 0) &&
        (this->convex_hull_vertices[idx].size() > 0 )))
    {
      PCL_ERROR("In pcl2FclConvexHull : vertices or points are not computed\n");
      return fcl_mesh;
    }
    else
    {
      PCL_ERROR("Convex hulls not computed\n");
      return fcl_mesh;
    }
  }
  // ----------------- fill vertices ----------------------------
  // for each vertex of the convex hull
  for (uint i = 0; i < this->convex_hull_objects[idx].points.size(); ++i)
  {
    fcl::Vec3f vec_tmp(this->convex_hull_objects[idx].points[i].x,
                       this->convex_hull_objects[idx].points[i].y,
                       this->convex_hull_objects[idx].points[i].z);
    fcl_mesh.vertices.push_back(vec_tmp);
  }

  // ----------------- fill triangles ---------------------------
  for (uint i = 0; i < this->convex_hull_vertices[idx].size(); ++i)
  {

    fcl::Triangle triangle_tmp(this->convex_hull_vertices[idx][i].vertices[0],
                               this->convex_hull_vertices[idx][i].vertices[1],
                               this->convex_hull_vertices[idx][i].vertices[2]);
    fcl_mesh.triangles.push_back(triangle_tmp);
  }

  return fcl_mesh; 
}

void CTableClearingPlanning::translate(PointCloudT& cloud1, PointCloudT& cloud2, Eigen::Vector4f& translation)
{
  Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity ();
  pcl::transformPointCloud<PointT>(cloud1, cloud2, translation.head<3>()  , orientation);
}


void CTableClearingPlanning::fcl2EigenTransform(Eigen::Vector4f& translate, Eigen::Quaternionf& q_rot,
                                                    fcl::Transform3f& tf)
{
  fcl::Vec3f T = tf.getTranslation();
  translate[0] = T[0];
  translate[1] = T[1];
  translate[2] = T[2];
  fcl::Quaternion3f q = tf.getQuatRotation();
  Eigen::Quaternionf q_eigen(q.getW(),q.getX(),q.getY(),q.getZ());
  q_rot = q_eigen;
}
void CTableClearingPlanning::eigen2FclTransform(Eigen::Vector4f& translate, Eigen::Quaternionf& q_rot,
                                                    fcl::Transform3f& tf)
{ 
  fcl::Quaternion3f q(q_rot.x(),q_rot.y(),q_rot.z(),q_rot.w());
  fcl::Vec3f T(translate[0],translate[1],translate[2]);
  tf.setTransform(q,T);
}

void CTableClearingPlanning::eigen2FclRotation(Eigen::Quaternionf& q_rot, fcl::Matrix3f& rot)
{
  Eigen::Matrix3f mat_rot = q_rot.toRotationMatrix();
  this->eigen2FclRotation(mat_rot,rot);
}

void CTableClearingPlanning::eigen2FclRotation(Eigen::Matrix3f eig_rot, fcl::Matrix3f& rot)
{
  rot(0,0) = eig_rot(0,0);
  rot(0,1) = eig_rot(0,1);
  rot(0,2) = eig_rot(0,2);

  rot(1,0) = eig_rot(1,0);
  rot(1,1) = eig_rot(1,1);
  rot(1,2) = eig_rot(1,2);

  rot(2,0) = eig_rot(2,0);
  rot(2,1) = eig_rot(2,1);
  rot(2,2) = eig_rot(2,2);
}

void CTableClearingPlanning::computeAABBObjects(bool refine_centroids)
{
  if(this->principal_directions_objects.size() == 0)
    this->computePrincipalDirections();

  this->aabb_objects.resize(this->n_objects);
  for (int i = 0; i < n_objects; ++i)
  {
    PrincipalDirectionsProjected* pd = &(principal_directions_objects[i]);
    // we have to compute the transformation
    Eigen::Matrix4f transform;
    transform(0,0) = pd->dir1[0]; transform(0,1) = pd->dir1[1]; transform(0,2) = pd->dir1[2];
    transform(1,0) = pd->dir3[0]; transform(1,1) = pd->dir3[1]; transform(1,2) = pd->dir3[2];
    transform(2,0) = this->plane_normal[0]; transform(2,1) = this->plane_normal[1]; transform(2,2) = this->plane_normal[2];
    //translation we don't care about the translation
    transform(0,3) = 0; 
    transform(1,3) = 0;
    transform(2,3) = 0;

    transform(3,0) = 0;transform(3,1) = 0;transform(3,2) = 0;transform(3,3) = 1;

    PointCloudT tmp_cloud;
    pcl::transformPointCloud<PointT>(this->objects[i], tmp_cloud, transform);

    PointT min_pt,max_pt;
    pcl::getMinMax3D(tmp_cloud,min_pt,max_pt);

    pd->dir1_limit= (max_pt.x - min_pt.x);
    pd->dir3_limit= (max_pt.y - min_pt.y);

    this->aabb_objects[i].deep = (max_pt.x - min_pt.x);
    this->aabb_objects[i].width = (max_pt.y - min_pt.y);
    this->aabb_objects[i].height = (max_pt.z - min_pt.z);

    if(refine_centroids)
    {
      pcl::PointXYZ center;
      center.x = (max_pt.x + min_pt.x)/2;
      center.y = (max_pt.y + min_pt.y)/2;
      center.z = (max_pt.z + min_pt.z)/2;
      pcl::PointCloud<pcl::PointXYZ> center_cloud;
      center_cloud.points.push_back(center);
      Eigen::Matrix4f tf_inv = transform.inverse();
      pcl::transformPointCloud<pcl::PointXYZ>(center_cloud, center_cloud, tf_inv );
      this->principal_directions_objects[i].centroid[0] = center_cloud.points[0].x;
      this->principal_directions_objects[i].centroid[1] = center_cloud.points[0].y;
      this->principal_directions_objects[i].centroid[2] = center_cloud.points[0].z;
    }
  }
}

void CTableClearingPlanning::computeSimpleHeuristicGraspingPoses(bool vertical_poses)
{
  for (int i = 0; i < this->n_objects; ++i)
  {
    OriginalPrincipalDirections* opd = &(this->original_principal_directions_objects[i]);
    PrincipalDirectionsProjected* pd = &(this->principal_directions_objects[i]);
    //we get the transformation matrices accordingly to the principal directions and grasping poses
    GraspingPose gp;
    //gp.translation = opd->centroid.head<3>();
    this->computeSurfaceGraspPoint(gp.translation,i);

    if(vertical_poses)
    {
      gp.rotation(0,0) = pd->dir3[0]; gp.rotation(0,1) = pd->dir3[1]; gp.rotation(0,2) = pd->dir3[2];   
      gp.rotation(1,0) = pd->dir2[0]; gp.rotation(1,1) = pd->dir2[1]; gp.rotation(1,2) = pd->dir2[2];   
      Eigen::Vector3f new_axis = pd->dir3.cross(pd->dir2); 
      gp.rotation(2,0) = new_axis[0]; gp.rotation(2,1) = new_axis[1]; gp.rotation(2,2) = new_axis[2];  
      
      // this express the rotation from the desired position to the frame, we have to get the inverse to
      // express the rotation from the current frame to the desired pose
      // if we do directly: gp.rotation == p.rotation.inverse() it returns error
      Eigen::Matrix3f mat_rot = gp.rotation.inverse();
      gp.rotation = mat_rot;
    }
    else
    {
      // the gripper has its x axis aligned with the p3, so we have to define
      // a rotation matrix in way that its x axis is aligned to the y axis of fingerModel 
   
      gp.rotation(0,0) = pd->dir3[0]; gp.rotation(0,1) = pd->dir3[1]; gp.rotation(0,2) = pd->dir3[2];   
      Eigen::Vector3f new_axis = pd->dir3.cross(opd->p3); 
      gp.rotation(1,0) = new_axis[0]; gp.rotation(1,1) = new_axis[1]; gp.rotation(1,2) = new_axis[2];   
      new_axis = pd->dir3.cross(new_axis);
      gp.rotation(2,0) = new_axis[0]; gp.rotation(2,1) = new_axis[1]; gp.rotation(2,2) = new_axis[2];   


      Eigen::Matrix3f mat_rot = gp.rotation.inverse();
      gp.rotation = mat_rot;
    }


    this->grasping_poses.push_back(gp);
  }
}

void CTableClearingPlanning::setGripperSimpleModel(double height, double deep, double width, double distance_plane)
{
  this->ee_simple_model.height = height;
  this->ee_simple_model.deep = deep;
  this->ee_simple_model.width = width;

  // //set the vertices of the bounding box
  // pcl::PointXYZ p;
  // p.x = width/2; p.y = deep/2; p.z = height/2;
  // this->ee_simple_model.cloud.points.push_back(p);
  // p.x = width/2; p.y = deep/2; p.z = - height/2;
  // this->ee_simple_model.cloud.points.push_back(p);
  // p.x = width/2; p.y = - deep/2; p.z = height/2;
  // this->ee_simple_model.cloud.points.push_back(p);
  // p.x = - width/2; p.y = deep/2; p.z = height/2;
  // this->ee_simple_model.cloud.points.push_back(p);
  // p.x = - width/2; p.y = - deep/2; p.z = height/2;
  // this->ee_simple_model.cloud.points.push_back(p);
  // p.x = - width/2; p.y = deep/2; p.z = - height/2;
  // this->ee_simple_model.cloud.points.push_back(p);
  // p.x = width/2; p.y = - deep/2; p.z = - height/2;
  // this->ee_simple_model.cloud.points.push_back(p);
  // p.x = - width/2; p.y = - deep/2; p.z = - height/2;
  // this->ee_simple_model.cloud.points.push_back(p);

  //set the vertices of the bounding box
  pcl::PointXYZ p;
  p.x = width/2; p.y = height/2; p.z = deep/2;
  this->ee_simple_model.cloud.points.push_back(p);
  p.x = width/2; p.y = height/2; p.z = - deep/2;
  this->ee_simple_model.cloud.points.push_back(p);
  p.x = width/2; p.y = - height/2; p.z = deep/2;
  this->ee_simple_model.cloud.points.push_back(p);
  p.x = - width/2; p.y = height/2; p.z = deep/2;
  this->ee_simple_model.cloud.points.push_back(p);
  p.x = - width/2; p.y = - height/2; p.z = deep/2;
  this->ee_simple_model.cloud.points.push_back(p);
  p.x = - width/2; p.y = height/2; p.z = - deep/2;
  this->ee_simple_model.cloud.points.push_back(p);
  p.x = width/2; p.y = - height/2; p.z = - deep/2;
  this->ee_simple_model.cloud.points.push_back(p);
  p.x = - width/2; p.y = - height/2; p.z = - deep/2;
  this->ee_simple_model.cloud.points.push_back(p);


  pcl::ConvexHull<pcl::PointXYZ> hull;
  hull.setInputCloud(this->ee_simple_model.cloud.makeShared());
  hull.reconstruct(this->ee_simple_model.cloud, this->ee_simple_model.vertices);
  
  this->ee_simple_model.distance_plane = distance_plane;
}

void CTableClearingPlanning::setFingersModel(double opening_width, double finger_width,
               double deep, double height, double closing_height)
{
  if(height <= closing_height)
  {
    PCL_ERROR("Height %d less or equal to closing_height %d. Exiting" ,height ,closing_height);
    return;
  }

  // creating the cloud
  pcl::PointXYZ p;
  p.x = - opening_width/2; p.y = - deep/2; p.z = - height/2;
  this->fingers_model.cloud.points.push_back(p);
  p.y = deep/2;
  this->fingers_model.cloud.points.push_back(p);

  p.x = - (opening_width +finger_width*2)/2; p.y = - deep/2; p.z = - height/2;
  this->fingers_model.cloud.points.push_back(p);
  p.y = deep/2;
  this->fingers_model.cloud.points.push_back(p);

  p.x = - (opening_width +finger_width*2)/2; p.y = - deep/2; p.z = height/2;
  this->fingers_model.cloud.points.push_back(p);
  p.y = deep/2;
  this->fingers_model.cloud.points.push_back(p);  

  p.x = (opening_width +finger_width*2)/2; p.y = - deep/2; p.z = height/2;
  this->fingers_model.cloud.points.push_back(p);
  p.y = deep/2;
  this->fingers_model.cloud.points.push_back(p);

  p.x = (opening_width +finger_width*2)/2; p.y = - deep/2; p.z = - height/2;
  this->fingers_model.cloud.points.push_back(p);
  p.y = deep/2;
  this->fingers_model.cloud.points.push_back(p);

  p.x = (opening_width)/2; p.y = - deep/2; p.z = - height/2;
  this->fingers_model.cloud.points.push_back(p);
  p.y = deep/2;
  this->fingers_model.cloud.points.push_back(p);

  p.x = (opening_width)/2; p.y = - deep/2; p.z = height/2 - (height - closing_height);
  this->fingers_model.cloud.points.push_back(p);
  p.y = deep/2;
  this->fingers_model.cloud.points.push_back(p);  

  p.x = - (opening_width)/2; p.y = - deep/2; p.z = height/2 - (height - closing_height);
  this->fingers_model.cloud.points.push_back(p);
  p.y = deep/2;
  this->fingers_model.cloud.points.push_back(p);  

  // // origin up
  // p.x = 0; p.y = - deep/2; p.z = height/2;
  // this->fingers_model.cloud.points.push_back(p);
  // p.y = deep/2;
  // this->fingers_model.cloud.points.push_back(p);  

  // // origin down
  // p.x = 0; p.y = - deep/2; p.z = height - closing_height;
  // this->fingers_model.cloud.points.push_back(p);
  // p.y = deep/2;
  // this->fingers_model.cloud.points.push_back(p);  

  //  //creating the concave hull
  // pcl::ConcaveHull<pcl::PointXYZ> hull;
  // hull.setInputCloud(this->fingers_model.cloud.makeShared());
  // hull.setAlpha(0.05);
  // hull.setDimension(3);
  // hull.reconstruct(this->fingers_model.cloud,this->fingers_model.vertices);


  // make the z axis pointing down
  Eigen::Vector3f translation(0,0,0);
  Eigen::Matrix3f mat_rot;
  mat_rot(0,0) = -1;mat_rot(0,1) = 0;mat_rot(0,2) = 0;
  mat_rot(1,0) = 0;mat_rot(1,1) = 1;mat_rot(1,2) = 0;
  mat_rot(2,0) = 0;mat_rot(2,1) = 0;mat_rot(2,2) = -1;
  Eigen::Matrix3f rot = mat_rot.inverse();
  Eigen::Quaternionf quat(rot);
  pcl::transformPointCloud<pcl::PointXYZ>(this->fingers_model.cloud, this->fingers_model.cloud, translation , quat);


  //manually polygonal mesh reconstruction
  pcl::Vertices v;
  v.vertices.resize(3);

  //front side
  v.vertices[0] = 1;v.vertices[1] = 3;v.vertices[2] = 5;
  this->fingers_model.vertices.push_back(v);
  v.vertices[0] = 1;v.vertices[1] = 5;v.vertices[2] = 15;
  this->fingers_model.vertices.push_back(v);
  v.vertices[0] = 5;v.vertices[1] = 7;v.vertices[2] = 15;
  this->fingers_model.vertices.push_back(v);
  v.vertices[0] = 15;v.vertices[1] = 7;v.vertices[2] = 13;
  this->fingers_model.vertices.push_back(v);
  v.vertices[0] = 11;v.vertices[1] = 7;v.vertices[2] = 13;
  this->fingers_model.vertices.push_back(v);
  v.vertices[0] = 7;v.vertices[1] = 9;v.vertices[2] = 11;
  this->fingers_model.vertices.push_back(v);

  //back side
  v.vertices[0] = 0;v.vertices[1] = 2;v.vertices[2] = 4;
  this->fingers_model.vertices.push_back(v);
  v.vertices[0] = 0;v.vertices[1] = 4;v.vertices[2] = 14;
  this->fingers_model.vertices.push_back(v);
  v.vertices[0] = 4;v.vertices[1] = 6;v.vertices[2] = 14;
  this->fingers_model.vertices.push_back(v);
  v.vertices[0] = 14;v.vertices[1] = 6;v.vertices[2] = 12;
  this->fingers_model.vertices.push_back(v);
  v.vertices[0] = 10;v.vertices[1] = 6;v.vertices[2] = 12;
  this->fingers_model.vertices.push_back(v);
  v.vertices[0] = 6;v.vertices[1] = 8;v.vertices[2] = 10;
  this->fingers_model.vertices.push_back(v);

  v.vertices[0] = 0;v.vertices[1] = 1;v.vertices[2] = 2;
  this->fingers_model.vertices.push_back(v);
  v.vertices[0] = 3;v.vertices[1] = 1;v.vertices[2] = 2;
  this->fingers_model.vertices.push_back(v);

  v.vertices[0] = 8;v.vertices[1] = 9;v.vertices[2] = 10;
  this->fingers_model.vertices.push_back(v);
  v.vertices[0] = 11;v.vertices[1] = 9;v.vertices[2] = 10;
  this->fingers_model.vertices.push_back(v);

  v.vertices[0] = 2;v.vertices[1] = 3;v.vertices[2] = 4;
  this->fingers_model.vertices.push_back(v);
  v.vertices[0] = 5;v.vertices[1] = 3;v.vertices[2] = 4;
  this->fingers_model.vertices.push_back(v);  

  v.vertices[0] = 8;v.vertices[1] = 7;v.vertices[2] = 6;
  this->fingers_model.vertices.push_back(v);
  v.vertices[0] = 8;v.vertices[1] = 9;v.vertices[2] = 7;
  this->fingers_model.vertices.push_back(v);  

  //top "roof"
  v.vertices[0] = 4;v.vertices[1] = 5;v.vertices[2] = 6;
  this->fingers_model.vertices.push_back(v);
  v.vertices[0] = 5;v.vertices[1] = 6;v.vertices[2] = 7;
  this->fingers_model.vertices.push_back(v);  

  //bottom "roof"
  v.vertices[0] = 14;v.vertices[1] = 15;v.vertices[2] = 12;
  this->fingers_model.vertices.push_back(v);
  v.vertices[0] = 12;v.vertices[1] = 15;v.vertices[2] = 13;
  this->fingers_model.vertices.push_back(v);  

  //internal sides
  v.vertices[0] = 0;v.vertices[1] = 1;v.vertices[2] = 14;
  this->fingers_model.vertices.push_back(v);
  v.vertices[0] = 15;v.vertices[1] = 1;v.vertices[2] = 14;
  this->fingers_model.vertices.push_back(v);  

  v.vertices[0] = 10;v.vertices[1] = 11;v.vertices[2] = 12;
  this->fingers_model.vertices.push_back(v);
  v.vertices[0] = 13;v.vertices[1] = 11;v.vertices[2] = 12;
  this->fingers_model.vertices.push_back(v);  
}

void CTableClearingPlanning::computeConvexHulls()
{
  pcl::ConvexHull<PointT> hull;
  this->convex_hull_objects.resize(this->objects.size());//prepare the conve hull vector
  this->convex_hull_vertices.resize(this->objects.size());
  for (uint i = 0; i < this->objects.size(); ++i)
  { 
    hull.setInputCloud(this->objects[i].makeShared());
    hull.reconstruct(this->convex_hull_objects[i], this->convex_hull_vertices[i]);
    
    if (hull.getDimension() != 3)
      PCL_ERROR ("No convex 3D surface found for object %d.\n",i);
  }
}

void CTableClearingPlanning::testTranslation(uint idx)
{
  Eigen::Vector4f translation;
  translation[0] = 0.1*principal_directions_objects[idx].dir1[0];
  translation[1] = 0.1*principal_directions_objects[idx].dir1[1];
  translation[2] = 0.1*principal_directions_objects[idx].dir1[2];
  this->translate(this->convex_hull_objects[idx],this->convex_hull_objects[idx],translation);
}

void CTableClearingPlanning::computeSurfaceGraspPoint(Eigen::Vector3f& surface_point, uint idx)
{
  //we project the centroid on the plane
  Eigen::Vector3f centroid_projected;
  pcl::geometry::project(this->principal_directions_objects[idx].centroid.head<3>(),this->plane_origin,this->plane_normal,centroid_projected);

  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud (this->projections[idx].makeShared());
  uint K = 1;

  PointT searchPoint = conv::vector3f2pointT<PointT>(centroid_projected);
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    surface_point = conv::pointT2vector3f<PointT>(this->objects[idx].points[pointIdxNKNSearch[0]]);
    //surface_point = this->pointT2vector3f(this->objects[idx].points[pointIdxNKNSearch[0]]);
  }
  else
  {
    PCL_ERROR("Impossible finding a surface contact");
  }
}

void CTableClearingPlanning::computeConcaveHulls()
{
  pcl::ConcaveHull<PointT> hull;
  this->concave_hull_objects.resize(this->objects.size());//prepare the conve hull vector
  for (uint i = 0; i < this->objects.size(); ++i)
  { 
    hull.setInputCloud(this->objects[i].makeShared());
    hull.setAlpha(1.0);
    hull.reconstruct(this->concave_hull_objects[i]);
  }
}

void CTableClearingPlanning::computeProjectionsOnTable()
{
  this->projections.resize(0);
  for (uint i = 0; i < this->objects.size(); ++i)
  {
    PointCloudT projection_cloud;
    // projects all the points on the plane
    for (uint p = 0; p < this->objects[i].points.size(); ++p)
    {
      Eigen::Vector3f eigen_point = this->pointT2vector3f(this->objects[i].points[p]);
      Eigen::Vector3f proj_eigen_point;
      pcl::geometry::project(eigen_point,this->plane_origin,this->plane_normal,proj_eigen_point);
      PointT projection_point = this->objects[i].points[p];
      projection_point.x = proj_eigen_point[0];
      projection_point.y = proj_eigen_point[1];
      projection_point.z = proj_eigen_point[2];
      projection_cloud.points.push_back(projection_point);
    }
    this->projections.push_back(projection_cloud);
  }
}

void CTableClearingPlanning::computeProjectionsConvexHull()
{
  pcl::ConvexHull<PointT> hull;
  hull.setDimension(2);
  this->convex_hull_projections.resize(this->objects.size());//prepare the conve hull vector
  this->convex_hull_vertices_projections.resize(this->objects.size());
  for (uint i = 0; i < this->projections.size(); ++i)
  { 
    hull.setInputCloud(this->projections[i].makeShared());
    hull.reconstruct(this->convex_hull_projections[i], this->convex_hull_vertices_projections[i]);
    
    if (!(hull.getDimension() >= 2))
      PCL_ERROR ("No convex 3D, or 2D, surface found for object %d.\n",i);
  }
}

void CTableClearingPlanning::computeOnTopPredicates()
{
  pcl::CropHull<PointT> crop;
  crop.setCropOutside(true);

  if(this->convex_hull_vertices_projections.size() == 0)
    this->computeProjectionsConvexHull();

  this->on_top_predicates.resize(this->n_objects);
  // for each object 
  for (uint i = 0; i < this->n_objects; ++i)
  {
    
    //for all the other objects - we should implemente a better strategy to choose
    // the close ones
    for (uint h = 0; h < this->n_objects; ++h)
    {
      if(h != i)
      {
        std::vector< int > indices1, indices2;
        // first check
        crop.setHullCloud(this->convex_hull_projections[h].makeShared());
        crop.setHullIndices(this->convex_hull_vertices_projections[h]);
        crop.setInputCloud(this->projections[i].makeShared());
        crop.setDim(2);
        crop.filter(indices1);
        
        // second check
        crop.setHullCloud(this->convex_hull_projections[i].makeShared());
        crop.setHullIndices(this->convex_hull_vertices_projections[i]);
        crop.setInputCloud(this->projections[h].makeShared()); 
        crop.setDim(2); 
        crop.filter(indices2);

        //if(indices1.size() > 0 && indices2.size() == 0)
        if(indices1.size() > 100 && indices2.size() < 100)
        {
          std::cout << "on top " << i << " " << h << std::endl;
          this->on_top_predicates[i].push_back(h); 
        }
      }
    }
  }
}

void CTableClearingPlanning::setPushingLimit(double pushing_limit)
{
  this->pushing_limit = pushing_limit;
}

void CTableClearingPlanning::computeBlockPredicates(bool print)
{
  if(this->convex_hull_objects.size() == 0)
  {
      this->computeProjectionsOnTable();
      this->computeRichConvexHulls();
  }
  if(this->principal_directions_objects.size() == 0)
  {
    this->computePrincipalDirections();
  }
  tic();
  for (uint obj_idx = 0; obj_idx < this->n_objects; ++obj_idx)
  {
    for (uint dir_idx = 1; dir_idx <= 4; ++dir_idx)
    {
      fcl::Vec3f T;
      double x,y,z;

      if((dir_idx < 1) || (dir_idx > 4))
      {
        PCL_ERROR("Index of the direction wrong: %d it has to belong to be 1,2,3,4\n",dir_idx);
        return;
      }
      uint n = 1;
      //for (uint n = 1; n <= this->n_pushes; ++n)
      double step_translation = 0;
      while(step_translation < this->pushing_limit)
      {
        switch(dir_idx)
        {
          case 1 :
                  step_translation = n * this->principal_directions_objects[obj_idx].dir1_limit;
                  if (step_translation > this->pushing_limit) //stop when we reach the desired pushing limit 
                    step_translation = this->pushing_limit;
                  x =  step_translation*principal_directions_objects[obj_idx].dir1[0];
                  y =  step_translation*principal_directions_objects[obj_idx].dir1[1];
                  z =  step_translation*principal_directions_objects[obj_idx].dir1[2];
                  T.setValue(x,y,z);               

                  break;
          case 2 :
                  step_translation = n * this->principal_directions_objects[obj_idx].dir1_limit;
                  if (step_translation > this->pushing_limit) //stop when we reach the desired pushing limit 
                    step_translation = this->pushing_limit;
                  x =  step_translation*principal_directions_objects[obj_idx].dir2[0];
                  y =  step_translation*principal_directions_objects[obj_idx].dir2[1];
                  z =  step_translation*principal_directions_objects[obj_idx].dir2[2];
                  T.setValue(x,y,z);

                  break; 
          case 3 :
                  step_translation = n * this->principal_directions_objects[obj_idx].dir3_limit;
                  if (step_translation > this->pushing_limit) //stop when we reach the desired pushing limit 
                    step_translation = this->pushing_limit;
                  x =  step_translation*principal_directions_objects[obj_idx].dir3[0];
                  y =  step_translation*principal_directions_objects[obj_idx].dir3[1];
                  z =  step_translation*principal_directions_objects[obj_idx].dir3[2];
                  T.setValue(x,y,z);

                  break; 
          case 4 :
                  step_translation = n * this->principal_directions_objects[obj_idx].dir3_limit;
                  if (step_translation > this->pushing_limit) //stop when we reach the desired pushing limit 
                    step_translation = this->pushing_limit;
                  x =  step_translation*principal_directions_objects[obj_idx].dir4[0];
                  y =  step_translation*principal_directions_objects[obj_idx].dir4[1];
                  z =  step_translation*principal_directions_objects[obj_idx].dir4[2];
                  T.setValue(x,y,z);

                  break; 
          default: break;
        }
        // identity matrix -> no rotation
        fcl::Matrix3f R(1,0,0,
                        0,1,0,
                        0,0,1);
        //for all the other objects
        fcl::Transform3f pose(R, T);

        for (uint i = 0; i < this->objects.size(); ++i)
        {
          if(i != obj_idx)
            if(this->areObjectCollidingFcl(obj_idx,pose,i))
            {
              //std::cout << "Object " << obj_idx << " is colliding with object " << i << " in direction " << dir_idx << std::endl;
              switch(dir_idx)
              {
                case 1 :            
                        this->blocks_predicates[obj_idx].block_dir1.push_back(i);
                        break;
                case 2 :
                        this->blocks_predicates[obj_idx].block_dir2.push_back(i);
                        break; 
                case 3 :
                        this->blocks_predicates[obj_idx].block_dir3.push_back(i);
                        break; 
                case 4 :
                        this->blocks_predicates[obj_idx].block_dir4.push_back(i);
                        break; 
                default: break;
              }
            }
        }
        n++;
      }

      // ----------------------- END EFFECTOR --------------------------------------

      PrincipalDirectionsProjected* pd = &(principal_directions_objects[obj_idx]);
      Eigen::Matrix3f rot;
      
      // project centroid to the table 
      Eigen::Vector3f eigen_point = pd->centroid.head<3>();
      Eigen::Vector3f proj_eigen_point;
      pcl::geometry::project(eigen_point,this->plane_origin,this->plane_normal,proj_eigen_point);

      Eigen::Vector3f scaled_diff = eigen_point - proj_eigen_point;
      scaled_diff.normalize();
      scaled_diff = (this->ee_simple_model.distance_plane )* scaled_diff;
      
      Eigen::Vector3f new_centroid = proj_eigen_point + scaled_diff;

      Eigen::Vector3f normal;
      switch(dir_idx)
  {
    case 1 :
            step_translation = - this->aabb_objects[obj_idx].deep/2 +
                               - this->ee_simple_model.deep/2;
            x =  step_translation*principal_directions_objects[obj_idx].dir1[0] + 
                 new_centroid[0];
            y =  step_translation*principal_directions_objects[obj_idx].dir1[1] +
                 new_centroid[1];
            z =  step_translation*principal_directions_objects[obj_idx].dir1[2] +
                 new_centroid[2];
            T.setValue(x,y,z);

            // old frame
            // x axis -> dir1, y axis-> dir3
            // rot(0,0) = pd->dir1[0]; rot(0,1) = pd->dir1[1]; rot(0,2) = pd->dir1[2];
            // rot(1,0) = pd->dir3[0]; rot(1,1) = pd->dir3[1]; rot(1,2) = pd->dir3[2];
            // rot(2,0) = this->plane_normal[0]; rot(2,1) = this->plane_normal[1]; rot(2,2) = this->plane_normal[2];
            //rot(2,0) = normal[0]; rot(2,1) = normal[1]; rot(2,2) = normal[2];

            rot(0,0) = pd->dir3[0]; rot(0,1) = pd->dir3[1]; rot(0,2) = pd->dir3[2];
            rot(1,0) = this->plane_normal[0]; rot(1,1) = this->plane_normal[1]; rot(1,2) = this->plane_normal[2];
            rot(2,0) = pd->dir1[0]; rot(2,1) = pd->dir1[1]; rot(2,2) = pd->dir1[2];
            
            break;
    case 2 :
            step_translation = - this->aabb_objects[obj_idx].deep/2 +
                               - this->ee_simple_model.deep/2;           
            x =  step_translation*principal_directions_objects[obj_idx].dir2[0] + 
                 new_centroid[0];
            y =  step_translation*principal_directions_objects[obj_idx].dir2[1] +
                 new_centroid[1];
            z =  step_translation*principal_directions_objects[obj_idx].dir2[2] +
                 new_centroid[2];
            T.setValue(x,y,z);

            // old frame
            // rot(0,0) = pd->dir2[0]; rot(0,1) = pd->dir2[1]; rot(0,2) = pd->dir2[2];
            // rot(1,0) = pd->dir4[0]; rot(1,1) = pd->dir4[1]; rot(1,2) = pd->dir4[2];
            // rot(2,0) = this->plane_normal[0]; rot(2,1) = this->plane_normal[1]; rot(2,2) = this->plane_normal[2];

            rot(0,0) = pd->dir4[0]; rot(0,1) = pd->dir4[1]; rot(0,2) = pd->dir4[2];
            rot(1,0) = this->plane_normal[0]; rot(1,1) = this->plane_normal[1]; rot(1,2) = this->plane_normal[2];
            rot(2,0) = pd->dir2[0]; rot(2,1) = pd->dir2[1]; rot(2,2) = pd->dir2[2];
            


            break; 
    case 3 :
            step_translation = - this->aabb_objects[obj_idx].width/2 +
                               - this->ee_simple_model.deep/2;
            x =  step_translation*principal_directions_objects[obj_idx].dir3[0] + 
                 new_centroid[0];
            y =  step_translation*principal_directions_objects[obj_idx].dir3[1] +
                 new_centroid[1];
            z =  step_translation*principal_directions_objects[obj_idx].dir3[2] +
                 new_centroid[2];
            T.setValue(x,y,z);

            // old frame
            // rot(0,0) = pd->dir3[0]; rot(0,1) = pd->dir3[1]; rot(0,2) = pd->dir3[2];
            // rot(1,0) = pd->dir1[0]; rot(1,1) = pd->dir1[1]; rot(1,2) = pd->dir1[2];
            // rot(2,0) = -this->plane_normal[0]; rot(2,1) = -this->plane_normal[1]; rot(2,2) = -this->plane_normal[2];

            rot(0,0) = pd->dir2[0]; rot(0,1) = pd->dir2[1]; rot(0,2) = pd->dir2[2];
            rot(1,0) = this->plane_normal[0]; rot(1,1) = this->plane_normal[1]; rot(1,2) = this->plane_normal[2];
            rot(2,0) = pd->dir3[0]; rot(2,1) = pd->dir3[1]; rot(2,2) = pd->dir3[2];
            

            break; 
    case 4 :
            step_translation = - this->aabb_objects[obj_idx].width/2 +
                               - this->ee_simple_model.deep/2;
            x =  step_translation*principal_directions_objects[obj_idx].dir4[0] + 
                 new_centroid[0];
            y =  step_translation*principal_directions_objects[obj_idx].dir4[1] +
                 new_centroid[1];
            z =  step_translation*principal_directions_objects[obj_idx].dir4[2] +
                 new_centroid[2];
            T.setValue(x,y,z);

            // old frame
            // rot(0,0) = pd->dir4[0]; rot(0,1) = pd->dir4[1]; rot(0,2) = pd->dir4[2];
            // rot(1,0) = pd->dir2[0]; rot(1,1) = pd->dir2[1]; rot(1,2) = pd->dir2[2];
            // rot(2,0) = -this->plane_normal[0]; rot(2,1) = -this->plane_normal[1]; rot(2,2) = -this->plane_normal[2];


            rot(0,0) = pd->dir1[0]; rot(0,1) = pd->dir1[1]; rot(0,2) = pd->dir1[2];
            rot(1,0) = this->plane_normal[0]; rot(1,1) = this->plane_normal[1]; rot(1,2) = this->plane_normal[2];
            rot(2,0) = pd->dir4[0]; rot(2,1) = pd->dir4[1]; rot(2,2) = pd->dir4[2];
            


            break; 
    default: break;
  }
      //for all the other objects

      //we have to compute the rotation accordingly to the direction

      fcl::Matrix3f R; // the rotation matrix has to be chosen accordingly to the irection, but now just let's try if it works
      Eigen::Matrix3f mat_rot = rot.inverse();
      this->eigen2FclRotation(mat_rot,R);

      fcl::Transform3f pose_ee(R, T);

      // check for all the other objects what are the ones that collide with it
      for (uint i = 0; i < this->objects.size(); ++i)
      {
        if(i != obj_idx)
          // is gripper colliding?
          if(this->isEEColliding(i,pose_ee))
          {
            //std::cout << "Object " << obj_idx << " is colliding with object " << i << " in direction " << dir_idx << std::endl;
            switch(dir_idx)
            {
              case 1 :            
                      this->blocks_predicates[obj_idx].block_dir1.push_back(i);
                      break;
              case 2 :
                      this->blocks_predicates[obj_idx].block_dir2.push_back(i);
                      break; 
              case 3 :
                      this->blocks_predicates[obj_idx].block_dir3.push_back(i);
                      break; 
              case 4 :
                      this->blocks_predicates[obj_idx].block_dir4.push_back(i);
                      break; 
              default: break;
            }
          }
      }

      //remove duplicates 
      std::vector<uint>* vec;
      switch(dir_idx)
      {
        case 1 :
                vec = &(this->blocks_predicates[obj_idx].block_dir1);            
                break;
        case 2 :
                vec = &(this->blocks_predicates[obj_idx].block_dir2);
                break; 
        case 3 :
                vec = &(this->blocks_predicates[obj_idx].block_dir3);
                break; 
        case 4 :
                vec = &(this->blocks_predicates[obj_idx].block_dir4);
                break; 
        default: break;
      }
      // reference:
      // http://stackoverflow.com/questions/1041620/whats-the-most-efficient-way-to-erase-duplicates-and-sort-a-vector
      sort( vec->begin(), vec->end() );
      vec->erase( unique( vec->begin(), vec->end() ), vec->end() );
      
      if(print)
      {
        // print blocks predicates
        std::vector<uint>* block_dir_pointer;
        switch(dir_idx)
        {
          case 1 :
                  block_dir_pointer = &(this->blocks_predicates[obj_idx].block_dir1);
                  break;
          case 2 :
                  block_dir_pointer = &(this->blocks_predicates[obj_idx].block_dir2);
                  break; 
          case 3 :
                  block_dir_pointer = &(this->blocks_predicates[obj_idx].block_dir3);
                  break; 
          case 4 :
                  block_dir_pointer = &(this->blocks_predicates[obj_idx].block_dir4);
                  break; 
          default: break;
        }
        for (uint i = 0; i < block_dir_pointer->size(); ++i)
        {
          std::cout << "Object " << obj_idx << " is colliding with object " <<  block_dir_pointer->at(i) << " in direction " << dir_idx << std::endl;
        }
      }    
    }
  }
  toc();
}

void CTableClearingPlanning::visualComputeBlockPredicates(Visualizer viewer, uint obj_idx,uint dir_idx, bool visualization,
                                                              bool print)
{ 
  fcl::Vec3f T;
  double x,y,z;

  if((dir_idx < 1) || (dir_idx > 4))
  {
    PCL_ERROR("Index of the direction wrong: %d it has to belong to be 1,2,3,4\n",dir_idx);
    return;
  }
  uint n = 1;
  //for (uint n = 1; n <= this->n_pushes; ++n)
  double step_translation = 0;
  while(step_translation < this->pushing_limit)
  {
    switch(dir_idx)
    {
      case 1 :
              step_translation = n * this->principal_directions_objects[obj_idx].dir1_limit;
              if (step_translation > this->pushing_limit) //stop when we reach the desired pushing limit 
                step_translation = this->pushing_limit;
              x =  step_translation*principal_directions_objects[obj_idx].dir1[0];
              y =  step_translation*principal_directions_objects[obj_idx].dir1[1];
              z =  step_translation*principal_directions_objects[obj_idx].dir1[2];
              T.setValue(x,y,z);               

              break;
      case 2 :
              step_translation = n * this->principal_directions_objects[obj_idx].dir1_limit;
              if (step_translation > this->pushing_limit) //stop when we reach the desired pushing limit 
                step_translation = this->pushing_limit;
              x =  step_translation*principal_directions_objects[obj_idx].dir2[0];
              y =  step_translation*principal_directions_objects[obj_idx].dir2[1];
              z =  step_translation*principal_directions_objects[obj_idx].dir2[2];
              T.setValue(x,y,z);

              break; 
      case 3 :
              step_translation = n * this->principal_directions_objects[obj_idx].dir3_limit;
              if (step_translation > this->pushing_limit) //stop when we reach the desired pushing limit 
                step_translation = this->pushing_limit;
              x =  step_translation*principal_directions_objects[obj_idx].dir3[0];
              y =  step_translation*principal_directions_objects[obj_idx].dir3[1];
              z =  step_translation*principal_directions_objects[obj_idx].dir3[2];
              T.setValue(x,y,z);

              break; 
      case 4 :
              step_translation = n * this->principal_directions_objects[obj_idx].dir3_limit;
              if (step_translation > this->pushing_limit) //stop when we reach the desired pushing limit 
                step_translation = this->pushing_limit;
              x =  step_translation*principal_directions_objects[obj_idx].dir4[0];
              y =  step_translation*principal_directions_objects[obj_idx].dir4[1];
              z =  step_translation*principal_directions_objects[obj_idx].dir4[2];
              T.setValue(x,y,z);

              break; 
      default: break;
    }
    // identity matrix -> no rotation
    fcl::Matrix3f R(1,0,0,
                    0,1,0,
                    0,0,1);
    //for all the other objects
    fcl::Transform3f pose(R, T);

    if(visualization)
    {
      Eigen::Vector4f translation;
      Eigen::Quaternionf quat;
      PointCloudT convex_hull_translated;
      this->fcl2EigenTransform(translation,quat, pose);
      pcl::transformPointCloud<PointT>(this->convex_hull_objects[obj_idx], convex_hull_translated, translation.head<3>()  , quat);

      std::string mesh_idx = "";
      std::ostringstream convert;   // stream used for the conversion
      convert << n;
      mesh_idx += convert.str();
      viewer->addPolygonMesh<PointT>(convex_hull_translated.makeShared(), this->convex_hull_vertices[obj_idx],mesh_idx);
    }

    for (uint i = 0; i < this->objects.size(); ++i)
    {
      if(i != obj_idx)
        if(this->areObjectCollidingFcl(obj_idx,pose,i))
        {
          //std::cout << "Object " << obj_idx << " is colliding with object " << i << " in direction " << dir_idx << std::endl;
          switch(dir_idx)
          {
            case 1 :            
                    this->blocks_predicates[obj_idx].block_dir1.push_back(i);
                    break;
            case 2 :
                    this->blocks_predicates[obj_idx].block_dir2.push_back(i);
                    break; 
            case 3 :
                    this->blocks_predicates[obj_idx].block_dir3.push_back(i);
                    break; 
            case 4 :
                    this->blocks_predicates[obj_idx].block_dir4.push_back(i);
                    break; 
            default: break;
          }
        }
    }
    n++;
  }

  // ----------------------- END EFFECTOR --------------------------------------

  PrincipalDirectionsProjected* pd = &(principal_directions_objects[obj_idx]);
  Eigen::Matrix3f rot;
  
  // project centroid to the table 
  Eigen::Vector3f eigen_point = pd->centroid.head<3>();
  Eigen::Vector3f proj_eigen_point;
  pcl::geometry::project(eigen_point,this->plane_origin,this->plane_normal,proj_eigen_point);

  Eigen::Vector3f scaled_diff = eigen_point - proj_eigen_point;
  scaled_diff.normalize();
  scaled_diff = (this->ee_simple_model.distance_plane )* scaled_diff;
  
  Eigen::Vector3f new_centroid = proj_eigen_point + scaled_diff;

  Eigen::Vector3f normal;
  switch(dir_idx)
  {
    case 1 :
            step_translation = - this->aabb_objects[obj_idx].deep/2 +
                               - this->ee_simple_model.deep/2;
            x =  step_translation*principal_directions_objects[obj_idx].dir1[0] + 
                 new_centroid[0];
            y =  step_translation*principal_directions_objects[obj_idx].dir1[1] +
                 new_centroid[1];
            z =  step_translation*principal_directions_objects[obj_idx].dir1[2] +
                 new_centroid[2];
            T.setValue(x,y,z);

            // old frame
            // x axis -> dir1, y axis-> dir3
            // rot(0,0) = pd->dir1[0]; rot(0,1) = pd->dir1[1]; rot(0,2) = pd->dir1[2];
            // rot(1,0) = pd->dir3[0]; rot(1,1) = pd->dir3[1]; rot(1,2) = pd->dir3[2];
            // rot(2,0) = this->plane_normal[0]; rot(2,1) = this->plane_normal[1]; rot(2,2) = this->plane_normal[2];
            //rot(2,0) = normal[0]; rot(2,1) = normal[1]; rot(2,2) = normal[2];

            rot(0,0) = pd->dir3[0]; rot(0,1) = pd->dir3[1]; rot(0,2) = pd->dir3[2];
            rot(1,0) = this->plane_normal[0]; rot(1,1) = this->plane_normal[1]; rot(1,2) = this->plane_normal[2];
            rot(2,0) = pd->dir1[0]; rot(2,1) = pd->dir1[1]; rot(2,2) = pd->dir1[2];
            
            break;
    case 2 :
            step_translation = - this->aabb_objects[obj_idx].deep/2 +
                               - this->ee_simple_model.deep/2;           
            x =  step_translation*principal_directions_objects[obj_idx].dir2[0] + 
                 new_centroid[0];
            y =  step_translation*principal_directions_objects[obj_idx].dir2[1] +
                 new_centroid[1];
            z =  step_translation*principal_directions_objects[obj_idx].dir2[2] +
                 new_centroid[2];
            T.setValue(x,y,z);

            // old frame
            // rot(0,0) = pd->dir2[0]; rot(0,1) = pd->dir2[1]; rot(0,2) = pd->dir2[2];
            // rot(1,0) = pd->dir4[0]; rot(1,1) = pd->dir4[1]; rot(1,2) = pd->dir4[2];
            // rot(2,0) = this->plane_normal[0]; rot(2,1) = this->plane_normal[1]; rot(2,2) = this->plane_normal[2];

            rot(0,0) = pd->dir4[0]; rot(0,1) = pd->dir4[1]; rot(0,2) = pd->dir4[2];
            rot(1,0) = this->plane_normal[0]; rot(1,1) = this->plane_normal[1]; rot(1,2) = this->plane_normal[2];
            rot(2,0) = pd->dir2[0]; rot(2,1) = pd->dir2[1]; rot(2,2) = pd->dir2[2];
            


            break; 
    case 3 :
            step_translation = - this->aabb_objects[obj_idx].width/2 +
                               - this->ee_simple_model.deep/2;
            x =  step_translation*principal_directions_objects[obj_idx].dir3[0] + 
                 new_centroid[0];
            y =  step_translation*principal_directions_objects[obj_idx].dir3[1] +
                 new_centroid[1];
            z =  step_translation*principal_directions_objects[obj_idx].dir3[2] +
                 new_centroid[2];
            T.setValue(x,y,z);

            // old frame
            // rot(0,0) = pd->dir3[0]; rot(0,1) = pd->dir3[1]; rot(0,2) = pd->dir3[2];
            // rot(1,0) = pd->dir1[0]; rot(1,1) = pd->dir1[1]; rot(1,2) = pd->dir1[2];
            // rot(2,0) = -this->plane_normal[0]; rot(2,1) = -this->plane_normal[1]; rot(2,2) = -this->plane_normal[2];

            rot(0,0) = pd->dir2[0]; rot(0,1) = pd->dir2[1]; rot(0,2) = pd->dir2[2];
            rot(1,0) = this->plane_normal[0]; rot(1,1) = this->plane_normal[1]; rot(1,2) = this->plane_normal[2];
            rot(2,0) = pd->dir3[0]; rot(2,1) = pd->dir3[1]; rot(2,2) = pd->dir3[2];
            

            break; 
    case 4 :
            step_translation = - this->aabb_objects[obj_idx].width/2 +
                               - this->ee_simple_model.deep/2;
            x =  step_translation*principal_directions_objects[obj_idx].dir4[0] + 
                 new_centroid[0];
            y =  step_translation*principal_directions_objects[obj_idx].dir4[1] +
                 new_centroid[1];
            z =  step_translation*principal_directions_objects[obj_idx].dir4[2] +
                 new_centroid[2];
            T.setValue(x,y,z);

            // old frame
            // rot(0,0) = pd->dir4[0]; rot(0,1) = pd->dir4[1]; rot(0,2) = pd->dir4[2];
            // rot(1,0) = pd->dir2[0]; rot(1,1) = pd->dir2[1]; rot(1,2) = pd->dir2[2];
            // rot(2,0) = -this->plane_normal[0]; rot(2,1) = -this->plane_normal[1]; rot(2,2) = -this->plane_normal[2];


            rot(0,0) = pd->dir1[0]; rot(0,1) = pd->dir1[1]; rot(0,2) = pd->dir1[2];
            rot(1,0) = this->plane_normal[0]; rot(1,1) = this->plane_normal[1]; rot(1,2) = this->plane_normal[2];
            rot(2,0) = pd->dir4[0]; rot(2,1) = pd->dir4[1]; rot(2,2) = pd->dir4[2];
            


            break; 
    default: break;
  }
  //for all the other objects

  //we have to compute the rotation accordingly to the direction

  fcl::Matrix3f R; // the rotation matrix has to be chosen accordingly to the irection, but now just let's try if it works
  Eigen::Matrix3f mat_rot = rot.inverse();
  this->eigen2FclRotation(mat_rot,R);

  fcl::Transform3f pose_ee(R, T);

  if(visualization)
  {
    Eigen::Vector4f translation_ee;
    Eigen::Quaternionf quat_ee;
    pcl::PointCloud<pcl::PointXYZ> ee_translated;
    this->fcl2EigenTransform(translation_ee, quat_ee, pose_ee);
    pcl::transformPointCloud<pcl::PointXYZ>(this->ee_simple_model.cloud, ee_translated, translation_ee.head<3>()  , quat_ee);

    viewer->addPolygonMesh<pcl::PointXYZ>(ee_translated.makeShared(), this->ee_simple_model.vertices,"ee");
  }

  // check for all the other objects what are the ones that collide with it
  for (uint i = 0; i < this->objects.size(); ++i)
  {
    if(i != obj_idx)
      // is gripper colliding?
      if(this->isEEColliding(i,pose_ee))
      {
        //std::cout << "Object " << obj_idx << " is colliding with object " << i << " in direction " << dir_idx << std::endl;
        switch(dir_idx)
        {
          case 1 :            
                  this->blocks_predicates[obj_idx].block_dir1.push_back(i);
                  break;
          case 2 :
                  this->blocks_predicates[obj_idx].block_dir2.push_back(i);
                  break; 
          case 3 :
                  this->blocks_predicates[obj_idx].block_dir3.push_back(i);
                  break; 
          case 4 :
                  this->blocks_predicates[obj_idx].block_dir4.push_back(i);
                  break; 
          default: break;
        }
      }
  }

  //remove duplicates 
  std::vector<uint>* vec;
  switch(dir_idx)
  {
    case 1 :
            vec = &(this->blocks_predicates[obj_idx].block_dir1);            
            break;
    case 2 :
            vec = &(this->blocks_predicates[obj_idx].block_dir2);
            break; 
    case 3 :
            vec = &(this->blocks_predicates[obj_idx].block_dir3);
            break; 
    case 4 :
            vec = &(this->blocks_predicates[obj_idx].block_dir4);
            break; 
    default: break;
  }
  // reference:
  // http://stackoverflow.com/questions/1041620/whats-the-most-efficient-way-to-erase-duplicates-and-sort-a-vector
  sort( vec->begin(), vec->end() );
  vec->erase( unique( vec->begin(), vec->end() ), vec->end() );
  
  if(print)
  {
    // print blocks predicates
    std::vector<uint>* block_dir_pointer;
    switch(dir_idx)
    {
      case 1 :
              block_dir_pointer = &(this->blocks_predicates[obj_idx].block_dir1);
              break;
      case 2 :
              block_dir_pointer = &(this->blocks_predicates[obj_idx].block_dir2);
              break; 
      case 3 :
              block_dir_pointer = &(this->blocks_predicates[obj_idx].block_dir3);
              break; 
      case 4 :
              block_dir_pointer = &(this->blocks_predicates[obj_idx].block_dir4);
              break; 
      default: break;
    }
    for (uint i = 0; i < block_dir_pointer->size(); ++i)
    {
      std::cout << "Object " << obj_idx << " is colliding with object " <<  block_dir_pointer->at(i) << " in direction " << dir_idx << std::endl;
    }
  }
}  

void CTableClearingPlanning::computeBlockGraspPredicates(bool print)
{
  if(this->grasping_poses.size()==0)
  {
    PCL_ERROR("The grasping poses are still not computed.");
    return;
  }

  this->block_grasp_predicates.resize(this->n_objects);
  for (int i = 0; i < this->n_objects; ++i)
  {
    GraspingPose* gp = &(this->grasping_poses[i]);
    fcl::Matrix3f R; // the rotation matrix has to be chosen accordingly to the irection, but now just let's try if it works
    this->eigen2FclRotation(gp->rotation,R);
    fcl::Vec3f T;
    T.setValue( gp->translation[0],
                gp->translation[1],
                gp->translation[2]);
    fcl::Transform3f grasp_pose(R, T);
    for (int o = 0; o < this->n_objects; ++o)
    {
      if(i != o)
        if(this->isFingersModelColliding(o,grasp_pose))
        {
          this->block_grasp_predicates[i].push_back(o);
          if(print)
            std::cout << "Object " << o << " blocks object " << i << " to be grasped\n";
        }
    }
  }
}

void CTableClearingPlanning::testFcl()
{
  // set mesh triangles and vertice indices
  std::vector<fcl::Vec3f> vertices;
  std::vector<fcl::Triangle> triangles;

  // code to set the vertices and triangles
  // set just a simple shape, 1 triangle
  fcl::Vec3f vec_tmp1(0,0,0);
  vertices.push_back(vec_tmp1);
  fcl::Vec3f vec_tmp2(0,0,1);
  vertices.push_back(vec_tmp2);
  fcl::Vec3f vec_tmp3(0,1,0);
  vertices.push_back(vec_tmp3);
  fcl::Vec3f vec_tmp4(0.5,0.5,0);
  vertices.push_back(vec_tmp4);

  fcl::Triangle triangle_tmp(0,1,2);
  triangles.push_back(triangle_tmp);
  fcl::Triangle triangle_tmp2(0,1,3);
  triangles.push_back(triangle_tmp2);
  fcl::Triangle triangle_tmp3(0,2,3);
  triangles.push_back(triangle_tmp3);
  fcl::Triangle triangle_tmp4(1,2,3);
  triangles.push_back(triangle_tmp4);

  // BVHModel is a template class for mesh geometry, for default OBBRSS template is used
  typedef fcl::BVHModel<fcl::OBBRSS> Model;
  Model* model = new Model();
  //fcl::BVHModel<fcl::BV>* model = new Model();
  // add the mesh data into the BVHModel structure
  
  model->beginModel();
  model->addSubModel(vertices, triangles);
  model->endModel();

  // identity matrix -> no rotation
  fcl::Matrix3f R(1,0,0,
                  0,1,0,
                  0,0,1);
  fcl::Vec3f T(0,0,0); // no translation

  fcl::Matrix3f R2(1,0,0,
                  0,1,0,
                  0,0,1);
  fcl::Vec3f T2(0,0,0); // no translation


  fcl::Transform3f pose(R, T);
  fcl::Transform3f pose2(R2, T2);
  

  bool enable_contact = true;
  int num_max_contacts = std::numeric_limits<int>::max();
  // set the collision request structure, here we just use the default setting
  fcl::CollisionRequest request(num_max_contacts, enable_contact);
  // result will be returned via the collision result structure
  fcl::CollisionResult result;

  std::cout << "Calling fcl::collide()\n";
  int num_contacts = fcl::collide(model, pose, model, pose2, 
                             request, result);
    
  if(num_contacts > 0)
    PCL_ERROR("FCL::COLLISION DETCTED\n");

}

void CTableClearingPlanning::testFcl2()
{

  FclMesh mesh1,mesh2;

  mesh1 = this->pcl2FclConvexHull(1);
  mesh2 = this->pcl2FclConvexHull(2);

  // BVHModel is a template class for mesh geometry, for default OBBRSS template is used
  typedef fcl::BVHModel<fcl::OBBRSS> Model;
  Model* model = new Model();
  //fcl::BVHModel<fcl::BV>* model = new Model();
  // add the mesh data into the BVHModel structure
  std::cout << "Setting model\n";
  model->beginModel();
  model->addSubModel(mesh1.vertices, mesh1.triangles);
  model->endModel();
  std::cout << "model ready\n";

  Model* model2 = new Model();
  //fcl::BVHModel<fcl::BV>* model = new Model();
  // add the mesh data into the BVHModel structure
  std::cout << "Setting model 2\n";
  model2->beginModel();
  model2->addSubModel(mesh2.vertices, mesh2.triangles);
  model2->endModel();
  std::cout << "model ready 2\n";

  // identity matrix -> no rotation
  fcl::Matrix3f R(1,0,0,
                  0,1,0,
                  0,0,1);
  fcl::Vec3f T(0,0,0); // no translation

  fcl::Matrix3f R2(1,0,0,
                  0,1,0,
                  0,0,1);
  fcl::Vec3f T2(0,0.0,0); // no translation


  fcl::Transform3f pose(R, T);
  fcl::Transform3f pose2(R2, T2);
  

  bool enable_contact = true;
  int num_max_contacts = std::numeric_limits<int>::max();
  // set the collision request structure, here we just use the default setting
  fcl::CollisionRequest request(num_max_contacts, enable_contact);
  // result will be returned via the collision result structure
  fcl::CollisionResult result;

  std::cout << "Calling fcl::collide()\n";
  int num_contacts = fcl::collide(model, pose, model2, pose2, 
                             request, result);
    
  if(num_contacts > 0)
    PCL_ERROR("FCL::COLLISION DETCTED\n");

}

void CTableClearingPlanning::viewerAddConvexHulls(Visualizer viewer, uint idx)
{
  if( (this->convex_hull_objects.size() >= (idx +1) ) && (this->convex_hull_vertices.size() >= (idx +1)) ) // check if the index is correct 
    viewer->addPolygonMesh<PointT>(this->convex_hull_objects[idx].makeShared(), this->convex_hull_vertices[idx] );  
  else
    PCL_ERROR("In CTableClearingPlanning::showConvexHulls -> convex hull not properly defined. Probaly is an indexing problem.");
}

void CTableClearingPlanning::viewerAddConcaveHulls(Visualizer viewer,uint idx)
{
  if( (this->concave_hull_projections.size() >= (idx +1) ) && (this->concave_hull_vertices_projections.size() >= (idx +1)) ) // check if the index is correct 
    viewer->addPolygonMesh<PointT>(this->concave_hull_projections[idx].makeShared(), this->concave_hull_vertices_projections[idx] );  
  else
    PCL_ERROR("In CTableClearingPlanning::showConcaveHulls -> concave hull not properly defined. Probaly is an indexing problem.");
}

void CTableClearingPlanning::computeRichConvexHulls()
{
  if(this->projections.size() == 0)
  {
    PCL_ERROR("objects Projections not computed\n");
    return;
  }

  pcl::ConvexHull<PointT> hull;
  this->convex_hull_objects.resize(this->projections.size());//prepare the conve hull vector
  this->convex_hull_vertices.resize(this->projections.size());
  this->rich_objects.resize(this->projections.size());
  for (uint i = 0; i < this->projections.size(); ++i)
  {
    this->rich_objects[i] = this->objects[i] +  this->projections[i];
    hull.setInputCloud(this->rich_objects[i].makeShared());
    hull.reconstruct(this->convex_hull_objects[i], this->convex_hull_vertices[i]);
    if (hull.getDimension() != 3)
      PCL_ERROR ("No convex 3D surface found for object %d.\n",i);
    
    //only for test, check if the vertices define a triangle
    // for (uint h = 0; h < this->convex_hull_vertices[i].size() ; ++h)
    // {
    //   if(this->convex_hull_vertices[i][h].vertices.size() != 3)
    //       PCL_ERROR("Not triangle mesh\n"); 
    // }
  }

}

void CTableClearingPlanning::computeRichConcaveHulls()
{
  if(this->projections.size() == 0)
  {
    PCL_ERROR("objects Projections not computed\n");
    return;
  }

  pcl::ConcaveHull<PointT> hull;
  hull.setKeepInformation(true);
  hull.setAlpha(0.00);

  this->concave_hull_objects.resize(this->projections.size());//prepare the conve hull vector
  this->concave_hull_vertices.resize(this->projections.size());
  this->rich_objects.resize(this->projections.size());
  for (uint i = 0; i < this->projections.size(); ++i)
  {
    this->rich_objects[i] = this->objects[i] +  this->projections[i];
    hull.setInputCloud(this->rich_objects[i].makeShared());
    hull.reconstruct(this->concave_hull_objects[i], this->concave_hull_vertices[i]);
    if (hull.getDimension() != 3)
      PCL_ERROR ("No convex 3D surface found for object %d.\n",i);
  }
}


void CTableClearingPlanning::computePrincipalDirectionsConvexHull()
{

  //get normal vector of the plane
  Eigen::Vector3f norm_plane;
  if(! this->plane_coefficients.values.size() > 0)
  {
    std::cout << "Plane model coefficients not set - Exiting from computePrincipalDirections()" << std::endl;
    return; 
  }
  norm_plane[0] = this->plane_coefficients.values[0];
  norm_plane[1] = this->plane_coefficients.values[1];
  norm_plane[2] = this->plane_coefficients.values[2];
  norm_plane.normalize();

	//it is better computing them with the convex hull
  pcl::PCA<PointT> pca_;
  for (uint i = 0; i < this->convex_hull_objects.size(); ++i)
  {	
    PrincipalDirectionsProjected prin_dir;

    Eigen::Matrix3f evecs;
    Eigen::Matrix3f covariance_matrix;

    pca_.setInputCloud(this->convex_hull_objects[i].makeShared());
    evecs = pca_.getEigenVectors();
    
    prin_dir.dir1 << evecs.col(0);
    prin_dir.dir3 << evecs.col(1);

    //projection of the principal directions vector onto the plane
    // projection of the principal direction 1 on to the normal plane vector
    double proj_normal_dir1 = prin_dir.dir1.dot(norm_plane);
    double proj_normal_dir3 = prin_dir.dir3.dot(norm_plane);
    /*
    if (proj_normal_dir1 < 0)
      proj_normal_dir1 = - proj_normal_dir1;
    if (proj_normal_dir2 < 0)
      proj_normal_dir2 = - proj_normal_dir2;
    */
    prin_dir.dir1 = prin_dir.dir1 - proj_normal_dir1*norm_plane;
    prin_dir.dir3 = prin_dir.dir3 - proj_normal_dir3*norm_plane;

    prin_dir.dir1.normalize();
    prin_dir.dir3.normalize();

    prin_dir.dir2 = - prin_dir.dir1;
    prin_dir.dir4 = - prin_dir.dir3;
    
    pcl::compute3DCentroid(this->convex_hull_objects[i], prin_dir.centroid);

    this->principal_directions_objects.push_back(prin_dir);

  }
}
 

void CTableClearingPlanning::computePrincipalDirectionsConcaveHull()
{

  //get normal vector of the plane
  Eigen::Vector3f norm_plane;
  if(! this->plane_coefficients.values.size() > 0)
  {
    std::cout << "Plane model coefficients not set - Exiting from computePrincipalDirections()" << std::endl;
    return; 
  }
  norm_plane[0] = this->plane_coefficients.values[0];
  norm_plane[1] = this->plane_coefficients.values[1];
  norm_plane[2] = this->plane_coefficients.values[2];
  norm_plane.normalize();

  //it is better computing them with the convex hull
  pcl::PCA<PointT> pca_;
  for (uint i = 0; i < this->concave_hull_objects.size(); ++i)
  { 
    PrincipalDirectionsProjected prin_dir;

    Eigen::Matrix3f evecs;
    Eigen::Matrix3f covariance_matrix;

    pca_.setInputCloud(this->concave_hull_objects[i].makeShared());
    evecs = pca_.getEigenVectors();
    
    prin_dir.dir1 << evecs.col(0);
    prin_dir.dir3 << evecs.col(1);

    //projection of the principal directions vector onto the plane
    // projection of the principal direction 1 on to the normal plane vector
    double proj_normal_dir1 = prin_dir.dir1.dot(norm_plane);
    double proj_normal_dir3 = prin_dir.dir3.dot(norm_plane);
    /*
    if (proj_normal_dir1 < 0)
      proj_normal_dir1 = - proj_normal_dir1;
    if (proj_normal_dir2 < 0)
      proj_normal_dir2 = - proj_normal_dir2;
    */
    prin_dir.dir1 = prin_dir.dir1 - proj_normal_dir1*norm_plane;
    prin_dir.dir3 = prin_dir.dir3 - proj_normal_dir3*norm_plane;

    prin_dir.dir1.normalize();
    prin_dir.dir2.normalize();

    prin_dir.dir2 = - prin_dir.dir1;
    prin_dir.dir4 = - prin_dir.dir3;

    pcl::compute3DCentroid(this->concave_hull_objects[i], prin_dir.centroid);

    principal_directions_objects.push_back(prin_dir);

  }
}


void CTableClearingPlanning::computePrincipalDirections()
{

  //get normal vector of the plane
  Eigen::Vector3f norm_plane;
  if(! this->plane_coefficients.values.size() > 0)
  {
    std::cout << "Plane model coefficients not set - Exiting from computePrincipalDirections()" << std::endl;
    return; 
  }
  norm_plane[0] = this->plane_coefficients.values[0];
  norm_plane[1] = this->plane_coefficients.values[1];
  norm_plane[2] = this->plane_coefficients.values[2];
  norm_plane.normalize();

  //it is better computing them with the convex hull
  pcl::PCA<PointT> pca_;
  for (uint i = 0; i < this->objects.size(); ++i)
  { 
    PrincipalDirectionsProjected prin_dir;
    OriginalPrincipalDirections opd;//Original Principal Direction

    pcl::compute3DCentroid(this->objects[i], prin_dir.centroid);

    Eigen::Matrix3f evecs;
    Eigen::Matrix3f covariance_matrix;

    pca_.setInputCloud(this->objects[i].makeShared());
    evecs = pca_.getEigenVectors();
   
    opd.p1 << evecs.col(0);
    // check that p2 is on the right of p1. Remember that the normal is pointing down
    Eigen::Vector3f tmp = opd.p1.cross(this->plane_normal);
    opd.p2 << evecs.col(1);
    if (opd.p2.dot(tmp) < 0) //it is on the right of p1, make it to be at the left
    {
        opd.p2 = - opd.p2;
    }
    // we want now the p3 to be z axis, so we can use easily them for the matrix rotation
    opd.p3 << opd.p1.cross(opd.p2);
    opd.centroid = prin_dir.centroid;
    this->original_principal_directions_objects.push_back(opd);


    prin_dir.dir1 << evecs.col(0);
    prin_dir.dir3 << evecs.col(1);

    //projection of the principal directions vector onto the plane
    // projection of the principal direction 1 on to the normal plane vector
    double proj_normal_dir1 = prin_dir.dir1.dot(norm_plane);
    double proj_normal_dir3 = prin_dir.dir3.dot(norm_plane);
    /*
    if (proj_normal_dir1 < 0)
      proj_normal_dir1 = - proj_normal_dir1;
    if (proj_normal_dir2 < 0)
      proj_normal_dir2 = - proj_normal_dir2;
    */
    prin_dir.dir1 = prin_dir.dir1 - proj_normal_dir1*norm_plane;
    prin_dir.dir3 = prin_dir.dir3 - proj_normal_dir3*norm_plane;

    prin_dir.dir1.normalize();
    prin_dir.dir2.normalize();

    prin_dir.dir2 = - prin_dir.dir1;
    prin_dir.dir4 = - prin_dir.dir3;

    // knowing the direction1 and the normal plane, if we do the croos product of the dir1 with the
    // plane normal, we should get the second direction (dir3). To check that our computation is correct
    // // we do the dot product between the one that should be and the the coputed one, 
    // if the dot product is negative it means that the orientation is the inverse. 
    // Actually we could directly ocmptue the dir3 with the cross product of dir1 with the plane normal
    Eigen::Vector3f real_dir3 = - this->plane_normal.cross(prin_dir.dir1);
    if(!(real_dir3.dot(prin_dir.dir3) < 0))
    {
      prin_dir.dir3 = - prin_dir.dir3;
      prin_dir.dir4 = - prin_dir.dir4;
    }


    this->principal_directions_objects.push_back(prin_dir);

  }
}

void CTableClearingPlanning::voxelizeObjects(double leaf_size)
{
  // Create the filtering object
  pcl::VoxelGrid<PointT > vox;
  for (std::vector<PointCloudT>::iterator i = this->objects.begin(); i != this->objects.end(); ++i)
  {
    vox.setInputCloud ((*i).makeShared());
    vox.setLeafSize (leaf_size, leaf_size, leaf_size);
    vox.filter (*i);
  }
}

void CTableClearingPlanning::voxelizeFullObjects(double leaf_size)
{
  //check
  if(this->objects_full.size() == 0)
  {
    PCL_ERROR("Impossible voxelizing full objects: there are no full objects built.");
    return;
  }

  // Create the filtering object
  pcl::VoxelGrid<PointT > vox;
  for (std::vector<ObjectFull>::iterator i = this->objects_full.begin(); i != this->objects_full.end(); ++i)
  {
    // we have to get the point cloud of the sides and proejctions and the original one
    // in order to voxelize them separately to preserve the index
    PointCloudT tmp,tmp_sides,tmp_projections;
    for (uint p = 0; p < (*i).index_sides; ++p)
      tmp.points.push_back((*i).cloud.points[p]);

    vox.setInputCloud (tmp.makeShared());
    vox.setLeafSize (leaf_size, leaf_size, leaf_size);
    vox.filter (tmp);

    for (uint p = (*i).index_sides; p < (*i).index_projection; ++p)
      tmp_sides.points.push_back((*i).cloud.points[p]);

    vox.setInputCloud (tmp_sides.makeShared());
    vox.setLeafSize (leaf_size, leaf_size, leaf_size);
    vox.filter (tmp_sides);

    for (uint p = (*i).index_projection; p < (*i).cloud.points.size(); ++p)
      tmp_projections.points.push_back((*i).cloud.points[p]);
    
    vox.setInputCloud (tmp_projections.makeShared());
    vox.setLeafSize (leaf_size, leaf_size, leaf_size);
    vox.filter (tmp_projections);

    // reconstruct full voxelized object
    (*i).cloud.points.resize(0);
    (*i).cloud = tmp;
    (*i).index_sides = (*i).cloud.points.size();
    (*i).cloud += tmp_sides;
    (*i).index_projection = (*i).cloud.points.size();
    (*i).cloud += tmp_projections;
  }
}

void CTableClearingPlanning::buildFullObjectsCloud(std::vector<PointCloudT>& occluded_sides)
{
  if(occluded_sides.size() != this->objects.size()) //check they have the correct dimension
  {
    PCL_ERROR("Impossible composing the full objects cloud, the sizes do not match.\n");
    return;
  }
  this->objects_full.resize(this->n_objects);
  for (uint i = 0; i < n_objects; ++i)
  {
    this->objects_full[i].cloud = this->objects[i];
    this->objects_full[i].cloud += occluded_sides[i];
    this->objects_full[i].cloud += this->projections[i];
    this->objects_full[i].index_sides = this->objects[i].points.size();
    this->objects_full[i].index_projection = this->objects[i].points.size() + occluded_sides[i].points.size();
  }

}

void CTableClearingPlanning::setObjectsPointCloud(std::vector<PointCloudT> &objects)
{
  this->objects = objects;
  this->n_objects = objects.size();
  this->blocks_predicates.resize(this->n_objects);
}
void CTableClearingPlanning::setPlaneCoefficients(pcl::ModelCoefficients &plane_coefficients)
{
  this->plane_coefficients = plane_coefficients;
  if(! this->plane_coefficients.values.size() > 0)
  {
    PCL_ERROR("Plane model coefficients not set - Exiting from setPlaneCoefficients()");
    return; 
  }
  this->plane_normal[0] = this->plane_coefficients.values[0];
  this->plane_normal[1] = this->plane_coefficients.values[1];
  this->plane_normal[2] = this->plane_coefficients.values[2];
  this->plane_normal.normalize();

  // CHECK ON THE ORIENTATION OF THE PLANE
  //we want the plane normal to be poiting down
  Eigen::Vector3f z_ax(0,0,1);
  if(this->plane_normal.dot(z_ax) < 0)
    this->plane_normal = - this->plane_normal;

  //the normal has to be looking for the Kinect, so 
    // Eigen::Vector3f kinect_pow(0,0,1);
    // if(this->plane_normal.dot(kinect_pow) > 0 )
    //   this->plane_normal = - this->plane_normal;

  // set plane origin
  this->plane_origin[0] = 1.0;
  this->plane_origin[1] = 1.0;
  // ax+by+cz+d=0 => z = (ax+by+d)/(-c)
  this->plane_origin[2] = (this->plane_coefficients.values[0] * this->plane_origin[0] +
                          this->plane_coefficients.values[1] * this->plane_origin[1] +
                          this->plane_coefficients.values[3]) / (- this->plane_coefficients.values[2]);
}

void CTableClearingPlanning::viewerAddPrincipalDirections(Visualizer viewer, uint i)
{
  
  double length = 0.2;
  viewer->addText(" dir1 : red \n dir2 : green \n dir3 : blue \n dir4 : white",0,100);
  for (uint d = 1; d <= 4; ++d)
  { 
    PointT pd_1,pd_2;
    std::string line_name;
    std::ostringstream convert;   // stream used for the conversion

    switch(d)
    {
      case 1:
              pd_1.x = this->principal_directions_objects[i].centroid[0] + this->principal_directions_objects[i].dir1[0]*length;
              pd_1.y = this->principal_directions_objects[i].centroid[1] + this->principal_directions_objects[i].dir1[1]*length;
              pd_1.z = this->principal_directions_objects[i].centroid[2] + this->principal_directions_objects[i].dir1[2]*length;

              pd_2.x = this->principal_directions_objects[i].centroid[0];
              pd_2.y = this->principal_directions_objects[i].centroid[1];
              pd_2.z = this->principal_directions_objects[i].centroid[2];

              line_name = "prin_dir1_o";
              convert << i;
              line_name += convert.str();

              viewer->addLine<PointT>(pd_1,pd_2,1,0,0,line_name);
      case 2:
              pd_1.x = this->principal_directions_objects[i].centroid[0] - this->principal_directions_objects[i].dir1[0]*length;
              pd_1.y = this->principal_directions_objects[i].centroid[1] - this->principal_directions_objects[i].dir1[1]*length;
              pd_1.z = this->principal_directions_objects[i].centroid[2] - this->principal_directions_objects[i].dir1[2]*length;

              pd_2.x = this->principal_directions_objects[i].centroid[0];
              pd_2.y = this->principal_directions_objects[i].centroid[1];
              pd_2.z = this->principal_directions_objects[i].centroid[2];

              line_name = "prin_dir2_o";
              convert << i;
              line_name += convert.str();

              viewer->addLine<PointT>(pd_2,pd_1,0,1,0,line_name);
      case 3:

              pd_1.x = this->principal_directions_objects[i].centroid[0] + this->principal_directions_objects[i].dir3[0]*length;
              pd_1.y = this->principal_directions_objects[i].centroid[1] + this->principal_directions_objects[i].dir3[1]*length;
              pd_1.z = this->principal_directions_objects[i].centroid[2] + this->principal_directions_objects[i].dir3[2]*length;

              pd_2.x = this->principal_directions_objects[i].centroid[0];
              pd_2.y = this->principal_directions_objects[i].centroid[1];
              pd_2.z = this->principal_directions_objects[i].centroid[2];

              line_name = "prin_dir3_o";
              convert << i;
              line_name += convert.str();

              viewer->addLine<PointT>(pd_1,pd_2,0,0,1,line_name);
      case 4:
              pd_1.x = this->principal_directions_objects[i].centroid[0] + this->principal_directions_objects[i].dir4[0]*length;
              pd_1.y = this->principal_directions_objects[i].centroid[1] + this->principal_directions_objects[i].dir4[1]*length;
              pd_1.z = this->principal_directions_objects[i].centroid[2] + this->principal_directions_objects[i].dir4[2]*length;

              pd_2.x = this->principal_directions_objects[i].centroid[0];
              pd_2.y = this->principal_directions_objects[i].centroid[1];
              pd_2.z = this->principal_directions_objects[i].centroid[2];

              line_name = "prin_dir4_o";
              convert << i;
              line_name += convert.str();

              viewer->addLine<PointT>(pd_1,pd_2,1,1,1,line_name);
      default: break;
    }
  }
}

void CTableClearingPlanning::viewerAddPrincipalDirections(Visualizer viewer)
{
  for (int i = 0; i < n_objects; ++i)
  {
    this->viewerAddPrincipalDirections(viewer,i);
  }
}

void CTableClearingPlanning::cleanPrincipalDirections(Visualizer viewer, uint i)
{
  for (uint d = 1; d <= 4; ++d)
  { 
    PointT pd_1,pd_2;
    std::string line_name;
    std::ostringstream convert;   // stream used for the conversion

    switch(d)
    {
      case 1:
              line_name = "prin_dir1_o";
              convert << i;
              line_name += convert.str();

              viewer->removeShape(line_name);
      case 2:
              line_name = "prin_dir2_o";
              convert << i;
              line_name += convert.str();

              viewer->removeShape(line_name);
      case 3:
              line_name = "prin_dir3_o";
              convert << i;
              line_name += convert.str();

              viewer->removeShape(line_name);
      case 4:
              line_name = "prin_dir4_o";
              convert << i;
              line_name += convert.str();

              viewer->removeShape(line_name);
      default: break;
    }
  }
}

void CTableClearingPlanning::cleanPolygonalMesh(Visualizer viewer)
{
  viewer->removePolygonMesh("ee");

  for (int i = 0; i < 100; ++i) //high value -> ineficcient but fast to implement -> it is only for visualization
  {
    std::string pol_name;
    std::ostringstream convert;   // stream used for the conversion
    pol_name = "";
    convert << i;
    pol_name += convert.str();
    viewer->removePolygonMesh(pol_name);
  }
  
}

void CTableClearingPlanning::viewerAddPlaneNormal(Visualizer viewer)
{
  double length = 100;

  //get normal vector of the plane
  Eigen::Vector3f norm_plane;
  if(! this->plane_coefficients.values.size() > 0)
  {
    std::cout << "Plane model coefficients not set - Exiting from computePrincipalDirections()" << std::endl;
    return; 
  }
  norm_plane[0] = this->plane_coefficients.values[0];
  norm_plane[1] = this->plane_coefficients.values[1];
  norm_plane[2] = this->plane_coefficients.values[2];
  norm_plane.normalize();
  
  for (uint i = 0; i <   this->principal_directions_objects.size(); ++i)
  {
    PointT pd_1,pd_2;
    pd_1.x = this->principal_directions_objects[i].centroid[0] + norm_plane[0]*length;
    pd_1.y = this->principal_directions_objects[i].centroid[1] + norm_plane[1]*length;
    pd_1.z = this->principal_directions_objects[i].centroid[2] + norm_plane[2]*length;

    pd_2.x = this->principal_directions_objects[i].centroid[0] - norm_plane[0]*length;
    pd_2.y = this->principal_directions_objects[i].centroid[1] - norm_plane[1]*length;
    pd_2.z = this->principal_directions_objects[i].centroid[2] - norm_plane[2]*length;

    std::string line_name = "plane_normal";
    std::ostringstream convert;   // stream used for the conversion
    convert << i;
    line_name += convert.str();

    viewer->addLine<PointT>(pd_1,pd_2,line_name);
  }
}

void CTableClearingPlanning::viewerAddObjectsClouds(Visualizer viewer)
{
  for (uint i = 0; i < this->objects.size(); ++i)
  { 
    std::string cloud_name = "object";
    std::ostringstream convert;   // stream used for the conversion
    convert << i;
    cloud_name += convert.str();
    viewer->addPointCloud(this->objects[i].makeShared(),cloud_name);
  }
}

void CTableClearingPlanning::viewerAddRichObjectsClouds(Visualizer viewer)
{
  for (uint i = 0; i < this->rich_objects.size(); ++i)
  { 
    std::string cloud_name = "object";
    std::ostringstream convert;   // stream used for the conversion
    convert << i;
    cloud_name += convert.str();
    viewer->addPointCloud(this->rich_objects[i].makeShared(),cloud_name);
  }
}
void CTableClearingPlanning::viewerAddFullObjectsClouds(Visualizer viewer)
{
    for (uint i = 0; i < this->objects_full.size(); ++i)
  { 
    std::string cloud_name = "object_full";
    std::ostringstream convert;   // stream used for the conversion
    convert << i;
    cloud_name += convert.str();
    viewer->addPointCloud(this->objects_full[i].cloud.makeShared(),cloud_name);
  }
}

void CTableClearingPlanning::viewerAddObjectsLabel(Visualizer viewer)
{
  for (uint i = 0; i < this->principal_directions_objects.size(); ++i)
  {
    pcl::PointXYZ p;
    p.x = this->principal_directions_objects[i].centroid[0];
    p.y = this->principal_directions_objects[i].centroid[1];
    p.z = this->principal_directions_objects[i].centroid[2];
    std::string text = "o";
    std::ostringstream convert;   // stream used for the conversion
    convert << i;
    text += convert.str();
    viewer->addText3D<pcl::PointXYZ>(text,p,10,1,0,0);
  }
}

void CTableClearingPlanning::viewerAddObjectsTransfomed(Visualizer viewer)
{
  uint i = 1;
  PrincipalDirectionsProjected* pd = &(this->principal_directions_objects[i]);
  // we have to compute the transformation
  Eigen::Matrix4f transform;
  transform(0,0) = pd->dir1[0]; transform(0,1) = pd->dir1[1]; transform(0,2) = pd->dir1[2];
  transform(1,0) = pd->dir3[0]; transform(1,1) = pd->dir3[1]; transform(1,2) = pd->dir3[2];
  transform(2,0) = this->plane_normal[0]; transform(2,1) = this->plane_normal[1]; transform(2,2) = this->plane_normal[2];

  //translation
  transform(0,3) = - pd->centroid[0]; 
  transform(1,3) = - pd->centroid[1];
  transform(2,3) = - pd->centroid[2];

  transform(3,0) = 0;transform(3,1) = 0;transform(3,2) = 0;transform(3,3) = 1;

  PointCloudT tmp_cloud;
  pcl::transformPointCloud<PointT>(this->objects[i], tmp_cloud, transform);

  PointT min_pt,max_pt;
  pcl::getMinMax3D(tmp_cloud,min_pt,max_pt);

  viewer->addPointCloud(tmp_cloud.makeShared(),"transformed_cloud");
}

void CTableClearingPlanning::viewerAddGraspingPose(Visualizer viewer,uint idx)
{
  if(this->grasping_poses.size() == 0)
  {
    PCL_ERROR("Grasping Poses still not computed.");
    return;
  }

  std::string grasp_name;
  std::ostringstream convert;   // stream used for the conversion
  grasp_name = "grasp";
  convert << idx;
  grasp_name += convert.str();

  pcl::PointCloud<pcl::PointXYZ> cloud;
  Eigen::Quaternionf quat(this->grasping_poses[idx].rotation);
  // Eigen::Quaternionf quat = Eigen::Quaternionf::Identity ();
  pcl::transformPointCloud<pcl::PointXYZ>(this->fingers_model.cloud, cloud, this->grasping_poses[idx].translation , quat);

  viewer->addPolygonMesh<pcl::PointXYZ>(cloud.makeShared(), this->fingers_model.vertices, grasp_name );  
}
void CTableClearingPlanning::viewerAddGraspingPoses(Visualizer viewer)
{
  for (int i = 0; i < this->n_objects; ++i)
  {
    this->viewerAddGraspingPose(viewer,i);
  }
}
uint CTableClearingPlanning::getNumObjects()
{
  return this->n_objects;
}

std::vector<ObjectFull> 
CTableClearingPlanning::getFullObjects()
{
  return this->objects_full;
}

std::vector<AABB> CTableClearingPlanning::getAABBObjects()
{
  return this->aabb_objects;
}
std::vector<BlocksPredicate> CTableClearingPlanning::getBlockPredicates()
{
  return this->blocks_predicates;
}
std::vector<std::vector<uint> > CTableClearingPlanning::getOnTopPrediates()
{
  return this->on_top_predicates;
}
std::vector<std::vector<uint> > CTableClearingPlanning::getBlockGraspPredicates()
{
  return this->block_grasp_predicates;
}
std::vector<PrincipalDirectionsProjected> CTableClearingPlanning::getProjectedPrincipalDirections()
{
  return this->principal_directions_objects;
}
std::vector<GraspingPose> CTableClearingPlanning::getGraspingPoses()
{
  return this->grasping_poses;
}


void CTableClearingPlanning::viewerShowFingersModel(Visualizer viewer)
{
  viewer->addCoordinateSystem (0.3);
  viewer->addPointCloud(this->fingers_model.cloud.makeShared(),"fingers model");
  viewer->addPolygonMesh<pcl::PointXYZ>(this->fingers_model.cloud.makeShared(), this->fingers_model.vertices );  
}