/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *  \author Raphael Favier
 * */
#include <iostream>
#include <cmath>

#include <pcl/io/auto_io.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/common/common.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>

#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/filter.h>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
#include <vtkCleanPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkLineSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>

class OctreeViewer
{
public:
  OctreeViewer (std::string &filename, double resolution) :
    viz ("Octree visualizator"),
    cloud (new pcl::PointCloud<pcl::PointXYZ>()),
    displayCloud (new pcl::PointCloud<pcl::PointXYZ>()),
    cloudVoxel (new pcl::PointCloud<pcl::PointXYZ>()),
    octree (resolution),
    wireframe (true),
    show_cubes_ (true),
    show_centroids_ (false),
    show_original_points_ (false),
    point_size_ (1.0)
  {

    //try to load the cloud
    if (!loadCloud(filename))
      return;

    //register keyboard callbacks
    viz.registerKeyboardCallback(&OctreeViewer::keyboardEventOccurred, *this, 0);

    //key legends
    viz.addText ("Keys:", 0, 170, 0.0, 1.0, 0.0, "keys_t");
    viz.addText ("a -> Increment displayed depth", 10, 155, 0.0, 1.0, 0.0, "key_a_t");
    viz.addText ("z -> Decrement displayed depth", 10, 140, 0.0, 1.0, 0.0, "key_z_t");
    viz.addText ("v -> Toggle octree cubes representation", 10, 125, 0.0, 1.0, 0.0, "key_v_t");
    viz.addText ("b -> Toggle centroid points representation", 10, 110, 0.0, 1.0, 0.0, "key_b_t");
    viz.addText ("n -> Toggle original point cloud representation", 10, 95, 0.0, 1.0, 0.0, "key_n_t");

    //set current level to half the maximum one
    displayedDepth = static_cast<int> (floor (octree.getTreeDepth() / 2.0));
    std::cout << "displayedDepth : " << displayedDepth << std::endl;
    if (displayedDepth == 0)
      displayedDepth = 1;

    // assign point cloud to octree
    octree.setInputCloud (cloud);

    // add points from cloud to octree
    octree.addPointsFromInputCloud();

    //show octree at default depth
    extractPointsAtLevel(displayedDepth);

    //reset camera
    viz.resetCameraViewpoint("cloud");
    viz.setBackgroundColor(0.5,0.5,0.5);

    //run main loop
    run();

  }

private:
  //========================================================
  // PRIVATE ATTRIBUTES
  //========================================================
  //visualizer
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz;

  pcl::visualization::PCLVisualizer viz;
  //original cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  //displayed_cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr displayCloud;
  // cloud which contains the voxel center
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVoxel;
  //octree
  pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree;
  //level
  int displayedDepth;
  //bool to decide what should be display
  bool wireframe;
  bool show_cubes_, show_centroids_, show_original_points_;
  float point_size_;

  double min_x;
  double min_y;
  double min_z;
  double max_x;
  double max_y;
  double max_z;
  //========================================================

  /* \brief Callback to interact with the keyboard
   *
   */
  void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void *){

      if (event.getKeySym () == "a" && event.keyDown ()){
          IncrementLevel ();
      } else if (event.getKeySym () == "z" && event.keyDown ()){
          DecrementLevel ();
      } else if (event.getKeySym () == "v" && event.keyDown ()){
          show_cubes_ = !show_cubes_;
          update ();
      } else if (event.getKeySym () == "b" && event.keyDown ()){
          show_centroids_ = !show_centroids_;
          update ();
      } else if (event.getKeySym () == "n" && event.keyDown ()){
          show_original_points_ = !show_original_points_;
          update ();
      } else if (event.getKeySym () == "w" && event.keyDown ()){
          if (!wireframe){
              wireframe = true;
          }
          update ();
      } else if (event.getKeySym () == "s" && event.keyDown ()){
          if (wireframe){
              wireframe = false;
          }
          update ();
      } else if ((event.getKeyCode () == '-') && event.keyDown ()){
          point_size_ = std::max(1.0f, point_size_ * (1 / 2.0f));
          update ();
      } else if ((event.getKeyCode () == '+') && event.keyDown ()){
          point_size_ *= 2.0f;
          update ();
       
      } else if((event.getKeySym() == "space") && event.keyDown ()){
          std::cout << "SAVE " << std::endl;
          //pcl::io::savePLYFileBinary("test.ply" , *cloudVoxel);
          pcl::io::savePLYFileBinary("test.ply" , *displayCloud);
          update();
      }
  }

  /* \brief Graphic loop for the viewer
   *
   */
  void run(){

    while (!viz.wasStopped()){
      //main loop of the visualizer
      viz.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  }

  /* \brief Helper function that read a pointcloud file (returns false if pbl)
   *  Also initialize the octree
   *
   */
  bool loadCloud(std::string &filename)
  {

    pcl::PointCloud<pcl::PointXYZ>::Ptr org_cloud (new pcl::PointCloud<pcl::PointXYZ>());    

    std::cout << "Loading file " << filename.c_str() << std::endl;
    //read cloud
    if (pcl::io::load (filename, *cloud))
    {
      return false;
    }

    /*
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(org_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 3.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter(*cloud);
    */
    Eigen::Matrix4f R = Eigen::Matrix4f::Identity();

    double theta = -44.5 / 180.0 * M_PI;
    R (1,1) = cos(theta);
    R (1,2) = -1*sin(theta);
    R (2,1) = sin(theta);
    R (2,2) = cos(theta);

    //std::cout << R << std::endl;

    //pcl::transformPointCloud(*cloud, *cloud, R);

    //remove NaN Points
    std::vector<int> nanIndexes;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndexes);
    std::cout << "Loaded " << cloud->points.size() << " points" << std::endl;

    //create octree structure
    octree.setInputCloud(cloud);
    //update bounding box automatically
    octree.defineBoundingBox();
    //add points in the tree
    octree.addPointsFromInputCloud();
    return true;
  }

  /* \brief Helper function that draw info for the user on the viewer
   *
   */
  void showLegend ()
  {
    char dataDisplay[256];
    sprintf (dataDisplay, "Displaying octree cubes: %s", (show_cubes_) ? ("True") : ("False"));
    viz.removeShape ("disp_octree_cubes");
    viz.addText (dataDisplay, 0, 75, 1.0, 0.0, 0.0, "disp_octree_cubes");

    sprintf (dataDisplay, "Displaying centroids voxel: %s", (show_centroids_) ? ("True") : ("False"));
    viz.removeShape ("disp_centroids_voxel");
    viz.addText (dataDisplay, 0, 60, 1.0, 0.0, 0.0, "disp_centroids_voxel");

    sprintf (dataDisplay, "Displaying original point cloud: %s", (show_original_points_) ? ("True") : ("False"));
    viz.removeShape ("disp_original_points");
    viz.addText (dataDisplay, 0, 45, 1.0, 0.0, 0.0, "disp_original_points");

    char level[256];
    sprintf (level, "Displayed depth is %d on %d", displayedDepth, octree.getTreeDepth());
    viz.removeShape ("level_t1");
    viz.addText (level, 0, 30, 1.0, 0.0, 0.0, "level_t1");

    viz.removeShape ("level_t2");
    sprintf (level, "Voxel size: %.4fm [%lu voxels]", std::sqrt (octree.getVoxelSquaredSideLen (displayedDepth)),
             cloudVoxel->points.size ());
    viz.addText (level, 0, 15, 1.0, 0.0, 0.0, "level_t2");
  }

  /* \brief Visual update. Create visualizations and add them to the viewer
   *
   */
  void update(){

    //remove existing shapes from visualizer
    clearView ();

    showLegend ();

    if (show_cubes_){
      //show octree as cubes
      /*
       * getVoxelSquaredSideLen
       * 単位ボクセル1辺の二乗(ボクセルの面)を返す
       * 1辺の長さが欲しいのでsqrtでルートをかけている
       */
      showCubes (std::sqrt(octree.getVoxelSquaredSideLen(displayedDepth)));
    }

    if (show_centroids_){
      //show centroid points
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler (cloudVoxel, "x");
      viz.addPointCloud (cloudVoxel, color_handler, "cloud_centroid");

      std::cout << "SAVE " << std::endl;
      pcl::io::savePLYFileBinary("test.ply" , *cloudVoxel);
      //pcl::io::savePLYFileBinary("test.ply" , *displayCloud);

      viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, "cloud_centroid");
    }

    if (show_original_points_){
      //show origin point cloud
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler (cloud, "z");
      viz.addPointCloud (cloud, color_handler, "cloud");
      viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, "cloud");
    }
  }

  /* \brief remove dynamic objects from the viewer
   *
   */
  void clearView(){

    //remove cubes if any
    vtkRenderer *renderer = viz.getRenderWindow()->GetRenderers()->GetFirstRenderer();
    while(renderer->GetActors()->GetNumberOfItems() > 0){
      renderer->RemoveActor(renderer->GetActors()->GetLastActor());
    }
    //remove point clouds if any
    viz.removePointCloud ("cloud");
    viz.removePointCloud ("cloud_centroid");
  }

  void getMaxMin(){
    
    max_x = 0;
    max_y = 0;
    max_z = 0;
    min_x = 0;
    min_y = 0;
    min_z = 0;

    for(auto point : cloudVoxel->points){
        if(max_z > point.z)
            max_z = max_z;
        else
            max_z = point.z;

        if(max_y > point.y)
            max_y = max_y;
        else
            max_y = point.y;

        if(max_x > point.x)
            max_x = max_x;
        else
            max_x = point.x;

        if(min_z < point.z)
            min_z = min_z;
        else
            min_z = point.z;

        if(min_y < point.y)
            min_y = min_y;
        else
            min_y = point.y;

        if(min_x < point.x)
            min_x = min_x;
        else
            min_x = point.x;
    }
    std::cout << "end min-max PointCloud" << std::endl;
  }

  /* \brief display octree cubes via vtk-functions
   *
   */
  void showCubes(double voxelSideLen){

    vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New ();
    // Create every cubes to be displayed
    double s = voxelSideLen / 2.0;
    
    for (size_t i = 0; i < cloudVoxel->points.size(); i++){

      double x = cloudVoxel->points[i].x;
      double y = cloudVoxel->points[i].y;
      double z = cloudVoxel->points[i].z;

      vtkSmartPointer<vtkCubeSource> wk_cubeSource = vtkSmartPointer<vtkCubeSource>::New ();

      wk_cubeSource->SetBounds(x-s, x+s, y-s, y+s, z-s, z+s);
      wk_cubeSource->Update();

      /*
       *生成したvoxelを足していく
       */
      appendFilter->AddInput (wk_cubeSource->GetOutput ());
    }

    getMaxMin();

    for(double i=max_x+s; i>=min_x-s; i-=voxelSideLen){
      for(double j=max_y+s; j>=min_y-s; j-=voxelSideLen){
      
        vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
        double min = min_z-s;
        double max = max_z+s;
        double p0[3] = {i, j, min};
        double p1[3] = {i, j, max};

        lineSource->SetPoint1(p0);
        lineSource->SetPoint2(p1);
        lineSource->Update();
      
        appendFilter->AddInput (lineSource->GetOutput ());
      }
    }

    for(double i=max_x+s; i>=min_x-s; i-=voxelSideLen){
      for(double j=max_z+s; j>=min_z-s; j-=voxelSideLen){
      
        vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
        double min = min_y-s;
        double max = max_y+s;
        double p0[3] = {i, max, j};
        double p1[3] = {i, min, j};

        lineSource->SetPoint1(p0);
        lineSource->SetPoint2(p1);
        lineSource->Update();
        appendFilter->AddInput (lineSource->GetOutput ());
      }
    }

    for(double i=max_z+s; i>=min_z-s; i-=voxelSideLen){
      for(double j=max_y+s; j>=min_y-s; j-=voxelSideLen){
      
        vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
        double min = min_x-s;
        double max = max_x+s;
        double p0[3] = {max, j, i};
        double p1[3] = {min, j, i};

        lineSource->SetPoint1(p0);
        lineSource->SetPoint2(p1);
        lineSource->Update();
        appendFilter->AddInput (lineSource->GetOutput ());
      }
    }

    // Remove any duplicate points
    vtkSmartPointer<vtkCleanPolyData> cleanFilter = vtkSmartPointer<vtkCleanPolyData>::New ();
    cleanFilter->SetInputConnection (appendFilter->GetOutputPort());
    cleanFilter->Update ();

    //Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> multiMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();

    multiMapper->SetInputConnection (cleanFilter->GetOutputPort ());

    vtkSmartPointer<vtkActor> multiActor = vtkSmartPointer<vtkActor>::New ();

    multiActor->SetMapper (multiMapper);

    multiActor->GetProperty ()->SetColor (1.0, 1.0, 1.0);
    multiActor->GetProperty ()->SetAmbient (1.0);
    multiActor->GetProperty ()->SetLineWidth (1);
    multiActor->GetProperty ()->EdgeVisibilityOn ();
    multiActor->GetProperty ()->SetOpacity (1.0);

    if (wireframe){
      multiActor->GetProperty ()->SetRepresentationToWireframe ();
    } else {
      multiActor->GetProperty ()->SetRepresentationToWireframe ();
      multiActor->GetProperty ()->SetRepresentationToSurface ();
    }

    // Add the actor to the scene
    viz.getRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(multiActor);

    // Render and interact
    viz.getRenderWindow()->Render();
    //viz.addCoordinateSystem();
  }

  /* \brief Extracts all the points of depth = level from the octree
   *
   */
  void extractPointsAtLevel(int depth)
  {
    displayCloud->points.clear();
    cloudVoxel->points.clear();

    pcl::PointXYZ pt_voxel_center;
    pcl::PointXYZ pt_centroid;
    std::cout << "===== Extracting data at depth " << depth << "... " << std::endl;
    double start = pcl::getTime ();

    for (pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::FixedDepthIterator tree_it = octree.fixed_depth_begin (depth);
         tree_it != octree.fixed_depth_end ();
         ++tree_it)
    {
      // Compute the point at the center of the voxel which represents the current OctreeNode
      Eigen::Vector3f voxel_min, voxel_max;
      octree.getVoxelBounds (tree_it, voxel_min, voxel_max);

      pt_voxel_center.x = (voxel_min.x () + voxel_max.x ()) / 2.0f;
      pt_voxel_center.y = (voxel_min.y () + voxel_max.y ()) / 2.0f;
      pt_voxel_center.z = (voxel_min.z () + voxel_max.z ()) / 2.0f;
      cloudVoxel->points.push_back(pt_voxel_center);

      // If the asked depth is the depth of the octree, retrieve the centroid at this LeafNode
      if (octree.getTreeDepth () == (unsigned int) depth)
      {
        pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::LeafNode* container = static_cast<pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::LeafNode*> (tree_it.getCurrentOctreeNode ());

        container->getContainer ().getCentroid (pt_centroid);
      }
      // Else, compute the centroid of the LeafNode under the current BranchNode
      else
      {
        // Retrieve every centroid under the current BranchNode
        pcl::octree::OctreeKey dummy_key;
        pcl::PointCloud<pcl::PointXYZ>::VectorType voxelCentroids;
        octree.getVoxelCentroidsRecursive (static_cast<pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::BranchNode*> (*tree_it), dummy_key, voxelCentroids);

        // Iterate over the leafs to compute the centroid of all of them
        pcl::CentroidPoint<pcl::PointXYZ> centroid;
        for (size_t j = 0; j < voxelCentroids.size (); ++j)
        {
          centroid.add (voxelCentroids[j]);
        }
        centroid.get (pt_centroid);
      }

      displayCloud->points.push_back (pt_centroid);
    }

    Eigen::Vector3f voxel_min, voxel_max;
    octree.getVoxelBounds (octree.fixed_depth_begin(depth), voxel_min, voxel_max);
    
    std::cout << "reinoare : " << voxel_max.x()-voxel_min.x() << std::endl;

    double end = pcl::getTime ();
    printf("%lu pts, %.4gs. %.4gs./pt. =====\n", displayCloud->points.size (), end - start,
           (end - start) / static_cast<double> (displayCloud->points.size ()));

    update();
  }

  /* \brief Helper function to increase the octree display level by one
   *
   */
  bool IncrementLevel()
  {
    if (displayedDepth < static_cast<int> (octree.getTreeDepth ()))
    {
      displayedDepth++;
      extractPointsAtLevel(displayedDepth);
      return true;
    }
    else
      return false;
  }

  /* \brief Helper function to decrease the octree display level by one
   *
   */
  bool DecrementLevel()
  {
    if (displayedDepth > 0)
    {
      displayedDepth--;
      extractPointsAtLevel(displayedDepth);
      return true;
    }
    return false;
  }

};

int main(int argc, char ** argv)
{
  if (argc != 3)
  {
    std::cerr << "ERROR: Syntax is octreeVisu <pcd file> <resolution>" << std::endl;
    std::cerr << "EXAMPLE: ./octreeVisu bun0.pcd 0.001" << std::endl;
    return -1;
  }

  std::string cloud_path(argv[1]);
  OctreeViewer v(cloud_path, atof(argv[2]));
}
