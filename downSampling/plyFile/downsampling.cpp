//
// Created by kawa on 10/7/18.
//
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

int main (int argc, char *argv[]) {

  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  std::string filename(argv[1]);

  // Fill in the cloud data
  pcl::PLYReader reader;
  // Replace the path below with the path where you saved your file
  reader.read (filename, *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.03f, 0.03f, 0.03f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

  pcl::PLYWriter writer;
  writer.write ("test_voxel.ply", cloud_filtered, \
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
  //writer.write<pcl::PointXYZRGB>("test_voxel.ply", *cloud_filtered, false);
  //pcl::io::savePLYFileBinary("test_voxel.ply", cloud_filtered);

  return (0);
}
