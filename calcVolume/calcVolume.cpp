//
// Created by kawa on 10/10/18.
//
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/impl/vtk_lib_io.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/cloud_viewer.h>


float Volume(Vector p1, Vector p2, Vector p3) {

    v321 = p3.X*p2.Y*p1.Z;
    v231 = p2.X*p3.Y*p1.Z;
    var v312 = p3.X*p1.Y*p2.Z;
    var v132 = p1.X*p3.Y*p2.Z;
    var v213 = p2.X*p1.Y*p3.Z;
    var v123 = p1.X*p2.Y*p3.Z;

    return (1.0f/6.0f)*(-v321 + v231 + v312 - v132 - v213 + v123);
}

int main(int argc, char *argv[]) {

    //https://stackoverflow.com/questions/20958844/mesh-visualizing-pcl-1-6-using-pclvisualizer
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileVTK("", mesh);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPolygonMesh(mesh,"meshes",0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}