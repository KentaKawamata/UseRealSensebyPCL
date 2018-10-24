//
// Created by kawa on 10/21/18.
//
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>

int user_data;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {

    viewer.setBackgroundColor (0, 0, 0);
}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer) {

    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);

    //FIXME: possible race condition here:
    user_data++;
}

void keyboardEvent(const pcl::visualization::KeyboardEvent &event,void *nothing){

    if(event.getKeySym() == "space" && event.keyDown()){
        std::cout << "---------- PUSH SPACE !!!!! ----------" << std::endl;
    }
        //next_iteration = true;
}

int main(int argc, char *argv[]) {

    std::string filename(argv[1]);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile (filename, *cloud);

    /* Reminder: how transformation matrices work :
     *     |-------> This column is the translation
     * | 1 0 0 x |  \
     * | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
     * | 0 0 1 z |  /
     * | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)
     * METHOD #1: Using a Matrix4f
     * This is the "manual" method, perfect to understand but error prone !
     *
     * https://github.com/PointCloudLibrary/pcl/blob/master/doc/tutorials/content/sources/matrix_transform/matrix_transform.cpp
     */
    Eigen::Matrix4f R = Eigen::Matrix4f::Identity();

    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    double theta = -45;
    R (1,1) = cos(theta);
    R (1,2) = -1*sin(theta);
    R (2,1) = sin(theta);
    R (2,2) = cos(theta);

    pcl::transformPointCloud(*cloud, *transed_cloud, R);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_test (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer_test->setBackgroundColor(0, 0, 0);
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(tra);
    viewer_test->addPointCloud<pcl::PointXYZRGB>(transed_cloud, "sample cloud");
    viewer_test->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer_test->addCoordinateSystem(1.0);
    viewer_test->initCameraParameters();
    viewer_test->registerKeyboardCallback(&keyboardEvent, (void*)NULL);

    /*
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    //blocks until the cloud is actually rendered
    viewer.showCloud(transed_cloud);

    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer

    //This will only get called once
    viewer.runOnVisualizationThreadOnce (viewerOneOff);

    //This will get called once per visualization iteration
    viewer.runOnVisualizationThread (viewerPsycho);

    viewer.registerKeyboardCallback(&keyboardEvent, (void*)NULL);
    */

    while (!viewer_test->wasStopped()) {

        viewer_test->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));

        //you can also do cool processing here
        //FIXME: Note that this is running in a separate thread from viewerPsycho
        //and you should guard against race conditions yourself...
        //user_data++;
    }
    return 0;
}
