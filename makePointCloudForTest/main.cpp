#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <cmath>

bool signal = false;

void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *nothing) {

    if(event.getKeySym() == "space" && event.keyDown()){
        signal = true;
    }
}

int main(int argc, char *argv[]) {

    const int circle_num=50;//円周上に配置する点の数
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    
    /*
    cloud->width    = 0.5;
    cloud->height   = 0.5;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    */

    uint8_t r = 255, g = 255, b = 255;    // Example: Red color
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    

    for(float i=0; i<=0.5; i+=0.05){
        for(float j=0; j<0.5; j+=0.05){
            pcl::PointXYZRGB point1;
            pcl::PointXYZRGB point2;

            point1.rgb = *reinterpret_cast<float*>(&rgb);
            point2.rgb = *reinterpret_cast<float*>(&rgb);
            
            point1.x = point2.x = i;
            point1.y = point2.y = j;
            point1.z = 0;
            point2.z = 0.5;

            cloud->points.push_back(point1);
            cloud->points.push_back(point2);
        }
    }

    for(float i=0; i<=0.5; i+=0.05){
        for(float j=0; j<0.5; j+=0.05){
            pcl::PointXYZRGB point3;
            pcl::PointXYZRGB point4;

            point3.rgb = *reinterpret_cast<float*>(&rgb);
            point4.rgb = *reinterpret_cast<float*>(&rgb);
            
            point3.z = point4.z = i;
            point3.y = point4.y = j;
            point3.x = 0;
            point4.x = 0.5;

            cloud->points.push_back(point3);
            cloud->points.push_back(point4);
        }
    }


    for(float i=0; i<=0.5; i+=0.05){
        for(float j=0; j<0.5; j+=0.05){
            pcl::PointXYZRGB point5;
            pcl::PointXYZRGB point6;

            point5.rgb = *reinterpret_cast<float*>(&rgb);
            point6.rgb = *reinterpret_cast<float*>(&rgb);

            point5.z = point6.z = i;
            point5.x = point6.x = j;
            point5.y = 0;
            point6.y = 0.5;

            cloud->points.push_back(point5);
            cloud->points.push_back(point6);
        }
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_test (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer_test->setBackgroundColor(0, 0, 0);
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(tra);
    viewer_test->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");
    viewer_test->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer_test->addCoordinateSystem(1.0);
    viewer_test->initCameraParameters();
    viewer_test->registerKeyboardCallback(&keyboardEvent, (void*)NULL);

    while (!viewer_test->wasStopped()) {

        viewer_test->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));

        if(signal == true){
            pcl::io::savePLYFileASCII("test_cube.ply", *cloud);
            std::cout << "SAVE PLY FILE" << std::endl;
            signal = false;
        }
    }

    return 0;
}