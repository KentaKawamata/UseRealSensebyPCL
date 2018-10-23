//
// Created by kawa on 10/22/18.
//
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>



int main(int argc, char *argv[]) {

    std::string filename(argv[1]);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile (filename, *cloud);

    std::vector<float> diffZ;
    std::vector<float> volume;

    for(int i=0; i<cloud->points.size(); i++){

        float x = cloud->points[i].x;
        float y = cloud->points[i].y;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr samePoints (new pcl::PointCloud<pcl::PointXYZRGB>);
        std::vector<int> index;

        for(int j=0; j<cloud->points.size(); j++){

            if(cloud->points[j].x==x && cloud->points[j].y==y){
                samePoints->points.push_back(cloud->points[j]);
                index.push_back(j);
            }
        }
        if(samePoints->points.size()==1){
            diffZ.push_back(cloud->points[i].z);
        }
        else if(samePoints->points.size()>1){

        } else {
            continue;
        }
    }

    return 0;
}