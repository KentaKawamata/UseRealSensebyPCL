//
// Created by kawa on 10/22/18.
//
#include <iostream>
#include <algorithm>
#include <vector>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

float calcDiff(pcl::PointCloud<pcl::PointXYZRGB> cloud, pcl::PointCloud<pcl::PointXYZRGB> samePoints, int num){

    float zMax=0;
    float zMin=10000;

    for(int i=0; i<samePoints.points.size(); i++){

        if(zMax < samePoints.points[i].z){
            zMax = samePoints.points[i].z;
        }
        if(zMin > samePoints.points[i].z){
            zMin = samePoints.points[i].z;
        }
    }

    if(zMax < cloud.points[num].z){
        zMax = cloud.points[num].z;
    }
    else if(zMin > cloud.points[num].z){
        zMin = cloud.points[num].z;
    }

    float zDiff = zMax - zMin;

    return zDiff;
}

bool findIndex(int num, std::vector<int> index){

    auto sameNum = std::find(index.begin(), index.end(), num);
    if (sameNum != index.end()){
        return false;
    } else {
        return true;
    }
}

int main(int argc, char *argv[]) {

    std::string filename(argv[1]);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile (filename, *cloud);

    std::vector<float> diffZ;
    std::vector<float> volume;
    std::vector<int> index;

    for(int i=0; i<cloud->points.size(); i++){

        if(findIndex(i, index) == false){
            continue;
        }

        float x = cloud->points[i].x;
        float y = cloud->points[i].y;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr samePoints (new pcl::PointCloud<pcl::PointXYZRGB>);
        int j=0;
        while(j<cloud->points.size()){

            if(cloud->points[j].x==x && cloud->points[j].y==y){
                samePoints->points.push_back(cloud->points[j]);
                index.push_back(j);
            }
            j++;
        }

        if(samePoints->points.size()==1){

            float diff = std::abs(cloud->points[i].z - samePoints->points[0].z);
            diffZ.push_back(diff);
        }
        else if(samePoints->points.size()>1){
            diffZ.push_back(calcDiff(*cloud, *samePoints, i));
        } else {
            continue;
        }
    }

    return 0;
}