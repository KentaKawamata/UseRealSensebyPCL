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

/*
 * 複数ある点群から最大値を持つ点と最小値を持つ点を抽出し,差分計算
 * zMax : 最大値を持つ点
 * zMin : 最小値を持つ点 
 */
float selectDiff(float cloudPoint, pcl::PointCloud<pcl::PointXYZRGB> samePoints){

    float zMax=0;
    float zMin=0;

    for(auto point : samePoints.points){

        if(zMax < point.z){
            zMax = point.z;
        }
        else if(zMin > point.z){
            zMin = point.z;
        }

        if(zMax < zMin){
            float tmp = zMax;
            zMax = zMin;
            zMin = tmp;
        }
    }

    if(zMax < cloudPoint){
        zMax = cloudPoint;
    }
    else if(zMin > cloudPoint){
        zMin = cloudPoint;
    }

    return std::abs(zMax - zMin);
}

/*
 * ２つの点群の差分を計算
 */
float calcDiff(float z1, float z2){
    return std::abs(z1-z2);
}

/*
 * すでに計算済みのxy座標に存在する点群を探す
 */ 
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
    // 一度計算に使用した点のインデックスjを格納していく
    std::vector<int> index;

    for(int i=0; i<cloud->points.size(); i++){

        if(findIndex(i, index) == false){
            continue;
        }

        float x = cloud->points[i].x;
        float y = cloud->points[i].y;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr samePoints (new pcl::PointCloud<pcl::PointXYZRGB>);
        for(int j=0; j<cloud->points.size(); j++){

            if( (cloud->points[j].x==x && cloud->points[j].y==y) && j!=i){
                samePoints->points.push_back(cloud->points[j]);
                index.push_back(j);
            }
        }

        if(samePoints->points.size()==1){
            diffZ.push_back(calcDiff(cloud->points[i].z, samePoints->points[0].z));
        }
        else if(samePoints->points.size()>=2){
            diffZ.push_back(selectDiff(cloud->points[i].z, *samePoints));
        } else {
            continue;
        }
    }

    return 0;
}