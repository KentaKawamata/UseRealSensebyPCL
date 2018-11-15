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
#include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/LU>

// 1cm(0.01m)
float voxel = 0.02;

/*
 * 複数ある点群から最大値を持つ点と最小値を持つ点を抽出し,差分計算
 * zMax : 最大値を持つ点
 * zMin : 最小値を持つ点 
 */
inline float selectDiff(float cloudPoint, pcl::PointCloud<pcl::PointXYZRGB> samePoints){

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
inline float calcDiff(float z1, float z2){
    
    float diff = std::abs(z1-z2);
    
    if(diff <= voxel*1.1){
        return 0.0;
    } else {
        return diff;
    }
}

/*
 * すでに計算済みのxy座標に存在する点群を探す
 */ 
inline bool findIndex(int num, std::vector<int> index){

    auto sameNum = std::find(index.begin(), index.end(), num);
    if (sameNum != index.end()){
        return false;
    } else {
        return true;
    }
}

/*
 * 体積計算
 * voxel : voxel一辺の長さ(m)
 * volume = voxel * voxel * diff
 */ 
float calcVolume(std::vector<float> diffs) {

    float volume=0;

    for(auto diff : diffs){
        std::cout << diff*voxel*voxel << std::endl;
        volume += diff*voxel*voxel;
    }
    return volume;
}

int main(int argc, char *argv[]) {

    std::string filename(argv[1]);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr org_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile (filename, *cloud);

    std::vector<float> diffZ;
    // 一度計算に使用した点のインデックスjを格納していく
    std::vector<int> index;

    Eigen::Matrix4f R = Eigen::Matrix4f::Identity();
    
    double theta = std::stod(argv[2]) / 180 * M_PI;
    R (1,1) = cos(theta);
    R (1,2) = -1*sin(theta);
    R (2,1) = sin(theta);
    R (2,2) = cos(theta);

    //pcl::transformPointCloud(*org_cloud, *cloud, R);

    for(int i=0; i<cloud->points.size(); i++){

        if(findIndex(i, index) == false){
            continue;
        }

        float x = cloud->points[i].x;
        float z = cloud->points[i].z;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr samePoints (new pcl::PointCloud<pcl::PointXYZRGB>);
        for(int j=0; j<cloud->points.size(); j++){

            if( (cloud->points[j].x==x && cloud->points[j].z==z) && j!=i){
                samePoints->points.push_back(cloud->points[j]);
                index.push_back(j);
            }
        }

        if(samePoints->points.size()==1){
             diffZ.push_back(calcDiff(cloud->points[i].y, samePoints->points[0].y));
        }
        else if(samePoints->points.size()>=2){
            diffZ.push_back(selectDiff(cloud->points[i].y, *samePoints));
        } else {
            continue;
        }
    }

    float volume = calcVolume(diffZ);
    std::cout << "volume : " << volume << std::endl; 

    return 0;
}
