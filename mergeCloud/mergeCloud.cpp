//
// Created by kawa on 10/21/18.
//

#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZRGB> merge_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2) {

    //２つの点群をまとめて格納する
    pcl::PointCloud<pcl::PointXYZRGB> cloud_merged;

    cloud_merged = *cloud1;
    cloud_merged += *cloud2;

    return cloud_merged;
}

int main(int argc, char *argv[]) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);


    pcl::io::loadPLYFile(argv[1], *cloud1);
    pcl::io::loadPLYFile(argv[2], *cloud2);

    pcl::io::savePLYFileBinary("merged.ply" , merge_cloud(cloud1, cloud2));

    return 0;
}