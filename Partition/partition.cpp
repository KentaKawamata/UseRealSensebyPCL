//
// Created by kawa on 10/14/18.
//
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <vector>
#include <ctime>

/*
 * http://virtuemarket-lab.blogspot.com/2015/02/blog-post_75.html
 *
 */
pcl::PointCloud<pcl::PointXYZ> difference_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_test) {

    //cloud_baseは元となる点群
    //cloud_testは比較対象の点群
    //cloud_diffは比較の結果差分とされた点群

    //Octreeの解像度を指定
    double resolution = 0.00001;

    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);//Octreeを作成

    //元となる点群を入力
    octree.setInputCloud (cloud_base);
    octree.addPointsFromInputCloud ();

    //バッファの切り替え
    octree.switchBuffers ();

    //比較対象の点群を入力
    octree.setInputCloud (cloud_test);
    octree.addPointsFromInputCloud ();

    std::vector<int> newPointIdxVector;

    //比較の結果差分と判断された点郡の情報を保管
    octree.getPointIndicesFromNewVoxels(newPointIdxVector);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff (new pcl::PointCloud<pcl::PointXYZ> );


    //保管先のサイズの設定
    cloud_diff->width = cloud_base->points.size() + cloud_test->points.size();
    cloud_diff->height = 1;
    cloud_diff->points.resize (cloud_diff->width * cloud_diff->height);

    int n = 0;//差分点群の数を保存する
    for(size_t i = 0; i < newPointIdxVector.size (); i++) {

        cloud_diff->points[i].x = cloud_test->points[newPointIdxVector[i]].x;
        cloud_diff->points[i].y = cloud_test->points[newPointIdxVector[i]].y;
        cloud_diff->points[i].z = cloud_test->points[newPointIdxVector[i]].z;
        n++;
    }

    //差分点群のサイズの再設定
    cloud_diff->width = n;
    cloud_diff->height = 1;
    cloud_diff->points.resize (cloud_diff->width * cloud_diff->height);

    return *cloud_diff;
}

int main(int argc, char *argv[]) {

    srand ((unsigned int) time (NULL));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base (new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_test (new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_difference (new pcl::PointCloud<pcl::PointXYZ> );

    pcl::io::loadPCDFile(argv[1],*cloud_base);
    pcl::io::loadPCDFile(argv[2], *cloud_test);

    cloud_difference = difference_extraction(cloud_base, cloud_test);

    pcl::io::savePCDFileBinary("difference.pcd", cloud_difference);

    return 0;
}
