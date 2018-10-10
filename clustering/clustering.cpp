//
// Created by kawa on 10/8/18.
//
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>

int main(int argc, char** argv) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if( pcl::io::loadPCDFile <pcl::PointXYZ> ("./../downSampling/pcdFile/test_downsampled.pcd", *cloud) == -1) {

        std::cout << "Cloud reading failed." << std::endl;
        return -1;
    }

    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (50);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    /*
     * 注意 型について要確認!!!
     * 3.0/180.0*M_PI
     * reg.setSmoothnessThreshold(3.0/180.0*M_PI);
     */
    reg.setSmoothnessThreshold(3.0/180.0*M_PI);
    reg.setCurvatureThreshold (1.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);

    std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
    std::cout << "These are the indices of the points of the initial" <<
              std::endl << "cloud that belong to the first cluster:" << std::endl;

    int counter = 0;
    while(counter < clusters[0].indices.size()) {

        std::cout << clusters[0].indices[counter] << ", ";
        counter++;
        if(counter % 10 == 0) {
            std::cout << std::endl;
        }
    }
    std::cout << std::endl;

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr extracted_cloud;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    //pcl::visualization::CloudViewer viewer1("Cluster viewer1");
    //viewer1.showCloud(colored_cloud);
    //while(!viewer1.wasStopped ()) {
    //}

    std::cout << "Num of Points : " << colored_cloud->points.size() << std::endl;
    int i=0;
    int pointsNum = 1000;
    while(i<clusters.size()) {

        std::cout << "????????????????" << std::endl;
        if(clusters[i].indices.size() < pointsNum) {

            int num=0;
            while(num<clusters[i].indices.size()) {

                inliers->indices.push_back(clusters[i].indices[num]);
                //extracted_cloud->points.[clusters[i].indices[num] = ;
                num++;
            }
        }
        i++;
    }

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(colored_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*colored_cloud);

    pcl::visualization::CloudViewer viewer2("Cluster viewer");
    viewer2.showCloud(colored_cloud);
    while(!viewer2.wasStopped ()) {
    }

    return (0);
}

/*
int main(int argc, char *argv[]) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented (new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::io::loadPLYFile (".ply", *segmented);
    pcl::io::loadPCDFile ("./../downSampling/pcdFile/test_pcd.pcd", *segmented);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree <pcl::PointXYZRGB>);
    //treeにsegmentedの点群データを格納
    tree->setInputCloud(segmented);

    std::vector <pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction <pcl::PointXYZRGB> clustering;
    clustering.setClusterTolerance(0.03);
    clustering.setMinClusterSize(1000);
    clustering.setMaxClusterSize(300000);
    clustering.setSearchMethod(tree);
    clustering.setInputCloud(segmented);
    //<setInputCloud(), setIndices()>によって与えられたPointCloudにおけるクラスタの抽出を行う
    //cluster_indicesにクラスタリングしたポイントクラスタを与えている
    //http://docs.pointclouds.org/1.7.1/classpcl_1_1_euclidean_cluster_extraction.html#a41e0cd5e3f7967d59013c967c909585c
    clustering.extract(cluster_indices);

    //PCLの主な単位系...m(メートル)?
    int j=0;
    std::vector<pcl::PointIndices>::const_iterator it;
    pcl::PCDWriter writer;

    for(it=cluster_indices.begin(); it!=cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::vector<int>::const_iterator pit;

        for(pit=it->indices.begin(); pit!=it->indices.end(); ++pit) {
            cloud_cluster->points.push_back(segmented->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
        j++;
    }

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr result = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>() );
    //pcl::ExtractIndices <pcl::PointXYZRGB> extract;
    //extract.setInputCloud(segmented);
    //pcl::IndicesPtr indices(new pcl::PointIndices);
    //*indices = cluster_indices[0].indices;
    //extract.setIndices(indices);
    //extract.setNegative(false);
    //extract.filter(*result);

    pcl::io::savePCDFileASCII("testClustered.pcd", *result);
    std::cerr << " Saved to PLY file" << std::endl;

    return 0;
}
*/
