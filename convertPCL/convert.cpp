//
// Created by kawa on 9/30/18.
//
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <vector>
#include <cstdint>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << "----------------!!!!!!!!!!!!!!---------------" << std::endl;

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = uint32_t(sp.width());
    cloud->height = uint32_t(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

int main(int argc, char *argv[]) {

    cv::Mat color_mat;
    cv::Mat depth_mat;
    int height = 480;
    int width = 640;
    int fps = 30;

    rs2::pipeline pipeline;
    rs2::pipeline_profile pipeline_profile;

    rs2::frameset frameset;
    // ポイントクラウドとテクスチャマッピングを計算するためのポイントクラウドオブジェクトの宣言
    rs2::pointcloud pc;
    // ポイントオブジェクトを永続化して、フレームがドロップしたときに最後のクラウドを表示できるようにします
    rs2::points points;
    rs2::colorizer color_map;
    rs2::frame color_frame;
    rs2::frame depth_frame;
    //rs2::frameset aligned_frames;
    //rs2::frameset frameset_filtered;

    // デバイスを見つける
    rs2::context ctx;
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() == 0)
        std::cout << "No device detected. Is it plugged in?" << std::endl;
    rs2::device dev = list.front();

    // Set Device Config
    rs2::config config;
    config.enable_stream(rs2_stream::RS2_STREAM_COLOR, width, height, rs2_format::RS2_FORMAT_BGR8, fps);
    config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, width, height, rs2_format::RS2_FORMAT_Z16, fps);

    pipeline_profile = pipeline.start(config);

    //while(!viewer->wasStopped()) {
    while (true) {

        // センサーからframeを得る
        frameset = pipeline.wait_for_frames();

        color_frame = frameset.get_color_frame();
        depth_frame = frameset.get_depth_frame().apply_filter(color_map);

        // Retrive Frame Size
        width = color_frame.as<rs2::video_frame>().get_width();
        height = color_frame.as<rs2::video_frame>().get_height();

        color_mat = cv::Mat(height, width, CV_8UC3, const_cast<void *>(color_frame.get_data()));
        depth_mat = cv::Mat(height, width, CV_8UC3, (void *) depth_frame.get_data(), cv::Mat::AUTO_STEP);

        cv::imshow("color_test", color_mat);
        cv::imshow("depth_test", depth_mat);

        if (cv::waitKey(20) == 27) {
            cv::destroyAllWindows();
            break;
        }

    }
    frameset = pipeline.wait_for_frames();
    color_frame = frameset.get_color_frame();
    depth_frame = frameset.get_depth_frame().apply_filter(color_map);

    // rs2 :: alignにより、深度フレームと他のフレームとの位置合わせを実行することができます
    // color frameとdepth frameを合わせることで，色付きのpoint cloudを作成
    rs2::frameset aligned_frames;
    rs2::align align(rs2_stream::RS2_STREAM_COLOR);
    aligned_frames = align.process(frameset);
    if( !aligned_frames.size() ){
        std::cout << "NO" << std::endl;
        return -1;
    }

    color_frame = aligned_frames.get_color_frame();
    depth_frame = aligned_frames.get_depth_frame();
    points = pc.calculate(depth_frame);
    auto p_color_frame = static_cast<uint8_t *>(const_cast<void *>(color_frame.get_data()));

    auto pcl_points = points_to_pcl(points);
    std::cout << "----------------!!!!!!!!!!!!!!---------------" << std::endl;


    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    // Fill in the cloud data
    cloud.width = uint32_t(width);
    cloud.height = uint32_t(height);
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    int j = 0;
    for(size_t i=0; i < cloud.points.size(); ++i) {
        cloud.points[i].x = pcl_points->points[i].x;
        cloud.points[i].y = pcl_points->points[i].y;
        cloud.points[i].z = pcl_points->points[i].z;

        uint8_t b = p_color_frame[j];
        uint8_t g = p_color_frame[j+1];;
        uint8_t r = p_color_frame[j+2];
        int32_t rgb = (r << 16) | (g << 8) | b;
        cloud.points[i].rgb = *(float *)(&rgb);

        //cloud.points[i].b = p_color_frame[j];
        //cloud.points[i].g = p_color_frame[j + 1];
        //cloud.points[i].r = p_color_frame[j + 2];
        j = j + 3;
    }

    pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;

    //for(size_t i=0; i < cloud.points.size(); ++i) {
    for(auto p : cloud) {
        /*
        std::cerr << "    " << cloud.points[i].x << " "
                            << cloud.points[i].y << " "
                            << cloud.points[i].z << " "
                            << cloud.points[i].b << " "
                            << cloud.points[i].g << " "
                            << cloud.points[i].r << std::endl;
        */
        std::cerr << "    " << p.x << " "
                            << p.y << " "
                            << p.z << " "
                            << p.rgb << std::endl;
    }

    pipeline.stop();

    return 0;
}

