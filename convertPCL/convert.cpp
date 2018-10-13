//
// Created by kawa on 9/30/18.
//
#include "convert.hpp"

ConvertPCL::ConvertPCL(){
    startSenser();
}

ConvertPCL::~ConvertPCL(){
    pipeline.stop();
}

void ConvertPCL::startSenser(){

    // デバイスを見つける
    rs2::context ctx;
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() == 0)
        std::cout << "No device detected. Is it plugged in?" << std::endl;
    rs2::device dev = list.front();

    // Set Device Config
    rs2::config config;
    config.enable_stream(rs2_stream::RS2_STREAM_COLOR, color_width_, color_height_, rs2_format::RS2_FORMAT_BGR8, fps_);
    config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, depth_width_, depth_height_, rs2_format::RS2_FORMAT_Z16, fps_);

    pipeline_profile = pipeline.start(config);
}

void ConvertPCL::updateFrame() {
    // センサーからframeを得る
    frameset = pipeline.wait_for_frames();

    color_frame = frameset.get_color_frame();
    depth_frame = frameset.get_depth_frame().apply_filter(color_map);

    // Retrive Frame Size
    color_width_ = color_frame.as<rs2::video_frame>().get_width();
    color_height_ = color_frame.as<rs2::video_frame>().get_height();
    depth_width_ = depth_frame.as<rs2::video_frame>().get_width();
    depth_height_ = depth_frame.as<rs2::video_frame>().get_height();
}

void ConvertPCL::draw() {

    color_mat_ = cv::Mat(color_height_, color_width_, CV_8UC3, const_cast<void *>(color_frame.get_data()));
    depth_mat_ = cv::Mat(depth_height_, depth_width_, CV_8UC3, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
}

void ConvertPCL::show() {

    cv::imshow("color_test", color_mat_);
    cv::imshow("depth_test", depth_mat_);
}

void ConvertPCL::pointToPCL() {

    auto sp = points.get_profile().as<rs2::video_stream_profile>();

    cloudXYZ.width = uint32_t(sp.width());
    cloudXYZ.height = uint32_t(sp.height());
    cloudXYZ.is_dense = false;
    cloudXYZ.points.resize(points.size());

    auto ptr = points.get_vertices();
    for (auto& p : cloudXYZ.points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }
}

void ConvertPCL::setAligned_frames() {

    // color_frameとdepth_frameの辻褄合わせ
    rs2::align align(rs2_stream::RS2_STREAM_COLOR);

    aligned_frames = align.process(frameset);
    if( !aligned_frames.size() ){
        std::cout << "NO" << std::endl;
        return;
    }
}

void ConvertPCL::setCloud() {

    // Fill in the cloud data
    cloud.width = uint32_t(width_);
    cloud.height = uint32_t(height_);
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);
}

void ConvertPCL::writePCDfile() {

    setAligned_frames();

    color_frame = aligned_frames.get_color_frame();
    depth_frame = aligned_frames.get_depth_frame();
    points = pc.calculate(depth_frame);
    auto colorArray = static_cast<uint8_t *>(const_cast<void *>(color_frame.get_data()));

    pointToPCL();
    setCloud();

    int j=0;
    for(size_t i=0; i < cloud.points.size(); ++i) {
        cloud.points[i].x = cloudXYZ.points[i].x;
        cloud.points[i].y = cloudXYZ.points[i].y;
        cloud.points[i].z = cloudXYZ.points[i].z;

        uint8_t b = colorArray[j];
        uint8_t g = colorArray[j+1];;
        uint8_t r = colorArray[j+2];
        int32_t rgb = (r << 16) | (g << 8) | b;
        cloud.points[i].rgb = *(float *)(&rgb);

        j+=3;
    }

    // Create the filtering object
i   pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 10.0);
    pass.setFilterLimitsNegative(true);
    pass.filter(*cloud);

    pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;

}

void ConvertPCL::run(){

    while(true){
        updateFrame();
        draw();
        show();

        const int key = cv::waitKey(20);
        if(key=='s'){
            writePCDfile();
        }
        else if(key==27){
            cv::destroyAllWindows();
            break;
        }
    }
}
