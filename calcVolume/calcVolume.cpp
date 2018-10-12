//
// Created by kawa on 10/10/18.
//
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/impl/vtk_lib_io.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/Vertices.h>


float Volume(Vector p1, Vector p2, Vector p3) {

    v321 = p3.X*p2.Y*p1.Z;
    v231 = p2.X*p3.Y*p1.Z;
    var v312 = p3.X*p1.Y*p2.Z;
    var v132 = p1.X*p3.Y*p2.Z;
    var v213 = p2.X*p1.Y*p3.Z;
    var v123 = p1.X*p2.Y*p3.Z;

    return (1.0f/6.0f)*(-v321 + v231 + v312 - v132 - v213 + v123);
}

int main(int argc, char *argv[]) {

    /*
     * http://www.pcl-users.org/How-do-I-retrieve-the-vertices-in-a-PolygonMesh-td4045663.html
     *
     * 私はクラウドからPolygonMeshを生成し、
     * メッシュ内のポイントはクラウドのポイントに従いません
     * だから私はメッシュの "クラウド"メンバーのポイントにアクセスしようとしましたが、
     * これはPointCloud2です。このために私はfromPCLPointCloud2を使って
     * 普通の雲。しかし、私にはわからない現象があります
     * 正しいことをした。
     *
     * ポイントの頂点座標を取得する正しい方法は何ですか？
     * メッシュ（ "polygon.vertices"配列でアクセスされる）？
     * >私はクラウドからPolygonMeshを生成し、
     * >メッシュ内のポイントはクラウド内のポイントに従いません。
     * これは驚くべきことではありません。生の点からサーフェスを推定しています
     * データと私はすべてのメソッドを使用する必要があるとは思わない
     * 元のポイントクラウドのポイント。
     * >点の頂点座標を取得する正しい方法は何ですか？
     * > mesh（ "polygon.vertices"配列でアクセスされる）？
     * >ポリゴンはstd :: vector <std :: vector <uint32_t >>です。そう：
     * ポリゴン[0] - は最初のポリゴンです
     * ポリゴン[0][0] - 最初のポリゴンの最初の点です。
     * これらのインデックスは、クラウド内のポイントの順番と一致します。
     * 私は完全に理解しています。
     * 私の2番目の質問は、クラウド内のポイントにアクセスすることでした（私はインデックスを取得する方法を知っていました）。
     * 今私はfromPCLPointCloud2を使うことができ、
     * 正確なストレージレイアウトを知っている "データ"ベクトルのアドレス計算を実行できることを知っています。
     *
     */


    //https://stackoverflow.com/questions/20958844/mesh-visualizing-pcl-1-6-using-pclvisualizer
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileVTK("", mesh);

    //https://stackoverflow.com/questions/43719848/how-to-access-a-point-in-the-type-of-pclpointcloud2
    // conversion
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::fromPCLPointCloud2( mesh.cloud, *vertices );

    // access each vertex
    for( int idx = 0; idx < vertices->size(); idx++ ) {

        pcl::PointXYZ v = vertices->points[ idx ];

        float x = v._PointXYZ::data[ 0 ];
        float y = v._PointXYZ::data[ 1 ];
        float z = v._PointXYZ::data[ 2 ];
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPolygonMesh(mesh,"meshes",0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    while (!viewer->wasStopped()) {

        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}