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


float calcVolume(pcl::PointXYZ  p1, pcl::PointXYZ p2, pcl::PointXYZ p3) {

    /*
     * http://chenlab.ece.cornell.edu/Publication/Cha/icip01_Cha.pdf
     */
    float x1 = p1._PointXYZ::data[0];
    float y1 = p1._PointXYZ::data[1];
    float z1 = p1._PointXYZ::data[2];

    float x2 = p2._PointXYZ::data[0];
    float y2 = p2._PointXYZ::data[1];
    float z2 = p2._PointXYZ::data[2];

    float x3 = p3._PointXYZ::data[0];
    float y3 = p3._PointXYZ::data[1];
    float z3 = p3._PointXYZ::data[2];

    float V321 = x3*y2*z1;
    float V231 = x2*y3*z1;
    float V312 = x3*y1*z2;
    float V132 = x1*y3*z2;
    float V213 = x2*y1*z3;
    float V123 = x1*y2*z3;

    float volume = (1.0f/6.0f)*(-V321 +V231 +V312 -V132 -V213 +V123);

    return volume;
}

int main(int argc, char *argv[]) {

    /*
     *
     * http://www.pcl-users.org/How-do-I-retrieve-the-vertices-in-a-PolygonMesh-td4045663.html
     *
     * 私はクラウドからPolygonMeshを生成しました.
     * メッシュ内のインデックスはクラウドのインデックスと一致しません.
     *
     * だから私はメッシュの "クラウド"メンバーのポイントにアクセスしようとしましたが,これはPointCloud2です.
     * このために私はfromPCLPointCloud2を使いましたが,普通のクラウドを得ました.
     * しかし、私にはわからない現象があります.
     *
     * - ポイントの頂点座標を取得する正しい方法は何ですか?
     * - メッシュ("polygon.vertices"配列でアクセスされる)?
     *
     * >私はクラウドからPolygonMeshを生成しました.
     * >メッシュ内のインデックスはクラウドのインデックスと一致しません.
     *
     *  これは驚くべきことではありません.生の点からサーフェスを推定しています
     *  元のポイントクラウドのすべての点を使用するメソッドを強制する必要はないと思います.
     *
     * >点の頂点座標を取得する正しい方法は何ですか？
     * > mesh("polygon.vertices"配列でアクセスされる)?
     * >ポリゴンはstd::vector<std::vector<uint32_t >>です.
     *
     *  ポリゴン[0] - 最初のポリゴンです
     *  ポリゴン[0][0] - 最初のポリゴンの最初の点です。
     *  これらのインデックスは、クラウド内のポイントのインデックスと一致します。
     *
     * 私は完全に理解しています。
     * 私の2番目の質問は、クラウド内のポイントにアクセスすることでした（私はインデックスを取得する方法を知っていました）。
     * 今私はfromPCLPointCloud2を使うことができ、
     * 正確なストレージレイアウトを知っている "データ"ベクトルのアドレス計算を実行できることを知っています。
     *
     */


    //https://stackoverflow.com/questions/20958844/mesh-visualizing-pcl-1-6-using-pclvisualizer
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileVTK("", mesh);

    /*
     * https://stackoverflow.com/questions/43719848/how-to-access-a-point-in-the-type-of-pclpointcloud2
     * conversion
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices( new pcl::PointCloud<pcl::PointXYZ> );

    /*
     * http://www.pcl-users.org/fromPCLPointCloud2-questions-td4039472.html
     * mesh構造の配列では,メッシュの頂点はPointCloud2に格納されている.
     * pcl::fromPCLPointCloud2()によって頂点のインデックスをPointCloud2配列からPointCloudにコピーする.
     * PointCloud2とPointCloudのインデックス構造は一致している.
     *
     */
    pcl::fromPCLPointCloud2(mesh.cloud, *vertices);

    float Volume=0;
    float VolumeAll=0;

    // access each vertex
    for(int i=0; i<vertices->size(); i+=3) {

        pcl::PointXYZ p1 = vertices->points[i];
        pcl::PointXYZ p2 = vertices->points[i + 1];
        pcl::PointXYZ p3 = vertices->points[i + 2];

        Volume = calcVolume(p1, p2, p3);
        VolumeAll += Volume;
    }


    //meshを表示
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