#include <iostream>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
int main(int argc,char* argv[]) {
	std::string filename = argv[1];
    pcl::PolygonMesh::Ptr mesh( new pcl::PolygonMesh() );
    pcl::PointCloud<pcl::PointXYZ>::Ptr obj_pnt( new pcl::PointCloud<pcl::PointXYZ>() );
   
    if ( pcl::io::loadPolygonFileOBJ( filename, *mesh ) != -1 ){
        pcl::fromPCLPointCloud2( mesh->cloud, *obj_pnt );
    }
    
    float vol = 0.0;
    for(auto &tri:mesh->polygons){
        pcl::PointXYZ p1,p2,p3;
        float h = 0.0;
        float m = 0.0;

        p1 = obj_pnt->points[tri.vertices[0]];
        p2 = obj_pnt->points[tri.vertices[1]];
        p3 = obj_pnt->points[tri.vertices[2]];
        h = ((p1.z + p2.z + p3.z)/3);
        m = abs((p1.x - p3.x)*(p2.y-p3.y) - (p2.x - p3.x)*(p1.y - p3.y))/2;
        vol +=m*h;
    }
    std::cout << vol <<"m^3"<< std::endl;   
    return 0;
}
