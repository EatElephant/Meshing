/****************************************************************************
Author:    Bailin Li
Brief:     Load a point cloud from a text file
		   Creating triangular meshing from the point cloud
		   Visualize the meshing result
*****************************************************************************/

#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_io.h>
#include <iostream>

using namespace std;

int
main (int argc, char** argv)
{
  if(argc != 2)
  {
	  cout << "Usage:" << argv[0] << " [PCD file name to load]" << endl;
	  return -1;
  }

  // Load input file into a PointCloud<T> with an appropriate type
  int ret;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ifstream ifile(argv[1]);
  if(!ifile.is_open())
  {
	  cout << "ERROR:fail to find point cloud file!!" << endl;
	  return -1;
  }

  //Create PointCloud based on the data in text file
  double x, y, z;
  string delim;
  while(ifile)
  {

	  ifile >> x >> y >> z;
	  cloud->points.push_back(pcl::PointXYZ(x,y,z));
  }
	  
  //* the data should be available in cloud

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (10);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (50);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  //Visualize Mesh result
  pcl::visualization::PCLVisualizer viewer("Mesh Visualizer");
  viewer.addPointCloud(cloud, "Cloud");
  viewer.addPolygonMesh(triangles,"Triangular Mesh");

  pcl::io::saveVTKFile ("mesh.vtk", triangles);

  while(!viewer.wasStopped())
  {
	viewer.spinOnce ();
	boost::this_thread::sleep (boost::posix_time::microseconds(100000));
  }

  // Finish
  return (0);
}