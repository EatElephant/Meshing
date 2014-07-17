/****************************************************************************
Author:    Bailin Li
<<<<<<< HEAD
Brief:     Load a point cloud from a pcd file or text file
		   Smooth the pointcloud using MovingLeastSquare method
		   Creating triangular meshing from the point cloud
		   Visualize the meshing result
*****************************************************************************/

#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_io.h>
#include <iostream>

using namespace std;

int
main (int argc, char** argv)
{
  if(argc != 2)
  {
	  cout << "Usage:" << argv[0] << " [XYZ file name to load]" << "[mode 0 for read pcd file, 1 for read xyz file]" << endl;
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
  if(argv[2] == '1')
  {
	  double x, y, z;
	  while(ifile)
	  {

		  ifile >> x >> y >> z;
		  cloud->points.push_back(pcl::PointXYZ(x,y,z));
	  }
  }
  else if(argv[2] == '0')
  {
	
  }
	  
	  
  //* the data should be available in cloud

  //smooth pointcloud and get cloud with normal info
  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  
  //output cloud of mls method
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ>);
  
  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
 
  mls.setComputeNormals (false);

  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (1);

  // Smooth
  mls.process (*cloud_smoothed);
  
  //end of mls method
  //cloud_smoothed is the ouput smoothed cloud without normal info

  // Normal estimation
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
  tree2->setInputCloud (cloud_smoothed);
  n.setInputCloud (cloud_smoothed);
  n.setSearchMethod (tree2);
  n.setKSearch (20);
  n.compute (*normals);

  // Concatenate the XYZ and normal fields
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud_smoothed, *normals, *cloud_with_normals);
  // cloud_with_normals = cloud_smoothed + normals
  


  // Create search tree
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree3 (new pcl::search::KdTree<pcl::PointNormal>);
  tree3->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (10);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (150);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);
  gp3.setConsistentVertexOrdering(true);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree3);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  //Visualize Mesh result
  pcl::visualization::PCLVisualizer viewer("Mesh Visualizer");
  viewer.addPointCloud(cloud_smoothed, "Cloud");
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