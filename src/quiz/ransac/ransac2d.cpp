/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

void print(std::vector<unsigned int> const &input)
{
	for (int i = 0; i < input.size(); i++) {
		std::cout << input.at(i) << ' ';
	}
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations 
	std::cout << "Points Count is " << cloud->points.size() << std::endl;
	while (maxIterations--){
		std::unordered_set<int> inliers;


		std::vector<unsigned int> subsampleIndices(cloud->points.size());
		std::iota(subsampleIndices.begin(), subsampleIndices.end(), 0);
		// print(subsampleIndices);
		std::random_shuffle(subsampleIndices.begin(), subsampleIndices.end());
		// print(subsampleIndices);
		pcl::PointXYZ point1 = cloud->points[subsampleIndices[0]];
		pcl::PointXYZ point2 = cloud->points[subsampleIndices[1]];
		std::cout << "Iteration " << maxIterations << " indices are " << subsampleIndices[0] << " and " << subsampleIndices[1] << "." << std::endl;
		
		inliers.insert(subsampleIndices[0]);
		inliers.insert(subsampleIndices[1]);

		float A = point1.y - point2.y + 0.0f;
		float B = point2.x - point1.x + 0.0f;
		float C = 1.0f * point1.x * point2.y - 1.0f * point2.x * point1.y;

		std::cout << "A = " << A << ", B = " << B << ", C = " << C << ". " << std::endl;

		for(int index = 0; index < cloud->points.size(); index++){
			if(index == subsampleIndices[0] || index == subsampleIndices[1]) continue;

			pcl::PointXYZ point = cloud->points[index];

			float Dist = fabs(A*point.x + B*point.y + C) / sqrt(A*A + B*B);

			// std::cout << "D = " << Dist << " ";

			if(Dist <= distanceTol)
				inliers.insert(index);
			
		}
		std::cout << "inliers count is " << inliers.size() << ". " << std::endl;

		if(inliersResult.size() < inliers.size())
			inliersResult = inliers;

		inliers.clear();

	}
	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations 
	std::cout << "Points Count is " << cloud->points.size() << std::endl;
	while (maxIterations--){
		std::unordered_set<int> inliers;

		std::vector<unsigned int> subsampleIndices(cloud->points.size());
		std::iota(subsampleIndices.begin(), subsampleIndices.end(), 0);
		std::random_shuffle(subsampleIndices.begin(), subsampleIndices.end());

		pcl::PointXYZ point1 = cloud->points[subsampleIndices[0]];
		pcl::PointXYZ point2 = cloud->points[subsampleIndices[1]];
		pcl::PointXYZ point3 = cloud->points[subsampleIndices[2]];
		// std::cout << "Iteration " << maxIterations << " indices are " << subsampleIndices[0] << " and " << subsampleIndices[1] << "." << std::endl;
		
		inliers.insert(subsampleIndices[0]);
		inliers.insert(subsampleIndices[1]);
		inliers.insert(subsampleIndices[2]);

		float A = (point2.y-point1.y)*(point3.z-point1.z)-(point2.z-point1.z)*(point3.y-point1.y);
		float B = (point2.z-point1.z)*(point3.x-point1.x)-(point2.x-point1.x)*(point3.z-point1.z);
		float C = (point2.x-point1.x)*(point3.y-point1.y)-(point2.y-point1.y)*(point3.x-point1.x);
		float D = -(A*point1.x + B*point1.y + C*point1.z);

		std::cout << "A= " << A << ", B= " << B << ", C= " << C << ", D= " << D;

		for(int index = 0; index < cloud->points.size(); index++){
			if(index == subsampleIndices[0] || index == subsampleIndices[1] || index == subsampleIndices[2]) continue;

			pcl::PointXYZ point = cloud->points[index];
			float Dist = fabs(A*point.x + B*point.y + C*point.z + D) / sqrt(A*A + B*B + C*C);
			if(Dist <= distanceTol)
				inliers.insert(index);
			
		}
		std::cout << " and inliers count is " << inliers.size() << ". " << std::endl;

		if(inliersResult.size() < inliers.size())
			inliersResult = inliers;
		inliers.clear();

	}
	return inliersResult;

}

int main ()
{
	
	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 20, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
