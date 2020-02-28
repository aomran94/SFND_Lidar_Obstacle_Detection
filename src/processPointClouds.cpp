// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>
#include "quiz/cluster/kdtree.h"
//#include "quiz/ransac/ransac2d.cpp"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
    
    for(int i : inliers->indices)
        planeCloud->points.push_back(cloud->points[i]);

    for(int i=0; i<cloud->points.size(); i++)
        if(std::count(inliers->indices.begin(), inliers->indices.end(), i) == 0)
            obstCloud->points.push_back(cloud->points[i]);
    /*
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);
    */

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    // TODO:: Fill in this function to find inliers for the cloud.
    /* /////////////////////
    This is plane segmentation using PCL
    ///////////////////
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)
        std::cout << "Could not estimate a planar model !!" << std::endl;
    /////////////////// */
    
    std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations 
	// std::cout << "Points Count is " << cloud->points.size() << std::endl;
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

		// std::cout << "A= " << A << ", B= " << B << ", C= " << C << ", D= " << D;

		for(int index = 0; index < cloud->points.size(); index++){
			if(index == subsampleIndices[0] || index == subsampleIndices[1] || index == subsampleIndices[2]) continue;

			pcl::PointXYZ point = cloud->points[index];
			float Dist = fabs(A*point.x + B*point.y + C*point.z + D) / sqrt(A*A + B*B + C*C);
			if(Dist <= distanceThreshold)
				inliers.insert(index);
			
		}
		// std::cout << " and inliers count is " << inliers.size() << ". " << std::endl;

		if(inliersResult.size() < inliers.size())
			inliersResult = inliers;
		inliers.clear();

	}

    pcl::PointIndices::Ptr inlierIndices {new pcl::PointIndices};
    for (int index : inliersResult)
        inlierIndices->indices.push_back(index);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inlierIndices, cloud);
    return segResult;
}

inline void proximity(std::vector<int>* cluster, int index, const std::vector<std::vector<float>> points, KdTree3D* tree, float distanceTol, std::vector<bool>* visited){
	if(visited->at(index)) return;
	std::vector<int> prox = tree->search(points[index], distanceTol);
	visited->at(index) = true;
	for(int i : prox)
		if(!visited->at(i)) {
			cluster->push_back(i);
			proximity(cluster, i, points, tree, distanceTol, visited);
		}
}

inline bool lessX(const std::vector<float>& a, const std::vector<float>& b){
    return a[0] <= b[0];
}
inline bool lessY(const std::vector<float>& a, const std::vector<float>& b){
    return a[1] <= b[1];
}
inline bool lessZ(const std::vector<float>& a, const std::vector<float>& b){
    return a[2] <= b[2];
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    
    std::vector<std::vector<float>> points;
    for(int i=0; i<cloud->points.size(); i++){
        std::vector<float> tmpPoint;
        tmpPoint.push_back(cloud->points[i].x); tmpPoint.push_back(cloud->points[i].y); tmpPoint.push_back(cloud->points[i].z);
        points.push_back(tmpPoint);
    }

    KdTree3D* tree = new KdTree3D;
    std::vector<std::vector<float>> pointsCopy(points); 
    int Dim=0;
    while(!pointsCopy.empty()){
        std::nth_element(pointsCopy.begin(), pointsCopy.begin() + pointsCopy.size()/2, pointsCopy.end(), (Dim%3==0)?lessX:((Dim%3==1)?lessY:lessZ));
        std::vector<float> medianPoint = pointsCopy[pointsCopy.size()/2];
        int originalIndex = std::find(points.begin(), points.end(), medianPoint) - points.begin();
        tree->insert(medianPoint, originalIndex);
        pointsCopy.erase(pointsCopy.begin() + pointsCopy.size()/2);
        Dim++;
    }
    
	std::vector<std::vector<int>> clusters;
	std::vector<bool> visited(points.size(), false);
	for(int i=0; i<points.size(); i++){
		if(visited[i]) continue;
		std::vector<int> cluster;
		cluster.push_back(i);
		proximity(&cluster, i, points, tree, clusterTolerance, &visited);
		clusters.push_back(cluster);
	}
	
	
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterClouds;
    for (std::vector<int> cluster : clusters){
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud (new pcl::PointCloud<pcl::PointXYZ>());
        for(int point : cluster)
            clusterCloud->points.push_back(pcl::PointXYZ(points[point][0], points[point][1], points[point][2]));
        clusterClouds.push_back(clusterCloud);
    }

    return clusterClouds;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}