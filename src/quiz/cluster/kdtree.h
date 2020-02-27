/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}


	void insertHelper(Node** node, uint depth, std::vector<float> point, int id){
		if(*node == NULL)
			*node = new Node(point, id);
		else {
			if(point[depth%2] < (*node)->point[depth%2])
				insertHelper(&((*node)->left), depth+1, point, id);
			else
				insertHelper(&((*node)->right), depth+1, point, id);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);

	}


	
	bool checkWithinBox(std::vector<float> p, std::vector<float> q, float d){
		return fabs(p[0]-q[0]) <= d && fabs(p[1]-q[1]) <= d;
	}
	bool checkWithinCircle(std::vector<float> p, std::vector<float> q, float d){
		return sqrt((p[0]-q[0])*(p[0]-q[0]) + (p[1]-q[1])*(p[1]-q[1])) <= d;
	}

	void searchHelper(std::vector<float> target, Node* node, uint depth, float distanceTol, std::vector<int>* ids){

		if(node == NULL) return;

		if(checkWithinBox(target, node->point, distanceTol) && checkWithinCircle(target, node->point, distanceTol))
			ids->push_back(node->id);
		
		if(target[depth%2] - distanceTol < node->point[depth%2])
			searchHelper(target, node->left, depth+1, distanceTol, ids);
		if(target[depth%2] + distanceTol > node->point[depth%2])
			searchHelper(target, node->right, depth+1, distanceTol, ids);

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, &ids);
		return ids;
	}





	

};




