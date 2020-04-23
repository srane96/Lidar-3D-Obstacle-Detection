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
	void insertHelper(Node **root, uint depth, std::vector<float> point, int id) {
		if(*root == NULL)
			*root = new Node(point, id);
		else {
			uint cd = depth % 2;
			if(point[cd] < ((*root)->point[cd])) {
				insertHelper(&((*root)->left),depth+1, point, id);
			} else {
				insertHelper(&((*root)->right), depth+1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
	}

	void searchHelper(std::vector<float> target, Node* root, int depth, float distanceTol, std::vector<int>& ids) {
		if( root != NULL){
			if(root->point[0] >= (target[0]-distanceTol) && root->point[0] <= (target[0]+distanceTol) && root->point[1] >= (target[1]-distanceTol) && root->point[1] <= (target[1]+distanceTol)) {
				float distance = sqrt((target[0] - root->point[0])*(target[0] - root->point[0]) + (target[1] - root->point[1])*(target[1] - root->point[1]));
				if(distance <= distanceTol)
					ids.push_back(root->id);
			}
			if((target[depth%2] - distanceTol) < root->point[depth%2]) {
				searchHelper(target, root->left, depth+1, distanceTol, ids);
			}
			if((target[depth%2] + distanceTol) > root->point[depth%2]) {
				searchHelper(target, root->right, depth+1, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




