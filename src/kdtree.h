#ifndef KDTREE_H
#define KDTREE_H
#include <iostream>
#include <vector>
template<typename PointT>
struct PCNode 
{
	PointT point;
	int id;
	PCNode<PointT>* right;
	PCNode<PointT>* left;
	PCNode<PointT>( PointT p, int setId)
	: id(setId), point(p), right(NULL), left(NULL)
	{}
};

template <typename PointT>
struct PCKDTree 
{
	PCNode<PointT>* root;

	PCKDTree<PointT>()
	:root(NULL)
	{}
	void insertHelper(PCNode<PointT>** root, uint depth, PointT point, int id) {
 		if(*root == NULL) 
 		{
 			*root = new PCNode<PointT>(point, id);
 		} else 
 		{
 			uint curr_dim = depth % 3;
 			if(curr_dim == 0)
 			{
 				if(point.x < (*root)->point.x)
 					insertHelper(&((*root)->left), depth+1, point, id);
 				else
 					insertHelper(&((*root)->right), depth+1, point, id);
 			} else if(curr_dim == 1) 
 			{
 				if(point.y < (*root)->point.y)
 					insertHelper(&((*root)->left), depth+1, point, id);
 				else
 					insertHelper(&((*root)->right), depth+1, point, id);
 			} else 
 			{
 				if(point.z < (*root)->point.z)
 					insertHelper(&((*root)->left), depth+1, point, id);
 				else
 					insertHelper(&((*root)->right), depth+1, point, id);
 			}
 		}
	}
	void insert(PointT point, int id) 
	{
		insertHelper(&root, 0, point, id);
	}

	void radiusSearchHelper(PCNode<PointT>* root, uint depth, PointT point, float clusterTolerance, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances)
	{
		if(root != NULL) {
			if(root->point.x >= (point.x - clusterTolerance) && root->point.x <= (point.x + clusterTolerance) && root->point.y >= (point.y - clusterTolerance) && root->point.y <= (point.y + clusterTolerance) && root->point.z >= (point.z - clusterTolerance) && root->point.z <= (point.z + clusterTolerance))
			{
				float dist = sqrt((point.x - root->point.x)*(point.x - root->point.x) + (point.y - root->point.y) * (point.y - root->point.y) + (point.z - root->point.z) * (point.z - root->point.z));
				if(dist <= clusterTolerance) {
					k_indices.push_back(root->id);
					k_sqr_distances.push_back(dist);
				}
			}
			uint currdim = depth % 3;
			float pointC, rootC;
			if(currdim == 0)
			{
				pointC = point.x;
				rootC = root->point.x;
			}
			else if(currdim == 1) 
			{
				pointC = point.y;
				rootC = root->point.y;	
			}
			else
			{
				pointC = point.z;
				rootC = root->point.z;
			}
			if((pointC - clusterTolerance) < rootC)
				radiusSearchHelper(root->left, depth+1, point, clusterTolerance, k_indices, k_sqr_distances);
			if((pointC + clusterTolerance) > rootC)
				radiusSearchHelper(root->right, depth+1, point, clusterTolerance, k_indices, k_sqr_distances);
		}
	}
	void radiusSearch(PointT current_point, float clusterTolerance, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances) 
	{
		radiusSearchHelper(root, 0, current_point, clusterTolerance, k_indices, k_sqr_distances);
	}
};

#endif