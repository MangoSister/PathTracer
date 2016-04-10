#include "bvh.h"

#include "CMU462/CMU462.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>
#include <queue>
#include <utility>

using namespace std;

namespace CMU462 { namespace StaticScene {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  this->primitives = _primitives;

  // TODO:
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.
	
  BBox bb_init;
  for (size_t i = 0; i < primitives.size(); ++i) {
    bb_init.expand(primitives[i]->get_bbox());
  }

  root = new BVHNode(bb_init, 0, primitives.size());
	
	const size_t bucket_size = 16;
	std::stack<BVHNode*> node_stack{};
	node_stack.push(root);
	while(!node_stack.empty())
	{
		BVHNode* curr = node_stack.top();
		node_stack.pop();
		
		BBox& bb_curr = curr->bb;
		double best_sah = std::numeric_limits<double>::max();
		std::pair<BBox, BBox> best_partition{};
		std::pair<std::vector<Primitive*>, std::vector<Primitive*>> best_pm{};
		
		for(size_t dim = 0; dim < 3; ++dim)
		{
			for(size_t bkt = 1; bkt < bucket_size; ++bkt)
			{
				double split_line = bb_curr.bounds[0][dim] +
				(static_cast<double>(bkt) /static_cast<double>(bucket_size)) * (bb_curr.bounds[1][dim] - bb_curr.bounds[0][dim]);
				
				BBox left{}, right{};
				std::vector<Primitive*> left_pm{}, right_pm{};

				for(size_t i = curr->start; i < curr->start + curr->range; i++)
				{
					Primitive* pm = primitives[i];
					if(pm->get_bbox().centroid()[dim] < split_line)
					{
						left.expand(pm->get_bbox());
						left_pm.push_back(pm);
					}
					else
					{
						right.expand(pm->get_bbox());
						right_pm.push_back(pm);
					}
				}
				
				//compute SAH
				double sah =
				left.surface_area() / bb_curr.surface_area() * static_cast<double>(left_pm.size()) +
				right.surface_area() / bb_curr.surface_area() * static_cast<double>(right_pm.size());
				
				if(sah < best_sah)
				{
					best_sah = sah;
					best_partition.first = std::move(left);
					best_partition.second = std::move(right);
					best_pm.first = std::move(left_pm);
					best_pm.second = std::move(right_pm);
				}
			}
			
		}
		
		//do partition
		//re-order the primitive (partially)
		primitives.erase(primitives.begin() + curr->start, primitives.begin() + curr->start + curr->range);
		primitives.insert(primitives.begin() + curr->start, best_pm.first.begin(), best_pm.first.end());
		primitives.insert(primitives.begin() + curr->start + best_pm.first.size(), best_pm.second.begin(), best_pm.second.end());
		
		//assign children node
		curr->l = new BVHNode(best_partition.first, curr->start, best_pm.first.size());
		curr->r = new BVHNode(best_partition.second, curr->start + best_pm.first.size(), best_pm.second.size());
		
		if(curr->l->range > max_leaf_size)
			node_stack.push(curr->l);
		
		if(curr->r->range > max_leaf_size)
			node_stack.push(curr->r);
	}
}

BVHAccel::~BVHAccel() {

  // TODO:
  // Implement a proper destructor for your BVH accelerator aggregate
	if(root == nullptr)
		return;
	
	std::stack<BVHNode*> node_stack{};
	BVHNode* curr = root;
	do
	{
		while(!curr->isLeaf())
		{
			node_stack.push(curr);
			if(curr->l != nullptr)
				curr = curr->l;
			else if(curr->r != nullptr)
				curr = curr->r;
		}
		
		BVHNode* parent = node_stack.top();
		if(curr == parent->l)
			parent->l = nullptr;
		else if(curr == parent->r)
			parent->r = nullptr;
		
		delete curr;
		curr = node_stack.top();
		node_stack.pop();
	}
	while(!node_stack.empty());
	
	root = nullptr;
}

BBox BVHAccel::get_bbox() const {
  return root->bb;
}

bool BVHAccel::intersect(const Ray &ray) const {

  // TODO:
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate.

	if(root == nullptr)
		return false;
	
	queue<BVHNode*> node_queue{};
	double t0{}, t1{};
	if(!root->bb.intersect(ray, t0, t1))
		return false;
	node_queue.push(root);
	while(!node_queue.empty())
	{
		BVHNode* curr_node = node_queue.front();
		node_queue.pop();
		
		if(curr_node->isLeaf())
		{
			for(size_t i = curr_node->start, end = curr_node->start + curr_node->range; i < end; ++i)
			{
				if(primitives[i]->intersect(ray))
				{
//					primitives[i]->intersect(ray);
					return true;
				}

			}
		}
		else
		{
			double l_t0{}, l_t1{}, r_t0, r_t1{};
			bool l_hit = curr_node->l->bb.intersect(ray, l_t0, l_t1);
			bool r_hit = curr_node->r->bb.intersect(ray, r_t0, r_t1);
			if(l_hit && r_hit)
			{
				node_queue.push(curr_node->l);
				node_queue.push(curr_node->r);
			}
			else if(l_hit)
				node_queue.push(curr_node->l);
			else if(r_hit)
				node_queue.push(curr_node->r);
		}
	}
	
	return false;
}

bool BVHAccel::intersect(const Ray &ray, Intersection *isect_closest) const {

  // TODO:
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate. When an intersection does happen.
  // You should store the non-aggregate primitive in the intersection data
  // and not the BVH aggregate itself.
	
	if(root == nullptr)
		return false;
	
	isect_closest->t = std::numeric_limits<double>::max();
	Intersection isect;
	
	queue<std::pair<BVHNode*, double>> node_queue{};
	double t0{}, t1{};
	if(!root->bb.intersect(ray, t0, t1))
		return false;
	node_queue.push(std::make_pair(root, t0));
	while(!node_queue.empty())
	{
		auto curr = node_queue.front();
		node_queue.pop();
		
		//#early exit
		if(curr.second > isect_closest->t)
			continue;
		
		BVHNode* curr_node = curr.first;
		if(curr_node->isLeaf())
		{
			for(size_t i = curr_node->start, end = curr_node->start + curr_node->range; i < end; ++i)
			{
				if(primitives[i]->intersect(ray, &isect))
				{
					if(isect.t < isect_closest->t)
						*isect_closest = isect;
				}
			}
		}
		else
		{
			double l_t0{}, l_t1{}, r_t0, r_t1{};
			bool l_hit = curr_node->l->bb.intersect(ray, l_t0, l_t1);
			bool r_hit = curr_node->r->bb.intersect(ray, r_t0, r_t1);
			if(l_hit && r_hit)
			{
				if(l_t0 < r_t0)
				{
					node_queue.push(std::make_pair(curr_node->l, l_t0));
					node_queue.push(std::make_pair(curr_node->r, r_t0));
				}
				else
				{
					node_queue.push(std::make_pair(curr_node->r, r_t0));
					node_queue.push(std::make_pair(curr_node->l, l_t0));
				}
			}
			else if(l_hit)
				node_queue.push(std::make_pair(curr_node->l, l_t0));
			else if(r_hit)
				node_queue.push(std::make_pair(curr_node->r, r_t0));
		}
	}
	
	return isect_closest->t < std::numeric_limits<double>::max();
}

}  // namespace StaticScene
}  // namespace CMU462
