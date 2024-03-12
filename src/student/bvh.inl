
#include "../rays/bvh.h"
#include "debug.h"
#include <stack>
#include <iostream>

namespace PT {

// construct BVH hierarchy given a vector of prims
template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {

    // NOTE (PathTracer):
    // This BVH is parameterized on the type of the primitive it contains. This allows
    // us to build a BVH over any type that defines a certain interface. Specifically,
    // we use this to both build a BVH over triangles within each Tri_Mesh, and over
    // a variety of Objects (which might be Tri_Meshes, Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.

    // Keep these two lines of code in your solution. They clear the list of nodes and
    // initialize member variable 'primitives' as a vector of the scene prims
    nodes.clear();
    primitives = std::move(prims);

    // TODO (PathTracer): Task 3
    // Modify the code ahead to construct a BVH from the given vector of primitives and maximum leaf
    // size configuration.
    //
    // Please use the SAH as described in class.  We recomment the binned build from lecture.
    // In general, here is a rough sketch:
    //
    //  For each axis X,Y,Z:
    //     Try possible splits along axis, evaluate SAH for each
    //  Take minimum cost across all axes.
    //  Partition primitives into a left and right child group
    //  Compute left and right child bboxes
    //  Make the left and right child nodes.
    //
    //
    // While a BVH is conceptually a tree structure, the BVH class uses a single vector (nodes)
    // to store all the nodes. Therefore, BVH nodes don't contain pointers to child nodes,
    // but rather the indices of the
    // child nodes in this array. Hence, to get the child of a node, you have to
    // look up the child index in this vector (e.g. nodes[node.l]). Similarly,
    // to create a new node, don't allocate one yourself - use BVH::new_node, which
    // returns the index of a newly added node.
    //
    // As an example of how to make nodes, the starter code below builds a BVH with a
    // root node that encloses all the primitives and its two descendants at Level 2.
    // For now, the split is hardcoded such that the first primitive is put in the left
    // child of the root, and all the other primitives are in the right child.
    // There are no further descendants.

    // edge case
    if(primitives.empty()) {
        return;
    }

    // compute bounding box for all primitives
    BBox bb;
    for(size_t i = 0; i < primitives.size(); ++i) {
        bb.enclose(primitives[i].bbox());
    }

    // set up root node (root BVH). Notice that it contains all primitives.
    size_t root_node_addr = new_node();
    Node& root = nodes[root_node_addr];
    root.bbox = bb;
    root.start = 0;
    root.size = primitives.size();

   //std::cout << "root_size=" << root.size << std::endl;


    size_t curr_node_addr = root_node_addr;
    split_node(curr_node_addr, max_leaf_size);
    //std::cout << "nodes_size=" << nodes.size() << std::endl;
}

template<typename Primitive>
void BVH<Primitive>::split_node(size_t curr_node_addr, size_t max_leaf_size) {
    const int NUM_PARTITIONS = 10;

    if(nodes[curr_node_addr].size <= max_leaf_size) {
        //std::cout << curr_node_addr << " at leaf" << std::endl;
        //std::cout << nodes[curr_node_addr].size << " size at leaf" << std::endl;
        return;
    }

    // need to keep track of which primitives are in which buckets
    std::vector<size_t>* p_idx[3][NUM_PARTITIONS];

    // bucket in each axis
    BBox buckets[3][NUM_PARTITIONS];


    // initialize the bbox for each bucket
    for(size_t i = 0; i < NUM_PARTITIONS; i++) {
        BBox x_bbox = BBox();
        BBox y_bbox = BBox();
        BBox z_bbox = BBox();

        buckets[0][i] = x_bbox;
        buckets[1][i] = y_bbox;
        buckets[2][i] = z_bbox;

        p_idx[0][i] = new std::vector<size_t>();
        p_idx[1][i] = new std::vector<size_t>();
        p_idx[2][i] = new std::vector<size_t>();
    }

    // put primitives in current node into buckets
    Vec3 step_size = (nodes[curr_node_addr].bbox.max - nodes[curr_node_addr].bbox.min) * (1.0f / NUM_PARTITIONS);

    for(size_t i = nodes[curr_node_addr].start; i < nodes[curr_node_addr].start + nodes[curr_node_addr].size; i++) {
        Primitive& p = primitives[i];
        BBox pbb = p.bbox();
        Vec3 p_center_normalized = pbb.center() - nodes[curr_node_addr].bbox.min;
        Vec3 p_bucket_idx = p_center_normalized / step_size;

        //p_bucket_idx = clamp(p_bucket_idx, Vec3(0), Vec3(NUM_PARTITIONS - 1));

        // These should never print if primitives are arranged correctly
        if(p_bucket_idx.x < 0 || p_bucket_idx.x >= NUM_PARTITIONS) {
            std::cout << "bucket idx is out-of-bound!" << std::endl;
            std::cout << "pbb_center=" << pbb.center() << std::endl;
            std::cout << "bbox_min=" << nodes[curr_node_addr].bbox.min << std::endl;
        }
        if(p_bucket_idx.y < 0 || p_bucket_idx.y >= NUM_PARTITIONS) {
            std::cout << "bucket idx is out-of-bound!" << std::endl;
            std::cout << "pbb_center=" << pbb.center() << std::endl;
            std::cout << "bbox_min=" << nodes[curr_node_addr].bbox.min << std::endl;
        }
        if(p_bucket_idx.z < 0 || p_bucket_idx.z >= NUM_PARTITIONS) {
            std::cout << "bucket idx is out-of-bound!" << std::endl;
            std::cout << "pbb_center=" << pbb.center() << std::endl;
            std::cout << "bbox_min=" << nodes[curr_node_addr].bbox.min << std::endl;
        }

        int p_x_bucket_idx = floor(p_bucket_idx.x);
        int p_y_bucket_idx = floor(p_bucket_idx.y);
        int p_z_bucket_idx = floor(p_bucket_idx.z);

        p_idx[0][p_x_bucket_idx]->push_back(i);
        buckets[0][p_x_bucket_idx].enclose(pbb);

        p_idx[1][p_y_bucket_idx]->push_back(i);
        buckets[1][p_y_bucket_idx].enclose(pbb);

        p_idx[2][p_z_bucket_idx]->push_back(i);
        buckets[2][p_z_bucket_idx].enclose(pbb);
    }

    // use SAH to find the best partitioning for the current node
    float min_cost = FLT_MAX;
    size_t min_axis = 0;
    size_t min_i = 0;
    Vec3 left_min, left_max;
    Vec3 right_min, right_max;
    size_t min_p_cnt_left = 0;
    size_t min_p_cnt_right = 0;
    float curr_node_sa_inv = 1.0f / nodes[curr_node_addr].bbox.surface_area();
    for(size_t axis = 0; axis < 3; axis++) {         // for each axis
        for(size_t i = 1; i < NUM_PARTITIONS; i++) { // for each plane
            // compute SAH cost for a particular plane split along a particular axis
            BBox split_leftBox = BBox();
            BBox split_rightBox = BBox();
            size_t p_cnt_left = 0;
            size_t p_cnt_right = 0;
            for(size_t l = 0; l < i; l++) {
                split_leftBox.enclose(buckets[axis][l]);
                p_cnt_left += p_idx[axis][l]->size();
            }
            for(size_t r = i; r < NUM_PARTITIONS; r++) {
                split_rightBox.enclose(buckets[axis][r]);
                p_cnt_right += p_idx[axis][r]->size();
            }

            if(p_cnt_left + p_cnt_right != nodes[curr_node_addr].size) {
                std::cout << "i=" << i << std::endl;
                std::cout << "p_cnt_left=" << p_cnt_left << std::endl;
                std::cout << "p_cnt_right=" << p_cnt_right << std::endl;
                std::cout << "node_size=" << nodes[curr_node_addr].size << std::endl;
            }

            // don't consider partition plane not resulting in actual split
            //if(p_cnt_left == 0 || p_cnt_right == 0) continue;

            float P_left = split_leftBox.surface_area() * curr_node_sa_inv;
            float P_right = split_rightBox.surface_area() * curr_node_sa_inv;
            float c = P_left * (float)p_cnt_left + P_right * (float)p_cnt_right;


            if(c < min_cost) {
                min_cost = c;
                min_axis = axis;
                min_i = i;
                left_min = split_leftBox.min;
                left_max = split_leftBox.max;
                right_min = split_rightBox.min;
                right_max = split_rightBox.max;
                min_p_cnt_left = p_cnt_left;
                min_p_cnt_right = p_cnt_right;
            }
        }
    }

    // rearrange the primitives
    if(min_p_cnt_left + min_p_cnt_right != nodes[curr_node_addr].size) {
        std::cout << "primitives count per partition don't sum up to size of node!" << std::endl;
        std::cout << "min_p_cnt_left=" << min_p_cnt_left << std::endl;
        std::cout << "min_p_cnt_right=" << min_p_cnt_right << std::endl;
        std::cout << "node_size=" << nodes[curr_node_addr].size << std::endl;
    }

    std::vector<Primitive> temp_vec;


    for(size_t l = 0; l < min_i; l++) {
        std::vector<size_t>* bucket_p_vec = p_idx[min_axis][l];
        for(size_t j = 0; j < bucket_p_vec->size(); j++) {
            size_t prim_idx = bucket_p_vec->at(j);
            temp_vec.push_back(std::move(primitives[prim_idx]));
        }
    }

    int left_size = temp_vec.size();
    if(min_p_cnt_left != left_size) {
        std::cout << "primitives count on the left child does not equal min_p_cnt_left!"
                  << std::endl;
    }

    for(size_t r = min_i; r < NUM_PARTITIONS; r++) {
        std::vector<size_t>* bucket_p_vec = p_idx[min_axis][r];
        for(size_t j = 0; j < bucket_p_vec->size(); j++) {
            size_t prim_idx = bucket_p_vec->at(j);
            temp_vec.push_back(std::move(primitives[prim_idx]));
        }
    }

    int right_size = temp_vec.size() - left_size;
    if(min_p_cnt_right != right_size) {
        std::cout << "primitives count on the right child does not equal min_p_cnt_right!"
                  << std::endl;
    }

    if(temp_vec.size() != nodes[curr_node_addr].size) {
        std::cout << "temp_vec has more primitives than expected!"
                  << std::endl;
    }

    // clear the original section of the primitive vector
    primitives.erase(primitives.begin() + nodes[curr_node_addr].start,
                     primitives.begin() + nodes[curr_node_addr].start + nodes[curr_node_addr].size);

    // copy the rearranged primitives into the original vector
    primitives.insert(primitives.begin() + nodes[curr_node_addr].start,
                      std::make_move_iterator(temp_vec.begin()),
                      std::make_move_iterator(temp_vec.end())); 


    BBox min_split_leftBox = BBox(left_min, left_max);
    BBox min_split_rightBox = BBox(right_min, right_max);

    size_t startl = nodes[curr_node_addr].start;         // starting prim index of left child
    size_t rangel = min_p_cnt_left;                      // number of prims in left child
    size_t startr = startl + rangel;                     // starting prim index of right child
    size_t ranger = min_p_cnt_right;                     // number of prims in right child

    if(min_p_cnt_left == 0 || min_p_cnt_right == 0) {
        return;
    } else {
        // create child nodes
        size_t node_addr_l = new_node();
        size_t node_addr_r = new_node();

        nodes[curr_node_addr].l = node_addr_l;
        nodes[node_addr_l].bbox = min_split_leftBox;
        nodes[node_addr_l].start = startl;
        nodes[node_addr_l].size = rangel;

        nodes[curr_node_addr].r = node_addr_r;
        nodes[node_addr_r].bbox = min_split_rightBox;
        nodes[node_addr_r].start = startr;
        nodes[node_addr_r].size = ranger;

        split_node(node_addr_l, max_leaf_size);
        split_node(node_addr_r, max_leaf_size); 
    }




    // clear the vectors of size_t
    for(size_t i = 0; i < NUM_PARTITIONS; i++) {
        delete p_idx[0][i];
        delete p_idx[1][i];
        delete p_idx[2][i];
    }
}

template<typename Primitive>
Trace BVH<Primitive>::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.
    
    Trace ret;

    for(const Primitive& prim : primitives) {
        Trace hit = prim.hit(ray);
        ret = Trace::min(ret, hit);
    }


    //std::stack<size_t> tstack;
    ////tstack.push(root_idx);
    //Vec2 times = Vec2(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
   
    ////check if ray hit the root first before entering while-loop
    //const Node& root = nodes[root_idx];
    //bool hit_root = root.bbox.hit(ray, times);
    //if(hit_root) {
    //    if(root.is_leaf()) {
    //        for(size_t i = root.start; i < root.start + root.size; i++) {
    //            const Primitive& prim = primitives[i];
    //            Trace hit = prim.hit(ray);
    //            ret = Trace::min(ret, hit);
    //        }
    //    } else {
    //         Vec2 time_l = Vec2(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
    //         Vec2 time_r = Vec2(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());

    //        bool hit_l = nodes[root.l].bbox.hit(ray, time_l);
    //        bool hit_r = nodes[root.r].bbox.hit(ray, time_r);

    //        // if entering the box at t > current closest primitive's t,
    //        // don't consider the bbox
    //        hit_l = hit_l && (time_l.x <= ray.dist_bounds.y);
    //        hit_r = hit_r && (time_r.x <= ray.dist_bounds.y);

    //        bool hit_first, hit_second;
    //        size_t node_first, node_second;

    //        if (time_l.x < time_r.x) {
    //            node_first = root.l;
    //            node_second = root.r;
    //            hit_first = hit_l;
    //            hit_second = hit_r;
    //        } else {
    //            node_first = root.r;
    //            node_second = root.l;
    //            hit_first = hit_r;
    //            hit_second = hit_l;
    //        }

    //        if (hit_second) {
    //            tstack.push(node_second);

    //        }
    //        if (hit_first) {
    //            tstack.push(node_first);
    //        }
    //    }
    //}

    //while(!tstack.empty()) {
    //    size_t curr_node_idx = tstack.top();
    //    const Node& node = nodes[curr_node_idx];
    //    tstack.pop();

    //    if (node.is_leaf()) {
    //        for(size_t i = node.start; i < node.start + node.size; i++) {
    //            const Primitive& prim = primitives[i];
    //            Trace hit = prim.hit(ray);
    //            ret = Trace::min(ret, hit);
    //        }
    //    } else {
    //        Vec2 time_l = Vec2(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
    //        Vec2 time_r = Vec2(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());

    //        bool hit_l = nodes[node.l].bbox.hit(ray, time_l);
    //        bool hit_r = nodes[node.r].bbox.hit(ray, time_r);

    //        // if entering the box at t > current closest primitive's t,
    //        // don't consider the bbox
    //        hit_l = hit_l && (time_l.x <= ray.dist_bounds.y);
    //        hit_r = hit_r && (time_r.x <= ray.dist_bounds.y);

    //        bool hit_first, hit_second;
    //        size_t node_first, node_second;

    //        if(time_l.x < time_r.x) {
    //            node_first = node.l;
    //            node_second = node.r;
    //            hit_first = hit_l;
    //            hit_second = hit_r;
    //        } else {
    //            node_first = node.r;
    //            node_second = node.l;
    //            hit_first = hit_r;
    //            hit_second = hit_l;
    //        }

    //        if(hit_second) {
    //            tstack.push(node_second);
    //        }
    //        if(hit_first) {
    //            tstack.push(node_first);
    //        }
    //    }
    //}


    return ret;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive>
BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive>
bool BVH<Primitive>::Node::is_leaf() const {
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive>
BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive>
std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive>
void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {

    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        max_level = std::max(max_level, lvl);
        const Node& node = nodes[idx];
        tstack.pop();
        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);

        GL::Lines& add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(node.l && node.r) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
