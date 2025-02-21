#include "../rays/bvh.h"
#include "debug.h"
#include <stack>

namespace PT {

#define NUM_BINS 32
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

    // set up root node (root BVH). Notice that it contains all primitives.
    size_t root_node_addr = new_node();
    Node& root = nodes[root_node_addr];
    root.start = 0, root.size = primitives.size();
    update_node_bounds(root_node_addr);
    // subdivide recursively
    subdivide(root_node_addr, max_leaf_size);
}

template<typename Primitive> void BVH<Primitive>::update_node_bounds(size_t node_idx) {
    Node& node = nodes[node_idx];
    node.bbox = BBox();
    for(size_t first = node.start, i = 0; i < node.size; ++i) {
        node.bbox.enclose(primitives[first + i].bbox());
    }
}

template<typename Primitive>
float BVH<Primitive>::find_best_split(size_t node_addr, int& axis, int& split_pos) {
    float best_cost = std::numeric_limits<float>::infinity();
    Vec3 scale = (nodes[node_addr].bbox.max - nodes[node_addr].bbox.min) / NUM_BINS;

    //  For each axis X,Y,Z:
    for(int a = 0; a < 3; a++) {
        if(scale[a] == 0) continue;

        BBox left_area[NUM_BINS];
        size_t count[NUM_BINS];
        memset(count, 0, sizeof(size_t) * NUM_BINS);

        for(size_t i = nodes[node_addr].start; i < nodes[node_addr].start + nodes[node_addr].size;
            i++) {
            int bucket =
                floor((primitives[i].bbox().center()[a] - nodes[node_addr].bbox.min[a]) / scale[a]);
            if(bucket == NUM_BINS) bucket = NUM_BINS - 1;
            left_area[bucket].enclose(primitives[i].bbox());
            count[bucket]++;
        }

        BBox right_area[NUM_BINS];
        memcpy(right_area, left_area, sizeof(BBox) * NUM_BINS);
        // Accumulate right areas by sweeping from right to left
        for(int i = 1; i < NUM_BINS; i++) {
            left_area[i].enclose(left_area[i - 1]);
            count[i] += count[i - 1];
            // Right area accumulates from the right side
            right_area[NUM_BINS - 1 - i].enclose(right_area[NUM_BINS - i]);
        }

        //  Try possible splits along axis, evaluate SAH for each
        float node_area = nodes[node_addr].bbox.surface_area();
        float axis_best_cost = std::numeric_limits<float>::infinity();
        int best_idx = 0;
        for(int i = 0; i < NUM_BINS - 1; i++) {
            size_t left_count = count[i];
            size_t right_count = nodes[node_addr].size - left_count;
            float cost = left_area[i].surface_area() / node_area * left_count +
                         right_area[i + 1].surface_area() / node_area * right_count;
            if(cost < axis_best_cost) {
                axis_best_cost = cost;
                best_idx = i;
            }
        }

        if(axis_best_cost < best_cost) {
            best_cost = axis_best_cost;
            axis = a;
            split_pos = best_idx;
        }
    }
    //  Take minimum cost across all axes.
    return nodes[node_addr].bbox.min[axis] + (split_pos + 1) * scale[axis];
}

template<typename Primitive> void BVH<Primitive>::subdivide(size_t node_idx, size_t max_leaf_size) {
    size_t node_start = nodes[node_idx].start;
    size_t node_size = nodes[node_idx].size;

    if(node_size <= std::max(static_cast<size_t>(NUM_BINS), max_leaf_size)) return;

    int axis;
    int split_pos;
    float split = find_best_split(node_idx, axis, split_pos);

    //  Partition primitives into a left and right child group
    size_t i = node_start;
    size_t j = i + node_size - 1;
    while(i <= j) {
        if(primitives[i].bbox().center()[axis] < split)
            i++;
        else
            std::swap(primitives[i], primitives[j--]);
    }

    // abort split if one of the sides is empty
    size_t left_count = i - node_start;
    if(left_count == 0 || left_count == node_size) return;

    size_t node_addr_l = new_node(BBox(), node_start, left_count);
    size_t node_addr_r = new_node(BBox(), node_start + left_count, node_size - left_count);
    //  Compute left and right child bboxes
    update_node_bounds(node_addr_l);
    update_node_bounds(node_addr_r);

    nodes[node_idx].l = node_addr_l;
    nodes[node_idx].r = node_addr_r;

    //  Make the left and right child nodes.
    subdivide(node_addr_l, max_leaf_size);
    subdivide(node_addr_r, max_leaf_size);
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {
    Trace ret;
    ret.distance = std::numeric_limits<float>::infinity();

    std::stack<size_t> node_stack;
    node_stack.push(root_idx);

    while(!node_stack.empty()) {
        size_t node_addr = node_stack.top();
        node_stack.pop();
        const Node& node = nodes[node_addr];

        Vec2 node_range(0.f, std::numeric_limits<float>::infinity());
        if(!node.bbox.hit(ray, node_range) || node_range.x >= ret.distance) continue;

        if(node.is_leaf()) {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                Trace trace = primitives[i].hit(ray);
                if(trace.hit && trace.distance < ret.distance) {
                    ret = trace;
                }
            }
        } else {
            Vec2 l_range(0.f, std::numeric_limits<float>::infinity());
            Vec2 r_range(0.f, std::numeric_limits<float>::infinity());
            bool hit_l = nodes[node.l].bbox.hit(ray, l_range);
            bool hit_r = nodes[node.r].bbox.hit(ray, r_range);

            bool l_valid = hit_l && (l_range.x < ret.distance);
            bool r_valid = hit_r && (r_range.x < ret.distance);

            if(l_valid && r_valid) {
                if(l_range.x < r_range.x) {
                    node_stack.push(node.r);
                    node_stack.push(node.l);
                } else {
                    node_stack.push(node.l);
                    node_stack.push(node.r);
                }
            } else if(l_valid) {
                node_stack.push(node.l);
            } else if(r_valid) {
                node_stack.push(node.r);
            }
        }
    }
    return ret;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive> BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {
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

template<typename Primitive> BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive> void BVH<Primitive>::clear() {
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
