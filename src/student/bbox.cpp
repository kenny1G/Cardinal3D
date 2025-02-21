#include "../lib/mathlib.h"
#include "debug.h"

bool BBox::hit(const Ray& ray, Vec2& times) const {
    // TODO (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.

    float t_entry = times.x;
    float t_exit = times.y;

    for(int axis = 0; axis < 3; ++axis) {
        float t0, t1;
        if(ray.dir[axis] == 0.0f) {
            if(ray.point[axis] < min[axis] || ray.point[axis] > max[axis]) return false;
            t0 = -FLT_MAX;
            t1 = FLT_MAX;
        } else {
            float inv_dir = 1.0f / ray.dir[axis];
            t0 = (min[axis] - ray.point[axis]) * inv_dir;
            t1 = (max[axis] - ray.point[axis]) * inv_dir;
            if(t0 > t1) std::swap(t0, t1);
        }

        t_entry = std::max(t_entry, t0);
        t_exit = std::min(t_exit, t1);
        if(t_entry > t_exit) return false;
    }

    if(t_exit < 0) return false;

    times.x = t_entry;
    times.y = t_exit;
    return true;
}