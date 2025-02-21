#include "../rays/shapes.h"
#include "debug.h"

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

Trace Sphere::hit(const Ray& ray) const {

    // (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!

    float b = dot(ray.point, ray.dir);
    float c = ray.point.norm_squared() - radius * radius;

    float discriminant = b * b - c;
    if(discriminant < 0.0f) return Trace{};

    float sqrt_d = std::sqrt(discriminant);
    float t0 = (-b - sqrt_d);
    float t1 = (-b + sqrt_d);

    float t = -1;
    if(t0 >= ray.dist_bounds.x && t0 <= ray.dist_bounds.y) t = t0;
    if(t1 >= ray.dist_bounds.x && t1 <= ray.dist_bounds.y && t1 < t) t = t1;

    if(t == -1) return Trace{};

    ray.dist_bounds.y = t;
    Trace ret;
    ret.origin = ray.point;
    ret.hit = true;                // was there an intersection?
    ret.distance = t;              // at what distance did the intersection occur?
    ret.position = ray.at(t);      // where was the intersection?
    ret.normal = ray.at(t).unit(); // what was the surface normal at the intersection?
    return ret;
}

} // namespace PT
