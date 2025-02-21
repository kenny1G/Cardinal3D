#include "../rays/tri_mesh.h"
#include "debug.h"

namespace PT {

BBox Triangle::bbox() const {

    // TODO (PathTracer): Task 2
    // compute the bounding box of the triangle

    // Beware of flat/zero-volume boxes! You may need to
    // account for that here, or later on in BBox::intersect

    BBox box;
    box.enclose(vertex_list[v0].position);
    box.enclose(vertex_list[v1].position);
    box.enclose(vertex_list[v2].position);
    return box;
}

Trace Triangle::hit(const Ray& ray) const {
    // Vertices of triangle - has postion and surface normal
    // See rays/tri_mesh.h for a description of this struct

    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];

    // (PathTracer): Task 2
    // Intersect this ray with a triangle defined by the above three points.
    // Intersection should yield a ray t-value, and a hit point (u,v) on the surface of the triangle

    // You'll need to fill in a "Trace" struct describing information about the hit (or lack of hit)

    //Moller–Trumbore intersection algorithm
    Vec3 p0 = v_0.position;
    Vec3 p1 = v_1.position;
    Vec3 p2 = v_2.position;

    Vec3 e1 = p1 - p0;
    Vec3 e2 = p2 - p0;
    Vec3 s = ray.point - p0;

    Vec3 e2_cross_d = cross(ray.dir, e2);
    float det = dot(e1, e2_cross_d);

    if(std::abs(det) == 0) return Trace{};
    float inv_det = 1.0f / det;

    float u = dot(s, e2_cross_d) * inv_det;
    if(u < 0.0f || u > 1.0f) return Trace{};

    Vec3 s_cross_e1 = cross(s, e1);
    float v = dot(ray.dir, s_cross_e1) * inv_det;
    if(v < 0.0f || u + v > 1.0f) return Trace{};

    float t = dot(e2, s_cross_e1) * inv_det;
    if(t < ray.dist_bounds.x || t > ray.dist_bounds.y) return Trace{};

    Vec3 normal = (1 - u - v) * v_0.normal + u * v_1.normal + v * v_2.normal;
    normal = normal.unit();

    Trace ret;
    ret.origin = ray.point;
    ret.hit = true;           // was there an intersection?
    ret.distance = t;         // at what distance did the intersection occur?
    ret.position = ray.at(t); // where was the intersection?
    ret.normal = normal;      // what was the surface normal at the intersection?
                              // (this should be interpolated between the three vertex normals)
    return ret;
}

Triangle::Triangle(Tri_Mesh_Vert* verts, unsigned int v0, unsigned int v1, unsigned int v2)
    : vertex_list(verts), v0(v0), v1(v1), v2(v2) {
}

void Tri_Mesh::build(const GL::Mesh& mesh) {

    verts.clear();
    triangles.clear();

    for(const auto& v : mesh.verts()) {
        verts.push_back({v.pos, v.norm});
    }

    const auto& idxs = mesh.indices();

    std::vector<Triangle> tris;
    for(size_t i = 0; i < idxs.size(); i += 3) {
        tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
    }

    triangles.build(std::move(tris), 4);
}

Tri_Mesh::Tri_Mesh(const GL::Mesh& mesh) {
    build(mesh);
}

Tri_Mesh Tri_Mesh::copy() const {
    Tri_Mesh ret;
    ret.verts = verts;
    ret.triangles = triangles.copy();
    return ret;
}

BBox Tri_Mesh::bbox() const {
    return triangles.bbox();
}

Trace Tri_Mesh::hit(const Ray& ray) const {
    Trace t = triangles.hit(ray);
    return t;
}

size_t Tri_Mesh::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                           const Mat4& trans) const {
    return triangles.visualize(lines, active, level, trans);
}

} // namespace PT
