
#include <cstddef>
#include <queue>
#include <set>
#include <unordered_map>

#include "../geometry/halfedge.h"
#include "debug.h"

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementaiton, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    (void)v;
    return std::nullopt;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;
    return std::nullopt;
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef edge) {
    HalfedgeRef halves[2] = {edge->halfedge(), edge->halfedge()->twin()};
    HalfedgeRef ends[2];
    int vertex_counts[2];
    for(int i = 0; i < 2; ++i) {
        HalfedgeRef first = halves[i];
        HalfedgeRef end = first;
        int count = 1;
        while(end->next() != first) {
            count++;
            end = end->next();
        }
        ends[i] = end;
        vertex_counts[i] = count;
    }
    if(halves[0]->is_boundary() || halves[1]->is_boundary()) {
        int inner_idx = halves[0]->is_boundary() ? 1 : 0;
        if(vertex_counts[inner_idx] == 3 && halves[inner_idx]->next()->twin()->is_boundary() &&
           ends[inner_idx]->twin()->is_boundary()) {
            return std::nullopt;
        }
    } else {
        for(int i = 0; i < 2; ++i) {
            if(vertex_counts[i] == 3 &&
               ends[i]->twin()->next()->edge() == halves[i]->next()->edge()) {
                return std::nullopt;
            }
        }
        std::vector<HalfedgeRef> ring_halfedges;
        HalfedgeRef iter = halves[0]->next();
        do {
            ring_halfedges.push_back(iter->twin());
            iter = iter->twin()->next();
        } while(iter != ends[1]->twin());
        iter = halves[1]->next();
        do {
            for(auto& hedge : ring_halfedges) {
                if(hedge->vertex()->id() == iter->twin()->vertex()->id() &&
                   !(hedge->edge()->id() == ends[1]->edge()->id() ||
                     iter->edge()->id() == ends[0]->edge()->id())) {
                    return std::nullopt;
                }
            }
            iter = iter->twin()->next();
        } while(iter != ends[0]->twin());
        int boundary_count = 0;
        for(int i = 0; i < 2; ++i) {
            HalfedgeRef walker = halves[i];
            do {
                if(walker->is_boundary()) {
                    boundary_count++;
                    break;
                }
                walker = walker->twin()->next();
            } while(walker != halves[i]);
        }
        if(boundary_count == 2) {
            return std::nullopt;
        }
    }
    for(int i = 0; i < 2; ++i) {
        vertex_counts[i] = halves[i]->face()->degree();
        if(vertex_counts[i] == 3) {
            FaceRef doomed_face = halves[i]->face();
            halves[i]->face() = ends[i]->twin()->face();
            halves[i]->next()->next() = ends[i]->twin()->next();
            halves[i]->next()->face() = ends[i]->twin()->face();
            HalfedgeRef twin_last = ends[i]->twin();
            while(twin_last->next() != ends[i]->twin()) {
                twin_last = twin_last->next();
            }
            twin_last->next() = halves[i];
            erase(doomed_face);
            ends[i]->vertex()->halfedge() = ends[i]->twin()->next();
            ends[i]->twin()->vertex()->halfedge() = halves[i];
            ends[i]->twin()->face()->halfedge() = ends[i]->twin()->next();
            erase(ends[i]->edge());
            erase(ends[i]->twin());
            erase(ends[i]);
            ends[i] = twin_last;
        }
    }
    HalfedgeRef current = halves[1]->next();
    while(current != halves[0]) {
        current->vertex() = halves[1]->vertex();
        current = current->twin()->next();
    }
    if(vertex_counts[0] == 3) {
        ends[0]->next() = halves[0]->next();
        ends[1]->next() = halves[1]->next();
        halves[0]->face()->halfedge() = ends[0];
        halves[1]->face()->halfedge() = ends[1];
    } else {
        ends[1]->next() = halves[1]->next();
        ends[0]->next() = halves[0]->next();
        halves[1]->face()->halfedge() = ends[1];
        halves[0]->face()->halfedge() = ends[0];
    }
    halves[1]->vertex()->halfedge() =
        herased.count(halves[1]->next()) ? halves[0]->next() : halves[1]->next();
    halves[1]->vertex()->pos =
        (halves[1]->vertex()->center() + halves[0]->vertex()->center()) / 2.f;
    VertexRef surviving_vertex = halves[1]->vertex();
    erase(halves[0]->edge());
    erase(halves[0]->vertex());
    erase(halves[0]);
    erase(halves[1]);
    return surviving_vertex;
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;
    return std::nullopt;
}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {
    EdgeRef e0 = e;
    if(e0->halfedge()->is_boundary() || e0->halfedge()->twin()->is_boundary()) {
        return std::nullopt;
    }

    HalfedgeRef h0 = e0->halfedge();
    HalfedgeRef h1 = h0->twin();

    HalfedgeRef h_check = h0->next()->next();
    do {
        if(h_check->next()->vertex() == h1->next()->next()->vertex()) {
            return std::nullopt;
        }
        h_check = h_check->twin()->next();
    } while(h_check != h0->next()->next());

    HalfedgeRef h2 = h0->next();
    HalfedgeRef h3 = h1->next();

    if(h0->vertex()->halfedge() == h0) {
        h0->vertex()->halfedge() = h3;
    }
    if(h1->vertex()->halfedge() == h1) {
        h1->vertex()->halfedge() = h2;
    }

    if(h2->face()->halfedge() == h2) {
        h2->face()->halfedge() = h2->next();
    }
    if(h3->face()->halfedge() == h3) {
        h3->face()->halfedge() = h3->next();
    }

    HalfedgeRef h4 = h0;
    do {
        h4 = h4->next();
    } while(h4->next() != h0);

    HalfedgeRef h5 = h1;
    do {
        h5 = h5->next();
    } while(h5->next() != h1);

    h0->vertex() = h1->next()->next()->vertex();
    h1->vertex() = h0->next()->next()->vertex();

    h0->next() = h0->next()->next();
    h1->next() = h1->next()->next();

    h2->face() = h1->face();
    h2->next() = h1;
    h3->face() = h0->face();
    h3->next() = h0;

    h4->next() = h3;
    h5->next() = h2;

    return e0;
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {

    if(!e->on_boundary()) {
        VertexRef v0 = new_vertex();
        v0->pos = e->center();
        std::vector<HalfedgeRef> halfedges_;
        auto h = e->halfedge()->next();
        do {
            halfedges_.push_back(h);
            h = h->next();
        } while(h->next() != e->halfedge()->next());

        h = e->halfedge()->twin()->next();
        do {
            halfedges_.push_back(h);
            h = h->next();
        } while(h->next() != e->halfedge()->twin()->next());

        HalfedgeRef h0 = e->halfedge();
        HalfedgeRef h1 = h0->twin();
        VertexRef v1 = h0->vertex();
        VertexRef v2 = h1->vertex();

        if(v1->halfedge() == h0) {
            v1->halfedge() = h1->next();
            h1->next()->vertex() = v1;
        }
        if(v2->halfedge() == h1) {
            v2->halfedge() = h0->next();
            h0->next()->vertex() = v2;
        }

        erase(h0->face());
        erase(h1->face());
        erase(h1);
        erase(h0);
        erase(e);

        std::vector<HalfedgeRef> right;
        std::vector<HalfedgeRef> left;
        for(size_t i = 0; i < halfedges_.size(); ++i) {
            EdgeRef e0 = new_edge();
            HalfedgeRef h2 = new_halfedge();
            HalfedgeRef h3 = new_halfedge();
            FaceRef f0 = new_face();
            e0->halfedge() = h2;
            f0->halfedge() = halfedges_[i];
            halfedges_[i]->face() = f0;
            h2->set_neighbors(halfedges_[i], h3, v0, e0, f0);
            h3->set_neighbors(h3->next(), h2, halfedges_[i]->vertex(), e0, h3->face());
            right.push_back(h3);
            left.push_back(h2);
        }

        int N = (int)halfedges_.size();
        for(size_t i = 0; i < halfedges_.size(); ++i) {
            int prev = (i + N - 1) % N;
            HalfedgeRef hprev = halfedges_[prev];
            hprev->next() = right[i];
            right[i]->face() = hprev->face();
            right[i]->next() = left[prev];
        }

        v0->halfedge() = left[0];
        return v0;
    }
    HalfedgeRef h0 = e->halfedge();
    if(h0->is_boundary()) {
        h0 = h0->twin();
    }
    HalfedgeRef h1 = h0->next();
    HalfedgeRef h2 = h1->next();
    HalfedgeRef h3 = h0->twin();

    if(h2->next() != h0) {
        return std::nullopt;
    }

    HalfedgeRef h4 = new_halfedge();
    HalfedgeRef h5 = new_halfedge();
    HalfedgeRef h6 = new_halfedge();
    HalfedgeRef h7 = new_halfedge();
    HalfedgeRef h8 = new_halfedge();
    HalfedgeRef h9 = new_halfedge();

    VertexRef v0 = new_vertex();
    v0->pos = (h0->vertex()->pos + h3->vertex()->pos) / 2.f;

    EdgeRef e0 = new_edge();
    EdgeRef e1 = new_edge();
    EdgeRef e2 = new_edge();

    FaceRef f0 = new_face();
    FaceRef f1 = new_face();

    h4->set_neighbors(h1, h5, v0, e0, f0);
    h5->set_neighbors(h6, h4, h1->vertex(), e0, h3->face());
    h6->set_neighbors(h3->next(), h7, v0, e1, h3->face());
    h7->set_neighbors(h8, h6, h0->vertex(), e1, f1);
    h8->set_neighbors(h2, h9, v0, e2, f1);
    h9->set_neighbors(h4, h8, h2->vertex(), e2, f0);

    h1->face() = f0;
    h1->next() = h9;

    h2->face() = f1;
    h2->next() = h7;

    unsigned int boundary_face_degree = h3->face()->degree();
    HalfedgeRef prev = h3;
    for(unsigned int i = 0; i < boundary_face_degree - 1; ++i) {
        prev = prev->next();
    }
    prev->next() = h5;

    v0->halfedge() = h4;
    h1->vertex()->halfedge() = h1;
    h2->twin()->vertex()->halfedge() = h2->twin();

    e0->halfedge() = h4;
    e1->halfedge() = h7;
    e2->halfedge() = h8;

    f0->halfedge() = h4;
    f1->halfedge() = h8;
    h3->face()->halfedge() = h5;

    FaceRef old_face = h0->face();
    erase(h0);
    erase(h3);
    erase(e);
    erase(old_face);

    return v0;
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for  bevel_vertex, it has one element, the original vertex position,
    for bevel_edge,  two for the two vertices, and for bevel_face, it has the original
    position of each vertex in halfedge order. You should use these positions, as well
    as the normal and tangent offset fields to assign positions to the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)v;
    return std::nullopt;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    unsigned int n = f->degree();

    std::vector<HalfedgeRef> h0;
    HalfedgeRef h = f->halfedge();
    for(unsigned int i = 0; i < n; ++i) {
        h0.push_back(h);
        h = h->next();
    }

    std::vector<HalfedgeRef> h1;
    std::vector<HalfedgeRef> h2;
    std::vector<HalfedgeRef> h3;
    std::vector<HalfedgeRef> h4;
    for(unsigned int i = 0; i < n; ++i) {
        h1.push_back(new_halfedge());
        h2.push_back(new_halfedge());
        h3.push_back(new_halfedge());
        h4.push_back(new_halfedge());
    }

    std::vector<VertexRef> v;
    for(unsigned int i = 0; i < n; ++i) {
        VertexRef new_v = new_vertex();
        new_v->pos = h0[i]->vertex()->pos;
        v.push_back(new_v);
    }

    std::vector<EdgeRef> e0;
    std::vector<EdgeRef> e1;
    for(unsigned int i = 0; i < n; ++i) {
        e0.push_back(new_edge());
        e1.push_back(new_edge());
    }

    std::vector<FaceRef> f1;
    for(unsigned int i = 0; i < n; ++i) {
        f1.push_back(new_face());
    }

    for(unsigned int i = 0; i < n; ++i) {
        h0[i]->face() = f1[i];
        h0[i]->next() = h1[(i + 1) % n];

        h1[i]->set_neighbors(h4[(i - 1 + n) % n], h2[i], h0[i]->vertex(), e0[i],
                             f1[(i - 1 + n) % n]);

        h2[i]->set_neighbors(h0[i], h1[i], v[i], e0[i], f1[i]);

        h3[i]->set_neighbors(h3[(i + 1) % n], h4[i], v[i], e1[i], f);

        h4[i]->set_neighbors(h2[i], h3[i], v[(i + 1) % n], e1[i], f1[i]);
    }

    for(unsigned int i = 0; i < n; ++i) {
        v[i]->halfedge() = h3[i];
    }

    for(unsigned int i = 0; i < n; ++i) {
        e0[i]->halfedge() = h1[i];
        e1[i]->halfedge() = h3[i];
    }

    f->halfedge() = h3[0];
    for(unsigned int i = 0; i < n; ++i) {
        f1[i]->halfedge() = h4[i];
    }

    return f;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    for(size_t i = 0; i < new_halfedges.size(); ++i) {
        Vec3 tv_pos = new_halfedges[i]->twin()->next()->twin()->vertex()->pos;
        new_halfedges[i]->vertex()->pos =
            start_positions[i] +
            std::max(0.05f, std::min(0.95f, -tangent_offset)) * (tv_pos - start_positions[i]);
    }
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions
    in orig.  So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); ++i)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions
    in orig. So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); ++i)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {

    if(flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    size_t N = new_halfedges.size();
    Vec3 center = Vec3(0, 0, 0);
    for(size_t i = 0; i < N; ++i) {
        center += start_positions[i];
    }
    tangent_offset = std::max(-0.95f, tangent_offset);
    for(size_t i = 0; i < N; ++i) {
        Vec3 start_pos = start_positions[i];
        new_halfedges[i]->vertex()->pos =
            start_pos + tangent_offset * (start_pos - center / N) + -normal_offset * face->normal();
    }
}

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {

    // For each face...
    for(FaceRef f = faces_begin(); f != faces_end(); ++f) {
        HalfedgeRef og_halfedge = f->halfedge();
        VertexRef og_vertex = og_halfedge->vertex();

        HalfedgeRef it = og_halfedge;
        while(it->next()->next()->next() != og_halfedge) {
            EdgeRef new_e = new_edge();
            FaceRef new_f = new_face();
            HalfedgeRef new_he = new_halfedge();
            HalfedgeRef twin_he = new_halfedge();

            new_he->set_neighbors(it->next()->next(), twin_he, og_vertex, new_e, new_f);
            twin_he->set_neighbors(it, new_he, it->next()->next()->vertex(), new_e, f);

            new_e->halfedge() = twin_he;
            new_f->halfedge() = twin_he;

            it->face() = new_f;
            it->next()->next() = twin_he;
            it->next()->face() = new_f;

            it = new_he;
        }

        it->next()->next()->next() = it;
        it->face() = f;
        f->halfedge() = it;
    }
}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided mesh).  They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {

    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.
    for(VertexRef v = vertices_begin(); v != vertices_end(); ++v) {
        v->new_pos = v->pos;
    }

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.
    for(EdgeRef e = edges_begin(); e != edges_end(); ++e) {
        e->new_pos = e->center();
    }

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
    for(FaceRef f = faces_begin(); f != faces_end(); ++f) {
        f->new_pos = f->center();
    }
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces
    Vec3 face_total;
    for(FaceRef f = faces_begin(); f != faces_end(); ++f) {
        f->new_pos = f->center();
        face_total += f->new_pos;
    }

    // Edges
    for(EdgeRef e = edges_begin(); e != edges_end(); ++e) {
        Vec3 ME = e->center();
        FaceRef A = e->halfedge()->face();
        FaceRef F = e->halfedge()->twin()->face();
        Vec3 AF = 0.5f * (A->new_pos + F->new_pos);
        e->new_pos = 0.5f * (AF + ME);
    }

    // Vertices
    for(VertexRef v = vertices_begin(); v != vertices_end(); ++v) {
        float n = 0;
        Vec3 Q;
        Vec3 R;
        Vec3 S = v->pos;
        HalfedgeRef h = v->halfedge();
        /*Iterate through each of v's connected faces*/
        do {
            h = h->twin();
            Q += h->face()->new_pos;
            R += h->edge()->center();
            h = h->next();
            n += 1;
        } while(h != v->halfedge());
        Q = Q / n;
        R = R / n;
        v->new_pos = (Q + (2 * R) + ((n - 3) * S)) / n;
    }
}

/*
        This routine should increase the number of triangles in the mesh
        using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

    // Compute new positions for all the vertices in the input mesh, using
    // the Loop subdivision rule, and store them in Vertex::new_pos.
    // -> At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    // -> Next, compute the updated vertex positions associated with edges, and
    //    store it in Edge::new_pos.
    // -> Next, we're going to split every edge in the mesh, in any order.  For
    //    future reference, we're also going to store some information about which
    //    subdivided edges come from splitting an edge in the original mesh, and
    //    which edges are new, by setting the flat Edge::is_new. Note that in this
    //    loop, we only want to iterate over edges of the original mesh.
    //    Otherwise, we'll end up splitting edges that we just split (and the
    //    loop will never end!)
    // -> Now flip any new edge that connects an old and new vertex.
    // -> Finally, copy the new vertex positions into final Vertex::pos.

    // Each vertex and edge of the original surface can be associated with a
    // vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh; navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    // Compute updated positions for all the vertices in the original mesh, using
    // the Loop subdivision rule.

    // Next, compute the updated vertex positions associated with edges.

    // Next, we're going to split every edge in the mesh, in any order. For
    // future reference, we're also going to store some information about which
    // subdivided edges come from splitting an edge in the original mesh, and
    // which edges are new.
    // In this loop, we only want to iterate over edges of the original
    // mesh---otherwise, we'll end up splitting edges that we just split (and
    // the loop will never end!)

    // Finally, flip any new edge that connects an old and new vertex.

    // Copy the updated vertex positions to the subdivided mesh.
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.
        const Mat4 Q = vertex_quadrics[e->halfedge()->vertex()] +
                       vertex_quadrics[e->halfedge()->twin()->vertex()];
        const Mat4 system(
            Vec4(Q[0][0], Q[0][1], Q[0][2], 0.f), Vec4(Q[0][1], Q[1][1], Q[1][2], 0.f),
            Vec4(Q[0][2], Q[1][2], Q[2][2], 0.f), Vec4(Q[0][3], Q[1][3], Q[2][3], 1.f));
        /* Assuming this matrix is invertible. A better solution should check the determinant and
         * possibly resort to a heuristic point on the edge. */
        const Vec4 solution = system.inverse()[3];
        optimal = Vec3(solution[0], solution[1], solution[2]);
        cost = dot((Q * solution), solution);
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;
    std::set<EdgeRef> collapsed_edges;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    for(FaceRef f = faces_begin(); f != faces_end(); ++f) {
        Vec3 norm = f->normal();
        float d = dot(norm, f->halfedge()->vertex()->pos);
        Vec4 v(norm.x, norm.y, norm.z, d);
        Mat4 quadric = outer(v, v);
        face_quadrics[f] = quadric;
    }
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    for(VertexRef v = vertices_begin(); v != vertices_end(); ++v) {
        /*Iterate through each of v's connected faces*/
        Mat4 quadric = Mat4::Zero;
        HalfedgeRef h = v->halfedge();
        do {
            quadric += face_quadrics[h->face()];
            h = h->twin()->next();
        } while(h != v->halfedge());
        vertex_quadrics[v] = quadric;
    }
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    for(EdgeRef e = edges_begin(); e != edges_end(); ++e) {
        Edge_Record er = Edge_Record(vertex_quadrics, e);
        edge_records[e] = er;
        edge_queue.insert(er);
    }

    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.
    Size face_target = n_faces() / 4;
    if(face_target < 2) return false;
    while(edge_queue.size() > 0 && n_faces() > face_target) {
        EdgeRef collapse_candidate = edge_queue.top().edge;
        edge_queue.pop();
        collapsed_edges.insert(collapse_candidate);

        std::set<EdgeRef> adjacent_edges;
        HalfedgeRef iter_half = collapse_candidate->halfedge()->next();
        while(iter_half->edge() != collapse_candidate) {
            adjacent_edges.insert(iter_half->edge());
            iter_half = iter_half->twin()->next();
        }
        iter_half = collapse_candidate->halfedge()->twin()->next();
        while(iter_half->edge() != collapse_candidate) {
            adjacent_edges.insert(iter_half->edge());
            iter_half = iter_half->twin()->next();
        }
        for(const EdgeRef& adj : adjacent_edges) edge_queue.remove(edge_records[adj]);

        const Mat4 merged_quadric =
            vertex_quadrics[collapse_candidate->halfedge()->vertex()] +
            vertex_quadrics[collapse_candidate->halfedge()->twin()->vertex()];
        std::optional<VertexRef> result_vertex = collapse_edge_erase(collapse_candidate);

        if(result_vertex) {
            vertex_quadrics[*result_vertex] = merged_quadric;
            iter_half = (*result_vertex)->halfedge();
            do {
                if(adjacent_edges.find(iter_half->edge()) != adjacent_edges.end() &&
                   collapsed_edges.find(iter_half->edge()) == collapsed_edges.end()) {
                    edge_records[iter_half->edge()] =
                        Edge_Record(vertex_quadrics, iter_half->edge());
                    edge_queue.insert(edge_records[iter_half->edge()]);
                }
                iter_half = iter_half->twin()->next();
            } while(iter_half != (*result_vertex)->halfedge());
        } else {
            for(const EdgeRef& adj : adjacent_edges) {
                if(collapsed_edges.find(adj) != collapsed_edges.end()) continue;

                edge_records[adj] = Edge_Record(vertex_quadrics, adj);
                edge_queue.insert(edge_records[adj]);
            }
        }
    }

    return true;
}
