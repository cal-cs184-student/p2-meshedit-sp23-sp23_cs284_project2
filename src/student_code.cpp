#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    int num_points = points.size();
    vector<Vector2D> intermediate_points;
    for (int i = 0; i < num_points - 1; i++) {
      intermediate_points.push_back((1.0 - t) * points[i] + t * points[i + 1]);
    }
    return intermediate_points;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    int num_points = points.size();
    vector<Vector3D> intermediate_points;
    for (int i = 0; i < num_points - 1; i++) {
      intermediate_points.push_back((1.0 - t) * points[i] + t * points[i + 1]);
    }
    return intermediate_points;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    if (points.size() == 1) return points[0];
    return evaluate1D(evaluateStep(points, t), t);
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    vector<Vector3D> movingBezierCurve;
    int num_curves = controlPoints.size();
    for (int i = 0; i < num_curves; i++) {
      movingBezierCurve.push_back(evaluate1D(controlPoints[i], u));
    }
    return evaluate1D(movingBezierCurve, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.

    // TODO: Change the do while loop into normal while loop
    // TODO: Check for boundary faces within do-while loop?
    Vector3D area_weighted_normal(0,0,0);
    HalfedgeCIter half_edge = halfedge();
    do {
      Vector3D vertex_0 = position;
      Vector3D vertex_1 = half_edge -> next() -> vertex() -> position;
      Vector3D vertex_2 = half_edge -> next() -> next() -> vertex() -> position;
      area_weighted_normal += cross(vertex_1 - vertex_0, vertex_2 - vertex_0);
      half_edge = half_edge -> twin() -> next();
    } while (half_edge != halfedge());
    return area_weighted_normal.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    if (e0 -> isBoundary()) return e0;

    HalfedgeIter half_edge = e0 -> halfedge(), half_edge_next = half_edge -> next(), half_edge_prev = half_edge_next -> next();
    HalfedgeIter twin = half_edge -> twin(), twin_next = twin -> next(), twin_prev = twin_next -> next();
    
    // Set upper triangle face
    // TODO: change order of these updates
    half_edge -> setNeighbors(twin_prev, twin, half_edge_prev -> vertex(), e0, twin_prev -> face());
    twin_prev -> next() = half_edge_next;
    half_edge_next -> next() = half_edge;
    half_edge_next -> face() = twin_prev -> face();
    twin_prev -> face() -> halfedge() = half_edge;
    half_edge_next -> vertex() -> halfedge() = half_edge_next;

    // Set lower triangle face
    twin -> setNeighbors(half_edge_prev, half_edge, twin_prev -> vertex(), e0, half_edge_prev -> face());
    half_edge_prev -> next() = twin_next;
    twin_next -> next() = twin;
    twin_next -> face() = half_edge_prev -> face();
    half_edge_prev -> face() -> halfedge() = twin;
    twin_next -> vertex() -> halfedge() = twin_next;

    return EdgeIter();
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    // if (e0 -> isBoundary()) return e0 -> halfedge() -> vertex();

    // // Get existing halfedges and faces
    // HalfedgeIter half_edge = e0 -> halfedge(), half_edge_next = half_edge -> next(), half_edge_prev = half_edge_next -> next();
    // HalfedgeIter twin = half_edge -> twin(), twin_next = twin -> next(), twin_prev = twin_next -> next();
    // FaceIter upper_left_face = half_edge -> face(), upper_right_face = twin -> face();

    // // Create new halfedges, edges, and faces
    // // TODO: Use vectors or maps (to give name to indices)
    // HalfedgeIter boundary_halfedges[] = {half_edge_next, half_edge_prev, twin_next, twin_prev};
    // HalfedgeIter outward_halfedges[] = {newHalfedge(), newHalfedge(), newHalfedge(), newHalfedge()};
    // HalfedgeIter inward_e0, newEdge(), newEdge(), newEdge()};
    // FaceIter faces[] = {upper_left_face, upper_right_face, newFace(), newFace()};

    // // Create new vertex
    // VertexIter new_vertex = newVertex();
    // new_vertex -> position = (half_edge -> vertex() -> position + twin -> vertex() -> position) / 2.0;
    // new_vertex -> halfedge() = outward_halfedges[0];

    
    // // Updating variables for each face 
    // // TODO: unroll for loop
    // for (int i = 0; i < 4; i++) {
    //   // TODO: reorder updates
    //   // Updating new halfedges
    //   outward_halfedges[i] -> setNeighbors(boundary_halfedges[i], inward_halfedges[(i + 3) % 4], new_vertex, edges[i], faces[i]);
    //   inward_halfedges[i] -> setNeighbors(outward_halfedges[i], outward_halfedges[(i + 1) % 4], boundary_halfedges[(i + 1) % 4] -> vertex(), edges[(i + 1) % 4], faces[i]);
    //   boundary_halfedges[i] -> next() = inward_halfedges[i];
    //   boundary_halfedges[i] -> face() = faces[i];

    //   // Updating new edges and faces
    //   edges[i] -> halfedge() = outward_halfedges[i];
    //   faces[i] -> halfedge() = outward_halfedges[i];

    //   // Updating old vertex
    //   inward_halfedges[i] -> vertex() -> halfedge() = inward_halfedges[i];

    // }

    // // Remove old halfedges
    // deleteHalfedge(half_edge);
    // deleteHalfedge(twin);

    
    // return new_vertex

    // TODO: Change order of declarations
    // view e0 as c_m_c edge
    if (e0 -> isBoundary()) return e0 -> halfedge() -> vertex();
    
    // Inner Halfedges
    HalfedgeIter m_c = e0 -> halfedge();
    HalfedgeIter c_a = m_c -> next();
    HalfedgeIter a_m = c_a -> next();
    HalfedgeIter c_m = m_c -> twin();
    HalfedgeIter m_d = c_m -> next();
    HalfedgeIter d_c = m_d -> next();

    // Outer Halfedges
    HalfedgeIter a_c = c_a -> twin();
    HalfedgeIter b_a = a_m -> twin();
    HalfedgeIter d_b = m_d -> twin();
    HalfedgeIter c_d = d_c -> twin();

    // Vertices
    VertexIter b = m_c -> vertex();  // first vertex at endpoint of previous edge
    VertexIter c = c_m -> vertex();  // second vertex at opposite endpoint of previous edge
    VertexIter a = a_m -> vertex();  // first vertex at endpoint of flipped edge
    VertexIter d = d_c -> vertex();  // second vertex at opposite endpoint of flipped edge

    // Edges
    EdgeIter a_c_a = c_a -> edge();
    EdgeIter a_b_a = a_m -> edge();
    EdgeIter b_d_b = m_d -> edge();
    EdgeIter c_d_c = d_c -> edge();

    // Faces
    FaceIter upper_left_face = m_c -> face();       // face 1
    FaceIter upper_right_face = c_m -> face();       // face 2

    // --------------- m == b before this line ------------
  
    // New inner halfedges for lower part of mesh split
    HalfedgeIter b_m = newHalfedge();
    HalfedgeIter m_a = newHalfedge();
    HalfedgeIter a_b = newHalfedge();
    HalfedgeIter m_b = newHalfedge();
    HalfedgeIter b_d = newHalfedge();
    HalfedgeIter d_m = newHalfedge();

    // New vertex
    VertexIter m = newVertex();
    
    // New edges
    EdgeIter a_m_a = newEdge();
    EdgeIter b_m_b = newEdge();
    EdgeIter d_m_d = newEdge();

    // New faces
    FaceIter bottom_left_face = newFace();
    FaceIter bottom_right_face = newFace();


    // --------------- UPDATES ------------------

    // Halfedges
    m_c -> setNeighbors(c_a, c_m, m, e0, upper_left_face);
    c_a -> setNeighbors(a_m, a_c, c, a_c_a, upper_left_face);
    a_m -> setNeighbors(m_c, m_a, a, a_m_a, upper_left_face);
    c_m -> setNeighbors(m_d, m_c, c, e0, upper_right_face);
    m_d -> setNeighbors(d_c, d_m, m, d_m_d, upper_right_face);
    d_c -> setNeighbors(c_m, c_d, d, c_d_c, upper_right_face);
    a_c -> setNeighbors(a_c -> next(), c_a, a, a_c_a, a_c -> face());
    b_a -> setNeighbors(b_a -> next(), a_b, b, a_b_a, b_a -> face());
    d_b -> setNeighbors(d_b -> next(), b_d, d, b_d_b, d_b -> face());
    c_d -> setNeighbors(c_d -> next(), d_c, c, c_d_c, c_d -> face());
    b_m -> setNeighbors(m_a, m_b, b, b_m_b, bottom_left_face);
    m_a -> setNeighbors(a_b, a_m, m, a_m_a, bottom_left_face);
    a_b -> setNeighbors(b_m, b_a, a, a_b_a, bottom_left_face);
    m_b -> setNeighbors(b_d, b_m, m, b_m_b, bottom_right_face);
    b_d -> setNeighbors(d_m, d_b, b, b_d_b, bottom_right_face);
    d_m -> setNeighbors(m_b, m_d, d, d_m_d, bottom_right_face);

    // Vertices
    // Update vertex position to edge midpoint
    m -> position = (b -> position + c -> position) / 2.0;
    // Update vertex status to isNew
    m -> isNew = true;
    // Update vertice halfedges          
    b -> halfedge() = b_m;
    c -> halfedge() = c_a;
    a -> halfedge() = a_b;
    d -> halfedge() = d_c;
    m -> halfedge() = m_c;
    
    // Edges
    e0 -> halfedge() = m_c;
    a_c_a -> halfedge() = c_a;
    a_b_a -> halfedge() = a_b;
    b_d_b -> halfedge() = b_d;
    c_d_c -> halfedge() = d_c;
    a_m_a -> halfedge() = a_m;
    b_m_b -> halfedge() = b_m;
    d_m_d -> halfedge() = m_d;
    
    // Set isNew to true for edges touching the new vertex 
    e0 -> isNew = false; // half of original edge
    b_m_b -> isNew = false; // half of original edge
    a_m_a -> isNew = true; // bisecting edge half
    d_m_d -> isNew = true; // bisecting edge half

    // Faces
    upper_left_face -> halfedge() = m_c;
    upper_right_face -> halfedge() = c_m;
    bottom_left_face -> halfedge() = b_m;
    bottom_right_face -> halfedge() = m_b;
    
    // Return the new vertex
    return m;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd();e++) {
            // Set isNew to false for all old edges
            e -> isNew = false;
            
            
            // Inner Halfedges
            HalfedgeIter h0 = e -> halfedge();
            HalfedgeIter h1 = h0 -> next();
            HalfedgeIter h2 = h1 -> next();
            HalfedgeIter h3 = h0 -> twin();
            HalfedgeIter h4 = h3 -> next();
            HalfedgeIter h5 = h4 -> next();

            // Vertices
            VertexIter b = h0 -> vertex();  // first vertex at endpoint of previous edge
            VertexIter a = h3 -> vertex();  // second vertex at opposite endpoint of previous edge
            VertexIter c = h2 -> vertex();  // first vertex at endpoint of flipped edge
            VertexIter d = h5 -> vertex();  // second vertex at opposite endpoint of flipped edge

            // Precompute and store position of new vertices in newPosition field of old edges
            e -> newPosition = 3.0 / 8.0 * (b -> position + a -> position) + 1.0 / 8.0 * (c -> position + d -> position);
            
    }
    
    
    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) { 
            // set isNew to false for all old vertices
            v -> isNew = false;
            
            // Set newPosition of old vertices as weighted combination of surrounding old vertices
            HalfedgeIter h = v -> halfedge();
            Vector3D original_neighbor_position_sum(0,0,0);
            do {
                original_neighbor_position_sum += h -> twin() -> vertex() -> position;
                h = h -> twin() -> next();

            } while (h != v -> halfedge());
            // TODO: why float for degree
            float n = (float) v -> degree();
            // TODO: rewrite if condition
            float u = (n == 3.0) ? (3.0 / 16.0) : (3.0 / (8.0 * n));
            
            // Set newPostion field for old vertices
            v -> newPosition = (1.0 - n * u) * v -> position + u * original_neighbor_position_sum;
            
    }

    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            VertexIter v1 = e -> halfedge() -> vertex();
            VertexIter v2 = e -> halfedge() -> twin() -> vertex();

            // Split only if its an old edge (prevents infinite loop)
            if(!v1 -> isNew && !v2 -> isNew) {
                VertexIter v = mesh.splitEdge(e);
                v -> newPosition = e -> newPosition; // set new vertex position
            }
    }
    
    // 4. Flip any new edge that connects an old and new vertex.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            VertexIter v1 = e->halfedge()->vertex();
            VertexIter v2 = e->halfedge()->twin()->vertex();

            if(e->isNew && ((v1->isNew && !v2->isNew) || (!v1->isNew && v2->isNew))) {
                mesh.flipEdge(e);  // automatically updates newPosition field in new vertices
            }
    }

    // 5. Copy the new vertex positions into final Vertex::position.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
            v->position = v->newPosition;
            v->isNew = false;
    } 

  }
}
