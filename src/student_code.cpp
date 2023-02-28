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
    vector<Vector2D> intermediate_points;
    for (int i = 0; i < points.size() - 1; i++)
      intermediate_points.push_back((1.0 - t) * points[i] + t * points[i + 1]);
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
    vector<Vector3D> intermediate_points;
    for (int i = 0; i < points.size() - 1; i++)
      intermediate_points.push_back((1.0 - t) * points[i] + t * points[i + 1]);
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
    for (int i = 0; i < controlPoints.size(); i++)
      movingBezierCurve.push_back(evaluate1D(controlPoints[i], u));
    return evaluate1D(movingBezierCurve, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.

    Vector3D area_weighted_normal(0,0,0);
    HalfedgeCIter half_edge = halfedge();
    while (1) {
      Vector3D vertex_0 = position;
      Vector3D vertex_1 = half_edge -> next() -> vertex() -> position;
      Vector3D vertex_2 = half_edge -> next() -> next() -> vertex() -> position;
      area_weighted_normal += cross(vertex_1 - vertex_0, vertex_2 - vertex_0);
      half_edge = half_edge -> twin() -> next();
      if (half_edge == halfedge())  break;
    }
    return area_weighted_normal.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    if (e0 -> isBoundary()) return e0;

    HalfedgeIter half_edge = e0 -> halfedge(), half_edge_next = half_edge -> next(), half_edge_prev = half_edge_next -> next();
    HalfedgeIter twin = half_edge -> twin(), twin_next = twin -> next(), twin_prev = twin_next -> next();
    
    // Set upper triangle face, counter-clockwisely
    half_edge -> setNeighbors(twin_prev, twin, half_edge_prev -> vertex(), e0, twin_prev -> face());
    twin_prev -> next() = half_edge_next;
    half_edge_next -> next() = half_edge;
    twin_prev -> face() -> halfedge() = half_edge;
    half_edge_next -> vertex() -> halfedge() = half_edge_next;
    half_edge_next -> face() = twin_prev -> face();

    // Set lower triangle face
    twin -> setNeighbors(half_edge_prev, half_edge, twin_prev -> vertex(), e0, half_edge_prev -> face());
    half_edge_prev -> next() = twin_next;
    twin_next -> next() = twin;
    half_edge_prev -> face() -> halfedge() = twin;
    twin_next -> vertex() -> halfedge() = twin_next;
    twin_next -> face() = half_edge_prev -> face();

    return EdgeIter();
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.

    // view e0 as c_m_c edge
    if (e0 -> isBoundary()) return e0 -> halfedge() -> vertex();
    
    // --------------- Initialization ------------------
    // Inner Halfedges
    // upper part of mesh split
    HalfedgeIter m_c = e0 -> halfedge();
    HalfedgeIter c_a = m_c -> next();
    HalfedgeIter a_m = c_a -> next();
    HalfedgeIter c_m = m_c -> twin();
    HalfedgeIter m_d = c_m -> next();
    HalfedgeIter d_c = m_d -> next();
    // New inner halfedges for lower part of mesh split
    HalfedgeIter b_m = newHalfedge();
    HalfedgeIter m_a = newHalfedge();
    HalfedgeIter a_b = newHalfedge();
    HalfedgeIter m_b = newHalfedge();
    HalfedgeIter b_d = newHalfedge();
    HalfedgeIter d_m = newHalfedge();

    // Outer Halfedges
    HalfedgeIter a_c = c_a -> twin();
    HalfedgeIter b_a = a_m -> twin();
    HalfedgeIter d_b = m_d -> twin();
    HalfedgeIter c_d = d_c -> twin();

    // Vertices
    VertexIter a = a_m -> vertex();  
    VertexIter b = m_c -> vertex();  
    VertexIter d = d_c -> vertex();  
    VertexIter c = c_m -> vertex();  
    // New vertex
    VertexIter m = newVertex();

    // Edges
    EdgeIter a_c_a = c_a -> edge();
    EdgeIter a_b_a = a_m -> edge();
    EdgeIter b_d_b = m_d -> edge();
    EdgeIter c_d_c = d_c -> edge();
    // New edges
    EdgeIter a_m_a = newEdge();
    EdgeIter b_m_b = newEdge();
    EdgeIter d_m_d = newEdge();

    // Faces
    FaceIter upper_left_face = m_c -> face();       // face 1
    FaceIter upper_right_face = c_m -> face();       // face 2
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
    // Update vertex status to isNew
    m -> isNew = true;
    // Update vertex position to edge midpoint
    m -> position = (a -> position + d -> position) / 2.0;
    // Update vertice halfedges          
    m -> halfedge() = m_c;
    a -> halfedge() = a_b;
    b -> halfedge() = b_m;
    d -> halfedge() = d_c;
    c -> halfedge() = c_a;
    
    // Edges
    a_b_a -> halfedge() = a_b;
    b_d_b -> halfedge() = b_d;
    c_d_c -> halfedge() = d_c;
    a_c_a -> halfedge() = c_a;
    e0 -> halfedge() = m_c; //e0 = c_m_c
    a_m_a -> halfedge() = a_m;
    b_m_b -> halfedge() = b_m;
    d_m_d -> halfedge() = m_d;
    
    // Set isNew to true for edges touching the new vertex 
    e0 -> isNew = false; // half of original edge
    a_m_a -> isNew = true; // bisecting edge half
    b_m_b -> isNew = false; // half of original edge
    d_m_d -> isNew = true; // bisecting edge half

    // Faces
    upper_left_face -> halfedge() = m_c;
    bottom_left_face -> halfedge() = b_m;
    bottom_right_face -> halfedge() = m_b;
    upper_right_face -> halfedge() = c_m;
    
    // Return the new vertex
    return m;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // Step A: Compute the positions of both new and old vertices using the original mesh. 
    // We want to perform these computations before subdivision because traversing a coarse mesh is 
    // much easier than traversing a subdivided mesh with more elements.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) { 
      // Set newPosition of old vertices as weighted combination of surrounding old vertices
      HalfedgeIter h = v -> halfedge();
      Vector3D original_neighbor_position_sum(0,0,0);
      while (1) {
        original_neighbor_position_sum += h -> twin() -> vertex() -> position;
        h = h -> twin() -> next();
        if (h == v -> halfedge()) break;
      }
      float n = (float) v -> degree(), u;
      if (n == 3.0) u = (3.0 / 16.0);
      else  u = (3.0 / (8.0 * n));
      
      // Set newPostion field for old vertices
      v -> newPosition = (1.0 - n * u) * v -> position + u * original_neighbor_position_sum;
      // set isNew to false for all old vertices
      v -> isNew = false;
    }

    // Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd();e++) {
      // Inner Halfedges
      HalfedgeIter h0 = e -> halfedge();
      HalfedgeIter h1 = h0 -> next();
      HalfedgeIter h2 = h1 -> next();
      HalfedgeIter h3 = h0 -> twin();
      HalfedgeIter h4 = h3 -> next();
      HalfedgeIter h5 = h4 -> next();

      // Vertices
      VertexIter a = h3 -> vertex();  // second vertex at opposite endpoint of previous edge
      VertexIter b = h0 -> vertex();  // first vertex at endpoint of previous edge
      VertexIter d = h5 -> vertex();  // second vertex at opposite endpoint of flipped edge
      VertexIter c = h2 -> vertex();  // first vertex at endpoint of flipped edge

      // Precompute and store position of new vertices in newPosition field of old edges
      e -> newPosition = 3.0 / 8.0 * (a -> position + b -> position) + 1.0 / 8.0 * (c -> position + d -> position);
      // Set isNew to false for all old edges
      e -> isNew = false;    
    }

    // Step B: Subdivide the original mesh via edge splits and flips as described.
    // Split every edge in the mesh, in any order.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      VertexIter v1 = e -> halfedge() -> vertex(), v2 = e -> halfedge() -> twin() -> vertex();

      // Split only if its an old edge (prevents infinite loop)
      if (!v1 -> isNew && !v2 -> isNew) mesh.splitEdge(e) -> newPosition = e -> newPosition; // set new vertex position
    }
    
    // Flip any new edge that connects an old and new vertex.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      VertexIter v1 = e -> halfedge() -> vertex(), v2 = e -> halfedge() -> twin() -> vertex();
      if (e -> isNew && ((v1 -> isNew && !v2 -> isNew) || (!v1 -> isNew && v2 -> isNew))) mesh.flipEdge(e);  // automatically updates newPosition field in new vertices
    }

    // Step C: Update all vertex positions in the subdivided mesh using the values already computed.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      v -> position = v -> newPosition;
      v -> isNew = false;
    } 

  }
}
