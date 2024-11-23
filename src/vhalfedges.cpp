#include <CGAL/Simple_cartesian.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Surface_mesh.h>
#include <iostream>
#include <vector>
#include <map>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Delaunay_triangulation_2<Kernel> Delaunay_2;
typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef Mesh::Vertex_index Vertex_index;
typedef Mesh::Halfedge_index Halfedge_index;

int main() {
    // Define 2D points
    std::vector<Point_2> points_2d = {
        Point_2(0, 0),
        Point_2(1, 0),
        Point_2(0, 1),
        Point_2(-1, 0),
        Point_2(0, -1)
    };

    // Perform 2D Delaunay triangulation
    Delaunay_2 dt;
    dt.insert(points_2d.begin(), points_2d.end());

    // Initialize 3D mesh and a map to track vertices
    Mesh mesh;
    std::map<Delaunay_2::Vertex_handle, Vertex_index> vertex_map;

    // Add vertices to the mesh, converting 2D points to 3D points with z=0
    for (auto v = dt.finite_vertices_begin(); v != dt.finite_vertices_end(); ++v) {
        Point_3 point3D(v->point().x(), v->point().y(), 0.0);
        vertex_map[v] = mesh.add_vertex(point3D);
    }

    // Add faces to the mesh
    for (auto f = dt.finite_faces_begin(); f != dt.finite_faces_end(); ++f) {
        Vertex_index v0 = vertex_map[f->vertex(0)];
        Vertex_index v1 = vertex_map[f->vertex(1)];
        Vertex_index v2 = vertex_map[f->vertex(2)];
        mesh.add_face(v0, v1, v2);
    }

    // Iterate over vertices and format the output
    for (auto v : mesh.vertices()) {
        const Point_3& point = mesh.point(v);
        std::cout << "Vertex (" << point.x() << ", " << point.y() << "):\n";

        // Get starting half-edge
        Halfedge_index start_edge = mesh.halfedge(v);
        start_edge = mesh.next(start_edge);
        if (start_edge == Mesh::null_halfedge()) {
            std::cout << "  No outgoing half-edges\n";
            continue;
        }

        // Iterate over outgoing half-edges
        Halfedge_index edge = start_edge;
        do {
            Vertex_index target_vertex = mesh.target(edge);
            const Point_3& target_point = mesh.point(target_vertex);
            std::cout << "  Half-edge to Vertex (" << target_point.x() << ", " << target_point.y() << ")\n";
            edge = mesh.next(mesh.opposite(edge)); // Move to next outgoing half-edge
        } while (edge != start_edge); // Complete loop around vertex
    }

    return 0;
}
