#include "mesh.hpp"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/io.h>
#include <fstream>
#include <iostream>

//typedef CGAL::Simple_cartesian<double> Kernel;
//typedef Kernel::Point_3 Point;
//typedef CGAL::Surface_mesh<Point> SurfaceMesh;

int main() {
    // Create an empty surface mesh
    Surface_mesh mesh=read_OFFfile("../../data/bunny2.off");
    
    std::cout << " Hola " << std::endl;

    // Find the mapping of longest edges
    auto is_longest_edge = find_longest_edges_mapping(mesh);
    auto is_frontier_edge = find_frontier_edges_mapping(mesh, is_longest_edge);
    auto is_seed_edge = find_seed_edges_mapping(mesh, is_longest_edge);
    auto is_Delta_normal_edge = find_Delta_Normal_edges_mapping(mesh, is_frontier_edge, is_seed_edge, 20);
    
    std::cout << "******************************* "  << ":\n";
    // Write longest edges to a .obj file
    write_mapped_edges_to_obj(mesh, is_frontier_edge, "../../data/frontier_edges.obj");
    write_mapped_edges_to_off(mesh, is_frontier_edge,"../../data/frontier_edges.off");
    write_mapped_edges_to_obj(mesh, is_seed_edge, "../../data/seed_edges.obj");
    write_mapped_edges_to_off(mesh, is_seed_edge,"../../data/seed_edges.off");
    write_mapped_edges_to_off(mesh, is_seed_edge,"../../data/Delta_normal_edges.off");
    print_triangle_edge_lengths(mesh, is_longest_edge, 4);
    bool tf = triangle_wnolongest(mesh, is_longest_edge);
    Surface_mesh PolMesh = Polylla(mesh, is_seed_edge, is_frontier_edge);
    std::ofstream outvor("../../data/Polylla.off");
    CGAL::IO::write_OFF(outvor, PolMesh); // Correct modern usage of write_OFF()
    std::cout << "Polylla surface mesh written to Polylla.off\n"; 
    
    write_NRP_seed_edges_file(mesh, "../../data/NRP_seed_edges.obj");
    write_obj_NRP_edges_file("../../data/NRP_edges.obj");


    return 0;
}