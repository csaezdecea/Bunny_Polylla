#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Kernel/global_functions.h> // For circumcenter and cross_product functions
#include <vector>
#include <iostream>
#include <algorithm>
#include <unordered_map>

#include <CGAL/Surface_mesh_default_triangulation_3.h>


typedef CGAL::Surface_mesh_default_triangulation_3 Tr;
typedef Tr::Geom_traits GT;
typedef GT::Point_3 Point_3;
typedef GT::Vector_3 Vector_3;

typedef CGAL::Surface_mesh<Point_3> Surface_mesh;
typedef Surface_mesh::Vertex_index Vertex_index;
typedef Surface_mesh::Face_index Face_index;
typedef Surface_mesh::Halfedge_index Halfedge_index;
typedef Surface_mesh::Edge_index Edge_index;

std::unordered_map<Halfedge_index, bool> is_NRP_seed_edge;
std::vector<std::vector<Point_3>> NRP_Polygons;


// Function to normalize a vector
Vector_3 normalize_vector(const Vector_3& vec) {
    double norm = std::sqrt(CGAL::to_double(vec.squared_length()));
    return vec / norm;
}


// Function to compute the normal of a triangle
Vector_3 triangle_normal(const Halfedge_index he, const Surface_mesh& mesh) {
    auto he1 = he;
    auto he2 = mesh.next(he1);
    // Get the points at each vertex of the triangle
    Point_3 p0 = mesh.point(source(he1, mesh));
    Point_3 p1 = mesh.point(target(he1, mesh));
    Point_3 p2 = mesh.point(target(he2, mesh));
    Vector_3 u = p1 - p0;
    Vector_3 v = p2 - p0;
    Vector_3 normal = CGAL::cross_product(u, v);
    return normalize_vector(normal);
}



// Function to remove degenerate faces
void remove_degenerate_faces(Surface_mesh& sm) {
    std::vector<Face_index> faces_to_remove;

    // Check for degenerate faces (e.g., faces with collinear points)
    for (auto f : sm.faces()) {
        auto halfedge = sm.halfedge(f);
        auto v0 = sm.target(halfedge);
        auto v1 = sm.target(sm.next(halfedge));
        auto v2 = sm.target(sm.next(sm.next(halfedge)));

        // Get points of the vertices
        const Point_3& p0 = sm.point(v0);
        const Point_3& p1 = sm.point(v1);
        const Point_3& p2 = sm.point(v2);

        // Check if the triangle is degenerate
        if (CGAL::collinear(p0, p1, p2)) {
            faces_to_remove.push_back(f);
        }
    }

    // Remove degenerate faces
    for (const auto& f : faces_to_remove) {
        sm.remove_face(f);
    }
}

// Function to remove isolated vertices
void remove_isolated_vertices(Surface_mesh& sm) {
    std::vector<Vertex_index> isolated_vertices;
    // Identify isolated vertices
    for (Vertex_index v : sm.vertices()) {
        if (sm.degree(v) == 0) {
            isolated_vertices.push_back(v);
        }
    }
    // Remove isolated vertices
    for (const auto& v : isolated_vertices) {
        sm.remove_vertex(v);
    }
}


// Function to check for non-manifold edges
bool has_non_manifold_edges(const Surface_mesh& sm) {
    std::map<std::pair<Vertex_index, Vertex_index>, int> edge_count;
    // Count occurrences of each edge
    for (auto f : sm.faces()) {
        auto halfedge = sm.halfedge(f);
        do {
            Vertex_index v1 = sm.target(halfedge);
            Vertex_index v2 = sm.target(sm.next(halfedge));
            if (v1 > v2) std::swap(v1, v2); // Order vertices
            edge_count[{v1, v2}]++;
            halfedge = sm.next(halfedge);
        } while (halfedge != sm.halfedge(f));
    }
    // Check for non-manifold edges
    for (const auto& edge : edge_count) {
        if (edge.second > 2) { // An edge shared by more than 2 faces
            return true;
        }
    }
    return false;
}


// Function to preprocess the mesh
void preprocess_mesh(Surface_mesh& sm) {
    // Step 1: Remove degenerate faces
    std::cout << "Removing degenerate faces..." << std::endl;
    remove_degenerate_faces(sm);

    // Step 2: Remove isolated vertices
    std::cout << "Removing isolated vertices..." << std::endl;
    remove_isolated_vertices(sm);

    // Step 3: Check for non-manifold edges
    if (has_non_manifold_edges(sm)) {
        std::cerr << "Warning: Mesh contains non-manifold edges!" << std::endl;
        // Optionally handle non-manifold edges
    }
}


// Function to read the mesh from a .off file
Surface_mesh read_OFFfile(const std::string& filename) {
    Surface_mesh mesh;
    // Specify the .off file path
    //const std::string filename = "../../data/bunny2.off";
    std::ifstream input(filename);    
    // Check if the file opened successfully
    if (!input) {
        std::cerr << "Error: Cannot open file " << filename << std::endl;
        return mesh;
    }
    // Read the .off file into the SurfaceMesh using CGAL::IO::read_OFF
    if (!CGAL::IO::read_OFF(input, mesh)) {
        std::cerr << "Error: Cannot read .off file into the mesh" << std::endl;
        return mesh;
    }
    std::cout << "Mesh loaded successfully with " 
              << mesh.num_vertices() << " vertices and " 
              << mesh.num_faces() << " faces." << std::endl;
    return mesh;
}


// Write in an .obj file a random sample of edges from a Surface mesh
void write_rdedges(Surface_mesh& mesh, const std::string& filename) {
// Collect all edges
    std::vector<Edge_index> edges;
    for (Edge_index e : mesh.edges()) {
        edges.push_back(e);
    }
    // Randomly select 10 unique edges
    std::srand(static_cast<unsigned>(std::time(0)));
    std::random_shuffle(edges.begin(), edges.end());
    std::vector<Edge_index> selected_edges(edges.begin(), edges.begin() + 10);
     // Save selected edges to .obj file
    std::ofstream output(filename);
    if (!output) {
        std::cerr << "Error: Cannot open file to write selected segments" << std::endl;
        return;
    }
    // Write vertices for the selected segments in .obj format
    std::vector<Point_3> points;
    for (const auto& e : selected_edges) {
        points.push_back(mesh.point(source(e, mesh)));
        points.push_back(mesh.point(target(e, mesh)));
    }
    for (const auto& p : points) {
        output << "v " << p.x() << " " << p.y() << " " << p.z() << std::endl;
    }
    // Write edge connections (lines) in .obj format
    for (int i = 0; i < 10; ++i) {
        output << "l " << 2 * i + 1 << " " << 2 * i + 2 << std::endl;
    }
    output.close();
    std::cout << "Selected segments saved to selected_segments.obj" << std::endl;
}


//print triangle edge lengths marking the longest edge trough the use of is_longest_edge.
void print_triangle_edge_lengths(const Surface_mesh& mesh,
                                 const std::unordered_map<Halfedge_index, bool>& is_longest_edge, int nfaces = 1000000) {
    // Loop through each face in the mesh
    int trcount = 0;
    for (auto f : faces(mesh)) {
        ++trcount;
        if (trcount > nfaces) break;
        auto he = halfedge(f, mesh);
        // Get the three halfedges of the triangle
        auto he1 = he;
        auto he2 = next(he1, mesh);
        auto he3 = next(he2, mesh);
        // Get the points at each vertex of the triangle
        Point_3 p1 = mesh.point(source(he1, mesh));
        Point_3 p2 = mesh.point(target(he1, mesh));
        Point_3 p3 = mesh.point(target(he2, mesh));
        // Print the coordinates of each point
        std::cout << "Triangle " << f << ":\n";
        std::cout << "  Point 1: (" << p1.x() << ", " << p1.y() << ", " << p1.z() << ")\n";
        std::cout << "  Point 2: (" << p2.x() << ", " << p2.y() << ", " << p2.z() << ")\n";
        std::cout << "  Point 3: (" << p3.x() << ", " << p3.y() << ", " << p3.z() << ")\n";
        // Compute the squared lengths of each edge
        double length1 = CGAL::squared_distance(p1, p2);
        double length2 = CGAL::squared_distance(p2, p3);
        double length3 = CGAL::squared_distance(p3, p1);
        // Check which edges are marked as longest in the mapping
        bool longest1 = is_longest_edge.at(he1);
        bool longest2 = is_longest_edge.at(he2);
        bool longest3 = is_longest_edge.at(he3);
        // Print the lengths and longest edge info
        std::cout << "  Edge 1 (Point 1 to Point 2): Length = " << std::sqrt(length1) 
                  << (longest1 ? " [Longest]" : "") << "\n";
        std::cout << "  Edge 2 (Point 2 to Point 3): Length = " << std::sqrt(length2)
                  << (longest2 ? " [Longest]" : "") << "\n";
        std::cout << "  Edge 3 (Point 3 to Point 1): Length = " << std::sqrt(length3)
                  << (longest3 ? " [Longest]" : "") << "\n";
    }
}



//Checks if there is a triangle with not longest edge
bool triangle_wnolongest(const Surface_mesh& mesh,
                                 const std::unordered_map<Halfedge_index, bool>& is_longest_edge) {
    // Loop through each face in the mesh
    for (auto f : faces(mesh)) {
        auto he = halfedge(f, mesh);
        // Get the three halfedges of the triangle
        auto he1 = he;
        auto he2 = next(he1, mesh);
        auto he3 = next(he2, mesh);
        // Check which edges are marked as longest in the mapping
        bool longest1 = is_longest_edge.at(he1);
        bool longest2 = is_longest_edge.at(he2);
        bool longest3 = is_longest_edge.at(he3);
        // Print the lengths and longest edge info
        if (!longest1 & !longest2 & !longest3) {
            std::cout << "Triangle " << f << " " << "has not longest edge \n";
            return true;

        }
    }
    return false;
}


// Method to find the longest edges in each triangle and store in the mapping
std::unordered_map<Halfedge_index, bool> find_longest_edges_mapping(const Surface_mesh& mesh) {
    std::unordered_map<Halfedge_index, bool> is_longest_edge;
    int trcount = 0;
    // Initialize all half-edges to false
    for (auto he : halfedges(mesh)) {
        is_longest_edge[he] = false;
    }
    // Loop through each face in the mesh
    for (auto f : faces(mesh)) {
        ++trcount;
        auto he = halfedge(f, mesh);
        // Get the three halfedges of the triangle
        auto he1 = he;
        auto he2 = next(he1, mesh);
        auto he3 = next(he2, mesh);
        // Get the points at each vertex of the triangle
        Point_3 p1 = mesh.point(source(he1, mesh));
        Point_3 p2 = mesh.point(target(he1, mesh));
        Point_3 p3 = mesh.point(target(he2, mesh));
        // Compute the squared lengths of each edge
        double length1 = CGAL::squared_distance(p1, p2);
        double length2 = CGAL::squared_distance(p2, p3);
        double length3 = CGAL::squared_distance(p3, p1);
        // Determine the longest edge
        Surface_mesh::Halfedge_index longest_he;
        if (length1 >= length2 && length1 >= length3) {
            longest_he = he1;
        } else if (length2 >= length1 && length2 >= length3) {
            longest_he = he2;
        } else {
            longest_he = he3;
        }
        is_longest_edge[longest_he] = true;
        if(trcount < 1) {
            std::cout << "Marking half-edge " << is_longest_edge[longest_he] << " " << is_longest_edge[mesh.opposite(longest_he)] <<" as longest\n";
            print_triangle_edge_lengths(mesh, is_longest_edge, 10);
        }
    }
    return is_longest_edge;
}


// Method to find the frontier edges and store in the mapping
std::unordered_map<Halfedge_index, bool> find_frontier_edges_mapping(const Surface_mesh& mesh, 
const std::unordered_map<Halfedge_index, bool>& is_longest_edge) {
    std::unordered_map<Halfedge_index, bool> is_frontier_edge;
    int trcount = 0;
    // Initialize all half-edges to false
    for (auto he : halfedges(mesh)) is_frontier_edge[he] = false;
    for (auto he : halfedges(mesh)) {
        auto twinhe = opposite(he, mesh);
        bool is_not_longest_edge = (!is_longest_edge.at(he) & !is_longest_edge.at(twinhe));
        if (is_not_longest_edge | mesh.is_border(he)) {
            is_frontier_edge[he] = true;
            is_frontier_edge[twinhe] = true;
        }
    }
    return is_frontier_edge;
}


// Method to find the seed edges and store in the mapping
std::unordered_map<Halfedge_index, bool> find_seed_edges_mapping(const Surface_mesh& mesh, 
const std::unordered_map<Halfedge_index, bool>& is_longest_edge) {
    std::unordered_map<Halfedge_index, bool> is_seed_edge;
    int trcount = 0;
    // Initialize all half-edges to false
    for (auto he : halfedges(mesh)) is_seed_edge[he] = false;
    for (auto he : halfedges(mesh)) {
        auto twinhe = opposite(he, mesh);
        bool is_terminal_edge = (is_longest_edge.at(he) & is_longest_edge.at(twinhe) & 
        !mesh.is_border(he) & !mesh.is_border(twinhe));
        bool is_terminal_border_edge = (is_longest_edge.at(he) & (mesh.is_border(he) |  mesh.is_border(twinhe))); 
        //| (is_longest_edge.at(twinhe)  & (mesh.is_border(twinhe) | mesh.is_border(he)));
        if ((is_terminal_edge | is_terminal_border_edge) & !is_seed_edge.at(twinhe)) {
            is_seed_edge[he] = true;
        }
    }
    return is_seed_edge;
}


// Method to find the Delta Normal edges and store in the mapping
std::unordered_map<Halfedge_index, bool> find_Delta_Normal_edges_mapping(const Surface_mesh& mesh, 
std::unordered_map<Halfedge_index, bool>& is_frontier_edge, std::unordered_map<Halfedge_index, bool>& is_seed_edge, double critical_angle) {
    std::unordered_map<Halfedge_index, bool> is_Delta_Normal_edge;
    double pi = atan(1)*4;
    std::cout << pi << std::endl;
    for (auto he : halfedges(mesh)) is_Delta_Normal_edge[he] = false;
    for (auto he : halfedges(mesh)) {
        auto twinhe = mesh.opposite(he);
        auto triangle_1 = mesh.face(he);
        auto triangle_2 = mesh.face (twinhe);
        Vector_3 normal1 = triangle_normal(he, mesh);
        Vector_3 normal2 = triangle_normal(twinhe, mesh);
        double cosAng = normal1 * normal2;
        double cosCrit = cos(critical_angle*pi/180);
        if (cosAng < cosCrit) {
            is_Delta_Normal_edge[he] = true;
            is_Delta_Normal_edge[twinhe] = true;
        }  
    }
    return is_Delta_Normal_edge;
}



// Method to write mapped edges to a .obj file
void write_mapped_edges_to_obj(const Surface_mesh& mesh, const std::unordered_map<Halfedge_index, bool>& is_mapped_edge, const std::string& filename) {
    std::ofstream output(filename);
    if (!output) {
        std::cerr << "Error: Cannot write to .obj file" << std::endl;
        return;
    }
    // Map to store vertex indices for unique vertices
    std::unordered_map<Point_3, int> vertex_map;
    int vertex_index = 1;
    // First, write unique vertices to the file
    for (auto he : mesh.halfedges()) {
        if (is_mapped_edge.at(he)) {
            Point_3 p1 = mesh.point(source(he, mesh));
            Point_3 p2 = mesh.point(target(he, mesh));
            // Check if each vertex is already added to the .obj file
            if (vertex_map.find(p1) == vertex_map.end()) {
                vertex_map[p1] = vertex_index++;
                output << "v " << p1 << "\n";
            }
            if (vertex_map.find(p2) == vertex_map.end()) {
                vertex_map[p2] = vertex_index++;
                output << "v " << p2 << "\n";
            }
        }
    }
    // Next, write edges using the vertex indices
    for (auto he : halfedges(mesh)) {
        if (is_mapped_edge.at(he)) {
            Point_3 p1 = mesh.point(source(he, mesh));
            Point_3 p2 = mesh.point(target(he, mesh));
            // Write the edge using the vertex indices
            output << "l " << vertex_map[p1] << " " << vertex_map[p2] << "\n";
        }
    }
    std::cout << "Longest edges written to " << filename << std::endl;
}
 


//Write a file with mapped edges in an .off format
void write_mapped_edges_to_off(const Surface_mesh& mesh, const std::unordered_map<Halfedge_index, 
bool>& is_mapped_edge, const std::string& filename) {
    std::ofstream output(filename);
    if (!output) {
        std::cerr << "Error: Cannot write to .off file" << std::endl;
        return;
    }
    // Map to store vertex indices for unique vertices
    std::unordered_map<Point_3, int> vertex_map;
    int vertex_index = 0;
    // Vector to store unique vertices and edges
    std::vector<Point_3> vertices;
    std::vector<std::pair<int, int>> edges;
    // Set to track unique edges (using a sorted pair of vertex indices)
    std::unordered_set<std::pair<int, int>, boost::hash<std::pair<int, int>>> unique_edges;
    for (auto he : halfedges(mesh)) {
        if (is_mapped_edge.at(he)) {
            // Determine canonical ordering of vertices for unique edge tracking
            auto v1 = source(he, mesh);
            auto v2 = target(he, mesh);
            if (v1 > v2) std::swap(v1, v2);  // Ensure consistent ordering
            // Check if this edge has already been processed
            auto edge_pair = std::make_pair(v1, v2);
            if (unique_edges.find(edge_pair) == unique_edges.end()) {
                unique_edges.insert(edge_pair);
                // Retrieve or assign indices for the edge vertices
                Point_3 p1 = mesh.point(v1);
                Point_3 p2 = mesh.point(v2);
                // Assign indices if vertices are new
                if (vertex_map.find(p1) == vertex_map.end()) {
                    vertex_map[p1] = vertex_index++;
                    vertices.push_back(p1);
                }
                if (vertex_map.find(p2) == vertex_map.end()) {
                    vertex_map[p2] = vertex_index++;
                    vertices.push_back(p2);
                }
                // Store the edge as a pair of vertex indices
                edges.emplace_back(vertex_map[p1], vertex_map[p2]);
            }
        }
    }
    // Write the OFF header
    output << "OFF\n";
    output << vertices.size() << " " << edges.size() << " 0\n";
    // Write vertices
    for (const auto& vertex : vertices) {
        output << vertex << "\n";
    }
    // Write edges
    for (const auto& edge : edges) {
        output << "2 " << edge.first << " " << edge.second << "\n";
    }
    std::cout << "Longest edges written to " << filename << std::endl;
}


//Function to find the next CW edge
Halfedge_index CWvertexEdge(const Halfedge_index he, const Surface_mesh& mesh) {
    Halfedge_index twinhe = mesh.opposite(he);
    return mesh.next(twinhe);
}


//Function to find the next CCW edge
Halfedge_index CCWvertexEdge(const Halfedge_index he, const Surface_mesh& mesh) {
    Halfedge_index prevhe = mesh.prev(he);
    return mesh.opposite(prevhe);
}


//From a seed edge (he) construct the list of points belonging to the associated polygon
std::vector<Point_3> Polygon_Construction(const Halfedge_index he, const Surface_mesh& mesh, 
const std::unordered_map<Halfedge_index, bool> is_frontier_edge, std::vector<Vertex_index>& polygon_indices) {
    std::vector<Point_3> polygon_vertices;
    Halfedge_index inihe = he;
    while (!is_frontier_edge.at(inihe)) {
        inihe = mesh.next_around_source(inihe);
    }
    Halfedge_index currhe = mesh.next(inihe);
    auto source_index = mesh.source(inihe);
    polygon_indices.push_back(source_index);
    polygon_vertices.push_back(mesh.point(source_index));
    while (inihe != currhe) {
        while (!is_frontier_edge.at(currhe)) {
            //currhe = CWvertexEdge(currhe,mesh);
            currhe = mesh.next_around_source(currhe);
        }
        if (inihe != currhe) {
            source_index = mesh.source(currhe);
            polygon_vertices.push_back(mesh.point(source_index));
            polygon_indices.push_back(source_index);
            if (inihe != currhe) currhe = mesh.next(currhe);
        }
    }
    return polygon_vertices;
}


//Inset Poligonal face in PolMesh, it also marks faces that belong to NR polygons
void Insert_Polygon(Surface_mesh& PolMesh,  const std::vector<Point_3>& Polig,
const std::vector<bool>& is_barrier, std::vector<Vertex_index>& polygon_indices,
std::vector<Vertex_index>& barrier_indices, bool NRP_face = false) {
    std::vector<Vertex_index> Polface;
    int npoints = Polig.size();
    for (int i=0; i < npoints; i++) {
        // Add poihts to Polmesh
        auto pol_point = Polig[i];
        Vertex_index vertex = PolMesh.add_vertex(pol_point);
        Polface.push_back(vertex);
        if (is_barrier[i]) barrier_indices.push_back(polygon_indices[i]);
    }
    if (true) {
        if (!NRP_face) Face_index faceind = PolMesh.add_face(Polface);
        if (NRP_face) NRP_Polygons.push_back(Polig);
        //is_NRP_face[faceind] = NRP_face;
    }
    
}


//Given a Polygon it checks if it is non simple (by finding a barrier tip)
bool is_nonsimple_polygon(const std::vector<Point_3>& Polig) {
    int npoints = Polig.size();
    for (int i=0; i < npoints; i++) {
        Point_3 p1 = Polig[i];
        Point_3 p3 = Polig[(i+2) % npoints];
        if (p1 == p3) return true;
    }
    return false;
}


//Mark the barrier points of a given Poligon
void find_barrier_vertices(std::vector<bool>& is_barrier, const std::vector<Point_3>& Polig) {
    int npoints = Polig.size();
    for (int i=0; i < npoints; i++) {
        Point_3 p1 = Polig[i];
        Point_3 p3 = Polig[(i+2) % npoints];
        if (p1 == p3) {
            is_barrier[(i+1) % npoints] = true;
            auto p2 = Polig[(i+1) % npoints];
            std::cout << "Point: [" << p2.x() << ", " << p2.y() << ", " << p2.z() << "]" << std::endl;
        }
    }
}


//Using the indices in a mesh of the barrier points prints their coordinates
void print_barrier_points(const Surface_mesh& mesh, const std::vector<Vertex_index>& barrier_indices) {
    std::ofstream output("../../data/barrier_points.dat");
    std::cout << "Barrier points: " << std::endl;
    for (const auto& vertex : barrier_indices) {
        Point_3 point = mesh.point(vertex);
        std::cout << "[" << point.x() << ", " << point.y() << ", " << point.z() << "]" << std::endl;
        output <<  point.x() << " " << point.y() << " " << point.z() << std::endl;
    }
    
}


//Obtains a halfedge that has as source a vertex given by a vertex index.
Halfedge_index edgeOfVertex(const Surface_mesh& mesh, Vertex_index vertex) {
    Halfedge_index incoming = mesh.halfedge(vertex); // Get an incoming half-edge to the vertex
    if (incoming == Surface_mesh::null_halfedge()) {
        return Halfedge_index(); // Return invalid if the vertex is isolated
    }
    // The outgoing half-edge is the opposite of the incoming half-edge
    Halfedge_index outgoing = mesh.opposite(incoming);
    return outgoing; // Return the outgoing half-edge
}


/**  From a list of barrier vertices it repairs non regular polygons and it generates the seed edges of the new
 generated regular polygons*/
std::vector<Halfedge_index>  RepairNRPolygon(std::vector<Vertex_index>& barrier_indices, 
const Surface_mesh& mesh, std::unordered_map<Halfedge_index,  bool>& is_frontier_edge) {
    std::vector<Halfedge_index> repaired_seed; 
    for (auto barrier : barrier_indices) {
        Point_3 point = mesh.point(barrier);
        //std::cout << "Barrier: [" << point.x() << ", " << point.y() << ", " << point.z() << "]" << std::endl;
        auto heini = edgeOfVertex(mesh, barrier);
        //point = mesh.point(mesh.source(heini));
        //std::cout << "Barrier2: [" << point.x() << ", " << point.y() << ", " << point.z() << "]" << std::endl;
        auto he = heini;
        int numFrontierEdges = 0;
        Halfedge_index hefrontier;
        int degree = 0;
        do {
            ++degree;
            if (is_frontier_edge.at(he)) {
                numFrontierEdges = numFrontierEdges + 1;
                hefrontier = he;
            }
            he = mesh.next_around_source(he);   
            //std::cout << "numF = " <<  numFrontierEdges << " " << degree << std::endl;     
        } while ((he != heini) | (degree == 0));
        if (numFrontierEdges == 1) {
            he = hefrontier;
            for (int i=0; i <= (degree-1)/2; i++) {
                he = mesh.next_around_source(he);
            }
            auto twinhe = mesh.opposite(he);
            is_frontier_edge[he] = true;
            is_frontier_edge[twinhe] = true;
            repaired_seed.push_back(he);
            repaired_seed.push_back(twinhe);
            //std::cout << "Repaired " << std::endl;
        }
    }
    return repaired_seed;
}


//It generates regular polygons from a list of repaired seed edges, which are the seed of repaired non regualar polygons 
void InsertNRPolygon(std::vector<Halfedge_index> repaired_seed_edges, Surface_mesh& PolMesh, const Surface_mesh& mesh, 
const std::unordered_map<Halfedge_index, bool>& is_frontier_edge) {
    for (auto repair_seed : repaired_seed_edges) {
        std::vector<Vertex_index> polygon_indices;
        std::vector<Point_3> Polig = Polygon_Construction(repair_seed, mesh, is_frontier_edge, polygon_indices);
        std::vector<Vertex_index> Polface;
        for (auto point : Polig) {
            Vertex_index vertex = PolMesh.add_vertex(point);
            Polface.push_back(vertex);
        }
        Face_index faceind = PolMesh.add_face(Polface);
    }

}



//Auxiliar function to check if a set of points (Polygon) has repeated points
bool has_repeated_points(const std::vector<Point_3>& points) {
    std::set<Point_3> unique_points;
    for (const auto& point : points) {
        // Try to insert into the set
        if (!unique_points.insert(point).second) {
            // If insertion fails, it's a duplicate
            std::cout << "Repeated: (" << point.x() << ", " << point.y() << ", " << point.z() << ")" << std::endl;
            return true;
        }
    }
    return false;
}


//Given a set of points (Polygon) prints its points
void print_points(const std::vector<Point_3>& points) {
    for (const auto& point : points) {
        std::cout << "(" << point.x() << ", " << point.y() << ", " << point.z() << ")" << std::endl;
    }
}



//Given a triangulation (mesh) and  maps to the seed and frontier egges obtain a Polylla mesh 
Surface_mesh Polylla(const Surface_mesh& mesh, const std::unordered_map<Halfedge_index, bool> is_seed_edge, 
std::unordered_map<Halfedge_index, bool>& is_frontier_edge) {
    Surface_mesh PolMesh;
    //std::unordered_map<Face_index, bool> is_NRP_face;
    std::vector<Vertex_index> barrier_indices;
    int countseed = 1;
    for (auto he : halfedges(mesh)) {
        is_NRP_seed_edge[he] = false;
        if (is_seed_edge.at(he)) {
            //std::cout << "kill1 " << std::endl;
            std::vector<Vertex_index> polygon_indices;
            std::vector<Point_3> Polig = Polygon_Construction(he, mesh, is_frontier_edge, polygon_indices);
            std::vector<bool> is_barrier(Polig.size(), false);
            //std::cout << "kill2 " << std::endl;
            if (has_repeated_points(Polig))  {
                std::cout << "Repeated vertices Found!! " << std::endl;
                //print_points(Polig); 
            }
            if (is_nonsimple_polygon(Polig)) {
                std::unordered_map<Point_3, bool> is_barrier_point;
                std::cout << "Non Simple Polygom Found!! " << std::endl;
                is_NRP_seed_edge[he] = true;
                find_barrier_vertices(is_barrier, Polig);
                //Insert_Polygon(PolMesh, Polig, is_barrier, polygon_indices, barrier_indices, is_NRP_face, true); 
                Insert_Polygon(PolMesh, Polig, is_barrier, polygon_indices, barrier_indices, true);
                continue;
            }
            Insert_Polygon(PolMesh, Polig, is_barrier, polygon_indices, barrier_indices); 
            ++countseed;
        }
    }
    //is_NRP_edge = find_NRP_edges(is_NRP_face, PolMesh);
    std::cout << "Number of generated polygons = " << countseed << std::endl; 
    std::cout << "Number of barrier points = " << barrier_indices.size() << std::endl; 
    print_barrier_points(mesh, barrier_indices);
    auto repaired_seed = RepairNRPolygon(barrier_indices, mesh, is_frontier_edge);
    InsertNRPolygon(repaired_seed, PolMesh, mesh, is_frontier_edge);
    return PolMesh;
}


void write_NRP_seed_edges_file(const Surface_mesh& mesh, const std::string& filename) {
    write_mapped_edges_to_obj(mesh, is_NRP_seed_edge, filename);
}


// Custom hash function for Point_3
struct Point_3_Hash {
    std::size_t operator()(const Point_3& point) const {
        std::hash<double> hasher;
        // Combine hashes of x, y, and z
        return hasher(point.x()) ^ (hasher(point.y()) << 1) ^ (hasher(point.z()) << 2);
    }
};


// Custom equality function for Point_3
struct Point_3_Equal {
    bool operator()(const Point_3& p1, const Point_3& p2) const {
        return p1 == p2; // Use CGAL's == operator
    }
};


/* From a vector list of nom-regular polygons (NRP_Polygons) generate an .obj file listing the
segments of the polygons */
void write_obj_NRP_edges_file(const std::string& filename) {
    std::ofstream obj_file(filename);
    if (!obj_file) {
        throw std::ios_base::failure("Failed to open file for writing");
    }
    std::vector<Point_3> vertices;
    std::unordered_map<Point_3, size_t, Point_3_Hash, Point_3_Equal> vertex_indices;
    size_t vertex_count = 0;
    // Iterate through all polygons
    for (const auto& polygon : NRP_Polygons) {
        for (const auto& point : polygon) {
            // Add unique vertices
            if (vertex_indices.find(point) == vertex_indices.end()) {
                vertex_indices[point] = ++vertex_count;
                vertices.push_back(point);
            }
        }
    }
    // Write vertices to the file
    for (const auto& vertex : vertices) {
        obj_file << "v " << vertex.x() << " " << vertex.y() << " " << vertex.z() << "\n";
    }
    // Write segments (lines)
    for (const auto& polygon : NRP_Polygons) {
        for (size_t i = 0; i < polygon.size(); ++i) {
            size_t start_idx = vertex_indices[polygon[i]];
            size_t end_idx = vertex_indices[polygon[(i + 1) % polygon.size()]]; // Wrap around
            obj_file << "l " << start_idx << " " << end_idx << "\n";
        }
    }
    obj_file.close();
}


