# Polylla Mesh Processing

This project processes 3D surface meshes to generate a refined Polylla mesh. 
It is a generalization of the 2D Polylla algorithm, which has been implemented through the half edge structure of CGAL. 
The code includes functions for identifying specific edge types, handling non-simple polygons, and writing the resulting data to standard formats for visualization and further analysis.

## Overview: This is a reference.

The project is structured as follows:
```
. 
├── build 
│ ├── bin 
│ │   └── Bunny # Compiled executable 
│ └── CMakeCache.txt # CMake configuration cache 
├── example.py # Example script for using the code 
├── include 
│   └── mesh.hpp # Header file with mesh-processing functions 
├── plot.py # Script for visualizing the results 
├── Readme.md # Project documentation 
└── src 
    └── bunny.cpp # Main entry point for the application
```

### Core Components

1. **`src/bunny.cpp`**:
   - The main routine for reading a surface mesh, processing it, and generating the Polylla mesh.
   - Implements edge mappings and outputs results in `.obj` and `.off` formats for visualization.

2. **`include/mesh.hpp`**:
   - Defines the `Polylla` function and supporting utilities like `find_longest_edges_mapping`, `find_seed_edges_mapping`, and more.

3. **Output Files**:
   - **`frontier_edges.obj/.off`**: Contains mapped frontier edges.
   - **`seed_edges.obj/.off`**: Contains mapped seed edges.
   - **`Polylla.off`**: The final refined Polylla surface mesh.

4. **Visualization**:
   - Use `plot.py` to visualize the generated meshes and edges interactively.

---

## Building the Project

This project uses **CMake** for build configuration. Ensure you have CMake installed along with a C++ compiler supporting CGAL.

1. **Clone the repository**:
```bash
git clone <repository-url>
cd <repository-folder>
```
```
mkdir build
cd build
cmake ..
make
```



The compiled executable will be available in build/bin as Bunny.

## Running the Application

Prepare Input Mesh: Ensure you have a valid .off file representing the input surface mesh (e.g., bunny2.off) in the data/ directory.

Execute the program:

./build/bin/Bunny

Output: The program generates multiple .obj and .off files containing:
* Seed edges
* Frontier edges
* Refined Polylla mesh
* Logs provide information about the number of polygons and barrier points generated.

Key Functions

## Polylla Function

The Polylla function is the main mesh-processing routine. It refines a given triangulation by constructing polygons and addressing potential issues such as repeated points or non-simple polygons.

**Inputs:**

    Surface_mesh mesh: Input surface mesh.
    std::unordered_map<Halfedge_index, bool> is_seed_edge: Mapping of seed edges.
    std::unordered_map<Halfedge_index, bool>& is_frontier_edge: Mapping of frontier edges.

**Outputs:**

    Surface_mesh PolMesh: Refined Polylla mesh.

**Function Comments:**

For each half-edge in the mesh, checks if it is a seed edge using is_seed_edge. If true, constructs a polygon using Polygon_Construction and verifies its properties:
* Detects repeated vertices and prints them.
* Identifies non-simple polygons and marks them as non-regular polygons (NRP).
* Inserts polygons into the refined mesh using Insert_Polygon.
* Tracks barrier vertices and counts polygons.
* Repairs non-regular polygons with RepairNRPolygon and adds them to the mesh.

**Key Logs:**

Logs the number of polygons generated:
* "Number of generated polygons = <count>"
* Logs the number of barrier points:
* "Number of barrier points = <count>"

**Edge Mapping Functions:**

These functions identify and map specific edges in the mesh:

    find_longest_edges_mapping:
        Identifies the longest edge of each triangle in the input mesh.

    find_seed_edges_mapping:
        Determines seed edges based on the longest edge mapping.

    find_frontier_edges_mapping:
        Maps frontier edges using seed and longest edges.

**Example Output:**

The program generates several output files in the data/ directory:

    Polylla Mesh:
    The refined mesh is saved in Polylla.off.

    Edge Files:
        Mapped frontier edges: frontier_edges.obj, frontier_edges.off
        Mapped seed edges: seed_edges.obj, seed_edges.off
        Non-regular polygon seed edges: NRP_seed_edges.obj
        Non-regular polygon edges: NRP_edges.obj


## Visualization ##

Use the provided plot.py script to visualize the generated output files.

Example Usage:

```
python3 plot.py
```

Ensure the output files (.obj, .off) are in the correct paths for the script to load them.
Comments in the Code

## bunny.cpp 

Contains comments explaining each step of the program. Here are key highlights:

    Reading the Input Mesh:

Surface_mesh mesh = read_OFFfile("../../data/bunny2.off");


Logging Outputs:

    Outputs edge mappings, polygon counts, and barrier points:

    std::cout << "Number of generated polygons = " << countseed << std::endl;
    std::cout << "Number of barrier points = " << barrier_indices.size() << std::endl;

Writing Results:

    Writes the final Polylla mesh:

        CGAL::IO::write_OFF(outvor, PolMesh);
        std::cout << "Polylla surface mesh written to Polylla.off\n";

Dependencies

    CGAL: Used for surface mesh representation and processing.
    C++ Compiler: Must support CGAL and C++17 or later.
    Python (optional): For running plot.py.


## References ##
- CGAL: https://www.cgal.org/
- CGAL Half-edge structure: https://doc.cgal.org/latest/HalfedgeDS/index.html
- Polylla 2D: https://github.com/ssalinasfe/Polylla-Mesh-DCEL


## License ##

This project is licensed under the MIT License.
Contact

For questions or issues, please contact Cristian.


## Notes ##

- The method  ```CWvertexEdge(he)``` to find the next conter wise (CW) edge of he, in the original Polylla 2D code can be replaced by the CGAL routine ```mesh.next_around_source(he)```.

## Explanation of `write_obj_NRP_edges_file` method ##

In ```mesh.hpp``` method 
```
void write_obj_NRP_edges_file(const std::string& filename)
``` 
we use `std::unordered_map` to store the indices of unique vertices in the `.obj` file:

```cpp
std::unordered_map<Point_3, size_t, Point_3_Hash, Point_3_Equal> vertex_indices;
```


Components of vertex_indices
Key (Point_3)

    Description: Each 3D point in the mesh.
    Example: Point_3(0, 0, 0).

Value (size_t)

    Description: The index of the vertex in the .obj file.
    Example: Vertex Point_3(0, 0, 0) might have an index of 1.

Custom Hash (Point_3_Hash)

    Description: A function that computes a hash value for a Point_3. This ensures std::unordered_map can store Point_3 as a key.
    Examples:
        Point_3(0, 0, 0) → Hash value: 12345.
        Point_3(1, 0, 0) → Hash value: 67890.

Custom Equality (Point_3_Equal)

    Description: A function that tells the map how to compare two Point_3 objects for equality.
    Examples:
        Point_3(0, 0, 0) == Point_3(0, 0, 0) → true.
        Point_3(0, 0, 0) == Point_3(1, 0, 0) → false.



The hash key is used in:

- Finding an element (find): Determines the bucket where a key might exist.
- Inserting an element ([] or insert): Computes the bucket to store the key-value pair.
- Retrieving an element ([]): Uses the hash to locate the key and retrieve its associated value.
- Collision resolution: When hashes collide, equality (Point_3_Equal) ensures distinct keys are handled correctly.

This makes std::unordered_map efficient for fast lookups, insertions, and deletions, as long as the hash and equality functions are properly implemented.
