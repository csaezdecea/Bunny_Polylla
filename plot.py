import os
import trimesh
import pyvista as pv
import numpy as np


def load_segments(segments_path):
    """
    Loads vertices and edges from a .obj file at the specified path.
    
    Parameters:
        segments_path (str): Path to the .obj file containing segment data.
    
    Returns:
        tuple: A tuple containing:
            - vertices (np.ndarray): Array of vertices.
            - edges (np.ndarray): Array of edge indices.
    """
    vertices = []
    edges = []
    try:
        segments_mesh = trimesh.load(segments_path, process=False)
        if not hasattr(segments_mesh, 'edges') or segments_mesh.edges is None:
            raise ValueError("No edges found in segments file.")
        
        # Use detected edges from trimesh
        vertices = np.array(segments_mesh.vertices)
        edges = np.array(segments_mesh.edges)
    except ValueError:
        # Manual parsing for OBJ format
        with open(segments_path, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if parts[0] == 'v':
                    # Vertex line in OBJ: "v x y z"
                    vertices.append([float(x) for x in parts[1:4]])
                elif parts[0] == 'l':
                    # Line segment in OBJ: "l idx1 idx2" (1-based indexing)
                    edges.append([int(parts[1]) - 1, int(parts[2]) - 1])  # Convert to 0-based indexing
        
        vertices = np.array(vertices)
        edges = np.array(edges)
        # Prepare edges array for PyVista by including the vertex count for each segment (2 for lines)
        edge_count = np.full((edges.shape[0], 1), 2)
        edges = np.hstack((edge_count, edges)).astype(int).ravel()
        segments = pv.PolyData(vertices)  # Use manually parsed vertices if needed
        segments.lines = edges  # Set lines directly
    return segments


def check_files(triangulation_path, segments_1_path, segments_2_path, segments_3_path, segments_4_path):
    # Check if files exist before loading
    if not os.path.isfile(triangulation_path):
        raise FileNotFoundError(f"Triangulation file not found: {triangulation_path}")
    if not os.path.isfile(segments_1_path):
        raise FileNotFoundError(f"Segments file not found: {segments_1_path}")
    if not os.path.isfile(segments_2_path):
        raise FileNotFoundError(f"Segments file not found: {segments_2_path}")
    if not os.path.isfile(segments_3_path):
        raise FileNotFoundError(f"Segments file not found: {segments_3_path}")
    if not os.path.isfile(segments_4_path):
        raise FileNotFoundError(f"Segments file not found: {segments_4_path}")



def add_dashed_lines(plotter, segments, color="white", line_width=4, dash_length=0.002, gap_length=0.001):
    """
    Adds dashed lines to the PyVista plotter by plotting segments with gaps in between.
    
    Parameters:
        plotter (pv.Plotter): The PyVista plotter instance.
        segments (pv.PolyData): PolyData object containing segment data with points and lines.
        color (str): Color of the dashed lines.
        line_width (float): Width of each line segment.
        dash_length (float): Length of each dash.
        gap_length (float): Length of the gap between dashes.
    """
    vertices = segments.points
    edges = segments.lines.reshape(-1, 3)[:, 1:]  # Assuming each line has 2 vertices (PolyData format)
    
    for edge in edges:
        start, end = vertices[edge[0]], vertices[edge[1]]
        # Calculate the direction vector and the total length of the edge
        direction = end - start
        edge_length = np.linalg.norm(direction)
        #print(edge_length)
        direction = direction.astype(float) / edge_length  # Normalize direction as float
        # Initialize position for the start of the dash
        current_position = start
        current_length = 0.0
        # Generate dashes along the edge
        while current_length < edge_length:
            # Calculate the end of the current dash segment
            dash_end = current_position + min(dash_length, edge_length - current_length) * direction
            # Add a line segment for this dash
            line_segment = np.array([current_position, dash_end])
            plotter.add_lines(line_segment, color=color, width=line_width)
            # Move to the next starting point (end of the current dash + gap)
            current_position = dash_end + gap_length * direction
            current_length += dash_length + gap_length


# Define the paths to your files
dataf = "data"
Delta_normal = False
triangulation_path = dataf+ "/bunny2.off"  # Replace with the actual path
segments_1_path = dataf + "/frontier_edges.obj"  # Use .obj format for segments
segments_2_path = dataf + "/seed_edges.obj"  # Use .obj format for segments
segments_3_path = dataf + "/NRP_edges.obj"  # Use .obj format for segments
segments_4_path = dataf + "/Delta_normal_edges.obj"  # Use .obj format for segments

if (Delta_normal): 
    check_files(triangulation_path, segments_1_path, segments_2_path, segments_3_path, segments_4_path)
else:
    check_files(triangulation_path, segments_1_path, segments_2_path, segments_3_path, segments_3_path)


# Load triangulation using trimesh
triangulation_mesh = trimesh.load(triangulation_path)

# Prepare faces array for PyVista by including the vertex count for each face
face_count = np.full((triangulation_mesh.faces.shape[0], 1), 3)  # Assuming triangular faces
faces = np.hstack((face_count, triangulation_mesh.faces)).astype(int).ravel()


# Prepare edges array for PyVista by including the vertex count for each segment (2 for lines)
segments_1 = load_segments(segments_1_path)
segments_2 = load_segments(segments_2_path)
segments_3 = load_segments(segments_3_path)
if (Delta_normal): 
    segments_4 = load_segments(segments_4_path)

# Create the PyVista PolyData for triangulation and segments
triangulation = pv.PolyData(triangulation_mesh.vertices, faces)

# Initialize a PyVista plotter
#plotter = pv.Plotter(off_screen=True)
plotter = pv.Plotter()

# Add the triangulation mesh
plotter.add_mesh(triangulation, color="lightblue", opacity=1, show_edges=True, edge_color="black", line_width=4)

# Add the segments as another mesh with distinct styling
plotter.add_mesh(segments_1, color="red", line_width=8)
plotter.add_mesh(segments_3, color="yellow", line_width=4)
if (Delta_normal):
    plotter.add_mesh(segments_4, color="blue", line_width=4)
add_dashed_lines(plotter, segments_2)

# Coordinates of the segment you want to highlight
#highlight_start = np.array([-0.046877, 0.0350273, 0.0454073])  # Replace with actual coordinates
#highlight_end = np.array([-0.0551207, 0.0340565, 0.0240362])    # Replace with actual coordinates
# Add the highlighted segment to the plotter
#plotter.add_lines(np.array([highlight_start, highlight_end]), color="yellow", width=10)

# Add barrier points to the plot
barrier_fname = dataf + "/barrier_points.dat"
barrier_points = np.loadtxt(barrier_fname)
barrier_points_polydata = pv.PolyData(barrier_points)
plotter.add_mesh(
    barrier_points_polydata,
    color="green",
    render_points_as_spheres=True,
    point_size=20
)


# Optional: Set plot view settings for better visualization
plotter.show_axes()  # Show axes for orientation
plotter.background_color = "white"  # Background color
plotter.view_isometric()  # Set initial view

# Set the camera position with the y-axis up
plotter.camera_position = [(0, -1, 0), (0, 0, 0), (0, 0, 1)]  # Adjust view
plotter.view_vector([-.5, 0, .5], [0, 1, 0])  # Set y-axis as up

# Render the plot
# plotter.show(screenshot=("data/bunny.png"))
plotter.show()

