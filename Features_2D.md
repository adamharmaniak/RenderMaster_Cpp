# Rendering Application Features 2D
These functionalities allow for comprehensive drawing, rendering, and manipulation of *2D geometric shapes* and *curves*.

![Snímka obrazovky (321)](https://github.com/user-attachments/assets/5d23d26d-e594-497e-9ef9-092a42eb4994)

### Color Selection
Use QColorDialog to set the global drawing color.

### Line Drawing
Select the line tool, use the left mouse button to mark two points, and draw the line. Choose between DDA or Bresenham algorithm for rasterization.

### Circle Drawing
Select the circle tool, mark the center with the left mouse button, then a second point for the radius. Use Bresenham's algorithm for rasterization.

### Polygon and Line Rendering
Draw polygons by marking points with the left mouse button, right-click to close. Rasterize using DDA or Bresenham algorithm. Only one object (polygon/segment) can exist at a time; delete to create new.

### Scrolling
Click and drag the left mouse button to move objects. Release to place the object at a new position.

### Rotation
Rotate around the first specified point using UI controls for angle (-360° to 360°). Rotate both clockwise and counterclockwise.

### Scaling
Scale from the first point using UI parameters or middle mouse button scrolling (fixed parameters: 0.75 for down, 1.25 for up).

### Axis Symmetry
Choose any polygon edge or a horizontal/vertical line through a segment point as the axis.

### Beveling
Bevel in the x-direction with a user-specified skew coefficient using the UI.

### Polygon Rendering and Filling
Fill polygons with selected color using the Scan-line algorithm.

### Triangle Rendering and Filling
Define vertex colors and interpolate using Nearest Neighbor or Barycentric methods. Use a modified Scan-line algorithm for filling.

### Curve Plotting
Control points are visible before and after plotting. Plot tangent vectors for Hermitian cubics.
#### a. Hermitian Cubics
Click ≥2 control points, rotate tangent vectors via UI, vectors length constant or user-defined (e.g., 150px).

#### b. Bezier Curve (degree n)
Click ≥2 control points, use de Casteljau's algorithm for plotting.

#### c. Coons Cubic B-Spline
Click ≥4 control points for plotting.
