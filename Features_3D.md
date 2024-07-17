# 3D Rendering Application Features
These functionalities enable comprehensive **3D model viewing**, **illumination**, **shading**, and **projection adjustments**.



### Cube Wire-frame Model Projection
Load the surface representation of a cube from a VTK Polydata file via the UI and view its wire-frame model projection.

### Sphere Wire-frame Model Projection
Load the surface representation of a polygonal (triangular) sphere from a VTK Polydata file via the UI and view its wire-frame model projection.

### Illuminated Sphere Model Projection
Display the projection of an illuminated polygonal (triangular) sphere model using Phong's illumination model.

## User-defined Settings:
#### Light Source 
Define a single point light source and its color (positioned in world view or view system).
#### Camera Position 
Set the observer/camera position on the normal averaged over a distance from the projective.
#### POM Coefficients
Input material coefficients for diffuse, specular, and ambient light response as triplets.
#### Specular Sharpness 
Specify the sharpness coefficient for the specular component.
#### Ambient Light Color
Define the color of simulated ambient light.
#### Shading Options
**Constant Shading**  
**Gouraud Shading:** Use barycentric interpolation.  
Visibility is handled using a z-buffer when displaying the shaded model.

#### Projection Adjustment
Adjust the camera using zenith and azimuth angles (θ=[0;π], φ=[0;2π]) via UI elements.

#### Projection Type Selection
Choose between parallel or centered projection in the UI. For centered projection, specify the center of the projection by indicating the distance on the projection normal.
