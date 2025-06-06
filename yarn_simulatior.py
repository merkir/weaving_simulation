import numpy as np
import math
from dataclasses import dataclass
import os

class YarnSimulation:
    """Generate points for strands spiraling around a central yarn path."""

    def __init__(self, strand_diameter=0.1, strand_count=3, color=(1.0, 1.0, 1.0), tension=0.9):
        # Properties
        self.segment_count = 100      # How smooth the strand is
        self.strand_count = strand_count  # Number of fiber strands for yarn
        self.strand_diameter = strand_diameter  # Diameter of a strand
        self.color = color            # RGB color tuple
        self.packing_density = 1    # How densely the strands are packed

        # Twists per unit length
        if self.strand_count == 3:
            # (strand_count+1) * twist_rate * strand_diameter <= 1
            self.twist_rate = 1/((self.strand_count+1) * self.strand_diameter) * self.packing_density
        elif self.strand_count == 4:
            self.twist_rate = 1/((self.strand_count+1) * self.strand_diameter) * self.packing_density
        elif self.strand_count == 1:
            self.twist_rate = 0
        else:
            raise ValueError("Strand count not supported")

        # yarn diameter
        if self.strand_count == 3:
            self.yarn_diameter = self.strand_diameter * (2/ np.sqrt(3) + 1)
        elif self.strand_count == 4:
            self.yarn_diameter = self.strand_diameter * (2/ np.sqrt(2) + 1)
        elif self.strand_count == 1:
            self.yarn_diameter = self.strand_diameter
        else:
            raise ValueError("Strand count not supported")

        self.tension = tension

        self.vertices_per_ring = 16
        
        # Components
        self.path = None              # Will hold the central path points
        self.strands = None           # Will hold the strand points
        self.initial_frame = None     # Will store the initial coordinate frame for consistency

    def set_yarn_path(self, path):
        """set the path of this yarn"""
        self.path = path

    def generate_yarn(self):
        """generate the yarn with the instance configurations"""
        if not self.path:
            raise ValueError("No path has been set for this yarn")
        self.generate_yarn_along_path(self.path)

        vertices, faces = self.create_obj()
        return vertices, faces

    def generate_yarn_along_path(self, path):
        """
        Generate a yarn along any given path.
        """        
        self.path = path
        self.strands = [None] * self.strand_count
    
        for i in range(self.strand_count):
            self.__generate_single_strand_along_path(i)
        
        return self
    
    def __create_consistent_frames(self):
        """
        Create consistent coordinate frames along the path using parallel transport.
        This prevents sudden flips in orientation as the path curves.
        Works for arbitrary paths in 3D space, not limited to any specific plane.
        """
        path_points = len(self.path)
        frames = []
        
        # Initialize the first frame
        if path_points <= 1:
            return [{'forward': np.array([1, 0, 0]), 'up': np.array([0, 1, 0]), 'right': np.array([0, 0, 1])}]
        
        # Get the initial forward direction
        forward = self.path[1] - self.path[0]
        forward_length = np.linalg.norm(forward)
        if forward_length > 0:
            forward = forward / forward_length
        
        # Choose a consistent initial up vector that works for any path in 3D space
        # We'll use the method of least-changing up vector
        
        # Start with a default up vector
        default_up = np.array([0, 1, 0])
        
        # If forward is too closely aligned with the default up vector,
        # choose a different default
        if abs(np.dot(forward, default_up)) > 0.9:
            default_up = np.array([0, 0, 1])
            # If still too aligned, use x-axis
            if abs(np.dot(forward, default_up)) > 0.9:
                default_up = np.array([1, 0, 0])
        
        # Make the up vector perpendicular to forward
        up = default_up - forward * np.dot(forward, default_up)
        up_length = np.linalg.norm(up)
        if up_length > 0:
            up = up / up_length
        
        # Complete the orthonormal basis
        right = np.cross(forward, up)
        
        frames.append({'forward': forward, 'up': up, 'right': right})
        
        # Now propagate this frame along the path using parallel transport
        for i in range(1, path_points):
            prev_forward = frames[i-1]['forward']
            prev_up = frames[i-1]['up']
            prev_right = frames[i-1]['right']
            
            # Calculate new forward direction
            if i < path_points - 1:
                new_forward = self.path[i+1] - self.path[i]
            else:
                new_forward = self.path[i] - self.path[i-1]
                
            new_forward_length = np.linalg.norm(new_forward)
            if new_forward_length > 0:
                new_forward = new_forward / new_forward_length
            
            # Calculate the rotation from previous forward to new forward
            rotation_axis = np.cross(prev_forward, new_forward)
            axis_length = np.linalg.norm(rotation_axis)
            
            if axis_length > 1e-10:  # If the directions are not parallel
                # Normalize rotation axis
                rotation_axis = rotation_axis / axis_length
                
                # Calculate rotation angle
                cos_angle = np.dot(prev_forward, new_forward)
                cos_angle = np.clip(cos_angle, -1.0, 1.0)  # Ensure within valid range
                angle = np.arccos(cos_angle)
                
                # Rotate previous up and right vectors around rotation_axis by angle
                new_up = self.__rotate_vector(prev_up, rotation_axis, angle)
                new_right = self.__rotate_vector(prev_right, rotation_axis, angle)
            else:
                # Vectors are parallel, no rotation needed
                new_up = prev_up
                new_right = prev_right
            
            # Ensure the new frame is orthonormal
            new_right = np.cross(new_forward, new_up)
            new_up = np.cross(new_right, new_forward)
            
            # Normalize
            new_up = new_up / np.linalg.norm(new_up)
            new_right = new_right / np.linalg.norm(new_right)
            
            frames.append({'forward': new_forward, 'up': new_up, 'right': new_right})
        
        return frames
    
    def __rotate_vector(self, v, axis, angle):
        """
        Rotate vector v around axis by angle using Rodrigues' rotation formula.
        Handles numerical precision issues and special cases.
        """
        # Ensure the axis is normalized
        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-10:
            return v  # No rotation if axis is too small
            
        axis = axis / axis_norm
        
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        
        # Handle special cases for numerical stability
        if abs(sin_angle) < 1e-10:
            if cos_angle > 0:
                return v  # No rotation
            else:
                # 180-degree rotation
                return v - 2 * (np.dot(v, axis) * axis)
                
        # Rodrigues' rotation formula
        return (v * cos_angle + 
                np.cross(axis, v) * sin_angle + 
                axis * np.dot(axis, v) * (1 - cos_angle))

    def __generate_single_strand_along_path(self, strand_index):
        """
        Generate a strand that spirals around the given path.
        
        Args:
            strand_index: Index of the strand to generate
        """
        if self.path is None:
            raise ValueError("Path must be set before generating strands")
        
        strand = []
        path_points = len(self.path)
        
        # Calculate accumulated distance along the path
        accumulated_distance = np.zeros(path_points)
        for i in range(1, path_points):
            segment_length = np.linalg.norm(self.path[i] - self.path[i-1])
            accumulated_distance[i] = accumulated_distance[i-1] + segment_length
        
        # Angle offset for this strand
        strand_angle_interval = 2 * math.pi / self.strand_count
        strand_offset_angle = strand_index * strand_angle_interval

        # Calculate distance from yarn center to strand center
        yarn_center_to_strand_center_dis = self.strand_diameter / 2
        if self.strand_count == 3:
            yarn_center_to_strand_center_dis = self.strand_diameter / np.sqrt(3)
        elif self.strand_count == 4:
            yarn_center_to_strand_center_dis = self.strand_diameter / np.sqrt(2)
        
        # Create consistent coordinate frames along the path
        frames = self.__create_consistent_frames()
        
        # Generate each point on the strand
        for i in range(path_points):
            center = self.path[i]
            
            # Calculate twist angle based on accumulated distance
            distance = accumulated_distance[i]
            angle = 2 * math.pi * self.twist_rate * distance
            total_angle = angle + strand_offset_angle
            
            # Get the local coordinate system from the frame
            frame = frames[i]
            right = frame['right']
            up = frame['up']
            
            # Calculate strand position at this point
            offset = yarn_center_to_strand_center_dis * (right * math.cos(total_angle) + up * math.sin(total_angle))
            point = center + offset
            
            strand.append(point)
        
        self.strands[strand_index] = strand

    def create_obj(self):
        """
        Create an obj to represent the yarn with strands
        
        Returns:
            vertices: string list of vertices to write in obj file
            faces: string list of faces to write in obj file
        """
        vertices = []
        faces = []
        vertex_index = 0 # init
        
        # Create consistent coordinate frames along the path
        frames = self.__create_consistent_frames()
        
        # Create for each strand tube
        for strand in self.strands:
            if strand is None:
                continue
                
            segments = len(strand) - 1
            radius = self.strand_diameter / 2 * self.packing_density
            
            for i in range(len(strand)):
                # Get the local coordinate system from the frames
                frame = frames[i]
                direction = frame['forward']
                perpendicular = frame['up']
                third = frame['right']
                
                # Create vertices on ring
                for j in range(self.vertices_per_ring):
                    angle = j * (2 * math.pi / self.vertices_per_ring)
                    dx = radius * (perpendicular[0] * math.cos(angle) + third[0] * math.sin(angle))
                    dy = radius * (perpendicular[1] * math.cos(angle) + third[1] * math.sin(angle))
                    dz = radius * (perpendicular[2] * math.cos(angle) + third[2] * math.sin(angle))
                    
                    vertex = strand[i] + np.array([dx, dy, dz])
                    vertices.append(vertex)
            
            # Create faces connecting the rings
            for i in range(segments):
                for j in range(self.vertices_per_ring):
                    # Calculate indices for the quad
                    v1 = vertex_index + i * self.vertices_per_ring + j
                    v2 = vertex_index + i * self.vertices_per_ring + (j + 1) % self.vertices_per_ring
                    v3 = vertex_index + (i + 1) * self.vertices_per_ring + (j + 1) % self.vertices_per_ring
                    v4 = vertex_index + (i + 1) * self.vertices_per_ring + j
                    
                    # Add the face
                    faces.append((v1, v2, v3))
                    faces.append((v1, v3, v4))
            
            # Update vertex index for the next strand
            vertex_index += len(strand) * self.vertices_per_ring
        
        return vertices, faces

    def save_yarn_to_obj_file(self, output_file):
        """Save the yarn geometry to an OBJ file."""
        # Ensure output directory exists
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        
        # Create OBJ file
        vertices, faces = self.create_obj()
        
        # Write the OBJ file
        with open(output_file, "w") as f:
            # Write comment with color information
            f.write(f"# Yarn color: {self.color}\n")
            
            # Write vertices
            for v in vertices:
                f.write(f"v {v[0]} {v[1]} {v[2]}\n")
            
            # Write faces (OBJ indices start at 1)
            for face in faces:
                if len(face) == 3:
                    f.write(f"f {face[0] + 1} {face[1] + 1} {face[2] + 1}\n")
                elif len(face) == 4:
                    f.write(f"f {face[0] + 1} {face[1] + 1} {face[2] + 1} {face[3] + 1}\n")
        
        return output_file

    
    def generate_straight_unit_yarn(self): 
        """
        Generate a default straight unit-length yarn.
        
        Returns:
            self: the YarnSimulation instance for method chaining
        """

        print("ttttt")

        # Generate yarn path
        self.__generate_straight_yarn_path()
        
        # Initialize strands array
        self.strands = [None] * self.strand_count
        
        # Generate each strand
        for i in range(self.strand_count):
            self.__generate_single_strand_along_path(i)
        
        return self

    def __generate_straight_yarn_path(self): 
        """Generate a straight unit-length path along the x-axis."""
        t = np.linspace(0, 1, num=self.segment_count)  
        x = t
        y = np.zeros_like(t)
        z = np.zeros_like(t)

        self.path = np.column_stack((x, y, z))


def main():
    os.makedirs("output", exist_ok=True)
    
    # Basic straight yarn
    yarn = YarnSimulation(strand_diameter=0.4, strand_count=3, color=(0.0, 1.0, 0.0))
    # yarn.generate_straight_unit_yarn()
    # yarn.save_yarn_to_obj_file("output/straight_yarn.obj")
    # print("Straight yarn generated successfully")

    # path = [(-1.2, 0, -0.5), (0, 0, -0.5), (1.2, 0, 0.5), (2.4, 0, -0.5), (3.5999999999999996, 0, 0.5), (4.8, 0, -0.5), (6.0, 0, 0.5), (7.2, 0, -0.5), (8.4, 0, 0.5), (9.6, 0, -0.5), (10.799999999999999, 0, 0.5), (11.999999999999998, 0, -0.5), (13.199999999999998, 0, 0.5), (14.399999999999997, 0, 0.5)]
    # path = [(2.0, -2, -0.5), (2.0, 0, -0.5), (2.0, 2.0, 0.5), (2.0, 4.0, -0.5), (2.0, 6.0, 0.5), (2.0, 8.0, 0.5)]
    path=[(-4.287854430489671, 30.165807537309526, 1.0772425350604955), (0.021546646268831696, 30.165807537309526, 1.0772425350604955), (0.15473602206457515, 30.165807537309526, 1.0717870399573415), (0.2873331755217699, 30.165807537309526, 1.0581061355015149), (0.41883061655984144, 30.165807537309526, 1.0362521827213147), (0.5487250640358105, 30.165807537309526, 1.0063088234142321), (0.6765193719559883, 30.165807537309526, 0.968390660024069), (0.8017244322065699, 30.165807537309526, 0.9226428170221466), (0.9238610465207883, 30.165807537309526, 0.8692403854713253), (1.216745830235748, 30.165807537309526, 0.6595368681750762), (1.5096306139507074, 30.165807537309526, 0.4498333508788271), (1.8025153976656672, 30.165807537309526, 0.240129833582578), (2.0954001813806267, 30.165807537309526, 0.03042631628632886), (2.388284965095586, 30.165807537309526, -0.17927720100992017), (2.6811697488105457, 30.165807537309526, -0.3889807183061693), (2.9740545325255052, 30.165807537309526, -0.5986842356024185), (3.2669393162404647, 30.165807537309526, -0.8083877528986676), (3.3855400302377148, 30.165807537309526, -0.8692403854713253), (3.507676644551933, 30.165807537309526, -0.9226428170221466), (3.6328817048025144, 30.165807537309526, -0.968390660024069), (3.7606760127226924, 30.165807537309526, -1.0063088234142321), (3.8905704601986617, 30.165807537309526, -1.0362521827213147), (4.022067901236733, 30.165807537309526, -1.0581061355015149), (4.154665054693928, 30.165807537309526, -1.0717870399573415), (4.330947723027335, 30.165807537309526, -1.0772425350604955), (4.464137098823078, 30.165807537309526, -1.0717870399573415), (4.596734252280273, 30.165807537309526, -1.0581061355015149), (4.728231693318344, 30.165807537309526, -1.0362521827213147), (4.8581261407943135, 30.165807537309526, -1.0063088234142321), (4.9859204487144915, 30.165807537309526, -0.968390660024069), (5.111125508965073, 30.165807537309526, -0.9226428170221466), (5.2332621232792915, 30.165807537309526, -0.8692403854713253), (5.526146906994251, 30.165807537309526, -0.6595368681750762), (5.819031690709211, 30.165807537309526, -0.4498333508788271), (6.11191647442417, 30.165807537309526, -0.240129833582578), (6.40480125813913, 30.165807537309526, -0.03042631628632886), (6.697686041854089, 30.165807537309526, 0.17927720100992017), (6.990570825569049, 30.165807537309526, 0.3889807183061693), (7.283455609284008, 30.165807537309526, 0.5986842356024185), (7.576340392998968, 30.165807537309526, 0.8083877528986676), (7.694941106996217, 30.165807537309526, 0.8692403854713253), (7.817077721310436, 30.165807537309526, 0.9226428170221466), (7.942282781561017, 30.165807537309526, 0.968390660024069), (8.070077089481195, 30.165807537309526, 1.0063088234142321), (8.199971536957165, 30.165807537309526, 1.0362521827213147), (8.331468977995236, 30.165807537309526, 1.0581061355015149), (8.46406613145243, 30.165807537309526, 1.0717870399573415), (8.640348799785837, 30.165807537309526, 1.0772425350604955), (8.773538175581582, 30.165807537309526, 1.0717870399573415), (8.906135329038776, 30.165807537309526, 1.0581061355015149), (9.037632770076847, 30.165807537309526, 1.0362521827213147), (9.167527217552816, 30.165807537309526, 1.0063088234142321), (9.295321525472994, 30.165807537309526, 0.968390660024069), (9.420526585723575, 30.165807537309526, 0.9226428170221466), (9.542663200037794, 30.165807537309526, 0.8692403854713253), (9.835547983752754, 30.165807537309526, 0.6595368681750762), (10.128432767467713, 30.165807537309526, 0.4498333508788271), (10.421317551182671, 30.165807537309526, 0.240129833582578), (10.714202334897632, 30.165807537309526, 0.03042631628632886), (11.007087118612592, 30.165807537309526, -0.17927720100992017), (11.29997190232755, 30.165807537309526, -0.3889807183061693), (11.59285668604251, 30.165807537309526, -0.5986842356024185), (11.88574146975747, 30.165807537309526, -0.8083877528986676), (12.004342183754721, 30.165807537309526, -0.8692403854713253), (12.12647879806894, 30.165807537309526, -0.9226428170221466), (12.25168385831952, 30.165807537309526, -0.968390660024069), (12.379478166239698, 30.165807537309526, -1.0063088234142321), (12.509372613715668, 30.165807537309526, -1.0362521827213147), (12.640870054753739, 30.165807537309526, -1.0581061355015149), (12.773467208210933, 30.165807537309526, -1.0717870399573415), (12.94974987654434, 30.165807537309526, -1.0772425350604955), (13.082939252340084, 30.165807537309526, -1.0717870399573415), (13.215536405797279, 30.165807537309526, -1.0581061355015149), (13.34703384683535, 30.165807537309526, -1.0362521827213147), (13.47692829431132, 30.165807537309526, -1.0063088234142321), (13.604722602231497, 30.165807537309526, -0.968390660024069), (13.729927662482078, 30.165807537309526, -0.9226428170221466), (13.852064276796296, 30.165807537309526, -0.8692403854713253), (14.144949060511257, 30.165807537309526, -0.6595368681750762), (14.437833844226216, 30.165807537309526, -0.4498333508788271), (14.730718627941176, 30.165807537309526, -0.240129833582578), (15.023603411656136, 30.165807537309526, -0.03042631628632886), (15.316488195371095, 30.165807537309526, 0.17927720100992017), (15.609372979086055, 30.165807537309526, 0.3889807183061693), (15.902257762801014, 30.165807537309526, 0.5986842356024185), (16.195142546515974, 30.165807537309526, 0.8083877528986676), (16.313743260513224, 30.165807537309526, 0.8692403854713253), (16.435879874827442, 30.165807537309526, 0.9226428170221466), (16.561084935078025, 30.165807537309526, 0.968390660024069), (16.6888792429982, 30.165807537309526, 1.0063088234142321), (16.81877369047417, 30.165807537309526, 1.0362521827213147), (16.950271131512242, 30.165807537309526, 1.0581061355015149), (17.082868284969436, 30.165807537309526, 1.0717870399573415), (17.259150953302843, 30.165807537309526, 1.0772425350604955), (17.392340329098587, 30.165807537309526, 1.0717870399573415), (17.52493748255578, 30.165807537309526, 1.0581061355015149), (17.656434923593853, 30.165807537309526, 1.0362521827213147), (17.786329371069822, 30.165807537309526, 1.0063088234142321), (17.91412367899, 30.165807537309526, 0.968390660024069), (18.03932873924058, 30.165807537309526, 0.9226428170221468), (18.1614653535548, 30.165807537309526, 0.8692403854713255), (18.45435013726976, 30.165807537309526, 0.6595368681750764), (18.74723492098472, 30.165807537309526, 0.4498333508788272), (19.04011970469968, 30.165807537309526, 0.240129833582578), (19.333004488414637, 30.165807537309526, 0.03042631628632886), (19.625889272129598, 30.165807537309526, -0.1792772010099204), (19.91877405584456, 30.165807537309526, -0.38898071830616954), (20.21165883955952, 30.165807537309526, -0.5986842356024187), (20.50454362327448, 30.165807537309526, -0.8083877528986678), (20.62314433727173, 30.165807537309526, -0.8692403854713255), (20.745280951585947, 30.165807537309526, -0.9226428170221468), (20.87048601183653, 30.165807537309526, -0.968390660024069), (20.998280319756706, 30.165807537309526, -1.0063088234142321), (21.128174767232675, 30.165807537309526, -1.0362521827213147), (21.259672208270747, 30.165807537309526, -1.0581061355015149), (21.39226936172794, 30.165807537309526, -1.0717870399573415), (21.568552030061348, 30.165807537309526, -1.0772425350604955), (21.701741405857092, 30.165807537309526, -1.0717870399573415), (21.834338559314286, 30.165807537309526, -1.0581061355015149), (21.965836000352358, 30.165807537309526, -1.0362521827213147), (22.095730447828327, 30.165807537309526, -1.0063088234142321), (22.223524755748503, 30.165807537309526, -0.968390660024069), (22.348729815999086, 30.165807537309526, -0.9226428170221468), (22.470866430313304, 30.165807537309526, -0.8692403854713255), (22.763751214028265, 30.165807537309526, -0.6595368681750764), (23.056635997743225, 30.165807537309526, -0.4498333508788272), (23.349520781458185, 30.165807537309526, -0.240129833582578), (23.642405565173142, 30.165807537309526, -0.03042631628632886), (23.935290348888103, 30.165807537309526, 0.1792772010099204), (24.228175132603063, 30.165807537309526, 0.38898071830616954), (24.521059916318023, 30.165807537309526, 0.5986842356024187), (24.813944700032984, 30.165807537309526, 0.8083877528986678), (24.932545414030233, 30.165807537309526, 0.8692403854713255), (25.054682028344452, 30.165807537309526, 0.9226428170221468), (25.179887088595034, 30.165807537309526, 0.968390660024069), (25.30768139651521, 30.165807537309526, 1.0063088234142321), (25.43757584399118, 30.165807537309526, 1.0362521827213147), (25.56907328502925, 30.165807537309526, 1.0581061355015149), (25.701670438486445, 30.165807537309526, 1.0717870399573415), (25.877953106819852, 30.165807537309526, 1.0772425350604955), (26.011142482615597, 30.165807537309526, 1.0717870399573415), (26.14373963607279, 30.165807537309526, 1.0581061355015149), (26.275237077110862, 30.165807537309526, 1.0362521827213147), (26.40513152458683, 30.165807537309526, 1.0063088234142321), (26.532925832507008, 30.165807537309526, 0.968390660024069), (26.65813089275759, 30.165807537309526, 0.9226428170221468), (26.78026750707181, 30.165807537309526, 0.8692403854713255), (27.07315229078677, 30.165807537309526, 0.6595368681750764), (27.36603707450173, 30.165807537309526, 0.4498333508788272), (27.65892185821669, 30.165807537309526, 0.240129833582578), (27.951806641931647, 30.165807537309526, 0.03042631628632886), (28.244691425646607, 30.165807537309526, -0.1792772010099204), (28.537576209361568, 30.165807537309526, -0.38898071830616954), (28.830460993076528, 30.165807537309526, -0.5986842356024187), (29.12334577679149, 30.165807537309526, -0.8083877528986678), (29.241946490788738, 30.165807537309526, -0.8692403854713255), (29.364083105102956, 30.165807537309526, -0.9226428170221468), (29.48928816535354, 30.165807537309526, -0.968390660024069), (29.617082473273715, 30.165807537309526, -1.0063088234142321), (29.746976920749685, 30.165807537309526, -1.0362521827213147), (29.878474361787756, 30.165807537309526, -1.0581061355015149), (30.01107151524495, 30.165807537309526, -1.0717870399573415), (30.187354183578357, 30.165807537309526, -1.0772425350604955), (30.3205435593741, 30.165807537309526, -1.0717870399573415), (30.453140712831296, 30.165807537309526, -1.0581061355015149), (30.584638153869367, 30.165807537309526, -1.0362521827213147), (30.714532601345336, 30.165807537309526, -1.0063088234142321), (30.842326909265513, 30.165807537309526, -0.968390660024069), (30.967531969516095, 30.165807537309526, -0.9226428170221468), (31.089668583830314, 30.165807537309526, -0.8692403854713255), (31.382553367545274, 30.165807537309526, -0.6595368681750764), (31.675438151260234, 30.165807537309526, -0.4498333508788272), (31.968322934975195, 30.165807537309526, -0.240129833582578), (32.26120771869015, 30.165807537309526, -0.03042631628632886), (32.55409250240511, 30.165807537309526, 0.1792772010099204), (32.84697728612007, 30.165807537309526, 0.38898071830616954), (33.13986206983503, 30.165807537309526, 0.5986842356024187), (33.43274685354999, 30.165807537309526, 0.8083877528986678), (33.55134756754724, 30.165807537309526, 0.8692403854713255), (33.67348418186146, 30.165807537309526, 0.9226428170221468), (33.79868924211204, 30.165807537309526, 0.968390660024069), (33.92648355003222, 30.165807537309526, 1.0063088234142321), (34.05637799750819, 30.165807537309526, 1.0362521827213147), (34.187875438546264, 30.165807537309526, 1.0581061355015149), (34.320472592003455, 30.165807537309526, 1.0717870399573415), (34.49675526033686, 30.165807537309526, 1.0772425350604955), (34.629944636132606, 30.165807537309526, 1.0717870399573415), (34.7625417895898, 30.165807537309526, 1.0581061355015149), (34.89403923062787, 30.165807537309526, 1.0362521827213147), (35.02393367810384, 30.165807537309526, 1.0063088234142321), (35.15172798602402, 30.165807537309526, 0.968390660024069), (35.2769330462746, 30.165807537309526, 0.9226428170221468), (35.39906966058882, 30.165807537309526, 0.8692403854713255), (35.69195444430378, 30.165807537309526, 0.6595368681750764), (35.98483922801874, 30.165807537309526, 0.4498333508788272), (36.2777240117337, 30.165807537309526, 0.240129833582578), (36.57060879544866, 30.165807537309526, 0.03042631628632886), (36.86349357916362, 30.165807537309526, -0.1792772010099204), (37.15637836287858, 30.165807537309526, -0.38898071830616954), (37.44926314659354, 30.165807537309526, -0.5986842356024187), (37.7421479303085, 30.165807537309526, -0.8083877528986678), (37.860748644305744, 30.165807537309526, -0.8692403854713255), (37.982885258619966, 30.165807537309526, -0.9226428170221468), (38.108090318870545, 30.165807537309526, -0.968390660024069), (38.23588462679073, 30.165807537309526, -1.0063088234142321), (38.3657790742667, 30.165807537309526, -1.0362521827213147), (38.49727651530477, 30.165807537309526, -1.0581061355015149), (38.62987366876196, 30.165807537309526, -1.0717870399573415), (38.806156337095366, 30.165807537309526, -1.0772425350604955), (38.93934571289111, 30.165807537309526, -1.0717870399573415), (39.0719428663483, 30.165807537309526, -1.0581061355015149), (39.20344030738637, 30.165807537309526, -1.0362521827213147), (39.33333475486234, 30.165807537309526, -1.0063088234142321), (39.461129062782526, 30.165807537309526, -0.968390660024069), (39.586334123033105, 30.165807537309526, -0.9226428170221468), (39.70847073734733, 30.165807537309526, -0.8692403854713255), (40.00135552106229, 30.165807537309526, -0.6595368681750764), (40.29424030477725, 30.165807537309526, -0.4498333508788272), (40.58712508849221, 30.165807537309526, -0.240129833582578), (40.88000987220717, 30.165807537309526, -0.03042631628632886), (41.17289465592212, 30.165807537309526, 0.1792772010099204), (41.46577943963708, 30.165807537309526, 0.38898071830616954), (41.75866422335204, 30.165807537309526, 0.5986842356024187), (42.051549007067, 30.165807537309526, 0.8083877528986678), (42.17014972106425, 30.165807537309526, 0.8692403854713255), (42.29228633537847, 30.165807537309526, 0.9226428170221468), (42.41749139562905, 30.165807537309526, 0.968390660024069), (42.54528570354923, 30.165807537309526, 1.0063088234142321), (42.6751801510252, 30.165807537309526, 1.0362521827213147), (42.806677592063274, 30.165807537309526, 1.0581061355015149), (42.939274745520464, 30.165807537309526, 1.0717870399573415), (43.11555741385387, 30.165807537309526, 1.0772425350604955), (43.248746789649616, 30.165807537309526, 1.0717870399573415), (43.381343943106806, 30.165807537309526, 1.0581061355015149), (43.51284138414488, 30.165807537309526, 1.0362521827213147), (43.64273583162085, 30.165807537309526, 1.0063088234142321), (43.77053013954103, 30.165807537309526, 0.968390660024069), (43.89573519979161, 30.165807537309526, 0.9226428170221468), (44.01787181410583, 30.165807537309526, 0.8692403854713255), (44.31075659782079, 30.165807537309526, 0.6595368681750764), (44.60364138153575, 30.165807537309526, 0.4498333508788272), (44.89652616525071, 30.165807537309526, 0.240129833582578), (45.18941094896567, 30.165807537309526, 0.03042631628632886), (45.482295732680626, 30.165807537309526, -0.1792772010099204), (45.77518051639559, 30.165807537309526, -0.38898071830616954), (46.06806530011055, 30.165807537309526, -0.5986842356024187), (46.36095008382551, 30.165807537309526, -0.8083877528986678), (46.47955079782275, 30.165807537309526, -0.8692403854713255), (46.601687412136975, 30.165807537309526, -0.9226428170221468), (46.726892472387554, 30.165807537309526, -0.968390660024069), (46.85468678030774, 30.165807537309526, -1.0063088234142321), (46.98458122778371, 30.165807537309526, -1.0362521827213147), (47.11607866882178, 30.165807537309526, -1.0581061355015149), (47.24867582227897, 30.165807537309526, -1.0717870399573415), (51.558076899037474, 30.165807537309526, -1.0717870399573415)]
    path = np.array(path)
    yarn.generate_yarn_along_path(path)
    yarn.save_yarn_to_obj_file("output/customized_yarn.obj")
    print("Customized yarn generated successfully")

if __name__ == "__main__":
    main()