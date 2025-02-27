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
        self.packing_density = 0.9    # How densely the strands are packed

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

        yarn_center_to_strand_center_dis = self.strand_diameter / 2
        if self.strand_count == 3:
            yarn_center_to_strand_center_dis = self.strand_diameter / np.sqrt(3)
        elif  self.strand_count == 4:
            yarn_center_to_strand_center_dis = self.strand_diameter / np.sqrt(2)

        
        # Generate each point on the strand
        for i in range(path_points):
            center = self.path[i]
            
            # Calculate twist angle based on accumulated distance
            distance = accumulated_distance[i]
            angle = 2 * math.pi * self.twist_rate * distance
            total_angle = angle + strand_offset_angle
            
            # Calculate direction vectors for local coordinate system
            if i < path_points - 1:
                forward = self.path[i+1] - center
            else:
                # forward = center - self.path[i-1] if i > 0 else np.array([1, 0, 0])
                forward = np.array([1,0,0])
            
            # Normalize forward vector
            forward_length = np.linalg.norm(forward)
            if forward_length > 0:
                forward = forward / forward_length
            
            # Calculate perpendicular vectors (right and up)

            if forward[1] == 0:
                up = np.array([0,1,0])
                right = np.cross(forward, up)
                if np.linalg.norm(right) :
                    right = right / np.linalg.norm(right)
            elif forward[0] == 0:
                right = np.array([1,0,0])
                up = np.cross(forward, right)
                if np.linalg.norm(up) :
                    up = up / np.linalg.norm(up)


            # if abs(forward[0]) < 0.999:  # Not aligned with X axis
            #     # Create a vector perpendicular to forward in the XZ plane
            #     # up = np.array([0, -forward[2], forward[1]])  # Start with Y axis
            #     up = np.array([0, 1, 0])  # Start with Y axis
            #     up = up - forward * np.dot(forward, up)
            #     # print(f"after up:{up}")
            #     up = up / np.linalg.norm(up)

            #     # z always pos
            #     if (up[1] < 0): up = -up
                
            #     right = np.cross(forward, up)
            #     if np.linalg.norm(right) :
            #         right = right / np.linalg.norm(right)

            #     # y alwyas pos
            #     if (right[2]<0): right = -right
            
            # else:
            #     # If aligned with X axis, use Z axis as up
            #     up = np.array([0,0,1])
            #     right = np.array([0,1,0])
            
            # print(f"total_angle: {total_angle}, cos: {math.cos(total_angle)}, sin: {math.sin(total_angle)}, forward:{forward} up:{up}, right: {right}")
            # print(f"center: {center}, right contribution: {right * math.cos(total_angle)}, up contributrion: {up * math.sin(total_angle)}")
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
        
        # create for each strand tube
        for strand in self.strands:
            if strand is None:
                continue
                
            segments = len(strand) - 1
            radius = self.strand_diameter / 2 * self.packing_density
            
            for i in range(len(strand)):
                # Calculate direction vector
                if i < len(strand) - 1:
                    direction = strand[i+1] - strand[i]
                else:
                    direction = strand[i] - strand[i-1] if i > 0 else np.array([1, 0, 0])
                # print(f"direction vector: {direction}")

                # normalize
                direction_length = np.linalg.norm(direction)
                if direction_length > 0:
                    direction = direction / direction_length
                
                if abs(direction[1]) > 0.001 or abs(direction[2]) > 0.001:
                    perpendicular = np.array([0, -direction[2], direction[1]])
                else:
                    perpendicular = np.array([0, 1, 0])
                perpendicular = perpendicular / np.linalg.norm(perpendicular)
                
                # #  third perpendicular vector
                third = np.cross(direction, perpendicular)

                #  vertice on ring
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
    path = [(2.0, -2, -0.5), (2.0, 0, -0.5), (2.0, 2.0, 0.5), (2.0, 4.0, -0.5), (2.0, 6.0, 0.5), (2.0, 8.0, 0.5)]
    path = np.array(path)
    yarn.generate_yarn_along_path(path)
    yarn.save_yarn_to_obj_file("output/customized_yarn.obj")
    print("Customized yarn generated successfully")

if __name__ == "__main__":
    main()