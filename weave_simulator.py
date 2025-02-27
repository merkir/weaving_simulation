import numpy as np
import sys
import os
import math
import json
from dataclasses import dataclass
from yarn_simulation import YarnSimulation
import scipy.interpolate as interpolate


class FabricSimulation:
    def __init__(self, pattern_data, weft_yarns, warp_yarns):
        """
        Initialize with pattern data in 2D matrix form where:
        - 1 represents weft over warp
        - 0 represents warp over weft
        
        The fabric is generated in the X-Y plane, with Z representing height.
        Weft yarns run along the X-axis (horizontal)
        Warp yarns run along the Y-axis (vertical)
        """
        # Properties
        self.weft_spacing_factor = 1     # packedness between weft yarns, 1 to inf
        self.warp_spacing_factor = 1     # packedness between warp yarns, 1 for exactly spaced without compression
        self.control_points_per_turning = 8   # smoothness of turning
        self.compress_fractor = 1.3
         
        # Components
        self.pattern = np.array(pattern_data, dtype=int)
        self.weft_count = len(self.pattern)
        self.warp_count = len(self.pattern[0]) if self.weft_count > 0 else 0

        # yarn information 
        self.weft_yarns = weft_yarns
        self.warp_yarns = warp_yarns

        # intersections
        self.control_grid = None

        # paths points
        self.weft_paths = None
        self.warp_paths = None
    
    def __generate_control_grid_and_yarn_paths(self):
        """
        Generate the control grid points for the fabric.
        Fabric is in X-Y plane, with Z as height.
        """
        max_warp_diameter = 0
        for yarn in self.warp_yarns:
            max_warp_diameter = max(max_warp_diameter, yarn.yarn_diameter)
        max_weft_diameter = 0
        for yarn in self.weft_yarns:
            max_weft_diameter = max(max_weft_diameter, yarn.yarn_diameter)

        print(f"max_warp_diameter: {max_warp_diameter}, max_weft_diameter: {max_weft_diameter}")

        # positions of warp yarns along x-axis
        warp_centers_x = [0]  # First warp at origin
        for i in range(self.warp_count-1):
            # Sum of current and next warp radii, multiplied by spacing factor
            next_center = warp_centers_x[-1] + (self.warp_yarns[i].yarn_diameter/2 + max_weft_diameter +
                                              self.warp_yarns[i+1].yarn_diameter/2) * self.warp_spacing_factor
            warp_centers_x.append(next_center)
        
        # positions of weft yarns along y-axis
        weft_centers_y = [0]
        for i in range(self.weft_count-1):
            next_center = weft_centers_y[-1] + (self.weft_yarns[i].yarn_diameter/2 + max_warp_diameter + 
                                              self.weft_yarns[i+1].yarn_diameter/2) * self.weft_spacing_factor
            weft_centers_y.append(next_center)
        
        print(f"Warp centers (x): {warp_centers_x}")
        print(f"Weft centers (y): {weft_centers_y}")
        
        # Now calculate the height (z) at each intersection based on yarn diameters and pattern
        self.control_grid = [[None for _ in range(self.warp_count)] for _ in range(self.weft_count)]
        self.weft_paths_centers = [[None for _ in range(self.warp_count)] for _ in range(self.weft_count)]
        self.warp_paths_centers = [[None for _ in range(self.weft_count)] for _ in range(self.warp_count)]
        
        for i in range(self.weft_count):
            for j in range(self.warp_count):
                x, y = warp_centers_x[j], weft_centers_y[i]
                self.control_grid[i][j] = (x, y, 0) # z=0, no height
                
                # Calculate z-coord for weft and warp the (x,y) intersection based on radius and tension
                weft_yarn, warp_yarn = self.weft_yarns[i], self.warp_yarns[j]
                weft_radius, warp_radius = weft_yarn.yarn_diameter / 2, warp_yarn.yarn_diameter / 2 # radius
                # print(f" tensions: {weft_yarn.tension}, {warp_tension.tension}")
                weft_tension_inverse, warp_tension_inverse = 1/weft_yarn.tension, 1/warp_yarn.tension # tension
                weft_tension_inverse_frac, warp_tension_inverse_frac = weft_tension_inverse/(weft_tension_inverse+warp_tension_inverse), warp_tension_inverse/(weft_tension_inverse+warp_tension_inverse)
                
                # Adjust z position based on which yarn is on top and their tensions
                weft_z, warp_z = 0,0 

                if self.pattern[i,j] == 1:  # weft on top
                    weft_z = (weft_radius + warp_radius) * weft_tension_inverse_frac
                    warp_z = -(weft_radius + warp_radius) * warp_tension_inverse_frac
                else:  # warp
                    weft_z = -(weft_radius + warp_radius) * weft_tension_inverse_frac
                    warp_z = (weft_radius + warp_radius) * warp_tension_inverse_frac

                self.weft_paths_centers[i][j], self.warp_paths_centers[j][i] = np.array([x,y,weft_z]), np.array([x, y, warp_z])

        self.__smoothen_and_set_paths()

        # self.__set_yarn_paths(self.weft_paths_centers, self.weft_yarns, np.array([1,0,0]))
        self.__set_yarn_paths(self.warp_paths_centers, self.warp_yarns, np.array([0,1,0]))


    def __smoothen_and_set_paths(self):
        self.weft_paths = [[] for _ in range(self.weft_count)]
        for i in range(self.weft_count):
            for j in range(self.warp_count):
                # skip the before-first segments
                if j != 0:
                    weft_segment = self.__generate_smooth_weft_segment(i,j)
                    self.weft_paths[i].extend(weft_segment)
                # if i != 0:
                #     warp_segment = self.__generate_warp_segment(i,j)
            print(f"weft_paths[i]: {self.weft_paths[i]} ")
            self.weft_yarns[i].set_yarn_path(self.weft_paths[i])

                
    # https://redblobgames.github.io/circular-obstacle-pathfinding/
    def __generate_smooth_weft_segment(self, i,j):

        """geneate weft segment form the pervious intersection (i,j-1) to the current one (i-j)"""
        prev_pattern, curr_pattern =  self.pattern[i][j-1], self.pattern[i][j]

        prev_warp_center, curr_warp_center = self.warp_paths_centers[j-1][i], self.warp_paths_centers[j][i]

        if prev_pattern == curr_pattern:
            raise ValueError("currently not supported, sorry!")
    
        weft_radius = self.weft_yarns[i].yarn_diameter/2
        prev_warp_radius, current_warp_radius = self.warp_yarns[j-1].yarn_diameter/2, self.warp_yarns[j].yarn_diameter/2
        
        rA, rB = prev_warp_radius + weft_radius, current_warp_radius + weft_radius
        forward = curr_warp_center - prev_warp_center
        forward_norm = np.linalg.norm(forward)
        theta = math.acos((rA +rB)/forward_norm)

        print(f"prev_warp_center: {prev_warp_center}, curr_warp_center: {curr_warp_center}, forward: {forward}, rA, rB : {rA}, {rB}")

        if prev_pattern == 1:
            # weft top -> under
            forward_angle = math.acos(-forward[2] / forward_norm) # angle between forward vector and (0,0,-1)
        else:
            # weft under -> top
            forward_angle = math.acos(forward[2] / forward_norm) # angle between forward vector and (0,0,1)

        circular_angle = math.pi - theta - forward_angle
        
        print(f"theta: {theta}, forward_angle: {forward_angle}, circular_angle: {circular_angle}")

        compressed_circular_angle = circular_angle / self.compress_fractor 

        # draw control points around the turning
        segment = []
        offset_angle = 0.01
        compressed_circular_angle_slice = compressed_circular_angle / self.control_points_per_turning

        if prev_pattern == 1:
            # the half on previous warp
            for i in range(self.control_points_per_turning):
                point_angle = offset_angle + i * compressed_circular_angle_slice
                segment_point = prev_warp_center + rA * np.array([math.sin(point_angle), 0, math.cos(point_angle)])
                segment.append(segment_point)

                print(f"poin: {segment_point}")
            
            print("next half")
            # the halp on the next warp
            for i in range(self.control_points_per_turning):
                point_angle = compressed_circular_angle + offset_angle - i * compressed_circular_angle_slice
                segment_point = curr_warp_center + rB *  np.array([-math.sin(point_angle), 0, -math.cos(point_angle)])
                segment.append(segment_point)
                print(f"point: {segment_point}")
        else:
            # the half on previous warp
            for i in range(self.control_points_per_turning):
                point_angle = offset_angle + i * compressed_circular_angle_slice
                segment_point = prev_warp_center + rA *  np.array([math.sin(point_angle), 0, -math.cos(point_angle)])
                segment.append(segment_point)

                print(f"poin: {segment_point}")
            
            print("next half")
            # the halp on the next warp
            for i in range(self.control_points_per_turning):
                point_angle = compressed_circular_angle + offset_angle - i * compressed_circular_angle_slice
                segment_point = curr_warp_center + rB *  np.array([-math.sin(point_angle), 0, math.cos(point_angle)])
                segment.append(segment_point)
                print(f"point: {segment_point}")


        return segment


    def __set_yarn_paths(self, target_paths_centers, target_yarns, direction):
        """direction: [1,0,0] for weft, [0,0,1] for warp"""
        for i in range(len(target_paths_centers)):
            """for each weft yarn, append an start and an end, then set path"""
            path_coord = target_paths_centers[i]

            # start
            first = path_coord[0]
            start_len = 2* target_yarns[0].yarn_diameter
            start = first - direction*start_len
            
            # end
            last = path_coord[-1]
            end_len = 2* target_yarns[-1].yarn_diameter
            end = last + direction * end_len

            path = [start] + path_coord + [end]
            target_yarns[i].set_yarn_path(path)
    

    def generate_obj_file(self, output_file):
        """
        Generate an OBJ file for the entire woven fabric.
        """
        # makedir
        os.makedirs(os.path.dirname(output_file), exist_ok=True)

        # Write OBJ file header
        with open(output_file, 'w') as f:
            f.write("# Woven fabric simulation OBJ file\n")
            f.write(f"# Weft count: {self.weft_count}\n")
            f.write(f"# Warp count: {self.warp_count}\n")
            
            vertex_offset = 0

            # f.write(f"o fabric\n")
            
            # weft
            for i in range(self.weft_count):
                yarn = self.weft_yarns[i]
            
                # generate obj
                f.write(f"o weft_{i}\n")
                vertices, faces = yarn.generate_yarn()
                for v in vertices:
                    f.write(f"v {v[0]} {v[1]} {v[2]}\n")
                for face in faces:
                    f.write(f"f {face[0] + 1 + vertex_offset} {face[1] + 1 + vertex_offset} {face[2] + 1 + vertex_offset}\n")
            
                vertex_offset += len(vertices)
            
            # Process warp yarns
            for j in range(self.warp_count):
                yarn = self.warp_yarns[j]
                
                # Start a new object for this yarn
                f.write(f"o warp_{j}\n")
                
                vertices, faces = yarn.generate_yarn()
                for v in vertices:
                    f.write(f"v {v[0]} {v[1]} {v[2]}\n")
                for face in faces:
                    f.write(f"f {face[0] + 1 + vertex_offset} {face[1] + 1 + vertex_offset} {face[2] + 1 + vertex_offset}\n")
            
                # Update vertex offset
                vertex_offset += len(vertices)
            
        
        return output_file
    
    def simulate_fabric(self):
        self.__generate_control_grid_and_yarn_paths()
        print("Control grid and yarn paths generated.")

def load_pattern(input_file):
    """Load a weaving pattern from a text file."""
    pattern_data = []
    with open(input_file, 'r') as file:
        for line in file:
            # Convert each line to array of integers
            row = [int(cell.strip()) for cell in line.strip().split() if cell.strip()]
            if row:  # Only add non-empty rows
                pattern_data.append(row)
    return pattern_data

def load_yarn_properties(input_file):
    """Load yarn properties from a JSON file."""
    with open(input_file, 'r') as file:
        return json.load(file)

def load_yarn_pattern(input_file):
    """Load yarn pattern (which yarns to use) from a JSON file."""
    with open(input_file, 'r') as file:
        yarn_pattern = json.load(file)
    
    return yarn_pattern["wefts"], yarn_pattern["warps"]

def load_and_validate_yarn_and_pattern():
    """Load pattern and yarn definitions from files."""
    
    # Load pattern
    pattern_file = "weaving_patterns/simple_pattern.txt"
    pattern = load_pattern(pattern_file)
    
    # Load yarn properties
    yarn_properties = load_yarn_properties("weaving_patterns/yarn_properties.json")
    
    # Load yarn pattern
    weft_yarn_names, warp_yarn_names = load_yarn_pattern("weaving_patterns/yarn_pattern.json")
    
    # Validate
    if len(pattern) != len(weft_yarn_names):
        raise ValueError(f"Pattern has {len(pattern)} weft rows but yarn pattern has {len(weft_yarn_names)}")
    
    if len(pattern[0]) != len(warp_yarn_names):
        raise ValueError(f"Pattern has {len(pattern[0])} warp columns but yarn pattern has {len(warp_yarn_names)}")
    
    # Create yarn instances
    weft_yarns = []
    for yarn_name in weft_yarn_names:
        if yarn_name not in yarn_properties:
            raise ValueError(f"Unknown weft yarn property: {yarn_name}")
        
        prop = yarn_properties[yarn_name]
        # Add default tension if not specified
        tension = prop.get('tension', 1.0)
        
        weft_yarns.append(
            YarnSimulation(
                strand_diameter=prop['strand_diameter'],
                strand_count=prop['strand_count'],
                color=tuple(prop['color'])
            )
        )
        # Add tension as attribute to the yarn instance
        weft_yarns[-1].tension = tension
    
    warp_yarns = []
    for yarn_name in warp_yarn_names:
        if yarn_name not in yarn_properties:
            raise ValueError(f"Unknown warp yarn property: {yarn_name}")
        
        prop = yarn_properties[yarn_name]
        # Add default tension if not specified
        tension = prop.get('tension', 1.0)
        
        warp_yarns.append(
            YarnSimulation(
                strand_diameter=prop['strand_diameter'],
                strand_count=prop['strand_count'],
                color=tuple(prop['color'])
            )
        )
        # Add tension as attribute to the yarn instance
        warp_yarns[-1].tension = tension
    
    return pattern, weft_yarns, warp_yarns

def main():
    # Create output directory
    os.makedirs("output", exist_ok=True)
    
    try:
        # Load pattern and yarn definitions
        pattern, weft_yarns, warp_yarns = load_and_validate_yarn_and_pattern()
        
        print(f"Loaded input files successfully.")
        print(f"Pattern size: {len(pattern)} rows x {len(pattern[0])} columns")
        print(f"Weft yarns: {len(weft_yarns)}")
        print(f"Warp yarns: {len(warp_yarns)}")
        
        # Create simulator
        simulator = FabricSimulation(pattern, weft_yarns, warp_yarns)
        
        # Run simulation
        simulator.simulate_fabric()
        
        # Generate OBJ file
        output_file = "output/woven_fabric.obj"
        simulator.generate_obj_file(output_file)
        
        print(f"Woven fabric simulation saved to {output_file}")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()