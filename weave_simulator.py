import numpy as np
import sys
import os
import math
import json
from dataclasses import dataclass
from yarn_simulatior import YarnSimulation
import scipy.interpolate as interpolate

from typing import List

import smooth_turning
import multi_cloth_manager


@dataclass
class FabricProperties:
    weft_spacing_factor: float = 1
    warp_spacing_factor: float = 1
    control_points_per_turning: int = 8
    compress_factor: float = 1.1

@dataclass
class FabricYarnSimulations:
    weft_count: int
    warp_count: int
    pattern: np.array

    weft_yarns: List[YarnSimulation]
    warp_yarns: List[YarnSimulation]
        

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

        self.fabricProperties = FabricProperties()
        
        self.fabricYarnSimulations = FabricYarnSimulations()

        # intersection points
        self.control_grid = None

        # path centers
        self.weft_paths_centers = None
        self.warp_paths_centers = None

        # paths
        self.weft_paths = None
        self.warp_paths = None


    def generate_warp_centers_x(self):
        """positions of warp yarns on the x axis. not changing with weaving patterns"""

        max_weft_diameter = 0
        for yarn in self.weft_yarns:
            max_weft_diameter = max(max_weft_diameter, yarn.yarn_diameter)

        warp_centers_x = [0]  # First warp at origin
        for i in range(self.warp_count-1):
            # Sum of current and next warp radii, multiplied by spacing factor
            next_center = warp_centers_x[-1] + (self.warp_yarns[i].yarn_diameter/2 + max_weft_diameter +
                                              self.warp_yarns[i+1].yarn_diameter/2) * self.warp_spacing_factor
            warp_centers_x.append(next_center)
        
        return warp_centers_x


    def __generate_yarn_paths_with_control_grid(self):
        self.weft_paths_centers = [[None for _ in range(self.warp_count)] for _ in range(self.weft_count)]
        self.warp_paths_centers = [[None for _ in range(self.weft_count)] for _ in range(self.warp_count)]
        
        for i in range(self.weft_count):
            for j in range(self.warp_count):
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

                self.weft_paths_centers[i][j] = self.control_grid[i][j] + np.array([0,0, weft_z]) 
                self.warp_paths_centers[j][i] = self.control_grid[i][j] + np.array([0,0, warp_z])


        self.weft_paths, self.warp_paths = smooth_turning.smoothen_and_set_paths(self)
        for i in range(self.weft_count):
            self.weft_yarns[i].set_yarn_path(self.weft_paths[i])
        for j in range(self.warp_count):
            self.warp_yarns[j].set_yarn_path(self.warp_paths[j])
    
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

        # positions of warp yarns along x-axis
        warp_centers_x = [0]  # First warp at origin
        for i in range(self.warp_count-1):
            # Sum of current and next warp radii, multiplied by spacing factor
            next_center = warp_centers_x[-1] + (self.warp_yarns[i].yarn_diameter/2 + max_weft_diameter +
                                              self.warp_yarns[i+1].yarn_diameter/2) * self.warp_spacing_factor
            warp_centers_x.append(next_center)
        
        # positions of weft yarns along y-axis
        # support double cloth
        # weft_centers_yz = self.__get_weft_centers_yz(max_warp_diameter)
        weft_centers_y = [0]  # First warp at origin
        for i in range(self.weft_count-1):
            # Sum of current and next warp radii, multiplied by spacing factor
            next_center = weft_centers_y[-1] + (self.weft_yarns[i].yarn_diameter/2 + max_warp_diameter +
                                              self.weft_yarns[i+1].yarn_diameter/2) * self.weft_spacing_factor
            weft_centers_y.append(next_center)
        

        print(f"Warp centers (x): {warp_centers_x}")
        print(f"Weft centers (y, z): {weft_centers_y}")
        
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

        self.weft_paths, self.warp_paths = smooth_turning.smoothen_and_set_paths(self)
        for i in range(self.weft_count):
            self.weft_yarns[i].set_yarn_path(self.weft_paths[i])
        for j in range(self.warp_count):
            self.warp_yarns[j].set_yarn_path(self.warp_paths[j])


    

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
                print(f"generating weft {i}")
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
                print(f"generating warp {j}")
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

    def __get_max_warp_diameter(self):
        max_warp_diameter = 0
        for yarn in self.warp_yarns:
            max_warp_diameter = max(max_warp_diameter, yarn.yarn_diameter)
        return max_warp_diameter
    
    def simulate_fabric(self):
        warp_centers_x = self.generate_warp_centers_x()
        max_warp_diameter = self.__get_max_warp_diameter()
        self.control_grid, self.weft_paths_centers, self.warp_paths_centers, self.weft_paths_warp, self.warp_paths_weft = multi_cloth_manager.generate_control_grid_and_yarn_paths(
            self.weft_count, self.warp_count, self.pattern, warp_centers_x, max_warp_diameter, self.weft_yarns, self.warp_yarns, self.weft_spacing_factor)
        # print(f"weft_paths_centers: {self.weft_paths_centers}, warp_paths_centers:{self.warp_paths_centers}")
        
        # print(f"weft_paths_warp: {self.weft_paths_warp}, warp_paths_weft: {self.warp_paths_weft}")
        # self.weft_paths, self.warp_paths = smooth_turning.smoothen_and_set_paths(self)
        # for i in range(self.weft_count):
        #     if self.weft_paths[i]:
        #         self.weft_yarns[i].set_yarn_path(self.weft_paths[i])
        # for j in range(self.warp_count):
        #     if self.warp_paths[i]:
        #         self.warp_yarns[j].set_yarn_path(self.warp_paths[j])


        self.weft_paths, self.warp_paths = self.weft_paths_centers, self.warp_paths_centers
        for i in range(self.weft_count):
            if self.weft_paths[i]:
                self.weft_yarns[i].set_yarn_path(self.weft_paths[i])
        for j in range(self.warp_count):
            if self.warp_paths[i]:
                self.warp_yarns[j].set_yarn_path(self.warp_paths[j])

        # self.__generate_control_grid_and_yarn_paths()
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
    # pattern_file = "weaving_patterns/simple_pattern.txt"
    pattern_file = "weaving_patterns/double_cloth.txt"
    pattern = load_pattern(pattern_file)
    
    # Load yarn properties
    yarn_properties = load_yarn_properties("weaving_patterns/yarn_properties.json")
    
    # Load yarn pattern
    # weft_yarn_names, warp_yarn_names = load_yarn_pattern("weaving_patterns/yarn_pattern.json")
    weft_yarn_names, warp_yarn_names = load_yarn_pattern("weaving_patterns/yarn_pattern_double_cloth.json")
    
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