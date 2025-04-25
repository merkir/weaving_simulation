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
import utils


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
        
        self.fabricYarnSimulations = FabricYarnSimulations(
            len(pattern_data), len(pattern_data[0]), pattern_data, weft_yarns, warp_yarns)

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
        for yarn in self.fabricYarnSimulations.weft_yarns:
            max_weft_diameter = max(max_weft_diameter, yarn.yarn_diameter)

        warp_centers_x = [0]  # First warp at origin
        for i in range(self.fabricYarnSimulations.warp_count-1):
            # Sum of current and next warp radii, multiplied by spacing factor
            next_center = warp_centers_x[-1] + (self.fabricYarnSimulations.warp_yarns[i].yarn_diameter/2 + max_weft_diameter +
                                              self.fabricYarnSimulations.warp_yarns[i+1].yarn_diameter/2) * self.fabricProperties.warp_spacing_factor
            warp_centers_x.append(next_center)
        
        return warp_centers_x

    def __get_max_warp_diameter(self):
        max_warp_diameter = 0
        for yarn in self.fabricYarnSimulations.warp_yarns:
            max_warp_diameter = max(max_warp_diameter, yarn.yarn_diameter)
        return max_warp_diameter
    
    def simulate_fabric(self):
        warp_centers_x = self.generate_warp_centers_x()
        max_warp_diameter = self.__get_max_warp_diameter()
        self.control_grid, self.weft_paths_centers, self.warp_paths_centers, self.weft_paths_warp, self.warp_paths_weft = multi_cloth_manager.generate_control_grid_and_yarn_paths(
            self.fabricYarnSimulations.weft_count, self.fabricYarnSimulations.warp_count, self.fabricYarnSimulations.pattern,
            warp_centers_x, max_warp_diameter, self.fabricYarnSimulations.weft_yarns, self.fabricYarnSimulations.warp_yarns, 
            self.fabricProperties.weft_spacing_factor)
        # print(f"weft_paths_centers: {self.weft_paths_centers}, warp_paths_centers:{self.warp_paths_centers}")
        
        # print(f"weft_paths_warp: {self.weft_paths_warp}, warp_paths_weft: {self.warp_paths_weft}")
        # self.weft_paths, self.warp_paths = smooth_turning.smoothen_and_set_paths(self)
        # for i in range(self.fabricYarnSimulations.weft_count):
        #     if self.weft_paths[i]:
        #         self.weft_yarns[i].set_yarn_path(self.weft_paths[i])
        # for j in range(self.fabricYarnSimulations.warp_count):
        #     if self.warp_paths[i]:
        #         self.warp_yarns[j].set_yarn_path(self.warp_paths[j])


        self.weft_paths, self.warp_paths = self.weft_paths_centers, self.warp_paths_centers
        for i in range(self.fabricYarnSimulations.weft_count):
            if self.weft_paths[i]:
                self.fabricYarnSimulations.weft_yarns[i].set_yarn_path(self.weft_paths[i])
        for j in range(self.fabricYarnSimulations.warp_count):
            if self.warp_paths[i]:
                self.fabricYarnSimulations.warp_yarns[j].set_yarn_path(self.warp_paths[j])

        # self.__generate_control_grid_and_yarn_paths()
        print("Control grid and yarn paths generated.")




def main():
    # Create output directory
    os.makedirs("output", exist_ok=True)
    
    try:
        # Load pattern and yarn definitions
        pattern, weft_yarns, warp_yarns = utils.load_and_validate_yarn_and_pattern()
        
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
        utils.generate_obj_file(len(weft_yarns), len(warp_yarns), weft_yarns, warp_yarns ,output_file)
        
        print(f"Woven fabric simulation saved to {output_file}")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()