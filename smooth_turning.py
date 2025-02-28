import numpy as np
import math

def smoothen_and_set_paths(self):
    weft_paths = [[] for _ in range(self.weft_count)]
    # weft [i] from warp [j-1] to warp [j] (j-1 segment, straight line, j segment)
    for i in range(self.weft_count):
        print(f"weft_paths_warp {i}: {self.weft_paths_warp[i]}")
        print(f"weft_paths_centers {i}: {self.weft_paths_centers[i]}")
        for jIdx  in range(len(self.weft_paths_warp[i])):
            if jIdx != 0:
                weft_segment = __generate_smooth_weft_segment(self, i,jIdx)
                if weft_segment:
                    weft_paths[i].extend(weft_segment)
        
        print(f"weft_paths {i}: {weft_paths[i]}")
        weft_paths[i] = __add_extra_start_and_end(self, weft_paths[i], self.weft_yarns[i].yarn_diameter, "weft")
        # self.weft_yarns[i].set_yarn_path(self.weft_paths[i])


    warp_paths = [[] for _ in range(self.warp_count)]
    for j in range(self.warp_count):
        idx = 1 # skip first
        skip_prev = False
        while idx < len(self.warp_paths_weft[j]):
            if not skip_prev: 
                warp_segment = __generate_smooth_warp_segment(self, idx, idx-1, j)
            else:
                warp_segment = __generate_smooth_warp_segment(self, idx, idx-2, j)

            if not warp_segment:
                # ignore
                skip_prev = True
            else:
                skip_prev = False
                warp_paths[j].extend(warp_segment)

            idx = idx+ 1

        warp_paths[j] = __add_extra_start_and_end(self, warp_paths[j], self.warp_yarns[j].yarn_diameter, "warp")
        # self.warp_yarns[j].set_yarn_path(self.warp_paths[j])
    
    return weft_paths, warp_paths

    """
    print("----debug---")
    print(f"control grid: {self.control_grid}")
    print("")
    for i in range(self.weft_count):
        print(f"** weft_path_centers {i}: {self.weft_paths_centers[i]})")
        print(f"** weft_path {i}: {[tuple([float(coord) for coord in point] ) for point in self.weft_paths[i]]}")
        print("")
    for j in range(self.warp_count):
        print(f"** warp_path_centers {j}: {self.warp_paths_centers[j]})")
        print(f"** warp_path {j}: {[tuple([float(coord) for coord in point] ) for point in self.warp_paths[i]]}")
        print("")
    """

def __smoothen_line_segment(self, start, end):
        line_segment = end - start
        smooth_segment = []
        for i in range(1, self.control_points_per_turning):
            alph = i / self.control_points_per_turning 
            smooth_segment.append(start +  alph * line_segment)
        return smooth_segment
    
def __add_extra_start_and_end(self, path_centers, yarn_diameter, type):
    direction = np.array([1,0,0]) if type == "weft" else np.array([0,1,0])

    first = path_centers[0]
    start = first - direction *2* yarn_diameter
    start_segment = __smoothen_line_segment(self, start, first)

    # end
    last = path_centers[-1]
    end = last + direction * 2* yarn_diameter
    end_segment = __smoothen_line_segment(self, last, end)

    return start_segment + path_centers + end_segment



def __generate_smooth_warp_segment(self, iIdx,previIdx, j):
    prevI, i = self.warp_paths_weft[j][previIdx], self.warp_paths_weft[j][iIdx]

    """geneate weft segment form the pervious intersection (i,j-1) to the current one (i-j)"""
    prev_pattern, curr_pattern =  self.pattern[prevI][j], self.pattern[i][j]

    prev_weft_center, curr_weft_center = self.warp_paths_centers[j][previIdx], self.weft_paths_centers[j][iIdx]

    if prev_pattern == curr_pattern:
        if prev_pattern == 0 and prev_weft_center[2] >= curr_weft_center[2]: # warp upper
            return None
        if prev_pattern == 1 and prev_weft_center[2] <= curr_weft_center[2]: # warp under 
            return None
        return None
        print("this shouldnt happen")
    
    if i != len(self.warp_paths_weft)-1:
        next_pattern, next_weft_center = self.pattern[i+1][j], self.weft_paths_centers[j][iIdx+1]
        if curr_pattern == next_pattern:
            if curr_pattern == 0 and curr_weft_center[2] <= next_weft_center[2]:
                return None
            if curr_pattern == 1 and curr_weft_center[2] >= next_weft_center[2]:
                return None
            
    warp_radius = self.warp_yarns[i].yarn_diameter/2
    prev_weft_radius, current_weft_radius = self.weft_yarns[prevI].yarn_diameter/2, self.weft_yarns[i].yarn_diameter/2
    
    rA, rB = (prev_weft_radius + warp_radius)/self.compress_fractor , (current_weft_radius + warp_radius)/self.compress_fractor 
    forward = curr_weft_center - prev_weft_center
    forward_norm = np.linalg.norm(forward/self.compress_fractor )
    theta = math.acos((rA +rB)/forward_norm)

    if prev_pattern == 0:
        # warp under -> top
        forward_angle = math.acos(-forward[2] / forward_norm) # angle between forward vector and (0,0,-1)
    else:
        # warp under -> top
        forward_angle = math.acos(forward[2] / forward_norm) # angle between forward vector and (0,0,1)

    circular_angle = math.pi - theta - forward_angle

    compressed_circular_angle = circular_angle

    # draw control points around the turning
    segment = []
    offset_angle = 0
    compressed_circular_angle_slice = compressed_circular_angle / self.control_points_per_turning

    if prev_pattern == 0:
        # the half on previous warp
        for i in range(self.control_points_per_turning):
            point_angle = offset_angle + i * compressed_circular_angle_slice
            segment_point = prev_weft_center + rA * np.array([ 0, math.sin(point_angle), math.cos(point_angle)])
            segment.append(segment_point)
        
        # the halp on the next warp
        for i in range(self.control_points_per_turning):
            point_angle = compressed_circular_angle + offset_angle - i * compressed_circular_angle_slice
            segment_point = curr_weft_center + rB *  np.array([ 0, -math.sin(point_angle), -math.cos(point_angle)])
            
            # middle
            if i == 0:
                middle_start = segment[-1]
                middle_segment = segment_point - middle_start
                for i in range(1, self.control_points_per_turning * (i-prevI)):
                    alph = i / (self.control_points_per_turning * (i-prevI))
                    segment.append(middle_start +  alph * middle_segment)

            segment.append(segment_point)
    else:
        # the half on previous warp
        for i in range(self.control_points_per_turning):
            point_angle = offset_angle + i * compressed_circular_angle_slice
            segment_point = prev_weft_center + rA *  np.array([ 0,math.sin(point_angle), -math.cos(point_angle)])
            segment.append(segment_point)
        
        # the halp on the next warp
        for i in range(self.control_points_per_turning):
            point_angle = compressed_circular_angle + offset_angle - i * compressed_circular_angle_slice
            segment_point = curr_weft_center + rB *  np.array([ 0,-math.sin(point_angle), math.cos(point_angle)])

            # middle
            if i == 0:
                middle_start = segment[-1]
                middle_segment = segment_point - middle_start
                for i in range(1, self.control_points_per_turning *  (i -prevI)):
                    alph = i / (self.control_points_per_turning *  (i -prevI))
                    segment.append(middle_start +  alph * middle_segment)
                    
            segment.append(segment_point)


    return segment


            
# https://redblobgames.github.io/circular-obstacle-pathfinding/
def __generate_smooth_weft_segment(self, i,jIdx):
    j, prevJ = self.weft_paths_warp[i][jIdx], self.weft_paths_warp[i][jIdx-1]

    print(f"__generate_smooth_weft_segment i: {i}, j:{j}, prevJ: {prevJ}")

    """geneate weft segment form the pervious intersection (i,j-1) to the current one (i-j)"""
    prev_pattern, curr_pattern =  self.pattern[i][prevJ], self.pattern[i][j]

    prev_warp_center, curr_warp_center = self.weft_paths_centers[i][jIdx-1], self.weft_paths_centers[i][jIdx]

    if prev_pattern == curr_pattern:
        return

    weft_radius = self.weft_yarns[i].yarn_diameter/2
    prev_warp_radius, current_warp_radius = self.warp_yarns[prevJ].yarn_diameter/2, self.warp_yarns[j].yarn_diameter/2
    
    rA, rB = prev_warp_radius + weft_radius, current_warp_radius + weft_radius
    forward = curr_warp_center - prev_warp_center
    forward_norm = np.linalg.norm(forward)
    theta = math.acos((rA +rB)/forward_norm)

    try :
        if prev_pattern == 1:
            # weft top -> under
            forward_angle = math.acos(-forward[2] / forward_norm) # angle between forward vector and (0,0,-1)
        else:
            # weft under -> top
            forward_angle = math.acos(forward[2] / forward_norm) # angle between forward vector and (0,0,1)
    except:
        print("wrong, return")

    circular_angle = math.pi - theta - forward_angle
    
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

        middle_start = segment[-1]

        # the halp on the next warp
        for i in range(self.control_points_per_turning):
            point_angle = compressed_circular_angle + offset_angle - i * compressed_circular_angle_slice
            segment_point = curr_warp_center + rB *  np.array([-math.sin(point_angle), 0, -math.cos(point_angle)])

            # middle
            if i == 0:
                middle_start = segment[-1]
                middle_segment = segment_point - middle_start
                for i in range(1, self.control_points_per_turning):
                    alph = i / self.control_points_per_turning 
                    segment.append(middle_start +  alph * middle_segment)

            segment.append(segment_point)
    else:
        # the half on previous warp
        for i in range(self.control_points_per_turning):
            point_angle = offset_angle + i * compressed_circular_angle_slice
            segment_point = prev_warp_center + rA *  np.array([math.sin(point_angle), 0, -math.cos(point_angle)])
            segment.append(segment_point)
        
        # the halp on the next warp
        for i in range(self.control_points_per_turning):
            point_angle = compressed_circular_angle + offset_angle - i * compressed_circular_angle_slice
            segment_point = curr_warp_center + rB *  np.array([-math.sin(point_angle), 0, math.cos(point_angle)])

            # middle
            if i == 0:
                middle_start = segment[-1]
                middle_segment = segment_point - middle_start
                for i in range(1, self.control_points_per_turning):
                    alph = i / self.control_points_per_turning 
                    segment.append(middle_start +  alph * middle_segment)

            segment.append(segment_point)

    return segment