import math
import numpy as np

_warp_centers_x = None
_max_warp_diameter = None
_weft_yarns = None
_warp_yarns = None
_weft_spacing_factor = None
_weft_count = None
_warp_count = None

_control_grid = None

class Cloth:
    def __init__(self, width, prev_y):
        """store weft, warp, pattern in this cloth"""
        self.weft_to_pattern = {} # dict weft -> weft_pattern
        self.warp_indices = None # warps that constructs this cloth

        self.length = 0 # count of weft in the cloth
        self.width = width # global warp_count

        """height fucnction in respect to the prev single cloth point"""
        self.height_func = None

        self.prev_y = prev_y

        self.length_y = 0 # the length in y coordinates(projection)

    def add_weft_pattern(self, weft_idx, weft_pattern):
        """add a weft into the cloth"""
        if len(self.weft_to_pattern) == 0:
            self.weft_to_pattern[weft_idx] = weft_pattern
            self.length += 1
            return
        
        prev_pattern = list(self.weft_to_pattern.values())[-1]

        """a single weft does not determine the warp involved in the cloth, thus, we set warp indices only after 2 wefts have been collected"""
        if not self.warp_indices:
            # set warps
            self.warp_indices = []
            for j in range(self.width):
                if prev_pattern[j] != weft_pattern[j]:
                    self.warp_indices.append(j)
            print(f"constructed warp_indices: {self.warp_indices}")

        else:
            # use warps to check weft
            for j in range(self.width):
                if (prev_pattern[j] != weft_pattern[j] and j not in self.warp_indices) or (prev_pattern[j] == weft_pattern[j] and j in self.warp_indices):
                    print(f"fault at {j}, prev_pattern[j]: {prev_pattern[j]}, weft_pattern[j]: {weft_pattern[j]}, ")
                    raise ValueError("Float not supported")

        # record
        self.weft_to_pattern[weft_idx] = weft_pattern
        self.length += 1

    def set_height_func(self, height_func):
        self.height_func = height_func

    def calcaulte_flat_length_y(self):
        length_y = 0 
        prev_diameter = None
        for i in self.weft_to_pattern.keys():
            if not prev_diameter:
                prev_diameter = _weft_yarns[i].yarn_diameter
                continue
            curr_diameter = _weft_yarns[i].yarn_diameter
            length_y += (prev_diameter/2 + _max_warp_diameter + curr_diameter/2) * _weft_spacing_factor
            prev_diameter = curr_diameter
        
        print(f"calculate flat len: {length_y}")
        return length_y

    def update_control_grid_and_yarn_paths(self, length_y):
        self.length_y = length_y

        if not self.length or not self.prev_y or not self.height_func or not self.length_y:
            raise ValueError("fields not set up for control grid calculation")

        slice_y = length_y / float(self.length + 1)

        print(f"update control grid: length_y: {length_y}, length: {self.length}, slice_y: {slice_y}")

        global _control_grid

        count_weft = 0 # since weft index is not their count in this particular cloth
        for i in self.weft_to_pattern.keys():
            y = self.prev_y + (count_weft+1) * slice_y
            print(f"{count_weft}: y={y}")
            count_weft += 1

            z = self.height_func(y-self.prev_y)

            for j in range(self.width):
                x = _warp_centers_x[j]

                _control_grid[i][j] = (x, y, z)
        
        # print(f"update control grid done: {_control_grid}")

        for i in self.weft_to_pattern.keys():
            for j in self.warp_indices:
                update_yarn_path_centers_at(i,j)

    def __repr__(self):
        return f"Cloth - weft_to_pattern: {self.weft_to_pattern}, warp_indices: {self.warp_indices}"


class MultiCloth:
    def __init__(self, prev_y):
        # ONLY support: full, multiple
        # height -> Cloth
        self.height_to_cloth_map = {}

        # the previous center of the single cloth last weft
        self.prev_y = prev_y

    
    def read_weft_pattern(self, weft_idx, weft_pattern):
        """find the Cloth that the weft belongs to, then add it"""
        height = -sum(weft_pattern)
        if height not in self.height_to_cloth_map:
            cloth = Cloth(len(weft_pattern), self.prev_y )
            self.height_to_cloth_map[height] = cloth
        cloth = self.height_to_cloth_map[height]

        cloth.add_weft_pattern(weft_idx, weft_pattern)
        print(f"unnormalized height: {height}, current cloth: {cloth}")

    def update_control_grid_and_yarn_paths(self):
        if len(self.height_to_cloth_map) > 2:
            raise ValueError(">2 cloth not supported")
        
        # first, get height and cloth
        sorted_heights = sorted(list(self.height_to_cloth_map.keys()))

        label_to_length = {} # label: increasing for height, 0 as the lowest. length is # weft in cloth

        length_y = -1
        for label in range(len(sorted_heights)):
            label_to_length[label] = self.height_to_cloth_map[sorted_heights[label]].length
            length_y = max(length_y, label_to_length[label])

        print(f"update_control_grid_and_yarn_paths multi cloth y: {length_y}, after y : {length_y+ self.prev_y}")

        firmestLabel, firmestLength = -1, 2**32
        for label in range(len(sorted_heights)):
            label_to_length[label] = self.height_to_cloth_map[sorted_heights[label]].length
            if label_to_length[label] < firmestLength:
                firmestLabel, firmestLength = label, label_to_length[label]
                
        print(f"label_to_length: {label_to_length}, firmestLabel: {firmestLabel}, firmestLength: {firmestLength}")

        # generate path of the firmest
        firmestCloth = self.height_to_cloth_map[sorted_heights[firmestLabel]]
        length_y = firmestCloth.calcaulte_flat_length_y()
        
        for label in range(len(sorted_heights)):
            cloth = self.height_to_cloth_map[sorted_heights[label]]


            # if sorted_heights[label] == -10:
            #     cloth.set_height_func(lambda x: -5) 
            # elif sorted_heights[label] == -30:
            #     cloth.set_height_func(lambda x: 5) 
            # else: 
            #     cloth.set_height_func(lambda x: 0) 

            if label == firmestLabel:
                cloth.set_height_func(lambda x: 0) # z=0
            else:
                # z = f(x) = alpha * (x - start) * (x - end)
                # -> alpha * (x-0) *  (x-length_y)
                # Later: welfCount, label function, require non-crossing
                if label < firmestLabel:
                    alpha = cloth.length     # count of weft * height
                else:
                    alpha = -cloth.length # up

                # figure out alpha by the max height
                current_max_height = (length_y / float(2)) ** 2
                expected_max_height = abs(firmestLabel-label) * 1.5 # how many cloth are in between
                alpha = alpha * expected_max_height / current_max_height
                print(f"current_max_height: {current_max_height}, expected_max_height: {expected_max_height}, alpha: {alpha}")

                height_func = lambda y:alpha * (y-0) * (y-length_y)
                cloth.set_height_func(height_func)

            cloth.update_control_grid_and_yarn_paths(length_y)

        # print(f"done generate_and_append_path_centers, control_grid: {_control_grid}")
        
        return self.prev_y + length_y

    def __repr__(self):
        return f"MultiCloth - height_to_cloth_map: {self.height_to_cloth_map}, prev_y: {self.prev_y}"


def update_yarn_path_centers_at(i,j):
    weft_yarn, warp_yarn = _weft_yarns[i], _warp_yarns[j]
    weft_radius, warp_radius = weft_yarn.yarn_diameter / 2, warp_yarn.yarn_diameter / 2 # radius
    # print(f" tensions: {weft_yarn.tension}, {warp_tension.tension}")
    weft_tension_inverse, warp_tension_inverse = 1/weft_yarn.tension, 1/warp_yarn.tension # tension
    weft_tension_inverse_frac, warp_tension_inverse_frac = weft_tension_inverse/(weft_tension_inverse+warp_tension_inverse), warp_tension_inverse/(weft_tension_inverse+warp_tension_inverse)
    
    # Adjust z position based on which yarn is on top and their tensions
    weft_z, warp_z = 0,0 

    if _pattern[i][j] == 1:  # weft on top
        weft_z = (weft_radius + warp_radius) * weft_tension_inverse_frac
        warp_z = -(weft_radius + warp_radius) * warp_tension_inverse_frac
    else:  # warp
        weft_z = -(weft_radius + warp_radius) * weft_tension_inverse_frac
        warp_z = (weft_radius + warp_radius) * warp_tension_inverse_frac

    _weft_paths_centers[i].append(_control_grid[i][j] + np.array([0,0,weft_z]))
    _warp_paths_centers[j].append(_control_grid[i][j] + np.array([0,0, warp_z]))

    _weft_paths_warp[i].append(j)
    _warp_paths_weft[j].append(i)

def update_control_grid_and_yarn_paths_single_cloth(prev_y, i, warp_count):
    y = prev_y + _weft_yarns[i].yarn_diameter # 
    z = 0

    for j in range(warp_count):
        x = _warp_centers_x[j]
        _control_grid[i][j] = np.array([x, y, z])

        update_yarn_path_centers_at(i,j)

    return y


def is_single_cloth(single_weft_pattern):
    """check if the pattern for this weft is 0/1 alternating. only support full"""
    for i in range(1, len(single_weft_pattern)):
        if single_weft_pattern[i-1] == single_weft_pattern[i]:
            return False
    return True



def generate_control_grid_and_yarn_paths(weft_count, warp_count, pattern, warp_centers_x, max_warp_diameter, weft_yarns, warp_yarns, weft_spacing_factor):

    global _warp_centers_x
    global _max_warp_diameter
    global _weft_yarns
    global _warp_yarns
    global _weft_spacing_factor
    global _weft_count
    global _warp_count
    global _pattern

    _warp_centers_x = warp_centers_x
    _max_warp_diameter = max_warp_diameter
    _weft_yarns = weft_yarns
    _warp_yarns = warp_yarns
    _weft_spacing_factor = weft_spacing_factor
    _weft_count = weft_count
    _warp_count = warp_count
    _pattern = pattern
    

    global _control_grid
    global _weft_paths_centers
    global _warp_paths_centers
    global _weft_paths_warp
    global _warp_paths_weft
    _control_grid = [[None for _ in range(warp_count)] for _ in range(weft_count)]
    _weft_paths_centers = [[] for _ in range(_weft_count)]
    _warp_paths_centers = [[] for _ in range(_warp_count)]
    _weft_paths_warp = [[] for _ in range(_weft_count)]
    _warp_paths_weft = [[] for _ in range(_warp_count)]

    i = 0
    multi_cloth = None
    prev_y = 0 # prev_y of the single-cloth

    while i < weft_count:
        """read in one weft pattern at a time"""

        """if current is non-single-cloth weft: initialize a chunk of non-single-cloth wefts"""
        while not is_single_cloth(pattern[i]) and i < weft_count:
            print(f"current weft {i} is multi cloth")
            if not multi_cloth:
                # mutli-cloth starts here 
                multi_cloth = MultiCloth(prev_y)
                print(f"initialize multi cloth with prev y {prev_y}")
            # record current nulti-cloth weft
            multi_cloth.read_weft_pattern(i, pattern[i])
            i += 1
        
        # end multi-cloth processing
        if not is_single_cloth(pattern[i]):
            continue

        """if current is single cloth weft"""
        print(f"current weft {i} is single cloth")

        # update previous group of multi cloth piece
        if multi_cloth:
            prev_y = multi_cloth.update_control_grid_and_yarn_paths()
            print(f"multi cloth end: {multi_cloth} at y {prev_y}")
    
        multi_cloth = None # reset
        prev_y = update_control_grid_and_yarn_paths_single_cloth(prev_y, i, warp_count)
        print(f"process single cloth ending at y {prev_y}")

        i += 1
    
    if multi_cloth:
        multi_cloth.update_control_grid_and_yarn_paths()

    print(f"final _control_grid:")
    for i in range(len(_control_grid)):
        for j in range(len(_control_grid[0])):
            print(f"[{i}][{j}]: {_control_grid[i][j]}")

    
    return _control_grid, _weft_paths_centers, _warp_paths_centers, _weft_paths_warp, _warp_paths_weft
