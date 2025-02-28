warp_centers_x = None
max_warp_diameter = None
weft_yarns = None
control_grid = None

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
        if not self.warp_indices:
            # set warps
            self.warp_indices = []
            for j in range(self.width):
                if prev_pattern[j] != weft_pattern[j]:
                    self.warp_indices.append(j)
            print("constructed warp_indices: {self.warp_indices}")
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
        for i in self.weft_to_pattern.items():
            if not prev_diameter:
                prev_diameter = weft_yarns[i].yarn_diameter
                continue
            curr_diameter = weft_yarns[i].yarn_diameter
            length_y += (prev_diameter/2 + max_warp_diameter + curr_diameter/2) * weft_spacing_factor
            prev_diameter = curr_diameter
        
        print(f"calculate flat len: {length_y}")
        return length_y


    def update_control_grid(self, length_y):
        self.length_y = length_y

        if not self.length or not self.prev_y or not self.height_func or not self.length_y:
            raise ValueError("fields not set up for control grid calculation")

        print(f"update control grid: length_y: {length_y}, length: {self.length}")
        slice_y = length_y / (self.length + 1)

        global control_grid

        for i in self.weft_to_pattern.items():
            y = self.prev_y + (i+1) * slice_y

            z = self.height_func(y-self.prev_y)

            for j in range(self.width):
                x = warp_centers_x[j]

                control_grid[i][j] = (x, y, z)
        
        print(f"update control grid done: {control_grid}")
        

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
        cloth = self.height_to_cloth_map[height]

        print(f"height: {height}, cloth: {cloth}")
        cloth.add_weft_pattern(weft_idx, weft_pattern)

    def update_control_grid(self):
        if len(self.height_to_cloth_map > 2):
            raise ValueError(">2 cloth not supported")
        
        # first, get height and cloth
        sorted_heights = sorted(list(self.height_to_cloth_map.items()))

        label_to_length = {} # label: increasing for height, 0 as the lowest. length is # weft in cloth
        firmestLabel, firmestLength = -1,-1
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
            if label == firmestLabel:
                cloth.set_height_func(lambda x: 0) # z=0
            else:
                # z = f(x) = alpha * (x - start) * (x - end)
                # -> alpha * (x-0) *  (x-length_y)
                # Later: welfCount, label function, require non-crossing
                if label < firmestLabel:
                    alpha = cloth.length        # count of weft * height
                else:
                    alpha = -cloth.length # up
                height_func = lambda x: cloth.length * x * (x-length_y)
                cloth.set_height_func(height_func)

            cloth.update_control_grid()
        print("done generate_and_append_path_centers, control_grid: {control_grid}")
        
        return self.prev_y + length_y

    def __repr__(self):
        return f"MultiCloth - height_to_cloth_map: {self.height_to_cloth_map}, start_y: {self.start_y}"


def update_control_grid_single_cloth(prev_y, i, warp_count):
    print(f"generate_yarn_center_single_cloth")
    y = prev_y + weft_yarns[i].diameter # 
    z = 0

    for j in range(warp_count):
        x = warp_centers_x[j]
        control_grid[i][j] = (x, y, z)

    
    

def generate_yarn_center(grid_point, grid_pattern, weft_yarn, warp_yarn):
    """from grid point to actual yarn centers of weft and warp this intersection"""
    pass

def is_single_cloth(weft_pattern):
    """check if the pattern for this weft is 0/1 alternating. only support full"""
    print(f"is_single_cloth: {weft_pattern}")
    for i in range(1, len(weft_pattern)):
        if weft_pattern[i-1] == weft_pattern[i]: return False
    return True


def generate_path_centers(weft_count, warp_count, pattern, warp_centers_x):

    global warp_centers_x
    global control_grid

    warp_centers_x = warp_centers_x
    control_grid = [[None for _ in range(warp_count)] for _ in range(weft_count)]

    i = 0
    multi_cloth = None
    prev_y = 0

    while i < weft_count:
        """read in one weft pattern at a time"""

        while not is_single_cloth(pattern[i]) and i < weft_count:
            print("not single cloth")
            """for a chunk of non-single-cloth patterns"""
            if not multi_cloth:
                # initialize
                multi_cloth = MultiCloth(prev_y)
            multi_cloth.read_weft_pattern(i, pattern[i])
            i += 1

        if multi_cloth:
            print(f"multi cloth end: {multi_cloth}")
            prev_y = multi_cloth.update_control_grid()
    
        multi_cloth = None # reset
        prev_y = update_control_grid_single_cloth(prev_y, i, warp_count)

        i += 1