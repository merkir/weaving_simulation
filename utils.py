import json
import os
from yarn_simulatior import YarnSimulation


def generate_obj_file(weft_count, warp_count, weft_yarns, warp_yarns, output_file):
    """
    Generate an OBJ file for the entire woven fabric.
    """

    # makedir
    os.makedirs(os.path.dirname(output_file), exist_ok=True)

    # Write OBJ file header
    with open(output_file, 'w') as f:
        f.write("# Woven fabric simulation OBJ file\n")
        f.write(f"# Weft count: {weft_count}\n")
        f.write(f"# Warp count: {warp_count}\n")
        
        vertex_offset = 0

        # f.write(f"o fabric\n")
        
        # weft
        for i in range(weft_count):
            print(f"generating weft {i}")
            yarn = weft_yarns[i]
        
            # generate obj
            f.write(f"o weft_{i}\n")
            vertices, faces = yarn.generate_yarn()
            for v in vertices:
                f.write(f"v {v[0]} {v[1]} {v[2]}\n")
            for face in faces:
                f.write(f"f {face[0] + 1 + vertex_offset} {face[1] + 1 + vertex_offset} {face[2] + 1 + vertex_offset}\n")
        
            vertex_offset += len(vertices)
        
        # Process warp yarns
        for j in range(warp_count):
            print(f"generating warp {j}")
            yarn = warp_yarns[j]
            
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

def load_and_validate_yarn_and_pattern():
    """Load pattern and yarn definitions from files."""
    
    # Load pattern
    # pattern_file = "weaving_patterns/simple_pattern.txt"
    pattern_file = "weaving_patterns/double_cloth_larger.txt"
    pattern = []
    with open(pattern_file, 'r') as file:
        for line in file:
            row = [int(cell.strip()) for cell in line.strip().split() if cell.strip()]
            if row: pattern.append(row)
    
    # Load yarn properties
    yarn_property_file = "weaving_patterns/yarn_properties.json"
    with open(yarn_property_file, 'r') as file:
        yarn_properties = json.load(file)
    
    # Load yarn pattern
    # yarn_pattern_file = "weaving_patterns/yarn_pattern.json"
    yarn_pattern_file = "weaving_patterns/yarn_pattern_double_cloth.json"
    with open(yarn_pattern_file, 'r') as file:
        yarn_pattern = json.load(file)
    
    weft_yarn_names, warp_yarn_names = None, None
    default_wefts = yarn_pattern["default_weft"] if "default_weft" in yarn_pattern else None
    default_warps = yarn_pattern["default_warp"] if "default_warp" in yarn_pattern else None
    if default_wefts:
        weft_yarn_names = [default_wefts] * len(pattern)
    if default_warps:
        warp_yarn_names = [default_warps] * len(pattern[0])

    weft_yarn_names, warp_yarn_names = yarn_pattern["wefts"] if "wefts" in yarn_pattern else weft_yarn_names , yarn_pattern["warps"] if "warps" in yarn_pattern else warp_yarn_names

    # Validate
    if weft_yarn_names and len(pattern) != len(weft_yarn_names):
        raise ValueError(f"Pattern has {len(pattern)} weft rows but yarn pattern has {len(weft_yarn_names)}")
    if warp_yarn_names and len(pattern[0]) != len(warp_yarn_names):
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