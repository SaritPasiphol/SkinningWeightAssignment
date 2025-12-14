import sys
import os
import json
import trimesh
import numpy as np

# --- CONFIGURATION ---
OUTPUT_MESH = "human.obj"
OUTPUT_SKEL = "human_skeleton.txt"

# Colors for bones
COLORS = [
    (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
    (1.0, 1.0, 0.0), (0.0, 1.0, 1.0), (1.0, 0.0, 1.0),
    (1.0, 0.5, 0.0), (0.5, 0.0, 1.0)
]

def find_files(start_dir):
    """Auto-detects the .glb and .json files in the folder."""
    found_glb = None
    found_json = None
   
    for root, dirs, files in os.walk(start_dir):
        for file in files:
            # Find GLB
            if file.lower().endswith(".glb") and not found_glb:
                found_glb = os.path.join(root, file)
           
            # Find JSON (Prefer 'bone3d.json', but take any json if missing)
            if file.lower().endswith(".json"):
                if "bone3d" in file.lower():
                    found_json = os.path.join(root, file)
                elif not found_json: # fallback to first json found
                    found_json = os.path.join(root, file)
   
    return found_glb, found_json

def convert_mesh(glb_path):
    print(f"  -> Converting Mesh: {os.path.basename(glb_path)}")
    try:
        scene = trimesh.load(glb_path, force='scene')
        if len(scene.geometry) == 0: return False
       
        # Merge and Export
        mesh = trimesh.util.concatenate(scene.dump())
       
        # Optional: Rotate if your mesh is face-down
        # mesh.apply_transform(trimesh.transformations.rotation_matrix(np.radians(-90), [1, 0, 0]))
       
        mesh.export(OUTPUT_MESH)
        return True
    except Exception as e:
        print(f"Error converting mesh: {e}")
        return False

def convert_skeleton(json_path):
    print(f"  -> Converting Skeleton: {os.path.basename(json_path)}")
    try:
        with open(json_path, 'r') as f:
            data = json.load(f)
           
        with open(OUTPUT_SKEL, "w") as out:
            out.write(f"# Converted from {os.path.basename(json_path)}\n")
            out.write("# Name           X      Y      Z       R    G    B\n")
           
            joints = []
           
            # Handle Dictionary format {"Hips": [x,y,z]}
            if isinstance(data, dict):
                for name, pos in data.items():
                    if isinstance(pos, dict): pos = pos.get('pos', pos.get('position', [0,0,0]))
                    joints.append((name, pos))
                   
            # Handle List format [{"name": "Hips", "pos": [x,y,z]}]
            elif isinstance(data, list):
                for i, item in enumerate(data):
                    name = item.get('name', f"Bone_{i}") if isinstance(item, dict) else f"Bone_{i}"
                    pos = item.get('pos', item.get('position', item)) if isinstance(item, dict) else item
                    joints.append((name, pos))

            color_idx = 0
            for name, coords in joints:
                if not isinstance(coords, (list, tuple, np.ndarray)) or len(coords) < 3: continue
                x, y, z = coords[0], coords[1], coords[2]
                r, g, b = COLORS[color_idx % len(COLORS)]
                color_idx += 1
                out.write(f"{name:<15} {x:>8.4f} {y:>8.4f} {z:>8.4f}    {r:.1f}  {g:.1f}  {b:.1f}\n")
               
        return True
    except Exception as e:
        print(f"Error converting skeleton: {e}")
        return False

def main():
    if len(sys.argv) < 2:
        print("Usage: python convert.py <folder_name>")
        return

    target_folder = sys.argv[1]

    if not os.path.isdir(target_folder):
        print(f"Error: '{target_folder}' is not a valid folder.")
        return

    print(f"Scanning folder: {target_folder}...")
    glb_file, json_file = find_files(target_folder)

    if not glb_file:
        print("Error: No .glb file found in this folder.")
        return
    if not json_file:
        print("Error: No .json file found in this folder.")
        return

    # Run Conversion
    if convert_mesh(glb_file) and convert_skeleton(json_file):
        print("\nSUCCESS!")
        print(f"Created: {OUTPUT_MESH}")
        print(f"Created: {OUTPUT_SKEL}")
        print("-" * 30)
        print(f"Run: .\\main.exe {OUTPUT_MESH} {OUTPUT_SKEL}")

if __name__ == "__main__":
    main()