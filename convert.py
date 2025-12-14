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
    (1.0, 0.5, 0.0), (0.5, 0.0, 1.0), (0.5, 1.0, 0.0),
    (1.0, 0.0, 0.5), (0.0, 0.5, 1.0), (0.5, 0.5, 0.5)
]

# Global to store mesh info for skeleton scaling
mesh_info = {}

def find_files(start_dir):
    """Auto-detects the . glb and .json files in the folder."""
    found_glb = None
    found_json = None
   
    for root, dirs, files in os.walk(start_dir):
        for file in files: 
            # Find GLB
            if file.lower().endswith(".glb") and not found_glb:
                found_glb = os.path.join(root, file)
           
            # Find JSON - Prefer 3D over 2D
            if file. lower().endswith(".json"):
                if "bone_3d" in file.lower() or "bone3d" in file.lower():
                    found_json = os.path.join(root, file)
                elif not found_json:  # fallback to any json
                    found_json = os. path.join(root, file)
   
    return found_glb, found_json

def convert_mesh(glb_path):
    print(f"  -> Converting Mesh: {os.path. basename(glb_path)}")
    try:
        # Load the GLB scene
        scene = trimesh.load(glb_path, force='scene')
        
        if len(scene.geometry) == 0:
            print("     ERROR: No geometry in GLB")
            return False
        
        print(f"     Found {len(scene.geometry)} sub-mesh(es)")
        
        # Collect all meshes
        meshes = []
        for name, geom in scene.geometry.items():
            print(f"       - {name}: {len(geom.vertices)} verts, {len(geom.faces)} faces")
            if len(geom.vertices) > 0 and len(geom.faces) > 0:
                if not geom.is_empty:
                    meshes.append(geom)
        
        if not meshes:
            print("     ERROR: No valid meshes found")
            return False
        
        # Merge all meshes
        if len(meshes) == 1:
            merged_mesh = meshes[0]
        else:
            print(f"     Merging {len(meshes)} meshes...")
            merged_mesh = trimesh.util. concatenate(meshes)
        
        # Clean mesh using available methods
        print(f"     Cleaning mesh...")
        
        try:
            merged_mesh.fix_normals()
        except:
            pass
        
        try:
            merged_mesh.remove_infinite_values()
        except:
            pass
        
        try:
            if hasattr(merged_mesh, 'nondegenerate_faces'):
                valid_faces = merged_mesh.nondegenerate_faces
                if len(valid_faces) < len(merged_mesh.faces):
                    print(f"     Removing {len(merged_mesh.faces) - len(valid_faces)} degenerate faces")
                    merged_mesh. update_faces(valid_faces)
        except:
            pass
        
        try:
            merged_mesh.remove_duplicate_faces()
        except:
            pass
        
        try:
            merged_mesh.remove_unreferenced_vertices()
        except:
            pass
        
        if len(merged_mesh.faces) == 0:
            print("     ERROR: No faces after cleanup")
            return False
        
        # Store mesh info for skeleton scaling
        mesh_info['original_min'] = merged_mesh.bounds[0]
        mesh_info['original_max'] = merged_mesh.bounds[1]
        mesh_info['original_center'] = (mesh_info['original_min'] + mesh_info['original_max']) / 2
        mesh_info['original_size'] = np. max(mesh_info['original_max'] - mesh_info['original_min'])
        
        print(f"     Final mesh:  {len(merged_mesh.vertices)} vertices, {len(merged_mesh.faces)} faces")
        print(f"     Bounds:  {mesh_info['original_min']} to {mesh_info['original_max']}")
        print(f"     Size: {mesh_info['original_size']:.4f}")
        
        # Export as OBJ
        print(f"     Exporting to {OUTPUT_MESH}...")
        merged_mesh.export(OUTPUT_MESH, file_type='obj')
        
        if os.path.exists(OUTPUT_MESH):
            file_size = os.path.getsize(OUTPUT_MESH)
            print(f"     ✅ Exported ({file_size} bytes)")
            return True
        else:
            print(f"     ERROR: Export failed")
            return False
            
    except Exception as e: 
        print(f"     ERROR:  {e}")
        import traceback
        traceback.print_exc()
        return False

def convert_skeleton(json_path):
    print(f"  -> Converting Skeleton: {os.path. basename(json_path)}")
    try:
        with open(json_path, 'r') as f:
            data = json.load(f)
        
        joints = []
        is_2d = False
        
        # Handle Dictionary format {"Hips": [x,y] or [x,y,z]}
        if isinstance(data, dict):
            for name, pos in data. items():
                if isinstance(pos, dict):
                    pos = pos.get('pos', pos. get('position', []))
                
                # Check if 2D or 3D
                if isinstance(pos, (list, tuple, np.ndarray)):
                    if len(pos) == 2:
                        # 2D format - add Z=0
                        is_2d = True
                        joints.append((name, [float(pos[0]), float(pos[1]), 0.0]))
                    elif len(pos) >= 3:
                        # 3D format
                        joints.append((name, [float(pos[0]), float(pos[1]), float(pos[2])]))
               
        # Handle List format [{"name": "Hips", "pos": [x,y,z]}]
        elif isinstance(data, list):
            for i, item in enumerate(data):
                name = item.get('name', f"Bone_{i}") if isinstance(item, dict) else f"Bone_{i}"
                pos = item.get('pos', item.get('position', item)) if isinstance(item, dict) else item
                
                if isinstance(pos, (list, tuple, np.ndarray)):
                    if len(pos) == 2:
                        is_2d = True
                        joints.append((name, [float(pos[0]), float(pos[1]), 0.0]))
                    elif len(pos) >= 3:
                        joints.append((name, [float(pos[0]), float(pos[1]), float(pos[2])]))
        
        if not joints:
            print("     ERROR:  No joints found in JSON")
            print(f"     JSON preview: {str(data)[:200]}")
            return False
        
        if is_2d:
            print(f"     ⚠️ Detected 2D skeleton (adding Z=0)")
        
        print(f"     Found {len(joints)} bones")
        
        # Extract valid joint positions
        valid_joints = joints  # Already validated above
        
        if not valid_joints:
            print("     ERROR: No valid bone positions")
            return False
        
        # Calculate skeleton bounds
        bone_positions = np.array([coords for _, coords in valid_joints])
        skel_min = bone_positions.min(axis=0)
        skel_max = bone_positions.max(axis=0)
        skel_center = (skel_min + skel_max) / 2
        skel_size = np.max(skel_max - skel_min)
        
        print(f"     Skeleton bounds: {skel_min} to {skel_max}")
        print(f"     Skeleton size: {skel_size:.4f}")
        
        # Determine if we need to scale skeleton to match mesh
        if mesh_info and 'original_size' in mesh_info:
            mesh_size = mesh_info['original_size']
            mesh_center = mesh_info['original_center']
            
            # Check if scales are vastly different
            scale_ratio = mesh_size / skel_size
            
            print(f"     Mesh/Skeleton size ratio: {scale_ratio:.2f}")
            
            # If skeleton is much smaller/larger than mesh, scale it
            if scale_ratio > 2.0 or scale_ratio < 0.5:
                print(f"     🔧 Applying scale factor: {scale_ratio:.2f}")
                print(f"     🔧 Centering skeleton to match mesh")
                
                scaled_joints = []
                for name, coords in valid_joints:
                    centered = np.array(coords) - skel_center
                    scaled = centered * scale_ratio
                    final_pos = scaled + mesh_center
                    scaled_joints.append((name, final_pos. tolist()))
                
                valid_joints = scaled_joints
                print(f"     ✅ Skeleton scaled and centered")
            else:
                print(f"     ✅ Skeleton scale looks good (ratio: {scale_ratio:.2f})")
        else:
            print(f"     ⚠️ No mesh info available, using original skeleton coordinates")
        
        # Write skeleton file
        with open(OUTPUT_SKEL, "w") as out:
            out.write(f"# Converted from {os.path.basename(json_path)}\n")
            out.write(f"# Total bones: {len(valid_joints)}\n")
            if is_2d:
                out.write(f"# Note: 2D skeleton converted to 3D (Z=0)\n")
            out.write("# Format: Name X Y Z R G B\n")
            out.write("#" + "-" * 60 + "\n")
            
            for i, (name, coords) in enumerate(valid_joints):
                x, y, z = coords[0], coords[1], coords[2]
                r, g, b = COLORS[i % len(COLORS)]
                
                # Clean bone name
                clean_name = name.replace(" ", "_").replace(".", "_")
                
                out.write(f"{clean_name: <15} {x: >8.4f} {y:>8.4f} {z:>8.4f}    {r:.1f}  {g:.1f}  {b:.1f}\n")
        
        print(f"     ✅ Wrote {len(valid_joints)} bones to {OUTPUT_SKEL}")
        
        # Show sample positions
        if len(valid_joints) > 0:
            print(f"     Sample bones:")
            for i, (name, coords) in enumerate(valid_joints[:3]):
                print(f"       {name}: [{coords[0]: >7.3f}, {coords[1]:>7.3f}, {coords[2]:>7.3f}]")
        
        return True
        
    except Exception as e:
        print(f"     ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    if len(sys.argv) < 2:
        print("=" * 60)
        print("GLB + JSON to OBJ + Skeleton Converter")
        print("=" * 60)
        print("\nUsage: python convert.py <folder_name>")
        print("\nExample: python convert.py ./human_model")
        print("\nSupports:")
        print("  - 2D and 3D skeleton formats")
        print("  - Automatic scaling and centering")
        return

    target_folder = sys.argv[1]

    if not os.path.isdir(target_folder):
        print(f"Error: '{target_folder}' is not a valid folder.")
        return

    print("\n" + "=" * 60)
    print(f"Scanning folder: {target_folder}")
    print("=" * 60 + "\n")
    
    glb_file, json_file = find_files(target_folder)

    if not glb_file:
        print("❌ Error: No . glb file found in this folder.")
        return
    if not json_file:
        print("❌ Error: No . json file found in this folder.")
        return

    print(f"Found files:")
    print(f"  Mesh:       {os.path.basename(glb_file)}")
    print(f"  Skeleton: {os. path.basename(json_file)}")
    print()

    # Run Conversion
    mesh_ok = convert_mesh(glb_file)
    skel_ok = convert_skeleton(json_file)
    
    if mesh_ok and skel_ok:
        print("\n" + "=" * 60)
        print("✅ SUCCESS!")
        print("=" * 60)
        print(f"Created: {OUTPUT_MESH}")
        print(f"Created: {OUTPUT_SKEL}")
        print("\nTo view in the viewer, run:")
        print(f"  . \\main.exe {OUTPUT_MESH} {OUTPUT_SKEL}")
        print("=" * 60 + "\n")
    else:
        print("\n" + "=" * 60)
        print("❌ CONVERSION FAILED")
        print("=" * 60)
        if not mesh_ok:
            print("  - Mesh conversion failed")
        if not skel_ok: 
            print("  - Skeleton conversion failed")
        print()

if __name__ == "__main__":
    main()