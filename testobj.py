# Quick test script
with open('human.obj', 'r') as f:
    lines = f.readlines()
    vertices = [l for l in lines if l.startswith('v ')]
    faces = [l for l in lines if l.startswith('f ')]
    print(f"Vertices: {len(vertices)}")
    print(f"Faces: {len(faces)}")
    print(f"First vertex: {vertices[0]. strip()}")
    print(f"First face: {faces[0].strip()}")