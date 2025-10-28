import ikpy

def load_robot(path):
    return ikpy.chain.Chain.from_urdf_file(path)
