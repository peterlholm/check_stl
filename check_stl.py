from pathlib import Path
from matplotlib.patches import Patch
import numpy as np
from matplotlib import use, pyplot as plt
#from matplotlib import use

O3D = True
if O3D:
    import open3d as o3d
    
_DEBUG = False
_SHOW = True

# picture formats
#
#   x: point right
#   y: point up
#   z: point out of mouth
#

MIN_WIDE = -20  # x left
MAX_WIDE = 20   # x right
MIN_HIGHT = -20 # y down
MAX_HIGHT = 5   # y up
MIN_DEPTH = -35 # z in mouth
MAX_DEPTH = 35  # z out mouth
MIN_VOLUME = 1000
MAX_VOLUME = 70000

def show_stl(path):
    if not path.exists():
        print("File not found", path)
        return
    mesh = o3d.io.read_triangle_mesh(str(path))
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=20, origin=[0, 0, 0])
    center = mesh.get_center()
    o3d.visualization.draw_geometries([mesh, mesh_frame], window_name="input",
                                  zoom=0.5,
                                  front=[0.0, 10.2125, 20.0],
                                  lookat=center,
                                  #lookat=[2.6172, 2.0475, 1.532],
                                  #up=[-0.0694, -0.9768, 0.2024],
                                  up=[0, 10.0, 0.0],
                                  point_show_normal=False,
                                  mesh_show_back_face=True,
                                  mesh_show_wireframe=True)
            
def stl_info(path):
    if not path.exists():
        print("File not found", path)
        return
    mesh = o3d.io.read_triangle_mesh(str(path))
    
    print("get_axis_aligned_bounding_box", mesh.get_axis_aligned_bounding_box())
    print("get_center", mesh.get_center())
    print("get_max_bound", mesh.get_max_bound())
    print("get_oriented_bounding_box", mesh.get_oriented_bounding_box())
    #print("get_max_bonds", mesh.get_max_bonds())
    print("get_surface_area", mesh.get_surface_area())
    #print("get_volume", mesh.get_volume())
    print("has_textures", mesh.has_textures())
    print("has_triangle_material_ids", mesh.has_triangle_material_ids())
    print("has_vertex_normals", mesh.has_vertex_normals())
    print("is_orientable", mesh.is_orientable())
    #print("get_volume", mesh.get_volume())
    #print("get_volume", mesh.get_volume())

def check_stl(path):
    "Check if stl file is write size and scale orientation"
    if not path.exists():
        print("File not found", path)
        return
    CHECK = True
    mesh = o3d.io.read_triangle_mesh(str(path))
    center = mesh.get_center()
    if min(center)<-10 or max(center)>10:
        print("Misplaced center:", center)
        CHECK = False
    bound_box = mesh.get_axis_aligned_bounding_box()
    bound_min = bound_box.min_bound
    bound_max = bound_box.max_bound
    if (bound_min[0] < MIN_WIDE) or (bound_min[1] < MIN_HIGHT) or (bound_min[2] < MIN_DEPTH) or (bound_max[0] > MAX_WIDE) or (bound_max[1] > MAX_HIGHT) or (bound_max[2] > MAX_DEPTH):
        print("Bounds error", bound_box)
        CHECK = False
    volume = bound_box.volume()
    if volume > MAX_VOLUME or volume < MIN_VOLUME:
        print("Volume error", volume)
        CHECK = False
   
    if _DEBUG:
        stl_info(path)

    if CHECK:
        if _DEBUG:
            print("stl file",path, "OK")
        return True
    if _DEBUG:
        print("stl file",path, "ERROR")
    return False
    
def pcl2mesh(filepath):
    "make a mesh stl file from the filepath"
    downsample = 0.1
    pcl =o3d.io.read_point_cloud(str(filepath))
    if _SHOW:
        o3d.visualization.draw_geometries([pcl], window_name="input",
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024],
                                  point_show_normal=True)
    if _DEBUG:
        outfile = filepath.with_suffix('.in.jpg')
        pcl2jpg(pcl, outfile)
        print("Number of points: ", len(pcl.points))
        print("Downsample the point cloud with a voxel of", downsample)
    downpcd = pcl.voxel_down_sample(voxel_size=downsample)
    if _SHOW:
        o3d.visualization.draw_geometries([mesh],  window_name="mesh",
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024],
                                  point_show_normal=False,
                                  mesh_show_back_face=True,
                                  mesh_show_wireframe=True)

    o3d.io.write_triangle_mesh(str(filepath.with_suffix('.stl')), mesh, print_progress=False)

    # make filtered stl
    filtered = o3d.geometry.TriangleMesh.filter_smooth_simple(mesh, 5)
    filtered = filtered.compute_triangle_normals()
    o3d.io.write_triangle_mesh(str(filepath.with_suffix('.filtered.stl')), filtered, print_progress=False)


if __name__=="__main__":
    FILE = Path(__file__).parent / 'testdata/LJ3.stl'
    FILE = Path(__file__).parent / 'testdata/niels/test3 LowerJawScan.stl'

    print (FILE)
    #stl_info(FILE)
    #print("CheckStl", check_stl(FILE))
    show_stl(FILE)
    