from pathlib import Path
from matplotlib.patches import Patch
import numpy as np
from matplotlib import use, pyplot as plt
#from matplotlib import use

O3D = True
if O3D:
    import open3d as o3d
    
_DEBUG = True
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

def check_stl(path):
    "Check if stl file is right size and scale orientation"
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

PICTURE_SIZE = 1000
OBJ_CENTER = [0.0,0.0,22.0]
CAM_POSITION = [0.0, 30.0, 0.0]
ZOOM = 0.9  # tand    

def stl2jpg(path):
    "create jpg picture of stl"
    zoom = ZOOM
    if not path.exists():
        print("File not found", path)
        return
    CHECK = True
    outfile = path.with_suffix('.jpg')
    print(outfile)
    mesh = o3d.io.read_triangle_mesh(str(path)) 
    obj_center = mesh.get_center()
     # camera position
    vis = o3d.visualization.Visualizer()
    res = vis.create_window(visible = _DEBUG, width=PICTURE_SIZE, height=PICTURE_SIZE)
    if not res:
        print("create window result", res)
    vis.add_geometry(mesh)
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=20, origin=[0, 0, 0])
    vis.add_geometry(mesh_frame)
    ctr = vis.get_view_control()
    if ctr is None:
        print("pcl2jpg cant get view_control", vis)
    # fix position
    cam_position=CAM_POSITION
    if _DEBUG:
        print('object center', obj_center, "cam position:", cam_position, "zoom", zoom)
    ctr.set_zoom(zoom)
    ctr.set_front(cam_position)
    ctr.set_lookat(obj_center)
    ctr.set_up([+10.0, 0, 0])
    opt = vis.get_render_option()
    #opt.point_size = 2.0
    opt.mesh_show_wireframe = True
    opt.mesh_show_back_face = True
    #opt.point_color_option.Color = 1
    if _DEBUG:
        vis.run()
    vis.capture_screen_image(str(outfile), do_render=True)

if __name__=="__main__":
    FILE = Path(__file__).parent / 'testdata/LJ3.stl'
    #FILE = Path(__file__).parent / 'testdata/niels/test3 LowerJawScan.stl'
    print (FILE)
    #stl_info(FILE)
    #print("CheckStl", check_stl(FILE))
    #show_stl(FILE)
    stl2jpg(FILE)

