import vtk
from vtk.util import numpy_support
import numpy as np
import os.path as osp
import argparse
from sys import exit

def vtkmatrix_from_array(array):
    """Convert a ``numpy.ndarray`` or array-like to a vtk matrix.
    Parameters
    ----------
    array : numpy.ndarray or array-like
        The array or array-like to be converted to a vtk matrix.
        Shape (3, 3) gets converted to a ``vtk.vtkMatrix3x3``, shape (4, 4)
        gets converted to a ``vtk.vtkMatrix4x4``. No other shapes are valid.
    """
    array = np.asarray(array)
    if array.shape == (3, 3):
        matrix = vtk.vtkMatrix3x3()
    elif array.shape == (4, 4):
        matrix = vtk.vtkMatrix4x4()
    else:
        raise ValueError(f'Invalid shape {array.shape}, must be (3, 3) or (4, 4).')
    m, n = array.shape
    for i in range(m):
        for j in range(n):
            matrix.SetElement(i, j, array[i, j])
    return matrix

def load_polydata(file_name):
    # get file extension (type)
    file_extension = file_name.split(".")[-1].lower()

    # todo better generic load
    if file_extension in ["vtk", 'vtp', "fib"]:
        reader = vtk.vtkPolyDataReader()
    elif file_extension == "ply":
        reader = vtk.vtkPLYReader()
    elif file_extension == "stl":
        reader = vtk.vtkSTLReader()
    elif file_extension == "xml":
        reader = vtk.vtkXMLPolyDataReader()
    elif file_extension == "obj":
        try:  # try to read as a normal obj
            reader = vtk.vtkOBJReader()
        except:  # than try load a MNI obj format
            reader = vtk.vtkMNIObjectReader()
    else:
        raise "polydata " + file_extension + " is not suported"

    reader.SetFileName(file_name)
    reader.Update()
    return reader.GetOutput()

def polyDataToActor(polydata):
    """Wrap the provided vtkPolyData object in a mapper and an actor, returning
    the actor."""
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(polydata)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().LightingOff()
    return actor

def load_paths(dataset, path):
    if dataset == 'scannet':
        intrinsic_path = osp.join(path, 'intrinsic')
        pose_path = osp.join(path, 'pose')
        intrinsic_depth = open(osp.join(intrinsic_path, 'intrinsic_depth.txt')).read()
        intrinsic_depth = [item.split() for item in intrinsic_depth.split('\n')[:-1]]
        axis_alignment = load_alignment(dataset, path)
        return pose_path, intrinsic_depth, axis_alignment
    else:
        raise NotImplementedError

def load_alignment(dataset, path):
    if dataset == 'scannet':
        with open(osp.join(path, osp.basename(path)+'.txt')) as f:
            file = f.read().split('\n')
        try:
            matrix = [float(item) for item in file[0].split()[2:]]
        except ValueError: #first line is "appVersionId"
            matrix = [float(item) for item in file[1].split()[2:]]
        return np.array(matrix).reshape(4,4)
    else:
        raise NotImplementedError

def load_model(dataset, path):
    if dataset == 'scannet':
        file_name = osp.join(path, osp.basename(path)+'_vh_clean_2.labels.ply')
    else:
        raise NotImplementedError
    polydata = load_polydata(file_name)
    return polydata

def load_camera(dataset, pose_path, intrinsic_depth, frame):
    if dataset == 'scannet':
        camera_pose = open(osp.join(pose_path, str(frame) + '.txt')).read()
        camera_pose = [item.split() for item in camera_pose.split('\n')[:-1]]
        camera_pose = np.array(camera_pose, dtype=float)
        p_matrix = np.matrix([intrinsic_depth[0][:],
                              intrinsic_depth[1][:],
                              intrinsic_depth[2][:]], dtype=float)
        return p_matrix, camera_pose
    else:
        raise NotImplementedError

def getImage(vtk_render_window, width, height):
    vtk_array = vtk.vtkUnsignedCharArray()
    vtk_render_window.GetPixelData(0, 0, width-1, height-1, 1, vtk_array)
    numpy_array = numpy_support.vtk_to_numpy(vtk_array)
    numpy_array = np.reshape(numpy_array, (-1, width, 3))
    numpy_array = np.flipud(numpy_array)  # flipping along the first axis (y)
    return numpy_array

parser = argparse.ArgumentParser('Check orientation of ScanNet instances')
parser.add_argument('--dataset', type=str, default='scannet',
                    help='dataset type [??? | scannet]')
parser.add_argument('--data_path1', type=str, default='/DATA/ScanNet/scans/scene0013_00',
                    help='folder containing scene')
parser.add_argument('--data_path2', type=str, default='/DATA/ScanNet/scans/scene0013_02',
                    help='folder containing scene')
parser.add_argument('--frame', type=int, default=1,
                    help='frame number for pose used')
parser.add_argument('--step', type=int, default=60,
                    help='frame step for next pose used')
parser.add_argument('--iter', type=int, default=20,#10,
                    help='number of steps before giving up')
parser.add_argument('--thres', type=float, default=0.2,#0.5,
                    help='acceptable threshold')
parser.add_argument('--width', type=int, default=640,#1298,#1920,
                    help='input image width')
parser.add_argument('--height', type=int, default=480,#968,#1440,
                    help='input image height')
parser.add_argument('--new_width', type=int, default=-1,
                    help='output image width')
parser.add_argument('--z_near', type=float, default=0.01,
                    help='z_near for depth computation')
parser.add_argument('--z_far', type=float, default=50,
                    help='z_far for depth computation')
parser.add_argument('--transpose', action='store_true',
                    help='alternate between landscape and portrait')

args = parser.parse_args()
dataset, data_path1, data_path2, frame, step, iter, thres, width, height, new_width, z_near, z_far, transpose = (getattr(args, arg) for arg in vars(args))
if new_width == -1:
    new_width = width
scale = new_width/width
width, height = new_width, int(scale*height)
if transpose:
    width, height = height, width
w2 = width*0.5
h2 = height*0.5

vtk.vtkObject.GlobalWarningDisplayOff()

axis_alignment1 = np.linalg.inv(load_alignment(dataset, data_path1))
pose_path2, intrinsic_depth2, axis_alignment2 = load_paths(dataset, data_path2)

# Setup renderer
vtk_renderer = vtk.vtkRenderer()
vtk_render_window = vtk.vtkRenderWindow()
vtk_render_window.SetSize(width, height)
vtk_render_window.AddRenderer(vtk_renderer)
vtk_render_window.OffScreenRenderingOn()

polydata1 = load_model(dataset, data_path1)
actor1 = polyDataToActor(polydata1)
polydata2 = load_model(dataset, data_path2)
actor2 = polyDataToActor(polydata2)
active_vtk_camera = vtk_renderer.GetActiveCamera()
active_vtk_camera.SetClippingRange(z_near, z_far)
active_vtk_camera.SetPosition(0, 0, 0)
active_vtk_camera.SetFocalPoint(0, 0, -1)
active_vtk_camera.SetViewUp(0,1,0)

R_0   = np.array([[ 1, 0,0,0], [ 0, 1,0,0], [0,0,1,0], [0,0,0,1]])
R_90  = np.array([[ 0,-1,0,0], [ 1, 0,0,0], [0,0,1,0], [0,0,0,1]])
R_180 = np.array([[-1, 0,0,0], [ 0,-1,0,0], [0,0,1,0], [0,0,0,1]])
R_270 = np.array([[ 0, 1,0,0], [-1, 0,0,0], [0,0,1,0], [0,0,0,1]])
R = [R_0, R_90, R_180, R_270]
r = [ '0', '90', '180', '270','-1']

j = 0
vmin = 1
imin = 4   #otherwise error message
while vmin > thres and j < iter:
    try:
        p_matrix2, camera_pose2 = load_camera(dataset, pose_path2, intrinsic_depth2, frame)
    except FileNotFoundError: #reached the end of the image sequence
        break #exit('{} {}'.format(r[imin], vmin))

    wcx = -(p_matrix2[0,2] - w2) / w2;
    wcy =  (p_matrix2[1,2] - h2) / h2;
    active_vtk_camera.SetWindowCenter(wcx, wcy)
    view_angle = 360 / np.pi * np.arctan2(h2, p_matrix2[0,0])
    active_vtk_camera.SetViewAngle(view_angle)

    '''Reference'''
    mat = camera_pose2.copy()
    mat = np.linalg.inv(mat)
    if dataset=='scannet':
        mat = -mat
        mat[0,:] = -mat[0,:]
        mat[3,3] = -mat[3,3]
    vmat = vtkmatrix_from_array(mat)
    active_vtk_camera.SetModelTransformMatrix(vmat)
    vtk_renderer.AddActor(actor2)
    vtk_render_window.Render()
    vtk_renderer.RemoveActor(actor2)
    image2 = getImage(vtk_render_window, width, height)

    for i in range(len(R)):
        gt_matrix = np.matmul(axis_alignment1, np.matmul(R[i], axis_alignment2))

        '''Transformed'''
        with np.errstate(invalid='raise'):
            try:
                mat = np.matmul(gt_matrix, camera_pose2)
            except FloatingPointError: #something wrong with the camera_pose ('inf')
                break #exit('{} {}'.format(r[imin], vmin))
        mat = np.linalg.inv(mat)
        if dataset=='scannet':
            mat = -mat
            mat[0,:] = -mat[0,:]
            mat[3,3] = -mat[3,3]
        vmat = vtkmatrix_from_array(mat)
        active_vtk_camera.SetModelTransformMatrix(vmat)
        vtk_renderer.AddActor(actor1)
        vtk_renderer.SetBackground(255, 255, 255) #unknown objects are not assumed the same
        vtk_render_window.Render()
        vtk_renderer.SetBackground(0, 0, 0)
        vtk_renderer.RemoveActor(actor1)
        image1 = getImage(vtk_render_window, width, height)

        diff = np.all(image1 != image2, axis=-1)
        v = np.sum(diff)/width/height
        if v < vmin and v > 0.0: #0.0 is suspicious, looking at the floor?
            vmin = v
            imin = i
    frame += step
    j += 1
print(r[imin], vmin) #print rotation and score
