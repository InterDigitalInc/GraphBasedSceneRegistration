import vtk
from vtk.util import numpy_support
import numpy as np
import os.path as osp
from os import makedirs
import argparse
from sys import exit
import cv2

import json

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

def load_info(dataset, path):
    if dataset == '3rscan':
        info = {}
        with open(osp.join(path, 'sequence', '_info.txt')) as f:
            for line in f.read().split('\n'):
                line = line.split(' = ')
                if len(line) == 2:
                    info[line[0]] = line[1]
        return info
    else:
        raise NotImplementedError

def load_json(dataset, path):
    if dataset == '3rscan':
        with open(osp.join(path, 'semseg.v2.json')) as f:
            data = json.load(f)
        return data
    else:
        raise NotImplementedError

def load_paths(dataset, path):
    if dataset == '3rscan':
        pose_path = osp.join(path, 'sequence')
        info = load_info(dataset, path)
        color_width  = int(info['m_colorWidth'])
        color_height = int(info['m_colorHeight'])
        depth_width  = int(info['m_depthWidth'])
        depth_height = int(info['m_depthHeight'])
        intrinsic_color = np.matrix(info['m_calibrationColorIntrinsic'].split(), dtype=float).reshape((4,4))
        intrinsic_depth = np.matrix(info['m_calibrationDepthIntrinsic'].split(), dtype=float).reshape((4,4))
        frame_count = int(info['m_frames.size'])
        json_data = load_json(dataset, path)
        return pose_path, color_width, color_height, depth_width, depth_height, intrinsic_color, intrinsic_depth, frame_count, json_data
    elif dataset == 'scannet':
        intrinsic_path = osp.join(path, 'intrinsic')
        pose_path = osp.join(path, 'pose')
        intrinsic_depth = open(osp.join(intrinsic_path, 'intrinsic_depth.txt')).read()
        intrinsic_depth = [item.split() for item in intrinsic_depth.split('\n')[:-1]]
        axis_alignment = load_alignment(dataset, path)
        return pose_path, intrinsic_depth, axis_alignment
    else:
        raise NotImplementedError

def load_model(dataset, path):
    if dataset == '3rscan':
        file_name = osp.join(path, 'labels.instances.annotated.v2.ply')
    elif dataset == 'scannet':
        file_name = osp.join(path, osp.basename(path)+'_vh_clean_2.labels.ply')
    else:
        raise NotImplementedError
    polydata = load_polydata(file_name)
    return polydata

def load_pose(dataset, pose_path, frame):
    if dataset == '3rscan':
        camera_pose = open(osp.join(pose_path, 'frame-{:06d}.pose.txt'.format(frame))).read()
        camera_pose = [item.split() for item in camera_pose.split('\n')[:-1]]
        camera_pose = np.array(camera_pose, dtype=float)
        return camera_pose
    elif dataset == 'scannet':
        camera_pose = open(osp.join(pose_path, str(frame) + '.txt')).read()
        camera_pose = [item.split() for item in camera_pose.split('\n')[:-1]]
        camera_pose = np.array(camera_pose, dtype=float)
        return camera_pose
    else:
        raise NotImplementedError

def getImage(vtk_render_window, width, height):
    vtk_array = vtk.vtkUnsignedCharArray()
    vtk_render_window.GetPixelData(0, 0, width-1, height-1, 1, vtk_array)
    numpy_array = numpy_support.vtk_to_numpy(vtk_array)
    numpy_array = np.reshape(numpy_array, (-1, width, 3))
    numpy_array = np.flipud(numpy_array)  # flipping along the first axis (y)
    return numpy_array

def getDepth(vtk_render_window, width, height, z_near, z_far):
    vtk_array = vtk.vtkFloatArray()
    vtk_render_window.GetZbufferData(0, 0, width-1, height-1, vtk_array)
    numpy_array = numpy_support.vtk_to_numpy(vtk_array)
    numpy_array = np.reshape(numpy_array, (-1, width))
    numpy_array = np.flipud(numpy_array)  # flipping along the first axis (y)
    numerator = 2.0 * z_near * z_far
    denominator = z_far + z_near - (2.0 * numpy_array - 1.0) * (z_far - z_near)
    numpy_array_2 = numerator / denominator
    numpy_array_2[numpy_array == 1.0] = np.nan
    return numpy_array_2

parser = argparse.ArgumentParser('Generates 2D label maps from a 3D model')
parser.add_argument('--dataset', type=str, default='3rscan',
                    help='dataset type [??? | 3rscan]')
parser.add_argument('--data_path', type=str, default='/DATA/3RScan/scans/0a4b8ef6-a83a-21f2-8672-dce34dd0d7ca/',
                    help='3D labelled model (in)')
parser.add_argument('--label_path', type=str, default='label',
                    help='folder to write label images (out)')
parser.add_argument('--depth_path', type=str, default='depth',
                    help='folder to write depth images (out)')
parser.add_argument('--start', type=int, default=0,
                    help='start frame')
parser.add_argument('--count', type=int, default=None,
                    help='number of frames to do')
parser.add_argument('--width', type=int, default=None,#224,#960,
                    help='output image width')
parser.add_argument('--height', type=int, default=None,#172,#540,
                    help='output image height')
parser.add_argument('--z_near', type=float, default=0.01,
                    help='z_near for depth computation')
parser.add_argument('--z_far', type=float, default=50,
                    help='z_far for depth computation')
parser.add_argument('--transpose', action='store_true',
                    help='alternate between landscape and portrait')
parser.add_argument('--preview', action='store_true',
                    help='show images')

args = parser.parse_args()
dataset, data_path, label_path, depth_path, start, iter, width, height, z_near, z_far, transpose, preview = (getattr(args, arg) for arg in vars(args))

pose_path, color_width, color_height, ref_width, ref_height, intrinsic_color, intrinsic_depth, count, json_data = load_paths(dataset, data_path)
if iter is None or iter > count: iter = count
if width  is None: width  = color_width
if height is None: height = color_height
intrinsic = intrinsic_color
w2 = 0.5*width
h2 = 0.5*height
wcx = -(intrinsic[0,2] - w2) / w2;
wcy =  (intrinsic[1,2] - h2) / h2;
view_angle = 360 / np.pi * np.arctan2(h2, intrinsic[1,1])

vtk.vtkObject.GlobalWarningDisplayOff()

# Setup renderer
vtk_renderer = vtk.vtkRenderer()
vtk_renderer.SetBackground(0, 0, 0)

vtk_render_window = vtk.vtkRenderWindow()
vtk_render_window.SetSize(width, height)
vtk_render_window.AddRenderer(vtk_renderer)
vtk_render_window.OffScreenRenderingOn()

polydata = load_model(dataset, data_path)
actor = polyDataToActor(polydata)
actor.GetProperty().SetInterpolation(0)
vtk_renderer.AddActor(actor)
#vtk_renderer.RemoveActor(actor)

#print(type(polydata), polydata)
#empty #polydata.GetVerts()
#empty #polydata.GetLines()
#triangles #polydata.GetPolys(), polydata.GetPolys().GetData()
#vertices #polydata.GetPoints(), polydata.GetPoints().GetData()
#empty #polydata.GetCellData()
#colors #polydata.GetPointData(), polydata.GetPointData().GetScalars()

#assuming subtuples of size n inside one large tuple
def getFace(vtkpolydata, idx, n=3):
    data = vtkpolydata.GetPolys().GetData()
    if idx >= vtkpolydata.GetNumberOfPolys():
        print('out of bounds')
        return None
    if int(data.GetComponent((n+1)*idx, 0)) != n:
        print('size is wrong')
        return None
    return [int(data.GetComponent((n+1)*idx+i+1, 0)) for i in range(n)]

def getPoint(vtkpolydata, idx, n=3):
    data = vtkpolydata.GetPoints().GetData()
    if idx >= vtkpolydata.GetNumberOfPoints():
        print('out of bounds')
        return None
    if n > data.GetNumberOfTuples():
        print('size is wrong')
        return None
    return [data.GetComponent(idx, i) for i in range(n)]

def getColor(vtkpolydata, idx, n=3):
    data = vtkpolydata.GetPointData().GetScalars()
    if idx >= vtkpolydata.GetNumberOfPoints():
        print('out of bounds')
        return None
    if n > data.GetNumberOfTuples():
        print('size is wrong')
        return None
    return [int(data.GetComponent(idx, i)) for i in range(n)]

#for i in range(100):
#    print(getFace(polydata,i))
#for i in range(100):
#    print(getPoint(polydata,i))
#for i in range(100):
#   print(getColor(polydata,i))

groups = json_data['segGroups']
print(type(groups), len(groups))
#all_segments = []
#uni_segments = set()
all_ranges = []
uni_ranges = set()
all_colors = []
uni_colors = set()
for group in groups:
    segments = group['segments']
    colors = set()
    i = 0
    # while i < len(segments)-1:
    #     if segments[i] < segments[i+1]:
    #         all_ranges += list(range(segments[i],segments[i+1]))
    #         uni_ranges |=  set(range(segments[i],segments[i+1]))
    #         i += 2
    #     else:
    #         all_ranges +=     [segments[i]]
    #         uni_ranges |= set([segments[i]])
    #         i += 1
    for segment in segments:
        face = getFace(polydata, segment)
        #col = set()
        for vert in face:
            #col.append(tuple(getColor(polydata, vert)))
            #col.add(tuple(getColor(polydata, vert)))
            colors.add(tuple(getColor(polydata, vert)))
        #print(segment, col)
    all_colors += list(colors)
    uni_colors |= colors
    #print(colors)
    #exit(0)
    #all_segments += segments
    #uni_segments |= set(segments)
    #print(group['objectId'], type(segments), len(segments))
# print(len(all_segments))
# print(len(uni_segments))
# print(min(uni_segments))
# print(max(uni_segments))
# print(len(all_ranges))
# print(len(uni_ranges))
print(len(all_colors))
print(len(uni_colors))
print(uni_colors)
exit(0)

active_vtk_camera = vtk_renderer.GetActiveCamera() #vtkRenderingOpenGL2Python.vtkOpenGLCamera
active_vtk_camera.SetClippingRange(z_near, z_far)
active_vtk_camera.SetPosition(0, 0, 0)
active_vtk_camera.SetFocalPoint(0, 0, -1)
active_vtk_camera.SetViewUp(0, 1, 0)
#active_vtk_camera.SetUseHorizontalViewAngle(True)
active_vtk_camera.SetWindowCenter(wcx, wcy)
active_vtk_camera.SetViewAngle(view_angle)
#active_vtk_camera.SetUseExplicitProjectionTransformMatrix(True)
#active_vtk_camera.SetExplicitProjectionTransformMatrix(intrinsic_depth)

# gl_camera = vtk.vtkExternalOpenGLCamera()
# gl_camera.SetClippingRange(z_near, z_far)
# gl_camera.SetPosition(0, 0, 0)
# gl_camera.SetFocalPoint(0, 0, -1)
# gl_camera.SetViewUp(0, 1, 0)
# gl_camera.SetViewTransformMatrix(np.reshape(np.eye(4,dtype=np.double),[-1,1]))
# gl_camera.SetModelTransformMatrix(np.reshape(np.eye(4,dtype=np.double),[-1,1]))
#gl_camera.SetProjectionTransformMatrix(np.reshape(np.transpose(intrinsic),[-1,1]))#np.reshape(np.transpose(P),[-1,1]))
#vtk_renderer.SetActiveCamera(gl_camera)

label_path = osp.join(data_path, label_path)
if not osp.exists(label_path): makedirs(label_path)
depth_path = osp.join(data_path, depth_path)
if not osp.exists(depth_path): makedirs(depth_path)

for frame in range(start, start+iter):
    try:
        camera_pose = load_pose(dataset, pose_path, frame)
    except FileNotFoundError: #reached the end of the image sequence
        break #exit('{} {}'.format(r[imin], vmin))
    #print(camera_pose)

    '''Label image'''
    mat = camera_pose.copy()
    mat = np.linalg.inv(mat)
    if dataset=='scannet' or dataset=='3rscan':
        mat = -mat
        mat[0,:] = -mat[0,:]
        mat[3,3] = -mat[3,3]
    #gl_camera.SetViewTransformMatrix(np.reshape(np.transpose(mat),[-1,1]))
    vmat = vtkmatrix_from_array(mat)
    active_vtk_camera.SetModelTransformMatrix(np.reshape(mat,[-1,1]))
    #active_vtk_camera.SetModelTransformMatrix(vmat)
    vtk_render_window.Render()
    label_image = getImage(vtk_render_window, width, height)
    depth_image = getDepth(vtk_render_window, width, height, z_near, z_far)
    #color_image = cv2.imread(osp.join(pose_path, 'frame-{:06d}.color.jpg'.format(frame)))
    #ref_image = cv2.imread(osp.join(pose_path, 'frame-{:06d}.depth.pgm'.format(frame)), cv2.IMREAD_UNCHANGED)
    # if transpose:
    #     label_image = cv2.rotate(label_image, cv2.ROTATE_90_CLOCKWISE)
    #     depth_image = cv2.rotate(depth_image, cv2.ROTATE_90_CLOCKWISE)
    #     color_image = cv2.rotate(color_image, cv2.ROTATE_90_CLOCKWISE)
    cv2.imwrite(osp.join(label_path, '{}.png'.format(frame)), label_image)
    cv2.imwrite(osp.join(depth_path, '{}.png'.format(frame)), depth_image)

    if preview:
        w = width#height if transpose else width
        h = height#width if transpose else height
        #preview_image = color_image.copy()
        preview_image = cv2.imread(osp.join(pose_path, 'frame-{:06d}.color.jpg'.format(frame)))
        preview_image[:h//2,:w//2,:] = label_image[:h//2,:w//2,:]
        preview_image[h//2:,w//2:,0] = 127*depth_image[h//2:,w//2:]
        preview_image[h//2:,w//2:,1] = 127*depth_image[h//2:,w//2:]
        preview_image[h//2:,w//2:,2] = 127*depth_image[h//2:,w//2:]
        if transpose:
            preview_image = cv2.rotate(preview_image, cv2.ROTATE_90_CLOCKWISE)
        cv2.imshow('Alignment', preview_image)
        #cv2.imshow('color', color_image)
        #cv2.imshow('reference', 32*ref_image)
        #cv2.imshow('label', label_image)
        #cv2.imshow('depth', 0.5*depth_image)
        if cv2.waitKey(1) == 27:
            break;
cv2.destroyAllWindows()
