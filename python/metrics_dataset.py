import numpy as np
import os.path as osp
import argparse
import matplotlib.pyplot as plt

def load_alignment(dataset, path):
    if dataset == 'scannet':
        axis_alignment = open(osp.join(path, osp.basename(path)+'.txt')).read()
        axis_alignment = [float(item) for item in axis_alignment.split('\n')[0].split()[2:]]
        return axis_alignment
    else:
        raise NotImplementedError

def load_paths(dataset, path):
    if dataset == 'scannet':
        intrinsic_path = osp.join(path, 'intrinsic')
        pose_path = osp.join(path, 'pose')
        intrinsic_depth = open(osp.join(intrinsic_path, 'intrinsic_depth.txt')).read()
        intrinsic_depth = [item.split() for item in intrinsic_depth.split('\n')[:-1]]
        p_matrix = np.matrix([ intrinsic_depth[0][:], intrinsic_depth[1][:], intrinsic_depth[2][:]], dtype='float')
        return pose_path, p_matrix, load_alignment(dataset, path)
    else:
        raise NotImplementedError

def load_pose(dataset, pose_path, frame):
    if dataset == 'scannet':
        try:
          camera_pose = open(osp.join(pose_path, str(frame) + '.txt')).read()
        except IOError:
          print("Frame {} does not exist".format(frame))
          return None
        camera_pose = [item.split() for item in camera_pose.split('\n')[:-1]]
        camera_pose = np.array(camera_pose, dtype=float)
        return camera_pose
    else:
        raise NotImplementedError

parser = argparse.ArgumentParser('Options to convert dataset to ScanNet specifications')
parser.add_argument('--dataset', type=str, default='scannet',
                    help='dataset type [??? | scannet]')
parser.add_argument('--data_path', type=str, default='/DATA/ScanNet/scans',
                    help='folder containing scans')
parser.add_argument('--name', type=str, default='scene0005_00',
                    help='name of scene')
parser.add_argument('--frame', type=int, default=1,
                    help='start frame number')
parser.add_argument('--step', type=int, default=1,
                    help='frame step')
parser.add_argument('--iter', type=int, default=-1,
                    help='number of steps')
parser.add_argument('--width', type=int, default=640,#1298,#1920,
                    help='input image width')
parser.add_argument('--height', type=int, default=480,#968,#1440,
                    help='input image height')
parser.add_argument('--new_width', type=int, default=None,
                    help='output image width')
parser.add_argument('--transpose', action='store_true',
                    help='alternate between landscape and portrait')

args = parser.parse_args()
dataset, data_path, name, frame, step, iter, width, height, new_width, transpose = (getattr(args, arg) for arg in vars(args))
data_path = osp.join(data_path, name)
if new_width is None:
    new_width = width
scale = new_width/width
width, height = new_width, int(scale*height)
if transpose:
    width, height = height, width
aspect_ratio = width/height

pose_path, p_matrix, axis_alignment = load_paths(dataset, data_path)

'''
RMSD speed?
'''
framerate = 25

norm = np.linalg.norm
inv  = np.linalg.inv

inv_pose = inv(load_pose(dataset, pose_path, frame))
frame += step
#Python 3.8
#print('frame', 'delta_T', 'delta_R', sep='\t')
F_X = []
T_Y = []
R_Y = []
while (new_pose := load_pose(dataset, pose_path, frame)) is not None and iter != 0:
    delta_pose = np.matmul(inv_pose, new_pose)
    delta_T = framerate*norm(delta_pose[:3,3])
    delta_R = framerate*norm(delta_pose[:3,:3]-np.eye(3))
    #print(frame, delta_T, delta_R, sep='\t')
    F_X.append(frame)
    T_Y.append(framerate*delta_T)
    R_Y.append(framerate*delta_R)
    frame += step
    if iter > 0: iter -= 1
    inv_pose = inv(new_pose)

plt.show()
