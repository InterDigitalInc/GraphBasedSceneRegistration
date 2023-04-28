import cv2
import numpy as np
import glob
import argparse
import quaternion

def pose2str(i, tx, ty, tz, qx, qy, qz, qw):
    return '{} {} {} {} {} {} {} {}'.format(i, tx, ty, tz, qx, qy, qz, qw)

parser = argparse.ArgumentParser('Match a ChangeSim sequence\' raw depth maps to the processed ones to export a list of trajectories')
parser.add_argument('--depth', type=str, default='/DATA/ChangeSim/',#Ref_Seq_Test/Warehouse_6/Seq_0/depth/',
                    help='folder containing the 8bit depth maps')
parser.add_argument('--raw_depth', type=str, default='/DATA/ChangeSim/',#Ref_Seq_Test/Warehouse_6/Seq_0/raw/depth/',
                    help='folder containing the 16bit depth maps')
parser.add_argument('--output', type=str, default='/SOURCES/Semantic-Graph-based--global-Localization/Dataset/',
                    help='folder containing the 16bit depth maps')
parser.add_argument('--type', type=str, default='Ref',
                    help='Ref/Query')
parser.add_argument('--mode', type=str, default='',
                    help='Test/Train')
parser.add_argument('--name', type=str, default=None,
                    help='full instance name')
parser.add_argument('--instance', type=str, default='6',
                    help='instance number')
parser.add_argument('--seq', type=str, default='0',
                    help='sequence number')
parser.add_argument('--start', type=int, default=-1,
                    help='frames to skip')
parser.add_argument('--raw_start', type=int, default=-1,
                    help='raw frames to skip')
parser.add_argument('--preview', action='store_true',
                    help='show the matched depth maps')
parser.add_argument('--debug', action='store_true',
                    help='print debug text')

args = parser.parse_args()
DEPdir, RAWdir, OUTdir, dtype, dmode, name, ins, seq, start, raw_start, show, verbose = (getattr(args, arg) for arg in vars(args))
if name is not None: ins = name.split('_')[-1]
else: name = 'Warehouse_'+ins
if not dmode: dmode = 'Train' if int(ins) < 6 else 'Test'
#             0         1       2       3       4       5         6         7         8         9
starts =     [[ -1, -1],[-1,-1],[-1,-1],[-1,-1],[11,79],[ 61,  0],[ 11,  6],[  8,  0],[ 12,  9],[  4,  4]]
raw_starts = [[ -1, -1],[-1,-1],[-1,-1],[-1,-1],[19,26],[487,555],[592,132],[175,552],[ 99,126],[121,129]]
if     start < 0:     start =     starts[int(ins)][int(seq)]
if raw_start < 0: raw_start = raw_starts[int(ins)][int(seq)]

path = DEPdir+dtype+'_Seq_'+dmode+'/'+name+'/Seq_'+seq+'/depth/'
if verbose: print(path)
dep_files = sorted(glob.glob(path+'*.png'), key=lambda file: int(file[len(path):-4]))

path = RAWdir+dtype+'_Seq_'+dmode+'/'+name+'/Seq_'+seq+'/raw/depth/'
if verbose: print(path)
raw_files = sorted(glob.glob(path+'*.png'), key=lambda file: int(file[len(path):-4]))

path = RAWdir+dtype+'_Seq_'+dmode+'/'+name+'/Seq_'+seq+'/raw/poses.g2o'
if verbose: print(path)
with open(path, "r") as f:
    lines = f.read().split('\n')[1:-1] #remove first and last line
poses = []
for line in lines:
    pose = line.split()
    if pose[0] != 'VERTEX_SE3:QUAT':
        break
    else:
        poses.append((int(pose[1]),
            np.array([float(value) for value in pose[2:5]]),
            quaternion.as_quat_array([float(value) for value in pose[5:]])))

#path = OUTdir+'Warehouse_'+ins+'_'+dtype+'_'+seq+'/trajectory.txt'
path = OUTdir+dtype+'_'+name+'_'+seq+'/trajectory.txt'
if verbose: print(path)
out_file = open(path, "w")

if verbose: print("frame counts", len(dep_files), len(raw_files))
img = cv2.imread(dep_files[0], cv2.IMREAD_UNCHANGED)
height, width = img.shape[:2]
pixels = height*width
if verbose: print("resolution:", width, height)
print(len(dep_files))
out_file.write(str(len(dep_files))+'\n')

count     = len(dep_files)
raw_count = len(raw_files)
ratio     = raw_count/count
if ratio < 1 :
    print("Less raw images than processed ones")
    exit(0)
forward   = int(6*ratio) #8
backward  = int(1*ratio) #2

max_val = 0 #for the preview
pose_i  = 0
raw_beg = raw_start
#start by going backwards
out_poses = []
for dep_i in range(start, -1, -1):
    dep = cv2.imread(dep_files[dep_i], cv2.IMREAD_UNCHANGED)
    mask = cv2.inRange(dep, 0, 254) #filter bad points
    dep = np.float64(dep)
    min_val = pixels*511 #big value
    min_loc = -1
    raw_end = max(raw_beg-forward, 0)         #reversed
    for raw_i in range(raw_beg, raw_end, -1): #look for the best frame
        raw = cv2.imread(raw_files[raw_i], cv2.IMREAD_UNCHANGED)/200
        delta = cv2.norm(raw, dep, normType=cv2.NORM_L2, mask=mask)
        if delta < min_val:
            min_val = delta
            min_loc = raw_i
    raw_beg = min(min_loc+backward, raw_count) #for next loop, reversed

    #min_loc+1 gives the image name (indexation starts at 1), reversed
    while poses[pose_i][0] > min_loc+1 and pose_i > 0:            pose_i -= 1
    while poses[pose_i][0] < min_loc+1 and pose_i < len(poses)-2: pose_i += 1

    p, t, q = poses[pose_i]
    if p != min_loc+1: #need for interpolation, reversed
        p2, t2, q2 = poses[pose_i-1]
        r = float(p-min_loc+1)/float(p-p2)
        t = (1-r)*t+r*t2
        q = quaternion.slerp_evaluate(q, q2, r)

    if verbose: print(min_loc+1,min_val,pixels-np.count_nonzero(mask), end='\t')
    print(pose2str(dep_i, t[0], t[1], t[2], q.w, q.x, q.y, q.z))
    out_poses.append(pose2str(dep_i, t[0], t[1], t[2], q.w, q.x, q.y, q.z))
    if show and (min_val > max_val or verbose):
        print('\tWORST:',min_loc+1,min_val,max_val)
        max_val = min_val
        raw = cv2.imread(raw_files[min_loc], cv2.IMREAD_UNCHANGED)/200
        # preview = np.zeros((height, width*3), np.float64)
        # preview[:,:width] = dep
        # preview[:,width:width*2] = raw
        # preview[:,width*2:width*3] = cv2.absdiff(raw, dep)*127
        preview = cv2.absdiff(raw, dep)*127
        cv2.imshow("preview", preview/255)
        cv2.waitKey(1)

for pose in reversed(out_poses[1:]):
    out_file.write(pose+'\n')

max_val = 0 #for the preview
pose_i  = 0
raw_beg = raw_start
for dep_i in range(start, count):
    dep = cv2.imread(dep_files[dep_i], cv2.IMREAD_UNCHANGED)
    mask = cv2.inRange(dep, 0, 254) #filter bad points
    dep = np.float64(dep)
    min_val = pixels*511 #big value
    min_loc = -1
    raw_end = min(raw_beg+forward, raw_count)
    for raw_i in range(raw_beg, raw_end): #look for the best frame
        raw = cv2.imread(raw_files[raw_i], cv2.IMREAD_UNCHANGED)/200
        delta = cv2.norm(raw, dep, normType=cv2.NORM_L2, mask=mask)
        if delta < min_val:
            min_val = delta
            min_loc = raw_i
    raw_beg = max(min_loc-backward, 0) #prepare for next loop

    #min_loc+1 gives the image name (indexation starts at 1)
    while poses[pose_i][0] < min_loc+1 and pose_i < len(poses)-2: pose_i += 1
    while poses[pose_i][0] > min_loc+1 and pose_i > 0:            pose_i -= 1

    p, t, q = poses[pose_i]
    if p != min_loc+1: #need for interpolation
        p2, t2, q2 = poses[pose_i+1]
        r = float(min_loc+1-p)/float(p2-p)
        t = (1-r)*t+r*t2
        q = quaternion.slerp_evaluate(q, q2, r)

    if verbose: print(min_loc+1,min_val,pixels-np.count_nonzero(mask), end='\t')
    print(pose2str(dep_i, t[0], t[1], t[2], q.w, q.x, q.y, q.z))
    out_file.write(pose2str(dep_i, t[0], t[1], t[2], q.w, q.x, q.y, q.z)+'\n')
    if show and (min_val > max_val or verbose):
        print('\tWORST:',min_loc+1,min_val,max_val)
        max_val = min_val
        raw = cv2.imread(raw_files[min_loc], cv2.IMREAD_UNCHANGED)/200
        # preview = np.zeros((height, width*3), np.float64)
        # preview[:,:width] = dep
        # preview[:,width:width*2] = raw
        # preview[:,width*2:width*3] = cv2.absdiff(raw, dep)*127
        preview = cv2.absdiff(raw, dep)*127
        cv2.imshow("preview", preview/255)
        cv2.waitKey(1)
out_file.close()
