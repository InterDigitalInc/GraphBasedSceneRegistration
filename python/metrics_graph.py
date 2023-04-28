import numpy as np
import os.path as osp
import argparse
import matplotlib.pyplot as plt

def adjustedRandIndex(I, J, n, nI, nJ):
    if len(I) != n or len(J) != n:
        print('could not compute ARI')
        return 0
    #A = [np.count_nonzero(I == i) for i in range(nI)]
    #B = [np.count_nonzero(J == j) for j in range(nJ)]
    #C = [np.count_nonzero(I == i & J == j) for i in range(nI) for j in range(nJ)]
    A = (np.sum(np.square([np.count_nonzero(I == i) for i in range(nI)]))-n)/2 #if larger than C : fusion
    B = (np.sum(np.square([np.count_nonzero(J == j) for j in range(nJ)]))-n)/2 #if larger than C : fission
    C = (np.sum(np.square([np.count_nonzero((I == i) & (J == j))
        for i in range(nI) for j in range(nJ)]))-n)/2

    #D = 2.0*sA2*sB2/(n*(n-1))
    D = 2.0*A*B/(n*(n-1))
    #return (sC2-D)/((sA2+sB2)/2.0-D);
    return (C-D)/((A+B)/2.0-D);

def load_snodes(data_path):
    with open(data_path+'.txt', "r") as f:
        lines = [line.split() for line in f.read().split('\n')[:-1]]
    counts    = [line[0]  for line in lines[1:] if len(line)==1] #np.cumsum
    data      = [line[1:] for line in lines[1:] if len(line)>11]
    positions = np.array([line[0:3] for line in data], dtype=float)
    objlabels = np.array([line[3]   for line in data], dtype=int)
    framenums = np.array([line[4]   for line in data], dtype=int)
    bounboxes = np.array([line[5:9] for line in data], dtype=int)
    change_gt = np.array([line[9]   for line in data], dtype=int)
    instan_gt = np.array([line[10]  for line in data], dtype=int)-1
    instances = np.array([idx for i,count in enumerate(counts)
        for idx in [i]*int(count)], dtype=int)
    num_instan_gt = np.max(instan_gt)+1
    num_instances = len(counts)
    num_nodes     = len(instances)
    super_indices = np.cumsum(np.array([0]+counts[:-1],int))
    return (positions, objlabels, framenums, bounboxes, change_gt, instan_gt,
        instances, num_instan_gt, num_instances, num_nodes, super_indices)

def triTostr(arr, justified=True, debug=False, floating=6):
    string = ''
    Rj = ['']*len(arr)
    if justified:
        for i, rj in enumerate(Rj):
            rj = int(np.log10(np.amax(arr[i:,i])))+floating+2
    for i, row in enumerate(arr):
        for val, rj in zip(row[:i+1], Rj):
            string += '{v:{p}.{f}f} '.format(p=rj,v=val,f=floating)
        string += '\n'
    if debug:
        print(string)
    return string

def strTotri(string, filter=None, fill=0, debug=False):
    list = [float(val) for line in string.split('\n') for val in line.split()]
    n = int((np.sqrt(1+8*len(list))-1)/2)
    arr = np.full((n,n), fill, dtype=float)
    i = j = 0
    for val in list:
        if filter is None:
            arr[j,i] = val
        elif filter[i] == filter[j]:
            arr[j,i] = val
        i += 1
        if i > j:
            j += 1
            i = 0
    if debug:
        print(arr)
    return arr

def linTostr(arr, linebreak=10, debug=False, floating=6):
    string = ''
    for i, val in enumerate(arr):
        string += '{v:.{f}f} '.format(v=val,f=floating)
        if (i+1)%linebreak == 0:
            string += '\n'
    if debug:
        print(string)
    return string

dataset='ESANet'#'3DGT'#
parser = argparse.ArgumentParser('Options to convert dataset to ScanNet specifications')
parser.add_argument('--data_path', type=str, default='/SOURCES/Semantic-Graph-based--global-Localization/build/snodes-backups/snodes-SN-'+dataset,
                    help='folder containing scans')
parser.add_argument('--name', type=str, default=None,#'scene0005_00',#
                    help='name of scene')
parser.add_argument('--suffix', type=str, default='_0_S_f',
                    help='name of scene')
parser.add_argument('--out_path', type=str, default='/SOURCES/Semantic-Graph-based--global-Localization/build/metrics/snodes-SN-'+dataset,
                    help='folder to save the metrics')
parser.add_argument('--num_labels', type=int, default=40,
                    help='number of object labels in the dataset')

args = parser.parse_args()
data_path, name, suffix, out_path, num_labels = (getattr(args, arg) for arg in vars(args))
files_dist = []
files_filt = []
files_ARI = []
files = []

file_dist_avg = osp.join(out_path, '{}_dist_avg.txt')
file_dist_min = osp.join(out_path, '{}_dist_min.txt')
file_dist_max = osp.join(out_path, '{}_dist_max.txt')
file_filt_avg = osp.join(out_path, '{}_filt_avg.txt')
file_filt_min = osp.join(out_path, '{}_filt_min.txt')
file_filt_max = osp.join(out_path, '{}_filt_max.txt')
file_ARI = osp.join(out_path, '{}_ARI.txt')
file_rad_avg = out_path+'_rad_avg.txt'
file_rad_min = out_path+'_rad_min.txt'
file_rad_max = out_path+'_rad_max.txt'
file_sep_avg = out_path+'_sep_avg.txt'
file_sep_min = out_path+'_sep_min.txt'
file_sep_max = out_path+'_sep_max.txt'

if name:
    files_dist.append(name)
    files_ARI.append(name)
else:
    for scene_n in range(1000):
        for scene_i in range(10):
            file = 'scene{:04d}_{:02d}'.format(scene_n,scene_i)
            if osp.isfile(osp.join(data_path, file+suffix+'.txt')):
                files.append(file)

                if (osp.isfile(file_dist_avg.format(file)) and
                    osp.isfile(file_dist_min.format(file)) and
                    osp.isfile(file_dist_max.format(file))):
                    print(file+' distance files exits')
                else:
                    files_dist.append(file)

                if (osp.isfile(file_filt_avg.format(file)) and
                    osp.isfile(file_filt_min.format(file)) and
                    osp.isfile(file_filt_max.format(file))):
                    print(file+' filtered files exits')
                else:
                    files_filt.append(file)

                if osp.isfile(file_ARI.format(file)):
                    print(file+' ARI file exits')
                else:
                    files_ARI.append(file)

for f,file in enumerate(files_dist):
    positions, _,_,_,_,_, instances,_, num_instances, num_nodes, indices = load_snodes(osp.join(data_path,file+suffix))
    print('{}/{} {} ({} nodes)'.format(f+1, len(files_dist), file, num_nodes))
    dists_avg = np.zeros((num_instances,num_instances), dtype=float)
    dists_min = np.full( (num_instances,num_instances), np.inf, dtype=float)
    dists_max = np.zeros((num_instances,num_instances), dtype=float)
    dists_cnt = np.zeros((num_instances,num_instances), dtype=int)
    bar = 0
    sup_nodes = np.split(range(num_nodes), indices[1:])
    for s, ss in enumerate(sup_nodes): #for every supernode ss (of index s)
        ns = instances[ss[0]]
        for j, i in enumerate(ss): #for each node in the supernode
            if int(10*i/num_nodes)%10 == bar:
                print(str(10*bar),end='%-',flush=True)
                bar += 1
            pi = positions[i]
            #same supernode:
            if j+1 < len(ss):
                dists_i = np.linalg.norm(positions[ss[j+1:]]-pi, axis=1)
                dists_min[ns,ns]  = np.minimum(dists_min[ns,ns], np.amin(dists_i))
                dists_max[ns,ns]  = np.maximum(dists_max[ns,ns], np.amax(dists_i))
                dists_avg[ns,ns] += np.sum(dists_i)
                dists_cnt[ns,ns] += len(ss)-j-1
            #other supernodes:
            if s+1 < len(sup_nodes):
                for t, st in enumerate(sup_nodes[s+1:]): #for every supernode after ss
                    nt = instances[st[0]]
                    dists_i = np.linalg.norm(positions[st]-pi, axis=1)
                    dists_min[nt,ns]  = np.minimum(dists_min[nt,ns], np.amin(dists_i))
                    dists_max[nt,ns]  = np.maximum(dists_max[nt,ns], np.amax(dists_i))
                    dists_avg[nt,ns] += np.sum(dists_i)
                    dists_cnt[nt,ns] += len(st)
    print('100%')
    dists_avg[dists_cnt>0] = dists_avg[dists_cnt>0] / dists_cnt[dists_cnt>0].astype(float)
    with open(file_dist_avg.format(file), "w") as out_file:
        out_file.write(triTostr(dists_min))
    with open(file_dist_min.format(file), "w") as out_file:
        out_file.write(triTostr(dists_max))
    with open(file_dist_max.format(file), "w") as out_file:
        out_file.write(triTostr(dists_avg))

for f,file in enumerate(files_filt):
    _, objlabels,_,_,_,_,_,_,_,_, indices = load_snodes(osp.join(data_path,file+suffix))
    print('{}/{} {} ({} instances)'.format(f+1, len(files_filt), file, len(indices)))
    labels = objlabels[indices]
    with open(file_dist_avg.format(file)) as in_file, open(file_filt_avg.format(
                                   file), "w") as out_file:
        out_file.write(strTotri(in_file.read(), labels))
    with open(file_dist_min.format(file)) as in_file, open(file_filt_min.format(
                                   file), "w") as out_file:
        out_file.write(strTotri(in_file.read(), labels))
    with open(file_dist_max.format(file)) as in_file, open(file_filt_max.format(
                                   file), "w") as out_file:
        out_file.write(strTotri(in_file.read(), labels))

if not (osp.isfile(file_rad_avg) and osp.isfile(file_rad_min) and
        osp.isfile(file_rad_max) and osp.isfile(file_sep_avg) and
        osp.isfile(file_sep_min) and osp.isfile(file_sep_max)):
    rad_avg = np.zeros((num_labels), dtype=float)
    rad_min = np.full( (num_labels), np.inf, dtype=float)
    rad_max = np.zeros((num_labels), dtype=float)
    rad_cnt = np.zeros((num_labels), dtype=int)
    sep_avg = np.zeros((num_labels), dtype=float)
    sep_min = np.full( (num_labels), np.inf, dtype=float)
    sep_max = np.zeros((num_labels), dtype=float)
    sep_cnt = np.zeros((num_labels), dtype=int)
    for f,file in enumerate(files):
        _, objlabels,_,_,_,_,_,_,num_instances, num_nodes, indices = load_snodes(osp.join(data_path,file+suffix))
        print('{}/{} {} ({} instances)'.format(f+1, len(files), file, len(indices)))
        labels = objlabels[indices]-1
        with open(file_filt_avg.format(file)) as in_file:
            filts_avg = strTotri(in_file.read(), labels)
        with open(file_filt_min.format(file)) as in_file:
            filts_min = strTotri(in_file.read(), labels)
        with open(file_filt_max.format(file)) as in_file:
            filts_max = strTotri(in_file.read(), labels)
        indices = np.append(indices, [num_nodes])
        counts = [v-u for u,v in zip(indices[:-1],indices[1:])]
        filts_min[filts_min==0] = np.inf
        for k,label in enumerate(labels):
            rad_avg[label] += counts[k]*filts_avg[k,k] #ponderation??
            rad_min[label] = np.minimum(rad_min[label], filts_min[k,k])
            rad_max[label] = np.maximum(rad_max[label], filts_max[k,k])
            rad_cnt[label] += counts[k]
            if k+1 < num_instances:
                sep_avg[label] += np.sum(counts[k+1:]*filts_avg[k+1:,k]) #ponderation??
                sep_min[label] = np.minimum(sep_min[label], np.amin(filts_min[k+1:,k]))
                sep_max[label] = np.maximum(sep_max[label], np.amax(filts_max[k+1:,k]))
                sep_cnt[label] += np.sum((counts*(labels==label).astype(int))[k+1:])
    rad_avg[rad_cnt>0] = rad_avg[rad_cnt>0] / rad_cnt[rad_cnt>0].astype(float)
    rad_min[rad_cnt==0] = 0
    sep_avg[sep_cnt>0] = sep_avg[sep_cnt>0] / sep_cnt[sep_cnt>0].astype(float)
    sep_min[sep_cnt==0] = 0
    with open(file_rad_avg, "w") as out_file: out_file.write(linTostr(rad_avg))
    with open(file_rad_min, "w") as out_file: out_file.write(linTostr(rad_min))
    with open(file_rad_max, "w") as out_file: out_file.write(linTostr(rad_max))
    with open(file_sep_avg, "w") as out_file: out_file.write(linTostr(sep_avg))
    with open(file_sep_min, "w") as out_file: out_file.write(linTostr(sep_min))
    with open(file_sep_max, "w") as out_file: out_file.write(linTostr(sep_max))

for f,file in enumerate(files_ARI):
    _,_,_,_,_, instan_gt, instances, num_instan_gt, num_instances, num_nodes,_ = load_snodes(osp.join(data_path,file+suffix))
    print('{}/{} {} ({} nodes)'.format(f+1, len(files_ARI), file, num_nodes))
    with open(file_ARI.format(file), "w") as out_file:
        out_file.write(str(adjustedRandIndex(instances, instan_gt, num_nodes,
            num_instances, num_instan_gt)))
