import os
import json
import argparse

parser = argparse.ArgumentParser('Organize 3RScan scenes using the json file')
parser.add_argument('--data_path', type=str, default='/DATA/3RScan/scans/',
                    help='3RScan scans folder')
parser.add_argument('--json_path', type=str, default='/DATA/3RScan/scans/3RScan.json',
                    help='')
parser.add_argument('--out_path', type=str, default='/SOURCES/Semantic-Graph-based--global-Localization/Dataset/',
                    help='Where to put the datset scenes')
parser.add_argument('--scene_prefix', type=str, default='rscan',
                    help='Prefix for the scene folder names')
parser.add_argument('--start', type=int, default=0,
                    help='start scene index')
parser.add_argument('--count', type=int, default=None,
                    help='number of scenes to do')
parser.add_argument('--verbose', action='store_true',
                    help='show debug information')

args = parser.parse_args()
data_path, json_path, out_path, prefix, start, iter, verbose = (
    getattr(args, arg) for arg in vars(args))

links = ('label', 'depth', 'instance', 'labels.instances.annotated.v2.ply')
info_file = os.path.join('sequence','_info.txt')

with open(json_path, 'r') as f:
    json_dict = json.load(f)

if iter is None or iter > len(json_dict): iter = len(json_dict)

instances = 0
scenes = 0
for i,dict_i in enumerate(json_dict[start:start+iter], start=start):
    srcs = []
    dsts = []
    src = os.path.join(data_path,dict_i['reference'])
    #if os.path.exists(src): #faster
    if (os.path.exists(os.path.join(src,links[0]))
    and os.path.exists(os.path.join(src,links[1]))):
        dst = os.path.join(out_path,'{}{:04d}_00'.format(prefix,i))
        #if verbose:
        #    print('{}{:04d}_00'.format(prefix, i), dict_i['reference'])
        srcs.append(src)
        dsts.append(dst)
    for j,dict_j in enumerate(dict_i['scans'], start=len(srcs)):
        src = os.path.join(data_path,dict_j['reference'])
        #if os.path.exists(src): #faster
        if (os.path.exists(os.path.join(src,links[0]))
        and os.path.exists(os.path.join(src,links[1]))):
            dst = os.path.join(out_path,'{}{:04d}_{:02d}'.format(prefix,i,j))
            #if verbose:
            #    print('{}{:04d}_{:02d}'.format(prefix,i,j), dict_j['reference'])
            srcs.append(src)
            dsts.append(dst)
    if verbose:
        print('{}{:04d}'.format(prefix,i), '{} out of {}'.format(len(srcs),1+len(dict_i['scans'])))
    #elif len(srcs) > 1:
    #    print('{}{:04d}'.format(prefix,i))
    if len(srcs) > 1:
        scenes += 1
        for src, dst in zip(srcs, dsts):
            instances += 1
            if verbose:
                print('',src)
                print('->',dst)
            if not os.path.exists(dst):
                os.makedirs(dst)
            dst_info = os.path.join(dst, os.path.basename(src)+'_info.txt')
            if os.path.lexists(dst_info):
                os.remove(dst_info)
            src_info = os.path.join(src, info_file)
            if os.path.exists(src_info):
                os.symlink(src_info, dst_info)
            else: #we create a empty info file
                open(dst_info, 'w').close()
            for link in links:
                link_src = os.path.join(src,link)
                if os.path.exists(link_src):
                    link_dst = os.path.join(dst,link)
                    if os.path.lexists(link_dst):
                        if os.path.islink(link_dst): #we can overwrite
                            os.remove(link_dst)
                        else:                        #we backup the existing file
                            os.rename(link_dst, link_dst+'_old')
                    os.symlink(link_src, link_dst)

print("Linked {} instances of {} scenes".format(instances, scenes))
