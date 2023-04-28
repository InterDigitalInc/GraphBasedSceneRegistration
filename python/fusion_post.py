import numpy as np
import os.path as osp
import argparse
from os import makedirs

def ouput(rows, cols, export_folder=None, merged=False, suffix="", fmt='{:.16f}', sep='\t'):
    if len(rows):
        if export_folder is not None:
            if not osp.exists(export_folder): makedirs(export_folder)
            if merged:
                file = osp.join(export_folder, '-'.join((cols[0],cols[1],suffix))+'.txt')
            else:
                file = osp.join(export_folder, '-'.join((cols[1],suffix))+'.txt')
            with open(file, "w") as f:
                for row in rows:
                    x,y = row[0],row[1]
                    if type(x) == float:
                        print(fmt.format(x)+sep+fmt.format(y), file=f)
                    else:
                        print('{}'.format(x)+sep+fmt.format(y), file=f)
        else:
            x,y = cols[0]+'-'+suffix, cols[1]+'-'+suffix
            print(x.rjust(18)+sep+y.rjust(18))
            for row in rows:
                x,y = row[0],row[1]
                if type(x) == float:
                    print(fmt.format(x)+sep+fmt.format(y))
                else:
                    print('{:18d}'.format(x)+sep+fmt.format(y))
    else:
        print("No results to print for", suffix)

parser = argparse.ArgumentParser('Merge runtime files as depth-time curves')
parser.add_argument('-i','--run_folder', type=str, default='',
                    help='where is the run\'s data located?')
parser.add_argument('--export', action='store_true',
                    help='create individual files for LaTeX')
parser.add_argument('--no_time', action='store_false',
                    help='do not merge time curves with error curves')
parser.add_argument('--min_depth', type=int, default=1,
                    help='minimum depth')
parser.add_argument('--max_depth', type=int, default=161,
                    help='maximum depth')
parser.add_argument('--labels', type=int, default=40,
                    help='number of labels (for descriptor size)')
parser.add_argument('--no_typef', action='store_false',
                    help='do not extract F descriptors from G descriptors')
parser.add_argument('--no_hist', action='store_false',
                    help='do not output truncated histograms')
parser.add_argument('-d','--dataset', type=str, default='SN',
                    help='dataset type for histograms')

args = parser.parse_args()
run_folder, export, times, min_depth, max_depth, labels, typef, hists, dataset = (getattr(args, arg) for arg in vars(args))
export_folder = osp.join(run_folder, 'export-postprocess') if export else None

ctypes = ['g','m']#['g','m','p','f']
rtypes = ['a','n']#['a','s','m','n']
dsafes = ['G','A','AW','B','BW','C','CW']
dhists = ([range(0,22,7)]+[range(20)]*4+[range(4)]*2 if dataset=='SN'
     else [range(0,15,7)]+[range(20)]*4+[range(3)]*2)
time_folder  = osp.join(run_folder, 'export-time')
error_folder = osp.join(run_folder, 'export-error')
namest = ['NNODES', 'TIMEDE','TIMEMA','NMATCH']
namese = ['TRA', 'ROT', 'RMS', 'HIS']
merges = [20, 10, 8, 5, 4, 2, 1]
for ctype in ctypes:
    for rtype in rtypes:
        for (dsafe, dhist) in zip(dsafes, dhists):
            if export: print('-'.join((ctype, rtype, dsafe)))
            if times:
                for namee in namese[:-1]:
                    filee = osp.join(error_folder, '-'.join((namee, ctype, rtype, dsafe))+'.txt')
                    if osp.isfile(filee):
                        for namet in namest[1:3]:
                            filet = osp.join(time_folder, '-'.join((namet, ctype, rtype, dsafe))+'.txt')
                            if osp.isfile(filet):
                                with open(filee, "r") as f:
                                    rese = f.read().split('\n')[:-1] #remove last line
                                with open(filet, "r") as f:
                                    rest = f.read().split('\n')[:-1] #remove last line
                                if len(rese) == len(rest):
                                    cols = [namee, namet]
                                    rows = [[float(e.split('\t')[1]),float(t.split('\t')[1])] for e,t in zip(rese,rest)]
                                    ouput(rows, cols, export_folder, merged=True,
                                        suffix='-'.join((ctype, rtype, dsafe)))
            if hists: #only pick the relevant lines
                namee = namese[-1]
                filee = osp.join(error_folder, '-'.join((namee, ctype, rtype, dsafe))+'.txt')
                if osp.isfile(filee):
                    with open(filee, "r") as f:
                        rese = f.read().split('\n')[:-1] #remove last line
                        cols = ['d', namee]
                        rows = [[i+1,100*float(rese[i].split('\t')[1])] for i in dhist] #in theory i+1 == int(rese[i].split('\t')[1]) if i in dhist
                        ouput(rows, cols, export_folder,
                            suffix='-'.join((ctype, rtype, dsafe)))
        if typef:
            dsafe = dsafes[0]
            for namee in namese[:-1]:
                filee = osp.join(error_folder, '-'.join((namee, ctype, rtype, dsafe))+'.txt')
                if osp.isfile(filee):
                    with open(filee, "r") as f:
                        rese = f.read().split('\n')[:-1] #remove last line
                        rese = [e.split('\t') for e in rese]
                        cols = ['s', namee]
                        rows = [[int(e[0]),float(e[1])] for d,e in enumerate(rese) if d%len(merges) == 0]
                        ouput(rows, cols, export_folder,
                            suffix='-'.join((ctype, rtype, 'F')))
            if hists and export: #the files must exist
                namee = namese[-1]
                filee = osp.join(export_folder, '-'.join((namee, ctype, rtype, dsafe))+'.txt')
                if osp.isfile(filee):
                    with open(filee, "r") as f:
                        rese = f.read().split('\n')[:-1] #remove last line
                        rese = [e.split('\t') for e in rese]+[[0,0]]*(len(dhists[1])-len(rese))
                        cols = ['d', namee]
                        rows = [[d+1,float(e[1])] for d,e in enumerate(rese)]
                        ouput(rows, cols, export_folder,
                            suffix='-'.join((ctype, rtype, 'F')))
            for namet in namest:
                filet = osp.join(time_folder, '-'.join((namet, ctype, rtype, dsafe))+'.txt')
                if osp.isfile(filet):
                    with open(filet, "r") as f:
                        rest = f.read().split('\n')[:-1] #remove last line
                        rest = [t.split('\t') for t in rest]
                        cols = ['s', namet]
                        rows = [[int(t[0]),float(t[1])] for d,t in enumerate(rest) if d%len(merges) == 0]
                        ouput(rows, cols, export_folder,
                            suffix='-'.join((ctype, rtype, 'F')))
            if times and export: #the files must exist
                for namee in namese[:-1]:
                    for namet in namest[1:3]:
                        file = osp.join(export_folder, '-'.join((namee, namet, ctype, rtype, dsafe))+'.txt')
                        if osp.isfile(file):
                            with open(file, "r") as f:
                                res  = f.read().split('\n')[:-1] #remove last line
                                res  = [v.split('\t') for v in res]
                                cols = [namee, namet]
                                rows = [[float(v[0]),float(v[1])] for d,v in enumerate(res) if d%len(merges) == 0]
                                ouput(rows, cols, export_folder, merged=True,
                                    suffix='-'.join((ctype, rtype, 'F')))
