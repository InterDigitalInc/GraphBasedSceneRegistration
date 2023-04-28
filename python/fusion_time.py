import numpy as np
import os.path as osp
import argparse
from os import makedirs

def ouput(rows, cols, dsizes, depths, export_folder=None, suffix="",
    fmt='{:.16f}', sep='\t'):
    if len(rows):
        if export_folder is not None:
            if not osp.exists(export_folder): makedirs(export_folder)
            fs = [open(osp.join(export_folder, col+'-'+suffix+'.txt'), "w")
                for col in cols]
            for s, row in zip(dsizes, rows):
                for f,y in zip(fs,row):
                    print('{}'.format(s)+sep+fmt.format(y), file=f)

            for f in fs:
                f.close()
        else:
            print('\"C-R-D-d\"', end=sep)
            for col in cols:
                print(col.rjust(18), end=sep)
            print()
            for d, row in zip(depths, rows):
                print('-'.join((suffix, '{}'.format(d))), end=sep)
                for col in row:
                    print(fmt.format(col), end=sep)
                print()
    else:
        print("No results to print for", suffix)

parser = argparse.ArgumentParser('Merge runtime files as depth-time curves')
parser.add_argument('-i','--run_folder', type=str, default='',
                    help='where is the run\'s data located?')
parser.add_argument('--export', action='store_true',
                    help='create individual files for LaTeX')
parser.add_argument('--min_depth', type=int, default=1,
                    help='minimum depth')
parser.add_argument('--max_depth', type=int, default=161,
                    help='maximum depth')
parser.add_argument('--labels', type=int, default=40,
                    help='number of labels (for descriptor size)')
parser.add_argument('--inliers', action='store_true',
                    help='add an inliers column even without ransac')
parser.add_argument('--no_extra', action='store_false',
                    help='do not compute extra columns')
parser.add_argument('--raw', action='store_true',
                    help='output the original columns')


args = parser.parse_args()
run_folder, export, min_depth, max_depth, labels, inliers, ext, raw = (getattr(args, arg) for arg in vars(args))
export_folder = osp.join(run_folder, 'export-time') if export else None

ctypes = ['g','m']#['g','m','p','f']
rtypes = ['a','n']#['a','s','m','n']
dtypes = ['g','A','a','B','b','C','c']#['g','b']
dsafes = ['G','A','AW','B','BW','C','CW']#['G','B'] #windows does not like the mix of upper and lower case
bcols  = ['Nsnodes1','Ngnodes1','des_time1','Nsnodes2','Ngnodes2','des_time2','match_time','Nmatches']
binds  = [        13,        15,         46,        22,        24,         50,          54,        56]
btype  = [       int,       int,      float,       int,       int,      float,       float,       int]
if ext:
    colse = [            'NNODES',             'TIMEDE',   'TIMEMA',   'NMATCH']
    indse = [[binds[0], binds[3]], [binds[2], binds[5]], [binds[6]], [binds[7]]]
    typee = [      int,     float,                float,                    int]
merges = [20, 10, 8, 5, 4, 2, 1]
# def_arr = [float('inf')]*(len(cols))
for ctype in ctypes:
    for rtype in rtypes:
        if rtype == 'n':
            colsr = bcols +(['Ninliers','registration_time'] if inliers else ['registration_time'])
            indsr = binds +([        56,                 58] if inliers else [                 58])
            typer = btype +([       int,              float] if inliers else [              float])
        else:
            colsr = bcols +['Ninliers','registration_time']
            indsr = binds +[        58,                 60]
            typer = btype +[       int,              float]
        for dtype, dsafe in zip(dtypes, dsafes):
            if export: print('-'.join((ctype, rtype, dtype)))
            depths = []
            dsizes = []
            files  = []
            for d in range(min_depth, max_depth):
                name = osp.join(run_folder, 'time', '-'.join((ctype, rtype, dtype, str(d)))+'.txt')
                if osp.isfile(name):
                    depths.append(d)
                    if   dtype=='f' or dtype=='F':
                        dsizes.append(labels**d)
                    elif dtype=='g' or dtype=='G':
                        dsizes.append(int(labels**(1+(d+len(merges)-2)//len(merges))/merges[(d+len(merges)-2)%len(merges)]))
                    else:
                        dsizes.append(labels*d)
                    files.append(name)
            #times = {}
            if raw: rowsr = []
            if ext: rowse = []
            for d, file in enumerate(files):
                with open(file, "r") as f:
                    res = f.read().split('\n')[:-1] #remove last line
                #tdepth = {i.split()[0]:
                #    [type(i.split()[ind]) for type,ind in zip(typer,indsr)] for i in res if len(i.split()) > indsr[-1]} #if we do not have Python 3.8
                if raw:
                    tdepthr = {l[0]:
                        [type(l[ind]) for type,ind in zip(typer,indsr)]
                        for i in res if len(l:=i.split()) > indsr[-1]}
                    rowsr.append(list(np.mean(np.array(list(tdepthr.values())), axis=0)))
                if ext:
                    tdepthe = {l[0]: [
                        (typee[0](l[indse[0][0]])+typee[0](l[indse[0][1]]))/2,
                        #(typee[0](l[indse[0][0]])*typee[1](l[indse[1][0]])
                        #+typee[0](l[indse[0][1]])*typee[1](l[indse[1][1]]))/(
                        (typee[1](l[indse[1][0]])+typee[1](l[indse[1][1]]))/( #total descriptor time
                         typee[0](l[indse[0][0]])+typee[0](l[indse[0][1]])),  #total number of nodes
                         typee[2](l[indse[2][0]]),typee[3](l[indse[3][0]])]
                        for i in res if len(l:=i.split()) > indse[-1][-1]}
                    rowse.append(list(np.mean(np.array(list(tdepthe.values())), axis=0)))
                # tdepth = {i.split()[0]:
                #     [type(i.split()[ind]) for type,ind in zip(typer,inds)] for i in res}
                # times = {i: times.get(i, def_arr*d) + [tdepth.get(i, def_arr)] for i in set(times) | set(tdepth)}
            if raw: ouput(rowsr, colsr, dsizes, depths, export_folder,
                suffix='-'.join((ctype, rtype, dsafe)))
            if ext: ouput(rowse, colse, dsizes, depths, export_folder,
                suffix='-'.join((ctype, rtype, dsafe)))
