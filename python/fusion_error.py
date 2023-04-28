import numpy as np
import os.path as osp
import argparse
from os import makedirs

def ouput(rows, cols, dsizes, depths, cancels, export_folder=None, suffix="",
    fmt='{:.16f}', sep='\t'):
    if len(rows):
        if export_folder is not None:
            if not osp.exists(export_folder): makedirs(export_folder)
            fs = [open(osp.join(export_folder, col+'-'+suffix+'.txt'), "w")
                for col in cols]
            for s, d, row in zip(dsizes, depths, rows):
                for f,y in zip(fs,row[:-1]):
                    print('{}'.format(s)+sep+fmt.format(y), file=f)
                print('{}'.format(d)+sep+fmt.format(row[-1]), file=fs[-1])
            for f in fs:
                f.close()
        else:
            print('\"C-R-D-d\"', end=sep)
            for col in cols:
                print(col.rjust(18), end=sep)
            print()
            for d, cancel, row in zip(depths, cancels, rows):
                print('-'.join((suffix, '{}({})'.format(d,cancel))), end=sep)
                for col in row:
                    print(fmt.format(col), end=sep)
                print()
    else:
        print("No results to output for", suffix)

parser = argparse.ArgumentParser('Merge error files as depth-error curves')
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
parser.add_argument('--no_histograms', action='store_false',
                    help='do not compute score distribution per depth')
parser.add_argument('--threshold', type=float, default=None,
                    help='optional threshold for failed registration')
parser.add_argument('-d','--dataset', type=str, default='SN',
                    help='dataset type for histograms')

args = parser.parse_args()
run_folder, export, min_depth, max_depth, labels, histograms, thresh, dataset = (getattr(args, arg) for arg in vars(args))
export_folder = osp.join(run_folder, 'export-error') if export else None

ctypes = ['g','m']#['g','m','p','f']
rtypes = ['a','n']#['a','s','m','n']
dtypes = ['g','A','a','B','b','C','c']#['g','b']
dsafes = ['G','A','AW','B','BW','C','CW']#['G','B'] #windows does not like the mix of upper and lower case
dhists = ([range(0,22,7)]+[range(20)]*4+[range(4)]*2 if dataset=='SN'
     else [range(0,15,7)]+[range(20)]*4+[range(3)]*2) if histograms else [range(1)]*len(dtypes) #[range(2), range(4)]
merges = [20, 10, 8, 5, 4, 2, 1]
cols   = ['TRA','ROT','RMS','HIS'] #['translation','rotation','RMS_dev','best_depth']
def_arr = [float('inf')]*(len(cols)-1)

for ctype in ctypes:
    for rtype in rtypes:
        for dtype, dsafe, dhist in zip(dtypes, dsafes, dhists):
            if export: print('-'.join((ctype, rtype, dtype)))
            depths = []
            dsizes = []
            files  = []
            for d in range(min_depth, max_depth):
                name = osp.join(run_folder, 'error', '-'.join((ctype, rtype, dtype, str(d)))+'.txt')
                if osp.isfile(name):
                    depths.append(d)
                    if   dtype=='f' or dtype=='F':
                        dsizes.append(labels**d)
                    elif dtype=='g' or dtype=='G':
                        dsizes.append(int(labels**(1+(d+len(merges)-2)//len(merges))/merges[(d+len(merges)-2)%len(merges)]))
                    else:
                        dsizes.append(labels*d)
                    files.append(name)
            errors  = {}
            rows    = []
            cancels = []
            for d, file in enumerate(files):
                with open(file, "r") as f:
                    res = f.read().split('\n')[:-1] #remove last line
                edepth = {i.split('\t')[0]:
                    [float(e.split()[-1]) for e in i.split('\t')[1:]] for i in res}
                errors = {i: errors.get(i, [def_arr]*d) + [edepth.get(i, def_arr)]
                    for i in set(errors) | set(edepth)}
                arr = np.array(list(edepth.values()))
                cancels.append(len(arr))
                arr = arr[~np.isnan(arr[:,2])] #filter failed registrations
                if thresh is not None: arr = arr[arr[:,2] < thresh] #10
                cancels[-1] -= len(arr) #store how many registrations failed
                #rows.append(list(np.mean(arr, axis=0)))
                #rows.append(list(1/np.mean(1/arr, axis=0)))
                rows.append(list(np.median(arr, axis=0)))
                #print(d)
            bests = {i: dhist[np.array(errors.get(i, [def_arr]))[dhist,2].argmin()]
                for i in set(errors)}
            dhist, _ = np.histogram(np.array(list(bests.values())),
                bins = range(len(depths)+1), density=True)
            rows = [row+[h] for row, h in zip(rows, dhist)]
            ouput(rows, cols, dsizes, depths, cancels, export_folder,
                suffix='-'.join((ctype, rtype, dsafe)))
