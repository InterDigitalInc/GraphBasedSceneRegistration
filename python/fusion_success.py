import numpy as np
import os.path as osp
import argparse
from os import makedirs

def ouput(rows, cols, duplicate, export_folder=None, prefix="",
    fmt='{:.16f}', sep='\t'):
    if len(rows):
        if export_folder is not None:
            if not osp.exists(export_folder): makedirs(export_folder)
            fs = [open(osp.join(export_folder, prefix+col+'.txt'), "w")
                for col in cols[1::(2 if duplicate else 1)]]
            for row in rows:
                if duplicate:
                    for f,x,y in zip(fs,row[1::2],row[2::2]):
                        print(fmt.format(x)+sep+fmt.format(y), file=f)
                else:
                    x = row[1]
                    for f,y in zip(fs,row[2:]):
                        print(fmt.format(x)+sep+fmt.format(y), file=f)
            for f in fs:
                f.close()
        else:
            for col in cols:
                print(col.rjust(18), end=sep)
            print()
            for row in rows:
                for col in row[1:]:
                    print(fmt.format(col), end=sep)
                print()
    else:
        print("No results to output!")

def ouputCS(rows, cols, export_folder=None,
    fmt='{:.16f}', sep='\t'):
    if len(rows):
        if export_folder is not None:
            if not osp.exists(export_folder): makedirs(export_folder)
            fs = [open(osp.join(export_folder, col+'.txt'), "w")
                for col in cols]
            for i,row in enumerate(rows):
                for f,y in zip(fs, row):
                    print('{}'.format(i)+sep+fmt.format(y), file=f)
            for f in fs:
                f.close()
        else:
            print('N', end=sep)
            for col in cols:
                print(col.rjust(18), end=sep)
            print()
            for i,row in enumerate(rows):
                print('{}'.format(i), end=sep)
                for col in row:
                    print(fmt.format(col), end=sep)
                print()
    else:
        print("No results to output!")

parser = argparse.ArgumentParser('Merge error files into success rate')
parser.add_argument('-i','--run_folder', type=str, default='',
                    help='where is the run\'s data located?')
parser.add_argument('--export', action='store_true',
                    help='create individual files for LaTeX')
parser.add_argument('--min_depth', type=int, default=1,
                    help='minimum depth')
parser.add_argument('--max_depth', type=int, default=161,
                    help='maximum depth')
parser.add_argument('--column', type=int, default=None,
                    help='which error to create the curve?')
parser.add_argument('--no_cdf', action='store_false',
                    help='do not compute cdf')
parser.add_argument('--pdf', action='store_true',
                    help='compute pdf')
parser.add_argument('--no_duplicates', action='store_false',
                    help='only print the error column once')
parser.add_argument('-d','--dataset', type=str, default='SN',
                    help='dataset type for histograms')

args = parser.parse_args()
run_folder, export, min_depth, max_depth, column, cdf, pdf, duplicate, dataset = (getattr(args, arg) for arg in vars(args))
export_folder = osp.join(run_folder, 'export-success') if export else None

ctypes = ['g','m']#['g','m','p','f']
rtypes = ['a','n']#['a','s','m','n']
dtypes = ['g','A','a','B','b','C','c']#['g','b']#
dsafes = ['G','A','AW','B','BW','C','CW']#['G','B'] #windows does not like the mix of upper and lower case
dhists = ([range(0,22,7)]+[[0,1,2,9,39,159]]*4+[range(4)]*2 if dataset=='SN'
     else [range(0,15,7)]+[[0,1,2,9,39,159]]*4+[range(3)]*2)
errs   = ['TRA','ROT','RMS']
max_err = 3 #x-axis maximum value
num_points = 901 #x-axis resolution
errns = errs if column is None else [errs[column]]
columns = range(len(errs)) if column is None else [column]
if dataset == 'SN': #output pdf/cdf for scannet
    for errn,column in zip(errns,columns): #not optimal, many file reads
        if cdf:
            colsc = []
            rowsc = [[e] for e in range(num_points)]
        if pdf:
            dif_points = 10
            colsp = []
            rowsp = [[e] for e in range(0,num_points-dif_points,dif_points)]
        for ctype in ctypes:
            for rtype in rtypes:
                for dtype, dsafe, dhist in zip(dtypes, dsafes, dhists):
                    if export: print('-'.join((errn, ctype, rtype, dtype)))
                    depths = []
                    files = []
                    for d in dhist:
                        name = osp.join(run_folder, 'error', '-'.join((ctype, rtype, dtype, str(d+1)))+'.txt')
                        if osp.isfile(name):
                            depths.append(d+1)
                            files.append(name)
                    for d, file in enumerate(files):
                        name = '-'.join((ctype, rtype, dsafe, '{}'.format(depths[d])))
                        with open(file, "r") as f:
                            res = f.read().split('\n')[:-1] #remove last line
                        edepth = {i.split('\t')[0]:
                            [float(e.split()[-1]) for e in i.split('\t')[1:]] for i in res}
                        #max_err = np.max(np.array(list(edepth.values()))[:,column]) #autoscaling?
                        if cdf:
                            if duplicate:
                                colsc.append(errn+'-'+name)
                            elif len(colsc) == 0:
                                colsc.append(errn)
                            colsc.append('CDF'+'-'+name)
                            for row in rowsc:
                                err = (max_err*row[0])/(len(rowsc)-1)
                                cnt = (100*np.sum(np.array(list(edepth.values()))[:,column] <= err))/len(edepth)
                                if duplicate or len(colsc) == 2:
                                    row.append(err)
                                row.append(cnt)
                        if pdf:
                            if duplicate:
                                colsc.append(errn+'-'+name)
                            elif len(colsc) == 0:
                                colsc.append(errn)
                            colsp.append('PDF'+'-'+name)
                            for row,row1,row2 in zip(rowsp,rowsc[:-dif_points:dif_points],rowsc[dif_points::dif_points]):
                                derr = (max_err*(row1[0]-row2[0]))/(len(rowsc)-1)
                                dcnt = row1[-1]-row2[-1]
                                if duplicate or len(colsp) == 2:
                                    err = (max_err*row1[0])/(len(rowsc)-1)
                                    row.append(err)
                                row.append(dcnt/derr)
                            #for row in rowsc[-dif_points:]: #padding?
                            #    row.append(0)
                            #    row.append(0)
        if cdf: ouput(rowsc, colsc, duplicate, export_folder, prefix=errn)
        if pdf: ouput(rowsp, colsp, duplicate, export_folder, prefix=errn)
else: #output whole dataset
    for ctype in ctypes:
        for rtype in rtypes:
            for dtype, dsafe, dhist in zip(dtypes, dsafes, dhists):
                if export: print('-'.join((ctype, rtype, dtype)))
                depths = []
                files = []
                for d in dhist:
                    name = osp.join(run_folder, 'error', '-'.join((ctype, rtype, dtype, str(d+1)))+'.txt')
                    if osp.isfile(name):
                        depths.append(d+1)
                        files.append(name)
                for d, file in enumerate(files):
                    name = '-'.join((ctype, rtype, dsafe, '{}'.format(depths[d])))
                    with open(file, "r") as f:
                        res = f.read().split('\n')[:-1] #remove last line
                    # edepth = {i.split('\t')[0]:
                    #     [float(e.split()[-1]) for e in i.split('\t')[1:]] for i in res}
                    cols = [err+'-'+name for err in errs]
                    rows = [[float(e.split()[-1]) for e in i.split('\t')[1:]] for i in res]
                    ouputCS(rows, cols, export_folder)
