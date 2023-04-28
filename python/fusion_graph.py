import numpy as np
import os.path as osp
import argparse
from os import makedirs

def ouput(vals, cols, used, export_folder=None, suffix="", fmt='{:.16f}', sep='\t'):
    if len(vals):
        if export_folder is not None:
            # if not osp.exists(export_folder): makedirs(export_folder)
            # for col, val, use in zip(cols, vals, used):
            #     if use:
            #         with open(osp.join(export_folder, col+'-'+suffix+'.txt'), "w") as f:
            #             if type(val) is float :
            #                 print( fmt.format(val), file=f)
            #             else:
            #                 print('{}'.format(val), file=f)
            with open(osp.join(export_folder+'-'+suffix+'.txt'), "w") as f:
                col_len = max([len(col) for col in cols])
                for col, val in zip(cols, vals):
                    if type(val) is float :
                        print(col.ljust(col_len)+sep+ fmt.format(val), file=f)
                    else:
                        print(col.ljust(col_len)+sep+'{}'.format(val), file=f)
        else:
            for col in cols:
                print((col+'-'+suffix).rjust(18), end=sep)
            print()
            for val in vals:
                if type(val) is float :
                    print(     fmt.format(val), end=sep)
                else:
                    print('{:18d}'.format(val), end=sep)
            print()
    else:
        print("No results to print for", suffix)

parser = argparse.ArgumentParser('Average results from graph extraction files')
parser.add_argument('-i','--run_folder', type=str, default='',
                    help='where is the run\'s data located?')
parser.add_argument('--export', action='store_true',
                    help='create individual files for LaTeX')
parser.add_argument('-d','--dataset', type=str, default='SN',
                    help='dataset type for histograms')

args = parser.parse_args()
run_folder, export, dataset = (getattr(args, arg) for arg in vars(args))
export_folder = osp.join(run_folder, 'export-graph') if export else None

ctypes = ['g','f']
if dataset == 'SN':
    split = 5
    colsi = ['NFRA1','ARI1','TIMEGR1','NSNOD1','NGNOD1',
             'NFRA2','ARI2','TIMEGR2','NSNOD2','NGNOD2']
    indsi = [      8,    10,       12,      16,      18,
                  20,    22,       24,      28,      30] #len 31
    typei = [    int, float,    float,     int,     int,
                 int, float,    float,     int,     int]
    cols = [ 'NGRAPH', 'NFRAMES', 'NGNODES', 'NSNODES', 'TOTTIME',
            'FRPERGR', 'FRPERGN', 'FRPERSN', 'FRPERTM',
            'GNPERGR', 'GNPERFR', 'GNPERSN', 'GNPERTM',
            'SNPERGR', 'SNPERFR', 'SNPERGN', 'SNPERTM',
            'TMPERGR', 'TMPERFR', 'TMPERGN', 'TMPERSN',
            'ARI', 'ARIWFR', 'ARIWGN', 'ARIWSN', 'ARIWTM']
    used = [True, True,  True,  True,  True,
            True, False, False, False, False,
            True, False, True,  False,
            True, False, False, False,
            True, False, False, False,
            True, False, False, False, False]
else:
    split = 4
    colsi = ['NFRA1','TIMEGR1','NSNOD1','NGNOD1',
             'NFRA2','TIMEGR2','NSNOD2','NGNOD2']
    indsi = [      8,       12,      16,      18,
                  20,       24,      28,      30] #len 31
    typei = [    int,    float,     int,     int,
                 int,    float,     int,     int]
    cols = [ 'NGRAPH', 'NFRAMES', 'NGNODES', 'NSNODES', 'TOTTIME',
            'FRPERGR', 'FRPERGN', 'FRPERSN', 'FRPERTM',
            'GNPERGR', 'GNPERFR', 'GNPERSN', 'GNPERTM',
            'SNPERGR', 'SNPERFR', 'SNPERGN', 'SNPERTM',
            'TMPERGR', 'TMPERFR', 'TMPERGN', 'TMPERSN']
    used = [True, True,  True,  True,  True,
            True, False, False, False, False,
            True, False, True,  False,
            True, False, False, False,
            True, False, False, False]
for ctype in ctypes:
    if export: print(ctype)
    file = osp.join(run_folder, ctype+'.txt') #there are only 2 useful files in practice
    if osp.isfile(file):
        #times = {}
        vals = []
        with open(file, "r") as f:
            resi = f.read().split('\n')[:-1] #remove last line
            resi = [[type(l[ind])
                    for type,ind in zip(typei[:split],indsi[:split])]
                    for i in resi if len(l:=i.split()) > indsi[-1]] + [
                    [type(l[ind if len(l)>indsi[-1] else ind-3])
                    for type,ind in zip(typei[split:],indsi[split:])]
                    for i in resi if len(l:=i.split()) > indsi[-1]-3]
            if dataset == 'SN':
                ress = [[1, l[0], l[4], l[3], l[2]] for l in resi]
                resm = [[l[0], l[0]/l[4], l[0]/l[3], l[0]/l[2],
                         l[4], l[4]/l[0], l[4]/l[3], l[4]/l[2],
                         l[3], l[3]/l[0], l[3]/l[4], l[3]/l[2],
                         l[2], l[2]/l[0], l[2]/l[4], l[2]/l[3]] for l in resi]
                resw = [[l[1], l[0]*l[1], l[4]*l[1], l[3]*l[1], l[2]*l[1]] for l in resi]
                vals = (
                    s:=np.sum( np.array(ress), axis=0)).astype(int).tolist()+(
                       np.mean(np.array(resm), axis=0)).tolist()+(
                       np.sum( np.array(resw), axis=0)/s).tolist()
            else:
                ress = [[1, l[0], l[3], l[2], l[1]] for l in resi]
                resm = [[l[0], l[0]/l[3], l[0]/l[2], l[0]/l[1],
                         l[3], l[3]/l[0], l[3]/l[2], l[3]/l[1],
                         l[2], l[2]/l[0], l[2]/l[3], l[2]/l[1],
                         l[1], l[1]/l[0], l[1]/l[3], l[2]/l[2]] for l in resi]
                vals = (np.sum( np.array(ress), axis=0)).astype(int).tolist()+(
                        np.mean(np.array(resm), axis=0)).tolist()
            ouput(vals, cols, used, export_folder, suffix=ctype)
