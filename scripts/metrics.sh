DS=${1:-"SN"}
# DT1=${3:-"2023-02-13"}
# DT2=${3:-"2023-03-20"}
#DT1=${3:-"2023-02-16"}
#DT2=${3:-"2023-04-22"}
DT1=${3:-"2023-02-15"}
DT2=${3:-"2023-04-25"}

# DS=${1:-"CS"}
# DT1=${3:-"2023-03-23"}
# DT2=${3:-"2023-03-31"}

# GT=${2:-"2DGT"}
#GT=${2:-"3DGT"}
GT=${2:-"2DGT-ESANet"}

name1=${DT1}-cmd-snodes-${DS}-${GT}
name2=${DT2}-${DS}-${GT}

echo "${DS}-${GT} graph..." &&
python3 fusion_graph.py  -i ../runs/${name1} -d ${DS} > ../runs/${name1}/graph-${DT1}-${DS}-${GT}.txt &&
python3 fusion_graph.py  -i ../runs/${name1} -d ${DS} --export &&
echo "${DS}-${GT} error..." &&
python3 fusion_error.py  -i ../runs/${name2} -d ${DS} --no_histograms > ../runs/${name2}/error-${name2}.txt &&
python3 fusion_error.py  -i ../runs/${name2} -d ${DS} --no_histograms --export &&
echo "${DS}-${GT} time..." &&
python3 fusion_time.py   -i ../runs/${name2}  > ../runs/${name2}/time-${name2}.txt &&
python3 fusion_time.py   -i ../runs/${name2} --export &&
echo "${DS}-${GT} post..." &&
python3 fusion_post.py   -i ../runs/${name2} -d ${DS} --no_hist > ../runs/${name2}/post-${name2}.txt &&
python3 fusion_post.py   -i ../runs/${name2} -d ${DS} --no_hist --export &&
echo "${DS}-${GT} success..." &&
python3 fusion_success.py -i ../runs/${name2} -d ${DS} > ../runs/${name2}/success-${name2}.txt &&
python3 fusion_success.py -i ../runs/${name2} -d ${DS} --export &&
echo "${DS}-${GT} done"
