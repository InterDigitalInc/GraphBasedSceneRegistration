
n=${1:-"last"}
i=${2:-"."}
o=${3:-"../runs"}
mkdir -p $o/$n &&
#python3 fusion_error.py --histograms > $o/$n/error-$n.txt &&
#python3 fusion_time.py > $o/$n/time-$n.txt &&
for d in time matches result error ground_truth ; do


    mv $i/$d $o/$n/$d &&
  mkdir -p $i/$d && #re-create the moved folder
  echo "$d done"
done
