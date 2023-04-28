x=${1:-""} #"0"
i=${2:-"/DATA/ScanNet/scans"}
o=${3:-"/SOURCES/Semantic-Graph-based--global-Localization/Dataset"}
c=${4:-"/SOURCES/Semantic-Graph-based--global-Localization/build/orientation_checker.py"}
# for d in $o/*/ ; do
#   mv $d/orientation.txt $d/orientation${n}.txt && echo "$d backuped"
# done
for d in $o/scene0${x}*_00 ; do
  p=${d%%_*} && s=${p##*/} &&
  if [[ ! -f "$o/${s}_00/orientation.txt" ]]; then
    echo ${s}_00 0 0 && echo 0 0 > $o/${s}_00/orientation.txt
  fi
  for f in $i/${s}_* ; do
    n=${d##*_} && m=${f##*_} &&
    if [ $m -gt $n ]; then #ignores .bad
      if [[ -d "$o/${s}_$m" ]] && [[ ! -f "$o/${s}_$m/orientation.txt" ]]; then #if there's a folder but no file
        r=$(python3 $c --data_path1 $d --data_path2 $f) &&
        echo ${s}_$m $r && echo $r > $o/${s}_$m/orientation.txt
      fi
    fi
  done
done
