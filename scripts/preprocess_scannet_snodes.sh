x=${1:-""} #"0"
i=${2:-"/SOURCES/Semantic-Graph-based--global-Localization/Dataset"}
#folder1 folder2 [dtype depth ctype task rtype ransac saving start1 count1 start2 count2 suffix




for d in $i/scene0${x}*_00/ ; do
  p=${d%%_*} && s=${p##*/} && echo "Doing $s..." &&
  for e in ${p}_* ; do
    for f in ${p}_* ; do
      n=${e##*_} && m=${f##*_} &&
      if [ $m -gt $n ]; then
        echo -e "\t$n vs $m..." &&
        for ct in g f m p; do




                echo -e "\t\tct=$ct" &&
                res=$(./sgRegistration ${s}_$n ${s}_$m 0 0 $ct P 0 1) && #export clouds
                echo ${s}_$n-${s}_$m $res >> cmd-snodes/$ct.txt
        done
      fi
    done
  done
done
