x=${1:-""}
i=${2:-"/SOURCES/Semantic-Graph-based--global-Localization/Dataset"}
#folder1 folder2 [dtype depth ctype task rtype ransac saving start1 count1 start2 count2 suffix


o="0" && #frame offset
c="0" && #frame count
for d in $i/Ref_Warehouse_*_${x}*/ ; do
  p=${d##*Ref_Warehouse_} && s="Warehouse_${p%%/*}" && echo "Doing $s..." &&
  for e in ${p}_* ; do
    for f in ${p}_* ; do


        echo -e "\tRef_$s vs Query_$s..." &&
        for ct in g m; do #f #g m p
          ./sgRegistration Ref_$s Query_$s 0 0 $ct t n 1 && #test GT
          for rt in n a s m; do #t #n #a s m
            for dt in A a B b; do #A a B b #dt="b" && for d in 1 2 5 40; do #for the adjacency descriptor
              for d in $(seq 1 1 19) $(seq 20 20 160); do
                echo -e "\t\tct=$ct rt=$rt dt=$dt d=$d" &&
                res=$(./sgRegistration Ref_$s Query_$s $dt $d $ct 0 $rt 1 $o $c 0 0 $x) &&
                echo Ref_$s-Query_$s $res >> time/$ct-$rt-$dt-$d\_$x.txt
              done
            done &&
            for dt in C c; do #C c
              for d in $(seq 1 1 3); do
                echo -e "\t\tct=$ct rt=$rt dt=$dt d=$d" &&
                res=$(./sgRegistration Ref_$s Query_$s $dt $d $ct 0 $rt 1 $o $c 0 0 $x) &&
                echo Ref_$s-Query_$s $res >> time/$ct-$rt-$dt-$d\_$x.txt
              done
            done &&
            dt="g" && for d in $(seq 1 1 15); do #dt="f" && for d in 2 3; do #for the histogram descriptor
                echo -e "\t\tct=$ct rt=$rt dt=$dt d=$d" &&
                res=$(./sgRegistration Ref_$s Query_$s $dt $d $ct 0 $rt 1 $o $c 0 0 $x) &&
                echo Ref_$s-Query_$s $res >> time/$ct-$rt-$dt-$d\_$x.txt
            done # &&
            # t="r" &&
            # for d in $(seq 4 1 4); do
            #   if [[ ! -f "time/${s}_$n-${s}_$m-$t-$d.txt" ]]; then
            #     res=$(./sgRegistration ${s}_$n ${s}_$m $t $d 0 $r 1 $o $c 0 0 $x) &&
            #     echo ${s}_$n-${s}_$m $res >> time/$t-$d\_$x.txt
            #   fi
            # done
          done
        done

    done
  done
done
