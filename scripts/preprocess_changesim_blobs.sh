x=${1:-""} #"0"
i=${2:-"/SOURCES/Semantic-Graph-based--global-Localization/Dataset"}
#folder1 folder2 [dtype depth ctype task rtype ransac saving start1 count1 start2 count2 suffix




for d in $i/Ref_Warehouse_*_${x}*/ ; do
  p=${d##*Ref_Warehouse_} && s="Warehouse_${p%%/*}" && echo "Doing $s..." &&
  for e in ${p}_* ; do
    for f in ${p}_* ; do


        echo -e "\tRef_$s vs Query_$s..." &&
        for ct in g; do #does not matter




                echo -e "\t\tct=$ct" &&
                res=$(./sgRegistration Ref_$s Query_$s 0 0 $ct B 0 1) && #export blobs
                echo Ref_$s-Query_$s $res >> cmd-blobs/$ct.txt
        done

    done
  done
done
