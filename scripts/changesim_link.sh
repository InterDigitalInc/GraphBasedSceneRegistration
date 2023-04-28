x=${1:-""} #"0"
i=${1:-"/DATA/ChangeSim"}
o=${2:-"/SOURCES/Semantic-Graph-based--global-Localization/Dataset"}

for d in $i/*/ ; do
  m="${d%"${d##*[!/]}"}" && m=${m##*/} && #extract dataset split folder name
  for e in $i/$m/*/ ; do
    s="${e%"${e##*[!/]}"}" && s=${s##*/} && #extract scene name
    for f in $i/$m/$s/*_${x}*/ ; do #TODO: remove '_${x}*' if issue
      n="${f%"${f##*[!/]}"}" && n=${n##*/} && #extract sequence name
      I="$i/$m/$s/$n/" &&
      O="$o/${m%%_*}_$s${n:3}" &&
      if [[ ! -d $O.bad ]]; then #ignore unusable instances
        if [[ ! -d $O ]]; then #skip slow operations for already done instances
          mkdir -p $O &&
          if [ ${m%%_*} == "Query" ]; then
            #the trajectory.txt file given with the dataset is incorrect
            # cp "$i/$m/$s/$n/trajectory.txt" "$O/trajectory_original.txt" &&
            #we reconstruct it from the files in the pose folder
            cd "$i/$m/$s/$n/pose" && ls | wc -l > "$O/trajectory.txt" &&
            ls -v | xargs grep -e '' | sed 's/.txt:/ /g' >> "$O/trajectory.txt" &&
            sed -i 's/inf/0/g' "$O/trajectory.txt"
          else #the trajectory.txt has to be created by matching depth maps
            python3 changesim_poses.py --depth "$i/" --raw_depth "$i/" --output "$o/" --name $s --seq ${n:3} #TODO: comment if issue
          fi &&
          echo "$O done"
        else
          echo "$O exists"
        fi &&
        #perform linking even if the instance was already done (fast)
        ln -f -s -n "$I/depth" "$O/depth" &&
        ln -f -s -n "$I/semantic_segmentation" "$O/label" &&
        if [ ${m%%_*} == "Query" ]; then
          ln -f -s -n "$I/change_segmentation" "$O/changes"
        fi &&
        #the models are in the Query instances (but match the Ref images)
        ln -f -s "$i/Query_${m#*_}/$s/$n/cloud_map.ply" $O
      fi
    done
  done
done
