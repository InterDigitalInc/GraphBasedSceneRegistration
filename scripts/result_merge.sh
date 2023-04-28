






for d in time matches result error ; do
  for t in $d/*_*.txt ; do #alphabetical
    s=${t%%_*} &&
    if [[ -f $t ]]; then #recheck: files get deleted when merging
      for e in ${s}_* ; do
        cat $e >> $s.txt && rm $e
      done &&
      echo "$s done"
    fi
  done
done
