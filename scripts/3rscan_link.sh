x=${1:-""} #"0"
i=${2:-"/DATA/3RScan/scans"}
o=${3:-"/SOURCES/Semantic-Graph-based--global-Localization/Dataset"}


python3 3rscan_link.py --data_path $i --json_path $i/3RScan.json --out_path $o &&

for d in $o/rscan*/${x}*-*-*-*-*_info.txt ; do
  r="${d%"${d##*[!/]}"}" && s=${r%/*} && s=${s##*/} && t=${r##*/} && t=${t%_*} &&
  if [[ ! -f "$o/$s/synthia.txt" ]] && [[ ! -d "$o/$s.bad" ]]; then
    if [[ -d "$i/$t/sequence" ]]; then









      echo "Rename link to $s label model" && #to match ScanNet
      mv -f $o/$s/labels.instances.annotated.v2.ply $o/$s/${s}_vh_clean_2.labels.ply &&
      echo "Creating $s synthia.txt (poses)..." &&
      cd $i/$t/sequence && ls *.pose.txt | wc -l > "$o/$s/synthia.txt" &&
      cat $(ls *.pose.txt | sort -h) >> "$o/$s/synthia.txt" &&
      sed -i 's/inf/0/g' "$o/$s/synthia.txt" &&
      echo "done $s"
    fi
  else
    echo "$o/$s exists"
  fi
done
