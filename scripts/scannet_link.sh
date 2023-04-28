x=${1:-""} #"0"
i=${2:-"/DATA/ScanNet/scans"}
o=${3:-"/SOURCES/Semantic-Graph-based--global-Localization/Dataset"}




for d in $i/scene0${x}*/ ; do
  s="${d%"${d##*[!/]}"}" && s=${s##*/} &&
  if [[ ! -d "$o/$s" ]] && [[ ! -d "$o/$s.bad" ]]; then
    if [[ -d "$i/$s/intrinsic" ]]; then
      mkdir -p $o/$s &&
      echo "Extracting $s labels..." &&
      unzip -n -qq $i/$s/${s}_2d-label.zip -d $i/$s && #'-n' ignore if exists, '-qq' make quiet
      echo "Create links to $s label images" &&
      ln -f -s "$i/$s/label" "$o/$s/label" &&
      echo "Create links to $s depth images" &&
      ln -f -s "$i/$s/depth" "$o/$s/depth" &&
      echo "Create link to $s label model" && #for preview and orientation
      ln -f -s $i/$s/${s}_vh_clean_2.labels.ply $o/$s/ &&
      echo "Create link to $s properties file" && #useful?
      ln -f -s $i/$s/$s.txt $o/$s/ &&
      echo "Creating $s synthia.txt (poses)..." &&
      cd $i/$s/pose && ls | wc -l > "$o/$s/synthia.txt" &&
      cat $(ls | sort -h) >> "$o/$s/synthia.txt" &&
      sed -i 's/inf/0/g' "$o/$s/synthia.txt" &&
      echo "done $s"
    fi
  else
    echo "$o/$s exists"
  fi
done
