x=${1:-""} #"0"
i=${2:-"/DATA/ScanNet/scans"}
o=${3:-"/SOURCES/Semantic-Graph-based--global-Localization/Dataset"}




for d in $i/scene0${x}*/ ; do
  s="${d%"${d##*[!/]}"}" && s=${s##*/} &&
  if [[ ! -d "$o/$s/instance" ]] && [[ ! -d "$o/$s.bad" ]]; then
    if [[ -d "$i/$s" ]]; then
      mkdir -p $o/$s && #in case the scene was not linked
      echo "Extracting $s instances..." &&
      unzip -n -qq $i/$s/${s}_2d-instance.zip -d $i/$s && #'-n' ignore if exists, '-qq' make quiet
      echo "Create links to $s instance images" &&
      ln -f -s "$i/$s/instance" "$o/$s/instance" &&
      echo "done $s"
    fi
  else
    echo "$o/$s/instance exists"
  fi
done
