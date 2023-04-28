x=${1:-""} #"0"
i=${2:-"/DATA/ScanNet/scans"}
o=${3:-"/SOURCES/Semantic-Graph-based--global-Localization/Dataset"}




for d in $i/scene0${x}*/ ; do
  s="${d%"${d##*[!/]}"}" && s=${s##*/} &&
  if [[ ! -d "$o/$s/esalab" ]] && [[ ! -d "$o/$s.bad" ]]; then
    if [[ -d "$i/$s" ]]; then
      mkdir -p $o/$s && #in case the scene was not linked
      #echo "Generating $s ESANet labels..." &&
      #ESANet label genetration is being done on Windows...
      echo "Create links to $s ESANet label images" &&
      ln -f -s "$i/$s/esacol" "$o/$s/esalab" &&
      echo "done $s"
    fi
  else
    echo "$o/$s/esalab exists"
  fi
done
