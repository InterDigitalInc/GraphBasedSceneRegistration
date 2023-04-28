x=${1:-""} #"0"
i=${2:-"/DATA/ScanNet/scans"}
r=${3:-"/SOURCES/ScanNet/SensReader/python/reader.py"}




for d in $i/scene0${x}*/ ; do
  s="${d%"${d##*[!/]}"}" && s=${s##*/} &&
  if [[ ! -d "$o/$s" ]] && [[ ! -d "$o/$s.bad" ]]; then
    if [[ -d "$i/$s/intrinsic" ]]; then
      echo "Extracting $s..." &&
      python3 $r \
        --filename $i/$s/$s.sens \
        --output_path $i/$s/ \
        --export_depth_images \
        --export_poses --export_intrinsics &&# --export_color_images is necessary on Windows for ESANet


        
      echo "done $s"
    else
      echo "$s already extracted"
    fi
  fi
done
