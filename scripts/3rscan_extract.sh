x=${1:-""} #"0"
i=${2:-"/DATA/3RScan/scans"}
r=${3:-"/SOURCES/3RScan/c++/rio_renderer/build/"}




for d in $i/${x}*-*-*-*-*/; do
  s="${d%"${d##*[!/]}"}" && s=${s##*/} &&
  if [[ ! -d "$i/$s/label" ]] && [[ ! -d "$i/$s/instance" ]] && [[ ! -d "$i/$s/depth" ]]; then
    if [[ ! -d "$i/$s/sequence.zip" ]]; then
      echo "Extracting $s sequence..." &&
      unzip -n -qq $i/$s/sequence.zip -d $i/$s/sequence #'-n' ignore if exists, '-qq' make quiet
      #&& rm -f $i/$s/sequence.zip #to save space
    else
      echo "$s sequence already extracted"
    fi &&
    mkdir -p $i/$s/label && mkdir -p $i/$s/instance && mkdir -p $i/$s/depth &&
    echo "Render $s label, instance and depth images" &&
    cd $r && ./rio_renderer_render_all $i $s 1 &&
    echo "rendered $s"
  else
    echo "$s already rendered"
  fi
done
