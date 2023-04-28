
n=${1:-"last"}
i=${2:-"."}
o=${3:-"../runs"}



for d in time matches result error ground_truth ; do
  rm -f $i/$d/*.txt && #empty the folders, -f forces success
  if [ ! -z "$1" ]; then
    mv $o/$n/$d $i #move the results
  fi &&
  echo "$d done"
done
