#i=${1:-"/SOURCES/Semantic-Graph-based--global-Localization/scripts"}
#o=${2:-"/SOURCES/Semantic-Graph-based--global-Localization/build"}
i=${1:-"../scripts"}
o=${2:-"."}
for d in $i/*.sh ; do
  f=${d##*/} &&
  chmod +x $d &&
  ln -f -s $d $o &&
  echo "$f done"
done
find . -xtype l -delete #remove broken
