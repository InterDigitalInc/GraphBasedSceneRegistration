#i=${1:-"/SOURCES/Semantic-Graph-based--global-Localization/scripts"}
#o=${2:-"/SOURCES/Semantic-Graph-based--global-Localization/build"}
i=${1:-"../python"}
o=${2:-"."}
for d in $i/*.py ; do
  f=${d##*/} &&
  
  ln -f -s $d $o &&
  echo "$f done"
done
find . -xtype l -delete #remove broken
