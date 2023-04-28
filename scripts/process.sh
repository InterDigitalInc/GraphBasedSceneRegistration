dtype=${1:-"b"} #r f g A a B b C c
task=${2:-"n"}
#s=${3:-"0"} #uncomment for ChangeSim
s=${3:-"0005"} #uncomment for ScanNet
m=${4:-"0"}
n=${5:-"1"}
save=${6:-"1"}
rtype=${7:-"n"} #n a s t m
ctype=${8:-"m"} #g f m p
#ct=g rt=n dt=g d=10

#folder1 folder2 [dtype depth ctype task rtype saving start1 count1 start2 count2 suffix
make &&
if [ ${#s} -gt 1 ]; then
  #./sgRegistration rscan$s\_0$m           rscan$s\_0$n           $dtype 5 $ctype $task $rtype $save
  ./sgRegistration scene$s\_0$m           scene$s\_0$n           $dtype 5 $ctype $task $rtype $save
else
  #./sgRegistration Ref_Warehouse_$s\_$m   Ref_Warehouse_$s\_$n   $dtype 4 $ctype $task $rtype $save
  #./sgRegistration Query_Warehouse_$s\_$m Query_Warehouse_$s\_$n $dtype 4 $ctype $task $rtype $save
  #./sgRegistration Ref_Warehouse_$s\_$m   Query_Warehouse_$s\_$n $dtype 4 $ctype $task $rtype $save
  ./sgRegistration Ref_Warehouse_$s\_$m   Query_Warehouse_$s\_$m $dtype 10 $ctype $task $rtype $save
fi
