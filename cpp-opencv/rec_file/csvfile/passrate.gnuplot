set xtics center offset 0,-1
set yrange [0:1]
set ylabel "PassRate 100%" offset 3,0
set ytics 0,50,100
# set style histogram clustered gap 1
     set style histogram rowstacked 
     set style histogram columnstacked
set style fill pattern border 2
unset xtics 
set boxwidth 15 relative

set datafile separator ','

 # set terminal pdf 
 # set output "passrate.pdf"


set xdata time
set timefmt "%Y-%m-%dT%H:%M:%S"
set datafile separator comma
set format x "%S"


set size 0.5,1
set multiplot layout 0.5,1

set origin 0,0
set size 0.5,0.5
set xlabel "previous" offset -1,0.5
plot '144821_previous.csv' using ($1-1584540000000000):5 with boxes title "passrate-previous"

set origin 0,0.5
set size 0.5,0.5
set xlabel "updated" offset -1,0.5
plot '144821.csv' using ($1-1584540000000000):5 with boxes title "passrate"
unset multiplot
set output



