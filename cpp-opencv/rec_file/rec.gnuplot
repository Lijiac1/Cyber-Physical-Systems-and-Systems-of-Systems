set datafile separator ','

set xdata time
set timefmt "%s"
set datafile separator comma
set format x "%s"
# set grid xtics


set title "DIT638-group14" offset -1,-1
set key autotitle columnhead
set xlabel "165277xxxx milliseconds" offset -1,1  # left,top
set ylabel "Streeing Value"

set key top right
set title font 'times.ttf,14' 
set xtics font 'times.ttf,5'
set ytics font 'times.ttf,8'

set terminal pdf
set output "144821.pdf"

plot '144821.csv' u ($1-1652770000000000):2 w lines,\
     '144821.csv' u ($1-1652770000000000):3 w lines

     
# using 1:2 with lines, '' using 1:3 with lines
