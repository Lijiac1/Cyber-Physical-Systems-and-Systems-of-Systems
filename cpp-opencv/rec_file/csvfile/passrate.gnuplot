set datafile separator ','


# set grid xtics


set title "DIT638-group14" offset -1,-1
set key autotitle columnhead
set xlabel "milliseconds" offset -1,1  # left,top
set ylabel "Passrate %"

set key top right
set title font 'times.ttf,14' 
set xtics font 'times.ttf,5'
set ytics font 'times.ttf,8'

set terminal pdf
set output "passrate_report.pdf"

set title "144821.rec" offset -1,-1

plot '144821.csv' u ($1-1584540000000000):5 w lines,\
     '144821_previous.csv' u ($1-1584540000000000):5 w lines

     set title "145043.rec" offset -1,-1
     plot '145043.csv' u ($1-1584540000000000):5 w lines,\
     '145043_previous.csv' u ($1-1584540000000000):5 w lines
     
     set title "145233.rec" offset -1,-1
     plot '145233.csv' u ($1-1584540000000000):5 w lines,\
     '145233_previous.csv' u ($1-1584540000000000):5 w lines
     
     set title "145641.rec" offset -1,-1
     plot '145641.csv' u ($1-1584540000000000):5 w lines,\
     '145641_previous.csv' u ($1-1584540000000000):5 w lines
     
     set title "150001.rec" offset -1,-1
     plot '150001.csv' u ($1-1584540000000000):5 w lines,\
     '150001_previous.csv' u ($1-1584540000000000):5 w lines



