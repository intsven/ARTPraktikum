#!/usr/bin/gnuplot
#
# Plotting coordinate points
#
# AUTHOR: Hagen Wierstorf
# gnuplot 4.6 patchlevel 0

reset

# wxt
set terminal wxt size 900,900 enhanced font 'Verdana,10' persist
# png
set terminal pngcairo size 900,900 enhanced font 'Verdana,10'
set output 'amcl.png'

# color definitions
set border linewidth 1.5
set style line 1 lc rgb '#0060ad' pt 7 ps 0.1 lt 1 lw 2 # --- blue

unset key

# Axes
set style line 11 lc rgb '#808080' lt 1
set border 3 back ls 11
set tics nomirror out scale 0.75
# Grid
set style line 12 lc rgb'#808080' lt 0 lw 1
set grid back ls 12

set xrange [-5:10]
set yrange [-13:2]

plot 'amcl.dat' w p ls 1

