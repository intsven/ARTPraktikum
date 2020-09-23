set xrange [-1.5:1.5]
set yrange [-1.5:1.5]
set xlabel "[m]"
set ylabel "[m]
plot "pos.dat" u 2:3 with points title "Trajektorie"
pause -1
reset
set ylabel "[m/s]"
set xlabel "time step
plot "pos.dat" u 1:5 with points title "Geschwindigkeit"
pause -1
set ylabel "[rad/s]"
set xlabel "time step
plot "pos.dat" u 1:6 with points title "Rotationsgeschwindigkeit"
pause -1
set ylabel "[m/s]"
set xlabel "time step
plot "pos.dat" u 1:7 with points title "Geschwindigkeit linker Motor", "pos.dat" u 1:8 with points title "Geschwindigkeit rechter Motor"
pause -1
