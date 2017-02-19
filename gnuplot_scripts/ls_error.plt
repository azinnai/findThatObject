set xlabel "Iteration"
set ylabel "Total error"
unset key
plot 'ls_error.txt' with linespoints lt 1 lw 2 ps 1 pointtype 7
set term png 
set output "ls_error.png"
replot