set xlabel "Iteration"
set ylabel "Normalized error"
unset key
plot 'ls_error_norm.txt' with linespoints lt 1 lc "green" lw 2 ps 1 pointtype 7
set term png 
set output "ls_error_norm.png"
replot