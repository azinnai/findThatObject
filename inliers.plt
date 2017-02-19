set xlabel "Iteration"
set ylabel "% Inliers"
unset key
plot 'inliers.txt' with linespoints lt 1 lc "orange" lw 2 ps 1 pointtype 7
set term png 
set output "inliers.png"
replot