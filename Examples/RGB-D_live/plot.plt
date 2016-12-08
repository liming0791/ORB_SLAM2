set style line 1 lc rgb '#006090' lt 1 lw 2 pt 7 ps 1.5   # --- blue
set style line 2 lc rgb '#007090' lt 1 lw 2 pt 5 ps 1.5   # --- red
set style line 3 lc rgb '#007290' lt 1 lw 2 pt 5 ps 1.5   # --- red
set style line 4 lc rgb '#ed281f' lt 1 lw 2 pt 5 ps 1.5   # --- red
set style line 5 lc rgb '#f3281f' lt 1 lw 2 pt 5 ps 1.5   # --- red
set style line 6 lc rgb '#dd281f' lt 1 lw 2 pt 5 ps 1.5   # --- red
set style line 7 lc rgb '#8de800' lt 1 lw 2 pt 5 ps 1.5   # --- red
set style line 8 lc rgb '#8dd800' lt 1 lw 2 pt 5 ps 1.5   # --- red
set style line 9 lc rgb '#8df800' lt 1 lw 2 pt 5 ps 1.5   # --- red
set style line 10 lc rgb '#fdff0f' lt 1 lw 2 pt 5 ps 1.5   # --- red
set style line 11 lc rgb '#fdf83f' lt 1 lw 2 pt 5 ps 1.5   # --- red
set style line 12 lc rgb '#fdf82f' lt 1 lw 2 pt 5 ps 1.5   # --- red
plot 'TrackingLog.txt' u 0:1 with lines ls 1, \
     ''                   u 0:2 with lines ls 2, \
     ''                   u 0:3 with lines ls 3, \
     ''                   u 0:4 with lines ls 4, \
     ''                   u 0:5 with lines ls 5, \
     ''                   u 0:6 with lines ls 6, \
     ''                   u 0:7 with lines ls 7, \
     ''                   u 0:8 with lines ls 8, \
     ''                   u 0:9 with lines ls 9, \
     ''                   u 0:10 with lines ls 10, \
     ''                   u 0:11 with lines ls 11, \
     ''                   u 0:12 with lines ls 12, \
