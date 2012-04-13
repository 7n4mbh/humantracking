set style data histograms
set style histogram rows
set style fill solid 1.0 border -1    
set boxwidth 0.9 relative
set xrange[0:]
set yrange[0:10000000]
plot 'tracking.log' using 4, '' using 5, '' using 6, '' using 7
