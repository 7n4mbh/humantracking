set style data histograms
set style histogram rows
set style fill solid 1.0    
set boxwidth 0.9 relative
plot 'statistics_pepmap7420008.csv' using 2, '' using 3, '' using 4, '' using 5, '' using 6
