set style data histograms
set style histogram rows
set style fill solid 1.0    
set boxwidth 0.9 relative
plot 'statistics_pepmap7420008_extended.csv' using 2, '' using 3, '' using 4, '' using 5, '' using 6
