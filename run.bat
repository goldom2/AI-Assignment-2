cd javac_out
set /p input= Input file: 
set /p solution= Solution file: 
java problem.Main ../%input% ../%solution%
cd ..
pause
