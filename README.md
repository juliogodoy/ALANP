# ALANP
Code for the ALAN-P multi agent navigation method
Compile with: 
make clean
make all

Then, go to the examples directory and (as an example) run:
./crowdSimulation 3 5 0 100 0.5 1

where
3 is the algorithm (ALAN-P)
5 is the sceanrio (Congested)
0 is the visualizer option (off)
100 is the number of runs
0.5 is the value of the coordination factor (gamma)
1 is the size of the Time Window (1 means considering only the last reward)
