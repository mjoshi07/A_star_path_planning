# Path Planning by Implementing A-Star Algorithm
* Searches an optimal path between a start and an end goal

## Dependencies
* Numpy
* OpenCV

## Run Instructions
* If you already have the binary world map, then rename it as final_map.png, else you can generate the map first as well
* Run the following command to start the solver
```
python Mayank-Joshi.py -h
```
* You will see the following options
```
usage: Mayank_Joshi.py [-h] [--init INIT] [--goal GOAL] [--robotRadius ROBOTRADIUS] [--clearance CLEARANCE] [--stepSize STEPSIZE] [--goalThresh GOALTHRESH]

optional arguments:
  -h, --help            	show this help message and exit
  --init INIT           	start location of the robot. Example - 10,12,0 [NOTE - without spaces], Default: Random 
  --goal GOAL           	end location of the robot. Example - 150,120,30 [NOTE - without spaces], Default: Random
  --robotRadius ROBOTRADIUS robot radius , DEFAULT: 5
  --clearance CLEARANCE     clearance dist from obstacles, DEFAULT: 5
  --stepSize STEPSIZE   	step size of movement in units, between 1 and 10, DEFAULT: 10
  --goalThresh GOALTHRESH   goal threshold distance, DEFAULT: 5

```
* --init takes input the start location of the robot: Input should be comma separated without any spaces, for example: 10,12,0
* --goal takes input the end location of the robot: Input should be comma separated without any spaces, for example: 10,12,30
* --robotRadius takes input the distance to maintain from the robot center from all of the obstacles boundaries
* --clearance takes input the min distance to maintain for the robot from all of the obstacles boundaries
* --stepSize takes input the step size for each move
* By default, if user does not specify any location for start or end, random locations would be selected
* By default, clearance is set to 5 and cannot be set more than 50
* If "final_map.png" file does not exists, then program will ask user whether to generate a new file
* The program also asks the user whether to display the progress of the map being generated using half planes techniques
* If you want to specify only the start location, modify and run the command below, end location would be generated randomly
```
python Mayank-Joshi.py --init 25,25,0
```
* If you want to specify only the end location, modify and run the command below, start location would be generated randomly
```
python Mayank-Joshi.py --goal 25,25,30
```
* If you want to specify both the start and end location, modify and run the command below
```
python Mayank-Joshi.py --init 10,100 --goal 150,50
```
* If you want to specify the robot radius for the robot, modify and run the command below
```
python Mayank-Joshi.py --robotRadius 10
```
* If you want to specify the clearance for the robot, modify and run the command below
```
python Mayank-Joshi.py --clearance 10
```
* If you want to specify the step size for the robot, modify and run the command below
```
python Mayank-Joshi.py --stepSize 10
```
* If you want to specify the goal threshold for the robot, modify and run the command below
```
python Mayank-Joshi.py --goalThresh 5
```