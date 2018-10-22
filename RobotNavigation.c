#include<stdlib.h>
#include<stdio.h>
#include<math.h>

/*

!IMPORTANT!

gcc doesn't link math.h so you need to compile like this:

	gcc make_Graph.c -lm

*/

//Macro
//---------------------------------------------------------------------------------------------------------

#define METER_CONVERSION 	0.3048
#define MAX_OBSTACLES   	25				//maximum number of obstacles
#define GridWidth 			16		//size of the grid x
#define GridLength 			10		//size of the grid y
#define INFINITI 			2147483647		// highest int value

//how many intersections there are
#define INTERSECTIONS 		((GridWidth+2)*(GridLength+2))		



//---------------------------------------------------------------------------------------------------------

//Global
//---------------------------------------------------------------------------------------------------------

int num_obstacles = 4;					//number of obstacles
double start[2] = {1, 4};	//start location
double goal[2] = {12, 6};	//goal location

//obstacle locations (their center at x then y coordinate)
//units are 0.305m per unit 
double obstacleLocation[MAX_OBSTACLES][2] = 	
{
	{  3,  9},{ 10,  9},{ 6, 5.5},{  9,  2},{-10,-10},
	{-10,-10},{-10,-10},{-10,-10},{-10,-10},{-10,-10},
	{-10,-10},{-10,-10},{-10,-10},{-10,-10},{-10,-10},
	{-10,-10},{-10,-10},{-10,-10},{-10,-10},{-10,-10},
	{-10,-10},{-10,-10},{-10,-10},{-10,-10},{-10,-10}
};

//obstacle dimension (width x and length y)
//the order should match with the order of obstacles in obstacleLocation
//units are 0.305m per unit 
double obstacleDimension[MAX_OBSTACLES][2] = 
{
	{  3,  1},{  3,  1},{  1,  4},{  1,  3},{  0,  0},
	{  0,  0},{  0,  0},{  0,  0},{  0,  0},{  0,  0},
	{  0,  0},{  0,  0},{  0,  0},{  0,  0},{  0,  0},
	{  0,  0},{  0,  0},{  0,  0},{  0,  0},{  0,  0},
	{  0,  0},{  0,  0},{  0,  0},{  0,  0},{  0,  0}
};	

//make no-pass zones from the perimeter
double obstacleRange[MAX_OBSTACLES][2][2];

//these corners are a part of the grid decompoistion we don's want to pass
double obstaclePerimeter[INTERSECTIONS][2];

//array tracks the Manhattan distance to goal of all points
double ManhattanDistArray[GridWidth+1][GridLength+1];

//track movement till goal
int pathway[INTERSECTIONS+2];
int pathwayIndex = 1;

//---------------------------------------------------------------------------------------------------------

//Printing functions
//---------------------------------------------------------------------------------------------------------

void printIntArray(int* input){
	int i = 0;
	while(input[i] != '\0'){
		printf("%d \n", input[i]);
		i++;
	}
	putchar('\n');
}

void printCoordinateArray(double input[][2], int size){
	int i;
	for(i = 0; i < size; i++){
		printf("%f, %f \n", input[i][0], input[i][1]);
	}
	putchar('\n');
}

void print3DArray(double input[][2][2], int size){
	int i;
	for(i = 0; i < size; i++){
		printf("%f, %f, %f, %f  \n", 	input[i][0][0], input[i][0][1],
										input[i][1][0], input[i][1][1]);
	}
	putchar('\n');
}

void printManhattanDistArray(){
	int x, y;
	double row;

	printf("Manhattan distance: \n");
	for(y = 0; y <= GridLength; y++){
		///*
		//debugger print
		row = y;
		printf("row %f: \n", row);
		//*/
		for(x = 0; x <= GridWidth; x++){
			printf("%f\n", ManhattanDistArray[x][y]);
		}
	}
}

void printPathway(){
	int i = 0;
	printf("\nPath taken: \n");
	while(pathway[i] != -1){
		switch(pathway[i]){
			case 1:	printf("^ \n");
					break;
			case 2:	printf("-> \n");
					break;
			case 3:	printf("v \n");
					break;
			case 4:	printf("<- \n");
					break;
		}
		i++;
	}
}

//---------------------------------------------------------------------------------------------------------


//More functions
//these function are mainly for planning the route for the robot
//---------------------------------------------------------------------------------------------------------

// convert the unit (tiles) to meters
void convertToMeter(){
	
	int i;
	
	//convert obstacle center coordinate to meters
	for (i = 0; i < MAX_OBSTACLES; i++){
		obstacleLocation[i][0] *= METER_CONVERSION; 
		obstacleLocation[i][1] *= METER_CONVERSION; 
	}

	//convert obstacle length and width coordinates to meters
	for (i = 0; i < MAX_OBSTACLES; i++){
		obstacleDimension[i][0] *= METER_CONVERSION; 
		obstacleDimension[i][1] *= METER_CONVERSION; 
	}

	//convert obstacle min and max x and y to meters
	for (i = 0; i < MAX_OBSTACLES; i++){
		obstacleRange[i][0][0] *= METER_CONVERSION;
		obstacleRange[i][0][1] *= METER_CONVERSION; 
		obstacleRange[i][1][0] *= METER_CONVERSION;
		obstacleRange[i][1][1] *= METER_CONVERSION; 
	}

	//convert obstacle center coordinate to meters
	for (i = 0; i < INTERSECTIONS; i++){
		obstaclePerimeter[i][0] *= METER_CONVERSION; 
		obstaclePerimeter[i][1] *= METER_CONVERSION; 
	}

	//convert starting coordinate to meters
	start[0] *= METER_CONVERSION;
	start[1] *= METER_CONVERSION;

	//convert goal coordinate to meters
	goal[0] *= METER_CONVERSION;
	goal[1] *= METER_CONVERSION;

	//debugger print, should be around 2.745 (9*METER_CONVERSION)
	//printf("%f \n", obstacleLocation[1][1]);
}


// make no-pass borders and put the coordinates in obstaclePerimeter
void gridBorder(){

	int i, n = 0;
	double temp, i2;
	//calculate the "walls" that cannot be touched
	//1st [] is for each obstacle
	//2nd [] is for x (range)
	//3rd [] is for y (range)
	for (i = 0; i < num_obstacles; i++){
		//leftmost x of the obstacle's points
		//center x coordinate - 1/2 of width of obstacle
		//rounded to the furthest left whole coordinate
		obstacleRange[i][0][0] = ceil(obstacleLocation[i][0] - (obstacleDimension[i][0]/2));
		//rightmost x of the obstacle's points
		//center x coordinate + 1/2 of width of obstacle
		//rounded to the further right whole coordinate
		obstacleRange[i][0][1] = floor(obstacleLocation[i][0] + (obstacleDimension[i][0]/2));
		//lowest y of the obstacle's points
		//center y coordinate - 1/2 of length of obstacle
		//rounded to the lowest whole coordinate
		obstacleRange[i][1][0] = ceil(obstacleLocation[i][1] - (obstacleDimension[i][1]/2));
		//highest y of the obstacle's points
		//center y coordinate + 1/2 of length of obstacle
		//rounded to ther highest whole coordinate
		obstacleRange[i][1][1] = floor(obstacleLocation[i][1] + (obstacleDimension[i][1]/2));

		// ^ note that the rounding is to push perimeter to the nearest tile
	}

	//the next 4 for loops makes the border of the map
	//note that temp is needed to cast i to a double

	//add the lower width of the entire graph
	for (i2 = 0; i2 <= GridWidth; i2++){
		obstaclePerimeter[n][0] = i2;	//x coordinate
		obstaclePerimeter[n][1] = 0;		//y coordinate
		n++;	//go to next coordinate slot
	}

	//add the upper width of the entire graph
	for (i2 = 0; i2 <= GridWidth; i2++){
		obstaclePerimeter[n][0] = i2;			//x coordinate
		obstaclePerimeter[n][1] = GridLength;	//y coordinate
		n++;	//go to next coordinate slot	
	}

	//add the left length of the entire graph
	//width already added the corners
	// i = 1 and i < GridLength to not add corner twice
	for (i2 = 1; i2 < GridLength; i2++){
		obstaclePerimeter[n][0] = 0;		//x coordinate
		obstaclePerimeter[n][1] = i2;	//y coordinate
		n++;	//go to next coordinate slot
	}

	//add the right length of the entire graph
	//width already added the corners
	// i = 1 and i < GridLength to not add corner twice
	for (i2 = 1; i2 < GridLength; i2++){
		obstaclePerimeter[n][0] = GridWidth;	//x coordinate
		obstaclePerimeter[n][1] = i2;			//y coordinate
		n++;	//go to next coordinate slot	
	}


	//actually making the perimter/ tile intersections robot can't go near
	//for each obstacle
	for (i = 0; i < num_obstacles; i++){
		
		//add lower width of the rectangle going left to right
		for (i2 = 0; i2 <= (obstacleRange[i][0][1] - obstacleRange[i][0][0]); i2+=(1.0)){
			if(	obstacleRange[i][0][0]+i2 != 0 &&  
				obstacleRange[i][0][0]+i2 != GridWidth && 
				obstacleRange[i][1][0] != 0 &&  
				obstacleRange[i][1][0] != GridLength	){

					obstaclePerimeter[n][0] = obstacleRange[i][0][0]+i2;	//x coordinate
					obstaclePerimeter[n][1] = obstacleRange[i][1][0];		//y coordinate
					n++;	//go to next coordinate slot	
			}
		
		}

		//add rightmost length of the rectangle going bottom up
		//i2 is now 1 to skip the lower right corner that was already added
		for (i2 = 1; i2 <= (obstacleRange[i][1][1] - obstacleRange[i][1][0]); i2+=(1.0)){
			if(	obstacleRange[i][0][1] != 0 &&  
				obstacleRange[i][0][1] != GridWidth && 
				obstacleRange[i][1][0]+i2 != 0 &&  
				obstacleRange[i][1][0]+i2 != GridLength	){

					obstaclePerimeter[n][0] = obstacleRange[i][0][1];		//x coordinate
					obstaclePerimeter[n][1] = obstacleRange[i][1][0]+i2;	//y coordinate
					n++;	//go to next coordinate slot
			}
		}

		//add upper width of the rectangle going right to left
		//i2 has a -1 to not add the upper right corner again
		for (i2 = (obstacleRange[i][0][1] - obstacleRange[i][0][0])-1; i2 >= 0; i2-=(1.0)){
			if(	obstacleRange[i][0][0]+i2 != 0 &&  
				obstacleRange[i][0][0]+i2 != GridWidth && 
				obstacleRange[i][1][1] != 0 &&  
				obstacleRange[i][1][1] != GridLength	){
					obstaclePerimeter[n][0] = obstacleRange[i][0][0]+i2;	//x coordinate
					obstaclePerimeter[n][1] = obstacleRange[i][1][1];		//y coordinate
					n++;	//go to next coordinate slot
				}
		}

		//add leftmost length of the rectangle going top down
		//i2 has a -1 to not add the upper left corner again
		//i2 is compared with > and not >= to not add lower left corner again
		for (i2 = (obstacleRange[i][1][1] - obstacleRange[i][1][0])-1; i2 > 0; i2-=(1.0)){
			if(	obstacleRange[i][0][0] != 0 &&  
				obstacleRange[i][0][0] != GridWidth && 
				obstacleRange[i][1][0]+i2 != 0 &&  
				obstacleRange[i][1][0]+i2 != GridLength	){
					obstaclePerimeter[n][0] = obstacleRange[i][0][0];		//x coordinate
					obstaclePerimeter[n][1] = obstacleRange[i][1][0]+i2;	//y coordinate
					n++;	//go to next coordinate slot
			}
		}
		
	}

}

double ManhattanDist (double position[2]){
	
	int i;
	double x_dist, y_dist, temp, man_dist = INFINITI;

	for (int i = 0; i < INTERSECTIONS; i++)
	{
		//perimeter of obstacles are set to be high as to never move onto them
		if(	obstaclePerimeter[i][0] == position[0] &&
			obstaclePerimeter[i][1] == position[1] ){
			return INFINITI;
		}
	}


	//get x distance (absolute value)
	x_dist = fabs(goal[0] - (position[0]) );
	//get y distance (absolute value)
	y_dist = fabs(goal[1] - (position[1]) );
	//debugger print
	//printf("temp: %f\n", temp);

	//sum of x and y distance
	return (x_dist + y_dist);
	
	/*
	//debugger print
	printf("Manhattan distance: ");
	printf("%f\n", man_dist);
	*/
}

void populateManhattanDist(){
	int x, y;
	double temp[2];
	for(x = 0; x <= GridWidth; x++){
		for(y = 0; y <= GridLength; y++){
			temp[0] = x;
			temp[1] = y;
			ManhattanDistArray[x][y] = ManhattanDist(temp);
		}
	}
}

//decide where to go next
//1 is up, 2 right, 3 down, 4 left
int move(double position[2]){
	int 	result,
			x = position[0],
			y = position[1];
	double 	temp = INFINITI;

	//compare manhattan distance

	//if up is not out of bound
	if(y < GridLength){
		if(	ManhattanDistArray[x][y+1] < temp && 
			pathway[pathwayIndex-1] != 3){
			temp = ManhattanDistArray[x][y+1];
			result = 1;
		}	
	}

	//if down is not out of bound
	if(y > 0){
		if(ManhattanDistArray[x][y-1] < temp && 
			pathway[pathwayIndex-1] != 1){
			temp = ManhattanDistArray[x][y-1];
			result = 3;
		}
	}

	//if right is not out of bound
	if(x < GridWidth){
		if(ManhattanDistArray[x+1][y] < temp && 
			pathway[pathwayIndex-1] != 4){
			temp = ManhattanDistArray[x+1][y];
			result = 2;
		}
	}

	//if left is not out of bound
	if(x > 0){
		if(ManhattanDistArray[x-1][y] < temp && 
			pathway[pathwayIndex-1] != 2){
			temp = ManhattanDistArray[x-1][y];
			result = 4;
		}
	}

	//after moving, make sure to try not move back to previous locations
	//the previous location is set to less than INIFINITI so that
	//it takes priority over running into obstacle but
	//less priority than non-taken path
	ManhattanDistArray[x][y] = INFINITI-1;

	return result;

}

void findRoute(double position[2]){
	int 	result,
			x = position[0],
			y = position[1];
	double temp[2];
	//success condition
	//goal found
	if(x == goal[0] && y == goal[1]){
		pathway[pathwayIndex] = -1;
		
		return;
	}

	pathway[pathwayIndex] = move(position);

	//printf("move: %d\n", pathway[pathwayIndex]);

	switch(move(position)){
		case 1:	temp[0] = x;
				temp[1] = y+1.0;
				pathwayIndex++;
				findRoute(temp);
				break;
		case 2:	temp[0] = x+1.0;
				temp[1] = y;
				pathwayIndex++;
				findRoute(temp);
				break;
		case 3:	temp[0] = x;
				temp[1] = y-1.0;
				pathwayIndex++;
				findRoute(temp);
				break;
		case 4:	temp[0] = x-1.0;
				temp[1] = y;
				pathwayIndex++;
				findRoute(temp);
				break;
	}
}



//functions that have to do with actually moving the robot
//---------------------------------------------------------------------------------------------------------

void moveForward(){
	//both wheels turn forward

	//debugger print
	printf("go forward\n");
}

void moveBackward(){
	//both wheels turn forward
	
	//debugger print
	printf("go backward\n");
}

void turnLeft(){
	int i = 1;

	//right wheel turns forward

	//left wheel turns backward

	//debugger print
	printf("turn left\n");

	//reorient the pathway 
	while(pathway[i] != -1){
		switch(pathway[i]){
			case 1:	pathway[i] = 2;
					break;
			case 2: pathway[i] = 3;
					break;
			case 3:	pathway[i] = 4;
					break;
			case 4:	pathway[i] = 1;
					break;
		}

	i++;

	}
}

void turnRight(){
	int i = 1;
	//right wheel turns backward

	//left wheel turns forward

	//debugger print
	printf("turn right\n");

	//reorient the pathway 
	while(pathway[i] != -1){
		switch(pathway[i]){
			case 1:	pathway[i] = 4;
					break;
			case 2: pathway[i] = 1;
					break;
			case 3:	pathway[i] = 2;
					break;
			case 4:	pathway[i] = 3;
					break;
		}

	i++;

	}
}

void RobotMoves(int direction){
	
	switch(direction){
		case 1:	moveForward();
				break;
		case 2: turnRight();
				moveForward();	
				break;
		case 3:	moveBackward();
				break;
		case 4:	turnLeft();
				moveForward();
				break;
	}
}

void executePath(){
	int i = 1;
	//debugger print
	printf("\nStart\n");
	while(pathway[i] != -1){
		RobotMoves(pathway[i]);
		i++;
	}
	//debugger print
	printf("Finish\n");
}


//---------------------------------------------------------------------------------------------------------

// Main
//---------------------------------------------------------------------------------------------------------

int main(void){

	gridBorder();
	populateManhattanDist();
	findRoute(start);
	executePath();

	//convertToMeter();
	printf("\nprogram ran successfully \n");
}

//---------------------------------------------------------------------------------------------------------