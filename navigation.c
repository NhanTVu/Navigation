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
#define MAX_OBSTACLES   	25		//maximum number of obstacles
#define GridWidth 			16		//size of the grid x
#define GridLength 			10		//size of the grid y
#define INTERSECTIONS 		(GridWidth+1)*(GridLength+1)-95	//how many intersections there are	
#define INFINITI 			2147483647	// highest int value

//---------------------------------------------------------------------------------------------------------

//Global
//---------------------------------------------------------------------------------------------------------

int num_obstacles = 4;				//number of obstacles
double start[2] = {1, 4};			//start location
double goal[2] = {12, 6};			//goal location

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

//array tracks the Manhattan distance of all points
double ManhattanDistArray[GridWidth+1][GridLength+1];

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

void printManhattanArray(){
	int x, y;
	printf("Manhattan distance: \n");
	for(y = 0; y <= GridLength; y++){
		///*
		//debugger print
		printf("row %d: \n", y);
		//*/
		for(x = 0; x <= GridWidth; x++){
			printf("%f\n", ManhattanDistArray[x][y]);
		}
	}
}

//---------------------------------------------------------------------------------------------------------


//More functions
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
	
	//debugger print, should be around 0.305 (METER_CONVERSION)
	//printf("%f \n", obstacleDimension[1][1]);
}

double* pathfinder(double position[2], int* path, int directions[4] ){

	//while not at goal
	while(position[0] != goal[0] || position[1] != goal[1]){
		//if not at the right x coordinate
		if(position[0] != goal[0]){
			//if to the left of goal
			if(position[0] < goal[0]){
				//if moving right is not out of bound
				if(position[0]+1 <= 16 && position[0]+1 >= 0){

				}
			//if to the right of goal
			}else if(position[0] > goal[0]){
				//if moving left is not out of bound
				if(position[0]-1 <= 16 && position[0]-1 >= 0){
					
				}	
			}
		//if not at the right y coordinate
		}else if(position[1] != goal[1]){
			//if below the goal
			if(position[1] < goal[1]){
				//if moving up is not out of bound
				if(position[1]+1 <= 10 && position[1]+1 >= 0){

				}
			//if above the goal
			}else if(position[1] > goal[1]){
				//if moving down is not out of bound
				if(position[1]-1 <= 10 && position[1]-1 >= 0){

				}
			}
		}	
	}

}

// make no-pass borders and put the coordinates in obstaclePerimeter
void GridDecomposition(){

	int i, i2, n = 0;

	//calculate the "walls" that cannot be touched
	//1st [] is for each obstacle
	//2nd [] is for x (range)
	//3rd [] is for y (range)
	for (i = 0; i < num_obstacles; i++){
		//leftmost x of the obstacle's points
		//center x coordinate - 1/2 of width of obstacle
		//rounded to the furthest left whole coordinate
		obstacleRange[i][0][0] = floor(obstacleLocation[i][0] - (obstacleDimension[i][0]/2));
		//rightmost x of the obstacle's points
		//center x coordinate + 1/2 of width of obstacle
		//rounded to the further right whole coordinate
		obstacleRange[i][0][1] = ceil(obstacleLocation[i][0] + (obstacleDimension[i][0]/2));
		//lowest y of the obstacle's points
		//center y coordinate - 1/2 of length of obstacle
		//rounded to the lowest whole coordinate
		obstacleRange[i][1][0] = floor(obstacleLocation[i][1] - (obstacleDimension[i][1]/2));
		//highest y of the obstacle's points
		//center y coordinate + 1/2 of length of obstacle
		//rounded to ther highest whole coordinate
		obstacleRange[i][1][1] = ceil(obstacleLocation[i][1] + (obstacleDimension[i][1]/2));

		// ^ note that the rounding is to push perimeter to the nearest tile
	}

	//add the lower width of the entire graph
	for (i = 0; i <= GridWidth; i++){
		obstaclePerimeter[n][0] = i;		//x coordinate
		obstaclePerimeter[n][1] = 0;		//y coordinate
		n++;	//go to next coordinate slot
	}

	//add the upper width of the entire graph
	for (i = 0; i <= GridWidth; i++){
		obstaclePerimeter[n][0] = i;			//x coordinate
		obstaclePerimeter[n][1] = GridLength;	//y coordinate
		n++;	//go to next coordinate slot	
	}

	//add the left length of the entire graph
	//width already added the corners
	// i = 1 and i < GridLength to not add corner twice
	for (i = 1; i < GridLength; i++){
		obstaclePerimeter[n][0] = 0;		//x coordinate
		obstaclePerimeter[n][1] = i;		//y coordinate
		n++;	//go to next coordinate slot
	}

	//add the right length of the entire graph
	//width already added the corners
	// i = 1 and i < GridLength to not add corner twice
	for (i = 1; i < GridLength; i++){
		obstaclePerimeter[n][0] = GridWidth;			//x coordinate
		obstaclePerimeter[n][1] = i;	//y coordinate
		n++;	//go to next coordinate slot	
	}


	/*
	//debugger print, use it to separate each for loop
	obstaclePerimeter[n][0] = -111;		//x coordinate
	obstaclePerimeter[n][1] = -111;		//y coordinate
	n++;	//go to next coordinate slot	
	*/

	//actually making the perimter/ tile intersections robot can't go near
	//for each obstacle
	for (i = 0; i < num_obstacles; i++){
		
		//add lower width of the rectangle going left to right
		for (i2 = 0; i2 <= (obstacleRange[i][0][1] - obstacleRange[i][0][0]); i2++){
			if(	obstacleRange[i][0][0]+i2 != 0 &&  
				obstacleRange[i][0][0]+i2 != GridWidth && 
				obstacleRange[i][1][0] != 0 &&  
				obstacleRange[i][1][0] != GridLength	){

					obstaclePerimeter[n][0] = obstacleRange[i][0][0]+i2;	//x coordinate
					obstaclePerimeter[n][1] = obstacleRange[i][1][0];		//y coordinate
					n++;	//go to next coordinate slot	
			}
		
		}

		/*
		//debugger print, use it to separate each for loop
		obstaclePerimeter[n][0] = -10;	//x coordinate
		obstaclePerimeter[n][1] = -10;	//y coordinate
		n++;	//go to next coordinate slot	
		*/

		//add rightmost length of the rectangle going bottom up
		//i2 is now 1 to skip the lower right corner that was already added
		for (i2 = 1; i2 <= (obstacleRange[i][1][1] - obstacleRange[i][1][0]); i2++){
			if(	obstacleRange[i][0][1] != 0 &&  
				obstacleRange[i][0][1] != GridWidth && 
				obstacleRange[i][1][0]+i2 != 0 &&  
				obstacleRange[i][1][0]+i2 != GridLength	){

					obstaclePerimeter[n][0] = obstacleRange[i][0][1];		//x coordinate
					obstaclePerimeter[n][1] = obstacleRange[i][1][0]+i2;		//y coordinate
					n++;	//go to next coordinate slot
			}
		}
		
		/*
		//debugger print, use it to separate each for loop
		obstaclePerimeter[n][0] = -10;	//x coordinate
		obstaclePerimeter[n][1] = -10;	//y coordinate
		n++;	//go to next coordinate slot	
		*/

		//add upper width of the rectangle going right to left
		//i2 has a -1 to not add the upper right corner again
		for (i2 = (obstacleRange[i][0][1] - obstacleRange[i][0][0])-1; i2 >= 0; i2--){
			if(	obstacleRange[i][0][0]+i2 != 0 &&  
				obstacleRange[i][0][0]+i2 != GridWidth && 
				obstacleRange[i][1][1] != 0 &&  
				obstacleRange[i][1][1] != GridLength	){
					obstaclePerimeter[n][0] = obstacleRange[i][0][0]+i2;	//x coordinate
					obstaclePerimeter[n][1] = obstacleRange[i][1][1];		//y coordinate
					n++;	//go to next coordinate slot
				}
		}
		
		/*
		//debugger print, use it to separate each for loop
		obstaclePerimeter[n][0] = -10;	//x coordinate
		obstaclePerimeter[n][1] = -10;	//y coordinate
		n++;	//go to next coordinate slot	
		*/

		//add leftmost length of the rectangle going top down
		//i2 has a -1 to not add the upper left corner again
		//i2 is compared with > and not >= to not add lower left corner again
		for (i2 = (obstacleRange[i][1][1] - obstacleRange[i][1][0])-1; i2 > 0; i2--){
			if(	obstacleRange[i][0][0] != 0 &&  
				obstacleRange[i][0][0] != GridWidth && 
				obstacleRange[i][1][0]+i2 != 0 &&  
				obstacleRange[i][1][0]+i2 != GridLength	){
					obstaclePerimeter[n][0] = obstacleRange[i][0][0];		//x coordinate
					obstaclePerimeter[n][1] = obstacleRange[i][1][0]+i2;	//y coordinate
					n++;	//go to next coordinate slot
			}
		}

		/*
		//debugger print, use it to separate each for loop
		obstaclePerimeter[n][0] = -10;	//x coordinate
		obstaclePerimeter[n][1] = -10;	//y coordinate
		n++;	//go to next coordinate slot	
		*/
	}

	/*
	//debugger print, use it to separate each for loop
	obstaclePerimeter[n][0] = -111;	//x coordinate
	obstaclePerimeter[n][1] = -111;		//y coordinate
	n++;	//go to next coordinate slot	
	*/
}

double ManhattanDist (double position[2]){
	
	int i;
	double x_dist, y_dist, temp, man_dist = INFINITI;

	//for each obstacle perimeter point
	for(i = 0; i < INTERSECTIONS; i++){
		//get x distance (absolute value)
		x_dist = fabs(obstaclePerimeter[i][0] - position[0]);
		//get y distance (absolute value)
		y_dist = fabs(obstaclePerimeter[i][1] - position[1]);
		//sum of x and y distance
		temp = x_dist + y_dist;
		//check to see if this obstacle is the closet manhattan dist
		if(temp < man_dist){
			man_dist = temp;
		}
	}

	return man_dist;

	/*
	//debugger print
	printf("Manhattan distance: ");
	printf("%f\n", man_dist);
	*/
}

void populateManhattanArray (){
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


// Main
//---------------------------------------------------------------------------------------------------------

int main(void){

	int i;

	GridDecomposition();
	populateManhattanArray();
	//convertToMeter();

	printf("obstacle center location: \n");
	printCoordinateArray(obstacleLocation, num_obstacles);

	printf("obstacle range: \n");
	print3DArray(obstacleRange, num_obstacles);

	printf("obstacle perimeter: \n");
	printCoordinateArray(obstaclePerimeter, INTERSECTIONS);

	printManhattanArray();

	//convertToMeter();
	printf("program ran successfully \n");
}

//---------------------------------------------------------------------------------------------------------