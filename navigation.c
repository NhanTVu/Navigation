#include<stdlib.h>
#include<stdio.h>
#include<math.h>

/*

!IMPORTANT!

gcc doesn't link math.h so you need to compile like this:

	gcc make_Graph.c -lm

*/

#define METER_CONVERSION 	0.3048
#define MAX_OBSTACLES   	25		//maximum number of obstacles
#define GridWidth 			16		//size of the grid x
#define GridLength 			10		//size of the grid y
#define INTERSECTIONS 	(GridWidth+1)*(GridLength+1)-120	//how many intersections there are	

int num_obstacles = 4;				//number of obstacles

/*
double obstacle[MAX_OBSTACLES][2] = 	//obstacle locations
{{0.61, 2.743},{0.915, 2.743},{1.219, 2.743},{1.829, 1.219},
{1.829, 1.524},{ 1.829, 1.829}, {1.829, 2.134},{2.743, 0.305},
{2.743, 0.61},{2.743, 0.915},{2.743, 2.743},{3.048, 2.743},
{3.353, 2.743},
{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
{-1-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}}; 
*/

double start[2] = {1, 4};					//start location
double goal[2] = {12, 6};					//goal location

//obstacle locations (their center at x then y coordinate)
//units are 0.305m per unit 
double obstacleLocation[MAX_OBSTACLES][2] = 	
{
	{  3,  9},{ 10,  9},{ 6, 5.5},{  8,  2},{-10,-10},
	{-10,-10},{-10,-10},{-10,-10},{-10,-10},{-10,-10},
	{-10,-10},{-10,-10},{-10,-10},{-10,-10},{-10,-10},
	{-10,-10},{-10,-10},{-10,-10},{-10,-10},{-10,-10},
	{-10,-10},{-10,-10},{-10,-10},{-10,-10},{-10,-10}
};

/*
double start[2] = {0.305, 1.219};          	//start location
double goal[2] = {3.658, 1.829};			//goal location
*/  

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


// convert the unit (tiles) to meters
void convertToMeter(){
	
	int i;
	
	for (i = 0; i < MAX_OBSTACLES; i++){
		obstacleLocation[i][0] *= METER_CONVERSION; 
		obstacleLocation[i][1] *= METER_CONVERSION; 
	}

	for (i = 0; i < MAX_OBSTACLES; i++){
		obstacleDimension[i][0] *= METER_CONVERSION; 
		obstacleDimension[i][1] *= METER_CONVERSION; 
	}

	start[0] *= METER_CONVERSION;
	start[1] *= METER_CONVERSION;
	goal[0] *= METER_CONVERSION;
	goal[1] *= METER_CONVERSION;

	//debugger print, should be around 2.745 (9*METER_CONVERSION)
	//printf("%f \n", obstacleLocation[1][1]);
	
	//debugger print, should be around 0.305 (METER_CONVERSION)
	//printf("%f \n", obstacleDimension[1][1]);
}

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

//make no-pass zones from the perimeter
double obstacleRange[MAX_OBSTACLES][2][2];

//these corners are a part of the grid decompoistion we don's want to pass
double obstaclePerimeter[INTERSECTIONS][2];


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

	//actually making the perimter/ tile intersections robot can't go near
	//for each obstacle
	for (i = 0; i < num_obstacles; i++){
		
		//add lower width of the rectangle going left to right
		for (i2 = 0; i2 <= (obstacleRange[i][0][1] - obstacleRange[i][0][0]); i2++){
			obstaclePerimeter[n][0] = obstacleRange[i][0][0]+i2;	//x coordinate
			obstaclePerimeter[n][1] = obstacleRange[i][1][0];		//y coordinate
			n++;	//go to next coordinate slot			
		}

		/*
		//debugger print, use it to separate each for loop
		obstaclePerimeter[n][0] = 3.141;	//x coordinate
		obstaclePerimeter[n][1] = 3.141;		//y coordinate
		n++;	//go to next coordinate slot	
		*/

		//add rightmost length of the rectangle going bottom up
		//i2 is now 1 to skip the lower right corner that was already added
		for (i2 = 1; i2 <= (obstacleRange[i][1][1] - obstacleRange[i][1][0]); i2++){
			obstaclePerimeter[n][0] = obstacleRange[i][0][1];		//x coordinate
			obstaclePerimeter[n][1] = obstacleRange[i][1][0]+i2;		//y coordinate
			n++;	//go to next coordinate slot
		}
		
		/*
		//debugger print, use it to separate each for loop
		obstaclePerimeter[n][0] = 3.141;	//x coordinate
		obstaclePerimeter[n][1] = 3.141;		//y coordinate
		n++;	//go to next coordinate slot	
		*/

		//add upper width of the rectangle going right to left
		//i2 has a -1 to not add the upper right corner again
		for (i2 = (obstacleRange[i][0][1] - obstacleRange[i][0][0])-1; i2 >= 0; i2--){
			obstaclePerimeter[n][0] = obstacleRange[i][0][0]+i2;	//x coordinate
			obstaclePerimeter[n][1] = obstacleRange[i][1][1];		//y coordinate
			n++;	//go to next coordinate slot
		}
		
		/*
		//debugger print, use it to separate each for loop
		obstaclePerimeter[n][0] = 3.141;	//x coordinate
		obstaclePerimeter[n][1] = 3.141;		//y coordinate
		n++;	//go to next coordinate slot	
		*/

		//add leftmost length of the rectangle going top down
		//i2 has a -1 to not add the upper left corner again
		//i2 is compared with > and not >= to not add lower left corner again
		for (i2 = (obstacleRange[i][1][1] - obstacleRange[i][1][0])-1; i2 > 0; i2--){
			obstaclePerimeter[n][0] = obstacleRange[i][0][0];		//x coordinate
			obstaclePerimeter[n][1] = obstacleRange[i][1][0]+i2;		//y coordinate
			n++;	//go to next coordinate slot
		}
		/*
		//debugger print, use it to separate each for loop
		obstaclePerimeter[n][0] = 3.141;	//x coordinate
		obstaclePerimeter[n][1] = 3.141;		//y coordinate
		n++;	//go to next coordinate slot	
		*/
	}

	//for(){

	//}

}

int main(void){

	int i;


	printf("obstacle center location: \n");
	printCoordinateArray(obstacleLocation, num_obstacles);
	
	GridDecomposition();

	printf("obstacle range: \n");
	print3DArray(obstacleRange, num_obstacles);

	printf("obstacle perimeter: \n");
	printCoordinateArray(obstaclePerimeter, INTERSECTIONS);

	//convertToMeter();
	printf("program ran successfully \n");
}