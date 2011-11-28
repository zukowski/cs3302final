package csci3302lab5;

/**
 * The MapCell class gives functionality to the cells used in the RatMap class. 
 * 
 * @author Rowan Wing 9/21/11
 *
 */
public class MapCell {	
	/**
	 * holds a value of either 1 or 0.  A 1 signifies the presence of a wall
	 * and a 0 means there is no wall.
	 */
	private int eastWall;
	/**
	 * holds a value of either 1 or 0.  A 1 signifies the presence of a wall
	 * and a 0 means there is no wall.
	 */
	private int northWall;
	/**
	 * holds a value of either 1 or 0.  A 1 signifies the presence of a wall
	 * and a 0 means there is no wall.
	 */
	private int westWall;
	/**
	 * holds a value of either 1 or 0.  A 1 signifies the presence of a wall
	 * and a 0 means there is no wall.
	 */
	private int southWall;
	/**
	 * holds a reference to the cell neighboring on the east.
	 */
	private MapCell eastCell;  
	/**
	 * holds a reference to the cell neighboring on the north.
	 */
	private MapCell northCell;
	/**
	 * holds a reference to the cell neighboring on the west.
	 */
	private MapCell westCell;
	/**
	 * holds a reference to the cell neighboring on the south.
	 */
	private MapCell southCell;
	/**
	 * a 1x2 array that holds the position of the cell in the format 
	 * [row][column]
	 */
	private int[] position = new int[2];
	/**
	 * the weight associated with each cell.  This value is initially set
	 * to 1000 as an arbitrarily high number, but will later be set according
	 * to the path-planning algorithm used.
	 */
	private int weight;
	
	/**
	 * Constructor for a MapCell with the row and column position on the map
	 * @param row the row value of the cell created
	 * @param column the column value of the cell created
	 */
	public MapCell(int row, int column){
		setPosition(row, column);
	}
	
	/**
	 * Constructor for a Map with the position and walls set
	 * @param row the row value of the cell created
	 * @param column the column value of the cell created
	 * @param e the boolean value of the east wall 1 = wall, 0 = no wall
	 * @param n the boolean value of the north wall 1 = wall, 0 = no wall
	 * @param w the boolean value of the west wall 1 = wall, 0 = no wall
	 * @param s the boolean value of the south wall 1 = wall, 0 = no wall
	 */
	public MapCell(int row, int column, int e, int n, int w, int s){
		setPosition(row,column);
		setWalls(e,n,w,s);
	}
	
	/**
	 * Sets the position of a MapCell
	 * @param row the row position being set referenced from the bottom left
	 * @param column the column position being set referenced from the bottom
	 * left
	 */
	public void setPosition(int row, int column){		
		position[0] = row;
		position[1] = column;		
	}	
	
	/**
	 * Sets the walls according to the values being passed in.  A 1 corresponds
	 * to a wall being created and a 0 corresponds to no wall.
	 * @param e the boolean value of the east wall 1 = wall, 0 = no wall
	 * @param n the boolean value of the north wall 1 = wall, 0 = no wall
	 * @param w the boolean value of the west wall 1 = wall, 0 = no wall
	 * @param s the boolean value of the south wall 1 = wall, 0 = no wall
	 */
	public void setWalls(int e, int n, int w, int s){
		setWall("east",e);
		setNeighborsWall("east",e);
		setWall("north",n);
		setNeighborsWall("north",n);
		setWall("west",w);
		setNeighborsWall("west",w);
		setWall("south",s);
		setNeighborsWall("south",s);
	}		
	
	/**
	 * Sets the wall identified with "east", "north", "west", or "south" with
	 * the value that is passed in 1 = wall, 0 = no wall, all others undefined.
	 * This method will also adjust the wall definitions of any neighbors
	 * affected by the change of wall status.
	 * @param wall the wall to be set use the labels "east", "north", "west", 
	 * or "south"
	 * @param value the value determines whether a wall exists or not, a 1
	 * will set a wall and a 0 will leave the side open
	 */
	public void setWall(String wall, int value){
		if(wall.equalsIgnoreCase("east") && (value == 0 || value == 1)){
			eastWall = value;
			setNeighborsWall("east", value);
		}else if(wall.equalsIgnoreCase("north") && (value == 0 || value == 1)){
			northWall = value;
			setNeighborsWall("north", value);
		}else if(wall.equalsIgnoreCase("west") && (value == 0 || value == 1)){
			westWall = value;
			setNeighborsWall("west", value);
		}else if(wall.equalsIgnoreCase("south") && (value == 0 || value == 1)){
			southWall = value;
			setNeighborsWall("south", value);
		}else {
			System.err.printf("Please specify the wall to be set using 'east', " +
					"'north', 'west', or 'south' and a value of 0 or 1 - line #%d\n",
					Thread.currentThread().getStackTrace()[2].getLineNumber());
		}	
	}
	
	/**
	 * sets the wall of the neighbor affected when a cell sets or changes its
	 * wall status.  This keeps the maze updated correctly.  If the neighboring
	 * cell is outside the maze, then this method returns without action.
	 * @param neighbor the neighboring cell to the wall being set, use
	 * "east", "north", "west", or "south" to indicate the neighbor.
	 * @param value the value corresponds to whether a wall exists or not, a
	 * 1 = wall and a 0 = no wall.
	 */
	public void setNeighborsWall(String neighbor, int value){
		try{				
			if(neighbor.equalsIgnoreCase("east") && (value == 0 || value == 1)){				
				eastCell.westWall = value;
			}else if(neighbor.equalsIgnoreCase("north") && (value == 0 || value == 1)){				
				northCell.southWall = value;
			}else if(neighbor.equalsIgnoreCase("west") && (value == 0 || value == 1)){				
				westCell.eastWall = value;
			}else if(neighbor.equalsIgnoreCase("south") && (value == 0 || value == 1)){				
				southCell.northWall = value;
			}
		}catch(NullPointerException np){
			return;
		}catch(Exception e){
			return;
		}
	}
	
	/**
	 * links the cells in the map together.  Specifically this method sets
	 * the fields "eastCell","northCell", "westCell", and "southCell" to 
	 * correspond to the neighboring cell on that side.  If the cell is along
	 * a wall then the neighbor is set to a null value.
	 */
	public void setNeighbors(){				
		int row = position[0];
		int column = position[1];		
		if(column+1 < RatMap.SIDELENGTH){
			eastCell = RatMap.getMapCell(row,column+1);			
		}else{
			eastCell = null;
		}	
		if(row+1 < RatMap.SIDELENGTH){
			northCell = RatMap.getMapCell(row+1,column);
		}else{
			northCell = null;
		}	
		if(column-1 > -1){
			westCell = RatMap.getMapCell(row, column-1);			
		}else{
			westCell = null;
		}
		if(row-1 > -1){
			southCell = RatMap.getMapCell(row-1, column);
		}else{
			southCell = null;
		}
	}
	
	/**
	 * accessor method for setting the weight of a cell.  To be used during
	 * path-planning.
	 * @param w the weight being assigned to the cell.
	 */
	public void setWeight(int w){
		weight = w;
	}
	
	/**
	 * returns a 1 by 2 int array the position of the cell in terms of row 
	 * number and column number.  The bottom left corner of the map is [0][0]
	 * and the top right is [9][9].
	 * @return a 1 by 2 int array containing the position of the cell in terms 
	 * of row number and column number.
	 */
	public int[] getMapPosition(){
		return position;
	}
	
	/**
	 * Given a tuple representing the coordinates of the E-Puck on the
	 * xz-plane, this method will estimate which cell the E-Puck is in and
	 * return the row and column number of the estimated cell.
	 * @param gpsCoord a a 1x3 array containing the x-coordinate, y-coordinate,
	 * and z-coordinate of the E-Puck.  Only the 1st and 3rd value are dealt 
	 * with since the E-Puck does not change its y-coordinate in this simulation.
	 * @return the estimated cell which corresponds to the coordinates given.
	 * The position of the cell is notated by a row and column number.
	 */
	public int[] getMapPosition(double[] gpsCoord){
		int[] mapPosition = new int[2];
		mapPosition[0] = (int) Math.round(gpsCoord[0]*9);
		mapPosition[1] = (int) Math.round(gpsCoord[2]*9);			
		return mapPosition;
	}
	
	/**
	 * Given the cell position in row/column number, this will return an
	 * estimation of the corresponding x-coordinate and z-coordinate of the 
	 * middle of the cell in the ratslife maze.
	 * @return a 1x2 array containing the x-coordinate and z-coordinate
	 * respectively of the cell calling the method.
	 */
	public double[] getCoordinates(){
		double[] coord = new double[2];
		double xcoord = (double)position[0]/9;
		double zcoord = (double)position[1]/9;
		coord[0] = xcoord;
		coord[1] = zcoord;		
		return coord;
	}
	
	/**
	 * returns the Manhatten Distance between the cell calling the method
	 * and the cell passed in as the parameter.
	 * @param goal the MapCell from which the Manhatten Distance is being
	 * calculated to the cell calling the method.
	 * @return an integer representing the Manhatten Distance.
	 */
	public int getManhattenDist(MapCell goal){
		int rowDist = Math.abs(position[0] - goal.getMapPosition()[0]);
		int colDist = Math.abs(position[1] - goal.getMapPosition()[1]);		
		return (rowDist+colDist);
	}
	
	/**	
	 * returns a 1x4 int array containing values of 1 or 0, where 1 corresponds to 
	 * a wall and 0 corresponds to no wall.  The values are given in the order
	 * of "east", "north", "west", and "south".
	 * @return a 1x4 int array containing values of 1 or 0, where 1 corresponds to 
	 * a wall and 0 corresponds to no wall.  The values are given in the order
	 * of "east", "north", "west", and "south".
	 */
	public int[] getWalls(){
		int[] walls = new int[4];
		walls[0] = eastWall;
		walls[1] = northWall;
		walls[2] = westWall;
		walls[3] = southWall;
		return walls;
	}
	
	/** 	 
	 * returns a 1x4 MapCell array of all neighboring cells in the order of "east", 
	 * "north", "west", "south".
	 * @return a 1x4 MapCell array of all neighboring cells in the order of "east", 
	 * "north", "west", "south".
	 */
	public MapCell[] getNeighbors(){
		MapCell[] neighbors = new MapCell[4];
		neighbors[0] = getNeighbor("east");
		neighbors[1] = getNeighbor("north");;
		neighbors[2] = getNeighbor("west");
		neighbors[3] = getNeighbor("south");
		return neighbors;
	}		
	
	/**
	 * returns a single neighbor designated by the string parameter or a null.
	 * @param neighbor the neighboring cell being returned.  Use the string
	 * "east", "north", "west", or "south".
	 * @return the neighboring cell specified by the string or a null value if
	 * the neighbor does not exist (as is the case with cells along the edge).
	 */
	public MapCell getNeighbor(String neighbor){
		MapCell neighborCell = null;
		try {
			if(neighbor.equalsIgnoreCase("east")){				
				neighborCell = eastCell;
			}else if(neighbor.equalsIgnoreCase("north")){				
				neighborCell = northCell;
			}else if(neighbor.equalsIgnoreCase("west")){				
				neighborCell = westCell;
			}else if(neighbor.equalsIgnoreCase("south")){				
				neighborCell = southCell;
			}
		}catch(NullPointerException np){
			return neighborCell;
		}catch(Exception e){
			return neighborCell;
		}
		return neighborCell;
	}
	
	/**	 
	 * returns the weight associated with the cell as an int, used for 
	 * path-planning.
	 * @return the weight associated with the cell, used for path-planning.
	 */
	public int getWeight(){
		return weight;
	}
	
	/**	 
	 * return a 1x4 integer array with the weights of the neighboring cells.  
	 * Values given in the order "east", "north", "west", or "south".
	 * @return a 1x4 integer array with the weights of the neighboring cells.  
	 * Values given in the order "east", "north", "west", or "south".
	 */
	public int[] getNeighborsWeights(){
		int[] weights = new int[4];
		weights[0] = getNeighbor("east").weight;
		weights[1] = getNeighbor("north").weight;
		weights[2] = getNeighbor("west").weight;
		weights[3] = getNeighbor("south").weight;		
		return weights;
	}
	
	/**
	 * Prints the cell position in row and column coordinates as well as a 
	 * tuple containing the boolean values of the walls around each cell.  
	 * Example, Cell [1][3] -- (1, 0, 1, 1) indicates that the cell located 
	 * at [1][3] on the map has a wall on the east side, no wall on the 
	 * north side, and a wall on the west and south sides respectively.
	 */
	public void printWalls(){
		System.out.printf("Cell [%d][%d] -- (%d, %d, %d, %d)\n",position[0],
						position[1],eastWall, northWall, westWall, southWall);
	}	
	
	/**
	 * Prints the position of each cell in row and column coordinates as well 
	 * as the weight of that cell. Example, Cell [1][3] weight = 4 indicates 
	 * that the cell at [1][3] has a weight of 4 in the path-planning algorithm 
	 * being used.
	 */
	public void printWeight(){
		System.out.printf("Cell [%d][%d] weight = %d\n",position[0],
				position[1],weight);
	}
	
}//end of MapCell class