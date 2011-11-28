package csci3302lab5;

/**
 * RatMap creates a map of the ratslife simulation.  Essentially the map
 * is a 2D array of MapCells.  The cell [0][0] corresponds to the bottom
 * left and the cell[9][9] corresponds to the top right of the ratslife 
 * maze.  The first parameter of each MapCell is the row number and the
 * second parameter is the column number. 
 *   
 * @author Rowan Wing 9/21/11
 *
 */
public class RatMap {
	/**
	 * the ratslife maze is 10x10 so the SIDELENGTH is finalized to 10
	 */
	public static final int SIDELENGTH = 10;
	/**
	 * the primary field of the RatMap containing the 2D 10x10 array that is
	 * used to model the maze
	 */
	public static MapCell[][] map;
	/**
	 * a 2D 10x10 array that is used during path-planning to hold the weights
	 * of the cells discovered by the path-planning algorithm.  Initially all
	 * cells are set to a weight of 1000 as an arbitrarily high weight.
	 */
	public MapCell[][] weightedMap;
	/**
	 * a 1x4 array that holds the cells in which the chargers can be found.
	 */
	public MapCell[] chargers;	
	
	
	/**
	 * Constructor that allows the user to create different types of maps
	 * @param type the type of map to create 
	 * Types include "simulation", "legoVersion", "random", and "blank", 
	 * only "simulation" and "blank" are implemented at this time 
	 */
	public RatMap(String type){
		if(type.equalsIgnoreCase("simulation")){
			loadSimulatedWorld();
		}else if(type.equalsIgnoreCase("lego")){
			//loadLegoWorld(); 
			System.err.println("This world has not yet been created");
		}else if(type.equalsIgnoreCase("random")){
			//loadRandomWorld();
			System.err.println("This world has not yet been created");
		}else if(type.equalsIgnoreCase("blank")){
			map = new MapCell[SIDELENGTH][SIDELENGTH];			
		}else{
			System.err.printf("You must specify the world to be created using " +
					"'simulation', 'lego', 'random', or 'blank'- line #%d\n",
					Thread.currentThread().getStackTrace()[2].getLineNumber());
		}
	}
	
	/**
	 * Creates a map based on the simulated maze we have been using in the
	 * csci3302 labs.  The map, weighted map, and charger array are created 
	 * and initialized and the neighbors are all linked together.  The weighted
	 * map has the weight of all cells set to 1000 as an arbitrarily high value.
	 */
	private void loadSimulatedWorld(){
		map = new MapCell[SIDELENGTH][SIDELENGTH];
		weightedMap = new MapCell[SIDELENGTH][SIDELENGTH];
		chargers = new MapCell[4];			
			
		//first column counting up and right from the bottom left
		addCell(0,0,1,0,1,1);
		addCell(1,0,0,0,1,0);
		addCell(2,0,0,1,1,0);
		addCell(3,0,1,0,1,1);
		addCell(4,0,0,1,1,0);
		addCell(5,0,0,0,1,1);
		addCell(6,0,0,1,1,0);
		addCell(7,0,0,1,1,1);
		addCell(8,0,0,1,1,1);
		addCell(9,0,0,1,1,1);
		
		//second column
		addCell(0,1,0,0,1,1);
		addCell(1,1,1,0,0,0);
		addCell(2,1,0,1,0,0);
		addCell(3,1,1,0,1,1);
		addCell(4,1,1,0,0,0);
		addCell(5,1,0,0,0,0);
		addCell(6,1,1,0,0,0);
		addCell(7,1,1,1,0,0);
		addCell(8,1,0,0,0,1);
		addCell(9,1,1,1,0,0);
		
		//third column
		addCell(0,2,1,0,0,1);
		addCell(1,2,1,1,1,0);
		addCell(2,2,0,0,0,1);
		addCell(3,2,0,0,1,0);
		addCell(4,2,0,0,1,0);
		addCell(5,2,1,0,0,0);
		addCell(6,2,0,0,1,0);
		addCell(7,2,1,0,1,0);
		addCell(8,2,0,0,0,0);
		addCell(9,2,0,1,1,0);
		
		//fourth column
		addCell(0,3,1,0,1,1);
		addCell(1,3,1,0,1,0);
		addCell(2,3,1,1,0,0);
		addCell(3,3,0,1,0,1);
		addCell(4,3,0,0,0,1);
		addCell(5,3,1,0,1,0);
		addCell(6,3,0,0,0,0);
		addCell(7,3,0,0,1,0);
		addCell(8,3,0,0,0,0);
		addCell(9,3,1,1,0,0);
		
		//fifth column
		addCell(0,4,0,0,1,1);
		addCell(1,4,0,0,1,0);
		addCell(2,4,0,0,1,0);
		addCell(3,4,1,1,0,0);
		addCell(4,4,1,0,0,1);
		addCell(5,4,0,0,1,0);
		addCell(6,4,1,0,0,0);
		addCell(7,4,0,1,0,0);
		addCell(8,4,1,0,0,1);
		addCell(9,4,1,1,1,0);
		
		//sixth column
		addCell(0,5,0,1,0,1);
		addCell(1,5,0,1,0,1);
		addCell(2,5,0,1,0,1);
		addCell(3,5,1,0,1,1);
		addCell(4,5,1,0,1,0);
		addCell(5,5,1,1,0,0);
		addCell(6,5,1,0,1,1);
		addCell(7,5,0,0,0,0);
		addCell(8,5,0,0,1,0);
		addCell(9,5,1,1,1,0);
		
		//seventh column
		addCell(0,6,0,1,0,1);
		addCell(1,6,1,0,0,1);
		addCell(2,6,0,0,0,0);
		addCell(3,6,1,1,1,0);
		addCell(4,6,1,0,1,1);
		addCell(5,6,0,0,1,0);
		addCell(6,6,0,0,1,0);
		addCell(7,6,0,1,0,0);
		addCell(8,6,1,0,0,1);
		addCell(9,6,1,1,1,0);
		
		//eighth column
		addCell(0,7,0,0,0,1);
		addCell(1,7,1,1,1,0);
		addCell(2,7,0,1,0,1);
		addCell(3,7,0,0,1,1);
		addCell(4,7,0,0,1,0);
		addCell(5,7,0,1,0,0);
		addCell(6,7,0,1,0,1);
		addCell(7,7,1,0,0,1);
		addCell(8,7,1,0,1,0);
		addCell(9,7,1,1,1,0);
		
		//ninth column
		addCell(0,8,0,0,0,1);
		addCell(1,8,1,1,1,0);
		addCell(2,8,0,1,0,1);
		addCell(3,8,1,0,0,1);
		addCell(4,8,0,1,0,0);
		addCell(5,8,1,1,0,1);
		addCell(6,8,0,0,0,1);
		addCell(7,8,1,0,1,0);
		addCell(8,8,0,0,1,0);
		addCell(9,8,0,1,1,0);
		
		//tenth and last column
		addCell(0,9,1,0,0,1);
		addCell(1,9,1,1,1,0);
		addCell(2,9,1,1,0,1);
		addCell(3,9,1,0,1,1);
		addCell(4,9,1,0,0,0);
		addCell(5,9,1,1,1,0);
		addCell(6,9,1,1,0,1);
		addCell(7,9,1,0,1,1);
		addCell(8,9,1,1,0,0);
		addCell(9,9,1,1,0,1);
		
		//set the positions of the chargers
		chargers[0] = map[0][3];
		chargers[1] = map[5][3];
		chargers[2] = map[8][5];
		chargers[3] = map[9][1];
		
		//initialize a weighted map with all cells equal to 1000
		for(int i=0; i<SIDELENGTH; i++){
			for(int j=0; j<SIDELENGTH; j++){
				weightedMap[i][j] = new MapCell(i,j);
				weightedMap[i][j].setWeight(1000);
			}
		}
		
		//set the neighbor fields for each cell in the map
		//neighbors are ordered east, north, west, and south
		for(int i=0; i<SIDELENGTH; i++){
			for(int j=0; j<SIDELENGTH; j++){
				map[i][j].setNeighbors();				
			}			
		}
	}		
	
	/**
	 * Adds a cell to the map.  Used in the initialization of the map.
	 * The weight of the cell is initially set to 1000 as an arbitrarily 
	 * high number.
	 * @param row the row number of the cell being initialized
	 * @param column the column number of the cell being initialized
	 * @param e the east wall (1 = wall, 0 = no wall)
	 * @param n the north wall (1 = wall, 0 = no wall)
	 * @param w the west wall (1 = wall, 0 = no wall)
	 * @param s the south wall (1 = wall, 0 = no wall)
	 */
	private void addCell(int row, int column, int e, int n, int w, int s){
		map[row][column] = new MapCell(row,column,e,n,w,s);
		map[row][column].setWeight(1000);
	}
	
	/**
	 * returns the MapCell of the position specified in row/column coordinates
	 * @param row the row number of the cell returned
	 * @param column the column number of the cell returned
	 * @return a MapCell specified by the row/column parameters
	 */
	public static MapCell getMapCell(int row, int column){
		return map[row][column];
	}
	
	/**
	 * Clears the weighted map and reinitializes to all zeros
	 */
	public void clearWeightedMap(){
		weightedMap = new MapCell[SIDELENGTH][SIDELENGTH];
	}
	
	/**
	 * returns a 4x1 array of MapCells each of which holds a charger
	 * @return a 4x1 array of MapCells each of which holds a charger
	 */
	public MapCell[] getChargers(){
		return chargers;
	}
	
	/**
	 * This method prints out each cell of the RatMap along with its
	 * position, its wall values, and the positions of its four neighbors.  
	 * Use this method to verify that the map has been initialized correctly.
	 */
	public void verifyMap(){		
		for(int i=0; i<RatMap.SIDELENGTH; i++){
			for(int j=0; j<RatMap.SIDELENGTH; j++){
				int [] cell = RatMap.map[i][j].getMapPosition();
				int [] walls = RatMap.map[i][j].getWalls();
				MapCell[] neighbors = RatMap.map[i][j].getNeighbors();
				System.out.printf("Cell [%d][%d], walls (%d, %d, %d, %d), neighbors ",
						cell[0],cell[1],walls[0],walls[1],walls[2],walls[3]);
				for(int k=0; k<4; k++){
					try{
						if(k==0)
							System.out.printf("East[%d][%d] ",
							neighbors[0].getMapPosition()[0],
							neighbors[0].getMapPosition()[1]);
						else if(k==1)
							System.out.printf("North[%d][%d] ",
							neighbors[1].getMapPosition()[0],
							neighbors[1].getMapPosition()[1]);
						else if(k==2)
							System.out.printf("West[%d][%d] ",
							neighbors[2].getMapPosition()[0],
							neighbors[2].getMapPosition()[1]);
						else
							System.out.printf("South[%d][%d] ",
							neighbors[3].getMapPosition()[0],
							neighbors[3].getMapPosition()[1]);
					
					}catch(Exception e){						
						System.out.print("=null ");
						continue;
					}					
				}
				System.out.println();
			}
		}
	}
	
	/**
	 * Prints the map out as a list containing the cell position in row and
	 * column coordinates as well as a tuple containing the boolean values
	 * of the walls around each cell.  Example, Cell [1][3] -- (1, 0, 1, 1)
	 * indicates that the cell located at [1][3] on the map has a wall on the
	 * east side, no wall on the north side, and a wall on the west and south
	 * sides respectively.
	 */
	public void printMap(){
		for(int i=0; i<SIDELENGTH; i++){
			for(int j=0; j<SIDELENGTH; j++){
				map[i][j].printWalls();
			}
		}
	}
	
	/**
	 * Prints the weightedMap out as a list containing the cell position in 
	 * row and column coordinates as well as the weight of that cell. Example,
	 * Cell [1][3] weight = 4 indicates that the cell at [1][3] has a weight
	 * of 4 in the path-planning algorithm being used.
	 */
	public void printWeightedMap(){
		for(int i=0; i<SIDELENGTH; i++){
			for(int j=0; j<SIDELENGTH; j++){
				weightedMap[i][j].printWeight();
			}
		}
	}	
}//end RapMap class

