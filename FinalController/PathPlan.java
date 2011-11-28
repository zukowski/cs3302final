
import csci3302lab5.RatMap;
import csci3302lab5.MapCell;

public class PathPlan {
  // RatMap object
  private RatMap map;
  // Map width
  int mapWidth;
  // Starting location
  private int[] start = new int[2];
  // Goal
  private int[] goal = new int[2];
  // Computed path
  int[][] path;
  // Directions for path
  int[] directions;
  // Direction convention for robot travel
  //
  //     ^        1
  //   <   >    2   4
  //     v        3
  //
  int dir;
  
  // PathPlan constructors
  public PathPlan() {
    // Build map
    map = new RatMap("simulation");
    mapWidth = map.SIDELENGTH;
    // Initialize key variables with invalid values
    start[0] = -1;
    start[1] = -1;
    goal[0] = -1;
    goal[1] = -1;
    dir = -1;
  }
  public PathPlan(int[] start, int dir) {
    // Build map
    map = new RatMap("simulation");
    mapWidth = map.SIDELENGTH;
    // Initialize key variables
    this.start = start;
    this.dir = dir;
    // Initialize goal to invalid value
    goal[0] = -1;
    goal[1] = -1;
  }
  
  // Public helper to get the direction
  public int getRobotDirection() {
    return dir;
  }
  
  // Public helper to set the start node
  public void setStart(int[] loc) {
    start = loc;
  }
  
  // Public helper to set the goal to a specific location
  public void setGoal(int[] loc) {
    goal = loc;
  }
  
  // Public helper to set the goal to be a specific charger
  public void setGoalToCharger(int charger) {
    // Get charger locations
    MapCell[] chargers = map.getChargers();
    // Get the map position of the requested charger
    goal = chargers[charger].getMapPosition(); //output as [col, row]
    int temp = goal[0];
    goal[0] = goal[1];
    goal[1] = temp;
    println("Goal set: ["+goal[0]+"]["+goal[1]+"]");
  }
  
  // A* algorithm
  public void Astar() {
    // Check to make sure start and goal nodes have been set
    if (start[0] == -1) {
      println("Error in A*: Start node has not been set.");
      return;
    }
    if (goal[0] == -1) {
      println("Error in A*: Goal node has not been set.");
      return;
    }
    // Set of nodes to be evaluated (aka. open set)
    boolean[][] openset = new boolean[mapWidth][mapWidth];
    boolean isClosed = false;
    // The set of nodes already evaluated (aka. closed set)
    boolean[][] closedset = new boolean[mapWidth][mapWidth];
    // The map of navigated nodes
    int[][][] camefrom = new int[mapWidth][mapWidth][2];
    // Set open and closed sets to reflect initial conditions
    for (int i = 0; i < mapWidth; i++) {
      for (int j = 0; j < mapWidth; j++) {
        openset[i][j] = false;
        closedset[i][j] = false;
        camefrom[i][j][0] = -1;
        camefrom[i][j][1] = -1;
      }
    }
    openset[start[0]][start[1]] = true;
    
    // Cost tables along best known path
    int[][] gScore = new int[mapWidth][mapWidth];
    int[][] hScore = new int[mapWidth][mapWidth];
    // Set all cost tables to max integer value
    for (int i = 0; i < mapWidth; i++) {
      for (int j = 0; j < mapWidth; j++) {
        gScore[i][j] = Integer.MAX_VALUE;
        hScore[i][j] = Integer.MAX_VALUE;
      }
    }
    // Initialize cost tables for starting position
    gScore[start[0]][start[1]] = 0;
    hScore[start[0]][start[1]] = manhattanDist(start,goal);
    
    // Initialize variables for loop
    int[] x, y;
    int[] wall;
    int r, c;
    int[][] q = new int[4][2];
    int qind;
    int tempScore;
    boolean needUpdate;
    // While the open set is not empty
    while (!isClosed) {
      // Find node in open set having the lowest heuristic score
      x = minHeurScore(hScore,openset);
      // Check to see if the goal is reached
      if (x[0] == goal[0] && x[1] == goal[1]) {
        println("Goal found!");
        break;
      }
      // Remove x from the open set and add it to the closed set
      openset[x[0]][x[1]] = false;
      closedset[x[0]][x[1]] = true;
      // Determine walls around x
      wall = map.getMapCell(x[0],x[1]).getWalls();
      // Determine neighbors of x from walls and place them in queue
      r = x[0];
      c = x[1];
      qind = -1; //reset queue index
      if (wall[0] == 0) {
        qind++;
        q[qind][0] = r;
        q[qind][1] = c+1;
      }
      if (wall[1] == 0) {
        qind++;
        q[qind][0] = r+1;
        q[qind][1] = c;
      }
      if (wall[2] == 0) {
        qind++;
        q[qind][0] = r;
        q[qind][1] = c-1;
      }
      if (wall[3] == 0) {
        qind++;
        q[qind][0] = r-1;
        q[qind][1] = c;
      }
      // For each y = neighbor(x)
      for (int i = 0; i <= qind; i++) {
        y = q[i];
        // Continue if y in closed set
        if (closedset[y[0]][y[1]] == true) {
          continue;
        }
        // Computer tentative score of y
        tempScore = gScore[x[0]][x[1]] + 1;
        // Check to see if y should be updated
        if (openset[y[0]][y[1]] == false) { //y not in open set
          openset[y[0]][y[1]] = true; //add to open set
          needUpdate = true;
        } else if (tempScore < gScore[y[0]][y[1]]) { //score is better
          needUpdate = true;
        } else { //no updates need to be made
          needUpdate = false;
        }
        // Perform update
        if (needUpdate == true) {
          camefrom[y[0]][y[1]][0] = x[0];
          camefrom[y[0]][y[1]][1] = x[1];
          gScore[y[0]][y[1]] = tempScore;
          hScore[y[0]][y[1]] = tempScore + manhattanDist(y,goal);
        }
      }
      // Determine if set is closed
      isClosed = true;
      search:
      for (int i = 0; i < mapWidth; i++) {
        for (int j = 0; j < mapWidth; j++) {
          if (closedset[i][j] == false) {
            isClosed = false;
            break search;
          }
        }
      }
    }
    // Reconstruct the path from the start node to the goal
    path = reconstructPath(camefrom, goal);
    // Display the path
    displayPath();
  }
  
  // Public helper to display the computed path
  public void displayPath() {
    print("Path: ");
    for (int i = 0; i < path.length; i++) {
      print("{"+path[i][0]+","+path[i][1]+"} ");
    }
    println("");
  }
  
  // Private helper function to the A* algorithm
  private int[][] reconstructPath(int[][][] cf, int[] goal) {
    // Generate a temp array to store reverse path from goal to start
    int[][] temp = new int[cf.length*cf.length][2];
    int tind = 0; //temp array row pointer
    temp[tind] = goal;
    int[] prev = cf[goal[0]][goal[1]];
    // Continue until we get back to the start node
    while (prev[0] != -1) {
      // Store previous node
      tind++;
      temp[tind] = prev;
      // Update to next node
      prev = cf[prev[0]][prev[1]];
    }
    // Reverse the order of the path and return
    int[][] path = new int[tind+1][2];
    for (int i = 0; i <= tind; i++) {
      path[i] = temp[tind-i];
    }
    return path;
  }
  
  // Private helper function to the A* algorithm
  private int[] minHeurScore(int[][] F, boolean[][] openset) {
    int[] x = new int[2];
    x[0] = -1; x[1] = -1;
    int minval = Integer.MAX_VALUE;
    for (int i = 0; i < F.length; i++) {
      for (int j = 0; j < F.length; j++) {
        if (openset[i][j] == true) {
          if (F[i][j] < minval) {
            x[0] = i;
            x[1] = j;
            minval = F[i][j];
          }
        }
      }
    }
    return x;
  }
  
  // Private helper function to the A* algorithm
  private int manhattanDist(int[] curr, int[] goal) {
    return Math.abs(goal[0]-curr[0])+Math.abs(goal[1]-curr[1]);
  }
  
  // Helper function to take a path list and turn it into directions
  private void makeDirections() {
    // Temporary array to store map directions
    int[] temp = new int[3*path.length];
    int ind = -1; //index pointing to the last element in temp array
    // Allocate memory for row and column variables
    int r1, r2, c1, c2;
    // Create directions from known path
    for (int i = 1; i < path.length; i++) {
      r1 = path[i-1][0];
      c1 = path[i-1][1];
      r2 = path[i][0];
      c2 = path[i][1];
      switch (dir) {
        case 1: //headed up
          if (c2 < c1) { //make left turn
            ind += 2;
            temp[ind-1] = 1;
            temp[ind] = 0;
            dir = 2; //headed left
          } else if (c2 > c1) { //make right turn
            ind += 2;
            temp[ind-1] = 2;
            temp[ind] = 0;
            dir = 4; //headed right
          } else { //continue up
            ind++;
            temp[ind] = 0;
          }
          break;
        case 2: //headed left
          if (r2 < r1) { //make left turn
            ind += 2;
            temp[ind-1] = 1;
            temp[ind] = 0;
            dir = 3; //headed down
          } else if (r2 > r1) { //make right turn
            ind += 2;
            temp[ind-1] = 2;
            temp[ind] = 0;
            dir = 1; //headed up
          } else { //continue left
            ind++;
            temp[ind] = 0;
          }
          break;
        case 3: //headed down
          if (c2 < c1) { //make right turn
            ind += 2;
            temp[ind-1] = 2;
            temp[ind] = 0;
            dir = 2; //headed left
          } else if (c2 > c1) { //make left turn
            ind += 2;
            temp[ind-1] = 1;
            temp[ind] = 0;
            dir = 4; //headed right
          } else { //continue down
            ind++;
            temp[ind] = 0;
          }
          break;
        case 4: //headed right
          if (r2 < r1) { //make right turn
            ind += 2;
            temp[ind-1] = 2;
            temp[ind] = 0;
            dir = 3; //headed down
          } else if (r2 > r1) { //make left turn
            ind += 2;
            temp[ind-1] = 1;
            temp[ind] = 0;
            dir = 1; //headed up
          } else { //continue right
            ind++;
            temp[ind] = 0;
          }
          break;
        default:
          println("Invalid direction in makeDirections.");
          break;
      }
    }
    // Generate directions output
    directions = new int[ind+1];
    for (int i = 0; i < directions.length; i++) {
      directions[i] = temp[i];
    }
    // Display the directions
    displayDirections();
  }
  
  // Public helper to display the directions
  public void displayDirections() {
    print("Directions: ");
    for (int i = 0; i < directions.length; i++) {
      print(directions[i]+" ");
    }
    println("");
  }
  
  // Public helper to return directions
  public int[] getDirections() {
    int[] invalid = {-1};
    // Check to make sure path has been computed
    if (path == null) {
      println("Error in getDirections: Path not yet computed.");
      return invalid;
    }
    // Make sure robot direction has been set
    if (dir == -1) {
      println("Error in getDirections: Robot direction never set.");
      return invalid;
    }
    // Make and return the directions
    makeDirections();
    return directions;
  }
  
  // Private printing helper function
  private void println(String s) {
    System.out.println(s);
  }
  
  // Private printing helper function
  private void print(String s) {
    System.out.print(s);
  }
} 