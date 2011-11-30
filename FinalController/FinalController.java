// File:         FinalController.java  
// Date:         12/05/2011
// Description:  Final Controller
// Author:       Kevin Dinkel, Andrew Zizzi

/////////////////////////////////////////////////////
// Imports:
/////////////////////////////////////////////////////
  
import com.cyberbotics.webots.controller.*;
import java.util.Random;
import static java.lang.Math.*;

/////////////////////////////////////////////////////
// Class Definition:
/////////////////////////////////////////////////////
  
public class FinalController extends DifferentialWheels {
  
  /////////////////////////////////////////////////////
  // Static Final Definitions:
  /////////////////////////////////////////////////////

  // Simulation var:
  private static final int TIME_STEP = 32; // milliseconds
    
  // Constants:
  private static final int NUM_LEDS = 10;
  private static final int NUM_DS = 8;
  private static final int NUM_LS = 8;
  private static final int NUM_FS = 3;
  private static final int PROX = 80;
  private static final int PROXLED = 500;
  private static final double R = 0.0205; // m
  private static final double D = 0.052; // m
  private static final double SPEED_MULT = 0.00628;
  private static final double ENC_MULT = 159.23;
  private static final double CELL_DIST = 0.111111111111; // m
  private static final double LINE_BOUND = CELL_DIST/10.0; // m
  private static final double AXEL_LENGTH = 0.052; // m
  private static final double SPEED = 400.0;
  private static final double WALL_MULT = SPEED/2.0; // m
  
  // Wheel Constants:
  private static final int NUM_WHEELS = 2;
  private static final int LEFT_WHEEL = 0;
  private static final int RIGHT_WHEEL = 1;
  
  // Distance Sensor Look-up Table:
  private static final double dsLookUp[][] = 
  {
    {0.00,4095,0.005},
    {0.01,3474,0.037},
    {0.02,2211,0.071},
    {0.03, 306,0.125},
    {0.04, 164,0.206},
    {0.05,  90,0.269},
    {0.06,  56,0.438},
    {0.07,  34,0.704},
  };
  
  // Distance Sensor Angles:
  /*
  private double dsAngle[] = 
  {
    toRadians( 10),
    toRadians( 45),
    toRadians( 90),
    toRadians(150),
    toRadians(210),
    toRadians(270),
    toRadians(315),
    toRadians(350),
  };
  */
  
  // Distance sensor angles (from x_body_frame = 0 deg angle)
  private static final double dsAngle[] = 
  {
    toRadians(  80), // sensor 0
    toRadians(  45), // sensor 1
    toRadians(   0), // sensor 2
    toRadians( -60), // etc.
    toRadians(-120),
    toRadians( 180),
    toRadians( 135),
    toRadians( 100),
  };
  
  /////////////////////////////////////////////////////
  // Global Objects:
  /////////////////////////////////////////////////////
  
  // Sensors:
  private LED[] led = new LED[NUM_LEDS];
  private DistanceSensor[] ds = new DistanceSensor[NUM_DS];
  private LightSensor[] ls = new LightSensor[NUM_LS];
  private DistanceSensor[] fs = new DistanceSensor[NUM_FS];
  private GPS gps;
  
  // Rand obs:
  private Random gen = new Random();
  
  // Path planner:
  private PathPlan planner;// = new PathPlan();
  
  /////////////////////////////////////////////////////
  // Global Variables:
  /////////////////////////////////////////////////////
  
  // LED control:
  private double ledInterval = 4;
  private boolean ledON = true;
  private int currLed = 0;
  
  // Path Control:
  private boolean moving = false;
  private int currMove = -1;
  
  // Speed:
  private static double[] wheel_speed = {0,0};
  
  /////////////////////////////////////////////////////
  // Controller Constructor:
  /////////////////////////////////////////////////////

  public FinalController() {
    
    // Call the Robot constructor:
    super();
    
    // Get devices:
    getDevices();
    
    int[] start = {0,0};
    int dir = 1;
    planner = new PathPlan(start,dir);
  }
  
  /////////////////////////////////////////////////////
  // Configure Devices:
  /////////////////////////////////////////////////////

  private void getDevices()
  {
    for (int i = 0; i < NUM_DS; ++i) {
      ds[i] = getDistanceSensor("ps"+i);
      ds[i].enable(TIME_STEP);
    }
     
    for (int i = 0; i < NUM_LS; ++i) {
      ls[i] = getLightSensor("ls"+i);
      ls[i].enable(TIME_STEP);
    }
     
    for (int i = 0; i < NUM_FS; ++i) {
      fs[i] = getDistanceSensor("fs"+i);
      ls[i].enable(TIME_STEP);
    }
    
    for (int i = 0; i < NUM_LEDS; ++i) {
     led[i] = getLED("led"+i);
    } 
    
    enableEncoders(TIME_STEP);
  }
  
  
  /////////////////////////////////////////////////////
  // MAIN:
  /////////////////////////////////////////////////////
  
  public static void main(String[] args) {
    FinalController controller = new FinalController();
    controller.run();
  }
  
  /////////////////////////////////////////////////////
  // RUN:
  /////////////////////////////////////////////////////
  
  public void run() {
    
    // Internal run vars:
    int step = 0;
        
    // Plan Path:
    int[] start = {0,0};
    planner.setGoalToCharger(0);
    planner.setStart(start);
    planner.Astar();
    int[] path = planner.getDirections();
    int[] wall = new int[4];
    
    while (step(TIME_STEP) != -1) {
        
      // If we have not reached our goal:
      if( currMove < path.length ){
        
        // Path Control:
        pathControl(path);
        
        // Correct for Walls:
        correctWall();
      }
      
      // Set wheel speeds:
      setSpeed(wheel_speed[LEFT_WHEEL],wheel_speed[RIGHT_WHEEL]);
      
      // Reset wheel speeds:
      wheel_speed[LEFT_WHEEL] = 0; wheel_speed[RIGHT_WHEEL] = 0;
      
      // Light LEDs:
      beaconLEDs(step);
      
      // Prox LEDs:
      proxLED();
      
      // Detect walls:
      wall = isWall(0);
      
      p(wall[0] + " " + wall[1] + " " + wall[2] + " " + wall[3]);
      
      // Increment step:
      step += 1;
    }    
  }
  
  /////////////////////////////////////////////////////
  // Detect Wall:
  /////////////////////////////////////////////////////
  
  private int[] isWall(int dir){ // dir: 0 = N, 1 = E, 2 = S, 3 = W 
  
    // Initialize:
    int temp = 0;
    int[] NESW = new int[4];
    for(int i = 0; i < 4; i++)
      NESW[i] = 0;
    
    if(ds[0].getValue() > PROX && ds[7].getValue() > PROX)
      NESW[0] = 1;
    if(ds[1].getValue() > PROX && ds[2].getValue() > PROX)
      NESW[1] = 1;
    if(ds[3].getValue() > PROX && ds[4].getValue() > PROX)
      NESW[2] = 1;
    if(ds[5].getValue() > PROX && ds[6].getValue() > PROX)
      NESW[3] = 1;
    
    switch (dir)
    {
      case 0:
        return NESW;
      case 1:
        temp = NESW[3];
        NESW[3] = NESW[2];
        NESW[2] = NESW[1];
        NESW[1] = NESW[0];
        NESW[0] = temp;
        break;
      case 2:
        temp = NESW[3];
        NESW[3] = NESW[1];
        NESW[1] = temp;
        temp = NESW[2];
        NESW[2] = NESW[0];
        NESW[0] = temp;
        break;
      case 3:
        temp = NESW[0];
        NESW[0] = NESW[1];
        NESW[1] = NESW[2];
        NESW[2] = NESW[3];
        NESW[3] = temp;
        break;
      default:
        p("invalid direction! should be [0-3]");
        break;
    }
    
    return NESW;
  }
  /////////////////////////////////////////////////////
  // Get Location (Odometry):
  /////////////////////////////////////////////////////
  
  private double[] getLocation() {
  
  double[] speed = new double[2]; // [m]
  double[] location = new double[3];
  
  // Convert to meters:
  for(int i=0;i<NUM_WHEELS;i++)
    speed[i] = wheel_speed[i] * SPEED_MULT;
    
  location[0] += -(speed[RIGHT_WHEEL]+speed[LEFT_WHEEL])*(R/2)*Math.cos(location[2])
                    *(TIME_STEP/1000.0); //[m]
  location[1] += -(speed[RIGHT_WHEEL]+speed[LEFT_WHEEL])*(R/2)*Math.sin(location[2])
                    *(TIME_STEP/1000.0); // [m]
  location[2] += (speed[RIGHT_WHEEL]-speed[LEFT_WHEEL])*(R/D)
                    *(TIME_STEP/1000.0); //[rad]
  location[2] %= 2*Math.PI;
  
  return location;
  
  }    
  
  /////////////////////////////////////////////////////
  // Wall Correction and Detection Functions:
  /////////////////////////////////////////////////////
  
  // Wall Correction:
  private void correctWall(){
    double wallDet[][] = new double[2][2];
    // Detect Wall:
    wallDet = detectWall();

    if(!Double.isNaN(wallDet[0][0]) && (wallDet[0][0] <= CELL_DIST/2.0))
      wheel_speed[0] += wallDet[0][1] * WALL_MULT;

    if(!Double.isNaN(wallDet[1][0]) && (wallDet[1][0] <= CELL_DIST/2.0))
      wheel_speed[1] += wallDet[1][1] * WALL_MULT;

  }
  
  // Wall Detection:
  private double[][] detectWall(){
    
    double points[][] = new double[3][2];
    double ret[][] = new double[2][2];
    
      // Get right side points:
    for(int i = 0; i < 3; i++){
      points[i] = getdsPoint(i+1);
    }
    
    ret[1] = getLine(points);
    
    // Correct Right Side:
    if(ret[1][1] >= 0)
      ret[1][1] = PI - ret[1][1];
    else
      ret[1][1] = -(PI + ret[1][1]);
      
    // Get left side points:
    for(int i = 0; i < 3; i++){
      points[i] = getdsPoint(i+4);
    }
    
    ret[0] = getLine(points);
    
    return ret;
  }
    
  private double[] getLine(double points[][]){
    
    double ret[] = new double[2];
    double m = 0; double b = 0; double alpha = 0;
    
    // Calculate line:
    // m = dx/dy
    m = (points[2][0] - points[0][0])/(points[2][1] - points[0][1]);
    
    // b = x - m*y
    b = points[0][0]-m*points[0][1];
    
    // Make sure it is a wall by checking center point:
    if( (m*points[1][1] + b) < points[1][0] + LINE_BOUND && 
        (m*points[1][1] + b) > points[1][0] - LINE_BOUND   ){

      alpha = atan2(points[2][0] - points[0][0],points[2][1] - points[0][1]);
      
      // store data:
      ret[0] = abs(b) - AXEL_LENGTH/2.0; //b/m*sin(alpha);
      ret[1] = alpha % PI;
    }
    else{
      ret[0] = Double.NaN;
      ret[1] = Double.NaN;
    }
    
    // return dist, orientation
    return ret;
  }
  
  // Return x,y point to wall:
  private double[] getdsPoint(int dsNum){
      double d = getDistance(ds[dsNum].getValue()) + AXEL_LENGTH/2.0;
      double phi = dsAngle[dsNum];
      double xy[] = new double[2];
      xy[0] = d*cos(phi);
      xy[1] = d*sin(phi);
      return xy;
  }
  
  /////////////////////////////////////////////////////
  // Path Control Functions:
  /////////////////////////////////////////////////////
  
  // Drive a specified path:
  private void pathControl(int[] path){
    if(moving){
      moving = move(path[currMove]);
    }
    else{
      setEncoders(0,0);
      moving = true;
      currMove++;
    }  
  }
  
  private boolean move(int path_type){
    switch (path_type) {
      case 0:  return  moveFoward(); 
      case 1:  return  turnLeft(); 
      case 2:  return  turnRight(); 
      default: return moving;
    }
  }
  
    // Go foward unit step:
  private boolean moveFoward()
  {
    if(abs(getLeftEncoder()) < getEncoderVal(CELL_DIST)){
        wheel_speed[0] += SPEED; wheel_speed[1] += SPEED;
        return true;
      } else{ 
        wheel_speed[0] = 0; wheel_speed[1] = 0;
        return false;
      }
  }
  
  private boolean turnRight()
  {
    if(abs(getLeftEncoder()) < getEncoderVal(CELL_DIST)/2.53){
        wheel_speed[0] += SPEED; wheel_speed[1] -= SPEED;
        return true;
      } else{ 
        wheel_speed[0] = 0; wheel_speed[1] = 0; 
        return false;
      }
  }
  
  private boolean turnLeft()
  {
    if(abs(getLeftEncoder()) < getEncoderVal(CELL_DIST)/2.53){
        wheel_speed[0] -= SPEED; wheel_speed[1] += SPEED;
        return true;
      } else{ 
        wheel_speed[0] = 0; wheel_speed[1] = 0;
        return false;
      }
  }
  
  /////////////////////////////////////////////////////
  // LED functions:
  /////////////////////////////////////////////////////
  
  // Private helper functions:
  private void beaconLEDs(int step){
    if(step%ledInterval == 0){
      
      if(ledON){
        led[currLed].set(1);
        currLed++;
      }
      else{
        led[currLed].set(0);
        currLed--;
      }
      
      if(currLed >= NUM_LEDS-2){
        ledON = false;
      }
      if(currLed < 0){
        ledON = true;
        currLed = 0;
      }
    }
  }
  
  // Proximal LED:
  private void proxLED(){
    
    if(ds[0].getValue() > PROXLED || ds[7].getValue() > PROXLED)
      led[NUM_LEDS-2].set(1);
    else{ led[NUM_LEDS-2].set(0); }
    
  }
  
  /////////////////////////////////////////////////////
  // Helper functions:
  /////////////////////////////////////////////////////
  
  // Print function:
  private void p(String str){
    System.out.println(str);
  }
  
  // Get encoder value for a given distance:
  private double getEncoderVal(double dist){
    return ENC_MULT*dist/0.0205;
  }
  
  // Get distance in meters from an encoder value:
  private double getDistance(double val){
    // Error check:
    if(val < 0 || val > 4095)
      return -1;
    
    // Find correct index in table:
    int lowIndex = dsLookUp.length-1; int highIndex = dsLookUp.length-2;
    for(int i = 0; i < dsLookUp.length; i++){
      if( dsLookUp[i][1] < val ){
        highIndex = i-1;
        lowIndex = i;
        break;
      }
    }
    
    // Linear interpolation:
    return ((val-dsLookUp[lowIndex][1])/(dsLookUp[highIndex][1]-dsLookUp[lowIndex][1]))*
           (dsLookUp[highIndex][0]-dsLookUp[lowIndex][0]) + dsLookUp[lowIndex][0];
  }
  
}