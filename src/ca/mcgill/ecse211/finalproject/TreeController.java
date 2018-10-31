package ca.mcgill.ecse211.finalproject;

import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

/**
 * This class handles all logic related to interacting with the tree and rings
 *
 */
public class TreeController implements UltrasonicController {
	public enum Color { ORANGE, BLUE, GREEN, YELLOW, NONE };
	private EV3ColorSensor lightSensor;
	private SensorMode color;
	private int distance;
	private int filterControl;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Navigation navigation;
	private Odometer odo;
	private static final int FILTER_OUT = 20;
	
	public TreeController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Navigation navigation, Odometer odometer, EV3ColorSensor lightSensor) throws OdometerExceptions {
		this.navigation = navigation;
		this.odo = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.lightSensor = lightSensor;
	}
	
	/**
	 * Make the robot travel to the starting position of the tree
	 * @param treeX : x coordinate of tree
	 * @param treeY : y coordinate of tree
	 */
	public void approachTree(int treeX, int treeY) {
		
	}
	
	/**
	 * Once at the tree, the robot must detect when it encounters a ring, and beeps a specified amount of times
	 */
	public void detectRing() {
		
	}
	
	/**
	 * Travel to new side of tree for ring search
	 */
	public void switchSides() {
		
	}
	
	
	/**
	 * drive a cycle around the tree
	 */
	public void driveACircle()
	{
		
	}
	
	
	/**
	 * This method is implemented to distinguish the color of a ring using RGB color mode
	 */
	public Color checkColour(){
		
		color = lightSensor.getRGBMode();
	    float[] sample = new float[3];
	    color.fetchSample(sample, 0);	//Get RGB of light sensor reading
	    
	    double red = sample[0] * 1000;	//Scale for easier numbers to read
	    double green = sample[1] * 1000;
	    double blue = sample[2] * 1000;

	    LCD.drawInt((int)red, 0, 4);
	    LCD.drawInt((int)green, 0, 5);
	    LCD.drawInt((int)blue, 0, 6);
	    
	    //Ranges were obtained based on sampling different results for the light sensor detection
	    
	    if(red > 150 && green >40 && blue > 0) {
	    	return Color.YELLOW;
	    }
	    if( 70 > red && red > 30 && green > 100 && 40 > blue && blue > 0){
	    	return Color.GREEN;
	    }
	    if(50 > red && red > 10 && 130 > green && green > 90 && 90 > blue && blue > 60){
	    	return Color.BLUE;
	    }
	    if(140 > red && red > 80 && 60 > green && green > 20 && 30 > blue){
	    	return Color.ORANGE;
	    }
	    else {
	    	return Color.NONE;
	    }
		
	}
	
	@Override
	public void processUSData(int distance) {
		if (distance >= 255 && filterControl < FILTER_OUT) {
		      // bad value, do not set the distance var, however do increment the
		      // filter value
		      filterControl++;
		    } else if (distance >= 255) {
		      // We have repeated large values, so there must actually be nothing
		      // there: leave the distance alone
		      this.distance = distance;
		    } else {
		      // distance went below 255: reset filter and leave
		      // distance alone.
		      filterControl = 0;
		      this.distance = distance;
		    }  
	}

	@Override
	public int readUSDistance() {
		// TODO Auto-generated method stub
		return this.distance;
	}
}