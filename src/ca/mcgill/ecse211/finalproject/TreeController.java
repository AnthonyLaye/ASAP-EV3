package ca.mcgill.ecse211.finalproject;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;

/**
 * This class handles all logic related to interacting with the tree and rings
 * @author Anthony Laye
 * @author Mai Zeng
 */
public class TreeController {
	public enum Color { ORANGE, BLUE, GREEN, YELLOW, NONE };
	private EV3ColorSensor lightSensor;
	private SensorMode color;
	private Navigation navigation;
	private Odometer odo;
	private ArmController armController;
	private boolean backOrFarward;
	
	public TreeController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Navigation navigation, Odometer odometer, EV3ColorSensor lightSensor, ArmController armController) throws OdometerExceptions {
		this.navigation = navigation;
		this.odo = odometer;
		this.lightSensor = lightSensor;
		this.armController = armController;
	}
	
	/**
	 * Make the robot travel to the starting position of the tree. We first travel along x, then y, so we approach the tree head on instead of at an angle
	 * @param treeX : x coordinate of tree
	 * @param treeY : y coordinate of tree
	 */
	public void approachTree(double treeX, double treeY) {
		
		double angle = 0;
		navigation.travelTo(treeX - 1, odo.getXYT()[1] / 30.48, false);	// Travel along x first, then y
		
		double new_y = getStopPosition(treeX, treeY);	// Get y position to stop at
		
		navigation.travelTo(treeX - 1, new_y, true);
		navigation.travelTo(treeX, new_y, false);	// Travel along y 
		
		if(backOrFarward)
		{
			navigation.rotateTheRobot(true, 95, false);
		}
		else
			navigation.rotateTheRobot(false, 95, false);
		
		Sound.beep();	// We are at tree
		Sound.beep();
		Sound.beep();
		
		angle = odo.getXYT()[2]; // Store initial angle for later use
		
		armController.openArms();
		
		findRings();	// Find those rings!!
		
		odo.setX(treeX);	// After full rotation, reset odometer to what it was before
		odo.setY(new_y);
		odo.setTheta(angle);
	}
	
	/**
	 * Once at a side of the tree, the robot must detect if it encountered a ring, and beeps a specified amount of times
	 */
	public boolean detectRing() {
		if(checkColour() != Color.NONE) 
			return true;
		else
			return false;
	}
	
	/**
	 * Get the stopping Y coordinate for the robot right before the tree, depending on if the robot is below or above it
	 * @param treeX : x coordinate for the tree
	 * @param treeY : y coordinate for the tree
	 * @return new_y : new stopping y coordinate
	 */
	public double getStopPosition(double treeX, double treeY) {
		
		double new_y = 0;
		
		if(odo.getXYT()[1] / 30.48 < treeY)	// We want to stop half a tile before the tree, so this check is to see if robot is above 
		{									// or below the tree.
			backOrFarward = false;
			new_y = treeY - 1.0; 				
		}
		else
		{
			backOrFarward = true;
			new_y = treeY + 1.0;
		}
		
		return new_y;
	}
	
	/**
	 * Travel to all sides of tree until we find a ring
	 */
	public void findRings() {
		int count = 0;
		boolean found = false;
		while(count < 4) { // The tree has four sides to visit
			
			navigation.advanceRobot(15, false);
			navigation.lightCorrect();
			armController.closeArms(); //get the rings
			armController.openArms();
			armController.closeArms();
			navigation.advanceRobot(-15, false);
			
			try {	// Sleep for a few seconds to let the ring settle in front of the light sensor
				Thread.sleep(2000);
			    } catch (InterruptedException e) {
			    // There is nothing to be done here
			}
			
			armController.rotateArms(-80);	// Open arms slightly to let light sensor detect color
			
			if(!found) {
				while(armController.leftArmMotor.isMoving() && armController.rightArmMotor.isMoving()) {
					if(!found) {
						if(detectRing()) {  // We have spotted a ring
							found = true;
						}
					}
				}
			}
			
			navigation.rotateTheRobot(true, 90, false); // Turn 90 degree to reach other side
			navigation.advanceRobot(10, false);
			navigation.advanceRobot(26, true);
			navigation.lightCorrect();
			navigation.rotateTheRobot(false, 90, false);
			navigation.advanceRobot(10, false);
			navigation.advanceRobot(26, true);
			navigation.lightCorrect();
			navigation.advanceRobot(5, false);
			navigation.rotateTheRobot(false, 105, false);
			
			count++;
		}
		
		navigation.rotateTheRobot(false, 90, false);
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
	    
	    //Ranges were obtained based on sampling different results for the light sensor detection
	    
	    if(red > 150 && green >40 && blue > 0) {
	    	Sound.beep();
	    	Sound.beep();
	    	Sound.beep();
	    	return Color.YELLOW;
	    }
	    if( 70 > red && red > 30 && green > 100 && 40 > blue && blue > 0){
	    	Sound.beep();
	    	Sound.beep();
	    	return Color.GREEN;
	    }
	    if(50 > red && red > 10 && 130 > green && green > 90 && 90 > blue && blue > 60){
	    	Sound.beep();
	    	return Color.BLUE;
	    }
	    if(140 > red && red > 80 && 60 > green && green > 20 && 30 > blue){
	    	Sound.beep();
	    	Sound.beep();
	    	Sound.beep();
	    	Sound.beep();
	    	return Color.ORANGE;
	    }
	    else {
	    	return Color.NONE;
	    }
		
	}
}