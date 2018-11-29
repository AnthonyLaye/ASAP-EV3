package ca.mcgill.ecse211.finalproject;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

/**
 * This class uses two light sensors to localize the robot to the closest grid line intersection
 * @author Anthony Laye
 * @author Mai Zeng
 */
public class LightLocalizer {
		private Navigation navigation;
		private EV3ColorSensor LSL;
		private EV3ColorSensor LSR;
		private Odometer odo;
		
		public LightLocalizer(Navigation navigation, EV3ColorSensor LSL, EV3ColorSensor LSR, Odometer odo) throws OdometerExceptions {
			this.navigation = navigation;
			this.LSL = LSL;
			this.LSR = LSR;
			this.odo = odo;
		}

		/**
		 * This method drives the robot forward until it it sees a first black line, then rotates to find the next one. It stops at the intersection
		 * and updates the x, y and theta coordinates. It also gets a sample of the floor tile value in order to check if a robot has reached a tile line.
		 * @param startX : x coordinate value at first grid line intersection
		 * @param startY : y coordinate value at first grid line intersection
		 * @param startAngle : the direction the robot will be facing based on the starting corner
		 */
		public void lightLocalize(int startX, int startY, int startAngle) {
			
			try {
				Thread.sleep(700);
			} catch (InterruptedException e) {
			}
			
			SensorMode colourLeft;
			colourLeft = LSL.getRedMode();
		    float[] sampleLeft = new float[3];
		    colourLeft.fetchSample(sampleLeft, 0);
		    
		    SensorMode colourRight;
			colourRight = LSR.getRedMode();
		    float[] sampleRight = new float[3];
		    colourRight.fetchSample(sampleRight, 0);
		    
		    Navigation.TILE_FLOOR_COLOR = sampleLeft[0];	// Save the color of the tile floor to be used for comparisons throughout entire code
		    
		    
		    Navigation.leftMotor.forward();
		    Navigation.rightMotor.forward();
		    navigation.lightCorrect();	// Drive forward and perform light correction
		    
		    navigation.advanceRobot(-5, false);	// Move back 5 cm since light sensors are 5 cm behind wheelbase center
			
			navigation.rotateTheRobot(true,90,false);	// Rotate robot and do the same towards the origin
			
			Navigation.leftMotor.forward();
		    Navigation.rightMotor.forward();
		    navigation.lightCorrect();
		    
		    navigation.advanceRobot(-5, false);
			
			odo.setX(startX * 30.48);	// Update robot coordinates based on starting corner data
			odo.setTheta(startAngle);
			odo.setY(startY * 30.48);
		}
}