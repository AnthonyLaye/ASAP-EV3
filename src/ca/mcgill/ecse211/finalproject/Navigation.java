/*
 * Navigation.java
 */
package ca.mcgill.ecse211.finalproject;


import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

/**
 * This class is used to drive the robot on the demo floor.
 */
public class Navigation implements UltrasonicController {
	public static EV3LargeRegulatedMotor leftMotor;
	public static EV3LargeRegulatedMotor rightMotor;
	public Odometer odo;
	public int distance = 0;  //this distance is for US, 130 is from lab1's data which is the max distance that the sensor cannot sense 
	private int filterControl;
	public boolean navigating = false;
	private static final int FORWARD_SPEED = 150;
	private static final int ROTATE_SPEED = 100;
	private static final int FILTER_OUT = 20;
	private static final double TILE_SIZE = 30.48;
	private static final double WHEEL_RAD = 2.08;
	private static final double TRACK = 13.17;
	public static final double TILE_LENGTH = 30.78;
	public static double TILE_FLOOR_COLOR = 0;
	private EV3ColorSensor LSL;
	private EV3ColorSensor LSR;
	private SensorData data;

	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer, SensorData data,
			EV3ColorSensor LSL, EV3ColorSensor LSR) throws OdometerExceptions {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odo = odometer;
		this.data = data;
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		this.LSL = LSL;
		this.LSR = LSR;
	}

	/**
	 * This method is meant to drive the robot to the predefined waypoints, user must enter if we wait for the robot to finish moving before
	 * return from the method
	 * @param x : x value to travel to
	 * @param y : y value to travel to
	 * @param immediateReturn : True if the function is to be instantly returned. False if the function is to be returned after the travel is completed
	 */
	public void travelTo(double x, double y, boolean immediateReturn) {
		
		SensorMode colourLeft;
		SensorMode colourRight;
		
		double prevSampleLeft = 0.75;
		double prevSampleRight = 0.75;
		
		SensorMode colour;
		colour = LSR.getRedMode();
	    float[] sample = new float[3];
	    colour.fetchSample(sample, 0);
		
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.stop();
		      motor.setAcceleration(1000);
		}

		// Sleep for 2 seconds
		try {
			Thread.sleep(2000);
		    } catch (InterruptedException e) {
		    // There is nothing to be done here
		}
		
		double minimalTheta = 0, travelDistance = 0, currentTheta = 0;
		double currentX = 0;
		double currentY = 0;
		double odometer[] = {0, 0, 0};

		// Get odometer readings
	
		odometer = odo.getXYT();
	

		// Set odometer reading angle as prev angle as well
		currentTheta = odometer[2];

		// Get displacement to travel on X and Y axis
		currentX = odometer[0];
		currentY = odometer[1];

		//Getting the distances with respect to the tile size
		double deltaX = x * TILE_SIZE - (currentX);
		double deltaY =	y * TILE_SIZE - (currentY);
		travelDistance = Math.hypot(Math.abs(deltaX), Math.abs(deltaY));

		//Calculating the minimal angle to get to destination
		minimalTheta = Math.toDegrees(Math.atan2(deltaX,deltaY));

		//If the angle is negative, we want its positive equivalent
		if(minimalTheta < 0) {
			minimalTheta = 360 - Math.abs(minimalTheta);
		}
		
		//Calling the rotate
		navigating = true;
		turnTo(minimalTheta, currentTheta);

		leftMotor.rotate(convertDistance(WHEEL_RAD, travelDistance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, travelDistance), immediateReturn);
		
		/*while (LSL.getColorID() != 2 || sample[0] > 0.4) {
			colour = LSR.getRedMode();
		    sample = new float[3];
		    colour.fetchSample(sample, 0);
			if (sample[0] < 0.4) {
				Sound.beep();
				Navigation.rightMotor.stop(true);
				while(LSL.getColorID() != 2)
				{
					int i = 0; 
				}
				break;
			}
			if (LSL.getColorID() == 2 ) {
				Sound.beep();
				Navigation.leftMotor.stop(true);
				while(sample[0] > 0.4)
				{
					int i =0;
				}
				break;
			}
		}
		*/
		//tell that this method has stopped
		navigating = false;
	}


	/**
	 * This method instructs the robot to rotate to a certain angle, with respect to its current
	 * angle as measured by the odometer
	 * @param minimalT : minimal angle to turn
	 * @param original : original angle on odometer
	 */
	public void turnTo(double minimalT, double original) {
		//Calculating by how much we have to rotate with respect to our current angle
		double deltaT = 0;
		deltaT = minimalT - original;
		boolean turnLeft = false;

		//Getting the positive equivalent
		if(deltaT < 0) {
			deltaT = 360 - Math.abs(deltaT); 
		}

		if(deltaT > 180) {
			//Turn left
			turnLeft = true;
			//Rotate by
			deltaT = 360 - Math.abs(deltaT); 
		}
		else {
			turnLeft= false;
		}

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		if(turnLeft) {
			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, deltaT), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, deltaT), false);
		}
		else {
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, deltaT), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, deltaT), false);
		}
		
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
	}
	
	/**
	 * This method is created to let the robot rotate an absolute angle
	 * @param clockWise : direction to rotate robot
	 * @param angleToRotate : absolute angle to rotate robot
	 * @param blocked : True if the function is to be instantly returned. False if the function is to be returned after the travel is completed
	 */
	public static void rotateTheRobot(boolean clockWise, double angleToRotate, boolean blocked) {
		double absAngleToRotate = Math.abs(angleToRotate);
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		//just a reminder that if want the robot to rotate clockwise then leftMotor rotate is positive and right is negative
		//true means cw
		//false means ccw
		if(clockWise) {
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, absAngleToRotate), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, absAngleToRotate), blocked);
		}
		else{
			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, absAngleToRotate), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, absAngleToRotate), blocked);
		}
	}
	
	/**
	* This method is to stop both left and right motors
	*/
	public void stopMotors() {
		leftMotor.stop(true);
		rightMotor.stop(false);
	}
	  
	/**Advances the robot a desired amount of cm.
	* 
	* @param distanceToTravel Distance to travel in cm.
	* @param immediateReturn True if the function is to be instantly returned. False if the function is to be returned after the travel is completed.
	*/
	public void advanceRobot(double distanceToTravel, boolean immediateReturn) {

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(WHEEL_RAD, distanceToTravel), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, distanceToTravel), immediateReturn);
	  }

	/**
	 * Returns if the robot is navigating or not
	 * @return
	 */
	public boolean isNavigating() {
		return this.navigating;
	}
	
	public void lightCorrect() {
		boolean first = true;
		boolean leftStopped = false;
		boolean rightStopped = false;
		
		
		
		while(leftMotor.isMoving() || rightMotor.isMoving()) {
			
			SensorMode colourLeft;
			colourLeft = LSL.getRedMode();
		    float[] sampleLeft = new float[3];
		    colourLeft.fetchSample(sampleLeft, 0);
		    
		    SensorMode colourRight;
			colourRight = LSR.getRedMode();
		    float[] sampleRight = new float[3];
		    colourRight.fetchSample(sampleRight, 0);
		    
			if(TILE_FLOOR_COLOR - sampleLeft[0] > 0.30 && !leftStopped) {
				if(first) {
					leftMotor.stop(true);
					first = false;
				}
				else {
					leftMotor.stop(false);
				}
				leftStopped = true;
			}
				
			if(TILE_FLOOR_COLOR - sampleRight[0] > 0.30 && !rightStopped) {
				if(first) {
					rightMotor.stop(true);
					first = false;
				}
				else {
					rightMotor.stop(false);
				}
				rightStopped = true;
			}
		}
	}
	
	public void localizeForTunnel(double angle, double x, double y) {
		
		turnTo(angle, odo.getXYT()[2]);
		
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		
		leftMotor.forward();
		rightMotor.forward();
	
		lightCorrect();
		
		odo.setX(x * TILE_LENGTH);
		odo.setTheta(angle);
	
		leftMotor.rotate(convertDistance(WHEEL_RAD, -19.8), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, -19.8), false);
		
		turnTo(0, odo.getXYT()[2]);
		
		leftMotor.forward();
		rightMotor.forward();
		
		lightCorrect();
			
		odo.setTheta(0);
		odo.setY(y * TILE_LENGTH);
		
		leftMotor.setSpeed(2*FORWARD_SPEED);
		rightMotor.setSpeed(2*FORWARD_SPEED);
	}
	
	public void localizeAfterTunnel(double angle, double endX, double endY) {
		
		turnTo(angle, odo.getXYT()[2]);
		
		leftMotor.setSpeed(60);
		rightMotor.setSpeed(60);
		
		leftMotor.forward();
		rightMotor.forward();
	
		lightCorrect();
		
		odo.setX(endX * TILE_LENGTH);
		odo.setTheta(angle);
	
		leftMotor.rotate(convertDistance(WHEEL_RAD, -19.8), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, -19.8), false);
		
		turnTo(0, odo.getXYT()[2]);
		
		leftMotor.forward();
		rightMotor.forward();
		
		lightCorrect();
			
		odo.setTheta(0);
		odo.setY((endY + 1) * TILE_LENGTH);
		
	}
	
	public void shake() {
		/*
		for(int i = 0; i < 100; i++) {
			this.leftMotor.setSpeed(800);
			this.rightMotor.setSpeed(800);
			this.leftMotor.backward();
			this.rightMotor.forward();
		}
		for(int i = 0; i < 100; i++) {
			this.leftMotor.setSpeed(800);
			this.rightMotor.setSpeed(800);
			this.leftMotor.forward();
			this.rightMotor.backward();
		}
		for(int i = 0; i < 100; i++) {
			this.leftMotor.setSpeed(800);
			this.rightMotor.setSpeed(800);
			this.leftMotor.backward();
			this.rightMotor.forward();
		}
		for(int i = 0; i < 100; i++) {
			this.leftMotor.setSpeed(800);
			this.rightMotor.setSpeed(800);
			this.leftMotor.forward();
			this.rightMotor.backward();
		}
		*/
		
		leftMotor.setSpeed(1200);
		rightMotor.setSpeed(1200);

		leftMotor.rotate(convertDistance(WHEEL_RAD, 22), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, -25), false);
		
		leftMotor.setSpeed(1200);
		rightMotor.setSpeed(1200);

		leftMotor.rotate(convertDistance(WHEEL_RAD, -25), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, 25), false);
		
		
		
	}


	/**
	 * This method allows the conversion of a distance to the total rotation of each wheel need to
	 * cover that distance.
	 * 
	 * @param radius : radius of wheel
	 * @param distance : distance to travel
	 * @return
	 */
	public static int convertDistance(double radius, double travelDistance) {
		return (int) ((180.0 * travelDistance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	@Override
	public int readUSDistance() {
		return this.distance;
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
}