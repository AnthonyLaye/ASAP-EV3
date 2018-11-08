/*
 * Navigation.java
 */
package ca.mcgill.ecse211.finalproject;


import lejos.hardware.motor.EV3LargeRegulatedMotor;

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
	private static final double WHEEL_RAD = 2.2;
	private static final double TRACK = 12.5;
	public static final double TILE_LENGTH = 30.78;

	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) throws OdometerExceptions {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odo = odometer;
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
	}

	/**
	 * This method is meant to drive the robot to the predefined waypoints, user must enter if we wait for the robot to finish moving before
	 * return from the method
	 * @param x : x value to travel to
	 * @param y : y value to travel to
	 * @param immediateReturn : True if the function is to be instantly returned. False if the function is to be returned after the travel is completed
	 */
	public void travelTo(double x, double y, boolean immediateReturn) {
		double minimalTheta = 0, travelDistance = 0, currentTheta = 0;
		double currentX = 0.0;
		double currentY = 0.0;
		double odometer[] = { 0, 0, 0 };

		// Get odometer readings
		try {
			odometer = Odometer.getOdometer().getXYT();
		} catch (OdometerExceptions e) {
			// Do nothing
			e.printStackTrace();
		}

		// Set odometer reading angle as prev angle as well
		currentTheta = odometer[2];

		// Get displacement to travel on X and Y axis
		currentX = odometer[0];
		currentY = odometer[1];

		//Getting the distances with respect to the tile size
		double deltaX = x * TILE_SIZE - currentX;
		double deltaY =	y * TILE_SIZE - currentY;
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
		deltaT = minimalT*1.15 - original;
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
	public void rotateTheRobot(boolean clockWise, double angleToRotate, boolean blocked) {
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