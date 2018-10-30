package ca.mcgill.ecse211.finalproject;

import lejos.hardware.motor.EV3MediumRegulatedMotor;

/**
 * This class is used to control the robot's two arms
 *
 */
public class ArmController {
	private EV3MediumRegulatedMotor leftArmMotor;
	private EV3MediumRegulatedMotor rightArmMotor;
	
	public ArmController(EV3MediumRegulatedMotor leftArmMotor, EV3MediumRegulatedMotor rightArmMotor) {
		this.leftArmMotor = leftArmMotor;
		this.rightArmMotor = rightArmMotor;
	}
	
	
	/**
	 * Make the robot arms move upwards or downwards by a specified distance
	 * @param distance : distance to move vertically
	 */
	public void moveArmVertically(int distance) {
		
	}
	
	/**
	 * Slowly close the arms together to grasp a ring
	 */
	public void closeArms() {
		
	}
	
	/**
	 * Open up arms wide enough to begin collecting rim
	 */
	public void openArms() {
		
	}
}