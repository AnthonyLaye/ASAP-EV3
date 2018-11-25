package ca.mcgill.ecse211.finalproject;

import lejos.hardware.motor.NXTRegulatedMotor;

/**
 * This class is used to control the robot's two arms
 *
 */
public class ArmController {
	private NXTRegulatedMotor leftArmMotor;
	private NXTRegulatedMotor rightArmMotor;
	
	public ArmController(NXTRegulatedMotor leftArmMotor, NXTRegulatedMotor rightArmMotor) {
		this.leftArmMotor = leftArmMotor;
		this.rightArmMotor = rightArmMotor;
	}
		
	/**
	 * Slowly close the arms together to grasp a ring
	 */
	public void closeArms() {
		leftArmMotor.setSpeed(100);
		rightArmMotor.setSpeed(100);
		
		rightArmMotor.rotate(70, true);
		leftArmMotor.rotate(70, false);
	}
	
	/**
	 * Open up arms wide enough to begin collecting rim
	 */
	public void openArms() {
		leftArmMotor.setSpeed(100);
		rightArmMotor.setSpeed(100);
		
		rightArmMotor.rotate(-70, true);
		leftArmMotor.rotate(-70, false);
	}
	
	/**
	 * Rotate the arms by a specified angle
	 * @param angle : angle to rotate arms
	 */
	public void rotateArms(int angle) {
		leftArmMotor.setSpeed(120);
		rightArmMotor.setSpeed(120);
		
		leftArmMotor.rotate(angle, true);
		rightArmMotor.rotate(angle, true);
	}
}