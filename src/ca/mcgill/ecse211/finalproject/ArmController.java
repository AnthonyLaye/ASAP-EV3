package ca.mcgill.ecse211.finalproject;

import lejos.hardware.motor.NXTRegulatedMotor;

/**
 * This class is used to control the robot's two arms
 *
 */
public class ArmController {
	public NXTRegulatedMotor leftArmMotor;
	public NXTRegulatedMotor rightArmMotor;
	
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
		
		rightArmMotor.rotate(80, true);
		leftArmMotor.rotate(80, false);
	}
	
	/**
	 * Open up arms wide enough to begin collecting rim
	 */
	public void openArms() {
		leftArmMotor.setSpeed(100);
		rightArmMotor.setSpeed(100);
		
		rightArmMotor.rotate(-80, true);
		leftArmMotor.rotate(-80, false);
	}
	
	/**
	 * Rotate the arms by a specified angle, slowly!
	 * @param angle : angle to rotate arms
	 */
	public void rotateArms(int angle) {
		leftArmMotor.setSpeed(8);
		rightArmMotor.setSpeed(8);
		
		leftArmMotor.rotate(angle, true);
		rightArmMotor.rotate(angle, true);
	}
}