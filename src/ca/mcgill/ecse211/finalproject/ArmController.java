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
	 * Slowly close the arms together to grasp a ring
	 */
	public void closeArms() {
		leftArmMotor.setSpeed(150);
		rightArmMotor.setSpeed(150);
		leftArmMotor.rotate(70, true);
		rightArmMotor.rotate(70, false);
	}
	
	/**
	 * Open up arms wide enough to begin collecting rim
	 */
	public void openArms() {
		leftArmMotor.setSpeed(150);
		rightArmMotor.setSpeed(150);
		leftArmMotor.rotate(-70, true);
		rightArmMotor.rotate(-70, false);
	}
}