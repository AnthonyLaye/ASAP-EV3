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
		leftArmMotor.setSpeed(120);
		rightArmMotor.setSpeed(120);
		leftArmMotor.rotate(80, true);
		rightArmMotor.rotate(80, false);
		leftArmMotor.stop(true);
		rightArmMotor.stop(false);
	}
	
	/**
	 * Open up arms wide enough to begin collecting rim
	 */
	public void openArms() {
		leftArmMotor.setSpeed(120);
		rightArmMotor.setSpeed(120);
		leftArmMotor.rotate(-80, true);
		rightArmMotor.rotate(-80, false);
		leftArmMotor.stop(true);
		rightArmMotor.stop(false);
	}
	
	public void rotateArms(int angle) {
		leftArmMotor.setSpeed(120);
		rightArmMotor.setSpeed(120);
		leftArmMotor.rotate(angle, true);
		rightArmMotor.rotate(angle, false);
		leftArmMotor.stop(true);
		rightArmMotor.stop(false);
	}
}