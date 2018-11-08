package ca.mcgill.ecse211.finalproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class prepares for, and executes tunnel traversal
 *
 */
public class TunnelFollower {
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Odometer odo;
	private Navigation navigation;
	private ArmController armController;
	private static final int FORWARD_SPEED = 150;
	
	
	public TunnelFollower(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Navigation navigation, Odometer odometer, ArmController armController) throws OdometerExceptions {
		this.navigation = navigation;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odo = odometer;
		this.armController = armController;
	}
	
	/**
	 * This method prepares the robot to traverse the tunnel.
	 * It may raise or lower the arms to ensure safe passage
	 * @param startX : x position before tunnel
	 * @param startY : y position before tunnel
	 * @param endX : x position after tunnel
	 * @param endY : y position after tunnel
	 */
	public void traverseTunnel(double startX, double startY, double endX, double endY) {
		
		armController.closeArms();
		
		Navigation.leftMotor.setSpeed(FORWARD_SPEED * 2); //TURBO MODE TO BLOW PAST THIS TUNNEL
		Navigation.rightMotor.setSpeed(FORWARD_SPEED * 2); //TURBO MODE TO BLOW PAST THIS TUNNEL
		
		navigation.travelTo(startX + 0.5, startY - 0.5, false);	//Offset values by 0.5 so we are lined up with center of tunnel
		
		navigation.travelTo(endX - 0.5, endY + 0.5, false);
		
		Navigation.leftMotor.setSpeed(FORWARD_SPEED); //TURBO MODE TO BLOW PAST THIS TUNNEL
		Navigation.rightMotor.setSpeed(FORWARD_SPEED); //TURBO MODE TO BLOW PAST THIS TUNNEL
		
		armController.openArms();
	}

}
