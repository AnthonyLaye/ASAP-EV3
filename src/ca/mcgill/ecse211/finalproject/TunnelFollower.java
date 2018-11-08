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
		
		double[] newValues = calculateTunnelEntry(startX, startY, endX, endY);
		
		navigation.travelTo(newValues[0], newValues[1], false);	//Offset values by 0.5 so we are lined up with center of tunnel
		
		navigation.travelTo(newValues[2], newValues[3], false);
		
		Navigation.leftMotor.setSpeed(FORWARD_SPEED); // Back to normal speed
		Navigation.rightMotor.setSpeed(FORWARD_SPEED); 
		
		armController.openArms();
	}
	
	/**
	 * This method calculates where to stop in front of the tunnel based on the coordinates.
	 * @param startX : lower left x value of tunnel
	 * @param startY : lower left y value of tunnel
	 * @param endX : upper right x value of tunnel
	 * @param endY : upper right y value of tunnel
	 * @return
	 */
	private double[] calculateTunnelEntry(double startX, double startY, double endX, double endY) {
		double[] offsetValues = new double[4];
		
		//DIFFERENT CASES FOR TUNNEL ALIGNMENT
		
		//CASE 1: Difference in Y values is 2 -> tunnel is vertical
		if(Math.abs(endY - startY) == 2) {
			if(endY > startY) {	// We are approaching from the bottom
				offsetValues[0] = startX + 0.5;
				offsetValues[1] = startY - 0.5;
				offsetValues[2] = endX - 0.5;
				offsetValues[3] = endY + 0.5;
			}
			else if(startY > endY) {	// We are approaching from the top
				offsetValues[0] = startX - 0.5;
				offsetValues[1] = startY + 0.5;
				offsetValues[2] = endX + 0.5;
				offsetValues[3] = endY - 0.5;
			}
		}
		//CASE 2: Difference in Y values is 1 -> tunnel is horizontal
		else if(Math.abs(endY - startY) == 1) {
			if(endY > startY) {	// Approaching from the left
				offsetValues[0] = startX - 0.5;
				offsetValues[1] = startY + 0.5;
				offsetValues[2] = endX + 0.5;
				offsetValues[3] = endY - 0.5;
			}
			else if(startY > endY) {	// We are approaching from the right
				offsetValues[0] = startX + 0.5;
				offsetValues[1] = startY - 0.5;
				offsetValues[2] = endX - 0.5;
				offsetValues[3] = endY + 0.5;
			}
		}
		else {
			offsetValues[0] = startX;
			offsetValues[1] = startY;
			offsetValues[2] = endX;
			offsetValues[3] = endY;
		}
		
		return offsetValues;
	}

}
