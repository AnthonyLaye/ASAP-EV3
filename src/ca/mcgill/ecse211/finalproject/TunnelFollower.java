package ca.mcgill.ecse211.finalproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class TunnelFollower {
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Odometer odo;
	private Navigation navigation;
	
	
	public TunnelFollower(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Navigation navigation, Odometer odometer) throws OdometerExceptions {
		this.navigation = navigation;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odo = odometer;
	}
	
	/**
	 * This method prepares the robot to traverse the tunnel.
	 * It may raise or lower the arms to ensure safe passage
	 * @param startX
	 * @param startY
	 * @param endX
	 * @param endY
	 */
	public void traverseTunnel(double startX, double startY, double endX, double endY) {
		
		
	}

}
