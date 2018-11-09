package ca.mcgill.ecse211.finalproject;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class UltrasonicLocalizer1 {
	
	public enum Localization { FALLING_EDGE, RISING_EDGE };
	private Odometer odometer;
	private Localization localization;
	private int filterControl;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Navigation navigation;
	
	private static final int ROTATE_SPEED = 100;
	private static final int FILTER_OUT = 20;
	private static final double WHEEL_RAD = 2.2;
	private static final double TRACK = 11.3;
	private static final int EDGE_TRACKER = 46;
	private static final int ERROR_NOISE = 5;
	private int TEST = 0;
	
	public UltrasonicLocalizer1(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Localization localization, Navigation simpleNavigation) {
		this.odometer = odometer;
		this.localization = localization;
		this.navigation = simpleNavigation;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}
	
	/**
	 * This method uses either Falling, or Rising edge localization to orient the robot towards the true 0 degrees.
	 */
	
	public void localize() {
		
		double firstAngle = 0, secondAngle = 0;
		
		if(this.localization == Localization.RISING_EDGE) {	//Rising edge calculation
			
			try {
				Thread.sleep(3000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			while(navigation.distance > 35) {	//Move towards wall to start
				leftMotor.setSpeed(ROTATE_SPEED);
			    rightMotor.setSpeed(ROTATE_SPEED);
			    leftMotor.forward();
			    rightMotor.backward();
			}
			
			leftMotor.setSpeed(ROTATE_SPEED);
		    rightMotor.setSpeed(ROTATE_SPEED);
		    leftMotor.forward();
		    rightMotor.backward();
			
			while(navigation.distance < EDGE_TRACKER + ERROR_NOISE) {	//Clear robot away from wall
				this.TEST = 0; //Line does nothing, it is just to keep the while loop busy
			}
			
			leftMotor.stop(true);
			rightMotor.stop(true);
			Sound.beep();
			
			firstAngle = this.odometer.getXYT()[2];	//First time robot cleared wall
			leftMotor.backward();
			rightMotor.forward();
	
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			while(navigation.distance < EDGE_TRACKER + ERROR_NOISE) {
				this.TEST = 0;
			}
			
			leftMotor.stop(true);
			rightMotor.stop(true);
			Sound.beep(); //We have reached second point
			
			secondAngle = this.odometer.getXYT()[2];
			double deltaT = 0;
			if(firstAngle < secondAngle){	//Find change in angle based on formula in tutorials
				deltaT = 225 - ((firstAngle + secondAngle) / 2);
			}
			else {
				deltaT= 45 - ((firstAngle + secondAngle) / 2);
			}
			
			this.odometer.setTheta(this.odometer.getXYT()[2] + deltaT - 180);
			navigation.turnTo(0, this.odometer.getXYT()[2]);
				
		}
		else if(this.localization == Localization.FALLING_EDGE) {
			
			try {
				Thread.sleep(3000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			while(navigation.distance < 200) {	//Move away from wall to start
				leftMotor.setSpeed(ROTATE_SPEED);
			    rightMotor.setSpeed(ROTATE_SPEED);
			    leftMotor.forward();
			    rightMotor.backward();
			}
			
			leftMotor.setSpeed(ROTATE_SPEED);
		    rightMotor.setSpeed(ROTATE_SPEED);
		    leftMotor.forward();
		    rightMotor.backward();
		    
		    try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			while(navigation.distance >= EDGE_TRACKER + ERROR_NOISE) {	//Move robot towards wall
				this.TEST = 0;
			}
			
			leftMotor.stop(true);
			rightMotor.stop(true);
			Sound.beep();
			
			firstAngle = this.odometer.getXYT()[2];	//First instance of falling edge
			leftMotor.backward();
			rightMotor.forward();
			
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			while(navigation.distance > EDGE_TRACKER + ERROR_NOISE) {
				this.TEST = 0;
			}
			
			leftMotor.stop(true);
			rightMotor.stop(true);
			Sound.beep(); //We have reached second point
			
			secondAngle = this.odometer.getXYT()[2];
			double deltaT = 0;
			if(firstAngle < secondAngle){
				deltaT = 225 - ((firstAngle + secondAngle)/2);
			}
			else {
				deltaT= 45 - ((firstAngle + secondAngle)/2);
			}
			
			this.odometer.setTheta(this.odometer.getXYT()[2] + deltaT);
			navigation.turnTo(0, this.odometer.getXYT()[2]);
		}
	}
}
