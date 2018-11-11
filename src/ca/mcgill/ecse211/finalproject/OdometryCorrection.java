/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.finalproject;



import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;
	private double squareSideLen = 30.48;
	
	private static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S4);
	//static SampleProvider myColorSample = lightSensor.getRedMode();
	//float [] sample = new float[myColorSample.sampleSize()];

	/**
	 * This is the default class constructor. An existing instance of the odometer is used. This is to
	 * ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection() throws OdometerExceptions {

		this.odometer = Odometer.getOdometer();


	}

	/**
	 * Here is where the odometer correction code should be run.
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	//below is our method for odometry correction implemented by Group37
	//the main task is to use the side length of the square in the lab which is 30.48cm as a reference to measure where the car is (in a corrdinate form)
	//need to pay attention to the fact that every time the color sensor detect a black line does not mean that the center of the rotation of the wheel 
	//is on the black line
	//need to minus the offset of the light sensor to the x-axis and y-axis.
	//we first got the offset between the light sensor and the center of rotation then it is easy to calculate the offset between light sensor and x-axis (lightSensorDis*cos(theta))
	//and y-axis (lightSensorDis*sin(theta))
	public void run() {
		final SampleProvider sp = gyroSensor.getAngleAndRateMode();
		int value = 0;
		double xpositionBefore = 0;
		double ypositionBefore = 0;
		xpositionBefore = odometer.getXYT()[0];
		ypositionBefore = odometer.getXYT()[1];
		long correctionStart, correctionEnd;
		
		while (true) {
			correctionStart = System.currentTimeMillis();
			
			//get the sample
			float [] sample = new float[sp.sampleSize()];
            sp.fetchSample(sample, 0);
            value = (int)sample[0];
			//
			if ( odometer.getXYT()[0] - xpositionBefore >= squareSideLen || odometer.getXYT()[1] - ypositionBefore >= squareSideLen) { //we find a black line
				
				
				xpositionBefore = odometer.getXYT()[0];
				ypositionBefore = odometer.getXYT()[1];
				double theta = odometer.getXYT()[2]; // we need to get our theta to find offset to X axis and offset to Y axis
				
				if (value > theta) {
					Navigation.rotateTheRobot(false, value - theta, false);
				}
				else
				{
					Navigation.rotateTheRobot(true, theta - value, false);
				}
				
			}

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
	}
}
