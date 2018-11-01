package ca.mcgill.ecse211.finalproject;
import lejos.hardware.Sound;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Gyro implements Runnable {

	private static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S4);
	
	private static int CORRECTION_PERIOD = 10;
	/**
	 * This is the default class constructor. An existing instance of the odometer is used. This is to
	 * ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public Gyro(){

	}

	/**
	 * Here is where the odometer correction code should be run.
	 * 
	 * @throws OdometerExceptions
	 */
	
	public void run() {
		final SampleProvider sp = gyroSensor.getAngleAndRateMode();
		int value = 0;
		int total = 0;
		long correctionStart, correctionEnd;
        while(true)
        {
			correctionStart = System.currentTimeMillis();
			
			float [] sample = new float[sp.sampleSize()];
            sp.fetchSample(sample, 0);
            value = (int)sample[0];

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
