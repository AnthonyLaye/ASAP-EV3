package ca.mcgill.ecse211.wallfollowing;




import lejos.hardware.Sound;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class OdometryCorrectionGyro implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;
	private double squareSideLen = 30.48;
	double initialY = 0;
	double initialX = 0;
	int countx = 0;
	int county = 0;
	int count = 0;
	
	private static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S3);
	
	private static int HALF_SECOND = 500;
	/**
	 * This is the default class constructor. An existing instance of the odometer is used. This is to
	 * ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrectionGyro() throws OdometerExceptions {

		this.odometer = Odometer.getOdometer();
		
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

			//System.out.println(""  + value);
			odometer.setTheta(value);

            //Delay.msDelay(HALF_SECOND);
            
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
