package ca.mcgill.ecse211.finalproject;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.port.UARTPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class is used to localize the robot to 0 degrees using light localization
 *
 */
public class LightLocalizer {
	private static final double LS_OFFSET = -13.6; //Distance from LS to center of rotation in cm. //negative because of light sensor is at back
	private static final long CORRECTION_PERIOD = 1000;
	private Navigation navigation;
	private EV3ColorSensor LS;
	private Odometer odo;
	private static float color;
	public static double xToMove, yToMove;//variables that we need to get or need to have
		
	public LightLocalizer(Navigation navigation, Odometer odometer, EV3ColorSensor lightSensor) throws OdometerExceptions {
		this.navigation = navigation;
		this.LS = lightSensor;
		this.odo = odometer;
	}

	/**
	 * This method is implemented to let the robot first detect 4 black lines in order to detect four angles
	 * which can be used get the x and y coordinates to move forward
	 */
	public void lightLocalize() {
		navigation.rotateTheRobot(true, 45, false);//rotate 45degree cw
			
		navigation.advanceRobot(10, false);//first advance the robot to avoid it detect nothing, according to the length of our robot, 10 is all good it is half of our robot length 
		//it is not hard code because our robot is in the first square after finishing the first task, so move for half of robot length will finish the second task
		//because the whole square is a 30.48*30.48 square, our robot is 23 cm add 10 cm is longer than the length of the side of the square and because of that 
		//this move will not let whole robot go out the square.
		navigation.rotateTheRobot(false, 45, false);//rotate 45 degree ccw to go back to the position
		SampleProvider myColorSample = LS.getMode("Red");
		float[] sampleColor = new float[LS.sampleSize()];
		myColorSample.fetchSample(sampleColor, 0);
		color = sampleColor[0]*1000;
			
		double theta[] = new double [4];

		double a = 0;
		double b = 0;

			
		long correctionStart, correctionEnd;
		int count = 0;
		myColorSample.fetchSample(sampleColor, 0);
		color = sampleColor[0]*1000;
			
		navigation.rotateTheRobot(false, 90 , true);//first let it roll, 90 degree is sufficient for detecting the first black line
		while (true) {//while i did not detect enough data i.e enough theta
			correctionStart = System.currentTimeMillis();
			myColorSample.fetchSample(sampleColor, 0);
			color = sampleColor[0]*1000;
			navigation.rotateTheRobot(false, 90 , true);
			while(LS.getColorID() != 13)//waiting for detecting a black line
			{
				
			}
				
			Sound.beep();//sound beep to let user know it have detect a rising edge
			navigation.stopMotors();
			theta[count] = odo.getXYT()[2];//get theta1
			count++;
			navigation.rotateTheRobot(false,10,false);//avoid detect same line twice
									
			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
				
			if(count == 4) {
					//now compute a, b and r
				a = theta[2] - theta[0];//this angle is for calculating for x
				b = theta[3] - theta[2] - theta[1];//this angle is for calculating for y
					
				navigation.rotateTheRobot(false, theta[3] - 10, false);//back to the direction of where it begins, why i use theta[3] is because it is a geometric symmetry
														   //minus 10 is because i first turn 10 degree to avoid double detection of black line
				xToMove = - LS_OFFSET*Math.cos((a/2)*Math.PI/180);
				yToMove = LS_OFFSET*Math.cos((b/2)*Math.PI/180);
				
				navigation.advanceRobot(yToMove-7,false);
				navigation.rotateTheRobot(true, 90, false);//now change x position
				navigation.advanceRobot(xToMove,false);
				navigation.rotateTheRobot(false, 90, false);//back to the direction of where it begins
				break;
			}
		}
		
	}
}