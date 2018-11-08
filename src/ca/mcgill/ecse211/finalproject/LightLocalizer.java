package ca.mcgill.ecse211.finalproject;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.port.UARTPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

public class LightLocalizer {
	private static final double LS_OFFSET = -13.6; //Distance from LS to center of rotation in cm. //negative because of light sensor is at back
	private static final long CORRECTION_PERIOD = 100;
	  private Navigation Navigator;
	  private EV3ColorSensor LS;
	  private Odometer odo;
	  public static double xToMove, yToMove;//variables that we need to get or need to have
		private static float color = 0;
		//private static final Port portColor = LocalEV3.get().getPort("S3");
		private static final int ROTATE_SPEED = 100;
		private static final double wheelRadius = 2.2;
		private static final double track = 11.3;
		private static final int FORWARDSPEED = 100;
		private static final double TILE_LENGTH = 30.48;
		//private static SensorModes myColor = new EV3ColorSensor(LS);
		
		public LightLocalizer(Navigation Navigator, EV3ColorSensor lightSensor) throws OdometerExceptions {
			this.Navigator = Navigator;
			this.LS = lightSensor;
			this.odo = Odometer.getOdometer();
		}

		/**
		 * This method is implemented to let the robot first detect 4 black lines in order to detect four angles
		 * using these four angles to get a and b
		 * then get x and y to move forward
		 */
		public void lightLocalize() {
			//LS.getMode("Red")
			//rotateTheRobot(true,45,false);//rotate 45degree cw
			//advanceRobot(-10,false);//first advance the robot to avoid it detect nothing, according to the length of our robot, 10 is all good it is half of our robot length 
			//it is not hard code because our robot is in the first square after finishing the first task, so move for half of robot length will finish the second task
			//because the whole square is a 30.48*30.48 square, our robot is 23 cm add 10 cm is longer than the length of the side of the square and because of that 
			//this move will not let whole robot go out the square.
			//rotateTheRobot(false,45,false);//rotate 45 degree ccw to go back to the position
			//SampleProvider myColorSample = LS.getColorIDMode();
			//SensorMode color = LS.getColorIDMode();
			SampleProvider myColorSample = LS.getMode("Red");
			float[] sampleColor = new float[LS.sampleSize()];
			
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				
			}
			long correctionStart, correctionEnd;
			int count = 0;
			
			advanceRobot(-50,true);
			
			while (true) {
				myColorSample.fetchSample(sampleColor, 0); 
				color = sampleColor[0] * 1000; 
				
				correctionStart = System.currentTimeMillis();
				advanceRobot(-50,true);
				
				//When a line is detected, go backwards for a distance equal to the distance between the light sensor and the center of the robot
				while ( LS.getColorID() != 13 ) 
				{ 
					
					
				}
				
				Sound.beep();
				
				stopMotor();
				
				//advanceRobot(10,false);//back the robot 10 cm
				
				//rotateTheRobot(true,90,false);
				
				count++;
				
				if(count == 2)
				{
					advanceRobot(6,false);
					rotateTheRobot(false, 90, false);
					advanceRobot(-3,false);//back the robot -10 cm
					break;
				}
				
				advanceRobot(10,false);//back the robot 10 cm
				stopMotor();
				rotateTheRobot(true,95,false);
				
				odo.setXYT(7 * TILE_LENGTH, TILE_LENGTH, 0);
				
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
		
		
		  /**
		   * this method is created to let the robot rotate certain angle
		   * @param cwOrCcw : define clockwise or counterclockwise direction
		   * @param angleToRotate : angle to rotate
		   */
		  private void rotateTheRobot(boolean cwOrCcw, double angleToRotate, boolean blocked) {
			  double absAngleToRotate = Math.abs(angleToRotate);
			  Navigation.leftMotor.setSpeed(ROTATE_SPEED);
			  Navigation.rightMotor.setSpeed(ROTATE_SPEED);
			  
			  //just a reminder that if want the robot to rotate clockwise then leftMotor rotate is positive and right is negative
			  //true means cw
			  //false means ccw
			  if(cwOrCcw) {
				  Navigation.leftMotor.rotate(convertAngle(wheelRadius, track, absAngleToRotate),true);
				  Navigation.rightMotor.rotate(-convertAngle(wheelRadius, track, absAngleToRotate),blocked);
			  }
			  else
			  {
				  Navigation.leftMotor.rotate(-convertAngle(wheelRadius, track, absAngleToRotate),true);
				  Navigation.rightMotor.rotate(convertAngle(wheelRadius, track, absAngleToRotate),blocked);
			  }
			  // TODO Auto-generated method stub
		  }
		  
		  /**
		   * This method allows the conversion of a distance to the total rotation of each wheel need to
		   * cover that distance.
		   * 
		   * @param radius : radius of wheels
		   * @param distance : distance to travel
		   * @return
		   */
		  private static int convertDistance(double radius, double travelDistance) {
			  return (int) ((180.0 * travelDistance) / (Math.PI * radius));
		  }

		  private static int convertAngle(double radius, double width, double angle) {
			  return convertDistance(radius, Math.PI * width * angle / 360.0);
		  }  

		  /**
		   * this method is to stop the motor ,both of them, left and right
		   */
		  private void stopMotor() {
			  // TODO Auto-generated method stub
			  Navigator.leftMotor.setSpeed(0);
			  Navigator.rightMotor.setSpeed(0);	
		  }
		  
		  /**Advances the robot a desired amount of cm.
		   * 
		   * @param distanceToTravel : Distance to travel in cm.
		   * @param instantReturn : True if the function is to be instantly returned. False if the function is to be returned after the travel is completed.
		   */
		   public void advanceRobot(double distanceToTravel, boolean instantReturn) {
		     
		     Navigator.leftMotor.setSpeed(FORWARDSPEED);
		     Navigator.rightMotor.setSpeed(FORWARDSPEED);
		              
		     Navigator.leftMotor.rotate(convertDistance(wheelRadius, distanceToTravel), true);
		     Navigator.rightMotor.rotate(convertDistance(wheelRadius, distanceToTravel), instantReturn);
		     
		   }
}
