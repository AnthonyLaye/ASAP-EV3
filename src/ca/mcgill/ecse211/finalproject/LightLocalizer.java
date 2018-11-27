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

/**
 * This class uses two light sensors to localize the robot to the closest grid line intersection
 *
 */
public class LightLocalizer {
		private static final long CORRECTION_PERIOD = 100;
		private Navigation Navigator;
		private EV3ColorSensor LSL;
		private EV3ColorSensor LSR;
		public static double xToMove, yToMove;//variables that we need to get or need to have
		private static final int ROTATE_SPEED = 100;
		private static final double wheelRadius = 2.2;
		private static final double track = 13.17;
		private static final int FORWARDSPEED = 300;
		private static Odometer odo;
		
		public LightLocalizer(Navigation Navigator, EV3ColorSensor LSL, EV3ColorSensor LSR, Odometer odo) throws OdometerExceptions {
			this.Navigator = Navigator;
			this.LSL = LSL;
			this.LSR = LSR;
			this.odo = odo;
		}

		/**
		 * This method drives the robot forward until it it sees a first black line, then rotates to find the next one. It stops at the intersection
		 * and updates the x, y and theta coordinates.
		 * @param startX : x coordinate value at first grid line intersection
		 * @param startY : y coordinate value at first grid line intersection
		 * @param startAngle : the direction the robot will be facing based on the starting corner
		 */
		public void lightLocalize(int startX, int startY, int startAngle) {
			
			try {
				Thread.sleep(700);
			} catch (InterruptedException e) {
			}
			
			long correctionStart, correctionEnd;
			int count = 0;
			
			SensorMode colourLeft;
			colourLeft = LSL.getRedMode();
		    float[] sampleLeft = new float[3];
		    colourLeft.fetchSample(sampleLeft, 0);
		    
		    SensorMode colourRight;
			colourRight = LSR.getRedMode();
		    float[] sampleRight = new float[3];
		    colourRight.fetchSample(sampleRight, 0);
		    
		    Navigation.TILE_FLOOR_COLOR = sampleLeft[0];	// Save the color of the tile floor to be used for comparisons
		    
		    //Navigation.FORWARD_SPEED = 270;
		    
		    Navigation.leftMotor.forward();
		    Navigation.rightMotor.forward();
		    Navigator.lightCorrect();
		    
		    advanceRobot(-5, false);
			
			rotateTheRobot(true,90,false);
			
			Navigation.leftMotor.forward();
		    Navigation.rightMotor.forward();
			Navigator.lightCorrect();
		    
		    advanceRobot(-5, false);
		    
			
			/*while (true) {
				
				correctionStart = System.currentTimeMillis();
				
				colourLeft = LSL.getRedMode();
				sampleLeft = new float[3];
			    colourLeft.fetchSample(sampleLeft, 0);
			    
			    colourRight = LSR.getRedMode();
			    sampleRight = new float[3];
			    colourRight.fetchSample(sampleRight, 0);
			    
				advanceRobot(50,true);
				int counter = 0;
				while(Navigation.TILE_FLOOR_COLOR - sampleLeft[0] < 0.30  || Navigation.TILE_FLOOR_COLOR -  sampleRight[0] < 0.30) {	// While no difference between colors
					
					colourLeft = LSL.getRedMode();
					sampleLeft = new float[3];
				    colourLeft.fetchSample(sampleLeft, 0);
					
					colourRight = LSR.getRedMode();
				    sampleRight = new float[3];
				    colourRight.fetchSample(sampleRight, 0);

					if (Navigation.TILE_FLOOR_COLOR - sampleRight[0] > 0.30) {
						Sound.beep();
						Navigation.rightMotor.stop(true);
						while(Navigation.TILE_FLOOR_COLOR - sampleLeft[0] < 0.3)//should be < 0.30
						//while(true)
						{
							colourLeft = LSL.getRedMode();
							sampleLeft = new float[3];
						    colourLeft.fetchSample(sampleLeft, 0);
						    counter++;
						    if(counter == 50000)
						    {
						    		Navigation.leftMotor.setSpeed(100);
						    		Navigation.leftMotor.backward();
						    		while(Navigation.TILE_FLOOR_COLOR - sampleLeft[0] < 0.30)
						    		{
									colourLeft = LSL.getRedMode();
									sampleLeft = new float[3];
									colourLeft.fetchSample(sampleLeft, 0);
						    		}
						    		break;
						    }
						    //counter = 0;
						}
						counter = 0;
						break;
					}
					if (Navigation.TILE_FLOOR_COLOR - sampleLeft[0] > 0.30) {
						Sound.beep();
						Navigation.leftMotor.stop(true);
						while(Navigation.TILE_FLOOR_COLOR - sampleRight[0] < 0.3)//should be < 0.30
						//while(true)
						{
							colourRight = LSR.getRedMode();
						    sampleRight = new float[3];
						    colourRight.fetchSample(sampleRight, 0);
						    counter++;
						    if(counter == 50000)
						    {
						    		Navigation.rightMotor.setSpeed(100);
						    		Navigation.rightMotor.backward();
						    		while(Navigation.TILE_FLOOR_COLOR - sampleLeft[0] < 0.30)
						    		{
									colourLeft = LSL.getRedMode();
									sampleLeft = new float[3];
									colourLeft.fetchSample(sampleLeft, 0);
						    		}
						    		break;
						    }
						    //counter = 0;
						}
						counter = 0;
						break;
					}
				}
				
				stopMotor();
				
				count++;
				
				if(count == 2)
				{
					advanceRobot(-5, false);
					break;
				}
				
				advanceRobot(-5, false);
				
				rotateTheRobot(true,90,false);
				
				correctionEnd = System.currentTimeMillis();
				if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
					try {
						Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
					} catch (InterruptedException e) {
						// there is nothing to be done here
					}
				}
			}*/
			
			odo.setX(startX * 30.48);
			odo.setTheta(startAngle);
			odo.setY(startY * 30.48);
			
			Navigation.FORWARD_SPEED = 200;
			
		}
		
		
	  /**
	   * This method is created to let the robot rotate a certain angle
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
	  }
	  
	  /**
	   * This method allows the conversion of a distance to the total rotation of each wheel need to
	   * cover that distance.
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
	   * This method is to stop both motors
	   */
	  private void stopMotor() {
		  // TODO Auto-generated method stub
		  Navigator.leftMotor.setSpeed(0);
		  Navigator.rightMotor.setSpeed(0);	
	  }
	  
	  /**Advances the robot a desired amount of cm
	   * @param distanceToTravel : Distance to travel in cm.
	   * @param instantReturn : True if the function is to be instantly returned. False if the function is to be returned after the travel is completed.
	   */
	   public void advanceRobot(double distanceToTravel, boolean instantReturn) {
	     
	     Navigator.leftMotor.setSpeed(FORWARDSPEED);
	     Navigator.rightMotor.setSpeed(FORWARDSPEED);
	              
	     Navigator.leftMotor.rotate(convertDistance(wheelRadius, distanceToTravel), true);
	     Navigator.rightMotor.rotate(convertDistance(wheelRadius, distanceToTravel), instantReturn);
	     
	   }
	   
	   /**
	    * This method updates the x and y coordinates of the robot without doing localization - used only for testing!
	    */
	   public void pollColour() {
		  
		  odo.setX(7 * 30.48);
		  odo.setY(30.48);
		  odo.setTheta(270);
			
		  SensorMode colourLeft;
		  colourLeft = LSL.getRedMode();
		  float[] sampleLeft = new float[3];
		  colourLeft.fetchSample(sampleLeft, 0);
			
		  Navigation.TILE_FLOOR_COLOR = sampleLeft[0];	// Save the color of the tile floor to be used for comparisons
	   }
}