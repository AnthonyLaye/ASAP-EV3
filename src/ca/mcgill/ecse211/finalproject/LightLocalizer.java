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
		private static final long CORRECTION_PERIOD = 100;
		private Navigation Navigator;
		private EV3ColorSensor LSL;
		private EV3ColorSensor LSR;
		public static double xToMove, yToMove;//variables that we need to get or need to have
		//private static final Port portColor = LocalEV3.get().getPort("S3");
		private static final int ROTATE_SPEED = 100;
		private static final double wheelRadius = 2.2;
		private static final double track = 12.5;
		private static final int FORWARDSPEED = 100;
		//private static SensorModes myColor = new EV3ColorSensor(LS);
		private SensorData data;
		private static Odometer odo;
		private static final int blackLineColor = -5;
		
		public LightLocalizer(Navigation Navigator, EV3ColorSensor LSL, EV3ColorSensor LSR) throws OdometerExceptions {
			this.Navigator = Navigator;
			this.LSL = LSL;//LS1 is the left one ultra is face against you
			this.LSR = LSR;//LS2 is the right one, ultra is face against you
		    this.data = SensorData.getSensorData();
		    this.odo = Odometer.getOdometer();
		}

		/**
		 * This method is implemented to let the robot first detect 4 black lines in order to detect four angles
		 * using these four angles to get a and b
		 * then get x and y to move forward
		 */
		public void lightLocalize() {
			//advanceRobot(-10,false);//first advance the robot to avoid it detect nothing, according to the length of our robot, 10 is all good it is half of our robot length 
			//it is not hard code because our robot is in the first square after finishing the first task, so move for half of robot length will finish the second task
			//because the whole square is a 30.48*30.48 square, our robot is 23 cm add 10 cm is longer than the length of the side of the square and because of that 
			//this move will not let whole robot go out the square.
		
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
			}
			long correctionStart, correctionEnd;
			int count = 0;
		   /* Navigation.leftMotor.forward();
		    Navigation. rightMotor.forward();
		    while (Navigation.leftMotor.isMoving() || Navigation.rightMotor.isMoving()) {
		      if (data.getL()[0] < blackLineColor) {
		    	  	Navigation.leftMotor.stop(true);
		      }
		      if (data.getL()[1] < blackLineColor) {
		    	  	Navigation.rightMotor.stop(true);
		      }
		    }
		    
		    rotateTheRobot(true, 90, false);
		    
		    Navigation.leftMotor.setSpeed(100);
		    Navigation.rightMotor.setSpeed(100);
		    Navigation.leftMotor.forward();
		    Navigation.rightMotor.forward();
		    while (Navigation.leftMotor.isMoving() || Navigation.rightMotor.isMoving()) {
		      if (data.getL()[0] < blackLineColor) {
		    	  	Navigation.leftMotor.stop(true);
		      }
		      if (data.getL()[1] < blackLineColor) {
		    	  	Navigation.rightMotor.stop(true);
		      }
		    }
		    
		    rotateTheRobot(false, 90, false);
		    
		    odo.setTheta(0);
		    odo.setX(0);
		    odo.setY(0);*/
			int buff = 0;
			while (true) {
				
				correctionStart = System.currentTimeMillis();
				advanceRobot(50,true);
				
				while (LSL.getColorID() != 13) {
					buff = 1;
					if (LSR.getColorID()== 13) {
						buff = 2;
						break;
					}
				}
				
				stopMotor();
				
				
				/*if(buff == 1)
				{
					Sound.beep();
					//Navigator.rightMotor.forward();
					Navigator.rightMotor.setSpeed(100);
					Navigator.rightMotor.forward();
				}
				
				if(buff == 2)
				{
					Sound.beepSequence();
					Navigator.leftMotor.forward();
					Navigator.leftMotor.setSpeed(100);
					while(LSL.getColorID() != 13);
				}*/
				
				buff = 0;
				
				
				count++;
				
				if(count == 2)
				{
					rotateTheRobot(false, 90, false);
					odo.setX(7 * 30.48);
					odo.setTheta(270);
					break;
				}
				else
					odo.setY(30.48);
				
				rotateTheRobot(true,90,false);
				
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
