package ca.mcgill.ecse211.wallfollowing;


import java.io.File;
import java.io.FileWriter;
import java.io.IOException;


import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.*;
import lejos.robotics.navigation.Navigator;

public class BangBangController implements UltrasonicController {

	private static final int FILTER_OUT = 10;
	//in case there are some mistakes

	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private int distance;
	int minDis;
	int maxDis;
	int filterControl;
	int[] distanceData = new int[200];
	int count = 0;
	//compare with FILTER_OUT

    /**
     * export CSV file
     */
    public static void generateCSVOutputFile(String filePathName, int[] distanceData) {
        File file = new File(filePathName);
        FileWriter fw;
        try {
            fw = new FileWriter(file);
            fw.append("distance");
            for(int i = 0; i < 200; i++)
            {
            		fw.append("," + distanceData[i]);
            }
            fw.append('\n');
            fw.flush();
            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = 35;//set the band center to 35, this is enough for our car to make a turn b/c it is fit the length of our car's length
		this.bandwidth = 5;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
	}
	@Override
	public void processUSData(int distance) {
		
		//filter which is same as p controller but change the max distance since we think this distance is enough
		if (distance >= 130 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 130) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}
		if(count == 0) {
			rotateTheRobot(false,45,false);
		}
		count++;
		advanceRobot(2.586,true);
		
		this.distance = distance - 20;
		if(this.distance > 255)
		{
			this.distance = 255;
		}
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)
		//to keep our code as simple as possible so we just use three if statement
		LCD.drawString("distance", 0, 3);
		LCD.drawInt(this.distance, 0, 4);
		System.out.println("" + this.distance);
	}


	@Override
	public int readUSDistance() {
		return this.distance;
	}
	
	  /**
	   * This method allows the conversion of a distance to the total rotation of each wheel need to
	   * cover that distance.
	   * 
	   * @param radius
	   * @param distance
	   * @return
	   */
	  private static int convertDistance(double radius, double travelDistance) {
		  return (int) ((180.0 * travelDistance) / (Math.PI * radius));
	  }

	  private static int convertAngle(double radius, double width, double angle) {
		  return convertDistance(radius, Math.PI * width * angle / 360.0);
	  }  

	  /**
	   * this method is to stip the motor ,both of them, left and right
	   */

	  
	  /**Advances the robot a desired amount of cm.
	   * 
	   * @param distanceToTravel Distance to travel in cm.
	   * @param instantReturn    True if the function is to be instantly returned. False if the function is to be returned after the travel is completed.
	   */
	   public void advanceRobot(double distanceToTravel, boolean instantReturn) {
	     
		   WallFollowingLab.leftMotor.setSpeed(150);
		   WallFollowingLab.rightMotor.setSpeed(150);
	              
		   WallFollowingLab.leftMotor.rotate(convertDistance(2.2, distanceToTravel), true);
		   WallFollowingLab.rightMotor.rotate(convertDistance(2.2, distanceToTravel), instantReturn);
	     
	   }
		  /**
		   * this method is created to let the robot rotate certain angle
		   * @param cwOrCcw
		   * @param angleToRotate
		   */
		  private void rotateTheRobot(boolean cwOrCcw, double angleToRotate, boolean blocked) {
			  double absAngleToRotate = Math.abs(angleToRotate);
			  WallFollowingLab.leftMotor.setSpeed(150);
			  WallFollowingLab.rightMotor.setSpeed(150);
			  
			  //just a reminder that if want the robot to rotate clockwise then leftMotor rotate is positive and right is negative
			  //true means cw
			  //false means ccw
			  if(cwOrCcw) {
				  WallFollowingLab.leftMotor.rotate(convertAngle(2.2, 11.3, absAngleToRotate),true);
				  WallFollowingLab.rightMotor.rotate(-convertAngle(2.2, 11.3, absAngleToRotate),blocked);
			  }
			  else
			  {
				  WallFollowingLab.leftMotor.rotate(-convertAngle(2.2, 11.3, absAngleToRotate),true);
				  WallFollowingLab.rightMotor.rotate(convertAngle(2.2, 11.3, absAngleToRotate),blocked);
			  }
			  // TODO Auto-generated method stub
		  }
}
