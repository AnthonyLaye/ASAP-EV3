package ca.mcgill.ecse211.finalproject;


import lejos.hardware.Sound;

/**
 * This class implements falling or rising edge localization to help direct the robot towards true 0 degrees
 * @author Mai Zeng
 */
public class UltrasonicLocalizer {
	private Navigation navigation;
	private Odometer odo; //get the current local position data
	private static final int NOISE = 1; //noise margin 
	private static final int DISTANCE = 50; //as localize tutorial slides said
	private int chooseWhichRoutine = -1; // If chooseWichEdge is equal to 0, then it is rising edge, if 1 it is falling edge, intial it as -1 so that it will confused by 0 or 1
	
	public UltrasonicLocalizer(Navigation navigation, int chooseWhichRoutine) throws OdometerExceptions {
		this.navigation = navigation;
		this.chooseWhichRoutine=chooseWhichRoutine;  
		this.odo = Odometer.getOdometer();
	}

  /**
   * This method is to process the input that the user just gave,
   * whether using a routine of falling edge or a routine of rising edge
   * if the chooseWhichRoutine == 0 then use the rising edge routine
   * else use a falling edge routine
   */
  public void whichRoutine()
  {
	  if(chooseWhichRoutine == 0)
		  risingEdgeRoutine();
	  else
		  fallingEdgeRoutine();
  }
  
  /**
   * If the robot starts facing a wall, it can:
   * Detect a rising edge, switch directions, then detect another rising edge
   * Detect a rising edge, continue in the same direction, then detect a falling edge
   */
  private void risingEdgeRoutine() {
	  double a = 0;
	  double b = 0;
	  navigation.rotateTheRobot(true, 360 , true);
	  
	  while(true)
	  {
		  if(navigation.readUSDistance() > DISTANCE + NOISE)
			  break;
	  }
	  
	  Sound.beep();

	  navigation.stopMotors();

	  a = odo.getXYT()[2];//get a
	 	  
	  navigation.rotateTheRobot(false,150,false);
	  
	  //now get b
	  navigation.rotateTheRobot(false,360,true);//let it roll to detect a falling edge again

	  while(navigation.readUSDistance() < DISTANCE + NOISE) {

	  } 
	    
	  navigation.stopMotors();
	  Sound.beep();

	  b = 360 - odo.getXYT()[2];
	  
	  double deltaTheta = (a+b)/2;
	  
	  navigation.stopMotors();
	  navigation.rotateTheRobot(true,deltaTheta,false);
	  navigation.stopMotors();
	  navigation.rotateTheRobot(true,135,false);
	  odo.setTheta(0);
	  
  }

  /**
   * If the robot starts facing away from the walls, it can:
   * Detect a falling edge, continue in the same direction, then detect a rising edge
   */
  private void fallingEdgeRoutine() {
	  // TODO Auto-generated method stub
		double a = 0;
		double b = 0;
		
		navigation.rotateTheRobot(true,360,true);
		int buff = 0;
		while(navigation.readUSDistance() < DISTANCE * 2)
		{
			if(navigation.readUSDistance() > DISTANCE * 2)
			{
				
				buff++;
				if(buff == 20)
					break;
				
			}
		}
		odo.setTheta(0);
		navigation.stopMotors();
		
		navigation.rotateTheRobot(true,360,true);//let it roll for 2 circle, this will make sure that it will finish the data collecting
		
		while(true) {
			if(navigation.readUSDistance() < DISTANCE - NOISE)   //detect a rising edge  
				break; 
		}
		Sound.beep();//sound buzz to let user know it have detect a rising edge
		navigation.stopMotors();
		a = odo.getXYT()[2];//get a
		navigation.rotateTheRobot(false,85,false);
		//now get b
		navigation.rotateTheRobot(false,360,true);//let it roll to detect a falling edge again
		
		while(navigation.readUSDistance() > DISTANCE - NOISE) {//continue to travel until it detect a falling edge

		}
		navigation.stopMotors();
		Sound.beep();
		b = 360 - odo.getXYT()[2];//what we want acutally is 360 - odo.getXY()[2] but for convenience i use this
		
		double deltaTheta = (a+b)/2;//now it is some value but it will process on robot to let it point to the 45 degree on the board
		
		navigation.stopMotors();
		navigation.rotateTheRobot(true,deltaTheta,false);
		navigation.stopMotors();
		navigation.rotateTheRobot(false,55,false);
		odo.setTheta(0);
  }
}
