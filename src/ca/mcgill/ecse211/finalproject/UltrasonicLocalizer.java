package ca.mcgill.ecse211.finalproject;


import lejos.hardware.Sound;

public class UltrasonicLocalizer {
		private Navigation navigation;
		private Odometer odo;  //get the current local position data
		private int chooseWhichRoutine = -1; //0 is rising edge, 1 is falling edge
		private static final int NOISE_MARGIN = 1; //noise margin 
		private static final int DISTANCE = 40; //as localize tutorial slides said
		
		public UltrasonicLocalizer(Navigation navigation, int chooseWhichRoutine, Odometer odometer) throws OdometerExceptions {
			this.navigation = navigation;
			this.chooseWhichRoutine = chooseWhichRoutine;  
			this.odo = odometer;
		}

	  /**
	   * This method is to process the user input, to determine if we should perform
	   * rising edge, or falling edge localization
	   */
	  public void whichRoutine() {
		  if(chooseWhichRoutine == 0)
			  risingEdgeRoutine();
		  else
			  fallingEdgeRoutine();
	  }
	  
	  /**
	   * This method implements rising edge localization
	   * If the robot starts facing a wall, it can:
	   * Detect a rising edge, switch directions, then detect another rising edge
	   * Detect a rising edge, continue in the same direction, then detect a falling edge
	   */
	  private void risingEdgeRoutine() {
		  double a = 0;
		  double b = 0;
		  navigation.rotateTheRobot(true, 360, true);
		  
		  while(true){
			  if(navigation.distance > DISTANCE + NOISE_MARGIN)
				  break;
		  }
		  
		  Sound.beep();
		  navigation.stopMotors();

		  a = odo.getXYT()[2];  //Get a
		 	  
		  navigation.rotateTheRobot(false, 45, false);
		  
		  //Now get b
		  navigation.rotateTheRobot(false, 360, true);  //let it roll to detect a falling edge again

		  while(navigation.distance < DISTANCE + NOISE_MARGIN ) {

		  } 
		    
		  navigation.stopMotors();
		  Sound.beep();

		  b = 360 - odo.getXYT()[2];
		  
		  double deltaTheta = (a + b) / 2;
		  
		  navigation.stopMotors();
		  navigation.rotateTheRobot(true, deltaTheta, false);
		  navigation.stopMotors();
		  navigation.rotateTheRobot(true, 135, false);
		  odo.setTheta(0);
	  }

	  /**
	   * This method implements falling edge localization
	   * If the robot starts facing away from the walls, it can:
	   * Detect a falling edge, continue in the same direction, then detect a rising edge
	   */
	private void fallingEdgeRoutine() {
		  // TODO Auto-generated method stub
			double a = 0;
			double b = 0;
			double moveFromLeftWall = 0;
			double moveFromBacktWall = 0;
			double cos10 = 0.9848;

			navigation.rotateTheRobot(true, 360, true);//let it roll for 2 circle, this will make sure that it will finish the data collecting
			
			while(true) {
				if(navigation.distance < DISTANCE - NOISE_MARGIN)   //detect a rising edge  
					break; 
			}
			Sound.beep();//sound buzz to let user know it have detect a rising edge
		    navigation.stopMotors();
			a = odo.getXYT()[2];//get a
			navigation.rotateTheRobot(false,45,false);
			//now get b
			navigation.rotateTheRobot(false,360,true);//let it roll to detect a falling edge again
			
			while(navigation.distance > DISTANCE - NOISE_MARGIN) {//continue to travel until it detect a falling edge
	
			}
			navigation.stopMotors();
			Sound.beep();
			b = 360 - odo.getXYT()[2];//what we want acutally is 360 - odo.getXY()[2] but for convenience i use this
			
			double deltaTheta = (a + b) / 2;//now it is some value but it will process on robot to let it point to the 45 degree on the board
			
		    navigation.stopMotors();
			navigation.rotateTheRobot(true, deltaTheta, false);
			navigation.stopMotors();
			navigation.rotateTheRobot(false, 45, false);
			odo.setTheta(0);
			
			//detect the distance from left wall
			int correctLeft = 0;
			int correctBack = 0;
			navigation.rotateTheRobot(false, 90, false);
			navigation.stopMotors();
			
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}//wait for 1 second
			
			int dx = navigation.distance;
			navigation.rotateTheRobot(false, 10, false);
			navigation.stopMotors();
			
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			int correctDx1 = navigation.distance;
			navigation.rotateTheRobot(true, 20, false);
			
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			int correctDx2 = navigation.distance;
			navigation.rotateTheRobot(false, 10, false);
			moveFromLeftWall = Math.floor(30.48 - (correctDx1/cos10 + correctDx2/cos10 + dx)/3 - 4);
			
			if(correctDx1 - correctDx2 > 1)
			{
				correctLeft = 3;
			}
			else if(correctDx2 - correctDx1 > 1)
			{
				correctLeft = -3;
			}
			else
				correctLeft = 0;
			
			navigation.rotateTheRobot(true, 90 - correctLeft, false);
			
			//detect the distance from back wall
			navigation.rotateTheRobot(true, 180, false);
			navigation.stopMotors();
			
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			int dy = navigation.distance;
			navigation.rotateTheRobot(true, 10, false);
			navigation.stopMotors();
			
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			int correctDy1 = navigation.distance;
			navigation.rotateTheRobot(false, 20, false);
			navigation.stopMotors();
			
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			int correctDy2 = navigation.distance;
			navigation.rotateTheRobot(true, 10, false);
			moveFromBacktWall = Math.floor(30.48 - (correctDy1/cos10 + correctDy2/cos10 + dy)/3 - 4);
			//navigation.navigation.rotateTheRobot(false, 180, false);
			if(correctDy1 - correctDy2 > 1) {
				correctBack = 3;
			}
			else if(correctDy2 - correctDy1 > 1) {
				correctBack = -3;
			}
			else
				correctBack = 0;
			
			navigation.rotateTheRobot(false, 180 - correctBack, false);
			//move the robot
			navigation.advanceRobot(moveFromBacktWall, false);
			navigation.rotateTheRobot(true, 90, false);
			navigation.advanceRobot(moveFromLeftWall, false);
			navigation.rotateTheRobot(false, 85, false);
			odo.setXYT(0, 0, 0);
	  }
}
