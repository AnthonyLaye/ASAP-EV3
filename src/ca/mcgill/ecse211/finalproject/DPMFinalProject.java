package ca.mcgill.ecse211.finalproject;



import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class DPMFinalProject {
		// Motor Objects, and Robot related parameters
		private static final EV3LargeRegulatedMotor leftMotor =
				new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
		private static final EV3LargeRegulatedMotor rightMotor =
				new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
		private static final EV3MediumRegulatedMotor leftArmMotor = 
				new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
		private static final EV3MediumRegulatedMotor rightArmMotor = 
				new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
		
		private static final TextLCD lcd = LocalEV3.get().getTextLCD();
		private static final EV3ColorSensor LS = 
				new EV3ColorSensor(LocalEV3.get().getPort("S3"));
		private static final EV3ColorSensor ColSensor =
				new EV3ColorSensor(LocalEV3.get().getPort("S2"));
		private static final EV3GyroSensor GS =
				new EV3GyroSensor(LocalEV3.get().getPort("S1"));
		private static final double WHEEL_RAD = 2.2;
		private static final double TRACK = 11.3;
	
		private static SensorModes usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4")); // usSensor is the instance
		private static SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
		private static float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
		
		
		  public static void main(String[] args) throws OdometerExceptions {
			  
			    int buttonChoice;
			    int chooseWhichRoutine;//if chooseWichEdge is equal to 0, then it is rising edge, else it is falling edge
			    
			    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
			    Display odometryDisplay = new Display(lcd); // No need to change
			  
			    UltrasonicPoller usPoller = null; 
			    
			    do {
			        // clear the display
			        lcd.clear();
			        
			        // ask the user whether the motors should drive in a square or float
			        lcd.drawString("< Left | Right >", 0, 0);
			        lcd.drawString("       |        ", 0, 1);
			        lcd.drawString("  Rise | Fall  ", 0, 2);

			        buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
			    }
			    
			    while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
			    
			    //see which mode are we in, choose rise is to 
			    if(buttonChoice == Button.ID_LEFT)
			    		chooseWhichRoutine = 0;//if chooseWhichEdge is equal to 0, then it is rising edge
			    else
			    		chooseWhichRoutine = 1;//if chooseWhichEdge is equal to 1, then it is falling edge
			 
			    Thread odoThread = new Thread(odometer);
			    odoThread.start();
			    Thread odoDisplayThread = new Thread(odometryDisplay);
			    odoDisplayThread.start();
		    	// final Gyro gyro = new Gyro();
			    final Navigation navigation = new Navigation(leftMotor, rightMotor, odometer);
			    final UltrasonicLocalizer USLocalizer = new UltrasonicLocalizer(navigation, chooseWhichRoutine);
			    final LightLocalizer LSLocalizer = new LightLocalizer(navigation, LS);
			    
			    usPoller = new UltrasonicPoller(usDistance, usData, navigation); // the selected controller on each cycle
			    usPoller.start();
			    
			    final ArmController armController = new ArmController(leftArmMotor, rightArmMotor);
			    final TunnelFollower tunnelFollower = new TunnelFollower(leftMotor, rightMotor, navigation, odometer, armController);
			    final TreeController ringController = new TreeController(leftMotor, rightMotor, navigation, odometer, ColSensor, armController);
			    //usPoller2 = new UltrasonicPoller(usDistanceCol, usDataCol, driveDetect);
			    //usPoller2.start();
			    
			    // Sleep for 2 seconds
			    try {
			      Thread.sleep(2000);
			    } catch (InterruptedException e) {
			      // There is nothing to be done here
			    }
			    
			    (new Thread() {
			        public void run() {
			          USLocalizer.whichRoutine(); // Ultrasonic Localize
			          LSLocalizer.lightLocalize();	// Light localize
			          /*navigation.rotateTheRobot(true, 180, false);
			          odometer.setTheta(theta);
			          
			          navigation.travelTo(2, 3, false); // Travel to start of tunnel, hardcode value for now
			          tunnelFollower.traverseTunnel(1.5, 3, 1.5, 7); // Travel to end of tunnel
			          
			          ringController.approachTree(2, 7.5); //Travel to tree and do collections
			          
			          navigation.travelTo(1.5,  7, false); // Travel back to tunnel
			          tunnelFollower.traverseTunnel(1.5, 7, 1.5, 3); // Travel opposite way through tunnel
			          
			          navigation.travelTo(1, 1, false); // Travel back to starting location
			          
			          armController.openArms(); //Drop off ring!*/
			        } 
			      }).start();
			 
			    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
			    System.exit(0);
			   
		  }
}
