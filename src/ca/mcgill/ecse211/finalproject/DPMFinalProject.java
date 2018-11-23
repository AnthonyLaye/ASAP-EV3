package ca.mcgill.ecse211.finalproject;



import java.util.Map;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
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
		private static final EV3ColorSensor LSL = 
				new EV3ColorSensor(LocalEV3.get().getPort("S1"));//LS1 is the left one ultra is face against you
		private static final EV3ColorSensor LSR = 
				new EV3ColorSensor(LocalEV3.get().getPort("S3"));//LS2 is the right one, ultra is face against you
		private static final EV3ColorSensor ColSensor =
				new EV3ColorSensor(LocalEV3.get().getPort("S2"));
		private static final double WHEEL_RAD = 2.08;
		private static final double TRACK = 12.27;
		private static final double TILE_LENGTH = 30.48;
	
		private static SensorModes usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4")); // usSensor is the instance
		private static SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
		private static float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
		
		
		  public static void main(String[] args) throws OdometerExceptions {
			  
			    int buttonChoice;
			    int chooseWhichRoutine;//if chooseWichEdge is equal to 0, then it is rising edge, else it is falling edge
			    
				// Sensor Related Stuff
				SensorData sensorData = SensorData.getSensorData();
				SampleProvider backLight[] = new SampleProvider[2];
				backLight[0] = LSL.getRedMode();
				backLight[1] = LSR.getRedMode();
				Thread lightThread = new Thread();
				lightThread.start();
			    
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

			    final Navigation navigation = new Navigation(leftMotor, rightMotor, odometer, sensorData, LSL, LSR);
			    final UltrasonicLocalizer USLocalizer = new UltrasonicLocalizer(navigation, chooseWhichRoutine);
			    final LightLocalizer LSLocalizer = new LightLocalizer(navigation, LSL, LSR);
			    
			    usPoller = new UltrasonicPoller(usDistance, usData, navigation); // the selected controller on each cycle
			    usPoller.start();
			    
			    final ArmController armController = new ArmController(leftArmMotor, rightArmMotor);
			    final TunnelFollower tunnelFollower = new TunnelFollower(leftMotor, rightMotor, navigation, odometer, armController);
			    final TreeController ringController = new TreeController(leftMotor, rightMotor, navigation, odometer, ColSensor, armController);
			    
			    
			    // Sleep for 2 seconds
			    try {
			      Thread.sleep(2000);
			    } catch (InterruptedException e) {
			      // There is nothing to be done here
			    }
			    
			    // ******************** OPTAIN ALL WIFI DATA FROM SERVER ***********************************
			    
			    Map wifiData = WifiController.readData();
			    //System.out.println(wifiData);
			    
			    boolean isRedTeam = false;
			    
			    int redTeam = ((Long) wifiData.get("RedTeam")).intValue();
			    int greenTeam = ((Long) wifiData.get("GreenTeam")).intValue();
			    
			   if(redTeam == 13)	//Check if team 13 is red! if not we are green
			    	isRedTeam = true;
			    else if(greenTeam == 13)
			    	isRedTeam = false;
			    else
			    	System.exit(-1); //This better not happen...*/
			    
			    final int corner, llX, llY, urX, urY, islandLLX, islandLLY, islandURX, islandURY, tnLLX, tnLLY, tnURX, tnURY, tX, tY;
			    
			    isRedTeam = false; // THIS IS JUST FOR BETA DEMO, GREEN BY DEFAULT IN DEMO*/
			    
			    if(isRedTeam) {
			    	corner = ((Long) wifiData.get("RedCorner")).intValue();
			    	llX = ((Long) wifiData.get("Red_LL_x")).intValue();
			    	llY = ((Long) wifiData.get("Red_LL_y")).intValue();
			    	urX = ((Long) wifiData.get("Red_UR_x")).intValue();
			    	urY = ((Long) wifiData.get("Red_UR_y")).intValue();
			    	tnLLX = ((Long) wifiData.get("TNR_LL_x")).intValue();
			    	tnLLY = ((Long) wifiData.get("TNR_LL_y")).intValue();
			    	tnURX = ((Long) wifiData.get("TNR_UR_x")).intValue();
			    	tnURY = ((Long) wifiData.get("TNR_UR_y")).intValue();
			    	tX = ((Long) wifiData.get("TR_x")).intValue();
			    	tY = ((Long) wifiData.get("TR_y")).intValue();
			    }
			    
			    else {
			    	corner = ((Long) wifiData.get("GreenCorner")).intValue();
			    	llX = ((Long) wifiData.get("Green_LL_x")).intValue();
			    	llY = ((Long) wifiData.get("Green_LL_y")).intValue();
			    	urX = ((Long) wifiData.get("Green_UR_x")).intValue();
			    	urY = ((Long) wifiData.get("Green_UR_y")).intValue();
			    	tnLLX = ((Long) wifiData.get("TNG_LL_x")).intValue();
			    	tnLLY = ((Long) wifiData.get("TNG_LL_y")).intValue();
			    	tnURX = ((Long) wifiData.get("TNG_UR_x")).intValue();
			    	tnURY = ((Long) wifiData.get("TNG_UR_y")).intValue();
			    	tX = ((Long) wifiData.get("TG_x")).intValue();
			    	tY = ((Long) wifiData.get("TG_y")).intValue();
			    }
			    
			    islandLLX = ((Long) wifiData.get("Island_LL_x")).intValue();
			    islandLLY = ((Long) wifiData.get("Island_LL_y")).intValue();
			    islandURX = ((Long) wifiData.get("Island_UR_x")).intValue();
			    islandURY = ((Long) wifiData.get("Island_UR_y")).intValue();
			    
			    final int startX;
				final int startY;
			    
			    if(corner == 0) {
			    	startX = 1;
			    	startY = 1;
			    }
			    else if (corner == 1) {
			    	startX = 14;
			    	startY = 1;
			    }
			    else if (corner == 2) {
			    	startX = 1;
			    	startY = 8;
			    }
			    else {	//Corner = 3 or the game parameters were wrong...
			    	startX = 14;
			    	startY = 8;
			    }
			    
			    
			    (new Thread() {
			        public void run() {
			        	
			          USLocalizer.whichRoutine(); // Ultrasonic Localize
			          LSLocalizer.lightLocalize(startX, startY);	// Light localize
			          LSLocalizer.pollColour();
			          
			          //Sound.beep();
			        	
			          navigation.shake();
			          //Beta demo starts in corner 1 -> (7, 1) -> Done in LightLocalizer.java
			        	
			          //navigation.travelTo(tnLLX, tnLLY, false); // Travel to start of tunnel
			          //tunnelFollower.traverseTunnel(tnLLX, tnLLY, tnURX, tnURY); // Travel to start of tunnel and then to end of tunnel

			          //navigation.travelTo(3, 0, false);
			          //ringController.approachTree(6, 3); //Travel to tree and do collections
			          
			          /* The rest is not used for beta demo
			           
			          navigation.travelTo(tnURX, tnURY, false); // Travel back to tunnel
			          tunnelFollower.traverseTunnel(tnURX, tnURY, tnLLX, tnLLY); // Travel opposite way through tunnel
			          
			          navigation.travelTo(1, 1, false); // Travel back to starting corner
			          
			          armController.openArms(); //Drop off ring!*/
			        } 
			      }).start();
			 
			    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
			    System.exit(0);
			   
		  }
}
