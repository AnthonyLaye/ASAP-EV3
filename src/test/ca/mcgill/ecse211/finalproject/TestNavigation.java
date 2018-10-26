package test.ca.mcgill.ecse211.finalproject;

import ca.mcgill.ecse211.finalproject.Navigation;
import ca.mcgill.ecse211.finalproject.Odometer;
import ca.mcgill.ecse211.finalproject.OdometerExceptions;

public class TestNavigation {
	 
	  public Odometer odo;
	  public Navigation navigation;
	  public static double WHEEL_RAD = 2.2;
	  public static double track = 11.3; 
	  
	  public TestNavigation(Navigation navi) throws OdometerExceptions
	  {
		  this.odo = Odometer.getOdometer(); 
		  this.navigation = navi;
	  }
	  
	  public void driveTo(double x, double y) {
		  navigation.travelTo(x, y, true);
	  }
}
