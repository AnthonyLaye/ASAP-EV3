package ca.mcgill.ecse211.finalproject;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;

/**
 * The following class obtains the game data map from the server and returns it to the main class
 *
 */
public class WifiController {
	
	// ** Set these as appropriate for your team and current situation **
<<<<<<< HEAD
	  private static final String SERVER_IP = "142.157.77.40";
=======
	  private static final String SERVER_IP = "192.168.2.16";
>>>>>>> 005fc694b9760a220a3a1838a35ea51c13930a10
	  private static final int TEAM_NUMBER = 13;

	  // Enable/disable printing of debug info from the WiFi class
	  private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

	  /**
	   * This method attempts to connect to the server to retrieve game data
	   * @return data : map of data obtained from server
	   */
	  @SuppressWarnings("rawtypes")
	  public static Map readData() {

	    //System.out.println("Running..");

	    // Initialize WifiConnection class
	    WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

	    // Connect to server and get the data, catching any errors that might occur
	    Map data = null;
	    try {
	      data = conn.getData();
	      } catch (Exception e) {
	      System.err.println("Error: " + e.getMessage());
	    }
	    
	    return data;	//Return the data map
	  }
}
