package test.ca.mcgill.ecse211;

import org.junit.Test;

import ca.mcgill.ecse211.finalproject.OdometerExceptions;
import ca.mcgill.ecse211.finalproject.TunnelFollower;

public class TestTunnelDriver {
	
	@Test
	public void testEntryPointGetter() {
		
		TunnelFollower tunnelFollower = null;
		try {
			tunnelFollower = new TunnelFollower(null, null, null, null, null);
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		double[] newValues = tunnelFollower.getTheEntry(8, 3, 2, 0, 3, 5, 2, 3, 6, 8, 0, 5);
		System.out.println("x: " + newValues[0] + " y: " + newValues[1] + " xf: " + newValues[2] + " yf: " + newValues[3]);	// Beta demo document
		
		newValues = tunnelFollower.getTheEntry(8, 3, 2, 0, 3, 4, 2, 3, 6, 8, 0, 4);
		System.out.println("x: " + newValues[0] + " y: " + newValues[1] + " xf: " + newValues[2] + " yf: " + newValues[3]);	// Beta demo document - square tunnel 
		
		newValues = tunnelFollower.getTheEntry(15, 4, 10, 0, 11, 5, 10, 3, 15, 9, 6, 5);
		System.out.println("x: " + newValues[0] + " y: " + newValues[1] + " xf: " + newValues[2] + " yf: " + newValues[3]);	// Final project document Green team
		
		newValues = tunnelFollower.getTheEntry(15, 4, 10, 0, 11, 5, 10, 4, 15, 9, 6, 5);
		System.out.println("x: " + newValues[0] + " y: " + newValues[1] + " xf: " + newValues[2] + " yf: " + newValues[3]);	// Final project document Green team - square tunnel
		
		newValues = tunnelFollower.getTheEntry(4, 9, 0, 5, 6, 8, 4, 7, 15, 9, 6, 5);
		System.out.println("x: " + newValues[0] + " y: " + newValues[1] + " xf: " + newValues[2] + " yf: " + newValues[3]);	// Final project document Red team
		
		newValues = tunnelFollower.getTheEntry(5, 9, 0, 5, 6, 8, 5, 7, 15, 9, 6, 5);
		System.out.println("x: " + newValues[0] + " y: " + newValues[1] + " xf: " + newValues[2] + " yf: " + newValues[3]); // Final project document Red team - square tunnel
		
		//System.out.println(tunnelFollower.getTheEntry(myZoneURX, myZoneURY, myZoneLLX, myZoneLLY, tNRURX, tNRURY, tNRLLX, tNRLLY, islandURX, islandURY, islandLLX, islandLLY));
	}

}
