package ca.mcgill.ecse211.finalproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class prepares for, and executes tunnel traversal
 *
 */
public class TunnelFollower {
	private Navigation navigation;
	private ArmController armController;
	
	
	public TunnelFollower(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Navigation navigation, Odometer odometer, ArmController armController) throws OdometerExceptions {
		this.navigation = navigation;
		this.armController = armController;
	}
	
	/**
	 * This method prepares the robot to traverse the tunnel.
	 * It may raise or lower the arms to ensure safe passage
	 * @param startX : x position before tunnel
	 * @param startY : y position before tunnel
	 * @param endX : x position after tunnel
	 * @param endY : y position after tunnel
	 * @param IslandURX : upper right x coordinate of the island
	 * @param IslandURY : upper right y coordinate of the island
	 * @param IslandLLX : lower left x coordinate of the island
	 * @param IslandLLY : lower left y coordinate of the island
	 * @param myZoneURX : upper right x coordinate of our robot's zone
	 * @param myZoneURY : upper right y coordinate of our robot's zone
	 * @param myZoneLLX : lower left x coordinate of our robot's zone
	 * @param myZoneLLY : lower left y coordinate of our robot's done
	 * @param TNRURX : upper right x coordinate of the tunnel
	 * @param TNRURY : upper right y coordinate of the tunnel
	 * @param TNRLLX : lower left x coordinate of the tunnel
	 * @param TNRLLY : lower left y coordinate of the tunnel
	 * @param isTravelingBack : determines if robot is traveling back to starting point, or traveling to tree
	 */
	public void traverseTunnel(double startX, double startY, double endX, double endY, 
			double IslandURX, double IslandURY, double IslandLLX, double IslandLLY, double myZoneURX, double myZoneURY,double myZoneLLX, 
			double myZoneLLY, double TNRURX, double TNRURY, double TNRLLX, double TNRLLY, boolean isTravelingBack) {
		
		armController.closeArms();
		double[] newValues = getTheEntry(myZoneURX,myZoneURY,myZoneLLX, myZoneLLY, TNRURX,TNRURY,TNRLLX,TNRLLY, IslandURX, IslandURY, IslandLLX, IslandLLY);	// Calculate modified coordinates
		
		if(!isTravelingBack) {	// If traveling to ring tree
			navigation.travelTo(newValues[0], newValues[1], false);	// Travel to entry point of tunnel
			double angle = 270;
			navigation.localizeForTunnel(angle, startX, startY);	// Localize before traversing
			
			navigation.travelTo(newValues[2], newValues[3], false);	// Travel to exit point of tunnel
			angle = 90;
			navigation.localizeAfterTunnel(angle, endX, endY);	// Localize after traversing
		}
		
		else {	// If traveling back from ring tree
			navigation.travelTo(newValues[2], newValues[3], false);	// Travel to entry point of tunnel
			double angle = 270;
			navigation.localizeForTunnel(angle, endX, endY);	// Localize before traversing
			
			navigation.travelTo(newValues[0], newValues[1], false);	// Travel to exit point of tunnel
			angle = 90;
			navigation.localizeAfterTunnel(angle, startX, startY);	// Localize after traversing
		}
		
		armController.openArms();
	}


	/**
	 * This method calculates the entry point of the tunnel. This is used so that the robot stops half a tile before the entrance
	 * so it can re-localize itself before traversal.
	 * @param myZoneURX : upper right x coordinate of the robot's zone
	 * @param myZoneURY : upper right y coordinate of the robot's zone
	 * @param myZoneLLX : lower left x coordinate of the robot's zone
	 * @param myZoneLLY : lower left y coordinate of the robot's zone
	 * @param tNRURX : upper right x coordinate of the tunnel
	 * @param tNRURY : upper right y coordinate of the tunnel
	 * @param tNRLLX : lower left x coordinate of the tunnel
	 * @param tNRLLY : lower left y coordinate of the tunnel
	 * @param islandURX : upper right x coordinate of the island
	 * @param islandURY : upper right y coordinate of the island
	 * @param islandLLX : lower left x coordinate of the island
	 * @param islandLLY : lower left y coordinate of the island
	 * @return array : array of modified tunnel coordinates
	 */
	public double[] getTheEntry(double myZoneURX, double myZoneURY, double myZoneLLX, double myZoneLLY, double tNRURX,
			double tNRURY, double tNRLLX, double tNRLLY, double islandURX, double islandURY, double islandLLX,
			double islandLLY) {
		
		/*double islandULX = islandLLX;
		double islandULY = islandURY;
		double islandlRX = islandURX;
		double islandLRY = islandLLY;
		
		double myZoneULX = myZoneLLX;
		double myZoneULY = myZoneURY;
		double myZoneLRX = myZoneURX;
		double myZoneLRY = myZoneLLY;
		
		double tNRULX = tNRLLX;
		double tNRULY = tNRURY;
		double tNRLRX = tNRURX;
		double tNRLRY = tNRLLY;*/
		
		double startVx = tNRURX - 0.5;
		double startVy = tNRURY + 0.5;
		double endVx = tNRLLX + 0.5;
		double endVy = tNRLLY - 0.5;
		
		double startHx = tNRLLX - 0.5;
		double startHy = tNRLLY + 0.5;
		double endHx = tNRURX + 0.5;
		double endHy = tNRURY - 0.5;
		
		double startVx2 = tNRLLX + 0.5;
		double startVy2 = tNRLLY - 0.5;
		double endVx2 = tNRURX - 0.5;
		double endVy2 = tNRURY + 0.5;
		
		double startHx2 = tNRURX + 0.5;
		double startHy2 = tNRURY - 0.5;
		double endHx2 = tNRLLX - 0.5;
		double endHy2 = tNRLLY + 0.5;
		
		//if the start point and end point are all in the robot's zone and island
		if( myZoneLLX <= startVx && startVx <= myZoneURX && myZoneLLY <= startVy && startVy <= myZoneURY &&
				islandLLX <= endVx && endVx <= islandURX && islandLLY <= endVy && endVy <= islandURY)
		{
			double array[] = {startVx, startVy, endVx, endVy};
			return array;
		}
		else if( myZoneLLX <= startHx && startHx <= myZoneURX && myZoneLLY <= startHy && startHy <= myZoneURY &&
				islandLLX <= endHx && endHx <= islandURX && islandLLY <= endHy && endHy <= islandURY)
		{
			double array[] = {startHx, startHy, endHx, endHy};
			return array;
		}
		else if( myZoneLLX <= startHx2 && startHx2 <= myZoneURX && myZoneLLY <= startHy2 && startHy2 <= myZoneURY &&
				islandLLX <= endHx2 && endHx2 <= islandURX && islandLLY <= endHy2 && endHy2 <= islandURY)
		{
			double array[] = {startHx2, startHy2, endHx2, endHy2};
			return array;
		}
		if( myZoneLLX <= startVx2 && startVx2 <= myZoneURX && myZoneLLY <= startVy2 && startVy2 <= myZoneURY &&
				islandLLX <= endVx2 && endVx2 <= islandURX && islandLLY <= endVy2 && endVy2 <= islandURY)
		{
			double array[] = {startVx2, startVy2, endVx2, endVy2};
			return array;
		}
		
		
		return null;	// Returns null if zones are touching
	}
}