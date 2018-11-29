package ca.mcgill.ecse211.finalproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class prepares for, and executes tunnel traversal. It computes the entrance and exit points of the tunnel, as well as the direction to turn 
 * in order to localize before and after crossing.
 * @author Anthony Laye
 * @author Mai Zeng
 */
public class TunnelFollower {
	private Navigation navigation;
	private ArmController armController;
	private Odometer odometer;
	
	public TunnelFollower(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Navigation navigation, Odometer odometer, ArmController armController) throws OdometerExceptions {
		this.navigation = navigation;
		this.armController = armController;
		this.odometer = odometer;
	}
	
	/**
	 * This method prepares the robot to traverse the tunnel.
	 * It may raise or lower the arms to ensure safe passage.
	 * First, it calculates the entry and exit coordinates of the tunnel: there are set to be the center of the tiles right before and right after the tunnel.
	 * From these, we can determine if the tunnel is oriented horizontally or vertically, and determine which entrance of the tunnel the robot will enter and exit from.
	 * Once done, we travel to the starting position and localize before the tunnel. There are 4 key parameters for tunnel localization: angle1, angle2, x_corr and y_corr.
	 * Angle 1 and 2 are the angles the robot must turn to to seek for a line. The correction values corrects the odometer once these lines are found.
	 * Once passed the bridge, the robot localizes again with slightly different parameters to account for changes in direction.
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
			double myZoneLLY, double TNRURX, double TNRURY, double TNRLLX, double TNRLLY) {
		
		armController.closeArms();
		double[] tunnelValues = getTheEntry(myZoneURX,myZoneURY,myZoneLLX, myZoneLLY, TNRURX,TNRURY,TNRLLX,TNRLLY, IslandURX, IslandURY, IslandLLX, IslandLLY);	// Calculate modified coordinates for entry and exit
		
		boolean isVertical;	// Determine if we cross tunnel vertically or horizontally
		
		if(tunnelValues[0] == tunnelValues[2])	// If the x values of new coordinate system are the same, we are traveling straight up or down - so vertically
			isVertical = true;
		else
			isVertical = false;
		
		// Check to see if robot is approaching from LL or UR
		boolean isTravelingBack; // True if approaching from UR
		if(isVertical) {	// Vertical tunnel
			if(odometer.getXYT()[1] <= TNRLLY * 30.48)	// If we are below bridge, we are approaching from LL entry point
				isTravelingBack = false;
			else	// Else we are above bridge and robot enters tunnel from UR entry point
				isTravelingBack = true;
		}
		else {	// Horizontal tunnel
			if(odometer.getXYT()[0] <= TNRLLX * 30.48)	// If we are left of the bridge, we are approaching from LL entry point
				isTravelingBack = false;
			else	// Else we are to the right of the bridge and entering tunnel from UR entry point
				isTravelingBack = true;
		}
		
		if(!isTravelingBack)
			navigation.travelTo(tunnelValues[0], tunnelValues[1], false);	// Travel to LL entry point of tunnel
		else
			navigation.travelTo(tunnelValues[2], tunnelValues[3], false);	// Travel to UR entry point of tunnel
			
		//Preparations to localize before tunnel
		double angle1, angle2, x_corr , y_corr;
		if(isVertical) {	// Vertical bridge
			if(odometer.getXYT()[1] <= TNRLLY * 30.48) {	// We are below the bridge if current y is less than tunnel lower left y coordinate
				angle1 = 270;
				angle2 = 0;
				x_corr = startX;
				y_corr = startY;
			}
			else {	// We are above the bridge
				angle1 = 90;
				angle2 = 180;
				x_corr = endX;
				y_corr = endY;
			}
		}
		else {	// Horizontal bridge
			if(odometer.getXYT()[0] <= TNRLLX * 30.48) {	// We are to the left of the bridge if current x is less than tunnel lower left x coordinate
				angle1 = 180;
				angle2 = 90;
				x_corr = startX;
				y_corr = startY;
			}				
			else {	// We are to the right of the bridge
				angle1 = 0;
				angle2 = 270;
				x_corr = endX;
				y_corr = endY;
			}
		}
		
		navigation.tunnelLocalize(angle1, angle2, x_corr, y_corr, isVertical);	// Localize before traversing
		
		navigation.rotateTheRobot(false, 7, false);
		if(!isTravelingBack) 
			navigation.travelTo(tunnelValues[2], tunnelValues[3], false);	// Travel to UR exit point of tunnel
		else 
			navigation.travelTo(tunnelValues[0], tunnelValues[1], false);	// Travel to LL exit point of tunnel
		
		// Preparations to localize after the tunnel
		if(isVertical) {	// Vertical bridge
			if(odometer.getXYT()[1] <= TNRLLY * 30.48) {	// We are below the bridge if current y is less than tunnel lower left y coordinate
				x_corr = startX;
				y_corr = startY - 1;
			}
			else {	// We are above the bridge
				x_corr = endX;
				y_corr = endY + 1;
			}
		}
		else {	// Horizontal bridge
			if(odometer.getXYT()[0] <= TNRLLX * 30.48) {	// We are to the left of the bridge if current x is less than tunnel lower left x coordinate
				x_corr = startX - 1;
				y_corr = startY;
			}				
			else {	// We are to the right of the bridge
				x_corr = endX + 1;
				y_corr = endY;
			}
		}
		
		navigation.tunnelLocalize(Math.abs(angle1 - 180), angle2, x_corr, y_corr, isVertical);	// Localize after traversing
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