package ca.mcgill.ecse211.finalproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class prepares for, and executes tunnel traversal
 *
 */
public class TunnelFollower {
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Odometer odo;
	private Navigation navigation;
	private ArmController armController;
	private static final int FORWARD_SPEED = 150;
	
	
	public TunnelFollower(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Navigation navigation, Odometer odometer, ArmController armController) throws OdometerExceptions {
		this.navigation = navigation;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odo = odometer;
		this.armController = armController;
	}
	
	/**
	 * This method prepares the robot to traverse the tunnel.
	 * It may raise or lower the arms to ensure safe passage
	 * @param startX : x position before tunnel
	 * @param startY : y position before tunnel
	 * @param endX : x position after tunnel
	 * @param endY : y position after tunnel
	 */
	public void traverseTunnel(double startX, double startY, double endX, double endY, 
			double IslandURX, double IslandURY, double IslandLLX, double IslandLLY, double myZoneURX, double myZoneURY,double myZoneLLX, double myZoneLLY, double TNRURX, double TNRURY, double TNRLLX, double TNRLLY) {
		
		armController.closeArms();
		
		//double[] newValues = calculateTunnelEntry(startX, startY, endX, endY, IslandURX, IslandURY, IslandLLX, IslandLLY, myZoneX, myZoneY, TNRURX, TNRURY, TNRLLX, TNRLLY);
		double[] newValues = getTheEntry(myZoneURX,myZoneURY,myZoneLLX, myZoneLLY, TNRURX,TNRURY,TNRLLX,TNRLLY, IslandURX, IslandURY, IslandLLX, IslandLLY);
		//navigation.travelTo(newValues[0], newValues[1], false);	//Offset values by 0.5 so we are lined up with center of tunnel
		navigation.travelTo(newValues[0], newValues[1] - 0.2, false);	//Offset values by 0.5 so we are lined up with center of tunnel
		double angle = 270;
		navigation.localizeForTunnel(angle, startX, startY);
		
		navigation.travelTo(newValues[2], newValues[3] - 0.2, false);
		angle = 90;
		navigation.localizeAfterTunnel(angle, endX, endY);
		
		
		armController.openArms();
	}


	
	private double[] getTheEntry(double myZoneURX, double myZoneURY, double myZoneLLX, double myZoneLLY, double tNRURX,
			double tNRURY, double tNRLLX, double tNRLLY, double islandURX, double islandURY, double islandLLX,
			double islandLLY) {
		// TODO Auto-generated method stub
		
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
		
		//if the start point and end point are all in the my zone and island
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
		
		
		return null;
	}

	/**
	 * This method calculates where to stop in front of the tunnel based on the coordinates.
	 * @param startX
	 * @param startY
	 * @param endX
	 * @param endY
	 * @param IslandURX
	 * @param IslandURY
	 * @param IslandLLX
	 * @param IslandLLY
	 * @param myZoneX
	 * @param myZoneY
	 * @return
	 */
	private double[] calculateTunnelEntry(double startX, double startY, double endX, double endY, 
			double IslandURX, double IslandURY, double IslandLLX, double IslandLLY, double myZoneX, double myZoneY,double TNRURX, double TNRURY, double TNRLLX, double TNRLLY) {
		double[] offsetValues = new double[4];
		
		//DIFFERENT CASES FOR TUNNEL ALIGNMENT
		
		//CASE 1: Difference in Y values is 2 -> tunnel is vertical
		if(Math.abs(endY - startY) == 2) {
			if(endY > startY) {	// We are approaching from the bottom
				offsetValues[0] = startX + 0.5;
				offsetValues[1] = startY - 0.5;
				offsetValues[2] = endX - 0.5;
				offsetValues[3] = endY + 0.5;
			}
			else if(startY > endY) {	// We are approaching from the top
				offsetValues[0] = startX - 0.5;
				offsetValues[1] = startY + 0.5;
				offsetValues[2] = endX + 0.5;
				offsetValues[3] = endY - 0.5;
			}
		}
		//CASE 2: Difference in Y values is 1 -> tunnel is horizontal
		else if(Math.abs(endY - startY) == 1) {
		
			
			if(Math.abs(TNRURX - TNRLLX) == 1 && Math.abs(TNRURY - TNRLLY) == 1)// this means that it is a square tunnel
			{
				
				if(TNRLLY == IslandURY)
				{
					offsetValues[0] = TNRURX - 0.5;
					offsetValues[1] = TNRURY + 0.5;
					offsetValues[2] = TNRLLX + 0.5;
					offsetValues[3] = TNRLLY - 0.5;
				}
				else if(TNRLLY == IslandLLY)
				{
					offsetValues[0] = TNRLLX + 0.5;
					offsetValues[1] = TNRLLY - 0.5;
					offsetValues[2] = TNRURX - 0.5;
					offsetValues[3] = TNRURY + 0.5;
				}
				else if(TNRLLX == IslandURX)
				{
					offsetValues[0] = TNRURX + 0.5;
					offsetValues[1] = TNRURY - 0.5;
					offsetValues[2] = TNRLLX - 0.5;
					offsetValues[3] = TNRLLY + 0.5;
				}
				else if(TNRURX == IslandLLX)
				{
					offsetValues[0] = TNRLLX - 0.5;
					offsetValues[1] = TNRLLY + 0.5;
					offsetValues[2] = TNRURX + 0.5;
					offsetValues[3] = TNRURY - 0.5;
				}
				
			}
			else																	// this means that it is not a square tunnel
			{
				if(endY > startY) {	// Approaching from the left
					offsetValues[0] = startX - 0.5;
					offsetValues[1] = startY + 0.5;
					offsetValues[2] = endX + 0.5;
					offsetValues[3] = endY - 0.5;
				}
				else if(startY > endY) {	// We are approaching from the right
					offsetValues[0] = startX + 0.5;
					offsetValues[1] = startY - 0.5;
					offsetValues[2] = endX - 0.5;
					offsetValues[3] = endY + 0.5;
				}
			}
		}
		else {
			offsetValues[0] = startX;
			offsetValues[1] = startY;
			offsetValues[2] = endX;
			offsetValues[3] = endY;
		}
		
		return offsetValues;
	}
}