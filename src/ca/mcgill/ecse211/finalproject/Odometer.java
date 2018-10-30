/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.finalproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * 
 * This class implements odometer methods to compute coordinates of the robot
 */
public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private int lastTachoLeft;
  private int lastTachoRight;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
 

  private final double TRACK;
  private final double WHEEL_RAD;


  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;
    
    this.lastTachoLeft = leftMotor.getTachoCount();      		// get tacho counts
    this.lastTachoRight = rightMotor.getTachoCount();

  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }
  
  /**
   * This method is where the logic for the odometer will run. Based on the current and previous tacho counts,
   * it calculates the displacement for each coordinate. It then uses the OdometerData class to update dX, dY and dTheta.
   * There is no input or output to this function.
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();

      double distL, distR, deltaD, deltaT, dX, dY;
      
      this.leftMotorTachoCount = this.leftMotor.getTachoCount();
      this.rightMotorTachoCount = this.rightMotor.getTachoCount();
      
      distL = Math.PI * WHEEL_RAD * (this.leftMotorTachoCount - this.lastTachoLeft) / 180;
      distR = Math.PI * WHEEL_RAD * (this.rightMotorTachoCount - this.lastTachoRight) / 180;
      
      this.lastTachoLeft = this.leftMotorTachoCount;
      this.lastTachoRight = this.rightMotorTachoCount;
      
      deltaD = 0.5 * (distL + distR);
      deltaT = Math.toDegrees(((distL - distR) / TRACK));
      
      double[] position = getXYT();
      dX = deltaD * Math.sin(Math.toRadians(position[2] + deltaT));
      dY = deltaD * Math.cos(Math.toRadians(position[2] + deltaT));
      
      odo.update(dX, dY, deltaT);

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

}
