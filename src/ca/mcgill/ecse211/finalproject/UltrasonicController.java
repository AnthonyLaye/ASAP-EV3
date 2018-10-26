package ca.mcgill.ecse211.finalproject;

public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
