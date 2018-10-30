package ca.mcgill.ecse211.wallfollowing;


import java.io.FileWriter;
import java.io.IOException;


import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.*;
import lejos.robotics.navigation.Navigator;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

public class ColorSampler {
	
	private static final EV3ColorSensor LS = 
			new EV3ColorSensor(LocalEV3.get().getPort("S3"));
	
	  public static void main(String[] args) {
			SensorMode color = LS.getRGBMode();
		    float[] sample = new float[3];
		    int i = 0;
		    System.out.println("R, G, B");
		    while (i < 200) {
		    		i++;
			    color.fetchSample(sample, 0);
			    double red = sample[0] * 1000;	//Scale for easier numbers to read
			    double green = sample[1] * 1000;
			    double blue = sample[2] * 1000;
			    System.out.println(red + "," + green + "," + blue);
		  }
		  

	  }

}
