package testsyallyeeha;

import lejos.hardware.sensor.*;
import testsyallyeeha.Odometer_Test;
import testsyallyeeha.OdometerExceptions_Test;

import java.util.Map;
import java.util.concurrent.TimeUnit;

import ca.mcgill.ecse211.WiFiClient.WiFi;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

/**
 * This is the main class for project SCARAB; 
 * It contains almost all the parameters/sensor initiations, as well as the parameters needed for the operation 
 * It will interact with the user through the EV3 module's display and buttons
 * The routines are performed as consecutive methods within threads
 * @author Team12
 *
 */

public class Project_Test {
	//The parameters for driving the robot
	//The island parameters
	public static double Island_LL_x = 0; //x coordinate of the lower left corner of the island
	public static double Island_LL_y = 0; //y coordinate of the lower left corner of the island
	public static double Island_UR_x = 4; //x coordinate of the upper right corner of the island
	public static double Island_UR_y = 5; //y coordinate of the upper right corner of the island
	
	//The team-specific parameters
	public static int corner = 2; //the starting corner
	public static double LL_x = 6; //x coordinate of the lower left corner of the home section
	public static double LL_y = 4; //y coordinate of the lower left corner of the home section
	public static double UR_x = 8; //x coordinate of the upper right corner of the home section
	public static double UR_y = 8; //y coordinate of the upper right corner of the home section
	public static double TN_LL_x = 4; //x coordinate of the lower left of the tunnel
	public static double TN_LL_y = 4; //y coordinate of the lower left of the tunnel
	public static double TN_UR_x = 6; //x coordinate of the upper right of the tunnel
	public static double TN_UR_y = 5; //y coordinate of the upper right of the tunnel
	public static double T_x = 3; //x coordinate of the ring tree
	public static double T_y = 2; //y coordinate of the ring tree
	
	//The operating parameters for arm and hook (grabbing mechanism)
	public static final int HOOK_SPEED = 10; //this is the speed used for the motor 
	public static final int ARM_SPEED = 80; //this is the speed for the arm for the arm motor 
	public static final int HOOK_ANGLE = -35; //this is the angle which the hook will open/close	
	public static final int LOW_ANGLE = 120; //the angle the arm motor needs to turn to reach lowly-hanged rings, with respect to the initial position 
	public static final int UNLOAD_ANGLE = 110; //the angle the arm motor needs to turn to unload the ring(s), with respect to the initial position 
	
	//The operating parameters for the navigation and diver system
	public static final double OFF_SET = 2.5; //this is the offset from the 2 line-detecting light sensors to the wheel base
	public static final int LOW_SPEED = 200; //this is the slow speed for precise movement 
	public static final int MEDIUM_SPEED = 300; //this is the medium speed for intermediate movement
	public static final int HIGH_SPEED = 400; //this is the fast motor speed for less precious, faster movement (long distance travel)
	public static final double WHEEL_RAD = 2.085; //the wheel radius of the wheels
	public static final double TRACK = 15.2; //the wheel base of the robot
	public static final double TILE_SIZE = 30.48; //the tile length of the grid
	public static final double HIGH_PROBE = 11;
	public static final double LOW_PROBE = 11.5;
	public static final int DISTANCE = 30; //distance from the wall used by the ultrasonic sensor during the ultrasonic localization 
	
	//create port and object for the motors (4 in total)
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A")); //the motor for the left wheel
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B")); //the motor for the right wheel
	public static final EV3LargeRegulatedMotor armMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C")); //the motor for raising/lowering the arm 
	public static final EV3MediumRegulatedMotor hookMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("D")); //the motor for motorizing the hooks 
	
	//create object for the lcd display
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	
	// create port and object for the ultrasonic sensor in the front (used for ultrasonic localization and when approaching the ring)
	public static final Port usPort = LocalEV3.get().getPort("S1");
	@SuppressWarnings("resource") // Because we don't bother to close this resource
	public static SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
	public static SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
	public static final float[] usData = new float[usDistance.sampleSize()]; //create an array for the sensor readings
	
	// create port and object for the light sensor for color recognition which is installed on the arm
	public static final Port portColor = LocalEV3.get().getPort("S2"); // get the port for the light (color sensor)
	public static final SensorModes myColor = new EV3ColorSensor(portColor); // create the color sensor object
	public static final SampleProvider myColorSample = myColor.getMode("RGB"); //set to RGB mode
	public static final float[] sampleColor = new float[3]; // create an array for the sensor readings
	
	//create port and object for the first light sensor for line detection in the left front
	public static final Port portLeftLine = LocalEV3.get().getPort("S3"); // get the port for the left front light sensor
	public static final SensorModes myLeftLine = new EV3ColorSensor(portLeftLine); // create the light sensor object
	public static final SampleProvider myLeftLineSample = myLeftLine.getMode("Red"); //set to Red mode
	public static final float[] sampleLeftLine = new float[myLeftLine.sampleSize()]; // create an array for the sensor readings 
	
	//create port and object for the second light sensor for line detection in the right front
	public static final Port portRightLine = LocalEV3.get().getPort("S4"); // get the port for the right front light sensor
	public static final SensorModes myRightLine = new EV3ColorSensor(portRightLine); // create the light sensor object
	public static final SampleProvider myRightLineSample = myRightLine.getMode("Red"); //set to Red mode
	public static final float[] sampleRightLine = new float[myRightLine.sampleSize()]; // create an array for the sensor readings 
																			
//
//	
//	/**
//	 * This is the main method for the lab, it will prompt the user to choose which functionality to execute
//	 * in a menu, and initiates the threads needed for the lab
//	 * @param args
//	 * @throws OdometerExceptions_Test
//	 */
//
//	public static void main(String[] args) throws OdometerExceptions_Test {
//		Map data = WiFi.Wifi();
//
//		int redTeam = ((Long) data.get("RedTeam")).intValue();
//		int greenTeam = ((Long) data.get("GreenTeam")).intValue();
//
//		if (redTeam == 12) {
//
//			corner = ((Long) data.get("RedCorner")).intValue(); // the starting corner
//			LL_x = ((Long) data.get("Red_LL_x")).intValue(); // x coordinate of the lower left corner of the home
//																// section
//			LL_y = ((Long) data.get("Red_LL_y")).intValue(); // y coordinate of the lower left corner of the home
//																// section
//			UR_x = ((Long) data.get("Red_UR_x")).intValue(); // x coordinate of the upper right corner of the home
//																// section
//			UR_y = ((Long) data.get("Red_UR_y")).intValue(); // y coordinate of the upper right corner of the home
//																// section
//			TN_LL_x = ((Long) data.get("TNR_LL_x")).intValue(); // x coordinate of the lower left of the tunnel
//			TN_LL_y = ((Long) data.get("TNR_LL_y")).intValue(); // y coordinate of the lower left of the tunnel
//			TN_UR_x = ((Long) data.get("TNR_UR_x")).intValue(); // x coordinate of the upper right of the tunnel
//			TN_UR_y = ((Long) data.get("TNR_UR_y")).intValue(); // y coordinate of the upper right of the tunnel
//			T_x = ((Long) data.get("TR_x")).intValue(); // x coordinate of the ring tree
//			T_y = ((Long) data.get("TR_y")).intValue(); // y coordinate of the ring tree
//
//		} else {
//
//			corner = ((Long) data.get("GreenCorner")).intValue(); // the starting corner
//			LL_x = ((Long) data.get("Green_LL_x")).intValue(); // x coordinate of the lower left corner of the home
//																// section
//			LL_y = ((Long) data.get("Green_LL_y")).intValue(); // y coordinate of the lower left corner of the home
//																// section
//			UR_x = ((Long) data.get("Green_UR_x")).intValue(); // x coordinate of the upper right corner of the home
//																// section
//			UR_y = ((Long) data.get("Green_UR_y")).intValue(); // y coordinate of the upper right corner of the home
//																// section
//			TN_LL_x = ((Long) data.get("TNG_LL_x")).intValue(); // x coordinate of the lower left of the tunnel
//			TN_LL_y = ((Long) data.get("TNG_LL_y")).intValue(); // y coordinate of the lower left of the tunnel
//			TN_UR_x = ((Long) data.get("TNG_UR_x")).intValue(); // x coordinate of the upper right of the tunnel
//			TN_UR_y = ((Long) data.get("TNG_UR_y")).intValue(); // y coordinate of the upper right of the tunnel
//			T_x = ((Long) data.get("TG_x")).intValue(); // x coordinate of the ring tree
//			T_y = ((Long) data.get("TG_y")).intValue(); // y coordinate of the ring tree
//
//		}
//
//		Island_LL_x = ((Long) data.get("Island_LL_x")).intValue(); // x coordinate of the lower left corner of the
//																	// island
//		Island_LL_y = ((Long) data.get("Island_LL_y")).intValue(); // y coordinate of the lower left corner of the
//																	// island
//		Island_UR_x = ((Long) data.get("Island_UR_x")).intValue(); // x coordinate of the upper right corner of the
//																	// island
//		Island_UR_y = ((Long) data.get("Island_UR_y")).intValue(); // y coordinate of the upper right corner of the
//																	// island
//
//		final Odometer_Test odometer = Odometer_Test.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
//
//		Thread odoThread = new Thread(odometer);
//		odoThread.start();
//
//		(new Thread() {
//			public void run() {
//				// add method :
//				Localizer_Test.fallingEdge(odometer);
//				Localizer_Test.lightLocalizeLite(odometer);
//				Navigation_Test.tunnelTravel(odometer);
//				Grabber_Test.travelToTree(odometer);
//				Navigation_Test.tunnelTravel(odometer);
//				Navigation_Test.cornerTravel(odometer);
//
//			}
//		}).start();
//
//	}
	
	public static void main(String[] args) throws OdometerExceptions_Test{
		
		final Odometer_Test odometer = Odometer_Test.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		int buttonChoice;
	
		do {
			lcd.clear();
			lcd.drawString("^ ..test1..", 0,1);
		    lcd.drawString("> ..test2..", 0,2);
		    lcd.drawString("v ..test3..", 0,3);
		    lcd.drawString("< ..test4..", 0,4);
		    buttonChoice = Button.waitForAnyPress(); 
		} while (buttonChoice!=Button.ID_UP && buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_DOWN && buttonChoice != Button.ID_LEFT);

		Thread odoThread = new Thread(odometer);
		odoThread.start();
		
		//test 1 
		if(buttonChoice==Button.ID_UP) {
			(new Thread() {
				public void run() {
					Testers_Test.newLine(odometer);
					
				}
			}).start();
		}
		//test 2
		if(buttonChoice==Button.ID_RIGHT) {
			(new Thread() {
				public void run() {
					//add method : driver wheel base test
					Testers_Test.trackCheck();
				}
			}).start();
		}
		//test 3
		if(buttonChoice==Button.ID_DOWN) {
			(new Thread() {
				public void run() {
					//add method : 
					Localizer_Test.fallingEdge(odometer);
					Localizer_Test.lightLocalizeLite(odometer);
					Navigation_Test.tunnelTravel(odometer);
					Grabber_Test.travelToTree(odometer);	
					Navigation_Test.tunnelTravel(odometer);
				    Navigation_Test.cornerTravel(odometer);
				
				}
			}).start();
		}
		//test 4
		if(buttonChoice==Button.ID_LEFT) {
			(new Thread() {
				public void run() {
					//add method here: test the new filter 
					Testers_Test.falseColor();
				}
			}).start();
		}
		
		//stop the system when the exit button is pressed
	    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
	    System.exit(0);
	}
}
