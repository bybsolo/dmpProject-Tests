package testsyallyeeha;

import java.util.concurrent.TimeUnit;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;

/**
 * This class is used for localizing the robot using the ultrasonic sensor and the 2 line detection sensors
 * @author team12
 */

public class Localizer_Test {

	private static final Port portLeftLine = Project_Test.portLeftLine;
	private static final SensorModes myLeftLine = Project_Test.myLeftLine;
	private static final SampleProvider myLeftLineSample = Project_Test.myLeftLineSample;
	private static final float[] sampleLeftLine = new float[1];
	
	private static final Port portRightLine = Project_Test.portRightLine;
	private static final SensorModes myRightLine = Project_Test.myRightLine;
	private static final SampleProvider myRightLineSample = Project_Test.myRightLineSample;
	private static final float[] sampleRightLine = new float[1];
	
	private static final Port portColor = Project_Test.portColor; // get the port for the light (color sensor)
	private static final SensorModes myColor = Project_Test.myColor; // create the color sensor object
	private static final SampleProvider myColorSample = Project_Test.myColorSample; //set to RGB mode
	private static final float[] sampleColor = Project_Test.sampleColor; // create an array for the sensor readings
	
	public static final Port usPort = Project_Test.usPort ;
	@SuppressWarnings("resource") // Because we don't bother to close this resource
	public static SensorModes usSensor = Project_Test.usSensor; // usSensor is the instance
	public static SampleProvider usDistance = Project_Test.usDistance; // usDistance provides samples from
	public static final float[] usData = Project_Test.usData; //create an array for the sensor readings
	
	private static final EV3LargeRegulatedMotor leftMotor = Project_Test.leftMotor; //the motor for the left wheel
	private static final EV3LargeRegulatedMotor rightMotor = Project_Test.rightMotor; //the motor for the right wheel
	private static final EV3LargeRegulatedMotor armMotor = Project_Test.armMotor; //the motor for raising/lowering the arm 
	private static final EV3MediumRegulatedMotor hookMotor =Project_Test.hookMotor; //the motor for motorizing the hooks 
	
	private static final double OFF_SET = Project_Test.OFF_SET;
	private static final double TILE_SIZE = Project_Test.TILE_SIZE;
	private static final int THRESHOLD = Project_Test.THRESHOLD;
	private static final double DISTANCE = Project_Test.DISTANCE;
	private static final double WHEEL_RAD = Project_Test.WHEEL_RAD; 
	private static final double TRACK = Project_Test.TRACK;  
	private static final int LOW_SPEED = Project_Test.LOW_SPEED; //this is the slow speed for precise movement 
	private static final int MEDIUM_SPEED = Project_Test.MEDIUM_SPEED; //this is the medium speed for intermediate movement
	private static final int HIGH_SPEED = Project_Test.HIGH_SPEED; //this is the fast motor speed for less precious, faster movement (long distance travel)

	
	
	/**
	 * This is the falling edge method used when the robots starts facing away from the wall (distance larger than D) 
	 * It will first turn clockwise to detect the back wall (angle: alpha)
	 * then counter-clockwise to detect the left wall (angle: beta)
	 * @param odometer the odometer used to determine the robots orientation
	 */
	static void fallingEdge(Odometer_Test odometer) {
		int distance;
		usDistance.fetchSample(usData, 0);
		distance = (int) (usData[0] * 100.0); 
		
		//check is it is indeed facing away from the wall
		if (distance > DISTANCE) {
		    double alpha =0; //the angle when the back wall is detected
		    double beta =0; //the angle when the left wall is detected
		    
		    //turn clockwise to find the back wall and the corresponding angle alpha
      	    leftMotor.stop();
      	    rightMotor.stop();
	  	    leftMotor.setSpeed(HIGH_SPEED); 
	  	    rightMotor.setSpeed(HIGH_SPEED);
	  	    
	  	    //detect alpha
	  	    boolean back_detected = false;
	  	    while(back_detected == false) {
			    leftMotor.forward();
			    rightMotor.backward();
	  	    	
	  	    	usDistance.fetchSample(usData, 0);
	  		    distance = (int) (usData[0] * 100.0); 		    
	  		    if(distance <= DISTANCE) {
	  		    	alpha = odometer.getXYT()[2];
	  		    	back_detected = true;
	  		    }
	  	    }	  
	  	    
	  	    //turn the robot counter-clockwise out of the alpha detection zone
	  	    leftMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 45), true);
	  	    rightMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 45), false);
	  	    
	  	    //turn counter-clockwise to find the left wall and the corresponding angle beta
      	    leftMotor.stop();
      	    rightMotor.stop();
	  	    leftMotor.setSpeed(HIGH_SPEED); 
	  	    rightMotor.setSpeed(HIGH_SPEED);
	  	    
	  	    //detect beta
	  	    boolean left_detected = false;
	  	    while(left_detected == false) {
			    leftMotor.backward();
			    rightMotor.forward();
			    
	  	    	usDistance.fetchSample(usData, 0);
	  		    distance = (int) (usData[0] * 100.0); 		    
	  		    if(distance <= DISTANCE) {
	  		    	beta = odometer.getXYT()[2];
	  		    	left_detected = true;
	  		    }
	  		 
	  	    }	  
	  	    
	  	    //move the sensor away from the beta detection zone
	  	    leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 45), true);
	  	    rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 45), false);
	  	    
      	    leftMotor.stop();
      	    rightMotor.stop();
	  	    leftMotor.setSpeed(HIGH_SPEED); 
	  	    rightMotor.setSpeed(HIGH_SPEED);
	  	    
	  	    //calculate the change in angle and then turn to the adjusted orientation
	  	    //delta is the angle of the real 0 axis when we use initial orientation as 0 axis 
	  	    double delta = (alpha+beta)/2 -225; 
	  	    if (delta<0) delta = 360+delta;
	  	    //turn to the real zero axis
	  	    leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, (delta-odometer.getXYT()[2])), true);
		    rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, (delta-odometer.getXYT()[2])), false);	    
		    
		    //correct the odometer orientation to zero
		    odometer.setTheta(0);
		}
		//invoke rising edge if it is facing the wall; use rising edge instead
		else {
			risingEdge(odometer);
		}
		
		

	}
	

	/**
	 * This is the rising edge method used when the robots starts facing the wall (distance smaller than D); 
	 * It will first turn clockwise to detect the left wall (angle: beta)
	 * then counter-clockwise to detect the back wall (angle: alpha)
	 * @param odometer the odometer used to determine the robots orientation
	 */	
	static void risingEdge(Odometer_Test odometer) {
	    int distance;    
	    usDistance.fetchSample(usData, 0);
		distance = (int) (usData[0] * 100.0); 	
		
		//check if indeed is facing the wall, only then use rising edge
        if(distance < DISTANCE) {
    	    double alpha =0; //the angle when the back wall is detected
    	    double beta =0; //the angle when the left wall is detected
    	    
    	    //turn clockwise to find the left wall and the corresponding angle beta
      	    leftMotor.stop();
      	    rightMotor.stop();
    	    leftMotor.setSpeed(HIGH_SPEED); 
      	    rightMotor.setSpeed(HIGH_SPEED);
      	    
      	    //detect beta
      	    boolean left_detected = false;
      	    while(left_detected == false) {
    		    leftMotor.forward();
    		    rightMotor.backward();
      	    	
      	    	usDistance.fetchSample(usData, 0);
      		    distance = (int) (usData[0] * 100.0); 		    
      		    if(distance >= DISTANCE) {
      		    	System.out.println("detect 1st rising");
      		    	beta = odometer.getXYT()[2];
      		    	left_detected = true;
      		    }
      	    }	  

      	    
      	    //turn the robot counter-clockwise out of the beta detection zone
      	    leftMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 45), true);
      	    rightMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 45), false);
      	    
      	    //turn counter-clockwise to find the back wall and the corresponding angle alpha
      	    leftMotor.stop();
      	    rightMotor.stop();	    
      	    leftMotor.setSpeed(HIGH_SPEED); 
      	    rightMotor.setSpeed(HIGH_SPEED);
    	    
      	    //detect alpha
      	    boolean back_detected = false;
      	    while(back_detected == false) {
    		    leftMotor.backward();
    		    rightMotor.forward();
    		    
      	    	usDistance.fetchSample(usData, 0);
      		    distance = (int) (usData[0] * 100.0); 		    
      		    if(distance >= DISTANCE) {
      		    	System.out.println("detect 2nd rising");
      		    	alpha = odometer.getXYT()[2];
      		    	back_detected = true;
      		    }
      		 
      	    }	  
      	    
      	    //turn the robot counter-clockwise out of the beta detection zone
      	    leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 45), true);
      	    rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 45), false);
      	    
      	    leftMotor.stop();
      	    rightMotor.stop();
      	    leftMotor.setSpeed(HIGH_SPEED); 
      	    rightMotor.setSpeed(HIGH_SPEED);
      	    
      	    //calculate the change in angle and then turn to the adjusted orientation
      	    //delta is the angle of the real 0 axis in the system where the original heading was the zero axis.
      	    double delta = (alpha+beta)/2 -45;
      	    if(delta <0) delta = 360+delta;
      	    //turn to the real 0 axis
      	    leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, (delta-odometer.getXYT()[2])), true);
    	    rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, (delta-odometer.getXYT()[2])), false);
    	    
    	    //correct the odometer orientation to zero
    	    odometer.setTheta(0);

        }
        //if the robot is actually facing away from the wall rather than facing the wall, it will correct itself and use fallingEdge instead
		else {
			fallingEdge(odometer);
		}

	}

	 /**
	  * The lineDetection() method is used to determine whether the left or right line detection sensor have picked up the line readings.
	  * The detection situation is represented as integer values, for easier implementation of the method's returned result.
	  * If only the left sensor have detected a line, the situation is labeled as 1.
	  * If only the right sensor have detected a line, the situation is labeled as 1.
	  * If both sensors have detected a line, the situation is labeled as 3.
	  * @return the current situation regarding line detection, represented as integers 
	  */
	public static int lineDetection() {
		//add a  differential filter
		int[] readingsLeft = new int[4];
		int[] readingsRight = new int[4];
		int[] filterCoeff = {1,1,-1,-1};
		int differentialLeft = 0;
		int differentialRight = 0;
		for(int i = 0; i<4; i++) {
			myLeftLineSample.fetchSample(sampleLeftLine, 0);
			readingsLeft[i] = (int)(sampleLeftLine[0]*1000.0);
			myRightLineSample.fetchSample(sampleRightLine, 0);
			readingsRight[i] = (int)(sampleRightLine[0]*1000.0);
			System.out.println();
			System.out.println();
			differentialLeft += readingsLeft[i]*filterCoeff[i];
			differentialRight += readingsRight[i]*filterCoeff[i];
		}
		if(differentialLeft >= 50 && differentialRight >= 50) return 3;
		else if(differentialLeft >= 50) return 1;
		else if(differentialRight >= 50) return 2;
		
		return 0;
	}

	
	/**
	 * The lightLocalizerLite() method is simplified version of light localization.
	 * It utilizes the two light sensors installed in the front of the robot, and performs direction and position adjustment on the grid lines.
	 * Notice the light localization should be performed only after the ultrasonic localization orients the robot to close to the 0-axis.
	 * @param odometer the odometer used by the robot
	 */
	public static void lightLocalizeLite(Odometer_Test odometer) {
		leftMotor.setSpeed(75);
		rightMotor.setSpeed(75);
		boolean left = false;
		boolean right = false;
		leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), true);
		rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), false);
		
		while (left == false && right == false) {
			leftMotor.forward();
			rightMotor.forward();
			if (lineDetection() ==3) {
				leftMotor.stop();
				rightMotor.stop();
				break;
			}
			else if (lineDetection()==1) {
				leftMotor.stop();
				left = true;
				//break;
		
			}
			else if (lineDetection()==2) {
				rightMotor.stop();
				right = true;
				
			}
		}
		

		leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, OFF_SET), true);
		rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, OFF_SET), false);
		
		leftMotor.stop();
		rightMotor.stop();
		
		leftMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), true);
		rightMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, 90), false);
		
		left = false;
		right = false;
		
		while (left == false && right == false) {
			leftMotor.forward();
			rightMotor.forward();
			if (lineDetection() ==3) {
				leftMotor.stop();
				rightMotor.stop();
				break;
			}
			
			else if (lineDetection()==1) {
				leftMotor.stop();
				left = true;
				
			}
			else if (lineDetection()==2) {
				rightMotor.stop();
				right = true;
			}
		}

		leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, OFF_SET), true);
		rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, OFF_SET), false);
		
		if(Project_Test.Cornor == 0) odometer.setXYT(TILE_SIZE, TILE_SIZE, 0);
		if(Project_Test.Cornor == 1) odometer.setXYT(7*TILE_SIZE, TILE_SIZE, 270);
		if(Project_Test.Cornor == 2) odometer.setXYT(7*TILE_SIZE, 7*TILE_SIZE, 180);
		if(Project_Test.Cornor == 3) odometer.setXYT(TILE_SIZE, 7*TILE_SIZE, 90);
		
	}
	
	
}
