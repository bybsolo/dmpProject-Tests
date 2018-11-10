package testsyallyeeha;

import testsyallyeeha.Odometer_Test;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * this class is used for navigating the robot to a specific point on the grid (coordinates in cm)
 * @author Team12
 *
 */
public class Navigation_Test {
	 
	private static final EV3LargeRegulatedMotor leftMotor = Project_Test.leftMotor; // the motor for the left wheel
	private static final EV3LargeRegulatedMotor rightMotor = Project_Test.rightMotor; // the motor for the right wheel
	private static final EV3LargeRegulatedMotor armMotor = Project_Test.armMotor; // the motor for raising/lowering the																				// arm
	private static final EV3MediumRegulatedMotor hookMotor = Project_Test.hookMotor; // the motor for motorizing the
																						// hooks

	private static final int FORWARD_SPEED = Project_Test.HIGH_SPEED;
	private static final int ROTATE_SPEED = Project_Test.MEDIUM_SPEED;
	private static final int CORRECT_SPEED = Project_Test.LOW_SPEED;
	private static final double WHEEL_RAD = Project_Test.WHEEL_RAD;
	private static final double TRACK = Project_Test.TRACK;
	private static final double TILE_SIZE = Project_Test.TILE_SIZE;
	private static final double OFF_SET = Project_Test.OFF_SET;

	private static final Port portLine = Project_Test.portLeftLine;
	private static final SensorModes myLeftLine = Project_Test.myLeftLine;
	private static final SampleProvider myLeftLineSample = Project_Test.myLeftLineSample;
	private static final float[] sampleLeftLine = Project_Test.sampleLeftLine;

	private static final Port portRightLine = Project_Test.portRightLine;
	private static final SensorModes myRightLine = Project_Test.myRightLine;
	private static final SampleProvider myRightLineSample = Project_Test.myRightLineSample;
	private static final float[] sampleRightLine = Project_Test.sampleRightLine;

	// THINGS TO ADD TO THE REAL ONE
	// //////////////////////////////////////////////////
	private static final double Island_LL_x = Project_Test.Island_LL_x; // x coordinate of the lower left corner of the island
	private static final double Island_LL_y = Project_Test.Island_LL_y; // y coordinate of the lower left corner of the island
	private static final double Island_UR_x = Project_Test.Island_UR_x; // x coordinate of the upper right corner of the island
	private static final double Island_UR_y = Project_Test.Island_UR_y; // y coordinate of the upper right corner of the island

	private static final int Cornor = Project_Test.Cornor; // the starting corner
	private static final double LL_x = Project_Test.LL_x; // x coordinate of the lower left corner of the home section
	private static final double LL_y = Project_Test.LL_y; // y coordinate of the lower left corner of the home section
	private static final double UR_x = Project_Test.UR_x; // x coordinate of the upper right corner of the home section
	private static final double UR_y = Project_Test.UR_y; // y coordinate of the upper right corner of the home section
	private static final double TN_LL_x = Project_Test.TN_LL_x; // x coordinate of the lower left of the tunnel
	private static final double TN_LL_y = Project_Test.TN_LL_y; // y coordinate of the lower left of the tunnel
	private static final double TN_UR_x = Project_Test.TN_UR_x; // x coordinate of the upper right of the tunnel
	private static final double TN_UR_y = Project_Test.TN_UR_y; // y coordinate of the upper right of the tunnel
	private static final double T_x = Project_Test.T_x; // x coordinate of the ring tree
	private static final double T_y = Project_Test.T_y; // y coordinate of the ring tree
	
	private static double currentX;
	private static double currentY;
	private static double currentT;
	/////////////////////////////////////////////////////////////////////
 
	/**
	 * This method is used to drive the robot to the destination point which is
	 * marked as an absolute coordinate (X, Y) 
	 * The method constantly calls the turnTo method to first adjust to the angle
	 * it needs to turn to before moving
	 * @param x the absolute x-coordinate of the destination, an integer value in double format
	 * @param y the absolute y-coordinate of the destination, an integer value in double format
	 * @param odometer the odometer object created in the main class
	 */	  	  
	public static void travelTo(double x, double y, Odometer_Test odometer) {
		
		//get the odometer readings to determine the action
		currentX = odometer.getXYT()[0]; //get the current x position in cm
		currentY = odometer.getXYT()[1]; //get the current y position in cm
		currentT = odometer.getXYT()[2]; //get the current direction in degrees
		boolean alongLine = Math.abs(x*TILE_SIZE - currentX) < 2||Math.abs(y*TILE_SIZE - currentY) < 2;	
		if(alongLine == true) {
			double x1 = x*TILE_SIZE;
			double y1 = y*TILE_SIZE;
			double dDistance = Math.sqrt(Math.pow((x1 - currentX), 2) + Math.pow((y1 - currentY), 2));	
			double dAngle = getDAngle(x1, y1, currentX, currentY);
			
			// reset the motor
			leftMotor.stop(true);
			rightMotor.stop(false);
			for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
				motor.setAcceleration(3000);
			}
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
			}
			
			turnTo(dAngle, currentT); //turn the robot to the direction of the new way point
			
			//move the robot towards the new way point
			//reset the motor
			leftMotor.stop(true);
			rightMotor.stop(false);
			for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			      motor.setAcceleration(3000);
			    }
		    try {
		      Thread.sleep(500);
		    } catch (InterruptedException e) {
		    }

		    leftMotor.setSpeed(FORWARD_SPEED);
		    rightMotor.setSpeed(FORWARD_SPEED);

		    leftMotor.rotate(convertDistance(WHEEL_RAD, dDistance-6), true);
		    rightMotor.rotate(convertDistance(WHEEL_RAD, dDistance-6), false);
		    
		    intersectionCorrection(odometer);
		}
		else {
			//four points
			double X0 = x*TILE_SIZE -15 ; //waypoint x coordinate in cm
			double Y0 = y*TILE_SIZE -15 ; //waypoint y coordinate in cm
			double X1 = x*TILE_SIZE +15 ; //waypoint x coordinate in cm
			double Y1 = y*TILE_SIZE -15 ; //waypoint y coordinate in cm
			double X2 = x*TILE_SIZE +15 ; //waypoint x coordinate in cm
			double Y2 = y*TILE_SIZE +15 ; //waypoint y coordinate in cm
			double X3 = x*TILE_SIZE -15 ; //waypoint x coordinate in cm
			double Y3 = y*TILE_SIZE +15 ; //waypoint y coordinate in cm
			
			double x1 = 0;
			double y1 = 0;
			int point = closestPoint(X0, Y0, X1, Y1, X2, Y2, X3, Y3, currentX, currentY);
			if(point == 0) {
				x1 = X0;
				y1 = Y0;
			}
			if(point == 1) {
				x1 = X1;
				y1 = Y1;
			}
			if(point == 2) {
				x1 = X2;
				y1 = Y2;
			}
			if(point == 3) {
				x1 = X3;
				y1 = Y3;
			}
			
			//calculate the moving distance and turning angle
			double dDistance = Math.sqrt(Math.pow((x1 - currentX), 2) + Math.pow((y1 - currentY), 2));	
			double dAngle = getDAngle(x1, y1, currentX, currentY);
			
			// reset the motor
			leftMotor.stop(true);
			rightMotor.stop(false);
			for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
				motor.setAcceleration(3000);
			}
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
			}
			
			turnTo(dAngle, currentT); //turn the robot to the direction of the new way point
			
			//move the robot towards the new way point
			//reset the motor
			leftMotor.stop(true);
			rightMotor.stop(false);
			for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			      motor.setAcceleration(3000);
			    }
		    try {
		      Thread.sleep(500);
		    } catch (InterruptedException e) {
		    }

		    leftMotor.setSpeed(FORWARD_SPEED);
		    rightMotor.setSpeed(FORWARD_SPEED);

		    leftMotor.rotate(convertDistance(WHEEL_RAD, dDistance), true);
		    rightMotor.rotate(convertDistance(WHEEL_RAD, dDistance), false);
		    
		    intersectionCorrection(odometer);
		}
		
	}
	
	/**
	 * This method returns the angle the robot needs to turn (the smallest angle) to the next point
	 * @param dAngle the angle to turn towards
	 * @param currentT the current direction
	 * @param leftMotor the left motor
	 * @param rightMotor the right motor
	 */
	public static void turnTo (double dAngle, double currentT) {
		//reset the motor
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.setAcceleration(2000);
		    }
	    try {
	      Thread.sleep(1000);
	    } catch (InterruptedException e) {
	      // There is nothing to be done here
	    }
	    
	    //find the smallest angle to turn to the destination
	    double angle1 = dAngle - currentT;
	    double angle2 = (angle1>=0 ? -(360-(Math.abs(angle1))) : (360-(Math.abs(angle1))));
	    double angle = (Math.abs(angle1) < Math.abs(angle2) ? angle1 : angle2); 
	    
	    //start the motors and make the turn
	    leftMotor.setSpeed(ROTATE_SPEED);
	    rightMotor.setSpeed(ROTATE_SPEED);

	    leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, angle), true);
	    rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, angle), false);
	    
	}
	
	/**
	 * This method returns the smallest angle needed to turn to the next waypoint
	 * @param x1 the x of way point
	 * @param y1 the y of way point
	 * @param xc the current x
	 * @param yc the current y
	 * @return the angle to turn in degrees
	 */
	public static double getDAngle(double x1, double y1, double xc, double yc) {
		double xr = x1 - xc;
		double yr = y1 - yc;
		
		//make the angle within 0 to 360
		if(xr == 0 && yr!=0) {
			if(yr>0) return 0; 
			else return 180;
		}
		if(xr != 0 && yr==0) {
			if(xr>0) return 90;
			else return 270;
		}
		if(xr != 0 && yr!=0) {
			if(xr>0 && yr>0) return Math.toDegrees(Math.atan(xr/yr));
			if(xr>0 && yr<0) return Math.toDegrees(Math.atan(Math.abs(yr/xr)))+90;
			if(xr<0 && yr<0) return Math.toDegrees(Math.atan(xr/yr))+180;
			if(xr<0 && yr>0) return Math.toDegrees(Math.atan(Math.abs(yr/xr)))+270;;
		}
		return 0;
	}
	
	/**
	 * converts distance to angle the wheel needs to turn in deg
	 * @param radius of the robot
	 * @param distance of the robot
	 * @return deg to turn
	 */
	public static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	  }
	
	/**
	 * converts angle to the actual angle needs to turn 
	 * @param radius radius of the robot 
	 * @param width radius of the car
	 * @param angle to turn
	 * @return angle that need to turn in deg
	 */
	public static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	  }
	
	
public static void tunnelTravel(Odometer_Test odometer) {
		
		double LL_x = 0; //x coordinate of the lower left corner of the home section
		double LL_y = 0; //y coordinate of the lower left corner of the home section
		double UR_x = 0; //x coordinate of the upper right corner of the home section
		double UR_y = 0; //y coordinate of the upper right corner of the home section
		double TN_LL_x = 0; //x coordinate of the lower left of the tunnel
		double TN_LL_y = 0; //y coordinate of the lower left of the tunnel
		double TN_UR_x = 0; //x coordinate of the upper right of the tunnel
		double TN_UR_y = 0; //y coordinate of the upper right of the tunnel
		
		boolean isTunnelVertical;
		
		if (Math.abs(TN_LL_x - TN_UR_x) ==1) {
			
			isTunnelVertical = true;
			
		} else {
			
			isTunnelVertical = false;
			
		}
		
		double leftPoint_x1;
		double leftPoint_y1;
		
		double rightPoint_x1;
		double rightPoint_y1;
		
		double leftPoint_x2;
		double leftPoint_y2;
		
		double rightPoint_x2;
		double rightPoint_y2;
		
		double dAngle;
		
		int point;
		
		double tunnelLength;
		
		if (isTunnelVertical) {
			
			tunnelLength = Math.abs(TN_LL_y - TN_UR_y);
			
//			if (y < TN_LL_y) {
			
				leftPoint_x1 = TN_LL_x;
				leftPoint_y1 = TN_LL_y - 1;
				
				rightPoint_x1 = TN_LL_x + 1;
				rightPoint_y1 = TN_LL_y - 1;
				
				leftPoint_x2 = TN_UR_x - 1;
				leftPoint_y2 = TN_UR_y + 1;
				
				rightPoint_x2 = TN_UR_x;
				rightPoint_y2 = TN_UR_y + 1;
				
//			} else {
//				
//				leftPoint_x1 = TN_LL_x;
//				leftPoint_y1 = TN_LL_y + TILE_SIZE;
//				
//				rightPoint_x1 = TN_LL_x - TILE_SIZE;
//				rightPoint_y1 = TN_LL_y + TILE_SIZE;
//				
//				leftPoint_x2 = TN_UR_x + TILE_SIZE;
//				leftPoint_y2 = TN_UR_y - TILE_SIZE;
//				
//				rightPoint_x2 = TN_UR_x;
//				rightPoint_y2 = TN_UR_y - TILE_SIZE;
//				
//			}
			
		} else {
			
			tunnelLength = Math.abs(TN_LL_x - TN_UR_x);
			
//			if (x < TN_LL_x) {
				
				leftPoint_x1 = TN_LL_x - 1;
				leftPoint_y1 = TN_LL_y;
				
				rightPoint_x1 = TN_LL_x - 1;
				rightPoint_y1 = TN_LL_y - 1;
				
				leftPoint_x2 = TN_UR_x + 1;
				leftPoint_y2 = TN_UR_y + 1;
				
				rightPoint_x2 = TN_UR_x + 1;
				rightPoint_y2 = TN_UR_y;
				
//			} else {
//				
//				leftPoint_x1 = TN_LL_x  + TILE_SIZE;
//				leftPoint_y1 = TN_LL_y;
//				
//				rightPoint_x1 = TN_LL_x + TILE_SIZE;
//				rightPoint_y1 = TN_LL_y + TILE_SIZE;
//				
//				leftPoint_x2 = TN_UR_x - TILE_SIZE;
//				leftPoint_y2 = TN_UR_y - TILE_SIZE;
//				
//				rightPoint_x2 = TN_UR_x - TILE_SIZE;
//				rightPoint_y2 = TN_UR_y;
//				
//			}
			
		}
		currentX = odometer.getXYT()[0];
		currentY = odometer.getXYT()[1];
		currentT = odometer.getXYT()[2];
		point = closestPoint(leftPoint_x1, leftPoint_y1, rightPoint_x1, rightPoint_y1, leftPoint_x2, leftPoint_y2, rightPoint_x2, rightPoint_y2, currentX, currentY);
		
		if (point == 0) {
			Navigation_Test.travelTo(leftPoint_x1, leftPoint_y1, odometer);
			currentX = odometer.getXYT()[0];
			currentY = odometer.getXYT()[1];
			currentT = odometer.getXYT()[2];
			dAngle = Navigation_Test.getDAngle(rightPoint_x1, rightPoint_y1, currentX, currentY);
			Navigation_Test.turnTo(dAngle, currentT);
			
		} else if (point == 1) {
			Navigation_Test.travelTo(rightPoint_x1, rightPoint_y1, odometer);
			currentX = odometer.getXYT()[0];
			currentY = odometer.getXYT()[1];
			currentT = odometer.getXYT()[2];
			dAngle = Navigation_Test.getDAngle(leftPoint_x1, leftPoint_y1, currentX, currentY);
			Navigation_Test.turnTo(dAngle, currentT);
			
		} else if (point == 2) {
			Navigation_Test.travelTo(leftPoint_x2, leftPoint_y2, odometer);
			currentX = odometer.getXYT()[0];
			currentY = odometer.getXYT()[1];
			currentT = odometer.getXYT()[2];
			dAngle = Navigation_Test.getDAngle(rightPoint_x2, rightPoint_y2, currentX, currentY);
			Navigation_Test.turnTo(dAngle, currentT);
			
		} else {
			
			Navigation_Test.travelTo(rightPoint_x2, rightPoint_y2, odometer);
			currentX = odometer.getXYT()[0];
			currentY = odometer.getXYT()[1];
			currentT = odometer.getXYT()[2];
			
			dAngle = Navigation_Test.getDAngle(leftPoint_x2, leftPoint_y2, currentX, currentY);
			Navigation_Test.turnTo(dAngle, currentT);
			
		}
		
		leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, TILE_SIZE/2), true);
		rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, TILE_SIZE/2), false);
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		currentX = odometer.getXYT()[0];
		currentY = odometer.getXYT()[1];
		currentT = odometer.getXYT()[2];
		
		if (isTunnelVertical) {
			
			if (point == 0 || point == 1) {
				
				dAngle = Navigation_Test.getDAngle(leftPoint_x2+ 0.5, leftPoint_y2, currentX, currentY);
				Navigation_Test.turnTo(dAngle, currentT);
				
			} else {
				
				dAngle = Navigation_Test.getDAngle(leftPoint_x1 + 0.5, leftPoint_y1, currentX, currentY);
				Navigation_Test.turnTo(dAngle, currentT);
				
			}
			
		} else {
			
			if (point == 0 || point == 1) {
				
				dAngle = Navigation_Test.getDAngle(leftPoint_x2, leftPoint_y2  + 0.5, currentX, currentY);
				Navigation_Test.turnTo(dAngle, currentT);
				
			} else {
				
				dAngle = Navigation_Test.getDAngle(leftPoint_x1, leftPoint_y1 + 0.5, currentX, currentY);
				Navigation_Test.turnTo(dAngle, currentT);
				
			}
		}
		
		leftMotor.rotate(-Navigation_Test.convertDistance(WHEEL_RAD, 7), true);
		rightMotor.rotate(-Navigation_Test.convertDistance(WHEEL_RAD, 7), false);
		
		lineCorrection(odometer);
		
		leftMotor.rotate(-Navigation_Test.convertDistance(WHEEL_RAD, (tunnelLength+1.8)*TILE_SIZE), true);
		rightMotor.rotate(-Navigation_Test.convertDistance(WHEEL_RAD, (tunnelLength + 1.8)*TILE_SIZE), false);
		
		lineCorrection(odometer);
		
	}		
	
	
	public static void intersectionCorrection(Odometer_Test odometer) {
		leftMotor.stop();
		rightMotor.stop();
		double fromOrientation = odometer.getXYT()[2];
		double toOrientation;
		if(fromOrientation >=0 && fromOrientation <90) toOrientation = 90;
		else if (fromOrientation>=90 && fromOrientation<180) toOrientation = 180;
		else if (fromOrientation >=180 && fromOrientation<270) toOrientation = 270; 
		else toOrientation = 0;
		leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK,smallAngle(fromOrientation, toOrientation)));
		rightMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK,smallAngle(fromOrientation, toOrientation)));
		
		boolean left = false;
		boolean right  =false;
		while (left == false && right == false) {
			leftMotor.forward();
			rightMotor.forward();
			if (Localizer_Test.lineDetection() ==3) {
				leftMotor.stop();
				rightMotor.stop();
				break;
			}
			else if (Localizer_Test.lineDetection()==1) {
				leftMotor.stop();
				left = true;
				//break;
		
			}
			else if (Localizer_Test.lineDetection()==2) {
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
			if (Localizer_Test.lineDetection() ==3) {
				leftMotor.stop();
				rightMotor.stop();
				break;
			}
			
			else if (Localizer_Test.lineDetection()==1) {
				leftMotor.stop();
				left = true;
				
			}
			else if (Localizer_Test.lineDetection()==2) {
				rightMotor.stop();
				right = true;
			}
		}
		

		leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, OFF_SET), true);
		rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, OFF_SET), false);
	}
	
	public static void lineCorrection(Odometer_Test odometer) {
		leftMotor.stop();
		rightMotor.stop();
		leftMotor.setSpeed(CORRECT_SPEED);
		rightMotor.setSpeed(CORRECT_SPEED);
		
		boolean left = false;
		boolean right = false;
		while (left == false && right == false) {
			leftMotor.forward();
			rightMotor.forward();
			if (Localizer_Test.lineDetection() ==3) {
				leftMotor.stop();
				rightMotor.stop();
				break;
			}
			else if (Localizer_Test.lineDetection()==1) {
				leftMotor.stop();
				left = true;
		
			}
			else if (Localizer_Test.lineDetection()==2) {
				rightMotor.stop();
				right = true;
			}
		}
		leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, OFF_SET), true);
		rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, OFF_SET), false);
		
		leftMotor.stop();
		rightMotor.stop();
		double currentT = odometer.getXYT()[2];
		if(currentT >=45 && currentT<135) odometer.setTheta(90);
		else if (currentT>=135 && currentT<225) odometer.setTheta(180);
		else if (currentT>=225 && currentT<315) odometer.setTheta(180);
		else odometer.setTheta(0);
		
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
	}
	
	/**
	 * This method is used for fast calculation of the smallest angle required to turn to turn from one orientation to another
	 * <P>
	 * This method differs from the getDAngle() and turnTo() methods in that it turns to arbitrary orientations rather than to a specific waypoint point 
	 * @param fromOrientation the current orientation
	 * @param toOrientation the orientation to orient to 
	 * @return
	 */
	public static double smallAngle(double fromOrientation, double toOrientation) {
	    double angle1 = toOrientation - fromOrientation;
	    double angle2 = (angle1>=0 ? -(360-(Math.abs(angle1))) : (360-(Math.abs(angle1))));
	    return (Math.abs(angle1) < Math.abs(angle2) ? angle1 : angle2); 
	}
	

	public static int closestPoint(double X0, double Y0, double X1, double Y1, double X2, double Y2, double X3, double Y3, double x, double y) {

		double[] distance = new double[4];

		distance[0] = Math.sqrt(Math.pow((X0*TILE_SIZE - x), 2) + Math.pow((Y0*TILE_SIZE - y), 2));
		distance[1] = Math.sqrt(Math.pow((X1*TILE_SIZE - x), 2) + Math.pow((Y1*TILE_SIZE - y), 2));
		distance[2] = Math.sqrt(Math.pow((X2*TILE_SIZE - x), 2) + Math.pow((Y2*TILE_SIZE - y), 2));
		distance[3] = Math.sqrt(Math.pow((X3*TILE_SIZE - x), 2) + Math.pow((Y3*TILE_SIZE - y), 2));

		int point = 0;
		for (int i = 1; i < 4; i++) {
			if (distance[point] > distance[i]) {
				point = i;
			}
		}
		return point;

	}
}
