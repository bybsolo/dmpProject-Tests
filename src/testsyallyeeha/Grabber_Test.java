package testsyallyeeha;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

/**
 * This class contains methods for the ring fetching routine and mechanism
 * 
 * @author Team12
 *
 */
public class Grabber_Test {
	private static final EV3LargeRegulatedMotor leftMotor = Project_Test.leftMotor; // the motor for the left wheel
	private static final EV3LargeRegulatedMotor rightMotor = Project_Test.rightMotor; // the motor for the right wheel
	private static final EV3LargeRegulatedMotor armMotor = Project_Test.armMotor; // the motor for raising/lowering the
																					// arm
	private static final EV3MediumRegulatedMotor hookMotor = Project_Test.hookMotor; // the motor for motorizing the
																						// hooks
	private static final int ARM_SPEED = Project_Test.ARM_SPEED; // this is the speed for the arm for the arm motor
	private static final int HOOK_SPEED = Project_Test.HOOK_SPEED; // this is the angle which the hook will open/close
	private static final int HOOK_ANGLE = Project_Test.HOOK_ANGLE; // this is the angle which the hook will open/close
	private static final int LOW_ANGLE = Project_Test.LOW_ANGLE; // the angle the arm motor needs to turn to reach
																	// lowly-hanged rings, with respect to the initial
																	// position
	private static final int HIGH_ANGLE = Project_Test.HIGH_ANGLE; // the angle the arm motor needs to turn to reach
																	// highly-hanged rings, with respect to the initial
																	// position
	private static final int UNLOAD_ANGLE = Project_Test.UNLOAD_ANGLE; // the angle the arm motor needs to turn to
																		// unload the ring(s), with respect to the
																		// initial position

	private static final int FORWARD_SPEED = Project_Test.HIGH_SPEED;
	private static final int ROTATE_SPEED = Project_Test.MEDIUM_SPEED;
	private static final int PROBE_SPEED  = Project_Test.LOW_SPEED;
	private static final double WHEEL_RAD = Project_Test.WHEEL_RAD;
	private static final double TRACK = Project_Test.TRACK;
	private static final double TILE_SIZE = Project_Test.TILE_SIZE;
	private static final double OFF_SET = Project_Test.OFF_SET;
	private static final double HIGH_PROBE = Project_Test.HIGH_PROBE;
	private static final double LOW_PROBE = Project_Test.LOW_PROBE;
	

	private static final double T_x = Project_Test.T_x;
	private static final double T_y = Project_Test.T_y;
	
	public static boolean FOUND = false;		///

	public static void travelToTree(Odometer_Test odometer) {
		
		double[] odometerData = odometer.getXYT();
		double x = odometerData[0];
		double y = odometerData[1];
		double t;

		int point;

		double X0 = T_x;
		double Y0 = T_y - 1;

		double X1 = T_x + 1;
		double Y1 = T_y;

		double X2 = T_x;
		double Y2 = T_y + 1;

		double X3 = T_x - 1;
		double Y3 = T_y;

		int color;
		
		point = Navigation_Test.closestPoint(X0, Y0, X1, Y1, X2, Y2, X3, Y3, x, y);
		
		if (point == 0) {	
			Navigation_Test.travelTo(X0, Y0, odometer);
		} else if (point == 1) {
			Navigation_Test.travelTo(X1, Y1, odometer);
		} else if (point == 2) {
			Navigation_Test.travelTo(X2, Y2, odometer);
		} else if (point == 3) {
			Navigation_Test.travelTo(X3, Y3, odometer);
		}
		
		//beep three times after arriving at the point 
		Sound.beep();
		Sound.beep();
		Sound.beep();
		
		openHook();
		probe(odometer, point);
	
		
		findRoute(point, odometer);
		
		
	}

	
	public static void probe(Odometer_Test odometer, int point) {
		
		double[] odometerData = odometer.getXYT();
		double x = odometerData[0];
		double y = odometerData[1];
		double t;
		
		int color;

		odometerData = odometer.getXYT();
		x = odometerData[0];
		y = odometerData[1];
		t = odometerData[2];

		double treeOrientation = 0;
		if (point == 1) treeOrientation = 270;
		if (point == 2) treeOrientation = 180;
		if (point == 3) treeOrientation = 90;
		double angle = Navigation_Test.smallAngle(t, treeOrientation);

		//reset motor before rotating
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
		}
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, angle), true);
		rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, angle), false);
		
		leftMotor.rotate(-Navigation_Test.convertDistance(WHEEL_RAD, 5), true);
		rightMotor.rotate(-Navigation_Test.convertDistance(WHEEL_RAD, 5), false);
		
		Navigation_Test.lineCorrection(odometer);
		
		color = Grabber_Test.highLevel();
		Navigation_Test.lineCorrection(odometer);
		if (color == 0) {
			color = Grabber_Test.lowLevel();
			Navigation_Test.lineCorrection(odometer);
		}

	}

	
	/**
	 * This method is used for turning the arm to fetch the rings on the upper level
	 * of the tree
	 */
	public static int lowLevel() {
		armMotor.setAcceleration(15000);
		armMotor.setSpeed(ARM_SPEED);
		armMotor.rotate(LOW_ANGLE);
		closeHook(); //////////////////////////////////////////////////////////////////////////////////
		leftMotor.setSpeed(PROBE_SPEED);
		rightMotor.setSpeed(PROBE_SPEED);
		leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, LOW_PROBE), true);
		rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, LOW_PROBE), false);
		leftMotor.stop(true);
		rightMotor.stop(false);
		///////////////////////////////////
		int color = Color_Test.color();

		if (color == 1 || color == 2 || color == 3 || color == 4) { 	// high level fetching
			if (color == 1) {
				Sound.beep();
			
			} else if (color == 2) {
				Sound.beep();
				Sound.beep();
			
			} else if (color == 3) {
				Sound.beep();
				Sound.beep();
				Sound.beep();
				
			} else {
				Sound.beep();
				Sound.beep();
				Sound.beep();
				Sound.beep();
		
			}
			
		}
		openHook(); /////////////////////////////////////////////////////////////////
		//reset motor before rotating
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
		}
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, -LOW_PROBE - 5), true);
		rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, -LOW_PROBE - 5), false);

		resetArm();
		
		return color;
		
	}

	/**
	 * This method is used for turning the arm to fetch the rings on the lower level
	 * of the tree
	 */
	public static int highLevel() {
		armMotor.setAcceleration(3000);
		armMotor.setSpeed(ARM_SPEED);
		armMotor.rotate(HIGH_ANGLE);
		closeHook();/////////////////////////////////////////////////////////////
		leftMotor.setSpeed(PROBE_SPEED);
		rightMotor.setSpeed(PROBE_SPEED);
		leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, HIGH_PROBE), true);
		rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, HIGH_PROBE), false);
		leftMotor.stop(true);
		rightMotor.stop(false);
		///////////////////////////////////
		int color = Color_Test.color();

		if (color == 1 || color == 2 || color == 3 || color == 4) { 	// high level fetching
			if (color == 1) {
				Sound.beep();
				openHook();
			} else if (color == 2) {
				Sound.beep();
				Sound.beep();
				
			} else if (color == 3) {
				Sound.beep();
				Sound.beep();
				Sound.beep();

			} else {
				Sound.beep();
				Sound.beep();
				Sound.beep();
				Sound.beep();
				
			}

		}
		
		openHook(); /////////////////////////////
		
		//reset motor before rotating backward
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
		}
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, -HIGH_PROBE -5), true);
		rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, -HIGH_PROBE -5), false);
		/////////////////////////////////////////
		resetArm();
		return color;
		
	}

	/**
	 * This method is used for opening the a the hook to fetch the ring
	 */
	public static void openHook() {
		hookMotor.rotate(HOOK_ANGLE);
	}

	/**
	 * This method is used for closing the hook to either probe though hole or drop
	 * the rings
	 */
	public static void closeHook() {
		hookMotor.rotate(-HOOK_ANGLE);
	}

	/**
	 * this method is used to unload the ring using the arm motor
	 */
	public static void unload() {
		closeHook();
		armMotor.setSpeed(ARM_SPEED);
		//reset motor before rotating
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
		}
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, -3), true);
		rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, -3), false);
		leftMotor.stop(true);
		rightMotor.stop(false);
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, 5), true);
		rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, 5), false);
		leftMotor.stop(true);
		rightMotor.stop(false);
	}

	/**
	 * this method is used to reset the arm to the initial position (falling on the
	 * back support)
	 */
	public static void resetArm() {
		armMotor.setAcceleration(3000);
		armMotor.setSpeed(ARM_SPEED);
		while (armMotor.getTachoCount() != 0) {
			armMotor.backward();
		}
		armMotor.resetTachoCount();
	}

	public static void findRoute(int point, Odometer_Test odometer) {

		double T_x = Project_Test.T_x; //x coordinate of the ring tree
		double T_y = Project_Test.T_y; //y coordinate of the ring tree
		double Island_LL_y = Project_Test.Island_LL_y;
		double Island_LL_x = Project_Test.Island_LL_x;
		double Island_UR_y = Project_Test.Island_UR_y;
		double Island_UR_x = Project_Test.Island_UR_x;
		
		int nextPoint1 = (point + 1)%4; 
		int nextPoint2 = (point + 2)%4;
		int nextPoint3 = (point + 3)%4;
		
		boolean[] availability = {(T_y != 1 && T_y != Island_LL_y), (T_x != 7 && T_x !=Island_UR_x), (T_y != 7 && T_y != Island_UR_y) , (T_x != 1 && T_x != Island_LL_x)};
		
		
		if (!availability[nextPoint1] && !availability[nextPoint2] && !availability[nextPoint3]) {
			//do nothing
		} else if (availability[nextPoint1] && availability[nextPoint2] && availability[nextPoint3]) {			
			treeTravel(point, nextPoint1, odometer);
			probe(odometer, nextPoint1);
			 
		
				treeTravel(nextPoint1, nextPoint2, odometer);
				probe(odometer, nextPoint2);
				
			
				
					treeTravel(nextPoint2, nextPoint3, odometer);
					probe(odometer, nextPoint3);
					
					treeTravel(nextPoint3, point ,odometer);
		
			
		} else if (availability[nextPoint1] && !availability[nextPoint2] && availability[nextPoint3]) {
			
			treeTravel(point, nextPoint1, odometer);
			probe(odometer, nextPoint1);
			
	
			
				treeTravel(nextPoint1, nextPoint3, odometer);
				probe(odometer, nextPoint3);
				
				treeTravel(nextPoint3, point ,odometer);
			
		} else if (availability[nextPoint1] && availability[nextPoint2] && !availability[nextPoint3]) {
			
			treeTravel(point, nextPoint1, odometer);
			probe(odometer, nextPoint1);
			
			
			
				treeTravel(nextPoint1, nextPoint2, odometer);
				probe(odometer, nextPoint2);
			
				treeTravel(nextPoint2, point ,odometer);
			
		} else if (availability[nextPoint1] && !availability[nextPoint2] && !availability[nextPoint3]) {
			
			treeTravel(point, nextPoint1, odometer);
			probe(odometer, nextPoint1);
			
			treeTravel(nextPoint1, point ,odometer);
		} else if (!availability[nextPoint1] && availability[nextPoint2] && availability[nextPoint3]) {
			
			treeTravel(point, nextPoint3, odometer);
			probe(odometer, nextPoint3);
			
			
			
				treeTravel(nextPoint3, nextPoint2, odometer);
				probe(odometer, nextPoint2);
				treeTravel(nextPoint2, point ,odometer);
				
		} else if (!availability[nextPoint1] && !availability[nextPoint2] && availability[nextPoint3]) {
			
			treeTravel(point, nextPoint3, odometer);
			probe(odometer, nextPoint3);
			
			treeTravel(nextPoint3, point ,odometer);
			if(point == 0) {
				odometer.setX((T_x) *TILE_SIZE);
				odometer.setY((T_y - 1) *TILE_SIZE);
			} else if(point == 1) {
				odometer.setX((T_x +1) *TILE_SIZE);
				odometer.setY((T_y) *TILE_SIZE);
			} else if(point == 2) {
				odometer.setX((T_x) *TILE_SIZE);
				odometer.setY((T_y + 1) *TILE_SIZE);
			} else if(point == 3) {
				odometer.setX((T_x -1) *TILE_SIZE);
				odometer.setY((T_y) *TILE_SIZE);
			}
		}
		
		
		
	}
	
	/**
	 * This method defines the routine for traveling between the sides, after the avalability of the side is determined 
	 * @param startPoint the side traveling from
	 * @param endPoint the side traveling to 
	 * @param odometer the odometer used by the robot
	 */
	public static void treeTravel(int startPoint, int endPoint, Odometer_Test odometer) {
		
		//reset motor before rotating
		leftMotor.stop(true);
		rightMotor.stop(false);
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(3000);
		}
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
		}
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		int direction;
		
		if (endPoint == (startPoint + 1)%4 || endPoint == (startPoint - 1)%4) {
			
			if (endPoint == (startPoint + 1)%4) {
				
				direction = -1;
				
			} else {
				
				direction = 1;
				
			}
			
			leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, -direction*90), true);
			rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, -direction*90), false);
			leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, TILE_SIZE - 8), true);
			rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, TILE_SIZE -8), false);
			Navigation_Test.lineCorrection(odometer);
			
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			
			leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, direction*90), true);
			rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, direction*90), false);
			leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, TILE_SIZE - 8), true);
			rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, TILE_SIZE -8), false);
			Navigation_Test.lineCorrection(odometer);
			
			
		} else if (endPoint == (startPoint + 2)%4 || endPoint == (startPoint - 2)%4) {
			
			if (endPoint == (startPoint + 2)%4) {
				
				direction = -1;
				
			} else {
				
				direction = 1;
				
			}
			
			leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, direction*90), true);
			rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, direction*90), false);
			leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, TILE_SIZE - 8), true);
			rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, TILE_SIZE -8), false);
			Navigation_Test.lineCorrection(odometer);
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, -direction*90), true);
			rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, -direction*90), false);
			leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, TILE_SIZE - 8), true);
			rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, TILE_SIZE -8), false);
			Navigation_Test.lineCorrection(odometer);
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, TILE_SIZE - 8), true);
			rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, TILE_SIZE -8), false);
			Navigation_Test.lineCorrection(odometer);
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(Navigation_Test.convertAngle(WHEEL_RAD, TRACK, -direction*90), true);
			rightMotor.rotate(-Navigation_Test.convertAngle(WHEEL_RAD, TRACK, -direction*90), false);
			leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, TILE_SIZE - 8), true);
			rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, TILE_SIZE -8), false);
			Navigation_Test.lineCorrection(odometer);
			
		}
		
		
	}
	
}