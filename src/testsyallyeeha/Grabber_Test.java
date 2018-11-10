package testsyallyeeha;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

/**
 * This class contains methods for the ring fetching routine and mechanism 
 * @author Team12
 *
 */
public class Grabber_Test {
	private static final EV3LargeRegulatedMotor leftMotor  = Project_Test.leftMotor; //the motor for the left wheel
	private static final EV3LargeRegulatedMotor rightMotor = Project_Test.rightMotor; //the motor for the right wheel
	private static final EV3LargeRegulatedMotor armMotor = Project_Test.armMotor; //the motor for raising/lowering the arm 
	private static final EV3MediumRegulatedMotor hookMotor = Project_Test.hookMotor; //the motor for motorizing the hooks 
	private static final int ARM_SPEED = Project_Test.ARM_SPEED; //this is the speed for the arm for the arm motor 
	private static final int HOOK_SPEED = Project_Test.HOOK_SPEED; //this is the angle which the hook will open/close	
	private static final int HOOK_ANGLE = Project_Test.HOOK_ANGLE; //this is the angle which the hook will open/close	
	private static final int LOW_ANGLE = Project_Test.LOW_ANGLE; //the angle the arm motor needs to turn to reach lowly-hanged rings, with respect to the initial position 
	private static final int HIGH_ANGLE = Project_Test.HIGH_ANGLE; //the angle the arm motor needs to turn to reach highly-hanged rings, with respect to the initial position 
	private static final int UNLOAD_ANGLE = Project_Test.UNLOAD_ANGLE; //the angle the arm motor needs to turn to unload the ring(s), with respect to the initial position 
	
	private static final int FORWARD_SPEED = Project_Test.HIGH_SPEED; 
	private static final int ROTATE_SPEED = Project_Test.MEDIUM_SPEED;
	private static final double WHEEL_RAD = Project_Test.WHEEL_RAD;
	private static final double TRACK = Project_Test.TRACK;
	private static final double TILE_SIZE = Project_Test.TILE_SIZE;
	private static final double OFF_SET = Project_Test.OFF_SET;
	
	public static void probe(Odometer_Test odometer) {

		double T_x = 0; // x coordinate of the ring tree
		double T_y = 0; // y coordinate of the ring tree

		double[] odometerData = odometer.getXYT();
		double x = odometerData[0];
		double y = odometerData[1];
		double t;

		int point;

		double X0 = T_x * TILE_SIZE;
		double Y0 = (T_y - 1) * TILE_SIZE;

		double X1 = T_x * TILE_SIZE;
		double Y1 = (T_y + 1) * TILE_SIZE;

		double X2 = (T_x + 1) * TILE_SIZE;
		double Y2 = T_y * TILE_SIZE;

		double X3 = (T_x - 1) * TILE_SIZE;
		double Y3 = T_y * TILE_SIZE;

		int color;

		point = Navigation_Test.closestPoint(X0, Y0, X1, Y1, X2, Y2, X3, Y3, x, y);

		if (point == 0) {

			Navigation_Test.travelTo(X0, Y0, odometer);

		} else if (point == 1) {

			Navigation_Test.travelTo(X1, Y1, odometer);

		} else if (point == 2) {

			Navigation_Test.travelTo(X2, Y2, odometer);

		} else {

			Navigation_Test.travelTo(X3, Y3, odometer);

		}

		odometerData = odometer.getXYT();
		x = odometerData[0];
		y = odometerData[1];
		t = odometerData[2];

		double dAngle = Navigation_Test.getDAngle(T_x, T_y, x, y);
		Navigation_Test.turnTo(dAngle, t);

		Grabber_Test.highLevel();

		leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, 20), true);
		rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, 20), false);

		color = Color_Test.color();

		if (color == 1 || color == 2 || color == 3 || color == 4) {

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

			Grabber_Test.openHook();

		} else {

			leftMotor.rotate(-Navigation_Test.convertDistance(WHEEL_RAD, 20), true);
			rightMotor.rotate(-Navigation_Test.convertDistance(WHEEL_RAD, 20), false);

			Grabber_Test.closeHook();
			Grabber_Test.lowLevel();

			leftMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, 20), true);
			rightMotor.rotate(Navigation_Test.convertDistance(WHEEL_RAD, 20), false);

			color = Color_Test.color();

			if (color == 1 || color == 2 || color == 3 || color == 4) {

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

				Grabber_Test.openHook();

			}

		}

		leftMotor.rotate(-Navigation_Test.convertDistance(WHEEL_RAD, 25), true);
		rightMotor.rotate(-Navigation_Test.convertDistance(WHEEL_RAD, 25), false);

	}
	/**
	 * This method is used for turning the arm to fetch the rings on the upper level of the tree
	 */
	public static void lowLevel() {
		armMotor.setSpeed(ARM_SPEED);
		armMotor.rotate(LOW_ANGLE);
	}
	
	/**
	 * This method is used for turning the arm to fetch the rings on the lower level of the tree
	 */
	public static void highLevel() {
		armMotor.setSpeed(ARM_SPEED);
		armMotor.rotate(HIGH_ANGLE);
	}

	/**
	 * This method is used for opening the a the hook to fetch the ring
	 */
	public static void openHook() {
		hookMotor.rotate(HOOK_ANGLE);
	}
	/**
	 * This method is used for closing the hook to either probe though hole or drop the rings
	 */
	public static void closeHook() {
		hookMotor.rotate(-HOOK_ANGLE);
	}
	
	/**
	 * this method is used to unload the ring using the arm motor
	 */
	public static void unload(){
		armMotor.setSpeed(ARM_SPEED);
		armMotor.rotate(UNLOAD_ANGLE);
	}
	
	/**
	 * this method is used to reset the arm to the initial position (falling on the back support)
	 */
	public static void resetArm(EV3LargeRegulatedMotor armMotor) {
		armMotor.setSpeed(ARM_SPEED);
		//shit how to do this....
	}
	

	
}
