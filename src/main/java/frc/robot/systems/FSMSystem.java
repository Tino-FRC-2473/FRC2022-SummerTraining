package frc.robot.systems;

// Third party Hardware Imports
import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.CANSparkMax;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.Constants;

public class FSMSystem {


	// FSM state definitions
	public enum FSMState {
		PURE_PERSUIT;
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax leftMotor;
	private CANSparkMax rightMotor;
	private CANSparkMax leftMotor2;
	private CANSparkMax rightMotor2;

	private double roboXPos = 0;
	private double roboYPos = 0;
	private double currentEncoderPos;
	private double prevEncoderPos = 0;
	private double gyroAngleForOdo = 0;
	private AHRS gyro;

	private int stateCounter = 1;
	private int partitions = 8; // should be even
	private double[][] waypoints = new double[2][partitions + 1];
	private int target = 0;
	private double innerVelocity = 1; // in/s
	private double outerVelocity = 1; // in/s
	private int pointNum = 0;
	private double lookAheadDistance = 10;
	private boolean firstRun = true;

	private static final double ROBOT_WIDTH = 20;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public FSMSystem() {
		// Perform hardware init
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT1,
										CANSparkMax.MotorType.kBrushless);
		rightMotor2 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT2,
										CANSparkMax.MotorType.kBrushless);
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT1,
										CANSparkMax.MotorType.kBrushless);
		leftMotor2 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT2,
										CANSparkMax.MotorType.kBrushless);
		gyro = new AHRS(SPI.Port.kMXP);

		// Reset state machine
		reset();

	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public FSMState getCurrentState() {
		return currentState;
	}

	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {

		rightMotor.getEncoder().setPosition(0);
		leftMotor.getEncoder().setPosition(0);

		gyro.reset();
		gyro.zeroYaw();
		gyroAngleForOdo = 0;

		currentState = FSMState.PURE_PERSUIT;

		roboXPos = 0;
		roboYPos = 0;
		System.out.println(roboXPos + " " + roboYPos);

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {

		gyroAngleForOdo = gyro.getAngle();

		currentEncoderPos = ((leftMotor.getEncoder().getPosition()
				- rightMotor.getEncoder().getPosition()) / 2.0);

		updateLineOdometryTele(gyro.getAngle(), currentEncoderPos);

		switch (currentState) {

			case PURE_PERSUIT:
				handlePurePursuit(input, 0, 0, 48, 0, 70, 20, 0);
				break;


			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
	}

	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
		switch (currentState) {

			case PURE_PERSUIT:
				if (stateCounter == 1) {
					return FSMState.PURE_PERSUIT;
				}

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */

	/**
	 * Handle behavior in PURE_PERSUIT.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void handlePurePursuit(TeleopInput input, double x1, double y1, double mx, double my, double x2, double y2, int dir) {

		if (input != null) {
			return;			
		}

		double roboX = -roboXPos;
		double roboY = roboYPos;
		double currentAngle = (-gyro.getAngle()) % 360;
		System.out.println("x: " + roboX + " y: " + roboY);
		
		if (firstRun) {
			calculateWaypoints(x1, y1, mx, my, x2, y2);
			firstRun = false;
		}

		if (roboX < x2 + 4 && roboX > x2 - 4 && roboY < y2 + 4 && roboY > y2 - 4) {
			System.out.println("STOP");
			resetPurePursuitProperties(8, 10, 0);
		}

		if (target != findTargetPoint(roboX, roboY)) { // when there is a new target point

			if (target == -1) resetPurePursuitProperties(8, 10, 0);
			if (target == waypoints[0].length - 1) {
				resetPurePursuitProperties(8, 10, 0);
			}
			pointNum++; // robot has advanced to a new point
			target = findTargetPoint(roboX, roboY);
			innerVelocity = calculateInnerCurveVelocity(currentAngle, roboX, roboY, waypoints[0][target], waypoints[1][target], outerVelocity);
		}

		// set motor powers (note: velcoties must be between [-7.9, +7.9])
		if (dir == 0) { // turning left
			leftMotor.set(-innerVelocity / 7.95867322835);
			leftMotor2.set(-innerVelocity / 7.95867322835);
			rightMotor.set(outerVelocity / 7.95867322835);
			rightMotor2.set(outerVelocity / 7.95867322835);
		} else if (dir == 1) { // turning right
			leftMotor.set(-outerVelocity / 7.95867322835);
			leftMotor2.set(-outerVelocity / 7.95867322835);
			rightMotor.set(innerVelocity / 7.95867322835);
			rightMotor2.set(innerVelocity / 7.95867322835);
		}
	}

	public void calculateWaypoints(double x1, double y1, double mx, double my, double x2, double y2) {
		double dx1 = 2 * (mx - x1) / partitions;
		double dy1 = 2 * (my - y1) / partitions;
		for (int i = 0; i <= partitions / 2; i++) {
			waypoints[0][i] = x1 + i * dx1;
			waypoints[1][i] = y1 + i * dy1;
		}
		double dx2 = 2 * (x2 - mx) / partitions;
		double dy2 = 2 * (y2 - my) / partitions;
		for (int i = partitions / 2 + 1; i <= partitions; i++) {
			waypoints[0][i] = mx + (i - partitions / 2) * dx2;
			waypoints[1][i] = my + (i - partitions / 2) * dy2;
		}
		System.out.println(Arrays.deepToString(waypoints));
	}

	public int findTargetPoint(double x, double y) {
		double closestDist = 1000000;
		int target = -1; // index of target point
		for (int i = pointNum; i < waypoints[0].length; i++) {
			double dist = Math.hypot(waypoints[0][i] - x, waypoints[1][i] - y); // distance between current pos and potential target point
			if (Math.abs(lookAheadDistance - dist) <= closestDist) {
				closestDist = Math.abs(lookAheadDistance - dist);
				target = i;
			}
		}
		System.out.println(waypoints[0][target] + " " + waypoints[1][target]);
		return target;
	}

	public double calculateInnerCurveVelocity(double startAngle, double x1, double y1, double x2, double y2, double outerVelocity) {
		double theta = Math.atan2(y2 - y1, x2 - x1) - Math.toRadians(startAngle);
		System.out.println("angle b points " + Math.toDegrees(Math.atan2(y2 - y1, x2 - x1)));
		System.out.println("theta:" + theta);
		System.out.println("start angle:" + startAngle);
		if (theta == 0) return outerVelocity; // innerVelocity = outerVelocity (going straight)

		double radius = (Math.tan(theta) + (1 / Math.tan(theta))) * Math.hypot(y2 - y1, x2 - x1) / 2;
		System.out.println("radius: " + radius);
		double innerS = 2 * theta * (radius - ROBOT_WIDTH); // inner curve length
		double outerS = 2 * theta * (radius + ROBOT_WIDTH); // outer curve length
		double innerVelocity = outerVelocity * (innerS/outerS); // (ensures that inner velcoity must be <= outer velocity)
		System.out.println(innerVelocity);
		return innerVelocity;
	}

	public void resetPurePursuitProperties(int partitions, double lookAheadDistance, double outerVelocity) {
		this.partitions = partitions; // should be even
		waypoints = new double[2][partitions + 1];
		this.lookAheadDistance = lookAheadDistance;
		target = 0;
		pointNum = 0;
		innerVelocity = outerVelocity;; // in/s
		this.outerVelocity = outerVelocity; // in/s
		firstRun = true;
		stateCounter++;
	}

	/**
	 * Tracks the robo's position on the field.
	 * @param gyroAngle robot's angle
	 * @param currentEncoderPos robot's current position
	 */
	public void updateLineOdometryTele(double gyroAngle, double currentEncoderPos) {

		// double currentEncoderPos = ((-leftEncoderPos + rightEncoderPos) / 2.0);
		double dEncoder = (currentEncoderPos - prevEncoderPos)
			/ Constants.REVOLUTIONS_PER_INCH;
		double dX = dEncoder * Math.cos(Math.toRadians(gyroAngleForOdo)) * 0.8880486672;
		double dY = dEncoder * Math.sin(Math.toRadians(gyroAngleForOdo)) * 1.1742067733;

		roboXPos += dX;
		roboYPos += dY;

		prevEncoderPos = this.currentEncoderPos;
		// return new Translation2d(robotPos.getX() + dX, robotPos.getY() + dY);
	}
}
