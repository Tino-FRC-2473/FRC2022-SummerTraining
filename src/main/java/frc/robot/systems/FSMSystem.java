package frc.robot.systems;

// Third party Hardware Imports
import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.CANSparkMax;
import com.kauailabs.navx.frc.AHRS;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.Constants;

public class FSMSystem {


	// FSM state definitions
	public enum FSMState {
		STATE1,
		STATE2,
		// STATE3,
		// STATE4;
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax leftMotor;
	private CANSparkMax rightMotor;

	private double roboXPos = 0;
	private double roboYPos = 0;
	private double currentEncoderPos;
	private double prevEncoderPos = 0;
	private double gyroAngleForOdo = 0;
	private AHRS gyro;

	private boolean turning = true;
	private boolean moving = true;
	private double velocity2 = 0;
	private int stateCounter = 1;

	private int pointNum = 0;
	private int n = 0;
	private double[][] waypoints = new double [2][n];

	private static final double ROBOT_WIDTH = 20;
	private static final double MOVE_POWER = 0.1;
	private static final double MOVE_THRESHOLD = 2;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public FSMSystem() {
		// Perform hardware init
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT,
										CANSparkMax.MotorType.kBrushless);
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT,
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

		currentState = FSMState.STATE1;

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

			case STATE1:
				handlePurePursuit(input, 60.0, 0.0, 20.0, 40.0, 0.0, 60.0, 20.0, true);
				break;

			// case STATE2:
			// 	handlePurePursuit(input, 60, 20);
			// 	break;

			// case STATE3:
			// 	handlePurePersuit(input, 0, DIST);
			// 	break;

			// case STATE4:
			// 	handlePurePersuit(input, 0, 0);
			// 	break;

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

			case STATE1:
				if (stateCounter == 1) {
					return FSMState.STATE1;
				} else if (stateCounter == 2) {
					return FSMState.STATE2;
				// } else if (stateCounter == 3) {
				// 	return FSMState.STATE3;
				// } else if (stateCounter == 4) {
				// 	return FSMState.STATE4;
				}

			case STATE2:
				if (stateCounter == 1) {
					return FSMState.STATE1;
				} else if (stateCounter == 2) {
					return FSMState.STATE2;
				// } else if (stateCounter == 3) {
				// 	return FSMState.STATE3;
				// } else if (stateCounter == 4) {
				// 	return FSMState.STATE4;
				}

			// case STATE3:
			// 	if (stateCounter == 1) {
			// 		return FSMState.STATE1;
			// 	} else if (stateCounter == 2) {
			// 		return FSMState.STATE2;
			// 	} else if (stateCounter == 3) {
			// 		return FSMState.STATE3;
			// 	} else if (stateCounter == 4) {
			// 		return FSMState.STATE4;
			// 	}

			// case STATE4:
			// 	if (stateCounter == 1) {
			// 		return FSMState.STATE1;
			// 	} else if (stateCounter == 2) {
			// 		return FSMState.STATE2;
			// 	} else if (stateCounter == 3) {
			// 		return FSMState.STATE3;
			// 	} else if (stateCounter == 4) {
			// 		return FSMState.STATE4;
			// 	}

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */

	/**
	 * Handle behavior in PURE_PERSUIT.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @param x go to x position
	 * @param y go to y position
	 * @param v2 velocity of the wheel that is making the larger turn
	 * @param x1 x position on line 1 curveDist away from the interestion
	 * @param y1 y position on line 1 curveDist away from the interestion
	 * @param x2 x position on line 2 curveDist away from the interestion
	 * @param y2 y position on line 3 curveDist away from the interestion
	 * @param curveDist distance at which robot should start turning
	 * @param left whether the robot should turn left or right
	 */
	public void handlePurePursuit(TeleopInput input, double lookAheadDist) {

		if (input != null) {
			return;			
		}

		double roboX = -roboXPos;
		double roboY = roboYPos;
		double goalX = waypoints[0][pointNum];
		double goalY = waypoints[2][pointNum];
		
	}
	// public void handlePurePursuit(TeleopInput input, double x, double y, double curveDist, double x1, double y1, double x2, double y2, boolean left) {

	// 	if (input != null) {
	// 		return;
	// 	}
		

	// 	System.out.println("t " + turning + " m " + moving);

	// 	double roboX = -roboXPos;
	// 	double roboY = roboYPos;
	// 	double deltaX = (x - roboX);
	// 	double deltaY = (y - roboY);
	// 	System.out.println("dx " + deltaX + " dy " + deltaY);


	// 	// calculates distance
	// 	double dist;
	// 	if (moving) {
	// 		dist = Math.sqrt(deltaY * deltaY + deltaX * deltaX);
	// 		System.out.println("dist: " + dist);
	// 		System.out.println("curX: " + roboX + " curY: " + roboYPos);
	// 		System.out.println("x " + x + "y " + y);

	// 		// fix threshold errors
	// 		dist += MOVE_THRESHOLD - curveDist;
	// 	} else {
	// 		dist = 0;
	// 	}

	// 	// calculates turn velocity
	// 	double velocity1 = 0;
	// 	if (turning) {
	// 		double c = Math.sqrt((y2 - y1)*(y2 - y1) + (x2 - x1)*(x2 - x1));
	// 		double cosTheta = (c*c -2*curveDist*curveDist) / (-2*curveDist*curveDist);
	// 		double theta = Math.abs(Math.acos(cosTheta));
	// 		velocity1 = velocity2 * ((curveDist * Math.tan(theta / 2) - ROBOT_WIDTH / 2) / (curveDist * Math.tan(theta / 2) + ROBOT_WIDTH / 2));
	// 	}

	// 	// complete or not
	// 	if (dist > MOVE_THRESHOLD) {
	// 		moving = false;
	// 	 } else if (Math.abs(x2 - roboX) <= MOVE_THRESHOLD && Math.abs(y2 - roboY) <= MOVE_THRESHOLD) {
	// 		turning = false;
	// 	} else {
	// 		turning = true;
	// 		moving = true;
	// 	}

	// 	// set motor power
	// 	if (turning) {
	// 		System.out.println("turning");
	// 		if (left) {
	// 			leftMotor.set(velocity1 / 7.95867322835);
	// 			rightMotor.set(velocity2 / 7.95867322835);
	// 		} else if (!left) {
	// 			leftMotor.set(velocity2 / 7.95867322835);
	// 			rightMotor.set(velocity1 / 7.95867322835);
	// 		}
	// 	} else  if (moving) { 
	// 		System.out.println("moving");
	// 		leftMotor.set(-MOVE_POWER);
	// 		rightMotor.set(MOVE_POWER);
	// 	} else if (!turning && !moving) {
	// 		leftMotor.set(0);
	// 		rightMotor.set(0);
	// 		System.out.println("STOP");
	// 		turning = true;
	// 		moving = true;
	// 		stateCounter++;
	// 	}

	// }

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
