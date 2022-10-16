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
		P1,
		// P2,
		// P3,
		// P4,
		TURN_TO_HUB;
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
	private boolean complete = false;
	private int stateCounter = 1;

	//private static final double START_ANGLE = -19.938;
	private static final double TURN_POWER = 0.05;
	private static final double MOVE_POWER = 0.1;
	private static final double DEGREES_360 = 360;
	private static final double DEGREES_180 = 180;
	private static final double DEGREES_270 = 270;
	private static final double DEGREES_90 = 90;
	private static final double TURN_THRESHOLD = 3;
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

		currentState = FSMState.P1;

		roboXPos = 0;
		roboYPos = 0;
		//System.out.println(roboXPos + " " + roboYPos);

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

			case P1:
				goToPos(input, 117.047, 0);
				break;

			// case P2:
			// 	goToPos(input, 117.047, -31.623);
			// 	break;

			// case P3:
			// 	goToPos(input, 56.814, 117.516);
			// 	break;

			// case P4:
			// 	goToPos(input, 0, 0);
			// 	break;	

			case TURN_TO_HUB:
				turnToHub(input);
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

			case P1:
				if (stateCounter == 1) {
					return FSMState.P1;
				} else if (stateCounter == 2) {
					return FSMState.TURN_TO_HUB;
				// } else if (stateCounter == 3) {
				// 	return FSMState.P2;
				// } else if (stateCounter == 4) {
				// 	return FSMState.TURN_TO_HUB;
				// } else if (stateCounter == 5) {
				// 	return FSMState.P3;
				// } else if (stateCounter == 6) {
				// 	return FSMState.TURN_TO_HUB;
				// } else if (stateCounter == 7) {
				// 	return FSMState.P4;
				}

			// case P2:
			// 	if (stateCounter == 1) {
			// 		return FSMState.P1;
			// 	} else if (stateCounter == 2) {
			// 		return FSMState.TURN_TO_HUB;
			// 	} else if (stateCounter == 3) {
			// 		return FSMState.P2;
			// 	} else if (stateCounter == 4) {
			// 		return FSMState.TURN_TO_HUB;
			// 	} else if (stateCounter == 5) {
			// 		return FSMState.P3;
			// 	} else if (stateCounter == 6) {
			// 		return FSMState.TURN_TO_HUB;
			// 	} else if (stateCounter == 7) {
			// 		return FSMState.P4;
			// 	}

			// case P3:
			// 	if (stateCounter == 1) {
			// 		return FSMState.P1;
			// 	} else if (stateCounter == 2) {
			// 		return FSMState.TURN_TO_HUB;
			// 	} else if (stateCounter == 3) {
			// 		return FSMState.P2;
			// 	} else if (stateCounter == 4) {
			// 		return FSMState.TURN_TO_HUB;
			// 	} else if (stateCounter == 5) {
			// 		return FSMState.P3;
			// 	} else if (stateCounter == 6) {
			// 		return FSMState.TURN_TO_HUB;
			// 	} else if (stateCounter == 7) {
			// 		return FSMState.P4;
			// 	}

			// case P4:
			// 	if (stateCounter == 1) {
			// 		return FSMState.P1;
			// 	} else if (stateCounter == 2) {
			// 		return FSMState.TURN_TO_HUB;
			// 	} else if (stateCounter == 3) {
			// 		return FSMState.P2;
			// 	} else if (stateCounter == 4) {
			// 		return FSMState.TURN_TO_HUB;
			// 	} else if (stateCounter == 5) {
			// 		return FSMState.P3;
			// 	} else if (stateCounter == 6) {
			// 		return FSMState.TURN_TO_HUB;
			// 	} else if (stateCounter == 7) {
			// 		return FSMState.P4;
			// 	}

			case TURN_TO_HUB:
				if (stateCounter == 1) {
					return FSMState.P1;
				} else if (stateCounter == 2) {
					return FSMState.TURN_TO_HUB;
				// } else if (stateCounter == 3) {
				// 	return FSMState.P2;
				// } else if (stateCounter == 4) {
				// 	return FSMState.TURN_TO_HUB;
				// } else if (stateCounter == 5) {
				// 	return FSMState.P3;
				// } else if (stateCounter == 6) {
				// 	return FSMState.TURN_TO_HUB;
				// } else if (stateCounter == 7) {
				// 	return FSMState.P4;
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
	 * @param x go to x position
	 * @param y go to y position
	 */
	public void goToPos(TeleopInput input, double x, double y) {

		if (input != null) {
			return;
		}

		System.out.println("t " + turning + " m " + moving);

		double roboX = -roboXPos;
		double roboY = roboYPos;
		double deltaX = (x - roboX);
		double deltaY = (y - roboY);
		if (deltaX > -1 && deltaX < 1) deltaX = 0;
		if (deltaY > -1 && deltaY < 1) deltaY = 0;
		System.out.println("dx " + deltaX + " dy " + deltaY);

		// assume unit circle angles (east = 0, positive counterclockwise)
		double currentAngle = (-gyro.getAngle()) % DEGREES_360;
		System.out.println("current angle " + currentAngle);

		// calculates turn angle
		double angle;
		if (deltaX == 0 && deltaY >= 0) {
			angle = DEGREES_90;
		} else if (deltaX == 0 && deltaY < 0) {
			angle = DEGREES_270;
		} else {
			angle = Math.toDegrees(Math.atan(deltaY / deltaX));
		}

		if (deltaX < 0) {
			angle += DEGREES_180;
		} if (deltaX > 0 && deltaY < 0) {
			angle += DEGREES_360;
		}

		System.out.println("turn angle " + angle);

		// calculate turn amount
		double turnAmount = angle - currentAngle;

		if (Math.abs(turnAmount - DEGREES_360) < Math.abs(turnAmount)) {
			turnAmount -= DEGREES_360;
		}

		System.out.println("turn amount: " + turnAmount);

		// calculates distance
		double dist = Math.sqrt(deltaY * deltaY + deltaX * deltaX);
		// System.out.println("dist: " + dist);
		System.out.println("curX: " + roboX + " curY: " + roboYPos);
		System.out.println("x " + x + "y " + y);

		// fix threshold errors
		dist += MOVE_THRESHOLD / 2;
		turnAmount += TURN_THRESHOLD / 2;

		// set motor power
		if ((turnAmount < -TURN_THRESHOLD || turnAmount > TURN_THRESHOLD) && !complete && turning) {
			System.out.println("turning");
			if (turnAmount > 0) {
				leftMotor.set(TURN_POWER);
				rightMotor.set(TURN_POWER);
			} else if (turnAmount < 0) {
				leftMotor.set(-TURN_POWER);
				rightMotor.set(-TURN_POWER);
			}
		} else  if (dist > MOVE_THRESHOLD && !complete && moving) { 
			System.out.println("moving");
			leftMotor.set(-MOVE_POWER);
			rightMotor.set(MOVE_POWER);
		} else if (!complete) {
			leftMotor.set(0);
			rightMotor.set(0);
			System.out.println("STOP");
			complete = true;
			turning = true;
			moving = true;
			stateCounter++;
		}

		// complete or not
		if (((Math.abs(roboY) > Math.abs(y) + Math.abs(deltaX) / 2 || Math.abs(roboX) > Math.abs(x) + Math.abs(deltaY) / 2)) && !complete) {
			moving = false;
			//System.out.println("HERE");
		} else if (!(turnAmount >= -TURN_THRESHOLD && turnAmount <= TURN_THRESHOLD)) {
			complete = false;
		} else if (dist > MOVE_THRESHOLD) {
			complete = false;
			turning = false;
		} else {
			complete = true;
			turning = true;
			moving = true;
		}
	}

	public void turnToHub(TeleopInput input) {

		double roboX = -roboXPos;
		double roboY = roboYPos;
		double deltaX = (-roboX);
		double deltaY = (-roboY);
		if (deltaX > -1 && deltaX < 1) deltaX = 0;
		if (deltaY > -1 && deltaY < 1) deltaY = 0;
		//System.out.println("dx " + deltaX + " dy " + deltaY);

		// assume unit circle angles (east = 0, positive counterclockwise)
		double currentAngle = (-gyro.getAngle()) % DEGREES_360;
		System.out.println("current angle " + currentAngle);

		// calculates turn angle
		double angle;
		if (deltaX == 0 && deltaY >= 0) {
			angle = DEGREES_90;
		} else if (deltaX == 0 && deltaY < 0) {
			angle = DEGREES_270;
		} else {
			angle = Math.toDegrees(Math.atan(deltaY / deltaX));
		}

		if (deltaX < 0) {
			angle += DEGREES_180;
		} if (deltaX > 0 && deltaY < 0) {
			angle += DEGREES_360;
		}

		if (angle < 0) {
			angle -= 5;
		} else {
			angle += 5;
		}

		System.out.println("turn angle hub " + angle);

		// calculate turn amount
		double turnAmount = angle - currentAngle;

		System.out.println("hub turn amount: " + turnAmount);

		if (turnAmount < -TURN_THRESHOLD || turnAmount > TURN_THRESHOLD) {
			System.out.println("turning to hub");
			if (turnAmount > 0) {
				leftMotor.set(TURN_POWER);
				rightMotor.set(TURN_POWER);
			} else if (turnAmount < 0) {
				leftMotor.set(-TURN_POWER);
				rightMotor.set(-TURN_POWER);
			}
		} else {
			System.out.println("STOP");
			leftMotor.set(0);
			rightMotor.set(0);
			stateCounter++;
		}
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
