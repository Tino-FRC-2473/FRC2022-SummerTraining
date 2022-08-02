package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class FSMSystem {

	/* ======================== Constants ======================== */

	// FSM state definitions
	public enum FSMState {
		TELEOP_STATE,
		TELEOP_ROTATION_STATE
	}

	// private static final float MOTOR_RUN_POWER = 0.1f;

	// MAGIC NUMBERS
	private static final int NUM2 = 2;
	private static final int NUM180 = 180;
	private static final int NUM45 = 45;
	private static final int NUM315 = 315;
	private static final int NUM135 = 135;
	private static final int NUM225 = 225;

	/* ======================== Private variables ======================== */

	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax frontLeftMotor;
	private CANSparkMax frontRightMotor;
	private CANSparkMax backLeftMotor;
	private CANSparkMax backRightMotor;

	/* ======================== Constructor ======================== */

	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public FSMSystem() {
		// Perform hardware init
		frontLeftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_LEFT,
										CANSparkMax.MotorType.kBrushless);
		frontRightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_RIGHT,
										CANSparkMax.MotorType.kBrushless);
		backLeftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_BACK_LEFT,
										CANSparkMax.MotorType.kBrushless);
		backRightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_BACK_RIGHT,
										CANSparkMax.MotorType.kBrushless);
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
		currentState = FSMState.TELEOP_STATE;

		frontLeftMotor.set(0);
		frontRightMotor.set(0);
		backRightMotor.set(0);
		backLeftMotor.set(0);
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
		switch (currentState) {

			case TELEOP_STATE:
				handleTeleopState(input);
				break;

			case TELEOP_ROTATION_STATE:
				handleTeleopRotationState(input);
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

			case TELEOP_STATE:
				if (input.getRightJoystickY() == 0) {
					return FSMState.TELEOP_STATE;
				} else {
					return FSMState.TELEOP_ROTATION_STATE;
				}

			case TELEOP_ROTATION_STATE:
				if (input.getRightJoystickY() == 0) {
					return FSMState.TELEOP_STATE;
				} else {
					return FSMState.TELEOP_ROTATION_STATE;
				}

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */

	/**
	 * Handle behavior in TELEOP_STATE.
	 * The left joystick controls the speed and direction the robot moves in.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTeleopState(TeleopInput input) {

		double rightX = input.getRightJoystickX();
		double rightY = input.getRightJoystickY();

		double joystickMagnitude = Math.sqrt(rightY * rightY + rightX * rightX);
		double joystickAngle = (rightX < 0) ? Math.toDegrees(Math.atan(rightY / rightX))
			: Math.toDegrees(Math.atan(rightY / rightX)) + NUM180;
		double angle = (joystickAngle <= NUM315) ? joystickAngle + NUM45
			: joystickAngle - NUM315;
		double leftPointer = joystickMagnitude * Math.cos(Math.toRadians(angle));
		double rightPointer = joystickMagnitude * Math.sin(Math.toRadians(angle));

		if (joystickAngle > -NUM45 && joystickAngle < NUM45) {
			leftPointer = -Math.abs(leftPointer);
			rightPointer = Math.abs(rightPointer);
		} else if (joystickAngle > NUM45 && joystickAngle < NUM135) {
			leftPointer = Math.abs(leftPointer);
			rightPointer = Math.abs(rightPointer);
		} else if (joystickAngle > NUM135 && joystickAngle < NUM225) {
			leftPointer = Math.abs(leftPointer);
			rightPointer = -Math.abs(rightPointer);
		} else if (joystickAngle > NUM225 || joystickAngle < -NUM45) {
			leftPointer = Math.abs(leftPointer);
			rightPointer = -Math.abs(rightPointer);
		}

		// ?? check whether should be divided or not
		frontLeftMotor.set(rightPointer / NUM2);
		backRightMotor.set(rightPointer / NUM2);
		frontRightMotor.set(leftPointer / NUM2);
		backLeftMotor.set(leftPointer / NUM2);
	}

	/**
	 * Handle behavior in TELEOP_ROTATION_STATE.
	 * The right joystick controls the rotation of the robot. Forward results in
	 *        clockwise movement; backward results in counterclockwise movement.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTeleopRotationState(TeleopInput input) {

		double amount = input.getRightJoystickY();

		frontLeftMotor.set(amount);
		frontRightMotor.set(-amount);
		backRightMotor.set(-amount);
		backLeftMotor.set(amount);
	}
}
