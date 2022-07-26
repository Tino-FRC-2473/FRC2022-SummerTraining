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
		TELEOP_STATE
	}

	// private static final float MOTOR_RUN_POWER = 0.1f;
	private static final int NUM_VECTORS = 2;
	private static final int INVERSE_TRIG_RANGE_ERROR = 180;
	final private static  int MEC_WHEEL_ANGLE = 45;
	final private static  int GRID_ROTAION_FACTOR = 315;

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
				return FSMState.TELEOP_STATE;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in TELEOP_STATE.
	 * The left joystick controls the direction the robot moves in.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTeleopState(TeleopInput input) {

		// Left Joystick
		double leftX = input.getLeftJoystickX();
		double leftY = input.getLeftJoystickY();

		double joystickMagnitude = Math.sqrt(leftY * leftY + leftX * leftX);
		double joystickAngle = (leftX < 0) ? Math.toDegrees(Math.atan(leftY / leftX)) : Math.toDegrees(Math.atan(leftY / leftX)) + INVERSE_TRIG_RANGE_ERROR;
		double angle = (joystickAngle <= GRID_ROTAION_FACTOR) ? joystickAngle + MEC_WHEEL_ANGLE : joystickAngle - GRID_ROTAION_FACTOR;
		double leftPointer = joystickMagnitude * Math.sin(Math.toRadians(angle));
		double rightPointer = joystickMagnitude * Math.cos(Math.toRadians(angle));

		frontLeftMotor.set(rightPointer / NUM_VECTORS);
		backRightMotor.set(rightPointer / NUM_VECTORS);
		frontRightMotor.set(leftPointer / NUM_VECTORS);
		backLeftMotor.set(leftPointer / NUM_VECTORS);

		// Right Joystick
	}
}
