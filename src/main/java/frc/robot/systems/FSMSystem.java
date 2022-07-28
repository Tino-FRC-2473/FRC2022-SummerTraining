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
		ROTATION_STATE
	}

	private static final float FRONT_RIGHT_MOTOR_RUN_POWER = 0.1f;
	private static final float FRONT_LEFT_MOTOR_RUN_POWER = 0.1f;
	private static final float BACK_RIGHT_MOTOR_RUN_POWER = 0.1f;
	private static final float BACK_LEFT_MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax frontRight;
	private CANSparkMax frontLeft;
	private CANSparkMax backLeft;
	private CANSparkMax backRight;

	private ANGLE_ERROR_OFFSET = 180;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to teleop state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public FSMSystem() {
		// Perform hardware init
		frontRight = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_RIGHT,
									CANSparkMax.MotorType.kBrushless);
		frontLeft = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_LEFT,
									CANSparkMax.MotorType.kBrushless);
		backLeft = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_BACK_LEFT,
										CANSparkMax.MotorType.kBrushless);
		backRight = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_BACK_RIGHT,
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
		frontRight.set(0);
		frontLeft.set(0);
		backLeft.set(0);
		backRight.set(0);

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
			case ROTATION_STATE:
				handleRotationState(input);
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
			case ROTATION_STATE:
				return FSMState.ROTATION_STATE;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */

	/**
	 * Handle behavior in OTHER_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTeleopState(TeleopInput input) {
		//get the magnitude of the power with which the joystick is pushed
		double powerMag = Math.toDegrees(Math.sqrt(input.getRightJoystickY() * input.getRightJoystickY() +
		input.getRightJoystickX() * input.getRightJoystickX()));

		//get the angle with which the joystick was pushed
		double joystickAngle = Math.toDegrees(Math.atan(input.getAngle())) + ANGLE_ERROR_OFFSET;

		//move the joystick

	}

	/**
	 * Handle behavior in ROTATION_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleRotationState(TeleopInput input) {

	}
}
