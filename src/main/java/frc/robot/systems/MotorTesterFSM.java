package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class MotorTesterFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		START_STATE,
		FOR_STATE,
		BETWEEN_STATE,
		REV_STATE
	}

	private static final float MOTOR_RUN_POWER = 0.1f;
	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax motor1;
	private CANSparkMax motor2;
	private CANSparkMax motor3;
	private CANSparkMax motor4;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public MotorTesterFSM() {
		// Perform hardware init
		motor1 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_MOTOR1,
				CANSparkMax.MotorType.kBrushless);
		motor2 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_MOTOR2,
				CANSparkMax.MotorType.kBrushless);
		motor3 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_MOTOR3,
				CANSparkMax.MotorType.kBrushless);
		motor4 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_MOTOR4,
				CANSparkMax.MotorType.kBrushless);
		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 *
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
		currentState = FSMState.START_STATE;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 *
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		switch (currentState) {
			case START_STATE:
				handleStartState(input);
				break;

			case FOR_STATE:
				handleOtherState(input);
				break;

			case BETWEEN_STATE:
				handleBetweenState(input);
				break;

			case REV_STATE:
				handleReverseState(input);
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
	 *
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
		if (input == null) {
			return FSMState.START_STATE;
		}
		switch (currentState) {
			case START_STATE:
				if (input.isClimberButtonPressed()) {
					return FSMState.FOR_STATE;
				} else {
					return FSMState.START_STATE;
				}

			case FOR_STATE:
				if (input.isClimberButtonPressed()) {
					return FSMState.FOR_STATE;
				} else {
					return FSMState.BETWEEN_STATE;
				}
			case BETWEEN_STATE:
				if (input.isClimberButtonPressed()) {
					return FSMState.REV_STATE;
				} else {
					return FSMState.BETWEEN_STATE;
				}
			case REV_STATE:
				if (input.isClimberButtonPressed()) {
					return FSMState.REV_STATE;
				} else {
					return FSMState.START_STATE;
				}
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 *
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	private void handleStartState(TeleopInput input) {
		motor1.set(0);
		motor2.set(0);
		motor3.set(0);
		motor4.set(0);
	}

	/**
	 * Handle behavior in OTHER_STATE.
	 *
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	private void handleOtherState(TeleopInput input) {
		motor1.set(MOTOR_RUN_POWER);
		motor2.set(MOTOR_RUN_POWER);
		motor3.set(MOTOR_RUN_POWER);
		motor4.set(MOTOR_RUN_POWER);
	}

	private void handleBetweenState(TeleopInput input) {
		motor1.set(0);
		motor2.set(0);
		motor3.set(0);
		motor4.set(0);
	}

	private void handleReverseState(TeleopInput input) {
		motor1.set(-1 * MOTOR_RUN_POWER);
		motor2.set(-1 * MOTOR_RUN_POWER);
		motor3.set(-1 * MOTOR_RUN_POWER);
		motor4.set(-1 * MOTOR_RUN_POWER);
	}
}
