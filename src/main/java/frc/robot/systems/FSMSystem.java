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

	private static final float MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax rightMotor;
	private CANSparkMax leftMotor;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public FSMSystem() {
		// Perform hardware init
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT,
			CANSparkMax.MotorType.kBrushless);
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT,
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
		currentState = FSMState.TELEOP_STATE;

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
	 *
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
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
 	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
 	 */
	private void handleTeleopState(TeleopInput input) {
		if (input == null) {
			return;
		}

		
		if(input.getLeftJoystickY() > 0.5 || input.getLeftJoystickY() < -.5){
			System.out.println("Running Drive");
		}

		rightMotor.set(input.getRightJoystickY());
		leftMotor.set(-input.getLeftJoystickY());
	}
}
