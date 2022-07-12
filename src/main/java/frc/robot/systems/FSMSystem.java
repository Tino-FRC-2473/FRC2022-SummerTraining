package frc.robot.systems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

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
		START_STATE,
		TURN_STATE,
		IDLE_STATE
		//OTHER_STATE
	}

	private static final float MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	private AHRS gyro = new AHRS(SPI.Port.kMXP);

	private static final int TURN_AMT = 180;
	private static final double TURN_VALUE = 0.5;
	private static final int ERROR = 5;


	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax exampleMotorLeft;
	private CANSparkMax exampleMotorRight;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public FSMSystem() {
		// Perform hardware init
		exampleMotorLeft = new CANSparkMax(HardwareMap.CAN_ID_LEFT,
										CANSparkMax.MotorType.kBrushless);
		exampleMotorRight = new CANSparkMax(HardwareMap.CAN_ID_RIGHT,
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
		currentState = FSMState.START_STATE;

		gyro.reset();

		gyro.calibrate();

		exampleMotorLeft.set(MOTOR_RUN_POWER);
		exampleMotorRight.set(MOTOR_RUN_POWER);

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
			case START_STATE:
				handleStartState(input);
				break;
			case IDLE_STATE:
				handleIdleState(input);
				break;
			case TURN_STATE:
				handleTurnState(input);
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
			case START_STATE:
				return FSMState.START_STATE;
			case IDLE_STATE:
				if (gyro.getAngle() % TURN_AMT <= ERROR
					|| gyro.getAngle() % TURN_AMT >= TURN_AMT - ERROR) {
					return FSMState.IDLE_STATE;
				} else {
					return FSMState.TURN_STATE;
				}
			case TURN_STATE:
				if (gyro.getAngle() % TURN_AMT <= ERROR
					|| gyro.getAngle() % TURN_AMT >= TURN_AMT - ERROR) {
					return FSMState.IDLE_STATE;
				} else {
					return FSMState.TURN_STATE;
				}

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleStartState(TeleopInput input) {
		if (input == null) {
			return;
		}
		exampleMotorLeft.set(input.getLeftJoystickY());
		exampleMotorRight.set(input.getRightJoystickY());
	}
	/**
	 * Handle behavior in OTHER_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	// private void handleOtherState(TeleopInput input) {
	// 	exampleMotor.set(MOTOR_RUN_POWER);
	// }

	private void handleTurnState(TeleopInput input) {
		double angle = gyro.getAngle();
		if (angle < TURN_AMT) {
			// values should be from -1 to 1
			exampleMotorLeft.set(TURN_VALUE);
		}
	}

	private void handleIdleState(TeleopInput input) {
		exampleMotorLeft.set(0);
		exampleMotorRight.set(0);
	}
}
