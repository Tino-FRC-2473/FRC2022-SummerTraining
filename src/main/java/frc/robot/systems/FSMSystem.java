package frc.robot.systems;

// WPILib Imports
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

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
		IDLE_STATE,
		TURNING_STATE
	}

	private static final float MOTOR_RUN_POWER = 0.1f;
	private static final int MAX_TURN = 180;
	private static final int TURN_AMOUNT = 5;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax rightMotor;
	private CANSparkMax leftMotor;
	private AHRS gyro;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public FSMSystem() {
		// Perform hardware init
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_RIGHT,
		CANSparkMax.MotorType.kBrushless);
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_LEFT,
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
		currentState = FSMState.IDLE_STATE;
		gyro.reset();
		gyro.calibrate();

		// Call one tick of update to ensure outputs reflect start state
		leftMotor.set(MOTOR_RUN_POWER);
		rightMotor.set(MOTOR_RUN_POWER);
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

			case IDLE_STATE:
				handleIdleState(input);
				break;

			case TURNING_STATE:
				handleTurningState(input);
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

			case IDLE_STATE:

				if (gyro.getAngle() < MAX_TURN) {
					return FSMState.TURNING_STATE;
				} else {
					return FSMState.IDLE_STATE;
				}

			case TURNING_STATE:
				if (gyro.getAngle() < MAX_TURN) {
					return FSMState.TURNING_STATE;
				} else {
					return FSMState.IDLE_STATE;
				}

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
		rightMotor.set(input.getRightJoystickY());
		leftMotor.set(input.getLeftJoystickY());
	}

	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		if (input == null) {
			return;
		}
		rightMotor.set(MOTOR_RUN_POWER);
		leftMotor.set(MOTOR_RUN_POWER);
	}
	/**
	 * Handle behavior in OTHER_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 * the robot is in autonomous mode.
	 */
	private void handleTurningState(TeleopInput input) {
		if (input == null) {
			rightMotor.set(-MOTOR_RUN_POWER * TURN_AMOUNT);
			leftMotor.set(MOTOR_RUN_POWER * TURN_AMOUNT);
		}
	}
}
