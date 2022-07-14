package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.SPI;

// Third party Hardware Imports
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class FSMSystem {

	/* ======================== Constants ======================== */

	// FSM state definitions
	public enum FSMState {
		TELEOP_STATE,
		ROTATE_STATE,
		BUTTON_STATE,
		IDLE_STATE,
		TURN_STATE
	}

	private static final float MOTOR_RUN_POWER = 0.5f;
	private static final int MAX_TURN = 180;
	private static final int THRESHOLD = 5;

	/* ======================== Private variables ======================== */

	private FSMState currentState;
	private boolean buttonPushed;

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
		buttonPushed = false;

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
		currentState = FSMState.BUTTON_STATE;
		gyro.reset();
		gyro.calibrate();

		// Call one tick of update to ensure outputs reflect start state
		leftMotor.set(0);
		rightMotor.set(0);
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

			case ROTATE_STATE:
				handleRotateState(input);
				break;

			case BUTTON_STATE:
				handleButtonState(input);
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
			case BUTTON_STATE:
				return FSMState.BUTTON_STATE;

			case TELEOP_STATE:
				if (input.isShooterButtonPressed()) {
					return FSMState.ROTATE_STATE;
				} else {
					return FSMState.TELEOP_STATE;
				}

			case ROTATE_STATE:
				if (gyro.getAngle() > MAX_TURN - THRESHOLD || gyro.getAngle() < THRESHOLD - MAX_TURN) {
					gyro.reset();
					gyro.calibrate();
					return FSMState.TELEOP_STATE;
				} else {
					return FSMState.ROTATE_STATE;
				}

			case IDLE_STATE:
				if (gyro.getAngle() >= -THRESHOLD && gyro.getAngle() <= THRESHOLD) {
					return FSMState.TURN_STATE;
				} else if (gyro.getAngle() % MAX_TURN < THRESHOLD
					|| gyro.getAngle() % MAX_TURN > MAX_TURN - THRESHOLD) {
					return FSMState.IDLE_STATE;
				} else {
					return FSMState.TURN_STATE;
				}

			case TURN_STATE:
				if (gyro.getAngle() >= -THRESHOLD && gyro.getAngle() <= THRESHOLD) {
					return FSMState.TURN_STATE;
				} else if (gyro.getAngle() % MAX_TURN < THRESHOLD
					|| gyro.getAngle() % MAX_TURN > MAX_TURN - THRESHOLD) {
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
 	 * Handle behavior in BUTTON_STATE.
 	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
 	 */
	private void handleButtonState(TeleopInput input) {
		if (input == null) {
			return;
		}

		if (input.isShooterButtonPressed()) {
			buttonPushed = true;
		}

		if (buttonPushed) {
			rightMotor.set(MOTOR_RUN_POWER);
			leftMotor.set(MOTOR_RUN_POWER);
			if (gyro.getAngle() > MAX_TURN - THRESHOLD || gyro.getAngle() < THRESHOLD - MAX_TURN) {
				buttonPushed = false;
				gyro.reset();
				gyro.calibrate();
			}
		} else {
			rightMotor.set(-input.getRightJoystickY());
			leftMotor.set(input.getLeftJoystickY());
		}
	}

	/**
	 * Handle behavior in ROTATE_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleRotateState(TeleopInput input) {
		rightMotor.set(MOTOR_RUN_POWER);
		leftMotor.set(MOTOR_RUN_POWER);
	}

	/**
	 * Handle behavior in IDLE_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		rightMotor.set(0);
		leftMotor.set(0);
	}

	/**
	 * Handle behavior in TURN_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTurnState(TeleopInput input) {
		rightMotor.set(MOTOR_RUN_POWER);
		leftMotor.set(MOTOR_RUN_POWER);
	}
}
