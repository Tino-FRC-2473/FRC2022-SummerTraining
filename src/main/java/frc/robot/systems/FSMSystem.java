package frc.robot.systems;

import com.kauailabs.navx.frc.AHRS;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.SPI;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class FSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		/*TELEOP_STATE,*/ TURNING_STATE, IDLE_STATE
	}

	private static final float MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax rightMotor;
	private CANSparkMax leftMotor;
	private AHRS gyro;

	private static final int ANGLE = 180;
	private static final int THRESHOLD = 5;
	private static final double MOVE = 0.5;

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
			case IDLE_STATE:
				handleIdleState(input);
				break;
			case TURNING_STATE:
				handleTurningState(input);
				break;
			// case TELEOP_STATE:
			// 	handleTeleopState(input);
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
			case IDLE_STATE:
				if (gyro.getAngle() < ANGLE - THRESHOLD
					|| gyro.getAngle() > ANGLE+ THRESHOLD) {
					return FSMState.TURNING_STATE;
				} else {
					return FSMState.IDLE_STATE;
				}
			case TURNING_STATE:
				if (gyro.getAngle() < ANGLE - THRESHOLD
					|| gyro.getAngle() > ANGLE+ THRESHOLD) {
					return FSMState.TURNING_STATE;
				} else {
					return FSMState.IDLE_STATE;
				}
			// case TELEOP_STATE:
			// 	return FSMState.TELEOP_STATE;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	private void handleIdleState(TeleopInput input) {
		rightMotor.set(0);
		leftMotor.set(0);
	}

	private void handleTurningState(TeleopInput input) {
		if (input == null) {
			rightMotor.set(-MOVE);
			leftMotor.set(MOVE);
		}
	}

	// private void handleTeleopState(TeleopInput input) {
	// 	if(input != null) {
	// 		double right = -input.getRightJoystickY()-input.getLeftJoystickX();
	// 		double left = input.getRightJoystickY()-input.getLeftJoystickX();
	// 		rightMotor.set(right);
	// 		leftMotor.set(left);
	// 	}
	// }
}
