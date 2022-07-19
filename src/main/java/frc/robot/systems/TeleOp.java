package frc.robot.systems;

import com.kauailabs.navx.frc.AHRS;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class TeleOp {

	private static final double ACCELERATION_CONSTANT = 0.5;

	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		MOVE,
		TURN_STATE,
		IDLE
	}

	private static final float MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	private CANSparkMax left;
	private CANSparkMax right;
	private AHRS gyro;
	private final double target = 180;
	private final double error = 5.0;
	private final double leftTurnSpeed = 0.1f;
	private final double rightTurnSpeed = -0.1f;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public TeleOp() {
		// Perform hardware init
		right = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_RIGHT,
				CANSparkMax.MotorType.kBrushless);
		left.setInverted((true));
		left = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_LEFT,
				CANSparkMax.MotorType.kBrushless);
		gyro = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);
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
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = FSMState.MOVE;
		gyro.reset();
		gyro.calibrate();

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		switch (currentState) {
			case MOVE:
				handleMoveState(input);
				break;
			case TURN_STATE:
				handleTurnState(input);
				break;
			case IDLE:
				handleIdleState(input);
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
	 *              the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
		switch (currentState) {
			case MOVE:
				if (input != null) {
					return FSMState.MOVE;
				} else {
					return FSMState.TURN_STATE;
				}
			case TURN_STATE:
				if (input != null) {
					return FSMState.MOVE;
				} else {
					return FSMState.TURN_STATE;
				}
			case IDLE:
				return FSMState.IDLE;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */

	/**
	 * Handle behavior in OTHER_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	private void handleMoveState(TeleopInput input) {
		// double ld = input.getLeftJoystickY() - left.get();
		// double rd = input.getRightJoystickY() - right.get();
		// left.set(left.get()+ld*ACCELERATION_CONSTANT);
		// right.set(right.get()+rd*ACCELERATION_CONSTANT);
		double lp = input.getLeftJoystickY();
		double rp = input.getRightJoystickY();

		left.set(lp);
		right.set(rp);
	}

	private void handleTurnState(TeleopInput input) {
		if (input != null) {
			return;
		}

		double currAngle = Math.abs(gyro.getAngle());

		double maxAngle = target + error;
		double minAngle = target - error;
		if (currAngle < minAngle || currAngle > maxAngle) {
			left.set(leftTurnSpeed);
			right.set(rightTurnSpeed);
		} else {
			currentState = FSMState.IDLE;
		}

	}

	private void handleIdleState(TeleopInput input) {
		left.set(0);
		right.set(0);
	}

}
