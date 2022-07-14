package frc.robot.systems;
import com.kauailabs.navx.frc.AHRS;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class TeleOp {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		TELEOP_STATE,
		TURN_STATE,
		IDLE_STATE
	}

	private static final float MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax leftMotor;
	private CANSparkMax rightMotor;
	private AHRS gyro;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public TeleOp() {
		// Perform hardware init
		leftMotor = new CANSparkMax(HardwareMap.LEFT_MOTOR, CANSparkMax.MotorType.kBrushless);
		rightMotor = new CANSparkMax(HardwareMap.RIGHT_MOTOR, CANSparkMax.MotorType.kBrushless);
		leftMotor.setInverted(true);
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
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = FSMState.TELEOP_STATE;
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
			case TELEOP_STATE:
				handleTeleOpState(input);
				break;
			case TURN_STATE:
				handleTurnState(input);
				break;
			case IDLE_STATE:
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
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
		switch (currentState) {
			case TELEOP_STATE:
				if (input != null) {
					return FSMState.TELEOP_STATE;
				} else {
					return FSMState.TURN_STATE;
				}
			case TURN_STATE:
				if (input != null) {
					return FSMState.TELEOP_STATE;
				} else {
					return FSMState.TURN_STATE;
				}
			case IDLE_STATE:
				return FSMState.IDLE_STATE;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	private void handleTeleOpState(TeleopInput input) {
		if (input == null) {
			return;
		}
		double l = input.getLeftJoystickY();
		double r = input.getRightJoystickY();
		leftMotor.set(l);
		rightMotor.set(r);
	}

	private void handleTurnState(TeleopInput input) {
		if (input != null) {
			return;
		}
		double angle = Math.abs(gyro.getAngle());
		final double MAX_ANGLE = 5;
		final double MIN_ANGLE = 175;
		final double RIGHT_SPEED = -0.1;
		final double LEFT_SPEED = 0.1;
		final double SIMPLIFY_NUM = 180;
		if (angle >= MIN_ANGLE) {
			angle %= SIMPLIFY_NUM;
			if (angle <= MAX_ANGLE || angle >= MIN_ANGLE) {
				currentState = FSMState.IDLE_STATE;
				return;
			} else {
				rightMotor.set(RIGHT_SPEED);
				leftMotor.set(LEFT_SPEED);
			}
		} else {
			rightMotor.set(RIGHT_SPEED);
			leftMotor.set(LEFT_SPEED);
		}

	}

	private void handleIdleState(TeleopInput input) {
		double angle = Math.abs(gyro.getAngle());
		final double MAX_ANGLE = 5;
		final double MIN_ANGLE = 175;
		final double SIMPLIFY_NUM = 180;
		if (angle >= MIN_ANGLE) {
			angle %= SIMPLIFY_NUM;
			if (angle <= MAX_ANGLE || angle >= MIN_ANGLE) {
				currentState = FSMState.TURN_STATE;
				return;
			}
		}
	}
}
