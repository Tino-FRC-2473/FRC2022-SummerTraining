package frc.robot.systems;

import com.kauailabs.navx.frc.AHRS;
// WPILib Imports
// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.HardwareMap;
// Robot Imports
import frc.robot.TeleopInput;

public class teleOp{
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		MOVE,
		IDLE,
		AUTO
	}

	// private static final float MOTOR_RUN_POWER = 0.1f;
	/* ======================== Private variables ======================== */
	private FSMState currentState;
	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax rightMotor;
	private CANSparkMax leftMotor;

	private AHRS gyro = new AHRS(SPI.Port.kMXP);

	private double ang;

	// private final double MIN_ANG = 5;
	private final double maxAng = 180;
	private final double speed = 0.2;
	private final double five = 5.0;
    private final double one = 1.01;
	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public teleOp() {
		// Perform hardware init
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_RIGHT,
				CANSparkMax.MotorType.kBrushless);
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_LEFT,
				CANSparkMax.MotorType.kBrushless);
		leftMotor.setInverted(true);
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
	 * when the robot is enabled.a
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = FSMState.AUTO;
		gyro.reset();
		gyro.calibrate();
		// Call one tick of update to ensure outputs reflect start state
		// update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 * the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		switch (currentState) {
			case AUTO:
				if (input == null) {
					handle();
				}
				break;
			case MOVE:
				moveHandle(input);
				break;
			case IDLE:
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
	 * the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
		switch (currentState) {
			case AUTO:
				if (input != null) {
					return FSMState.AUTO;
				} else {
					return FSMState.IDLE;
				}

				// case OTHER_STATE:
				// return FSMState.OTHER_STATE;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * input Global TeleopInput if robot in teleop mode or null if
	 * the robot is in autonomous mode.
	 */
	private void handle() {
		rightMotor.set(speed);
		leftMotor.set(speed);
		ang = gyro.getAngle();
		if (maxAng + five / ang == one) {
			gyro.reset();
			currentState = nextState(null);
		}
	}

	private void moveHandle(TeleopInput input) {
		rightMotor.set(0);
		leftMotor.set(0);
	}
}
