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
		IDLE_STATE,
		TURN_STATE,
	}

	//private static final float MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */
	private static final double THRESHOLD = 5;
	private static final double ANGLE = 180;
	private static final double POW = 0.5;
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
	public FSMSystem() {
		// Perform hardware init
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_LEFT,
										CANSparkMax.MotorType.kBrushless);
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_RIGHT,
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
		System.out.println(currentState + " " + gyro.getAngle());
		switch (currentState) {
			case IDLE_STATE:
				handleStartState(input);
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
			case IDLE_STATE:
				if (gyro.getAngle() >= ANGLE - THRESHOLD && gyro.getAngle() <= ANGLE + THRESHOLD) {
					return FSMState.IDLE_STATE;
				} else {
					return FSMState.TURN_STATE;
				}
			case TURN_STATE:
				if (gyro.getAngle() >= ANGLE - THRESHOLD && gyro.getAngle() <= ANGLE + THRESHOLD) {
					return FSMState.IDLE_STATE;
				} else {
					return FSMState.TURN_STATE;
				}
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	private void handleTurnState(TeleopInput input) {
		System.out.println("TURN_STATE:" + gyro.getAngle());
		if (input == null) {
			leftMotor.set(POW);
			rightMotor.set(POW);
		}
	}

	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleStartState(TeleopInput input) {
		System.out.println("IDLE_STATE");
		leftMotor.set(0);
		rightMotor.set(0);
	}
}
