package frc.robot.systems;

import com.kauailabs.navx.frc.AHRS;

//public package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.HardwareMap;
// Robot Imports
import frc.robot.TeleopInput;


public class AutonomousSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		START_STATE, OTHER_STATE, TELEOP_STATE, TURN_STATE, IDLE_STATE
	}

	private static final float MOTOR_RUN_POWER = 0.1f;
    private AHRS gyro = new AHRS(SPI.Port.kMXP);


	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax exampleMotor;
	private CANSparkMax rightMotor;
	private CANSparkMax leftMotor;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public AutonomousSystem() {
		// Perform hardware init
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_RIGHT,
										CANSparkMax.MotorType.kBrushless);
										
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_LEFT,
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
		currentState = FSMState.TURN_STATE;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
        gyro.reset();
        gyro.calibrate();
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
				handleStartState(input);
				break;
			case OTHER_STATE:
				handleOtherState(input);
				break;

			case TELEOP_STATE:
				handleTeleOpState(input);
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
				if (input != null) {
					return FSMState.OTHER_STATE;
				} else {
					return FSMState.START_STATE;
				}

			case OTHER_STATE:
				return FSMState.OTHER_STATE;
			case TELEOP_STATE:
				return FSMState.TELEOP_STATE;
			case TURN_STATE:
				double angle = gyro.getAngle();
				if (angle <= 180) {
					return FSMState.TURN_STATE;
				} else {
					return FSMState.IDLE_STATE;
				}
			case IDLE_STATE:
				return FSMState.IDLE_STATE;

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
		leftMotor.set(0);
		rightMotor.set(0);
	}
	/**
	 * Handle behavior in OTHER_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleOtherState(TeleopInput input) {
		exampleMotor.set(MOTOR_RUN_POWER);
	}

	private void handleTeleOpState(TeleopInput input) {
		if (input == null) {
			return; 
		} 
		leftMotor.set(input.getLeftJoystickY());
		rightMotor.set(-input.getRightJoystickY());
	}

    private void handleTurnState(TeleopInput input) {
        leftMotor.set(-0.1);
		rightMotor.set(-0.1);
	}
}