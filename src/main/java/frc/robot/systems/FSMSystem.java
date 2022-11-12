package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.LimeLight;

public class FSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		START_STATE,
		OTHER_STATE
	}

	private static final float MOTOR_ALIGN_POWER = 0.05f;
	private static final float MOTOR_MOVE_POWER = 0.15f;
	private static final float MOTOR_SEEK_POWER = 0.05f;



	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax leftMotor;
	private CANSparkMax rightMotor;
	private CANSparkMax leftMotor2;
	private CANSparkMax rightMotor2;
	private LimeLight limeLight;
	private String mode = "SHOOT";

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public FSMSystem() {
		// Perform hardware init
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT1,
										CANSparkMax.MotorType.kBrushless);
		rightMotor2 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT2,
										CANSparkMax.MotorType.kBrushless);
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT1,
										CANSparkMax.MotorType.kBrushless);
		leftMotor2 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT2,
										CANSparkMax.MotorType.kBrushless);
		limeLight = new LimeLight();
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
	}
	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		limeLight.update();
		switch (currentState) {
			case START_STATE:
				handleStartState(input);
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

			case OTHER_STATE:
				return FSMState.OTHER_STATE;

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
		if (mode == "COLLECT") {
			if (limeLight.getBallTurnDirection() == 1) {
				leftMotor.set(-MOTOR_ALIGN_POWER);
				rightMotor.set(-MOTOR_ALIGN_POWER);
				leftMotor2.set(-MOTOR_ALIGN_POWER);
				rightMotor2.set(-MOTOR_ALIGN_POWER);
			} else if (limeLight.getBallTurnDirection() == -1) {
				leftMotor.set(MOTOR_ALIGN_POWER);
				rightMotor.set(MOTOR_ALIGN_POWER);
				leftMotor2.set(MOTOR_ALIGN_POWER);
				rightMotor2.set(MOTOR_ALIGN_POWER);
			} else if (limeLight.getBallTurnDirection() == 0) {
				if (limeLight.getIntakeStatus() == 1) {
					leftMotor.set(0.0);
					rightMotor.set(0.0);
					leftMotor2.set(0.0);
					rightMotor2.set(0.0);
					//INTAKE BALL
					//mode = "SHOOT";
				} else {
					leftMotor.set(-MOTOR_MOVE_POWER);
					rightMotor.set(MOTOR_MOVE_POWER);
					leftMotor2.set(-MOTOR_MOVE_POWER);
					rightMotor2.set(MOTOR_MOVE_POWER);
				}
			} else {
				//leftMotor.set(-MOTOR_SEEK_POWER);
				//rightMotor.set(-MOTOR_SEEK_POWER);
				//leftMotor2.set(-MOTOR_SEEK_POWER);
				//rightMotor2.set(-MOTOR_SEEK_POWER);
				leftMotor.set(0);
				rightMotor.set(0);
				leftMotor2.set(0);
				rightMotor2.set(0);
			}
		} else if (mode == "SHOOT") {
			if(){

			
			} else {
				leftMotor.set(-MOTOR_SEEK_POWER);
				rightMotor.set(-MOTOR_SEEK_POWER);
				leftMotor2.set(-MOTOR_SEEK_POWER);
				rightMotor2.set(-MOTOR_SEEK_POWER);
			}
		}
	}
}
