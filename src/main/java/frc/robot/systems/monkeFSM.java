package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class monkeFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		START_IDLE,
		RETRACT_ARM, //grabbers latch on
		IDLE_2,
		EXTEND_ARM_LITTLE,
		IDLE_3,
		ARM_PISTON_EXT, //roll back and extend
		IDLE_4,
		RETRACT_ARM_2
	}

	private static final float MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax armMotor;
	private DoubleSolenoid armSolenoid;
	private DigitalInput armLimitSwitch;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public monkeFSM() {
		// Perform hardware init
		armMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_CLIMBER, CANSparkMax.MotorType.kBrushless);
		armSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, HardwareMap.PCM_CHANNEL_ARM_CYLINDER_EXTEND, HardwareMap.PCM_CHANNEL_ARM_CYLINDER_RETRACT);

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
		currentState = FSMState.START_IDLE;

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
			case START_IDLE:
				handleStartIdleState(input);
				break;
			case RETRACT_ARM:
				handleFirstRetractState(input);
				break;
			case IDLE_2:
				handleSecondIdleState(input);
				break;
			case EXTEND_ARM_LITTLE:
				handleExtendLittleState(input);
				break;
			case IDLE_3:
				handleIdleThreeState(input);
				break;
			case ARM_PISTON_EXT:
				handleArmPistonExtendState(input);
				break;
			case IDLE_4:
				handleIdleFourState(input);
				break;
			case RETRACT_ARM_2:
				handleSecondRetractState(input);
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
			case START_IDLE:
				if(input.isClimberButtonPressed()) {
					return FSMState.RETRACT_ARM;
				}
				return FSMState.START_IDLE;
			case RETRACT_ARM:
				return FSMState.OTHER_STATE;
			case IDLE_2:
				return FSMState.OTHER_STATE;
			case EXTEND_ARM_LITTLE:
				return FSMState.OTHER_STATE;
			case IDLE_3:
				return FSMState.OTHER_STATE;
			case ARM_PISTON_EXT:
				return FSMState.OTHER_STATE;
			case IDLE_4:
				return FSMState.OTHER_STATE;
			case RETRACT_ARM_2:
				return FSMState.OTHER_STATE;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		// START_IDLE,
		// RETRACT_ARM, //grabbers latch on
		// IDLE_2,
		// EXTEND_ARM_LITTLE,
		// IDLE_3,
		// ARM_PISTON_EXT, //roll back and extend
		// IDLE_4,
		// RETRACT_ARM_2
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in start idle state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleStartIdleState(TeleopInput input) {
		exampleMotor.set(0);
	}
	/**
	 * Handle behavior in first retract state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleFirstRetractState(TeleopInput input) {
		exampleMotor.set(MOTOR_RUN_POWER);
	}

	private void handleSecondIdleState(TeleopInput input) {
		exampleMotor.set(MOTOR_RUN_POWER);
	}

	private void handleExtendLittleState(TeleopInput input) {
		exampleMotor.set(MOTOR_RUN_POWER);
	}

	private void handleIdleThreeState(TeleopInput input) {
		exampleMotor.set(MOTOR_RUN_POWER);
	}

	private void handleArmPistonExtendState(TeleopInput input) {
		exampleMotor.set(MOTOR_RUN_POWER);
	}

	private void handleIdleFourState(TeleopInput input) {
		exampleMotor.set(MOTOR_RUN_POWER);
	}

	private void handleSecondRetractState(TeleopInput input) {
		exampleMotor.set(MOTOR_RUN_POWER);
	}
}

