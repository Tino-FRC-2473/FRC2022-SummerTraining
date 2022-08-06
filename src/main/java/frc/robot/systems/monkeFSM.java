package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class monkeFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		START_IDLE,
		RETRACT_ARM, //grabbers latch on
		IDLE_1,
		IDLE_2,
		LATCHED_RELEASE_TO_CONTINUE,
		EXTEND_ARM_LITTLE,
		IDLE_3,
		ARM_PISTON_EXT, //roll back and extend
		IDLE_4,
		RETRACT_PISTON,
		IDLE_5, 
		FINISHED_RELEASE_TO_CONTINUE
	}

	private static final float MOTOR_RUN_POWER = 0.1f;
	private static final double ARM_MOTOR_RETRACT_POWER = -0.2; 
	private static final double ARM_MOTOR_EXTEND_POWER = 0.2;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax armMotor;
	private DoubleSolenoid armSolenoid;
	private DigitalInput armLimitSwitchFirst;
	private DigitalInput armLimitSwitchSecond;

	//final encoder thresholds for arm motor
	private final int armEncoderLimit = 1000; //THIS IS NOT THE REAL VALUE, FOR PLACEHOLDING PURPOSES!!
	
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
		armLimitSwitchFirst = new DigitalInput(HardwareMap.LIMIT_SWITCH_ID_FIRST);
		armLimitSwitchSecond = new DigitalInput(HardwareMap.LIMIT_SWITCH_ID_SECOND);
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
			case IDLE_1:
				handleFirstIdleState(input);
				break;
			case IDLE_2:
				handleSecondIdleState(input);
				break;
			case LATCHED_RELEASE_TO_CONTINUE:
				handleLatchedRelease(input);
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
			case RETRACT_PISTON:
				handleRetractPiston(input);
				break;
			case IDLE_5:
				handleIdleFiveState(input);
				break;
			case FINISHED_RELEASE_TO_CONTINUE:
				handleFinishState(input);
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
					//if climber button is pressed, continue to next state
					return FSMState.RETRACT_ARM;
				}
				//else, stay in idle
				return FSMState.START_IDLE;
			case RETRACT_ARM:
				if(!input.isClimberButtonPressed()) {
					//if climber button is not pressed, go to idle
					return FSMState.IDLE_1;
				}else if(input.isClimberButtonPressed() && !armLimitSwitchFirst.get()) {
					//if climber is pressed and limit switch has not been activated yet,
					//continue to retract arm
					return FSMState.RETRACT_ARM;
				}else{
					//climber button is pressed and limit switch activated, go to next state
					return FSMState.LATCHED_RELEASE_TO_CONTINUE;
				}
			case IDLE_1:
				if(!input.isClimberButtonPressed()) {
					//climber button not pressed, stay in idle
					return FSMState.IDLE_1;
				}
				//climber button pressed, return to previous state
				return FSMState.RETRACT_ARM;
			case LATCHED_RELEASE_TO_CONTINUE:
				if(input.isClimberButtonPressed()) {
					//if climber button is pressed stay in current state
					//drivers need to release the button
					return FSMState.LATCHED_RELEASE_TO_CONTINUE;
				}
				//button is released continue to next state
				return FSMState.IDLE_2;
			case IDLE_2:
				if(!input.isClimberButtonPressed()) {
					//button needs to be repressed to continue to next state, if not stay in IDLE2
					return FSMState.IDLE_2;
				}
				//button is pressed so continue to next state
				return FSMState.EXTEND_ARM_LITTLE;
			case EXTEND_ARM_LITTLE:
				if(!input.isClimberButtonPressed()) {
					//if climber button not pressed go to idle
					return FSMState.IDLE_3;
				}else if(input.isClimberButtonPressed() && armMotor.get()<armEncoderLimit) {
					//climber button pressed and encoder count less than threshold then continue to extend
					return FSMState.EXTEND_ARM_LITTLE;
				}else{
					//climber button pressed and encoder threshold reached, go to next state
					return FSMState.ARM_PISTON_EXT;
				}
			case IDLE_3:
				if(!input.isClimberButtonPressed()) {
					//climber button not pressed, stay in idle
					return FSMState.IDLE_3;
				}
				//climber button pressed return to previous state
				return FSMState.EXTEND_ARM_LITTLE;
			case ARM_PISTON_EXT:
				if(!input.isClimberButtonPressed()) {
					//climber button not pressed, go to idle
					return FSMState.IDLE_4;
				}else if(input.isClimberButtonPressed() && !armLimitSwitchSecond.get()) {
					//climber button pressed, but limit switch not activated, stay in same state
					return FSMState.ARM_PISTON_EXT;
				}else{
					//climber button pressed and limit switch activated, go to next state
					return FSMState.RETRACT_PISTON;
				}
			case IDLE_4:
				if(!input.isClimberButtonPressed()) {
					//climber button not pressed, stay idle
					return FSMState.IDLE_4;
				}
				//climber button pressed, go to previous state
				return FSMState.ARM_PISTON_EXT;
			case RETRACT_PISTON:
				if(!input.isClimberButtonPressed()) {
					//climber button not pressed, go idle
					return FSMState.IDLE_5;
				}
				//as soon as pressed go to start idle since pistons are immediate
				return FSMState.FINISHED_RELEASE_TO_CONTINUE;
			case IDLE_5:
				if(!input.isClimberButtonPressed()) {
					//climber button not pressed, stay idle
					return FSMState.IDLE_5;
				}
				//climber button pressed, return to previous state
				return FSMState.RETRACT_PISTON;
			case FINISHED_RELEASE_TO_CONTINUE:
				//at this point driver needs to release button and repress to enter new cycle and start from beginning
				if(input.isClimberButtonPressed()) {
					return FSMState.FINISHED_RELEASE_TO_CONTINUE;
				}
				//button released, head to start to redo cycle
				return FSMState.START_IDLE;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in start idle state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleStartIdleState(TeleopInput input) {
		armMotor.set(0);
		armSolenoid.set(Value.kReverse);
	}
	/**
	 * Handle behavior in first retract state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleFirstRetractState(TeleopInput input) {
		armMotor.set(ARM_MOTOR_RETRACT_POWER);
	}

	private void handleFirstIdleState(TeleopInput input) {
		armMotor.set(0);
	}

	private void handleSecondIdleState(TeleopInput input) {
		armMotor.set(0);
	}

	private void handleLatchedRelease(TeleopInput input) {
		armMotor.set(0);

	}

	private void handleExtendLittleState(TeleopInput input) {
		armMotor.set(ARM_MOTOR_EXTEND_POWER);
	}

	private void handleIdleThreeState(TeleopInput input) {
		armMotor.set(0);
	}

	private void handleArmPistonExtendState(TeleopInput input) {
		armSolenoid.set(Value.kForward);
		armMotor.set(ARM_MOTOR_EXTEND_POWER);
	}

	private void handleIdleFourState(TeleopInput input) {
		armMotor.set(0);
	}

	private void handleRetractPiston(TeleopInput input) {
		//i actually dont think this is possible because you cant make the piston only 
		//retract partially
	}

	private void handleIdleFiveState(TeleopInput input) {
		armMotor.set(0);
	}

	private void handleFinishState(TeleopInput input) {
		armMotor.set(0);
	}
}

