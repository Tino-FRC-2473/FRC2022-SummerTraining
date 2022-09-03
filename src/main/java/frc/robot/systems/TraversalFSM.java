package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class TraversalFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		IDLE,
		EXTEND_TOTAL, //grabbers extend to max height
		IDLE_MAX_EXTENDED,
		RETRACTING_TO_MIN,
		REST_STATE,
		EXTEND_PARTIAL,
		STATIC_HANG,
		PNEUMATIC_ACTIVATE,
		IDLE_TILT,
		EXTENDING_TILT,
		IDLE_MAX_EXTENDED2
	}

	private static final double ARM_MOTOR_RETRACT_POWER = -0.1;
	private static final double ARM_MOTOR_EXTEND_POWER = 0.1;
	private static final int ARM_ENCODER_LIMIT = 50;
	private static final int MAX_CYCLE = 3;
	private int cycleCount = 0;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax armMotor;
	private DoubleSolenoid armSolenoid;
	private SparkMaxLimitSwitch armLimitSwitchFirst;
	private SparkMaxLimitSwitch armLimitSwitchSecond;
	private Timer pneumaticTimer;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public TraversalFSM() {
		// Perform hardware init
		armMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_CLIMBER,
			CANSparkMax.MotorType.kBrushless);
		armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
			HardwareMap.PCM_CHANNEL_ARM_CYLINDER_EXTEND,
			HardwareMap.PCM_CHANNEL_ARM_CYLINDER_RETRACT);
		armLimitSwitchFirst =
		armMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
		armLimitSwitchFirst.enableLimitSwitch(true);
		armLimitSwitchSecond =
		armMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
		armLimitSwitchSecond.enableLimitSwitch(true);
		pneumaticTimer = new Timer();
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
		currentState = FSMState.IDLE;
		armMotor.getEncoder().setPosition(0);
		cycleCount = 0;
		armSolenoid.set(Value.kReverse); // reset piston
		pneumaticTimer.reset();
		updateDashboard(null);
		update(null);
	}
	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		updateDashboard(input);
		switch (currentState) {
			case IDLE:
				handleIdleState(input);
				break;
			case EXTEND_TOTAL:
				handleExtendTotalState(input);
				break;
			case IDLE_MAX_EXTENDED:
				handleIdleMaxExtendedState(input);
				break;
			case RETRACTING_TO_MIN:
				handleRetractingToMinState(input);
				break;
			case REST_STATE:
				handleRestState(input);
				break;
			case EXTEND_PARTIAL:
				handleExtendPartial(input);
				break;
			case STATIC_HANG:
				handleStaticHang(input);
				break;
			case PNEUMATIC_ACTIVATE:
				handlePneumaticActivate(input);
				break;
			case IDLE_TILT:
				handleIdleTilt(input);
				break;
			case EXTENDING_TILT:
				handleExtendingTilt(input);
				break;
			case IDLE_MAX_EXTENDED2:
				handleIdleMaxExtended2(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		FSMState state = nextState(input);
		if (currentState != state) {
			System.out.println(state);
		}
		currentState = state;
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
		if (input == null) {
			return FSMState.IDLE;
		}
		switch (currentState) {
			case IDLE:
				if (input.isClimberButtonPressed()) {
					//if climber button is pressed, continue to next state
					return FSMState.EXTEND_TOTAL;
				}
				//else, stay in idle
				return FSMState.IDLE;
			case EXTEND_TOTAL:
				if (getFirstCondition()) {
					return FSMState.IDLE_MAX_EXTENDED;
				}
				return FSMState.EXTEND_TOTAL;
			case IDLE_MAX_EXTENDED:
				if (!input.isClimberButtonPressed()) {
					//climber button not pressed, stay in idle
					return FSMState.IDLE_MAX_EXTENDED;
				}
				//climber button pressed, return to previous state
				return FSMState.RETRACTING_TO_MIN;
			case IDLE_MAX_EXTENDED2:
				if (input.isClimberButtonReleased()) {
					return FSMState.IDLE_MAX_EXTENDED;
				}
				return FSMState.IDLE_MAX_EXTENDED2;
			case RETRACTING_TO_MIN:
				if (getSecondCondition()) {
					//if climber button is pressed stay in current state
					//drivers need to release the button
					return FSMState.REST_STATE;
				}
				//button is released continue to next state
				return FSMState.RETRACTING_TO_MIN;
			case REST_STATE:
				if (!input.isClimberButtonPressed()) {
					//button needs to be repressed to continue to next state, if not stay in IDLE2
					return FSMState.REST_STATE;
				}
				//button is pressed so continue to next state
				armMotor.getEncoder().setPosition(0);
				return FSMState.EXTEND_PARTIAL;
			case EXTEND_PARTIAL:
				if (littleExtensionEncoder()) {
					cycleCount++;
					return FSMState.STATIC_HANG;
				}
				return FSMState.EXTEND_PARTIAL;
			case STATIC_HANG:
				if (!input.isClimberButtonPressed() || cycleCount == MAX_CYCLE) {
					//climber button not pressed, stay in idle
					return FSMState.STATIC_HANG;
				}
				//climber button pressed return to previous state
				pneumaticTimer.reset();
				pneumaticTimer.start();
				return FSMState.PNEUMATIC_ACTIVATE;
			case PNEUMATIC_ACTIVATE:
				if (pneumaticTimer.hasElapsed(1)) {
					return FSMState.IDLE_TILT;
				}
				return FSMState.PNEUMATIC_ACTIVATE;
			case IDLE_TILT:
				if (!input.isClimberButtonPressed()) {
					//climber button not pressed, stay idle
					return FSMState.IDLE_TILT;
				}
				//climber button pressed, go to previous state
				return FSMState.EXTENDING_TILT;
			case EXTENDING_TILT:
				if (!getFirstCondition() && input.isClimberButtonPressed()) {
					return FSMState.EXTENDING_TILT;
				} else if (!input.isClimberButtonPressed()) {
					return FSMState.IDLE_TILT;
				} else {
					return FSMState.IDLE_MAX_EXTENDED2;
				}
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
	private void handleIdleState(TeleopInput input) {
		armMotor.set(0);
		armSolenoid.set(Value.kReverse);
	}
	/**
	 * Handle behavior in first retract state.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleExtendTotalState(TeleopInput input) {
		armMotor.set(ARM_MOTOR_EXTEND_POWER);
		armSolenoid.set(Value.kReverse);
	}

	private void handleIdleMaxExtendedState(TeleopInput input) {
		armMotor.set(0);
		armSolenoid.set(Value.kReverse);
	}

	private void handleRetractingToMinState(TeleopInput input) {
		armMotor.set(ARM_MOTOR_RETRACT_POWER);
		armSolenoid.set(Value.kReverse);
	}

	private void handleRestState(TeleopInput input) {
		armMotor.set(0);
		armSolenoid.set(Value.kReverse);
	}

	private void handleExtendPartial(TeleopInput input) {
		armMotor.set(ARM_MOTOR_EXTEND_POWER);
		armSolenoid.set(Value.kReverse);
	}

	private void handleStaticHang(TeleopInput input) {
		armMotor.set(0);
		armSolenoid.set(Value.kReverse);
	}

	private void handlePneumaticActivate(TeleopInput input) {
		armSolenoid.set(Value.kForward);
		armMotor.set(0);
	}

	private void handleIdleTilt(TeleopInput input) {
		armMotor.set(0);
		armSolenoid.set(Value.kReverse);
	}

	private void handleExtendingTilt(TeleopInput input) {
		armMotor.set(ARM_MOTOR_EXTEND_POWER);
		armSolenoid.set(Value.kReverse);
	}

	private void handleIdleMaxExtended2(TeleopInput input) {
		armMotor.set(0);
		armSolenoid.set(Value.kReverse);
	}
	private boolean littleExtensionEncoder() {
		return armMotor.getEncoder().getPosition() >= ARM_ENCODER_LIMIT;
	}

	private boolean getFirstCondition() {
		return armLimitSwitchFirst.isPressed();
	}

	private boolean getSecondCondition() {
		return armLimitSwitchSecond.isPressed();
	}

	private void updateDashboard(TeleopInput input) {
		if (input == null) {
			SmartDashboard.putBoolean("Climber Button Pressed", false);
		} else {
			SmartDashboard.putBoolean("Climber Button Pressed", input.isClimberButtonPressed());
		}
		SmartDashboard.putNumber("Cycle Count", cycleCount);
		SmartDashboard.putNumber("Motor Encoder Position", armMotor.getEncoder().getPosition());
		SmartDashboard.putBoolean("Extension Limit Switch", armLimitSwitchFirst.isPressed());
		SmartDashboard.putBoolean("Retraction Limit Switch", armLimitSwitchSecond.isPressed());
		SmartDashboard.putNumber("Motor Power", armMotor.get());
		SmartDashboard.putNumber("Pneumatic Timer", pneumaticTimer.get());
		SmartDashboard.putString("Current State", currentState + "");
		SmartDashboard.putBoolean("Solenoid Extended", armSolenoid.get().equals(Value.kForward));
	}
}
