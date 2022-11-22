package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class ClimberFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		IDLE,
		EXTEND_TO_MAX,
		IDLE_MAX_EXTENDED,
		RETRACTING_TO_MIN,
		CLIMBER_ARM_HANG,
		ATTACH_STATIC_HANG,
		STATIC_HANG,
		PNEUMATIC_ACTIVATE,
		IDLE_TILT,
		EXTENDING_TILT,
		HIT_NEXT_HOOK
	}

	private static final float ARM_MOTOR_EXTEND_POWER = 0.1f;
	private static final float ARM_MOTOR_RETRACT_POWER = 0.1f;
	private static final double MAX_ENCODER = 50;
	private static int cycleCount = 0;
	private static final int MAX_CYCLE = 4;
	private FSMState currentState;
	private Value preferredValue = Value.kReverse;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax armMotor;
	private DoubleSolenoid armSolenoid;
	private Timer pneumaticTimer;
	private SparkMaxLimitSwitch armLimitSwitchExtend;
	private SparkMaxLimitSwitch armLimitSwitchRetract;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ClimberFSM() {
		armMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_CLIMBER,
				CANSparkMax.MotorType.kBrushless);
		armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
				HardwareMap.PCM_CHANNEL_ARM_CYLINDER_EXTEND,
				HardwareMap.PCM_CHANNEL_ARM_CYLINDER_RETRACT);
		pneumaticTimer = new Timer();
		armLimitSwitchExtend = armMotor.getForwardLimitSwitch(
				SparkMaxLimitSwitch.Type.kNormallyClosed);
		armLimitSwitchRetract = armMotor.getReverseLimitSwitch(
				SparkMaxLimitSwitch.Type.kNormallyClosed);
		armLimitSwitchExtend.enableLimitSwitch(true);
		armLimitSwitchRetract.enableLimitSwitch(true);
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * 
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

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * 
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		Value oldValue = preferredValue;
		switch (currentState) {
			case IDLE:
				handleIdleState(input);
				break;
			case EXTEND_TO_MAX:
				handleExtendToMaxState(input);
				break;
			case IDLE_MAX_EXTENDED:
				handleIdleMaxExtendedState(input);
				break;
			case RETRACTING_TO_MIN:
				handleRetractingToMinState(input);
				break;
			case CLIMBER_ARM_HANG:
				handleClimberArmHangState(input);
				break;
			case ATTACH_STATIC_HANG:
				handleAttachStaticHangState(input);
				break;
			case STATIC_HANG:
				handleStaticHangState(input);
				break;
			case PNEUMATIC_ACTIVATE:
				handlePneumaticActivateState(input);
				break;
			case IDLE_TILT:
				handleIdleTiltState(input);
				break;
			case EXTENDING_TILT:
				handleExtendingTiltState(input);
				break;
			case HIT_NEXT_HOOK:
				handleHitNextHookState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		if (preferredValue != oldValue) {
			armSolenoid.set(preferredValue);
		} else {
			armSolenoid.set(Value.kOff);
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
	 * 
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
		if (input == null) {
			return FSMState.IDLE;
		}
		switch (currentState) {
			case IDLE:
				if (input.isClimberButtonPressed()) {
					// if climber button is pressed, continue to next state
					return FSMState.EXTEND_TO_MAX;
				}
				// else, stay in idle
				return FSMState.IDLE;
			case EXTEND_TO_MAX:
				if (input.maxExtended()) {
					return FSMState.IDLE_MAX_EXTENDED;
				}
				return FSMState.EXTEND_TO_MAX;
			case IDLE_MAX_EXTENDED:
				if (!input.isClimberButtonPressed()) {
					// climber button not pressed, stay in idle
					return FSMState.IDLE_MAX_EXTENDED;
				}
				// climber button pressed, return to previous state
				return FSMState.RETRACTING_TO_MIN;
			case RETRACTING_TO_MIN:
				if (input.minRetracted()) {
					// if climber button is pressed stay in current state
					// drivers need to release the button
					return FSMState.CLIMBER_ARM_HANG;
				}
				// button is released continue to next state
				return FSMState.RETRACTING_TO_MIN;
			case CLIMBER_ARM_HANG:
				if (!input.isClimberButtonPressed()) {
					// button needs to be repressed to continue to next state, if not stay in IDLE2
					return FSMState.CLIMBER_ARM_HANG;
				}
				// button is pressed so continue to next state
				armMotor.getEncoder().setPosition(0);
				return FSMState.ATTACH_STATIC_HANG;
			case ATTACH_STATIC_HANG:
				if (armMotor.getEncoder().getPosition() >= MAX_ENCODER) {
					cycleCount++;
					return FSMState.STATIC_HANG;
				}
				return FSMState.ATTACH_STATIC_HANG;
			case STATIC_HANG:
				if (!input.isClimberButtonPressed() || cycleCount == MAX_CYCLE) {
					// climber button not pressed, stay in idle
					return FSMState.STATIC_HANG;
				}
				// climber button pressed return to previous state
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
					// climber button not pressed, stay idle
					return FSMState.IDLE_TILT;
				}
				// climber button pressed, go to previous state
				return FSMState.EXTENDING_TILT;
			case EXTENDING_TILT:
				if (!input.maxExtended() && input.isClimberButtonPressed()) {
					return FSMState.EXTENDING_TILT;
				} else if (!input.isClimberButtonPressed()) {
					return FSMState.IDLE_TILT;
				} else {
					pneumaticTimer.reset();
					pneumaticTimer.start();
					return FSMState.HIT_NEXT_HOOK;
				}
			case HIT_NEXT_HOOK:
				if (pneumaticTimer.hasElapsed(1)) {
					return FSMState.IDLE_MAX_EXTENDED;
				}
				return FSMState.HIT_NEXT_HOOK;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

	}

	// private void pulseSolenoid(DoubleSolenoid.Value value, TeleopInput input) {
	// FSMState state = nextState(input);
	// if (currentState != state) {
	// armSolenoid.set(value);
	// }
	// armSolenoid.set(Value.kOff);
	// }

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * 
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		armMotor.set(0);
		// pulseSolenoid(Value.kReverse, input);
		preferredValue = Value.kReverse;
	}

	/**
	 * Handle behavior in first retract state.
	 * 
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */

	private void handleExtendToMaxState(TeleopInput input) {
		armMotor.set(ARM_MOTOR_EXTEND_POWER);
		// pulseSolenoid(Value.kOff, input);
		preferredValue = Value.kReverse;
	}

	private void handleIdleMaxExtendedState(TeleopInput input) {
		armMotor.set(0);
		// pulseSolenoid(Value.kOff, input);
		preferredValue = Value.kReverse;
	}

	private void handleRetractingToMinState(TeleopInput input) {
		armMotor.set(ARM_MOTOR_RETRACT_POWER);
		// pulseSolenoid(Value.kOff, input);
		preferredValue = Value.kReverse;
	}

	private void handleClimberArmHangState(TeleopInput input) {
		armMotor.set(0);
		// pulseSolenoid(Value.kOff, input);
		preferredValue = Value.kReverse;
	}

	private void handleAttachStaticHangState(TeleopInput input) {
		armMotor.set(ARM_MOTOR_EXTEND_POWER);
		// armSolenoid.set(Value.kOff);
		preferredValue = Value.kReverse;
	}

	private void handleStaticHangState(TeleopInput input) {
		armMotor.set(0);
		// armSolenoid.set(Value.kOff);
		preferredValue = Value.kReverse;
	}

	private void handlePneumaticActivateState(TeleopInput input) {
		// pulseSolenoid(Value.kForward, input);
		preferredValue = Value.kForward;
		armMotor.set(0);
	}

	private void handleIdleTiltState(TeleopInput input) {
		armMotor.set(0);
		// pulseSolenoid(Value.kOff, input);
		preferredValue = Value.kForward;
	}

	private void handleExtendingTiltState(TeleopInput input) {
		armMotor.set(ARM_MOTOR_EXTEND_POWER);
		// pulseSolenoid(Value.kOff, input);
		preferredValue = Value.kForward;
	}

	private void handleHitNextHookState(TeleopInput input) {
		armMotor.set(0);
		// pulseSolenoid(Value.kReverse, input);
		preferredValue = Value.kReverse;
	}

	// private void updateDashboard(TeleopInput input) {
	// if (input == null) {
	// SmartDashboard.putBoolean("Climber Button Pressed", false);
	// } else {
	// SmartDashboard.putBoolean("Climber Button Pressed",
	// input.isClimberButtonPressed());
	// }
	// SmartDashboard.putNumber("Cycle Count", cycleCount);
	// SmartDashboard.putNumber("Motor Encoder Position",
	// armMotor.getEncoder().getPosition());
	// SmartDashboard.putBoolean("Extension Limit Switch",
	// armLimitSwitchFirst.isPressed());
	// SmartDashboard.putBoolean("Retraction Limit Switch",
	// armLimitSwitchSecond.isPressed());
	// SmartDashboard.putNumber("Motor Power", armMotor.get());
	// SmartDashboard.putNumber("Pneumatic Timer", pneumaticTimer.get());
	// SmartDashboard.putString("Current State", currentState + "");
	// SmartDashboard.putBoolean("Solenoid Extended",
	// armSolenoid.get().equals(Value.kForward));
	// }

}
