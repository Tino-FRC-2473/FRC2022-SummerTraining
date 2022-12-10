package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class ClimberTester {
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
	}

	private static final float ARM_MOTOR_EXTEND_POWER = 0.1f;
	private static final float ARM_MOTOR_RETRACT_POWER = -0.1f;
	private static final double MAX_ENCODER = 50;
	private static int cycleCount = 0;
	private static final int MAX_CYCLE = 4;
	private FSMState currentState;
	private Value preferredValue = Value.kReverse;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax armMotorLeft;
	private CANSparkMax armMotorRight;
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
	public ClimberTester() {
		armMotorLeft = new CANSparkMax(HardwareMap.CAN_ID_SPARK_CLIMBER_LEFT,
				CANSparkMax.MotorType.kBrushless);
		armMotorRight = new CANSparkMax(HardwareMap.CAN_ID_SPARK_CLIMBER_RIGHT,
				CANSparkMax.MotorType.kBrushless);
		//armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
				//HardwareMap.PCM_CHANNEL_ARM_CYLINDER_EXTEND,
				//HardwareMap.PCM_CHANNEL_ARM_CYLINDER_RETRACT);
		//pneumaticTimer = new Timer();
		armLimitSwitchExtend = armMotorLeft.getForwardLimitSwitch(
				SparkMaxLimitSwitch.Type.kNormallyClosed);
		armLimitSwitchRetract = armMotorRight.getReverseLimitSwitch(
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
		armMotorLeft.getEncoder().setPosition(0);
		armMotorRight.getEncoder().setPosition(0);
		cycleCount = 0;
		//armSolenoid.set(Value.kReverse); // reset piston
		pneumaticTimer.reset();
		updateDashboard(null);
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
		updateDashboard(input);
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
				armMotorLeft.getEncoder().setPosition(0);
				armMotorRight.getEncoder().setPosition(0);
				return FSMState.ATTACH_STATIC_HANG;
			case ATTACH_STATIC_HANG:
				if (input.isClimberButtonPressed()) {
					cycleCount++;
					System.out.println(armMotorRight.getEncoder().getPosition());
					System.out.println(armMotorLeft.getEncoder().getPosition());
					return FSMState.STATIC_HANG;
				}
				return FSMState.ATTACH_STATIC_HANG;
			case STATIC_HANG:
				if (!input.isClimberButtonPressed() || cycleCount == MAX_CYCLE) {
					// climber button not pressed, stay in idle
					return FSMState.STATIC_HANG;
				}
				// climber button pressed return to previous state
				return FSMState.STATIC_HANG;
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
		armMotorRight.set(0);
		armMotorLeft.set(0);
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
		armMotorRight.set(ARM_MOTOR_EXTEND_POWER);
		armMotorLeft.set(-ARM_MOTOR_EXTEND_POWER);
		// pulseSolenoid(Value.kOff, input);
		preferredValue = Value.kReverse;
	}

	private void handleIdleMaxExtendedState(TeleopInput input) {
		armMotorLeft.set(0);
		armMotorRight.set(0);
		// pulseSolenoid(Value.kOff, input);
		preferredValue = Value.kReverse;
	}

	private void handleRetractingToMinState(TeleopInput input) {
		armMotorLeft.set(-ARM_MOTOR_RETRACT_POWER);
		armMotorRight.set(ARM_MOTOR_RETRACT_POWER);
		// pulseSolenoid(Value.kOff, input);
		preferredValue = Value.kReverse;
	}

	private void handleClimberArmHangState(TeleopInput input) {
		armMotorLeft.set(0);
		armMotorRight.set(0);
		// pulseSolenoid(Value.kOff, input);
		preferredValue = Value.kReverse;
	}

	private void handleAttachStaticHangState(TeleopInput input) {
		armMotorLeft.set(-ARM_MOTOR_EXTEND_POWER);
		armMotorRight.set(ARM_MOTOR_EXTEND_POWER);
		// armSolenoid.set(Value.kOff);
		preferredValue = Value.kReverse;
	}

	private void handleStaticHangState(TeleopInput input) {
		armMotorLeft.set(0);
		armMotorRight.set(0);
		// armSolenoid.set(Value.kOff);
		preferredValue = Value.kReverse;
	}

	private void updateDashboard(TeleopInput input) {
		if (input == null) {
			SmartDashboard.putBoolean("Climber Button Pressed", false);
		} else {
			SmartDashboard.putBoolean("Climber Button Pressed",
				input.isClimberButtonPressed());
			SmartDashboard.putBoolean("Max Extended", input.maxExtended());
			SmartDashboard.putBoolean("Retraced to Min", input.minRetracted());
		}
		SmartDashboard.putNumber("Motor Encoder Position",
			armMotorLeft.getEncoder().getPosition());
		SmartDashboard.putNumber("Motor Encoder Position",
			armMotorRight.getEncoder().getPosition());
		SmartDashboard.putNumber("Motor Power", armMotorLeft.get());
		SmartDashboard.putNumber("Motor Power", armMotorRight.get());
		SmartDashboard.putString("Current State", currentState + "");
	}

}
