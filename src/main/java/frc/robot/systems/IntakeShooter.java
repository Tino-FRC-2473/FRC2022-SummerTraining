package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class IntakeShooter {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		EXTENDED_RUNNING,
		RETRACTED_NO_BALL,
		RETRACTED_RUNNING,
		RETRACTED_BALL,
		PREP_SHOOTER_MOTOR,
		SHOOT,
		EJECT
	}

	private static final double PROXIMITY_THRESHOLD = 0;
	private static final float MOTOR_RUN_POWER = 0.1f;
	private static final float INTER1_RUN_POWER = 0.1f;
	private static final float INTER2_RUN_POWER = 0.1f;
	private static final double PREP_SHOOTER_MOTOR_DELAY = 1;
	private static final double SHOOT_DELAY = 1;
	private static final double RETRACTED_RUNNING_DELAY = 1;
	private static final double SHOOT_POWER = 0.1;
	private static final double MAX_SHOOT_POWER = 0.1;
	private Value preferredValue = Value.kReverse;
	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax intakeMotor;
	private DoubleSolenoid armSolenoid;
	private CANSparkMax interMotor1;
	private CANSparkMax interMotor2;
	private CANSparkMax shooterMotor;
	private ColorSensorV3 color;
	private Timer shooterTimer;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public IntakeShooter() {
		// Perform hardware init
		/*IMPORTANT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		*BRUSHED OR BRUSHLESS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		*!
		*!
		*!
		*DONT EXPLODE MOTOR!!!!!!!!!!!!!
		*/
		intakeMotor = new CANSparkMax(HardwareMap.INTAKE_MOTOR, CANSparkMax.MotorType.kBrushless);
		armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
		HardwareMap.PCM_CHANNEL_INTAKE_CYLINDER_EXTEND,
		HardwareMap.PCM_CHANNEL_INTAKE_CYLINDER_RETRACT);
		interMotor1 = new CANSparkMax(HardwareMap.INTER1, CANSparkMax.MotorType.kBrushless);
		interMotor2 = new CANSparkMax(HardwareMap.INTER2, CANSparkMax.MotorType.kBrushless);
		shooterMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER,
										CANSparkMax.MotorType.kBrushless);
		color = new ColorSensorV3(Port.kOnboard);
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
		currentState = FSMState.RETRACTED_NO_BALL;
		// Call one tick of update to ensure outputs reflect start state
		//updateDashboard(null);
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
		//System.out.println(currentState);
		//System.out.println(input.isIntakeButtonPressed());
		//updateDashboard(input);
		Value oldValue = preferredValue;
		switch (currentState) {
			case RETRACTED_NO_BALL:
				handleRetractedNoBallState(input);
				break;
			case EXTENDED_RUNNING:
				handleExtendedRunningState(input);
				break;
			case RETRACTED_BALL:
				handleRetractedBallState(input);
				break;
			case PREP_SHOOTER_MOTOR:
				handlePrepMotorState(input);
				break;
			case SHOOT:
				handleShoot(input);
				break;
			case RETRACTED_RUNNING:
				handleRetractedRunningState(input);
			case EJECT:
				handleEjectState(input);
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		if (oldValue != preferredValue) {
			armSolenoid.set(preferredValue);
		} else {
			armSolenoid.set(Value.kOff);
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
		if (input == null) {
			return FSMState.RETRACTED_NO_BALL;
		}
		switch (currentState) {
			case RETRACTED_NO_BALL:
				if (input.isEjectButtonPressed()) {
					return FSMState.EJECT;
				}
				if (ballInIntermediate()) {
					return FSMState.RETRACTED_BALL;
				}
				if (input.isIntakeButtonPressed()) {
					return FSMState.EXTENDED_RUNNING;
				}
				return FSMState.RETRACTED_NO_BALL;
			case EXTENDED_RUNNING:
				if (input.isEjectButtonPressed()) {
					return FSMState.EJECT;
				}
				if (ballInIntermediate()) {
					return FSMState.RETRACTED_BALL;
				}
				if (input.isIntakeButtonPressed()) {
					return FSMState.EXTENDED_RUNNING;
				}
				shooterTimer.reset();
				return FSMState.RETRACTED_RUNNING;
			case RETRACTED_RUNNING:
				if (input.isEjectButtonPressed()) {
					return FSMState.EJECT;
				}
				if (ballInIntermediate()) {
					return FSMState.RETRACTED_BALL;
				}
				if (input.isIntakeButtonPressed()) {
					return FSMState.EXTENDED_RUNNING;
				}
				if (shooterTimer.hasElapsed(RETRACTED_RUNNING_DELAY)) {
					return FSMState.RETRACTED_NO_BALL;
				}
				return FSMState.RETRACTED_RUNNING;
			case RETRACTED_BALL:
				if (input.isEjectButtonPressed()) {
					return FSMState.EJECT;
				}
				if (input.isShooterButtonPressed()) {
					shooterTimer.reset();
					return FSMState.PREP_SHOOTER_MOTOR;
				}
				return FSMState.RETRACTED_BALL;
			case PREP_SHOOTER_MOTOR:
				if (input.isEjectButtonPressed()) {
					return FSMState.EJECT;
				}
				if (shooterReady()) {
					shooterTimer.reset();
					return FSMState.SHOOT;
				}
				return FSMState.PREP_SHOOTER_MOTOR;
			case SHOOT:
				if (input.isEjectButtonPressed()) {
					return FSMState.EJECT;
				}
				if (!shooterFinished()) {
					return FSMState.SHOOT;
				}
				if (ballInIntermediate()) {
					return FSMState.RETRACTED_BALL;
				}
				if (input.isIntakeButtonPressed()) {
					return FSMState.EXTENDED_RUNNING;
				}
				return FSMState.RETRACTED_NO_BALL;
			case EJECT:
				if (input.isEjectButtonPressed()) {
					return FSMState.EJECT;
				}
				return FSMState.RETRACTED_NO_BALL;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}
	private boolean shooterReady() {
		return shooterTimer.hasElapsed(PREP_SHOOTER_MOTOR_DELAY);
	}
	private boolean shooterFinished() {
		return shooterTimer.hasElapsed(SHOOT_DELAY);
	}
	private boolean ballInIntermediate() {
		if (color.getProximity() < PROXIMITY_THRESHOLD) {
			return true;
		}
		return false;
	}
	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleEjectState(TeleopInput input) {
		intakeMotor.set(0);
		preferredValue = Value.kReverse;
		interMotor1.set(INTER1_RUN_POWER);
		interMotor2.set(INTER2_RUN_POWER);
		shooterMotor.set(MAX_SHOOT_POWER);
	}
	private void handleExtendedRunningState(TeleopInput input) {
		intakeMotor.set(MOTOR_RUN_POWER);
		preferredValue = Value.kForward;
		interMotor1.set(INTER1_RUN_POWER);
		interMotor2.set(0);
		shooterMotor.set(0);
	}
	private void handleRetractedBallState(TeleopInput input) {
		intakeMotor.set(0);
		preferredValue = Value.kReverse;
		interMotor1.set(0);
		interMotor2.set(0);
		shooterMotor.set(0);
	}
	private void handleRetractedRunningState(TeleopInput input) {
		intakeMotor.set(MOTOR_RUN_POWER);
		preferredValue = Value.kReverse;
		interMotor1.set(INTER1_RUN_POWER);
		interMotor2.set(0);
		shooterMotor.set(0);
	}

	private void handleRetractedNoBallState(TeleopInput input) {
		intakeMotor.set(0);
		preferredValue = Value.kReverse;
		interMotor1.set(0);
		interMotor2.set(0);
		shooterMotor.set(0);
	}

	private void handlePrepMotorState(TeleopInput input) {
		intakeMotor.set(0);
		preferredValue = Value.kReverse;
		interMotor1.set(0);
		interMotor2.set(0);
		shooterMotor.set(SHOOT_POWER);
		//change shoot power to be cv power
	}

	private void handleShoot(TeleopInput input) {
		intakeMotor.set(0);
		interMotor1.set(INTER1_RUN_POWER);
		interMotor2.set(INTER2_RUN_POWER);
		preferredValue = Value.kReverse;
		shooterMotor.set(SHOOT_POWER);
	}

	private void updateDashboard(TeleopInput input) {
		if (input == null) {
			SmartDashboard.putBoolean("Button Pressed", false);
		} else {
			SmartDashboard.putBoolean("Button Pressed", input.isIntakeButtonPressed());
		}
		SmartDashboard.putNumber("Motor Power", intakeMotor.get());
		//SmartDashboard.putBoolean("Solenoid Extended", armSolenoid.get().equals(Value.kForward));
		SmartDashboard.putString("Current State", currentState + "");
	}

}
