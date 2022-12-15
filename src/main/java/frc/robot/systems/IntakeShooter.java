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
import frc.robot.LimeLight;

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



	private static double currentDist;
	private static final double PROXIMITY_THRESHOLD = 300;
	private static final float MOTOR_RUN_POWER = 0.1f;
	private static final float TRANSFER_RUN_POWER = 0.05f;
	private static final float INTER_RUN_POWER = 0.1f;
	private static final double PREP_SHOOTER_MOTOR_DELAY = 1;
	private static final double SHOOT_DELAY = 1;
	private static final double RETRACTED_RUNNING_DELAY = 1;
	//private static final double SHOOT_POWER = 0.1;
	private static final double MAX_SHOOT_POWER = 0.1;
	private static final float PREP_RUN_POWER = 0.3f;
	private Value preferredValue = Value.kReverse;
	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax intakeMotor;
	private DoubleSolenoid armSolenoid;
	private DoubleSolenoid armSolenoid2;
	private CANSparkMax transferMotor1;
	private CANSparkMax transferMotor2;
	private CANSparkMax interMotor;
	private CANSparkMax shooterMotor;
	private ColorSensorV3 color;
	private Timer shooterTimer;
	private LimeLight limeLight;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public IntakeShooter() {
		// Perform hardware init
		/*
		 * IMPORTANT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		 * BRUSHED OR BRUSHLESS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		 * !
		 * !
		 * !
		 * DONT EXPLODE MOTOR!!!!!!!!!!!!!
		 */
		limeLight = new LimeLight();
		intakeMotor = new CANSparkMax(HardwareMap.INTAKE_MOTOR, CANSparkMax.MotorType.kBrushless);
		armSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
				HardwareMap.PCM_CHANNEL_INTAKE_CYLINDER_EXTEND,
				HardwareMap.PCM_CHANNEL_INTAKE_CYLINDER_RETRACT);
		armSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH,
				HardwareMap.PCM_CHANNEL_INTAKE_CYLINDER_EXTEND2,
				HardwareMap.PCM_CHANNEL_INTAKE_CYLINDER_RETRACT2);
		transferMotor1 = new CANSparkMax(HardwareMap.TRANSFER_MOTOR_LEFT,
							CANSparkMax.MotorType.kBrushless);
		transferMotor2 = new CANSparkMax(HardwareMap.TRANSFER_MOTOR_RIGHT,
							CANSparkMax.MotorType.kBrushless);
		interMotor = new CANSparkMax(HardwareMap.INTER, CANSparkMax.MotorType.kBrushless);
		shooterMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER,
				CANSparkMax.MotorType.kBrushed);
		color = new ColorSensorV3(Port.kOnboard);
		shooterTimer = new Timer();
		shooterTimer.start();

		// Reset state machine
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
		currentState = FSMState.RETRACTED_NO_BALL;
		// Call one tick of update to ensure outputs reflect start state
		// updateDashboard(null);
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
		updateDashboard(input);
		// System.out.println(currentState);
		// System.out.println(input.isIntakeButtonPressed());
		// updateDashboard(input);
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
				break;
			case EJECT:
				handleEjectState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		if (oldValue != preferredValue) {
			armSolenoid.set(preferredValue);
			armSolenoid2.set(preferredValue);
		} else {
			armSolenoid.set(Value.kOff);
			armSolenoid2.set(Value.kOff);
		}
		currentState = nextState(input);
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
		if (color.getProximity() > PROXIMITY_THRESHOLD) {
			return true;
		}
		return false;
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 *
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	private void handleEjectState(TeleopInput input) {
		intakeMotor.set(0);
		preferredValue = Value.kReverse;
		transferMotor1.set(TRANSFER_RUN_POWER);
		transferMotor2.set(TRANSFER_RUN_POWER);
		shooterMotor.set(MAX_SHOOT_POWER);
		interMotor.set(INTER_RUN_POWER);
	}

	private void handleExtendedRunningState(TeleopInput input) {
		intakeMotor.set(MOTOR_RUN_POWER);
		preferredValue = Value.kForward;
		transferMotor1.set(TRANSFER_RUN_POWER);
		transferMotor2.set(TRANSFER_RUN_POWER);
		shooterMotor.set(0);
		interMotor.set(0);
	}

	private void handleRetractedBallState(TeleopInput input) {
		intakeMotor.set(0);
		preferredValue = Value.kReverse;
		transferMotor1.set(0);
		transferMotor2.set(0);
		shooterMotor.set(0);
		interMotor.set(0);
	}

	private void handleRetractedRunningState(TeleopInput input) {
		intakeMotor.set(MOTOR_RUN_POWER);
		preferredValue = Value.kReverse;
		transferMotor1.set(TRANSFER_RUN_POWER);
		transferMotor2.set(TRANSFER_RUN_POWER);
		shooterMotor.set(0);
		interMotor.set(0);
	}

	private void handleRetractedNoBallState(TeleopInput input) {
		intakeMotor.set(0);
		preferredValue = Value.kReverse;
		transferMotor1.set(0);
		transferMotor2.set(0);
		shooterMotor.set(0);
		interMotor.set(0);
	}

	private void handlePrepMotorState(TeleopInput input) {
		intakeMotor.set(0);
		preferredValue = Value.kReverse;
		transferMotor1.set(0);
		transferMotor2.set(0);
		shooterMotor.set(limeLight.getShootingPower());
		interMotor.set(0);
		// change shoot power to be cv power
	}

	private void handleShoot(TeleopInput input) {
		intakeMotor.set(0);
		transferMotor1.set(TRANSFER_RUN_POWER);
		transferMotor2.set(TRANSFER_RUN_POWER);
		preferredValue = Value.kReverse;
		interMotor.set(INTER_RUN_POWER);
		shooterMotor.set(limeLight.getShootingPower());
	}

	private void updateDashboard(TeleopInput input) {
		// System.out.println(currentState);
		// System.out.println(color.getProximity());
		SmartDashboard.putNumber("Current Proximity", color.getProximity());
		if (input == null) {
			SmartDashboard.putBoolean("Button Pressed", false);
		} else {
			SmartDashboard.putBoolean("Button Pressed", input.isIntakeButtonPressed());
		}
		SmartDashboard.putNumber("Motor Power", intakeMotor.get());
		SmartDashboard.putString("Solenoid Extended", armSolenoid.get().toString());
		SmartDashboard.putString("Current State", currentState + "");
		SmartDashboard.updateValues();
	}

}
