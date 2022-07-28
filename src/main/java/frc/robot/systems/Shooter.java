package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Robot Imports
import frc.robot.TeleopInput;

public class Shooter {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		AUTO,
		TELEOP
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
    private Compressor pcmCompressor;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public Shooter() {
		// Perform hardware init
		//frontLeftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_LEFT,CANSparkMax.MotorType.kBrushless);
		//frontRightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_RIGHT,CANSparkMax.MotorType.kBrushless);
		//backLeftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_BACK_LEFT,CANSparkMax.MotorType.kBrushless);
		//backRightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_BACK_RIGHT,CANSparkMax.MotorType.kBrushless);

        pcmCompressor = new Compressor(PneumaticsModuleType.REVPH);

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
		currentState = FSMState.AUTO;

        pcmCompressor.enableDigital();

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
			case AUTO:
				handleAuto(input);
				break;

			case TELEOP:
				handleTeleop(input);
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
			case AUTO:
				if (input != null){
					return FSMState.TELEOP;
				} else {
					return FSMState.AUTO;
				}

			case TELEOP:
				if (input != null){
					return FSMState.TELEOP;
				} else {
					return FSMState.AUTO;
				}

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in AUTO.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleAuto(TeleopInput input) {
		if (input != null) return;

		
	}
	/**
	 * Handle behavior in TELEOP.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTeleop(TeleopInput input) {
		if (input == null) return;

        boolean enabled = pcmCompressor.enabled();
        boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
        double current = pcmCompressor.getCurrent();

		SmartDashboard.putBoolean("Enabled", enabled);
		SmartDashboard.putBoolean("Pressure Switch", pressureSwitch);
        SmartDashboard.putNumber("Current", current);

	}
}
