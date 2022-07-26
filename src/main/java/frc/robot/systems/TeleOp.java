package frc.robot.systems;
import com.kauailabs.navx.frc.AHRS;
// WPILib Imports
// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
// import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Robot Imports
import frc.robot.TeleopInput;

public class TeleOp {

	static final double ACCELERATION_CONSTANT = 0.5;
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		TELEOP
	}

	private static final float MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */
	private FSMState currentState;
	private double motorSpeed = 0.2;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	private CANSparkMax motor;
	private AHRS gyro;
	private AnalogPotentiometer pot;
	private DigitalInput limitSwitch;

	private CvSink cvSink;
	private CvSource outputStream;
	private int cameraWidth = 640;
	private int cameraHeight = 480;

	private int potFullRange = 180;
	private int potOffset = 30;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public TeleOp() {
		// Perform hardware init
		motor = new CANSparkMax(2, CANSparkMax.MotorType.kBrushless);
		gyro = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);
		pot = new AnalogPotentiometer(0, potFullRange, potOffset);
		limitSwitch = new DigitalInput(0);

		CameraServer.startAutomaticCapture();
		CvSink cvSink = CameraServer.getVideo();
		CvSource outputStream = CameraServer.putVideo("RobotFrontCamera", cameraWidth, cameraHeight);

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
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = FSMState.TELEOP;
		gyro.reset();
		gyro.calibrate();
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
			case TELEOP:
				handleTeleOpState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
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
		return FSMState.TELEOP;
	}

	/* ------------------------ FSM state handlers ------------------------ */
	
	/**
	 * Handle behavior in OTHER_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTeleOpState(TeleopInput input) {
		if (input == null) {
			return;
		}

		motor.set(motorSpeed);

		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
		SmartDashboard.putNumber("Potentiometer Voltage", pot.get());
		SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
		SmartDashboard.putNumber("Encoder Ticks", motor.getEncoder().getPosition());
	}

}
