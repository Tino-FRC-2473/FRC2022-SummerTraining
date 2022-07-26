package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.SPI;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.kauailabs.navx.frc.AHRS;

// Camera and Shuffleboard
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class Dashboard {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		AUTO,
		TELEOP
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;
	private static final int CAM_WIDTH = 640;
	private static final int CAM_HEIGHT = 640;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax motor;
	private DigitalInput lim;
	private AnalogPotentiometer pot;
	private AHRS gyro;
	private AnalogInput dist;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public Dashboard() {
		// Perform hardware init
		motor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER,
										CANSparkMax.MotorType.kBrushless);
		lim = new DigitalInput(HardwareMap.LIMIT_SWITCH_DIO);
		pot = new AnalogPotentiometer(HardwareMap.POTENTIOMETER);
		gyro = new AHRS(SPI.Port.kMXP);
		dist = new AnalogInput(HardwareMap.DISTANCE);

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
		currentState = FSMState.TELEOP;

		CameraServer.startAutomaticCapture();
		CvSink cvSink = CameraServer.getVideo();
		CvSource outputStream = CameraServer.putVideo("RobotFrontCamera", CAM_WIDTH, CAM_HEIGHT);

		cvSink.isValid();
		outputStream.isValid();

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
			case AUTO:
				handleAutoState(input);
				break;

			case TELEOP:
				handleTeleopState(input);
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
				if (input != null) {
					return FSMState.TELEOP;
				} else {
					return FSMState.AUTO;
				}

			case TELEOP:
				return FSMState.TELEOP;

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
	private void handleAutoState(TeleopInput input) {

	}

	/**
	 * Handle behavior in TELEOP.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTeleopState(TeleopInput input) {
		SmartDashboard.putNumber("Motor Encoder Value", motor.getEncoder().getPosition());
		SmartDashboard.putNumber("Potentiometer", pot.get());
		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
		SmartDashboard.putNumber("Distance", dist.getValue());
		SmartDashboard.putBoolean("Limit Switch", lim.get());
	}

}
