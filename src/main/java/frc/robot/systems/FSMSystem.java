package frc.robot.systems;

import com.kauailabs.navx.frc.AHRS;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class FSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		TELEOP_STATE
	}

	private static final float MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	private CvSink cvSink;
	private CvSource outputStream;
	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax leftMotor;
	private AHRS gyro;
	private AnalogInput analogInput;
	private AnalogPotentiometer potentiometer;
	private DigitalInput limitSwitch;
	private DigitalInput distanceSwitch;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public FSMSystem() {
		// Perform hardware init
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_BACK_RIGHT,
										CANSparkMax.MotorType.kBrushless);
		// Creates UsbCamera and MjpegServer [1] and connects them
		CameraServer.startAutomaticCapture();
		// Creates the CvSink and connects it to the UsbCameras
		cvSink = CameraServer.getVideo();
		// Creates the CvSource and MjpegServer [2] and connects them
		outputStream = CameraServer.putVideo("RobotFrontCamera", 640, 480);
		// Creates the gyro
		gyro = new AHRS(SPI.Port.kMXP);
		// Creates the analogInput
		analogInput = new AnalogInput(0);
		analogInput.getAverageBits();
		// Creates the potentiometer
		potentiometer = new AnalogPotentiometer(analogInput);
		// Creates the limit switch
		limitSwitch = new DigitalInput(0);
		// Creates the distance switch
		distanceSwitch = new DigitalInput(1);
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
		currentState = FSMState.TELEOP_STATE;

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
			case TELEOP_STATE:
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
			case TELEOP_STATE:
				return FSMState.TELEOP_STATE;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTeleopState(TeleopInput input) {
		if(input != null) {
			SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
			SmartDashboard.putNumber("Encoder Ticks", leftMotor.getEncoder().getPosition());
			SmartDashboard.putNumber("Potentiometer", potentiometer.get());
			SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
			SmartDashboard.putBoolean("Distance Switch", distanceSwitch.get());
		}
	}
}
