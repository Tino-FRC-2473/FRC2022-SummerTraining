package frc.robot.systems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class Teleop {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		TELEOP_STATE,
		TURN_STATE,
	}

	//private static final float MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */
	private static final double THRESHOLD = 5;
	private static final double ANGLE = 180;
	private static final double POW = 0.3;
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax leftMotor;
	private CANSparkMax rightMotor;
	private AHRS gyro;

	/* ======================== Constructor ======================== */

	public Teleop() {
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_LEFT,
										CANSparkMax.MotorType.kBrushless);
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_RIGHT,
										CANSparkMax.MotorType.kBrushless);
		gyro = new AHRS(SPI.Port.kMXP);
		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */

	public FSMState getCurrentState() {
		return currentState;
	}

	public void reset() {
		currentState = FSMState.TELEOP_STATE;
		gyro.reset();
		gyro.calibrate();
		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	public void update(TeleopInput input) {
		System.out.println(currentState + ": " + gyro.getAngle());
		switch (currentState) {
			case TELEOP_STATE:
				handleTeleopState(input);
				break;
			case TURN_STATE:
				handleTurnState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
	}

	/* ======================== Private methods ======================== */

	private FSMState nextState(TeleopInput input) {
		switch (currentState) {
			case TELEOP_STATE:
				if (input != null && input.isShooterButtonPressed()) {
                    return FSMState.TURN_STATE;
                } else {
                    return FSMState.TELEOP_STATE;
                }
			case TURN_STATE:
                if (input != null && gyro.getAngle() >= ANGLE - THRESHOLD && gyro.getAngle() <= ANGLE + THRESHOLD) {
                    gyro.reset();
					gyro.calibrate();
					return FSMState.TELEOP_STATE;
                } else {
                    return FSMState.TURN_STATE;
                }
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	private void handleTurnState(TeleopInput input) {
		System.out.println("TURN_STATE:" + gyro.getAngle());
		leftMotor.set(-POW);
		rightMotor.set(-POW);
	}

	private void handleTeleopState(TeleopInput input) {
		if (input == null) {
			return;
		}
		System.out.println("TELEOP_STATE");
		leftMotor.set(input.getLeftJoystickY());
		rightMotor.set(-input.getRightJoystickY());
	}
}
