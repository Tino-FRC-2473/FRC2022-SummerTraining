package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.Joystick;

/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public class TeleopInput {
	/* ======================== Constants ======================== */
	private static final int LEFT_JOYSTICK_PORT = 0;
	private static final int RIGHT_JOYSTICK_PORT = 1;

	/* ======================== Private variables ======================== */
	// Input objects
	private Joystick leftJoystick;
	private Joystick rightJoystick;

	private static final int SHOOTER_BUTTON_ID = 1;
	private static final int INTAKE_BUTTON_ID = 4;
	private static final int CLIMBER_BUTTON_ID = 5;
	private static final int EJECT_BUTTON_ID = 6;
	private static final int EXTENDED_BUTTON_ID = 7;
	private static final int RETRACTED_BUTTON_ID = 8;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);

		rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	/* ------------------------ Left Joystick ------------------------ */
	/**
	 * Get X axis of Left Joystick.
	 * @return Axis value
	 */
	public double getLeftJoystickX() {
		return leftJoystick.getX();
	}
	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getLeftJoystickY() {
		return leftJoystick.getY();
	}
	/**
	 * Get the value of the shooter button.
	 * @return True if button is pressed
	 */
	public boolean isShooterButtonPressed() {
		return leftJoystick.getRawButton(SHOOTER_BUTTON_ID);
	}
	/**
	 * Get the value of the intake button.
	 * @return True if button is pressed
	 */
	public boolean isIntakeButtonPressed() {
		return leftJoystick.getRawButton(INTAKE_BUTTON_ID);
	}

	/**
	 * Get the value of the intake button.
	 * @return True if button is released
	 */
	public boolean isIntakeButtonReleased() {
		return leftJoystick.getRawButtonReleased(INTAKE_BUTTON_ID);
	}

	/**
	 * Get the value of the climber button.
	 * @return True if button is pressed
	 */
	public boolean isClimberButtonPressed() {
		return leftJoystick.getRawButton(CLIMBER_BUTTON_ID);
	}

	/**
	 * NOTIFY WHEN MECHANISM IS EXTENDED.
	 * @return True if button is pressed
	 */
	public boolean maxExtended() {
		return leftJoystick.getRawButton(EXTENDED_BUTTON_ID);
	}

	/**
	 * NOTIFY WHEN MECHANISM IS RETRACTED.
	 * @return True if button is pressed
	 */
	public boolean minRetracted() {
		return leftJoystick.getRawButton(RETRACTED_BUTTON_ID);
	}

	/**
	 * Get the value of the eject button.
	 * @return True if button is pressed
	 */
	public boolean isEjectButtonPressed() {
		return leftJoystick.getRawButton(EJECT_BUTTON_ID);
	}

	/**
	 * Get the value of the climber button.
	 * @return True if button is released
	 */
	public boolean isClimberButtonReleased() {
		return leftJoystick.getRawButtonReleased(CLIMBER_BUTTON_ID);
	}


	/* ------------------------ Right Joystick ------------------------ */
	/**
	 * Get X axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickX() {
		return rightJoystick.getX();
	}
	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickY() {
		return rightJoystick.getY();
	}

	/* ======================== Private methods ======================== */

}
