package org.usfirst.frc.team5015.robot;

import java.io.FileOutputStream;
import java.io.OutputStream;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	AnalogGyro drive_gyro = new AnalogGyro(0);
	Encoder drive_encoder = new Encoder(0, 1);
	Talon left_drive = new Talon(0);
	Talon right_drive = new Talon(1);
	RobotDrive drivetrain = new RobotDrive(left_drive, right_drive);
	SWATDrive drive_system = new SWATDrive(drivetrain, drive_gyro, drive_encoder);
	
	Joystick drive_stick = new Joystick(0);
	Joystick shooter_stick = new Joystick(1);
	
	Spark climbing_motor = new Spark(2);
	
	CANTalon shooter_motor = new CANTalon(1);
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		CameraServer.getInstance().startAutomaticCapture();
		drive_gyro.calibrate();
		shooter_motor.changeControlMode(TalonControlMode.PercentVbus);
		shooter_motor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		shooter_motor.enableBrakeMode(false);
		shooter_motor.set(0.0);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {

	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {

	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		drive_system.controlDrive(drive_stick.getRawAxis(1), drive_stick.getRawAxis(2));
		
		if(shooter_stick.getRawButton(8)){
			shooter_motor.set(-0.45);
		}
		else{
			shooter_motor.set(0.0);
		}
		
		if(shooter_stick.getRawButton(6)){
			climbing_motor.set(-1.0);
		}
		else{
			climbing_motor.set(0.0);
		}
		
		SmartDashboard.putNumber("Shooter RPM: ", shooter_motor.getSpeed());
		SmartDashboard.putNumber("Shooter Velocity: ", shooter_motor.getEncVelocity());
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}

