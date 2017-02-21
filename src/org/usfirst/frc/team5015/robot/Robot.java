package org.usfirst.frc.team5015.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.*;
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
	/**   b
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		try{
		CameraServer.getInstance().startAutomaticCapture();
		}
		catch(Exception ex){}
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


	Path path = Paths.get("/U/output.csv");
	public String log_data = "";
	private boolean was_pressed = false;
	private int line_count = 0;
	private double S=13400, G=0.005, Y=1, e, d=1, b=2*0.40-1;
	//0.40 is approximately the correct percent power for the shooter.
	//The TBH algorithm implemented using https://www.chiefdelphi.com/forums/showpost.php?p=1539758&postcount=2
	
	private void initialize_variables() {
		Y=1;
		d=1;
		b=2*M-1;
	}
	
	public void teleopInit() {
		initialize_variables();
	}
	
	@Override
	public void teleopPeriodic() {
		drive_system.controlDrive(drive_stick.getRawAxis(1), drive_stick.getRawAxis(2));
		
		if(shooter_stick.getRawButton(8)){
			/*shooter_motor.set(-0.40);*/
			e = S - shooter_motor.getEncVelocity();
			Y += G*e;
			if(Y>1) {
				Y = 1;
			}
			else if (Y<0) {
				Y = 0;
			}
			if (Math.signum(e) != Math.signum(d)){
				Y = b = 0.5*(Y+b);
				d = e;
			}
			shooter_motor.set(-Y);
			
			was_pressed = true;
			log_data += Integer.toString(shooter_motor.getEncVelocity())+",";
			System.out.print(Integer.toString(shooter_motor.getEncVelocity())+",");
		}
		else{
			if(was_pressed){
				initialize_variables();
				System.out.println("");
				line_count++;
				log_data += "\n\n";
				was_pressed = false;
			}
			shooter_motor.set(0.0);
		}
		
		if(shooter_stick.getRawButton(6)){
			climbing_motor.set(-1.0);
		}
		else{
			climbing_motor.set(0.0);
		}
		
		SmartDashboard.putNumber("Log line count", line_count);
		SmartDashboard.putNumber("Shooter Velocity: ", shooter_motor.getEncVelocity());
	}
	

	public void disabledInit()
	{
		System.out.println("\n\n");
		System.out.println(log_data);
		try {
			Files.write(path, log_data.getBytes());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}

