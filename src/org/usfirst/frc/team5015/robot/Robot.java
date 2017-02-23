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
		
        drive_encoder.setDistancePerPulse(Math.PI*6.0/250.0);
	}

	boolean next_step = false;
	int step_number = 0;
	Timer auto_timer = new Timer();
	
	@Override
	public void autonomousInit() {
		next_step = false;
		step_number = 0;
		auto_timer.reset();
		auto_timer.start();
		drive_gyro.reset();
		drive_system.distanceEncoder.reset();
		SmartDashboard.putBoolean("Finished", false);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putNumber("Encoder", drive_system.distanceEncoder.getDistance());
		if(next_step){
			step_number += 1;
			drive_system.resetControllers();
			next_step = false;
			drive_gyro.reset();
		}

		//Left peg auto
				/*switch(step_number)
				{
				case 0:
		            next_step = drive_system.gyroDistanceDrive(133.0, 0.45);	
				break;
				
				
				case 1:
				    next_step = drive_system.gyroTurn(55, 0.4);
		            auto_timer.reset();
				break;
				
				case 2:
					next_step = true;
					drive_system.stopDrive();
				break;
				
				case 3:
				    drive_system.gyroDrive(0.5);
				    if(auto_timer.get() >= 1.5){
				    	next_step = true;
				    }
				break;
				 
				default:
					SmartDashboard.putBoolean("Finished", true);
					drive_system.stopDrive();
				break;
				}*/
				
		//Right peg auto
		switch(step_number)
		{
		case 0:
            next_step = drive_system.gyroDistanceDrive(129.0, 0.45);	
		break;
		
		
		case 1:
		    next_step = drive_system.gyroTurn(-45.0, 0.4);
            auto_timer.reset();
		break;
		
		case 2:
			next_step = true;
			drive_system.stopDrive();
		break;
		
		case 3:
		    drive_system.gyroDrive(0.5);
		    if(auto_timer.get() >= 0.7){
		    	next_step = true;
		    }
		break;
		 
		default:
			SmartDashboard.putBoolean("Finished", true);
			drive_system.stopDrive();
		break;
		}
		
		//Middle peg auto
		/*switch(step_number)
		{
		case 0:
            next_step = drive_system.gyroDistanceDrive(111.0, 0.45);	
		break;
		 
		default:
			SmartDashboard.putBoolean("Finished", true);
			drive_system.stopDrive();
		break;
		}*/
	
		
	}


	Path path = Paths.get("/U/output.csv");
	public String log_data = "";
	private boolean was_pressed = false;
	private int line_count = 0;
	private double M = 0.40;
	//0.40 is approximately the correct percent power for the shooter.
	private double S=13400, G=0.0000045, Y=0.5, e, d=1, b=2*M-1;
	//The TBH algorithm implemented using https://www.chiefdelphi.com/forums/showpost.php?p=1539758&postcount=2
	
	private void initialize_variables() {
		Y=0.5;
		d=1;
		b=2*M-1;
	}
	
	public void teleopInit() {
		initialize_variables();
	}
	
	@Override
	public void teleopPeriodic() {
		if(drive_stick.getRawButton(8))
		{
			drive_system.setMaxSpeed(1.0);
		}
		else{
			drive_system.setMaxSpeed(0.75);
		}
		drive_system.setMaxSpeed(0.75);
		drive_system.controlDrive(drive_stick.getRawAxis(1), drive_stick.getRawAxis(2)+0.2);
		
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

