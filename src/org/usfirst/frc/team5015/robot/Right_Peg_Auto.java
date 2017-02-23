package org.usfirst.frc.team5015.robot;

import edu.wpi.first.wpilibj.Timer;

public class Right_Peg_Auto extends Auto_Mode {

	public Right_Peg_Auto(String name, SWATDrive drive, Timer timer) {
		super(name, drive, timer);
	}
	
	public void init_auto(){
		next_step = false;
		step_number = 0;
		auto_timer.reset();
		auto_timer.start();
		drive_system.driveGyro.reset();
		drive_system.distanceEncoder.reset();
	}
	
	public boolean periodic_auto(){
		if(next_step){
			step_number += 1;
			drive_system.resetControllers();
			next_step = false;
			drive_system.driveGyro.reset();
		}
		
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
					drive_system.stopDrive();
					return true;
				}
				return false;

	}

}
