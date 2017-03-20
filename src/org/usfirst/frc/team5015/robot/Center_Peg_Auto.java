package org.usfirst.frc.team5015.robot;

import edu.wpi.first.wpilibj.Timer;

public class Center_Peg_Auto extends Auto_Mode {

	public Center_Peg_Auto(String name, SWATDrive drive, Timer timer) {
		super(name, drive, timer);
	}
	
	@Override
	public void init_auto(){
		next_step = false;
		step_number = 0;
		auto_timer.reset();
		auto_timer.start();
		drive_system.driveGyro.reset();
		drive_system.distanceEncoder.reset();
	}
	
	@Override
	public boolean periodic_auto(){
		if(next_step){
			step_number += 1;
			drive_system.resetControllers();
			next_step = false;
			drive_system.driveGyro.reset();
		}
		//Middle peg auto
			switch(step_number)
				{
				case 0:
		            next_step = drive_system.gyroDistanceDrive(111.0, 0.45);
		            if(auto_timer.get() > 7.5){
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
