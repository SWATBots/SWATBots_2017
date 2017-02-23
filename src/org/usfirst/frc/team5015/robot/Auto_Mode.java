package org.usfirst.frc.team5015.robot;

import edu.wpi.first.wpilibj.Timer;

public class Auto_Mode {
	private String auto_name;
	public SWATDrive drive_system;
	public Timer auto_timer;
	public boolean next_step = false;
	public int step_number = 0;
	
	public Auto_Mode(String name, SWATDrive drive, Timer timer){
		auto_name = name;
		drive_system = drive;
		auto_timer = timer;
	}
	
	public String get_name(){
		return auto_name;
	}
	
	public void init_auto(){
	}
	
	public boolean periodic_auto(){
		return true;
	}
}
