package org.usfirst.frc.team5015.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Encoder;

public class SWATDrive {

    Encoder distanceEncoder;
    RobotDrive driveTrain;
    AnalogGyro driveGyro;
    private double maxSpeed = 1.0;
    double lastDistanceError = 0.0;
    int counts = 0;

    public SWATDrive(RobotDrive drive, AnalogGyro Drive_Gyro, Encoder driveEncoder) {
        maxSpeed = 1.0;
        lastDistanceError = 0.0;
        counts = 0;

        distanceEncoder = driveEncoder;
        driveTrain = drive;
        driveTrain.setSafetyEnabled(false);
        driveGyro = Drive_Gyro;
    }

    public void controlDrive(double moveValue, double rotateValue) {

        driveTrain.arcadeDrive(moveValue * maxSpeed, -rotateValue * maxSpeed);
    }
    
    public void stopDrive()
    {
        this.controlDrive(0.0, 0.0);
    }

    public void setMaxSpeed(double maximumSpeed) {
        /**
         * Sets the maximum speed (0.0 - 1.0) for the drive.
         */
        maxSpeed = maximumSpeed;
    }

    public void gyroDrive(double speed) {
        /**
         * Drive forward using a gyro and a proportional controller to go
         * straight.
         */
        this.driveTrain.arcadeDrive(speed, 0.2 * driveGyro.getAngle());
    }

    double distanceError;
    public boolean gyroDistanceDrive(double distance, double speed) {
        /*causes the robot to drive forward a set distance (measured in
         * the units the encoders return), and it returns true if the
         * robot is at its target*/
        distanceError = distance - distanceEncoder.getDistance();
        this.gyroDrive(speed * (distance / Math.abs(distance)));


        if (Math.abs(distanceError) < 0.5) {
            return true;
        } else {
            return false;
        }
    }
    
    public boolean gyroDistanceDrive(double distance)
    {
        return this.gyroDistanceDrive(distance, 0.5);
    }
    
    double errorSum = 0.0;

    public void resetControllers() {
        /**
         * Reset all sensors and variables so that the robot can start another
         * maneuver.
         */
        distanceEncoder.reset();
        errorSum = 0.0;
        driveGyro.reset();
    }

    double turnError;
    boolean turnTargetReached;
    
    public boolean gyroTurn(double targetAngle, double speed) {
        /**
         * Turn targetAngle degrees (clockwise is positive and counter-clockwise
         * is negative) using a gyro sensor and a PI controller.
         */
        
         turnError = driveGyro.getAngle() - targetAngle;
        turnTargetReached = false;


        if (Math.abs(turnError) > 0.5) {
            this.controlDrive(0.0, -speed * (turnError/Math.abs(turnError)));
            turnTargetReached = false;
        } else {
            this.controlDrive(0.0, 0.0);
            turnTargetReached = true;
        }

        return turnTargetReached;
    }
    
    public boolean gyroTurn(double targetAngle)
    {
        return this.gyroTurn(targetAngle, 0.5);
    }
}
