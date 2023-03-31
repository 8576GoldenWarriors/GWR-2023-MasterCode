// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class PID_DriveCurve extends CommandBase {
  /** Creates a new PID_Drive. */
  public double setpoint;
  public double Kp;
  public double Ki; 
  public double Kd;
  public double error;
  public double lastError;
  public double integral;
  public double derivative;
  public double avgPos;
  public double motorPower;

  public Boolean done = false;

  public DrivetrainSubsystem drive;

  public PID_DriveCurve(DrivetrainSubsystem drivetrainSubsystem, double s) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
    RobotContainer.getDriveTrainSubsystem().resetEncoders();
    setpoint = s;
    Kp = 0.07; //controls speed - DOES NOT EFFECT DISTANCE
    Ki = 0; 
    Kd = 0.01; //effects stopping smoothness
    error = 0;
    lastError = 0;
    integral = 0;
    derivative = 0;
    avgPos = RobotContainer.getDriveTrainSubsystem().getAverageEncoderDistance();
    motorPower = 0;
    //drivetrainSubsystem.zeroHeading();
    System.out.println("STARTING");

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    avgPos = RobotContainer.getDriveTrainSubsystem().getAverageEncoderDistance();
    error = setpoint - avgPos;
    integral = 0;
    RobotContainer.getDriveTrainSubsystem().resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(error) > (Math.abs(setpoint) / 2)){

      error = Math.abs(setpoint) - Math.abs(DrivetrainSubsystem.getAverageEncoderDistance());
      integral = integral + error;
      derivative = error - lastError;
      motorPower = (Kp * error) + (Ki * integral) + (Kd * derivative);
      
      if (motorPower > .65){
        motorPower = .65;
      }

      if (motorPower < .20){
        motorPower = 0;
      }
      
      if (setpoint < 0){
        motorPower = -motorPower;
      }

      if (motorPower == 0){
        done = true;
      }

      System.out.println(error);
      
      //DrivetrainSubsystem.arcadeDrive((-motorPower), 0);
      DrivetrainSubsystem.TankDrive((-motorPower - (motorPower/40)), (-motorPower));                                                                                                                                                                                                                                        

    }
    else{
      //System.out.println("Done");
      done = true;
      //end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    RobotContainer.getDriveTrainSubsystem().TankDrive(0, 0);
    //DrivetrainSubsystem.resetEncoders();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (done){
      return true;
    }
    return false;
  }
}
