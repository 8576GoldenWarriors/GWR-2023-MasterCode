// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class PID_Drive extends CommandBase {
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

  public PID_Drive(DrivetrainSubsystem drivetrainSubsystem, double s) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
    setpoint = s;
    Kp = 0.05;
    Ki = 0; 
    Kd = 0;
    error = 0;
    lastError = 0;
    integral = 0;
    derivative = 0;
    avgPos = DrivetrainSubsystem.getAverageEncoderDistance();
    motorPower = 0;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    avgPos = DrivetrainSubsystem.getAverageEncoderDistance();
    error = setpoint - avgPos;
    integral = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(error) > (Math.abs(setpoint) / 2)){

      error = Math.abs(setpoint) - Math.abs(DrivetrainSubsystem.getAverageEncoderDistance());
      integral = integral + error;
      derivative = error - lastError;
      motorPower = (Kp * error) + (Ki * integral) + (Kd * derivative);
      
      // the value set here should be adjusted based on the weight of the robot
      if (motorPower > .50){
        motorPower = .50;
      }
      
      if (setpoint < 0){
        motorPower = -motorPower;
      }

      System.out.println(error);
      
      DrivetrainSubsystem.arcadeDrive((-motorPower), 0);

    }
    else{
      end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    DrivetrainSubsystem.TankDrive(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
