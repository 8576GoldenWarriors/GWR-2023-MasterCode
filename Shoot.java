// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
  /** Creates a new Shoot. */
  public double speed;
  public Shoot(Shooter shoot, double s) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoot);
    speed = s;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.count++;
    //System.out.println(Robot.count);
    if (Robot.count == 2){
      Shooter.shooter(speed);
      Kicker.kicker(0);
      }
      if (Robot.count == 1){
        Shooter.shooter(speed);
        Kicker.kicker(-0.4);
      }
      if (Robot.count == 3){
        Robot.count = 0;
        end(true);
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if (!Robot.input.get()){
      System.out.println("Detected");
    //Shooter.shooter(0.3);
    //Kicker.kicker(0.3);
    }
    System.out.println("Not Detected");*/
    //if(RobotContainer.joystick.getRawButtonPressed(2)){
      // if (!Robot.input.get()){
      // //if (Robot.count == 1){
      //   Shooter.shooter(speed);
      // //}
      // if (Robot.count == 2){
      //   //Shooter.shooter(speed);
      //   Kicker.kicker(-0.4);
      // }
      // if (Robot.count == 3){
      //   Robot.count = 0;
      //   end(true);
      // }
      // }
      // System.out.print("Not Detected");
    //}
    // if (RobotContainer.joystick.getRawButtonPressed(3)){
    //   Shooter.shooter(0);
    //   Kicker.kicker(0);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Robot.count = 0;
    Shooter.shooter(0);
    Kicker.kicker(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
