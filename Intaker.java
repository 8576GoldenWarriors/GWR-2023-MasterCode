// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;

public class Intaker extends CommandBase {
  /** Creates a new Intaker. */

  // DigitalInput photoelectric = new DigitalInput(5);
  // AddressableLED m_led = new AddressableLED(9);
  // AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(20);

  public Intaker(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Intake.intake(-0.5);
    Kicker.kicker(-0.5);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(RobotContainer.joystick.getRawButtonPressed(1)){
    //   //if (!Robot.input.get()){
    //     Intake.intake(0.3);
    //     Kicker.kicker(-0.5);
    //   //}
    // }
    // // if(RobotContainer.joystick.getRawButtonPressed(7)){
    // //   Intake.intake(0.3);
    // //   Kicker.kicker(0.3);
    // // }
    // if (RobotContainer.joystick.getRawButtonPressed(5)){
    //   Intake.intake(0);
    //   Kicker.kicker(0);
    // }
    if (!Robot.input.get()){
      Kicker.kicker(0);
      //System.out.println(Robot.input.get());

      Robot.led.start();

    } else {

      Robot.led.stop();

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.intake(0);
    Kicker.kicker(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
