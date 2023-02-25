// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.commands.DriveWithJoystickCommand;
import frc.robot.commands.Flush;
import frc.robot.commands.Intaker;
import frc.robot.commands.Kick;
import frc.robot.commands.Out;
import frc.robot.commands.PID_Drive;
import frc.robot.commands.PneumaticDown;
import frc.robot.commands.PneumaticLeftMiddle;
import frc.robot.commands.PneumaticRightMiddle;
import frc.robot.commands.PneumaticUp;
import frc.robot.commands.Reverse;
import frc.robot.commands.Shoot;
import frc.robot.commands.TimeShoot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();

  private final DriveWithJoystickCommand driveWithJoystickCommand = new DriveWithJoystickCommand(drivetrainSubsystem);

  private final Intake intake = new Intake();

  //private final Intaker intaker = new Intaker(intake);

  private final Shooter shooter = new Shooter();

  //private final Shoot shoot = new Shoot(shooter, -1);

  private final Pneumatics pneumatics = new Pneumatics();

  private final Kicker kicker = new Kicker();


  public static Joystick joystick = new Joystick(0);

  SendableChooser<Command> chooser = new SendableChooser<>();
  
  Trigger shootTrigger = new JoystickButton(joystick, 2); 
  Trigger intakeTrigger = new JoystickButton(joystick, 1);

  Trigger pneumaticDownTrigger = new JoystickButton(joystick, 4);
  Trigger pneumaticUpTrigger = new JoystickButton(joystick, 3);

  Trigger pneumaticMiddleTrigger = new JoystickButton(joystick, 5);

  Trigger flushTrigger = new JoystickButton(joystick, 6);

  Trigger reverseTrigger = new JoystickButton(joystick, 7);
  Trigger outTrigger = new JoystickButton(joystick, 8);

  Trigger shoot = new JoystickButton(joystick, 9); 
  Trigger off = new JoystickButton(joystick, 10); 


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drivetrainSubsystem.setDefaultCommand(driveWithJoystickCommand);
    //shooter.setDefaultCommand(shoot);
    //intake.setDefaultCommand(intaker);

    // chooser.addOption("Middle", loadPathplannerTrajectoryToRamseteCommand(
    //     "deploy/pathplanner/generatedJSON/Middle.wpilib.json",
    //     true));
    // chooser.addOption("straight", loadPathplannerTrajectoryToRamseteCommand(
    //     "deploy/pathplanner/generatedJSON/New Path.wpilib.json",
    //     true));

    Shuffleboard.getTab("Autonomous").add(chooser);

  }

  public Command loadPathplannerTrajectoryToRamseteCommand(String filename, boolean resetOdomtry) {
    //Trajectory trajectory;
    //PathPlannerTrajectory examplePath;

    //try {
      //filename = "deploy/pathplanner/generatedJSON/New Path.wpilib.json";
      //Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      PathPlannerTrajectory examplePath = PathPlanner.loadPath("deploy/pathplanner/Middle.path", new PathConstraints(4, 3));

      //PathPlannerState exampleState = (PathPlannerState) examplePath.sample(1.2);
      //System.out.println(trajectoryPath);
      //trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      //System.out.println("YYYYYYYYYYYYYYYYYYYYYY");
    // } catch (IOException exception) {
    //   DriverStation.reportError("Unable to open trajectory" + filename, exception.getStackTrace());
    //   //System.out.println("XXXXXXXXXXXXXXXXXXXXXX");
    //   return new InstantCommand();
    // }
    // Tries to get some information from a file - J

    PPRamseteCommand ramseteCommand = new PPRamseteCommand(examplePath, drivetrainSubsystem::getPose,
        new RamseteController(DriveTrainConstants.kRamseteB, DriveTrainConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveTrainConstants.ksVolts, DriveTrainConstants.kvVoltSecondsPerMeter,
            DriveTrainConstants.kaVoltSecondsSquaredPerMeter),
        DriveTrainConstants.kDriveKinematics, drivetrainSubsystem::getWheelSpeeds,
        new PIDController(DriveTrainConstants.kPDriveVel, 0, 0),
        new PIDController(DriveTrainConstants.kPDriveVel, 0, 0), drivetrainSubsystem::tankDriveVolts,
        drivetrainSubsystem);
        // I assume something for getting the robot to go in the right direction with PID - J

    if (resetOdomtry) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> drivetrainSubsystem.resetOdometry(examplePath.getInitialPose())), ramseteCommand);
    } else {
      return ramseteCommand;
    }

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //3 button press system for shooter
    //to-do: add new commands to create three different speeds
    //talk to driver for button configurations on shooter speed
    shootTrigger.toggleOnTrue(new Shoot(shooter, -1));
    shootTrigger.toggleOnFalse(new Shoot(shooter, -1));

    //shooter constantly engaged
    //Shooter.shooter(-0.5);

    shootTrigger.whileTrue(new Kick(kicker, -0.4));

    intakeTrigger.whileTrue(new Intaker(intake));

    pneumaticDownTrigger.onTrue(new PneumaticDown(pneumatics));
    pneumaticUpTrigger.onTrue(new PneumaticUp(pneumatics));

    pneumaticMiddleTrigger.onTrue(Commands.parallel(new PneumaticLeftMiddle(), new PneumaticRightMiddle()));

    // sets every subsystem in reverse
    flushTrigger.whileTrue(new Flush(-0.7));

    // sets the shooter to reverse
    reverseTrigger.onTrue(new Reverse(shooter, -0.5));

    // reverses kicker and intake
    outTrigger.whileTrue(new Out(intake, -0.5));

    shoot.onTrue(new TimeShoot(shooter, -0.45));
    //low score: (-.2, -2.5)
    
    off.onTrue(new TimeShoot(shooter, 0));
    
  }

  /**co
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    /*return loadPathplannerTrajectoryToRamseteCommand(
           "deploy/pathplanner/generatedJSON/Middle.wpilib.json",
      true); //chooser.getSelected();*/

      return new SequentialCommandGroup(new PID_Drive(drivetrainSubsystem, -10), new PID_Drive(drivetrainSubsystem, 10));
  }

  public DrivetrainSubsystem getDriveTrainSubsystem() {
    return drivetrainSubsystem;
  }

}