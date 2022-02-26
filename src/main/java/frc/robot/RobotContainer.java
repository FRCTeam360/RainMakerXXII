// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.OIConstants.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CANIds.*;

import frc.robot.commands.*;
import frc.robot.commands.autos.TestingGroup.Test;
import frc.robot.operatorInterface.DriverControl;
import frc.robot.operatorInterface.OperatorControl;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final DriverControl driverCont = DriverControl.getInstance();
  private final OperatorControl operatorCont = OperatorControl.getInstance();
  // The robot's subsystems and commands are defined here...

  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final Shooter shooter = Shooter.getInstance();
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final DriveTrain driveTrain = new DriveTrain();
  // public final Feeder feeder = Feeder.getInstance();
  // public final Intake intake = Intake.getInstance();
  // public final Limelight limelight = new Limelight();
  // public final Tower tower = Tower.getInstance(); 

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  // private final ShooterJoy shooterJoy = new ShooterJoy();
  // private final SetShoot setShoot = new SetShoot(limelight);
  // public final RunFeeder runFeeder = new RunFeeder();
  // public final RunIntake runIntake = new RunIntake();
  private final TankDrive tankDrive = new TankDrive(driveTrain);
  // private final ArcadeDrive arcadeDrive = new ArcadeDrive(driveTrain);
  // private final FieldOrientedDrive fieldOrientedDrive = new FieldOrientedDrive(driveTrain);
  // private final Characterize characterize = new Characterize(driveTrain);

  // public final Test test = new Test(driveTrain);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureDefaultCommands();
    configureButtonBindings();
  }

  //scheduler will run these commands when nothing else scheduled
  private void configureDefaultCommands() {
    // tower.setDefaultCommand(runFeeder);
    // feeder.setDefaultCommand(runFeeder);
    // intake.setDefaultCommand(runIntake);
    // shooter.setDefaultCommand(setShoot);
    driveTrain.setDefaultCommand(tankDrive);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */


  private void configureButtonBindings() {
    // new JoystickButton(driverCont, 7).whenPressed(fieldOrientedDrive);
    // new JoystickButton(driverCont, 4).whenPressed(tankDrive);
    // new JoystickButton(driverCont, 3).whenPressed(arcadeDrive);
    // new JoystickButton(operatorCont, 7).whenHeld(shooterJoy);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {

    // return test;

    // // Create a voltage constraint to ensure we don't accelerate too fast
    // var autoVoltageConstraint =
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(
    //             AutoConstants.ksVolts,
    //             AutoConstants.kvVoltSecondsPerMeter,
    //             AutoConstants.kaVoltSecondsSquaredPerMeter),
    //         DriveTrain.kDriveKinematics,
    //         10);

    // // Create config for trajectory
    // TrajectoryConfig config =
    //     new TrajectoryConfig(
    //             AutoConstants.kMaxSpeedMetersPerSecond,
    //             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(DriveTrain.kDriveKinematics)
    //         // Apply the voltage constraint
    //         .addConstraint(autoVoltageConstraint);

    // // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(3, 0, new Rotation2d(0)),
    //         // Pass config
    //         config);

    // RamseteCommand ramseteCommand =
    //     new RamseteCommand(
    //         exampleTrajectory,
    //         driveTrain::getPose,
    //         new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    //         new SimpleMotorFeedforward(
    //             AutoConstants.ksVolts,
    //             AutoConstants.kvVoltSecondsPerMeter,
    //             AutoConstants.kaVoltSecondsSquaredPerMeter),
    //         DriveTrain.kDriveKinematics,
    //         driveTrain::getWheelSpeeds,
    //         new PIDController(AutoConstants.kPDriveVel, 0, 0),
    //         new PIDController(AutoConstants.kPDriveVel, 0, 0),
    //         // RamseteCommand passes volts to the callback
    //         driveTrain::tankDriveVolts,
    //         driveTrain);

    // // Reset odometry to the starting pose of the trajectory.
    // // driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
  // }
}