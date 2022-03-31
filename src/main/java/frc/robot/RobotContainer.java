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

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OIConstants.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CANIds.*;

import frc.robot.commands.*;
import frc.robot.commands.autos.TestingGroup.Test;
import frc.robot.operatorInterface.DriverControl;
import frc.robot.operatorInterface.OperatorControl;
import frc.robot.subsystems.*;
import frc.robot.operatorInterface.OperatorControl;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final DriverControl driverCont = DriverControl.getInstance();
  private final OperatorControl operatorCont = OperatorControl.getInstance();
  // The robot's subsystems and commands are defined here...

  private final Turret turret = Turret.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final DriveTrain driveTrain = DriveTrain.getInstance();
  private final Feeder feeder = Feeder.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Limelight limelight = Limelight.getInstance();
  private final Tower tower = Tower.getInstance();
  private final Pneumatics pneumatics = new Pneumatics();
  private final Climber climber = new Climber();

  private final ShooterTesting shooterTesting = new ShooterTesting();
  private final ShooterJoy shooterJoy = new ShooterJoy();
  private final SetShoot setShoot = new SetShoot(limelight);
  public final RunFeeder runFeeder = new RunFeeder();
  public final RunIntake runIntake = new RunIntake();
  private final TankDrive tankDrive = new TankDrive();
  private final ArcadeDrive arcadeDrive = new ArcadeDrive();
  private final FieldOrientedDrive fieldOrientedDrive = new FieldOrientedDrive();
  private final RunClimberManual runClimberManual = new RunClimberManual(climber);
  private final TurretAuto turretAuto = new TurretAuto(limelight, turret);
  private final TurretManual turretManual = new TurretManual();
  private final Pressurize pressurize = new Pressurize(pneumatics);
  private final RunFeederManually runFeederManually = new RunFeederManually();
  private final RunTowerManually runTowerManually = new RunTowerManually();

  private final QueueBalls queueCargo = new QueueBalls(false);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureDefaultCommands();
    configureButtonBindings();
  }

  // scheduler will run these commands when nothing else scheduled
  private void configureDefaultCommands() {
    // tower.setDefaultCommand(queueCargo);
    // feeder.setDefaultCommand(queueCargo);
    intake.setDefaultCommand(runIntake);
    shooter.setDefaultCommand(setShoot);
    driveTrain.setDefaultCommand(fieldOrientedDrive);
    pneumatics.setDefaultCommand(pressurize);
    turret.setDefaultCommand(turretManual);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    new JoystickButton(driverCont, 2).whenPressed(fieldOrientedDrive);
    new JoystickButton(driverCont, 8).whenPressed(tankDrive);
    new JoystickButton(driverCont, 4).whenPressed(arcadeDrive);
    new JoystickButton(operatorCont, 7).whenHeld(shooterJoy);
    new JoystickButton(operatorCont, 8).whenHeld(turretAuto);
    new JoystickButton(operatorCont, 10).whileHeld(runClimberManual);
    new JoystickButton(driverCont, 5).whileHeld(runFeederManually);
    new JoystickButton(operatorCont, 5).whileHeld(runFeederManually);
    new JoystickButton(driverCont, 6).whileHeld(runTowerManually);
    new JoystickButton(operatorCont, 6).whileHeld(runTowerManually);
    new JoystickButton(operatorCont, 4).whileHeld(queueCargo);
  }

  public DriveTrain getDriveTrain() {
    return driveTrain;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}