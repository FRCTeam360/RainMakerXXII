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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CANIds.*;

import frc.robot.commands.*;
// import frc.robot.commands.autos.TestingGroup.Test;
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

  private final DriverControl DriverCont = DriverControl.getInstance();
  private final OperatorControl OperatorCont = OperatorControl.getInstance();
  // The robot's subsystems and commands are defined here...

  private final Shooter shooter = Shooter.getInstance();
  private final Feeder feeder = Feeder.getInstance();
  private final RunFeeder runFeeder = new RunFeeder();
  private final Intake intake = Intake.getInstance();
  private RunIntake runIntake = new RunIntake();
  private final Tower tower = Tower.getInstance();
  private RunTower runTower = new RunTower();
  private RunShooter runShooter = new RunShooter();

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
    feeder.setDefaultCommand(runFeeder);
    shooter.setDefaultCommand(runShooter);
    tower.setDefaultCommand(runTower);
    intake.setDefaultCommand(runIntake);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {

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