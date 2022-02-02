// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.OIConstants;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;


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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Limelight limelight = new Limelight();
  private final Turret turret = new Turret();
  private final DriveTrain driveTrain = new DriveTrain();
  public final Feeder feeder = Feeder.getInstance();
  public final Intake intake = Intake.getInstance();

  private final AlignTurret align = new AlignTurret(limelight, turret);
  public final RunFeeder runFeeder = new RunFeeder();
  public final RunIntake runIntake = new RunIntake();
  private final TankDrive tankDrive = new TankDrive(driveTrain);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureDefaultCommands();
    // Configure the button bindings
    configureButtonBindings();

  }

  //scheduler will run these commands when nothing else scheduled
  private void configureDefaultCommands() {
    turret.setDefaultCommand(align);
    feeder.setDefaultCommand(runFeeder);
    intake.setDefaultCommand(runIntake);
    driveTrain.setDefaultCommand(tankDrive);
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
