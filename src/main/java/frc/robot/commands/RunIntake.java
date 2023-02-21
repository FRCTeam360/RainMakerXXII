// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.operatorInterface.DriverControl;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntake extends CommandBase {
  private Intake motor;
  public final DriverControl drive;
  /** Creates a new runIntake. */
  public RunIntake() {  
    motor = Intake.getInstance();
    drive = DriverControl.getInstance();
    addRequirements(motor);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (drive.getLeftBumper()){
      motor.runIntake(1.0);
      if(drive.getXButton()){
      motor.runIntake(-1.0);
      }
    } else {
      motor.stopIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
