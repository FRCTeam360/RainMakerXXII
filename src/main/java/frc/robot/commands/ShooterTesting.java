// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.operatorInterface.DriverControl;
import frc.robot.operatorInterface.OperatorControl;
import frc.robot.subsystems.Shooter;

public class ShooterTesting extends CommandBase {
  private double testSetPoint = 0.0;

  private final Shooter shooter;
  /** Creates a new ShooterTesting. */
  public ShooterTesting() {
    shooter = Shooter.getInstance();

    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Shooter Setpoint", testSetPoint);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shootGoal = SmartDashboard.getNumber("Shooter Setpoint", 0.0);
    SmartDashboard.getNumber("Shooter Speed", shooter.getShooterVelocity());
    if(OperatorControl.getInstance().getRightTrigger()){
      shooter.setVelocity(shootGoal);
    } else {
      shooter.setVelocity(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
