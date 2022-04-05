// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.operatorInterface.OperatorControl;
import frc.robot.subsystems.Shooter;

public class ManualSetShoot extends CommandBase {
  private Shooter shooter;
  private OperatorControl operatorCont = OperatorControl.getInstance();

  /** Creates a new ManualSetShoot. */
  public ManualSetShoot() {
    shooter = Shooter.getInstance();
    addRequirements(shooter); 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(operatorCont.getRawButton(7)){
      if(operatorCont.getXButton()){
        shooter.setVelocity(-1000);
      } else {
        shooter.setVelocity(1500);
      }
    }else if(operatorCont.getLeftStickButton()){
      shooter.setVelocity(3625);
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
