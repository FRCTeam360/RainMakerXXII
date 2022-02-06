// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.OIConstants.*;
import frc.robot.subsystems.Turret;
import frc.robot.operatorInterface.OperatorControl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class TurretManual extends CommandBase {
  private OperatorControl operatorCont;
  private Turret myTurret;

  /** Creates a new TurretManual. */
  public TurretManual() {

    myTurret = Turret.getInstance();
    operatorCont = OperatorControl.getInstance();
    addRequirements(myTurret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(operatorCont.getRightX()) > .125) {
      myTurret.turn(operatorCont.getRightX());
    } else {
      myTurret.turn(0);
  
    }

    double encoderTick = myTurret.getEncoderTick();
    SmartDashboard.putNumber("gettick", encoderTick);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myTurret.turn(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
