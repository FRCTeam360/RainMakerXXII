// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.operatorInterface.DriverControl;
import frc.robot.operatorInterface.OperatorControl;
import frc.robot.subsystems.Feeder;

public class RunFeederManually extends CommandBase {
  private final Feeder myFeeder;
  private final OperatorControl operatorCont;
  private final DriverControl driverCont;
  /** Creates a new RunFeederManually. */
  public RunFeederManually() {
    myFeeder = Feeder.getInstance();
    operatorCont = OperatorControl.getInstance();
    driverCont = DriverControl.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (operatorCont.getXButton() || driverCont.getXButton()) {
      myFeeder.runFeeder(-1.0);
    } else {
      myFeeder.runFeeder(1.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myFeeder.runFeeder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
