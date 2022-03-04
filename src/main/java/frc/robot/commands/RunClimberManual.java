// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.operatorInterface.OperatorControl;
import frc.robot.subsystems.Climber;

public class RunClimberManual extends CommandBase {

  private final Climber myClimber;
  private final OperatorControl operatorCont = OperatorControl.getInstance();
  /** Creates a new RunClimber. */
  public RunClimberManual(Climber climber) {
    myClimber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(operatorCont.getLeftY()) >= 0.125) {
      myClimber.runLeftClimber(operatorCont.getLeftY());
    } else {
      myClimber.runLeftClimber(0);
    }
    if(Math.abs(operatorCont.getRightY()) >= 0.125){
      myClimber.runRightClimber(operatorCont.getRightY());
    } else {
      myClimber.runRightClimber(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myClimber.runLeftClimber(0);
    myClimber.runRightClimber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
