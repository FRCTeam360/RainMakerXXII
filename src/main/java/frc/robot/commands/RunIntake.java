/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.operatorInterface.XboxCont;

public class RunIntake extends CommandBase {
  
  private final Intake myIntake;
  private final XboxCont driverCont;
  private final XboxCont operatorCont;

  public RunIntake() {
    driverCont = new XboxCont(driverContPort);
    operatorCont = new XboxCont(operatorContPort);
    myIntake = Intake.getInstance(); 
    addRequirements(myIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driverCont.getLeftTrigger() || operatorCont.getLeftBumper()){
      if (operatorCont.getXButton() || driverCont.getBButton() ) {
        myIntake.run(-1.0);
      } else {
        myIntake.run(1.0); 
      }
    } else {
      myIntake.run(0.0); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
