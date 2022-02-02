/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.operatorInterface.*;

public class RunIntake extends CommandBase {
  
  private final Intake myIntake;
  private final DriverControl driverCont;
  private final OperatorControl operatorCont;

  public boolean isIntakeOut;

  public RunIntake() {
    driverCont = DriverControl.getInstance();
    operatorCont = OperatorControl.getInstance();
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

    if(driverCont.getAButtonPressed() || operatorCont.getYButtonPressed()) {
      if(isIntakeOut){
        myIntake.intakeIn();
        isIntakeOut = false;
      } else {
        myIntake.intakeOut();
        isIntakeOut = true;
      }
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
