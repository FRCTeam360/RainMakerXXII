// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;

public class RetractRunIntake extends SequentialCommandGroup {
  
  /** Creates a new RetractRunIntake. */
  public RetractRunIntake() {
    // Use addRequirements() here to declare subsystem dependencies.

    addCommands(
      new AutoRetractIntake(),
      new AutoRunIntake()
    );
  }

  public RetractRunIntake(double time){
    addCommands(
      new AutoRetractIntake(),
      
      new ParallelRaceGroup(
        new AutoRunIntake(),
        new WaitCommand(time)
      )
    );
  }
}
