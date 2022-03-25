// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.TerminalRight;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoConfig;
import frc.robot.commands.MoveWithRamsete;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class T_R_2ball extends SequentialCommandGroup {
  
  DriveTrain driveTrain = DriveTrain.getInstance();

  public static final Trajectory phase1 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(1, 0, new Rotation2d(0)),
      AutoConfig.configRevLow);

  /** Creates a new T_R_2ball. */
  public T_R_2ball() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new MoveWithRamsete(
            phase1,
            driveTrain)
                .andThen(() -> driveTrain.tankDriveVolts(0, 0)));

    driveTrain.setAngleOffset(-88.5);
  }
}
