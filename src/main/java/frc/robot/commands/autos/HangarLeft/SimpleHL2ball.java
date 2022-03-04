// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.HangarLeft;

import frc.robot.commands.autos.Anywhere.Anywhere2ball;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class SimpleHL2ball extends Anywhere2ball{

    public SimpleHL2ball(DriveTrain driveTrain){
        super(driveTrain, 43.5); //offset to be determined
    }
}
