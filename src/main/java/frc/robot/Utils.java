// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/** Add your docs here. */

public class Utils {

    public static PneumaticsModuleType getPneumaticsType() {
        if (Constants.robotType == RobotType.COMP) {
            return PneumaticsModuleType.CTREPCM;
        }
        return PneumaticsModuleType.REVPH;
    }
}
