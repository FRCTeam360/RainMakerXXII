/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.operatorInterface;

import edu.wpi.first.wpilibj.XboxController;
import static frc.robot.Constants.OIConstants.*;

/**
 * Add your docs here.
 */
public class operatorControl extends XboxCont{

    private static operatorControl instance;

    private operatorControl(){
      super(operatorContPort);
    }

    /**
     * gets instance for the singleton
     * @return instance
     */
    public static operatorControl getInstance(){
        if(instance == null){
            instance = new operatorControl();
        }
        return instance;
    }
}

