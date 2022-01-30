/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.operatorInterface;

import static frc.robot.Constants.OIConstants.*;

/**
 * Add your docs here.
 */
public class OperatorControl extends XboxCont{

    private static OperatorControl instance;

    private OperatorControl(){
      super(operatorContPort);
    }

    /**
     * gets instance for the singleton
     * @return instance
     */
    public static OperatorControl getInstance(){
        if(instance == null){
            instance = new OperatorControl();
        }
        return instance;
    }
}
