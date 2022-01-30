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
public class DriverControl extends XboxCont{
    
    private static DriverControl instance;

    private DriverControl(){
      super(driverContPort);
    }

    /**
     * gets instance for the singleton
     * @return instance
     */
    public static DriverControl getInstance(){
        if(instance == null){
            instance = new DriverControl();
        }
        return instance;
    }
}