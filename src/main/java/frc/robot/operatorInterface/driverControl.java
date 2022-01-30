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
public class driverControl extends XboxController{
    
    private static driverControl instance;

    private driverControl(){
      super(driverContPort);
    }

    /**
     * gets instance for the singleton
     * @return instance
     */
    public static driverControl getInstance(){
        if(instance == null){
            instance = new driverControl();
        }
        return instance;
    }
}
