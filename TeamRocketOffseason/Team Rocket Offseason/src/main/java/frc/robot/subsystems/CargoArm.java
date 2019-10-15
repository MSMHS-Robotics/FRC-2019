/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class CargoArm extends PIDSubsystem {

  public CargoArm(){
    super("CargoArm",1.0,0.0,0.0);
  }

  public void initDefaultCommand() {
  }


  public void pickup(){

  }

  public void score(){

  }

  public void rest(){
    
  }

  @Override
  protected double returnPIDInput() {
    return 0;
  }

  @Override
  protected void usePIDOutput(double output) {

  }
}

