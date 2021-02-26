/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.shooter;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class ChangePosition extends SubsystemBase {
  
  private Compressor airow = new Compressor(20);
  
  private Solenoid leftPiston = new Solenoid(compressorModule, leftPoseMoverPort);
  private Solenoid rightPiston = new Solenoid(compressorModule, rightPoseMoverPort);
  
  private boolean collecting = false;
  public boolean isSwapping = false;

  public ChangePosition(){
    airow.start();
  }

  public boolean isCollectingPose(){
    return collecting;
  }

/**
 * Switch form a shooting position to a collecting position and vice versa
 */
public void swapHeight() {
  if (collecting) {
    shootPose();
  } else {
    collectPose();
  }
  isSwapping = true;
}

public void collectPose() {
  leftPiston.set(true);
  rightPiston.set(true);

  collecting = true;
  System.out.println("Collecting Pose Set");
}

public void shootPose(){
  leftPiston.set(false);
  rightPiston.set(false);

  collecting = false;
  System.out.println("Shooting Pose Set");
}


/*
  public void flagSwitch() {
      if (collecting) {
          leftPiston.set(false);
          collecting = false;
      } else {
          leftPiston.set(true);
          collecting = true;
      }
  }

  public ChangePosition() {
      airow.start();

  }
*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
