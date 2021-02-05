/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;

import static frc.robot.Constants.*;

public class Flag extends SubsystemBase {
  /**
   * Creates a new Flag.
   */
  private Solenoid flag = new Solenoid(compressorModule, flagPort);

  private Compressor airow = new Compressor(0);
  
  private boolean flagUp = false;

  public void flagSwitch() {
      if (flagUp) {
          flag.set(false);
      } else {
          flag.set(true);
      }
  }

  public Flag() {
      airow.start();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
