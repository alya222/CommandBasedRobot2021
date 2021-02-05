/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase { 
    private CANSparkMax LFrontWheel = new CANSparkMax(kLeftFrontPort, MotorType.kBrushless);
    private CANSparkMax RFrontWheel = new CANSparkMax(kRightFrontPort, MotorType.kBrushless);
    
    private CANSparkMax LRearWheel = new CANSparkMax(kLeftRearPort, MotorType.kBrushless);
    private CANSparkMax RRearWheel = new CANSparkMax(kRightRearPort, MotorType.kBrushless);

    private DifferentialDrive roboDrive = new DifferentialDrive(LFrontWheel, RFrontWheel); 

  public Drivetrain() { 
      LRearWheel.follow(LFrontWheel); 
      RRearWheel.follow(RFrontWheel);

  }

  public DifferentialDrive getDifferentialDrive () { 
      return roboDrive;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
