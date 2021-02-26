/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drive;


//import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import static frc.robot.Gains.*;

import static frc.robot.Constants.*;

public class RevDrivetrain extends SubsystemBase { 
    private CANSparkMax LFrontWheel = new CANSparkMax(kLeftFrontPort, MotorType.kBrushless);
    private CANSparkMax RFrontWheel = new CANSparkMax(kRightFrontPort, MotorType.kBrushless);
    
    private CANSparkMax LRearWheel = new CANSparkMax(kLeftRearPort, MotorType.kBrushless);
    private CANSparkMax RRearWheel = new CANSparkMax(kRightRearPort, MotorType.kBrushless);

    private DifferentialDrive roboDrive = new DifferentialDrive(LFrontWheel, RFrontWheel); 

    private SimpleMotorFeedforward feedforward
      = new SimpleMotorFeedforward(driveFeedforward.ks, driveFeedforward.kv, driveFeedforward.ka);

    private PIDController leftDrivePID
      = new PIDController(leftDrive.kP, leftDrive.kI, leftDrive.kD);

      private PIDController rightDrivePID
      = new PIDController(rightDrive.kP, rightDrive.kI, rightDrive.kD);

      private Pose2d pose = new Pose2d();

      private SlewRateLimiter speedlimiter = new SlewRateLimiter(3);

      private SlewRateLimiter leftLimiter = new SlewRateLimiter(0.3);
      private SlewRateLimiter rightLimiter = new SlewRateLimiter(0.3);
      
  
    public RevDrivetrain() { 
      LRearWheel.follow(LFrontWheel); 
      RRearWheel.follow(RFrontWheel);

      LFrontWheel.getEncoder().setPosition(0);
      RFrontWheel.getEncoder().setPosition(0);

  }

  public void limiterDrive(double leftPercent, double rightPercent) {
    roboDrive.tankDrive(leftLimiter.calculate(leftPercent), rightLimiter.calculate(rightPercent), false);
  }

  public void setOutputVolts(double leftVolts, double rightVolts) {
    LFrontWheel.setVoltage(leftVolts);
    RFrontWheel.setVoltage(rightVolts);
  }

  public void setOutputPercent(double leftPercent, double rightPercent) {
    LFrontWheel.set(leftPercent);
    RFrontWheel.set(rightPercent);
  }

  public void setOuputFeedforward(double leftVolts, double rightVolts) {
    LFrontWheel.setVoltage(feedforward.calculate(leftVolts));
    RFrontWheel.setVoltage(feedforward.calculate(leftVolts));
  } 

  public DifferentialDrive getDifferentialDrive () { 
      return roboDrive;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
