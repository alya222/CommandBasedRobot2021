package frc.robot.shooter;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.Limelight;

import static frc.robot.Gains.shooterPID.*;

import static frc.robot.Constants.*;


public class Shooter extends SubsystemBase {
    private ChangePosition goalMover;

    private Limelight vision;

    private CANSparkMax launcher = new CANSparkMax(kShooterPort, MotorType.kBrushless);

    private CANEncoder launcherEncoder = launcher.getEncoder();

    private CANPIDController launcherController = launcher.getPIDController();

    // Create toggle for shooting
    private boolean engaged = false;

    public double shooterRPM = midShooterRPM;

    public Shooter(ChangePosition changePosition, Limelight limelight) {
        // Makes changePosition instance the same as in RobotContainer
        goalMover = changePosition;
        vision = limelight;
        // Spark PID Stuff
        launcherController.setP(kP);
        launcherController.setI(kI);
        launcherController.setD(kD);
        launcherController.setFF(kF);

        //launcherEncoder.setVelocityConversionFactor(factor)
    }


    public void collect() {
        launcher.setVoltage(intakeVolts);
        engaged = true;
    }

    public void shoot() {
        launcher.setVoltage(shooterVolts);
        engaged = true;
    }

    public void stop() {
        launcher.setVoltage(0);
        launcherController.setReference(0, ControlType.kVoltage);

        engaged = false;
    }
    
    /**
     * Sets a voltage based on whether the robot is in shooting position or
     * intake position. 
     * @param inVolts The voltage sent to the shooter while in the intake position
     * @param outVolts The voltage sent to the shooter while in the shooting position
     */
    public void setSpeedVolts() {
        if (goalMover.isCollectingPose()) {
            // if in intake position, intake
            collect();
    
        } else {
            shoot();

        }
    }

    /**
     * Toggles shooter on and off with the specified voltage
     * @param inVolts voltage applied with intake
     * @param outVolts voltage applied with shooting
     */
    public void toggleSpeedVolts() {
        if (engaged) {
            stop();

        } else {
            setSpeedVolts();
        }
    }

    /**
   * Sets RPM based on whether the robot is in shooting position or 
   * intake position. 
   */
  public void setSpeedSpark() {
      if (goalMover.isCollectingPose()) {
          launcherController.setReference(intakeRPM, ControlType.kVelocity);
      
        } else {
            launcherController.setReference(shooterRPM, ControlType.kVelocity);

        }

        engaged = true;
  }

  public void toggleSpeedSpark() {
      if (engaged) {
          stop();

      } else {
          setSpeedSpark();

      }
  }

    /**
   * Using a ballistics equation and input distance, converts the output of meters per second
   * to RPM which can be output by the shooter
   * @param distance distance from the target
   * @return Rotations Per Minute (RPM) required to shoot a ball into the goal
   */
  private double calculateRPM(double distanceFeet) {
      // Get the velocity needed to shoot the ball
      double distance = Units.feetToMeters(distanceFeet);
      double top = Math.sqrt(-4.9 * Math.pow(distance, 2));
      double bottom = Math.sqrt(Math.pow(Math.cos(Math.toRadians(shooterAngle)), 2) * 
        (highGoalHeight - shooterHeight - Math.tan(shooterAngle) * distance));

    double metersPerSecond = top/bottom;

    double radiansPerSecond = metersPerSecond / kShooterWheelRadiusMeters;

    return Units.radiansPerSecondToRotationsPerMinute(radiansPerSecond);
  }
  /**
   * Sets RPM based on whether the robot is in shooting position or intake position. 
   * If it is in shooting position, calculates the RPM needed for the shooter to 
   * hit the target.
   * @param inRPM the velocity in RPM the shooter goes to while in the intake position
   * @param distance the distance in meters from the target
   */
  public void setRelativeSpeedSpark(double distance){
      if (goalMover.isCollectingPose()) {
          launcherController.setReference(intakeRPM, ControlType.kVelocity);
          engaged = true;
      
        } else {
            launcherController.setReference(calculateRPM(distance), ControlType.kVelocity);
            engaged = true;

        }
    }

    public void toggleRelativeSpeedSpark(double distance) {
        if (engaged) {
            stop();
            
            vision.driverMode();
            vision.lightOff();
        
        } else {
            setRelativeSpeedSpark(distance);
            vision.visionMode();
            vision.lightOn();
        }
    }

    public boolean isEngaged() {
        return engaged;
    }

    /**
   * Gets the shooter velocity in RPM
   */
  public double getVelocity() {
      return launcherEncoder.getVelocity();
  }

  public zoneSelector selector = zoneSelector.mid;

  public enum zoneSelector {
    near("Near"), 
    mid("Mid"), 
    far("Far");

    public final String positionName;

    zoneSelector (final String positionName) {
      this.positionName = positionName;
    }
}
/**
 * switches through the zones and switches RPM of shooter
 * @return new zone
 */
  public void zoneSwitch() {
    switch(selector) {
      case far:
        selector = zoneSelector.mid;
        shooterRPM = midShooterRPM;
        //SmartDashboard.putString("Shooting Postion:", zoneSelector.mid.positionName);
        break;

      case mid:
        selector = zoneSelector.near;
        shooterRPM = nearShooterRPM;
        //SmartDashboard.putString("Shooting Postion:", zoneSelector.near.positionName);
        break;

      case near:
        selector = zoneSelector.far;
        shooterRPM = farShooterRPM;
        //SmartDashboard.putString("Shooting Postion:", zoneSelector.far.positionName);
        break;
        
      default: 
        selector = zoneSelector.mid;
        shooterRPM = midShooterRPM;
        //SmartDashboard.putString("Shooting Postion:", zoneSelector.far.positionName);

    }

  } 

  @Override
  public void periodic() {
      /*
      if (goalMover.isSwapping) {
          stop();
          goalMover.isSwapping = false;
      }
      */
  }
}
