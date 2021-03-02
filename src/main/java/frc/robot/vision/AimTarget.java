package frc.robot.vision;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.drive.RevDrivetrain;

import static frc.robot.Gains.*;

public class AimTarget extends CommandBase {
    private Limelight vision;
    private RevDrivetrain drive;

    private PIDController angleCorrector
      = new PIDController(angleCorrection.kP, angleCorrection.kI, angleCorrection.kD);
      
      
      /** 
       * Creates a new FollowTarget
       */
      public AimTarget(Limelight limelight, RevDrivetrain rDrive) {
          vision = limelight;
          drive = rDrive;
      

      // use addRequirements() here to declare subsystem dependencies
      addRequirements(vision, drive);

      angleCorrector.setTolerance(0.2);

      }

      // called when the command is initially scheduled
      @Override
      public void initialize() {
          angleCorrector.setSetpoint(0);
          vision.lightOn();
          vision.visionMode();
      }

      // called every time the scheduler runs while the command is scheduled
      @Override
      public void execute() {
          // passing aim PID output to the drive
          drive.getDifferentialDrive().arcadeDrive(
              0, 
              MathUtil.clamp(
                angleCorrector.calculate(vision.getXError()),
                 -0.5, 0.5),
          false
        );
      }

      // called once the command ends or is interrupted
      @Override
      public void end(boolean interrupted) {
          angleCorrector.reset();
          vision.lightOff();
      }


}
