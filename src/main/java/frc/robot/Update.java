package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.shooter.Plucker;
import frc.robot.shooter.Shooter;

import static frc.robot.Constants.*;
import static frc.robot.Gains.*;

public class Update {
    private Shooter m_shooter;
    private Plucker m_plucker;

    //starting positions
    private final Pose2d left = new Pose2d(-1, 0, Rotation2d.fromDegrees(0));
    private final Pose2d center = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    private final Pose2d right = new Pose2d(1, 0, Rotation2d.fromDegrees(0));

    private static final SendableChooser choosePosition = new SendableChooser<Pose2d>();

    public Update(Shooter shooter, Plucker plucker) {
        m_shooter = shooter;
        m_plucker = plucker;

        choosePosition.setDefaultOption("Center", center);
        choosePosition.addOption("Left", left);
        choosePosition.addOption("Right", right);
        SmartDashboard.putData("Satrting Position", choosePosition);

        // display PID values (angle)
        SmartDashboard.putNumber("P value(angle)", angleCorrection.kP);
        SmartDashboard.putNumber("I value(angle)", angleCorrection.kI);
        SmartDashboard.putNumber("D value(angle)", angleCorrection.kD);

        //display left and right shooter velocities
        SmartDashboard.putNumber("Shooter RPM", m_shooter.getVelocity());

        // displays whether plucker is engaged
        SmartDashboard.putBoolean("Plucker Engaged", m_plucker.getEngaged());
    }

    public static Pose2d getStartingPose() {
        final Pose2d position = (Pose2d) choosePosition.getSelected();
        return position;
    }

    public void periodic() {
        // update left and right shooter velocities
        SmartDashboard.putNumber("Shooter RPM", m_shooter.getVelocity());

        // displays whether plucker is engaged
        SmartDashboard.putBoolean("Plucker Engaged", m_plucker.getEngaged());

        // change PID values for angle correction
        if (angleCorrection.kP != SmartDashboard.getNumber("P value(angle)", angleCorrection.kP)) {
            angleCorrection.kP = SmartDashboard.getNumber("P value(angle)", angleCorrection.kP);
        }

        if (angleCorrection.kI != SmartDashboard.getNumber("I value(angle)", angleCorrection.kI)) {
            angleCorrection.kI = SmartDashboard.getNumber("I value(angle)", angleCorrection.kD);
        }
    }
}
