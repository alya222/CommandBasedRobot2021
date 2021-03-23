package frc.robot.climber;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Lift extends SubsystemBase {
    private WPI_TalonSRX leftArm = new WPI_TalonSRX(leftArmPort);

    private boolean negation = true; // on true for safety, in case arm is accidentally activated

    public void move(double speed) {
        leftArm.set(speed);
    }

    public void moveOneAxis (double speed) {
        if (negation) {
            leftArm.set(-speed);
        } else {
            leftArm.set(speed);
        }
    }

    public void switchMovement() {
        if (negation) {
            negation = false;
        } else {
            negation = true;
        }
    }

    @Override
    public void periodic() {
        // this method will be called once per scheduler run
    }
    
}
