package frc.robot.drive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Gears extends SubsystemBase {
    private Solenoid leftGear = new Solenoid(compressorModule, leftGearPort);
    private Solenoid rightGear = new Solenoid(compressorModule, rightGearPort);

    private boolean isFast = false;

    private void fast(){
        leftGear.set(true);
        rightGear.set(true);

        isFast = true;
    }

    private void slow(){
        leftGear.set(false);
        rightGear.set(false);

        isFast = false;
    }
    
    public void switchGears() {
        if (isFast) {
            slow();
        
        } else {
            fast();
        }
    }
    
}
