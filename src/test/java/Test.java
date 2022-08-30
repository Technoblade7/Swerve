import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import static frc.robot.Constants.*;
import edu.wpi.first.math.geometry.Rotation2d;

public class Test {
    public Rotation2d toRoation2d(int ticks) {
        return new Rotation2d(ticks % kTicksPerRotationAngleMotor / (double) kTicksPerRotationAngleMotor * 2 * Math.PI);
    }

    public int toTicks(Rotation2d angle) {
        return (int) (angle.getRadians() / (2 * Math.PI) * kTicksPerRotationAngleMotor);
    }

    @org.junit.Test
    public void test() {
        int error = toTicks(Rotation2d.fromDegrees(0).minus(toRoation2d((int) 300))) + 50;
        System.out.println(error);
        System.out.println(300 + error);
    }
}
