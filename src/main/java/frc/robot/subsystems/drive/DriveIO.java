package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
    
    @AutoLog
    public static class DriveIOInputs {
        public double frontLeftAppliedOutput = 0.0;
        public double frontRightAppliedOutput = 0.0;
        public double backLeftAppliedOutput = 0.0;
        public double backRightAppliedOutput = 0.0;
    }

    public default void updateInputs(DriveIOInputs inputs) {}

    public default void driveCartesian(double xSpeed, double ySpeed, double rotation) {}

    public default void stop() {}
}
