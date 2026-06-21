package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {

    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    // Only non-null in sim
    private final DriveIOSim simIO;

    public Drive(DriveIO io) {
        this.io = io;
        this.simIO = (io instanceof DriveIOSim) ? (DriveIOSim) io : null;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);

        // Log pose in sim so AdvantageScope can display it on the field
        if (simIO != null) {
            Logger.recordOutput("Drive/Pose", simIO.getPose());
        }
    }

    public void driveCartesian(double xSpeed, double ySpeed, double rotation) {
        io.driveCartesian(xSpeed, ySpeed, rotation);
        Logger.recordOutput("Drive/xSpeed", xSpeed);
        Logger.recordOutput("Drive/ySpeed", ySpeed);
        Logger.recordOutput("Drive/rotation", rotation);
    }

    public void stop() {
        io.stop();
    }
}