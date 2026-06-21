package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

import java.util.function.DoubleSupplier;

public class TeleopDrive extends Command {
    private final Drive drive;
    private final DoubleSupplier xSpeed;
    private final DoubleSupplier ySpeed;
    private final DoubleSupplier rotation;
    

    public TeleopDrive(Drive drive, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotation) {
        this.drive = drive;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotation = rotation;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.driveCartesian(
            xSpeed.getAsDouble(), 
            ySpeed.getAsDouble(), 
            rotation.getAsDouble()
        );
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
