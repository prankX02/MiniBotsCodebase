package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;

public class DriveIOSim implements DriveIO {

    // TODO: Replace with actual wheel position measurements in meters
    private static final double TRACK_WIDTH_M  = 0.36; // TODO: left-to-right wheel distance
    private static final double TRACK_LENGTH_M = 0.45; // TODO: front-to-back wheel distance

    // TODO: Replace with your robot's actual max speed in meters per second
    private static final double MAX_SPEED_MPS = 2.0;

    private static final double LOOP_PERIOD_SECS = 0.02;

    private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
        new Translation2d( TRACK_LENGTH_M / 2,  TRACK_WIDTH_M / 2),  // front left
        new Translation2d( TRACK_LENGTH_M / 2, -TRACK_WIDTH_M / 2),  // front right
        new Translation2d(-TRACK_LENGTH_M / 2,  TRACK_WIDTH_M / 2),  // rear left
        new Translation2d(-TRACK_LENGTH_M / 2, -TRACK_WIDTH_M / 2)   // rear right
    );

    private final MecanumDriveOdometry odometry = new MecanumDriveOdometry(
        kinematics,
        Rotation2d.kZero,
        new MecanumDriveWheelPositions(),
        new Pose2d(0, 0, Rotation2d.kZero)
    );

    private double flPosition = 0.0;
    private double frPosition = 0.0;
    private double rlPosition = 0.0;
    private double rrPosition = 0.0;

    private MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds();

    private Pose2d pose = new Pose2d();

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        // Integrate wheel speeds into positions
        flPosition += wheelSpeeds.frontLeftMetersPerSecond  * LOOP_PERIOD_SECS;
        frPosition += wheelSpeeds.frontRightMetersPerSecond * LOOP_PERIOD_SECS;
        rlPosition += wheelSpeeds.rearLeftMetersPerSecond   * LOOP_PERIOD_SECS;
        rrPosition += wheelSpeeds.rearRightMetersPerSecond  * LOOP_PERIOD_SECS;

        pose = odometry.update(
            Rotation2d.kZero,
            new MecanumDriveWheelPositions(flPosition, frPosition, rlPosition, rrPosition)
        );

        inputs.frontLeftAppliedOutput  = wheelSpeeds.frontLeftMetersPerSecond  / MAX_SPEED_MPS;
        inputs.frontRightAppliedOutput = wheelSpeeds.frontRightMetersPerSecond / MAX_SPEED_MPS;
        inputs.backLeftAppliedOutput   = wheelSpeeds.rearLeftMetersPerSecond   / MAX_SPEED_MPS;
        inputs.backRightAppliedOutput  = wheelSpeeds.rearRightMetersPerSecond  / MAX_SPEED_MPS;
    }

    @Override
    public void driveCartesian(double xSpeed, double ySpeed, double rotation) {
        wheelSpeeds = kinematics.toWheelSpeeds(
            new ChassisSpeeds(
                xSpeed   * MAX_SPEED_MPS,
                ySpeed   * MAX_SPEED_MPS,
                rotation * MAX_SPEED_MPS
            )
        );
    }

    @Override
    public void stop() {
        wheelSpeeds = new MecanumDriveWheelSpeeds();
    }

    public Pose2d getPose() {
        return pose;
    }
}