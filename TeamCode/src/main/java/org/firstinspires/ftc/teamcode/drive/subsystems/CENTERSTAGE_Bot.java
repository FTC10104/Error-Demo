package org.firstinspires.ftc.teamcode.drive.subsystems;
//update

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import android.app.Activity;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.IntakePosition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class CENTERSTAGE_Bot {

    public static final String CONFIG_BOT_16464 = "WireECoyote";
    public static final String CONFIG_BOT_10104 = "Error";

    public final OpMode opMode;
    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;
    public final SampleMecanumDrive drive;
    public final Arm arm;
    public final Intake intake;
    public final AirplaneLauncher airplaneLauncher;
    public final Climber climber;
    public final DistanceSensor rightSensor;
    public final DistanceSensor leftSensor;
    public final DistanceSensor centerSensor;
    private final int botLength;
    public int position = 2;

    public CENTERSTAGE_Bot(OpMode opMode) {

        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.botLength = 16;

        String activeConfigName = getActiveConfigName(hardwareMap);

        this.arm = new Arm(
                hardwareMap.get(DcMotorEx.class, "Arm"),
                hardwareMap.get(Servo.class, "finger"),
                hardwareMap.get(DistanceSensor.class, "centerSensor"),
                activeConfigName
        );

        this.rightSensor = drive.rightSensor;
        this.leftSensor = drive.leftSensor;
        this.centerSensor = hardwareMap.tryGet(DistanceSensor.class, "centerSensor");


        this.intake = new Intake(
                hardwareMap.get(DcMotor.class, "Intake"),
                hardwareMap.get(Servo.class, "lift"),
                activeConfigName
        );

        this.airplaneLauncher = new AirplaneLauncher(
                hardwareMap.get(Servo.class, "launch")
        );

        this.climber = new Climber(hardwareMap.get(DcMotorEx.class, "climber"));
        if (getActiveConfigName(hardwareMap).equals(CONFIG_BOT_16464)){
            airplaneLauncher.currentLauncherPosition = AirplaneLauncher.LauncherPosition.LOAD;
            airplaneLauncher.airplaneLauncherServo.setPosition(0.9);
        } else {
            airplaneLauncher.currentLauncherPosition = AirplaneLauncher.LauncherPosition.LOAD;
            airplaneLauncher.airplaneLauncherServo.setPosition(0);
        }
    }

    public static String getActiveConfigName(HardwareMap hardwareMap) {
        RobotConfigFileManager configFileManager = new RobotConfigFileManager((Activity) hardwareMap.appContext);
        return configFileManager.getActiveConfig().getName();
    }

    public void gyroTurn(Integer angle) {
        double error = 0;
        double current = Math.toDegrees(drive.getRawExternalHeading());
        double target = current + angle;
        do {
            current = Math.toDegrees(drive.getRawExternalHeading());
            error = target - current;
            double motorPower = error * 0.02;

            if (motorPower > 0) {
                motorPower = Range.clip(motorPower, 0.3, 0.6);
            } else {
                motorPower = Range.clip(motorPower, -0.6, -0.3);
            }
            setWeightedDrivePowers(0, 0, motorPower);
            update();
        } while (Math.abs(error) > 1 && !Thread.currentThread().isInterrupted());
        stop();

    }

    public void findProp() {
        Trajectory FindDaProp = trajectoryBuilder(new Pose2d(), DriveConstants.MAX_VEL * 0.25)
                .forward(46 - botLength)
                .build();

        followTrajectoryAsync(FindDaProp);

        while (isBusy()) {
            if (getPoseEstimateX() >= 14) {
                if (getDistanceLeft(DistanceUnit.CM) <= 14) {
                    position = 1;
                }
                if (getDistanceRight(DistanceUnit.CM) <= 14) {
                    position = 3;
                }
            }

            update();
        }
        telemetry.addData("Position", position);
        telemetry.update();
    }

    public void reverseFindProp() {
        back(46 - botLength);
    }

    public void launchPlane() {
        if (CONFIG_BOT_16464.equals(getActiveConfigName(hardwareMap))){
            airplaneLauncher.launchPlane16464();
        } else {
            airplaneLauncher.launchPlane10104();
        }
    }
/*
    public void holdClimber() {
        climber.hold();
    }

    public void releaseClimber() {
        climber.release();
    }


    public boolean isClimberOverCurrent() {
        return climber.isOverCurrent();
    }
 */

    public double getDistanceLeft(DistanceUnit unit) {
        return leftSensor.getDistance(unit);
    }

    public double getDistanceRight(DistanceUnit unit) {
        return rightSensor.getDistance(unit);
    }

    public double getDistanceCenter(DistanceUnit unit) {
        return centerSensor.getDistance(unit);
    }

    public boolean opModeIsActive() {
        return !Thread.currentThread().isInterrupted();
    }

    public void armExtend() {
        arm.armExtend();
    }

    public void extend16464() {
        arm.Extend16464();
    }


    public void resetIntake() {
        intake.resetIntake();
    }

    public void init(Arm.FingerPosition position) {
        resetIntake();
        resetArm();
        if (position != null) {
            moveFinger(position);
        }
    }

    public void armRetract() {
        arm.armRetract();
    }

    public double autoExtendArm() {
        return arm.autoExtendArm();
    }

    public void resetArm() {
        arm.resetArm();
    }

    public int getArmTicks() {
        return arm.getArmEncoder();
    }

    public int getArmTarget() {
        return arm.getArmTarget();
    }

    public void moveFinger(Arm.FingerPosition position) {
        arm.moveFinger(position);
    }

    public void toggleFinger() {
        arm.toggleFinger();
    }

    public boolean isBusy() {
        return drive.isBusy();
    }

    public void setPoseEstimate(double x, double y, double heading) {
        Pose2d startPose = new Pose2d(x, y, heading);
        drive.setPoseEstimate(startPose);
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public double getPoseEstimateX() {
        return drive.getPoseEstimate().getX();
    }

    public double getPoseEstimateY() {
        return drive.getPoseEstimate().getY();
    }

    public double getPoseEstimateHeading() {
        return drive.getPoseEstimate().getHeading();
    }

    public void forward(double distance) {
        forward(distance, MAX_VEL);
    }

    public void forward(double distance, double maxVelocity) {
        Trajectory trajectory = drive.trajectoryBuilder(getPoseEstimate(), maxVelocity)
                .forward(distance)
                .build();
        drive.followTrajectory(trajectory);
    }

    public void back(double distance) {
        back(distance, MAX_VEL);
    }

    public void back(double distance, double maxVelocity) {
        Trajectory trajectory = drive.trajectoryBuilder(getPoseEstimate(), maxVelocity)
                .back(distance)
                .build();
        drive.followTrajectory(trajectory);
    }

    public void strafeLeft(double distance) {
        strafeLeft(distance, MAX_VEL);
    }

    public void strafeLeft(double distance, double maxVelocity) {
        Trajectory trajectory = drive.trajectoryBuilder(getPoseEstimate(), maxVelocity)
                .strafeLeft(distance)
                .build();
        drive.followTrajectory(trajectory);
    }

    public void strafeRight(double distance) {
        strafeRight(distance, MAX_VEL);
    }

    public void strafeRightAsync(double distance) {
        strafeRightAsync(distance, MAX_VEL);
        while (isBusy()) {
            if (getDistanceRight(DistanceUnit.CM) < 5) {
                stop();
            }
            update();
        }
    }

    public void strafeLeftAsync(double distance) {
        strafeLeftAsync(distance, MAX_VEL);
        while (isBusy()) {
            if (getDistanceLeft(DistanceUnit.CM) < 5) {
                stop();
            }
            update();
        }
    }

    public void strafeRight(double distance, double maxVelocity) {
        Trajectory trajectory = drive.trajectoryBuilder(getPoseEstimate(), maxVelocity)
                .strafeRight(distance)
                .build();
        drive.followTrajectory(trajectory);
    }

    public void strafeRightAsync(double distance, double maxVelocity) {
        Trajectory trajectory = trajectoryBuilder(getPoseEstimate(), maxVelocity)
                .strafeRight(distance)
                .build();
        followTrajectoryAsync(trajectory);
    }

    public void strafeLeftAsync(double distance, double maxVelocity) {
        Trajectory trajectory = trajectoryBuilder(getPoseEstimate(), maxVelocity)
                .strafeLeft(distance)
                .build();
        followTrajectoryAsync(trajectory);
    }

    public void turn(double angle, double maxAngVel) {
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(getPoseEstimate())
                .turn(Math.toRadians(angle), maxAngVel, MAX_ANG_ACCEL)
                .build();
        drive.followTrajectorySequence(trajectorySequence);
    }

    public void stop() {
        drive.setMotorPowers(0, 0, 0, 0);
    }

    public void turn(double angle) {
        turn(angle, MAX_ANG_VEL);
    }

    public void followTrajectory(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
        drive.waitForIdle();
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(double x, double y, double heading) {
        return new TrajectorySequenceBuilder(
                new Pose2d(x, y, heading),
                SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL),
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        drive.followTrajectorySequence(trajectorySequence);
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        drive.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d position) {
        return new TrajectoryBuilder(position,
                SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL));
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d position, double maxVelocity) {
        return new TrajectoryBuilder(position,
                SampleMecanumDrive.getVelocityConstraint(maxVelocity, MAX_ANG_VEL, TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL));
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d position, double maxVelocity, boolean reversed) {
        return new TrajectoryBuilder(position, reversed,
                SampleMecanumDrive.getVelocityConstraint(maxVelocity, MAX_ANG_VEL, TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL));
    }

    public void update() {
        drive.update();
    }

    public void setWeightedDrivePowers(double x, double y, double heading) {
        drive.setWeightedDrivePower(
                new Pose2d(
                        x,
                        y,
                        heading
                )
        );
    }

    public void setMotorPowers(double leftFront, double leftRear, double rightRear, double rightFront) {
        drive.setMotorPowers(leftFront, leftRear, rightRear, rightFront);
    }

    public void setMode(DcMotor.RunMode runMode) {
        drive.setMode(runMode);
    }

    public void setLift(IntakePosition position) {
        intake.setPosition(position);
    }

    public void togglePickup() {
        intake.togglePickup();
    }

    public void toggleEject() {
        intake.toggleEject();
    }

    public void toggleLift() {
        intake.togglePosition();
    }

    public int getBotLength() {
        return botLength;
    }

    public void setIntakeSpeed(double speed) {
        intake.setIntakeSpeed(speed);
    }

    public void setEjectSpeed(double speed) {
        intake.setEjectSpeed(speed);
    }

    public void extendClimber(double speed) {
        climber.extend(speed);
    }

    public void retractClimber(double speed) {
        climber.retract(speed);
    }

    public void stopClimber() {
        climber.stop();
    }

}