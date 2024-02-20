package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ButtonTracker;
import org.firstinspires.ftc.teamcode.drive.subsystems.Arm;
import org.firstinspires.ftc.teamcode.drive.subsystems.CENTERSTAGE_Bot;

@TeleOp
public class ArcadeDrive extends OpMode {
    private final ButtonTracker fingerButton = new ButtonTracker();
    private final ButtonTracker fingerCarryButton = new ButtonTracker();
    private final ButtonTracker autoExtendArmButton = new ButtonTracker();
    private final ButtonTracker resetArmButton = new ButtonTracker();
    private final ButtonTracker driveFlipButton = new ButtonTracker();
    private final ButtonTracker liftButton = new ButtonTracker();
    private final ButtonTracker intakeButton = new ButtonTracker();
    private final ButtonTracker ejectButton = new ButtonTracker();
    private final ButtonTracker launchButton = new ButtonTracker();
    private final ButtonTracker holdButton = new ButtonTracker();
    private final ButtonTracker releaseButton = new ButtonTracker();
    private final ButtonTracker initButton = new ButtonTracker();
    double slowdown = 0.7;
    double driveFlip = 1;
    boolean armOverCurrent = false;
    boolean climberOverCurrent = false;
    private CENTERSTAGE_Bot bot;

    @Override
    public void init() {
        bot = new CENTERSTAGE_Bot(this);
        bot.init(Arm.FingerPosition.LOAD);
        bot.resetArm();
        bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        fingerButton.update(gamepad2.b);
        fingerCarryButton.update(gamepad2.right_trigger > 0.3);
        driveFlipButton.update(gamepad1.right_bumper);
        liftButton.update(gamepad2.a);
        autoExtendArmButton.update(gamepad2.dpad_up);
        resetArmButton.update(gamepad2.dpad_down);
        intakeButton.update(gamepad2.x);
        ejectButton.update(gamepad2.y);
        launchButton.update(gamepad2.back);
        holdButton.update(gamepad1.dpad_up);
        releaseButton.update(gamepad1.dpad_down);
        initButton.update(gamepad1.start);

        if (driveFlipButton.wasReleased) {
            driveFlip = -driveFlip;
        }

        bot.setWeightedDrivePowers(
                -gamepad1.left_stick_y * slowdown * driveFlip,
                -gamepad1.left_stick_x * slowdown * driveFlip,
                -gamepad1.right_stick_x * slowdown
        );
        if (initButton.wasPressed){
            bot.init(Arm.FingerPosition.LOAD);
        }

        if (gamepad2.left_bumper) {
            bot.armExtend();
        } else if (gamepad2.right_bumper) {
            bot.armRetract();
        }

        if (fingerButton.wasPressed) {
            bot.toggleFinger();
        } else if (fingerCarryButton.wasPressed) {
            bot.moveFinger(Arm.FingerPosition.CARRY);
        }

        if (intakeButton.wasPressed) {
            bot.togglePickup();
        } else if (ejectButton.wasPressed) {
            bot.toggleEject();
        }

        if (liftButton.wasPressed) {
            bot.toggleLift();
        }

        if (autoExtendArmButton.wasPressed) {
            bot.autoExtendArm();
        }

        if (resetArmButton.wasPressed) {
            bot.resetArm();
        }

        if (launchButton.wasPressed) {
            bot.launchPlane();
        }
       /* if (holdButton.wasPressed) {
            bot.holdClimber();
        } else if (releaseButton.wasPressed) {
            bot.releaseClimber();
        }

        */

        if (bot.arm.isArmOverCurrent()) {
            armOverCurrent = true;
            bot.arm.stopArm();
            telemetry.speak("Arm is over current!");
            gamepad1.rumble(1, 1, 500);
            gamepad2.rumble(1, 1, 500);
        }
        if (armOverCurrent && !bot.arm.isArmOverCurrent()) {
            telemetry.speak("Arm is at a safe current. Please be cautious.");
            armOverCurrent = false;
        }
        /*
        if (climberOverCurrent && !bot.isClimberOverCurrent()) {
            telemetry.speak("Climber is at a safe current. Please be cautious.");
            climberOverCurrent = false;
        }
        if (bot.isClimberOverCurrent()) {
            climberOverCurrent = true;
            bot.stopClimber();
            telemetry.speak("Climber is over current!");
            gamepad1.rumble(1, 1, 500);
            gamepad2.rumble(1, 1, 500);
        }

         */
        if (bot.climber.holding) {
            int current = bot.climber.climberMotor.getCurrentPosition();
            bot.climber.climberMotor.setTargetPosition(current);
        } else {
            if (gamepad1.left_trigger > 0.1) {
                bot.extendClimber(gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.1) {
                bot.retractClimber(gamepad1.right_trigger);
            } else {
                bot.stopClimber();
            }
        }

        bot.update();
        telemetry.addData("Position X", Math.toDegrees(bot.getPoseEstimateX()));
        telemetry.addData("Position Y", Math.toDegrees(bot.getPoseEstimateY()));
        telemetry.addData("Heading", Math.toDegrees(bot.getPoseEstimateHeading()));
        telemetry.addData("Finger", bot.arm.getCurrentFingerPosition());
        telemetry.addData("Intake Position", bot.intake.getPosition());
        telemetry.addData("Forward Distance", bot.arm.getCenterSensorDistance());
        telemetry.addData("Arm Ticks", bot.getArmTicks());
        telemetry.addData("Arm Target pos", bot.getArmTarget());
        telemetry.addData("Arm Current", bot.arm.getArmCurrent());
        telemetry.update();
    }
}