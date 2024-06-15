package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.subsystems.CENTERSTAGE_Bot;

@TeleOp
public class PublicDrive extends OpMode {
    private double slowdown = 0.4;
    private CENTERSTAGE_Bot bot;
    @Override
    public void init() {
        bot = new CENTERSTAGE_Bot(this);
        bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.speak("Team members should use other Op Mode");
    }

    @Override
    public void loop() {
        bot.setWeightedDrivePowers(
                -gamepad1.left_stick_y * slowdown,
                -gamepad1.left_stick_x * slowdown,
                -gamepad1.right_stick_x * slowdown
        );
    }
}
