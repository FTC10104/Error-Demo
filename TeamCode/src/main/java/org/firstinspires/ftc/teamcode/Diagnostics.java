package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.subsystems.CENTERSTAGE_Bot;

@TeleOp
public class Diagnostics extends OpMode {
    private CENTERSTAGE_Bot bot;

    @Override
    public void init() {
        bot = new CENTERSTAGE_Bot(this);
    }


    @Override
    public void loop() {
        if (gamepad1.y) {
            bot.drive.leftFront.setPower(1);
        } else {
            bot.drive.leftFront.setPower(0);
        }
        if (gamepad1.b) {
            bot.drive.rightFront.setPower(1);
        } else {
            bot.drive.rightFront.setPower(0);
        }

        if (gamepad1.a) {
            bot.drive.rightRear.setPower(1);
        } else {
            bot.drive.rightRear.setPower(0);
        }
        if (gamepad1.x) {
            bot.drive.leftRear.setPower(1);
        } else {
            bot.drive.leftRear.setPower(0);
        }
        telemetry.addData("Right Distance (CM)", bot.rightSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Left Distance (CM)", bot.leftSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Center Distance (CM)", bot.centerSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Heading (deg)", Math.toDegrees(bot.drive.getExternalHeading()));
        telemetry.update();
    }
}
