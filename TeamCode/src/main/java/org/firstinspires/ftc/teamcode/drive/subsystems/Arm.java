package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Arm {
    public final DcMotorEx armMotor;
    public final DistanceSensor centerSensor;
    private final Servo fingerServo;
    private final double currentLimit = 10;
    /**
     * Angle of the arm
     */
    private final double armAngle = 45;
    /**
     * Distance between the front sensor and the base of the arm, in centimeters
     */
    private final double armSensorDistanceCm;
    /**
     * Servo position for loading a pixel on the finger
     */
    private final double fingerLoadPosition;
    /**
     * Servo position for dropping a pixel on the backdrop
     */
    private final double fingerDropPosition;
    /**
     * Servo position for carrying a pixel around without dropping it
     */
    private final double fingerCarryPosition;
    /**
     * Maximum encoder ticks for a fully retracted arm
     */
    public final int minArmTicks = 0;
    /**
     * Maximum encoder ticks for a fully extended arm
     */
    private final int maxArmTicks;
    /**
     * Minimum number of arm ticks before the finger can be flipped
     */
    private final int minSafeTicks;
    /**
     * Maximum length that the arm can be extended, in centimeters
     */
    private final double maxArmLengthCm;
    /**
     * Distance to back off from the backboard to account for the length of the finger
     */
    private final int fingerBackboardSafety;
    /**
     * Number of ticks to add per cycle when extending or retracting the arm
     */
    private final int armExtendIncrement = 50;
    /**
     * Speed that the arm motor should go to reach its target position. -1 to 1
     */
    private final double armSpeed = 0.75;
    private FingerPosition currentFingerPosition = FingerPosition.LOAD;

    public Arm(DcMotorEx armMotor, Servo fingerServo, DistanceSensor centerSensor, String activeConfigName) {
        this.armMotor = armMotor;
        this.centerSensor = centerSensor;
        this.fingerServo = fingerServo;

        fingerServo.setDirection(Servo.Direction.FORWARD);

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setCurrentAlert(currentLimit, CurrentUnit.AMPS);

        if (CENTERSTAGE_Bot.CONFIG_BOT_16464.equals(activeConfigName)) {
            armSensorDistanceCm = 30;
            fingerLoadPosition = 0.82;
            fingerCarryPosition = 0.6;
            fingerDropPosition = 0;
            maxArmTicks = 3130;
            minSafeTicks = 800;
            maxArmLengthCm = 103.505;
            fingerBackboardSafety = 0;

        } else {
            armSensorDistanceCm = 30;
            fingerLoadPosition = 0.925;
            fingerCarryPosition = 0.6;
            fingerDropPosition = 0.1;
            maxArmTicks = 3130;
            minSafeTicks = 800;
            maxArmLengthCm = 103.505;
            fingerBackboardSafety = 5;
        }
    }

    public int cmToArmTicks(double distance) {
        double oneTick = maxArmTicks / maxArmLengthCm;
        return (int) (oneTick * distance);
    }

    public double getCenterSensorDistance() {
        return centerSensor.getDistance(DistanceUnit.CM);
    }

    public double calcAutoArmLength() {
        double sensorDistanceCm = getCenterSensorDistance() + armSensorDistanceCm;
        return sensorDistanceCm / Math.cos(Math.toRadians(armAngle));
    }

    public double autoExtendArm() {

        double calculatedMaxArmLength = calcAutoArmLength();
        int tickAmounts = cmToArmTicks(calculatedMaxArmLength);

        if (tickAmounts >= maxArmTicks) {
            tickAmounts = maxArmTicks;
        }

        armMotor.setTargetPosition(tickAmounts - fingerBackboardSafety);
        armMotor.setPower(-armSpeed);

        return calculatedMaxArmLength;
    }

    public void resetArm() {
        moveFinger(FingerPosition.LOAD);
        armMotor.setTargetPosition(minArmTicks);
    }

    public int getArmEncoder() {
        return armMotor.getCurrentPosition();
    }

    public int getArmTarget() {
        return armMotor.getTargetPosition();
    }

    public FingerPosition getCurrentFingerPosition() {
        return currentFingerPosition;
    }

    public void armExtend() {
        int targetPosition = armMotor.getTargetPosition();
        targetPosition += armExtendIncrement;

        if (targetPosition >= maxArmTicks) {
            return;
        }

        armMotor.setTargetPosition(targetPosition);
        armMotor.setPower(-armSpeed);
    }

    public void Extend16464() {
        armExtend();
        armExtend();
        armExtend();
        armExtend();
        armExtend();
        armExtend();
        armExtend();
    }

    public void armRetract() {

        int targetPosition = armMotor.getTargetPosition();
        targetPosition -= armExtendIncrement;

        if (targetPosition <= minArmTicks) {
            return;
        }

        if (targetPosition < minSafeTicks) {
            moveFinger(FingerPosition.LOAD);
        }

        armMotor.setTargetPosition(targetPosition);
        armMotor.setPower(armSpeed);
    }

    public void moveFinger(FingerPosition position) {

        switch (position) {
            case LOAD:
                fingerServo.setPosition(fingerLoadPosition);
                break;
            case CARRY:
                fingerServo.setPosition(fingerCarryPosition);
                break;
            default:
                if (getArmEncoder() < minSafeTicks) {
                    // If the arm is retracted too far, don't allow the finger to move
                    return;
                }
                fingerServo.setPosition(fingerDropPosition);
                break;
        }
        currentFingerPosition = position;
    }

    public void toggleFinger() {
        if (currentFingerPosition == FingerPosition.DROP) {
            moveFinger(FingerPosition.LOAD);
        } else {
            moveFinger(FingerPosition.DROP);
        }
    }

    public boolean isArmOverCurrent() {
        return armMotor.isOverCurrent();
    }

    public double getArmCurrent() {
        return armMotor.getCurrent(CurrentUnit.AMPS);
    }

    public void stopArm() {
        armMotor.setPower(0);
    }

    public enum FingerPosition {
        LOAD,
        CARRY,
        DROP
    }

}
