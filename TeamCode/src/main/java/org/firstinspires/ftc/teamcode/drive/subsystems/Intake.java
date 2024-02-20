package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.IntakePosition;

public class Intake {
    public final DcMotor intake;
    private final Servo lift;
    public IntakePosition intakePosition;

    public boolean intakeActive = false;

    private double intakeSpeed;
    private double ejectSpeed;

    private final double intakePickupPosition;
    private final double intakeStowedPosition;
    private final boolean liftAvailable;

    public Intake(DcMotor intake, Servo lift, String activeConfigName) {
        this.intake = intake;
        this.lift = lift;
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setDirection(Servo.Direction.REVERSE);

        if (activeConfigName.equals(CENTERSTAGE_Bot.CONFIG_BOT_16464)) {
            intakeSpeed = 1;
            ejectSpeed = 0.25;

            // No Intake lift on 16464 bot
            liftAvailable = false;
            intakePosition = IntakePosition.PICKUP;
            intakePickupPosition = 0;
            intakeStowedPosition = 0;
        } else {
            intakeSpeed = 0.75;
            ejectSpeed = 0.25;

            liftAvailable = true;
            intakePosition = IntakePosition.STOWED;
            intakePickupPosition = 0.8;
            intakeStowedPosition = 1;
        }

    }


    public void setPosition(IntakePosition position) {
        if (!liftAvailable) {
            return;
        }

        switch (position) {
            case PICKUP:
                lift.setPosition(intakePickupPosition);
                break;
            case STOWED:
                lift.setPosition(intakeStowedPosition);

                // If the intake is stowed, it should stop spinning as well
                if (intakeActive) {
                    stopIntake();
                }

                break;
        }
        intakePosition = position;
    }

    public IntakePosition getPosition() {
        return intakePosition;
    }

    public void stopIntake() {
        intake.setPower(0);
        intakeActive = false;
    }

    public void togglePickup() {
        if (intakeActive) {
            stopIntake();
        } else {
            // If the intake was turned on, make sure it's on the ground
            if (intakePosition != IntakePosition.PICKUP) {
                setPosition(IntakePosition.PICKUP);
            }

            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setPower(intakeSpeed);
            intakeActive = true;
        }
    }

    public void toggleEject() {
        if (intakeActive) {
            stopIntake();
        } else {
            //Intake shouldn't be on ground for ejecting
            if (intakePosition != IntakePosition.STOWED) {
                setPosition(IntakePosition.STOWED);
            }

            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(ejectSpeed);
            intakeActive = true;
        }
    }

    public void togglePosition() {
        if (!liftAvailable) {
            return;
        }

        if (intakePosition == IntakePosition.STOWED) {
            setPosition(IntakePosition.PICKUP);
        } else {
            setPosition(IntakePosition.STOWED);
        }
    }
    public void setIntakeSpeed(double speed){
        intakeSpeed = speed;
    }
    public void setEjectSpeed(double speed){
        ejectSpeed = speed;
    }
    public void resetIntake(){
        setPosition(IntakePosition.STOWED);
    }
}
