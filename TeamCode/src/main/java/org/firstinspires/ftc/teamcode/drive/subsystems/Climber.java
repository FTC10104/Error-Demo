package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Climber {

    public final DcMotorEx climberMotor;
    public Boolean holding = false;


    public Climber(DcMotorEx climberMotor) {
        this.climberMotor = climberMotor;
        //climberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //climberMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void extend(double speed) {
        climberMotor.setPower(speed / 2);
    }

    public void retract(double speed) {
        climberMotor.setPower(-speed / 2);
    }

    public void stop() {
        climberMotor.setPower(0);
    }
/*
    public void hold() {
        holding = true;
    }

    public void release() {
        holding = false;
    }
    public void toggleHold(){
        if (holding){
            release();
        } else {
            hold();
        }
    }
    public boolean isOverCurrent(){
        return climberMotor.isOverCurrent();
    }
 */


}
