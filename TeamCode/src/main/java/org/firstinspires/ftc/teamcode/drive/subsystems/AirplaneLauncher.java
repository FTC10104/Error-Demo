package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class AirplaneLauncher {
    enum LauncherPosition {
        LOAD,
        SEND
    }
    public LauncherPosition currentLauncherPosition = LauncherPosition.LOAD;

    public Servo airplaneLauncherServo;

    public AirplaneLauncher(Servo airplaneLauncherServo){
        this.airplaneLauncherServo = airplaneLauncherServo;

    }

    public void launchPlane10104(){
        if(currentLauncherPosition == LauncherPosition.LOAD){
            currentLauncherPosition = LauncherPosition.SEND;
            airplaneLauncherServo.setPosition(0.5);
        }
        else{
            currentLauncherPosition = LauncherPosition.LOAD;
            airplaneLauncherServo.setPosition(0);
        }
    }
    public void resetLauncher10104(){
        currentLauncherPosition = LauncherPosition.LOAD;
        airplaneLauncherServo.setPosition(0);
    }
    public void launchPlane16464(){
        if(currentLauncherPosition == LauncherPosition.LOAD){
            currentLauncherPosition = LauncherPosition.SEND;
            airplaneLauncherServo.setPosition(0.45);
        }
        else{
            currentLauncherPosition = LauncherPosition.LOAD;
            airplaneLauncherServo.setPosition(0.9);
        }
    }
    public void resetLauncher16464(){
        currentLauncherPosition = LauncherPosition.LOAD;
        airplaneLauncherServo.setPosition(0.9);
    }
}
