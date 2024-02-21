import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.subsystems.CENTERSTAGE_Bot;

//This auton will speak from the Driver Station whenever a kid puts their hand near a
// distance sensor and will make a different voice depending on the sensor
@Autonomous
public class DemoAuton extends OpMode {
    private CENTERSTAGE_Bot bot;
    @Override
    public void init() {
        bot = new CENTERSTAGE_Bot(this);
    }

    @Override
    public void loop() {
        if (bot.getDistanceLeft(DistanceUnit.INCH) <= 3){
            telemetry.speak("I See you", "en", "US");
        } else if (bot.getDistanceCenter(DistanceUnit.INCH) <=3) {
            telemetry.speak("Sneaking up behind me?", "en", "FR");
        } else if (bot.getDistanceRight(DistanceUnit.INCH) <= 3) {
            telemetry.speak("Blud is not very sneaky", "en", "GB");
        }
    }
}
