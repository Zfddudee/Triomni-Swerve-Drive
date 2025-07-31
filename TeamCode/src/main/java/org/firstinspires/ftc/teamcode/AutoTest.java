package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "AutoTest")
public class AutoTest extends OpMode {


    private Functions functions = new Functions();

    @Override
    public void init() {
        //initializing all hardware
        functions.mapHardware(hardwareMap);
        functions.newPoint(10,10,0);
        functions.newPoint(0,0,0);

    }

    @Override
    public void loop() {
        functions.odo.update();
        functions.followPath();

       if(gamepad1.a) Constants.powerMult = 0;
       if(gamepad1.b) Constants.powerMult = 0.3;

        telemetry.addData("X:", functions.getX()); //forwards +x
        telemetry.addData("Y:", functions.getY()); //right -y
        telemetry.addData("Update Time:", functions.getLoopTime(getRuntime()));
        telemetry.update();
    }
}
