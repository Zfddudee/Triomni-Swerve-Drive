package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AutoTest")
public class AutoTest extends OpMode {


    private Functions functions = new Functions();

    @Override
    public void init() {
        //initializing all hardware
        functions.mapHardware(hardwareMap);
    }

    @Override
    public void loop() {
        functions.odo.update();
       functions.driveTo(10,10,0,0.5);
       if(gamepad1.a) Constants.powerMult = 0;
       if(gamepad1.b) Constants.powerMult = 0.3;

        telemetry.addData("X:", functions.getX()); //forwards +x
        telemetry.addData("Y:", functions.getY()); //right -y
        telemetry.addData("Update Time:", functions.getLoopTime(getRuntime()));
        telemetry.update();
    }
}
