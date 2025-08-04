package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "AutoTest")
public class AutoTest extends OpMode {
    //allows code to access functions class inside of the opmode
    private Functions functions = new Functions();

    @Override
    public void init() {
        //initializing all hardware
        functions.mapHardware(hardwareMap);
        //creating points to follow
        functions.newPoint(10,10,90);
//        functions.newPoint(0,0,0);
    }

    @Override
    public void loop() {
        //calls to update odometry every loop
        functions.odo.update();
        //follows the path that was creates on init
        functions.followPath(getRuntime());
        //telemetry
        telemetry.addData("X:", functions.getX()); //forwards +x
        telemetry.addData("Y:", functions.getY()); //right -y
        telemetry.addData("Update Time:", functions.getLoopTime(getRuntime()));
        telemetry.update();
    }
}
