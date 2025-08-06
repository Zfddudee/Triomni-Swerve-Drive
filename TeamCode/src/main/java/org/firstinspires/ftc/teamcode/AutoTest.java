package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "AutoTest")
public class AutoTest extends OpMode {
    //allows code to access functions class inside of the opmode
    private Functions functions = new Functions();
    private Functions.Path path1 = functions.new Path();
    private Functions.Path path2 = functions.new Path();

    @Override
    public void init() {
        //initializing all hardware
        functions.mapHardware(hardwareMap);
        //creating points to follow
        path1.newPoint(10,10,0);
        path1.newActionWait(() -> {
            functions.odo.setPosX(0, DistanceUnit.INCH);
            functions.odo.setPosY(0, DistanceUnit.INCH);
        }, 500);
        path1.newPoint(-10,-10,0);


        path2.newPoint(0,25);
        path2.newPoint(-10,35);
        path2.newActionWait(() -> {}, 250);
        path2.newPoint(0,25);
        path2.newPoint(0,0);
//        functions.newPoint(0,0,0);
    }

    @Override
    public void loop() {
        //calls to update odometry every loop
        functions.odo.update();
        //follows the path that was creates on init
        functions.followPath(path2,getRuntime() * 1000);
        //telemetry
        telemetry.addData("X:", functions.getX()); //forwards +x
        telemetry.addData("Y:", functions.getY()); //right -y
        telemetry.addData("Speed:", functions.speed); //right -y
        telemetry.addData("Error:", functions.error); //right -y
        telemetry.addData("Precision:", functions.precision); //right -y
        telemetry.addData("Time Error:", functions.deltaTime); //right -y
        telemetry.addData("Update Time:", functions.getLoopTime(getRuntime()));
        telemetry.update();
    }
}
