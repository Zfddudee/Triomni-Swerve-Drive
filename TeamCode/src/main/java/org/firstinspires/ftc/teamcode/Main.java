package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.Functions;

import java.util.List;

@TeleOp(name = "Main")
public class Main extends OpMode {


    private Functions functions = new Functions();

    @Override
    public void init() {
        //initializing all hardware
        functions.mapHardware(hardwareMap);
    }

    @Override
    public void loop() {
        //variables to store the joystick positions
        double x = -gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double a = -gamepad1.right_stick_x;

        functions.Drive(x,y,a, -1);

        if(Math.abs(gamepad1.right_stick_x) > 0.05) functions.Drive(x,y,a,-1);
        else if(gamepad1.dpad_up) functions.Drive(x,y,a,0);
        else if(gamepad1.dpad_left) functions.Drive(x,y,a,90);
        else if(gamepad1.dpad_down) functions.Drive(x,y,a,180);
        else if(gamepad1.dpad_right) functions.Drive(x,y,a,270);
                //telemetry read outs on screen
//        telemetry.addData("Heading", botHeading);
//        telemetry.addData("Heading", functions.botHeadingUsable);
//        telemetry.addData("Heading vel", functions.headingVel);
        telemetry.addData("X:", functions.odo.getPosX(DistanceUnit.INCH)); //forwards +x
        telemetry.addData("Y:", functions.odo.getPosY(DistanceUnit.INCH)); //right -y
        telemetry.addData("Update Time:", functions.getLoopTime(getRuntime()));
        telemetry.update();
    }
    @Override
    public void stop(){

    }
}
