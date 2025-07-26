package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.Functions;

import java.util.List;

@TeleOp(name = "Main")
public class Main extends OpMode {
    DcMotor steeringMotor;
    DcMotor driveA, driveB, driveC;
    GoBildaPinpointDriver odo;

    private Functions functions;

    @Override
    public void init() {

        steeringMotor = hardwareMap.get(DcMotor.class, "steering_motor");
        driveA = hardwareMap.get(DcMotor.class, "Drive_A");
        driveB = hardwareMap.get(DcMotor.class, "Drive_B");
        driveC = hardwareMap.get(DcMotor.class, "Drive_C");
        // Reverse motors if needed
        driveC.setDirection(DcMotorSimple.Direction.REVERSE);

        //odometry initialization
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.recalibrateIMU();

        //steering motor settings
        steeringMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        steeringMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//enables bulk reads to speed up loop times
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
    @Override
    public void loop() {
        //updates pinpoint
        odo.update();

        //variables to store the joystick positions
        double x = -gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double a = -gamepad1.right_stick_x;

        functions.Drive(x,y,a);
        //        //telemetry read outs on screen
//        telemetry.addData("Heading", botHeading);
//        telemetry.addData("Angle", angle);
//        telemetry.addData("Actual Wheel Angle:", currentWheelAngle);
//        telemetry.addData("Wheel Angle:", wheelAngle);
//        telemetry.addData("Angle delta:", angleDelta);
//        telemetry.addData("Update Time:", frequency);
//
//        telemetry.update();
    }
    @Override
    public void stop(){}

}
