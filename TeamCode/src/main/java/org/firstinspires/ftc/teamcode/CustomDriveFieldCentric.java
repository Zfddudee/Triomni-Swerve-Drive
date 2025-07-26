package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

//sets the teleop name visable on the driver station and sets it as a teleop program
@TeleOp(name = "Field Centric Drive")
public class CustomDriveFieldCentric extends OpMode {

    //publicly used variables set off the bat
    DcMotor steeringMotor;
    DcMotor driveA, driveB, driveC;
    GoBildaPinpointDriver odo;

    double ticksPerRev = 751.8;
    double gearRatio = 32/16;
    double powerDir = 1;
    int ticksToMove = 0;
    double angle = 0;
    double steeringPower = 1;
    double wheelFlip = 0;
    double flipPoint = 110;
    double powerMult = 0.5;
    double oldTime = 0;
    boolean isFlipped = false;


    //function for when you press init but does not loop
    @Override
    public void init() {
        // Hardware mapping
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

        //telemetry to state robot has initialized
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    //main loop that loops through code on start
    @Override
    public void loop() {
        odo.update();

        //calculation to get code loop times
        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;

        //variables to store the joystick positions
        double botHeading = odo.getHeading(UnnormalizedAngleUnit.DEGREES);
        double headingRad = -Math.toRadians(botHeading);
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double a = -gamepad1.right_stick_x;
        double rotX = x * Math.cos(-headingRad) - y * Math.sin(-headingRad);
        double rotY = x * Math.sin(-headingRad) + y * Math.cos(-headingRad);

        //power for the wheels based off the magnitude of the joysticks movement
        double power = Math.sqrt((x * x)+(y * y));
        if(power > 1) power = 1;

        //angle of the joystick calculation
        if(x != 0 || y != 0) angle = Math.toDegrees(Math.atan2(rotX, rotY));
        if (angle < 0) angle += 360;
        if (x <= 0.05 && y <= 0.05 &&  ticksToMove < 15) steeringMotor.setPower(0);
        if (a <= 0.05){
            double lastHeading = botHeading;
            if(lastHeading < botHeading){

            } else if (lastHeading > botHeading) {


            }
        }

        //current swerve module angle calculation
        double currentWheelAngle = ((((steeringMotor.getCurrentPosition() / (ticksPerRev * gearRatio)) * 360) + 180) % 360) -180;

        double actualWheelAngle = (steeringMotor.getCurrentPosition() / (ticksPerRev * gearRatio)) * 360;
        actualWheelAngle %= 360;
        if(actualWheelAngle < 0) actualWheelAngle += 360;

        currentWheelAngle %= 180;
        double wheelAngle = ((steeringMotor.getCurrentPosition() / (ticksPerRev * gearRatio)) * 360) + wheelFlip;
        wheelAngle %= 360;


        //difference between current wheel angle and target wheel angle
        double angleDelta = angle - wheelAngle;
        angleDelta = ((angleDelta + 540) % 360) - 180;
        wheelFlip %= 360;

        //if difference in angles is too large flips direction of wheels so the turning is more efficient

        if(Math.abs(angleDelta) >= flipPoint){
            wheelFlip += 180 * Math.signum(angleDelta);
            powerDir *= -1;
        }

        //re-calculates the wheel angle and delta after the corrections
        wheelAngle = ((steeringMotor.getCurrentPosition() / (ticksPerRev * gearRatio)) * 360) + wheelFlip;
        wheelAngle %= 360;
        angleDelta = ((angleDelta + 540) % 360) - 180;

        //calculates the target motor position to turn the wheels
        double targetPos = (angleDelta / 360) * ticksPerRev * gearRatio;
        ticksToMove = steeringMotor.getCurrentPosition() + (int)Math.round(targetPos);

        //turns the module to set position
        steeringMotor.setTargetPosition(ticksToMove);
        steeringMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        steeringMotor.setPower(steeringPower);

        //sets drive power on wheels
//        driveA.setPower(power * powerDir * powerMult);
//        driveB.setPower(power * powerDir * powerMult);
//        driveC.setPower(power * powerDir * powerMult);
        boolean inBackSide = actualWheelAngle > 150 && actualWheelAngle < 330;

        if (inBackSide && !isFlipped){
            a *= -1;
            isFlipped = true;
        }
        else if (!inBackSide && isFlipped){
            a *= -1;
            isFlipped = false;
        }
//        boolean isFlippedPhysically = ((int)(Math.floor(actualWheelAngle / 180))) % 2 != 0;
// Flip turn input if modules are reversed
//        if (isFlippedPhysically) a *= -1;
        //2
        if(currentWheelAngle > 30 && currentWheelAngle <= 90 || currentWheelAngle < -90 && currentWheelAngle >= -150){
            driveA.setPower((power * powerDir + a) * Constants.powerMult); //
            driveB.setPower((power * powerDir - a) * Constants.powerMult); //
            driveC.setPower((power * powerDir) * Constants.powerMult);
        }//3
        else if (currentWheelAngle > 90 && currentWheelAngle <= 150 || currentWheelAngle < -30 && currentWheelAngle >= -90){
            driveA.setPower((power * powerDir - a) * Constants.powerMult); //
            driveB.setPower(power * powerDir * Constants.powerMult);
            driveC.setPower((power * powerDir + a) * Constants.powerMult); //
        }//1
        else{
            driveA.setPower((power * powerDir) * Constants.powerMult);
            driveB.setPower((power * powerDir + a) * Constants.powerMult); //
            driveC.setPower((power * powerDir - a) * Constants.powerMult); //
        }
//        //2
//        if(currentWheelAngle > 30 && currentWheelAngle <= 90){
//            driveA.setPower((power * powerDir - a) * powerMult); //
//            driveB.setPower((power * powerDir + a) * powerMult); //
//            driveC.setPower((power * powerDir) * powerMult);
//        }
//        else if(currentWheelAngle < -90 && currentWheelAngle >= -150){
//            driveA.setPower((power * powerDir + a) * powerMult); //
//            driveB.setPower((power * powerDir - a) * powerMult); //
//            driveC.setPower((power * powerDir) * powerMult);
//        }//3
//        else if (currentWheelAngle > 90 && currentWheelAngle <= 150){
//            driveA.setPower((power * powerDir - a) * powerMult); //
//            driveB.setPower(power * powerDir * powerMult);
//            driveC.setPower((power * powerDir + a) * powerMult); //
//        }else if(currentWheelAngle < -30 && currentWheelAngle >= -90){
//            driveA.setPower((power * powerDir + a) * powerMult); //
//            driveB.setPower(power * powerDir * powerMult);
//            driveC.setPower((power * powerDir - a) * powerMult); //
//        }//1
//        else if(currentWheelAngle >= -30 && currentWheelAngle <= 30){
//            driveA.setPower((power * powerDir) * powerMult);
//            driveB.setPower((power * powerDir + a) * powerMult); //
//            driveC.setPower((power * powerDir - a) * powerMult); //
//        }else{
//            driveA.setPower((power * powerDir) * powerMult);
//            driveB.setPower((power * powerDir - a) * powerMult); //
//            driveC.setPower((power * powerDir + a) * powerMult); //
//        }

        //stops the wheels from turning for testing purposes
        if(gamepad1.a) {
            steeringPower = 0;
            powerMult = 0;
        }
        if(gamepad1.b) {
            steeringPower = 1;
            powerMult = 0.5;
        }
        if(gamepad1.x) {
            steeringMotor.setTargetPosition(0);
            steeringMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            steeringMotor.setPower(steeringPower);
        }
        if(gamepad1.y)powerMult = 0;

        //telemetry read outs on screen
//        telemetry.addData("Wheel Flip", wheelFlip);
//        telemetry.addData("Ticks", steeringMotor.getCurrentPosition());
//        telemetry.addData("Ticks to move", ticksToMove);

        telemetry.addData("Heading", botHeading);
        telemetry.addData("Angle", angle);
        telemetry.addData("Actual Wheel Angle:", actualWheelAngle);
        telemetry.addData("Current Wheel Angle:", currentWheelAngle);
        telemetry.addData("Wheel Angle:", wheelAngle);
        telemetry.addData("Angle delta:", angleDelta);
        telemetry.addData("Right:", (power * powerDir - a));
        telemetry.addData("Left:", (power * powerDir + a));
        telemetry.addData("isFLipped:", isFlipped);
        telemetry.addData("a:", a);
        telemetry.addData("Update Time:", frequency);

        telemetry.update();
    }
}
