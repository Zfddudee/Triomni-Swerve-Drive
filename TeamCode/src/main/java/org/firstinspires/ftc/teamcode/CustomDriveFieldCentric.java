package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.List;

//sets the teleop name visable on the driver station and sets it as a teleop program
@Disabled
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
    boolean latch = false;
    double lastHeading = 0;
    double holdDelta = 0;


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

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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

        //variables for the heading of the robot in degrees then in radians
        double botHeading = odo.getHeading(UnnormalizedAngleUnit.DEGREES);
        double botHeadingUsable = (botHeading + 360) %360;
        double headingRad = Math.toRadians(botHeading);
        double headingVel = Math.abs(odo.getHeadingVelocity());

        //variables to store the joystick positions
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double a = -gamepad1.right_stick_x;
        //rotated x and y variables to allow for field centric driving
        double rotX = x * Math.cos(headingRad) - y * Math.sin(headingRad);
        double rotY = x * Math.sin(headingRad) + y * Math.cos(headingRad);

        //power for the wheels based off the magnitude of the joysticks movement then making sure it doesnt go above 1
        double power = Math.sqrt((x * x)+(y * y));
        if(power > 1) power = 1;

        //angle of the joystick calculation and then normalize angle of joystick to be in 0 to 360 degrees
        if(x != 0 || y != 0) angle = Math.toDegrees(Math.atan2(rotX, rotY));
        if (angle < 0) angle += 360;

        //if joysticks are sitting still set power for steering motor to 0 to conserve power
        if (x <= 0.05 && y <= 0.05 &&  ticksToMove < 15) steeringMotor.setPower(0);

        //TODO: tune heading velocity constraint, strenght and fix constant spinning
        //correction so if not moving right joystick it holds heading to account for drift
        if (Math.abs(a) <= 0.05){
            if(!latch && headingVel < 1){
                lastHeading = botHeadingUsable;
                latch = true;
            }else if(latch){
                holdDelta = (lastHeading - botHeadingUsable + 540) % 360 - 180;
                if(Math.abs(holdDelta) > 2) a = holdDelta * Constants.turningGainP;
            }
        }else latch = false;

        //current swerve module angle calculation
        double actualWheelAngle = (steeringMotor.getCurrentPosition() / (ticksPerRev * gearRatio)) * 360;

        //current swerve module angle normalized to -180 to 180
        double currentWheelAngle = ((actualWheelAngle + 180) % 360) -180;
        currentWheelAngle %= 180;

        //current swerve module angle as seen by robot with flip correction in a -360 to 360
        double wheelAngle = actualWheelAngle + wheelFlip;
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

        //sets drive power on wheels depending on the direction that the swerve mdodules are facing to account for the turning
        //2
        if(currentWheelAngle > 30 && currentWheelAngle <= 90){
            driveA.setPower((power * powerDir - a) * powerMult); //
            driveB.setPower((power * powerDir + a) * powerMult); //
            driveC.setPower((power * powerDir) * powerMult);
        }
        else if(currentWheelAngle <= -90 && currentWheelAngle > -150){
            driveA.setPower((power * powerDir + a) * powerMult); //
            driveB.setPower((power * powerDir - a) * powerMult); //
            driveC.setPower((power * powerDir) * powerMult);
        }//3
        else if (currentWheelAngle > 90 && currentWheelAngle <= 150){
            driveA.setPower((power * powerDir - a) * powerMult); //
            driveB.setPower(power * powerDir * powerMult);
            driveC.setPower((power * powerDir + a) * powerMult); //
        }else if(currentWheelAngle <= -30 && currentWheelAngle > -90){
            driveA.setPower((power * powerDir + a) * powerMult); //
            driveB.setPower(power * powerDir * powerMult);
            driveC.setPower((power * powerDir - a) * powerMult); //
        }//1
        else if(currentWheelAngle > -30 && currentWheelAngle <= 30){
            driveA.setPower((power * powerDir) * powerMult);
            driveB.setPower((power * powerDir + a) * powerMult); //
            driveC.setPower((power * powerDir - a) * powerMult); //
        }else{
            driveA.setPower((power * powerDir) * powerMult);
            driveB.setPower((power * powerDir - a) * powerMult); //
            driveC.setPower((power * powerDir + a) * powerMult); //
        }

        //stops the wheels from turning for testing purposes
        if(gamepad1.a) {
            steeringPower = 0;
            powerMult = 0;
        }
        if(gamepad1.b) {
            steeringPower = 1;
            powerMult = 0.5;
        }

        //telemetry read outs on screen
        telemetry.addData("Heading", botHeadingUsable);
//        telemetry.addData("Angle", angle);
//        telemetry.addData("Actual Wheel Angle:", actualWheelAngle);
//        telemetry.addData("Current Wheel Angle:", currentWheelAngle);
//        telemetry.addData("Wheel Angle:", wheelAngle);
//        telemetry.addData("Angle delta:", angleDelta);
        telemetry.addData("last Heading:", lastHeading);
        telemetry.addData("Hold Delta:", holdDelta);
        telemetry.addData("a:", a);
        telemetry.addData("Heading Velocity:", headingVel);
        telemetry.addData("Update Time:", frequency);
        telemetry.update();
    }
}
