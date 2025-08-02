package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.ArrayList;
import java.util.List;

//sets the teleop name visable on the driver station and sets it as a teleop program
public class Functions {
    DcMotor steeringMotor;
    DcMotor driveA, driveB, driveC;
    GoBildaPinpointDriver odo;
    private HardwareMap hardwareMap;


    //publicly used variables set off the bat
    double powerDir = 1;
    int ticksToMove = 0;
    double angle = 0;
    double steeringPower = 1;
    double wheelFlip = 0;
    boolean latch = false;
    double lastHeading = 0;
    double botHeadingUsable = 0;
    double headingVel = 0;
    double holdDelta = 0;

    double oldTime = 0;


    //function to map hardware called on init
    public void mapHardware(HardwareMap map){
        this.hardwareMap = map;
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

        //enables bulk reads to speed up code
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public double getX(){
        return odo.getPosX(DistanceUnit.INCH);
    }
    public double getY(){
        return -odo.getPosY(DistanceUnit.INCH);
    }
    public double getLoopTime(double newTime){ //input getRunTime() function to this
        //calculation to get code loop times
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;
        return frequency;
    }

    public double normalizeTo180 (double angle) {
        return ((angle + 540) % 360) - 180;
    }

    public void recenterModules(){
        steeringMotor.setTargetPosition(0);
        steeringMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        steeringMotor.setPower(steeringPower);
    }
    public void Drive(double x, double y, double a, double heading) {
        //updates pinpoint
        odo.update();

        double botHeading = odo.getHeading(UnnormalizedAngleUnit.DEGREES);
        botHeadingUsable = (botHeading + 360) % 360;
        double headingRad = Math.toRadians(botHeading);
        headingVel = Math.abs(odo.getHeadingVelocity());

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

        //TODO: tune heading velocity constraint, strength and fix constant spinning
        //correction so if not moving right joystick it holds heading to account for drift
        if (Math.abs(a) <= 0.05){
            if(!latch && headingVel < 0.2 * Constants.powerMult){
                lastHeading = botHeadingUsable;
                latch = true;
            }else if(latch){
                if(heading != -1) lastHeading = heading;
                holdDelta = normalizeTo180(lastHeading - botHeadingUsable);
                if(Math.abs(holdDelta) > 2) a = holdDelta * Constants.turningGainP;
            }
        }else latch = false;

        //current swerve module angle calculation
        double unnormalizedWheelAngle = (steeringMotor.getCurrentPosition() / (Constants.ticksPerRev * Constants.gearRatio)) * 360;

        //current swerve module angle normalized to -180 to 180
        double currentWheelAngle = normalizeTo180(unnormalizedWheelAngle);

        //current swerve module angle as seen by robot with flip correction in a -360 to 360
        double flippedWheelAngle = unnormalizedWheelAngle + wheelFlip;
        flippedWheelAngle %= 360;


        //difference between current wheel angle and target wheel angle
        double angleDelta = normalizeTo180(angle - flippedWheelAngle);
        wheelFlip %= 360;

        //if difference in angles is too large flips direction of wheels so the turning is more efficient
        if(Math.abs(angleDelta) >= Constants.flipPoint){
            wheelFlip += 180 * Math.signum(angleDelta);
            powerDir *= -1;
        }

        //re-calculates the wheel angle and delta after the corrections
        flippedWheelAngle = ((steeringMotor.getCurrentPosition() / (Constants.ticksPerRev * Constants.gearRatio)) * 360) + wheelFlip;
        flippedWheelAngle %= 360;

        angleDelta = normalizeTo180(angle - flippedWheelAngle);

        //calculates the target motor position to turn the wheels
        double targetPos = (angleDelta / 360) * Constants.ticksPerRev * Constants.gearRatio;
        ticksToMove = steeringMotor.getCurrentPosition() + (int)Math.round(targetPos);

        //turns the module to set position
        steeringMotor.setTargetPosition(ticksToMove);
        steeringMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        steeringMotor.setPower(steeringPower);

        double leftWheel = (power * powerDir + a) * Constants.powerMult;
        double rightWheel = (power * powerDir - a) * Constants.powerMult;
        double middleWheel = (power * powerDir) * Constants.powerMult;
        //sets drive power on wheels depending on the direction that the swerve modules are facing to account for the turning
        //2
        if(currentWheelAngle > 30 && currentWheelAngle <= 90){
            driveA.setPower(rightWheel); //
            driveB.setPower(leftWheel); //
            driveC.setPower(middleWheel);
        }
        else if(currentWheelAngle <= -90 && currentWheelAngle > -150){
            driveA.setPower(leftWheel); //
            driveB.setPower(rightWheel); //
            driveC.setPower(middleWheel);
        }//3
        else if (currentWheelAngle > 90 && currentWheelAngle <= 150){
            driveA.setPower(rightWheel); //
            driveB.setPower(middleWheel);
            driveC.setPower(leftWheel); //
        }else if(currentWheelAngle <= -30 && currentWheelAngle > -90){
            driveA.setPower(leftWheel); //
            driveB.setPower(middleWheel);
            driveC.setPower(rightWheel); //
        }//1
        else if(currentWheelAngle > -30 && currentWheelAngle <= 30){
            driveA.setPower(middleWheel);
            driveB.setPower(leftWheel); //
            driveC.setPower(rightWheel); //
        }else{
            driveA.setPower(middleWheel);
            driveB.setPower(rightWheel); //
            driveC.setPower(leftWheel); //
        }
    }

    public static class PathPoint {
        public double x, y, heading, precision;

        public PathPoint(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.precision = 1;
        }
        public PathPoint(double x, double y, double heading, double precision) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.precision = precision;
        }
    }
    List<PathPoint> points = new ArrayList<>();

    public void newPoint(double x, double y, double heading){
        points.add(new PathPoint(x, y, heading));
    }
    int currentPointIndex = 0;
    double p = 0;
    double i = 0;
    double d = 0;
    double lastTime = 0;
    double lastError = 0;
    public void followPath(double time){
        PathPoint currentPoint = points.get(currentPointIndex);
        double x = currentPoint.x;
        double y = currentPoint.y;
        double heading = currentPoint.heading;
        double precision = currentPoint.precision;


        double curX = odo.getPosX(DistanceUnit.INCH);
        double curY = odo.getPosY(DistanceUnit.INCH);

        double driveX = -curY;
        double driveY = curX;

        double dx = x - driveX;
        double dy = y - driveY;
        double dt = time - oldTime;

        double error = Math.sqrt((dx * dx)+(dy * dy));

        p = error * Constants.driveToPointGainP;
        i += (error * dt) * Constants.driveToPointGainI;
        d += ((error - lastError) / dt) * Constants.driveToPointGainD;

        double speed = Math.min(p + i + d, 1.0); // Speed factor, proportional to distance

        double xOutput = (-dx/error) * speed;
        double yOutput = (dy/error) * speed;

        lastError = error;
        lastTime = time;
        if(error > precision) Drive(xOutput, yOutput, 0, heading);
        else if(currentPointIndex + 1 < points.size()) currentPointIndex++;
        else recenterModules();
    }
}

