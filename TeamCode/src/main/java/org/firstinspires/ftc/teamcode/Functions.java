package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

//sets the teleop name visable on the driver station and sets it as a teleop program
public class Functions {

    //publicly used variables set off the bat

    double powerDir = 1;
    int ticksToMove = 0;
    double angle = 0;
    double steeringPower = 1;
    double wheelFlip = 0;

    double oldTime = 0;

    private Main main;

    public double getLoopTime(){
        //calculation to get code loop times
        double newTime = main.getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;
        return frequency;
    }

    public void Drive(double x, double y, double a) {
        double botHeading = main.odo.getHeading(UnnormalizedAngleUnit.DEGREES);
        double headingRad = -Math.toRadians(botHeading);

        double rotX = x * Math.cos(-headingRad) - y * Math.sin(-headingRad);
        double rotY = x * Math.sin(-headingRad) + y * Math.cos(-headingRad);

        //power for the wheels based off the magnitude of the joysticks movement
        double power = Math.sqrt((x * x)+(y * y));
        if(power > 1) power = 1;

        //angle of the joystick calculation
        if(x != 0 || y != 0) angle = Math.toDegrees(Math.atan2(rotX, rotY));
        if (angle < 0) angle += 360;
        if (x <= 0.05 && y <= 0.05 &&  ticksToMove < 15) main.steeringMotor.setPower(0);
        if (a <= 0.05){
            double lastHeading = botHeading;
            if(lastHeading < botHeading){

            } else if (lastHeading > botHeading) {


            }
        }

        //current swerve module angle calculation
        double currentWheelAngle = ((((main.steeringMotor.getCurrentPosition() / (Constants.ticksPerRev * Constants.gearRatio)) * 360) + 180) % 360) -180;
        currentWheelAngle %= 180;
        double wheelAngle = ((main.steeringMotor.getCurrentPosition() / (Constants.ticksPerRev * Constants.gearRatio)) * 360) + wheelFlip;
        wheelAngle %= 360;


        //difference between current wheel angle and target wheel angle
        double angleDelta = angle - wheelAngle;
        angleDelta = ((angleDelta + 540) % 360) - 180;
        wheelFlip %= 360;

        //if difference in angles is too large flips direction of wheels so the turning is more efficient

        if(Math.abs(angleDelta) >= Constants.flipPoint){
            wheelFlip += 180 * Math.signum(angleDelta);
            powerDir *= -1;
        }

        //re-calculates the wheel angle and delta after the corrections
        wheelAngle = ((main.steeringMotor.getCurrentPosition() / (Constants.ticksPerRev * Constants.gearRatio)) * 360) + wheelFlip;
        wheelAngle %= 360;
        angleDelta = ((angleDelta + 540) % 360) - 180;

        //calculates the target motor position to turn the wheels
        double targetPos = (angleDelta / 360) * Constants.ticksPerRev * Constants.gearRatio;
        ticksToMove = main.steeringMotor.getCurrentPosition() + (int)Math.round(targetPos);

        //turns the module to set position
        main.steeringMotor.setTargetPosition(ticksToMove);
        main.steeringMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        main.steeringMotor.setPower(steeringPower);

        //sets drive power on wheels
        //2
        if(currentWheelAngle > 30 && currentWheelAngle <= 90 || currentWheelAngle < -90 && currentWheelAngle >= -150){
            main.driveA.setPower((power * powerDir + a) * Constants.powerMult); //
            main.driveB.setPower((power * powerDir - a) * Constants.powerMult); //
            main.driveC.setPower((power * powerDir) * Constants.powerMult);
        }//3
        else if (currentWheelAngle > 90 && currentWheelAngle <= 150 || currentWheelAngle < -30 && currentWheelAngle >= -90){
            main.driveA.setPower((power * powerDir - a) * Constants.powerMult); //
            main.driveB.setPower(power * powerDir * Constants.powerMult);
            main.driveC.setPower((power * powerDir + a) * Constants.powerMult); //
        }//1
        else{
            main.driveA.setPower((power * powerDir) * Constants.powerMult);
            main.driveB.setPower((power * powerDir + a) * Constants.powerMult); //
            main.driveC.setPower((power * powerDir - a) * Constants.powerMult); //
        }
    }
}

