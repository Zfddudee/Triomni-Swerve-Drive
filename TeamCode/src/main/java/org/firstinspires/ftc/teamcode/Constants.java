package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Constants {

    public static double ticksPerRev = 751.8;
    public static double gearRatio = 32/16;
    public static double flipPoint = 110;
    public static double powerMult = 0.3; //stops driving all together at 0.08 power
    public static double lowSpeed = 0.065; //motor power at which the robot stops moving
    public static double driveToPointF = lowSpeed / powerMult;
    public static double turningGainP = 0.05;
    public static double driveToPointGainP = 0.1;//0.05/powerMult 0.1 / 0.4
    public static double defaultPrecision = 2;
}
