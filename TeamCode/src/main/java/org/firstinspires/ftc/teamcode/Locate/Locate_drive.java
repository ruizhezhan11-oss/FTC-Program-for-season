package org.firstinspires.ftc.teamcode.Locate;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Locate_drive")
public class Locate_drive {
    private DcMotor FL,FR,BL,BR;
    public IMU imu;
    public void init (HardwareMap hwM){
        FL=hwM.get(DcMotor.class,"FL");
        FR=hwM.get(DcMotor.class,"FR");
        BL=hwM.get(DcMotor.class,"BL");
        BR=hwM.get(DcMotor.class,"BR");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        imu=hwM.get(IMU.class,"imu");
        RevHubOrientationOnRobot RevOrientation =new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(RevOrientation));
    }
    public void drive(double forward,double strafe,double rotate){
        double FLPower = forward + strafe + rotate;
        double BLPower = forward - strafe + rotate;
        double FRPower = forward - strafe - rotate;
        double BRPower = forward + strafe - rotate;

        double MaxPower = 1.0;
        double MaxSpeed = 1.0;

        MaxPower = Math.max(MaxPower,Math.abs(FLPower));
        MaxPower = Math.max(MaxPower,Math.abs(FRPower));
        MaxPower = Math.max(MaxPower,Math.abs(BLPower));
        MaxPower = Math.max(MaxPower,Math.abs(BRPower));

        FL.setPower(MaxSpeed * (FLPower/MaxPower));
        FR.setPower(MaxSpeed * (FRPower/MaxPower));
        BL.setPower(MaxSpeed * (BLPower/MaxPower));
        BR.setPower(MaxSpeed * (BRPower/MaxPower));
    }
    public void driveFieldRelative(double forward,double strafe,double rotate){
        double theta = Math.atan2(forward,strafe);
        double r = Math.hypot(strafe,forward);

        theta = AngleUnit.normalizeRadians(
                theta- imu.getRobotYawPitchRollAngles().getYaw()
        );
        double new_forward = r*Math.sin(theta);
        double new_strafe = r*Math.cos(theta);
        this.drive(new_forward,new_strafe,rotate);
    }
}
