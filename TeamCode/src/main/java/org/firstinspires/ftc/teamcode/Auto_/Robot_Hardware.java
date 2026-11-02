package org.firstinspires.ftc.teamcode.Auto_;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
@Autonomous
public class Robot_Hardware extends LinearOpMode {
    DcMotor FL,FR,BL,BR;
    GoBildaPinpointDriver pinpointDriver;
    double x=0,y=0,heading=0;
    public void init_hardware (){
        BL=hardwareMap.get(DcMotor.class,"BL");
        BR=hardwareMap.get(DcMotor.class,"BR");
        FL=hardwareMap.get(DcMotor.class,"FL");
        FR=hardwareMap.get(DcMotor.class,"FR");
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void reset(){pinpointDriver.resetPosAndIMU();
        sleep(300);}
    public void init_odo(){
        pinpointDriver=hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpointDriver.setOffsets(0,0, DistanceUnit.MM);
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        reset();
    }
    public void Math_Odo(){
        pinpointDriver.update();
        Pose2D pose2D = pinpointDriver.getPosition();
        x=pose2D.getX(DistanceUnit.INCH);
        y= pose2D.getY(DistanceUnit.INCH);
        heading=pose2D.getHeading(AngleUnit.DEGREES);
    }
    public void telemetry(){
        telemetry.addLine(String.format("XYH,%2.1f %2.1f %2.1f",x,y,heading));
    }
    enum state {

    }
    double target_x = 100 , target_y = 100 , target_h = 0;
    double tolerance = 2;
    double Kp_drive = 0.01 , Kp_turn = 1.0;

    @Override
    public void runOpMode(){
        init_hardware();
        init_odo();
        while (opModeIsActive()){
        Math_Odo();
        double dx = (target_x-x), dy = (target_y-y) , eh = (target_h-heading);
        double dist = Math.hypot(dx,dy); //直角三角形定位
        if(dist<tolerance){
            break; //後面接上state
        }
        double forward = dx*Math.cos(heading) + dy*Math.sin(heading);
        double strafe = dx*Math.sin(heading) +dy*Math.cos(heading);
        eh=Math.atan2(Math.sin(eh),Math.cos(eh));
        double fwdPower = Range.clip(Kp_drive*forward , -0.5,0.5);
        double strPower = Range.clip(Kp_drive*strafe,-0.5,0.5);
        double turnPower = Range.clip(Kp_turn*eh,-0.3,0.3);

            double fl = fwdPower + strPower + turnPower;
            double fr = fwdPower - strPower - turnPower;
            double bl = fwdPower - strPower + turnPower;
            double br = fwdPower + strPower - turnPower;

            FL.setPower(fl);
            FR.setPower(fr);
            BL.setPower(bl);
            BR.setPower(br);
        }
    }
}
