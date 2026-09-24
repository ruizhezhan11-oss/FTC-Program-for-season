package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="TEAM_AUTO")
public abstract class  SEASON_AUTO extends LinearOpMode {
    private GoBildaPinpointDriver Pinpoint;
    private DcMotor BL,BR,FL,FR;
    public void init_hardware(){
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
        Pinpoint=hardwareMap.get(GoBlidaPinpointDriver.class,"Pinpoint");
    }
    private void init_xy(){
        pinpoint1.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint1.setOffsets(111, 111, DistanceUnit.MM);
        pinpoint1.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint1.resetPosAndIMU();
    }
    @Override
    public void runOpMode(){
        init_hardware();
        init_xy();
        waitForStart();
        while(opModeIsActive()){

        }
    }
}
