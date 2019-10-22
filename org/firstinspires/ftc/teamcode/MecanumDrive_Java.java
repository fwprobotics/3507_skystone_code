package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MecanumDriveTeleOp", group = "")
public class MecanumDrive_Java extends LinearOpMode {

  private DcMotor backLeftDrive;
  private DcMotor backRightDrive;
  private DcMotor frontLeftDrive;
  private DcMotor frontRightDrive;
  private DcMotor liftMotor;
  private Servo clawServo;
  private Servo leftFoundationServo;
  private Servo rightFoundationServo;
  private AnalogInput armPot;
  private DcMotor armMotor;

  double LeftY;
  double RightX;
  double LeftX;
  double bottomArmPosPot = 3.3;
  double topArmPosPot = 1.4;
  double potMargin = 0.1;
  boolean aButtonDown;
  boolean aButtonDown2;
  String clawPos = "closed";
  String armPos = "down";
  String hookPos = "up";


  @Override
  public void runOpMode() {

    backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
    backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
    frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
    frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
    liftMotor = hardwareMap.dcMotor.get("liftMotor");
    clawServo = hardwareMap.servo.get("clawServo");
    leftFoundationServo = hardwareMap.servo.get("leftFoundationServo");
    rightFoundationServo = hardwareMap.servo.get("rightFoundationServo");
    armPot = hardwareMap.analogInput.get("armPot");
    armMotor = hardwareMap.dcMotor.get("armMotor");

    // Put initialization blocks here.
    backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    telemetry.addData("Arm Pot Value", armPot.getVoltage());
    // Servos
    leftFoundationServo.setPosition(1);
    rightFoundationServo.setPosition(0);
    
    telemetry.update();
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        LeftX = -gamepad1.left_stick_x;
        LeftX = (LeftX / 1.07) * (0.62 * (LeftX * LeftX) + 0.45);
        LeftY = -gamepad1.left_stick_y;
        RightX = -gamepad1.right_stick_x;
        
        driving();
        armControl();
        clawControl();
        foundationHooks();
        
        liftMotor.setPower(gamepad2.right_stick_y);
        
        telemetry();
      }
    }
  }

  /**
   * This function controls the motors for driving. Simple mecanum math.
   */
  private void driving() {
    frontLeftDrive.setPower((LeftY - RightX) - LeftX);
    frontRightDrive.setPower(LeftY + RightX + LeftX);
    backLeftDrive.setPower((LeftY - RightX) + LeftX);
    backRightDrive.setPower((LeftY + RightX) - LeftX);
  }
  
  /**
   * This function opens and closes the claw on the arm.
   */
  private void clawControl() {
    if (gamepad2.a && clawPos.equals("closed") && !aButtonDown) {
      clawServo.setPosition(0.6);
      aButtonDown = true;
      clawPos = "open";
      sleep (100);
    } else if (gamepad2.a && clawPos.equals("open") && !aButtonDown) {
      clawServo.setPosition(1);
      clawPos = "closed";
      aButtonDown = true;
      sleep(100);
    }
    if (!gamepad2.a) {
      aButtonDown = false;
    }
  }

  /**
   * This function moves the arm on the lift. X is used to put it up, and y
   * to put it down. Essentially it checks the potentiometer and if it is not
   * where it should be the motor is powered in a while loop until it gets there.
   * The motor is then turned off.
   */
  
  private void armControl() {
    if (gamepad2.x) {
      armPos = "down";
    } 
    
    else if (gamepad2.y) {
      armPos = "up";
    }
    
    // Bringing the arm up
    if (armPos == "up" && armPot.getVoltage() >= topArmPosPot + potMargin) {
      armMotor.setPower(0.7);
      
      // if (clawPos == "closed") {
      //   armMotor.setPower(-0.8);
      // }
      
      // else if (clawPos == "open") {
      //   armMotor.setPower(-0.3);
      // }
    }
    
    // Bringing the arm down
    else if (armPos == "down" && armPot.getVoltage() <= bottomArmPosPot - potMargin) {
      armMotor.setPower(-0.2);
    }

    else {
      armMotor.setPower(0);
    }
  }

  
  // private void armControl() {
  //   // going down
  //   //&& armPot.getVoltage() >= bottomArmPosPot - potMargin
  //   if (gamepad2.x) {
  //     armMotor.setPower(1);
  //   // going up
  //   } else if (gamepad2.y) {
  //     armMotor.setPower(-1);
  //   } else {
  //     armMotor.setPower(0);
  //   }
  // }
  
  private void foundationHooks() {
    if (gamepad1.a && hookPos.equals("down") && !aButtonDown2) {
      leftFoundationServo.setPosition(1);
      rightFoundationServo.setPosition(0);
      aButtonDown2 = true;
      hookPos = "up";
      sleep (100);
    } else if (gamepad1.a && hookPos.equals("up") && !aButtonDown2) {
      leftFoundationServo.setPosition(0);
      rightFoundationServo.setPosition(1);
      hookPos = "down";
      aButtonDown2 = true;
      sleep(100);
    }
    if (!gamepad1.a) {
      aButtonDown2 = false;
    }
  }
  
  
  private void telemetry() {
    telemetry.addData("A button down?", aButtonDown);
    telemetry.addData("Arm Pot Value", armPot.getVoltage());
    telemetry.addData("Lift Power", liftMotor.getPower());
    telemetry.addData("Lift Encoder Value", liftMotor.getCurrentPosition());
    telemetry.addData("LeftStickX", LeftX);
    telemetry.addData("LeftStickY", LeftY);
    telemetry.addData("RightStickX", RightX);
    telemetry.addData("FrontLeft", frontLeftDrive.getPower());
    telemetry.addData("FrontRight", frontRightDrive.getPower());
    telemetry.addData("BackLeft", backLeftDrive.getPower());
    telemetry.addData("BackRight", backRightDrive.getPower());
    telemetry.update();
  }
}
