/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification,
 * are permitted (subject to the limitations in the disclaimer below)
 * provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 *  endorse or
 * promote products derived from this software without specific prior written
 *  permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 *  THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Locale;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.Pullbot.RingOrientationAnalysisPipeline.Stage.*;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single
 * robot. In this case that robot is a Pullbot, a front wheel drive version of
 * the Pushbot in the external samples.
 * <p>
 * The Pullbot class assumes the following device names have been configured
 * on the robot:
 * Note:  All names are lower case.
 * <p>
 * Motor channel:  Left front  drive motor:        "motor0"
 * Motor channel:  Right front drive motor:        "motor1"
 * Servo channel:                                  "arm"
 * Color sensor:                                   "colorSensor"
 */

/* Version history
 * ======= =======
 * v 0.1   Late Sept. 2020. Initial conversion of https://github
 * .com/FIRST-Tech-Challenge/FtcRobotController.
 * v.0.2   Early Oct. 2020. Pullbot robot class.
 * v 0.21  11/3/20  Vuforia-assisted approach to Blue Tower Goal.
 * v 1.0    11/14/20 Competition ready. Let's see if inheritance from
 *          GenericFTCRobot is correctly done. 11/16 It's not.
 * v 2.0  11/23/20 and later: Rebuilt from FtcRobotController, external
 *        samples, and pieces of failed FtcRobotController60.
 * v 2.1  11/28/20: added nudge methods and stick tempering.
 * v 2.2  12/1/20: improved nudging and simple driving.
 *        12/2/20 Feature freeze for UGScrimmage2.
 */

public class Pullbot extends GenericFTCRobot {

  // Vision properties
  public OpenCvInternalCamera2 phoneCam;
  public RingOrientationAnalysisPipeline pipeline;
  public static boolean PHONE_IS_PORTRAIT = false;
  int cameraMonitorViewId;
  public static final int tooFarRight = 100;
  public static final int tooHigh = 140;
  public static final int tooWide = 70;
  public static final int tooTall = 50;


  // Field related constants.
  public static final float mmPerInch = 25.4f; // use mm dimensions
  // Constants for perimeter Vuforia navigation targets
  // Field outside: 12'. Inside: 1" shorter than that, each Wall.
  public static final float fullField = 142 * mmPerInch;
  public static final float halfField = fullField / 2;
  public static final float quarterField = fullField / 4;
  // the height of the center of the target image above the floor
  public static final float mmTargetHeight = (6) * mmPerInch;

  /* Drive train. */
  // Drive train related constants in inches.
  // These next two are adjusted by calibration.
  public static final double DRIVE_WHEEL_SEPARATION = 13.0;
  public static final double DRIVE_WHEEL_DIAMETER = 3.5;
  // Fixed by drive train design. < 1.0: geared UP. > 1.0: geared DOWN.
  public static final double DRIVE_GEAR_REDUCTION = 1.0;
  public static final double COUNTS_PER_MOTOR_TURN = 1120; // REV HD hex 40:1
  static final double COUNTS_PER_INCH =
      (COUNTS_PER_MOTOR_TURN * DRIVE_GEAR_REDUCTION) /
          (DRIVE_WHEEL_DIAMETER * Math.PI);

  // Arm related properties
  public final double DEPLOYED = 1.0;   // arm extended in front of the Pullbot
  public final double STOWED = 0.0;     // arm retracted back over the Pullbot
  public DcMotorEx leftDrive = null;
  public DcMotorEx rightDrive = null;
  public Servo arm = null;

  // Pullbot specific sensor members.
  public ColorSensor colorSensor;
  public static VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
  /* local OpMode members. */

  // Initialization.
  HardwareMap hwMap = null;
  private LinearOpMode currentOpMode;
  private ElapsedTime period = new ElapsedTime();

  /* Constructors */
  public Pullbot() {
    super();
  }
  public Pullbot(LinearOpMode linearOpMode) {
    currentOpMode = linearOpMode;
  }

  public String init(HardwareMap someHWMap) {
    hwMap = someHWMap;
    String initializationReport = "";
    // Initialize vision hardware.
    colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

    int ringsDetected = 0;
    // Create camera instance
    cameraMonitorViewId =
        someHWMap.appContext.getResources().getIdentifier(
            "cameraMonitorViewId", "id",
            someHWMap.appContext.getPackageName());
    initializationReport += "Camera Id: " + cameraMonitorViewId;
    phoneCam =
        OpenCvCameraFactory.getInstance().createInternalCamera2
            (OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

    // Open async and start streaming inside opened callback
    phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
      @Override
      public void onOpened() {
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

        pipeline = new Pullbot.RingOrientationAnalysisPipeline();
        phoneCam.setPipeline(pipeline);
      }
    });
    initializationReport += "Camera is set up. Pipeline ";
    initializationReport += (pipeline == null) ? "empty.": "ready.";

    // Define and initialize motors. Stop them.
    leftDrive = hwMap.get(DcMotorEx.class, "motor0");
    rightDrive = hwMap.get(DcMotorEx.class, "motor1");
    leftDrive.setDirection(DcMotor.Direction.FORWARD);
    rightDrive.setDirection(DcMotor.Direction.REVERSE);
    leftDrive.setPower(0);
    rightDrive.setPower(0);

    // Set all motors to run without encoders.
    // May want to use RUN_USING_ENCODERS if encoders are installed.
    leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    // Define and initialize installed servos.
    arm = hwMap.get(Servo.class, "arm");

    return initializationReport;
  }

  /*
   *										Vision methods
   */

  static class RingOrientationAnalysisPipeline extends OpenCvPipeline {

    //   RGB colors.
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);
    //    Threshold values.
    static final int CB_CHAN_MASK_THRESHOLD = 110;
    // Todo: find threshold for  Blue Wobble Goal mast. A later Pullbot may be
    //  able to look for one and grab it.
    static final int CONTOUR_LINE_THICKNESS = 2;
    static final int CB_CHAN_IDX = 2;
    //    Our working image buffers.
    Mat cbMat = new Mat();
    Mat thresholdMat = new Mat();
    Mat morphedThreshold = new Mat();
    Mat contoursOnPlainImageMat = new Mat();
    //   The elements we use for noise reduction.
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
        new Size(3, 3));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
        new Size(6, 6));
    ArrayList<AnalyzedRing> internalRingList = new ArrayList<>();
    volatile ArrayList<AnalyzedRing> clientRingList = new ArrayList<>();
    Stage[] stages = values();
    //   Currently displayed stage buffer.
    int stageNum = 0;

    static void drawRotatedRect(RotatedRect rect, Mat drawOn) {
      //   Draws a rotated rect by drawing each of the 4 lines individually.
      Point[] points = new Point[4];
      rect.points(points);

      for (int i = 0; i < 4; ++i) {
        Imgproc.line(drawOn, points[i], points[(i + 1) % 4], GREEN, 2);
      }
    }

    @Override
    public Mat processFrame(Mat input) {
      // We'll be updating input with new data below.
      internalRingList.clear();

      //   Process the image.
      for (MatOfPoint contour : findContours(input)) {
        analyzeContour(contour, input);
      }
      clientRingList = new ArrayList<>(internalRingList);
      return input;
    }

    public ArrayList<AnalyzedRing> getDetectedRings() {
      return clientRingList;
    }

    ArrayList<MatOfPoint> findContours(Mat input) {
      // A list we'll be using to store the contours we find.
      ArrayList<MatOfPoint> contoursList = new ArrayList<>();

      // Convert the input image to YCrCb color space, then extract the Cb
      // channel.
      Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
      Core.extractChannel(cbMat, cbMat, CB_CHAN_IDX);

      // Threshold the Cb channel to form a mask, then run some noise reduction.
      Imgproc.threshold(cbMat, thresholdMat, CB_CHAN_MASK_THRESHOLD, 255,
          Imgproc.THRESH_BINARY_INV);
      morphMask(thresholdMat, morphedThreshold);

      // Ok, now actually look for the contours! We only look for external
      // contours.
      Imgproc.findContours(morphedThreshold, contoursList, new Mat(),
          Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

      // We do draw the contours we find, but not to the main input buffer.
      input.copyTo(contoursOnPlainImageMat);
      Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1,
          BLUE, CONTOUR_LINE_THICKNESS, 8);

      return contoursList;
    }

    void morphMask(Mat input, Mat output) {
      //   Noise reduction.
      Imgproc.erode(input, output, erodeElement);
      Imgproc.erode(output, output, erodeElement);
      Imgproc.dilate(output, output, dilateElement);
      Imgproc.dilate(output, output, dilateElement);
    }

    void analyzeContour(MatOfPoint contour, Mat input) {
      boolean isMaybeRing = true;
      AnalyzedRing analyzedRing = new AnalyzedRing();
      //   Transform the contour to a different format.
      Point[] points = contour.toArray();
      MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

      //   Do a rect fit to the contour, and draw it on the screen.
      RotatedRect rotatedRectFitToContour =
          Imgproc.minAreaRect(contour2f);
      drawRotatedRect(rotatedRectFitToContour, input);
      analyzedRing.width = (int) rotatedRectFitToContour.size.width;
      analyzedRing.aspectRatio = rotatedRectFitToContour.size.width /
          rotatedRectFitToContour.size.height;
      analyzedRing.top = rotatedRectFitToContour.boundingRect().y;
      analyzedRing.left = rotatedRectFitToContour.boundingRect().x;
      analyzedRing.height = rotatedRectFitToContour.boundingRect().height;
      //   Throw out "Rings" not in proper position.
      if (analyzedRing.top < tooHigh) isMaybeRing = false;
      if (analyzedRing.left > tooFarRight) isMaybeRing = false;
      if (analyzedRing.width > tooWide) isMaybeRing = false;
      if (analyzedRing.height > tooTall) isMaybeRing = false;
      // TODO: consolidate these with filter code in CountRings.
      if (isMaybeRing) {
        internalRingList.add(analyzedRing);
        // The angle OpenCV gives us can be ambiguous, so look at the shape of
        // the rectangle to fix that.
        double rotRectAngle = rotatedRectFitToContour.angle;
        if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height) {
          rotRectAngle += 90;
        }
      }
    }

    //   Pipeline processing stages. Different image buffers are available at
    //   each one.
    enum Stage {
      FINAL,
      Cb,
      MASK,
      MASK_NR,
      CONTOURS
    }

    static class AnalyzedRing {
      double aspectRatio;
      int top;
      int left;
      int height;
      int width;
    }
  }
  int CountRings (int viewID){
    int ringsDetected = 0;
    init(); // prolly don't have to initialize everything.

    phoneCam =
        OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, viewID);

    // Open async and start streaming inside opened callback
    phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
      @Override
      public void onOpened() {
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
        pipeline = new RingOrientationAnalysisPipeline();
        phoneCam.setPipeline(pipeline);
      }
    });

    sleep(20);
    if (pipeline == null) return -2;
    // Report detected rectangles to telemetry. There should be only 1.
    // Todo: check this assertion.
    ArrayList<RingOrientationAnalysisPipeline.AnalyzedRing> rings =
        pipeline.getDetectedRings();
    if (rings.isEmpty()) {
      // ringsDetected will be left at zero.
      currentOpMode.telemetry.addLine("No rings detected.");
    } else {
      for (RingOrientationAnalysisPipeline.AnalyzedRing ring :
          rings) {
        if (ring.left > 100) continue;
        if (ring.top < 0) continue;
        if (ring.width > 100) continue;
        if (ring.height > 60 ) continue;
        currentOpMode.telemetry.addLine(String.format(Locale.US,
            "Ring aspect ratio = %.2f.",
            ring.aspectRatio));
        currentOpMode.telemetry.addLine(String.format(Locale.US,
            "Ring top = %2d  left = %2d.",
            ring.top, ring.left));
        currentOpMode.telemetry.addLine(String.format(Locale.US,
            "Ring width = %2d  height=%2d.",
            ring.width, ring.height));
        if (ring.aspectRatio > 1 && ring.aspectRatio <= 2) ringsDetected = 4;
        if (ring.aspectRatio > 2 && ring.aspectRatio <= 4) ringsDetected = 1;
        currentOpMode.telemetry.addLine(String.format(Locale.US,
            "Rings detected = %2d.",
            ringsDetected));
      }
    }
    currentOpMode.telemetry.update();

    return ringsDetected;
  }

  /*
   *										Drive Train methods
   */
  private double NUDGE_SPEED = 0.30;
  private double NUDGE_INCHES = 1.0;

  /*                      Primitive layer.                    */
  // Task layer methods are built up out of members at this layer.
  private double temperedControl (double input) {
    return Math.pow(input, 3.0);
  }

  public void setDriveRunMode(DcMotor.RunMode someRunMode) {
    leftDrive.setMode(someRunMode);
    rightDrive.setMode(someRunMode);
  }

  // Set both drive motors to some behavior when they're told to stop.
  public void setDriveStopBehavior(DcMotor.ZeroPowerBehavior someBehavior) {
    leftDrive.setZeroPowerBehavior(someBehavior);
    rightDrive.setZeroPowerBehavior(someBehavior);
  }

  public void moveMotor (DcMotor motor, double speed, double inches) {
    int newTarget;

    //  Discard current encoder positions.
    //setDriveStopBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // Determine new target position, and pass to motor controller.
    // Negative target because Pullbot motors pull, not push.
    newTarget = (int) (-inches * COUNTS_PER_INCH);
    motor.setTargetPosition(newTarget);

    // Turn On RUN_TO_POSITION
    setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

    // Go!
    motor.setPower(Math.abs(speed));

    // keep looping while we are still active, and both motors are running.
    while (motor.isBusy()) {
      // Wait until motor is done before doing anything else.
    }
    // Clean up, prepare for next segment.
    motor.setPower(0);

    // Turn off RUN_TO_POSITION
    //leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  public void encoderDrive(double leftSpeed, double rightSpeed,
                           double leftInches, double rightInches) {
    // Todo: can two calls to moveMotor work here?
    int newLeftTarget;
    int newRightTarget;

    //  Discard current encoder positions.
    setDriveStopBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // Determine new target positions, and pass to motor controller.
    // Negative targets because Pullbot motors pull, not push.
    newLeftTarget = (int) (-leftInches * COUNTS_PER_INCH);
    newRightTarget = (int) (-rightInches * COUNTS_PER_INCH);
    leftDrive.setTargetPosition(newLeftTarget);
    rightDrive.setTargetPosition(newRightTarget);

    // Turn On RUN_TO_POSITION
    setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

    // Go!
    leftDrive.setPower(Math.abs(leftSpeed));
    rightDrive.setPower(Math.abs(rightSpeed));

    // keep looping while we are still active, and both motors are running.
    while (leftDrive.isBusy() && rightDrive.isBusy()) {
      // Wait until motors done before doing anything else.
    }
    // Clean up, prepare for next segment.
    leftDrive.setPower(0);
    rightDrive.setPower(0);

    // Turn off RUN_TO_POSITION.
    // Todo: is this right? Does not agree with comment.
    leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  ElapsedTime runtime = new ElapsedTime();

  // Sigmoid profile for change of speed on a single motor. Todo: test.
  int changeSpeedSigmoid (double time2DoIt, double startSpeed, double endSpeed,
                          DcMotor someMotor)
  {
    int Counts = 0;
    double time;
    double power;
    double powerScale = endSpeed - startSpeed;
    do {
      time = runtime.time();
      power = startSpeed + powerScale * (0.5 - 0.5 * Math.cos(Math.PI * time / time2DoIt));
      someMotor.setPower(power);
    } while (time < time2DoIt);
    Counts = someMotor.getCurrentPosition();
    return Counts;
  }

  /*                         Task layer.                    */
  // Opmodes are built up out of methods at this layer.

  //  This one requires no command layer to hardware layer translation.
  //  Just continue going straight.
  public void continueStraight(double speed) {
    leftDrive.setPower(speed);
    rightDrive.setPower(speed);
  }

  //   Simple wrapper for encoderDrive. Just go straight a number of inches.
  public void driveStraight(double speed, double inches) {
    encoderDrive(speed, speed, inches, inches);
  }

  //   Turn on axis, as though with left and right tank drive joysticks in
  //   equal but opposite deflection.
  public void turnAngle(double speed, double angle) { // angle in radians
    double inches = angle * DRIVE_WHEEL_SEPARATION / 2;
    encoderDrive(speed, speed, -inches, inches);
  }

  /*  Turning movements. All angles are in radians. */
  //  Turn at speed through an angle, with a given radius.
  public void turnAngleRadiusDrive(double speed, double angle,
                                   double radius) {

    // One or both turning arcs could be negative.
    // Degenerate cases: angle = 0, R = 0, R = d/2, R = +infinity (straight
    // drive).
    // Calculate 2 target distances, 2 speeds. Then feed 'em to
    // encoderDrive.
    // Arc lengths are angle * radius adjusted for the drive wheels: one
    // shorter, the other longer. Speeds need to be adjusted as well.
    // TODO: handle 4 quadrant cases: forward CW, forward CCW, back CW,
    //  back CCW
    // ** Note negative angle enforces backward movement. What would
    // negative angle and negative radius do?
    double leftRadius = radius - DRIVE_WHEEL_SEPARATION / 2.0;
    double rightRadius = radius + DRIVE_WHEEL_SEPARATION / 2.0;
    double turnAdjustLeft = leftRadius / radius;
    double turnAdjustRight = rightRadius / radius;
    double leftArc = leftRadius * angle;
    double rightArc = rightRadius * angle;
    double leftSpeed = speed * turnAdjustLeft;
    double rightSpeed = speed * turnAdjustRight;

    encoderDrive(leftSpeed, rightSpeed, leftArc, rightArc);
  }

  //    Wrapper for turnAngleRadius
  // TODO make this a wrapper for encoderDrive instead.
  public void turnArcRadiusDrive(double speed, double arc, double radius) {
    double targetAngle = arc / radius;
    turnAngleRadiusDrive(speed, targetAngle, radius);
  }

  // TurnAngleArc not implemented.

  //  Begin a left turn at speed, sharpness of turn decided by ratio.
  // TODO Test on Scrimmage2 build.
  //    1:  go straight.
  //    0:  turn axis is left wheel.
  //    -1: turn axis is between drive wheels. Robot turns on own axis.
  public void steerLeft(double speed, double ratio) {
    Range.clip(ratio, -1.0, 1.0);
    leftDrive.setPower(speed * ratio);
    rightDrive.setPower(speed);
  }

  //  Right analog of steerLeft.
  public void steerRight(double speed, double ratio) {
    Range.clip(ratio, -1.0, 1.0);
    leftDrive.setPower(speed);
    rightDrive.setPower(speed * ratio);
  }

  //  Drive a curved path by making left wheels turn slower and go
  //    shorter path by a factor of ratio. The right wheels will spin
  //    at parameter speed, and travel the full arc.
  public void turnLeft(double speed, double ratio, double arcInches) {
    Range.clip(ratio, -1.0, 1.0);
    encoderDrive(speed * ratio, speed,
        arcInches * ratio, arcInches);
  }

  //  Right analog of turnLeft.
  public void turnRight(double speed, double ratio, double arcInches) {
    Range.clip(ratio, -1.0, 1.0);
    encoderDrive(speed, speed * ratio,
        arcInches, arcInches * ratio);
  }


  /*                      Command layer.                    */
  // Human driver issues commands with gamepad.
  public void enableNudge () {
    // Gamepad mapping is similar to tank drive.
    if (currentOpMode.gamepad1.left_trigger > 0){
      // nudge left wheel forward a little
      //moveMotor(leftDrive, NUDGE_SPEED, NUDGE_INCHES);
      leftDrive.setPower (-NUDGE_SPEED);
    }
    if (currentOpMode.gamepad1.right_trigger > 0){
      // nudge right wheel forward a little

      rightDrive.setPower(-NUDGE_SPEED);
    }
    if (currentOpMode.gamepad1.left_bumper){
      // nudge left wheel back a little
      leftDrive.setPower (NUDGE_SPEED);
    }
    if (currentOpMode.gamepad1.right_bumper){
      // nudge right wheel back a little
      rightDrive.setPower (NUDGE_SPEED);
    }
  }

  public void tankDrive() {
    //  Tank drive with the two sticks.
    double leftSpeed = currentOpMode.gamepad1.left_stick_y;
    double rightSpeed = currentOpMode.gamepad1.right_stick_y;
    leftDrive.setPower(temperedControl(leftSpeed));
    rightDrive.setPower(temperedControl(rightSpeed));
  }

  public void simpleDrive() {
    //  Left stick for fore-and-aft, right one for turns.
    double drive = currentOpMode.gamepad1.left_stick_y;
    double turn  = currentOpMode.gamepad1.right_stick_x/2.0;
    leftDrive.setPower(temperedControl(drive - turn));
    rightDrive.setPower(temperedControl(drive + turn));
  }

  // Macros can go here. Most will be used in the opmodes.
}
