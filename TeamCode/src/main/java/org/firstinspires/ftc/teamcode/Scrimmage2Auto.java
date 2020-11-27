/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 *  all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.ArrayList;
import java.util.Locale;

/*
 *   Looks for yellow rectangles, and reports their aspect ratio. It will be
 * used in Ultimate Goal to count Rings in the Starter Stack.
 * o A case: no rectangles means no Rings. Put Wobble Goal in Zone A.
 * o B case: rectangle with short squat aspect ratio, 4 - 8. Go for Zone B.
 * o C case: taller rectangle, ratio 1-2. Go for Zone C.
 */

@Autonomous(name = "Ring Detect Go Zone", group = "Vision test")
//@Disabled
public class Scrimmage2Auto extends LinearOpMode {

  Pullbot robot = new Pullbot(this);
  String initReport = "";
  boolean doPaths = true;

  @Override
  public void runOpMode() {
    initReport = robot.init(hardwareMap);

    // Tell telemetry to update faster than the default 250ms period :)
    telemetry.setMsTransmissionInterval(100);
    telemetry.addLine (initReport);
    telemetry.update();
    sleep (3000);

    double straightSpeed = 0.60;
    double turnSpeed = 0.30;
    waitForStart();

    int ringsDetected = 0;
    ArrayList<Pullbot.RingOrientationAnalysisPipeline.AnalyzedRing> rings =
        robot.pipeline.getDetectedRings();
    if (rings.isEmpty()) {
      // ringsDetected will be left at zero.
      telemetry.addLine("No rings detected.");
    } else {
      for (Pullbot.RingOrientationAnalysisPipeline.AnalyzedRing ring :
          rings) {
        telemetry.addLine(String.format(Locale.US,
            "Ring aspect ratio = %.2f.",
            ring.aspectRatio));
        telemetry.addLine(String.format(Locale.US,
            "Ring top = %2d  left = %2d.",
            ring.top, ring.left));
        telemetry.addLine(String.format(Locale.US,
            "Ring width = %2d  height=%2d.",
            ring.width, ring.height));
        if (ring.aspectRatio > 1 && ring.aspectRatio <= 2) ringsDetected = 4;
        if (ring.aspectRatio > 2 && ring.aspectRatio <= 4) ringsDetected = 1;
        telemetry.addLine(String.format(Locale.US,
            "Rings detected = %2d.",
            ringsDetected));
      }
    }

    switch (ringsDetected) {
      case (0): {
        telemetry.addLine("Going to Zone A.");
        telemetry.update();
        //telemetry.speak("Ziel: der A Zone.", "deutsch", "de");
        // Henry and Orlando's code here.
        //if (doPaths) robot.turnAngleRadiusDrive(turnSpeed, 1, 80.0);
        // Mine, translated to turnArcRadius from theirs:
        if (doPaths) robot.turnArcRadiusDrive (turnSpeed, 84.0, 80.0);
        // no backward move to park on Launch Line needed; robot ends up
        // there on pushing the Wobble Goal.
        telemetry.addLine("A path Complete.");
        telemetry.update();
        break;
      }
      case (1): {
        telemetry.addLine("Going to Zone B.");
        telemetry.update();
        // Henry and Orlando's code here.
        //if (doPaths) robot.turnAngleRadiusDrive(turnSpeed, 0.6, 150.0);
        //if (doPaths) robot.turnAngleRadiusDrive(turnSpeed, -0.07, 150.0);
        // Mine, translated to turnArcRadius from theirs:
        if (doPaths) robot.turnArcRadiusDrive (turnSpeed, 96.0, 240.0);
        if (doPaths) robot.turnArcRadiusDrive (turnSpeed, -20.0, 240.0);
        telemetry.addLine("B path Complete.");
        telemetry.update();
        break;
      }
      case (4): {
        telemetry.addLine("Going to Zone C.");
        telemetry.update();
        // My code here.
        //if (doPaths) robot.turnAngleRadiusDrive(turnSpeed, 0.82, 150.0);
        //if (doPaths) robot.turnAngleRadiusDrive(turnSpeed, -0.32, 150.0);
        // Mine, translated to turnArcRadius from theirs:
        if (doPaths) robot.turnArcRadiusDrive (turnSpeed, 123.0, 150.0);
        if (doPaths) robot.turnArcRadiusDrive (turnSpeed, -46.0, 150.0);
        telemetry.addLine("C path Complete.");
        telemetry.update();
        break;
      }
      default:
        telemetry.addLine(
            "I'm lost. Going to Zone A, and hoping for the best.");
        ringsDetected = 0;
        if (doPaths) robot.turnArcRadiusDrive(turnSpeed, 84.0, 80.0);
    }

    telemetry.update();
    sleep (2000);

  }
}