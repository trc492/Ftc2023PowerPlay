/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import TrcCommonLib.trclib.TrcOpenCvColorBlobPipeline;
import TrcCommonLib.trclib.TrcOpenCvDetector;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcFtcLib.ftclib.FtcEocvAprilTagPipeline;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcTensorFlow;
import TrcFtcLib.ftclib.FtcVuforia;

/**
 * This class implements Vuforia/TensorFlow/Grip/Eocv Vision for the game season. It creates and initializes all the
 * vision target info as well as providing info for the robot, camera and the field. It also provides methods to get
 * the location of the robot and detected targets.
 */
public class Vision
{
    public static final String OPENCV_NATIVE_LIBRARY_NAME = "EasyOpenCV";
    public static final String[] TARGET_LABELS = {
        BlinkinLEDs.LABEL_BOLT, BlinkinLEDs.LABEL_BULB, BlinkinLEDs.LABEL_PANEL
    };

    private final Robot robot;
    public VuforiaVision vuforiaVision;
    public TensorFlowVision tensorFlowVision;
    public EocvVision frontEocvVision;
    public EocvVision elevatorEocvVision;

    private int lastSignal = 0;

    /**
     * Constructor: Create an instance of the object. Vision is required by both Vuforia and TensorFlow and must be
     * instantiated if either is used. However, to use either Vuforia or TensorFlow, one must explicitly initialize
     * them by calling the initVuforia or initTensorFlow methods respectively.
     *
     * @param robot specifies the robot object.
     */
    public Vision(Robot robot)
    {
        FtcOpMode opMode = FtcOpMode.getInstance();
        int cameraViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        this.robot = robot;
        if (RobotParams.Preferences.useEasyOpenCV)
        {
            OpenCvWebcam frontWebcam = null, elevatorWebcam = null;

            if (RobotParams.Preferences.showEasyOpenCvView)
            {
                int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(
                    cameraViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);

                if (RobotParams.Preferences.useFrontWebcam)
                {
                    frontWebcam =
                        OpenCvCameraFactory.getInstance().createWebcam(
                            opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_FRONT_WEBCAM),
                            viewportContainerIds[0]);
                    frontWebcam.showFpsMeterOnViewport(false);
                }

                if (RobotParams.Preferences.useElevatorWebcam)
                {
                    elevatorWebcam =
                        OpenCvCameraFactory.getInstance().createWebcam(
                            opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_ELEVATOR_WEBCAM),
                            viewportContainerIds[1]);
                    elevatorWebcam.showFpsMeterOnViewport(false);
                }
            }
            else
            {
                if (RobotParams.Preferences.useFrontWebcam)
                {
                    frontWebcam =
                        OpenCvCameraFactory.getInstance().createWebcam(
                            opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_FRONT_WEBCAM));
                }

                if (RobotParams.Preferences.useElevatorWebcam)
                {
                    elevatorWebcam =
                        OpenCvCameraFactory.getInstance().createWebcam(
                            opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_ELEVATOR_WEBCAM));
                }
            }
            // EOCV sometimes timed out on opening the camera. The default timeout was 2 seconds. It seems setting
            // it to 3 seconds would do wonder here.
            robot.globalTracer.traceInfo("Vision", "Starting EocvVision...");

            if (frontWebcam != null)
            {
                frontWebcam.setMillisecondsPermissionTimeout(RobotParams.FRONTCAM_PERMISSION_TIMEOUT);
                frontEocvVision = new EocvVision(
                    "frontEocvVision", RobotParams.FRONTCAM_IMAGE_WIDTH, RobotParams.FRONTCAM_IMAGE_HEIGHT,
                    RobotParams.cameraRect, RobotParams.worldRect, frontWebcam, OpenCvCameraRotation.UPRIGHT, null);
            }

            if (elevatorWebcam != null)
            {
                elevatorWebcam.setMillisecondsPermissionTimeout(RobotParams.ELEVATORCAM_PERMISSION_TIMEOUT);
                elevatorEocvVision = new EocvVision(
                    "elevatorEocvVision", RobotParams.ELEVATORCAM_IMAGE_WIDTH, RobotParams.ELEVATORCAM_IMAGE_HEIGHT,
                    null, null, elevatorWebcam, OpenCvCameraRotation.SIDEWAYS_RIGHT, null);
            }
        }
        else if (RobotParams.Preferences.useVuforia || RobotParams.Preferences.useTensorFlow)
        {
            final String VUFORIA_LICENSE_KEY =
                "ARbBwjf/////AAABmZijKPKUWEY+uNSzCuTOUFgm7Gr5irDO55gtIOjsOXmhLzLEILJp45qdPrwMfoBV2Yh7F+Wh8iEjnSA" +
                "NnnRKiJNHy1T9Pr2uufETE40YJth10Twv0sTNSEqxDPhg2t4PJXwRImMaEsTE53fmcm08jT9qMso2+1h9eNk2b4x6DVKgBt" +
                "Tv5wocDs949Gkh6lRt5rAxATYYO9esmyKyfyzfFLMMpfq7/uvQQrSibNBqa13hJRmmHoM2v0Gfk8TCTTfP044/XsOm54u8k" +
                "dv0HfeMBC91uQ/NvWHVV5XCh8pZAzmL5sry1YwG8FSRNVlSAZ1zN/m6jAe98q6IxpwQxP0da/TpJoqDI7x4RGjOs1Areunf";
            //
            // If no camera view ID, do not activate camera monitor view to save power.
            //
            VuforiaLocalizer.Parameters vuforiaParams =
                RobotParams.Preferences.showVuforiaView?
                    new VuforiaLocalizer.Parameters(cameraViewId): new VuforiaLocalizer.Parameters();

            vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
            vuforiaParams.cameraName = opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_FRONT_WEBCAM);
            vuforiaParams.useExtendedTracking = false;
            vuforiaParams.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
            FtcVuforia vuforia = new FtcVuforia(vuforiaParams);

            vuforiaVision = RobotParams.Preferences.useVuforia? new VuforiaVision(vuforia, robot.blinkin): null;
            tensorFlowVision = RobotParams.Preferences.useTensorFlow? new TensorFlowVision(vuforia, null): null;
        }
    }   //Vision

    /**
     * This method shuts down TensorFlow.
     */
    public void tensorFlowShutdown()
    {
        if (tensorFlowVision != null)
        {
            tensorFlowVision.shutdown();
            tensorFlowVision = null;
        }
    }   //tensorFlowShutdown

    /**
     * This method calls vision to detect the signal and returns the detected info.
     *
     * @return detected signal info, null if none detected.
     */
    public TrcVisionTargetInfo<?> getDetectedSignalInfo()
    {
        TrcVisionTargetInfo<?>[] targets = null;

        if (tensorFlowVision != null && tensorFlowVision.isEnabled())
        {
            targets = tensorFlowVision.getDetectedTargetsInfo(
                null, null, this::compareConfidence,
                RobotParams.APRILTAG_HEIGHT_OFFSET, RobotParams.FRONTCAM_HEIGHT_OFFSET);
        }
        else if (frontEocvVision != null && frontEocvVision.isEnabled() &&
                 frontEocvVision.getDetectObjectType() == EocvVision.ObjectType.APRIL_TAG)
        {
            targets = frontEocvVision.getDetectedTargetsInfo(
                null, null, RobotParams.APRILTAG_HEIGHT_OFFSET, RobotParams.FRONTCAM_HEIGHT_OFFSET);
        }

        return targets != null? targets[0]: null;
    }   //getDetectedSignalInfo

    /**
     * This method determines the signal value from the detected object info.
     *
     * @return signal value of the detected object, 0 if no detected object.
     */
    public int determineDetectedSignal(TrcVisionTargetInfo<?> target)
    {
        int detectedSignal = 0;

        if (target != null)
        {
            if (tensorFlowVision != null)
            {
                FtcTensorFlow.DetectedObject detectedObj = (FtcTensorFlow.DetectedObject) target.detectedObj;

                switch (detectedObj.label)
                {
                    case BlinkinLEDs.LABEL_BOLT:
                        detectedSignal = 1;
                        break;

                    case BlinkinLEDs.LABEL_BULB:
                        detectedSignal = 2;
                        break;

                    case BlinkinLEDs.LABEL_PANEL:
                        detectedSignal = 3;
                        break;
                }
            }
            else if (frontEocvVision != null && frontEocvVision.getPipeline() instanceof FtcEocvAprilTagPipeline)
            {
                FtcEocvAprilTagPipeline.DetectedObject detectedObj =
                    (FtcEocvAprilTagPipeline.DetectedObject) target.detectedObj;

                detectedSignal = detectedObj.object.id;
            }
        }

        if (detectedSignal != 0)
        {
            lastSignal = detectedSignal;
            if (robot.blinkin != null)
            {
                // Turn off previous detection indication.
                robot.blinkin.setPatternState(BlinkinLEDs.LABEL_BOLT, false);
                robot.blinkin.setPatternState(BlinkinLEDs.LABEL_BULB, false);
                robot.blinkin.setPatternState(BlinkinLEDs.LABEL_PANEL, false);

                switch (detectedSignal)
                {
                    case 1:
                        robot.blinkin.setPatternState(BlinkinLEDs.LABEL_BOLT, true, 1.0);
                        break;

                    case 2:
                        robot.blinkin.setPatternState(BlinkinLEDs.LABEL_BULB, true, 1.0);
                        break;

                    case 3:
                        robot.blinkin.setPatternState(BlinkinLEDs.LABEL_PANEL, true, 1.0);
                        break;
                }
                robot.dashboard.displayPrintf(15, "Found the signal at %d", detectedSignal);
            }
        }

        return detectedSignal;
    }   //determineDetectedSignal

    /**
     * This method calls the appropriate vision detection to detect the signal position.
     *
     * @return detected signal position, 0 if none detected.
     */
    public int getDetectedSignal()
    {
        return determineDetectedSignal(getDetectedSignalInfo());
    }   //getDetectedSignal

    /**
     * This method does not initiate detection. It simply returns the signal detected by the last call to
     * getDetectedSignal. This is typically used to get the last detected signal during the init period.
     *
     * @return last detected signal.
     */
    public int getLastSignal()
    {
        return lastSignal;
    }   //getLastSignal

    /**
     * This method calls vision to detect the cone and returns the detected info.
     *
     * @return detected cone info, null if none detected.
     */
    public TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> getDetectedConeInfo()
    {
        TrcVisionTargetInfo<?>[] targets = null;

        if (frontEocvVision != null && frontEocvVision.isEnabled())
        {
            EocvVision.ObjectType detectObjType = frontEocvVision.getDetectObjectType();

            if (detectObjType == EocvVision.ObjectType.RED_CONE || detectObjType == EocvVision.ObjectType.BLUE_CONE)
            {
                targets = frontEocvVision.getDetectedTargetsInfo(null, this::compareBottomY, 0.0, 0.0);

                if (targets != null && robot.blinkin != null)
                {
                    robot.blinkin.setPatternState(
                        detectObjType == EocvVision.ObjectType.RED_CONE?
                            BlinkinLEDs.GOT_RED_CONE: BlinkinLEDs.GOT_BLUE_CONE,
                        true, 1.0);
                }
            }
        }

        return targets != null? (TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject>) targets[0]: null;
    }   //getDetectedConeInfo

    /**
     * This method calls vision to detect the pole and returns the detected info.
     *
     * @return detected pole info, null if none detected.
     */
    public TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> getDetectedPoleInfo()
    {
        TrcVisionTargetInfo<?>[] targets = null;

        if (elevatorEocvVision != null && elevatorEocvVision.isEnabled())
        {
            targets = elevatorEocvVision.getDetectedTargetsInfo(null, null, 0.0, 0.0);

            if (targets != null && robot.blinkin != null)
            {
                robot.blinkin.setPatternState(BlinkinLEDs.GOT_YELLOW_POLE, true, 1.0);
            }
        }

        return targets != null? (TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject>) targets[0]: null;
    }   //getDetectedPoleInfo

    /**
     * This method returns the angle of the detected pole.
     *
     * @return detected pole angle.
     */
    public Double getPoleAngle()
    {
        Double poleAngle = null;
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> poleInfo = getDetectedPoleInfo();

        if (poleInfo != null)
        {
            poleAngle = poleInfo.distanceFromImageCenter.x * RobotParams.ELEVATORCAM_ANGLE_PER_PIXEL;
        }

        return poleAngle;
    }   //getPoleAngle
    /**
     * This method returns the angle of the detected pole.
     *
     * @return detected pole angle.
     */
    public Double getConeAngle()
    {
        Double coneAngle = null;
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> coneInfo = getDetectedPoleInfo();

        if (coneInfo != null)
        {
            coneAngle = coneInfo.distanceFromImageCenter.x * RobotParams.ELEVATORCAM_ANGLE_PER_PIXEL;
        }

        return coneAngle;
    }   //getPoleAngle
    /**
     * This method is called by the Arrays.sort to sort the target object by decreasing confidence.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has higher confidence than b, 0 if a and b have equal confidence, positive value
     *         if a has lower confidence than b.
     */
    private int compareConfidence(
        TrcVisionTargetInfo<FtcTensorFlow.DetectedObject> a, TrcVisionTargetInfo<FtcTensorFlow.DetectedObject> b)
    {
        return (int)((b.detectedObj.confidence - a.detectedObj.confidence)*100);
    }   //compareConfidence

    /**
     * This method is called by the Arrays.sort to sort the target object by increasing object distance.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has smaller distance to image center than b, 0 if a and b have equal distance to
     * image center, positive value if a has larger distance to image center than b.
     */
    private int compareDistanceFromCamera(TrcVisionTargetInfo<?> a, TrcVisionTargetInfo<?> b)
    {
        return (int)((Math.abs(a.distanceFromCamera.y) - Math.abs(b.distanceFromCamera.y))*1000);
    }   //compareDistanceFromCamera

    /**
     * This method is called by the Arrays.sort to sort the target object by increasing camera angle.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has smaller camera angle than b, 0 if a and b have equal camera angle, positive
     *         value if a has larger camera angle than b.
     */
    private int compareCameraAngle(TrcVisionTargetInfo<?> a, TrcVisionTargetInfo<?> b)
    {
        return (int)((Math.abs(a.horizontalAngle) - Math.abs(b.horizontalAngle))*1000);
    }   //compareCameraAngle

    /**
     * This method is called by the Arrays.sort to sort the target object by decreasing object size.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has smaller area than b, 0 if a and b have equal area, positive value if a has
     *         larger area than b.
     */
    private int compareObjectSize(
        TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> a,
        TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> b)
    {
        return (int)((a.detectedObj.getArea() - b.detectedObj.getArea())*1000);
    }   //compareObjectSize

    /**
     * This method is called by the Arrays.sort to sort the target object by decreasing bottom Y.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has smaller bottom Y than b, 0 if a and b have equal bottom Y,
     *         positive value if a has larger bottom Y than b.
     */
    private int compareBottomY(
            TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> a,
            TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> b)
    {
        Rect aRect = a.detectedObj.getRect();
        Rect bRect = b.detectedObj.getRect();

        return (bRect.y + bRect.height) - (aRect.y + aRect.height);
    }   //compareBottomY

}   //class Vision
