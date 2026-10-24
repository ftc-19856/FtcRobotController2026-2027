package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.RobotConfig;

/** Read-only vision setup: does not map or command any mechanism or drive motor. */
@TeleOp(name = "Hive Vision Setup", group = "Test")
public final class HiveVisionSetup extends OpMode {
    private Limelight3A limelight;
    private int requestedPipeline;
    private boolean previousUp;
    private boolean previousDown;
    private boolean switchAccepted;
    private double timestampAtSwitch = Double.NaN;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, RobotConfig.LIMELIGHT);
        limelight.setPollRateHz(50);
        selectPipeline(RobotConfig.POLLEN_PIPELINE_INDEX);
        limelight.start();
        telemetry.setMsTransmissionInterval(100);
        showVision();
    }

    @Override
    public void init_loop() {
        updatePipelineSelection();
        showVision();
    }

    @Override
    public void loop() {
        updatePipelineSelection();
        showVision();
    }

    private void updatePipelineSelection() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        if (up && !previousUp && !down && requestedPipeline < 9) {
            selectPipeline(requestedPipeline + 1);
        } else if (down && !previousDown && !up && requestedPipeline > 0) {
            selectPipeline(requestedPipeline - 1);
        }
        previousUp = up;
        previousDown = down;
    }

    private void selectPipeline(int pipeline) {
        LLResult previous = limelight.getLatestResult();
        timestampAtSwitch = previous == null ? Double.NaN : previous.getTimestamp();
        requestedPipeline = pipeline;
        switchAccepted = limelight.pipelineSwitch(pipeline);
    }

    private void showVision() {
        telemetry.addLine("D-pad UP/DOWN: select pipeline 0-9. No motors or servos are commanded.");
        telemetry.addLine("Record readings with each hive DOWN and UP from the same position.");
        telemetry.addData("Requested pipeline", requestedPipeline);
        telemetry.addData("Switch request accepted", switchAccepted);
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            telemetry.addLine("Waiting for camera data");
        } else if (!switchAccepted || result.getPipelineIndex() != requestedPipeline
                || result.getTimestamp() == timestampAtSwitch) {
            telemetry.addData("Reported pipeline", result.getPipelineIndex());
            telemetry.addLine("Waiting for a new frame from the requested pipeline");
        } else if (result.getStaleness() > 150) {
            telemetry.addData("Stale result (ms)", result.getStaleness());
        } else {
            telemetry.addData("Pipeline type", result.getPipelineType());
            telemetry.addData("Valid target", result.isValid());
            telemetry.addData("Frame age (ms)", result.getStaleness());
            if (result.isValid()) {
                telemetry.addData("Main target tx / ty (deg)", "%.2f / %.2f",
                        result.getTx(), result.getTy());
                telemetry.addData("Main target area (%)", "%.3f", result.getTa());
                java.util.List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                telemetry.addData("AprilTags seen", tags == null ? 0 : tags.size());
                if (tags != null) {
                    for (LLResultTypes.FiducialResult tag : tags) {
                        String label = "Tag " + tag.getFiducialId();
                        telemetry.addData(label + " tx / ty (deg)", "%.2f / %.2f",
                                tag.getTargetXDegrees(), tag.getTargetYDegrees());
                        telemetry.addData(label + " area (%)", "%.3f", tag.getTargetArea());
                        Pose3D cameraPose = tag.getTargetPoseCameraSpace();
                        if (cameraPose != null && cameraPose.getPosition() != null) {
                            Position p = cameraPose.getPosition().toUnit(DistanceUnit.INCH);
                            telemetry.addData(label + " camera-space xyz (in)", "%.1f / %.1f / %.1f",
                                    p.x, p.y, p.z);
                        }
                    }
                }
            }
            telemetry.addLine("3D values require correct tag size and 3D pipeline setup.");
            telemetry.addLine("A visible AprilTag alone does not establish that a hive is down.");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
    }
}
