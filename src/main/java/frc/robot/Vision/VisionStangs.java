
package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Stable.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionStangs {
    private PhotonCamera camera;
    PhotonPoseEstimator ppe;
    private Pose2d currentPose;
    private double bigAmbiguity;
    private boolean isLL;
    private double timestampSeconds;
    private Transform3d robotToCam;
    private String CameraName;

    public VisionStangs(String LLname, Transform3d robotToCam) {
        camera = new PhotonCamera(LLname);
        this.CameraName = LLname;
        ppe = new PhotonPoseEstimator(Constants.kAprilTagFieldLayout, robotToCam);
        this.robotToCam=robotToCam;
        bigAmbiguity = -1;
        isLL = false;
    }

    public VisionStangs(String LLname) {
        this.CameraName = LLname;
        bigAmbiguity = -1;
        isLL = true;

        LimelightHelpers.SetFiducialIDFiltersOverride(LLname, Constants.VisionConstants.goodIDS);
    }

    private void Photonperiodic() {
        SmartDashboard.putNumber("Vision/" + camera.getName() + " tet", Math.random());
        bigAmbiguity = 0;
        PhotonPipelineResult result = camera.getLatestResult();
        var target = result.getBestTarget();
        if (target!=null && Constants.kAprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), Constants.kAprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), robotToCam);
            Robot.teleopField.getObject(camera.getName() + " Pose").setPose(robotPose.toPose2d());
            currentPose=robotPose.toPose2d();
            timestampSeconds=result.getTimestampSeconds();
            CommandSwerveDrivetrain.getInstance().addVisionMeasurement(currentPose, timestampSeconds);
        }
                // Optional<EstimatedRobotPose> visionEst = Optional.empty();
        // for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
        //     SmartDashboard.putNumber("Vision/" + camera.getName() + " posess", bigAmbiguity);
        //     visionEst = ppe.estimateCoprocMultiTagPose(result);

        //     for (var target : result.targets) {
        //         if (target.poseAmbiguity > bigAmbiguity) {
        //             bigAmbiguity = target.poseAmbiguity;
        //         }
        //     }

        //     SmartDashboard.putNumber("Vision/" + camera.getName() + " biggest ambiguity", bigAmbiguity);
        // }
        // //SmartDashboard.putNumber("Vision/" + camera.getName() + " Pose Estimator", visionEst.get().estimatedPose.getX());
        // if (bigAmbiguity < Constants.VisionConstants.kPhotonAmbiguityThres ) {
        //     SmartDashboard.putBoolean("Vision/"+CameraName+"/isgoo", visionEst.isPresent());
        //     if (visionEst.isPresent()) {
        //             Robot.teleopField.getObject(camera.getName() + " Pose").setPose(visionEst.get().estimatedPose.toPose2d());

        //         if (isPosegoo(visionEst.get())) {
        //             var pose2d = visionEst.get().estimatedPose != null ? visionEst.get().estimatedPose.toPose2d()
        //                     : null;

        //             timestampSeconds = visionEst.get().timestampSeconds;
        //             currentPose = pose2d;
        //         }
        //     } 
        //     else {
        //         SmartDashboard.putString("Vision" + CameraName + "/Photon_RejectReason", "Ts not present bro");

        //     }

        // }else{
        //     SmartDashboard.putString("Vision" + CameraName + "/Photon_RejectReason", "BAD");
        // }

    }

    public double getavgDisPhoton(EstimatedRobotPose yoPoseEsti) {
        double avgDist = 0;
        for (PhotonTrackedTarget target : yoPoseEsti.targetsUsed) {
            avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        avgDist /= yoPoseEsti.targetsUsed.size();
        return avgDist;
    }

    private boolean isPosegoo(EstimatedRobotPose yoPoseEsti) {

        if (Robot.isSimulation()) {
            SmartDashboard.putString("Vision" + CameraName + "/Photon_RejectReason", "Simulation mode");
            return false;
        }
        if (yoPoseEsti == null) {
            SmartDashboard.putString("Vision" + CameraName + "/Photon_RejectReason", "Pose null");
            return false;
        }
        if (yoPoseEsti.targetsUsed.size() < 2) {
            SmartDashboard.putString("Vision" + CameraName + "/Photon_RejectReason",
                    "Not enough tags: " + yoPoseEsti.targetsUsed.size());
            return false;
        }

        if (getavgDisPhoton(yoPoseEsti) > Constants.VisionConstants.kLLDisThres) {
            SmartDashboard.putString("Vision" + CameraName + "/Photon_RejectReason",
                    "Too far: " + getavgDisPhoton(yoPoseEsti));
            return false;
        }
        SmartDashboard.putNumber("Vision/" + CameraName + " avg dis", getavgDisPhoton(yoPoseEsti));

        if (bigAmbiguity > Constants.VisionConstants.kLLAmbiguityThres) {
            SmartDashboard.putString("Vision" + CameraName + "/Photon_RejectReason", "High ambiguity: " + bigAmbiguity);
            return false;
        }

        if (!RobotContainer.useVision) {
            SmartDashboard.putString("Vision" + CameraName + "/Photon_RejectReason", "useVision disabled");
            return false;
        }

        SmartDashboard.putString("Vision" + CameraName + "/Photon_RejectReason", "Accepted");
        return true;
    }

    private void LLperiodic() {

        bigAmbiguity = 0;

        /*
         * Only seed ll4 with external yaw angle if not rotating fast, otherwise use
         * internal imu with mode 2
         */

        boolean isDisabled = DriverStation.isDisabled();
        CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

        if (isDisabled) {
            LimelightHelpers.SetIMUMode(CameraName, 1);

            LimelightHelpers.SetThrottle(CameraName, 200);
        } else {

            LimelightHelpers.SetRobotOrientation(CameraName,
                    drivetrain.getState().Pose.getRotation().getDegrees(),
                    0d,
                    0d,
                    0d,
                    0d,
                    0d);

            LimelightHelpers.SetIMUMode(CameraName, 4);
            LimelightHelpers.SetThrottle(CameraName, 0);
            // LimelightHelpers.SetIMUAssistAlpha(CameraName, 0.01);

        }

        if (!Robot.isSimulation()) {

            LimelightHelpers.PoseEstimate fourPoseEsti;
            String mtLabel;
            double yawRate = drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
            boolean isFastJohn = Math.abs(yawRate) > Constants.VisionConstants.kVisionAngularThreshold;

            if (isDisabled) {
                fourPoseEsti = LimelightHelpers.getBotPoseEstimate_wpiBlue(CameraName);
                mtLabel = "MT1 (disabled)";
            } else if (isFastJohn) {
                fourPoseEsti = null;
                mtLabel = "Rejected (too fast)";
            } else {
                fourPoseEsti = LimelightHelpers.getBotPoseEstimate_wpiBlue(CameraName);
                mtLabel = "MT2";
            }

            if (fourPoseEsti != null) {
                SmartDashboard.putString("Vision/" + CameraName + "/MT Mode ", mtLabel);
                Robot.teleopField.getObject("Raw " + CameraName).setPose(fourPoseEsti.pose);
                SmartDashboard.putNumber("Tag count " + CameraName, fourPoseEsti.tagCount);

            }

            if (isPosegoo(fourPoseEsti)) {
                this.currentPose = fourPoseEsti.pose;
                this.timestampSeconds = fourPoseEsti.timestampSeconds;
                Robot.teleopField.getObject(CameraName + " Pose LL").setPose(fourPoseEsti.pose);

                for (LimelightHelpers.RawFiducial fiducial : fourPoseEsti.rawFiducials) {
                    if (fiducial.ambiguity > bigAmbiguity) {
                        bigAmbiguity = fiducial.ambiguity;
                    }
                }
                SmartDashboard.putNumber("Vision/" + CameraName + "/Biggest Ambiguity", bigAmbiguity);
            } else {
                this.currentPose = null;
            }
        }
    }

    public double getTimeStamp() {
        return timestampSeconds;
    }

    public double getAmbiguity() {
        return bigAmbiguity;
    }

    public Pose2d getPose() {
        return currentPose;
    }

    public String getName() {
        return CameraName;
    }

    public boolean isLL() {
        return isLL;
    }

    private boolean isPosegoo(LimelightHelpers.PoseEstimate yoPoseEsti) {

        if (Robot.isSimulation()) {
            SmartDashboard.putString("Vision" + CameraName + "/LL_RejectReason", "Simulation mode");
            return false;
        }
        if (yoPoseEsti == null) {
            SmartDashboard.putString("Vision" + CameraName + "/LL_RejectReason", "Pose null");
            return false;
        }
        if (yoPoseEsti.tagCount < 2) {
            SmartDashboard.putString("Vision" + CameraName + "/LL_RejectReason",
                    "Not enough tags: " + yoPoseEsti.tagCount);
            return false;
        }
        if (yoPoseEsti.avgTagDist > Constants.VisionConstants.kLLDisThres) {
            SmartDashboard.putString("Vision" + CameraName + "/LL_RejectReason", "Too far: " + yoPoseEsti.avgTagDist);
            return false;
        }
        SmartDashboard.putNumber("Vision/" + CameraName + " avg dis", yoPoseEsti.avgTagDist);

        double maxAmbiguity = 0;
        for (LimelightHelpers.RawFiducial fiducial : yoPoseEsti.rawFiducials) {
            if (fiducial.ambiguity > maxAmbiguity) {
                maxAmbiguity = fiducial.ambiguity;
            }
        }
        bigAmbiguity = maxAmbiguity;
        SmartDashboard.putNumber("Vision/" + CameraName + "/Biggest Ambiguity", bigAmbiguity);

        if (bigAmbiguity > Constants.VisionConstants.kLLAmbiguityThres) {
            SmartDashboard.putString("Vision" + CameraName + "/LL_RejectReason", "High ambiguity: " + bigAmbiguity);
            return false;
        }

        if (!RobotContainer.useVision) {
            SmartDashboard.putString("Vision" + CameraName + "/LL_RejectReason", "useVision disabled");
            return false;
        }

        SmartDashboard.putString("Vision" + CameraName + "/LL_RejectReason", "Accepted");
        return true;
    }

    public void periodic() {
        currentPose = null;
        if (isLL) {
            LLperiodic();
        } else {
            Photonperiodic();
        }
    }
}