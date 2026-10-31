package frc.robot.vision;

import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import java.util.Set;

import static frc.robot.Constants.VisionConstants.*;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final PhotonCamera[] cameras;
    private final PhotonPoseEstimator[] estimators;

    private boolean disabled = false;
    private boolean climbing = false;

    public Vision(VisionConsumer consumer) {
        this.consumer = consumer;

        int numCameras = CAMERA_NAMES.length;
        cameras = new PhotonCamera[numCameras];
        estimators = new PhotonPoseEstimator[numCameras];

        for (int i = 0; i < numCameras; i++) {
            cameras[i] = new PhotonCamera(CAMERA_NAMES[i]);
            
            estimators[i] = new PhotonPoseEstimator(
                APRIL_TAGS, 
                MULTI_TAG_PNP_ON_COPROCESSOR, 
                CAMERA_TRANSFORMS[i]
            );
        }
    }

    @Override
    public void periodic() {
        if (disabled) return;

        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        Set<Integer> hubTags = isBlue ? BLUE_HUB_TAG_IDS : RED_HUB_TAG_IDS;

        for (int i = 0; i < cameras.length; i++) {
            PhotonCamera camera = cameras[i];

            if (!camera.isConnected()) {
                continue;
            }

            var result = camera.getLatestResult();
            Optional<EstimatedRobotPose> estimatedPoseOpt = estimators[i].update(result);

            if (estimatedPoseOpt.isPresent()) {
                EstimatedRobotPose estimatedPose = estimatedPoseOpt.get();
                Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();
                int tagCount = estimatedPose.targetsUsed.size();

                if (tagCount == 0) continue;

                if (tagCount == 1 && estimatedPose.targetsUsed.get(0).getPoseAmbiguity() > MAX_AMBIGUITY) {
                    continue;
                }

                if (estimatedPose.estimatedPose.getZ() > MAX_Z_HEIGHT.in(Meters)) {
                    continue;
                }

                if (pose2d.getX() < 0.0 || pose2d.getX() > APRIL_TAGS.getFieldLength() ||
                    pose2d.getY() < 0.0 || pose2d.getY() > APRIL_TAGS.getFieldWidth()) {
                    continue;
                }

                double totalDistance = 0.0;
                for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
                    totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
                }
                double averageDistance = totalDistance / tagCount;
                double stdDevFactor = Math.pow(averageDistance, 2.0) / tagCount;

                for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
                int tagId = target.getFiducialId();
    
                // 如果不是 HUB 標籤，就增加標準差偏置 (降低其在定位中的權重)
                if (!hubTags.contains(tagId)) {
                    stdDevFactor += HUB_TAG_STD_DEV_BIAS;
                }
                }

                double linearStdDev = LINEAR_STD_DEV_BASELINES[i] * stdDevFactor;
                double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;

                consumer.accept(
                    pose2d, 
                    estimatedPose.timestampSeconds, 
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
                );
                
                SmartDashboard.putString("Vision/" + camera.getName() + " Pose", pose2d.toString());
            }
        }
    }

    public boolean hasTarget(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= cameras.length) return false;
        return cameras[cameraIndex].getLatestResult().hasTargets();
    }

    public double getTargetYaw(int cameraIndex) {
        if (hasTarget(cameraIndex)) {
            return cameras[cameraIndex].getLatestResult().getBestTarget().getYaw();
        }
        return 0.0;
    }

    public void setClimbing(boolean climbing) {
        this.climbing = climbing;
    }

    public void setDisabled(boolean disabled) {
        this.disabled = disabled;
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}