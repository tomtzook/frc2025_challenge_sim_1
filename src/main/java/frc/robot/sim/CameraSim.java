package frc.robot.sim;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;

public class CameraSim {

    private final double projectionWidth;
    private final double projectionHeight;
    private final double projectionDistance;

    public CameraSim(double projectionWidth, double projectionHeight, double projectionDistance) {
        this.projectionWidth = projectionWidth;
        this.projectionHeight = projectionHeight;
        this.projectionDistance = projectionDistance;
    }

    public boolean isPointInFov(Vector<N3> point, Rotation3d cameraRotation) {
        Vector<N3> forward = VecBuilder.fill(1, 0, 0);
        Vector<N3> right = VecBuilder.fill(0, 1, 0);
        Vector<N3> up = VecBuilder.fill(0, 0, 1);

        // get unit vectors for rotation
        Quaternion rotation = cameraRotation.getQuaternion();
        forward = rotate(forward, rotation);
        right = rotate(right, rotation);
        up = rotate(up, rotation);

        double d1 = point.dot(forward);
        if (d1 < 0 || d1 > projectionDistance) {
            // behind camera, or too far out front
            return false;
        }

        Vector<N3> p1 = point.div(d1).minus(forward).times(projectionDistance);

        double right1 = p1.dot(right);
        double up1 = p1.dot(up);

        double halfWidth = projectionWidth / 2.0;
        double halfHeight = projectionHeight / 2.0;

        if (right1 < -halfWidth || right1 > halfWidth) {
            // out of fov, too far right or left
            return false;
        }
        if (up1 < -halfHeight || up1 > halfHeight) {
            // out of fov, too far up or down
            return false;
        }

        return true;
    }

    public boolean isFacingFront(Rotation3d pointRotation, Rotation3d cameraRotation) {
        Vector<N3> forward = VecBuilder.fill(1, 0, 0);

        Vector<N3> camForward = rotate(forward, cameraRotation.getQuaternion());
        Vector<N3> pointForward = rotate(forward, pointRotation.getQuaternion());

        return angleBetween(pointForward, camForward) > Math.PI / 2;
    }

    private static Vector<N3> rotate(Vector<N3> vec, Quaternion quaternion) {
        Quaternion p = new Quaternion(0.0, vec.get(0), vec.get(1), vec.get(2));
        Quaternion qPrime = quaternion.times(p).times(quaternion.inverse());
        return VecBuilder.fill(qPrime.getX(), qPrime.getY(), qPrime.getZ());
    }

    private static double angleBetween(Vector<N3> first, Vector<N3> second) {
        return Math.acos(first.dot(second) / (magnitude(first) * magnitude(second)));
    }

    private static double magnitude(Vector<N3> vec) {
        double x = vec.get(0);
        double y = vec.get(1);
        double z = vec.get(2);
        return Math.sqrt(x * x + y * y + z * z);
    }
}
