package pioneer.helpers

import java.util.concurrent.Callable
import java.util.concurrent.Executors
import pioneer.Constants
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

class ModelPredictiveControl(
    private val horizon: Int = 10,
    private val maxAccel: Double = 50.0,
    private val posWeight: Double = 1.0,
    private val controlWeight: Double = 0.01,
    private val iterations: Int = 3,
    private val wheelSpeedLimit: Double = Constants.Drive.MAX_FWD_VEL_CMPS,
    private val wheelSpeedPenalty: Double = 100.0,
    private val trackWidth: Double = Constants.Drive.TRACK_WIDTH_CM,
    private val wheelBase: Double = Constants.Drive.WHEEL_BASE_CM,
) {
    private val threadPool = Executors.newFixedThreadPool(6)
    // Half the drivebase diagonal: |omega| * halfDiagonal gives per-wheel tangential speed
    private val halfDiagonal = 0.5 * (trackWidth + wheelBase)

    fun update(
        current: Pose,
        target: Pose,
        dt: Double,
    ): Pose {
        require(dt > 0 && dt.isFinite()) { "dt must be a finite, positive value" }
        val initialControl = computeInitialControl(current, target, dt)
        return optimize(current, target, initialControl, dt)
    }

    fun shutdown() = threadPool.shutdown()

    private fun optimize(
        current: Pose,
        target: Pose,
        initControl: Pose,
        dt: Double,
    ): Pose {
        var best = clampControl(initControl, current, dt)
        var bestCost = evaluateCost(current, target, best, dt)

        repeat(iterations) { iter ->
            val delta = maxAccel * 0.2 / (1 + iter * 0.5)

            val perturbations = listOf(
                Callable { Pair(best.copy(ax = best.ax + delta), 0) },
                Callable { Pair(best.copy(ax = best.ax - delta), 1) },
                Callable { Pair(best.copy(ay = best.ay + delta), 2) },
                Callable { Pair(best.copy(ay = best.ay - delta), 3) },
                Callable { Pair(best.copy(alpha = best.alpha + delta), 4) },
                Callable { Pair(best.copy(alpha = best.alpha - delta), 5) },
            )

            val futures = threadPool.invokeAll(perturbations)
            
            futures.forEach { future ->
                val (control, _) = future.get()
                val clamped = clampControl(control, current, dt)
                val c = evaluateCost(current, target, clamped, dt)
                if (c < bestCost) {
                    best = clamped
                    bestCost = c
                }
            }
        }

        return best
    }

    private fun computeInitialControl(current: Pose, target: Pose, dt: Double): Pose {
        val totalTime = dt * horizon
        val dt2 = totalTime * totalTime
        val dTheta = wrapAngle(target.theta - current.theta)
        // Seed optimizer with constant-accel inputs that roughly land on the goal at end of horizon (x = x0 + vx*T + a*T^2).
        val ax = ((target.x - current.x - current.vx * totalTime) / dt2).coerceIn(-maxAccel, maxAccel)
        val ay = ((target.y - current.y - current.vy * totalTime) / dt2).coerceIn(-maxAccel, maxAccel)
        val alpha = ((dTheta - current.omega * totalTime) / dt2).coerceIn(-maxAccel, maxAccel)
        return Pose(ax = ax, ay = ay, alpha = alpha)
    }

    private fun evaluateCost(current: Pose, target: Pose, control: Pose, dt: Double): Double {
        var pose = current
        var sum = 0.0
        val controlEffort = control.ax * control.ax + control.ay * control.ay + control.alpha * control.alpha
        val constantControlCost = controlWeight * controlEffort
        val wheelLimitActive = wheelSpeedLimit.isFinite() && wheelSpeedLimit > 0.0

        repeat(horizon) {
            pose = pose.copy(
                vx = pose.vx + control.ax * dt,
                vy = pose.vy + control.ay * dt,
                omega = pose.omega + control.alpha * dt,
                ax = control.ax,
                ay = control.ay,
                alpha = control.alpha,
            ).integrate(dt)

            val posError = pose.distanceTo(target)
            val angError = abs(wrapAngle(pose.theta - target.theta))
            if (wheelLimitActive) {
                val peakWheelSpeed = maxWheelSpeed(pose.vx, pose.vy, pose.omega)
                val overshoot = peakWheelSpeed - wheelSpeedLimit
                if (overshoot > 0) {
                    sum += wheelSpeedPenalty * overshoot * overshoot // Quadratic penalty on violating wheel speed
                }
            }

            sum += posWeight * (posError + angError) + constantControlCost
        }

        return sum
    }

    private fun clampControl(
        control: Pose,
        current: Pose,
        dt: Double,
    ): Pose {
        val accelClamped =
            control.copy(
                ax = control.ax.coerceIn(-maxAccel, maxAccel),
                ay = control.ay.coerceIn(-maxAccel, maxAccel),
                alpha = control.alpha.coerceIn(-maxAccel, maxAccel),
            )

        if (!wheelSpeedLimit.isFinite() || wheelSpeedLimit <= 0.0) {
            return accelClamped
        }

        val predictedVx = current.vx + accelClamped.ax * dt
        val predictedVy = current.vy + accelClamped.ay * dt
        val predictedOmega = current.omega + accelClamped.alpha * dt
        val predictedWheelSpeed = maxWheelSpeed(predictedVx, predictedVy, predictedOmega)
        if (predictedWheelSpeed <= wheelSpeedLimit) {
            return accelClamped
        }

        val currentWheelSpeed = maxWheelSpeed(current.vx, current.vy, current.omega)
        if (currentWheelSpeed > wheelSpeedLimit && predictedWheelSpeed < currentWheelSpeed) {
            // Already over limit; allow braking accelerations to pass through.
            return accelClamped
        }

        val scale = (wheelSpeedLimit / predictedWheelSpeed).coerceAtMost(1.0)
        return accelClamped.copy(
            ax = accelClamped.ax * scale,
            ay = accelClamped.ay * scale,
            alpha = accelClamped.alpha * scale,
        )
    }

    private fun wrapAngle(angle: Double): Double = atan2(sin(angle), cos(angle))

    private fun maxWheelSpeed(vx: Double, vy: Double, omega: Double): Double {
        // Mecanum inverse kinematics: rotation contributes +/- omega * r to each wheel
        val rotComponent = omega * halfDiagonal
        val lf = vy + vx + rotComponent
        val lb = vy - vx + rotComponent
        val rf = vy - vx - rotComponent
        val rb = vy + vx - rotComponent
        return maxOf(abs(lf), abs(lb), abs(rf), abs(rb))
    }
}
