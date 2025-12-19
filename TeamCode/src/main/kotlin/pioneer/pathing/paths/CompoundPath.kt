package pioneer.pathing.paths

import pioneer.helpers.Pose

/**
 * Represents a compound path made up of multiple paths.
 * The paths are assumed to be connected end-to-end, meaning the end pose of one path
 * should match the start pose of the next path in the list.
 * @param paths A list of paths that make up the compound path.
 */
class CompoundPath(
    private val paths: List<Path>,
) : Path {
    override var startPose: Pose = paths.first().startPose
    override var endPose: Pose = paths.last().endPose

    init {
        // Validate that the paths are connected end-to-end
        for (i in 0 until paths.size - 1) {
            if (paths[i].endPose.distanceTo(paths[i + 1].startPose) > 1e-6) {
                throw IllegalArgumentException("Paths are not connected end-to-end")
            }
        }
    }

    override fun getLength(): Double = paths.sumOf { it.getLength() }

    override fun getPoint(t: Double): Pose {
        val totalLength = getLength()
        val targetLength = (t.coerceIn(0.0, 1.0)) * totalLength
        var remaining = targetLength
        for (path in paths) {
            val length = path.getLength()
            if (remaining <= length) {
                val localT = path.getTFromLength(remaining)
                return path.getPoint(localT)
            }
            remaining -= length
        }
        return paths.last().getPoint(1.0)
    }

    override fun getPose(t: Double): Pose {
        val totalLength = getLength()
        val targetLength = (t.coerceIn(0.0, 1.0)) * totalLength
        var remaining = targetLength
        for (path in paths) {
            val length = path.getLength()
            if (remaining <= length) {
                val localT = path.getTFromLength(remaining)
                return path.getPose(localT)
            }
            remaining -= length
        }
        return paths.last().getPose(1.0)
    }

    override fun getCurvature(t: Double): Double {
        val totalLength = getLength()
        val targetLength = (t.coerceIn(0.0, 1.0)) * totalLength
        var remaining = targetLength
        for (path in paths) {
            val length = path.getLength()
            if (remaining <= length) {
                val localT = path.getTFromLength(remaining)
                return path.getCurvature(localT)
            }
            remaining -= length
        }
        return paths.last().getCurvature(1.0)
    }

    override fun getClosestPointT(position: Pose): Double {
        // Find the closest point on each path and return the closest one
        var closestTLength = 0.0
        var minDistance = Double.MAX_VALUE
        var accumulatedLength = 0.0

        for (path in paths) {
            val pathLength = path.getLength()
            val t = path.getClosestPointT(position)
            val point = path.getPoint(t)
            val distance = position.distanceTo(point)

            if (distance < minDistance) {
                minDistance = distance
                closestTLength = accumulatedLength + t * pathLength
            }
            accumulatedLength += pathLength
        }

        val totalLength = getLength()
        return if (totalLength > 0.0) closestTLength / totalLength else 0.0
    }

    override fun getLengthSoFar(t: Double): Double {
        val totalLength = getLength()
        return (t.coerceIn(0.0, 1.0)) * totalLength
    }

    override fun getTFromLength(length: Double): Double {
        val totalLength = getLength()
        if (totalLength <= 0.0) return 0.0
        return (length / totalLength).coerceIn(0.0, 1.0) // If length exceeds total, return end of the compound path
    }

    class Builder {
        private val paths = mutableListOf<Path>()

        fun addPath(path: Path): Builder {
            paths.add(path)
            return this
        }

        fun addPaths(vararg newPaths: Path): Builder {
            paths.addAll(newPaths)
            return this
        }

        fun build(): CompoundPath = CompoundPath(paths)
    }
}
