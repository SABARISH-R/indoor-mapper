package com.arroute.nav.core

enum class MarkerStatus { UNSURVEYED, SURVEY_PENDING, SURVEYED, SURVEY_FAILED, STALE }

/** WALL: reference axis is the marker's outward normal. FLOOR: reference axis is the image +X axis. */
enum class Mount { WALL, FLOOR }

/** One surveyed (or not) marker from the released map. Only SURVEYED markers can localize a visitor. */
data class MarkerRecord(
    val id: String,
    val floorId: String,
    val mapVersion: String,
    val x: Double,
    val z: Double,
    val heightAboveFloor: Double,
    val refYaw: Double,
    val mount: Mount,
    val status: MarkerStatus,
    val uncertaintyM: Double,
)

data class AnchorBudget(val maxDistanceM: Double, val maxSeconds: Double)

/** A marker as seen in the AR session frame. [refYawSession] must use the same reference axis as the plan record. */
data class SessionObservation(
    val markerId: String,
    val x: Double, val y: Double, val z: Double,
    val refYawSession: Double,
    val fullTracking: Boolean,
)

sealed interface LocState {
    data class NoFix(val reason: String) : LocState
    data class Coarse(val reason: String) : LocState
    data class Fine(val markerId: String, val floorId: String) : LocState
    data class Lost(val reason: String) : LocState
}

enum class RejectReason {
    CAMERA_NOT_ALLOWED, TRACKING_NOT_NORMAL, NOT_FULLY_TRACKED, AMBIGUOUS_MULTIPLE_MARKERS,
    UNKNOWN_MARKER, WRONG_MAP_VERSION, NOT_SURVEYED, FLOOR_MISMATCH,
}

sealed interface ScanResult {
    data class Accepted(val markerId: String) : ScanResult
    data class Rejected(val reason: RejectReason, val detail: String) : ScanResult
}

/**
 * Fine localization rules. Core guarantee: after ANY invalidation the old transform is deleted,
 * so a stale pose can never be reused. Only a newly accepted marker scan restores FINE.
 */
class LocalizationManager(
    markers: List<MarkerRecord>,
    mapVersion: String,
    private val budget: AnchorBudget,
    private val maxStepM: Double = 3.0,
) {
    private val byId = markers.associateBy { it.id }
    var mapVersion: String = mapVersion
        private set
    var state: LocState = LocState.NoFix("No marker scanned yet.")
        private set
    var cameraAllowed: Boolean = true
        private set
    /** Floor from a manual pick or barometer. If a marker disagrees, fine localization is dropped. */
    var expectedFloorId: String? = null
    /** Session-frame y of the finished floor at the anchor marker, or null when not anchored. */
    var floorYSession: Double? = null
        private set

    private var tf: Rigid2D? = null
    private var walked = 0.0
    private var anchoredAt = 0.0
    private var last: Vec2? = null

    val walkedSinceAnchorM: Double get() = walked

    private fun invalidate(s: LocState) {
        state = s; tf = null; floorYSession = null; last = null; walked = 0.0
    }

    private fun reject(r: RejectReason, d: String) = ScanResult.Rejected(r, d)

    fun onMarkerObserved(obs: List<SessionObservation>, trackingNormal: Boolean, nowSec: Double): ScanResult {
        if (!cameraAllowed) return reject(RejectReason.CAMERA_NOT_ALLOWED, "Camera is not allowed in this zone.")
        if (!trackingNormal) return reject(RejectReason.TRACKING_NOT_NORMAL, "AR tracking is not normal.")
        val good = obs.filter { it.fullTracking }
        if (good.isEmpty()) return reject(RejectReason.NOT_FULLY_TRACKED, "No marker is being fully tracked.")
        if (good.map { it.markerId }.distinct().size > 1)
            return reject(RejectReason.AMBIGUOUS_MULTIPLE_MARKERS, "More than one marker is in view.")
        val o = good.first()
        val m = byId[o.markerId] ?: return reject(RejectReason.UNKNOWN_MARKER, "Marker ${o.markerId} is not in this map.")
        if (m.mapVersion != mapVersion) return reject(RejectReason.WRONG_MAP_VERSION, "Marker is from map ${m.mapVersion}.")
        if (m.status != MarkerStatus.SURVEYED) return reject(RejectReason.NOT_SURVEYED, "Marker ${m.id} is ${m.status}.")
        val exp = expectedFloorId
        if (exp != null && exp != m.floorId) {
            invalidate(LocState.Coarse("Conflicting floor evidence: expected $exp, marker says ${m.floorId}."))
            return reject(RejectReason.FLOOR_MISMATCH, "Expected floor $exp but marker ${m.id} is on ${m.floorId}.")
        }
        tf = Rigid2D.fromMarker(Vec2(o.x, o.z), o.refYawSession, Vec2(m.x, m.z), m.refYaw)
        floorYSession = o.y - m.heightAboveFloor
        walked = 0.0; anchoredAt = nowSec; last = null
        state = LocState.Fine(m.id, m.floorId)
        return ScanResult.Accepted(m.id)
    }

    /** Returns a plan pose only while fine localization is valid; otherwise null. */
    fun onFrame(trackingNormal: Boolean, camX: Double, camZ: Double, fwdX: Double, fwdZ: Double, nowSec: Double): PlanPose? {
        val s = state as? LocState.Fine ?: return null
        val t = tf ?: return null
        if (!trackingNormal) { invalidate(LocState.Lost("Tracking lost. Scan a marker.")); return null }
        val pos = Vec2(camX, camZ)
        last?.let {
            val step = (pos - it).length()
            if (step > maxStepM) { invalidate(LocState.Lost("Pose jumped %.1f m. Scan a marker.".format(step))); return null }
            walked += step
        }
        last = pos
        if (walked > budget.maxDistanceM) { invalidate(LocState.Coarse("Distance budget used. Scan the next marker.")); return null }
        if (nowSec - anchoredAt > budget.maxSeconds) { invalidate(LocState.Coarse("Time budget used. Scan the next marker.")); return null }
        return PlanPose(t.apply(pos), t.applyYaw(headingOf(fwdX, fwdZ)), s.floorId, mapVersion)
    }

    fun onSessionRestart() = invalidate(LocState.NoFix("Session restarted. Scan a marker."))
    fun onTrackingLost() = invalidate(LocState.Lost("Tracking lost. Scan a marker."))
    fun onFloorChange(newFloorId: String) { expectedFloorId = newFloorId; invalidate(LocState.Lost("Floor changed. Scan a marker on this floor.")) }
    fun onMapVersionChange(v: String) { mapVersion = v; invalidate(LocState.Lost("Map updated. Scan a marker.")) }
    fun setCameraAllowed(allowed: Boolean) {
        cameraAllowed = allowed
        if (!allowed) invalidate(LocState.Lost("Camera not allowed in this zone."))
    }
}
