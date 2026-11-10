package app.organicmaps.sdk.routing;

import androidx.annotation.NonNull;

/**
 * Comaps++: roads and areas kept out of the current car route. They belong to that route: they're cleared when
 * the route is closed, saved with it when it's saved, and never stored on their own. Nothing has to be prepared
 * for them. A place at a route's start, finish or a stop doesn't apply to it.
 */
public final class PlusAvoidPlaces
{
  public static final int KIND_AREA = 0;
  public static final int KIND_ROAD = 1;

  public static final class Place
  {
    public int kind;
    /** Area only, metres. */
    public int radiusM;
    /** Road: its name or number ("" for an unnamed road). Area: the settlement's name, if any. */
    @NonNull
    public String name = "";
  }

  private PlusAvoidPlaces() {}

  /**
   * Works out what the place page offers for the place shown in it. A place the route already avoids (drawn in
   * red): Don't avoid. Otherwise, with a car route shown: Avoid for the settlement the route passes through, or
   * the route's own road at that point. False for places off the route and places the route has to go to anyway
   * (its start, finish or a stop).
   */
  public static native boolean nativePrepareRouteAvoid();

  /** Whether the prepared place is one the route already avoids. */
  public static native boolean nativeIsPendingRemoval();

  /** {kind, radius in metres} of the prepared place (empty if none). */
  @NonNull
  public static native double[] nativePendingNumbers();

  /** Name of that place: the settlement's, or the road's name or number ("" for an unnamed road). */
  @NonNull
  public static native String nativePendingName();

  /** Adds that place to the current route's avoided places, or removes it from them. */
  public static native void nativeApplyPending();

  /** How many places the current route avoids. */
  public static native int nativeCount();

  /** How many avoided places applied to the last car route (covered roads, not at its ends or stops). */
  public static native int nativeLastApplied();
}
