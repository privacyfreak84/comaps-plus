package app.organicmaps.sdk.routing;

import androidx.annotation.NonNull;

/**
 * Comaps++: per-map overlay files that make long car routes fast. The router uses a map's file once
 * it exists and matches the downloaded map version; until then it uses the normal search.
 */
public final class PlusOverlay
{
  public static final int BUILD_OK = 0;
  public static final int BUILD_CANCELLED = 1;
  public static final int BUILD_FAILED = 2;
  public static final int BUILD_NO_MAP = 3;
  public static final int BUILD_UP_TO_DATE = 4;

  private PlusOverlay() {}

  /**
   * Downloaded maps whose base overlay file is missing or stale (map updated, or built by another
   * version). Files for avoid options are built only for maps a route needs.
   */
  @NonNull
  public static native String[] nativeMapsToBuild(boolean hasPosition, double lat, double lon);

  /** Whether a map lacks its base file or the file for the current avoid options. */
  public static native boolean nativeNeedsBuild(@NonNull String countryId);

  /**
   * Builds what one map lacks: the base overlay file, then, with {@code withAvoid}, the one for the
   * current avoid options; blocks for minutes. Returns one of the BUILD_* codes.
   */
  public static native int nativeBuild(@NonNull String countryId, int threads, boolean withAvoid);

  /** Stops a running build soon (it then returns BUILD_CANCELLED and writes nothing). */
  public static native void nativeCancel();

  /** Avoid options (bit mask) of the file being built now; 0 while building a base file or idle. */
  public static native int nativeBuildingAvoidMask();

  /** Current car avoid options as a bit mask. */
  public static native int nativeCurrentAvoidMask();

  /** Progress of the running build, 0..1. */
  public static native double nativeProgress();

  /** Short name of the running build's stage. */
  @NonNull
  public static native String nativeStage();

  /**
   * Deletes overlay files of maps that were deleted or updated, and leftovers of interrupted builds. Unless
   * {@code keepAllAvoid}, also the files for avoid settings other than the current ones (with no avoid
   * settings on, the last ones are kept).
   */
  public static native int nativeDeleteOrphans(boolean keepAllAvoid);

  /**
   * {engine, seconds, reason, map ids...} of the last car route; engine: "none", "fast", "rejoin", "normal"
   * or "approximate" (fast, but crossing the listed maps, whose files aren't built yet, on estimates).
   */
  @NonNull
  public static native String[] nativeLastEngine();

  /** Builds the current route again (from the current position when navigating); main thread only. */
  public static native void nativeRefreshRoute();

  /** Bytes used by all overlay files. */
  public static native long nativeStorageBytes();
}
