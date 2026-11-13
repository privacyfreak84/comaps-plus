package app.organicmaps.sdk.routing;

import androidx.annotation.NonNull;

/**
 * Tabula: per-map overlay files that make long car routes fast. The router uses a map's file once
 * it exists and matches the downloaded map version; until then it crosses that map on estimates.
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
   * Downloaded maps whose overlay file is missing or stale (map updated, built by another version, or in an
   * older layout), nearest first.
   */
  @NonNull
  public static native String[] nativeMapsToBuild(boolean hasPosition, double lat, double lon);

  /** Whether a map's overlay file is missing or stale. */
  public static native boolean nativeNeedsBuild(@NonNull String countryId);

  /** Builds a map's overlay file; blocks for minutes. Returns one of the BUILD_* codes. */
  public static native int nativeBuild(@NonNull String countryId, int threads);

  /** Stops a running build soon (it then returns BUILD_CANCELLED and writes nothing). */
  public static native void nativeCancel();

  /** Progress of the running build, 0..1. */
  public static native double nativeProgress();

  /** Short name of the running build's stage. */
  @NonNull
  public static native String nativeStage();

  /**
   * Deletes overlay files of maps that were deleted or updated, leftovers of interrupted builds, and the
   * avoid-option files earlier versions built (avoid options need no file now).
   */
  public static native int nativeDeleteOrphans();

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
