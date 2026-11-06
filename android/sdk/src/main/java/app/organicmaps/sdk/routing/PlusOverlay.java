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

  /** Downloaded maps whose overlay file is missing or stale (map updated, or built by another version). */
  @NonNull
  public static native String[] nativeMapsToBuild();

  /** Builds one map's overlay file; blocks for minutes. Returns one of the BUILD_* codes. */
  public static native int nativeBuild(@NonNull String countryId, int threads);

  /** Stops a running build soon (it then returns BUILD_CANCELLED and writes nothing). */
  public static native void nativeCancel();

  /** Progress of the running build, 0..1. */
  public static native double nativeProgress();

  /** Short name of the running build's stage. */
  @NonNull
  public static native String nativeStage();

  /** Deletes overlay files of maps that were deleted or updated, and leftovers of interrupted builds. */
  public static native int nativeDeleteOrphans();
}
