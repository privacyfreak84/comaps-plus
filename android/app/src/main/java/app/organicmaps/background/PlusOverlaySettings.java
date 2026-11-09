package app.organicmaps.background;

import android.content.Context;
import android.text.format.Formatter;
import androidx.annotation.NonNull;
import app.organicmaps.MwmApplication;
import app.organicmaps.R;
import app.organicmaps.sdk.downloader.MapManager;
import app.organicmaps.sdk.routing.PlusOverlay;

/**
 * Comaps++: the fast-route entries in Settings → Navigation (how many maps are ready, space used, and the
 * test switch that shows how each car route was calculated). Data for avoid settings is kept for the
 * current mix only: routes with any mix are exact without it, so there is nothing to choose.
 */
public final class PlusOverlaySettings
{
  private static final String KEY_SHOW_ENGINE = "plus_show_engine";

  public static boolean isShowEngine(@NonNull Context context)
  {
    return MwmApplication.prefs(context).getBoolean(KEY_SHOW_ENGINE, false);
  }

  public static void setShowEngine(@NonNull Context context, boolean show)
  {
    MwmApplication.prefs(context).edit().putBoolean(KEY_SHOW_ENGINE, show).apply();
  }

  /** After a car route: which router answered (only when the user turned this on). */
  public static void maybeShowEngine(@NonNull Context context, int resultCode)
  {
    if (!isShowEngine(context))
      return;
    final String[] r = PlusOverlay.nativeLastEngine();
    if (r.length < 3)
      return;
    final String secs = String.format(java.util.Locale.US, "%.1f", Double.parseDouble(r[1]));
    final String text;
    switch (r[0])
    {
    case "fast":
    {
      final String[] avoid = PlusOverlay.nativeLastAvoidMaps();
      final java.util.List<String> names = new java.util.ArrayList<>();
      for (String id : avoid)
        names.add(MapManager.nativeGetName(id));
      text = names.isEmpty() ? context.getString(R.string.plus_engine_fast, secs)
                             : context.getString(R.string.plus_engine_fast_noavoid, secs,
                                                 android.text.TextUtils.join(", ", names));
      break;
    }
    case "rejoin": text = context.getString(R.string.plus_engine_rejoin, secs); break;
    case "normal": text = context.getString(R.string.plus_engine_normal, secs, r[2]); break;
    case "approximate": text = context.getString(R.string.plus_engine_approximate, secs); break;
    default: return;
    }
    android.widget.Toast.makeText(context, text, android.widget.Toast.LENGTH_LONG).show();
  }

  private PlusOverlaySettings() {}

  @NonNull
  public static String usedSpace(@NonNull Context context)
  {
    return Formatter.formatShortFileSize(context, PlusOverlay.nativeStorageBytes());
  }

  /** "Ready for N of M downloaded maps" and the space the data uses. */
  @NonNull
  public static String summary(@NonNull Context context)
  {
    final String storage = context.getString(R.string.plus_overlay_storage_pref_used, usedSpace(context));
    final int maps = MapManager.nativeGetDownloadedCount();
    if (maps <= 0)
      return storage;
    final int ready = Math.max(0, maps - PlusOverlay.nativeMapsToBuild(false, 0, 0, false /* withAvoid */).length);
    return context.getString(R.string.plus_overlay_ready_count, ready, maps) + "\n" + storage;
  }
}
