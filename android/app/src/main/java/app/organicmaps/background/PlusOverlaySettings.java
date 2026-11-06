package app.organicmaps.background;

import android.content.Context;
import android.content.SharedPreferences;
import android.text.format.Formatter;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.RadioGroup;
import android.widget.TextView;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import app.organicmaps.MwmApplication;
import app.organicmaps.R;
import app.organicmaps.sdk.downloader.MapManager;
import app.organicmaps.sdk.routing.PlusOverlay;
import com.google.android.material.dialog.MaterialAlertDialogBuilder;

/**
 * Comaps++: whether fast-route data is kept only for the current avoid settings (default) or for every
 * mix of avoid settings used. Asked once at startup and changeable in Settings → Navigation.
 */
public final class PlusOverlaySettings
{
  private static final String KEY_KEEP_ALL = "plus_overlay_keep_all_avoid";
  private static final String KEY_ASKED = "plus_overlay_storage_asked";
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
    case "fast": text = context.getString(R.string.plus_engine_fast, secs); break;
    case "rejoin": text = context.getString(R.string.plus_engine_rejoin, secs); break;
    case "normal": text = context.getString(R.string.plus_engine_normal, secs, r[2]); break;
    default: return;
    }
    android.widget.Toast.makeText(context, text, android.widget.Toast.LENGTH_LONG).show();
  }

  private PlusOverlaySettings() {}

  public static boolean isKeepAll(@NonNull Context context)
  {
    return MwmApplication.prefs(context).getBoolean(KEY_KEEP_ALL, false);
  }

  private static void setKeepAll(@NonNull Context context, boolean keepAll)
  {
    final boolean was = isKeepAll(context);
    MwmApplication.prefs(context).edit().putBoolean(KEY_KEEP_ALL, keepAll).apply();
    if (was && !keepAll)
      PlusOverlayWork.enqueue(context);  // removes the data of other avoid settings
  }

  @NonNull
  public static String usedSpace(@NonNull Context context)
  {
    return Formatter.formatShortFileSize(context, PlusOverlay.nativeStorageBytes());
  }

  @NonNull
  public static String summary(@NonNull Context context)
  {
    return context.getString(isKeepAll(context) ? R.string.plus_overlay_storage_pref_all
                                                : R.string.plus_overlay_storage_pref_current,
                             usedSpace(context));
  }

  /** Once, when there is at least one downloaded map (before that the question means nothing). */
  public static void maybeAskAtStartup(@NonNull Context context)
  {
    final SharedPreferences prefs = MwmApplication.prefs(context);
    if (prefs.getBoolean(KEY_ASKED, false) || MapManager.nativeGetDownloadedCount() == 0)
      return;
    prefs.edit().putBoolean(KEY_ASKED, true).apply();
    showDialog(context, null);
  }

  public static void showDialog(@NonNull Context context, @Nullable Runnable onChanged)
  {
    final View view = LayoutInflater.from(context).inflate(R.layout.dialog_plus_overlay_storage, null);
    final RadioGroup choice = view.findViewById(R.id.plus_overlay_storage_choice);
    choice.check(isKeepAll(context) ? R.id.plus_overlay_storage_all : R.id.plus_overlay_storage_current);
    final TextView used = view.findViewById(R.id.plus_overlay_storage_used);
    used.setText(context.getString(R.string.plus_overlay_storage_used, usedSpace(context)));
    new MaterialAlertDialogBuilder(context)
        .setTitle(R.string.plus_overlay_storage_title)
        .setView(view)
        .setPositiveButton(R.string.ok, (dialog, which) -> {
          setKeepAll(context, choice.getCheckedRadioButtonId() == R.id.plus_overlay_storage_all);
          if (onChanged != null)
            onChanged.run();
        })
        .setNegativeButton(R.string.cancel, null)
        .show();
  }
}
