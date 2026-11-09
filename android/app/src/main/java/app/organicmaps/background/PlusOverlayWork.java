package app.organicmaps.background;

import static android.Manifest.permission.POST_NOTIFICATIONS;
import static android.content.pm.PackageManager.PERMISSION_GRANTED;

import android.app.Notification;
import android.app.PendingIntent;
import android.content.Context;
import android.content.SharedPreferences;
import android.content.pm.ServiceInfo;
import android.os.Build;
import android.os.Handler;
import android.os.Looper;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.core.app.NotificationChannelCompat;
import androidx.core.app.NotificationCompat;
import androidx.core.app.NotificationManagerCompat;
import androidx.core.content.ContextCompat;
import androidx.work.ExistingWorkPolicy;
import androidx.work.ForegroundInfo;
import androidx.work.OneTimeWorkRequest;
import androidx.work.WorkManager;
import androidx.work.Worker;
import androidx.work.WorkerParameters;
import app.organicmaps.MwmApplication;
import app.organicmaps.R;
import app.organicmaps.sdk.downloader.MapManager;
import app.organicmaps.sdk.routing.PlusOverlay;
import app.organicmaps.sdk.util.log.Logger;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.Set;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Comaps++: builds the overlay files for fast car routing on the phone, one map at a time, after a map
 * download or update (and at app start for anything missed). Shows a progress notification with a
 * Cancel button; cancelling stops the running build until the app starts again or a route needs a map.
 * Until a map's base file is built, routes cross that map on estimates (see PlusRouteStatus); maps a
 * route needs are built first. Every downloaded map gets its base file and the file for the current avoid
 * options (routes with avoid options are exact without it, only slower; files for other mixes are deleted).
 *
 * Nothing makes it give up on a map: it doesn't wait for battery or storage, a build the system killed
 * is tried again (with one thread, after the other maps), and failed builds are retried later.
 */
public class PlusOverlayWork extends Worker
{
  private static final String TAG = PlusOverlayWork.class.getSimpleName();
  private static final String WORK_NAME = "PlusOverlayBuild";
  private static final String RETRY_WORK_NAME = "PlusOverlayRetry";
  private static final String CHANNEL_ID = "plus_overlay";
  private static final int PROGRESS_NOTIFICATION_ID = 7301;
  private static final int RESULT_NOTIFICATION_ID = 7302;
  private static final int BUILD_THREADS = 2;
  // A build the app didn't live to finish (the system killed it, e.g. for memory): retried with one
  // thread (less memory), after the maps that haven't been killed.
  private static final String KEY_BUILDING = "plus_overlay_building";
  private static final String KEY_KILLED_PREFIX = "plus_overlay_killed_";
  private static final String KEY_KILLED_AT_PREFIX = "plus_overlay_killed_at_";
  // From the third kill on, a map waits before the next try (15 min, doubling, up to 6 h), so a phone that
  // can't spare the memory isn't kept busy; the map is still tried again.
  private static final int KILLS_BEFORE_WAIT = 3;
  // Runs in a row in which a build failed: the next try waits 1, 4, 16, 64 minutes ... up to 6 hours.
  private static final String KEY_FAILED_RUNS = "plus_overlay_failed_runs";
  private static final long MAX_RETRY_DELAY_MIN = 6 * 60;

  /** Called on the main thread when a map's fast-route files are ready. */
  public interface MapReadyListener
  {
    void onMapReady(@NonNull String countryId);
  }

  // Maps a route crossed on estimates: built before the others.
  private static final Set<String> sPriority = Collections.synchronizedSet(new LinkedHashSet<>());
  @Nullable
  private static volatile String sBuilding;
  private static volatile boolean sPreempted;
  @Nullable
  private static MapReadyListener sMapReady;

  public static void setMapReadyListener(@Nullable MapReadyListener listener)
  {
    sMapReady = listener;
  }

  /** The map being built now, or null. */
  @Nullable
  public static String building()
  {
    return sBuilding;
  }

  /** After map downloads or updates: builds what they need, and tries maps given up on again. */
  public static void onMapsChanged(@NonNull Context context)
  {
    final SharedPreferences prefs = MwmApplication.prefs(context);
    final SharedPreferences.Editor edit = prefs.edit();
    for (String key : prefs.getAll().keySet())
      if (key.startsWith(KEY_KILLED_PREFIX) || key.startsWith(KEY_KILLED_AT_PREFIX))
        edit.remove(key);
    edit.apply();
    enqueue(context);
  }

  /**
   * Builds these maps' files (with those for the current avoid options) before the others. With
   * {@code preempt}, a build of another map that isn't half done yet is stopped and done later.
   */
  public static void prioritize(@NonNull Context context, @NonNull Collection<String> countryIds, boolean preempt)
  {
    if (countryIds.isEmpty())
      return;
    sPriority.addAll(countryIds);
    final String now = sBuilding;
    if (preempt && now != null && !sPriority.contains(now) && PlusOverlay.nativeProgress() < 0.5)
    {
      Logger.i(TAG, "Pausing the build for " + now + " for a map a route needs");
      sPreempted = true;
      PlusOverlay.nativeCancel();
    }
    enqueue(context);
  }

  @NonNull
  private final Context mContext;

  public PlusOverlayWork(@NonNull Context context, @NonNull WorkerParameters workerParams)
  {
    super(context, workerParams);
    mContext = context;
  }

  public static void createNotificationChannel(@NonNull Context context)
  {
    final NotificationChannelCompat channel =
        new NotificationChannelCompat.Builder(CHANNEL_ID, NotificationManagerCompat.IMPORTANCE_LOW)
            .setName(context.getString(R.string.plus_overlay_channel))
            .setShowBadge(false)
            .setVibrationEnabled(false)
            .setLightsEnabled(false)
            .setSound(null, null)
            .build();
    NotificationManagerCompat.from(context).createNotificationChannel(channel);
  }

  /**
   * Queues a run after any running one. A run with nothing to build ends at once without a
   * notification.
   */
  public static void enqueue(@NonNull Context context)
  {
    // No constraints: waiting for a charged battery or free storage could leave maps unprepared for days.
    final OneTimeWorkRequest request = new OneTimeWorkRequest.Builder(PlusOverlayWork.class).build();
    WorkManager.getInstance(context).enqueueUniqueWork(WORK_NAME, ExistingWorkPolicy.APPEND_OR_REPLACE, request);
  }

  /**
   * Another run in |delay| minutes, for failed builds or killed maps waiting their turn (a separate request,
   * so new runs don't wait for it).
   */
  private static void scheduleRetry(@NonNull Context context, long delay)
  {
    Logger.i(TAG, "Trying the remaining builds again in " + delay + " min");
    final OneTimeWorkRequest request = new OneTimeWorkRequest.Builder(Retry.class)
                                           .setInitialDelay(delay, TimeUnit.MINUTES)
                                           .build();
    WorkManager.getInstance(context).enqueueUniqueWork(RETRY_WORK_NAME, ExistingWorkPolicy.REPLACE, request);
  }

  /** Queues a build run (PlusOverlayWork) when it fires. */
  public static class Retry extends Worker
  {
    public Retry(@NonNull Context context, @NonNull WorkerParameters params)
    {
      super(context, params);
    }

    @NonNull
    @Override
    public Result doWork()
    {
      enqueue(getApplicationContext());
      return Result.success();
    }
  }

  // Started without the app (after a reboot, or the system restarting the work): start the core here, as
  // Android Auto does, so the maps get prepared without waiting for the app to be opened.
  private boolean ensureCore()
  {
    final MwmApplication app = MwmApplication.from(mContext);
    if (app.getOrganicMaps().arePlatformAndCoreInitialized())
      return true;
    final CountDownLatch done = new CountDownLatch(1);
    new Handler(Looper.getMainLooper()).post(() -> {
      try
      {
        if (!app.initOrganicMaps(done::countDown))
          done.countDown();
      }
      catch (Exception e)
      {
        Logger.e(TAG, "Can't start the core for building", e);
        done.countDown();
      }
    });
    try
    {
      done.await(2, TimeUnit.MINUTES);
    }
    catch (InterruptedException e)
    {
      Thread.currentThread().interrupt();
    }
    return app.getOrganicMaps().arePlatformAndCoreInitialized();
  }

  @NonNull
  @Override
  public Result doWork()
  {
    if (!ensureCore())
    {
      // The app queues a run again once it starts.
      Logger.w(TAG, "Core is not initialized, will run when the app starts");
      return Result.success();
    }

    PlusOverlay.nativeDeleteOrphans(false /* keepAllAvoid */);
    final SharedPreferences prefs = MwmApplication.prefs(mContext);
    final String killed = prefs.getString(KEY_BUILDING, null);
    if (killed != null)
    {
      final int n = prefs.getInt(KEY_KILLED_PREFIX + killed, 0) + 1;
      Logger.w(TAG, "The build for " + killed + " didn't finish last time (" + n + ")");
      prefs.edit()
          .remove(KEY_BUILDING)
          .putInt(KEY_KILLED_PREFIX + killed, n)
          .putLong(KEY_KILLED_AT_PREFIX + killed, System.currentTimeMillis())
          .apply();
    }
    if (nextMap(new HashSet<>()) == null)
    {
      if (mKilledWaitMs > 0)
        scheduleRetry(mContext, Math.max(1, mKilledWaitMs / 60_000L + 1));
      return Result.success();
    }

    int built = 0;
    int failed = 0;
    final Set<String> attempted = new HashSet<>();
    final ScheduledExecutorService ticker = Executors.newSingleThreadScheduledExecutor();
    try
    {
      while (!isStopped())
      {
        final String next = nextMap(attempted);
        if (next == null)
          break;
        attempted.add(next);

        final String countryId = next;
        final String name = MapManager.nativeGetName(countryId);
        try
        {
          setForegroundAsync(foregroundInfo(name, 0, "", false)).get();
        }
        catch (Exception e)
        {
          // Not allowed to start a foreground service right now: build anyway, without the notification.
          Logger.w(TAG, "Can't show the progress notification", e);
        }
        final var tick = ticker.scheduleWithFixedDelay(() -> {
          if (isStopped())
            return;
          final int building = PlusOverlay.nativeBuildingAvoidMask();
          if (building != 0 && building != PlusOverlay.nativeCurrentAvoidMask())
          {
            // Avoid options changed meanwhile: the run queued by that change builds the right files.
            Logger.i(TAG, "Avoid options changed, stopping the build for the old ones");
            PlusOverlay.nativeCancel();
            return;
          }
          updateProgress(name, PlusOverlay.nativeProgress(), PlusOverlay.nativeStage(), building != 0);
        }, 1, 1, TimeUnit.SECONDS);

        final int threads = killedCount(countryId) > 0 ? 1 : BUILD_THREADS;
        Logger.i(TAG, "Building the fast-routing overlay for " + countryId + " with " + threads + " threads");
        prefs.edit().putString(KEY_BUILDING, countryId).commit();
        sBuilding = countryId;
        final int r = PlusOverlay.nativeBuild(countryId, threads, true /* withAvoid */);
        sBuilding = null;
        final boolean preempted = sPreempted;
        sPreempted = false;
        prefs.edit().remove(KEY_BUILDING).apply();
        tick.cancel(false);
        Logger.i(TAG, "Overlay build for " + countryId + ": " + r);
        if (r == PlusOverlay.BUILD_CANCELLED)
        {
          if (preempted && !isStopped())
          {
            // Paused for a map a route needs: that one next, this one again later.
            attempted.remove(countryId);
            continue;
          }
          break;
        }
        sPriority.remove(countryId);
        if (r == PlusOverlay.BUILD_OK || r == PlusOverlay.BUILD_UP_TO_DATE)
        {
          prefs.edit().remove(KEY_KILLED_PREFIX + countryId).remove(KEY_KILLED_AT_PREFIX + countryId).apply();
          if (r == PlusOverlay.BUILD_OK)
            ++built;
          final MapReadyListener listener = sMapReady;
          if (listener != null)
            new Handler(Looper.getMainLooper()).post(() -> listener.onMapReady(countryId));
        }
        else if (r == PlusOverlay.BUILD_FAILED)
          ++failed;
      }
    }
    finally
    {
      ticker.shutdownNow();
    }

    if (!isStopped() && (built > 0 || failed > 0))
      notifyResult(built, failed);
    if (failed > 0)
    {
      final int runs = prefs.getInt(KEY_FAILED_RUNS, 0) + 1;
      prefs.edit().putInt(KEY_FAILED_RUNS, runs).apply();
      scheduleRetry(mContext, Math.min(MAX_RETRY_DELAY_MIN, (long) Math.pow(4, Math.min(runs - 1, 6))));
    }
    else
    {
      if (!isStopped())
        prefs.edit().remove(KEY_FAILED_RUNS).apply();
      if (mKilledWaitMs > 0)
        scheduleRetry(mContext, Math.max(1, mKilledWaitMs / 60_000L + 1));
    }
    return Result.success();
  }

  // Next map to build: those a route needs, then the others (base files before avoid-option files), maps
  // whose build the system killed last. Every build makes both files a map lacks.
  @Nullable
  private String nextMap(@NonNull Set<String> attempted)
  {
    final String[] priority;
    synchronized (sPriority)
    {
      priority = sPriority.toArray(new String[0]);
    }
    for (String id : priority)
    {
      if (!PlusOverlay.nativeNeedsBuild(id))
        sPriority.remove(id);
      else if (!attempted.contains(id))
        return id;
    }
    String killed = null;
    for (String id : mapsToBuild())
    {
      if (attempted.contains(id))
        continue;
      if (killedCount(id) == 0)
        return id;
      final long wait = killedWaitMs(id);
      if (wait > 0)
      {
        mKilledWaitMs = mKilledWaitMs == 0 ? wait : Math.min(mKilledWaitMs, wait);
        continue;
      }
      if (killed == null)
        killed = id;
    }
    return killed;
  }

  // Shortest wait of a killed map skipped in this run (0: none).
  private long mKilledWaitMs = 0;

  private long killedWaitMs(@NonNull String countryId)
  {
    final int kills = killedCount(countryId);
    if (kills < KILLS_BEFORE_WAIT)
      return 0;
    final long pause = Math.min(MAX_RETRY_DELAY_MIN, 15L << Math.min(kills - KILLS_BEFORE_WAIT, 5)) * 60_000L;
    final long at = MwmApplication.prefs(mContext).getLong(KEY_KILLED_AT_PREFIX + countryId, 0);
    return Math.max(0, at + pause - System.currentTimeMillis());
  }

  private int killedCount(@NonNull String countryId)
  {
    return MwmApplication.prefs(mContext).getInt(KEY_KILLED_PREFIX + countryId, 0);
  }

  // Maps that need building: base files first, then files for the current avoid options; the map you are
  // in and the nearest first.
  @NonNull
  private String[] mapsToBuild()
  {
    final android.location.Location loc = MwmApplication.from(mContext).getLocationHelper().getSavedLocation();
    if (loc == null)
      return PlusOverlay.nativeMapsToBuild(false, 0, 0, true /* withAvoid */);
    return PlusOverlay.nativeMapsToBuild(true, loc.getLatitude(), loc.getLongitude(), true /* withAvoid */);
  }

  @Override
  public void onStopped()
  {
    // Cancel button, the system stopping the work, or the constraints no longer met.
    PlusOverlay.nativeCancel();
    super.onStopped();
  }

  @NonNull
  private Notification progressNotification(@NonNull String name, double progress, @NonNull String stage,
                                            boolean avoidOptions)
  {
    final PendingIntent cancel = WorkManager.getInstance(mContext).createCancelPendingIntent(getId());
    final int percent = (int) Math.round(progress * 100);
    return new NotificationCompat.Builder(mContext, CHANNEL_ID)
        .setSmallIcon(R.drawable.ic_logo_small)
        .setColor(ContextCompat.getColor(mContext, R.color.notification))
        .setContentTitle(mContext.getString(avoidOptions ? R.string.plus_overlay_building_avoid
                                                         : R.string.plus_overlay_building, name))
        .setContentText(stage.isEmpty() ? mContext.getString(R.string.plus_overlay_building_text)
                                        : stage + " · " + percent + "%")
        .setSubText(sBuilding != null && PlusRouteStatus.isOnRoute(sBuilding)
                        ? mContext.getString(R.string.plus_overlay_for_route)
                        : null)
        .setProgress(100, percent, progress <= 0)
        .setOngoing(true)
        .setOnlyAlertOnce(true)
        .setSilent(true)
        .setCategory(NotificationCompat.CATEGORY_PROGRESS)
        .addAction(android.R.drawable.ic_menu_close_clear_cancel, mContext.getString(R.string.cancel), cancel)
        .build();
  }

  @NonNull
  private ForegroundInfo foregroundInfo(@NonNull String name, double progress, @NonNull String stage,
                                        boolean avoidOptions)
  {
    final Notification n = progressNotification(name, progress, stage, avoidOptions);
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q)
      return new ForegroundInfo(PROGRESS_NOTIFICATION_ID, n, ServiceInfo.FOREGROUND_SERVICE_TYPE_DATA_SYNC);
    return new ForegroundInfo(PROGRESS_NOTIFICATION_ID, n);
  }

  private void updateProgress(@NonNull String name, double progress, @NonNull String stage, boolean avoidOptions)
  {
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU
        && ContextCompat.checkSelfPermission(mContext, POST_NOTIFICATIONS) != PERMISSION_GRANTED)
      return;
    NotificationManagerCompat.from(mContext).notify(PROGRESS_NOTIFICATION_ID,
                                                    progressNotification(name, progress, stage, avoidOptions));
  }

  private void notifyResult(int built, int failed)
  {
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU
        && ContextCompat.checkSelfPermission(mContext, POST_NOTIFICATIONS) != PERMISSION_GRANTED)
      return;
    final String text = failed == 0 ? mContext.getString(R.string.plus_overlay_done, built)
                                    : mContext.getString(R.string.plus_overlay_failed, failed);
    final Notification n = new NotificationCompat.Builder(mContext, CHANNEL_ID)
                               .setSmallIcon(R.drawable.ic_logo_small)
                               .setColor(ContextCompat.getColor(mContext, R.color.notification))
                               .setContentTitle(mContext.getString(R.string.plus_overlay_channel))
                               .setContentText(text)
                               .setAutoCancel(true)
                               .setSilent(true)
                               .build();
    NotificationManagerCompat.from(mContext).notify(RESULT_NOTIFICATION_ID, n);
  }
}
