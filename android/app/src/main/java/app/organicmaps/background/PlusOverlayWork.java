package app.organicmaps.background;

import static android.Manifest.permission.POST_NOTIFICATIONS;
import static android.content.pm.PackageManager.PERMISSION_GRANTED;

import android.app.Notification;
import android.app.PendingIntent;
import android.content.Context;
import android.content.pm.ServiceInfo;
import android.os.Build;
import androidx.annotation.NonNull;
import androidx.core.app.NotificationChannelCompat;
import androidx.core.app.NotificationCompat;
import androidx.core.app.NotificationManagerCompat;
import androidx.core.content.ContextCompat;
import androidx.work.Constraints;
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
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Comaps++: builds the overlay files for fast car routing on the phone, one map at a time, after a map
 * download or update (and at app start for anything missed). Shows a progress notification with a
 * Cancel button; cancelling stops the running build and leaves the router on the normal search for
 * that map until the next run.
 */
public class PlusOverlayWork extends Worker
{
  private static final String TAG = PlusOverlayWork.class.getSimpleName();
  private static final String WORK_NAME = "PlusOverlayBuild";
  private static final String CHANNEL_ID = "plus_overlay";
  private static final int PROGRESS_NOTIFICATION_ID = 7301;
  private static final int RESULT_NOTIFICATION_ID = 7302;
  private static final int BUILD_THREADS = 2;

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
    final Constraints constraints =
        new Constraints.Builder().setRequiresBatteryNotLow(true).setRequiresStorageNotLow(true).build();
    final OneTimeWorkRequest request =
        new OneTimeWorkRequest.Builder(PlusOverlayWork.class).setConstraints(constraints).build();
    WorkManager.getInstance(context).enqueueUniqueWork(WORK_NAME, ExistingWorkPolicy.APPEND_OR_REPLACE, request);
  }

  @NonNull
  @Override
  public Result doWork()
  {
    if (!MwmApplication.from(mContext).getOrganicMaps().arePlatformAndCoreInitialized())
    {
      // Started without the app (e.g. after a reboot): the app queues a run again once it starts.
      Logger.i(TAG, "Core is not initialized, will run when the app starts");
      return Result.success();
    }

    PlusOverlay.nativeDeleteOrphans();
    if (PlusOverlay.nativeMapsToBuild().length == 0)
      return Result.success();

    int built = 0;
    int failed = 0;
    final Set<String> attempted = new HashSet<>();
    final ScheduledExecutorService ticker = Executors.newSingleThreadScheduledExecutor();
    try
    {
      while (!isStopped())
      {
        String next = null;
        for (String id : PlusOverlay.nativeMapsToBuild())
        {
          if (!attempted.contains(id))
          {
            next = id;
            break;
          }
        }
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

        Logger.i(TAG, "Building the fast-routing overlay for " + countryId);
        final int r = PlusOverlay.nativeBuild(countryId, BUILD_THREADS);
        tick.cancel(false);
        Logger.i(TAG, "Overlay build for " + countryId + ": " + r);
        if (r == PlusOverlay.BUILD_CANCELLED)
          break;
        if (r == PlusOverlay.BUILD_OK)
          ++built;
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
    return Result.success();
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
