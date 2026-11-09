package app.organicmaps.background;

import android.content.Context;
import android.os.SystemClock;
import android.widget.Toast;
import androidx.annotation.MainThread;
import androidx.annotation.NonNull;
import app.organicmaps.R;
import app.organicmaps.sdk.downloader.MapManager;
import app.organicmaps.sdk.routing.PlusOverlay;
import app.organicmaps.sdk.routing.RoutingController;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Comaps++: what the user sees when a car route crosses maps whose fast-route files aren't built yet.
 * Such a route is still fast to calculate, but parts of it rest on estimates, so it may be a few minutes
 * off. The user is told so once per set of maps; those maps are built first, and when they are ready the
 * route is calculated again by itself.
 */
public final class PlusRouteStatus
{
  private static final long REMIND_AFTER_MS = 10 * 60 * 1000;

  // Maps the current car route crossed on estimates (read by the build worker for its notification).
  private static final Set<String> sApprox = Collections.synchronizedSet(new HashSet<>());
  private static String sToldKey = "";
  private static long sToldAt;

  private PlusRouteStatus() {}

  /** Whether the current route waits for this map's files. */
  public static boolean isOnRoute(@NonNull String countryId)
  {
    return sApprox.contains(countryId);
  }

  /** After each car route result. */
  @MainThread
  public static void onCarRoute(@NonNull Context context)
  {
    // Maps used without their file for the current avoid options: the route is exact, the file makes the
    // next ones faster. Built next, without interrupting another map's build.
    PlusOverlayWork.prioritize(context, Arrays.asList(PlusOverlay.nativeLastAvoidMaps()), false /* preempt */);
    final String[] r = PlusOverlay.nativeLastEngine();
    sApprox.clear();
    if (r.length < 4 || !"approximate".equals(r[0]))
      return;
    final List<String> ids = Arrays.asList(r).subList(3, r.length);
    sApprox.addAll(ids);
    PlusOverlayWork.prioritize(context, ids, true /* preempt */);

    final String names = names(ids);
    final long now = SystemClock.elapsedRealtime();
    if (names.equals(sToldKey) && now - sToldAt < REMIND_AFTER_MS)
      return;  // reroutes while driving: said already
    sToldKey = names;
    sToldAt = now;
    Toast.makeText(context, context.getString(R.string.plus_route_approximate, names), Toast.LENGTH_LONG).show();
  }

  /** A map's files are ready: if the current route waits for them, it is calculated again. */
  @MainThread
  public static void onMapReady(@NonNull Context context, @NonNull String countryId)
  {
    if (!sApprox.remove(countryId))
      return;
    final RoutingController rc = RoutingController.get();
    if (!rc.isVehicleRouterType() || !(rc.isBuilt() || rc.isNavigating()))
    {
      sApprox.clear();
      return;
    }
    Toast.makeText(context, context.getString(R.string.plus_route_refreshing, MapManager.nativeGetName(countryId)),
                   Toast.LENGTH_LONG)
        .show();
    PlusOverlay.nativeRefreshRoute();
  }

  @NonNull
  private static String names(@NonNull List<String> ids)
  {
    final List<String> names = new ArrayList<>();
    for (String id : ids)
      names.add(MapManager.nativeGetName(id));
    return android.text.TextUtils.join(", ", names);
  }
}
