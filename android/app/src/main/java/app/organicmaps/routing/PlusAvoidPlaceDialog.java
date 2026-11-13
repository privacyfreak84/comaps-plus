package app.organicmaps.routing;

import android.app.Activity;
import android.content.Context;
import android.widget.Toast;
import androidx.annotation.NonNull;
import app.organicmaps.R;
import app.organicmaps.sdk.routing.PlusAvoidPlaces;
import app.organicmaps.sdk.routing.PlusOverlay;
import app.organicmaps.sdk.routing.RoutingController;
import com.google.android.material.dialog.MaterialAlertDialogBuilder;

/**
 * Tabula roadmap item 1: Avoid on the place page while a car route is shown. For a settlement the route
 * passes through it keeps this route out of that settlement; for any other point on the route, off the route's
 * road there. Tapping a place the route avoids (drawn in red) offers Don't avoid. The places belong to this route
 * only and go when it's closed. Nothing is prepared: the route is recalculated straight away.
 */
public final class PlusAvoidPlaceDialog
{
  private PlusAvoidPlaceDialog() {}

  /** Whether the place page should offer Avoid or Don't avoid (see PlusAvoidPlaces.nativePrepareRouteAvoid). */
  public static boolean available()
  {
    return PlusAvoidPlaces.nativePrepareRouteAvoid();
  }

  /** After available(): true if the place page should offer Don't avoid rather than Avoid. */
  public static boolean isRemoval()
  {
    return PlusAvoidPlaces.nativeIsPendingRemoval();
  }

  /** Asks to confirm the change prepared by available(), applies it to this route and recalculates it. */
  public static void show(@NonNull Activity activity)
  {
    final double[] n = PlusAvoidPlaces.nativePendingNumbers();
    if (n.length < 2)
      return;
    final boolean removal = PlusAvoidPlaces.nativeIsPendingRemoval();
    final PlusAvoidPlaces.Place place = new PlusAvoidPlaces.Place();
    place.kind = (int) n[0];
    place.radiusM = (int) n[1];
    place.name = PlusAvoidPlaces.nativePendingName();
    final String what = place.kind == PlusAvoidPlaces.KIND_ROAD && place.name.isEmpty()
                          ? activity.getString(R.string.plus_avoid_this_road)
                          : describe(activity, place);
    final String message;
    if (removal)
      message = activity.getString(R.string.plus_avoid_confirm_remove, what);
    else if (place.kind == PlusAvoidPlaces.KIND_ROAD)
      message = activity.getString(R.string.plus_avoid_confirm_road, what);
    else
      message = activity.getString(R.string.plus_avoid_confirm_area, what, distance(place.radiusM));
    new MaterialAlertDialogBuilder(activity)
        .setTitle(removal ? R.string.plus_avoid_remove_title : R.string.plus_avoid_title)
        .setMessage(message)
        .setPositiveButton(removal ? R.string.plus_avoid_remove_button : R.string.plus_avoid_button,
                           (dialog, which) -> {
                             PlusAvoidPlaces.nativeApplyPending();
                             Toast
                                 .makeText(activity,
                                           activity.getString(removal ? R.string.plus_avoid_removed
                                                                      : R.string.plus_avoid_added,
                                                              what),
                                           Toast.LENGTH_SHORT)
                                 .show();
                             refreshRoute();
                           })
        .setNegativeButton(R.string.cancel, null)
        .show();
  }

  /** The current car route, calculated again with its changed avoided places. */
  public static void refreshRoute()
  {
    final RoutingController rc = RoutingController.get();
    if (!rc.isVehicleRouterType())
      return;
    if (rc.isNavigating())
      PlusOverlay.nativeRefreshRoute();
    else if (rc.isPlanning())
      rc.checkAndBuildRoute();  // keeps the stops, unlike rebuildLastRoute()
  }

  @NonNull
  public static String describe(@NonNull Context context, @NonNull PlusAvoidPlaces.Place place)
  {
    if (place.kind == PlusAvoidPlaces.KIND_ROAD)
      return place.name.isEmpty() ? context.getString(R.string.plus_avoid_road_unnamed)
                                  : context.getString(R.string.plus_avoid_road_named, place.name);
    if (!place.name.isEmpty())
      return place.name;
    return context.getString(R.string.plus_avoid_area_named, distance(place.radiusM));
  }

  @NonNull
  static String distance(int metres)
  {
    if (metres < 1000)
      return metres + " m";
    return (metres % 1000 == 0 ? String.valueOf(metres / 1000) : String.format(java.util.Locale.US, "%.1f",
                                                                                 metres / 1000.0))
        + " km";
  }
}
