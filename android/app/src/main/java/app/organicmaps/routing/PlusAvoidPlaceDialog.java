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
import java.util.function.IntConsumer;

/**
 * Comaps++ roadmap item 1: "Avoid" on the place page while a car route is shown. For a settlement the route
 * passes through it keeps routes out of that settlement; for any other point on the route, off the route's
 * road there. Nothing is prepared: the route is recalculated straight away.
 */
public final class PlusAvoidPlaceDialog
{
  static final int[] RADII_M = {100, 250, 500, 1000, 2000, 5000, 10000, 20000};
  static final int DEFAULT_RADIUS_INDEX = 1;

  private PlusAvoidPlaceDialog() {}

  /** Whether the place page should offer Avoid (see PlusAvoidPlaces.nativePrepareRouteAvoid). */
  public static boolean available()
  {
    return PlusAvoidPlaces.nativePrepareRouteAvoid();
  }

  /** Asks to confirm the place prepared by available(), then adds it and recalculates the route. */
  public static void show(@NonNull Activity activity)
  {
    final double[] n = PlusAvoidPlaces.nativePendingNumbers();
    if (n.length < 2)
      return;
    final PlusAvoidPlaces.Place place = new PlusAvoidPlaces.Place();
    place.kind = (int) n[0];
    place.radiusM = (int) n[1];
    place.name = PlusAvoidPlaces.nativePendingName();
    new MaterialAlertDialogBuilder(activity)
        .setTitle(R.string.plus_avoid_title)
        .setMessage(place.kind == PlusAvoidPlaces.KIND_ROAD
                        ? activity.getString(R.string.plus_avoid_confirm_road,
                                             place.name.isEmpty() ? activity.getString(R.string.plus_avoid_this_road)
                                                                  : place.name)
                        : activity.getString(R.string.plus_avoid_confirm_area, describe(activity, place),
                                             distance(place.radiusM)))
        .setPositiveButton(R.string.plus_avoid_button,
                           (dialog, which) -> {
                             PlusAvoidPlaces.nativeAddPending();
                             Toast
                                 .makeText(activity, activity.getString(R.string.plus_avoid_added, describe(activity, place)),
                                           Toast.LENGTH_LONG)
                                 .show();
                             refreshRoute();
                           })
        .setNegativeButton(R.string.cancel, null)
        .show();
  }

  public static void chooseRadius(@NonNull Context context, int current, @NonNull IntConsumer onChosen)
  {
    final CharSequence[] labels = new CharSequence[RADII_M.length];
    int checked = DEFAULT_RADIUS_INDEX;
    for (int i = 0; i < RADII_M.length; ++i)
    {
      labels[i] = context.getString(R.string.plus_avoid_radius_item, distance(RADII_M[i]));
      if (RADII_M[i] == current)
        checked = i;
    }
    new MaterialAlertDialogBuilder(context)
        .setTitle(R.string.plus_avoid_radius_title)
        .setSingleChoiceItems(labels, checked,
                              (dialog, which) -> {
                                dialog.dismiss();
                                onChosen.accept(RADII_M[which]);
                              })
        .setNegativeButton(R.string.cancel, null)
        .show();
  }

  /** The current car route, calculated again with the changed list. */
  public static void refreshRoute()
  {
    final RoutingController rc = RoutingController.get();
    if (!rc.isVehicleRouterType())
      return;
    if (rc.isNavigating())
      PlusOverlay.nativeRefreshRoute();
    else if (rc.isPlanning())
      rc.rebuildLastRoute();
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
