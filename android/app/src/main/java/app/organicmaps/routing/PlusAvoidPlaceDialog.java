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
import java.util.ArrayList;
import java.util.List;
import java.util.function.IntConsumer;

/**
 * Comaps++ roadmap item 1: "Avoid" on the place page. Keeps car routes off the road at that point, or out of
 * an area around it. Nothing is prepared: the next route already avoids it.
 */
public final class PlusAvoidPlaceDialog
{
  static final int[] RADII_M = {100, 250, 500, 1000, 2000, 5000};
  static final int DEFAULT_RADIUS_INDEX = 1;

  private PlusAvoidPlaceDialog() {}

  public static void show(@NonNull Activity activity, double lat, double lon)
  {
    final String road = PlusAvoidPlaces.nativeRoadAt(lat, lon);
    final List<CharSequence> items = new ArrayList<>();
    if (road != null)
      items.add(road.isEmpty() ? activity.getString(R.string.plus_avoid_this_road_unnamed)
                               : activity.getString(R.string.plus_avoid_this_road, road));
    items.add(activity.getString(R.string.plus_avoid_this_area));
    new MaterialAlertDialogBuilder(activity)
        .setTitle(R.string.plus_avoid_title)
        .setItems(items.toArray(new CharSequence[0]),
                  (dialog, which) -> {
                    if (road != null && which == 0)
                      add(activity, PlusAvoidPlaces.Place.road(lat, lon, road));
                    else
                      chooseRadius(activity, RADII_M[DEFAULT_RADIUS_INDEX],
                                   radius -> add(activity, PlusAvoidPlaces.Place.area(lat, lon, radius)));
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

  private static void add(@NonNull Activity activity, @NonNull PlusAvoidPlaces.Place place)
  {
    PlusAvoidPlaces.add(place);
    Toast.makeText(activity, activity.getString(R.string.plus_avoid_added, describe(activity, place)),
                   Toast.LENGTH_LONG)
        .show();
    refreshRoute();
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
