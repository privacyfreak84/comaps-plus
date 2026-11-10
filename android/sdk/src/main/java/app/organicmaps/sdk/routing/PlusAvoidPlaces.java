package app.organicmaps.sdk.routing;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import java.util.ArrayList;
import java.util.List;

/**
 * Comaps++: roads and areas kept out of car routes. The list is saved in the core and read by every car route;
 * nothing has to be prepared for it. A place at a route's start, finish or a stop doesn't apply to that route.
 */
public final class PlusAvoidPlaces
{
  public static final int KIND_AREA = 0;
  public static final int KIND_ROAD = 1;
  private static final int NUMBERS_PER_PLACE = 5;

  public static final class Place
  {
    public int kind;
    public double lat;
    public double lon;
    /** Area only, metres. */
    public int radiusM;
    public boolean enabled = true;
    /** Road: its name or number ("" for an unnamed road). Area: empty. */
    @NonNull
    public String name = "";

    @NonNull
    public static Place area(double lat, double lon, int radiusM)
    {
      final Place p = new Place();
      p.kind = KIND_AREA;
      p.lat = lat;
      p.lon = lon;
      p.radiusM = radiusM;
      return p;
    }

    @NonNull
    public static Place road(double lat, double lon, @NonNull String name)
    {
      final Place p = new Place();
      p.kind = KIND_ROAD;
      p.lat = lat;
      p.lon = lon;
      p.name = name;
      return p;
    }
  }

  private PlusAvoidPlaces() {}

  @NonNull
  public static List<Place> load()
  {
    final double[] n = nativeGetNumbers();
    final String[] names = nativeGetNames();
    final List<Place> places = new ArrayList<>();
    for (int i = 0; i < names.length && (i + 1) * NUMBERS_PER_PLACE <= n.length; ++i)
    {
      final Place p = new Place();
      final int o = i * NUMBERS_PER_PLACE;
      p.kind = (int) n[o];
      p.lat = n[o + 1];
      p.lon = n[o + 2];
      p.radiusM = (int) n[o + 3];
      p.enabled = n[o + 4] != 0;
      p.name = names[i] == null ? "" : names[i];
      places.add(p);
    }
    return places;
  }

  public static void save(@NonNull List<Place> places)
  {
    final double[] n = new double[places.size() * NUMBERS_PER_PLACE];
    final String[] names = new String[places.size()];
    for (int i = 0; i < places.size(); ++i)
    {
      final Place p = places.get(i);
      final int o = i * NUMBERS_PER_PLACE;
      n[o] = p.kind;
      n[o + 1] = p.lat;
      n[o + 2] = p.lon;
      n[o + 3] = p.radiusM;
      n[o + 4] = p.enabled ? 1 : 0;
      names[i] = p.name;
    }
    nativeSet(n, names);
  }

  public static void add(@NonNull Place place)
  {
    final List<Place> places = load();
    places.add(place);
    save(places);
  }

  public static int enabledCount()
  {
    int count = 0;
    for (Place p : load())
      if (p.enabled)
        ++count;
    return count;
  }

  /**
   * The car road "avoid this road" would pick at this point: its name or number ("" if it has neither), or
   * null if no car road is near enough.
   */
  @Nullable
  public static native String nativeRoadAt(double lat, double lon);

  /** How many avoided places applied to the last car route (covered roads, not at its ends or stops). */
  public static native int nativeLastApplied();

  @NonNull
  private static native double[] nativeGetNumbers();

  @NonNull
  private static native String[] nativeGetNames();

  private static native void nativeSet(@NonNull double[] numbers, @NonNull String[] names);
}
