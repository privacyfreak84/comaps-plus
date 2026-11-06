package app.organicmaps.sdk.routing;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import app.organicmaps.sdk.Router;
import app.organicmaps.sdk.settings.RoadType;
import app.organicmaps.sdk.util.log.Logger;
import java.util.HashSet;
import java.util.Set;

public final class RoutingOptions
{
  private static final String TAG = RoutingOptions.class.getSimpleName();

  // Comaps++: told about every change of the avoid options (the app prepares fast routes for them).
  @Nullable
  private static volatile Runnable sOnChanged;

  public static void setOnChangedListener(@Nullable Runnable listener)
  {
    sOnChanged = listener;
  }

  private static void notifyChanged()
  {
    final Runnable r = sOnChanged;
    if (r != null)
      r.run();
  }

  public static void addOption(@NonNull RoadType roadType, @NonNull Router router)
  {
    nativeAddOption(roadType.ordinal(), router.ordinal());
    notifyChanged();
  }

  public static void removeOption(@NonNull RoadType roadType, @NonNull Router router)
  {
    nativeRemoveOption(roadType.ordinal(), router.ordinal());
    notifyChanged();
  }

  public static boolean hasOption(@NonNull RoadType roadType, @NonNull Router router)
  {
    if (router == Router.Ruler)
      return false;
    return nativeHasOption(roadType.ordinal(), router.ordinal());
  }

  public static boolean hasAnyOptions(@NonNull Router router)
  {
    for (RoadType each : RoadType.values())
    {
      if (hasOption(each, router))
        return true;
    }
    return false;
  }

  @NonNull
  public static Set<RoadType> getActiveRoadTypes(@NonNull Router router)
  {
    Set<RoadType> roadTypes = new HashSet<>();
    for (RoadType each : RoadType.values())
    {
      if (hasOption(each, router))
        roadTypes.add(each);
    }
    return roadTypes;
  }

  private RoutingOptions() throws IllegalAccessException
  {
    throw new IllegalAccessException("RoutingOptions is a utility class and should not be instantiated");
  }
  private static native void nativeAddOption(int option, int vehicle);

  private static native void nativeRemoveOption(int option, int vehicle);

  private static native boolean nativeHasOption(int option, int vehicle);
}
