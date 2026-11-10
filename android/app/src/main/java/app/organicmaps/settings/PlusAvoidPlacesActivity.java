package app.organicmaps.settings;

import android.content.Context;
import android.content.Intent;
import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;
import app.organicmaps.base.BaseMwmFragmentActivity;

/** Comaps++: the list of avoided roads and places. */
public class PlusAvoidPlacesActivity extends BaseMwmFragmentActivity
{
  public static void start(@NonNull Context context)
  {
    context.startActivity(new Intent(context, PlusAvoidPlacesActivity.class));
  }

  @Override
  protected Class<? extends Fragment> getFragmentClass()
  {
    return PlusAvoidPlacesFragment.class;
  }
}
