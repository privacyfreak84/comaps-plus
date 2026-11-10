package app.organicmaps.settings;

import android.content.Context;
import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.LinearLayout;
import android.widget.TextView;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import app.organicmaps.R;
import app.organicmaps.base.BaseMwmToolbarFragment;
import app.organicmaps.routing.PlusAvoidPlaceDialog;
import app.organicmaps.sdk.routing.PlusAvoidPlaces;
import com.google.android.material.dialog.MaterialAlertDialogBuilder;
import com.google.android.material.materialswitch.MaterialSwitch;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Comaps++ roadmap item 1: roads and areas kept out of car routes. Places are added from the map ("Avoid" on a
 * place page); here they can be switched off, shown on the map, resized or deleted.
 */
public class PlusAvoidPlacesFragment extends BaseMwmToolbarFragment
{
  @Nullable
  private LinearLayout mList;

  @Override
  public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup container,
                           @Nullable Bundle savedInstanceState)
  {
    return inflater.inflate(R.layout.fragment_plus_avoid_places, container, false);
  }

  @Override
  public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState)
  {
    super.onViewCreated(view, savedInstanceState);
    getToolbarController().setTitle(R.string.plus_avoid_list_title);
    mList = view.findViewById(R.id.plus_avoid_list);
  }

  @Override
  public void onResume()
  {
    super.onResume();
    refresh();
  }

  private void refresh()
  {
    if (mList == null)
      return;
    mList.removeAllViews();
    final List<PlusAvoidPlaces.Place> places = PlusAvoidPlaces.load();
    final LayoutInflater inflater = LayoutInflater.from(requireContext());
    if (places.isEmpty())
    {
      final TextView empty = new TextView(requireContext());
      final int pad = getResources().getDimensionPixelSize(R.dimen.margin_base);
      empty.setPadding(pad, pad, pad, pad);
      empty.setText(R.string.plus_avoid_empty);
      mList.addView(empty);
      return;
    }
    for (int i = 0; i < places.size(); ++i)
    {
      final int index = i;
      final PlusAvoidPlaces.Place p = places.get(i);
      final View row = inflater.inflate(R.layout.item_plus_avoid_place, mList, false);
      ((TextView) row.findViewById(R.id.plus_avoid_title)).setText(PlusAvoidPlaceDialog.describe(requireContext(), p));
      ((TextView) row.findViewById(R.id.plus_avoid_subtitle))
          .setText(String.format(Locale.US, "%.5f, %.5f", p.lat, p.lon));
      final MaterialSwitch enabled = row.findViewById(R.id.plus_avoid_enabled);
      enabled.setChecked(p.enabled);
      enabled.setOnCheckedChangeListener((button, checked) -> update(list -> list.get(index).enabled = checked));
      row.setOnClickListener(v -> showActions(index, p));
      mList.addView(row);
    }
  }

  private interface Change
  {
    void apply(@NonNull List<PlusAvoidPlaces.Place> places);
  }

  private void update(@NonNull Change change)
  {
    final List<PlusAvoidPlaces.Place> places = new ArrayList<>(PlusAvoidPlaces.load());
    change.apply(places);
    PlusAvoidPlaces.save(places);
    PlusAvoidPlaceDialog.refreshRoute();
  }

  private void showActions(int index, @NonNull PlusAvoidPlaces.Place place)
  {
    final Context context = requireContext();
    final List<CharSequence> items = new ArrayList<>();
    final List<Runnable> actions = new ArrayList<>();
    items.add(getString(R.string.plus_avoid_show_on_map));
    actions.add(() -> showOnMap(place));
    if (place.kind == PlusAvoidPlaces.KIND_AREA)
    {
      items.add(getString(R.string.plus_avoid_change_radius));
      actions.add(() -> PlusAvoidPlaceDialog.chooseRadius(context, place.radiusM, radius -> {
        update(list -> list.get(index).radiusM = radius);
        refresh();
      }));
    }
    items.add(getString(R.string.delete));
    actions.add(() -> {
      update(list -> list.remove(index));
      refresh();
    });
    new MaterialAlertDialogBuilder(context)
        .setTitle(PlusAvoidPlaceDialog.describe(context, place))
        .setItems(items.toArray(new CharSequence[0]), (dialog, which) -> actions.get(which).run())
        .setNegativeButton(R.string.cancel, null)
        .show();
  }

  private void showOnMap(@NonNull PlusAvoidPlaces.Place place)
  {
    final Intent intent = new Intent(Intent.ACTION_VIEW,
                                     Uri.parse(String.format(Locale.US, "geo:%.6f,%.6f?z=16", place.lat, place.lon)));
    intent.setPackage(requireContext().getPackageName());
    intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP | Intent.FLAG_ACTIVITY_SINGLE_TOP);
    startActivity(intent);
  }
}
