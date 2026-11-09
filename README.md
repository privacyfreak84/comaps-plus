# Comaps++

Comaps++ is an offline maps app for Android, built on [OpenStreetMap](https://www.openstreetmap.org) data. It is a fork of [CoMaps](https://codeberg.org/comaps/comaps) and keeps everything CoMaps does. The main change so far: long car routes are calculated in a few seconds instead of minutes.

This is a test-stage project. There is no release yet, and the app still installs under the CoMaps name and icon. See [Get a test build](#get-a-test-build) and the [roadmap](#roadmap).

## Fast car routes

On a phone, CoMaps could take several minutes to calculate a car route from Lucknow to Pune. Comaps++ prepares extra route data on the phone for each downloaded map, once, and then answers the same route in about two seconds.

The extra data is a cell overlay (the method behind OSRM's MLD and Microsoft's CRP). Each map's road network is split into nested cells, and the travel time between the entry and exit points of every cell is stored. A route then only searches inside the cells at its two ends and jumps across everything in between. The overlay is built from CoMaps' own road graph and travel-time model, including turn costs, U-turns, turn restrictions and access rules, so the routes it returns are CoMaps' routes. Roads that are open only at certain times are left out of the stored times and checked for the moment the route gets there, as CoMaps does. Turn-by-turn directions, street names and arrival times still come from CoMaps' normal route builder.

Measured on a 2-core Linux VM with India maps (version 261004), against CoMaps' router in its exact search mode on the same machine:

| Route | CoMaps | Comaps++ |
|---|---|---|
| Lucknow to Pune (3 state maps) | 124 s | 2.3 s |
| 15 random routes over 300 km (median) | 28.2 s | 1.96 s |
| 6 routes of 1,500 to 2,000 km (median) | 95.2 s | 2.58 s |
| 30 random routes of 0.2 to 50 km (median) | 0.82 s | 0.70 s |

Short trips were already quick, so the gain is in long ones. In maps with time-restricted roads (all the tested German and Austrian maps; none of the Indian ones) the search runs in one direction so it knows the arrival time at each road: the longest Austrian test routes (the farthest pairs across Tyrol, Salzburg and Vorarlberg) took 0.5 s against 5.7 s for CoMaps, and short routes 0.1 to 0.5 s (medians).

### What happens on the phone

- After you download or update a map, a background job prepares its route data. A notification shows progress and has a Cancel button. The job also runs when the app starts, to catch anything missed, and after a reboot even if the app isn't opened.
- The job doesn't wait for a charged battery or free storage, and doesn't give up on a map. If Android stops a preparation (for example for memory), it is tried again with one thread after the other maps, and from the third time with a growing pause (15 minutes up to 6 hours). A preparation that fails is tried again after 1, 4, 16 minutes and so on, up to 6 hours. A stopped preparation keeps its cell layout, so the next attempt skips the slowest step.
- Until a map is prepared, car routes through it still work. Comaps++ estimates the part of the route inside that map from CoMaps' border-to-border times and shows a message that the route may be a few minutes off. That map is prepared first, and the route is recalculated by itself when it's ready.
- Avoid settings (tolls, motorways, ferries, dirt roads, in any mix) work with fast routes, and their routes are exact from the moment you change the settings. Each map's route data records which of these road types every cell contains; for a map without data for your mix, the route goes through the cells with avoided roads road by road. That data, one smaller file per map, is prepared for every downloaded map as soon as you change the mix, the maps your routes use first, and the files of the previous mix are deleted. Until then routes avoiding tolls or motorways take about as long as with the data (0.9 to 3.9 s for the 10 long Indian routes, against 1.0 to 2.0 s), and routes avoiding dirt roads, which are in about half of all cells in India, are much slower (2.5 to 46 s on the test VM).
- Rerouting after a missed turn goes straight to the fast search. In 110 test reroutes the new route was as quick as a fresh route from the same spot (one was 5 s slower), and took 0.3 to 3.2 seconds.
- Routes with stops use the fast search for every leg.
- App updates don't throw the data away. Map updates reuse the previous cell layout, which skips the slowest step: Uttar Pradesh takes 49 s instead of 153 s on the test VM.
- If a data file turns out to be damaged, the app deletes it and prepares it again; until then the map is treated as not prepared (or, for avoid-setting data, routes there are exact but slower).
- Car routes don't hand over to CoMaps' old search. Only the stretch inside a map that isn't prepared yet uses CoMaps' search ([roadmap item 10](ROADMAP.md#10-route-data-always-ready-never-comaps-old-search)).
- Settings, Navigation, Fast routes shows how many downloaded maps are ready (with data for your avoid settings) and how much space the data uses. A test switch there, "Show how each car route was calculated", adds a short message after each route.

Walking and cycling routes use CoMaps' router unchanged. Walking routes take a few seconds even over 75 km. Cycling is slow only for multi-day trips (37 s for 600 km on the test VM), so a cycling overlay is not planned for now.

### What it costs

Storage, per map:

| Map | Map file | Route data | Extra for one avoid mix (tolls) |
|---|---|---|---|
| Uttar Pradesh | 200.9 MB | 63.8 MB | 7.7 MB |
| Maharashtra | 165.7 MB | 33.9 MB | 3.6 MB |
| Madhya Pradesh | 134.2 MB | 21.5 MB | 1.9 MB |
| Berlin | 111.7 MB | 6.0 MB | 0.6 MB (motorways) |

An avoid mix stores only the parts of the network it changes, so its size depends on the settings: avoiding dirt roads, which are common in India, takes 24.1 MB for Uttar Pradesh.

Preparation time and peak memory on the 2-core test VM: Uttar Pradesh about 1.5 minutes and 800 MB, Maharashtra 1 minute and 550 MB, Madhya Pradesh 35 s and 375 MB, Berlin 6 s and about 120 MB. On a test phone, preparation ran in the background without Android closing the app and without the app lagging.

Memory while routing: for Lucknow to Pune the whole router process on the test VM peaked at about 650 MB, against about 2.2 GB for CoMaps' own search, with the same arrival time. Most of the 650 MB is CoMaps' road graph for the three maps, which CoMaps loads whole.

### How the routes were checked

The point of the overlay is speed without different routes, so every change was checked against CoMaps' own router:

- 13 routes of 108 to 1,218 km across Uttar Pradesh, Madhya Pradesh and Maharashtra: identical arrival times.
- 327 generated routes in 12 groups (state borders, bridges and tunnels, dead ends, small islands, one-way streets, ramps, restricted access, roundabouts, random pairs, the longest possible pairs, and 20 hand-picked routes such as the Mumbai to Pune expressway ghats and the Bandra-Worli sea link): the same outcome in all 327 (routed or not), no crash or hang. Where both routed, arrival times matched within a minute except on two routes. On one, Comaps++ found a route more than a minute quicker. On the other, CoMaps' route arrives 64 s sooner but scores worse by CoMaps' own measure, which adds penalties for roads with restricted access.
- 24 routes with avoid settings, 6 routes with 1 to 3 stops, 110 reroutes: same arrival times.
- After the storage changes (roadmap item 8): the 10 long Indian routes, with and without tolls, and 386 German routes gave the same arrival times as before, except two German routes that changed by 4 and 7 seconds.
- Storing travel times in 2 bytes instead of 4 (roadmap item 8) rounds each stored time by at most half a second. The 10 long Indian routes, with and without tolls, didn't change. Of 386 German routes, 348 didn't change, 36 changed by at most 1.1 s, and two by about 7 s (one faster, one slower). Of 390 Austrian routes, 382 didn't change and 8 changed by at most 0.1 s. Of 40 German routes avoiding motorways, 34 didn't change and 6 changed by at most 0.1 s.
- Inside one map, 100 random routes against a plain shortest-path search: no differences.
- Outside India, 776 generated routes in Berlin with Brandenburg (386) and in Tyrol, Salzburg and Vorarlberg (390), with the clock set to Monday 08:30, including 40 routes per region that start on a time-restricted road: the same outcome in all 776, and arrival times within a minute except on 5 routes. The 40 time-restricted routes in Germany were also run at Sunday 03:00 and Friday 17:30. Avoiding motorways, 39 German border routes: all within a minute.

Two findings along the way. On 2 of 20 test routes in Uttar Pradesh, CoMaps' router returns a slower route than the best legal one (17% and 32% slower), because of how it checks turn restrictions that span several roads; Comaps++ returns the best one. And on one German route at 03:00, CoMaps' route crosses a road signed closed from 22:00 to 05:00 and reaches it at about 03:40; Comaps++ takes a legal route, 49 minutes longer. At noon both routers return the same route.

### Known limits

- Tested with India, Germany (Berlin, Brandenburg) and Austria (Tyrol, Salzburg, Vorarlberg) maps.
- Near map borders a few routes differ slightly: in India 1 of 40 border test routes is more than a minute off (78 s). Comaps++ switches between maps only where a road crosses the border; CoMaps can switch at any junction of a road present in both maps. Doing the same was tried: it made long routes about twice as slow and improved 2 of 40 border routes, so it was left out.
- A route that starts on a road closed at that moment starts on it and leaves it at the nearest junction; CoMaps starts from a neighbouring road instead. In tests this changed 1 of 40 such routes (5 minutes shorter).
- Preparing a large map needs about 800 MB of memory at its peak (Uttar Pradesh). A phone that can never spare that much keeps retrying (with pauses) without finishing, and routes through that map stay estimated.
- Each new route-data layout makes the app prepare every map again after updating; routes are estimated until it's done. Until there are releases this happens freely; after that, layout changes will keep old files readable.
- Only the Android app prepares route data. The iOS and desktop apps from CoMaps are still in this repository but haven't been changed or tested.
- The new messages and settings are in English only.

## Everything else is CoMaps

Downloadable offline maps, search by name, address or category, voice navigation for walking, cycling and driving, an outdoor mode with hiking trails and contour lines, bookmarks and tracks (KML, KMZ, GPX), track recording, offline Wikipedia articles, a subway layer, dark mode, a basic OpenStreetMap editor, and Android Auto. Maps are downloaded from CoMaps' servers.

## Get a test build

There are no releases yet. Test APKs come from the Android Check workflow:

1. Open [Actions, Android Check](https://github.com/privacyfreak84/comaps-plus/actions/workflows/android-check.yaml) and pick the latest successful run on `main`. The workflow runs on demand: the repository owner starts it with Run workflow.
2. Download the `android-WebBeta` artifact (64-bit ARM phones). You need to be signed in to GitHub.
3. Unzip and install the APK. It installs as `app.comaps.test`, next to any regular CoMaps.

To build it yourself, follow [docs/INSTALL.md](docs/INSTALL.md).

## Roadmap

1. Avoid specific roads and places, not only types of road.
2. Faster navigation. Done.
3. An update checker and downloader in the app's settings.
4. Full OpenStreetMap editing, added over time.
5. Remove links to CoMaps (website, donations and so on), keeping it in the credits.
6. A new package name, app name and icon throughout the app.
7. Several route options to choose from, as in Google Maps.
8. Less storage space and memory used by maps on the phone. Done for the route data; the map files come from CoMaps.
9. The same changes in the iOS and desktop apps.
10. Route data always ready for every downloaded map, so car routes never fall back to CoMaps' old search. Done.
11. Simpler avoid settings whose data is always there. Done: routes with avoid settings are exact even before their data is ready.

Details, status and what each item needs: [ROADMAP.md](ROADMAP.md).

## For developers

### Branches

- `main`: Comaps++.
- `upstream`: an untouched mirror of CoMaps from Codeberg. Its release tags are mirrored as `upstream/<tag>`.
- The Sync upstream workflow ([.github/workflows/plus-sync-upstream.yml](.github/workflows/plus-sync-upstream.yml), script [.plus/sync-upstream.sh](.plus/sync-upstream.sh)) checks Codeberg every 6 hours. If the new upstream commits merge without conflicts, it merges them into `main` and starts the Android Check build as a compile check. If they conflict, it opens an issue and leaves `main` alone. It also opens an issue when upstream changes routing code (prepared route files may need `kRoutingDataVersion` raised) and one for each new upstream release.

### Releases

The Release workflow ([.github/workflows/plus-release.yml](.github/workflows/plus-release.yml)) builds a signed arm64 APK from `main` and publishes it as a GitHub release tagged `v<date>-<n>`, with the commit list, the signing certificate's SHA-256 and the APK's SHA-256 in the notes. Start it with `gh workflow run plus-release.yml` or from the Actions tab. It needs the secrets `PLUS_KEYSTORE` (base64 of the keystore), `PLUS_KEYSTORE_PASSWORD` and `PLUS_KEY_ALIAS`, and it refuses to run while the app ID is still CoMaps' `app.comaps`.

### Where the routing code is

- [libs/routing/plus_overlay/](libs/routing/plus_overlay/): the overlay builder (`overlay_builder`), file format (`overlay_file`), query (`overlay`, `overlay_router`) and turn-restriction handling (`via_way_automaton.hpp`).
- [libs/routing/index_router.cpp](libs/routing/index_router.cpp): `TryPlusOverlay` hands car routes to the overlay before CoMaps' own search.
- Android: [PlusOverlay](android/sdk/src/main/java/app/organicmaps/sdk/routing/PlusOverlay.java) (JNI), [PlusOverlayWork](android/app/src/main/java/app/organicmaps/background/PlusOverlayWork.java) (background preparation), [PlusRouteStatus](android/app/src/main/java/app/organicmaps/background/PlusRouteStatus.java) and [PlusOverlaySettings](android/app/src/main/java/app/organicmaps/background/PlusOverlaySettings.java).

Files are stored next to each map as `<map>.plusovl`, plus `<map>.avoid<mask>.plusovl` for an avoid mix. A file only counts if its map version, build settings and routing data version match. Raise `kRoutingDataVersion` only when a code change makes old files give wrong routes; it forces every user to prepare all maps again.

### Desktop tools

Built with the desktop targets:

- `plus_overlay_build`: prepares one map's file, as the phone does (`--mwm`, `--maps_path`, `--resources_path`, `--avoid_mask`, `--check_file`).
- `plus_overlay_multi`: routes across several maps and compares with CoMaps' router (`--mode=overlay|router|reroute|gen_stress|runtime`).
- `plus_overlay_tool`: the single-map prototype and its exactness checks.
- [.plus/bench/run.sh](.plus/bench/run.sh): downloads India state maps and times CoMaps' router on [.plus/bench/routes.txt](.plus/bench/routes.txt); [.plus/bench/profile.sh](.plus/bench/profile.sh) records a perf profile.

Environment switches for testing:

| Variable | Effect |
|---|---|
| `COMAPS_PLUS_OVERLAY=0` | Skip the overlay and use CoMaps' router |
| `COMAPS_PLUS_FORCE_JOINTS=1` | Run CoMaps' router in its exact search mode for long routes |
| `COMAPS_PLUS_MAX_CANDIDATES` | Cap the candidates in CoMaps' long-route search (changes routes; for experiments) |
| `COMAPS_PLUS_ROAD_CACHE` | Size of CoMaps' road geometry cache |

### Other changes from CoMaps

- Submodule URLs in `.gitmodules` are absolute, so they resolve from GitHub.
- The AI-usage rules in the pull request template and contribution guide are removed.

Bugs and ideas: [issues](https://github.com/privacyfreak84/comaps-plus/issues).

## Credits and license

Comaps++ is built on [CoMaps](https://codeberg.org/comaps/comaps), which is a fork of [Organic Maps](https://organicmaps.app), originally based on Maps.ME. Map data comes from [OpenStreetMap](https://www.openstreetmap.org) contributors. The cell overlay follows the approach of Delling, Goldberg, Pajor and Werneck's customizable route planning, as used in OSRM's MLD.

Licensed under the Apache License 2.0, like CoMaps. See [LICENSE](LICENSE), [NOTICE](NOTICE) and [data/copyright.html](data/copyright.html).
