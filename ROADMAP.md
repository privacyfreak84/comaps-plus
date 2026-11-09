# Comaps++ roadmap

What's planned, what exists so far, and what each item still needs. The order is the list from the [README](README.md#roadmap), not a schedule. Items 5 and 6 come before item 3, because an update checker needs releases to check.

Status words: done, almost done, started (some groundwork exists), not started.

## 1. Avoid specific roads and places

Status: started.

CoMaps lets you avoid types of road: tolls, motorways, ferries, dirt roads. The goal is to also avoid one particular road, bridge, junction or area that you pick yourself.

What exists:

- The fast routes ([README](README.md#fast-car-routes)) store travel times per map cell. Avoiding a place only changes the cells that contain it, so only those cells and the larger cells above them need new times, not the whole map.
- A desktop test of exactly that, in `plus_overlay_tool` on Uttar Pradesh: recalculating 57 cells took 0.5 to 0.75 s, and 58 test routes then matched an exact search.

Still needed:

- Picking what to avoid in the app (for example tapping a road, or marking an area) and keeping that list.
- Recalculating the affected cells on the phone when the list changes, alongside the avoid-settings data that already exists.
- A clear message when an avoided place leaves no route at all.
- Deciding whether an avoided place is never used or only made much slower.

## 2. Faster navigation

Status: done.

Done:

- Long car routes in seconds instead of minutes, with the same routes CoMaps would choose. Measurements and checks are in the [README](README.md#fast-car-routes).
- Route data prepared on the phone in the background, after downloads, map updates and app start, with progress and Cancel.
- Routes through maps that aren't prepared yet: estimated, with a message, that map prepared first, the route recalculated by itself afterwards.
- Avoid settings, with data prepared only for the maps your routes go through, and a choice to keep data for one mix or every mix.
- Rerouting, routes with stops, start direction, app updates and map updates, damaged files.
- Once every map on a route is prepared, the route doesn't use CoMaps' old car search.
- Roads open only at certain times (access:conditional) are checked for the time the route gets there, not frozen when the map is prepared.
- Preparation memory down by about 17% (Uttar Pradesh 960 MB to 800 MB at the peak).
- Tested outside India: Berlin with Brandenburg, and Tyrol, Salzburg and Vorarlberg.

Decided to leave as is:

- Near map borders, Comaps++ switches between maps only where a road crosses the border, while CoMaps can switch at any junction of a road present in both maps. Matching CoMaps was built and measured: long routes became about twice as slow, and 2 of 40 border test routes improved. It was removed again.
- A cycling overlay. Cycling routes are slow only for multi-day trips.

## 3. Update checker and downloader in settings

Status: not started.

A screen in Settings that checks this repository for a newer release, downloads the APK and hands it to Android's installer. Until item 6 is done there are no releases and no stable app ID to update.

Still needed:

- Releases on GitHub with signed APKs (item 6).
- A check against the GitHub releases of this repository, by hand from Settings and optionally now and then in the background.
- Download with progress, then Android's install prompt. This needs the permission to install packages, which Android shows to the user.
- Showing the release notes before installing.

## 4. Full OpenStreetMap editing

Status: not started. This one grows over time.

CoMaps has a basic editor for places (name, opening hours, address and some other details). The goal is to edit more of OpenStreetMap from the app, step by step. Which parts come first is not decided yet.

## 5. Remove references to CoMaps, keep it in the credits

Status: started.

Done:

- The README no longer links to CoMaps' website, donation pages, app store pages or chat, and mentions CoMaps only as the project this fork is built on and in the credits.

Still needed:

- The same inside the app. References to CoMaps' website and donations show up in at least `HelpFragment.java`, `MwmActivity.java`, `Utils.java`, `NavigationSettingsFragment.java`, `strings.xml` and `donottranslate.xml`, plus the support address `apk@comaps.app` in the build file.
- An About screen with credits to CoMaps, Organic Maps, Maps.ME and OpenStreetMap contributors.
- A note to the CoMaps team about the app downloading maps from their servers, or a decision about where maps should come from.

## 6. New package name, app name and icon

Status: started.

The app still uses CoMaps' app ID (`app.comaps`), name and icon. Test builds install as `app.comaps.test`.

Done:

- A release workflow ([plus-release.yml](.github/workflows/plus-release.yml)) that builds a signed arm64 APK and publishes it as a GitHub release, which Obtainium and the update checker (item 3) can follow. It refuses to run until the app ID is changed.

Still needed:

- The signing key, created by the project owner and stored as GitHub secrets.
- The new app name and an icon, both still to be chosen by the project owner.
- A new app ID, so Comaps++ installs next to CoMaps and can be updated on its own. Only the app ID changes; the code's package names (`app.organicmaps`) stay, so upstream merges keep working.
- The name changed throughout the app and its texts.

## 7. Several route options, as in Google Maps

Status: not started.

Show two or three different routes between the same places, each with its time and distance, and let the user pick one before starting navigation.

Still needed:

- A method to find alternatives that are actually different and still reasonable, not the best route with one small detour. Which method fits CoMaps' road graph and the prepared cells is not decided yet. Since a long route now takes a few seconds, a few extra searches per request are affordable.
- Showing the alternatives on the map and in the route panel, and switching between them.
- Keeping rerouting on the route the user picked.

## 8. Less storage space and memory used by maps

Status: done, for the parts Comaps++ controls.

Sizes on the phone, maps version 261101 (all in decimal MB; the earlier toll figures for Maharashtra and Madhya Pradesh are from maps 261004):

| Map | Map file | Route data before / now | Avoid tolls before / now |
|---|---|---|---|
| Uttar Pradesh | 200.9 MB | 114.1 / 91.6 MB | 71.5 / 11.1 MB |
| Maharashtra | 165.7 MB | 59.8 / 46.2 MB | 33.2 / 5.9 MB |
| Madhya Pradesh | 134.2 MB | 37.8 / 29.0 MB | 19.9 / 2.8 MB |
| Berlin | 111.7 MB | 8.6 / 6.7 MB | 3.5 / 0.7 MB (motorways) |

Done:

- Route data about 20% smaller, with the same routes: each vertex's cell is stored as runs of equal cells (Uttar Pradesh 17 MB to 3 MB), and which vertices are cell boundaries is read from the boundary lists already in the file instead of a byte per vertex (8.5 MB).
- Avoid-setting data about 85% smaller: it stores only the cells whose stored times differ from the normal route data. Avoiding dirt roads changes more cells (38.6 MB for Uttar Pradesh).
- Preparation memory: Uttar Pradesh route data 960 MB to about 800 MB at the peak (with roadmap item 2), avoid-setting data 830 MB to 770 MB.
- Memory while routing, measured: Lucknow to Pune peaks at about 650 MB with Comaps++, against about 2.2 GB for CoMaps' own search.
- The new file layout makes the app prepare every map once more after updating.

Not done, and why:

- The map files themselves come from CoMaps' servers in CoMaps' format. Making them smaller would mean generating and hosting maps, which this project doesn't do.
- The stored travel times are now about 70% of the route data, as 4-byte numbers. Storing them in 2 bytes would save roughly another third of the route data, but rounds the stored times, so routes could differ from CoMaps' by a few seconds. Optional, if the space matters more than exact matching.
- Most of the memory while routing is CoMaps' road graph, which CoMaps loads whole for each map a route touches. Loading it in parts would mean reworking CoMaps' own routing code.

## 9. The same changes in the iOS and desktop apps

Status: not started.

Everything in items 1 to 8 is built and tested in the Android app only. The router itself is shared C++ code, so the iOS and desktop apps would already use fast routes if the route data were on the device, but nothing in those apps prepares it.

Needs:

- Preparing route data on the device: after downloads, map updates and app start, as on Android. On iOS that means a background processing task, which iOS runs when it chooses (usually while charging); on desktop a background thread.
- The same status messages, Settings entries and avoid-setting choices as on Android, in both apps.
- The other items (avoided places, route options, update checker, rebranding) in both apps as they are done.
- Testing both apps, which so far haven't been built with Comaps++'s changes.

## 10. Route data always ready, never CoMaps' old search

Status: not started.

The route data is prepared on the phone, not downloaded (the project doesn't host files). The goal is that every downloaded map always has it, so no car route ever falls back to CoMaps' old search.

Cases found so far in the code where a map can stay without route data, or a route can fall back:

- The preparation job waits while the battery is low or storage is low, and Android can postpone it further.
- A map whose preparation Android stopped 3 times is not tried again.
- A preparation that fails is counted and not retried in the same run.
- While a map isn't prepared, routes through it are estimated (item 2), and in some cases fall back to the old search: when the route may pass through such a map in a way the estimate can't cover, when no way into the finish or out of the start map is found, or when unpacking a route fails.

Needs:

- Every one of these cases removed or handled, so the old search is never used for a car route. Where data really can't exist yet (the first minutes after a download), the estimate is used, never the old search.
- A test for each case, including Android killing the job, low battery, low storage and app updates in the middle of preparation.
- Lower preparation memory where it makes Android stop the job (about 800 MB at the peak for Uttar Pradesh).

## 11. Simpler avoid settings that always have their data

Status: not started.

Today the data for avoid settings (tolls, motorways, ferries, dirt roads) is prepared only for the maps a route has gone through with that mix, and only after that route asked for it. In use this often leaves a map without the data, and the route then falls back to CoMaps' old search.

Cases found so far in the code:

- A new mix has no data anywhere until a route asks for it, so the first routes with that mix are not fast.
- Changing the mix during preparation stops it; the run for the new mix starts over.
- With "keep data for the current mix only", switching back to an earlier mix starts from nothing again.
- The cases in item 10 apply to this data too.

Needs:

- One rule that can't miss: for example, preparing the current mix for every downloaded map as soon as it is chosen. Avoid data is now small (11 MB for Uttar Pradesh with tolls avoided, item 8), so this is affordable.
- Fewer choices in Settings, if the rule makes the current-mix-or-every-mix choice unnecessary.
- No fallback to the old search for any mix (item 10).
