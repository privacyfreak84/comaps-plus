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
