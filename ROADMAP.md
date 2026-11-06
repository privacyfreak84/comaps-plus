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

Status: almost done.

Done:

- Long car routes in seconds instead of minutes, with the same routes CoMaps would choose. Measurements and checks are in the [README](README.md#fast-car-routes).
- Route data prepared on the phone in the background, after downloads, map updates and app start, with progress and Cancel.
- Routes through maps that aren't prepared yet: estimated, with a message, that map prepared first, the route recalculated by itself afterwards.
- Avoid settings, with data prepared only for the maps your routes go through, and a choice to keep data for one mix or every mix.
- Rerouting, routes with stops, start direction, app updates and map updates, damaged files.
- Once every map on a route is prepared, the route doesn't use CoMaps' old car search.

Left:

- Time-dependent road access. Some roads are open only at certain hours or on certain days. The prepared data gives these a fixed one-hour penalty, while CoMaps checks the time when routing. Plan: keep such roads out of the prepared cells and check them when the route is calculated. None exist in the tested maps, so this needs a region that has them.
- Memory while preparing large maps: close to 1 GB at the peak for Uttar Pradesh. Reducing it needs a smaller representation of the cell layout during preparation.
- Testing outside India. Everything so far was tested with Uttar Pradesh, Madhya Pradesh and Maharashtra.

Decided to leave as is:

- Near state borders, 3 of 24 border-area test routes were 30 s to 2 minutes off, because Comaps++ switches between maps only where a road crosses the border. Fixing it changes the file format and makes every user prepare all maps again.
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

Status: not started.

The app still uses CoMaps' app ID (`app.comaps`), name and icon. Test builds install as `app.comaps.test`.

Still needed:

- The new app name and an icon, both still to be chosen by the project owner.
- A new app ID, so Comaps++ installs next to CoMaps and can be updated on its own.
- The name changed throughout the app and its texts.
- A signing key of its own, kept as a GitHub secret.
- A release workflow that builds signed APKs and publishes them as GitHub releases, so they work with Obtainium and with the update checker (item 3).

## 7. Several route options, as in Google Maps

Status: not started.

Show two or three different routes between the same places, each with its time and distance, and let the user pick one before starting navigation.

Still needed:

- A method to find alternatives that are actually different and still reasonable, not the best route with one small detour. Which method fits CoMaps' road graph and the prepared cells is not decided yet. Since a long route now takes a few seconds, a few extra searches per request are affordable.
- Showing the alternatives on the map and in the route panel, and switching between them.
- Keeping rerouting on the route the user picked.
