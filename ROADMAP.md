# Comaps++ roadmap

What's planned, what exists so far, and what each item still needs. The order is the list from the [README](README.md#roadmap), not a schedule. Items 5 and 6 come before item 3, because an update checker needs releases to check.

Status words: done, almost done, started (some groundwork exists), not started.

## 1. Avoid specific roads and places

Status: done.

CoMaps lets you avoid types of road: tolls, motorways, ferries, dirt roads. Comaps++ also keeps a car route away from a road or an area you pick on that route, with nothing prepared for it.

Done:

- Avoid is offered on the place page only while a car route is shown, and depends on what you tap. A town or village (city, town or village in OSM) the route passes through: the route stays out of it, within a radius set by its population as CoMaps' own routing sizes towns (about 13 km for Lucknow, 1 to 2 km for a town, under 1 km for a village). Any other point within 40 m of the route, tapped or long-pressed: the route stays off its own road there (the whole OSM way; saved at the middle of the route segment, so a crossing road isn't picked by mistake). Anything off the route, and the route's start, destination and stops, get no Avoid. After confirming, the route is calculated again.
- Avoided places belong to the route they were picked on. They live in the core's memory while the route exists: kept through recalculations, reroutes, added stops and navigation, and cleared when the route is closed (`RoutingManager::CloseRouting` with its points removed). There is no list in Settings and no permanent storage; the old list from earlier builds is deleted the first time the app reads avoided places.
- They're drawn in red (an area's edge, a road's line) only while their route is on the map. Tapping a place drawn in red offers Don't avoid, which drops it from the route and recalculates; this works even when the avoided places left no route.
- They're saved only with their route: in the file the app uses to restore a route after a restart, and in a route saved as a track (KML property `comaps_plus_avoid`), where they're drawn while that track is selected. A road is saved as its point and name; each route looks it up again, so it survives map updates.
- Each car route looks up the road segments a place covers in every loaded map (3 to 13 ms for 3 to 8 places on the test VM) and leaves them out of CoMaps' road graph. The fast search goes through the cells holding those segments road by road, so it stays exact; CoMaps' own search leaves them out as well.
- Places are never used, not only made slower. A place doesn't apply to a route that starts, ends or stops in it (within 30 m for a road), so the route can still get there.
- Tested on the test VM what Avoid offers for 3 routes: the route's road at its middle (for example "Kanpur-Lucknow Expressway"), nothing 1 km off the route, a 3 km town 1.5 km off the route, nothing for a town at the start. Avoiding the three roads it offered (two expressways and NH135A) made those routes 12 to 35 minutes longer, on the same roads as CoMaps' exact search, with no segment of the avoided roads used.
- 16 random routes across Uttar Pradesh and 8 into Madhya Pradesh avoiding dirt roads, against CoMaps' exact search: the same roads on all 16 in Uttar Pradesh. Into Madhya Pradesh, 5 routes were found by both: 3 on the same roads, one equally fast, one 63 s slower out of 9 hours, which is the border difference again (it is the route without avoid settings, and the per-entry rule below doesn't change it).
- If no route is left, the "no route" message adds that the avoided places may be the reason, and how to undo one.
- Tested: 8 routes across Uttar Pradesh with 8 places on them (4 roads, 4 areas of 250 m to 2 km): every route changed, none uses a segment of a place, and all take the same roads as CoMaps' exact search with the same places (one 10 s apart in arrival time, as without places), in 1.2 to 1.4 s against 3.9 to 53 s for that search. Three routes from Uttar Pradesh into Madhya Pradesh with 3 places, also avoiding dirt roads: no segment of a place used; two within 5 s of CoMaps' search, as without places, and one 30 s slower out of 10.8 hours, its avoided area being next to the state border (see the border note in the [README](README.md#known-limits)).

Not done:

- The earlier desktop test that recalculated the times of the cells around a place isn't used: leaving those cells to the road-by-road search needs no preparation and costs little for a few places.

## 2. Faster navigation

Status: done.

Done:

- Long car routes in seconds instead of minutes, with the same routes CoMaps would choose. Measurements and checks are in the [README](README.md#fast-car-routes).
- Route data prepared on the phone in the background, after downloads, map updates and app start, with progress and Cancel.
- Routes through maps that aren't prepared yet: estimated, with a message, that map prepared first, the route recalculated by itself afterwards.
- Avoid settings, exact with or without their extra data (roadmap item 11).
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
| Uttar Pradesh | 200.9 MB | 114.1 / 63.8 MB | 71.5 / 7.7 MB |
| Maharashtra | 165.7 MB | 59.8 / 33.9 MB | 33.2 / 3.6 MB |
| Madhya Pradesh | 134.2 MB | 37.8 / 21.5 MB | 19.9 / 1.9 MB |
| Berlin | 111.7 MB | 8.6 / 6.0 MB | 3.5 / 0.6 MB (motorways) |

Done:

- Route data about 20% smaller, with the same routes: each vertex's cell is stored as runs of equal cells (Uttar Pradesh 17 MB to 3 MB), and which vertices are cell boundaries is read from the boundary lists already in the file instead of a byte per vertex (8.5 MB).
- Avoid-setting data about 85% smaller: it stores only the cells whose stored times differ from the normal route data. Avoiding dirt roads changes more cells (24.1 MB for Uttar Pradesh).
- Stored travel times in 2 bytes instead of 4: each cell stores its times as steps of its own scale (its longest time divided by 65,535), which rounds a stored time by at most half a second in the tested maps. Penalties for time-restricted roads get one more byte per value, but only in cells that have such roads. Indian route data is 26 to 30% smaller, avoid data 31 to 39%; German and Austrian route data only 9 to 17%, because most of their cells have time-restricted roads. Arrival times: no change on the 10 long Indian routes; changes of at most 1.1 s on 36 of 386 German routes and two of about 7 s (one faster, one slower); at most 0.1 s on 8 of 390 Austrian routes. Search times didn't change. Damaged scales or penalty offsets are rejected when a file is opened.
- Preparation memory: Uttar Pradesh route data 960 MB to about 800 MB at the peak (with roadmap item 2), avoid-setting data 830 MB to 770 MB.
- Memory while routing, measured: Lucknow to Pune peaks at about 650 MB with Comaps++, against about 2.2 GB for CoMaps' own search.
- The new file layout makes the app prepare every map once more after updating.

Not done, and why:

- The map files themselves come from CoMaps' servers in CoMaps' format. Making them smaller would mean generating and hosting maps, which this project doesn't do.
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

Status: done, for everything the app can control.

The route data is prepared on the phone, not downloaded (the project doesn't host files). Every case found in the code where a map could stay without route data, or a car route could fall back to CoMaps' old search, is handled:

| Case | Before | Now |
|---|---|---|
| Battery low, storage low | The preparation job waited, possibly for days | Doesn't wait |
| Android stops a preparation (memory, time limits) | Retried with one thread, given up after 3 times | Retried with one thread after the other maps, never given up; from the third time with a pause of 15 minutes, doubling up to 6 hours |
| A preparation fails | Not retried until the next app start or map change | Retried after 1, 4, 16 ... minutes, up to 6 hours, while new runs aren't held up |
| Stopped halfway | Started over | The cell layout (the slowest step) is kept and reused |
| Phone restarted, app not opened | Waited for the app to be opened | The job starts the app's core itself and continues |
| No data for the current avoid settings | The map counted as unprepared: estimates, or CoMaps' search | Exact routes on the existing data (item 11) |
| A data file lists a border crossing CoMaps doesn't know | Whole route on CoMaps' old search | That crossing is skipped (CoMaps can't use it either) |
| Unpacking a stretch of the route fails | Whole route on CoMaps' old search | That stretch is searched on the roads |
| Route ends in an unprepared map and the handover point doesn't connect | Whole route on CoMaps' old search | The stretch from the border crossing is searched instead |

Tested: with Madhya Pradesh unprepared, all 10 long Indian test routes stayed on the fast search (estimated through Madhya Pradesh), none on CoMaps' old search.

What's left, and why:

- A map that isn't prepared yet (just downloaded or updated) is crossed on estimates, and the stretch inside it uses CoMaps' search, until its data is ready (a minute or two per map). This is the design chosen in item 2; preparing on the phone can't be instant.
- One internal safety net remains: if a stretch can't be unpacked even on the roads, the route goes to CoMaps' search. It means a bug, and it didn't happen in any test.
- A phone that can never spare the memory for a large map (about 800 MB for Uttar Pradesh) keeps retrying without finishing. Lower preparation memory would be the next step.
- Cancel in the notification stops preparation until the app starts again or a route needs a map.

## 11. Simpler avoid settings that always have their data

Status: done.

Done:

- Avoid settings need no data of their own any more. Each map's route data records, per cell, which road types (tolls, motorways, ferries, dirt roads) the cell's stored travel times rely on, leaving out each stored path's own start and end. A route avoiding one of those types goes through such a cell road by road; the search also never stands on an entry or exit of a cell that is itself on an avoided road. The files earlier versions prepared per avoid mix are deleted.
- Recording what the stored times rely on, instead of which road types a cell contains, is what makes this fast. In Uttar Pradesh, of the 49,823 smallest cells, 27,333 contain dirt roads but only 4,032 rely on them; tolls 180 against 63, motorways 343 against 58. The road types of each cell's entries and exits add 1.4 MB to Uttar Pradesh's route data (65.4 MB, maps 261105).
- Tested on 8 routes across Uttar Pradesh against CoMaps' exact search with the same settings, avoiding tolls, motorways and dirt roads (24 routes): the same roads on all 24 and the same arrival times, except one route that is 10 s apart with each setting, as it is without avoid settings. Times on the test VM: 1.0 to 1.2 s without avoid settings, 1.1 to 1.6 s avoiding tolls, 1.1 to 2.0 s avoiding motorways, 1.2 to 3.5 s avoiding dirt roads. Before (no avoid file yet), avoiding dirt roads took 8.5 to 27 s on the same routes.
- Route data in the previous layout keeps working; the app prepares it again in the background, and avoid settings are slower with it until then. Layout 8, which was only on main for a few hours, is read like layout 7.

Left as is:

- Routes across two maps that avoid dirt roads take 2.3 to 7.6 s (routes of 9 to 14 hours from Uttar Pradesh into Madhya Pradesh, against 2.6 to 3.2 s without), because the largest cells nearly all rely on some dirt road and the search crosses them on smaller cells. Layout 9 records the road types per cell entry (its stored paths to every exit) and per exit as well, packed into the byte that already held the entry's or exit's own road types, so it costs no space. The search then keeps using a cell from entries whose own paths don't rely on avoided roads. That cut the vertices searched on these routes by 11 to 15%, not more: in Madhya Pradesh 71% of the entries of the largest cells have some path on a dirt road (44% one level down, 4% in the smallest cells).
