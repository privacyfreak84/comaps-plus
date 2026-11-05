project(plus_overlay_tool)

# Comaps++ cell-overlay (MLD/CRP) prototype. Desktop only, not linked into the app.

set(SRC
  overlay.cpp
  overlay.hpp
  plus_overlay_tool.cpp
)

omim_add_executable(${PROJECT_NAME} ${SRC})

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    routing
    routing_common
    storage
    traffic
    indexer
    platform
    geometry
    coding
    base
    gflags::gflags
)
