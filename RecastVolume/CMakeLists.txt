# Optional 3D navigation volumes. Fully additive: depends only on the existing
# Recast and Detour targets and does not modify any of their sources, so the
# default 2D build output is unchanged.
add_library(RecastVolume)
add_library(RecastNavigation::RecastVolume ALIAS RecastVolume)

set_target_properties(RecastVolume PROPERTIES
    DEBUG_POSTFIX -d
    SOVERSION ${SOVERSION}
    VERSION ${LIB_VERSION}
    CXX_STANDARD 98
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_include_directories(RecastVolume PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation>"
)

target_sources(RecastVolume PRIVATE
    Source/RecastVolume.cpp
    Source/RecastVolumeFallback.cpp
)

target_link_libraries(RecastVolume PUBLIC Recast Detour)

install(TARGETS RecastVolume
    EXPORT recastnavigation-targets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT library
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} ${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation
)

install(DIRECTORY Include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation
    FILES_MATCHING PATTERN "*.h"
)
