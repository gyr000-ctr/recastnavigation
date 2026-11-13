# RecastParallel - optional, explicitly opt-in parallel tile builder.
#
# This target is only created when RECASTNAVIGATION_PARALLEL is ON. The core
# Recast and Detour libraries are untouched and remain C++98 / single threaded.
# RecastParallel is a separate C++17 static library that depends on the public
# Recast and Detour APIs and on the platform threading support.

find_package(Threads REQUIRED)

add_library(RecastParallel STATIC
    Source/RecastParallelRunner.cpp
    Source/RecastParallelTileBuild.cpp
)
add_library(RecastNavigation::RecastParallel ALIAS RecastParallel)

target_compile_features(RecastParallel PUBLIC cxx_std_17)
set_target_properties(RecastParallel PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# The top-level build disables RTTI/exceptions for the C++98 core libraries.
# RecastParallel is a modern C++17 component that uses std::thread/std::function
# and defensively catches exceptions from caller-supplied tasks, so re-enable
# exception handling for this target only. RTTI stays disabled to stay ABI
# consistent with the no-RTTI Recast/Detour libraries (a derived rcContext
# must not reference typeinfo that those libraries do not emit). The core
# libraries themselves are unchanged.
if(MSVC)
    target_compile_options(RecastParallel PRIVATE /EHsc)
else()
    target_compile_options(RecastParallel PRIVATE -fexceptions)
endif()

target_include_directories(RecastParallel PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation>"
)

target_link_libraries(RecastParallel
    PUBLIC Recast Detour
    PRIVATE Threads::Threads
)

install(TARGETS RecastParallel
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
