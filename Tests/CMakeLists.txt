add_executable(Tests)

set_target_properties(Tests PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF  # Disable compiler-specific extensions
)

# Remove warnings-as-errors
if(MSVC)
	target_compile_options(Tests PRIVATE /W3 /WX-)
else()
	target_compile_options(Tests PRIVATE -Wno-error)
endif()

target_include_directories(Tests PRIVATE ./Contrib)

target_sources(Tests PRIVATE
	Contrib/catch2/catch_amalgamated.cpp
	Detour/Tests_Detour.cpp
	DetourCrowd/Tests_DetourPathCorridor.cpp
	Recast/Tests_Alloc.cpp
	Recast/Tests_Recast.cpp
	Recast/Tests_RecastFilter.cpp
	Recast/Tests_RecastRasterization.cpp
)

target_link_libraries(Tests PRIVATE Recast Detour DetourCrowd)

# Optional parallel-tile tests. Only compiled when the opt-in RecastParallel
# component is enabled; default builds are completely unaffected.
if (RECASTNAVIGATION_PARALLEL)
	find_package(Threads REQUIRED)
	target_sources(Tests PRIVATE
		RecastParallel/Tests_ParallelRunner.cpp
		RecastParallel/Tests_ParallelTileBuild.cpp
	)
	target_link_libraries(Tests PRIVATE RecastParallel Threads::Threads)
	if(NOT MSVC)
		# Re-enable exceptions for the cancellation/exception-propagation tests;
		# keep RTTI off to match the no-RTTI core and RecastParallel libraries.
		target_compile_options(Tests PRIVATE -fexceptions)
	endif()
endif()

add_test(NAME Tests COMMAND Tests)
