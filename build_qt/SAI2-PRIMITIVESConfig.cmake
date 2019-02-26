# compute paths
get_filename_component(SAI2-PRIMITIVES_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(SAI2-PRIMITIVES_INCLUDE_DIRS "/home/adrian/Documents/research/sai2/core/mod-sai2-primitives/src")
set(SAI2-PRIMITIVES_DEFINITIONS "")
 
# library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET sai2-primitives AND NOT SAI2-PRIMITIVES_BINARY_DIR)
  include("${SAI2-PRIMITIVES_CMAKE_DIR}/SAI2-PRIMITIVESTargets.cmake")
endif()

# IMPORTED target created by SAI2-PRIMITIVESTargets.cmake
set(SAI2-PRIMITIVES_LIBRARIES "sai2-primitives")
