# Find our globally shared libraries:
FIND_PACKAGE(Armadillo REQUIRED)
FIND_PACKAGE(LibGFortran REQUIRED)
FIND_PACKAGE(OpenBLAS)
FIND_PACKAGE(Protobuf REQUIRED)
FIND_PACKAGE(CATCH REQUIRED)
FIND_PACKAGE(YAML-CPP REQUIRED)
FIND_PACKAGE(muParserX REQUIRED)
FIND_PACKAGE(CPPFormat REQUIRED)

# Resolve problems when OpenBLAS isn't found:
IF(OpenBLAS_FOUND)
	SET(BLAS_LIBRARIES    ${OpenBLAS_LIBRARIES})
	SET(BLAS_INCLUDE_DIRS ${OpenBLAS_INCLUDE_DIRS})
ELSE()
	FIND_PACKAGE(BLAS REQUIRED)
	MESSAGE(WARNING "OpenBLAS was not found. Using BLAS instead.")
ENDIF()

# Some definitions for armadillo
ADD_DEFINITIONS(-DARMA_DONT_USE_WRAPPER -DARMA_32BIT_WORD)

# Set include directories and libraries:
INCLUDE_DIRECTORIES(SYSTEM ${BLAS_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(SYSTEM ${PROTOBUF_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(SYSTEM ${CATCH_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(SYSTEM ${YAML-CPP_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(SYSTEM ${muParserX_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(SYSTEM ${CPPFormat_INCLUDE_DIRS})


SET(NUCLEAR_ADDITIONAL_SHARED_LIBRARIES
    ${BLAS_LIBRARIES}
    ${LIBGFORTRAN_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${YAML-CPP_LIBRARIES}
    ${muParserX_LIBRARIES}
    ${CPPFormat_LIBRARIES}
)
