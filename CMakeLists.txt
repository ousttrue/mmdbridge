CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(mmdbridge) # .sln

# shared compiler options
SET(CMAKE_CXX_FLAGS "/EHsc /wd4267"
    )
# embeded pdb 
SET(CMAKE_CXX_FLAGS_DEBUG
    /Z7
    )

# path settings
FIND_PACKAGE(PythonLibs REQUIRED)
FIND_PACKAGE(pybind11 REQUIRED)
GET_FILENAME_COMPONENT(MMD_DIR
    ${CMAKE_CURRENT_LIST_DIR}/MikuMikuDance_x64
    ABSOLUTE
    )
#FIND_PACKAGE(ilmbase REQUIRED)
SET(ilmbase_INCLUDE_DIRS 
    ${CMAKE_INSTALL_PREFIX}/include/OpenEXR)
#FIND_PACKAGE(Alembic REQUIRED)
SET(ALEMBIC_INCLUDE_DIRS
    ${CMAKE_INSTALL_PREFIX}/include)
SET(ALEMBIC_LIBRARIES
    ${CMAKE_INSTALL_PREFIX}/lib/Alembic.lib)

# projects
ADD_SUBDIRECTORY(src)

# additional install files
INSTALL(FILES 
    ${CMAKE_INSTALL_PREFIX}/bin/python36.dll
    ${CMAKE_INSTALL_PREFIX}/bin/Alembic.dll
    ${CMAKE_INSTALL_PREFIX}/bin/hdf5.dll
    ${CMAKE_INSTALL_PREFIX}/bin/imath-2_2.dll
    ${CMAKE_INSTALL_PREFIX}/bin/iex-2_2.dll
    ${CMAKE_INSTALL_PREFIX}/bin/half.dll
    ${CMAKE_INSTALL_PREFIX}/bin/zlib1.dll
    ${CMAKE_INSTALL_PREFIX}/bin/szip.dll
    DESTINATION ${MMD_DIR}
    )
FILE(GLOB python_sciprts Release/Win32/*.py
    )
INSTALL(FILES ${python_sciprts}
    DESTINATION ${MMD_DIR}
    )
INSTALL(DIRECTORY Release/Win32/alembic_assign_scripts 
    DESTINATION ${MMD_DIR}
    )

