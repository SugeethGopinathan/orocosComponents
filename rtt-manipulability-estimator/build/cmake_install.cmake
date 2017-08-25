# Install script for directory: /home/sgo/git-repos/orocosComponents/rtt-manipulability-estimator

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt-manipulability-estimator/librtt-manipulability-estimator-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt-manipulability-estimator/librtt-manipulability-estimator-gnulinux.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt-manipulability-estimator/librtt-manipulability-estimator-gnulinux.so"
         RPATH "/home/sgo/citk/systems/cogimon-minimal-nightly/lib:/home/sgo/citk/systems/cogimon-minimal-nightly/lib/orocos/gnulinux:/home/sgo/citk/systems/cogimon-minimal-nightly/lib/orocos/gnulinux/plugins:/home/sgo/citk/systems/cogimon-minimal-nightly/lib/orocos/gnulinux/types:/usr/local/lib/orocos/gnulinux/rtt-manipulability-estimator:/usr/local/lib/orocos/gnulinux/rtt-manipulability-estimator/types:/usr/local/lib/orocos/gnulinux/rtt-manipulability-estimator/plugins:/usr/local/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt-manipulability-estimator" TYPE SHARED_LIBRARY FILES "/home/sgo/git-repos/orocosComponents/rtt-manipulability-estimator/build/orocos/gnulinux/rtt-manipulability-estimator/librtt-manipulability-estimator-gnulinux.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt-manipulability-estimator/librtt-manipulability-estimator-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt-manipulability-estimator/librtt-manipulability-estimator-gnulinux.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt-manipulability-estimator/librtt-manipulability-estimator-gnulinux.so"
         OLD_RPATH "/home/sgo/citk/systems/cogimon-minimal-nightly/share/rst-rt-0.1/../../lib:/home/sgo/citk/systems/cogimon-minimal-nightly/share/rtt-core-extensions-0.1/../../lib:/home/sgo/citk/systems/cogimon-minimal-nightly/lib:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/home/sgo/citk/systems/cogimon-minimal-nightly/lib:/home/sgo/citk/systems/cogimon-minimal-nightly/lib/orocos/gnulinux:/home/sgo/citk/systems/cogimon-minimal-nightly/lib/orocos/gnulinux/plugins:/home/sgo/citk/systems/cogimon-minimal-nightly/lib/orocos/gnulinux/types:/usr/local/lib/orocos/gnulinux/rtt-manipulability-estimator:/usr/local/lib/orocos/gnulinux/rtt-manipulability-estimator/types:/usr/local/lib/orocos/gnulinux/rtt-manipulability-estimator/plugins:/usr/local/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt-manipulability-estimator/librtt-manipulability-estimator-gnulinux.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/sgo/git-repos/orocosComponents/rtt-manipulability-estimator/build/rtt-manipulability-estimator-gnulinux.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(MAKE_DIRECTORY $ENV{DESTDIR}/usr/local/lib/orocos/gnulinux/rtt-manipulability-estimator)
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/sgo/git-repos/orocosComponents/rtt-manipulability-estimator/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
