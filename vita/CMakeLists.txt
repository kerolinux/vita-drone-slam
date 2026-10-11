cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  if(DEFINED ENV{VITASDK})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VITASDK}/share/vita.toolchain.cmake" CACHE PATH "toolchain file")
  else()
    message(FATAL_ERROR "Please define VITASDK to point to your SDK path!")
  endif()
endif()

project(vita_drone)
include("${VITASDK}/share/vita.cmake" REQUIRED)

set(VITA_APP_NAME "Vita Drone")
set(VITA_TITLEID  "VDRN00001")
set(VITA_VERSION  "01.00")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -std=gnu11")

# The on-screen text uses debugScreen from the official vitasdk samples. If you already have a copy, pass
# -DDEBUGSCREEN_DIR=/path/to/samples/common; otherwise CMake downloads the samples repo (source only, nothing
# from it is built except debugScreen.c).
if(NOT DEFINED DEBUGSCREEN_DIR)
  include(FetchContent)
  FetchContent_Declare(vita_samples
    GIT_REPOSITORY https://github.com/vitasdk/samples.git
    GIT_TAG master
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR do_not_build_the_samples)
  FetchContent_MakeAvailable(vita_samples)
  set(DEBUGSCREEN_DIR "${vita_samples_SOURCE_DIR}/common")
endif()

include_directories(src ${DEBUGSCREEN_DIR})

# debugScreen.c is not listed: main.c includes it (DEBUG_SCREEN_CODE_INCLUDE) to reach the framebuffer.
add_executable(${PROJECT_NAME}
  src/main.c
)

target_link_libraries(${PROJECT_NAME}
  SceDisplay_stub
  SceCtrl_stub
  SceNet_stub
  SceSysmodule_stub
)

vita_create_self(${PROJECT_NAME}.self ${PROJECT_NAME})
# The FILE lines add the bubble icon and the LiveArea pictures (made by tools/make_assets.py) to the VPK.
vita_create_vpk(${PROJECT_NAME}.vpk ${VITA_TITLEID} ${PROJECT_NAME}.self
  VERSION ${VITA_VERSION}
  NAME ${VITA_APP_NAME}
  FILE sce_sys/icon0.png sce_sys/icon0.png
  FILE sce_sys/livearea/contents/bg.png sce_sys/livearea/contents/bg.png
  FILE sce_sys/livearea/contents/startup.png sce_sys/livearea/contents/startup.png
  FILE sce_sys/livearea/contents/template.xml sce_sys/livearea/contents/template.xml
)