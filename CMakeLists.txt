cmake_minimum_required(VERSION 3.16)

include(ExternalProject)
set(CMAKE_TOOLCHAIN_FILE toolchain.cmake)
project(ExistOS-OSLoader VERSION 0.1)



set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 11)


SET(BUILD_SHARED_LIBRARIES OFF)


add_link_options(-T ${CMAKE_SOURCE_DIR}/ldr_ld.script -Wl,--gc-sections)
 
SET(CMAKE_EXE_LINKER_FLAGS "-static")
set(CMAKE_BUILD_TYPE "MinSizeRel") 
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

include_directories(.)

aux_source_directory(.                                  LOADER_SRCS)


add_executable(rom.elf ${LOADER_SRCS})

target_link_libraries(rom.elf PRIVATE -lc_nano -nostartfiles)

target_compile_options(rom.elf PRIVATE -mtune=arm926ej-s -mcpu=arm926ej-s -mlittle-endian -marm -Wall -pipe -Os
)
 

add_custom_command(TARGET rom.elf POST_BUILD
    COMMAND arm-none-eabi-size ${CMAKE_CURRENT_BINARY_DIR}/rom.elf 
    )
	
add_custom_target(
  BOOT1.ROM ALL
  COMMAND ${CMAKE_OBJCOPY} -I elf32-little -O binary -R .note -R .comment -S ${CMAKE_CURRENT_BINARY_DIR}/rom.elf ${CMAKE_CURRENT_BINARY_DIR}/BOOT1.ROM 
  DEPENDS rom.elf 
  )
 