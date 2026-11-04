Import("env")

# micro_ros_platformio uses env["CFLAGS"] to generate its MCU CMake toolchain.
# The Teensy newlib time.h only declares clock_gettime when _POSIX_TIMERS is
# defined; zero keeps micro_ros_platformio's Arduino clock implementation active.
env.Append(CFLAGS=["-D_POSIX_TIMERS=0"])
