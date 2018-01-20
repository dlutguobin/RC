SET(CMAKE_C_COMPILER "/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-gcc")
SET(CMAKE_C_COMPILER_ARG1 "")
SET(CMAKE_C_COMPILER_ID "GNU")
SET(CMAKE_C_PLATFORM_ID "Linux")

SET(CMAKE_AR "/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-ar")
SET(CMAKE_RANLIB "/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-ranlib")
SET(CMAKE_LINKER "/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-ld")
SET(CMAKE_COMPILER_IS_GNUCC 1)
SET(CMAKE_C_COMPILER_LOADED 1)
SET(CMAKE_COMPILER_IS_MINGW )
SET(CMAKE_COMPILER_IS_CYGWIN )
IF(CMAKE_COMPILER_IS_CYGWIN)
  SET(CYGWIN 1)
  SET(UNIX 1)
ENDIF(CMAKE_COMPILER_IS_CYGWIN)

SET(CMAKE_C_COMPILER_ENV_VAR "CC")

IF(CMAKE_COMPILER_IS_MINGW)
  SET(MINGW 1)
ENDIF(CMAKE_COMPILER_IS_MINGW)
SET(CMAKE_C_COMPILER_ID_RUN 1)
SET(CMAKE_C_SOURCE_FILE_EXTENSIONS c)
SET(CMAKE_C_IGNORE_EXTENSIONS h;H;o;O;obj;OBJ;def;DEF;rc;RC)
SET(CMAKE_C_LINKER_PREFERENCE 10)

# Save compiler ABI information.
SET(CMAKE_C_SIZEOF_DATA_PTR "4")
SET(CMAKE_C_COMPILER_ABI "ELF")
SET(CMAKE_C_LIBRARY_ARCHITECTURE "")

IF(CMAKE_C_SIZEOF_DATA_PTR)
  SET(CMAKE_SIZEOF_VOID_P "${CMAKE_C_SIZEOF_DATA_PTR}")
ENDIF(CMAKE_C_SIZEOF_DATA_PTR)

IF(CMAKE_C_COMPILER_ABI)
  SET(CMAKE_INTERNAL_PLATFORM_ABI "${CMAKE_C_COMPILER_ABI}")
ENDIF(CMAKE_C_COMPILER_ABI)

IF(CMAKE_C_LIBRARY_ARCHITECTURE)
  SET(CMAKE_LIBRARY_ARCHITECTURE "")
ENDIF()

SET(CMAKE_C_HAS_ISYSROOT "")


SET(CMAKE_C_IMPLICIT_LINK_LIBRARIES "c")
SET(CMAKE_C_IMPLICIT_LINK_DIRECTORIES "/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/toolchain/lib/gcc/i686-wrs-linux-gnu/4.6.3;/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/toolchain/lib/gcc;/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/toolchain/i686-wrs-linux-gnu/lib;/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/intel_atom_baytrail-wrs-linux/lib;/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/intel_atom_baytrail-wrs-linux/usr/lib")
