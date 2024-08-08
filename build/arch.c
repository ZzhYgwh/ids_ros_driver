
    #if defined(__aarch64__) || defined(__ARM64__) || defined(_M_ARM64)
        #error cmake_ARCH arm64
    #elif defined(__arm__) || defined(__TARGET_ARCH_ARM)
        #error cmake_ARCH armhf
    #elif defined(__i386) || defined(__i386__) || defined(_M_IX86)
        #error cmake_ARCH x86_32
    #elif defined(__x86_64) || defined(__x86_64__) || defined(__amd64) || defined(_M_X64)
        #error cmake_ARCH x86_64
    #elif defined(__ia64) || defined(__ia64__) || defined(_M_IA64)
        #error cmake_ARCH ia64
    #elif defined(__ppc__) || defined(__ppc) || defined(__powerpc__)             || defined(_ARCH_COM) || defined(_ARCH_PWR) || defined(_ARCH_PPC)              || defined(_M_MPPC) || defined(_M_PPC)
        #if defined(__ppc64__) || defined(__powerpc64__) || defined(__64BIT__)
            #error cmake_ARCH ppc64
        #else
            #error cmake_ARCH ppc
        #endif
    #else
        #error cmake_ARCH unknown
    #endif
