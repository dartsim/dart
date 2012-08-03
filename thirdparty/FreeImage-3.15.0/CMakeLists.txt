PROJECT(freeimage-wrapper)

include(ExternalProject)

SET(FREEIMAGE_SRC_ROOT
        "Source/FreeImage/BitmapAccess.cpp"
        "Source/FreeImage/ColorLookup.cpp"
        "Source/FreeImage/FreeImage.cpp"
        "FreeImage.rc"
        "Source/FreeImage/FreeImageC.c"
        "Source/FreeImage/FreeImageIO.cpp"
        "Source/FreeImage/GetType.cpp"
        "Source/FreeImage/MemoryIO.cpp"
        "Source/FreeImage/PixelAccess.cpp" )

SET(FREEIMAGE_SRC_PLUGINS
        "Source/FreeImage/J2KHelper.cpp"
        "Source/FreeImage/Plugin.cpp"
        "Source/FreeImage/PluginBMP.cpp"
        "Source/FreeImage/PluginCUT.cpp"
        "Source/FreeImage/PluginDDS.cpp"
        "Source/FreeImage/PluginEXR.cpp"
        "Source/FreeImage/PluginG3.cpp"
        "Source/FreeImage/PluginGIF.cpp"
        "Source/FreeImage/PluginHDR.cpp"
        "Source/FreeImage/PluginICO.cpp"
        "Source/FreeImage/PluginIFF.cpp"
        "Source/FreeImage/PluginJ2K.cpp"
        "Source/FreeImage/PluginJP2.cpp"
        "Source/FreeImage/PluginJPEG.cpp"
        "Source/FreeImage/PluginKOALA.cpp"
        "Source/FreeImage/PluginMNG.cpp"
        "Source/FreeImage/PluginPCD.cpp"
        "Source/FreeImage/PluginPCX.cpp"
        "Source/FreeImage/PluginPFM.cpp"
        "Source/FreeImage/PluginPICT.cpp"
        "Source/FreeImage/PluginPNG.cpp"
        "Source/FreeImage/PluginPNM.cpp"
        "Source/FreeImage/PluginPSD.cpp"
        "Source/FreeImage/PluginRAS.cpp"
        "Source/FreeImage/PluginRAW.cpp"
        "Source/FreeImage/PluginSGI.cpp"
        "Source/FreeImage/PluginTARGA.cpp"
        "Source/FreeImage/PluginTIFF.cpp"
        "Source/FreeImage/PluginWBMP.cpp"
        "Source/FreeImage/PluginXBM.cpp"
        "Source/FreeImage/PluginXPM.cpp"
        "Source/FreeImage/PSDParser.cpp"
        "Source/FreeImage/TIFFLogLuv.cpp" )

SET(FREEIMAGE_SRC_CONV
        "Source/FreeImage/Conversion.cpp"
        "Source/FreeImage/Conversion16_555.cpp"
        "Source/FreeImage/Conversion16_565.cpp"
        "Source/FreeImage/Conversion24.cpp"
        "Source/FreeImage/Conversion32.cpp"
        "Source/FreeImage/Conversion4.cpp"
        "Source/FreeImage/Conversion8.cpp"
        "Source/FreeImage/ConversionFloat.cpp"
        "Source/FreeImage/ConversionRGBF.cpp"
        "Source/FreeImage/ConversionType.cpp"
        "Source/FreeImage/ConversionUINT16.cpp"
        "Source/FreeImage/Halftoning.cpp"
        "Source/FreeImage/tmoColorConvert.cpp"
        "Source/FreeImage/tmoDrago03.cpp"
        "Source/FreeImage/tmoFattal02.cpp"
        "Source/FreeImage/tmoReinhard05.cpp"
        "Source/FreeImage/ToneMapping.cpp" )

SET(FREEIMAGE_SRC_QUANT # Quantizers
        "Source/FreeImage/NNQuantizer.cpp"
        "Source/FreeImage/WuQuantizer.cpp" )

SET(FREEIMAGE_SRC_DEPRC # DeprecationMgr
        "Source/DeprecationManager/Deprecated.cpp"
        "Source/DeprecationManager/DeprecationMgr.cpp" )

SET(FREEIMAGE_SRC_PAGING # MultiPaging
        "Source/FreeImage/CacheFile.cpp"
        "Source/FreeImage/MultiPage.cpp"
        "Source/FreeImage/ZLibInterface.cpp" )

SET(FREEIMAGE_SRC_METADATA # Metadata
        "Source/Metadata/Exif.cpp"
        "Source/Metadata/FIRational.cpp"
        "Source/Metadata/FreeImageTag.cpp"
        "Source/Metadata/IPTC.cpp"
        "Source/Metadata/TagConversion.cpp"
        "Source/Metadata/TagLib.cpp"
        "Source/Metadata/XTIFF.cpp" )

SET(FREEIMAGE_SRC_TOOLKIT # "Toolkit Files"
        "Source/FreeImageToolkit/Background.cpp"
        "Source/FreeImageToolkit/BSplineRotate.cpp"
        "Source/FreeImageToolkit/Channels.cpp"
        "Source/FreeImageToolkit/ClassicRotate.cpp"
        "Source/FreeImageToolkit/Colors.cpp"
        "Source/FreeImageToolkit/CopyPaste.cpp"
        "Source/FreeImageToolkit/Display.cpp"
        "Source/FreeImageToolkit/Flip.cpp"
        "Source/FreeImageToolkit/JPEGTransform.cpp"
        "Source/FreeImageToolkit/MultigridPoissonSolver.cpp"
        "Source/FreeImageToolkit/Rescale.cpp"
        "Source/FreeImageToolkit/Resize.cpp"
        "Source/FreeImageToolkit/Resize.h" )

SET(FREEIMAGE_SRC ${FREEIMAGE_SRC_ROOT}
                  ${FREEIMAGE_SRC_PLUGINS}
                  ${FREEIMAGE_SRC_CONV}
                  ${FREEIMAGE_SRC_QUANT}
                  ${FREEIMAGE_SRC_DEPRC}
                  ${FREEIMAGE_SRC_PAGING}
                  ${FREEIMAGE_SRC_METADATA}
                  ${FREEIMAGE_SRC_TOOLKIT} )

SET(FREEIMAGE_HDR # Header Files
        "Source/CacheFile.h"
        "Source/DeprecationManager/DeprecationMgr.h"
        "Source/Metadata/FIRational.h"
        "Source/FreeImage.h"
        "Source/FreeImageIO.h"
        "Source/Metadata/FreeImageTag.h"
        "Source/Plugin.h"
        "Source/FreeImage/PSDParser.h"
        "Source/Quantizers.h"
        "Source/ToneMapping.h"
        "Source/Utilities.h" )

SET(FREEIMAGE_MISC Whatsnew.txt)


FUNCTION(INCLUDE_FREEIMAGE_VS MAINVER SUBVER PJEXT)
    SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

    SET(SUBPJS LibJPEG LibMNG LibPNG LibTIFF ZLib OpenEXR LibOpenJPEG LibRawLite)
    FOREACH(SUBPJ ${SUBPJS})
        INCLUDE_EXTERNAL_MSPROJECT(${SUBPJ} ${CMAKE_CURRENT_SOURCE_DIR}/Source/${SUBPJ}/${SUBPJ}.${SUBVER}.${PJEXT} )
    ENDFOREACH(SUBPJ)

    #INCLUDE_EXTERNAL_MSPROJECT(FreeImage ${CMAKE_CURRENT_SOURCE_DIR}/FreeImage.${MAINVER}.${PJEXT} ${SUBPJS})

    INCLUDE_DIRECTORIES( Source
                         Source/ZLib
                         Source/DeprecationManager
                         Source/OpenEXR/Half
                         Source/OpenEXR/Iex
                         Source/OpenEXR/IlmImf
                         Source/OpenEXR/Imath
                         Source/OpenEXR/IlmThread )

    ADD_DEFINITIONS( -DWIN32
                     -D_WINDOWS
                     -D_USRDLL
                     -DOPJ_STATIC
                     -DFREEIMAGE_EXPORTS
                     -D_CRT_SECURE_NO_DEPRECATE
                     -DLIBRAW_NODLL )

    SET(COMPILE_DEFINITIONS_DEBUG ${COMPILE_DEFINITIONS_DEBUG} _DEBUG)
    SET(COMPILE_DEFINITIONS_RELEASE ${COMPILE_DEFINITIONS_DEBUG} NDEBUG)
    SET(COMPILE_DEFINITIONS_MINSIZEREL ${COMPILE_DEFINITIONS_DEBUG} NDEBUG)
    SET(COMPILE_DEFINITIONS_RELWITHDEBINFO ${COMPILE_DEFINITIONS_DEBUG} NDEBUG)
    MESSAGE(STATUS "COMPILE_DEFINITIONS_DEBUG ${COMPILE_DEFINITIONS_DEBUG}")

    ADD_LIBRARY( FreeImage SHARED ${FREEIMAGE_SRC} ${FREEIMAGE_HDR} ${FREEIMAGE_MISC} )
    ADD_DEPENDENCIES( FreeImage ${SUBPJS} )
    SOURCE_GROUP( "Source Files" FILES ${FREEIMAGE_SRC_ROOT} )
    SOURCE_GROUP( "Source Files\\Plugins" FILES ${FREEIMAGE_SRC_PLUGINS} )
    SOURCE_GROUP( "Source Files\\Conversions" FILES ${FREEIMAGE_SRC_CONV} )
    SOURCE_GROUP( "Source Files\\Quantizers" FILES ${FREEIMAGE_SRC_QUANT} )
    SOURCE_GROUP( "Source Files\\Deprecation Manager" FILES ${FREEIMAGE_SRC_DEPRC} )
    SOURCE_GROUP( "Source Files\\MultiPaging" FILES ${FREEIMAGE_SRC_PAGING} )
    SOURCE_GROUP( "Source Files\\Metadata" FILES ${FREEIMAGE_SRC_METADATA} )
    SOURCE_GROUP( "Toolkit" FILES ${FREEIMAGE_SRC_TOOLKIT} )
    SOURCE_GROUP( "Header Files" FILES ${FREEIMAGE_HDR} )

    SET_TARGET_PROPERTIES( FreeImage ${SUBPJS}
                           PROPERTIES FOLDER ThirdParty/FreeImage )

    install(TARGETS FreeImage
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib/static )

ENDFUNCTION(INCLUDE_FREEIMAGE_VS)

if(CMAKE_GENERATOR MATCHES "Visual Studio 7")
    INCLUDE_FREEIMAGE_VS(2003 2003 vcproj)
elseif(CMAKE_GENERATOR MATCHES "Visual Studio 8")
    INCLUDE_FREEIMAGE_VS(2005 2005 vcproj)
elseif(CMAKE_GENERATOR MATCHES "Visual Studio 9")
    INCLUDE_FREEIMAGE_VS(2008 2008 vcproj)
elseif(CMAKE_GENERATOR MATCHES "Visual Studio 10")
    INCLUDE_FREEIMAGE_VS(2010 2008 vcxproj)
else()
    ExternalProject_Add(
        freeimage-ext
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
        CONFIGURE_COMMAND ""
    #	BUILD_COMMAND make
        BUILD_IN_SOURCE 1
        INSTALL_COMMAND ${CMAKE_COMMAND} -DINSTALL_PREFIX=${THIRDPARTY_INSTALL_PREFIX} -P ${CMAKE_CURRENT_SOURCE_DIR}/Install.cmake
        )
endif()

