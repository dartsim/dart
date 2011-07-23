unit FreeImage;

// ==========================================================
// Delphi wrapper for FreeImage 3
//
// Design and implementation by
// - Simon Beavis
// - Peter Bystr�m
// - Anatoliy Pulyaevskiy (xvel84@rambler.ru)
//
// Contributors:
// - Lorenzo Monti (LM)  lomo74@gmail.com
//
// Revision history
// When        Who   What
// ----------- ----- -----------------------------------------------------------
// 2010-07-14  LM    Fixed some C->Delphi translation errors,
//                   updated to 3.13.1, made RAD2010 compliant (unicode)
// 2010-07-29  LM    Added Free Pascal / Lazarus 32 bit support
// 2010-11-12  LM    Updated to 3.14.1
//
// This file is part of FreeImage 3
//
// COVERED CODE IS PROVIDED UNDER THIS LICENSE ON AN "AS IS" BASIS, WITHOUT WARRANTY
// OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, WITHOUT LIMITATION, WARRANTIES
// THAT THE COVERED CODE IS FREE OF DEFECTS, MERCHANTABLE, FIT FOR A PARTICULAR PURPOSE
// OR NON-INFRINGING. THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE COVERED
// CODE IS WITH YOU. SHOULD ANY COVERED CODE PROVE DEFECTIVE IN ANY RESPECT, YOU (NOT
// THE INITIAL DEVELOPER OR ANY OTHER CONTRIBUTOR) ASSUME THE COST OF ANY NECESSARY
// SERVICING, REPAIR OR CORRECTION. THIS DISCLAIMER OF WARRANTY CONSTITUTES AN ESSENTIAL
// PART OF THIS LICENSE. NO USE OF ANY COVERED CODE IS AUTHORIZED HEREUNDER EXCEPT UNDER
// THIS DISCLAIMER.
//
// Use at your own risk!
// ==========================================================

interface

{$MINENUMSIZE 4} // Make sure enums are stored as an integer to be compatible with C/C++

{$I 'Version.inc'}

{$IFDEF MSWINDOWS}
uses Windows;
{$ELSE}
type
  LONG = LongInt;
  DWORD = Cardinal;

  BITMAPINFOHEADER = record
    biSize : DWORD;
    biWidth : LONG;
    biHeight : LONG;
    biPlanes : WORD;
    biBitCount : WORD;
    biCompression : DWORD;
    biSizeImage : DWORD;
    biXPelsPerMeter : LONG;
    biYPelsPerMeter : LONG;
    biClrUsed : DWORD;
    biClrImportant : DWORD;
  end;
  LPBITMAPINFOHEADER = ^BITMAPINFOHEADER;
  TBITMAPINFOHEADER = BITMAPINFOHEADER;
  PBITMAPINFOHEADER = ^BITMAPINFOHEADER;

  RGBQUAD = record
    rgbBlue : BYTE;
    rgbGreen : BYTE;
    rgbRed : BYTE;
    rgbReserved : BYTE;
  end;
  tagRGBQUAD = RGBQUAD;
  TRGBQUAD = RGBQUAD;
  PRGBQUAD = ^RGBQUAD;

  BITMAPINFO = record
    bmiHeader : BITMAPINFOHEADER;
    bmiColors : array[0..0] of RGBQUAD;
  end;
  LPBITMAPINFO = ^BITMAPINFO;
  PBITMAPINFO = ^BITMAPINFO;
  TBITMAPINFO = BITMAPINFO;
{$ENDIF}

const
  FIDLL = {$IFDEF MSWINDOWS}'FreeImage.dll';{$ENDIF}
          {$IFDEF LINUX}'libfreeimage.so';{$ENDIF}

const
  // Version information
  FREEIMAGE_MAJOR_VERSION  = 3;
  FREEIMAGE_MINOR_VERSION  = 14;
  FREEIMAGE_RELEASE_SERIAL = 1;
  // This really only affects 24 and 32 bit formats, the rest are always RGB order.
  FREEIMAGE_COLORORDER_BGR = 0;
  FREEIMAGE_COLORORDER_RGB = 1;
  FREEIMAGE_COLORORDER = FREEIMAGE_COLORORDER_BGR;

// --------------------------------------------------------------------------
// Bitmap types -------------------------------------------------------------
// --------------------------------------------------------------------------

type
  FIBITMAP = record
    data: Pointer;
  end;
  PFIBITMAP = ^FIBITMAP;

  FIMULTIBITMAP = record
    data: Pointer;
  end;
  PFIMULTIBITMAP = ^FIMULTIBITMAP;

// --------------------------------------------------------------------------
// Types used in the library (specific to FreeImage) ------------------------
// --------------------------------------------------------------------------

type
  {* 48-bit RGB }
  tagFIRGB16 = packed record
    red: WORD;
    green: WORD;
    blue: WORD;
  end;
  FIRGB16 = tagFIRGB16;

  {* 64-bit RGBA }
  tagFIRGBA16 = packed record
    red: WORD;
    green: WORD;
    blue: WORD;
    alpha: WORD;
  end;
  FIRGBA16 = tagFIRGBA16;

  {* 96-bit RGB Float }
  tagFIRGBF = packed record
    red: Single;
    green: Single;
    blue: Single;
  end;
  FIRGBF = tagFIRGBF;

  {* 128-bit RGBA Float }
  tagFIRGBAF = packed record
    red: Single;
    green: Single;
    blue: Single;
    alpha: Single;
  end;
  FIRGBAF = tagFIRGBAF;

  {* Data structure for COMPLEX type (complex number) }
  tagFICOMPLEX = packed record
    /// real part
    r: Double;
    /// imaginary part
    i: Double;
  end;
  FICOMPLEX = tagFICOMPLEX;

// --------------------------------------------------------------------------
// Indexes for byte arrays, masks and shifts for treating pixels as words ---
// These coincide with the order of RGBQUAD and RGBTRIPLE -------------------
// Little Endian (x86 / MS Windows, Linux) : BGR(A) order -------------------
// --------------------------------------------------------------------------

const
  FI_RGBA_RED         = 2;
  FI_RGBA_GREEN       = 1;
  FI_RGBA_BLUE        = 0;
  FI_RGBA_ALPHA       = 3;
  FI_RGBA_RED_MASK    = $00FF0000;
  FI_RGBA_GREEN_MASK  = $0000FF00;
  FI_RGBA_BLUE_MASK   = $000000FF;
  FI_RGBA_ALPHA_MASK  = $FF000000;
  FI_RGBA_RED_SHIFT   = 16;
  FI_RGBA_GREEN_SHIFT = 8;
  FI_RGBA_BLUE_SHIFT  = 0;
  FI_RGBA_ALPHA_SHIFT = 24;

  FI_RGBA_RGB_MASK = FI_RGBA_RED_MASK or FI_RGBA_GREEN_MASK or FI_RGBA_BLUE_MASK;

// --------------------------------------------------------------------------
// The 16bit macros only include masks and shifts, --------------------------
// since each color element is not byte aligned -----------------------------
// --------------------------------------------------------------------------

const
  FI16_555_RED_MASK    = $7C00;
  FI16_555_GREEN_MASK  = $03E0;
  FI16_555_BLUE_MASK   = $001F;
  FI16_555_RED_SHIFT   = 10;
  FI16_555_GREEN_SHIFT = 5;
  FI16_555_BLUE_SHIFT  = 0;
  FI16_565_RED_MASK    = $F800;
  FI16_565_GREEN_MASK  = $07E0;
  FI16_565_BLUE_MASK   = $001F;
  FI16_565_RED_SHIFT   = 11;
  FI16_565_GREEN_SHIFT = 5;
  FI16_565_BLUE_SHIFT  = 0;

// --------------------------------------------------------------------------
// ICC profile support ------------------------------------------------------
// --------------------------------------------------------------------------

const
  FIICC_DEFAULT = $0;
  FIICC_COLOR_IS_CMYK = $1;

type
  FIICCPROFILE = record
    flags: WORD;   // info flag
    size: DWORD;   // profile's size measured in bytes
    data: Pointer; // points to a block of contiguous memory containing the profile
  end;
  PFIICCPROFILE = ^FIICCPROFILE;

// --------------------------------------------------------------------------
// Important enums ----------------------------------------------------------
// --------------------------------------------------------------------------

type
  FREE_IMAGE_FORMAT         = type Integer;
  FREE_IMAGE_TYPE           = type Integer;
  FREE_IMAGE_COLOR_TYPE     = type Integer;
  FREE_IMAGE_QUANTIZE       = type Integer;
  FREE_IMAGE_DITHER         = type Integer;
  FREE_IMAGE_FILTER         = type Integer;
  FREE_IMAGE_COLOR_CHANNEL  = type Integer;
  FREE_IMAGE_MDTYPE         = type Integer;
  FREE_IMAGE_MDMODEL        = type Integer;
  FREE_IMAGE_JPEG_OPERATION = type Integer;
  FREE_IMAGE_TMO            = type Integer;

const
  // I/O image format identifiers.
  FIF_UNKNOWN = FREE_IMAGE_FORMAT(-1);
  FIF_BMP     = FREE_IMAGE_FORMAT(0);
  FIF_ICO     = FREE_IMAGE_FORMAT(1);
  FIF_JPEG    = FREE_IMAGE_FORMAT(2);
  FIF_JNG     = FREE_IMAGE_FORMAT(3);
  FIF_KOALA   = FREE_IMAGE_FORMAT(4);
  FIF_LBM     = FREE_IMAGE_FORMAT(5);
  FIF_IFF     = FIF_LBM;
  FIF_MNG     = FREE_IMAGE_FORMAT(6);
  FIF_PBM     = FREE_IMAGE_FORMAT(7);
  FIF_PBMRAW  = FREE_IMAGE_FORMAT(8);
  FIF_PCD     = FREE_IMAGE_FORMAT(9);
  FIF_PCX     = FREE_IMAGE_FORMAT(10);
  FIF_PGM     = FREE_IMAGE_FORMAT(11);
  FIF_PGMRAW  = FREE_IMAGE_FORMAT(12);
  FIF_PNG     = FREE_IMAGE_FORMAT(13);
  FIF_PPM     = FREE_IMAGE_FORMAT(14);
  FIF_PPMRAW  = FREE_IMAGE_FORMAT(15);
  FIF_RAS     = FREE_IMAGE_FORMAT(16);
  FIF_TARGA   = FREE_IMAGE_FORMAT(17);
  FIF_TIFF    = FREE_IMAGE_FORMAT(18);
  FIF_WBMP    = FREE_IMAGE_FORMAT(19);
  FIF_PSD     = FREE_IMAGE_FORMAT(20);
  FIF_CUT     = FREE_IMAGE_FORMAT(21);
  FIF_XBM     = FREE_IMAGE_FORMAT(22);
  FIF_XPM     = FREE_IMAGE_FORMAT(23);
  FIF_DDS     = FREE_IMAGE_FORMAT(24);
  FIF_GIF     = FREE_IMAGE_FORMAT(25);
  FIF_HDR     = FREE_IMAGE_FORMAT(26);
  FIF_FAXG3   = FREE_IMAGE_FORMAT(27);
  FIF_SGI     = FREE_IMAGE_FORMAT(28);
  FIF_EXR     = FREE_IMAGE_FORMAT(29);
  FIF_J2K     = FREE_IMAGE_FORMAT(30);
  FIF_JP2     = FREE_IMAGE_FORMAT(31);
  FIF_PFM     = FREE_IMAGE_FORMAT(32);
  FIF_PICT    = FREE_IMAGE_FORMAT(33);
  FIF_RAW     = FREE_IMAGE_FORMAT(34);

  // Image type used in FreeImage.
  FIT_UNKNOWN = FREE_IMAGE_TYPE(0);  // unknown type
  FIT_BITMAP  = FREE_IMAGE_TYPE(1);  // standard image: 1-, 4-, 8-, 16-, 24-, 32-bit
  FIT_UINT16  = FREE_IMAGE_TYPE(2);  // array of unsigned short: unsigned 16-bit
  FIT_INT16   = FREE_IMAGE_TYPE(3);  // array of short: signed 16-bit
  FIT_UINT32  = FREE_IMAGE_TYPE(4);  // array of unsigned long: unsigned 32-bit
  FIT_INT32   = FREE_IMAGE_TYPE(5);  // array of long: signed 32-bit
  FIT_FLOAT   = FREE_IMAGE_TYPE(6);  // array of float: 32-bit IEEE floating point
  FIT_DOUBLE  = FREE_IMAGE_TYPE(7);  // array of double: 64-bit IEEE floating point
  FIT_COMPLEX = FREE_IMAGE_TYPE(8);  // array of FICOMPLEX: 2 x 64-bit IEEE floating point
  FIT_RGB16   = FREE_IMAGE_TYPE(9);  // 48-bit RGB image: 3 x 16-bit
  FIT_RGBA16  = FREE_IMAGE_TYPE(10); // 64-bit RGBA image: 4 x 16-bit
  FIT_RGBF    = FREE_IMAGE_TYPE(11); // 96-bit RGB float image: 3 x 32-bit IEEE floating point
  FIT_RGBAF   = FREE_IMAGE_TYPE(12); // 128-bit RGBA float image: 4 x 32-bit IEEE floating point

  // Image color type used in FreeImage.
  FIC_MINISWHITE = FREE_IMAGE_COLOR_TYPE(0); // min value is white
  FIC_MINISBLACK = FREE_IMAGE_COLOR_TYPE(1); // min value is black
  FIC_RGB        = FREE_IMAGE_COLOR_TYPE(2); // RGB color model
  FIC_PALETTE    = FREE_IMAGE_COLOR_TYPE(3); // color map indexed
  FIC_RGBALPHA   = FREE_IMAGE_COLOR_TYPE(4); // RGB color model with alpha channel
  FIC_CMYK       = FREE_IMAGE_COLOR_TYPE(5); // CMYK color model

  // Color quantization algorithms. Constants used in FreeImage_ColorQuantize.
  FIQ_WUQUANT = FREE_IMAGE_QUANTIZE(0); // Xiaolin Wu color quantization algorithm
  FIQ_NNQUANT = FREE_IMAGE_QUANTIZE(1); // NeuQuant neural-net quantization algorithm by Anthony Dekker

  // Dithering algorithms. Constants used FreeImage_Dither.
  FID_FS            = FREE_IMAGE_DITHER(0); // Floyd & Steinberg error diffusion
  FID_BAYER4x4      = FREE_IMAGE_DITHER(1); // Bayer ordered dispersed dot dithering (order 2 dithering matrix)
  FID_BAYER8x8      = FREE_IMAGE_DITHER(2); // Bayer ordered dispersed dot dithering (order 3 dithering matrix)
  FID_CLUSTER6x6    = FREE_IMAGE_DITHER(3); // Ordered clustered dot dithering (order 3 - 6x6 matrix)
  FID_CLUSTER8x8    = FREE_IMAGE_DITHER(4); // Ordered clustered dot dithering (order 4 - 8x8 matrix)
  FID_CLUSTER16x16  = FREE_IMAGE_DITHER(5); // Ordered clustered dot dithering (order 8 - 16x16 matrix)
  FID_BAYER16x16    = FREE_IMAGE_DITHER(6); // Bayer ordered dispersed dot dithering (order 4 dithering matrix)

  // Lossless JPEG transformations Constants used in FreeImage_JPEGTransform
  FIJPEG_OP_NONE        = FREE_IMAGE_JPEG_OPERATION(0); // no transformation
  FIJPEG_OP_FLIP_H      = FREE_IMAGE_JPEG_OPERATION(1); // horizontal flip
  FIJPEG_OP_FLIP_V      = FREE_IMAGE_JPEG_OPERATION(2); // vertical flip
  FIJPEG_OP_TRANSPOSE   = FREE_IMAGE_JPEG_OPERATION(3); // transpose across UL-to-LR axis
  FIJPEG_OP_TRANSVERSE  = FREE_IMAGE_JPEG_OPERATION(4); // transpose across UR-to-LL axis
  FIJPEG_OP_ROTATE_90   = FREE_IMAGE_JPEG_OPERATION(5); // 90-degree clockwise rotation
  FIJPEG_OP_ROTATE_180  = FREE_IMAGE_JPEG_OPERATION(6); // 180-degree rotation
  FIJPEG_OP_ROTATE_270  = FREE_IMAGE_JPEG_OPERATION(7); // 270-degree clockwise (or 90 ccw)

  // Tone mapping operators. Constants used in FreeImage_ToneMapping.
  FITMO_DRAGO03    = FREE_IMAGE_TMO(0); // Adaptive logarithmic mapping (F. Drago, 2003)
  FITMO_REINHARD05 = FREE_IMAGE_TMO(1); // Dynamic range reduction inspired by photoreceptor physiology (E. Reinhard, 2005)
  FITMO_FATTAL02   = FREE_IMAGE_TMO(2); // Gradient domain high dynamic range compression (R. Fattal, 2002)

  // Upsampling / downsampling filters. Constants used in FreeImage_Rescale.
  FILTER_BOX        = FREE_IMAGE_FILTER(0); // Box, pulse, Fourier window, 1st order (constant) b-spline
  FILTER_BICUBIC    = FREE_IMAGE_FILTER(1); // Mitchell & Netravali's two-param cubic filter
  FILTER_BILINEAR   = FREE_IMAGE_FILTER(2); // Bilinear filter
  FILTER_BSPLINE    = FREE_IMAGE_FILTER(3); // 4th order (cubic) b-spline
  FILTER_CATMULLROM = FREE_IMAGE_FILTER(4); // Catmull-Rom spline, Overhauser spline
  FILTER_LANCZOS3   = FREE_IMAGE_FILTER(5); // Lanczos3 filter

  // Color channels. Constants used in color manipulation routines.
  FICC_RGB   = FREE_IMAGE_COLOR_CHANNEL(0); // Use red, green and blue channels
  FICC_RED   = FREE_IMAGE_COLOR_CHANNEL(1); // Use red channel
  FICC_GREEN = FREE_IMAGE_COLOR_CHANNEL(2); // Use green channel
  FICC_BLUE  = FREE_IMAGE_COLOR_CHANNEL(3); // Use blue channel
  FICC_ALPHA = FREE_IMAGE_COLOR_CHANNEL(4); // Use alpha channel
  FICC_BLACK = FREE_IMAGE_COLOR_CHANNEL(5); // Use black channel
  FICC_REAL  = FREE_IMAGE_COLOR_CHANNEL(6); // Complex images: use real part
  FICC_IMAG  = FREE_IMAGE_COLOR_CHANNEL(7); // Complex images: use imaginary part
  FICC_MAG   = FREE_IMAGE_COLOR_CHANNEL(8); // Complex images: use magnitude
  FICC_PHASE = FREE_IMAGE_COLOR_CHANNEL(9); // Complex images: use phase

  // Tag data type information (based on TIFF specifications)
  FIDT_NOTYPE    = FREE_IMAGE_MDTYPE(0);  // placeholder
  FIDT_BYTE      = FREE_IMAGE_MDTYPE(1);  // 8-bit unsigned integer
  FIDT_ASCII     = FREE_IMAGE_MDTYPE(2);  // 8-bit bytes w/ last byte null
  FIDT_SHORT     = FREE_IMAGE_MDTYPE(3);  // 16-bit unsigned integer
  FIDT_LONG      = FREE_IMAGE_MDTYPE(4);  // 32-bit unsigned integer
  FIDT_RATIONAL  = FREE_IMAGE_MDTYPE(5);  // 64-bit unsigned fraction
  FIDT_SBYTE     = FREE_IMAGE_MDTYPE(6);  // 8-bit signed integer
  FIDT_UNDEFINED = FREE_IMAGE_MDTYPE(7);  // 8-bit untyped data
  FIDT_SSHORT    = FREE_IMAGE_MDTYPE(8);  // 16-bit signed integer
  FIDT_SLONG     = FREE_IMAGE_MDTYPE(9);  // 32-bit signed integer
  FIDT_SRATIONAL = FREE_IMAGE_MDTYPE(10); // 64-bit signed fraction
  FIDT_FLOAT     = FREE_IMAGE_MDTYPE(11); // 32-bit IEEE floating point
  FIDT_DOUBLE    = FREE_IMAGE_MDTYPE(12); // 64-bit IEEE floating point
  FIDT_IFD       = FREE_IMAGE_MDTYPE(13); // 32-bit unsigned integer (offset)
  FIDT_PALETTE   = FREE_IMAGE_MDTYPE(14); // 32-bit RGBQUAD

  // Metadata models supported by FreeImage
  FIMD_NODATA         = FREE_IMAGE_MDMODEL(-1);
  FIMD_COMMENTS       = FREE_IMAGE_MDMODEL(0);  // single comment or keywords
  FIMD_EXIF_MAIN      = FREE_IMAGE_MDMODEL(1);  // Exif-TIFF metadata
  FIMD_EXIF_EXIF      = FREE_IMAGE_MDMODEL(2);  // Exif-specific metadata
  FIMD_EXIF_GPS       = FREE_IMAGE_MDMODEL(3);  // Exif GPS metadata
  FIMD_EXIF_MAKERNOTE = FREE_IMAGE_MDMODEL(4);  // Exif maker note metadata
  FIMD_EXIF_INTEROP   = FREE_IMAGE_MDMODEL(5);  // Exif interoperability metadata
  FIMD_IPTC           = FREE_IMAGE_MDMODEL(6);  // IPTC/NAA metadata
  FIMD_XMP            = FREE_IMAGE_MDMODEL(7);  // Abobe XMP metadata
  FIMD_GEOTIFF        = FREE_IMAGE_MDMODEL(8);  // GeoTIFF metadata (to be implemented)
  FIMD_ANIMATION      = FREE_IMAGE_MDMODEL(9);  // Animation metadata
  FIMD_CUSTOM         = FREE_IMAGE_MDMODEL(10); // Used to attach other metadata types to a dib
  FIMD_EXIF_RAW       = FREE_IMAGE_MDMODEL(11); // Exif metadata as a raw buffer

type
  // Handle to a metadata model
  FIMETADATA = record
    data: Pointer;
  end;
  PFIMETADATA = ^FIMETADATA;

  // Handle to a metadata tag
  FITAG = record
    data: Pointer;
  end;
  PFITAG = ^FITAG;

// --------------------------------------------------------------------------
// File IO routines ---------------------------------------------------------
// --------------------------------------------------------------------------

type
  fi_handle = Pointer;

  FI_ReadProc = function(buffer: Pointer; size, count: Cardinal;
    handle: fi_handle): Cardinal; stdcall;
  FI_WriteProc = function(buffer: Pointer; size, count: Cardinal;
    handle: fi_handle): Cardinal; stdcall;
  FI_SeekProc = function(handle: fi_handle; offset: LongInt;
    origin: Integer): Integer; stdcall;
  FI_TellProc = function(handle: fi_handle): LongInt; stdcall;

  FreeImageIO = packed record
    read_proc : FI_ReadProc;     // pointer to the function used to read data
    write_proc: FI_WriteProc;    // pointer to the function used to write data
    seek_proc : FI_SeekProc;     // pointer to the function used to seek
    tell_proc : FI_TellProc;     // pointer to the function used to aquire the current position
  end;
  PFreeImageIO = ^FreeImageIO;

  // Handle to a memory I/O stream
  FIMEMORY = record
    data: Pointer;
  end;
  PFIMEMORY = ^FIMEMORY;

const
  // constants used in FreeImage_Seek for Origin parameter
  SEEK_SET = 0;
  SEEK_CUR = 1;
  SEEK_END = 2;

// --------------------------------------------------------------------------
// Plugin routines ----------------------------------------------------------
// --------------------------------------------------------------------------

type
  PPlugin = ^Plugin;

  FI_FormatProc = function: PAnsiChar; stdcall;
  FI_DescriptionProc = function: PAnsiChar; stdcall;
  FI_ExtensionListProc = function: PAnsiChar; stdcall;
  FI_RegExprProc = function: PAnsiChar; stdcall;
  FI_OpenProc = function(io: PFreeImageIO; handle: fi_handle;
    read: Boolean): Pointer; stdcall;
  FI_CloseProc = procedure(io: PFreeImageIO; handle: fi_handle;
    data: Pointer); stdcall;
  FI_PageCountProc = function(io: PFreeImageIO; handle: fi_handle;
    data: Pointer): Integer; stdcall;
  FI_PageCapabilityProc = function(io: PFreeImageIO; handle: fi_handle;
    data: Pointer): Integer; stdcall;
  FI_LoadProc = function(io: PFreeImageIO; handle: fi_handle; page, flags: Integer;
    data: Pointer): PFIBITMAP; stdcall;
  FI_SaveProc = function(io: PFreeImageIO; dib: PFIBITMAP; handle: fi_handle;
    page, flags: Integer; data: Pointer): Boolean; stdcall;
  FI_ValidateProc = function(io: PFreeImageIO; handle: fi_handle): Boolean; stdcall;
  FI_MimeProc = function: PAnsiChar; stdcall;
  FI_SupportsExportBPPProc = function(bpp: integer): Boolean; stdcall;
  FI_SupportsExportTypeProc = function(atype: FREE_IMAGE_TYPE): Boolean; stdcall;
  FI_SupportsICCProfilesProc = function: Boolean; stdcall;
  FI_SupportsNoPixelsProc = function: Boolean; stdcall;

  Plugin = record
    format_proc: FI_FormatProc;
    description_proc: FI_DescriptionProc;
    extension_proc: FI_ExtensionListProc;
    regexpr_proc: FI_RegExprProc;
    open_proc: FI_OpenProc;
    close_proc: FI_CloseProc;
    pagecount_proc: FI_PageCountProc;
    pagecapability_proc: FI_PageCapabilityProc;
    load_proc: FI_LoadProc;
    save_proc: FI_SaveProc;
    validate_proc: FI_ValidateProc;
    mime_proc: FI_MimeProc;
    supports_export_bpp_proc: FI_SupportsExportBPPProc;
    supports_export_type_proc: FI_SupportsExportTypeProc;
    supports_icc_profiles_proc: FI_SupportsICCProfilesProc;
    supports_no_pixels_proc: FI_SupportsNoPixelsProc;
  end;

  FI_InitProc = procedure(aplugin: PPlugin; format_id: Integer); stdcall;

// --------------------------------------------------------------------------
// Load/Save flag constants -------------------------------------------------
// --------------------------------------------------------------------------

const
  FIF_LOAD_NOPIXELS   = $8000;  // loading: load the image header only (not supported by all plugins)
  BMP_DEFAULT         = 0;
  BMP_SAVE_RLE        = 1;
  CUT_DEFAULT         = 0;
  DDS_DEFAULT         = 0;
  EXR_DEFAULT         = 0;      // save data as half with piz-based wavelet compression
  EXR_FLOAT           = $0001;  // save data as float instead of as half (not recommended)
  EXR_NONE            = $0002;  // save with no compression
  EXR_ZIP             = $0004;  // save with zlib compression, in blocks of 16 scan lines
  EXR_PIZ             = $0008;  // save with piz-based wavelet compression
  EXR_PXR24           = $0010;  // save with lossy 24-bit float compression
  EXR_B44             = $0020;  // save with lossy 44% float compression - goes to 22% when combined with EXR_LC
  EXR_LC              = $0040;  // save images with one luminance and two chroma channels, rather than as RGB (lossy compression)
  FAXG3_DEFAULT       = 0;
  GIF_DEFAULT         = 0;
  GIF_LOAD256         = 1;     // Load the image as a 256 color image with ununsed palette entries, if it's 16 or 2 color
  GIF_PLAYBACK        = 2;     // 'Play' the GIF to generate each frame (as 32bpp) instead of returning raw frame data when loading
  HDR_DEFAULT         = 0;
  ICO_DEFAULT         = 0;
  ICO_MAKEALPHA       = 1;     // convert to 32bpp and create an alpha channel from the AND-mask when loading
  IFF_DEFAULT         = 0;
  J2K_DEFAULT         = 0;     // save with a 16:1 rate
  JP2_DEFAULT         = 0;     // save with a 16:1 rate
  JPEG_DEFAULT        = 0;
  JPEG_FAST           = 1;
  JPEG_ACCURATE       = 2;
  JPEG_CMYK           = $0004; // load separated CMYK "as is" (use | to combine with other flags)
  JPEG_EXIFROTATE     = $0008; // load and rotate according to Exif 'Orientation' tag if available
  JPEG_QUALITYSUPERB  = $0080; // save with superb quality (100:1)
  JPEG_QUALITYGOOD    = $0100; // save with good quality (75:1)
  JPEG_QUALITYNORMAL  = $0200; // save with normal quality (50:1)
  JPEG_QUALITYAVERAGE = $0400; // save with average quality (25:1)
  JPEG_QUALITYBAD     = $0800; // save with bad quality (10:1)
  JPEG_PROGRESSIVE    = $2000; // save as a progressive-JPEG (use | to combine with other save flags)
  JPEG_SUBSAMPLING_411 = $1000;  // save with high 4x1 chroma subsampling (4:1:1)
  JPEG_SUBSAMPLING_420 = $4000;  // save with medium 2x2 medium chroma subsampling (4:2:0) - default value
  JPEG_SUBSAMPLING_422 = $8000;  // save with low 2x1 chroma subsampling (4:2:2)
  JPEG_SUBSAMPLING_444 = $10000; // save with no chroma subsampling (4:4:4)
  JPEG_OPTIMIZE       = $20000; // on saving, compute optimal Huffman coding tables (can reduce a few percent of file size)
  KOALA_DEFAULT       = 0;
  LBM_DEFAULT         = 0;
  MNG_DEFAULT         = 0;
  PCD_DEFAULT         = 0;
  PCD_BASE            = 1;     // load the bitmap sized 768 x 512
  PCD_BASEDIV4        = 2;     // load the bitmap sized 384 x 256
  PCD_BASEDIV16       = 3;     // load the bitmap sized 192 x 128
  PCX_DEFAULT         = 0;
  PFM_DEFAULT         = 0;
  PICT_DEFAULT        = 0;
  PNG_DEFAULT         = 0;
  PNG_IGNOREGAMMA     = 1;     // avoid gamma correction
  PNG_Z_BEST_SPEED          = $0001; // save using ZLib level 1 compression flag (default value is 6)
  PNG_Z_DEFAULT_COMPRESSION = $0006; // save using ZLib level 6 compression flag (default recommended value)
  PNG_Z_BEST_COMPRESSION    = $0009; // save using ZLib level 9 compression flag (default value is 6)
  PNG_Z_NO_COMPRESSION      = $0100; // save without ZLib compression
  PNG_INTERLACED            = $0200; // save using Adam7 interlacing (use | to combine with other save flags)
  PNM_DEFAULT         = 0;
  PNM_SAVE_RAW        = 0;     // If set the writer saves in RAW format (i.e. P4, P5 or P6)
  PNM_SAVE_ASCII      = 1;     // If set the writer saves in ASCII format (i.e. P1, P2 or P3)
  PSD_DEFAULT         = 0;
  PSD_CMYK            = 1; // reads tags for separated CMYK (default is conversion to RGB)
  PSD_LAB             = 2; // reads tags for CIELab (default is conversion to RGB)
  RAS_DEFAULT         = 0;
  RAW_DEFAULT         = 0; // load the file as linear RGB 48-bit
  RAW_PREVIEW         = 1; // try to load the embedded JPEG preview with included Exif Data or default to RGB 24-bit
  RAW_DISPLAY         = 2; // load the file as RGB 24-bit
  SGI_DEFAULT         = 0;
  TARGA_DEFAULT       = 0;
  TARGA_LOAD_RGB888   = 1;     // If set the loader converts RGB555 and ARGB8888 -> RGB888.
  TARGA_SAVE_RLE      = 2;     // If set, the writer saves with RLE compression
  TIFF_DEFAULT        = 0;
  TIFF_CMYK           = $0001;  // reads/stores tags for separated CMYK (use | to combine with compression flags)
  TIFF_PACKBITS       = $0100;  // save using PACKBITS compression
  TIFF_DEFLATE        = $0200;  // save using DEFLATE compression
  TIFF_ADOBE_DEFLATE  = $0400;  // save using ADOBE DEFLATE compression
  TIFF_NONE           = $0800;  // save without any compression
  TIFF_CCITTFAX3      = $1000;  // save using CCITT Group 3 fax encoding
  TIFF_CCITTFAX4      = $2000;  // save using CCITT Group 4 fax encoding
  TIFF_LZW            = $4000;  // save using LZW compression
  TIFF_JPEG           = $8000;  // save using JPEG compression
  TIFF_LOGLUV         = $10000; // save using LogLuv compression
  WBMP_DEFAULT        = 0;
  XBM_DEFAULT         = 0;
  XPM_DEFAULT         = 0;

// --------------------------------------------------------------------------
// Background filling options -----------------------------------------------
// Constants used in FreeImage_FillBackground and FreeImage_EnlargeCanvas
// --------------------------------------------------------------------------

const
  FI_COLOR_IS_RGB_COLOR         = $00; // RGBQUAD color is a RGB color (contains no valid alpha channel)
  FI_COLOR_IS_RGBA_COLOR        = $01; // RGBQUAD color is a RGBA color (contains a valid alpha channel)
  FI_COLOR_FIND_EQUAL_COLOR     = $02; // For palettized images: lookup equal RGB color from palette
  FI_COLOR_ALPHA_IS_INDEX       = $04; // The color's rgbReserved member (alpha) contains the palette index to be used
  FI_COLOR_PALETTE_SEARCH_MASK  = FI_COLOR_FIND_EQUAL_COLOR or FI_COLOR_ALPHA_IS_INDEX; // No color lookup is performed

// --------------------------------------------------------------------------
// Init/Error routines ------------------------------------------------------
// --------------------------------------------------------------------------

procedure FreeImage_Initialise(load_local_plugins_only: Boolean = False); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_Initialise@4'{$ENDIF};
procedure FreeImage_DeInitialise; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_DeInitialise@0'{$ENDIF};

// --------------------------------------------------------------------------
// Version routines ---------------------------------------------------------
// --------------------------------------------------------------------------

function FreeImage_GetVersion: PAnsiChar; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetVersion@0'{$ENDIF};
function FreeImage_GetCopyrightMessage: PAnsiChar; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetCopyrightMessage@0'{$ENDIF};

// --------------------------------------------------------------------------
// Message output functions -------------------------------------------------
// --------------------------------------------------------------------------

type
  FreeImage_OutputMessageFunction = procedure(fif: FREE_IMAGE_FORMAT;
    msg: PAnsiChar); cdecl;
  FreeImage_OutputMessageFunctionStdCall = procedure(fif: FREE_IMAGE_FORMAT;
    msg: PAnsiChar); stdcall;

procedure FreeImage_SetOutputMessageStdCall(omf: FreeImage_OutputMessageFunctionStdCall); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetOutputMessageStdCall@4'{$ENDIF};
procedure FreeImage_SetOutputMessage(omf: FreeImage_OutputMessageFunction); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetOutputMessage@4'{$ENDIF};
{$IFDEF DELPHI6}
//this is declared stdcall in the C header but it is actually cdecl.
//with varargs functions, clearing the stack is caller's responsibility
//(since the callee doesn't know how many parameters were passed).
//cdecl is the right convention here, not stdcall
procedure FreeImage_OutputMessageProc(fif: Integer; fmt: PAnsiChar); cdecl; varargs;
  external FIDLL {$IFDEF MSWINDOWS}name 'FreeImage_OutputMessageProc'{$ENDIF};
{$ELSE}
//older Delphi versions (<6) do not support varargs.
//we provide a wrapper that uses open arrays instead
procedure FreeImage_OutputMessageProc(fif: Integer; fmt: PAnsiChar; args: array of const);
{$ENDIF}

// --------------------------------------------------------------------------
// Allocate / Clone / Unload routines ---------------------------------------
// --------------------------------------------------------------------------

function FreeImage_Allocate(width, height, bpp: Integer; red_mask: Cardinal = 0;
  green_mask: Cardinal = 0; blue_mask: Cardinal = 0): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_Allocate@24'{$ENDIF};
function FreeImage_AllocateT(atype: FREE_IMAGE_TYPE; width, height: Integer;
  bpp: Integer = 8; red_mask: Cardinal = 0; green_mask: Cardinal = 0;
  blue_mask: Cardinal = 0): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_AllocateT@28'{$ENDIF};
function FreeImage_Clone(dib: PFIBITMAP): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_Clone@4'{$ENDIF};
procedure FreeImage_Unload(dib: PFIBITMAP); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_Unload@4'{$ENDIF};

// --------------------------------------------------------------------------
// Header loading routines
// --------------------------------------------------------------------------
function FreeImage_HasPixels(dib: PFIBITMAP): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_HasPixels@4'{$ENDIF};

// --------------------------------------------------------------------------
// Load / Save routines -----------------------------------------------------
// --------------------------------------------------------------------------

function FreeImage_Load(fif: FREE_IMAGE_FORMAT; filename: PAnsiChar;
  flags: Integer = 0): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_Load@12'{$ENDIF};
function FreeImage_LoadU(fif: FREE_IMAGE_FORMAT; filename: PWideChar;
  flags: Integer = 0): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_LoadU@12'{$ENDIF};
function FreeImage_LoadFromHandle(fif: FREE_IMAGE_FORMAT; io: PFreeImageIO;
  handle: fi_handle; flags: Integer = 0): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_LoadFromHandle@16'{$ENDIF};
function FreeImage_Save(fif: FREE_IMAGE_FORMAT; dib: PFIBITMAP; filename: PAnsiChar;
  flags: Integer = 0): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_Save@16'{$ENDIF};
function FreeImage_SaveU(fif: FREE_IMAGE_FORMAT; dib: PFIBITMAP; filename: PWideChar;
  flags: Integer = 0): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SaveU@16'{$ENDIF};
function FreeImage_SaveToHandle(fif: FREE_IMAGE_FORMAT; dib: PFIBITMAP;
  io: PFreeImageIO; handle: fi_handle; flags: Integer = 0): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SaveToHandle@20'{$ENDIF};

// --------------------------------------------------------------------------
// Memory I/O stream routines -----------------------------------------------
// --------------------------------------------------------------------------

function FreeImage_OpenMemory(data: PByte = nil; size_in_bytes: DWORD = 0): PFIMEMORY; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_OpenMemory@8'{$ENDIF};
procedure FreeImage_CloseMemory(stream: PFIMEMORY); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_CloseMemory@4'{$ENDIF};
function FreeImage_LoadFromMemory(fif: FREE_IMAGE_FORMAT; stream: PFIMEMORY;
  flags: Integer = 0): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_LoadFromMemory@12'{$ENDIF};
function FreeImage_SaveToMemory(fif: FREE_IMAGE_FORMAT; dib: PFIBITMAP;
  stream: PFIMEMORY; flags: Integer = 0): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SaveToMemory@16'{$ENDIF};
function FreeImage_TellMemory(stream: PFIMEMORY): LongInt; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_TellMemory@4'{$ENDIF};
function FreeImage_SeekMemory(stream: PFIMEMORY; offset: LongInt;
  origin: Integer): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SeekMemory@12'{$ENDIF};
function FreeImage_AcquireMemory(stream: PFIMEMORY; var data: PByte;
  var size_in_bytes: DWORD): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_AcquireMemory@12'{$ENDIF};
function FreeImage_ReadMemory(buffer: Pointer; size, count: Cardinal;
  stream: PFIMEMORY): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ReadMemory@16'{$ENDIF};
function FreeImage_WriteMemory(buffer: Pointer; size, count: Cardinal;
  stream: PFIMEMORY): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_WriteMemory@16'{$ENDIF};
function FreeImage_LoadMultiBitmapFromMemory(fif: FREE_IMAGE_FORMAT; stream: PFIMEMORY;
  flags: Integer = 0): PFIMULTIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_LoadMultiBitmapFromMemory@12'{$ENDIF};
function FreeImage_SaveMultiBitmapToMemory(fif: FREE_IMAGE_FORMAT; bitmap: PFIMULTIBITMAP;
  stream: PFIMEMORY; flags: Integer): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SaveMultiBitmapToMemory@16'{$ENDIF};

// --------------------------------------------------------------------------
// Plugin Interface ---------------------------------------------------------
// --------------------------------------------------------------------------

function FreeImage_RegisterLocalPlugin(proc_address: FI_InitProc; format: PAnsiChar = nil;
  description: PAnsiChar = nil; extension: PAnsiChar = nil;
  regexpr: PAnsiChar = nil): FREE_IMAGE_FORMAT; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_RegisterLocalPlugin@20'{$ENDIF};
function FreeImage_RegisterExternalPlugin(path: PAnsiChar; format: PAnsiChar = nil;
  description: PAnsiChar = nil; extension: PAnsiChar = nil;
  regexpr: PAnsiChar = nil): FREE_IMAGE_FORMAT; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_RegisterExternalPlugin@20'{$ENDIF};
function FreeImage_GetFIFCount: Integer; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetFIFCount@0'{$ENDIF};
procedure FreeImage_SetPluginEnabled(fif: FREE_IMAGE_FORMAT; enable: Boolean); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetPluginEnabled@8'{$ENDIF};
function FreeImage_IsPluginEnabled(fif: FREE_IMAGE_FORMAT): Integer; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_IsPluginEnabled@4'{$ENDIF};
function FreeImage_GetFIFFromFormat(format: PAnsiChar): FREE_IMAGE_FORMAT; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetFIFFromFormat@4'{$ENDIF};
function FreeImage_GetFIFFromMime(mime: PAnsiChar): FREE_IMAGE_FORMAT; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetFIFFromMime@4'{$ENDIF};
function FreeImage_GetFormatFromFIF(fif: FREE_IMAGE_FORMAT): PAnsiChar; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetFormatFromFIF@4'{$ENDIF};
function FreeImage_GetFIFExtensionList(fif: FREE_IMAGE_FORMAT): PAnsiChar; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetFIFExtensionList@4'{$ENDIF};
function FreeImage_GetFIFDescription(fif: FREE_IMAGE_FORMAT): PAnsiChar; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetFIFDescription@4'{$ENDIF};
function FreeImage_GetFIFRegExpr(fif: FREE_IMAGE_FORMAT): PAnsiChar; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetFIFRegExpr@4'{$ENDIF};
function FreeImage_GetFIFMimeType(fif: FREE_IMAGE_FORMAT): PAnsiChar; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetFIFMimeType@4'{$ENDIF};
function FreeImage_GetFIFFromFilename(filename: PAnsiChar): FREE_IMAGE_FORMAT; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetFIFFromFilename@4'{$ENDIF};
function FreeImage_GetFIFFromFilenameU(filename: PWideChar): FREE_IMAGE_FORMAT; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetFIFFromFilenameU@4'{$ENDIF};
function FreeImage_FIFSupportsReading(fif: FREE_IMAGE_FORMAT): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_FIFSupportsReading@4'{$ENDIF};
function FreeImage_FIFSupportsWriting(fif: FREE_IMAGE_FORMAT): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_FIFSupportsWriting@4'{$ENDIF};
function FreeImage_FIFSupportsExportBPP(fif: FREE_IMAGE_FORMAT;
  bpp: Integer): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_FIFSupportsExportBPP@8'{$ENDIF};
function FreeImage_FIFSupportsExportType(fif: FREE_IMAGE_FORMAT;
  atype: FREE_IMAGE_TYPE): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_FIFSupportsExportType@8'{$ENDIF};
function FreeImage_FIFSupportsICCProfiles(fif: FREE_IMAGE_FORMAT): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_FIFSupportsICCProfiles@4'{$ENDIF};
function FreeImage_FIFSupportsNoPixels(fif: FREE_IMAGE_FORMAT): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_FIFSupportsNoPixels@4'{$ENDIF};

// --------------------------------------------------------------------------
// Multipaging interface ----------------------------------------------------
// --------------------------------------------------------------------------

function FreeImage_OpenMultiBitmap(fif: FREE_IMAGE_FORMAT; filename: PAnsiChar;
  create_new, read_only: Boolean; keep_cache_in_memory: Boolean = False;
  flags: Integer = 0): PFIMULTIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_OpenMultiBitmap@24'{$ENDIF};
function FreeImage_OpenMultiBitmapFromHandle(fif: FREE_IMAGE_FORMAT; io: PFreeImageIO;
  handle: fi_handle; flags: Integer = 0): PFIMULTIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_OpenMultiBitmapFromHandle@16'{$ENDIF};
function FreeImage_SaveMultiBitmapToHandle(fif: FREE_IMAGE_FORMAT; bitmap: PFIMULTIBITMAP;
  io: PFreeImageIO; handle: fi_handle; flags: Integer = 0): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SaveMultiBitmapToHandle@20'{$ENDIF};
function FreeImage_CloseMultiBitmap(bitmap: PFIMULTIBITMAP;
  flags: Integer = 0): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_CloseMultiBitmap@8'{$ENDIF};
function FreeImage_GetPageCount(bitmap: PFIMULTIBITMAP): Integer; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetPageCount@4'{$ENDIF};
procedure FreeImage_AppendPage(bitmap: PFIMULTIBITMAP; data: PFIBITMAP); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_AppendPage@8'{$ENDIF};
procedure FreeImage_InsertPage(bitmap: PFIMULTIBITMAP; page: Integer;
  data: PFIBITMAP); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_InsertPage@12'{$ENDIF};
procedure FreeImage_DeletePage(bitmap: PFIMULTIBITMAP; page: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_DeletePage@8'{$ENDIF};
function FreeImage_LockPage(bitmap: PFIMULTIBITMAP; page: Integer): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_LockPage@8'{$ENDIF};
procedure FreeImage_UnlockPage(bitmap: PFIMULTIBITMAP; data: PFIBITMAP;
  changed: Boolean); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_UnlockPage@12'{$ENDIF};
function FreeImage_MovePage(bitmap: PFIMULTIBITMAP; target, source: Integer): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_MovePage@12'{$ENDIF};
function FreeImage_GetLockedPageNumbers(bitmap: PFIMULTIBITMAP; var pages: Integer;
  var count: Integer): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetLockedPageNumbers@12'{$ENDIF};

// --------------------------------------------------------------------------
// Filetype request routines ------------------------------------------------
// --------------------------------------------------------------------------

function FreeImage_GetFileType(filename: PAnsiChar;
  size: Integer = 0): FREE_IMAGE_FORMAT; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetFileType@8'{$ENDIF};
function FreeImage_GetFileTypeU(filename: PWideChar;
  size: Integer = 0): FREE_IMAGE_FORMAT; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetFileTypeU@8'{$ENDIF};
function FreeImage_GetFileTypeFromHandle(io: PFreeImageIO; handle: FI_Handle;
  size: Integer = 0): FREE_IMAGE_FORMAT; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetFileTypeFromHandle@12'{$ENDIF};
function FreeImage_GetFileTypeFromMemory(stream: PFIMEMORY;
  size: Integer = 0): FREE_IMAGE_FORMAT; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetFileTypeFromMemory@8'{$ENDIF};

// --------------------------------------------------------------------------
// ImageType request routine ------------------------------------------------
// --------------------------------------------------------------------------

function FreeImage_GetImageType(dib: PFIBITMAP): FREE_IMAGE_TYPE; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetImageType@4'{$ENDIF};

// --------------------------------------------------------------------------
// FreeImage helper routines ------------------------------------------------
// --------------------------------------------------------------------------

function FreeImage_IsLittleEndian: Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_IsLittleEndian@0'{$ENDIF};
function FreeImage_LookupX11Color(szColor: PAnsiChar; var nRed, nGreen, nBlue: Byte): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_LookupX11Color@16'{$ENDIF};
function FreeImage_LookupSVGColor(szColor: PAnsiChar; var nRed, nGreen, nBlue: Byte): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_LookupSVGColor@16'{$ENDIF};

// --------------------------------------------------------------------------
// Pixels access routines ---------------------------------------------------
// --------------------------------------------------------------------------

function FreeImage_GetBits(dib: PFIBITMAP): PByte; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetBits@4'{$ENDIF};
function FreeImage_GetScanLine(dib: PFIBITMAP; scanline: Integer): PByte; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetScanLine@8'{$ENDIF};

function FreeImage_GetPixelIndex(dib: PFIBITMAP; x, y: Cardinal; var value: Byte): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetPixelIndex@16'{$ENDIF};
function FreeImage_GetPixelColor(dib: PFIBITMAP; x, y: Cardinal; var value: RGBQUAD): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetPixelColor@16'{$ENDIF};
function FreeImage_SetPixelIndex(dib: PFIBITMAP; x, y: Cardinal; var value: Byte): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetPixelIndex@16'{$ENDIF};
function FreeImage_SetPixelColor(dib: PFIBITMAP; x, y: Cardinal; var value: RGBQUAD): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetPixelColor@16'{$ENDIF};

// --------------------------------------------------------------------------
// DIB info routines --------------------------------------------------------
// --------------------------------------------------------------------------

function FreeImage_GetColorsUsed(dib: PFIBITMAP): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetColorsUsed@4'{$ENDIF};
function FreeImage_GetBPP(dib: PFIBITMAP): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetBPP@4'{$ENDIF};
function FreeImage_GetWidth(dib: PFIBITMAP): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetWidth@4'{$ENDIF};
function FreeImage_GetHeight(dib: PFIBITMAP): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetHeight@4'{$ENDIF};
function FreeImage_GetLine(dib: PFIBITMAP): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetLine@4'{$ENDIF};
function FreeImage_GetPitch(dib: PFIBITMAP): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetPitch@4'{$ENDIF};
function FreeImage_GetDIBSize(dib: PFIBITMAP): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetDIBSize@4'{$ENDIF};
function FreeImage_GetPalette(dib: PFIBITMAP): PRGBQuad; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetPalette@4'{$ENDIF};

function FreeImage_GetDotsPerMeterX(dib: PFIBITMAP): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetDotsPerMeterX@4'{$ENDIF};
function FreeImage_GetDotsPerMeterY(dib: PFIBITMAP): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetDotsPerMeterY@4'{$ENDIF};
procedure FreeImage_SetDotsPerMeterX(dib: PFIBITMAP; res: Cardinal); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetDotsPerMeterX@8'{$ENDIF};
procedure FreeImage_SetDotsPerMeterY(dib: PFIBITMAP; res: Cardinal); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetDotsPerMeterY@8'{$ENDIF};

function FreeImage_GetInfoHeader(dib: PFIBITMAP): PBITMAPINFOHEADER; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetInfoHeader@4'{$ENDIF};
function FreeImage_GetInfo(dib: PFIBITMAP): PBITMAPINFO; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetInfo@4'{$ENDIF};
function FreeImage_GetColorType(dib: PFIBITMAP): FREE_IMAGE_COLOR_TYPE; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetColorType@4'{$ENDIF};

function FreeImage_GetRedMask(dib: PFIBITMAP): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetRedMask@4'{$ENDIF};
function FreeImage_GetGreenMask(dib: PFIBITMAP): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetGreenMask@4'{$ENDIF};
function FreeImage_GetBlueMask(dib: PFIBITMAP): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetBlueMask@4'{$ENDIF};

function FreeImage_GetTransparencyCount(dib: PFIBITMAP): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetTransparencyCount@4'{$ENDIF};
function FreeImage_GetTransparencyTable(dib: PFIBITMAP): PByte; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetTransparencyTable@4'{$ENDIF};
procedure FreeImage_SetTransparent(dib: PFIBITMAP; enabled: Boolean); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetTransparent@8'{$ENDIF};
procedure FreeImage_SetTransparencyTable(dib: PFIBITMAP; table: PByte;
  count: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetTransparencyTable@12'{$ENDIF};
function FreeImage_IsTransparent(dib: PFIBITMAP): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_IsTransparent@4'{$ENDIF};
procedure FreeImage_SetTransparentIndex(dib: PFIBITMAP; index: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetTransparentIndex@8'{$ENDIF};
function FreeImage_GetTransparentIndex(dib: PFIBITMAP): Integer; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetTransparentIndex@4'{$ENDIF};

function FreeImage_HasBackgroundColor(dib: PFIBITMAP): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_HasBackgroundColor@4'{$ENDIF};
function FreeImage_GetBackgroundColor(dib: PFIBITMAP; var bkcolor: RGBQUAD): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetBackgroundColor@8'{$ENDIF};
function FreeImage_SetBackgroundColor(dib: PFIBITMAP; bkcolor: PRGBQuad): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetBackgroundColor@8'{$ENDIF};

// --------------------------------------------------------------------------
// ICC profile routines -----------------------------------------------------
// --------------------------------------------------------------------------

function FreeImage_GetICCProfile(dib: PFIBITMAP): PFIICCPROFILE; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetICCProfile@4'{$ENDIF};
function FreeImage_CreateICCProfile(dib: PFIBITMAP; data: Pointer;
  size: LongInt): PFIICCPROFILE; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name 'FreeImage_CreateICCProfile@12'{$ENDIF};
procedure FreeImage_DestroyICCProfile(dib: PFIBITMAP); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name 'FreeImage_DestroyICCProfile@4'{$ENDIF};

// --------------------------------------------------------------------------
// Line conversion routines -------------------------------------------------
// --------------------------------------------------------------------------

procedure FreeImage_ConvertLine1To4(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine1To4@12'{$ENDIF};
procedure FreeImage_ConvertLine8To4(target, source: PByte; width_in_pixels: Integer;
  palette: PRGBQuad);  stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine8To4@16'{$ENDIF};
procedure FreeImage_ConvertLine16To4_555(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine16To4_555@12'{$ENDIF};
procedure FreeImage_ConvertLine16To4_565(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine16To4_565@12'{$ENDIF};
procedure FreeImage_ConvertLine24To4(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine24To4@12'{$ENDIF};
procedure FreeImage_ConvertLine32To4(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine32To4@12'{$ENDIF};

procedure FreeImage_ConvertLine1To8(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine1To8@12'{$ENDIF};
procedure FreeImage_ConvertLine4To8(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine4To8@12'{$ENDIF};
procedure FreeImage_ConvertLine16To8_555(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine16To8_555@12'{$ENDIF};
procedure FreeImage_ConvertLine16To8_565(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine16To8_565@12'{$ENDIF};
procedure FreeImage_ConvertLine24To8(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine24To8@12'{$ENDIF};
procedure FreeImage_ConvertLine32To8(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine32To8@12'{$ENDIF};

procedure FreeImage_ConvertLine1To16_555(target, source: PByte; width_in_pixels: Integer;
  palette: PRGBQuad); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine1To16_555@16'{$ENDIF};
procedure FreeImage_ConvertLine4To16_555(target, source: PByte; width_in_pixels: Integer;
  palette: PRGBQuad); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine4To16_555@16'{$ENDIF};
procedure FreeImage_ConvertLine8To16_555(target, source: PByte; width_in_pixels: Integer;
  palette: PRGBQuad); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine8To16_555@16'{$ENDIF};
procedure FreeImage_ConvertLine16_565_To16_555(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine16_565_To16_555@12'{$ENDIF};
procedure FreeImage_ConvertLine24To16_555(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine24To16_555@12'{$ENDIF};
procedure FreeImage_ConvertLine32To16_555(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine32To16_555@12'{$ENDIF};

procedure FreeImage_ConvertLine1To16_565(target, source: PByte; width_in_pixels: Integer;
  palette: PRGBQuad); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine1To16_565@16'{$ENDIF};
procedure FreeImage_ConvertLine4To16_565(target, source: PByte; width_in_pixels: Integer;
  palette: PRGBQuad); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine4To16_565@16'{$ENDIF};
procedure FreeImage_ConvertLine8To16_565(target, source: PByte; width_in_pixels: Integer;
  palette: PRGBQuad); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine8To16_565@16'{$ENDIF};
procedure FreeImage_ConvertLine16_555_To16_565(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine16_555_To16_565@12'{$ENDIF};
procedure FreeImage_ConvertLine24To16_565(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine24To16_565@12'{$ENDIF};
procedure FreeImage_ConvertLine32To16_565(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine32To16_565@12'{$ENDIF};

procedure FreeImage_ConvertLine1To24(target, source: PByte; width_in_pixels: Integer;
  palette: PRGBQuad); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine1To24@16'{$ENDIF};
procedure FreeImage_ConvertLine4To24(target, source: PByte; width_in_pixels: Integer;
  palette: PRGBQuad); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine4To24@16'{$ENDIF};
procedure FreeImage_ConvertLine8To24(target, source: PByte; width_in_pixels: Integer;
  palette: PRGBQuad); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine8To24@16'{$ENDIF};
procedure FreeImage_ConvertLine16To24_555(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine16To24_555@12'{$ENDIF};
procedure FreeImage_ConvertLine16To24_565(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine16To24_565@12'{$ENDIF};
procedure FreeImage_ConvertLine32To24(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine32To24@12'{$ENDIF};

procedure FreeImage_ConvertLine1To32(target, source: PByte; width_in_pixels: Integer;
  palette: PRGBQuad); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine1To32@16'{$ENDIF};
procedure FreeImage_ConvertLine4To32(target, source: PByte; width_in_pixels: Integer;
  palette: PRGBQuad); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine4To32@16'{$ENDIF};
procedure FreeImage_ConvertLine8To32(target, source: PByte; width_in_pixels: Integer;
  palette: PRGBQuad); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine8To32@16'{$ENDIF};
procedure FreeImage_ConvertLine16To32_555(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine16To32_555@12'{$ENDIF};
procedure FreeImage_ConvertLine16To32_565(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine16To32_565@12'{$ENDIF};
procedure FreeImage_ConvertLine24To32(target, source: PByte; width_in_pixels: Integer); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertLine24To32@12'{$ENDIF};

// --------------------------------------------------------------------------
// Smart conversion routines ------------------------------------------------
// --------------------------------------------------------------------------

function FreeImage_ConvertTo4Bits(dib: PFIBITMAP): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertTo4Bits@4'{$ENDIF};
function FreeImage_ConvertTo8Bits(dib: PFIBITMAP): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertTo8Bits@4'{$ENDIF};
function FreeImage_ConvertToGreyscale(dib: PFIBITMAP): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertToGreyscale@4'{$ENDIF};
function FreeImage_ConvertTo16Bits555(dib: PFIBITMAP): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertTo16Bits555@4'{$ENDIF};
function FreeImage_ConvertTo16Bits565(dib: PFIBITMAP): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertTo16Bits565@4'{$ENDIF};
function FreeImage_ConvertTo24Bits(dib: PFIBITMAP): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertTo24Bits@4'{$ENDIF};
function FreeImage_ConvertTo32Bits(dib: PFIBITMAP): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertTo32Bits@4'{$ENDIF};
function FreeImage_ColorQuantize(dib: PFIBITMAP; quantize: FREE_IMAGE_QUANTIZE): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ColorQuantize@8'{$ENDIF};
function FreeImage_ColorQuantizeEx(dib: PFIBITMAP; quantize: FREE_IMAGE_QUANTIZE = FIQ_WUQUANT;
  PaletteSize: Integer = 256; ReserveSize: Integer = 0;
  ReservePalette: PRGBQuad = nil): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ColorQuantizeEx@20'{$ENDIF};
function FreeImage_Threshold(dib: PFIBITMAP; T: Byte): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_Threshold@8'{$ENDIF};
function FreeImage_Dither(dib: PFIBITMAP; algorithm: FREE_IMAGE_DITHER): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_Dither@8'{$ENDIF};

function FreeImage_ConvertFromRawBits(bits: PByte; width, height, pitch: Integer;
  bpp, red_mask, green_mask, blue_mask: Cardinal; topdown: Boolean = False): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertFromRawBits@36'{$ENDIF};
procedure FreeImage_ConvertToRawBits(bits: PByte; dib: PFIBITMAP; pitch: Integer;
  bpp, red_mask, green_mask, blue_mask: Cardinal; topdown: Boolean = False); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertToRawBits@32'{$ENDIF};

function FreeImage_ConvertToFloat(dib: PFIBITMAP): PFIBITMAP;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertToFloat@4'{$ENDIF};
function FreeImage_ConvertToRGBF(dib: PFIBITMAP): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertToRGBF@4'{$ENDIF};

function FreeImage_ConvertToStandardType(src: PFIBITMAP;
  scale_linear: Boolean = True): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertToStandardType@8'{$ENDIF};
function FreeImage_ConvertToType(src: PFIBITMAP; dst_type: FREE_IMAGE_TYPE;
  scale_linear: Boolean = True): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ConvertToType@12'{$ENDIF};

// tone mapping operators
function FreeImage_ToneMapping(dib: PFIBITMAP; tmo: FREE_IMAGE_TMO;
  first_param: Double = 0; second_param: Double = 0): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ToneMapping@24'{$ENDIF};
function FreeImage_TmoDrago03(src: PFIBITMAP; gamma: Double = 2.2;
  exposure: Double = 0): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_TmoDrago03@20'{$ENDIF};
function FreeImage_TmoReinhard05(src: PFIBITMAP; intensity: Double = 0;
  contrast: Double = 0): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_TmoReinhard05@20'{$ENDIF};
function FreeImage_TmoReinhard05Ex(src: PFIBITMAP; intensity: Double = 0;
  contrast: Double = 0; adaptation: Double = 1; color_correction: Double = 0): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_TmoReinhard05Ex@36'{$ENDIF};

function FreeImage_TmoFattal02(src: PFIBITMAP; color_saturation: Double = 0.5;
  attenuation: Double = 0.85): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_TmoFattal02@20'{$ENDIF};

// --------------------------------------------------------------------------
// ZLib interface -----------------------------------------------------------
// --------------------------------------------------------------------------

function FreeImage_ZLibCompress(target: PByte; target_size: DWORD; source: PByte; source_size: DWORD): DWORD; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ZLibCompress@16'{$ENDIF};
function FreeImage_ZLibUncompress(target: PByte; target_size: DWORD; source: PByte; source_size: DWORD): DWORD; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ZLibUncompress@16'{$ENDIF};
function FreeImage_ZLibGZip(target: PByte; target_size: DWORD; source: PByte; source_size: DWORD): DWORD; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ZLibGZip@16'{$ENDIF};
function FreeImage_ZLibGUnzip(target: PByte; target_size: DWORD; source: PByte; source_size: DWORD): DWORD; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ZLibGUnzip@16'{$ENDIF};
function FreeImage_ZLibCRC32(crc: DWORD; source: PByte; source_size: DWORD): DWORD; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ZLibCRC32@12'{$ENDIF};

// --------------------------------------------------------------------------
// Metadata routines --------------------------------------------------------
// --------------------------------------------------------------------------

// tag creation / destruction
function FreeImage_CreateTag: PFITAG; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_CreateTag@0'{$ENDIF};
procedure FreeImage_DeleteTag(tag: PFITAG); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_DeleteTag@4'{$ENDIF};
function FreeImage_CloneTag(tag: PFITAG): PFITAG; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_CloneTag@4'{$ENDIF};

// tag getters and setters
function FreeImage_GetTagKey(tag: PFITAG): PAnsiChar; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetTagKey@4'{$ENDIF};
function FreeImage_GetTagDescription(tag: PFITAG): PAnsiChar; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetTagDescription@4'{$ENDIF};
function FreeImage_GetTagID(tag: PFITAG): Word; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetTagID@4'{$ENDIF};
function FreeImage_GetTagType(tag: PFITAG): FREE_IMAGE_MDTYPE; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetTagType@4'{$ENDIF};
function FreeImage_GetTagCount(tag: PFITAG): DWORD; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetTagCount@4'{$ENDIF};
function FreeImage_GetTagLength(tag: PFITAG): DWORD; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetTagLength@4'{$ENDIF};
function FreeImage_GetTagValue(tag: PFITAG): Pointer; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetTagValue@4'{$ENDIF};

function FreeImage_SetTagKey(tag: PFITAG; key: PAnsiChar): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetTagKey@8'{$ENDIF};
function FreeImage_SetTagDescription(tag: PFITAG; description: PAnsiChar): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetTagDescription@8'{$ENDIF};
function FreeImage_SetTagID(tag: PFITAG; id: Word): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetTagID@8'{$ENDIF};
function FreeImage_SetTagType(tag: PFITAG; atype: FREE_IMAGE_MDTYPE): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetTagType@8'{$ENDIF};
function FreeImage_SetTagCount(tag: PFITAG; count: DWORD): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetTagCount@8'{$ENDIF};
function FreeImage_SetTagLength(tag: PFITAG; length: DWORD): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetTagLength@8'{$ENDIF};
function FreeImage_SetTagValue(tag: PFITAG; value: Pointer): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetTagValue@8'{$ENDIF};

// iterator
function FreeImage_FindFirstMetadata(model: FREE_IMAGE_MDMODEL; dib: PFIBITMAP;
  var tag: PFITAG): PFIMETADATA; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_FindFirstMetadata@12'{$ENDIF};
function FreeImage_FindNextMetadata(mdhandle: PFIMETADATA; var tag: PFITAG): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_FindNextMetadata@8'{$ENDIF};
procedure FreeImage_FindCloseMetadata(mdhandle: PFIMETADATA); stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_FindCloseMetadata@4'{$ENDIF};

// metadata setter and getter
function FreeImage_SetMetadata(model: FREE_IMAGE_MDMODEL; dib: PFIBITMAP;
  key: PAnsiChar; tag: PFITAG): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetMetadata@16'{$ENDIF};
function FreeImage_GetMetaData(model: FREE_IMAGE_MDMODEL; dib: PFIBITMAP;
  key: PAnsiChar; var tag: PFITAG): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetMetadata@16'{$ENDIF};

// helpers
function FreeImage_GetMetadataCount(model: FREE_IMAGE_MDMODEL; dib: PFIBITMAP): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetMetadataCount@8'{$ENDIF};
function FreeImage_CloneMetadata(dst, src: PFIBITMAP): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_CloneMetadata@8'{$ENDIF};

// tag to C string conversion
function FreeImage_TagToString(model: FREE_IMAGE_MDMODEL; tag: PFITAG;
  Make: PAnsiChar = nil): PAnsiChar; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_TagToString@12'{$ENDIF};

// --------------------------------------------------------------------------
// Image manipulation toolkit -----------------------------------------------
// --------------------------------------------------------------------------

// rotation and flipping
function FreeImage_RotateClassic(dib: PFIBITMAP; angle: Double): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_RotateClassic@12'{$ENDIF};
function FreeImage_Rotate(dib: PFIBITMAP; angle: Double; bkcolor: Pointer = nil): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_Rotate@16'{$ENDIF};
function FreeImage_RotateEx(dib: PFIBITMAP; angle, x_shift, y_shift, x_origin, y_origin: Double;
  use_mask: Boolean): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_RotateEx@48'{$ENDIF};
function FreeImage_FlipHorizontal(dib: PFIBITMAP): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_FlipHorizontal@4'{$ENDIF};
function FreeImage_FlipVertical(dib: PFIBITMAP): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_FlipVertical@4'{$ENDIF};
function FreeImage_JPEGTransform(src_file, dst_file: PAnsiChar; operation: FREE_IMAGE_JPEG_OPERATION;
  perfect: Boolean = False): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_JPEGTransform@16'{$ENDIF};
function FreeImage_JPEGTransformU(src_file, dst_file: PWideChar; operation: FREE_IMAGE_JPEG_OPERATION;
  perfect: Boolean = False): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_JPEGTransformU@16'{$ENDIF};

// upsampling / downsampling
function FreeImage_Rescale(dib: PFIBITMAP; dst_width, dst_height: Integer; filter: FREE_IMAGE_FILTER): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_Rescale@16'{$ENDIF};
function FreeImage_MakeThumbnail(dib: PFIBITMAP; max_pixel_size: Integer; convert: Boolean = True): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_MakeThumbnail@12'{$ENDIF};

// color manipulation routines (point operations)
function FreeImage_AdjustCurve(dib: PFIBITMAP; LUT: PByte;
  channel: FREE_IMAGE_COLOR_CHANNEL): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_AdjustCurve@12'{$ENDIF};
function FreeImage_AdjustGamma(dib: PFIBITMAP; gamma: Double): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_AdjustGamma@12'{$ENDIF};
function FreeImage_AdjustBrightness(dib: PFIBITMAP; percentage: Double): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_AdjustBrightness@12'{$ENDIF};
function FreeImage_AdjustContrast(dib: PFIBITMAP; percentage: Double): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_AdjustContrast@12'{$ENDIF};
function FreeImage_Invert(dib: PFIBITMAP): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_Invert@4'{$ENDIF};
function FreeImage_GetHistogram(dib: PFIBITMAP; histo: PDWORD;
  channel: FREE_IMAGE_COLOR_CHANNEL = FICC_BLACK): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetHistogram@12'{$ENDIF};
function FreeImage_GetAdjustColorsLookupTable(LUT: PByte; brightness, contrast, gamma: Double;
  invert: Boolean): Integer; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetAdjustColorsLookupTable@32'{$ENDIF};
function FreeImage_AdjustColors(dib: PFIBITMAP; brightness, contrast, gamma: Double;
  invert: Boolean = False): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_AdjustColors@32'{$ENDIF};
function FreeImage_ApplyColorMapping(dib: PFIBITMAP; srccolors, dstcolors: PRGBQuad;
  count: Cardinal; ignore_alpha, swap: Boolean): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ApplyColorMapping@24'{$ENDIF};
function FreeImage_SwapColors(dib: PFIBITMAP; color_a, color_b: PRGBQuad;
  ignore_alpha: Boolean): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SwapColors@16'{$ENDIF};
function FreeImage_ApplyPaletteIndexMapping(dib: PFIBITMAP; srcindices, dstindices: PByte;
  count: Cardinal; swap: Boolean): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_ApplyPaletteIndexMapping@20'{$ENDIF};
function FreeImage_SwapPaletteIndices(dib: PFIBITMAP; index_a, index_b: PByte): Cardinal; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SwapPaletteIndices@12'{$ENDIF};

// channel processing routines
function FreeImage_GetChannel(dib: PFIBITMAP; channel: FREE_IMAGE_COLOR_CHANNEL): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetChannel@8'{$ENDIF};
function FreeImage_SetChannel(dst, src: PFIBITMAP; channel: FREE_IMAGE_COLOR_CHANNEL): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetChannel@12'{$ENDIF};
function FreeImage_GetComplexChannel(src: PFIBITMAP; channel: FREE_IMAGE_COLOR_CHANNEL): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_GetComplexChannel@8'{$ENDIF};
function FreeImage_SetComplexChannel(dst, src: PFIBITMAP; channel: FREE_IMAGE_COLOR_CHANNEL): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_SetComplexChannel@12'{$ENDIF};

// copy / paste / composite routines

function FreeImage_Copy(dib: PFIBITMAP; left, top, right, bottom: Integer): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_Copy@20'{$ENDIF};
function FreeImage_Paste(dst, src: PFIBITMAP; left, top, alpha: Integer): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_Paste@20'{$ENDIF};
function FreeImage_Composite(fg: PFIBITMAP; useFileBkg: Boolean = False;
  appBkColor: PRGBQuad = nil; bg: PFIBITMAP = nil): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_Composite@16'{$ENDIF};
function FreeImage_JPEGCrop(src_file, dst_file: PAnsiChar;
  left, top, right, bottom: Integer): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_JPEGCrop@24'{$ENDIF};
function FreeImage_JPEGCropU(src_file, dst_file: PWideChar;
  left, top, right, bottom: Integer): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_JPEGCropU@24'{$ENDIF};
function FreeImage_PreMultiplyWithAlpha(dib: PFIBITMAP): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_PreMultiplyWithAlpha@4'{$ENDIF};

// background filling routines
function FreeImage_FillBackground(dib: PFIBITMAP; color: Pointer;
  options: Integer = 0): Boolean; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_FillBackground@12'{$ENDIF};
function FreeImage_EnlargeCanvas(src: PFIBITMAP; left, top, right, bottom: Integer;
  color: Pointer; options: Integer = 0): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_EnlargeCanvas@28'{$ENDIF};
function FreeImage_AllocateEx(width, height, bpp: Integer; color: PRGBQuad;
  options: Integer = 0; palette: PRGBQuad = nil; red_mask: Cardinal = 0;
  green_mask: Cardinal = 0; blue_mask: Cardinal = 0): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_AllocateEx@36'{$ENDIF};
function FreeImage_AllocateExT(atype: FREE_IMAGE_TYPE; width, height, bpp: Integer;
  color: Pointer; options: Integer = 0; palette: PRGBQuad = nil; red_mask: Cardinal = 0;
  green_mask: Cardinal = 0; blue_mask: Cardinal = 0): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_AllocateExT@40'{$ENDIF};

// miscellaneous algorithms
function FreeImage_MultigridPoissonSolver(Laplacian: PFIBITMAP;
  ncycle: Integer = 3): PFIBITMAP; stdcall;
  external FIDLL {$IFDEF MSWINDOWS}name '_FreeImage_MultigridPoissonSolver@8'{$ENDIF};


implementation

{$IFNDEF DELPHI6}
uses SysUtils;

//we provide a wrapper since we haven't varargs in older versions of Delphi
procedure __FreeImage_OutputMessageProc;
  external FIDLL name 'FreeImage_OutputMessageProc';
procedure FreeImage_OutputMessageProc(fif: Integer; fmt: PAnsiChar; args: array of const);
  function ArrayToBuffer(Args: array of const;
    var Argv: Pointer; Buffer: Pointer; Size: Cardinal): Integer;
  var
    i: Integer;
    temp: AnsiString;
    parg: Pointer;
    psrc, pbuf: PAnsiChar;
    len: Cardinal;
  begin
    Result := High(Args) + 1;
    if Result = 0 then
      Exit;
    //array of pointers to push on stack
    GetMem(Argv, Result * SizeOf(Pointer));
    //pointer to current string in buffer
    pbuf := Buffer;
    //pointer to current arg
    parg := Argv;
    //for each const...
    for i := 0 to Result - 1 do begin
      case Args[i].VType of
        vtInteger: begin
          //integer
          psrc := nil;
          len := 0;
          Integer(parg^) := Args[i].VInteger;
        end;
        vtString: begin
          //short string
          psrc := PAnsiChar(Cardinal(Args[i].VString) + SizeOf(Byte));
          len := PByte(Args[i].VString)^;
          PAnsiChar(parg^) := pbuf;
        end;
        vtPChar: begin
          //NULL terminated MBCS string
          psrc := nil;
          len := 0;
          PAnsiChar(parg^) := Args[i].VPChar;
        end;
        vtPWideChar: begin
          //NULL terminated Unicode string
          temp := AnsiString(Args[i].VPWideChar);
          psrc := PAnsiChar(temp);
          len := Length(temp);
          PAnsiChar(parg^) := pbuf;
        end;
        vtAnsiString: begin
          //ANSI string
          psrc := PAnsiChar(Args[i].VAnsiString);
          len := StrLen(psrc);
          PAnsiChar(parg^) := pbuf;
        end;
        vtWideString: begin
          //Wide string (OLE)
          temp := AnsiString(PWideChar(Args[i].VWideString));
          psrc := PAnsiChar(temp);
          len := Length(temp);
          PAnsiChar(parg^) := pbuf;
        end;
        else raise Exception.Create('Unsupported argument type');
      end;
      if (psrc <> nil) and (len <> 0) then begin
        //enough space to hold string?
        if Size < (len + 1) then
          raise Exception.Create('Buffer overflow');
        //copy string
        Move(psrc^, pbuf^, len);
        //NULL terminator
        PAnsiChar(Cardinal(pbuf) + len)^ := #0;
        //shift pointer...
        Inc(pbuf, len + 1);
        //...and decrease space left
        Dec(Size, len + 1);
      end;
      Cardinal(parg) := Cardinal(parg) + SizeOf(Pointer);
    end;
  end;

  procedure DoVarargsCall(fif: Integer; fmt: PAnsiChar; Argv: Pointer; Argc: Integer);
  {
  fif     -> EAX
  fmt     -> EDX
  Argv    -> ECX
  Argc    -> [EBP+$08]
  }
  asm
      PUSH    EAX                      //remember fif
      PUSH    ECX                      //make room for ESP backup

      MOV     DWORD PTR [EBP-$08], ESP //backup stack pointer

      MOV     EAX, DWORD PTR [EBP+$08] //store Argc

      TEST    EAX, EAX                 //Argc <= 0?
      JLE     @Call

    @Loop:
      PUSH    DWORD PTR [ECX+EAX*$04-$04] //push Argv in right to left order
      DEC     EAX
      JNZ     @Loop

    @Call:
      PUSH    EDX                      //push fmt
      PUSH    DWORD PTR [EBP-$04]      //push fif
      CALL    __FreeImage_OutputMessageProc

      MOV     ESP, DWORD PTR [EBP-$08] //restore stack pointer

      POP     ECX                      //clean stack
      POP     EAX
  end;
var
  Argc: Integer;
  Argv: Pointer;
  //buffer to hold strings - FreeImage allocates 512 bytes, we needn't more...
  Buffer: array[1..512] of Byte;
begin
  Argv := nil;
  //build array of pointers from array of const
  Argc := ArrayToBuffer(args, Argv, @Buffer, SizeOf(Buffer));
  try
    //mimic cdecl call with varargs
    DoVarargsCall(fif, fmt, Argv, Argc);
  finally
    //cleanup
    FreeMem(Argv);
  end;
end;
{$ENDIF}

end.
