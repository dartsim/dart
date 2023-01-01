/* A Bison parser, made by GNU Bison 2.3.  */

/* Skeleton implementation for Bison's Yacc-like parsers in C

Copyright (C) 1984, 1989, 1990, 2000, 2001, 2002, 2003, 2004, 2005, 2006
Free Software Foundation, Inc.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2, or (at your option)
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor,
Boston, MA 02110-1301, USA.  */

/* As a special exception, you may create a larger work that contains
part or all of the Bison parser skeleton and distribute that work
under terms of your choice, so long as that work isn't itself a
parser generator using the skeleton or a modified version thereof
as a parser skeleton.  Alternatively, if you modify or redistribute
the parser skeleton itself, you may (at your option) remove this
special exception, which will cause the skeleton and the resulting
Bison output files to be licensed under the GNU General Public
License without this special exception.

This special exception was added by the Free Software Foundation in
version 2.2 of Bison.  */

/* C LALR(1) parser skeleton written by Richard Stallman, by
simplifying the original so-called "semantic" parser.  */

/* All symbols defined below should begin with yy or YY, to avoid
infringing on user name space.  This should be done even for local
variables, as they might otherwise be expanded by user macros.
There are some unavoidable exceptions within include files to
define necessary library symbols; they are noted "INFRINGES ON
USER NAME SPACE" below.  */

/* Identify Bison output.  */
#define YYBISON 1

/* Bison version.  */
#define YYBISON_VERSION "2.3"

/* Skeleton name.  */
#define YYSKELETON_NAME "yacc.c"

/* Pure parsers.  */
#define YYPURE 0

/* Using locations.  */
#define YYLSP_NEEDED 0



/* Tokens.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
/* Put the tokens into the symbol table, so that GDB and other debuggers
know about them.  */
enum yytokentype {
    YY_FLOAT = 258,
    YY_INTEGER = 259,
    YY_STRING = 260,
    YY_PRIMITIVE = 261,
    YY_CHAIN = 262,
    YY_TRANSLATE = 263,
    YY_TELESCOPE = 264,
    YY_SCALE = 265,
    YY_ROTATE_QUAT = 266,
    YY_ROTATE_EXPMAP = 267,
    YY_ROTATE_EULER = 268,
    YY_ROTATE_CONS = 269,
    YY_MARKER = 270,
    YY_NODE = 271,
    YY_CONST = 272,
    YY_DOFS = 273,
    YY_MARKERS = 274,
    YY_MASS = 275
};
#endif
/* Tokens.  */
#define YY_FLOAT 258
#define YY_INTEGER 259
#define YY_STRING 260
#define YY_PRIMITIVE 261
#define YY_CHAIN 262
#define YY_TRANSLATE 263
#define YY_TELESCOPE 264
#define YY_SCALE 265
#define YY_ROTATE_QUAT 266
#define YY_ROTATE_EXPMAP 267
#define YY_ROTATE_EULER 268
#define YY_ROTATE_CONS 269
#define YY_MARKER 270
#define YY_NODE 271
#define YY_CONST 272
#define YY_DOFS 273
#define YY_MARKERS 274
#define YY_MASS 275




/* Copy the first part of user declarations.  */
//#line 1 "src/skel.y"


#if !defined(__APPLE__)
#include <malloc.h>
#endif
#include <fstream>
#include <iostream>
#include <assert.h>

#include "Transformation.h"
#include "Shape.h"
#include "Dof.h"
#include "ParserSkel.h"

#include <Eigen/Dense>
using namespace Eigen;

using namespace kinematics;

#include "Skeleton.h"
#include "BodyNode.h"
#include "Joint.h"
#include "Marker.h"
#include "Transformation.h"
#include "TrfmRotateEuler.h"
#include "TrfmRotateExpmap.h"
#include "TrfmRotateQuat.h"
#include "TrfmTranslate.h"
#include "ShapeBox.h"
#include "ShapeEllipsoid.h"
//#include "Capsule.h"
//#include "PrimMesh.h"

using namespace std;

//#define YYDEBUG 1

int yylex();
void yyerror(const char*);
void openFile(FILE*);
void closeFile();

Skeleton* gSkel;
char namePrefix[128];

BodyNode* cur_node;
BodyNode* stack[256];
int	stack_top;

struct dof_lookup1{
    char* name;
    Dof* dof;
} dof_lookup[256];
int num_dofs;

struct node_lookup1{
    char* name;
    BodyNode* node;
} node_lookup[256];
int num_nodes;

struct mass_lookup1{
    char *name;
    double mass;
} mass_lookup[256];
int num_masses;

void __finishSkel();
Dof* __createDOF( double );
Dof* __createDOF( char* );
Dof* __createDOF( char*, double, double, double );
void __recordMass(char*, double);
void __startNode( const char*, int );
void __endNode();
void __createTranslate( dofVec3 );
void __createTelescope( doubleVec3, Dof* );
void __createRotateEuler( Dof*, int );
void __createRotateEuler( double, int );
void __createRotateExpMap( dofVec3 );
void __createRotateQuat( dofVec4 );
void __setShape( doubleVec3, doubleVec3, Dof*);
void __setShape( doubleVec3, doubleVec3, Dof*, Shape*);
Shape* __setGeometry(const char*, const char*, doubleVec3);
Shape* __setGeometry(const char*, const char*);
Shape* __setGeometry(const char*, doubleVec3);
Shape* __setGeometry(const char*);
Shape* __setGeometry(doubleVec3);
void __createMarker( char*, doubleVec3, int, char* );



/* Enabling traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif

/* Enabling verbose error messages.  */
#ifdef YYERROR_VERBOSE
# undef YYERROR_VERBOSE
# define YYERROR_VERBOSE 1
#else
# define YYERROR_VERBOSE 0
#endif

/* Enabling the token table.  */
#ifndef YYTOKEN_TABLE
# define YYTOKEN_TABLE 0
#endif

#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
typedef union YYSTYPE
//#line 77 "src/skel.y"
{
    double	dValue;
    int iValue;
    char* sValue;
    dofVec3 v3DValue;
    dofVec4 v4DValue;
    doubleVec3 v3VValue;
    Transformation* tValue;
    Shape* pValue;
    Dof* dofValue;
}
/* Line 193 of yacc.c.  */
//#line 224 "src/SkelParser.cpp"
YYSTYPE;
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
# define YYSTYPE_IS_TRIVIAL 1
#endif



/* Copy the second part of user declarations.  */


/* Line 216 of yacc.c.  */
//#line 237 "src/SkelParser.cpp"

#ifdef short
# undef short
#endif

#ifdef YYTYPE_UINT8
typedef YYTYPE_UINT8 yytype_uint8;
#else
typedef unsigned char yytype_uint8;
#endif

#ifdef YYTYPE_INT8
typedef YYTYPE_INT8 yytype_int8;
#elif (defined __STDC__ || defined __C99__FUNC__ \
    || defined __cplusplus || defined _MSC_VER)
typedef signed char yytype_int8;
#else
typedef short int yytype_int8;
#endif

#ifdef YYTYPE_UINT16
typedef YYTYPE_UINT16 yytype_uint16;
#else
typedef unsigned short int yytype_uint16;
#endif

#ifdef YYTYPE_INT16
typedef YYTYPE_INT16 yytype_int16;
#else
typedef short int yytype_int16;
#endif

#ifndef YYSIZE_T
# ifdef __SIZE_TYPE__
#  define YYSIZE_T __SIZE_TYPE__
# elif defined size_t
#  define YYSIZE_T size_t
# elif ! defined YYSIZE_T && (defined __STDC__ || defined __C99__FUNC__ \
    || defined __cplusplus || defined _MSC_VER)
#  include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  define YYSIZE_T size_t
# else
#  define YYSIZE_T unsigned int
# endif
#endif

#define YYSIZE_MAXIMUM ((YYSIZE_T) -1)

#ifndef YY_
# if defined YYENABLE_NLS && YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> /* INFRINGES ON USER NAME SPACE */
#   define YY_(msgid) dgettext ("bison-runtime", msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(msgid) msgid
# endif
#endif

/* Suppress unused-variable warnings by "using" E.  */
#if ! defined lint || defined __GNUC__
# define YYUSE(e) ((void) (e))
#else
# define YYUSE(e) /* empty */
#endif

/* Identity function, used to suppress warnings about constant conditions.  */
#ifndef lint
# define YYID(n) (n)
#else
#if (defined __STDC__ || defined __C99__FUNC__ \
    || defined __cplusplus || defined _MSC_VER)
static int
YYID (int i)
#else
static int
YYID (i)
int i;
#endif
{
    return i;
}
#endif

#if ! defined yyoverflow || YYERROR_VERBOSE

/* The parser invokes alloca or malloc; define the necessary symbols.  */

# ifdef YYSTACK_USE_ALLOCA
#  if YYSTACK_USE_ALLOCA
#   ifdef __GNUC__
#    define YYSTACK_ALLOC __builtin_alloca
#   elif defined __BUILTIN_VA_ARG_INCR
#    include <alloca.h> /* INFRINGES ON USER NAME SPACE */
#   elif defined _AIX
#    define YYSTACK_ALLOC __alloca
#   elif defined _MSC_VER
#    include <malloc.h> /* INFRINGES ON USER NAME SPACE */
#    define alloca _alloca
#   else
#    define YYSTACK_ALLOC alloca
#    if ! defined _ALLOCA_H && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
    || defined __cplusplus || defined _MSC_VER)
#     include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#     ifndef _STDLIB_H
#      define _STDLIB_H 1
#     endif
#    endif
#   endif
#  endif
# endif

# ifdef YYSTACK_ALLOC
/* Pacify GCC's `empty if-body' warning.  */
#  define YYSTACK_FREE(Ptr) do { /* empty */; } while (YYID (0))
#  ifndef YYSTACK_ALLOC_MAXIMUM
/* The OS might guarantee only one guard page at the bottom of the stack,
and a page size can be as small as 4096 bytes.  So we cannot safely
invoke alloca (N) if N exceeds 4096.  Use a slightly smaller number
to allow for a few compiler-allocated temporary stack slots.  */
#   define YYSTACK_ALLOC_MAXIMUM 4032 /* reasonable circa 2006 */
#  endif
# else
#  define YYSTACK_ALLOC YYMALLOC
#  define YYSTACK_FREE YYFREE
#  ifndef YYSTACK_ALLOC_MAXIMUM
#   define YYSTACK_ALLOC_MAXIMUM YYSIZE_MAXIMUM
#  endif
#  if (defined __cplusplus && ! defined _STDLIB_H \
    && ! ((defined YYMALLOC || defined malloc) \
    && (defined YYFREE || defined free)))
#   include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#   ifndef _STDLIB_H
#    define _STDLIB_H 1
#   endif
#  endif
#  ifndef YYMALLOC
#   define YYMALLOC malloc
#   if ! defined malloc && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
    || defined __cplusplus || defined _MSC_VER)
void *malloc (YYSIZE_T); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
#  ifndef YYFREE
#   define YYFREE free
#   if ! defined free && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
    || defined __cplusplus || defined _MSC_VER)
void free (void *); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
# endif
#endif /* ! defined yyoverflow || YYERROR_VERBOSE */


#if (! defined yyoverflow \
    && (! defined __cplusplus \
    || (defined YYSTYPE_IS_TRIVIAL && YYSTYPE_IS_TRIVIAL)))

/* A type that is properly aligned for any stack member.  */
union yyalloc
{
    yytype_int16 yyss;
    YYSTYPE yyvs;
};

/* The size of the maximum gap between one aligned stack and the next.  */
# define YYSTACK_GAP_MAXIMUM (sizeof (union yyalloc) - 1)

/* The size of an array large to enough to hold all stacks, each with
N elements.  */
# define YYSTACK_BYTES(N) \
    ((N) * (sizeof (yytype_int16) + sizeof (YYSTYPE)) \
    + YYSTACK_GAP_MAXIMUM)

/* Copy COUNT objects from FROM to TO.  The source and destination do
not overlap.  */
# ifndef YYCOPY
#  if defined __GNUC__ && 1 < __GNUC__
#   define YYCOPY(To, From, Count) \
    __builtin_memcpy (To, From, (Count) * sizeof (*(From)))
#  else
#   define YYCOPY(To, From, Count)		\
    do					\
{					\
    YYSIZE_T yyi;				\
    for (yyi = 0; yyi < (Count); yyi++)	\
    (To)[yyi] = (From)[yyi];		\
}					\
    while (YYID (0))
#  endif
# endif

/* Relocate STACK from its old location to the new one.  The
local variables YYSIZE and YYSTACKSIZE give the old and new number of
elements in the stack, and YYPTR gives the new location of the
stack.  Advance YYPTR to a properly aligned location for the next
stack.  */
# define YYSTACK_RELOCATE(Stack)					\
    do									\
{									\
    YYSIZE_T yynewbytes;						\
    YYCOPY (&yyptr->Stack, Stack, yysize);				\
    Stack = &yyptr->Stack;						\
    yynewbytes = yystacksize * sizeof (*Stack) + YYSTACK_GAP_MAXIMUM; \
    yyptr += yynewbytes / sizeof (*yyptr);				\
}									\
    while (YYID (0))

#endif

/* YYFINAL -- State number of the termination state.  */
#define YYFINAL  5
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   165

/* YYNTOKENS -- Number of terminals.  */
#define YYNTOKENS  29
/* YYNNTS -- Number of nonterminals.  */
#define YYNNTS  34
/* YYNRULES -- Number of rules.  */
#define YYNRULES  60
/* YYNRULES -- Number of states.  */
#define YYNSTATES  161

/* YYTRANSLATE(YYLEX) -- Bison symbol number corresponding to YYLEX.  */
#define YYUNDEFTOK  2
#define YYMAXUTOK   275

#define YYTRANSLATE(YYX)						\
    ((unsigned int) (YYX) <= YYMAXUTOK ? yytranslate[YYX] : YYUNDEFTOK)

/* YYTRANSLATE[YYLEX] -- Bison symbol number corresponding to YYLEX.  */
static const yytype_uint8 yytranslate[] =
{
    0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,    23,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    24,     2,    25,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    26,    27,    28,    21,     2,    22,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
    2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
    5,     6,     7,     8,     9,    10,    11,    12,    13,    14,
    15,    16,    17,    18,    19,    20
};

#if YYDEBUG
/* YYPRHS[YYN] -- Index of the first RHS symbol of rule number YYN in
YYRHS.  */
static const yytype_uint8 yyprhs[] =
{
    0,     0,     3,     7,    10,    15,    20,    25,    30,    32,
    35,    40,    44,    48,    53,    55,    58,    60,    62,    64,
    66,    68,    70,    72,    74,    76,    78,    81,    85,    89,
    94,   101,   106,   113,   120,   125,   127,   130,   135,   137,
    140,   149,   151,   154,   163,   172,   182,   189,   194,   199,
    202,   205,   213,   223,   231,   233,   235,   237,   239,   241,
    243
};

/* YYRHS -- A `-1'-separated list of the rules' RHS.  */
static const yytype_int8 yyrhs[] =
{
    30,     0,    -1,    31,    35,    33,    -1,    31,    35,    -1,
    31,    32,    35,    33,    -1,    18,    21,    53,    22,    -1,
    20,    21,    49,    22,    -1,    19,    21,    51,    22,    -1,
    35,    -1,    34,    35,    -1,    36,    37,    34,    22,    -1,
    36,    37,    22,    -1,    36,    34,    22,    -1,    16,     5,
    21,     4,    -1,    38,    -1,    38,    37,    -1,    39,    -1,
    55,    -1,    42,    -1,    43,    -1,    44,    -1,    45,    -1,
    46,    -1,    47,    -1,    48,    -1,    39,    -1,    40,    39,
    -1,     7,    21,     4,    -1,    41,    40,    22,    -1,     8,
    21,    57,    22,    -1,     9,    21,    59,    23,    60,    22,
    -1,    11,    21,    58,    22,    -1,    13,    21,    60,    23,
    61,    22,    -1,    14,    21,    62,    23,    61,    22,    -1,
    12,    21,    57,    22,    -1,    50,    -1,    49,    50,    -1,
    5,    21,    62,    22,    -1,    52,    -1,    51,    52,    -1,
    5,    21,    59,    23,     4,    23,     5,    22,    -1,    54,
    -1,    53,    54,    -1,     5,    21,    62,    23,    62,    23,
    62,    22,    -1,     6,    21,    59,    23,    59,    23,    60,
    22,    -1,     6,    21,    59,    23,    59,    23,    60,    23,
    56,    -1,     5,    23,     5,    23,    59,    22,    -1,     5,
    23,     5,    22,    -1,     5,    23,    59,    22,    -1,    59,
    22,    -1,     5,    22,    -1,    24,    60,    23,    60,    23,
    60,    25,    -1,    24,    60,    23,    60,    23,    60,    23,
    60,    25,    -1,    24,    62,    23,    62,    23,    62,    25,
    -1,    62,    -1,     5,    -1,    26,    -1,    27,    -1,    28,
    -1,     4,    -1,     3,    -1
};

/* YYRLINE[YYN] -- source line where rule number YYN was defined.  */
static const yytype_uint16 yyrline[] =
{
    0,   116,   116,   117,   118,   122,   126,   130,   134,   135,
    139,   140,   141,   145,   149,   150,   154,   155,   159,   160,
    161,   162,   163,   164,   165,   169,   170,   174,   178,   182,
    186,   190,   194,   198,   202,   206,   207,   211,   215,   216,
    220,   225,   226,   230,   234,   235,   239,   240,   241,   242,
    243,   247,   251,   255,   259,   260,   264,   265,   266,   270,
    271
};
#endif

#if YYDEBUG || YYERROR_VERBOSE || YYTOKEN_TABLE
/* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
First, the terminals, then, starting at YYNTOKENS, nonterminals.  */
static const char *const yytname[] =
{
    "$end", "error", "$undefined", "YY_FLOAT", "YY_INTEGER", "YY_STRING",
    "YY_PRIMITIVE", "YY_CHAIN", "YY_TRANSLATE", "YY_TELESCOPE", "YY_SCALE",
    "YY_ROTATE_QUAT", "YY_ROTATE_EXPMAP", "YY_ROTATE_EULER",
    "YY_ROTATE_CONS", "YY_MARKER", "YY_NODE", "YY_CONST", "YY_DOFS",
    "YY_MARKERS", "YY_MASS", "'{'", "'}'", "','", "'<'", "'>'", "'x'", "'y'",
    "'z'", "$accept", "skel_file", "dof_list", "mass_list", "marker_list",
    "nodes", "node", "node_start", "node_elements", "node_element",
    "transform", "transforms", "chain_start", "chain", "translate",
    "telescope", "rotate_quat", "rotate_euler", "rotate_cons",
    "rotate_expmap", "masses", "mass", "markers", "marker", "dofs", "dof",
    "primitive", "geometry", "dof_vec3", "dof_vec4", "val_vec3", "dof_val",
    "axis", "number", 0
};
#endif

# ifdef YYPRINT
/* YYTOKNUM[YYLEX-NUM] -- Internal token number corresponding to
token YYLEX-NUM.  */
static const yytype_uint16 yytoknum[] =
{
    0,   256,   257,   258,   259,   260,   261,   262,   263,   264,
    265,   266,   267,   268,   269,   270,   271,   272,   273,   274,
    275,   123,   125,    44,    60,    62,   120,   121,   122
};
# endif

/* YYR1[YYN] -- Symbol number of symbol that rule YYN derives.  */
static const yytype_uint8 yyr1[] =
{
    0,    29,    30,    30,    30,    31,    32,    33,    34,    34,
    35,    35,    35,    36,    37,    37,    38,    38,    39,    39,
    39,    39,    39,    39,    39,    40,    40,    41,    42,    43,
    44,    45,    46,    47,    48,    49,    49,    50,    51,    51,
    52,    53,    53,    54,    55,    55,    56,    56,    56,    56,
    56,    57,    58,    59,    60,    60,    61,    61,    61,    62,
    62
};

/* YYR2[YYN] -- Number of symbols composing right hand side of rule YYN.  */
static const yytype_uint8 yyr2[] =
{
    0,     2,     3,     2,     4,     4,     4,     4,     1,     2,
    4,     3,     3,     4,     1,     2,     1,     1,     1,     1,
    1,     1,     1,     1,     1,     1,     2,     3,     3,     4,
    6,     4,     6,     6,     4,     1,     2,     4,     1,     2,
    8,     1,     2,     8,     8,     9,     6,     4,     4,     2,
    2,     7,     9,     7,     1,     1,     1,     1,     1,     1,
    1
};

/* YYDEFACT[STATE-NAME] -- Default rule to reduce with in state
STATE-NUM when YYTABLE doesn't specify something else to do.  Zero
means the default is an error.  */
static const yytype_uint8 yydefact[] =
{
    0,     0,     0,     0,     0,     1,     0,     0,     0,     3,
    0,     0,     0,    41,     0,     0,     0,     0,     2,     0,
    0,     0,     0,     0,     0,     0,     0,     0,     8,     0,
    14,    16,     0,    18,    19,    20,    21,    22,    23,    24,
    17,     0,     5,    42,     0,     0,     0,    35,     4,     0,
    0,     0,     0,     0,     0,     0,     0,     0,    12,     9,
    11,     0,    15,    25,     0,    60,    59,     0,    13,     0,
    6,    36,     0,     0,    38,     0,     0,    27,     0,     0,
    0,     0,     0,     0,    55,     0,    54,     0,    10,    28,
    26,     0,     0,     0,     7,    39,     0,     0,     0,    29,
    0,     0,    31,    34,     0,     0,     0,    37,     0,     0,
    0,     0,     0,     0,    56,    57,    58,     0,     0,     0,
    0,     0,     0,     0,    30,     0,    32,    33,     0,     0,
    0,     0,     0,     0,    43,     0,     0,    44,     0,     0,
    0,     0,    53,     0,    45,     0,    51,     0,    40,    50,
    0,    49,     0,     0,     0,    52,    47,     0,    48,     0,
    46
};

/* YYDEFGOTO[NTERM-NUM].  */
static const yytype_int16 yydefgoto[] =
{
    -1,     2,     3,     8,    18,    27,    28,    10,    29,    30,
    31,    64,    32,    33,    34,    35,    36,    37,    38,    39,
    46,    47,    73,    74,    12,    13,    40,   144,    79,    82,
    76,    85,   117,    86
};

/* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
STATE-NUM.  */
#define YYPACT_NINF -76
static const yytype_int16 yypact[] =
{
    -7,    24,    46,    23,    51,   -76,    54,    47,    55,    48,
    75,    49,     8,   -76,    52,    69,    48,    56,   -76,    57,
    59,    71,    81,    82,    92,    93,    94,   -12,   -76,    -4,
    87,   -76,    98,   -76,   -76,   -76,   -76,   -76,   -76,   -76,
    -76,    20,   -76,   -76,    72,    95,     9,   -76,   -76,    70,
    84,   113,    96,    84,    97,    96,    16,    20,   -76,   -76,
    -76,     0,   -76,   -76,    41,   -76,   -76,    99,   -76,    20,
    -76,   -76,   102,    10,   -76,    20,   101,   -76,    16,   103,
    104,    16,   106,   107,   -76,   108,   -76,   109,   -76,   -76,
    -76,    20,   111,    84,   -76,   -76,   112,    84,   114,   -76,
    16,   115,   -76,   -76,    38,    38,   116,   -76,   117,    20,
    118,    16,   120,    16,   -76,   -76,   -76,   121,   122,    20,
    126,   123,    16,   124,   -76,   125,   -76,   -76,   127,   128,
    20,    11,    16,    16,   -76,   129,   130,   -76,     3,   131,
    134,   132,   -76,    19,   -76,   136,   -76,    16,   -76,   -76,
    4,   -76,   135,    39,   137,   -76,   -76,    84,   -76,   139,
    -76
};

/* YYPGOTO[NTERM-NUM].  */
static const yytype_int16 yypgoto[] =
{
    -76,   -76,   -76,   -76,   110,    89,    -1,   -76,   133,   -76,
    -27,   -76,   -76,   -76,   -76,   -76,   -76,   -76,   -76,   -76,
    -76,    73,   -76,    63,   -76,   138,   -76,   -76,    90,   -76,
    -53,   -75,    60,   -40
};

/* YYTABLE[YYPACT[STATE-NUM]].  What to do in state STATE-NUM.  If
positive, shift that token.  If negative, reduce the rule which
number is the opposite.  If zero, do what YYDEFACT says.
If YYTABLE_NINF, syntax error.  */
#define YYTABLE_NINF -1
static const yytype_uint8 yytable[] =
{
    80,    67,     9,    98,     6,    63,   101,    16,   143,   153,
    58,     1,     6,    11,    45,    72,     6,    87,    60,    65,
    66,    84,    88,    65,    66,   112,    59,    75,    75,    92,
    42,    70,    94,   137,   138,    96,   123,    90,   125,     6,
    108,   149,   150,     7,   110,     4,     5,   131,    20,    21,
    22,   106,    23,    24,    25,    26,    11,   139,   140,    14,
    59,   156,   157,    89,   114,   115,   116,    17,    15,   121,
    41,     6,   152,    44,    45,    72,    68,    49,    50,   128,
    51,    19,    20,    21,    22,   145,    23,    24,    25,    26,
    136,     6,    52,    19,    20,    21,    22,   154,    23,    24,
    25,    26,    53,    54,   159,    20,    21,    22,    75,    23,
    24,    25,    26,    55,    56,    57,    69,    77,    61,    71,
    78,    81,    91,    93,    97,    99,    48,   100,   102,   103,
    129,   104,   105,   107,   141,   109,    95,   111,   113,   119,
    120,   122,   124,   126,   127,    83,   130,   132,   133,   134,
    43,   135,     0,     0,   148,   142,   146,   147,   151,   158,
    155,   160,     0,    62,     0,   118
};

static const yytype_int16 yycheck[] =
{
    53,    41,     3,    78,    16,    32,    81,     8,     5,     5,
    22,    18,    16,     5,     5,     5,    16,    57,    22,     3,
    4,     5,    22,     3,     4,   100,    27,    24,    24,    69,
    22,    22,    22,    22,    23,    75,   111,    64,   113,    16,
    93,    22,    23,    20,    97,    21,     0,   122,     7,     8,
    9,    91,    11,    12,    13,    14,     5,   132,   133,     5,
    61,    22,    23,    22,    26,    27,    28,    19,    21,   109,
    21,    16,   147,    21,     5,     5,     4,    21,    21,   119,
    21,     6,     7,     8,     9,   138,    11,    12,    13,    14,
    130,    16,    21,     6,     7,     8,     9,   150,    11,    12,
    13,    14,    21,    21,   157,     7,     8,     9,    24,    11,
    12,    13,    14,    21,    21,    21,    21,     4,    29,    46,
    24,    24,    23,    21,    23,    22,    16,    23,    22,    22,
    4,    23,    23,    22,     5,    23,    73,    23,    23,    23,
    23,    23,    22,    22,    22,    55,    23,    23,    23,    22,
    12,    23,    -1,    -1,    22,    25,    25,    23,    22,    22,
    25,    22,    -1,    30,    -1,   105
};

/* YYSTOS[STATE-NUM] -- The (internal number of the) accessing
symbol of state STATE-NUM.  */
static const yytype_uint8 yystos[] =
{
    0,    18,    30,    31,    21,     0,    16,    20,    32,    35,
    36,     5,    53,    54,     5,    21,    35,    19,    33,     6,
    7,     8,     9,    11,    12,    13,    14,    34,    35,    37,
    38,    39,    41,    42,    43,    44,    45,    46,    47,    48,
    55,    21,    22,    54,    21,     5,    49,    50,    33,    21,
    21,    21,    21,    21,    21,    21,    21,    21,    22,    35,
    22,    34,    37,    39,    40,     3,     4,    62,     4,    21,
    22,    50,     5,    51,    52,    24,    59,     4,    24,    57,
    59,    24,    58,    57,     5,    60,    62,    62,    22,    22,
    39,    23,    62,    21,    22,    52,    62,    23,    60,    22,
    23,    60,    22,    22,    23,    23,    62,    22,    59,    23,
    59,    23,    60,    23,    26,    27,    28,    61,    61,    23,
    23,    62,    23,    60,    22,    60,    22,    22,    62,     4,
    23,    60,    23,    23,    22,    23,    62,    22,    23,    60,
    60,     5,    25,     5,    56,    59,    25,    23,    22,    22,
    23,    22,    60,     5,    59,    25,    22,    23,    22,    59,
    22
};

#define yyerrok		(yyerrstatus = 0)
#define yyclearin	(yychar = YYEMPTY)
#define YYEMPTY		(-2)
#define YYEOF		0

#define YYACCEPT	goto yyacceptlab
#define YYABORT		goto yyabortlab
#define YYERROR		goto yyerrorlab


/* Like YYERROR except do call yyerror.  This remains here temporarily
to ease the transition to the new meaning of YYERROR, for GCC.
Once GCC version 2 has supplanted version 1, this can go.  */

#define YYFAIL		goto yyerrlab

#define YYRECOVERING()  (!!yyerrstatus)

#define YYBACKUP(Token, Value)					\
    do								\
    if (yychar == YYEMPTY && yylen == 1)				\
{								\
    yychar = (Token);						\
    yylval = (Value);						\
    yytoken = YYTRANSLATE (yychar);				\
    YYPOPSTACK (1);						\
    goto yybackup;						\
    }								\
  else								\
{								\
    yyerror (YY_("syntax error: cannot back up")); \
    YYERROR;							\
    }								\
    while (YYID (0))


#define YYTERROR	1
#define YYERRCODE	256


/* YYLLOC_DEFAULT -- Set CURRENT to span from RHS[1] to RHS[N].
If N is 0, then set CURRENT to the empty location which ends
the previous symbol: RHS[0] (always defined).  */

#define YYRHSLOC(Rhs, K) ((Rhs)[K])
#ifndef YYLLOC_DEFAULT
# define YYLLOC_DEFAULT(Current, Rhs, N)				\
    do									\
    if (YYID (N))                                                    \
{								\
    (Current).first_line   = YYRHSLOC (Rhs, 1).first_line;	\
    (Current).first_column = YYRHSLOC (Rhs, 1).first_column;	\
    (Current).last_line    = YYRHSLOC (Rhs, N).last_line;		\
    (Current).last_column  = YYRHSLOC (Rhs, N).last_column;	\
    }								\
      else								\
{								\
    (Current).first_line   = (Current).last_line   =		\
    YYRHSLOC (Rhs, 0).last_line;				\
    (Current).first_column = (Current).last_column =		\
    YYRHSLOC (Rhs, 0).last_column;				\
    }								\
    while (YYID (0))
#endif


/* YY_LOCATION_PRINT -- Print the location on the stream.
This macro was not mandated originally: define only if we know
we won't break user code: when these are the locations we know.  */

#ifndef YY_LOCATION_PRINT
# if defined YYLTYPE_IS_TRIVIAL && YYLTYPE_IS_TRIVIAL
#  define YY_LOCATION_PRINT(File, Loc)			\
    fprintf (File, "%d.%d-%d.%d",			\
    (Loc).first_line, (Loc).first_column,	\
    (Loc).last_line,  (Loc).last_column)
# else
#  define YY_LOCATION_PRINT(File, Loc) ((void) 0)
# endif
#endif


/* YYLEX -- calling `yylex' with the right arguments.  */

#ifdef YYLEX_PARAM
# define YYLEX yylex (YYLEX_PARAM)
#else
# define YYLEX yylex ()
#endif

/* Enable debugging if requested.  */
#if YYDEBUG

# ifndef YYFPRINTF
#  include <stdio.h> /* INFRINGES ON USER NAME SPACE */
#  define YYFPRINTF fprintf
# endif

# define YYDPRINTF(Args)			\
    do {						\
    if (yydebug)					\
    YYFPRINTF Args;				\
    } while (YYID (0))

# define YY_SYMBOL_PRINT(Title, Type, Value, Location)			  \
    do {									  \
    if (yydebug)								  \
{									  \
    YYFPRINTF (stderr, "%s ", Title);					  \
    yy_symbol_print (stderr,						  \
    Type, Value); \
    YYFPRINTF (stderr, "\n");						  \
    }									  \
    } while (YYID (0))


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

/*ARGSUSED*/
#if (defined __STDC__ || defined __C99__FUNC__ \
    || defined __cplusplus || defined _MSC_VER)
static void
yy_symbol_value_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
#else
static void
yy_symbol_value_print (yyoutput, yytype, yyvaluep)
FILE *yyoutput;
int yytype;
YYSTYPE const * const yyvaluep;
#endif
{
    if (!yyvaluep)
        return;
# ifdef YYPRINT
    if (yytype < YYNTOKENS)
        YYPRINT (yyoutput, yytoknum[yytype], *yyvaluep);
# else
    YYUSE (yyoutput);
# endif
    switch (yytype)
    {
    default:
        break;
    }
}


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
    || defined __cplusplus || defined _MSC_VER)
static void
yy_symbol_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
#else
static void
yy_symbol_print (yyoutput, yytype, yyvaluep)
FILE *yyoutput;
int yytype;
YYSTYPE const * const yyvaluep;
#endif
{
    if (yytype < YYNTOKENS)
        YYFPRINTF (yyoutput, "token %s (", yytname[yytype]);
    else
        YYFPRINTF (yyoutput, "nterm %s (", yytname[yytype]);

    yy_symbol_value_print (yyoutput, yytype, yyvaluep);
    YYFPRINTF (yyoutput, ")");
}

/*------------------------------------------------------------------.
| yy_stack_print -- Print the state stack from its BOTTOM up to its |
| TOP (included).                                                   |
`------------------------------------------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
    || defined __cplusplus || defined _MSC_VER)
static void
yy_stack_print (yytype_int16 *bottom, yytype_int16 *top)
#else
static void
yy_stack_print (bottom, top)
yytype_int16 *bottom;
yytype_int16 *top;
#endif
{
    YYFPRINTF (stderr, "Stack now");
    for (; bottom <= top; ++bottom)
        YYFPRINTF (stderr, " %d", *bottom);
    YYFPRINTF (stderr, "\n");
}

# define YY_STACK_PRINT(Bottom, Top)				\
    do {								\
    if (yydebug)							\
    yy_stack_print ((Bottom), (Top));				\
    } while (YYID (0))


/*------------------------------------------------.
| Report that the YYRULE is going to be reduced.  |
`------------------------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
    || defined __cplusplus || defined _MSC_VER)
static void
yy_reduce_print (YYSTYPE *yyvsp, int yyrule)
#else
static void
yy_reduce_print (yyvsp, yyrule)
YYSTYPE *yyvsp;
int yyrule;
#endif
{
    int yynrhs = yyr2[yyrule];
    int yyi;
    unsigned long int yylno = yyrline[yyrule];
    YYFPRINTF (stderr, "Reducing stack by rule %d (line %lu):\n",
        yyrule - 1, yylno);
    /* The symbols being reduced.  */
    for (yyi = 0; yyi < yynrhs; yyi++)
    {
        fprintf (stderr, "   $%d = ", yyi + 1);
        yy_symbol_print (stderr, yyrhs[yyprhs[yyrule] + yyi],
            &(yyvsp[(yyi + 1) - (yynrhs)])
            );
        fprintf (stderr, "\n");
    }
}

# define YY_REDUCE_PRINT(Rule)		\
    do {					\
    if (yydebug)				\
    yy_reduce_print (yyvsp, Rule); \
    } while (YYID (0))

/* Nonzero means print parse trace.  It is left uninitialized so that
multiple parsers can coexist.  */
int yydebug;
#else /* !YYDEBUG */
# define YYDPRINTF(Args)
# define YY_SYMBOL_PRINT(Title, Type, Value, Location)
# define YY_STACK_PRINT(Bottom, Top)
# define YY_REDUCE_PRINT(Rule)
#endif /* !YYDEBUG */


/* YYINITDEPTH -- initial size of the parser's stacks.  */
#ifndef	YYINITDEPTH
# define YYINITDEPTH 200
#endif

/* YYMAXDEPTH -- maximum size the stacks can grow to (effective only
if the built-in stack extension method is used).

Do not make this value too large; the results are undefined if
YYSTACK_ALLOC_MAXIMUM < YYSTACK_BYTES (YYMAXDEPTH)
evaluated with infinite-precision integer arithmetic.  */

#ifndef YYMAXDEPTH
# define YYMAXDEPTH 10000
#endif



#if YYERROR_VERBOSE

# ifndef yystrlen
#  if defined __GLIBC__ && defined _STRING_H
#   define yystrlen strlen
#  else
/* Return the length of YYSTR.  */
#if (defined __STDC__ || defined __C99__FUNC__ \
    || defined __cplusplus || defined _MSC_VER)
static YYSIZE_T
yystrlen (const char *yystr)
#else
static YYSIZE_T
yystrlen (yystr)
const char *yystr;
#endif
{
    YYSIZE_T yylen;
    for (yylen = 0; yystr[yylen]; yylen++)
        continue;
    return yylen;
}
#  endif
# endif

# ifndef yystpcpy
#  if defined __GLIBC__ && defined _STRING_H && defined _GNU_SOURCE
#   define yystpcpy stpcpy
#  else
/* Copy YYSRC to YYDEST, returning the address of the terminating '\0' in
YYDEST.  */
#if (defined __STDC__ || defined __C99__FUNC__ \
    || defined __cplusplus || defined _MSC_VER)
static char *
yystpcpy (char *yydest, const char *yysrc)
#else
static char *
yystpcpy (yydest, yysrc)
char *yydest;
const char *yysrc;
#endif
{
    char *yyd = yydest;
    const char *yys = yysrc;

    while ((*yyd++ = *yys++) != '\0')
        continue;

    return yyd - 1;
}
#  endif
# endif

# ifndef yytnamerr
/* Copy to YYRES the contents of YYSTR after stripping away unnecessary
quotes and backslashes, so that it's suitable for yyerror.  The
heuristic is that double-quoting is unnecessary unless the string
contains an apostrophe, a comma, or backslash (other than
backslash-backslash).  YYSTR is taken from yytname.  If YYRES is
null, do not copy; instead, return the length of what the result
would have been.  */
static YYSIZE_T
yytnamerr (char *yyres, const char *yystr)
{
    if (*yystr == '"')
    {
        YYSIZE_T yyn = 0;
        char const *yyp = yystr;

        for (;;)
            switch (*++yyp)
        {
            case '\'':
            case ',':
                goto do_not_strip_quotes;

            case '\\':
                if (*++yyp != '\\')
                    goto do_not_strip_quotes;
                /* Fall through.  */
            default:
                if (yyres)
                    yyres[yyn] = *yyp;
                yyn++;
                break;

            case '"':
                if (yyres)
                    yyres[yyn] = '\0';
                return yyn;
        }
do_not_strip_quotes: ;
    }

    if (! yyres)
        return yystrlen (yystr);

    return yystpcpy (yyres, yystr) - yyres;
}
# endif

/* Copy into YYRESULT an error message about the unexpected token
YYCHAR while in state YYSTATE.  Return the number of bytes copied,
including the terminating null byte.  If YYRESULT is null, do not
copy anything; just return the number of bytes that would be
copied.  As a special case, return 0 if an ordinary "syntax error"
message will do.  Return YYSIZE_MAXIMUM if overflow occurs during
size calculation.  */
static YYSIZE_T
yysyntax_error (char *yyresult, int yystate, int yychar)
{
    int yyn = yypact[yystate];

    if (! (YYPACT_NINF < yyn && yyn <= YYLAST))
        return 0;
    else
    {
        int yytype = YYTRANSLATE (yychar);
        YYSIZE_T yysize0 = yytnamerr (0, yytname[yytype]);
        YYSIZE_T yysize = yysize0;
        YYSIZE_T yysize1;
        int yysize_overflow = 0;
        enum { YYERROR_VERBOSE_ARGS_MAXIMUM = 5 };
        char const *yyarg[YYERROR_VERBOSE_ARGS_MAXIMUM];
        int yyx;

# if 0
        /* This is so xgettext sees the translatable formats that are
        constructed on the fly.  */
        YY_("syntax error, unexpected %s");
        YY_("syntax error, unexpected %s, expecting %s");
        YY_("syntax error, unexpected %s, expecting %s or %s");
        YY_("syntax error, unexpected %s, expecting %s or %s or %s");
        YY_("syntax error, unexpected %s, expecting %s or %s or %s or %s");
# endif
        char *yyfmt;
        char const *yyf;
        static char const yyunexpected[] = "syntax error, unexpected %s";
        static char const yyexpecting[] = ", expecting %s";
        static char const yyor[] = " or %s";
        char yyformat[sizeof yyunexpected
            + sizeof yyexpecting - 1
            + ((YYERROR_VERBOSE_ARGS_MAXIMUM - 2)
            * (sizeof yyor - 1))];
        char const *yyprefix = yyexpecting;

        /* Start YYX at -YYN if negative to avoid negative indexes in
        YYCHECK.  */
        int yyxbegin = yyn < 0 ? -yyn : 0;

        /* Stay within bounds of both yycheck and yytname.  */
        int yychecklim = YYLAST - yyn + 1;
        int yyxend = yychecklim < YYNTOKENS ? yychecklim : YYNTOKENS;
        int yycount = 1;

        yyarg[0] = yytname[yytype];
        yyfmt = yystpcpy (yyformat, yyunexpected);

        for (yyx = yyxbegin; yyx < yyxend; ++yyx)
            if (yycheck[yyx + yyn] == yyx && yyx != YYTERROR)
            {
                if (yycount == YYERROR_VERBOSE_ARGS_MAXIMUM)
                {
                    yycount = 1;
                    yysize = yysize0;
                    yyformat[sizeof yyunexpected - 1] = '\0';
                    break;
                }
                yyarg[yycount++] = yytname[yyx];
                yysize1 = yysize + yytnamerr (0, yytname[yyx]);
                yysize_overflow |= (yysize1 < yysize);
                yysize = yysize1;
                yyfmt = yystpcpy (yyfmt, yyprefix);
                yyprefix = yyor;
            }

            yyf = YY_(yyformat);
            yysize1 = yysize + yystrlen (yyf);
            yysize_overflow |= (yysize1 < yysize);
            yysize = yysize1;

            if (yysize_overflow)
                return YYSIZE_MAXIMUM;

            if (yyresult)
            {
                /* Avoid sprintf, as that infringes on the user's name space.
                Don't have undefined behavior even if the translation
                produced a string with the wrong number of "%s"s.  */
                char *yyp = yyresult;
                int yyi = 0;
                while ((*yyp = *yyf) != '\0')
                {
                    if (*yyp == '%' && yyf[1] == 's' && yyi < yycount)
                    {
                        yyp += yytnamerr (yyp, yyarg[yyi++]);
                        yyf += 2;
                    }
                    else
                    {
                        yyp++;
                        yyf++;
                    }
                }
            }
            return yysize;
    }
}
#endif /* YYERROR_VERBOSE */


/*-----------------------------------------------.
| Release the memory associated to this symbol.  |
`-----------------------------------------------*/

/*ARGSUSED*/
#if (defined __STDC__ || defined __C99__FUNC__ \
    || defined __cplusplus || defined _MSC_VER)
static void
yydestruct (const char *yymsg, int yytype, YYSTYPE *yyvaluep)
#else
static void
yydestruct (yymsg, yytype, yyvaluep)
const char *yymsg;
int yytype;
YYSTYPE *yyvaluep;
#endif
{
    YYUSE (yyvaluep);

    if (!yymsg)
        yymsg = "Deleting";
    YY_SYMBOL_PRINT (yymsg, yytype, yyvaluep, yylocationp);

    //switch (yytype)
    //{

    //default:
    //    break;
    //}
}


/* Prevent warnings from -Wmissing-prototypes.  */

#ifdef YYPARSE_PARAM
#if defined __STDC__ || defined __cplusplus
int yyparse (void *YYPARSE_PARAM);
#else
int yyparse ();
#endif
#else /* ! YYPARSE_PARAM */
#if defined __STDC__ || defined __cplusplus
int yyparse (void);
#else
int yyparse ();
#endif
#endif /* ! YYPARSE_PARAM */



/* The look-ahead symbol.  */
int yychar;

/* The semantic value of the look-ahead symbol.  */
YYSTYPE yylval;

/* Number of syntax errors so far.  */
int yynerrs;



/*----------.
| yyparse.  |
`----------*/

#ifdef YYPARSE_PARAM
#if (defined __STDC__ || defined __C99__FUNC__ \
    || defined __cplusplus || defined _MSC_VER)
int
yyparse (void *YYPARSE_PARAM)
#else
int
yyparse (YYPARSE_PARAM)
void *YYPARSE_PARAM;
#endif
#else /* ! YYPARSE_PARAM */
#if (defined __STDC__ || defined __C99__FUNC__ \
    || defined __cplusplus || defined _MSC_VER)
int
yyparse (void)
#else
int
yyparse ()

#endif
#endif
{

    int yystate;
    int yyn;
    int yyresult;
    /* Number of tokens to shift before error messages enabled.  */
    int yyerrstatus;
    /* Look-ahead token as an internal (translated) token number.  */
    int yytoken = 0;
#if YYERROR_VERBOSE
    /* Buffer for error messages, and its allocated size.  */
    char yymsgbuf[128];
    char *yymsg = yymsgbuf;
    YYSIZE_T yymsg_alloc = sizeof yymsgbuf;
#endif

    /* Three stacks and their tools:
    `yyss': related to states,
    `yyvs': related to semantic values,
    `yyls': related to locations.

    Refer to the stacks thru separate pointers, to allow yyoverflow
    to reallocate them elsewhere.  */

    /* The state stack.  */
    yytype_int16 yyssa[YYINITDEPTH];
    yytype_int16 *yyss = yyssa;
    yytype_int16 *yyssp;

    /* The semantic value stack.  */
    YYSTYPE yyvsa[YYINITDEPTH];
    YYSTYPE *yyvs = yyvsa;
    YYSTYPE *yyvsp;



#define YYPOPSTACK(N)   (yyvsp -= (N), yyssp -= (N))

    YYSIZE_T yystacksize = YYINITDEPTH;

    /* The variables used to return semantic value and location from the
    action routines.  */
    YYSTYPE yyval;


    /* The number of symbols on the RHS of the reduced rule.
    Keep to zero when no symbol should be popped.  */
    int yylen = 0;

    YYDPRINTF ((stderr, "Starting parse\n"));

    yystate = 0;
    yyerrstatus = 0;
    yynerrs = 0;
    yychar = YYEMPTY;		/* Cause a token to be read.  */

    /* Initialize stack pointers.
    Waste one element of value and location stack
    so that they stay on the same level as the state stack.
    The wasted elements are never initialized.  */

    yyssp = yyss;
    yyvsp = yyvs;

    goto yysetstate;

    /*------------------------------------------------------------.
    | yynewstate -- Push a new state, which is found in yystate.  |
    `------------------------------------------------------------*/
yynewstate:
    /* In all cases, when you get here, the value and location stacks
    have just been pushed.  So pushing a state here evens the stacks.  */
    yyssp++;

yysetstate:
    *yyssp = yystate;

    if (yyss + yystacksize - 1 <= yyssp)
    {
        /* Get the current used size of the three stacks, in elements.  */
        YYSIZE_T yysize = yyssp - yyss + 1;

#ifdef yyoverflow
        {
            /* Give user a chance to reallocate the stack.  Use copies of
            these so that the &'s don't force the real ones into
            memory.  */
            YYSTYPE *yyvs1 = yyvs;
            yytype_int16 *yyss1 = yyss;


            /* Each stack pointer address is followed by the size of the
            data in use in that stack, in bytes.  This used to be a
            conditional around just the two extra args, but that might
            be undefined if yyoverflow is a macro.  */
            yyoverflow (YY_("memory exhausted"),
                &yyss1, yysize * sizeof (*yyssp),
                &yyvs1, yysize * sizeof (*yyvsp),

                &yystacksize);

            yyss = yyss1;
            yyvs = yyvs1;
        }
#else /* no yyoverflow */
# ifndef YYSTACK_RELOCATE
        goto yyexhaustedlab;
# else
        /* Extend the stack our own way.  */
        if (YYMAXDEPTH <= yystacksize)
            goto yyexhaustedlab;
        yystacksize *= 2;
        if (YYMAXDEPTH < yystacksize)
            yystacksize = YYMAXDEPTH;

        {
            yytype_int16 *yyss1 = yyss;
            union yyalloc *yyptr =
                (union yyalloc *) YYSTACK_ALLOC (YYSTACK_BYTES (yystacksize));
            if (! yyptr)
                goto yyexhaustedlab;
            YYSTACK_RELOCATE (yyss);
            YYSTACK_RELOCATE (yyvs);

#  undef YYSTACK_RELOCATE
            if (yyss1 != yyssa)
                YYSTACK_FREE (yyss1);
        }
# endif
#endif /* no yyoverflow */

        yyssp = yyss + yysize - 1;
        yyvsp = yyvs + yysize - 1;


        YYDPRINTF ((stderr, "Stack size increased to %lu\n",
            (unsigned long int) yystacksize));

        if (yyss + yystacksize - 1 <= yyssp)
            YYABORT;
    }

    YYDPRINTF ((stderr, "Entering state %d\n", yystate));

    goto yybackup;

    /*-----------.
    | yybackup.  |
    `-----------*/
yybackup:

    /* Do appropriate processing given the current state.  Read a
    look-ahead token if we need one and don't already have one.  */

    /* First try to decide what to do without reference to look-ahead token.  */
    yyn = yypact[yystate];
    if (yyn == YYPACT_NINF)
        goto yydefault;

    /* Not known => get a look-ahead token if don't already have one.  */

    /* YYCHAR is either YYEMPTY or YYEOF or a valid look-ahead symbol.  */
    if (yychar == YYEMPTY)
    {
        YYDPRINTF ((stderr, "Reading a token: "));
        yychar = YYLEX;
    }

    if (yychar <= YYEOF)
    {
        yychar = yytoken = YYEOF;
        YYDPRINTF ((stderr, "Now at end of input.\n"));
    }
    else
    {
        yytoken = YYTRANSLATE (yychar);
        YY_SYMBOL_PRINT ("Next token is", yytoken, &yylval, &yylloc);
    }

    /* If the proper action on seeing token YYTOKEN is to reduce or to
    detect an error, take that action.  */
    yyn += yytoken;
    if (yyn < 0 || YYLAST < yyn || yycheck[yyn] != yytoken)
        goto yydefault;
    yyn = yytable[yyn];
    if (yyn <= 0)
    {
        if (yyn == 0 || yyn == YYTABLE_NINF)
            goto yyerrlab;
        yyn = -yyn;
        goto yyreduce;
    }

    if (yyn == YYFINAL)
        YYACCEPT;

    /* Count tokens shifted since error; after three, turn off error
    status.  */
    if (yyerrstatus)
        yyerrstatus--;

    /* Shift the look-ahead token.  */
    YY_SYMBOL_PRINT ("Shifting", yytoken, &yylval, &yylloc);

    /* Discard the shifted token unless it is eof.  */
    if (yychar != YYEOF)
        yychar = YYEMPTY;

    yystate = yyn;
    *++yyvsp = yylval;

    goto yynewstate;


    /*-----------------------------------------------------------.
    | yydefault -- do the default action for the current state.  |
    `-----------------------------------------------------------*/
yydefault:
    yyn = yydefact[yystate];
    if (yyn == 0)
        goto yyerrlab;
    goto yyreduce;


    /*-----------------------------.
    | yyreduce -- Do a reduction.  |
    `-----------------------------*/
yyreduce:
    /* yyn is the number of a rule to reduce with.  */
    yylen = yyr2[yyn];

    /* If YYLEN is nonzero, implement the default value of the action:
    `$$ = $1'.

    Otherwise, the following line sets YYVAL to garbage.
    This behavior is undocumented and Bison
    users should not rely upon it.  Assigning to YYVAL
    unconditionally makes the parser a bit smaller, and it avoids a
    GCC warning that YYVAL may be used uninitialized.  */
    yyval = yyvsp[1-yylen];


    YY_REDUCE_PRINT (yyn);
    switch (yyn)
    {
    case 2:
        //#line 116 "src/skel.y"
        { __finishSkel(); YYACCEPT; ;}
        break;

    case 3:
        //#line 117 "src/skel.y"
        { __finishSkel(); YYACCEPT; ;}
        break;

    case 4:
        //#line 118 "src/skel.y"
        { __finishSkel(); YYACCEPT; ;}
        break;

    case 10:
        //#line 139 "src/skel.y"
        { __endNode(); ;}
        break;

    case 11:
        //#line 140 "src/skel.y"
        { __endNode(); ;}
        break;

    case 12:
        //#line 141 "src/skel.y"
        { __endNode(); ;}
        break;

    case 13:
        //#line 145 "src/skel.y"
        { __startNode((yyvsp[(2) - (4)].sValue),(yyvsp[(4) - (4)].iValue)); ;}
        break;

    case 29:
        //#line 182 "src/skel.y"
        { __createTranslate((yyvsp[(3) - (4)].v3DValue)); ;}
        break;

    case 30:
        //#line 186 "src/skel.y"
        { __createTelescope((yyvsp[(3) - (6)].v3VValue),(yyvsp[(5) - (6)].dofValue)); ;}
        break;

    case 31:
        //#line 190 "src/skel.y"
        { __createRotateQuat((yyvsp[(3) - (4)].v4DValue)); ;}
        break;

    case 32:
        //#line 194 "src/skel.y"
        { __createRotateEuler((yyvsp[(3) - (6)].dofValue),(yyvsp[(5) - (6)].iValue)); ;}
        break;

    case 33:
        //#line 198 "src/skel.y"
        { __createRotateEuler((yyvsp[(3) - (6)].dValue),(yyvsp[(5) - (6)].iValue)); ;}
        break;

    case 34:
        //#line 202 "src/skel.y"
        { __createRotateExpMap((yyvsp[(3) - (4)].v3DValue)); ;}
        break;

    case 37:
        //#line 211 "src/skel.y"
        { __recordMass((yyvsp[(1) - (4)].sValue), (yyvsp[(3) - (4)].dValue)); ;}
        break;

    case 40:
        //#line 220 "src/skel.y"
        { __createMarker((yyvsp[(1) - (8)].sValue),(yyvsp[(3) - (8)].v3VValue),(yyvsp[(5) - (8)].iValue),(yyvsp[(7) - (8)].sValue)); ;}
        break;

    case 43:
        //#line 230 "src/skel.y"
        { (yyval.dofValue) = __createDOF((yyvsp[(1) - (8)].sValue),(yyvsp[(3) - (8)].dValue),(yyvsp[(5) - (8)].dValue),(yyvsp[(7) - (8)].dValue)); ;}
        break;

    case 44:
        //#line 234 "src/skel.y"
        { __setShape((yyvsp[(3) - (8)].v3VValue), (yyvsp[(5) - (8)].v3VValue), (yyvsp[(7) - (8)].dofValue)); ;}
        break;

    case 45:
        //#line 235 "src/skel.y"
        { __setShape((yyvsp[(3) - (9)].v3VValue), (yyvsp[(5) - (9)].v3VValue), (yyvsp[(7) - (9)].dofValue), (yyvsp[(9) - (9)].pValue)); ;}
        break;

    case 46:
        //#line 239 "src/skel.y"
        { (yyval.pValue) = __setGeometry((yyvsp[(1) - (6)].sValue), (yyvsp[(3) - (6)].sValue), (yyvsp[(5) - (6)].v3VValue)); ;}
        break;

    case 47:
        //#line 240 "src/skel.y"
        { (yyval.pValue) = __setGeometry((yyvsp[(1) - (4)].sValue), (yyvsp[(3) - (4)].sValue)); ;}
        break;

    case 48:
        //#line 241 "src/skel.y"
        { (yyval.pValue) = __setGeometry((yyvsp[(1) - (4)].sValue), (yyvsp[(3) - (4)].v3VValue)); ;}
        break;

    case 49:
        //#line 242 "src/skel.y"
        { (yyval.pValue) = __setGeometry((yyvsp[(1) - (2)].v3VValue)); ;}
        break;

    case 50:
        //#line 243 "src/skel.y"
        { (yyval.pValue) = __setGeometry((yyvsp[(1) - (2)].sValue)); ;}
        break;

    case 51:
        //#line 247 "src/skel.y"
        { (yyval.v3DValue)[0]=(yyvsp[(2) - (7)].dofValue); (yyval.v3DValue)[1]=(yyvsp[(4) - (7)].dofValue); (yyval.v3DValue)[2]=(yyvsp[(6) - (7)].dofValue); ;}
        break;

    case 52:
        //#line 251 "src/skel.y"
        { (yyval.v4DValue)[0]=(yyvsp[(2) - (9)].dofValue); (yyval.v4DValue)[1]=(yyvsp[(4) - (9)].dofValue); (yyval.v4DValue)[2]=(yyvsp[(6) - (9)].dofValue); (yyval.v4DValue)[3]=(yyvsp[(8) - (9)].dofValue); ;}
        break;

    case 53:
        //#line 255 "src/skel.y"
        { (yyval.v3VValue)[0]=(yyvsp[(2) - (7)].dValue); (yyval.v3VValue)[1]=(yyvsp[(4) - (7)].dValue); (yyval.v3VValue)[2]=(yyvsp[(6) - (7)].dValue); ;}
        break;

    case 54:
        //#line 259 "src/skel.y"
        { (yyval.dofValue) = __createDOF((yyvsp[(1) - (1)].dValue)); ;}
        break;

    case 55:
        //#line 260 "src/skel.y"
        { (yyval.dofValue) = __createDOF((yyvsp[(1) - (1)].sValue)); ;}
        break;

    case 56:
        //#line 264 "src/skel.y"
        { (yyval.iValue) = 0; ;}
        break;

    case 57:
        //#line 265 "src/skel.y"
        { (yyval.iValue) = 1; ;}
        break;

    case 58:
        //#line 266 "src/skel.y"
        { (yyval.iValue) = 2; ;}
        break;

    case 59:
        //#line 270 "src/skel.y"
        { (yyval.dValue) = (double)(yyvsp[(1) - (1)].iValue); ;}
        break;

    case 60:
        //#line 271 "src/skel.y"
        { (yyval.dValue) = (yyvsp[(1) - (1)].dValue); ;}
        break;


        /* Line 1267 of yacc.c.  */
        //#line 1729 "src/SkelParser.cpp"
    default: break;
    }
    YY_SYMBOL_PRINT ("-> $$ =", yyr1[yyn], &yyval, &yyloc);

    YYPOPSTACK (yylen);
    yylen = 0;
    YY_STACK_PRINT (yyss, yyssp);

    *++yyvsp = yyval;


    /* Now `shift' the result of the reduction.  Determine what state
    that goes to, based on the state we popped back to and the rule
    number reduced by.  */

    yyn = yyr1[yyn];

    yystate = yypgoto[yyn - YYNTOKENS] + *yyssp;
    if (0 <= yystate && yystate <= YYLAST && yycheck[yystate] == *yyssp)
        yystate = yytable[yystate];
    else
        yystate = yydefgoto[yyn - YYNTOKENS];

    goto yynewstate;


    /*------------------------------------.
    | yyerrlab -- here on detecting error |
    `------------------------------------*/
yyerrlab:
    /* If not already recovering from an error, report this error.  */
    if (!yyerrstatus)
    {
        ++yynerrs;
#if ! YYERROR_VERBOSE
        yyerror (YY_("syntax error"));
#else
        {
            YYSIZE_T yysize = yysyntax_error (0, yystate, yychar);
            if (yymsg_alloc < yysize && yymsg_alloc < YYSTACK_ALLOC_MAXIMUM)
            {
                YYSIZE_T yyalloc = 2 * yysize;
                if (! (yysize <= yyalloc && yyalloc <= YYSTACK_ALLOC_MAXIMUM))
                    yyalloc = YYSTACK_ALLOC_MAXIMUM;
                if (yymsg != yymsgbuf)
                    YYSTACK_FREE (yymsg);
                yymsg = (char *) YYSTACK_ALLOC (yyalloc);
                if (yymsg)
                    yymsg_alloc = yyalloc;
                else
                {
                    yymsg = yymsgbuf;
                    yymsg_alloc = sizeof yymsgbuf;
                }
            }

            if (0 < yysize && yysize <= yymsg_alloc)
            {
                (void) yysyntax_error (yymsg, yystate, yychar);
                yyerror (yymsg);
            }
            else
            {
                yyerror (YY_("syntax error"));
                if (yysize != 0)
                    goto yyexhaustedlab;
            }
        }
#endif
    }



    if (yyerrstatus == 3)
    {
        /* If just tried and failed to reuse look-ahead token after an
        error, discard it.  */

        if (yychar <= YYEOF)
        {
            /* Return failure if at end of input.  */
            if (yychar == YYEOF)
                YYABORT;
        }
        else
        {
            yydestruct ("Error: discarding",
                yytoken, &yylval);
            yychar = YYEMPTY;
        }
    }

    /* Else will try to reuse look-ahead token after shifting the error
    token.  */
    goto yyerrlab1;


    /*---------------------------------------------------.
    | yyerrorlab -- error raised explicitly by YYERROR.  |
    `---------------------------------------------------*/
yyerrorlab:

    /* Pacify compilers like GCC when the user code never invokes
    YYERROR and the label yyerrorlab therefore never appears in user
    code.  */
    if (/*CONSTCOND*/ 0)
        goto yyerrorlab;

    /* Do not reclaim the symbols of the rule which action triggered
    this YYERROR.  */
    YYPOPSTACK (yylen);
    yylen = 0;
    YY_STACK_PRINT (yyss, yyssp);
    yystate = *yyssp;
    goto yyerrlab1;


    /*-------------------------------------------------------------.
    | yyerrlab1 -- common code for both syntax error and YYERROR.  |
    `-------------------------------------------------------------*/
yyerrlab1:
    yyerrstatus = 3;	/* Each real token shifted decrements this.  */

    for (;;)
    {
        yyn = yypact[yystate];
        if (yyn != YYPACT_NINF)
        {
            yyn += YYTERROR;
            if (0 <= yyn && yyn <= YYLAST && yycheck[yyn] == YYTERROR)
            {
                yyn = yytable[yyn];
                if (0 < yyn)
                    break;
            }
        }

        /* Pop the current state because it cannot marker the error token.  */
        if (yyssp == yyss)
            YYABORT;


        yydestruct ("Error: popping",
            yystos[yystate], yyvsp);
        YYPOPSTACK (1);
        yystate = *yyssp;
        YY_STACK_PRINT (yyss, yyssp);
    }

    if (yyn == YYFINAL)
        YYACCEPT;

    *++yyvsp = yylval;


    /* Shift the error token.  */
    YY_SYMBOL_PRINT ("Shifting", yystos[yyn], yyvsp, yylsp);

    yystate = yyn;
    goto yynewstate;


    /*-------------------------------------.
    | yyacceptlab -- YYACCEPT comes here.  |
    `-------------------------------------*/
yyacceptlab:
    yyresult = 0;
    goto yyreturn;

    /*-----------------------------------.
    | yyabortlab -- YYABORT comes here.  |
    `-----------------------------------*/
yyabortlab:
    yyresult = 1;
    goto yyreturn;

#ifndef yyoverflow
    /*-------------------------------------------------.
    | yyexhaustedlab -- memory exhaustion comes here.  |
    `-------------------------------------------------*/
yyexhaustedlab:
    yyerror (YY_("memory exhausted"));
    yyresult = 2;
    /* Fall through.  */
#endif

yyreturn:
    if (yychar != YYEOF && yychar != YYEMPTY)
        yydestruct ("Cleanup: discarding lookahead",
        yytoken, &yylval);
    /* Do not reclaim the symbols of the rule which action triggered
    this YYABORT or YYACCEPT.  */
    YYPOPSTACK (yylen);
    YY_STACK_PRINT (yyss, yyssp);
    while (yyssp != yyss)
    {
        yydestruct ("Cleanup: popping",
            yystos[*yyssp], yyvsp);
        YYPOPSTACK (1);
    }
#ifndef yyoverflow
    if (yyss != yyssa)
        YYSTACK_FREE (yyss);
#endif
#if YYERROR_VERBOSE
    if (yymsg != yymsgbuf)
        YYSTACK_FREE (yymsg);
#endif
    /* Make sure YYID is used.  */
    return YYID (yyresult);
}


//#line 274 "src/skel.y"


void __finishSkel(){
    gSkel->initSkel();
}

/* create a new marker and add it to the apropriate node*/
void __createMarker( char* name, doubleVec3 offset, int id, char* node_name ) {

    BodyNode *node = NULL;

    // cout << "   reading marker: name\n";

    char fullNodeName[256];
    strcpy(fullNodeName, namePrefix);
    strcat(fullNodeName, node_name);

    //look up the node it's attached to by node name
    for( int i=0;i<num_nodes;i++ ) {
        if( !strcmp(node_lookup[i].name,fullNodeName) ) {
            node = node_lookup[i].node;
        }
    }

    //the node name is not used before
    if (node == NULL) {
        fprintf(stderr, "Parse Error: "
            "node %s undefined\n", node_name);
        return;
    }

    //create marker and add to the model; it will be added to the node automatically
    char fullName[256];
    strcpy(fullName, namePrefix);
    strcat(fullName, name);

    Vector3d vecOffset(offset[0],offset[1],offset[2]);
    Marker *tempMarker = new Marker(fullName, vecOffset, node);
    gSkel->addMarker(tempMarker);
}

/* create a new dof from the dof list at the beginning of the file*/
Dof* __createDOF( double val ) {
    Dof* d = new Dof(val);
    return d;
}

/* look up the dof by name from the existing dof list */
Dof* __createDOF( char* name ) {
    for( int i=0;i<num_dofs;i++ ) {
        if( !strcmp(dof_lookup[i].name,name) ) {
            return dof_lookup[i].dof;
        }
    }
    fprintf(stderr, "Parse Error: "
        "dof %s undefined\n", name);
    return NULL;
}

/* create a new dof from the dof list at the beginning of the file*/
Dof* __createDOF( char* name, double val, double lo, double hi ) {
    char fullName[256];
    strcpy(fullName, namePrefix);
    strcat(fullName, name);

    Dof *d = new Dof( val, fullName, lo, hi);

    dof_lookup[num_dofs].name = name;
    dof_lookup[num_dofs].dof = d;
    num_dofs++;

    // cout << "   reading dof: name\n";

    return d;
}

/* pop cur_node from stack; its parent becomes the cur_node again */
void __endNode()
{
    // add current node to node_lookup
    for( int i = 0; i < num_nodes; i++ ) {
        if( !strcmp(node_lookup[i].name, cur_node->getName()) ) {
            fprintf(stderr, "Parse Error: "
                "node multiply assigned to nodes %s,%s\n",
                cur_node->getName(), node_lookup[i].name);
        }
    }

    node_lookup[num_nodes].name = cur_node->getName();
    node_lookup[num_nodes].node = cur_node;
    num_nodes++;

    if( stack_top > 0 ) {
        cur_node = stack[--stack_top];
    }
}

/* create a new node whose parent node is cur_node, and create a new joint to connect them; add the new node to model*/
void __startNode( const char* s, int id ) {
    char fullName[256];
    strcpy(fullName, namePrefix);
    strcat(fullName, s);

    // create a new node
    // BodyNode* newNode = new BodyNode( fullName );
    BodyNode* newNode = gSkel->createBodyNode( fullName );
    // cout << "   reading node: s\n";

    // push the cur_node (parent) to the stack
    if( cur_node != NULL ){
        stack[stack_top++] = cur_node;
    }

    // create a new joint
    // constructor link the joint with node; the joint is then linked in model when the node is added to skel
    Joint* tempJ = new Joint(cur_node, newNode);

    // add the new node to model
    gSkel->addNode(newNode);

    //assign the new node to be cur_node
    cur_node = newNode;
}

/* create a new primitive */
void __setShape( doubleVec3 s, doubleVec3 t, Dof* bone ) {
    double bone_length = bone->getValue();

    Vector3d vecScale(s[0],s[1],s[2]);
    vecScale *= bone_length;
    Shape* prim = new ShapeEllipsoid(vecScale);

    Vector3d vecTrans(t[0],t[1],t[2]);
    vecTrans *= bone_length;
    // Set COM to be Shape offset
    prim->setOffset(vecTrans);

    cur_node->addVisualizationShape(prim);
    cur_node->addCollisionShape(prim);
    cur_node->setLocalCOM( vecTrans );
}

/* set offset and dimension of an existing primitive */
void __setShape( doubleVec3 s, doubleVec3 t, Dof* bone, Shape *geo ) {
    double bone_length = bone->getValue();

	/* SHAPE MASS IS DEPRECIATED **************************************************/
	/* TEMPORARY HACK TO RETURN MASS TO CALLER ************************************/
	/* SHAPE SETDIM SILENTLY CALLS COMPUTEVOLUME WHICH OVERWRITES VOLUME **********/
	/* DO NOT MOVE ****************************************************************/
		//FIXME: Refactor grammer: Move massName into __setShape()
		double mass = geo->getVolume();
		cur_node->setMass(mass);
	/******************************************************************************/

	geo->setDim(bone_length*Vector3d(s[0], s[1], s[2]));

    Vector3d vecTrans(t[0],t[1],t[2]);
    vecTrans *= bone_length;
    // Set COM to be Shape offset
    geo->setOffset(vecTrans);

    cur_node->addVisualizationShape(geo);
    cur_node->addCollisionShape(geo);
    cur_node->setLocalCOM(vecTrans);
}

/* create translation transformation */
void __createTranslate( dofVec3 v )
{
    //----  get common name of all three dofs
    int pos;

    char *underscore = strrchr(v[0]->getName(), '_');

    if(underscore != NULL){
        pos = underscore - v[0]->getName();
    }else{
        pos = strlen(v[0]->getName());
    }
    char *commonName = new char[pos + 1];
    strncpy(commonName, v[0]->getName(), pos);
    commonName[pos] = '\0';
    // cout << "common name is commonName\n";
    //----------

    // create new transformation
    TrfmTranslate* trans = new TrfmTranslate(v[0], v[1], v[2], commonName);

    // add transformation to joint
    if(cur_node!=NULL){
        cur_node->getParentJoint()->addTransform((Transformation*)trans);
    }
    // add transformation to model because it's a variable dof
    gSkel->addTransform((Transformation*)trans);

}

/* create a constant translation transformation */
void __createTelescope( doubleVec3 v, Dof* l )
{
    // create new transformation
    Dof** dofs = new Dof*[3];
    for(int i=0; i<3; i++){
        dofs[i] = new Dof(v[i]*l->getValue());
    }

    TrfmTranslate* tele = new TrfmTranslate(dofs[0],dofs[1],dofs[2]);

    // add transformation to joint
    // don't add to model because it's not variable
    if(cur_node!=NULL){
        cur_node->getParentJoint()->addTransform((Transformation*)tele, false);
    }
}

/* create rotation transformation using exponential map*/
void __createRotateExpMap( dofVec3 v )
{
    //----  get the common name of all three dofs
    int pos;

    char *underscore = strrchr(v[0]->getName(), '_');

    if(underscore != NULL){
        pos = underscore - v[0]->getName();
    }else{
        pos = strlen(v[0]->getName());
    }

    char *commonName = new char[pos + 1];
    strncpy(commonName, v[0]->getName(), pos);
    commonName[pos] = '\0';
    //--------

    // create new transformation
    TrfmRotateExpMap* expmap= new TrfmRotateExpMap(v[0], v[1], v[2], commonName);

    // add transformation to joint
    if(cur_node!=NULL){
        cur_node->getParentJoint()->addTransform((Transformation*)expmap);
    }

    //add to model because it's variable
    gSkel->addTransform((Transformation*)expmap);
}

/* create rotation transformation using quaternion */
void __createRotateQuat( dofVec4 v )
{
    //----  get the common name of all four dofs
    int pos;

    char *underscore = strrchr(v[0]->getName(), '_');

    if(underscore != NULL){
        pos = underscore - v[0]->getName();
    }else{
        pos = strlen(v[0]->getName());
    }

    char *commonName = new char[pos + 1];
    strncpy(commonName, v[0]->getName(), pos);
    commonName[pos] = '\0';
    //--------

    // cout << "    assuming the last value is w for quaternion\n";
    // create a new transformation
    TrfmRotateQuat*  quat = new TrfmRotateQuat(v[3], v[0], v[1], v[2], commonName);;

    // add transformation to joint
    if(cur_node!=NULL){
        cur_node->getParentJoint()->addTransform((Transformation*)quat);
    }

    // add to model because it's variable
    gSkel->addTransform((Transformation*)quat);
}

/* create rotation transformation using euler angels */
void __createRotateEuler( Dof* val, int axis ) {

    Transformation* trans=NULL;
    // create RotateEuler according to the axis
    switch(axis){
case 0: // 'x'
    trans = new TrfmRotateEulerX(val);
    break;
case 1: // 'y'
    trans = new TrfmRotateEulerY(val);
    break;
case 2: // 'z'
    trans = new TrfmRotateEulerZ(val);
    break;
    }

    // add transformation to joint
    if(cur_node!=NULL){
        cur_node->getParentJoint()->addTransform(trans);
    }

    // add to model because it's variable
    gSkel->addTransform(trans);

}

/* create rotation transformation using euler angels */
/* rotate_cons */
void __createRotateEuler( double val, int axis )
{
    // create new dof
    Dof *d = new Dof(val);

    Transformation* trans=NULL;
    // create RotateEuler according to the axis
    switch(axis){
case 0: // 'x'
    trans = new TrfmRotateEulerX(d);
    break;
case 1: // 'y'
    trans = new TrfmRotateEulerY(d);
    break;
case 2: // 'z'
    trans = new TrfmRotateEulerZ(d);
    break;
    }

    // not a variable dof, do not add to model
    // add transformation to joint
    if(cur_node!=NULL){
        cur_node->getParentJoint()->addTransform(trans, false);
    }
}

/* record mass */
void __recordMass(char* massName, double massValue)
{
    mass_lookup[num_masses].name = massName;
    mass_lookup[num_masses].mass = massValue;
    num_masses++;
}

/* create new primitive */
Shape* __setGeometry(const char* shape, const char *massName, doubleVec3 color)
{
    Shape *prim = NULL;

    // lookup mass by name
    double massValue = 0.0;
    for( int i = 0; i < num_masses; i++ ) {
        if( !strcmp(mass_lookup[i].name, massName) )
            massValue = mass_lookup[i].mass;
    }

    // create new primitive according to the shape
    if(!strcmp(shape, "CUBE"))
        prim = new ShapeBox(Vector3d(0,0,0));
    //	else if(!strcmp(shape, "CAPSULE"))
    //		prim = new Capsule(Vector3d(0,0,0), massValue);
    else if(!strcmp(shape, "") || !strcmp(shape, "SPHERE"))
        prim = new ShapeEllipsoid(Vector3d(0,0,0));
    else{
        cout<<"Unknown primitive type "<<shape<<endl;
        exit(0);
    }

    //set color
    prim->setColor(Vector3d(color[0], color[1], color[2]));

	/* SHAPE MASS IS DEPRECIATED **************************************************/
	/* TEMPORARY HACK TO RETURN MASS TO CALLER ************************************/
		//FIXME: Refactor grammer: Move massName into __setShape()
		prim->setVolume(massValue);
	/******************************************************************************/

	return prim;
}

/* create new primitive */
Shape* __setGeometry(const char *shape, const char *massName)
{
    Shape *prim = NULL;

    //lookup mass by name
    double massValue = 0.0;
    for( int i = 0; i < num_masses; i++ ) {
        if( !strcmp(mass_lookup[i].name, massName) ){
            massValue = mass_lookup[i].mass;
            break;
        }
    }

    // create new primitive according to the shape
    if(!strcmp(shape, "CUBE"))
        prim = new ShapeBox(Vector3d(0,0,0));
    //	else if(!strcmp(shape, "CAPSULE"))
    //		prim = new Capsule(Vector3d(0,0,0), massValue);
    else if(!strcmp(shape, "") || !strcmp(shape, "SPHERE"))
        prim = new ShapeEllipsoid(Vector3d(0,0,0));
    else{
        cout<<"Unknown primitive type "<<shape<<endl;
        exit(0);
    }

	/* SHAPE MASS IS DEPRECIATED **************************************************/
	/* TEMPORARY HACK TO RETURN MASS TO CALLER ************************************/
		//FIXME: Refactor grammer: Move massName into __setShape()
		prim->setVolume(massValue);
	/******************************************************************************/

	return prim;
}

/* create new primitive */
Shape* __setGeometry(const char *shape, doubleVec3 color)
{
    Shape *prim = NULL;

    // create new primitive by shape
    double massValue = 0.0;
    if(!strcmp(shape, "CUBE"))
        prim = new ShapeBox(Vector3d(0,0,0));
    //	else if(!strcmp(shape, "CAPSULE"))
    //		prim = new Capsule(Vector3d(0,0,0), massValue);
    else if(!strcmp(shape, "") || !strcmp(shape, "SPHERE"))
        prim = new ShapeEllipsoid(Vector3d(0,0,0));
    else{
        // mass name instead of shape is specified
        //lookup mass by name
        bool found = false;
        for( int i = 0; i < num_masses; i++ ) {
            if( !strcmp(mass_lookup[i].name, shape) ){
                massValue = mass_lookup[i].mass;
                found = true;
                break;
            }
        }
        // create new ellipsoid primitive
        if(found)
            prim = new ShapeEllipsoid(Vector3d(0,0,0));
    }

    // set color
    prim->setColor(Vector3d(color[0], color[1], color[2]));

    /* SHAPE MASS IS DEPRECIATED **************************************************/
    /* TEMPORARY HACK TO RETURN MASS TO CALLER ************************************/
        //FIXME: Refactor grammer: Move massName into __setShape()
        prim->setVolume(massValue);
    /******************************************************************************/

    return prim;
}

/* create new primitive */
Shape* __setGeometry(doubleVec3 color)
{
    Shape *prim = new ShapeEllipsoid(Vector3d(0,0,0));
    prim->setColor(Vector3d(color[0], color[1], color[2]));
    return prim;
}

/* create new primitive */
Shape* __setGeometry(const char *shape)
{
    Shape *prim = NULL;
    double massValue = 0.0;
    if(!strcmp(shape, "CUBE"))
        prim = new ShapeBox(Vector3d(0,0,0));
    //	else if(!strcmp(shape, "CAPSULE"))
    //		prim = new Capsule(Vector3d(0,0,0), massValue);
    else if(!strcmp(shape, "SPHERE") || !strcmp(shape, ""))
        prim = new ShapeEllipsoid(Vector3d(0,0,0));
    else{
        // mass name instead of shape is specified
        //lookup mass by name
        bool found = false;
        for( int i = 0; i < num_masses; i++ ) {
            if( !strcmp(mass_lookup[i].name, shape) ){
                massValue = mass_lookup[i].mass;
                found = true;
                break;
            }
        }
        // create new ellipsoid primitive
        if(found)
            prim = new ShapeEllipsoid(Vector3d(0,0,0));
    }

    /* SHAPE MASS IS DEPRECIATED **************************************************/
    /* TEMPORARY HACK TO RETURN MASS TO CALLER ************************************/
        //FIXME: Refactor grammer: Move massName into __setShape()
    if(prim)
        prim->setVolume(massValue);
    /******************************************************************************/

    return prim;
}

void yyerror( const char* error ) {
    printf( "%s\n", error );
}

Skeleton* readSkelFile( FILE* file, vector<const char*>* dofNames ) {
    gSkel = new Skeleton();

    cur_node = NULL;
    stack_top = 0;
    num_dofs = 0;
    num_nodes = 0;
    num_masses = 0;

    openFile( file );

    //yydebug = 1;
    int error = yyparse();

    //closeFile();

    if( error ) {
        delete gSkel;
        return NULL;
    }

    return gSkel;
}

int readSkelFile( const char* const filename, Skeleton* skel ) {
    FILE* file = fopen( filename, "r" );

    if( file == NULL ){
        fprintf(stderr, "Failed to open file %s", filename);
        return -1;
    }

    int first, last;
    first = 0; last = strlen(filename-1);
    for(int i=strlen(filename)-1; i>=0; i--){
        if(filename[i]=='.')
            last = i-1;
        else if(filename[i]=='/'){
            first = i+1;
            break;
        }
    }

    for(int i=0; i<last-first+1; i++)
        namePrefix[i] = filename[first+i];
    namePrefix[last-first+1] = '_';
    namePrefix[last-first+2] = '\0';

    gSkel = skel;
    cur_node = NULL;
    stack_top = 0;
    num_dofs = 0;
    num_nodes = 0;
    num_masses = 0;

    openFile( file );

    //yydebug = 1;
    int error = yyparse();

    //closeFile();

    fclose(file);
    return error;
}

Skeleton* readSkelFile( const char* filename, vector<const char*>* dofNames ) {
    FILE* file = fopen( filename, "r" );

    if( file == NULL )
        return NULL;

    Skeleton* skel = readSkelFile( file, dofNames );

    fclose(file);

    return skel;
}
