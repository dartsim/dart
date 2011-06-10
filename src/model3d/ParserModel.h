#ifndef SRC_MODEL3D_PARSER_MODEL_H
#define SRC_MODEL3D_PARSER_MODEL_H

#include "parser_defs.h"

typedef union {
  double	dValue;
  int iValue;
  char* sValue;
  vec3d v3DValue;
  vec4d v4DValue;
  vec3v v3VValue;
  model3d::Transformation* tValue;
  model3d::Primitive* pValue;
  model3d::Dof* dofValue;
} yystype;

# define YYSTYPE yystype
# define YYSTYPE_IS_TRIVIAL 1
# define YYSTYPE_IS_DECLARED

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
#define YY_HANDLE 270
#define YY_NODE 271
#define YY_CONST 272
#define YY_DOFS 273
#define YY_HANDLES 274
#define YY_MASS 275

extern YYSTYPE yylval;

#endif // #ifndef SRC_MODEL3D_PARSER_MODEL_H


