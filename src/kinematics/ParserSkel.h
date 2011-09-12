/*
RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
All rights reserved.

Author	Sehoon Ha
Date		06/12/2011
*/


#ifndef KINEMATICS_PARSER_SKEL_H
#define KINEMATICS_PARSER_SKEL_H

namespace kinematics {
    class Transformation;
    class Primitive;
    class Dof;
}

typedef kinematics::Dof* dofVec3[3];
typedef kinematics::Dof* dofVec4[4];
typedef double doubleVec3[3];

typedef union {
    double	dValue;
    int iValue;
    char* sValue;
    dofVec3 v3DValue;
    dofVec4 v4DValue;
    doubleVec3 v3VValue;
    kinematics::Transformation* tValue;
    kinematics::Primitive* pValue;
    kinematics::Dof* dofValue;
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
#define YY_MARKER 270
#define YY_NODE 271
#define YY_CONST 272
#define YY_DOFS 273
#define YY_MARKERS 274
#define YY_MASS 275

extern YYSTYPE yylval;

#endif // #ifndef KINEMATICS_PARSER_SKEL_H
