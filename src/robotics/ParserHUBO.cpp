/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Matthew Dutton <MatthewRDutton@gmail.com>
 * Date: 08/09/2012
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */
#include "ParserHUBO.h"

#include "Model3D.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <sstream>

// TinyXML-2 Library
// http://www.grinninglizard.com/tinyxml2/index.html
#include <tinyxml2.h>


using boost::shared_ptr;

using namespace boost::filesystem;
using namespace std;
using namespace tinyxml2;

using Eigen::Matrix3f;
typedef Eigen::Matrix<float,3,3,Eigen::RowMajor> RowMatrix3f;
using Eigen::Matrix4f;
using Eigen::Vector3f;


//---------------------------------------------------------------------------------------
struct ModelStore
{
    //typedef boost::filesystem::path path;
    typedef list<path> PathList;
    PathList paths;

    path eval( path const & relPath ) const
    {
        BOOST_FOREACH( path p, paths )
        {
            path absPath = boost::filesystem::absolute(relPath, p);
            //cout << "Eval: " << endl;
            //cout << '\t' << relPath << endl;
            //cout << '\t' << p << endl;
            //cout << '\t' << absPath << endl;
            if( exists(absPath) && is_regular_file( absPath ) )
                return absPath;
        }
        return path();
    }
};

//---------------------------------------------------------------------------------------
struct ParseCtx
{
    path basePath;
    ModelStore * pStore;
};

//---------------------------------------------------------------------------------------
static inline void PrintName( std::ostream & out, XMLElement const * p )
{
    if( p == 0 )
        return;

    XMLElement const * parent = p->Parent()->ToElement();
    if( parent )
        PrintName( out, parent );

    out << '/' << p->Name();
}

//---------------------------------------------------------------------------------------
static inline bool ElementNameIs( XMLElement const & elm, std::string const & name )
{
    return boost::iequals( elm.Name(), name );
}

//---------------------------------------------------------------------------------------
static inline XMLAttribute const * GetAttr( XMLElement const & elm, std::string const & attrName )
{
    for( XMLAttribute const * p = elm.FirstAttribute(); p; p = p->Next() )
    {
        if( boost::algorithm::iequals( attrName, string(p->Name()) ) )
            return p;
    }

    return 0;
}

//---------------------------------------------------------------------------------------
static inline char const * GetAttrValue( XMLElement const & elm, std::string const & attrName )
{
    XMLAttribute const * pAttr = GetAttr( elm, attrName );
    if( pAttr )
        return pAttr->Value();
    else
        return 0;
}

//---------------------------------------------------------------------------------------
static inline XMLElement const * GetElement( XMLElement const & elm, std::string const & name )
{
    for( XMLElement const * p = elm.FirstChildElement(); p; p = p->NextSiblingElement() )
    {
        if( boost::algorithm::iequals( name, string(p->Name()) ) )
            return p;
    }
    return 0;
}

//---------------------------------------------------------------------------------------
static inline char const * GetSubValue( XMLElement const & elm, std::string const & name )
{
    XMLAttribute const * pAttr = GetAttr(elm, name);
    if( pAttr )
        return pAttr->Value();

    XMLElement const * pElm = GetElement(elm, name);
    if( pElm )
        return pElm->GetText();

    return 0;
}

//---------------------------------------------------------------------------------------
template<typename T>
static inline vector<T> ParseValues( std::string const & str )
{
    stringstream ss( str );
    typedef istream_iterator<T> ISS;
    ISS const eos;
    return vector<T>( ISS(ss), ISS() );
}

//---------------------------------------------------------------------------------------
static inline bool ParseBool( string const & val )
{
    if( boost::algorithm::iequals(val, "true") || (val == "1") )
        return true;
    else if( boost::algorithm::iequals(val, "false") || (val == "0") )
        return false;
    else
    {
        cerr << "Invalid boolean value: " << val << endl;
        return false;
    }
}

//---------------------------------------------------------------------------------------
struct HUBOFile;
typedef shared_ptr<HUBOFile> HUBOFilePtr;

//---------------------------------------------------------------------------------------
struct ObjXform
{
    ObjXform() : rotation( Matrix3f::Identity() ), translation() { }
    ObjXform( ObjXform const & rhs )
        : rotation( rhs.rotation )
        , translation( rhs.translation )
    {   }

    // For some reason, rotations and translations are treated separately
    // http://openrave.programmingvision.com/wiki/index.php/Format:XML#Transformations
    Matrix3f rotation; // Rotation
    Vector3f translation; // Translation from parent node
};

//---------------------------------------------------------------------------------------
struct KinBody
{
    string name;
    bool makeJoinedLinksAdjacent;
    ObjXform xform;
};
typedef shared_ptr<KinBody> KinBodyPtr;

//---------------------------------------------------------------------------------------
struct Body
{
    enum Type { Type_Static, Type_Dynamic };

    string name;
    Type type;
    string offsetfrom;
    ObjXform xform;
};
typedef shared_ptr<Body> BodyPtr;

//---------------------------------------------------------------------------------------
struct Geometry
{
    ObjXform xform;
};
typedef shared_ptr<Geometry> GeometryPtr;

//---------------------------------------------------------------------------------------
void ParseFile( path const & p );

void ParseEnvironment( ParseCtx const & ctx, XMLElement const & rootElm ) { }

void ParseRobot( ParseCtx const & ctx, XMLElement const & rootElm ) { }

KinBodyPtr ParseKinBody( ParseCtx const & ctx, XMLElement const & rootElm );
KinBodyPtr ParseKinBodyFrom( ModelStore const * pModelStore, path const & p ) { return KinBodyPtr(); }

BodyPtr ParseBody( ParseCtx const & ctx, XMLElement const & rootElm );

GeometryPtr ParseGeometry( ParseCtx const & ctx, XMLElement const & rootElm );
GeometryPtr ParseGeometry_Box( ParseCtx const & ctx, XMLElement const & rootElm );
GeometryPtr ParseGeometry_Cylinder( ParseCtx const & ctx, XMLElement const & rootElm );
GeometryPtr ParseGeometry_TriMesh( ParseCtx const & ctx, XMLElement const & rootElm );

void ParseMass( ParseCtx const & ctx, XMLElement const & rootElm );
void ParseJoint( ParseCtx const & ctx, XMLElement const & rootElm );

bool ParseXformFrom( XMLElement const & rootElm, ObjXform & xform );
Matrix3f ParseRotationAxis( XMLElement const & rootElm );
Matrix3f ParseRotationMatrix( XMLElement const & rootElm );
Vector3f ParseTranslation( XMLElement const & rootElm );

//---------------------------------------------------------------------------------------
void ParseFile( path const & p )
{
    XMLDocument doc;

    path p2 = absolute( p, current_path() );
    cerr << p2 << '\t' << exists(p2) << endl;

    if( XML_NO_ERROR != doc.LoadFile( p2.native().c_str() ) )
    {
        cerr << "Could not load file: " << p.native() << endl;
        return;
    }

    XMLElement const * pRoot = doc.RootElement();
    string rootname( pRoot->Name() );

    ParseCtx ctx;
    ctx.basePath = p.parent_path();

    ModelStore store;
    store.paths.push_back( current_path() );
    ctx.pStore = &store;

    cout << "Loading file with root elm: " << rootname << endl;

    if( boost::iequals( rootname, "Environment" ) )
        ParseEnvironment( ctx, *pRoot );
    else if( boost::iequals( rootname, "KinBody") )
        ParseKinBody( ctx, *pRoot );
    else if( boost::iequals( rootname, "Robot") )
        ParseRobot( ctx, *pRoot );
    else
    {
        cerr << "Unrecognized document root: " << rootname << endl;
    }

    return;
}

//---------------------------------------------------------------------------------------
KinBodyPtr ParseKinBody( ParseCtx const & ctx, XMLElement const & rootElm )
{
    KinBodyPtr ret;
    char const * attr;

    if( !ElementNameIs(rootElm, "KinBody") )
    {
        cerr << "Invalid KinBody element: " << rootElm.Name() << endl;
        return KinBodyPtr();
    }

    attr = GetAttrValue(rootElm, "file");
    if( attr )
    {
        path p = ctx.pStore->eval( attr );
        if( p.empty() )
        {
            cerr << "Could not find referenced file: " << attr << endl;
            return KinBodyPtr();
        }

        ret = ParseKinBodyFrom( ctx.pStore, p );
        if( ret.get() == 0 )
        {
            cerr << "Could not parse referenced file: " << p << endl;
            return KinBodyPtr();
        }
    }
    else
    {
        ret = KinBodyPtr( new KinBody );
    }

    attr = GetAttrValue(rootElm, "name");
    if( attr )
    {
        ret->name = attr;
    }

    attr = GetAttrValue(rootElm, "makejoinedlinksadjacent");
    if( attr )
    {
        ret->makeJoinedLinksAdjacent = ParseBool(attr);
    }

    auto_ptr<ModelStore> pLocalStore;
    ParseCtx localCtx( ctx );

    for( XMLElement const * pChild = rootElm.FirstChildElement(); pChild;
         pChild = pChild->NextSiblingElement() )
    {
        if( ElementNameIs(*pChild, "adjacent") )
            ;//ParseAdjacent( localCtx, *pChild );
        else if( ElementNameIs(*pChild, "body") )
            BodyPtr pBody = ParseBody( localCtx, *pChild );
        else if( ElementNameIs(*pChild, "joint") )
            ParseJoint( localCtx, *pChild );
        else if( ElementNameIs(*pChild, "kinbody") )
            ParseKinBody( localCtx, *pChild );
        else if( ElementNameIs(*pChild, "modelsdir") )
        {
            if( pLocalStore.get() == 0 )
            {
                pLocalStore.reset( new ModelStore );
                localCtx.pStore = pLocalStore.get();
            }
            path p = absolute( path(pChild->GetText()), localCtx.basePath );
            cout << p << '\t' << localCtx.basePath << endl;
            localCtx.pStore->paths.push_back( p );
        }
        else if( ParseXformFrom(*pChild, ret->xform) )
            /* Handled in ParseXformFrom */;
        else
        {
            cerr << "Ignoring unknown KinBody element: " << pChild->Name() << endl;
        }
    }

    return ret;
}

//---------------------------------------------------------------------------------------
BodyPtr ParseBody( ParseCtx const & ctx, XMLElement const & rootElm )
{
    BodyPtr ret;
    char const * attr;

    if( !ElementNameIs(rootElm, "Body") )
    {
        cerr << "Invalid Body element: " << rootElm.Name() << endl;
        return BodyPtr();
    }

    ret = BodyPtr( new Body );

    attr = GetAttrValue(rootElm, "name");
    if( attr )
    {
        ret->name = attr;
    }

    attr = GetAttrValue(rootElm, "type");
    if( attr )
    {
        if( boost::iequals( attr, "static") )
            ret->type = Body::Type_Static;
        else if( boost::iequals( attr, "dynamic") )
            ret->type = Body::Type_Dynamic;
    }

    auto_ptr<ModelStore> pLocalStore;
    ParseCtx localCtx( ctx );

    for( XMLElement const * pChild = rootElm.FirstChildElement(); pChild;
         pChild = pChild->NextSiblingElement() )
    {
        if( ElementNameIs(*pChild, "geom") )
            ParseGeometry( localCtx, *pChild );
        else if( ElementNameIs(*pChild, "mass") )
            ParseMass( localCtx, *pChild );
        else if( ElementNameIs(*pChild, "offsetfrom") )
        {
            ret->offsetfrom = pChild->GetText();
        }
        else if( ElementNameIs(*pChild, "modelsdir") )
        {
            if( pLocalStore.get() == 0 )
            {
                pLocalStore.reset( new ModelStore );
                localCtx.pStore = pLocalStore.get();
            }
            path p = absolute( path(pChild->GetText()), localCtx.basePath );
            localCtx.pStore->paths.push_back( p );
        }
        else if( ParseXformFrom(*pChild, ret->xform) )
            /* Handled in ParseXformFrom */;
        else
        {
            cerr << "Ignoring unknown Body element: " << pChild->Name() << endl;
        }
    }


}

//---------------------------------------------------------------------------------------
GeometryPtr ParseGeometry( ParseCtx const & ctx, XMLElement const & rootElm )
{
    char const * attr = GetAttrValue( rootElm, "type" );

    if( !attr )
    {
        cerr << "Geom is missing type attribute!" << endl;
        return GeometryPtr();
    }

    GeometryPtr ret;
    if( boost::iequals(attr, "box") )
        ret = ParseGeometry_Box( ctx, rootElm );
    else if( boost::iequals(attr, "cylinder") )
        ret = ParseGeometry_Cylinder( ctx, rootElm );
    else if( boost::iequals(attr, "trimesh") )
        ret = ParseGeometry_TriMesh( ctx, rootElm );
    else
    {
        cerr << "Unrecognized <Geom> type: " << attr << endl;
        return GeometryPtr();
    }

    if( ret.get() == 0 )
        return ret;

    for( XMLElement const * pChild = rootElm.FirstChildElement(); pChild;
         pChild = pChild->NextSiblingElement() )
    {
        ParseXformFrom(*pChild, ret->xform);
    }

    Vector3f diffuseColor(0.5, 0.5, 0.5);
    char const * pDiffuse = GetSubValue( rootElm, "diffuseColor" );
    if( pDiffuse )
    {
        vector<float> values( ParseValues<float>( pDiffuse ) );
        if( values.size() != 3 )
        {
            cerr << "Geom <diffuseColor> must contain three values [r,g,b]: " << pDiffuse << endl;
        }
        else
        {
            diffuseColor = Eigen::Map<Vector3f>( &values[0] );
        }
    }

    Vector3f ambientColor(0.5, 0.5, 0.5);
    char const * pAmbient = GetSubValue( rootElm, "diffuseColor" );
    if( pAmbient )
    {
        vector<float> values( ParseValues<float>( pAmbient ) );
        if( values.size() != 3 )
        {
            cerr << "Geom <ambientColor> must contain three values [r,g,b]: " << pAmbient << endl;
        }
        else
        {
            ambientColor = Eigen::Map<Vector3f>( &values[0] );
        }
    }

    float transparency = 1.0f;
    char const * pTransparency = GetSubValue( rootElm, "transparency" );
    if( pTransparency )
    {
        string str( pTransparency );
        stringstream ss( str );
        ss >> transparency;
        if( !ss )
            cerr << "Failed to parse transparency: " << str << endl;
    }

    return ret;
}

//---------------------------------------------------------------------------------------
GeometryPtr ParseGeometry_Box( ParseCtx const & ctx, XMLElement const & rootElm )
{
    char const * pExtents = GetSubValue( rootElm, "extents" );
    if( !pExtents )
    {
        cerr << "<Geom type=box> must contain an <extents> element" << endl;
        return GeometryPtr();
    }

    vector<float> extents = ParseValues<float>( pExtents );
    if( extents.size() != 3 )
    {
        cerr << "Box <extents> should contain three values [x,y,z]: " << pExtents << endl;
        return GeometryPtr();
    }

    return GeometryPtr();
}

//---------------------------------------------------------------------------------------
GeometryPtr ParseGeometry_Cylinder( ParseCtx const & ctx, XMLElement const & rootElm )
{
    char const * pRadius = GetSubValue( rootElm, "radius" );
    if( !pRadius )
    {
        cerr << "<Geom type=cylinder> must contain a <radius> element" << endl;
        return GeometryPtr();
    }

    float radius;
    {
        string str( pRadius );
        stringstream ss( str );
        ss >> radius;
    }

    char const * pHeight = GetSubValue( rootElm, "height" );
    if( !pHeight )
    {
        cerr << "<Geom type=cylinder> must contain a <height> element" << endl;
        return GeometryPtr();
    }

    float height;
    {
        string str( pHeight );
        stringstream ss( str );
        ss >> height;
    }

    return GeometryPtr();
}

//---------------------------------------------------------------------------------------
GeometryPtr ParseGeometry_TriMesh( ParseCtx const & ctx, XMLElement const & rootElm )
{
    XMLElement const * pRenderElm = GetElement( rootElm, "render" );
    //XMLElement const * pVerticesElm = GetElement( rootElm, "vertices" );

    if( pRenderElm )
    {
        string filename;
        float scale = 1.0f;

        char const * pFile = GetSubValue( *pRenderElm, "file" );
        if( pFile )
        {
            filename = pFile;
            char const * pScale = GetSubValue( *pRenderElm, "scale" );
            if( pScale )
                stringstream(pScale) >> scale;
        }
        else
        {
            static char const * const WHITESPACE = " \t\r\n";

            string str( pRenderElm->GetText() );
            boost::algorithm::trim( str );
            stringstream ss( str );

            ss >> filename;
            if( ss )
                ss >> scale;
        }

        path filepath = ctx.pStore->eval( filename );
        if( filepath.empty() )
        {
            cerr << "Could not locate model file: " << filename << endl;
            return GeometryPtr();
        }

        auto_ptr<Model3D> pModel( new Model3D );
        if( !pModel->loadModel( filepath.native() ) )
        {
            cerr << "Could not load model file: " << filepath.native() << endl;
            return GeometryPtr();
        }

        cerr << "Done loading file with assimp" << endl;

    }
    //else if( pVerticesElm ) { }
    else
    {
        cerr << "A <Geom> node must contain either a <render> or <vertices> node" << endl;
    }

    return GeometryPtr();
}

//---------------------------------------------------------------------------------------
void ParseMass( ParseCtx const & ctx, XMLElement const & rootElm )
{
    char const * const pType = GetAttrValue( rootElm, "type" );
    if( !pType )
    {
        cerr << "<Mass> is missing the 'type' attribute" << endl;
        return;
    }

    string type( pType );
    if( boost::iequals( type, "box" ) )
    {
        float totalMass = 0.0f;
        Vector3f extents;

        char const * pTotal = GetSubValue( rootElm, "total" );
        if( !pTotal )
        {
            cerr << "<Mass> missing total mass value" << endl;
            return;
        }
        stringstream( string(pTotal ) ) >> totalMass;


        char const * pExtents = GetSubValue( rootElm, "extents" );
        if( !pExtents )
        {
            cerr << "<Mass type='box'> missing extents value" << endl;
            return;
        }

        vector<float> values( ParseValues<float>(pExtents) );
        if( values.size() != 3 )
        {
            cerr << "<Mass> extents must have three values: " << pExtents << endl;
            return;
        }

        extents = Eigen::Map<Vector3f>( &values[0] );
    }
    else if( boost::iequals( type, "custom") )
    {
        float totalMass = 0.0f;
        Matrix3f inertia;

        char const * pTotal = GetSubValue( rootElm, "total" );
        if( !pTotal )
        {
            cerr << "<Mass> missing total mass value" << endl;
            return;
        }
        stringstream( string(pTotal ) ) >> totalMass;

        char const * pInertia = GetSubValue( rootElm, "inertia" );
        if( !pInertia )
        {
            cerr << "<Mass type='custom'> missing inertia value" << endl;
            return;
        }

        vector<float> values( ParseValues<float>(pInertia) );
        if( values.size() != 9 )
        {
            cerr << "<Mass> inertia must have nine values: " << pInertia << endl;
            return;
        }

        inertia = Eigen::Map<RowMatrix3f>( &values[0] );
    }
    else if( boost::iequals( type, "sphere") )
    {
        float totalMass = 0.0f;
        float radius = 1.0f;

        char const * pTotal = GetSubValue( rootElm, "total" );
        if( !pTotal )
        {
            cerr << "<Mass> missing total mass value" << endl;
            return;
        }
        stringstream( string(pTotal ) ) >> totalMass;

        char const * pRadius = GetSubValue( rootElm, "total" );
        if( !pRadius )
        {
            cerr << "<Mass type='sphere'> missing radius value" << endl;
            return;
        }
        stringstream( string(pRadius ) ) >> radius;
    }
    else
    {
        cerr << "Unrecognized <Mass> type: " << type << endl;
        return;
    }
}

//---------------------------------------------------------------------------------------
void ParseJoint( ParseCtx const & ctx, XMLElement const & rootElm )
{

}

//---------------------------------------------------------------------------------------
bool ParseXformFrom( XMLElement const & elm, ObjXform & xform )
{
    if( ElementNameIs(elm, "rotationaxis") )
        xform.rotation = ParseRotationAxis( elm ) * xform.rotation;
    else if( ElementNameIs(elm, "rotationmat") )
        xform.rotation = ParseRotationMatrix( elm ) * xform.rotation;
    else if( ElementNameIs(elm, "translation") )
        xform.translation += ParseTranslation( elm );
    else
        return false;
    return true;
}

//---------------------------------------------------------------------------------------
Matrix3f ParseRotationAxis( XMLElement const & rootElm )
{
    Matrix3f ret = Matrix3f::Identity();

    std::vector<float> vals( ParseValues<float>( rootElm.GetText() ) );

    if( vals.size() != 4 )
    {
        cerr << "Incorrect number of values for <RotationAxis>: " << rootElm.GetText() << endl;
        return ret;
    }

    Vector3f axis( vals[0], vals[1], vals[2] );
    axis.normalize();

    ret = Eigen::AngleAxisf( vals[3], axis ).toRotationMatrix();

    return ret;
}

//---------------------------------------------------------------------------------------
Matrix3f ParseRotationMatrix( XMLElement const & rootElm )
{
    Matrix3f ret = Matrix3f::Identity();

    std::vector<float> vals( ParseValues<float>( rootElm.GetText() ) );

    if( vals.size() != 9 )
    {
        cerr << "Incorrect number of values for <RotationMat>: " << rootElm.GetText() << endl;
        return ret;
    }

    ret = Eigen::Map<RowMatrix3f>( &vals[0] );

    return ret;
}

//---------------------------------------------------------------------------------------
Vector3f ParseTranslation( XMLElement const & rootElm )
{
    std::vector<float> vals( ParseValues<float>( rootElm.GetText() ) );

    if( vals.size() != 3 )
    {
        cerr << "Incorrect number of values for <Translation>: " << rootElm.GetText() << endl;
        return Vector3f::Zero();
    }

    return  Eigen::Map<Vector3f>( &vals[0] );
}

