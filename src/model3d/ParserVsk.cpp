/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#include "ParserVsk.h"

// Standard Library
#include <map>
#include <sstream>
using namespace std;

// Boost Library
// You can replace this with your favorit tokenizer
#include <boost/tokenizer.hpp>
#include <boost/format.hpp>

// TiCPP library
// http://code.google.com/p/ticpp/
#include "ticpp.h"

#include <Eigen/Dense>
using namespace Eigen;

// Local Files
#include "Skeleton.h"
#include "BodyNode.h"
#include "Joint.h"
#include "Marker.h"
#include "Dof.h"
#include "Transformation.h"
#include "TrfmTranslate.h"
#include "TrfmRotateExpmap.h"
#include "TrfmRotateEuler.h"
#include "Marker.h"
#include "Primitive.h"
#include "PrimitiveEllipsoid.h"
#include "PrimitiveCube.h"
//#include "Quaternion.h"

#define mScaleVSK 1.0e-3
Vector3d expShoulder(0,0,0);
double lenShoulder = 0;

using namespace model3d;

// Forward Declarations of helper functions
Vector3d adjustPos(const Vector3d& _pos);
void Tokenize(const string& inputStr, vector<string>& tokens);
Quaterniond ExpToQuat(const Vector3d& v);
Vector3d RotatePoint(const Quaterniond& q, const Vector3d& pt);
Vector3d RotatePoint(const Quaterniond& q, double x, double y, double z);
double str2double(const string& str);
VectorXd getDofVectorXd(Transformation* trfm);

// Parsing Helper Functions    
bool readJointFree(ticpp::Element* _je, Joint* _jt, Skeleton* _model);
bool readJointBall(ticpp::Element* _je, Joint* _jt, Skeleton* _model, Vector3d orient);
bool readJointHardySpicer(ticpp::Element* _je, Joint* _jt, Skeleton* _model);
bool readJointHinge(ticpp::Element* _je, Joint* _jt, Skeleton* _model);
bool readSegment(ticpp::Element*_segment, BodyNode* _parent, map<string, double>& _paramsList, map<string, int>& _segmentindex, Skeleton* _model);
bool readMarker(ticpp::Element*_marker, map<string, double>& _paramsList, map<string, int>& _segmentindex, Skeleton* _model);
bool readPrimitive(ticpp::Element* _prim, map<string, double>& _paramsList, map<string, double>& _massList, map<string, int>& _segmentindex, Skeleton* _model);
void autoGeneratePrimitive(Skeleton* skel);
void autoGeneratePrimitive2(Skeleton* skel);


//
// Interface function.
//
int readVSKFile(const char* const filename, Skeleton* _skel)
{
  cout << "Read SKel File!! Yay!!" << endl;

  // Load xml and create Document
  ticpp::Document _stateFile(filename);
  try
  {
    _stateFile.LoadFile();
  }
  catch(ticpp::Exception e)
  {
    cout << "LoadFile Fails: " << e.what() << endl;
    return VSK_ERROR;
  }
  cout << "Load " << filename << " (xml) Successfully" << endl;

  // Load Kinematic Model which defines Parameters, Skeletons and Markers
  ticpp::Element* kinmodel = NULL;
  kinmodel = _stateFile.FirstChildElement( "KinematicModel" );
  if(!kinmodel) return VSK_ERROR;
    
  // Read parameters and fill paramsList
  map<string, double> paramsList;
  paramsList.clear();
  {
    ticpp::Element* params = 0;
    params = kinmodel->FirstChildElement( "Parameters" );
    if(!params) return VSK_ERROR;
    // read all params
    ticpp::Iterator< ticpp::Element > childparam("Parameter");
    for ( childparam = childparam.begin( params ); childparam != childparam.end(); childparam++ ){
      string pname = childparam->GetAttribute("NAME");
      double val = 0; 
      childparam->GetAttribute("VALUE", &val);
      paramsList[pname] = val;
      // cout << pname << " = " << val << endl;
    }
  }

  // Read skeleton and fill the Skeleton* and segmentindex
  map<string, int> segmentindex;
  segmentindex.clear();
  {
    ticpp::Element* skel= 0;
    skel = kinmodel->FirstChildElement( "Skeleton" );
    if(!skel) return VSK_ERROR;
    // read all segments
    ticpp::Iterator< ticpp::Element > childseg("Segment");
    for ( childseg = childseg.begin( skel ); childseg != childseg.end(); childseg++ ){
      if(!readSegment(childseg->ToElement(), NULL, paramsList,
                      segmentindex, _skel))
      {
        return VSK_ERROR;
      }
    }
  }        

  // Read markers and add them to _skel
  {
    ticpp::Element* markerset = 0;
    markerset = kinmodel->FirstChildElement( "MarkerSet" );
    if(!markerset) return VSK_ERROR;
    // read all markers
    ticpp::Element* markers = 0;
    markers = markerset->FirstChildElement( "Markers" );
    if(!markers) return VSK_ERROR;
    ticpp::Iterator< ticpp::Element > childmarker("Marker");
    for ( childmarker = childmarker.begin( markers ); childmarker != childmarker.end(); childmarker++ ){
      if(!readMarker(childmarker->ToElement(), paramsList,
                     segmentindex, _skel))
      {
        return VSK_ERROR;
      }
    }
  }

  // Read masses
  ticpp::Element* masses = 0;
  map<string, double> masslist;
  masslist.clear();
  {
    try {
      masses = kinmodel->FirstChildElement( "Masses" );
      if(!masses) return false;
      ticpp::Iterator< ticpp::Element > childmass("Mass");
      for ( childmass = childmass.begin( masses ); childmass != childmass.end(); childmass++ ){
        string mname = childmass->GetAttribute("NAME");
        double mi=0; childmass->GetAttribute("VALUE", &mi);
        masslist[mname] = mi;
        cout<<"mass: "<<mname<<" "<<mi<<endl;
      }
    } 
    catch( ticpp::Exception ){
      cout<<"no masses found!\n";
    }
  }

  // Read primitives and fill the _skel
  {
    ticpp::Element* prims = 0;
    try {
        prims = kinmodel->FirstChildElement( "Primitives" );
        if(!prims) return false;
        ticpp::Iterator< ticpp::Element > childprim("Primitive");
        for ( childprim = childprim.begin( prims ); childprim != childprim.end(); childprim++ ){
            if(!readPrimitive(childprim->ToElement(), paramsList, masslist, segmentindex, _skel)) return false;
        }
    } 
    catch( ticpp::Exception ){}

    // fill in the default if prims absent
  }

  autoGeneratePrimitive2(_skel);
  
  _skel->initSkel();
  // 
  cout << "Everything Looks good. I'm happy!" << endl;
  return VSK_OK;
}



bool readSegment(ticpp::Element*_segment, BodyNode* _parent, map<string, double>& _paramsList, map<string, int>& _segmentindex, Skeleton* _model)
{
  string sname = _segment->GetAttribute("NAME");

  cout<<"\nsegment: "<<sname<<" ";
  if(_parent) cout<<"parent: "<<_parent->getName()<<endl;
  else cout<<"parent: NULL\n";

  // make bodylink out of current segment
  BodyNode* blink = new BodyNode((char *)sname.c_str());

  // make a joint
  Joint* jt = new Joint(_parent, blink); 
  Vector3d orientation(0,0,0);
  
  //// HARDCODED: constant rotation for humerus and changed translation
  if(sname.compare(1, 8, "humerus")!=0)
  {
    cout << "THE COMMON CODE!!" << endl;
    string txyz = _segment->GetAttribute("POSITION");
    vector<string> tokens; Tokenize(txyz, tokens);
    assert(tokens.size()==3);
    Vector3d pos(0,0,0);
    for(int i=0; i<tokens.size(); i++) {
      string strval = tokens[i];
      int neg = 1;
      if(strval.c_str()[0]=='-') {
        neg = -1;
        strval.erase(strval.begin());
      }
      map<string, double>::iterator it = _paramsList.find(strval);
      if(it !=_paramsList.end()) pos[i] = neg*it->second;
      else {
        istringstream instr(tokens[i]);
        instr >> pos[i];
      }
    }
    Vector3d pos2 = adjustPos(pos);
    if(pos2 != Vector3d(0,0,0)){
      // create new transformation	
      Dof** dofs = new Dof*[3];
      for(int i=0; i<3; i++)
      {
        string dofName = (boost::format("%1%_%2%") % sname % i).str();
        char* pDofName = const_cast<char*>(dofName.c_str());
        dofs[i] = new Dof(pos2[i], pDofName);
      }
      TrfmTranslate* tele = new TrfmTranslate(dofs[0],dofs[1],dofs[2]);
      // add transformation to joint
      jt->addTransform(tele, false);	
      // don't add to model because it's not variable
      cout<<"telescope: "<<pos2<<endl;
    }

    {
      string txtOrientation = _segment->GetAttribute("ORIENTATION");
      vector<string> tokens; Tokenize(txtOrientation, tokens);
      assert(tokens.size()==3);
      for (int i = 0; i < 3; ++i) {
        orientation[i] = str2double(tokens[i]);
        // cout << "ORI = " << orientation[i] << " <== " << tokens[i] << endl;
      }
      orientation = adjustPos(orientation) / mScaleVSK;
    }

  }
  // HARDCODED: constant rotation for humerus and changed translation
  else {
    cout << "HUMERUS SPECIAL CODE!!!!!" << endl;
    char lr = sname[0];
    //adjusted: +-Shoulder ShoulderHeight 0 
    string paramShoulder = "ShoulderLen";
    paramShoulder.push_back(lr);

    // create new transformations
    // telescope
    Vector3d pos(0.0, 1.0, 0.0);	// adjusted for skel
    pos *= _paramsList[paramShoulder];
    cout<<"shoulder len: "<<_paramsList[paramShoulder]<<endl;
    Dof** dofs = new Dof*[3];
    for(int i=0; i<3; i++) dofs[i] = new Dof(pos[i]);
    TrfmTranslate* tele = new TrfmTranslate(dofs[0],dofs[1],dofs[2]);
    // add transformation to joint
    jt->addTransform(tele, false);	
    // don't add to model because it's not variable
    cout<<"telescope: "<<pos<<endl;

    // const rotation
    Dof *dofx = new Dof(-expShoulder[0]);
    Dof *dofy = new Dof(-expShoulder[1]);
    Dof *dofz = new Dof(-expShoulder[2]);
    TrfmRotateExpMap *consrot= new TrfmRotateExpMap(dofx, dofy, dofz);
    jt->addTransform(consrot, false);	
    // don't add to model because it's not variable
    cout<<"const rotation: "<<-expShoulder<<endl;
  }

  // HARDCODED: constant rotation for clavicle
  if(sname.compare(1, 8, "clavicle")==0){
    cout << "CLAVICLE SPECIAL CODE!!" << endl;
    char lr = sname[0];
    string hname = "humerus";
    hname.insert(hname.begin(), lr);
    cout<<hname<<endl;
    // read the childsegment humerus
    ticpp::Element *humerus = _segment->FirstChildElement( "Segment");
    string cname = humerus->GetAttribute("NAME");
    if(cname.compare(hname)!=0){
      cout<<"Error: childname of "<<sname<<" doesnt match: "<<hname<<" vs "<<cname<<endl;
      return false;
    }
    // add telescope and const rot transforms
    string hxyz = humerus->GetAttribute("POSITION");
    vector<string> tokens; Tokenize(hxyz, tokens);
    assert(tokens.size()==3);
    Vector3d pos(0,0,0);
    for(int i=0; i<tokens.size(); i++) {
      string strval = tokens[i];
      int neg = 1;
      if(strval.c_str()[0]=='-') {
        neg = -1;
        strval.erase(strval.begin());
      }
      map<string, double>::iterator it = _paramsList.find(strval);
      if(it !=_paramsList.end()) pos[i] = neg*it->second;
      else {
        istringstream instr(tokens[i]);
        instr >> pos[i];
      }
    }
    pos = adjustPos(pos);
    lenShoulder = pos.norm();
    string paramShoulder = "ShoulderLen";
    paramShoulder.push_back(lr);
    _paramsList[paramShoulder] = lenShoulder;
    cout<<"shoulder len: "<<_paramsList[paramShoulder]<<endl;
    //adjusted: +-Shoulder ShoulderHeight z 
    
    pos.normalize();
    double angleShoulder = pos[1];
    Vector3d axisShoulder = Vector3d(pos[2], 0, -pos[0]);
    axisShoulder.normalize();
    expShoulder = axisShoulder*angleShoulder;

    // create new transformation	
    Dof *dofx = new Dof(expShoulder[0]);
    Dof *dofy = new Dof(expShoulder[1]);
    Dof *dofz = new Dof(expShoulder[2]);
    TrfmRotateExpMap *consrot= new TrfmRotateExpMap(dofx, dofy, dofz);
    jt->addTransform(consrot, false);	
    // don't add to model because it's not variable
    cout<<"const rotation: "<<expShoulder<<endl;
  }

    
  // make transforms
  ticpp::Element *tf = 0;
  bool foundJoint = false;
  if(!foundJoint){
    try {
      tf = _segment->FirstChildElement( "JointFree" );
      foundJoint = true;
      readJointFree(tf, jt, _model);
    }catch( ticpp::Exception e){
    }
  }
  if(!foundJoint){
    try {
      tf = _segment->FirstChildElement( "JointBall" );
      foundJoint = true;
      readJointBall(tf, jt, _model, orientation);
    }catch( ticpp::Exception e){
    }
  }
  if(!foundJoint){
    try {
      tf = _segment->FirstChildElement( "JointHardySpicer" );
      foundJoint = true;
      readJointHardySpicer(tf, jt, _model);
    }catch( ticpp::Exception e){
    }
  }
  if(!foundJoint){
    try {
      tf = _segment->FirstChildElement( "JointHinge" );
      foundJoint = true;
      readJointHinge(tf, jt, _model);
    }catch( ticpp::Exception e){
    }
  }
  if(!foundJoint) cout<<"fixed joint!\n";

  for(int i=0; i<jt->getNumTransforms(); i++){
    if(!jt->getTransform(i)->getVariable()) continue;
    for(int j=0; j<jt->getTransform(i)->getNumDofs(); j++){
      cout<<jt->getTransform(i)->getDof(j)->getName()<<" ";
    }
    cout<<endl;
  }

  // add to the model
  _model->addNode(blink);
  // _segmentindex[sname]=blink->getModelID();
  _segmentindex[sname]=blink->getModelIndex();

  // handle the subtree
  ticpp::Iterator< ticpp::Element > childseg("Segment");
  for ( childseg = childseg.begin( _segment ); childseg != childseg.end(); childseg++ ){
    if(!readSegment(childseg->ToElement(), blink, _paramsList, _segmentindex, _model)) return false;
  }
  return true;
}

bool readJointFree(ticpp::Element* _je, Joint* _jt, Skeleton* _model)
{
  cout<<"read free\n";

  // create new transformation
  string tname1 = string(_jt->getNodeOut()->getName()) + "_t";
  string tname1_0 = tname1 + "Free0";
  string tname1_1 = tname1 + "Free1";
  string tname1_2 = tname1 + "Free2";
  vector<Dof*> dofs1;
  dofs1.resize(3);
  dofs1[0] = new Dof(0.0, const_cast<char*>(tname1_0.c_str()), -100.0, 100.0);
  dofs1[1] = new Dof(0.0, const_cast<char*>(tname1_1.c_str()), -100.0, 100.0);
  dofs1[2] = new Dof(0.0, const_cast<char*>(tname1_2.c_str()), -100.0, 100.0);
  // dofs1[1] = new Dof(0.0, -100.0, 100.0);
  // dofs1[2] = new Dof(0.0, -100.0, 100.0);
  // add transformation to joint
  TrfmTranslate* trans = new TrfmTranslate(dofs1[0], dofs1[1], dofs1[2], (char*)tname1.c_str()); 
  _jt->addTransform(trans);	
  // add transformation to model because it's a variable dof
  _model->addTransform(trans);

  string tname2 = string(_jt->getNodeOut()->getName()) + "_a";
  string tname2_0 = tname2 + "Free3";
  string tname2_1 = tname2 + "Free4";
  string tname2_2 = tname2 + "Free5";
  vector<Dof*> dofs2;
  dofs2.resize(3);
  dofs2[0] = new Dof(0.0, const_cast<char*>(tname2_0.c_str()), -3.1415, 3.1415);
  dofs2[1] = new Dof(0.0, const_cast<char*>(tname2_1.c_str()), -3.1415, 3.1415);
  dofs2[2] = new Dof(0.0, const_cast<char*>(tname2_2.c_str()), -3.1415, 3.1415);
  // dofs2[0] = new Dof(0.0, -3.1415, 3.1415);
  // dofs2[1] = new Dof(0.0, -3.1415, 3.1415);
  // dofs2[2] = new Dof(0.0, -3.1415, 3.1415);
  // add transformation to joint
  TrfmRotateExpMap* expmap= new TrfmRotateExpMap(dofs2[0], dofs2[1], dofs2[2], (char*)tname2.c_str());
  _jt->addTransform(expmap);	
  // add transformation to model because it's a variable dof
  _model->addTransform(expmap);

  return true;
}

bool readJointBall(ticpp::Element* _je, Joint* _jt, Skeleton* _model, Vector3d orient)
{
  cout << "read ball\n";
  cout << "orientation = " << orient << endl;
  string tname2 = string(_jt->getNodeOut()->getName()) + "_a";
  string tname2_0 = tname2 + "Ball0";
  string tname2_1 = tname2 + "Ball1";
  string tname2_2 = tname2 + "Ball2";
  vector<Dof*> dofs2;
  dofs2.resize(3);
  dofs2[0] = new Dof(orient[0], const_cast<char*>(tname2_0.c_str()), -3.1415, 3.1415);
  dofs2[1] = new Dof(orient[1], const_cast<char*>(tname2_1.c_str()), -3.1415, 3.1415);
  dofs2[2] = new Dof(orient[2], const_cast<char*>(tname2_2.c_str()), -3.1415, 3.1415);

  
  // dofs2[1] = new Dof(0.0, -3.1415, 3.1415);
  // dofs2[2] = new Dof(0.0, -3.1415, 3.1415);
  // add transformation to joint
  TrfmRotateExpMap* expmap= new TrfmRotateExpMap(dofs2[0], dofs2[1], dofs2[2], (char*)tname2.c_str());
  _jt->addTransform(expmap);	
  // add transformation to model because it's a variable dof
  _model->addTransform(expmap);

  return true;
}


bool readJointHardySpicer(ticpp::Element* _je, Joint* _jt, Skeleton* _model)
{
  cout<<"read hardy spicer\n";

  // Read axisxyz and parse it into tokens
  string axisxyz = _je->GetAttribute("AXIS-PAIR");
  vector<string> tokens;
  tokens.clear();

  string tname2 = string(_jt->getNodeOut()->getName()) + "_a";
  string tname2_1 = tname2 + "Hardy0";
  char* pTname1 = const_cast<char*>(tname2_1.c_str());
  string tname2_2 = tname2 + "Hardy1";
  char* pTname2 = const_cast<char*>(tname2_2.c_str());


  // Use boost::tokenizer
  boost::tokenizer<> tok(axisxyz);
  typedef boost::tokenizer<>::iterator tok_iter;
  for (tok_iter i = tok.begin(); i != tok.end(); ++i)
  {
    string temp = (*i);
    tokens.push_back(temp);
  }
  assert(tokens.size()==6);

  // Which axis do we have?
  Transformation *r1=NULL;
  if(tokens[1].compare("1")==0){
    r1 = new TrfmRotateEulerX(new Dof(0.0, pTname1, -3.1415, 3.1415));
  }
  else if(tokens[2].compare("1")==0){
    r1 = new TrfmRotateEulerY(new Dof(0.0, pTname1, -3.1415, 3.1415));
  }
  else if(tokens[0].compare("1")==0){
    r1 = new TrfmRotateEulerZ(new Dof(0.0, pTname1, -3.1415, 3.1415));
  }
  assert(r1!=NULL);
  _jt->addTransform(r1);	
  _model->addTransform(r1);

  Transformation *r2=NULL;
  if(tokens[4].compare("1")==0){
    r2 = new TrfmRotateEulerX(new Dof(0.0, pTname2, -3.1415, 3.1415));
  }
  else if(tokens[5].compare("1")==0){
    r2 = new TrfmRotateEulerY(new Dof(0.0, pTname2, -3.1415, 3.1415));
  }
  else if(tokens[3].compare("1")==0){
    r2 = new TrfmRotateEulerZ(new Dof(0.0, pTname2, -3.1415, 3.1415));
  }
  assert(r2!=NULL);
  _jt->addTransform(r2);	
  _model->addTransform(r2);

  return true;
}


bool readJointHinge(ticpp::Element* _je, Joint* _jt, Skeleton* _model)
{
  cout<<"read hinge\n";

  string tname = string(_jt->getNodeOut()->getName()) + "_a";
  tname += "Hinge0";
  char* pTname = const_cast<char*>(tname.c_str());

  string axisxyz = _je->GetAttribute("AXIS");
  vector<string> tokens;
  tokens.clear();

  // Use boost::tokenizer
  boost::tokenizer<> tok(axisxyz);
  typedef boost::tokenizer<>::iterator tok_iter;
  for (tok_iter i = tok.begin(); i != tok.end(); ++i)
  {
    string temp = (*i);
    tokens.push_back(temp);
  }
  assert(tokens.size()==3);

  // Read axes data
  Transformation *r1=NULL;
  Vector3d axis(str2double(tokens[0]),
             str2double(tokens[1]),
             str2double(tokens[2]));
  // if(tokens[1].compare("1")==0){
  if ((axis - adjustPos(Vector3d(1.0, 0.0, 0.0)) / mScaleVSK ).norm() < 0.01) {
    r1 = new TrfmRotateEulerX(new Dof(0.0, pTname, -3.1415, 3.1415));
    cout << "RotateEulerX" << endl;
  }
  else if ((axis - adjustPos(Vector3d(0.0, 1.0, 0.0)) / mScaleVSK ).norm() < 0.01) {
  // else if(tokens[2].compare("1")==0){
    r1 = new TrfmRotateEulerY(new Dof(0.0, pTname, -3.1415, 3.1415));
    cout << "RotateEulerY" << endl;
  }
  else if ((axis - adjustPos(Vector3d(0.0, 0.0, 1.0)) / mScaleVSK ).norm() < 0.01) {
  // else if(tokens[0].compare("1")==0){
    r1 = new TrfmRotateEulerZ(new Dof(0.0, pTname, -3.1415, 3.1415));
    cout << "RotateEulerZ" << endl;
  }
  assert(r1!=NULL);
  _jt->addTransform(r1);	
  _model->addTransform(r1);

  return true;
}

bool readMarker(ticpp::Element*_marker, map<string, double>& _paramsList, map<string, int>& _segmentindex, Skeleton* _model)
{
  string mname = _marker->GetAttribute("NAME");
  string sname = _marker->GetAttribute("SEGMENT");

  // get the local position
  string pxyz = _marker->GetAttribute("POSITION");
  vector<string> tokens;
  Tokenize(pxyz, tokens);
  assert(tokens.size()==3);


  Vector3d lpos(0,0,0);
  for(int i=0; i<tokens.size(); i++) {
    string strval = tokens[i];
    int neg = 1;
    if(strval.c_str()[0]=='-') {
      neg = -1;
      strval.erase(strval.begin());
    }
    map<string, double>::iterator it = _paramsList.find(strval);
    if(it !=_paramsList.end()) lpos[i] = neg*it->second;
    else {
      istringstream instr(tokens[i]);
      instr >> lpos[i];
    }
  }
  // rearrange the coordinates
  Vector3d lpos2 = adjustPos(lpos);

  // HARDCODED for clavicle
  if(sname.compare(1, 8, "clavicle")==0){
   // char lr = sname[0];
    // compute angle for the clavicle
    // left first; so negate the previous computed for right in reading segments and then same for the right
    expShoulder = -expShoulder;	

    // create new  position
    Quaterniond qr = ExpToQuat(-expShoulder);	// negative for the markers
    RotatePoint(qr, lpos2);
  }

  Marker* m = new Marker((char*)mname.c_str(), lpos2, _model->getNode(_segmentindex[sname]));
  _model->addHandle(m);
  cout<<"marker: "<<mname<<" ";
  cout<<"segment: "<<sname<<" ";
  cout<<"sindex: "<<_segmentindex[sname]<<" ";
  cout<<"lpos: "<<lpos2<<endl;
  return true;
}


bool readPrimitive(ticpp::Element* _prim, map<string, double>& _paramsList, map<string, double>& _massList, map<string, int>& _segmentindex, Skeleton* _model)
{
  string bname = _prim->GetAttribute("SEGMENT");
  int segIdx = _segmentindex[bname];
  BodyNode* blink = _model->getNode(segIdx);
	
  string mname = _prim->GetAttribute("MASS");
  double mass = _massList[mname];
	
  string sname = _prim->GetAttribute("SCALE");
  double scale = 0;
  map<string, double>::iterator it = _paramsList.find(sname);
  if(it !=_paramsList.end()) scale = it->second;
  else {
    istringstream instr(sname);
    instr >> scale;
  }

  string dimxyz = _prim->GetAttribute("DIMENSION");
  vector<string> tokens;
  Tokenize(dimxyz, tokens);
     
  assert(tokens.size()==3);

  Vector3d dim(0,0,0);
  for(int i=0; i<tokens.size(); i++) {
    istringstream instr(tokens[i]);
    instr >> dim[i];
  }
  dim = adjustPos(dim*scale);

  string offxyz = _prim->GetAttribute("OFFSET");
  Tokenize(offxyz, tokens);
  assert(tokens.size()==3);
    
  Vector3d off(0,0,0);
  for(int i=0; i<tokens.size(); i++) {
    istringstream instr(tokens[i]);
    instr >> off[i];
  }
  off = adjustPos(off*scale);


  Primitive *prim = NULL;
  string ptype = _prim->GetAttribute("TYPE");
  if(ptype.compare("ELLIPSOID")==0){
    prim = new PrimitiveEllipsoid(dim, mass);
  }
  // else if(ptype.compare("SPHERE")==0){
  //     prim = new PrimitiveSphere(off, dim[0], mass);
  // }
  // else if(ptype.compare("CYLINDER")==0){
  //     prim = new PrimitiveCylinder(off, dim, mass);
  // }
  // else if(ptype.compare("CYLINDERX")==0){
  //     prim = new PrimitiveCylinderX(off, dim, mass);
  // }
  // else if(ptype.compare("CYLINDERZ")==0){
  //     prim = new PrimitiveCylinderZ(off, dim, mass);
  // }
  // else if(ptype.compare("HEAD")==0){
  //     prim = new PrimitiveHead(off, dim, mass);
  // }
  else if(ptype.compare("CUBE")==0){
    prim = new PrimitiveCube(dim, mass);
  }
  else {
    cout << "Primitive type " << ptype << " not recognized!\n";
    return false;
  }

  //set color	
  try {
    string cname = _prim->GetAttribute("RGB");
    tokens.clear();
    Tokenize(cname, tokens);
    if (tokens.size() == 3)
    {
      Vector3d clr(0,0,0);
      for(int i=0; i<tokens.size(); i++) {
        istringstream instr(tokens[i]);
        instr >> clr[i];
      }
      prim->setColor(clr/255.0);
    }
    else
    {
      prim->setColor(Vector3d(0.5, 0.5, 1.0));
    }
  } 
  catch( ticpp::Exception ){
  }

  blink->setPrimitive(prim);
  blink->setOffset( off );
  return true;
}

void Tokenize(const string& inputStr, vector<string>& tokens)
{
  tokens.clear();
  // Use boost::tokenizer
  const char escapes[3] = " ";
  typedef boost::char_separator<char> sep_type;
  sep_type::char_separator<char> sep(escapes);
  boost::tokenizer< sep_type > tok(inputStr, sep);
  typedef boost::tokenizer< sep_type >::iterator tok_iter;
  for (tok_iter i = tok.begin(); i != tok.end(); ++i)
  {
    string temp = (*i);
    tokens.push_back(temp);
  }
}


Vector3d adjustPos(const Vector3d& _pos)
{
  // rearrange the coordinates
  Vector3d pos2 =_pos;
  pos2[0] = _pos[1];
  pos2[1] = _pos[2];
  pos2[2] = _pos[0];
  pos2 *= mScaleVSK;
  return pos2;
}

Quaterniond ExpToQuat(const Vector3d& v) {
  double mag = v.norm();
  if(mag > 1e-10){
    Quaterniond q(mag, v[0]/mag, v[1]/mag, v[2]/mag);
    return q;
  }
  else{
    Quaterniond q(1,0,0,0);
    return q;
  }
}

double str2double(const string& str) {
  stringstream ss(str);
  double value;
  ss >> value;
  return value;
}

VectorXd getDofVectorXd(Transformation* tr) {
  const int nDofs = tr->getNumDofs();
  VectorXd data(nDofs);
  for (int i = 0; i < nDofs; ++i) {
    Dof* dof = tr->getDof(i);
    data[i] = dof->getValue();
  }
  return data;
}

void autoGeneratePrimitive(Skeleton* skel)
{
  for(int i=0; i<skel->getNumNodes(); i++){
    if(skel->getNode(i)->getPrimitive()) continue;
    PrimitiveEllipsoid *pm = new PrimitiveEllipsoid(0.05 * Vector3d(1.0,1.0,1.0), 1.0);
    pm->setColor(Vector3d(0.5, 0.5, 1.0));
    BodyNode* node = skel->getNode(i);
    node->setPrimitive(pm);
    Vector3d vecZero(0,0,0);
    node->setOffset(vecZero);
  }
}

void autoGeneratePrimitive2(Skeleton* skel)
{
  // autoGeneratePrimitive(skel); return;
  
  cout << "Auto-generating primitives" << endl;

  double massSum = 0.0;
  for(int i=0; i<skel->getNumNodes(); i++){
    BodyNode* node = skel->getNode(i);
    Joint* joint = node->getJointIn();
    if(node->getPrimitive()) continue;
    // Search translate matrix
    Vector3d size = 0.1 * Vector3d(1,1,1);
    Vector3d offset(0,0,0);
    cout << endl;
    cout << "Node = " << node->getName() << endl;
    if (node->getNodeIn() == NULL)
    {
      cout << "I'm root!!!!!" << endl;
      size = 0.1 * Vector3d(1,1,1);
      continue;
    }
    BodyNode* parent = node->getNodeIn();

    cout << "Parent Node = " << node->getNodeIn()->getName() << endl;
    for (int j = 0; j < joint->getNumTransforms(); ++j)
    {
      Transformation* trfm = joint->getTransform(j);
      if (trfm->getType() == Transformation::T_TRANSLATE)
      {
        const VectorXd dofdata = getDofVectorXd(trfm);
        if (dofdata.size() == 3)
        {
          for (int k = 0; k < 3; ++k)
          {
            size[k] = fabs(dofdata[k]);
            offset[k] = dofdata[k] * 0.5;
          }
          break;
        }
      }
    }

    double maxSize = max(size[0], size[1]);
    maxSize = max(maxSize, size[2]);
    maxSize *= 0.35;
    maxSize = min(0.1, maxSize);
    
    for (int j = 0; j < 3; ++j)
    {
      size[j] = max(size[j], maxSize);
    }

    double density = 2000.0;
    double mass = density * size[0] * size[1] * size[2];
    massSum += mass;
    cout << "Size = " << size << endl;
    cout << "Offset = " << offset << endl;
    cout << "Mass = " << mass << endl;

    // size = 0.1 * Vector3d(vl_1);
    // offset = Vector3d(vl_0);
    // Ellipsoid *pm = new Ellipsoid(vl_0, 0.1*Vector3d(vl_1), 1.0);
    PrimitiveEllipsoid *pm = new PrimitiveEllipsoid(size, mass);
    BodyNode* target = parent;
    target->setOffset(offset);
    pm->setColor(Vector3d(0.5, 0.5, 1.0));
    target->setPrimitive(pm);

  }

  autoGeneratePrimitive(skel);
  cout << "Sum of mass = " << massSum << endl;
}

Vector3d RotatePoint(const Quaterniond& q, const Vector3d& pt)
{
	Quaterniond quat_pt(0, pt[0], pt[1], pt[2]);
	Quaterniond qinv = q.inverse();

	Quaterniond rot = q*quat_pt*qinv;

	// check below - assuming same format of point achieved
	Vector3d temp;
	//cout<<"Point before: "<<0<<" "<<pt.x<<" "<<pt.y<<" "<<pt.z<<"\n";
	//cout<<"Point after:  "<<rot.x<<" "<<rot.y<<" "<<rot.z<<" "<<rot.w<<"\n";
	temp[0]=rot.x();
	temp[1]=rot.y();
	temp[2]=rot.z();

	//cout<<"Point after rotation: "<<temp[0]<<" "<<temp[1]<<" "<<temp[2]<<endl;
	return temp;
}

Vector3d RotatePoint(const Quaterniond& q, double x, double y, double z){
	Vector3d pt;
	pt[0]=x;
	pt[1]=y;
	pt[2]=z;

	return RotatePoint(q, pt);
}

