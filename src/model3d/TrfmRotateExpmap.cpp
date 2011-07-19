/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#include "TrfmRotateExpmap.h"
#include "Dof.h"
#include "utils/RotationConversion.h"


using namespace std;

inline double Tsinc(double theta){
    return 0.5-sqrt(theta)/48;
}

inline bool isZero(double theta){
    return(fabs(theta)<EPSILON);
}

inline int delta(int i, int j){
    if(i==j) return 1;
    return 0;
}

inline double sqr(double d) {
    return d * d;
}

namespace model3d {

    TrfmRotateExpMap::TrfmRotateExpMap(Dof *x, Dof *y, Dof *z, char* _name){
        mDofs.resize(3);
        mDofs[0]=x;
        mDofs[1]=y;
        mDofs[2]=z;
        x->setTrans(this);
        y->setTrans(this);
        z->setTrans(this);
        mType = Transformation::T_ROTATEEXPMAP;
        if(_name!=NULL)
            strcpy(mName, _name);
        else
            strcpy(mName, "EXPMAP");
    }

    void TrfmRotateExpMap::evalTransform(){
        Vector3d v(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue());
        double theta= v.norm();
        Vector3d vhat = Vector3d::Zero();
        if(!isZero(theta)) vhat= v/theta;
        Quaterniond q(AngleAxisd(theta, vhat));
        // Quaternion q(vhat, theta);

        mTransform.setZero();
        // TODO: check to make sure this is a mat3d
        Matrix3d rot = q.matrix();      
        for(int i=0; i<3; i++){
            for(int j=0; j<3; j++)
                mTransform(i, j) = rot(i, j);
        }
        mTransform(3, 3) = 1.0;
    }

    Matrix4d TrfmRotateExpMap::getInvTransform(){
        if(isDirty){
            evalTransform();
            isDirty=false;
        }
        return mTransform.transpose();
    }

    double TrfmRotateExpMap::get_dq_dv(int i, int j, double theta, Vector3d v, Vector3d vhat){
        double dq_dv=0;
        double sinc_theta_half = (sin(0.5*theta)/theta);
        if (i==0) dq_dv = -0.5*v(j)*sinc_theta_half;
        else {
            i=i-1;
            dq_dv = sinc_theta_half*delta(i,j) + 0.5*vhat(i)*vhat(j)*(cos(0.5*theta)-2*sinc_theta_half);
        }
        return dq_dv;
    }

    double TrfmRotateExpMap::get_dq_dv_approx(int i, int j, double theta, Vector3d v){
        // printf("========== inside approx ==========\n");
        double dq_dv=0;
        double sinc_theta_half = Tsinc(theta);
        if (i==0) dq_dv = -0.5*v(j)*sinc_theta_half;
        else {
            i=i-1;
            dq_dv = sinc_theta_half*delta(i,j) + (v(i)*v(j)*(sqr(theta)/40-1))/24;
        }
        return dq_dv;
    }

    Matrix4d TrfmRotateExpMap::getDeriv(const Dof *d){
        Vector3d v(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue());
        double theta = v.norm();

        Vector3d vhat = Vector3d::Zero();
        if(!isZero(theta)) vhat= v/theta;
        Quaterniond q(AngleAxisd(theta, vhat));

        
        // compute derivative of R wrt each qi
        vector<Matrix3d> dR_dq;
        dR_dq.resize(4);
// TODO need to rewrite
        for(int i=0; i<4; i++) dR_dq[i] = utils::rot_conv::getDerivativeMatrix(q, i);

        // derivative wrt which dof 
        int j=-1;
        for(unsigned int i=0; i<mDofs.size(); i++) if(d==mDofs[i]) j=i;
        assert(j!=-1);

        // compute derivative of qi's wrt v[j]
        vector<double> dq_dv;
        dq_dv.resize(4);
        if(fabs(theta)<EPSILON) {
            for(int i=0; i<4; i++){
                dq_dv[i] = get_dq_dv_approx(i, j, theta, v);
            }
        }
        else {
            for(int i=0; i<4; i++){
                dq_dv[i] = get_dq_dv(i, j, theta, v, vhat);
            }
        }

        // compute the reqd derivative
        Matrix3d mat= Matrix3d::Zero();

        for(int i=0; i<4; i++){
            mat+=dR_dq[i]*dq_dv[i];
        }

        Matrix4d ret = Matrix4d::Zero();
        for(int i=0; i<3; i++){
            for(int j=0; j<3; j++){
                ret(i, j) = mat(i, j);
            }
        }


        return ret;
    }

    double TrfmRotateExpMap::get_dq_dv_dv(int i, int j, int k, double theta, Vector3d vhat){
        double dq_dv_dv=0;
	
        double cos_half = cos(0.5*theta);
        double sin_half = sin(0.5*theta);
        double sinc_theta_half = (sin_half/theta);

        if (i==0) {
            dq_dv_dv = -0.5*sinc_theta_half*delta(j,k);
            dq_dv_dv += - 0.25*vhat(j)*vhat(k)*(cos_half-2*sinc_theta_half);
        }
        else {
            i=i-1;
            dq_dv_dv = ((cos_half-2*sinc_theta_half)/(2*theta))*(vhat(k)*delta(i,j)+vhat(i)*delta(k,j)+vhat(j)*delta(i,k));
            dq_dv_dv += 0.25*vhat(i)*vhat(j)*vhat(k)*(-sin_half-(6/theta)*(cos_half-2*sinc_theta_half));
        }
        return dq_dv_dv;
    }

    double TrfmRotateExpMap::get_dq_dv_dv_approx(int i, int j, int k, double theta, Vector3d v){
        // printf("========= inside approx ==========\n");
        double dq_dv_dv=0;

        if (i==0) {
            dq_dv_dv = v(j)*v(k)/48 - 0.5*Tsinc(theta)*delta(j,k);
        }
        else {
            i=i-1;
            dq_dv_dv = -(v(k)*delta(i,j)+v(i)*delta(k,j)+v(j)*delta(i,k))/24;
            dq_dv_dv += (sqr(theta)/960)*(v(i)*delta(k,j)+v(j)*delta(i,k));
            dq_dv_dv += v(i)*v(j)*v(k)/480;
        }
        return dq_dv_dv;
    }

    Matrix4d TrfmRotateExpMap::getSecondDeriv(const Dof *q1, const Dof *q2){
        Vector3d v(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue());
        double theta = v.norm();
        Vector3d vhat = Vector3d::Zero();
        if(!isZero(theta)) vhat= v/theta;
        Quaterniond q(AngleAxisd(theta, vhat));
	
        // derivative wrt which mDofs
        int j=-1, k=-1;
        for(unsigned int i=0; i<mDofs.size(); i++) {
            if(q1==mDofs[i]) j=i;
            if(q2==mDofs[i]) k=i;
        }
        assert(j!=-1);
        assert(k!=-1);

        // compute derivative of R wrt each qi
        vector<Matrix3d> dR_dq;
        dR_dq.resize(4);
// TODO need to rewrite
//  for(int i=0; i<4; i++) dR_dq[i] = utils::rot_conv::getDerivativeMatrix(q, i);

        // compute derivative of R wrt each qi and ql
        vector<vector<Matrix3d> > dR_dq_dq;
        dR_dq_dq.resize(4);
        for(int i=0; i<4; i++) dR_dq_dq[i].resize(4);
        for(int i=0; i<4; i++) {
            for(int l=i; l<4; l++) {
                // TODO need to rewrite
                //dR_dq_dq[i][l] = utils::rot_conv::getDerivativeMatrix(q, i, l);
                dR_dq_dq[l][i]=dR_dq_dq[i][l];
            }
        }

        // compute derivative of qi's wrt v[j]
        vector<double> dq_dvj;
        dq_dvj.resize(4);
        if(fabs(theta)<EPSILON) {
            for(int i=0; i<4; i++){
                dq_dvj[i] = get_dq_dv_approx(i, j, theta, v);
            }
        }
        else {
            for(int i=0; i<4; i++){
                dq_dvj[i] = get_dq_dv(i, j, theta, v, vhat);
            }
        }

        // compute derivative of qi's wrt v[k]
        vector<double> dq_dvk;
        dq_dvk.resize(4);
        if(fabs(theta)<EPSILON) {
            for(int i=0; i<4; i++){
                dq_dvk[i] = get_dq_dv_approx(i, k, theta, v);
            }
        }
        else {
            for(int i=0; i<4; i++){
                dq_dvk[i] = get_dq_dv(i, k, theta, v, vhat);
            }
        }

        // compute double derivative of qi's wrt v[j] and v[k]
        vector<double> dq_dvj_dvk;
        dq_dvj_dvk.resize(4);
        if(fabs(theta)<EPSILON) {
            for(int i=0; i<4; i++){
                dq_dvj_dvk[i] = get_dq_dv_dv_approx(i, j, k, theta, v);
            }
        }
        else {
            for(int i=0; i<4; i++){
                dq_dvj_dvk[i] = get_dq_dv_dv(i, j, k, theta, vhat);
            }
        }

        // compute the reqd derivative
        Matrix3d mat = Matrix3d::Zero();
        for(int i=0; i<4; i++){
            mat+=dR_dq[i]*dq_dvj_dvk[i];
            Matrix3d localmat = Matrix3d::Zero();
            for(int l=0; l<4; l++)
                localmat+=dR_dq_dq[i][l]*dq_dvk[l];
            mat+=localmat*dq_dvj[i];
        }

        Matrix4d ret = Matrix4d::Zero();
        for(int i=0; i<3; i++){
            for(int j=0; j<3; j++)
                ret(i, j) = mat(i, j);
        }

        return ret;
    }

    void TrfmRotateExpMap::applyGLTransform(renderer::RenderInterface* _ri) const{
		Vector3d v(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue());
		double theta = v.norm();
		Vector3d vhat = Vector3d::Zero();
		if(!isZero(theta)) {
			vhat= v/theta;
			_ri->rotate(vhat, theta * 180 / M_PI);
        }

    }
} // namespace model3d
