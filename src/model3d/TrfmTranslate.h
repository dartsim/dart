/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef MODEL3D_TRFM_TRANSLATE_H
#define MODEL3D_TRFM_TRANSLATE_H

#include <cassert>
#include "Transformation.h"

namespace model3d {
    class TrfmTranslate : public Transformation {
    public:
        TrfmTranslate(Dof *x, Dof *y, Dof *z, char *_name = NULL);
	
        Eigen::Matrix4d getInvTransform();
        void applyTransform(Eigen::Vector3d&);
        void applyTransform(Eigen::Matrix4d&);
        void applyInvTransform(Eigen::Vector3d&);
        void applyInvTransform(Eigen::Matrix4d&);

		void applyGLTransform(renderer::RenderInterface* _ri) const;
        void evalTransform();
	
        Eigen::Matrix4d getDeriv(const Dof *q);
        void applyDeriv(const Dof* q, Eigen::Vector3d& v);
        void applyDeriv(const Dof* q, Eigen::Matrix4d& m);

        Eigen::Matrix4d getDeriv2(const Dof *q1, const Dof *q2){ return Eigen::Matrix4d(); } // TODO::placeholder to run code
//    void applyDeriv2(Dof* q1, Dof* q2, Eigen::Vector3d& v);
//    void applyDeriv2(Dof* q1, Dof* q2, Eigen::Matrix4d& m);
    };

    class TrfmTranslateX : public Transformation {
    public:
        TrfmTranslateX(Dof *x, char *_name = NULL);
	
        Eigen::Matrix4d getInvTransform();
        void applyTransform(Eigen::Vector3d&);
        void applyTransform(Eigen::Matrix4d&);
        void applyInvTransform(Eigen::Vector3d&);
        void applyInvTransform(Eigen::Matrix4d&);

        void applyGLTransform(renderer::RenderInterface* _ri) const;
        void evalTransform();
	
        Eigen::Matrix4d getDeriv(const Dof *q);
        void applyDeriv(const Dof* q, Eigen::Vector3d& v);
        void applyDeriv(const Dof* q, Eigen::Matrix4d& m);

//    Eigen::Matrix4d getDeriv2(Dof *q1, Dof *q2);
//    void applyDeriv2(Dof* q1, Dof* q2, Eigen::Vector3d& v);
//    void applyDeriv2(Dof* q1, Dof* q2, Eigen::Matrix4d& m);
    };

    class TrfmTranslateY : public Transformation {
    public:
        TrfmTranslateY(Dof *y, char *_name = NULL);
	
        Eigen::Matrix4d getInvTransform();
        void applyTransform(Eigen::Vector3d&);
        void applyTransform(Eigen::Matrix4d&);
        void applyInvTransform(Eigen::Vector3d&);
        void applyInvTransform(Eigen::Matrix4d&);

        void applyGLTransform(renderer::RenderInterface* _ri) const;
        void evalTransform();
	
        Eigen::Matrix4d getDeriv(const Dof *q);
        void applyDeriv(const Dof* q, Eigen::Vector3d& v);
        void applyDeriv(const Dof* q, Eigen::Matrix4d& m);

        //   Eigen::Matrix4d getDeriv2(Dof *q1, Dof *q2);
        //   void applyDeriv2(Dof* q1, Dof* q2, Eigen::Vector3d& v);
        //   void applyDeriv2(Dof* q1, Dof* q2, Eigen::Matrix4d& m);
    };

    class TrfmTranslateZ : public Transformation {
    public:
        TrfmTranslateZ(Dof *z, char *_name = NULL);
	
        Eigen::Matrix4d getInvTransform();
        void applyTransform(Eigen::Vector3d&);
        void applyTransform(Eigen::Matrix4d&);
        void applyInvTransform(Eigen::Vector3d&);
        void applyInvTransform(Eigen::Matrix4d&);

        void applyGLTransform(renderer::RenderInterface* _ri) const;
        void evalTransform();
	
        Eigen::Matrix4d getDeriv(const Dof *q);
        void applyDeriv(const Dof* q, Eigen::Vector3d& v);
        void applyDeriv(const Dof* q, Eigen::Matrix4d& m);

//    Eigen::Matrix4d getDeriv2(Dof *q1, Dof *q2);
//    void applyDeriv2(Dof* q1, Dof* q2, Eigen::Vector3d& v);
//    void applyDeriv2(Dof* q1, Dof* q2, Eigen::Matrix4d& m);
    };
} // namespace model3d

#endif // #ifndef MODEL3D_TRFM_TRANSLATE_H

