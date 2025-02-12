/// autogenerated analytical inverse kinematics code from ikfast program part of OpenRAVE
/// \author Rosen Diankov
///
/// Licensed under the Apache License, Version 2.0 (the "License");
/// you may not use this file except in compliance with the License.
/// You may obtain a copy of the License at
///     http://www.apache.org/licenses/LICENSE-2.0
/// 
/// Unless required by applicable law or agreed to in writing, software
/// distributed under the License is distributed on an "AS IS" BASIS,
/// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
/// See the License for the specific language governing permissions and
/// limitations under the License.
///
/// ikfast version 0x1000004a generated on 2021-05-01 16:24:19.081168
/// Generated using solver transform6d
/// To compile with gcc:
///     gcc -lstdc++ ik.cpp
/// To compile without any main function as a shared object (might need -llapack):
///     gcc -fPIC -lstdc++ -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY -shared -Wl,-soname,libik.so -o libik.so ik.cpp
#define IKFAST_HAS_LIBRARY
#include "ikfast.h" // found inside share/openrave-X.Y/python/ikfast.h
using namespace ikfast;

// check if the included ikfast version matches what this file was compiled with
#define IKFAST_COMPILE_ASSERT(x) extern int __dummy[(int)x]
IKFAST_COMPILE_ASSERT(IKFAST_VERSION==0x1000004a);

#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <complex>

#ifndef IKFAST_ASSERT
#include <stdexcept>
#include <sstream>
#include <iostream>

#ifdef _MSC_VER
#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __FUNCDNAME__
#endif
#endif

#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __func__
#endif

#define IKFAST_ASSERT(b) { if( !(b) ) { std::stringstream ss; ss << "ikfast exception: " << __FILE__ << ":" << __LINE__ << ": " <<__PRETTY_FUNCTION__ << ": Assertion '" << #b << "' failed"; throw std::runtime_error(ss.str()); } }

#endif

#if defined(_MSC_VER)
#define IKFAST_ALIGNED16(x) __declspec(align(16)) x
#else
#define IKFAST_ALIGNED16(x) x __attribute((aligned(16)))
#endif

#define IK2PI  ((IkReal)6.28318530717959)
#define IKPI  ((IkReal)3.14159265358979)
#define IKPI_2  ((IkReal)1.57079632679490)

#ifdef _MSC_VER
#ifndef isnan
#define isnan _isnan
#endif
#ifndef isinf
#define isinf _isinf
#endif
//#ifndef isfinite
//#define isfinite _isfinite
//#endif
#endif // _MSC_VER

// lapack routines
extern "C" {
  void dgetrf_ (const int* m, const int* n, double* a, const int* lda, int* ipiv, int* info);
  void zgetrf_ (const int* m, const int* n, std::complex<double>* a, const int* lda, int* ipiv, int* info);
  void dgetri_(const int* n, const double* a, const int* lda, int* ipiv, double* work, const int* lwork, int* info);
  void dgesv_ (const int* n, const int* nrhs, double* a, const int* lda, int* ipiv, double* b, const int* ldb, int* info);
  void dgetrs_(const char *trans, const int *n, const int *nrhs, double *a, const int *lda, int *ipiv, double *b, const int *ldb, int *info);
  void dgeev_(const char *jobvl, const char *jobvr, const int *n, double *a, const int *lda, double *wr, double *wi,double *vl, const int *ldvl, double *vr, const int *ldvr, double *work, const int *lwork, int *info);
}

using namespace std; // necessary to get std math routines

#ifdef IKFAST_NAMESPACE
namespace IKFAST_NAMESPACE {
#endif

inline float IKabs(float f) { return fabsf(f); }
inline double IKabs(double f) { return fabs(f); }

inline float IKsqr(float f) { return f*f; }
inline double IKsqr(double f) { return f*f; }

inline float IKlog(float f) { return logf(f); }
inline double IKlog(double f) { return log(f); }

// allows asin and acos to exceed 1. has to be smaller than thresholds used for branch conds and evaluation
#ifndef IKFAST_SINCOS_THRESH
#define IKFAST_SINCOS_THRESH ((IkReal)1e-7)
#endif

// used to check input to atan2 for degenerate cases. has to be smaller than thresholds used for branch conds and evaluation
#ifndef IKFAST_ATAN2_MAGTHRESH
#define IKFAST_ATAN2_MAGTHRESH ((IkReal)1e-7)
#endif

// minimum distance of separate solutions
#ifndef IKFAST_SOLUTION_THRESH
#define IKFAST_SOLUTION_THRESH ((IkReal)1e-6)
#endif

// there are checkpoints in ikfast that are evaluated to make sure they are 0. This threshold speicfies by how much they can deviate
#ifndef IKFAST_EVALCOND_THRESH
#define IKFAST_EVALCOND_THRESH ((IkReal)0.00001)
#endif


inline float IKasin(float f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return float(-IKPI_2);
else if( f >= 1 ) return float(IKPI_2);
return asinf(f);
}
inline double IKasin(double f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return -IKPI_2;
else if( f >= 1 ) return IKPI_2;
return asin(f);
}

// return positive value in [0,y)
inline float IKfmod(float x, float y)
{
    while(x < 0) {
        x += y;
    }
    return fmodf(x,y);
}

// return positive value in [0,y)
inline double IKfmod(double x, double y)
{
    while(x < 0) {
        x += y;
    }
    return fmod(x,y);
}

inline float IKacos(float f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return float(IKPI);
else if( f >= 1 ) return float(0);
return acosf(f);
}
inline double IKacos(double f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return IKPI;
else if( f >= 1 ) return 0;
return acos(f);
}
inline float IKsin(float f) { return sinf(f); }
inline double IKsin(double f) { return sin(f); }
inline float IKcos(float f) { return cosf(f); }
inline double IKcos(double f) { return cos(f); }
inline float IKtan(float f) { return tanf(f); }
inline double IKtan(double f) { return tan(f); }
inline float IKsqrt(float f) { if( f <= 0.0f ) return 0.0f; return sqrtf(f); }
inline double IKsqrt(double f) { if( f <= 0.0 ) return 0.0; return sqrt(f); }
inline float IKatan2Simple(float fy, float fx) {
    return atan2f(fy,fx);
}
inline float IKatan2(float fy, float fx) {
    if( isnan(fy) ) {
        IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return float(IKPI_2);
    }
    else if( isnan(fx) ) {
        return 0;
    }
    return atan2f(fy,fx);
}
inline double IKatan2Simple(double fy, double fx) {
    return atan2(fy,fx);
}
inline double IKatan2(double fy, double fx) {
    if( isnan(fy) ) {
        IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return IKPI_2;
    }
    else if( isnan(fx) ) {
        return 0;
    }
    return atan2(fy,fx);
}

template <typename T>
struct CheckValue
{
    T value;
    bool valid;
};

template <typename T>
inline CheckValue<T> IKatan2WithCheck(T fy, T fx, T epsilon)
{
    CheckValue<T> ret;
    ret.valid = false;
    ret.value = 0;
    if( !isnan(fy) && !isnan(fx) ) {
        if( IKabs(fy) >= IKFAST_ATAN2_MAGTHRESH || IKabs(fx) > IKFAST_ATAN2_MAGTHRESH ) {
            ret.value = IKatan2Simple(fy,fx);
            ret.valid = true;
        }
    }
    return ret;
}

inline float IKsign(float f) {
    if( f > 0 ) {
        return float(1);
    }
    else if( f < 0 ) {
        return float(-1);
    }
    return 0;
}

inline double IKsign(double f) {
    if( f > 0 ) {
        return 1.0;
    }
    else if( f < 0 ) {
        return -1.0;
    }
    return 0;
}

template <typename T>
inline CheckValue<T> IKPowWithIntegerCheck(T f, int n)
{
    CheckValue<T> ret;
    ret.valid = true;
    if( n == 0 ) {
        ret.value = 1.0;
        return ret;
    }
    else if( n == 1 )
    {
        ret.value = f;
        return ret;
    }
    else if( n < 0 )
    {
        if( f == 0 )
        {
            ret.valid = false;
            ret.value = (T)1.0e30;
            return ret;
        }
        if( n == -1 ) {
            ret.value = T(1.0)/f;
            return ret;
        }
    }

    int num = n > 0 ? n : -n;
    if( num == 2 ) {
        ret.value = f*f;
    }
    else if( num == 3 ) {
        ret.value = f*f*f;
    }
    else {
        ret.value = 1.0;
        while(num>0) {
            if( num & 1 ) {
                ret.value *= f;
            }
            num >>= 1;
            f *= f;
        }
    }
    
    if( n < 0 ) {
        ret.value = T(1.0)/ret.value;
    }
    return ret;
}

/// solves the forward kinematics equations.
/// \param pfree is an array specifying the free joints of the chain.
IKFAST_API void ComputeFk(const IkReal* j, IkReal* eetrans, IkReal* eerot) {
IkReal x0,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28,x29,x30,x31,x32,x33,x34,x35,x36,x37,x38,x39,x40,x41,x42,x43,x44,x45,x46,x47;
x0=IKcos(j[0]);
x1=IKcos(j[1]);
x2=IKsin(j[2]);
x3=IKcos(j[2]);
x4=IKsin(j[1]);
x5=IKcos(j[3]);
x6=IKsin(j[3]);
x7=IKcos(j[5]);
x8=IKsin(j[5]);
x9=IKcos(j[4]);
x10=IKsin(j[4]);
x11=IKsin(j[0]);
x12=((0.0996)*x5);
x13=((0.3922)*x3);
x14=((0.0997)*x3);
x15=((1.0)*x11);
x16=((1.0)*x6);
x17=((1.0)*x5);
x18=((1.0)*x9);
x19=((0.0996)*x6);
x20=((0.0997)*x2);
x21=(x0*x9);
x22=(x0*x1);
x23=(x11*x9);
x24=(x2*x4);
x25=(x1*x11);
x26=(x1*x2);
x27=(x3*x4);
x28=(x1*x3);
x29=(x0*x10);
x30=(x10*x15);
x31=((1.0)*x22*x3);
x32=(x15*x28);
x33=((((-1.0)*x24))+x28);
x34=(x26+x27);
x35=(((x0*x27))+((x2*x22)));
x36=((((-1.0)*x31))+((x0*x24)));
x37=(((x2*x25))+((x11*x27)));
x38=((((-1.0)*x32))+((x11*x24)));
x39=(x36*x5);
x40=(((x33*x5))+((x6*(((((-1.0)*x26))+(((-1.0)*x27)))))));
x41=((((-1.0)*x17*x34))+(((-1.0)*x16*x33)));
x42=(((x35*x6))+x39);
x43=(((x38*x5))+((x37*x6)));
x44=(x43*x9);
x45=(x42*x9);
x46=((((-1.0)*x17*x35))+((x16*(((((-1.0)*x31))+(((1.0)*x0*x24)))))));
x47=(((x16*(((((-1.0)*x32))+((x15*x24))))))+(((-1.0)*x17*x37)));
eerot[0]=(((x46*x8))+((x7*(((((-1.0)*x18*x42))+x30)))));
eerot[1]=(((x46*x7))+((x8*(((((-1.0)*x30))+x45)))));
eerot[2]=(((x10*x42))+x23);
IkReal x48=(x0*x4);
eetrans[0]=(((x10*((((x19*x35))+((x12*x36))))))+((x5*((((x14*x48))+((x20*x22))))))+(((0.1333)*x11))+(((-1.0)*x13*x22))+(((-0.425)*x22))+((x6*(((((-1.0)*x20*x48))+((x14*x22))))))+(((0.0996)*x23))+(((0.3922)*x0*x24)));
eerot[3]=(((x47*x8))+((x7*(((((-1.0)*x18*x43))+(((-1.0)*x29)))))));
eerot[4]=(((x47*x7))+((x8*((x44+x29)))));
eerot[5]=(((x10*x43))+(((-1.0)*x0*x18)));
IkReal x49=(x11*x4);
eetrans[1]=(((x6*(((((-1.0)*x20*x49))+((x14*x25))))))+(((-1.0)*x13*x25))+((x5*((((x14*x49))+((x20*x25))))))+(((0.3922)*x11*x24))+(((-0.425)*x25))+(((-0.0996)*x21))+(((-0.1333)*x0))+((x10*((((x19*x37))+((x12*x38)))))));
eerot[6]=(((x40*x8))+((x7*x9*((((x34*x5))+((x33*x6)))))));
eerot[7]=(((x40*x7))+((x41*x8*x9)));
eerot[8]=(x10*x41);
eetrans[2]=((0.1625)+((x10*(((((-1.0)*x12*x34))+(((-1.0)*x19*x33))))))+(((-0.425)*x4))+(((-0.3922)*x26))+((x5*(((((-1.0)*x1*x14))+((x20*x4))))))+((x6*((((x1*x20))+((x14*x4))))))+(((-1.0)*x13*x4)));
}

IKFAST_API int GetNumFreeParameters() { return 0; }
IKFAST_API int* GetFreeParameters() { return NULL; }
IKFAST_API int GetNumJoints() { return 6; }

IKFAST_API int GetIkRealSize() { return sizeof(IkReal); }

IKFAST_API int GetIkType() { return 0x67000001; }

class IKSolver {
public:
IkReal j0,cj0,sj0,htj0,j0mul,j1,cj1,sj1,htj1,j1mul,j2,cj2,sj2,htj2,j2mul,j3,cj3,sj3,htj3,j3mul,j4,cj4,sj4,htj4,j4mul,j5,cj5,sj5,htj5,j5mul,new_r00,r00,rxp0_0,new_r01,r01,rxp0_1,new_r02,r02,rxp0_2,new_r10,r10,rxp1_0,new_r11,r11,rxp1_1,new_r12,r12,rxp1_2,new_r20,r20,rxp2_0,new_r21,r21,rxp2_1,new_r22,r22,rxp2_2,new_px,px,npx,new_py,py,npy,new_pz,pz,npz,pp;
unsigned char _ij0[2], _nj0,_ij1[2], _nj1,_ij2[2], _nj2,_ij3[2], _nj3,_ij4[2], _nj4,_ij5[2], _nj5;

IkReal j100, cj100, sj100;
unsigned char _ij100[2], _nj100;
bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {
j0=numeric_limits<IkReal>::quiet_NaN(); _ij0[0] = -1; _ij0[1] = -1; _nj0 = -1; j1=numeric_limits<IkReal>::quiet_NaN(); _ij1[0] = -1; _ij1[1] = -1; _nj1 = -1; j2=numeric_limits<IkReal>::quiet_NaN(); _ij2[0] = -1; _ij2[1] = -1; _nj2 = -1; j3=numeric_limits<IkReal>::quiet_NaN(); _ij3[0] = -1; _ij3[1] = -1; _nj3 = -1; j4=numeric_limits<IkReal>::quiet_NaN(); _ij4[0] = -1; _ij4[1] = -1; _nj4 = -1; j5=numeric_limits<IkReal>::quiet_NaN(); _ij5[0] = -1; _ij5[1] = -1; _nj5 = -1; 
for(int dummyiter = 0; dummyiter < 1; ++dummyiter) {
    solutions.Clear();
r00 = eerot[0*3+0];
r01 = eerot[0*3+1];
r02 = eerot[0*3+2];
r10 = eerot[1*3+0];
r11 = eerot[1*3+1];
r12 = eerot[1*3+2];
r20 = eerot[2*3+0];
r21 = eerot[2*3+1];
r22 = eerot[2*3+2];
px = eetrans[0]; py = eetrans[1]; pz = eetrans[2];

new_r00=((-1.0)*r00);
new_r01=((-1.0)*r01);
new_r02=((-1.0)*r02);
new_px=((((-1.0)*px))+(((0.0996)*r02)));
new_r10=((-1.0)*r10);
new_r11=((-1.0)*r11);
new_r12=((-1.0)*r12);
new_py=((((0.0996)*r12))+(((-1.0)*py)));
new_r20=r20;
new_r21=r21;
new_r22=r22;
new_pz=((-0.1625)+pz+(((-0.0996)*r22)));
r00 = new_r00; r01 = new_r01; r02 = new_r02; r10 = new_r10; r11 = new_r11; r12 = new_r12; r20 = new_r20; r21 = new_r21; r22 = new_r22; px = new_px; py = new_py; pz = new_pz;
IkReal x50=((1.0)*px);
IkReal x51=((1.0)*pz);
IkReal x52=((1.0)*py);
pp=((px*px)+(py*py)+(pz*pz));
npx=(((px*r00))+((py*r10))+((pz*r20)));
npy=(((px*r01))+((py*r11))+((pz*r21)));
npz=(((px*r02))+((py*r12))+((pz*r22)));
rxp0_0=((((-1.0)*r20*x52))+((pz*r10)));
rxp0_1=(((px*r20))+(((-1.0)*r00*x51)));
rxp0_2=((((-1.0)*r10*x50))+((py*r00)));
rxp1_0=((((-1.0)*r21*x52))+((pz*r11)));
rxp1_1=(((px*r21))+(((-1.0)*r01*x51)));
rxp1_2=((((-1.0)*r11*x50))+((py*r01)));
rxp2_0=(((pz*r12))+(((-1.0)*r22*x52)));
rxp2_1=(((px*r22))+(((-1.0)*r02*x51)));
rxp2_2=((((-1.0)*r12*x50))+((py*r02)));
{
IkReal j0eval[2];
j0eval[0]=((IKabs(px))+(IKabs(py)));
j0eval[1]=((px*px)+(py*py));
if( IKabs(j0eval[0]) < 0.0000010000000000  || IKabs(j0eval[1]) < 0.0000010000000000  )
{
continue; // no branches [j0]

} else
{
{
IkReal j0array[2], cj0array[2], sj0array[2];
bool j0valid[2]={false};
_nj0 = 2;
CheckValue<IkReal> x55 = IKatan2WithCheck(IkReal(py),IkReal(((-1.0)*px)),IKFAST_ATAN2_MAGTHRESH);
if(!x55.valid){
continue;
}
IkReal x53=((1.0)*(x55.value));
if((((px*px)+(py*py))) < -0.00001)
continue;
CheckValue<IkReal> x56=IKPowWithIntegerCheck(IKabs(IKsqrt(((px*px)+(py*py)))),-1);
if(!x56.valid){
continue;
}
if( (((0.1333)*(x56.value))) < -1-IKFAST_SINCOS_THRESH || (((0.1333)*(x56.value))) > 1+IKFAST_SINCOS_THRESH )
    continue;
IkReal x54=IKasin(((0.1333)*(x56.value)));
j0array[0]=(x54+(((-1.0)*x53)));
sj0array[0]=IKsin(j0array[0]);
cj0array[0]=IKcos(j0array[0]);
j0array[1]=((3.14159265358979)+(((-1.0)*x53))+(((-1.0)*x54)));
sj0array[1]=IKsin(j0array[1]);
cj0array[1]=IKcos(j0array[1]);
if( j0array[0] > IKPI )
{
    j0array[0]-=IK2PI;
}
else if( j0array[0] < -IKPI )
{    j0array[0]+=IK2PI;
}
j0valid[0] = true;
if( j0array[1] > IKPI )
{
    j0array[1]-=IK2PI;
}
else if( j0array[1] < -IKPI )
{    j0array[1]+=IK2PI;
}
j0valid[1] = true;
for(int ij0 = 0; ij0 < 2; ++ij0)
{
if( !j0valid[ij0] )
{
    continue;
}
_ij0[0] = ij0; _ij0[1] = -1;
for(int iij0 = ij0+1; iij0 < 2; ++iij0)
{
if( j0valid[iij0] && IKabs(cj0array[ij0]-cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0]-sj0array[iij0]) < IKFAST_SOLUTION_THRESH )
{
    j0valid[iij0]=false; _ij0[1] = iij0; break; 
}
}
j0 = j0array[ij0]; cj0 = cj0array[ij0]; sj0 = sj0array[ij0];

{
IkReal j5array[2], cj5array[2], sj5array[2];
bool j5valid[2]={false};
_nj5 = 2;
IkReal x57=cj0*cj0;
IkReal x58=r01*r01;
IkReal x59=r00*r00;
IkReal x60=((1.0)*sj0);
IkReal x61=((4.0)*x59);
IkReal x62=((8.0)*cj0*sj0);
IkReal x63=((4.0)*x58);
IkReal x64=((4.0)*x57);
CheckValue<IkReal> x69=IKPowWithIntegerCheck(((((-1.0)*r01*x60))+((cj0*r11))),-1);
if(!x69.valid){
continue;
}
IkReal x65=x69.value;
IkReal x66=(cj0*r10*x65);
CheckValue<IkReal> x70=IKPowWithIntegerCheck(((((-1.0)*r01*sj0))+((cj0*r11))),-1);
if(!x70.valid){
continue;
}
IkReal x67=(r00*x60*(x70.value));
if(((((x64*(r11*r11)))+(((-1.0)*r00*r10*x62))+((x64*(r10*r10)))+x61+x63+(((-1.0)*r01*r11*x62))+(((-1.0)*x57*x63))+(((-1.0)*x57*x61)))) < -0.00001)
continue;
IkReal x68=((0.5)*x65*(IKsqrt((((x64*(r11*r11)))+(((-1.0)*r00*r10*x62))+((x64*(r10*r10)))+x61+x63+(((-1.0)*r01*r11*x62))+(((-1.0)*x57*x63))+(((-1.0)*x57*x61))))));
j5array[0]=((2.0)*(atan((x66+x68+(((-1.0)*x67))))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
j5array[1]=((-2.0)*(atan((x67+x68+(((-1.0)*x66))))));
sj5array[1]=IKsin(j5array[1]);
cj5array[1]=IKcos(j5array[1]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
if( j5array[1] > IKPI )
{
    j5array[1]-=IK2PI;
}
else if( j5array[1] < -IKPI )
{    j5array[1]+=IK2PI;
}
j5valid[1] = true;
for(int ij5 = 0; ij5 < 2; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 2; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
htj5 = IKtan(j5/2);

{
IkReal j2array[2], cj2array[2], sj2array[2];
bool j2valid[2]={false};
_nj2 = 2;
cj2array[0]=((-1.026710981792)+(((0.598134205237424)*npx*sj5))+(((2.99967003629601)*pp))+(((0.598134205237424)*cj5*npy)));
if( cj2array[0] >= -1-IKFAST_SINCOS_THRESH && cj2array[0] <= 1+IKFAST_SINCOS_THRESH )
{
    j2valid[0] = j2valid[1] = true;
    j2array[0] = IKacos(cj2array[0]);
    sj2array[0] = IKsin(j2array[0]);
    cj2array[1] = cj2array[0];
    j2array[1] = -j2array[0];
    sj2array[1] = -sj2array[0];
}
else if( isnan(cj2array[0]) )
{
    // probably any value will work
    j2valid[0] = true;
    cj2array[0] = 1; sj2array[0] = 0; j2array[0] = 0;
}
for(int ij2 = 0; ij2 < 2; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 2; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[1];
IkReal x71=(cj0*sj5);
IkReal x72=((0.16949)*py);
IkReal x73=((0.084745)*pp);
IkReal x74=(sj0*sj5);
IkReal x75=(cj0*cj5);
IkReal x76=(cj5*sj0);
IkReal x77=((0.16949)*px);
evalcond[0]=((0.0387814280852)+(((-1.0)*r00*x73*x74))+(((-0.113305)*pp))+(((0.03777248785)*(IKcos(j2))))+(((-1.0)*r01*x73*x76))+(((-1.0)*npx*x71*x72))+(((-1.0)*npy*x72*x75))+((r10*x71*x73))+((npx*x74*x77))+((r11*x73*x75))+((npy*x76*x77)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
IkReal j3eval[2];
j3eval[0]=((-1.00322716501185)+(((-1.0)*cj2)));
j3eval[1]=IKsign(((-533508.003968)+(((-531791.824)*cj2))));
if( IKabs(j3eval[0]) < 0.0000010000000000  || IKabs(j3eval[1]) < 0.0000010000000000  )
{
continue; // no branches [j1, j3, j4]

} else
{
{
IkReal j3array[1], cj3array[1], sj3array[1];
bool j3valid[1]={false};
_nj3 = 1;
IkReal x78=(rxp1_0*sj0);
IkReal x79=(cj0*rxp1_1);
IkReal x80=((3400000.0)*pp);
IkReal x81=((677960.0)*sj2);
IkReal x82=((625637.44)*cj5);
IkReal x83=((677960.0)*cj2);
IkReal x84=(cj5*x83);
IkReal x85=(cj0*rxp0_1*sj5);
IkReal x86=(rxp0_0*sj0*sj5);
CheckValue<IkReal> x87=IKPowWithIntegerCheck(IKsign(((-533508.003968)+(((-531791.824)*cj2)))),-1);
if(!x87.valid){
continue;
}
CheckValue<IkReal> x88 = IKatan2WithCheck(IkReal(((-1136296.963232)+(((-1.0)*x81*x86))+((cj5*x79*x81))+(((-2277308.1)*cj2))+((x81*x85))+((cj2*x80))+(((3137600.0)*pp))+(((-1.0)*cj5*x78*x81))+(((-1133458.0)*(cj2*cj2))))),IkReal((((sj2*x80))+(((-1.0)*x83*x85))+((x78*x84))+((x78*x82))+(((-1231326.388)*sj2))+(((-1.0)*x79*x84))+(((-1.0)*x79*x82))+(((625637.44)*x86))+(((-625637.44)*x85))+(((-1133458.0)*cj2*sj2))+((x83*x86)))),IKFAST_ATAN2_MAGTHRESH);
if(!x88.valid){
continue;
}
j3array[0]=((-1.5707963267949)+(((1.5707963267949)*(x87.value)))+(x88.value));
sj3array[0]=IKsin(j3array[0]);
cj3array[0]=IKcos(j3array[0]);
if( j3array[0] > IKPI )
{
    j3array[0]-=IK2PI;
}
else if( j3array[0] < -IKPI )
{    j3array[0]+=IK2PI;
}
j3valid[0] = true;
for(int ij3 = 0; ij3 < 1; ++ij3)
{
if( !j3valid[ij3] )
{
    continue;
}
_ij3[0] = ij3; _ij3[1] = -1;
for(int iij3 = ij3+1; iij3 < 1; ++iij3)
{
if( j3valid[iij3] && IKabs(cj3array[ij3]-cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3]-sj3array[iij3]) < IKFAST_SOLUTION_THRESH )
{
    j3valid[iij3]=false; _ij3[1] = iij3; break; 
}
}
j3 = j3array[ij3]; cj3 = cj3array[ij3]; sj3 = sj3array[ij3];
{
IkReal evalcond[4];
IkReal x89=IKsin(j3);
IkReal x90=IKcos(j3);
IkReal x91=(cj5*npy);
IkReal x92=(npx*sj5);
IkReal x93=((1.0)*sj0);
IkReal x94=(pp*sj5);
IkReal x95=(cj0*cj5);
IkReal x96=((2.0)*px*sj0);
IkReal x97=((0.425)*x90);
IkReal x98=(cj2*x89);
IkReal x99=((2.0)*cj0*py);
IkReal x100=(sj2*x90);
evalcond[0]=((0.36215482)+(((-0.084745)*x98))+(((-0.084745)*x100))+(((-1.0)*pp))+(((-0.07820468)*x89))+(((0.33337)*cj2)));
evalcond[1]=((0.0997)+(((-1.0)*sj2*x97))+(((-0.425)*x98))+(((-0.3922)*x89))+x91+x92);
evalcond[2]=((((-0.3922)*x90))+(((-1.0)*cj5*rxp1_0*x93))+((cj0*rxp0_1*sj5))+((rxp1_1*x95))+(((-1.0)*cj2*x97))+(((0.425)*sj2*x89))+(((-1.0)*rxp0_0*sj5*x93)));
evalcond[3]=((-0.02658002)+((pp*r11*x95))+(((-1.0)*cj5*pp*r01*x93))+((x92*x96))+(((0.113305)*x98))+(((-1.0)*x91*x99))+(((0.113305)*x100))+((cj0*r10*x94))+(((-1.0)*x92*x99))+(((-1.0)*r00*x93*x94))+(((0.10456052)*x89))+((x91*x96)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
IkReal j1array[1], cj1array[1], sj1array[1];
bool j1valid[1]={false};
_nj1 = 1;
IkReal x101=(cj3*sj2);
IkReal x102=(cj2*sj3);
IkReal x103=(cj2*cj3);
IkReal x104=(sj2*sj3);
IkReal x105=((1.0)*cj5*r21);
IkReal x106=((7.50187546886722)*cj5*rxp1_2);
IkReal x107=((1.0)*r20*sj5);
IkReal x108=((7.50187546886722)*rxp0_2*sj5);
IkReal x109=(sj3*x108);
if( IKabs(((((-1.0)*x104*x108))+(((-1.0)*x104*x106))+(((-1.0)*x101*x105))+(((-1.0)*x101*x107))+((x103*x106))+((x103*x108))+(((-1.0)*x102*x105))+(((-1.0)*x102*x107)))) < IKFAST_ATAN2_MAGTHRESH && IKabs((((cj5*r21*x103))+(((-1.0)*x104*x105))+(((-1.0)*x104*x107))+((r20*sj5*x103))+((x102*x106))+((x102*x108))+((x101*x106))+((x101*x108)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0)*x104*x108))+(((-1.0)*x104*x106))+(((-1.0)*x101*x105))+(((-1.0)*x101*x107))+((x103*x106))+((x103*x108))+(((-1.0)*x102*x105))+(((-1.0)*x102*x107))))+IKsqr((((cj5*r21*x103))+(((-1.0)*x104*x105))+(((-1.0)*x104*x107))+((r20*sj5*x103))+((x102*x106))+((x102*x108))+((x101*x106))+((x101*x108))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j1array[0]=IKatan2(((((-1.0)*x104*x108))+(((-1.0)*x104*x106))+(((-1.0)*x101*x105))+(((-1.0)*x101*x107))+((x103*x106))+((x103*x108))+(((-1.0)*x102*x105))+(((-1.0)*x102*x107))), (((cj5*r21*x103))+(((-1.0)*x104*x105))+(((-1.0)*x104*x107))+((r20*sj5*x103))+((x102*x106))+((x102*x108))+((x101*x106))+((x101*x108))));
sj1array[0]=IKsin(j1array[0]);
cj1array[0]=IKcos(j1array[0]);
if( j1array[0] > IKPI )
{
    j1array[0]-=IK2PI;
}
else if( j1array[0] < -IKPI )
{    j1array[0]+=IK2PI;
}
j1valid[0] = true;
for(int ij1 = 0; ij1 < 1; ++ij1)
{
if( !j1valid[ij1] )
{
    continue;
}
_ij1[0] = ij1; _ij1[1] = -1;
for(int iij1 = ij1+1; iij1 < 1; ++iij1)
{
if( j1valid[iij1] && IKabs(cj1array[ij1]-cj1array[iij1]) < IKFAST_SOLUTION_THRESH && IKabs(sj1array[ij1]-sj1array[iij1]) < IKFAST_SOLUTION_THRESH )
{
    j1valid[iij1]=false; _ij1[1] = iij1; break; 
}
}
j1 = j1array[ij1]; cj1 = cj1array[ij1]; sj1 = sj1array[ij1];
{
IkReal evalcond[8];
IkReal x110=IKsin(j1);
IkReal x111=IKcos(j1);
IkReal x112=((0.0997)*sj3);
IkReal x113=((0.1333)*cj3);
IkReal x114=((0.34227464)*cj3);
IkReal x115=(sj0*sj5);
IkReal x116=((1.0)*r10);
IkReal x117=((2.0)*pz);
IkReal x118=(cj5*npy);
IkReal x119=((0.32661704)*sj3);
IkReal x120=(r20*sj5);
IkReal x121=(npx*sj5);
IkReal x122=((2.0)*py);
IkReal x123=((1.0)*cj5);
IkReal x124=((0.01897536)*cj3);
IkReal x125=((0.0997)*cj3);
IkReal x126=((0.1333)*sj3);
IkReal x127=(r11*sj0);
IkReal x128=(cj0*r01);
IkReal x129=(cj0*px);
IkReal x130=(cj5*r21);
IkReal x131=(cj2*x111);
IkReal x132=(sj2*x111);
IkReal x133=(sj3*x110);
IkReal x134=((0.33337)*x111);
IkReal x135=(sj2*x110);
IkReal x136=(cj2*x110);
IkReal x137=((1.0)*cj0*r00*sj5);
evalcond[0]=(((cj3*x135))+((cj2*x133))+((sj3*x132))+x130+x120+(((-1.0)*cj3*x131)));
evalcond[1]=((((-1.0)*x113*x136))+(((-1.0)*x113*x132))+((cj5*rxp1_2))+(((-1.0)*x126*x131))+((x126*x135))+((rxp0_2*sj5)));
evalcond[2]=((((-1.0)*x123*x127))+(((-1.0)*x123*x128))+((cj3*x136))+((cj3*x132))+((sj3*x131))+(((-1.0)*x115*x116))+(((-1.0)*x137))+(((-1.0)*sj2*x133)));
evalcond[3]=(((x112*x132))+((x112*x136))+(((-1.0)*pz))+((x125*x135))+(((-1.0)*x125*x131))+(((-0.425)*x110))+(((-0.3922)*x132))+(((-0.3922)*x136)));
evalcond[4]=((((-1.0)*rxp0_1*x115))+(((-1.0)*x113*x131))+(((-1.0)*rxp1_1*sj0*x123))+((x113*x135))+((x126*x136))+((x126*x132))+(((-1.0)*cj0*rxp1_0*x123))+(((-1.0)*cj0*rxp0_0*sj5)));
evalcond[5]=(((x112*x131))+(((-1.0)*x112*x135))+x129+((x125*x132))+((x125*x136))+(((-0.425)*x111))+((py*sj0))+(((-0.3922)*x131))+(((0.3922)*x135)));
evalcond[6]=((((-1.0)*x117*x118))+(((0.084745)*x110))+(((-0.33337)*x133))+(((0.07820468)*x136))+(((0.07820468)*x132))+(((-1.0)*cj3*x134))+(((-1.0)*x124*x135))+(((-1.0)*x117*x121))+((pp*x130))+((pp*x120))+(((-1.0)*x114*x131))+(((0.03463296)*sj3*x132))+(((-1.0)*x119*x136)));
evalcond[7]=((((-0.03463296)*sj2*x133))+(((-0.07820468)*x135))+(((-1.0)*sj3*x134))+(((-1.0)*pp*x115*x116))+(((0.084745)*x111))+(((2.0)*x121*x129))+((npx*x115*x122))+(((0.07820468)*x131))+((x114*x136))+(((-1.0)*x124*x132))+(((2.0)*x118*x129))+(((-1.0)*pp*x137))+((sj0*x118*x122))+(((-1.0)*pp*x123*x127))+(((-1.0)*pp*x123*x128))+(((0.33337)*cj3*x110))+(((-1.0)*x119*x131)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
j4array[0]=0;
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(6);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}
}
}
}
}

}

}
}
}
}
}
}
}

}

}
}
return solutions.GetNumSolutions()>0;
}
};


/// solves the inverse kinematics equations.
/// \param pfree is an array specifying the free joints of the chain.
IKFAST_API bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {
IKSolver solver;
return solver.ComputeIk(eetrans,eerot,pfree,solutions);
}

IKFAST_API bool ComputeIk2(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions, void* pOpenRAVEManip) {
IKSolver solver;
return solver.ComputeIk(eetrans,eerot,pfree,solutions);
}

IKFAST_API const char* GetKinematicsHash() { return "<robot:GenericRobot - ur5e (a8c61b631fa1a8b14cdcf470a0304349)>"; }

IKFAST_API const char* GetIkFastVersion() { return "0x1000004a"; }

#ifdef IKFAST_NAMESPACE
} // end namespace
#endif

#ifndef IKFAST_NO_MAIN
#include <stdio.h>
#include <stdlib.h>
#ifdef IKFAST_NAMESPACE
using namespace IKFAST_NAMESPACE;
#endif
int main(int argc, char** argv)
{
    if( argc != 12+GetNumFreeParameters()+1 ) {
        printf("\nUsage: ./ik r00 r01 r02 t0 r10 r11 r12 t1 r20 r21 r22 t2 free0 ...\n\n"
               "Returns the ik solutions given the transformation of the end effector specified by\n"
               "a 3x3 rotation R (rXX), and a 3x1 translation (tX).\n"
               "There are %d free parameters that have to be specified.\n\n",GetNumFreeParameters());
        return 1;
    }

    IkSolutionList<IkReal> solutions;
    std::vector<IkReal> vfree(GetNumFreeParameters());
    IkReal eerot[9],eetrans[3];
    eerot[0] = atof(argv[1]); eerot[1] = atof(argv[2]); eerot[2] = atof(argv[3]); eetrans[0] = atof(argv[4]);
    eerot[3] = atof(argv[5]); eerot[4] = atof(argv[6]); eerot[5] = atof(argv[7]); eetrans[1] = atof(argv[8]);
    eerot[6] = atof(argv[9]); eerot[7] = atof(argv[10]); eerot[8] = atof(argv[11]); eetrans[2] = atof(argv[12]);
    for(std::size_t i = 0; i < vfree.size(); ++i)
        vfree[i] = atof(argv[13+i]);
    bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

    if( !bSuccess ) {
        fprintf(stderr,"Failed to get ik solution\n");
        return -1;
    }

    printf("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
    std::vector<IkReal> solvalues(GetNumJoints());
    for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
        printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        for( std::size_t j = 0; j < solvalues.size(); ++j)
            printf("%.15f, ", solvalues[j]);
        printf("\n");
    }
    return 0;
}

#endif
