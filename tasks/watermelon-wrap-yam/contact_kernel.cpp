// Exact sequential point/OBB projection used by film.py, with an AABB rejection.
// No task phases, success conditions, forces or motion targets enter this code.
#include <cmath>
extern "C" void project_boxes(double* p, long n, const double* transforms,
                              const double* bounds, const double* aabbs,
                              long count, const unsigned char* pinned) {
  for(long b=0;b<count;++b) {
    const double* t=transforms+16*b;
    const double* q=bounds+6*b;
    const double* a=aabbs+6*b;
    for(long i=0;i<n;++i) {
      if(pinned[i]) continue;
      double* v=p+3*i;
      if(v[0]<a[0]-1e-10 || v[1]<a[1]-1e-10 || v[2]<a[2]-1e-10 ||
         v[0]>a[3]+1e-10 || v[1]>a[4]+1e-10 || v[2]>a[5]+1e-10) continue;
      double d[3]={v[0]-t[3],v[1]-t[7],v[2]-t[11]},l[3];
      bool inside=true;
      for(int j=0;j<3;++j) {
        l[j]=d[0]*t[j]+d[1]*t[4+j]+d[2]*t[8+j];
        inside &= l[j]>q[j]-.0004 && l[j]<q[3+j]+.0004;
      }
      if(!inside) continue;
      int axis=0;double best=1e100;
      for(int j=0;j<3;++j) {
        double low=l[j]-q[j]+.0004, high=q[3+j]+.0004-l[j];
        double distance=low<high?low:high;
        if(distance<best) {best=distance;axis=j;}
      }
      double low=l[axis]-q[axis]+.0004, high=q[3+axis]+.0004-l[axis];
      l[axis]=low<high?q[axis]-.0004:q[3+axis]+.0004;
      for(int j=0;j<3;++j) v[j]=t[4*j]*l[0]+t[4*j+1]*l[1]+t[4*j+2]*l[2]+t[4*j+3];
    }
  }
}
