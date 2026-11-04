#include "../ImuMath.h"
#include <stdio.h>
#include <stdlib.h>

static int checks=0;
static void require(bool ok,const char *message) { ++checks;if(!ok) { fprintf(stderr,"FAIL: %s\n",message);exit(1); } }
static bool close(float x,float y,float tolerance=.0001f) { return fabsf(x-y)<tolerance; }
static float yaw(const ImuMath &m) { return atan2f(2*(m.q[0]*m.q[3]+m.q[1]*m.q[2]),1-2*(m.q[2]*m.q[2]+m.q[3]*m.q[3]))*57.2957795131f; }
static float roll(const ImuMath &m) { return atan2f(2*(m.q[0]*m.q[1]+m.q[2]*m.q[3]),1-2*(m.q[1]*m.q[1]+m.q[2]*m.q[2]))*57.2957795131f; }
int main() {
  float bias[3]={},level[3]={0,0,ImuMath::G};
  {
    ImuMath m;
    float r=-20*.0174532925199433f,p=30*.0174532925199433f;
    float a[3]={-ImuMath::G*sinf(p),ImuMath::G*cosf(p)*sinf(r),ImuMath::G*cosf(p)*cosf(r)};
    m.initialize(a);
    require(close(roll(m),-20,.001f),"gravity initializes roll sign");
    float recoveredPitch=asinf(2*(m.q[0]*m.q[2]-m.q[3]*m.q[1]))*57.2957795131f;
    require(close(recoveredPitch,30,.001f),"gravity initializes pitch sign");
    float sample[6]={a[0],a[1],a[2],0,0,0};
    for(int i=0;i<1000;++i)m.update(sample,bias,1000000ULL+5000ULL*i);
    require(close(m.position[0],0,.0001f)&&close(m.position[1],0,.0001f)&&close(m.position[2],0,.0001f),"tilted stationary gravity removed");
    require(m.stationary,"tilted stationary ZUPT");
  }
  {
    ImuMath m;m.initialize(level);float sample[6]={0,0,ImuMath::G,0,0,90};
    for(int i=0;i<200;++i)m.update(sample,bias,1000000ULL+5000ULL*i);
    require(close(yaw(m),90,.02f),"positive Z rate rotates body X toward world Y");
    float norm=0;for(float x:m.q)norm+=x*x;
    require(close(norm,1,.00001f),"quaternion normalization");
    require(!m.stationary,"rotating is not stationary");
  }
  {
    ImuMath m;m.initialize(level);
    for(int i=0;i<200;++i) {
      float r=(i+1)*.005f*90*.0174532925199433f;
      float sample[6]={0,ImuMath::G*sinf(r),ImuMath::G*cosf(r),90,0,0};
      m.update(sample,bias,1000000ULL+5000ULL*i);
    }
    require(close(roll(m),90,2),"positive roll tracks consistent rotating gravity");
  }
  {
    ImuMath m;m.initialize(level);float b[3]={5,-3,2};float sample[6]={0,0,ImuMath::G,5,-3,2};
    for(int i=0;i<600;++i)m.update(sample,b,1000000ULL+5000ULL*i);
    require(close(m.filtered[3],0)&&close(m.filtered[4],0)&&close(m.filtered[5],0),"gyro bias subtracted before filtering");
    require(close(yaw(m),0),"bias-corrected attitude does not drift");
  }
  {
    ImuMath m;m.initialize(level);float sample[6]={0,0,ImuMath::G,0,0,0};
    m.update(sample,bias,1000000);sample[0]=1;m.update(sample,bias,1005000);
    require(close(m.filtered[0],.4665119f,.00001f),"20Hz matched-pole lowpass coefficient");
  }
  {
    ImuMath m;m.initialize(level);float sample[6]={0,0,ImuMath::G,0,0,90};
    m.update(sample,bias,1000000);float previousYaw=yaw(m);m.velocity[0]=2;m.position[0]=3;
    require(m.update(sample,bias,1050000),"50ms gap flagged");
    require(close(yaw(m),previousYaw),"attitude not integrated across gap");
    require(close(m.velocity[0],0)&&close(m.position[0],3),"gap clears velocity and preserves position");
    require(!m.update(sample,bias,1055000),"normal integration resumes after gap");
    require(close(yaw(m),previousYaw+.45f,.001f),"only current 5ms integrated after gap");
  }
  {
    ImuMath m;m.initialize(level);float sample[6]={0,0,ImuMath::G,0,0,0};uint64_t t=1000000;
    for(int i=0;i<80;++i,t+=5000)m.update(sample,bias,t);
    require(m.stationary,"rest enables ZUPT after 200ms");
    sample[0]=2;m.update(sample,bias,t);t+=5000;
    require(!m.stationary,"horizontal acceleration exits ZUPT");
    for(int i=0;i<20;++i,t+=5000)m.update(sample,bias,t);
    require(m.position[0]>.001f,"horizontal movement produces positive X position");
    sample[0]=0;for(int i=0;i<200;++i,t+=5000)m.update(sample,bias,t);
    require(m.stationary&&close(m.velocity[0],0),"rest restores zero velocity");
    float before=m.q[0];m.zeroPosition();
    require(close(m.position[0],0)&&close(m.velocity[0],0)&&close(m.q[0],before),"ZERO preserves attitude");
  }
  {
    ImuMath m;m.initialize(level);float sample[6]={0,0,ImuMath::G,0,0,90};uint64_t t=1000000;
    m.update(sample,bias,t);
    for(int i=0;i<200;++i) {t+=(i%2)?7000:3000;m.update(sample,bias,t);}
    require(close(yaw(m),90.45f,.02f),"actual varying dt drives gyro integration");
  }
  {
    ImuMath m;float measured[3]={0,0,1.19f*ImuMath::G};m.initialize(measured);
    const float sample[6]={0,0,1.19f*ImuMath::G,0,0,0};
    float scale=ImuMath::G/sqrtf(measured[2]*measured[2]);
    for(int i=0;i<600;++i)m.update(sample,bias,1000000ULL+5000ULL*i,scale);
    require(close(m.filtered[2],ImuMath::G,.00001f),"single-pose acceleration scale applies to processed SI data");
    require(close(sample[2],1.19f*ImuMath::G,.00001f),"acceleration scale leaves raw SI input intact");
    require(m.stationary&&close(m.position[2],0,.0001f),"scaled stationary sample supports gravity removal and ZUPT");
  }
  printf("PASS: %d mathematical checks\n",checks);
  return 0;
}
