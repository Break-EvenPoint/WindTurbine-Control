/*
 * A Bladed DISCON ABI wrapper for ROSCO/OpenFAST.
 * ROSCO remains the feedback controller; this wrapper applies scheduled
 * torque/pitch offsets and a yaw-angle-offset tracking correction.
 * Optional measured BASELINE yaw reference (YAW_REFERENCE_CSV) prevents
 * residual terminal yaw error from the old virtual-yaw baseline.
 *
 * Compile with the SAME architecture (64-bit) as the host OpenFAST binary.
 * Linux: gcc -O2 -std=c11 -shared -fPIC -o libdiscon_offset.so ... -ldl -lm
 * Win:   gcc -O2 -std=c11 -shared -o discon_offset.dll ... -lm
 *
 * Not a validated real-turbine actuator / power-cost model.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#ifdef _WIN32
#  include <windows.h>
#  define EXPORT __declspec(dllexport)
#else
#  include <dlfcn.h>
#  define EXPORT __attribute__((visibility("default")))
#endif

/* OpenFAST / ROSCO Fortran BIND(C), single-precision avrSWAP. */
typedef void (*discon_fn)(float *, int *, const char *, const char *, char *);
typedef struct { double t, tq, pitch_deg, yaw_deg; } Event;
#define MAX_EVENTS 1024
#define MAX_REFERENCE 200000
static Event schedule[MAX_EVENTS];
static size_t event_count=0, event_index=0;
static double ref_time[MAX_REFERENCE], ref_yaw[MAX_REFERENCE];
static size_t ref_count=0;
static discon_fn rosco_discon=NULL;
static int initialized=0;
static int torque_ever_offset=0, pitch_ever_offset=0;
static FILE *trace=NULL;
static double prev_t=0.0, yaw_virtual_base=0.0;
static double last_torque=0.0, last_pitch[3]={0,0,0};
static const double D2R=0.017453292519943295769;
static const double PI=3.14159265358979323846;
static double torque_max=47402.91, torque_slew=15000.0;
static double pitch_max_deg=90.0, pitch_slew_deg=8.0;
static double yaw_rate_deg=0.3, yaw_kp=0.2;

static double clamp(double x, double lo, double hi) {
    return fmax(lo, fmin(hi, x));
}
static double rate_limit(double old, double goal, double rate, double dt) {
    if(dt <= 0) return old;
    return old + clamp(goal-old, -rate*dt, rate*dt);
}
static double wrap_rad(double a) { return atan2(sin(a),cos(a)); }
static double env_double(const char *key, double def, double lo, double hi) {
    const char *s=getenv(key);
    if(!s || !*s) return def;
    char *end=NULL;
    double x=strtod(s,&end);
    if(end==s || *end!='\0' || !isfinite(x) || x<lo || x>hi) {
        fprintf(stderr,"DISCON wrapper: invalid %s: %s\n",key,s); exit(2);
    }
    return x;
}
static void fail_with(float *sw, int *err, char *msg, const char *fmt, ...) {
    char buf[512];
    va_list ap; va_start(ap,fmt);
    vsnprintf(buf,sizeof(buf),fmt,ap); va_end(ap);
    fprintf(stderr,"DISCON wrapper ERROR: %s\n",buf);
    *err=-1;
    int n=(int)sw[48]; /* Fortran record 49 = C index 48 */
    if(msg && n>1 && n<100000) snprintf(msg,(size_t)n,"%s",buf);
}
static int read_schedule(const char *path) {
    FILE *f=fopen(path,"r");
    if(!f) return -1;
    char line[1024];
    int line_no=0;
    while(fgets(line,sizeof(line),f)) {
        line_no++;
        char *p=line;
        while(*p==' ' || *p=='\t') p++;
        if(*p=='#' || *p=='\n' || *p=='\r' || !*p || (*p=='t' && strncmp(p,"time",4)==0)) continue;
        if(event_count>=MAX_EVENTS) { fclose(f); return -2; }
        Event e;
        if(sscanf(p," %lf , %lf , %lf , %lf",&e.t,&e.tq,&e.pitch_deg,&e.yaw_deg)!=4 ||
           !isfinite(e.t) || !isfinite(e.tq) || !isfinite(e.pitch_deg) || !isfinite(e.yaw_deg) ||
           e.t<0 || (event_count>0 && e.t<=schedule[event_count-1].t)) {
            fprintf(stderr,"DISCON wrapper: bad schedule row %d: %s",line_no,line);
            fclose(f); return -3;
        }
        schedule[event_count++]=e;
    }
    fclose(f);
    return 0;
}
static int read_yaw_reference(const char *path) {
    if (!path || !*path) return 0;
    FILE *f=fopen(path,"r");
    if(!f) { fprintf(stderr,"DISCON reference: cannot open %s\n",path); return -1; }
    char line[512];
    ref_count=0;
    while(fgets(line,sizeof(line),f)) {
        char *p=line;
        while(*p==' ' || *p=='\t') p++;
        if(*p=='#' || *p=='\n' || *p=='\r' || *p=='t') continue;
        double t, deg;
        if(sscanf(p," %lf , %lf",&t,&deg)!=2 || !isfinite(t) || !isfinite(deg) ||
           (ref_count>0 && t<=ref_time[ref_count-1]) || ref_count>=MAX_REFERENCE) {
            fclose(f);return -2;
        }
        ref_time[ref_count]=t;
        ref_yaw[ref_count]=deg*D2R;
        ref_count++;
    }
    fclose(f);
    return ref_count>1 ? 0 : -3;
}
static double get_ref_yaw(double t) {
    if(t<=ref_time[0]) return ref_yaw[0];
    if(t>=ref_time[ref_count-1]) return ref_yaw[ref_count-1];
    size_t lo=0,hi=ref_count-1;
    while(hi-lo>1) { size_t m=(lo+hi)/2; if(ref_time[m]<=t)lo=m;else hi=m; }
    double f=(t-ref_time[lo])/(ref_time[hi]-ref_time[lo]);
    return ref_yaw[lo]+f*wrap_rad(ref_yaw[hi]-ref_yaw[lo]);
}
EXPORT int OFFSET_REF_VERSION(void) { return 1; }
static int load_rosco(const char *path) {
#ifdef _WIN32
    HMODULE lib=LoadLibraryA(path);
    if(!lib) return -1;
    rosco_discon=(discon_fn)GetProcAddress(lib,"DISCON");
#else
    void *lib=dlopen(path,RTLD_NOW|RTLD_LOCAL);
    if(!lib) { fprintf(stderr,"dlopen failed: %s\n",dlerror()); return -1; }
    rosco_discon=(discon_fn)dlsym(lib,"DISCON");
#endif
    return rosco_discon ? 0 : -2;
}
static void finalize(void) { if(trace){fclose(trace);trace=NULL;} }
static int initialize(const char *libpath, const char *schedule_path, const char *log_path) {
    if(!libpath || !*libpath) return -1;
    if(load_rosco(libpath)<0) return -2;
    event_count=event_index=0;
    torque_ever_offset=pitch_ever_offset=0;
    if(schedule_path && *schedule_path && read_schedule(schedule_path)<0) return -3;
    if(read_yaw_reference(getenv("YAW_REFERENCE_CSV"))<0) return -5;
    torque_max=env_double("OFFSET_TORQUE_MAX_NM",47402.91,1,1e10);
    torque_slew=env_double("OFFSET_TORQUE_RATE_NM_S",15000,1e-4,1e10);
    pitch_max_deg=env_double("OFFSET_PITCH_MAX_DEG",90,0.1,180);
    pitch_slew_deg=env_double("OFFSET_PITCH_RATE_DEG_S",8,1e-5,360);
    yaw_rate_deg=env_double("OFFSET_YAW_RATE_DEG_S",0.3,1e-6,360);
    yaw_kp=env_double("OFFSET_YAW_KP_S_INV",0.2,1e-5,100);
    if(log_path && *log_path) {
        trace=fopen(log_path,"w");
        if(!trace) return -4;
        fprintf(trace,"time_s,base_torque_nm,command_torque_nm,base_pitch_deg,command_pitch_deg,base_yaw_rate_deg_s,command_yaw_rate_deg_s,yaw_angle_measured_deg,yaw_reference_deg,target_yaw_offset_deg,measured_offset_deg,target_torque_offset_nm,target_pitch_offset_deg\n");
        fflush(trace);
        atexit(finalize);
    }
    initialized=1;
    return 0;
}

EXPORT void DISCON(float *sw, int *err, const char *infile, const char *outname, char *msg) {
    if(!sw || !err) return;
    const int status=(int)lrint(sw[0]);   /* record 1, status 0 on init */
    double t=(double)sw[1];                /* record 2, seconds */
    if(!initialized) {
        int rc=initialize(getenv("ROSCO_REAL_LIBRARY"),getenv("CONTROL_SCHEDULE"),getenv("CONTROL_LOG"));
        if(rc) {fail_with(sw,err,msg,"initialization failed (%d): check library, schedule and log paths",rc);return;}
        prev_t=t;
        yaw_virtual_base=sw[36];         /* record 37 nacelle yaw angle [rad] */
    }
    /* Invoke unmodified ROSCO first (actual feedback remains active). */
    rosco_discon(sw,err,infile,outname,msg);
    if(*err<0) return;
    if(status<0) return;
    const double tq_base=sw[46], pitch_base=sw[44], yaw_base=sw[47];
    const double yaw_meas=sw[36];
    if(!isfinite(tq_base) || !isfinite(pitch_base) || !isfinite(yaw_base)) {
        fail_with(sw,err,msg,"nonfinite baseline ROSCO command at t=%.6f",t);return;
    }
    double dt=fmax(0.0,t-prev_t);
    if(t < prev_t-1e-9) {
        fail_with(sw,err,msg,"time ran backwards (%.6f vs %.6f)",t,prev_t);return;
    }
    yaw_virtual_base += yaw_base*dt;
    while(event_index<event_count && schedule[event_index].t<=t+1e-8) event_index++;
    Event e={0,0,0,0};
    if(event_index>0) e=schedule[event_index-1];
    /* No intervention = exact passthrough. This preserves ROSCO baseline. */
    if(event_index==0) {
        /* Before the first scheduled event, preserve ROSCO exactly and
           synchronize state for a rate-limited first intervention. */
        last_torque=tq_base;
        for(int i=0;i<3;i++) last_pitch[i]=(sw[27]>0.5f)?sw[41+i]:pitch_base;
    }
    if(event_index>0) {
        int first=(t<=prev_t+1e-12 && status==0);
        if(first) {
            last_torque=tq_base;
            for(int i=0;i<3;i++) last_pitch[i]=(sw[27]>0.5f)? sw[41+i] : pitch_base;
        }
        /* Preserve the real ROSCO output when a channel has never been
           offset. After resetting an active offset, rate-limit its return. */
        if(fabs(e.tq)>1e-9) torque_ever_offset=1;
        if(fabs(e.pitch_deg)>1e-9) pitch_ever_offset=1;
        if(torque_ever_offset) {
            double tq_target=clamp(tq_base+e.tq,0.0,torque_max);
            double tq=rate_limit(last_torque,tq_target,torque_slew,dt);
            if(first) tq=clamp(tq_target,0.0,torque_max);
            sw[46]=(float)tq;
            last_torque=tq;
        } else {
            last_torque=tq_base;
        }
        if(sw[27]>0.5f) { /* record 28: individual blade pitch commands 42..44 */
            double sum_deg=0;
            for(int i=0;i<3;i++) {
                const double pbase=sw[41+i];
                if(pitch_ever_offset) {
                    const double ptgt=clamp(pbase+e.pitch_deg*D2R,0.0,pitch_max_deg*D2R);
                    const double pnew=first?ptgt:rate_limit(last_pitch[i],ptgt,pitch_slew_deg*D2R,dt);
                    last_pitch[i]=pnew;
                    sw[41+i]=(float)pnew;
                } else {
                    last_pitch[i]=pbase;
                }
                sum_deg+=sw[41+i]/D2R;
            }
            sw[44]=(float)(sum_deg/3.0*D2R);
        } else { /* record 45: collective blade pitch */
            if(pitch_ever_offset) {
                const double ptgt=clamp(pitch_base+e.pitch_deg*D2R,0.0,pitch_max_deg*D2R);
                const double pnew=first?ptgt:rate_limit(last_pitch[0],ptgt,pitch_slew_deg*D2R,dt);
                sw[44]=(float)pnew;
                last_pitch[0]=pnew;
            } else {
                last_pitch[0]=pitch_base;
            }
        }
        /* ServoDyn expects yaw RATE (rad/s). In reference mode track
           the time-aligned yaw of an ACTUAL baseline OpenFAST trajectory;
           otherwise preserve the original virtual-reference behavior. */
        const double err_yaw=(ref_count>1)
            ? wrap_rad(get_ref_yaw(t)+e.yaw_deg*D2R-yaw_meas)
            : wrap_rad(e.yaw_deg*D2R-wrap_rad(yaw_meas-yaw_virtual_base));
        double ycmd=yaw_base+yaw_kp*err_yaw;
        ycmd=clamp(ycmd,-yaw_rate_deg*D2R,yaw_rate_deg*D2R);
        sw[47]=(float)ycmd;
    }
    if(trace) {
        fprintf(trace,"%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
            t,tq_base,(double)sw[46],pitch_base/D2R,(double)sw[44]/D2R,
            yaw_base/D2R,(double)sw[47]/D2R,yaw_meas/D2R,
            (ref_count>1 ? get_ref_yaw(t) : yaw_virtual_base)/D2R,
            e.yaw_deg,wrap_rad(yaw_meas-(ref_count>1 ? get_ref_yaw(t) : yaw_virtual_base))/D2R,
            e.tq,e.pitch_deg);
        if(status==0 || (int)(t*10)!=(int)(prev_t*10)) fflush(trace);
    }
    prev_t=t;
}
