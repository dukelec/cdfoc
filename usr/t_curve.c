/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#include <math.h>
#include "cd_utils.h"
#include "cd_debug.h"


int t_curve_plan(float v_s, float v_e, float s, float v_unsign, float a_unsign,
                float *s_seg, float *t_seg, float *a_seg)
{
    float v_c = sign(s) * v_unsign;
    float a_s = (v_c >= v_s ? 1 : -1) * a_unsign;
    float a_e = (v_e >= v_c ? 1 : -1) * a_unsign;
    
    float s_s = (powf(v_c, 2) - powf(v_s, 2)) / (a_s * 2);
    float s_e = (powf(v_e, 2) - powf(v_c, 2)) / (a_e * 2);
    
    //fprintf(stderr, "s_s: %f, s_e: %f\n", s_s, s_e);
    
    if (sign(s) * s >= sign(s) * (s_s + s_e)) {
        float s_c = s - s_s - s_e;
        float t_c = s_c / v_c;
        //fprintf(stderr, "s_c: %f, t_c: %f\n", s_c, t_c);
        
        s_seg[0] = s_s;
        s_seg[1] = s_c;
        s_seg[2] = s_e;
        a_seg[0] = a_s;
        a_seg[1] = 0;
        a_seg[2] = a_e;
        t_seg[0] = (v_c - v_s) / a_s;
        t_seg[1] = t_c;
        t_seg[2] = (v_e - v_c) / a_e;
        
        if (t_c >= 0)
            return 0;
    }
    
    if (a_s != a_e) {
        float v_c_real = sign(v_c) * sqrtf(a_s * s + (powf(v_s, 2) + powf(v_e, 2)) / 2.0f);
        //fprintf(stderr, "v_c_real: %f\n", v_c_real);
        
        s_seg[0] = (powf(v_c_real, 2) - powf(v_s, 2)) / (a_s * 2);
        s_seg[1] = 0;
        s_seg[2] = (powf(v_e, 2) - powf(v_c_real, 2)) / (a_e * 2);
        a_seg[0] = a_s;
        a_seg[1] = 0;
        a_seg[2] = a_e;
        t_seg[0] = (v_c_real - v_s) / a_s;
        t_seg[1] = 0;
        t_seg[2] = (v_e - v_c_real) / a_e;
        
        // avoid situation like: v_s = 0, v_e = -10; s = -40; v_unsign = 10; a_unsign = 1; period = 0.1;
        if (t_seg[2] >= 0)
            return 0;
    }
    
    {
        float a_urgent = (powf(v_e, 2) - powf(v_s, 2)) / (s * 2);
        //fprintf(stderr, "a_urgent: %f\n", a_urgent);
        
        s_seg[0] = 0;
        s_seg[1] = 0;
        s_seg[2] = (powf(v_e, 2) - powf(v_s, 2)) / (a_urgent * 2);
        a_seg[0] = 0;
        a_seg[1] = 0;
        a_seg[2] = a_urgent;
        t_seg[0] = 0;
        t_seg[1] = 0;
        t_seg[2] = (v_e - v_s) / a_urgent;
        
        if (t_seg[2] >= 0)
            return 1;
    }
    
    return -1;
}

int t_curve_step(const float *s_seg, const float *t_seg, const float *a_seg,
        float v_s, float t_cur, float *v_cur, float *s_cur)
{
    float v_c_real = v_s + a_seg[0] * t_seg[0];
    
    if (t_cur > t_seg[0] + t_seg[1] + t_seg[2]) {
        //fprintf(stderr, "t_cur out of range %f %f\n", t_cur, t_seg[0] + t_seg[1] + t_seg[2]);
        return -1;
    }
    
    if (t_cur >= t_seg[0] + t_seg[1]) {
        float t_e_cur = t_cur - t_seg[0] - t_seg[1];
        float s_e_cur = a_seg[2] * powf(t_e_cur, 2) / 2 + v_c_real * t_e_cur;
        
        *v_cur = v_c_real + a_seg[2] * t_e_cur;
        *s_cur = s_seg[0] + s_seg[1] + s_e_cur;
    } else if (t_cur >= t_seg[0]) {
        float t_c_cur = t_cur - t_seg[0];
        float s_c_cur = v_c_real * t_c_cur;
        
        *v_cur = v_c_real;
        *s_cur = s_seg[0] + s_c_cur;
    } else {
        float t_s_cur = t_cur;
        float s_s_cur = a_seg[0] * powf(t_s_cur, 2) / 2 + v_s * t_s_cur;
        
        *v_cur = v_s + a_seg[0] * t_s_cur;
        *s_cur = s_s_cur;
    }
    
    return 0;
}

#if 0
int main(void)
{
    int i = 0;
    float period = 1.3;
    
    float v_s = 0, v_e = -5;
    float s = -170;
    float v_unsign = 10;
    float a_unsign = 0.9;
    
    float s_seg[3], t_seg[3], a_seg[3];
    int ret = t_curve_plan(v_s, v_e, s, v_unsign, a_unsign,  s_seg, t_seg, a_seg);
    
    fprintf(stderr, "ret: %d, s_seg: %f %f %f, t_seg: %f %f %f, a_seg: %f %f %f\n",
            ret, s_seg[0], s_seg[1], s_seg[2], t_seg[0], t_seg[1], t_seg[2], a_seg[0], a_seg[1], a_seg[2]);
    if (t_seg[0] < 0 || t_seg[1] < 0 || t_seg[2] < 0)
        fprintf(stderr, "t_seg < 0 !!!\n");
    
    int sum_step = ((t_seg[0] + t_seg[1] + t_seg[2]) / period + 0.5f);
    
    printf("[\n");
    for (i = 0; i < sum_step; i++) {
        float v_cur, s_cur;
        t_curve_step(s_seg, t_seg, a_seg,  v_s, i * period,  &v_cur, &s_cur);
        printf("    [%.3f, %.3f, %.3f, %.3f, %.3f],\n",
                s, sign(s) * v_unsign, v_cur, s_cur, s - s_cur);
    }
    printf("]\n");
}
#endif
