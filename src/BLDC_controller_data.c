/*
 * File: BLDC_controller_data.c
 *
 * Code generated for Simulink model 'BLDC_controller'.
 *
 * Model version                  : 1.1297
 * Simulink Coder version         : 8.13 (R2017b) 24-Jul-2017
 * C/C++ source code generated on : Sun Mar  6 11:02:11 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */


#include "BLDC_controller.h"

/* Constant parameters (auto storage) */
const ConstP rtConstP = {
  /* Computed Parameter: r_sin_M1_Table
   * Referenced by: '<S52>/r_sin_M1'
   */
  { 8192, 8682, 9162, 9630, 10087, 10531, 10963, 11381, 11786, 12176, 12551,
    12911, 13255, 13583, 13894, 14189, 14466, 14726, 14968, 15191, 15396, 15582,
    15749, 15897, 16026, 16135, 16225, 16294, 16344, 16374, 16384, 16374, 16344,
    16294, 16225, 16135, 16026, 15897, 15749, 15582, 15396, 15191, 14968, 14726,
    14466, 14189, 13894, 13583, 13255, 12911, 12551, 12176, 11786, 11381, 10963,
    10531, 10087, 9630, 9162, 8682, 8192, 7692, 7182, 6664, 6138, 5604, 5063,
    4516, 3964, 3406, 2845, 2280, 1713, 1143, 572, 0, -572, -1143, -1713, -2280,
    -2845, -3406, -3964, -4516, -5063, -5604, -6138, -6664, -7182, -7692, -8192,
    -8682, -9162, -9630, -10087, -10531, -10963, -11381, -11786, -12176, -12551,
    -12911, -13255, -13583, -13894, -14189, -14466, -14726, -14968, -15191,
    -15396, -15582, -15749, -15897, -16026, -16135, -16225, -16294, -16344,
    -16374, -16384, -16374, -16344, -16294, -16225, -16135, -16026, -15897,
    -15749, -15582, -15396, -15191, -14968, -14726, -14466, -14189, -13894,
    -13583, -13255, -12911, -12551, -12176, -11786, -11381, -10963, -10531,
    -10087, -9630, -9162, -8682, -8192, -7692, -7182, -6664, -6138, -5604, -5063,
    -4516, -3964, -3406, -2845, -2280, -1713, -1143, -572, 0, 572, 1143, 1713,
    2280, 2845, 3406, 3964, 4516, 5063, 5604, 6138, 6664, 7182, 7692, 8192 },

  /* Computed Parameter: r_cos_M1_Table
   * Referenced by: '<S52>/r_cos_M1'
   */
  { 14189, 13894, 13583, 13255, 12911, 12551, 12176, 11786, 11381, 10963, 10531,
    10087, 9630, 9162, 8682, 8192, 7692, 7182, 6664, 6138, 5604, 5063, 4516,
    3964, 3406, 2845, 2280, 1713, 1143, 572, 0, -572, -1143, -1713, -2280, -2845,
    -3406, -3964, -4516, -5063, -5604, -6138, -6664, -7182, -7692, -8192, -8682,
    -9162, -9630, -10087, -10531, -10963, -11381, -11786, -12176, -12551, -12911,
    -13255, -13583, -13894, -14189, -14466, -14726, -14968, -15191, -15396,
    -15582, -15749, -15897, -16026, -16135, -16225, -16294, -16344, -16374,
    -16384, -16374, -16344, -16294, -16225, -16135, -16026, -15897, -15749,
    -15582, -15396, -15191, -14968, -14726, -14466, -14189, -13894, -13583,
    -13255, -12911, -12551, -12176, -11786, -11381, -10963, -10531, -10087,
    -9630, -9162, -8682, -8192, -7692, -7182, -6664, -6138, -5604, -5063, -4516,
    -3964, -3406, -2845, -2280, -1713, -1143, -572, 0, 572, 1143, 1713, 2280,
    2845, 3406, 3964, 4516, 5063, 5604, 6138, 6664, 7182, 7692, 8192, 8682, 9162,
    9630, 10087, 10531, 10963, 11381, 11786, 12176, 12551, 12911, 13255, 13583,
    13894, 14189, 14466, 14726, 14968, 15191, 15396, 15582, 15749, 15897, 16026,
    16135, 16225, 16294, 16344, 16374, 16384, 16374, 16344, 16294, 16225, 16135,
    16026, 15897, 15749, 15582, 15396, 15191, 14968, 14726, 14466, 14189 },

  /* Computed Parameter: r_sin3PhaA_M1_Table
   * Referenced by: '<S96>/r_sin3PhaA_M1'
   */
  { -13091, -13634, -14126, -14565, -14953, -15289, -15577, -15816, -16009,
    -16159, -16269, -16340, -16377, -16383, -16362, -16317, -16253, -16172,
    -16079, -15977, -15870, -15762, -15656, -15555, -15461, -15377, -15306,
    -15248, -15206, -15180, -15172, -15180, -15206, -15248, -15306, -15377,
    -15461, -15555, -15656, -15762, -15870, -15977, -16079, -16172, -16253,
    -16317, -16362, -16383, -16377, -16340, -16269, -16159, -16009, -15816,
    -15577, -15289, -14953, -14565, -14126, -13634, -13091, -12496, -11849,
    -11154, -10411, -9623, -8791, -7921, -7014, -6075, -5107, -4115, -3104,
    -2077, -1041, 0, 1041, 2077, 3104, 4115, 5107, 6075, 7014, 7921, 8791, 9623,
    10411, 11154, 11849, 12496, 13091, 13634, 14126, 14565, 14953, 15289, 15577,
    15816, 16009, 16159, 16269, 16340, 16377, 16383, 16362, 16317, 16253, 16172,
    16079, 15977, 15870, 15762, 15656, 15555, 15461, 15377, 15306, 15248, 15206,
    15180, 15172, 15180, 15206, 15248, 15306, 15377, 15461, 15555, 15656, 15762,
    15870, 15977, 16079, 16172, 16253, 16317, 16362, 16383, 16377, 16340, 16269,
    16159, 16009, 15816, 15577, 15289, 14953, 14565, 14126, 13634, 13091, 12496,
    11849, 11154, 10411, 9623, 8791, 7921, 7014, 6075, 5107, 4115, 3104, 2077,
    1041, 0, -1041, -2077, -3104, -4115, -5107, -6075, -7014, -7921, -8791,
    -9623, -10411, -11154, -11849, -12496, -13091 },

  /* Computed Parameter: r_sin3PhaB_M1_Table
   * Referenced by: '<S96>/r_sin3PhaB_M1'
   */
  { 15172, 15180, 15206, 15248, 15306, 15377, 15461, 15555, 15656, 15762, 15870,
    15977, 16079, 16172, 16253, 16317, 16362, 16383, 16377, 16340, 16269, 16159,
    16009, 15816, 15577, 15289, 14953, 14565, 14126, 13634, 13091, 12496, 11849,
    11154, 10411, 9623, 8791, 7921, 7014, 6075, 5107, 4115, 3104, 2077, 1041, 0,
    -1041, -2077, -3104, -4115, -5107, -6075, -7014, -7921, -8791, -9623, -10411,
    -11154, -11849, -12496, -13091, -13634, -14126, -14565, -14953, -15289,
    -15577, -15816, -16009, -16159, -16269, -16340, -16377, -16383, -16362,
    -16317, -16253, -16172, -16079, -15977, -15870, -15762, -15656, -15555,
    -15461, -15377, -15306, -15248, -15206, -15180, -15172, -15180, -15206,
    -15248, -15306, -15377, -15461, -15555, -15656, -15762, -15870, -15977,
    -16079, -16172, -16253, -16317, -16362, -16383, -16377, -16340, -16269,
    -16159, -16009, -15816, -15577, -15289, -14953, -14565, -14126, -13634,
    -13091, -12496, -11849, -11154, -10411, -9623, -8791, -7921, -7014, -6075,
    -5107, -4115, -3104, -2077, -1041, 0, 1041, 2077, 3104, 4115, 5107, 6075,
    7014, 7921, 8791, 9623, 10411, 11154, 11849, 12496, 13091, 13634, 14126,
    14565, 14953, 15289, 15577, 15816, 16009, 16159, 16269, 16340, 16377, 16383,
    16362, 16317, 16253, 16172, 16079, 15977, 15870, 15762, 15656, 15555, 15461,
    15377, 15306, 15248, 15206, 15180, 15172 },

  /* Computed Parameter: r_sin3PhaC_M1_Table
   * Referenced by: '<S96>/r_sin3PhaC_M1'
   */
  { -13091, -12496, -11849, -11154, -10411, -9623, -8791, -7921, -7014, -6075,
    -5107, -4115, -3104, -2077, -1041, 0, 1041, 2077, 3104, 4115, 5107, 6075,
    7014, 7921, 8791, 9623, 10411, 11154, 11849, 12496, 13091, 13634, 14126,
    14565, 14953, 15289, 15577, 15816, 16009, 16159, 16269, 16340, 16377, 16383,
    16362, 16317, 16253, 16172, 16079, 15977, 15870, 15762, 15656, 15555, 15461,
    15377, 15306, 15248, 15206, 15180, 15172, 15180, 15206, 15248, 15306, 15377,
    15461, 15555, 15656, 15762, 15870, 15977, 16079, 16172, 16253, 16317, 16362,
    16383, 16377, 16340, 16269, 16159, 16009, 15816, 15577, 15289, 14953, 14565,
    14126, 13634, 13091, 12496, 11849, 11154, 10411, 9623, 8791, 7921, 7014,
    6075, 5107, 4115, 3104, 2077, 1041, 0, -1041, -2077, -3104, -4115, -5107,
    -6075, -7014, -7921, -8791, -9623, -10411, -11154, -11849, -12496, -13091,
    -13634, -14126, -14565, -14953, -15289, -15577, -15816, -16009, -16159,
    -16269, -16340, -16377, -16383, -16362, -16317, -16253, -16172, -16079,
    -15977, -15870, -15762, -15656, -15555, -15461, -15377, -15306, -15248,
    -15206, -15180, -15172, -15180, -15206, -15248, -15306, -15377, -15461,
    -15555, -15656, -15762, -15870, -15977, -16079, -16172, -16253, -16317,
    -16362, -16383, -16377, -16340, -16269, -16159, -16009, -15816, -15577,
    -15289, -14953, -14565, -14126, -13634, -13091 },

  /* Computed Parameter: iq_maxSca_M1_Table
   * Referenced by: '<S80>/iq_maxSca_M1'
   */
  { 65535U, 65523U, 65484U, 65418U, 65326U, 65207U, 65062U, 64890U, 64691U,
    64465U, 64211U, 63930U, 63620U, 63281U, 62913U, 62516U, 62088U, 61630U,
    61140U, 60618U, 60062U, 59473U, 58848U, 58187U, 57489U, 56752U, 55974U,
    55155U, 54291U, 53381U, 52422U, 51413U, 50349U, 49227U, 48043U, 46792U,
    45470U, 44069U, 42581U, 40997U, 39307U, 37494U, 35541U, 33422U, 31105U,
    28540U, 25655U, 22323U, 18304U, 12974U },

  /* Computed Parameter: z_commutMap_M1_table
   * Referenced by: '<S94>/z_commutMap_M1'
   */
  { -1, 1, 0, -1, 0, 1, 0, -1, 1, 1, -1, 0, 1, 0, -1, 0, 1, -1 },

  /* Computed Parameter: vec_hallToPos_Value
   * Referenced by: '<S11>/vec_hallToPos'
   */
  { 0, 2, 0, 1, 4, 3, 5, 0 }
};

P rtP = {
  /* Variable: dV_openRate
   * Referenced by: '<S37>/dV_openRate'
   */
  12288,

  /* Variable: dz_cntTrnsDetHi
   * Referenced by: '<S17>/dz_cntTrnsDet'
   */
  40,

  /* Variable: dz_cntTrnsDetLo
   * Referenced by: '<S17>/dz_cntTrnsDet'
   */
  20,

  /* Variable: n_cruiseMotTgt
   * Referenced by: '<S61>/n_cruiseMotTgt'
   */
  0,

  /* Variable: z_maxCntRst
   * Referenced by:
   *   '<S13>/Counter'
   *   '<S13>/z_maxCntRst'
   *   '<S13>/z_maxCntRst2'
   *   '<S13>/UnitDelay3'
   *   '<S17>/z_counter'
   */
  2000,

  /* Variable: cf_speedCoef
   * Referenced by: '<S17>/cf_speedCoef'
   */
  10667U,

  /* Variable: t_errDequal
   * Referenced by: '<S20>/t_errDequal'
   */
  9600U,

  /* Variable: t_errQual
   * Referenced by: '<S20>/t_errQual'
   */
  1280U,

  /* Variable: Vd_max
   * Referenced by:
   *   '<S36>/Vd_max'
   *   '<S80>/Vd_max1'
   */
  14400,

  /* Variable: Vq_max_M1
   * Referenced by: '<S80>/Vq_max_M1'
   */
  { 14400, 14396, 14386, 14368, 14343, 14311, 14271, 14225, 14171, 14109, 14040,
    13963, 13879, 13786, 13685, 13576, 13459, 13333, 13198, 13053, 12900, 12736,
    12562, 12377, 12181, 11973, 11753, 11520, 11273, 11011, 10733, 10438, 10124,
    9790, 9433, 9051, 8640, 8196, 7713, 7184, 6597, 5935, 5170, 4245, 3019, 0 },

  /* Variable: Vq_max_XA
   * Referenced by: '<S80>/Vq_max_XA'
   */
  { 0, 320, 640, 960, 1280, 1600, 1920, 2240, 2560, 2880, 3200, 3520, 3840, 4160,
    4480, 4800, 5120, 5440, 5760, 6080, 6400, 6720, 7040, 7360, 7680, 8000, 8320,
    8640, 8960, 9280, 9600, 9920, 10240, 10560, 10880, 11200, 11520, 11840,
    12160, 12480, 12800, 13120, 13440, 13760, 14080, 14400 },

  /* Variable: a_phaAdvMax
   * Referenced by: '<S42>/a_phaAdvMax'
   */
  400,

  /* Variable: i_max
   * Referenced by:
   *   '<S36>/i_max'
   *   '<S80>/i_max'
   */
  12000,

  /* Variable: id_fieldWeakMax
   * Referenced by: '<S42>/id_fieldWeakMax'
   */
  4000,

  /* Variable: n_commAcvLo
   * Referenced by: '<S13>/n_commDeacv'
   */
  240,

  /* Variable: n_commDeacvHi
   * Referenced by: '<S13>/n_commDeacv'
   */
  480,

  /* Variable: n_fieldWeakAuthHi
   * Referenced by: '<S42>/n_fieldWeakAuthHi'
   */
  6400,

  /* Variable: n_fieldWeakAuthLo
   * Referenced by: '<S42>/n_fieldWeakAuthLo'
   */
  4800,

  /* Variable: n_max
   * Referenced by:
   *   '<S36>/n_max'
   *   '<S80>/n_max1'
   */
  16000,

  /* Variable: n_stdStillDet
   * Referenced by: '<S13>/n_stdStillDet'
   */
  48,

  /* Variable: r_errInpTgtThres
   * Referenced by: '<S20>/r_errInpTgtThres'
   */
  9600,

  /* Variable: r_fieldWeakHi
   * Referenced by: '<S42>/r_fieldWeakHi'
   */
  16000,

  /* Variable: r_fieldWeakLo
   * Referenced by: '<S42>/r_fieldWeakLo'
   */
  12000,

  /* Variable: cf_KbLimProt
   * Referenced by:
   *   '<S82>/cf_KbLimProt'
   *   '<S83>/cf_KbLimProt'
   */
  768U,

  /* Variable: cf_idKp
   * Referenced by: '<S63>/cf_idKp1'
   */
  819U,

  /* Variable: cf_iqKp
   * Referenced by: '<S62>/cf_iqKp'
   */
  1229U,

  /* Variable: cf_nKp
   * Referenced by: '<S61>/cf_nKp'
   */
  4833U,

  /* Variable: cf_currFilt
   * Referenced by: '<S50>/cf_currFilt'
   */
  7864U,

  /* Variable: cf_idKi
   * Referenced by: '<S63>/cf_idKi1'
   */
  737U,

  /* Variable: cf_iqKi
   * Referenced by: '<S62>/cf_iqKi'
   */
  1229U,

  /* Variable: cf_iqKiLimProt
   * Referenced by:
   *   '<S81>/cf_iqKiLimProt'
   *   '<S83>/cf_iqKiLimProt'
   */
  737U,

  /* Variable: cf_nKi
   * Referenced by: '<S61>/cf_nKi'
   */
  251U,

  /* Variable: cf_nKiLimProt
   * Referenced by:
   *   '<S82>/cf_nKiLimProt'
   *   '<S83>/cf_nKiLimProt'
   */
  246U,

  /* Variable: n_polePairs
   * Referenced by: '<S15>/n_polePairs'
   */
  15U,

  /* Variable: z_ctrlTypSel
   * Referenced by: '<S1>/z_ctrlTypSel'
   */
  2U,

  /* Variable: z_selPhaCurMeasABC
   * Referenced by: '<S49>/z_selPhaCurMeasABC'
   */
  0U,

  /* Variable: b_angleMeasEna
   * Referenced by:
   *   '<S3>/b_angleMeasEna'
   *   '<S13>/b_angleMeasEna'
   */
  0,

  /* Variable: b_cruiseCtrlEna
   * Referenced by: '<S1>/b_cruiseCtrlEna'
   */
  0,

  /* Variable: b_diagEna
   * Referenced by: '<S4>/b_diagEna'
   */
  1,

  /* Variable: b_fieldWeakEna
   * Referenced by:
   *   '<S6>/b_fieldWeakEna'
   *   '<S97>/b_fieldWeakEna'
   */
  0
};                                     /* Modifiable parameters */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
