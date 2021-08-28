
/**
 * UWB Indoor Positionning System: Module Start
 *
 * @reboot_required true
 * @min 0
 * @max 1
 * @group Sensors
 * @value 0 Disabled
 * @value 1 Enabled
 */
PARAM_DEFINE_INT32(SENS_EN_UWB, 1);

/**
 * UWB Indoor Positionning System: Initialisation with Analytical Solution
 *
 * @min 0
 * @max 1
 * @value 0 Disable
 * @value 1 Enable
 * @group UWB Indoor Position
 */
PARAM_DEFINE_INT32(UWB_INIT_ANA, 1);

/**
 * UWB Indoor Positionning System: Estimation Mode
 *
 * @min 0
 * @max 2
 * @value 0 UKF
 * @value 1 Trilateration Analytical Solution
 * @group UWB Indoor Position
 */
PARAM_DEFINE_INT32(UWB_EST_MODE, 0);

/**
 * UWB Indoor Positionning System: Output Data described in NED Frame
 *
 * @boolean
 * @group UWB Indoor Position
 */
PARAM_DEFINE_INT32(UWB_OUTPUT_NED, 0);

/**
 * UWB Indoor Positionning System: Test Mode
 *
 * @boolean
 * @group UWB Indoor Position
 */
PARAM_DEFINE_INT32(UWB_TEST, 0);




/* ************** OUTDOOR ****************** */

/**
 * UWB Indoor Positionning System: Tag_0 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */

PARAM_DEFINE_FLOAT(UWB_TAG_0_X, 0.1665313);

/**
 * UWB Indoor Positionning System: Tag_0 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_0_Y, 5.5063);

/**
 * UWB Indoor Positionning System: Tag_0 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_0_Z, 1.3942);

/**
 * UWB Indoor Positionning System: Tag_1 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_1_X, 5.3303);

/**
 * UWB Indoor Positionning System: Tag_1 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_1_Y, 0.030688);

/**
 * UWB Indoor Positionning System: Tag_1 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_1_Z, 2.0750);

/**
 * UWB Indoor Positionning System: Tag_2 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_2_X, -0.394);

/**
 * UWB Indoor Positionning System: Tag_2 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_2_Y, 0.2819677);

/**
 * UWB Indoor Positionning System: Tag_2 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_2_Z, 3.8663);

/**
 * UWB Indoor Positionning System: Tag_3 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_3_X, -0.0147865);

/**
 * UWB Indoor Positionning System: Tag_3 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_3_Y, -5.3608);

/**
 * UWB Indoor Positionning System: Tag_3 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_3_Z, 1.4274);

/**
 * UWB Indoor Positionning System: Tag_4 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_4_X, -2.8381);

/**
 * UWB Indoor Positionning System: Tag_4 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_4_Y, -2.0786);

/**
 * UWB Indoor Positionning System: Tag_4 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_4_Z, 1.2520);

/**
 * UWB Indoor Positionning System: Tag_5 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_5_X, -4.2857);

/**
 * UWB Indoor Positionning System: Tag_5 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_5_Y, 0.0683559);

/**
 * UWB Indoor Positionning System: Tag_5 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_5_Z, 2.0544);

/**
 * UWB Indoor Positionning System: Tag_6 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_6_X, 2.716);

/**
 * UWB Indoor Positionning System: Tag_6 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_6_Y, 2.978);

/**
 * UWB Indoor Positionning System: Tag_6 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_6_Z, 1.2523);

/**
 * UWB Indoor Positionning System: Tag_7 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_7_X, 2.7164);

/**
 * UWB Indoor Positionning System: Tag_7 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_7_Y, -4.1178);

/**
 * UWB Indoor Positionning System: Tag_7 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_7_Z, 0.4806959);

/**
 * UWB Indoor Positionning System: Tag_8 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_8_X, 0.0);

/**
 * UWB Indoor Positionning System: Tag_8 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_8_Y, 0.0);

/**
 * UWB Indoor Positionning System: Tag_8 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_8_Z, 0.0);

/**
 * UWB Indoor Positionning System: Tag_9 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_9_X, 0.0);

/**
 * UWB Indoor Positionning System: Tag_9 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_9_Y, 0.0);

/**
 * UWB Indoor Positionning System: Tag_9 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_9_Z, 0.0);

/**
 * UWB Indoor Positionning System: Tag_10 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_10_X, 0.0);

/**
 * UWB Indoor Positionning System: Tag_10 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_10_Y, 0.0);

/**
 * UWB Indoor Positionning System: Tag_10 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_10_Z, 0.0);

/**
 * UWB Indoor Positionning System: Tag_11 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_11_X, 0.0);

/**
 * UWB Indoor Positionning System: Tag_11 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_11_Y, 0.0);

/**
 * UWB Indoor Positionning System: Tag_11 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Indoor Position
 */
PARAM_DEFINE_FLOAT(UWB_TAG_11_Z, 0.0);


/* ************** INDOOR ****************** */

/*/***/
/* * UWB Indoor Positionning System: Tag_0 position X*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/

/*PARAM_DEFINE_FLOAT(UWB_TAG_0_X, -2.03891);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_0 position Y*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_0_Y, 2.202);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_0 position Z*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_0_Z, 1.42737);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_1 position X*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_1_X, 2.26821);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_1 position Y*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_1_Y, 0.367897);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_1 position Z*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_1_Z, 2.55851);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_2 position X*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_2_X, 1.12156);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_2 position Y*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_2_Y, -2.53193);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_2 position Z*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_2_Z, 1.69995);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_3 position X*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_3_X, -1.95372);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_3 position Y*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_3_Y, 0.3793);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_3 position Z*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_3_Z, 2.48569);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_4 position X*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_4_X, 2.0476);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_4 position Y*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_4_Y, 2.30709);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_4 position Z*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_4_Z, 1.31057);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_5 position X*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_5_X, 0.107978);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_5 position Y*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_5_Y, 2.56309);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_5 position Z*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_5_Z, 1.45993);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_6 position X*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_6_X, -2.04012);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_6 position Y*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_6_Y, -1.46311);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_6 position Z*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_6_Z, 1.2817);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_7 position X*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_7_X, -0.563168);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_7 position Y*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_7_Y, -2.70575);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_7 position Z*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_7_Z, 1.68344);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_8 position X*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_8_X, 0.0);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_8 position Y*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_8_Y, 0.0);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_8 position Z*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_8_Z, 0.0);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_9 position X*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_9_X, 0.0);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_9 position Y*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_9_Y, 0.0);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_9 position Z*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_9_Z, 0.0);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_10 position X*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_10_X, 0.0);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_10 position Y*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_10_Y, 0.0);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_10 position Z*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_10_Z, 0.0);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_11 position X*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_11_X, 0.0);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_11 position Y*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_11_Y, 0.0);*/

/*/***/
/* * UWB Indoor Positionning System: Tag_11 position Z*/
/* **/
/* * @unit m*/
/* * @decimal 3*/
/* * @group UWB Indoor Position*/
/* */*/
/*PARAM_DEFINE_FLOAT(UWB_TAG_11_Z, 0.0);*/

