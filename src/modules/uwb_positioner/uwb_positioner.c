
/**
 * UWB Positionning System: Initialisation with Analytical Solution
 *
 * @min 0
 * @max 1
 * @value 0 Disable
 * @value 1 Enable
 * @group UWB Positioner
 */
PARAM_DEFINE_INT32(UWB_INIT_ANA, 0);

/**
 * UWB Positionning System: Estimation Mode
 *
 * @min 0
 * @max 2
 * @value 0 Trilateration Analytical Solution
 * @value 1 UKF
 * @group UWB Positioner
 */
PARAM_DEFINE_INT32(UWB_EST_UKF, 1);

/**
 * UWB Positionning System: Output Data described in NED Frame
 *
 * @boolean
 * @group UWB Positioner
 */
PARAM_DEFINE_INT32(UWB_OUTPUT_NED, 1);

/**
 * UWB Positionning System: Indoor?Outdoor
 *
 * @boolean
 * @group UWB Positioner
 */
PARAM_DEFINE_INT32(UWB_OUTDOOR_FLY, 1);


/**
 * UWB Positionning System: Test Mode
 *
 * @boolean
 * @group UWB Positioner
 */
PARAM_DEFINE_INT32(UWB_TEST, 0);




/* ************** OUTDOOR ****************** */

/**
 * UWB Positionning System: Tag_0 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */

PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_0_X, 0.1665313);

/**
 * UWB Positionning System: Tag_0 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_0_Y, 5.5063);

/**
 * UWB Positionning System: Tag_0 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_0_Z, 1.3942);

/**
 * UWB Positionning System: Tag_1 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_1_X, 5.3303);

/**
 * UWB Positionning System: Tag_1 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_1_Y, 0.030688);

/**
 * UWB Positionning System: Tag_1 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_1_Z, 2.0750);

/**
 * UWB Positionning System: Tag_2 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_2_X, -0.394);

/**
 * UWB Positionning System: Tag_2 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_2_Y, 0.2819677);

/**
 * UWB Positionning System: Tag_2 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_2_Z, 3.8663);

/**
 * UWB Positionning System: Tag_3 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_3_X, -0.0147865);

/**
 * UWB Positionning System: Tag_3 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_3_Y, -5.3608);

/**
 * UWB Positionning System: Tag_3 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_3_Z, 1.4274);

/**
 * UWB Positionning System: Tag_4 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_4_X, -2.8381);

/**
 * UWB Positionning System: Tag_4 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_4_Y, -2.0786);

/**
 * UWB Positionning System: Tag_4 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_4_Z, 1.2520);

/**
 * UWB Positionning System: Tag_5 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_5_X, -4.2857);

/**
 * UWB Positionning System: Tag_5 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_5_Y, 0.0683559);

/**
 * UWB Positionning System: Tag_5 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_5_Z, 2.0544);

/**
 * UWB Positionning System: Tag_6 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_6_X, 2.716);

/**
 * UWB Positionning System: Tag_6 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_6_Y, 2.978);

/**
 * UWB Positionning System: Tag_6 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_6_Z, 1.2523);

/**
 * UWB Positionning System: Tag_7 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_7_X, 2.7164);

/**
 * UWB Positionning System: Tag_7 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_7_Y, -4.1178);

/**
 * UWB Positionning System: Tag_7 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_7_Z, 0.4806959);

/**
 * UWB Positionning System: Tag_8 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_8_X, 0.0);

/**
 * UWB Positionning System: Tag_8 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_8_Y, 0.0);

/**
 * UWB Positionning System: Tag_8 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_8_Z, 0.0);

/**
 * UWB Positionning System: Tag_9 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_9_X, 0.0);

/**
 * UWB Positionning System: Tag_9 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_9_Y, 0.0);

/**
 * UWB Positionning System: Tag_9 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUTD_9_Z, 0.0);

/**
 * UWB Positionning System: Tag_10 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUT_10_X, 0.0);

/**
 * UWB Positionning System: Tag_10 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUT_10_Y, 0.0);

/**
 * UWB Positionning System: Tag_10 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUT_10_Z, 0.0);

/**
 * UWB Positionning System: Tag_11 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUT_11_X, 0.0);

/**
 * UWB Positionning System: Tag_11 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUT_11_Y, 0.0);

/**
 * UWB Positionning System: Tag_11 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_OUT_11_Z, 0.0);





/* ************** INDOOR ****************** */

/*
 * UWB Positionning System: Tag_0 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */

PARAM_DEFINE_FLOAT(UWB_TAG_IND_0_X, 2.27831);

/*
 * UWB Positionning System: Tag_0 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_0_Y, -2.45657);

/*
 * UWB Positionning System: Tag_0 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_0_Z, 2.46174);

/*
 * UWB Positionning System: Tag_1 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_1_X, -2.55815);

/*
 * UWB Positionning System: Tag_1 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_1_Y, -2.73888);

/*
 * UWB Positionning System: Tag_1 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_1_Z, 1.95311);

/*
 * UWB Positionning System: Tag_2 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_2_X, -1.80823);

/*
 * UWB Positionning System: Tag_2 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_2_Y, 2.82529);

/*
 * UWB Positionning System: Tag_2 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_2_Z, 2.7342);

/*
 * UWB Positionning System: Tag_3 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_3_X, 2.43927);

/*
 * UWB Positionning System: Tag_3 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_3_Y, 2.53581);

/*
 * UWB Positionning System: Tag_3 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_3_Z, 2.46312);

/*
 * UWB Positionning System: Tag_4 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_4_X, 0.0306982);

/*
 * UWB Positionning System: Tag_4 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_4_Y, -2.51589);

/*
 * UWB Positionning System: Tag_4 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_4_Z, 1.26173);

/*
 * UWB Positionning System: Tag_5 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_5_X, -2.72107);

/*
 * UWB Positionning System: Tag_5 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_5_Y, -0.27203);

/*
 * UWB Positionning System: Tag_5 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_5_Z, 1.34782);

/*
 * UWB Positionning System: Tag_6 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_6_X, 0.339571);

/*
 * UWB Positionning System: Tag_6 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_6_Y, 2.67917);

/*
 * UWB Positionning System: Tag_6 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_6_Z, 1.25228);

/*
 * UWB Positionning System: Tag_7 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_7_X, 2.41325);

/*
 * UWB Positionning System: Tag_7 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_7_Y, -0.0245378);

/*
 * UWB Positionning System: Tag_7 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_7_Z, 0.469889);

/*
 * UWB Positionning System: Tag_8 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_8_X, 0.0);

/*
 * UWB Positionning System: Tag_8 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_8_Y, 0.0);

/*
 * UWB Positionning System: Tag_8 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_8_Z, 0.0);

/*
 * UWB Positionning System: Tag_9 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_9_X, 0.0);

/*
 * UWB Positionning System: Tag_9 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_9_Y, 0.0);

/*
 * UWB Positionning System: Tag_9 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_9_Z, 0.0);

/*
 * UWB Positionning System: Tag_10 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_10_X, 0.0);

/*
 * UWB Positionning System: Tag_10 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_10_Y, 0.0);

/*
 * UWB Positionning System: Tag_10 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_10_Z, 0.0);

/*
 * UWB Positionning System: Tag_11 position X
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_11_X, 0.0);

/*
 * UWB Positionning System: Tag_11 position Y
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_11_Y, 0.0);

/*
 * UWB Positionning System: Tag_11 position Z
 *
 * @unit m
 * @decimal 3
 * @group UWB Positioner
 */
PARAM_DEFINE_FLOAT(UWB_TAG_IND_11_Z, 0.0);

