# Blackbox description file for Bebop2 (firmware 4.0.6)

All data are provided as double (8 bytes), you may convert them to their true type.

|	NAME	|	True type	|	Address \| 10	(	\|16	) |	Description	
|	---	|	---	|	---			|	---	|
|	BLDC_temperature_degC	|	INT16	|	0	(	0	)|		|
|	RCarpet	|	FLOAT	|	8	(	8	)|		|
|	RVisionX	|	FLOAT	|	16	(	10	)|	??	|
|	RVisionY	|	FLOAT	|	24	(	18	)|	??	|
|	RVisionZ	|	FLOAT	|	32	(	20	)|	??	|
|	acc_NED_REF_x	|	FLOAT	|	40	(	28	)|		|
|	acc_NED_REF_y	|	FLOAT	|	48	(	30	)|		|
|	acc_NED_REF_z	|	FLOAT	|	56	(	38	)|		|
|	acc_bias_x_m_s2	|	FLOAT	|	64	(	40	)|		|
|	acc_bias_y_m_s2	|	FLOAT	|	72	(	48	)|		|
|	acc_bias_z_m_s2	|	FLOAT	|	80	(	50	)|		|
|	acc_x_EST_m_s2	|	FLOAT	|	88	(	58	)|		|
|	acc_y_EST_m_s2	|	FLOAT	|	96	(	60	)|		|
|	acc_z_EST_m_s2	|	FLOAT	|	104	(	68	)|		|
|	airspeed_body_x_m_s	|	FLOAT	|	112	(	70	)|	??	|
|	airspeed_body_y_m_s	|	FLOAT	|	120	(	78	)|	//	|
|	airspeed_body_z_m_s	|	FLOAT	|	128	(	80	)|	//	|
|	angular_acc_ref_x	|	FLOAT	|	136	(	88	)|	?? 50, -50	|
|	angular_acc_ref_y	|	FLOAT	|	144	(	90	)|	//	|
|	angular_acc_ref_z	|	FLOAT	|	152	(	98	)|	//	|
|	battery_compensated_V	|	FLOAT	|	160	(	A0	)|		|
|	battery_filt	|	FLOAT	|	168	(	A8	)|		|
|	battery_percent	|	FLOAT	|	176	(	B0	)|		|
|	battery_raw_V	|	FLOAT	|	184	(	B8	)|		|
|	biais_pression_m	|	FLOAT	|	192	(	C0	)|		|
|	camera_tilt	|	FLOAT	|	200	(	C8	)|		|
|	controller_state	|	UINT32	|	208	(	D0	)|		|
|	covALT	|	FLOAT	|	216	(	D8	)|		|
|	covBP	|	FLOAT	|	224	(	E0	)|		|
|	covBX	|	FLOAT	|	232	(	E8	)|		|
|	covBY	|	FLOAT	|	240	(	F0	)|		|
|	covBZ	|	FLOAT	|	248	(	F8	)|		|
|	covDX	|	FLOAT	|	256	(	100	)|		|
|	covDY	|	FLOAT	|	264	(	108	)|		|
|	covDZ	|	FLOAT	|	272	(	110	)|		|
|	covPHI	|	FLOAT	|	280	(	118	)|		|
|	covPSI	|	FLOAT	|	288	(	120	)|		|
|	covTHETA	|	FLOAT	|	296	(	128	)|		|
|	covU	|	FLOAT	|	304	(	130	)|		|
|	covV	|	FLOAT	|	312	(	138	)|		|
|	covW	|	FLOAT	|	320	(	140	)|		|
|	covWE	|	FLOAT	|	328	(	148	)|		|
|	covWN	|	FLOAT	|	336	(	150	)|		|
|	covX	|	FLOAT	|	344	(	158	)|		|
|	covY	|	FLOAT	|	352	(	160	)|		|
|	debugStateHeightVision	|	INT32	|	360	(	168	)|		|
|	estimatorGlobalSensorFlags	|	UINT16	|	368	(	170	)|		|
|	estimator_psi_fused_rad	|	FLOAT	|	376	(	178	)|		|
|	fmCfg_azimuth	|	FLOAT	|	384	(	180	)|		|
|	fmCfg_azimuthIsDefault	|	UINT8	|	392	(	188	)|		|
|	fmCfg_distance	|	FLOAT	|	400	(	190	)|		|
|	fmCfg_distanceIsDefault	|	UINT8	|	408	(	198	)|		|
|	fmCfg_elevation	|	FLOAT	|	416	(	1A0	)|		|
|	fmCfg_elevationIsDefault	|	UINT8	|	424	(	1A8	)|		|
|	fmCfg_relativeCourseIsLocked	|	BOOL	|	432	(	1B0	)|		|
|	fmCfg_revolutionNbIsDefault	|	UINT8	|	440	(	1B8	)|		|
|	fmCfg_revolutionNumber	|	UINT8	|	448	(	1C0	)|		|
|	fmCfg_speed	|	FLOAT	|	456	(	1C8	)|		|
|	fmCfg_speedIsDefault	|	UINT8	|	464	(	1D0	)|		|
|	fmCfg_type	|	UINT32	|	472	(	1D8	)|		|
|	fmCfg_verticalDistance	|	FLOAT	|	480	(	1E0	)|		|
|	fmCfg_verticalDistanceIsDefault	|	UINT8	|	488	(	1E8	)|		|
|	follow_me_criteria	|	UINT16	|	496	(	1F0	)|		|
|	gpsDataOk	|	BOOL	|	504	(	1F8	)|		|
|	gpsDeviationPostionErrorAlt_m	|	FLOAT	|	512	(	200	)|		|
|	gpsDeviationPostionErrorLat_m	|	FLOAT	|	520	(	208	)|		|
|	gpsDeviationPostionErrorLong_m	|	FLOAT	|	528	(	210	)|		|
|	gpsDeviationSpeedError	|	FLOAT	|	536	(	218	)|		|
|	gpsEstSpeed_m_s	|	FLOAT	|	544	(	220	)|		|
|	gpsLatitudeRelative_m	|	FLOAT	|	552	(	228	)|		|
|	gpsLongitudeRelative_m	|	FLOAT	|	560	(	230	)|		|
|	gpsNorthSpeed_m_s	|	FLOAT	|	568	(	238	)|		|
|	gpsSpeedX	|	FLOAT	|	576	(	240	)|		|
|	gpsSpeedY	|	FLOAT	|	584	(	248	)|		|
|	gpsSpeedZ	|	FLOAT	|	592	(	250	)|		|
|	gpsUpSpeed_m_s	|	FLOAT	|	600	(	258	)|	------------------- v	|
|	gyro_bias_x_rad_s	|	FLOAT	|	608	(	260	)|	~OK BUT MAYBE NOT IMPORTANT	|
|	gyro_bias_y_rad_s	|	FLOAT	|	616	(	268	)|	//	|
|	gyro_bias_z_rad_s	|	FLOAT	|	624	(	270	)|	//	|
|	gyro_filt_x_rad_s	|	FLOAT	|	632	(	278	)|	GOOD VALUES (probably filtered with magnetometer)	|
|	gyro_filt_y_rad_s	|	FLOAT	|	640	(	280	)|	//	|
|	gyro_filt_z_rad_s	|	FLOAT	|	648	(	288	)|	//	|
|	gyro_unbias_x_rad_s	|	FLOAT	|	656	(	290	)|	NOT WORKING	|
|	gyro_unbias_y_rad_s	|	FLOAT	|	664	(	298	)|	//	|
|	gyro_unbias_z_rad_s	|	FLOAT	|	672	(	2A0	)|	//	|
|	heading_magneto_rad	|	FLOAT	|	680	(	2A8	)|		|
|	height_EST_m	|	FLOAT	|	688	(	2B0	)|		|
|	height_INPUT_m	|	FLOAT	|	696	(	2B8	)|		|
|	height_REF_m	|	FLOAT	|	704	(	2C0	)|		|
|	height_corrected	|	FLOAT	|	712	(	2C8	)|	NOT WORKING	|
|	height_ground_m	|	FLOAT	|	720	(	2D0	)|	??  (-0.2;0.5)	|
|	height_vision_m	|	FLOAT	|	728	(	2D8	)|	SEEMS TO BE OK (see sensor_sonic_height)	|
|	lynx_change_of_scale	|	FLOAT	|	736	(	2E0	)|		|
|	lynx_confidence_index	|	FLOAT	|	744	(	2E8	)|		|
|	lynx_frame_timestamp	|	FLOAT	|	752	(	2F0	)|		|
|	lynx_is_lost	|	BOOL	|	760	(	2F8	)|		|
|	lynx_is_new_selection	|	FLOAT	|	768	(	300	)|		|
|	lynx_measurement_timestamp	|	FLOAT	|	776	(	308	)|		|
|	lynx_pan_measurement	|	FLOAT	|	784	(	310	)|		|
|	lynx_tilt_measurement	|	FLOAT	|	792	(	318	)|		|
|	magneto_bias_x	|	FLOAT	|	800	(	320	)|	Probably important ?	|
|	magneto_bias_y	|	FLOAT	|	808	(	328	)|	//	|
|	magneto_bias_z	|	FLOAT	|	816	(	330	)|	//	|
|	magneto_calibration_state	|	INT32	|	824	(	338	)|		|
|	magneto_radius	|	FLOAT	|	832	(	340	)|		|
|	motor1_error	|	UINT8	|	840	(	348	)|		|
|	motor1_obs_speed_rpm	|	UINT16	|	848	(	350	)|		|
|	motor1_status	|	UINT8	|	856	(	358	)|		|
|	motor2_error	|	UINT8	|	864	(	360	)|		|
|	motor2_obs_speed_rpm	|	UINT16	|	872	(	368	)|		|
|	motor2_status	|	UINT8	|	880	(	370	)|		|
|	motor3_error	|	UINT8	|	888	(	378	)|		|
|	motor3_obs_speed_rpm	|	UINT16	|	896	(	380	)|		|
|	motor3_status	|	UINT8	|	904	(	388	)|		|
|	motor4_error	|	UINT8	|	912	(	390	)|		|
|	motor4_obs_speed_rpm	|	UINT16	|	920	(	398	)|		|
|	motor4_status	|	UINT8	|	928	(	3A0	)|		|
|	motor_cmd_1_rpm	|	FLOAT	|	936	(	3A8	)|		|
|	motor_cmd_2_rpm	|	FLOAT	|	944	(	3B0	)|		|
|	motor_cmd_3_rpm	|	FLOAT	|	952	(	3B8	)|		|
|	motor_cmd_4_rpm	|	FLOAT	|	960	(	3C0	)|		|
|	motor_cmd_ff	|	FLOAT	|	968	(	3C8	)|		|
|	motor_cmd_height	|	FLOAT	|	976	(	3D0	)|		|
|	new_flat_ground	|	BOOL	|	984	(	3D8	)|		|
|	new_lynx_detection	|	BOOL	|	992	(	3E0	)|		|
|	new_user_GPS_data	|	BOOL	|	1000	(	3E8	)|		|
|	new_user_baro_data	|	BOOL	|	1008	(	3F0	)|		|
|	p_EST_rad_s	|	FLOAT	|	1016	(	3F8	)|		|
|	p_INPUT_rad_s	|	FLOAT	|	1024	(	400	)|		|
|	p_REF_rad_s	|	FLOAT	|	1032	(	408	)|		|
|	phi_EST_rad	|	FLOAT	|	1040	(	410	)|		|
|	phi_INPUT_rad	|	FLOAT	|	1048	(	418	)|		|
|	phi_REF_rad	|	FLOAT	|	1056	(	420	)|		|
|	position_EST_x	|	FLOAT	|	1064	(	428	)|	TODO CHECK	|
|	position_EST_y	|	FLOAT	|	1072	(	430	)|	//	|
|	position_EST_z	|	FLOAT	|	1080	(	438	)|	//	|
|	position_INPUT_x	|	FLOAT	|	1088	(	440	)|	TODO CHECK	|
|	position_INPUT_y	|	FLOAT	|	1096	(	448	)|	//	|
|	position_INPUT_z	|	FLOAT	|	1104	(	450	)|	//	|
|	position_REF_x	|	FLOAT	|	1112	(	458	)|	TODO CHECK	|
|	position_REF_y	|	FLOAT	|	1120	(	460	)|	//	|
|	position_REF_z	|	FLOAT	|	1128	(	468	)|	//	|
|	pressureAltitude_comp_m	|	FLOAT	|	1136	(	470	)|		|
|	pressureAltitude_filt_comp_m	|	FLOAT	|	1144	(	478	)|		|
|	pressureAltitude_filt_m	|	FLOAT	|	1152	(	480	)|		|
|	pressureAltitude_m	|	FLOAT	|	1160	(	488	)|		|
|	psi_EST_rad	|	FLOAT	|	1168	(	490	)|		|
|	psi_INPUT_rad	|	FLOAT	|	1176	(	498	)|		|
|	psi_REF_rad	|	FLOAT	|	1184	(	4A0	)|		|
|	q_EST_rad_s	|	FLOAT	|	1192	(	4A8	)|		|
|	q_INPUT_rad_s	|	FLOAT	|	1200	(	4B0	)|		|
|	q_REF_rad_s	|	FLOAT	|	1208	(	4B8	)|		|
|	r_EST_rad_s	|	FLOAT	|	1216	(	4C0	)|		|
|	r_INPUT_rad_s	|	FLOAT	|	1224	(	4C8	)|		|
|	r_REF_rad_s	|	FLOAT	|	1232	(	4D0	)|		|
|	sensor_acc_raw_x_m_s2	|	FLOAT	|	1240	(	4D8	)|	ok	|
|	sensor_acc_raw_y_m_s2	|	FLOAT	|	1248	(	4E0	)|	ok	|
|	sensor_acc_raw_z_m_s2	|	FLOAT	|	1256	(	4E8	)|	ok	|
|	sensor_barometer_temperature_degC	|	DOUBLE	|	1264	(	4F0	)|		|
|	sensor_gps_accuracy	|	FLOAT	|	1272	(	4F8	)|		|
|	sensor_gps_altitude_m	|	DOUBLE	|	1280	(	500	)|	important ?	|
|	sensor_gps_bearing_deg	|	FLOAT	|	1288	(	508	)|	//	|
|	sensor_gps_latitude_deg	|	DOUBLE	|	1296	(	510	)|	//	|
|	sensor_gps_longitude_deg	|	DOUBLE	|	1304	(	518	)|	//	|
|	sensor_gps_num_svs	|	INT32	|	1312	(	520	)|		|
|	sensor_gps_speed_m_s	|	FLOAT	|	1320	(	528	)|		|
|	sensor_gps_sv_used_in_fix	|	INT32	|	1328	(	530	)|		|
|	sensor_gps_timestamp	|	INT64	|	1336	(	538	)|		|
|	sensor_gps_used_in_fix_mask	|	UINT32	|	1344	(	540	)|		|
|	sensor_gyro_raw_x_rad_s	|	FLOAT	|	1352	(	548	)|	ok	|
|	sensor_gyro_raw_y_rad_s	|	FLOAT	|	1360	(	550	)|	ok	|
|	sensor_gyro_raw_z_rad_s	|	FLOAT	|	1368	(	558	)|	ok	|
|	sensor_imu_obs_temperature_degC	|	FLOAT	|	1376	(	560	)|		|
|	sensor_imu_ref_temperature_degC	|	FLOAT	|	1384	(	568	)|		|
|	sensor_mag_raw_x_mG	|	FLOAT	|	1392	(	570	)|	GUESS IT’S OK	|
|	sensor_mag_raw_y_mG	|	FLOAT	|	1400	(	578	)|	//	|
|	sensor_mag_raw_z_mG	|	FLOAT	|	1408	(	580	)|	//	|
|	sensor_pressure_Pa	|	FLOAT	|	1416	(	588	)|	??	|
|	sensor_ultrasound_height_m	|	FLOAT	|	1424	(	590	)|	ok	|
|	sensor_ultrasound_id	|	INT32	|	1432	(	598	)|		|
|	sensor_ultrasound_mode	|	UINT8	|	1440	(	5A0	)|		|
|	sensor_vision_speed_x_m_s	|	FLOAT	|	1448	(	5A8	)|	NOT SO GOOD 	|
|	sensor_vision_speed_y_m_s	|	FLOAT	|	1456	(	5B0	)|	//	|
|	sensor_vision_speed_z_m_s	|	FLOAT	|	1464	(	5B8	)|	MAYBE NOT WORKING	|
|	speedNEDx	|	FLOAT	|	1472	(	5C0	)|	Using referential at the reception of the command ?	|
|	speedNEDy	|	FLOAT	|	1480	(	5C8	)|	//	|
|	speedNEDz	|	FLOAT	|	1488	(	5D0	)|	//	|
|	speed_NED_REF_x	|	FLOAT	|	1496	(	5D8	)|	?? (probably used for ^ )	|
|	speed_NED_REF_y	|	FLOAT	|	1504	(	5E0	)|	//	|
|	speed_NED_REF_z	|	FLOAT	|	1512	(	5E8	)|	//	|
|	speed_body_x_m_s	|	FLOAT	|	1520	(	5F0	)|	SPEED	|
|	speed_body_y_m_s	|	FLOAT	|	1528	(	5F8	)|	//	|
|	speed_body_z_m_s	|	FLOAT	|	1536	(	600	)|	//	|
|	speed_horiz_x_m_s	|	FLOAT	|	1544	(	608	)|	maybe speed ?	|
|	speed_horiz_y_m_s	|	FLOAT	|	1552	(	610	)|	//	|
|	speed_horiz_z_m_s	|	FLOAT	|	1560	(	618	)|	//	|
|	speed_z_INPUT_m_s	|	FLOAT	|	1568	(	620	)|		|
|	speed_z_REF_m_s	|	FLOAT	|	1576	(	628	)|		|
|	theta_EST_rad	|	FLOAT	|	1584	(	630	)|		|
|	theta_INPUT_rad	|	FLOAT	|	1592	(	638	)|		|
|	theta_REF_rad	|	FLOAT	|	1600	(	640	)|		|
|	time_us	|	INT64	|	1608	(	648	)|	Time since boot (us)	|
|	user_GPS_phone_timestamp	|	FLOAT	|	1616	(	650	)|		|
|	user_GPS_timestamp	|	FLOAT	|	1624	(	658	)|		|
|	user_accuracy_GPS_x	|	FLOAT	|	1632	(	660	)|		|
|	user_accuracy_GPS_y	|	FLOAT	|	1640	(	668	)|		|
|	user_accuracy_GPS_z	|	FLOAT	|	1648	(	670	)|		|
|	user_baro_altitude	|	FLOAT	|	1656	(	678	)|		|
|	user_baro_phone_timestamp	|	FLOAT	|	1664	(	680	)|		|
|	user_baro_pressure	|	FLOAT	|	1672	(	688	)|		|
|	user_baro_pressure_filt	|	FLOAT	|	1680	(	690	)|		|
|	user_baro_timestamp	|	FLOAT	|	1688	(	698	)|		|
|	user_estimation_state	|	INT32	|	1696	(	6A0	)|		|
|	user_flat_ground_timestamp	|	FLOAT	|	1704	(	6A8	)|		|
|	user_position_GPS_x	|	FLOAT	|	1712	(	6B0	)|		|
|	user_position_GPS_y	|	FLOAT	|	1720	(	6B8	)|		|
|	user_position_GPS_z	|	FLOAT	|	1728	(	6C0	)|		|
|	user_position_cov_x	|	FLOAT	|	1736	(	6C8	)|		|
|	user_position_cov_xy	|	FLOAT	|	1744	(	6D0	)|		|
|	user_position_cov_xz	|	FLOAT	|	1752	(	6D8	)|		|
|	user_position_cov_y	|	FLOAT	|	1760	(	6E0	)|		|
|	user_position_cov_yz	|	FLOAT	|	1768	(	6E8	)|		|
|	user_position_cov_z	|	FLOAT	|	1776	(	6F0	)|		|
|	user_position_x	|	FLOAT	|	1784	(	6F8	)|	NOT WORKING (GPS?)	|
|	user_position_y	|	FLOAT	|	1792	(	700	)|	//	|
|	user_position_z	|	FLOAT	|	1800	(	708	)|	//	|
|	user_speed_GPS_x	|	FLOAT	|	1808	(	710	)|		|
|	user_speed_GPS_y	|	FLOAT	|	1816	(	718	)|		|
|	user_speed_GPS_z	|	FLOAT	|	1824	(	720	)|		|
|	user_speed_NED_cov_x	|	FLOAT	|	1832	(	728	)|		|
|	user_speed_NED_cov_y	|	FLOAT	|	1840	(	730	)|		|
|	user_speed_NED_cov_z	|	FLOAT	|	1848	(	738	)|		|
|	user_speed_NED_x	|	FLOAT	|	1856	(	740	)|	NOT WORKING (always 0)	|
|	user_speed_NED_y	|	FLOAT	|	1864	(	748	)|	//	|
|	user_speed_NED_z	|	FLOAT	|	1872	(	750	)|	//	|
|	vertical_model_Cz	|	FLOAT	|	1880	(	758	)|		|
|	vertical_model_Gamma	|	FLOAT	|	1888	(	760	)|		|
|	vision_indicator	|	FLOAT	|	1896	(	768	)|	??	|
|	windE	|	FLOAT	|	1904	(	770	)|	MAYBE ?	|
|	windN	|	FLOAT	|	1912	(	778	)|	//	|
|	wind_body_x_m_s	|	FLOAT	|	1920	(	780	)|	MAYBE ?	|
|	wind_body_y_m_s	|	FLOAT	|	1928	(	788	)|	//	|
|	wind_body_z_m_s	|	FLOAT	|	1936	(	790	)|	//	|
