// Map Matching v.0.4.21x2

#ifndef _MAPMATCHING_H
#define _MAPMATCHING_H

#include <limits.h>
#include "xdb.h"
#include "geotypes.h"

#define INCREMENTAL
#define HEADING

//#define DEBUG
//#define DEBUG_T8
//#define DEBUG_SELECT
//#define DEBUG_START_END
//#define	DEBUG_R
//#define DEBUG_SCORES
//#define DEBUG_FIND_MM
//#define TESTING_COUNTS
//#define TESTING_TIMES
//#define TESTING_TIMES1
//#define TESTING_TIMES2
//#define TESTING_TIMES3
//#define TESTING_TIMES4
//#define P_QUERY
#define ROUTING_OPT

#ifndef MAX
#define MAX(a,b) ((a) > (b) ? a : b)
#define MIN(a,b) ((a) < (b) ? a : b)
#endif 

#define KPH_TO_M_PER_S(a) (0.27778 * a)
#define MPH_TO_M_PER_S(a) (0.44704 * a)
/*
// new DB structure
#define NEW_STREETS_TABLE		"way_splitted"
#define NEW_LINE_ATTRIBUTE 		"geo_line"
#define WAY_ID_ATTRIBUTE 		"id"
#define WAY_LENGTH_ATTRIBUTE 	"distance"

//#define CONS_TO_FIX_LL 	1.7		// constant to eliminate different length od lat and lon degree in UK - WORKS ONLY FOR UK!!!!

#define GPS_DISCREPANCY 	0.005//0.003	//+ 200-300 m boundary enlargement when finding nearest lines - DON'T LOWER THIS VALUE
#define MAX_CANDIDATES		35//15//40//1//40//35//15		// max candidates - closest to GPS point
#define MAX_DIST_FROM_GPS	400//250//250		// max accepted distance from line to GPS point
#define MAX_DB_LINES		5000//5000//700//250		// max lines from DB in one mm cycle - IF LOWER, ROUND ABOUTS in URBAN AREA WON'T MM.
#define MAX_GPS_COUNT		1000//100	//250	// max count of input gps coords in one mm cycle
#define	EMIS_COEF  			1 / sqrt(2 * M_PI * 20)
#define	TRANS_COEF 			1/1.4427
#define	MAX_D 				-1
#define	MINMOVE				5
#define	EMIS_PARAM  		25//20		// its average value of difference between GPS point and real position
#define	TRANS_PARAM 		1.4427	// from theory = 1/(ln 2)
//#define MIN_GPS_DIFF		0.002//0.001	// minimal distance between gps points to be accepted
//#define MAX_GPS_DIFF		0.003//			// max distance between gps points - if bigger distance, new route will be started
#define MAX_GPS_PTS_SKIP	50//10000//50
#define MAX_GPS_DIFFER		10.01//0.002//
#define MAX_GPS_VZD			500//200//100000//200
#define MAX_HED_DIF			45//45//400//45


#define MAX_DIFF_FOR_NEW_SEGMENT		250				// max distance in m between gps points - if bigger distance, segment will end
#define MAX_DIFF_FOR_POINT_IN_SEGMENT	200				// max distance in m between gps points - if smaller distance, point not taken to mm
#define MAX_HEADING_DIFFERENCE			45				// max difference for erasing gps point from segment
#define MIN_SPEED_FOR_HEADING			20				// min speed to take heading as relevant
#define MAX_SPEED_FOR_DUTCH				10				// max speed for flying dutch
#define MIN_COUNT_FOR_DUTCH				60//10//5//10				// max count of points having speed for flying dutch
#define FD_MAX_SPACE_BETWEEN_POINTS 30
#define FD_MAX_DIST_BETWEEN_POINTS 500//100


//#define PATTERN_SKIP_POINTS_BETWEEN 5
#define MIN_PAT_DIST 100
#define HEADING_TOLERANCE 35
#define STEP 20
#define C360 360.0000000000000000000
#define ANGLE_TOLERANCE_NEARBY 25
#define ANGLE_TOLERANCE_TOTAL 15
#define PI 3.14159258
#define DISTANCE_TOLERANCE 500
#define FLOATING_COUNTER 5 // pro plouvouci posouvani index_line vnejsiho cyklu pri hledani matchu
#define DIST_FOR_PAT_FINDING 200*/
// new DB structure
#define OSM_TABLE          			"way_splitted"
#define HERE_TABLE         			"here_imm_data"		///HERE///
#define HERE_ADMIN_TABLE  			"here_imm_admin"	///HERE///
#define NEW_LINE_ATTRIBUTE          "geo_line"
#define OSM_ID_ATTRIBUTE          	"id"
#define HERE_ID_ATTRIBUTE          	"link_id"
#define WAY_LENGTH_ATTRIBUTE      	"distance"

#define POSTCODES_TABLE		"postcode"
#define INITIAL_STEP 		0.005
#define INITIAL_STEP2 		0.0005
#define STEP_RATIO 			4
#define STEP_MAX 			0.1
#define POST_CODE_DIST	    0.005

/*
static int MAX_KPH[8] = {10, 30, 50, 70, 90, 100, 130, 130};
static int MAX_MPH[8] = {5, 20, 30, 40, 54, 64, 80, 80};
static int AVG_KPH[8] = {10, 20, 40, 60, 80, 95, 115, 130};
static int AVG_MPH[8] = {5, 12, 25, 35, 47, 60, 72, 80};
*/
//static double MAX_M_PER_S_FROM_KPH[8] = {2.77778, 8.33333,	13.88888, 19.44444, 25.00000, 27.77778, 36.11111, 36.11111};
//static double MAX_M_PER_S_FROM_MPH[8] = {2.23520, 8.94080,	13.41120, 17.88160, 24.14016, 28.61056, 35.76320, 35.76320};

//#define CONS_TO_FIX_LL      1.7          // constant to eliminate different length od lat and lon degree in UK - WORKS ONLY FOR UK!!!!

#define GPS_DISCREPANCY      0.005//0.003     //+ 200-300 m boundary enlargement when finding nearest lines - DON'T LOWER THIS VALUE
#define MAX_CANDIDATES          35//15//40//1//40//35//15          // max candidates - closest to GPS point
#define MAX_DIST_FROM_GPS     400//250//250          // max accepted distance from line to GPS point
#define MAX_DB_LINES          7500//5000//700//250          // max lines from DB in one mm cycle - IF LOWER, ROUND ABOUTS in URBAN AREA WON'T MM.
#define MAX_GPS_COUNT          1000//100     //250     // max count of input gps coords in one mm cycle
#define     EMIS_COEF                 1 / sqrt(2 * M_PI * 20)
#define     TRANS_COEF                1/1.4427
#define     MAX_D                     -1
#define     MINMOVE                    5
#define     EMIS_PARAM            25//20//25//20       ZZZ   // its average value of difference between GPS point and real position
#define     TRANS_PARAM           1.4427     // from theory = 1/(ln 2)
//#define MIN_GPS_DIFF          0.002//0.001     // minimal distance between gps points to be accepted
//#define MAX_GPS_DIFF          0.003//               // max distance between gps points - if bigger distance, new route will be started
#define MAX_GPS_PTS_SKIP     50//10000//50
#define MAX_GPS_DIFFER          10.01//0.002//
#define MAX_GPS_VZD               200//500//200//100000//200
#define MAX_HED_DIF               30//45//400//45


//#define MAX_DIFF_FOR_NEW_SEGMENT          250                    // max distance in m between gps points - if bigger distance, segment will end
//#define MAX_DIFF_FOR_POINT_IN_SEGMENT     200                    // max distance in m between gps points - if smaller distance, point not taken to mm
//#define MAX_HEADING_DIFFERENCE               45                    // max difference for erasing gps point from segment
#define MAX_SPEED_FOR_DUTCH                    10                    // max speed for flying dutch
#define MIN_COUNT_FOR_DUTCH                    555//60//10//5//10                    // max count of points having speed for flying dutch
#define FD_MAX_SPACE_BETWEEN_POINTS 240//180//30
#define FD_MAX_DIST_BETWEEN_POINTS 200//100
//#define PATTERN_SKIP_POINTS_BETWEEN 5
#define C360 360.0000000000000000000
#define PI 3.14159258
#define MAX_PAT_CANDIDATES 5


// for finding patterns candidates
//#define MAX_DIST_FOR_PAT 150
#define MIN_SPEED_FOR_HEADING 10//15     // min speed to take heading as relevant
#define HEADING_TOLERANCE 25
#define MIN_PAT_DIST 100//150//100
#define STEP 60//20
#define MAX_ANGLE_FOR_PAT 80

// for finding patterns
#define ANGLE_TOLERANCE_NEARBY 26//20//26 ZZZ
#define ANGLE_TOLERANCE_TOTAL 16
#define DISTANCE_TOLERANCE 500
#define FLOATING_COUNTER 5 // pro plouvouci posouvani index_line vnejsiho cyklu pri hledani matchu
#define DIST_FOR_PAT_FINDING 200

#define MAX_DIST_FOR_PAT 150
//#define MAX_ANGLE_FOR_PAT 120

#define MAX_PAT_CANDIDATES 5
#define BEST_PAT_CANDIDATE_LIMIT 0.9
//#define QUALITY_LIMIT_FOR_WRONGS 10


#define DEGREE_LENGTH 					111321.0      	// length of degree on equator in meters
#define MAX_DIFF_FOR_NEW_SEGMENT		250				// max distance in m between gps points - if bigger distance, new mm will be started
#define MAX_DIFF_FOR_POINT_IN_SEGMENT	50//200				// max distance in m between gps points - if smaller distance, point not taken to mm
//#define MIN_SPEED_FOR_HEADING			10				// min speed to take heading as relevant
#define MAX_HEADING_DIFFERENCE			45				// max difference for erasing gps point from segment
//#define MAX_GPS_COUNT					100				// max count of input gps coords in one segment
//#define MAX_GPS_PTS_SKIP				50				// max skipped GPS point till new one in segment
#define MAX_GPS_CACHE					10//50			// max GPS points in cache


#ifdef DEBUG
int mm_route_file_counter = 1;
FILE* file_can_and_matched = NULL;
FILE* file_pattern_matched = NULL;
FILE* file_original_filtered = NULL;
FILE* file_mm_filtered = NULL;
FILE*	file_angles = NULL;
int 	d_act_segment = 0;
int 	d_act_pattern = 0;
int d_act_global_pattern = 0;
#endif

#ifndef TRUE
	#define TRUE  1
	#define FALSE 0
#endif

#ifndef UINT
	typedef unsigned char			BYTE;		// 1B
	typedef int                 	INT;
	typedef unsigned int			UINT;		// 4B
	typedef unsigned char			BOOL;		// 1B
#endif

typedef struct MM_POINT_COORDS
{
	double x;
	double y;
} __attribute__((packed)) MM_POINT_LL;

typedef struct MM_WKBLINE_t
{
	unsigned char	order;
	unsigned int	wkbType;
	unsigned int	num_points;
	MM_POINT_LL 		points[1];
} __attribute__((packed)) MM_WKBLINE;

/**********************************************************************************************************************
* Map matched point.
**********************************************************************************************************************/
typedef struct MM_MAP_POINT
{
	UINT			id;				// id attribute (primary key) value of map matched edge from 'streets' table
	double			latitude;		// map matched latitude
	double			longitude;		// map matched longitude
	double			road_dist;		// distance from GPS point to map matched point	//TODO
	int 			match_quality; // 1=pattern match,
} MM_MAP_PT;

/**********************************************************************************************************************
* Map matching and geocoding return values.
**********************************************************************************************************************/
typedef enum
{
	MM_OK				= 0,		// OK
	MM_NO_MEMORY		= 1,		// Not enough memory
	MM_NO_DB_RESPONSE	= 2,		// No response from DB.
	MM_ERR_UNKNOWN		= UINT_MAX,	// Unknown error
	MM_NO_LINES			= 3,			// no lines in boundary found
	MM_WRONG_EVENT_TIME = 4, // Event time between two following points has decreased
	MM_NO_VALID_POINT 	= 2
} MM_ERRORS;

/**********************************************************************************************************************
* Map matching workspace.
**********************************************************************************************************************/
typedef struct MM_WORKSPACE
{
	xdb_t		xdb;			// DB data.
} MM_WS;

/**********************************************************************************************************************
* Map matching initialization data.
**********************************************************************************************************************/
typedef struct MM_INIT_DATA
{
	char 		*host;		// Name of the server, where DB is placed.
	char 		*db;		// Name of the DB.
	char 		*user;		// DB User name.
	char 		*passwd;	// DB User password.
	int 		port;		// DB Port number.
} MM_INIT;

/**********************************************************************************************************************
* GPS input point.
**********************************************************************************************************************/
typedef struct MM_GPS_POINT
{
	unsigned long long device_id;	// Vehicle identification
	time_t jny_start;	// Start time of journey
	time_t	jny_end;	// End time of journey
	time_t	event_time;	// Time of event
	int	RPM;	// Rounds per minute
	int	sats_count;	// Count of satellites
	int	fix_status;	// Fix status
	int HDOP;		// HDOP
	int	dist_covered;	// Covered distance

	double	latitude;	// latitude of vehicle
	double	longitude;	// longitude of vehicle

	int	heading;	// gps heading vehicle
	int	speed;	// GPS speed vehicle

	int	can_speed;	// CAN bus Vehicle Speed
	double	gyro_diff;	// GYRO difference

	double acc_x;	// acceleration in X
	double acc_y;	// acceleration in Y
} MM_GPS_PT;

/**********************************************************************************************************************
* Map matching input structure.
**********************************************************************************************************************/
typedef struct MM_REQUEST_TYPE
{
	int 			gps_pts_size; 	// number of MM_GPS_PT(s)
	MM_GPS_PT* 		gps_pts; 		// pointer to first MM_GPS_PT in array
} MM_REQUEST;

/**********************************************************************************************************************
* Map matching output structure.
**********************************************************************************************************************/
typedef struct MM_RESPONSE_TYPE
{
	MM_MAP_PT* 		map_pts;		// pointer to first MM_MAP_PT in array
	int				quality;		// number of segments high mm quality in % (100 = all segments are OK)
} MM_RESPONSE;

/**********************************************************************************************************************
* Initializes library workspace. Waill have to be called once, before using any other methods of the library.
* @in init_data		Map matching initialization data.
* @out workspace	Workspace of the library, if successfull.
* @return			Return code.
**********************************************************************************************************************/
MM_ERRORS mm_init(MM_INIT *init_data, MM_WS** workspace);

/**********************************************************************************************************************
* Releases all resources used by map matching library.
* @in workspace	Workspace of the library.
**********************************************************************************************************************/
void mm_close(MM_WS* workspace);

/**********************************************************************************************************************
* Used to find all map matching points and it's related data for all GPS points stored in input array. Before first
* call of this method, mm_init must be called once by every thread. Count of map matched points will be the same as
* count of GPS input points. Output array must be allocated before calling method.
* @in ws			Workspace of the library.
* @in mm_request	Input data.
* @out mm_response	Output data.
* @return ret_code	Return code.
**********************************************************************************************************************/
MM_ERRORS mm_map_match(MM_WS * ws, MM_REQUEST* mm_request, MM_RESPONSE* mm_response);

#ifdef TESTING
	void debug_print();
#endif	//TESTING


	typedef enum
	{
		IMM_HERE		= 1,		// HERE data
		IMM_OSM			= 2,		// OSM data
	} IMM_MAP_SOURCE;

	typedef enum
	{
		IMM_NEAREST_ONLY	= 1,		// nearest algorithm and no geocoding
		IMM_NEAREST			= 2,		// nearest algorithm + geocoding
		IMM_MARKOV			= 3,		// Markov � A* routing algorithm + geocoding
	} IMM_ALGORITHM;


	/**********************************************************************************************************************
	* Incremental Map matching workspace structure for DB
	**********************************************************************************************************************/

	typedef struct IMM_WORKSPACE_DB
	{
		xdb_t		xdb;			// DB data.
	} IMM_WS_DB;

	/**********************************************************************************************************************
	* Incremental Map matching workspace structure for unit cache data
	**********************************************************************************************************************/

	typedef struct IMM_WORKSPACE_UNIT
	{
		void*		imm_cache;		// Pointer to cache, structure allocated in imm_init,freed in imm_close. Memory space allocated/reallocated in imm_map_match, freed in imm_close.
	} IMM_WS_UNIT;


	/**********************************************************************************************************************
	* GPS input point.
	**********************************************************************************************************************/
	typedef struct IMM_GPS_POINT
	{
		//int	id; //!!!!  timestamp or sequence, not necessary if we are sure that processed in correct order

		double	latitude;	// latitude of vehicle
		double	longitude;	// longitude of vehicle

		int	gps_heading;	// gps heading vehicle
		int	gps_speed;	// GPS speed vehicle

		unsigned long long device_id;	// Vehicle identification
		time_t jny_start;	// Start time of journey
		time_t	jny_end;	// End time of journey
		time_t	event_time;	// Time of event
		int	RPM;	// Rounds per minute
		int	sats_count;	// Count of satellites
		int	fix_status;	// Fix status
		int HDOP;		// HDOP
		int	dist_covered;	// Covered distance


		int	can_speed;	// CAN bus Vehicle Speed
		int	can_validity_flag; // CAN bus valid flag
		int	can_rpm;	// CAN RPM
		int	gps_fix;	// GPS Fix status
		int	sat_visible;	// GPS Satellites visible
		double	gyro_heading;	// GYRO Heading
		int	acc_speed;	// Accelerometer speed

		double acc_x;	// acceleration in X
		double acc_y;	// acceleration in Y

	} IMM_GPS_PT;



	/**********************************************************************************************************************
	* Incremental Map matching input structure.
	**********************************************************************************************************************/
	typedef struct IMM_REQUEST_TYPE
	{
		int 				gps_pts_size;			// number of IMM_GPS_PT(s)
		IMM_GPS_PT* 		gps_pts; 				// pointer to first IMM_GPS_PT in array
	} IMM_REQUEST;



	/**********************************************************************************************************************
	* Incremental Map matched point.
	**********************************************************************************************************************/
	typedef struct IMM_MAP_POINT
	{
		double			latitude;					// map matched latitude
		double			longitude;					// map matched longitude
		double			diff_GPS_MM;				// distance from GPS point to map matched point	//TODO
		double			speed_limit; 				// Legal speed limit
		double			average_speed; 				// Average_Speed
		UINT			id;							// id attribute (primary key) value of map matched edge from 'streets' table
		char 			route_name[20];				// Name Route, e.g. M25, A303, B20, E112
		char			street_name[100]; 			//Street name UTF8 Native
		char			street_name_latin[100]; 	//Street name UTF8 Latin
		char			street_name_translate[100];	//Street name UTF8 Translation
		char			town_name[100]; 			//town name UTF8 Native
		char			town_name_latin[100]; 		//town name UTF8 Latin
		char			town_name_translate[100];	//town name UTF8 Translation
		char			county_name[100]; 			//county name UTF8 Native
		char			county_name_latin[100]; 	//county name UTF8 Latin
		char			county_name_translate[100];	//county name UTF8 Translation
		char			country_name[100]; 			//country name UTF8 Native
		char			country_name_latin[100]; 	//country name UTF8 Latin
		char			country_name_translate[100];//country name UTF8 Translation
		char			post_code[11];				//Post Code / Zip Code
		char			time_zone[4];				//Time Zone ID

		UINT			func_class;  				// mayby int, dont know yet Road_Type � e.g. Motorway, Dual carriage way, A road, B - road.  (its probably function class in navteq) but we need this for working out the speed limit specific for the vehicle (e.g. Trucks are allowed 60mph on a motorway and 50 mph on a dual carriage way).
        UINT            route_type;
        BOOL            tollway;
        UINT            speed_cat;
	} IMM_MAP_PT;


	/**********************************************************************************************************************
	* Incremental Map matching output structure.
	**********************************************************************************************************************/
	typedef struct IMM_RESPONSE_TYPE
	{
		IMM_MAP_PT* 		map_pts;		// pointer to first IMM_MAP_PT in array
	} IMM_RESPONSE;


	/**********************************************************************************************************************
	* Incremental Map matching initialization db data.
	**********************************************************************************************************************/
	typedef struct IMM_INIT_DATA_DB
	{
		char 			*host;		// Name of the server, where DB is placed.
		char 			*db;		// Name of the DB.
		char 			*user;		// DB User name.
		char 			*passwd;	// DB User password.
		int 			port;		// DB Port number.

	} IMM_INIT_DB;

	/**********************************************************************************************************************
	* Incremental Map matching initialization data for unit
	**********************************************************************************************************************/
	typedef struct IMM_INIT_DATA_UNIT
	{
		IMM_MAP_SOURCE	map_source; //enum defines OSM/HERE/...
		IMM_ALGORITHM	algorithm;	//enum defines level of geocoding/map matching
	} IMM_INIT_UNIT;


	/**********************************************************************************************************************
	* Initializes library db workspace. Waill have to be called once, before using any other methods of the library.
	* @in init_data		Map matching initialization db data.
	* @out workspace	Workspace of the library, if successfull.
	* @return			Return code.
	**********************************************************************************************************************/
	int imm_init_db(IMM_INIT_DB *init_data, IMM_WS_DB** workspace);


	/**********************************************************************************************************************
	* Initializes library unit workspace. Waill have to be called once, before using any other methods of the library.
	* @in init_data		Map matching initialization data.
	* @out workspace	Workspace of the library, if successfull.
	* @return			Return code.
	**********************************************************************************************************************/
	int imm_init_unit(IMM_INIT_UNIT *init_data,IMM_WS_UNIT** workspace);

	/**********************************************************************************************************************
	* Releases db resources used by map matching library.
	* @in workspace	Workspace of the library.
	**********************************************************************************************************************/
	void imm_close_db(IMM_WS_DB* workspace);

	/**********************************************************************************************************************
	* Releases unit resources used by map matching library.
	* @in workspace	Workspace of the library.
	**********************************************************************************************************************/
	void imm_close_unit(IMM_WS_UNIT* workspace);


	/**********************************************************************************************************************
	* Used to find all map matching points and it's related data for all GPS points stored in input array. Before first
	* call of this method, imm_init must be called once by every thread. Count of map matched points will be the same as
	* count of GPS input points. Output array must be allocated before calling method.
	* @in ws			Workspace of the library.
	* @in imm_request	Input data.
	* @out imm_response	Output data.
	* @return ret_code	Return code.
	**********************************************************************************************************************/
	int imm_map_match(IMM_WS_DB * ws_db,IMM_WS_UNIT * ws_unit, IMM_REQUEST* imm_request, IMM_RESPONSE* imm_response);


	typedef struct MM_GPS_POINT_W
	{
		double		latitude;		// latitude of vehicle
		double		longitude;		// longitude of vehicle

		int			heading;		// heading of vehicle
		int			speed;			// speed of vehicle in km/h
		int			index;			// speed of vehicle in km/h
	} MM_GPS_PT_W;

	/**********************************************************************************************************************
	* Map matching input structure.
	**********************************************************************************************************************/
	typedef struct MM_REQUEST_TYPE_W
	{
		int 			gps_pts_size; 	// number of MM_GPS_PT_W(s)
		MM_GPS_PT_W* 		gps_pts; 		// pointer to first MM_GPS_PT_W in array
	} MM_REQUEST_W;

	/**********************************************************************************************************************
	* Map matched point.
	**********************************************************************************************************************/
	typedef struct MM_MAP_POINT_W
	{
		unsigned int	id;				// id attribute (primary key) value of map matched edge from 'streets' table
		double			latitude;		// map matched latitude
		double			longitude;		// map matched longitude
		double			road_dist;		// distance from GPS point to map matched point	//TODO
		int				geocoded;
		int				line_id;
		int 			start_Node;
		int 			end_Node;
		double			dist_to_start;
		double			dist_start_end;
		MM_WKBLINE* 	line;
		int 			one_way;
	#ifdef HEADING
		int				heading;		// Angle in degrees between candidate, first point on the line in the direction to start and X axis.
	#endif	//HEADING
	} MM_MAP_PT_W;

	/**********************************************************************************************************************
	* Map matching output structure.
	**********************************************************************************************************************/
	typedef struct MM_RESPONSE_TYPE_W
	{
		MM_MAP_PT_W* 		map_pts;		// pointer to first MM_MAP_PT in array
	} MM_RESPONSE_W;


	/**********************************************************************************************************************
	 * MM_REQUEST + MM_RESPONSE
	 **********************************************************************************************************************/
	typedef struct MM_DATA_STR
	{
		// MM_REQUEST
		unsigned long long 	device_id;		// Vehicle identification
		time_t 				jny_start;		// Start time of journey
		time_t				jny_end;		// End time of journey
		time_t				event_time;		// Time of event
		int					RPM;			// Rounds per minute
		int					sats_count;		// Count of satellites
		int					fix_status;		// Fix status
		int					dist_covered;	// Covered distance
		double				latitude;		// latitude of vehicle
		double				longitude;		// longitude of vehicle
		int					gps_heading;	// heading of vehicle
		int					gps_speed;		// speed of vehicle in km/h

		// MM_RESPONSE
		UINT				id;				// id attribute (primary key) value of map matched edge from 'streets' table
		double				mm_latitude;	// map matched latitude
		double				mm_longitude;	// map matched longitude
		double				road_dist;		// distance from GPS point to map matched point	//TODO
		int 				match_quality; 	// 1=pattern match

		// NEW PROPERTIES
		int					is_valid;		// 0 = OK, 1 = invalid
		int 				segment_no;		// number of segment
		double 				gyro_diff;		// heading difference from gyroscop
		double				gyro_heading;	// calculated heading from gyro and gps_heading
		double				HDOP;			// HDOP
		double 				acc_x;			// acceleration in X
		double 				acc_y;			// acceleration in Y
		int					can_speed;		// speed from CAN bus

		// NEW PROPERTIES 2
		double				pre_mm_latitude;
		double				pre_mm_longitude;
		int					is_good_point;
	} MM_DATA_STRUCT;

	typedef struct MM_DATA_TYPE
	{
		int 			arr_size; 	// number of MM_GPS_PT(s)
		MM_DATA_STRUCT*	pts; 		// pointer to first MM_GPS_PT in array
	} MM_DATA;


/*
#ifndef INCREMENTAL

	typedef struct MM_GPS_POINT_W
	{
		double		latitude;		// latitude of vehicle
		double		longitude;		// longitude of vehicle

		int		heading;		// heading of vehicle
		int			speed;			// speed of vehicle in km/h
		int			index;			// speed of vehicle in km/h
	} MM_GPS_PT_W;

	typedef struct MM_REQUEST_TYPE_W
	{
		int 			gps_pts_size; 	// number of MM_GPS_PT_W(s)
		MM_GPS_PT_W* 		gps_pts; 		// pointer to first MM_GPS_PT_W in array
	} MM_REQUEST_W;

	typedef struct MM_MAP_POINT_W
	{
		UINT			id;				// id attribute (primary key) value of map matched edge from 'streets' table
		double			latitude;		// map matched latitude
		double			longitude;		// map matched longitude
		double			road_dist;		// distance from GPS point to map matched point	//TODO
		int					geocoded;
		int					line_id;
		int start_Node;
		int end_Node;
		double		dist_to_start;
		double		dist_start_end;
		MM_WKBLINE* line;
		int one_way;
	#ifdef HEADING
		int			heading;		// Angle in degrees between candidate, first point on the line in the direction to start and X axis.
	#endif	//HEADING
	} MM_MAP_PT_W;
		typedef struct MM_RESPONSE_TYPE_W
	{
		MM_MAP_PT_W* 		map_pts;		// pointer to first MM_MAP_PT in array
	} MM_RESPONSE_W;

#endif
*/

typedef struct TIME_SLOT
{
	MM_GPS_PT_W* 	gps_pt;							// GPS point in this time slot.
	int 		line_id[MAX_CANDIDATES];		// Array of line id for candidates
	long double		scores[MAX_CANDIDATES];			// score for HMM
	int			parents[MAX_CANDIDATES];		// Index number in previous time slot.
	double		distances[MAX_CANDIDATES];		// Dist from gps to closet point on the line.
	double		dist_to_start[MAX_CANDIDATES];	// Dist form  candidate (point on lien) to start point of the line - for routing
	int 		start_Node[MAX_CANDIDATES];		// ID Node on start of the line - for routing.
	int 		end_Node[MAX_CANDIDATES];		// ID Node on end of the line - for routing.
	double		dist_start_end[MAX_CANDIDATES];	// Dist form  start to end point of the line - for routing
#ifdef HEADING
	int			heading[MAX_CANDIDATES];		// Angle in degrees between candidate, first point on the line in the direction to start and X axis.
#endif	//HEADING
	MM_POINT_LL	points[MAX_CANDIDATES];			// candidate - closest point on the line from gps point.
	int 		count;							// Count of candidates in this time slot.
	double 		dist_to_next;					// Length to next timeslot - dist from thos GPS to tne next one
	int 		one_way[MAX_CANDIDATES];
	int 		last_Node[MAX_CANDIDATES];
	MM_WKBLINE* 	line[MAX_CANDIDATES];
} TIMESLOT;

typedef struct 		// structure for routing
{
	double x;					// x of the node
	double y;					//y of the node
	double Fcost;				// A star value - uusing when calculating routing
	double Gcost;				// A star value - uusing when calculating routing
	int neighborsCount;			// Count of neighbors nodes
	int neighborsID[10];		// list od neigbors ID
	int neighborsDistance[10]; 	// lis of distances from this node to neigbbors
	BOOL closeList;				// A star value - uusing when calculating routing
	BOOL openList;				// A star value - uusing when calculating routing
	int parent;					// remembers parent node during calculation
	MM_WKBLINE* line[10];
	int startNode[10];
}Node;

typedef struct MM_SEGMENT
{
	int 		i_start;
	int 		length;
	int			type;
	double		l_offset_lon;
	double		l_offset_lat;
	double		r_offset_lon;
	double		r_offset_lat;
} segment;

typedef struct MM_PATTERN_CANDIDATES
{
	int 		left_index;
	int 		right_index;
	MM_POINT_LL	left_candidate[MAX_PAT_CANDIDATES];
	MM_POINT_LL	right_candidate[MAX_PAT_CANDIDATES];
	int			left_line_index[MAX_PAT_CANDIDATES];
	int			right_line_index[MAX_PAT_CANDIDATES];

} MM_PATT_CANDIDATES;


typedef struct IMM_CACHE_STRUCT		///HERE///
{
	IMM_REQUEST			gps_cache;
	IMM_MAP_SOURCE		map_source; //enum defines OSM/HERE/...
	IMM_ALGORITHM		algorithm;	//enum defines level of geocoding/map matching
} IMM_CACHE;

#endif	/* MAPMATCHING */
